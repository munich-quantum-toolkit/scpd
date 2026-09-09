/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/pipeline/GlobalRouter.hpp"

#include "mqt-scpd/design/Bridges.hpp"
#include "mqt-scpd/design/Roles.hpp"
#include "mqt-scpd/flatbuffers/artifacts.hpp"
#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/geometry/Geometry.hpp"
#include "mqt-scpd/milp/Backend.hpp"
#include "mqt-scpd/milp/Model.hpp"
#include "mqt-scpd/pipeline/Solver.hpp"
#include "mqt-scpd/pipeline/Stages.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <map>
#include <memory>
#include <numbers>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace mqt::scpd::pipeline {
namespace {

namespace fba = flatbuffers::artifacts;
namespace fbg = flatbuffers::geometry;
namespace fbd = flatbuffers::design;

/// Two lattice coordinates closer than this are the same line. The chip is
/// measured in micrometres, so this is far below anything the input resolves.
constexpr double COORDINATE_EPSILON = 1.0e-7;

/// The port each ring label names.
std::unordered_map<std::string, std::uint32_t> portsByLabel(const ChipT& chip) {
  std::unordered_map<std::string, std::uint32_t> byLabel;
  byLabel.reserve(chip.ports.size());
  for (std::uint32_t index = 0; index < chip.ports.size(); ++index) {
    byLabel.emplace(chip.ports[index]->label, index);
  }
  return byLabel;
}

/// One Hanan lattice: the rectilinear grid a set of ports induces.
struct Lattice {
  struct Node {
    fbg::Point position;
    /// The port this node is, or none.
    std::optional<std::uint32_t> port;
    /// The four axis neighbours, by node index.
    std::vector<std::size_t> neighbors;
  };
  std::vector<Node> nodes;
  /// The node of each port that took part.
  std::unordered_map<std::uint32_t, std::size_t> nodeOfPort;
};

/// The point a port's lattice line runs through.
///
/// The line lands in front of the port rather than on the component's own
/// artwork, which is why it is projected along the port's own orientation
/// first. Without that every lattice line of a qubit would run through the
/// qubit.
fbg::Point latticeAnchor(const fbd::PortT& port, const double reach) {
  const auto radians = port.orientation * std::numbers::pi / 180.0;
  return {port.center.x() + (reach * std::cos(radians)),
          port.center.y() + (reach * std::sin(radians))};
}

/// Whether a point lies inside a polygon, by ray casting.
bool insidePolygon(const fbg::PolygonT& polygon, const fbg::Point& point) {
  const auto& vertices = polygon.vertices;
  if (vertices.size() < 3) {
    return false;
  }
  bool inside = false;
  for (std::size_t index = 0, previous = vertices.size() - 1; index < vertices.size();
       previous = index++) {
    const auto& a = vertices[index];
    const auto& b = vertices[previous];
    if ((a.y() > point.y()) != (b.y() > point.y()) &&
        point.x() < ((b.x() - a.x()) * (point.y() - a.y()) / (b.y() - a.y())) + a.x()) {
      inside = !inside;
    }
  }
  return inside;
}

/// Whether a lattice point lies on chip artwork.
///
/// The first polygon is the chip outline and is skipped, exactly as the
/// obstacle rasterization skips it. The prototype's own test skipped the
/// first *two* polygons and ignored every polygon with fewer than ten
/// vertices, which on the 69-qubit chip silently exempted real artwork —
/// including the first polygon, which has four vertices. That is not
/// reproduced.
bool onArtwork(const ChipT& chip, const fbg::Point& point) {
  for (std::size_t index = 1; index < chip.obstacles.size(); ++index) {
    if (insidePolygon(*chip.obstacles[index], point)) {
      return true;
    }
  }
  return false;
}

/// The Hanan lattice of a set of ports: one line through every anchor's x
/// and one through every anchor's y.
///
/// A lattice node that lands on chip artwork is removed, and with it every
/// edge that touched it. Without that the inner circuit routes straight
/// through the couplers and the qubits it is supposed to route between, and
/// the objective it reports is over paths that could never be built.
Lattice buildLattice(const ChipT& chip, const std::vector<std::uint32_t>& ports,
                     const double reach) {
  Lattice lattice;
  if (ports.empty()) {
    return lattice;
  }

  std::set<double> xs;
  std::set<double> ys;
  std::vector<fbg::Point> anchors;
  anchors.reserve(ports.size());
  for (const auto port : ports) {
    const auto anchor = latticeAnchor(*chip.ports[port], reach);
    anchors.push_back(anchor);
    xs.insert(anchor.x());
    ys.insert(anchor.y());
  }

  const std::vector<double> columns(xs.begin(), xs.end());
  const std::vector<double> rows(ys.begin(), ys.end());
  lattice.nodes.reserve(columns.size() * rows.size());
  for (std::size_t row = 0; row < rows.size(); ++row) {
    for (std::size_t column = 0; column < columns.size(); ++column) {
      lattice.nodes.push_back({.position = {columns[column], rows[row]}, .port = std::nullopt});
    }
  }

  const auto at = [&](const std::size_t column, const std::size_t row) {
    return (row * columns.size()) + column;
  };
  for (std::size_t row = 0; row < rows.size(); ++row) {
    for (std::size_t column = 0; column < columns.size(); ++column) {
      auto& node = lattice.nodes[at(column, row)];
      if (column + 1 < columns.size()) {
        node.neighbors.push_back(at(column + 1, row));
      }
      if (column > 0) {
        node.neighbors.push_back(at(column - 1, row));
      }
      if (row + 1 < rows.size()) {
        node.neighbors.push_back(at(column, row + 1));
      }
      if (row > 0) {
        node.neighbors.push_back(at(column, row - 1));
      }
    }
  }

  for (std::size_t index = 0; index < ports.size(); ++index) {
    const auto column = static_cast<std::size_t>(
        std::lower_bound(columns.begin(), columns.end(), anchors[index].x() - COORDINATE_EPSILON) -
        columns.begin());
    const auto row = static_cast<std::size_t>(
        std::lower_bound(rows.begin(), rows.end(), anchors[index].y() - COORDINATE_EPSILON) -
        rows.begin());
    const auto node = at(column, row);
    lattice.nodes[node].port = ports[index];
    lattice.nodeOfPort.emplace(ports[index], node);
  }

  // A node on artwork goes, and so does every edge that reached it. A port's
  // own node is kept whatever it sits on: the port is where a wire has to
  // start or end, and removing it would make the model unable to say so.
  std::vector<bool> removed(lattice.nodes.size(), false);
  bool any = false;
  for (std::size_t node = 0; node < lattice.nodes.size(); ++node) {
    if (!lattice.nodes[node].port.has_value() && onArtwork(chip, lattice.nodes[node].position)) {
      removed[node] = true;
      any = true;
    }
  }
  if (!any) {
    return lattice;
  }
  for (std::size_t node = 0; node < lattice.nodes.size(); ++node) {
    if (removed[node]) {
      lattice.nodes[node].neighbors.clear();
      continue;
    }
    std::erase_if(lattice.nodes[node].neighbors,
                  [&](const std::size_t other) { return removed[other]; });
  }
  return lattice;
}

/// The distance between two lattice nodes, which is the weight of the edge
/// that joins them.
double edgeWeight(const Lattice& lattice, const std::size_t from, const std::size_t to) {
  return geometry::distance(lattice.nodes[from].position, lattice.nodes[to].position);
}

/// The global router of the first release.
class HananMilpRouter final : public IGlobalRouter {
public:
  [[nodiscard]] GlobalRoutingT run(const ChipT& chip, const CapacityPlanT& capacity,
                                   const ConfigT& config) const override {
    const auto circuit = innerCircuitOf(chip, config);
    const auto reach = config.rules != nullptr ? config.rules->min_straight_length : 0.0;

    GlobalRoutingT routing;
    routing.objective = 0.0;

    // A chip whose ring is its whole port set has no inner circuit. That is
    // a valid pipeline state, and the artifact is written empty rather than
    // skipped.
    if (circuit.targets.empty()) {
      fillRing(routing, chip, config, circuit, {});
      return routing;
    }

    const auto lattices = latticesOf(chip, capacity, circuit, reach);
    if (lattices.empty()) {
      // The chip has inner targets, but no capacity chain offers a lattice
      // that could reach one: every chain is either all targets, so nothing
      // could supply them, or a single point. There is no circuit to solve,
      // and an empty model is not something to hand a solver.
      fillRing(routing, chip, config, circuit, {});
      return routing;
    }

    milp::Model model("inner-circuit");
    std::vector<milp::Var> unreached;
    double penalty = 0.0;
    const auto variables = build(model, chip, capacity, lattices, circuit,
                                 internalBridgesAllowed(config), unreached, penalty);

    const auto solution = solveWith(model, config);
    if (!solution.hasValues()) {
      throw std::runtime_error(std::format("the inner circuit did not solve: {}{}",
                                           milp::statusName(solution.status),
                                           solution.message.empty() ? "" : ", " + solution.message));
    }
    // The objective is the length of the circuit. A target the lattice could
    // not serve is a fact the connections already report, so its penalty
    // comes back out rather than swamping the cost.
    const auto missed = std::ranges::count_if(
        unreached, [&](const milp::Var marker) { return solution.isSet(marker); });
    routing.objective = solution.objective - (penalty * static_cast<double>(missed));

    const auto sources = readBack(routing, chip, lattices, variables, solution, circuit);
    fillRing(routing, chip, config, circuit, sources);
    return routing;
  }

private:
  /// Whether an inner wire may cross a component the outer ring does not
  /// reach. Off unless the configuration says otherwise.
  [[nodiscard]] static bool internalBridgesAllowed(const ConfigT& config) {
    return config.stages != nullptr && config.stages->global != nullptr &&
           config.stages->global->internal_bridges;
  }

  /// One directed lattice adjacency, and the variable that decides it.
  struct Arc {
    std::size_t lattice = 0;
    std::size_t from = 0;
    std::size_t to = 0;
    milp::Var variable;
  };

  /// The lattices the model is built over: one per capacity chain that has an
  /// inner target to serve.
  [[nodiscard]] static std::vector<Lattice> latticesOf(const ChipT& chip,
                                                       const CapacityPlanT& capacity,
                                                       const InnerCircuit& circuit,
                                                       const double reach) {
    const std::unordered_set<std::uint32_t> targets(circuit.targets.begin(),
                                                    circuit.targets.end());
    std::vector<Lattice> lattices;
    for (const auto root : capacity.chains) {
      std::vector<std::uint32_t> ports;
      std::unordered_set<std::uint32_t> seen;
      collectTargets(capacity, root, ports, seen);

      // Every port of the chain takes part, inner and outer alike. An inner
      // wire has to be able to leave through an outer port — that is what a
      // coupler bridge is for, and which outer port it surfaces at is what
      // the assignment reads back out of this stage. A lattice built from
      // the inner ports alone can never surface anywhere.
      if (std::ranges::none_of(ports,
                               [&](const std::uint32_t port) { return targets.contains(port); })) {
        continue;
      }
      auto lattice = buildLattice(chip, ports, reach);
      // A single node has no edge at all, so there is no lattice to speak of.
      // Everything else is kept, including a chain whose ports are all
      // targets: that is a demand with no supply, and the artifact says so by
      // carrying the lattice with nothing selected on it rather than by
      // leaving the grid out of the picture.
      if (lattice.nodes.size() > 1) {
        lattices.push_back(std::move(lattice));
      }
    }
    return lattices;
  }

  static void collectTargets(const CapacityPlanT& capacity, const std::uint32_t node,
                             std::vector<std::uint32_t>& ports,
                             std::unordered_set<std::uint32_t>& seen) {
    if (node >= capacity.nodes.size()) {
      return;
    }
    const auto& entry = *capacity.nodes[node];
    if (entry.kind == fba::CapacityElement::Target && seen.insert(entry.id).second) {
      ports.push_back(entry.id);
    }
    for (const auto next : entry.next) {
      collectTargets(capacity, next, ports, seen);
    }
  }

  /// The flow model over every lattice.
  ///
  /// `unreached` collects the marker of every target the model is allowed to
  /// leave unserved, so the caller can take those markers back out of the
  /// objective: what they carry is a penalty large enough to be a last
  /// resort, and leaving it in would make the reported cost incomparable
  /// with a run where every target was reached.
  [[nodiscard]] static std::vector<Arc> build(milp::Model& model, const ChipT& chip,
                                              const CapacityPlanT& capacity,
                                              const std::vector<Lattice>& lattices,
                                              const InnerCircuit& circuit, const bool internal,
                                              std::vector<milp::Var>& unreached,
                                              double& penalty) {
    const std::unordered_set<std::uint32_t> targets(circuit.targets.begin(),
                                                    circuit.targets.end());
    std::vector<Arc> arcs;
    // Where each directed adjacency's variable sits, so the constraints can
    // find it again without a search.
    std::vector<std::map<std::pair<std::size_t, std::size_t>, milp::Var>> byPair(lattices.size());

    double totalWeight = 0.0;
    for (std::size_t index = 0; index < lattices.size(); ++index) {
      const auto& lattice = lattices[index];
      for (std::size_t from = 0; from < lattice.nodes.size(); ++from) {
        for (const auto to : lattice.nodes[from].neighbors) {
          const auto weight = edgeWeight(lattice, from, to);
          totalWeight += weight;
          const auto variable =
              model.addBinary(std::format("arc_{}_{}_{}", index, from, to), weight);
          byPair[index].emplace(std::pair{from, to}, variable);
          arcs.push_back({.lattice = index, .from = from, .to = to, .variable = variable});
        }
      }
    }
    const auto unreachedPenalty = totalWeight + 1.0;
    penalty = unreachedPenalty;

    const auto flow = [&](const std::size_t index, const std::size_t node, const bool out) {
      milp::LinearExpr expression;
      for (const auto other : lattices[index].nodes[node].neighbors) {
        const auto key = out ? std::pair{node, other} : std::pair{other, node};
        expression.add(byPair[index].at(key), 1.0);
      }
      return expression;
    };

    // What the circuit sends through a port, over every lattice that carries
    // it. A port is a node of every chain whose chamber it borders, so a
    // constraint on a port is a constraint on that sum and not on one grid:
    // read per lattice, a bridge whose two ends fall in different chambers
    // would go unconstrained, and a wire could start at one of them out of
    // nothing.
    const auto portFlow = [&](const std::uint32_t port, const bool out) {
      milp::LinearExpr expression;
      for (std::size_t index = 0; index < lattices.size(); ++index) {
        const auto found = lattices[index].nodeOfPort.find(port);
        if (found != lattices[index].nodeOfPort.end()) {
          expression.add(flow(index, found->second, out));
        }
      }
      return expression;
    };

    for (std::size_t index = 0; index < lattices.size(); ++index) {
      const auto& lattice = lattices[index];
      for (std::size_t node = 0; node < lattice.nodes.size(); ++node) {
        const auto port = lattice.nodes[node].port;
        if (port.has_value() && targets.contains(*port)) {
          // A target draws its one wire over every lattice at once, which is
          // the constraint below rather than anything per node.
          continue;
        }
        if (port.has_value()) {
          // Another port is where a wire may start or pass through, once.
          model.addLessOrEqual(std::format("port_in_{}_{}", index, node),
                               flow(index, node, false), 1.0);
          model.addLessOrEqual(std::format("port_out_{}_{}", index, node),
                               flow(index, node, true), 1.0);
          continue;
        }
        // A plain crossing conserves what passes through it, and carries at
        // most one wire.
        model.addEqual(std::format("inner_{}_{}", index, node),
                       flow(index, node, false) - flow(index, node, true), 0.0);
        model.addLessOrEqual(std::format("inner_cap_{}_{}", index, node),
                             flow(index, node, false), 1.0);
      }

      // A lattice edge carries a wire in one direction at most, so two
      // opposite arcs are never both selected.
      for (std::size_t from = 0; from < lattice.nodes.size(); ++from) {
        for (const auto to : lattice.nodes[from].neighbors) {
          if (from >= to) {
            continue;
          }
          model.addLessOrEqual(
              std::format("once_{}_{}_{}", index, from, to),
              milp::LinearExpr(byPair[index].at({from, to})) + byPair[index].at({to, from}), 1.0);
        }
      }
    }

    // A target is where a wire ends: everything that arrives stays, and one
    // wire is what a port takes. The demand is on the port, not on a lattice
    // node — a chamber border runs between two chains, so a target beside one
    // is a node of both, and a demand per lattice would ask for a second wire
    // to a port that already has one. That second wire is not merely waste:
    // it takes the supply the chip has, and the target it was taken from is
    // then reported unreached.
    //
    // Whether a target is reached at all is a variable rather than a
    // constraint. A chip that cannot serve one of its targets is a fact about
    // the chip and the grid it was partitioned on, and the stage has to say
    // which target that was; a hard constraint would only say "infeasible"
    // and lose the rest of the circuit with it. The penalty is above the cost
    // of every edge together, so a target is left unreached only when it
    // truly cannot be reached, and the objective of a circuit that reaches
    // all of them is exactly what the hard constraint would have given.
    for (const auto target : circuit.targets) {
      const auto carried = std::ranges::any_of(lattices, [&](const Lattice& lattice) {
        return lattice.nodeOfPort.contains(target);
      });
      if (!carried) {
        // No lattice reaches it, so there is no variable to constrain and
        // nothing to leave unreached either.
        continue;
      }
      const auto missed = model.addBinary(std::format("unreached_{}", target), unreachedPenalty);
      unreached.push_back(missed);
      model.addEqual(std::format("target_in_{}", target),
                     portFlow(target, false) + missed, 1.0);
      model.addEqual(std::format("target_out_{}", target), portFlow(target, true), 0.0);
    }

    // A bridge that surfaces on the ring is where a wire leaves the inner
    // circuit for good, and what may leave there is not a matter of the far
    // end's flow — the far end is outside, on no lattice — but of what the
    // free space behind it can carry, which is the chain capacity below. So
    // only a bridge with both ends inside is treated here.
    std::unordered_set<std::uint32_t> surfaces;
    for (const auto& bridge : circuit.outerBridges) {
      surfaces.insert(bridge.to);
    }
    for (const auto& bridge : circuit.bridges) {
      if (surfaces.contains(bridge.to) || surfaces.contains(bridge.from)) {
        continue;
      }
      if (!internal) {
        // An internal crossing is not allowed, so both of its ports are
        // stilled rather than merely left uncoupled: a wire that ended at one
        // of them would end inside a component the outer ring never reaches,
        // and no later stage could say where it goes from there. The nodes
        // stay in the lattice — the picture still shows the ports — but
        // nothing runs through them.
        for (const auto port : {bridge.from, bridge.to}) {
          model.addEqual(std::format("bridge_shut_in_{}", port), portFlow(port, false), 0.0);
          model.addEqual(std::format("bridge_shut_out_{}", port), portFlow(port, true), 0.0);
        }
        continue;
      }
      // What leaves one of its ports is what entered the other. The two ends
      // usually sit in different chambers, and therefore in different
      // lattices, so the flow is taken over all of them.
      model.addEqual(std::format("bridge_{}_{}", bridge.from, bridge.to),
                     portFlow(bridge.from, true) - portFlow(bridge.to, false), 0.0);
      model.addEqual(std::format("bridge_back_{}_{}", bridge.from, bridge.to),
                     portFlow(bridge.to, true) - portFlow(bridge.from, false), 0.0);
      model.addLessOrEqual(std::format("bridge_once_{}_{}", bridge.from, bridge.to),
                           portFlow(bridge.from, true), 1.0);
      model.addLessOrEqual(std::format("bridge_once_back_{}_{}", bridge.from, bridge.to),
                           portFlow(bridge.to, true), 1.0);
    }

    // The free space the wire has to cross to get there. A lattice edge says
    // a wire may run somewhere; it does not say the corridor it runs through
    // has room for it. That is what the capacity chains measure, and without
    // them the circuit surfaces wherever the wire is shortest — typically at
    // the coupler it is already standing on — rather than where the chip can
    // actually carry it out to a launcher.
    addChainCapacity(model, capacity, circuit, portFlow);

    static_cast<void>(chip);
    model.setSense(milp::Sense::Minimize);
    return arcs;
  }

  /// One link of a chain as the flow model sees it: its plan node, its parent
  /// and its children, all by position in the tree rather than in the plan.
  ///
  /// Two chains may share a subtree, and a shared link carries its own flow
  /// in each, so the tree is walked into fresh positions instead of being
  /// indexed by plan node.
  struct ChainLink {
    std::uint32_t node = 0;
    std::optional<std::size_t> parent;
    std::vector<std::size_t> children;
  };

  /// The links of one chain, rooted at `root`.
  [[nodiscard]] static std::vector<ChainLink> chainLinks(const CapacityPlanT& capacity,
                                                         const std::uint32_t root) {
    std::vector<ChainLink> links{{.node = root, .parent = std::nullopt, .children = {}}};
    for (std::size_t at = 0; at < links.size(); ++at) {
      for (const auto next : capacity.nodes[links[at].node]->next) {
        links[at].children.push_back(links.size());
        links.push_back({.node = next, .parent = at, .children = {}});
      }
    }
    return links;
  }

  /// Tie the inner circuit to the free space it has to cross.
  ///
  /// Each chain that describes outer space carries an integer flow of its own:
  /// its launcher supplies, each of its gates passes at most its capacity, and
  /// each of its targets draws one wire. Where a target is the outer end of a
  /// bridge, what it draws is exactly what the inner circuit sends out through
  /// that bridge — so an inner wire may surface at an outer port only when the
  /// chain behind that port can carry it to a launcher.
  ///
  /// A chain that already belongs to the inner circuit is left out: its
  /// targets are what the circuit has to reach, or the ports it reaches them
  /// through, and constraining those against themselves would say nothing.
  template <typename PortFlow>
  static void addChainCapacity(milp::Model& model, const CapacityPlanT& capacity,
                               const InnerCircuit& circuit, PortFlow&& portFlow) {
    // Which port a bridge surfaces at, and which ports the inner circuit
    // already owns.
    std::unordered_map<std::uint32_t, std::uint32_t> insideOf;
    for (const auto& bridge : circuit.outerBridges) {
      insideOf.emplace(bridge.to, bridge.from);
    }
    std::unordered_set<std::uint32_t> claimed(circuit.targets.begin(), circuit.targets.end());
    std::unordered_set<std::uint32_t> outerSide;
    for (const auto& bridge : circuit.outerBridges) {
      claimed.insert(bridge.from);
      outerSide.insert(bridge.to);
    }
    for (const auto& bridge : circuit.bridges) {
      if (!outerSide.contains(bridge.to)) {
        // An internal bridge: both of its ends are the circuit's own.
        claimed.insert(bridge.from);
        claimed.insert(bridge.to);
      }
    }

    for (std::size_t chain = 0; chain < capacity.chains.size(); ++chain) {
      const auto links = chainLinks(capacity, capacity.chains[chain]);

      const auto isTarget = [&](const ChainLink& link) {
        return capacity.nodes[link.node]->kind == fba::CapacityElement::Target;
      };
      const auto portOf = [&](const ChainLink& link) { return capacity.nodes[link.node]->id; };

      if (std::ranges::any_of(links, [&](const ChainLink& link) {
            return isTarget(link) && claimed.contains(portOf(link));
          })) {
        continue;
      }

      // What the chain can actually reach: a launcher supplies it, and a gate
      // with no room left stops it. A target the supply cannot reach draws
      // nothing — and nothing may surface there either, because a wire that
      // came out at such a port would have no free space to leave through.
      const auto supplied = suppliedLinks(capacity, links);

      const auto demand = static_cast<double>(std::ranges::count_if(links, isTarget));
      std::map<std::pair<std::size_t, std::size_t>, milp::Var> edges;
      for (std::size_t at = 0; at < links.size(); ++at) {
        for (const auto child : links[at].children) {
          edges.emplace(std::pair{at, child},
                        model.addInteger(std::format("chain_{}_{}_{}", chain, at, child), 0.0,
                                         demand));
          edges.emplace(std::pair{child, at},
                        model.addInteger(std::format("chain_{}_{}_{}", chain, child, at), 0.0,
                                         demand));
        }
      }

      // Everything a link sends, everything it receives, and the same two
      // over its parent edge alone, which is the one a gate limits.
      const auto around = [&](const std::size_t at, const bool out, const bool parentOnly) {
        milp::LinearExpr expression;
        if (const auto parent = links[at].parent) {
          expression.add(edges.at(out ? std::pair{at, *parent} : std::pair{*parent, at}), 1.0);
        }
        if (!parentOnly) {
          for (const auto child : links[at].children) {
            expression.add(edges.at(out ? std::pair{at, child} : std::pair{child, at}), 1.0);
          }
        }
        return expression;
      };

      for (std::size_t at = 0; at < links.size(); ++at) {
        const auto& element = *capacity.nodes[links[at].node];
        const auto in = around(at, false, false);
        const auto out = around(at, true, false);
        switch (element.kind) {
        case fba::CapacityElement::Launcher:
          // The launcher supplies the chain, at most one wire per target.
          model.addLessOrEqual(std::format("chain_{}_launcher_{}", chain, at), out, demand);
          break;
        case fba::CapacityElement::Bottleneck:
          // A gate passes on what it takes, and no more than it fits.
          model.addEqual(std::format("chain_{}_gate_{}", chain, at), in - out, 0.0);
          model.addLessOrEqual(std::format("chain_{}_gate_out_{}", chain, at),
                               around(at, true, true), element.capacity);
          model.addLessOrEqual(std::format("chain_{}_gate_in_{}", chain, at),
                               around(at, false, true), element.capacity);
          break;
        case fba::CapacityElement::Target: {
          const auto inside = insideOf.find(element.id);
          if (!supplied[at]) {
            // Cut off from every launcher of its chain. Insisting it draws a
            // wire would only make the model unsolvable, and letting one
            // surface here would claim free space that does not exist.
            model.addEqual(std::format("chain_{}_cut_in_{}", chain, at), in, 0.0);
            model.addEqual(std::format("chain_{}_cut_out_{}", chain, at), out, 0.0);
            if (inside != insideOf.end()) {
              model.addEqual(std::format("chain_{}_sealed_{}", chain, at),
                             portFlow(inside->second, true), 0.0);
            }
            break;
          }
          if (inside == insideOf.end()) {
            // A plain outer target draws the one wire it is there for.
            model.addEqual(std::format("chain_{}_target_{}", chain, at), in - out, 1.0);
            break;
          }
          // A target the inner circuit can surface at draws exactly what the
          // circuit sends out through the bridge, and never the other way
          // round: the wire leaves the inner region there, it does not enter.
          model.addEqual(std::format("chain_{}_bridge_{}", chain, at),
                         in - out - portFlow(inside->second, true), 0.0);
          model.addLessOrEqual(std::format("chain_{}_bridge_cap_{}", chain, at), in - out, 1.0);
          model.addEqual(std::format("chain_{}_bridge_back_{}", chain, at),
                         portFlow(inside->second, false), 0.0);
          break;
        }
        case fba::CapacityElement::Unset:
        default:
          break;
        }
      }
    }
  }

  /// Which links of a chain a launcher can still reach: the walk starts at
  /// every launcher of the chain and stops at a gate with no room left.
  [[nodiscard]] static std::vector<bool> suppliedLinks(const CapacityPlanT& capacity,
                                                       const std::vector<ChainLink>& links) {
    std::vector<bool> supplied(links.size(), false);
    std::vector<std::size_t> pending;
    for (std::size_t at = 0; at < links.size(); ++at) {
      if (capacity.nodes[links[at].node]->kind == fba::CapacityElement::Launcher) {
        supplied[at] = true;
        pending.push_back(at);
      }
    }
    while (!pending.empty()) {
      const auto at = pending.back();
      pending.pop_back();
      const auto step = [&](const std::size_t other) {
        const auto& element = *capacity.nodes[links[other].node];
        if (supplied[other] ||
            (element.kind == fba::CapacityElement::Bottleneck && element.capacity == 0)) {
          return;
        }
        supplied[other] = true;
        pending.push_back(other);
      };
      if (const auto parent = links[at].parent) {
        step(*parent);
      }
      for (const auto child : links[at].children) {
        step(child);
      }
    }
    return supplied;
  }

  /// Read the solved flow back into the artifact, and report which ports the
  /// circuit actually left through.
  [[nodiscard]] static std::unordered_set<std::uint32_t>
  readBack(GlobalRoutingT& routing, const ChipT& chip, const std::vector<Lattice>& lattices,
           const std::vector<Arc>& arcs, const milp::Solution& solution,
           const InnerCircuit& circuit) {
    const std::unordered_set<std::uint32_t> targets(circuit.targets.begin(),
                                                    circuit.targets.end());

    // The lattices, with the selected edges marked, so a picture of the
    // inner circuit can be drawn from the artifact alone.
    std::vector<std::map<std::pair<std::size_t, std::size_t>, std::uint32_t>> edgeIndex(
        lattices.size());
    for (std::size_t index = 0; index < lattices.size(); ++index) {
      const auto& lattice = lattices[index];
      auto entry = std::make_unique<fba::LatticeT>();
      entry->points.reserve(lattice.nodes.size());
      for (const auto& node : lattice.nodes) {
        entry->points.push_back(node.position);
      }
      for (std::size_t from = 0; from < lattice.nodes.size(); ++from) {
        for (const auto to : lattice.nodes[from].neighbors) {
          if (from >= to) {
            continue;
          }
          edgeIndex[index].emplace(std::pair{from, to},
                                   static_cast<std::uint32_t>(entry->edges.size() / 2));
          entry->edges.push_back(static_cast<std::uint32_t>(from));
          entry->edges.push_back(static_cast<std::uint32_t>(to));
        }
      }
      routing.lattices.push_back(std::move(entry));
    }

    // The adjacency the solution selected, per lattice.
    std::vector<std::vector<std::vector<std::size_t>>> selected(lattices.size());
    std::unordered_set<std::uint32_t> sources;
    for (std::size_t index = 0; index < lattices.size(); ++index) {
      selected[index].assign(lattices[index].nodes.size(), {});
    }
    for (const auto& arc : arcs) {
      if (!solution.isSet(arc.variable)) {
        continue;
      }
      selected[arc.lattice][arc.from].push_back(arc.to);
      selected[arc.lattice][arc.to].push_back(arc.from);

      const auto key = std::pair{std::min(arc.from, arc.to), std::max(arc.from, arc.to)};
      if (const auto found = edgeIndex[arc.lattice].find(key);
          found != edgeIndex[arc.lattice].end()) {
        routing.lattices[arc.lattice]->selected.push_back(found->second);
      }
      // A port with an outgoing arc is where a wire of the inner circuit
      // leaves, which is what the ring has to keep.
      if (const auto port = lattices[arc.lattice].nodes[arc.from].port;
          port.has_value() && !targets.contains(*port)) {
        sources.insert(*port);
      }
    }
    for (auto& lattice : routing.lattices) {
      std::ranges::sort(lattice->selected);
      const auto duplicates = std::ranges::unique(lattice->selected);
      lattice->selected.erase(duplicates.begin(), duplicates.end());
    }

    // Each connected component of the selected edges is one wire. Its two
    // port ends are the connection the later stages route.
    for (std::size_t index = 0; index < lattices.size(); ++index) {
      std::vector<bool> seen(lattices[index].nodes.size(), false);
      for (std::size_t start = 0; start < lattices[index].nodes.size(); ++start) {
        if (seen[start] || selected[index][start].empty()) {
          continue;
        }
        std::vector<std::uint32_t> ends;
        std::vector<std::size_t> pending{start};
        seen[start] = true;
        while (!pending.empty()) {
          const auto node = pending.back();
          pending.pop_back();
          if (const auto port = lattices[index].nodes[node].port; port.has_value()) {
            ends.push_back(*port);
          }
          for (const auto next : selected[index][node]) {
            if (!seen[next]) {
              seen[next] = true;
              pending.push_back(next);
            }
          }
        }
        if (ends.size() != 2) {
          continue;
        }
        // The target end is the one the wire ends at; the other is its
        // source.
        const auto targetFirst = targets.contains(ends[0]);
        auto connection = std::make_unique<fbd::ConnectionT>();
        connection->source =
            std::make_unique<fbd::PortRef>(targetFirst ? ends[1] : ends[0]);
        connection->target = fbd::PortRef(targetFirst ? ends[0] : ends[1]);
        connection->source_role = fbd::AssignedRole::ConventionalSource;
        connection->target_role = fbd::AssignedRole::ConventionalTarget;
        routing.connections.push_back(std::move(connection));
      }
    }
    static_cast<void>(chip);
    return sources;
  }

  /// The ring the assignment consumes, and the resonators it has to feed.
  ///
  /// The configured ring is kept in order, except that a port on the far
  /// side of a bridge is dropped unless the inner circuit surfaced there:
  /// such a port carries an inner wire, and only then is it something the
  /// outer assignment has to route.
  static void fillRing(GlobalRoutingT& routing, const ChipT& chip, const ConfigT& config,
                       const InnerCircuit& circuit,
                       const std::unordered_set<std::uint32_t>& sources) {
    // Which inner port each ring port is the far side of. A wire surfaces at
    // the ring port, but it is the inner port that carries it, so that is
    // what the solved flow names as a source.
    std::unordered_map<std::uint32_t, std::uint32_t> insideOf;
    for (const auto& bridge : circuit.outerBridges) {
      insideOf.emplace(bridge.to, bridge.from);
    }

    const auto byLabel = portsByLabel(chip);
    for (const auto& label : config.ports->sequences->all_outer) {
      const auto port = byLabel.at(label);
      const auto inside = insideOf.find(port);
      if (inside == insideOf.end() || sources.contains(inside->second)) {
        routing.outer_ring.emplace_back(port);
      }
    }

    // Where each inner wire surfaces, by the inner port it left through.
    std::unordered_map<std::uint32_t, std::uint32_t> surfaceOf;
    for (const auto& bridge : circuit.outerBridges) {
      surfaceOf.emplace(bridge.from, bridge.to);
    }

    // A resonator inside the ring is fed where its own inner wire surfaces.
    // The assignment works on the ring, so the ring port the wire came out of
    // is a resonator to it as well; the qubit's own port stays, because that
    // is still the resonator the chip has and the two are the same wire seen
    // from either end.
    std::unordered_set<std::uint32_t> resonators;
    for (std::uint32_t index = 0; index < chip.ports.size(); ++index) {
      if (chip.ports[index]->role == fbd::UnassignedRole::Resonator) {
        resonators.insert(index);
      }
    }
    std::vector<std::uint32_t> surfaced;
    for (const auto& connection : routing.connections) {
      if (connection->source == nullptr || !resonators.contains(connection->target.index())) {
        continue;
      }
      if (const auto surface = surfaceOf.find(connection->source->index());
          surface != surfaceOf.end()) {
        surfaced.push_back(surface->second);
      }
    }
    resonators.insert(surfaced.begin(), surfaced.end());

    std::vector<std::uint32_t> ordered(resonators.begin(), resonators.end());
    std::ranges::sort(ordered);
    for (const auto port : ordered) {
      routing.resonators.emplace_back(port);
    }
  }
};

} // namespace

InnerCircuit innerCircuitOf(const ChipT& chip, const ConfigT& config) {
  if (config.ports == nullptr || config.ports->sequences == nullptr) {
    throw std::invalid_argument("the configuration carries no port ring");
  }
  const auto byLabel = portsByLabel(chip);

  std::unordered_set<std::uint32_t> ring;
  for (const auto& label : config.ports->sequences->all_outer) {
    const auto found = byLabel.find(label);
    if (found == byLabel.end()) {
      throw std::invalid_argument(
          std::format("the ring names '{}', which the chip does not carry", label));
    }
    ring.insert(found->second);
  }

  InnerCircuit circuit;
  for (std::uint32_t index = 0; index < chip.ports.size(); ++index) {
    const auto& port = *chip.ports[index];
    if (!design::isRoutable(port.role) || ring.contains(index)) {
      continue;
    }
    circuit.innerPorts.push_back(index);
  }
  const std::unordered_set<std::uint32_t> inner(circuit.innerPorts.begin(),
                                                circuit.innerPorts.end());

  for (const auto& component : design::componentsOf(chip, config.ports->bridge_pairs)) {
    for (const auto& pair : component.bridges) {
      // A crossing runs from the inside out. When the ring carries one of the
      // two ports, that port is the one the assignment sees and so is the
      // pair's far side.
      const auto firstInRing = ring.contains(pair.first);
      const auto secondInRing = ring.contains(pair.second);
      if (firstInRing && secondInRing) {
        // Both ends outside leaves no inside for the wire to come from. The
        // ring names a port the inner circuit can never surface at, which is
        // a ring that does not match the bridge rules rather than a circuit
        // that cannot be solved.
        throw std::invalid_argument(
            std::format("the ring carries both ends of the bridge '{}' - '{}'",
                        chip.ports[pair.first]->label, chip.ports[pair.second]->label));
      }
      const PortBridge bridge{.from = secondInRing ? pair.first : pair.second,
                              .to = secondInRing ? pair.second : pair.first};
      circuit.bridges.push_back(bridge);
      if (firstInRing || secondInRing) {
        circuit.outerBridges.push_back(bridge);
      }
    }
    // A port no bridge claims ends a wire, and an inner one is something the
    // circuit has to reach: a qubit's ports are what an inner qubit needs the
    // circuit for, and a coupler's leftover port is the one that faces the
    // qubit pair it joins.
    for (const auto port : component.ends) {
      if (inner.contains(port)) {
        circuit.targets.push_back(port);
      }
    }
  }
  std::ranges::sort(circuit.targets);
  return circuit;
}

std::unique_ptr<IGlobalRouter> makeHananMilpRouter() {
  return std::make_unique<HananMilpRouter>();
}

} // namespace mqt::scpd::pipeline
