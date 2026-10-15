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
#include "mqt-scpd/flatbuffers/geometry.hpp"
#include "mqt-scpd/geometry/Geometry.hpp"
#include "mqt-scpd/milp/Model.hpp"
#include "mqt-scpd/pipeline/Report.hpp"
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
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace mqt::scpd::pipeline {
namespace {

namespace fba = flatbuffers::artifacts;
namespace fbd = flatbuffers::design;
namespace fbg = flatbuffers::geometry;

/// Two lattice coordinates closer than this are the same line. The chip is
/// measured in micrometres, so this is far below anything the input resolves.
constexpr double COORDINATE_EPSILON = 1.0e-7;

/// An edge runs the way a port faces only where the cosine of the angle
/// between the two exceeds this. The direction of a port comes from a sine
/// and a cosine, and for a port that faces along an axis, the component
/// across that axis is a rounding error of either sign rather than zero.
constexpr double DIRECTION_EPSILON = 1.0e-9;

/// The port each label names.
std::unordered_map<std::string, std::uint32_t> portsByLabel(const ChipT& chip) {
  std::unordered_map<std::string, std::uint32_t> byLabel;
  byLabel.reserve(chip.ports.size());
  for (std::uint32_t index = 0; index < chip.ports.size(); ++index) {
    byLabel.emplace(chip.ports[index]->label, index);
  }
  return byLabel;
}

/// Refuses a capacity plan whose chains the stage cannot walk: a chain or a
/// link that names a node the plan does not carry, a target that names a port
/// the chip does not carry, or a chain that leads back into itself. The
/// Capacity stage writes no such plan, so the plan is from another chip or
/// was edited.
void checkCapacityPlan(const ChipT& chip, const CapacityPlanT& capacity) {
  const auto nodes = capacity.nodes.size();
  const auto refuse = [](const std::string& what) {
    throw std::invalid_argument(
        std::format("the capacity plan does not fit the chip: {}", what));
  };
  for (const auto root : capacity.chains) {
    if (root >= nodes) {
      refuse(std::format("a chain starts at node {} of {}", root, nodes));
    }
  }
  for (std::size_t index = 0; index < nodes; ++index) {
    const auto& node = *capacity.nodes[index];
    if (node.kind == fba::CapacityElement::Target &&
        node.id >= chip.ports.size()) {
      refuse(std::format("node {} names port {} of {}", index, node.id,
                         chip.ports.size()));
    }
    for (const auto next : node.next) {
      if (next >= nodes) {
        refuse(
            std::format("node {} leads to node {} of {}", index, next, nodes));
      }
    }
  }
  // A depth-first walk that meets a node it is still inside has found a
  // cycle. Chains may share a subtree, so a node it has left is no cycle.
  enum class Visit : std::uint8_t { Unseen, Inside, Left };
  std::vector<Visit> visits(nodes, Visit::Unseen);
  for (const auto root : capacity.chains) {
    if (visits[root] != Visit::Unseen) {
      continue;
    }
    // Each entry is a node and how many of its links the walk has taken.
    std::vector<std::pair<std::uint32_t, std::size_t>> path{{root, 0}};
    visits[root] = Visit::Inside;
    while (!path.empty()) {
      auto& [node, taken] = path.back();
      const auto& next = capacity.nodes[node]->next;
      if (taken == next.size()) {
        visits[node] = Visit::Left;
        path.pop_back();
        continue;
      }
      const auto child = next[taken++];
      if (visits[child] == Visit::Inside) {
        refuse(std::format("node {} leads back to node {}", node, child));
      }
      if (visits[child] == Visit::Unseen) {
        visits[child] = Visit::Inside;
        path.emplace_back(child, 0);
      }
    }
  }
}

/// One Hanan lattice: the rectilinear grid that a set of ports induces.
struct Lattice {
  /// One crossing of the lattice.
  struct Node {
    /// Where the node is.
    fbg::Point position;
    /// The port this node is, or none.
    std::optional<std::uint32_t> port;
    /// The axis neighbours, by node index.
    std::vector<std::size_t> neighbors;
  };
  /// The nodes, row by row.
  std::vector<Node> nodes;
  /// The node of each port that takes part.
  std::unordered_map<std::uint32_t, std::size_t> nodeOfPort;
};

/// The point that the lattice lines of a port run through.
///
/// The point lies in front of the port, along the direction the port faces,
/// rather than on the artwork of its component. Without that step every
/// lattice line of a qubit would run through the qubit.
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
  for (std::size_t index = 0, previous = vertices.size() - 1;
       index < vertices.size(); previous = index++) {
    const auto& a = vertices[index];
    const auto& b = vertices[previous];
    if ((a.y() > point.y()) != (b.y() > point.y()) &&
        point.x() <
            ((b.x() - a.x()) * (point.y() - a.y()) / (b.y() - a.y())) + a.x()) {
      inside = !inside;
    }
  }
  return inside;
}

/// Whether a point lies on a side of a polygon.
bool onBorder(const fbg::PolygonT& polygon, const fbg::Point& point) {
  const auto& vertices = polygon.vertices;
  for (std::size_t index = 0, previous = vertices.size() - 1;
       index < vertices.size(); previous = index++) {
    const auto side = geometry::subtract(vertices[index], vertices[previous]);
    const auto offset = geometry::subtract(point, vertices[previous]);
    const auto length = geometry::dot(side, side);
    const auto along =
        length > 0.0
            ? std::clamp(geometry::dot(offset, side) / length, 0.0, 1.0)
            : 0.0;
    if (geometry::distance(point,
                           {vertices[previous].x() + (along * side.x()),
                            vertices[previous].y() + (along * side.y())}) <
        COORDINATE_EPSILON) {
      return true;
    }
  }
  return false;
}

/// Whether a point lies on chip artwork.
///
/// Every polygon of the chip is artwork, the first one included: the chip
/// input carries no outline, as the obstacle raster also assumes.
bool onArtwork(const ChipT& chip, const fbg::Point& point) {
  return std::ranges::any_of(chip.obstacles, [&](const auto& polygon) {
    return insidePolygon(*polygon, point);
  });
}

/// Whether a lattice edge runs through chip artwork.
///
/// The edge is cut where it crosses the border of a polygon, and each piece
/// is tested at its midpoint. The anchor of a port may lie on the artwork of
/// its component, and a wire from there has to leave that artwork first. So
/// the piece next to a port end is exempt if all of these hold:
///
/// - the anchor lies strictly inside the polygon,
/// - the port itself lies inside the polygon or on its border, so the
///   polygon is artwork of the port's own component,
/// - the edge runs the way the port faces, and
/// - the edge leaves the polygon there.
///
/// Every other piece on artwork blocks the edge. @p fromPort and @p toPort
/// are the ports at the two ends, or null at a node without a port.
bool throughArtwork(const ChipT& chip, const fbg::Point& from,
                    const fbg::Point& to, const fbd::PortT* fromPort,
                    const fbd::PortT* toPort) {
  const auto direction = geometry::subtract(to, from);
  for (const auto& polygon : chip.obstacles) {
    const auto& vertices = polygon->vertices;
    if (vertices.size() < 3) {
      continue;
    }
    const auto box = geometry::boundingBox(*polygon);
    if (std::max(from.x(), to.x()) < box.minX ||
        std::min(from.x(), to.x()) > box.maxX ||
        std::max(from.y(), to.y()) < box.minY ||
        std::min(from.y(), to.y()) > box.maxY) {
      continue;
    }
    // The edge parameters where the edge crosses the border, 0 at from and 1
    // at to.
    std::vector<double> cuts{0.0, 1.0};
    for (std::size_t index = 0, previous = vertices.size() - 1;
         index < vertices.size(); previous = index++) {
      const auto side = geometry::subtract(vertices[index], vertices[previous]);
      const auto denominator = geometry::cross(direction, side);
      if (std::abs(denominator) < COORDINATE_EPSILON) {
        // A side parallel to the edge cuts nothing. The pieces next to that
        // side decide.
        continue;
      }
      const auto offset = geometry::subtract(vertices[previous], from);
      const auto along = geometry::cross(offset, side) / denominator;
      const auto across = geometry::cross(offset, direction) / denominator;
      if (along > 0.0 && along < 1.0 && across >= 0.0 && across <= 1.0) {
        cuts.push_back(along);
      }
    }
    // A cut through a vertex is found on both of its sides. The empty piece
    // between the two would be tested at the vertex, which is no test.
    std::ranges::sort(cuts);
    cuts.erase(std::ranges::unique(cuts).begin(), cuts.end());
    const auto pieces = cuts.size() - 1;
    std::vector<bool> covered(pieces);
    for (std::size_t piece = 0; piece < pieces; ++piece) {
      const auto middle = (cuts[piece] + cuts[piece + 1]) / 2.0;
      covered[piece] =
          insidePolygon(*polygon, {from.x() + (middle * direction.x()),
                                   from.y() + (middle * direction.y())});
    }
    // A port end leaves the artwork only if its anchor lies strictly inside
    // the polygon of its own component, the edge runs the way the port
    // faces, and the next piece lies outside. An anchor on the border, an
    // anchor on the artwork of another component, an edge back through the
    // component, or an edge that stays inside has no exit to exempt.
    const auto leaves = [&](const fbd::PortT* port, const fbg::Point& end,
                            const fbg::Point& away, const std::size_t next) {
      if (port == nullptr || pieces < 2 || covered[next] ||
          !insidePolygon(*polygon, end) || onBorder(*polygon, end)) {
        return false;
      }
      if (!insidePolygon(*polygon, port->center) &&
          !onBorder(*polygon, port->center)) {
        return false;
      }
      const auto radians = port->orientation * std::numbers::pi / 180.0;
      const auto along = geometry::subtract(away, end);
      return geometry::dot(along, {std::cos(radians), std::sin(radians)}) >
             DIRECTION_EPSILON * geometry::norm(along);
    };
    const auto fromLeaves = leaves(fromPort, from, to, 1);
    const auto toLeaves = leaves(toPort, to, from, pieces - 2);
    for (std::size_t piece = 0; piece < pieces; ++piece) {
      if ((piece == 0 && fromLeaves) || (piece + 1 == pieces && toLeaves)) {
        continue;
      }
      if (covered[piece]) {
        return true;
      }
    }
  }
  return false;
}

/// The lines of a lattice through @p values, in ascending order.
///
/// Two anchors on one line can differ in the last bits, since each is a
/// port centre moved along its direction. A value within COORDINATE_EPSILON
/// of the first value of its run is the same line, so the run gives one line
/// and not several parallel ones a hair apart.
std::vector<double> linesThrough(const std::set<double>& values) {
  std::vector<double> lines;
  for (const auto value : values) {
    if (lines.empty() || value - lines.back() > COORDINATE_EPSILON) {
      lines.push_back(value);
    }
  }
  return lines;
}

/// The Hanan lattice of a set of ports: one line through the x of every
/// anchor and one through its y.
///
/// A node on chip artwork is removed, and every edge that touched it goes
/// with it. An edge that runs through artwork goes as well, since its two
/// ends may lie on either side of it. Without that the inner circuit routes
/// through the couplers and the qubits it routes between, and its objective
/// is the length of paths that nobody can build.
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

  const auto columns = linesThrough(xs);
  const auto rows = linesThrough(ys);
  lattice.nodes.reserve(columns.size() * rows.size());
  for (const auto row : rows) {
    for (const auto column : columns) {
      lattice.nodes.push_back(
          {.position = {column, row}, .port = std::nullopt, .neighbors = {}});
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
        std::ranges::lower_bound(columns,
                                 anchors[index].x() - COORDINATE_EPSILON) -
        columns.begin());
    const auto row = static_cast<std::size_t>(
        std::ranges::lower_bound(rows,
                                 anchors[index].y() - COORDINATE_EPSILON) -
        rows.begin());
    const auto node = at(column, row);
    lattice.nodes[node].port = ports[index];
    lattice.nodeOfPort.emplace(ports[index], node);
  }

  // A node on artwork goes, and so does every edge that reached it. The node
  // of a port stays whatever it sits on: a wire has to start or end there,
  // and without the node the model could not say so.
  std::vector<bool> removed(lattice.nodes.size(), false);
  for (std::size_t node = 0; node < lattice.nodes.size(); ++node) {
    if (!lattice.nodes[node].port.has_value() &&
        onArtwork(chip, lattice.nodes[node].position)) {
      removed[node] = true;
    }
  }
  for (std::size_t node = 0; node < lattice.nodes.size(); ++node) {
    if (removed[node]) {
      lattice.nodes[node].neighbors.clear();
      continue;
    }
    std::erase_if(lattice.nodes[node].neighbors,
                  [&](const std::size_t other) { return removed[other]; });
  }

  // Two nodes off the artwork can still have artwork between them. Each edge
  // is tested once, and a blocked edge goes in both directions.
  std::set<std::pair<std::size_t, std::size_t>> blocked;
  const auto portAt = [&](const Lattice::Node& end) -> const fbd::PortT* {
    return end.port.has_value() ? chip.ports[*end.port].get() : nullptr;
  };
  for (std::size_t node = 0; node < lattice.nodes.size(); ++node) {
    for (const auto other : lattice.nodes[node].neighbors) {
      const auto& from = lattice.nodes[node];
      const auto& to = lattice.nodes[other];
      if (node < other && throughArtwork(chip, from.position, to.position,
                                         portAt(from), portAt(to))) {
        blocked.emplace(node, other);
      }
    }
  }
  for (std::size_t node = 0; node < lattice.nodes.size(); ++node) {
    std::erase_if(lattice.nodes[node].neighbors, [&](const std::size_t other) {
      return blocked.contains(std::minmax(node, other));
    });
  }
  return lattice;
}

/// The distance between two lattice nodes, which is the weight of the edge
/// between them.
double edgeWeight(const Lattice& lattice, const std::size_t from,
                  const std::size_t to) {
  return geometry::distance(lattice.nodes[from].position,
                            lattice.nodes[to].position);
}

/// One directed lattice edge, and the variable that selects it.
struct Arc {
  std::size_t lattice = 0;
  std::size_t from = 0;
  std::size_t to = 0;
  milp::Var variable;
};

/// The marker of a ring port that a chain may find no room for.
struct Shortfall {
  milp::Var marker;
  std::uint32_t port = 0;
};

/// What the solved flow says beyond the artifact.
struct Circuit {
  /// The ports a wire of the circuit leaves from.
  std::unordered_set<std::uint32_t> sources;
  /// The length of each connection, in the order of the connections.
  std::vector<double> lengths;
};

/// The global router of the first release.
class HananMilpRouter final : public IGlobalRouter {
public:
  [[nodiscard]] GlobalRoutingT run(const ChipT& chip,
                                   const CapacityPlanT& capacity,
                                   const ConfigT& config,
                                   const Report& report) const override {
    const auto circuit = innerCircuitOf(chip, config);
    checkCapacityPlan(chip, capacity);
    if (report.wants(Detail::Steps)) {
      report.line(
          Detail::Steps,
          std::format(
              "{}{}{}{}{}{}{}",
              counted(circuit.innerPorts.size(), "inner port"),
              FIGURE_SEPARATOR, counted(circuit.targets.size(), "target"),
              FIGURE_SEPARATOR, counted(circuit.bridges.size(), "bridge"),
              FIGURE_SEPARATOR,
              counted(circuit.outerBridges.size(), "ring bridge")));
    }
    const auto reach =
        config.rules != nullptr ? config.rules->min_straight_length : 0.0;

    GlobalRoutingT routing;
    routing.objective = 0.0;

    // A chip whose ring is its whole port set has no inner circuit. That is
    // a valid state of the pipeline, and the artifact is written empty.
    if (circuit.targets.empty()) {
      fillRing(routing, chip, config, circuit, {});
      finish(routing, chip, circuit, {}, report);
      return routing;
    }

    const auto lattices = latticesOf(chip, capacity, circuit, reach, report);
    if (lattices.empty()) {
      // The chip has inner targets, but no capacity chain offers a lattice
      // that could reach one: every chain is either all targets, so nothing
      // could supply them, or a single point. There is no circuit to solve,
      // and a solver takes no empty model.
      fillRing(routing, chip, config, circuit, {});
      finish(routing, chip, circuit, {}, report);
      return routing;
    }

    // The first model asks the chains for room for every ring port behind
    // their gates. Only where the gates have no room for all of them does a
    // second model let ring ports go short, so that a chip with room keeps
    // the circuit of the first.
    std::vector<milp::Var> unreached;
    std::vector<Shortfall> crowded;
    double penalty = 0.0;
    std::vector<Arc> arcs;
    milp::Solution solution;
    for (const bool shortfalls : {false, true}) {
      report.progress("building the model", "", 0, 0);
      milp::Model model("inner-circuit");
      unreached.clear();
      crowded.clear();
      arcs = build(model, capacity, lattices, circuit,
                   internalBridgesAllowed(config), shortfalls, unreached,
                   crowded, penalty);
      solution = solveWith(model, config, report, "solving the inner circuit");
      if (solution.status != milp::SolveStatus::Infeasible) {
        break;
      }
    }
    if (!solution.hasValues()) {
      throw std::runtime_error(
          std::format("the inner circuit did not solve: {}{}",
                      milp::statusName(solution.status),
                      solution.message.empty() ? "" : ", " + solution.message));
    }
    // The objective is the length of the circuit. The connections already
    // say which target the lattice could not serve, and the shortfalls which
    // ring port a chain had no room for, so the penalty of each comes back
    // out of the cost.
    auto missed = std::ranges::count_if(unreached, [&](const milp::Var marker) {
      return solution.isSet(marker);
    });
    std::set<std::uint32_t> withoutRoom;
    for (const auto& [marker, port] : crowded) {
      if (solution.isSet(marker)) {
        ++missed;
        withoutRoom.insert(port);
      }
    }
    routing.objective =
        solution.objective - (penalty * static_cast<double>(missed));

    const auto solved = readBack(routing, lattices, arcs, solution, circuit);
    fillRing(routing, chip, config, circuit, solved.sources);
    finish(routing, chip, circuit, solved.lengths, report,
           {withoutRoom.begin(), withoutRoom.end()});
    return routing;
  }

private:
  /// Whether an inner wire may cross a component that the outer ring does not
  /// reach. Off unless the configuration says otherwise.
  [[nodiscard]] static bool internalBridgesAllowed(const ConfigT& config) {
    return config.stages != nullptr && config.stages->global != nullptr &&
           config.stages->global->internal_bridges;
  }

  /// Reports the ring, the wires and the result of the stage.
  ///
  /// A target counts as a fail when no connection reaches it, whatever the
  /// reason: no lattice carries it, or the model left it unreached. A ring
  /// port in @p withoutRoom counts as a fail too: a chain behind it has no
  /// room for its wire to a launcher.
  static void finish(const GlobalRoutingT& routing, const ChipT& chip,
                     const InnerCircuit& circuit,
                     const std::vector<double>& lengths, const Report& report,
                     const std::vector<std::uint32_t>& withoutRoom = {}) {
    if (report.wants(Detail::Steps)) {
      report.line(Detail::Steps,
                  std::format("{}{}{}",
                              counted(routing.outer_ring.size(), "ring port"),
                              FIGURE_SEPARATOR,
                              counted(routing.resonators.size(), "resonator")));
      if (!withoutRoom.empty()) {
        report.line(Detail::Steps,
                    std::format("{} without room to a launcher",
                                counted(withoutRoom.size(), "ring port")));
      }
    }
    if (report.wants(Detail::Items)) {
      for (const auto port : withoutRoom) {
        report.line(Detail::Items, std::format("{} has no room to a launcher",
                                               chip.ports[port]->label));
      }
      for (std::size_t index = 0; index < routing.connections.size(); ++index) {
        const auto& connection = *routing.connections[index];
        report.line(Detail::Items,
                    std::format("{} {} {}{}length {:.2f}",
                                chip.ports[connection.source->index()]->label,
                                ARROW,
                                chip.ports[connection.target.index()]->label,
                                FIGURE_SEPARATOR, lengths[index]));
      }
    }

    const std::unordered_set<std::uint32_t> targets(circuit.targets.begin(),
                                                    circuit.targets.end());
    std::unordered_set<std::uint32_t> reached;
    for (const auto& connection : routing.connections) {
      if (targets.contains(connection->target.index())) {
        reached.insert(connection->target.index());
      }
    }
    const auto fails =
        circuit.targets.size() - reached.size() + withoutRoom.size();
    if (circuit.targets.empty()) {
      report.result({{.text = "no inner circuit"},
                     {.text = counted(routing.outer_ring.size(), "ring port")}},
                    fails);
      return;
    }
    report.result({{.text = counted(routing.connections.size(), "inner wire")},
                   {.text = counted(routing.lattices.size(), "lattice")},
                   {.text = std::format("length {:.2f}", routing.objective)}},
                  fails);
  }

  /// The lattices the model is built over: one per capacity chain that has an
  /// inner target to serve.
  [[nodiscard]] static std::vector<Lattice>
  latticesOf(const ChipT& chip, const CapacityPlanT& capacity,
             const InnerCircuit& circuit, const double reach,
             const Report& report) {
    const std::unordered_set<std::uint32_t> targets(circuit.targets.begin(),
                                                    circuit.targets.end());
    std::vector<Lattice> lattices;
    for (std::size_t chain = 0; chain < capacity.chains.size(); ++chain) {
      report.progress("building the lattices", "", chain,
                      capacity.chains.size());
      std::vector<std::uint32_t> ports;
      std::unordered_set<std::uint32_t> seen;
      collectTargets(capacity, capacity.chains[chain], ports, seen);

      // Every port of the chain takes part, inner and outer alike. An inner
      // wire has to be able to leave through an outer port: that is what a
      // coupler bridge is for, and the assignment reads which outer port it
      // surfaces at out of this stage. A lattice of the inner ports alone
      // could never surface anywhere.
      if (std::ranges::none_of(ports, [&](const std::uint32_t port) {
            return targets.contains(port);
          })) {
        continue;
      }
      auto lattice = buildLattice(chip, ports, reach);
      // A single node has no edge, so it is no lattice. Everything else
      // stays, a chain whose ports are all targets included: that is a demand
      // without supply, and the artifact shows it as a lattice on which
      // nothing is selected.
      if (lattice.nodes.size() > 1) {
        lattices.push_back(std::move(lattice));
      }
    }
    report.progress("building the lattices", "", capacity.chains.size(),
                    capacity.chains.size());

    if (report.wants(Detail::Steps)) {
      std::size_t nodes = 0;
      std::size_t edges = 0;
      for (const auto& lattice : lattices) {
        nodes += lattice.nodes.size();
        for (const auto& node : lattice.nodes) {
          edges += node.neighbors.size();
        }
      }
      report.line(Detail::Steps,
                  std::format("{}{}{}{}{}", counted(lattices.size(), "lattice"),
                              FIGURE_SEPARATOR, counted(nodes, "node"),
                              FIGURE_SEPARATOR, counted(edges / 2, "edge")));
    }
    return lattices;
  }

  /// Collects the target ports of a chain, from @p node down.
  static void collectTargets(const CapacityPlanT& capacity,
                             const std::uint32_t node,
                             std::vector<std::uint32_t>& ports,
                             std::unordered_set<std::uint32_t>& seen) {
    const auto& entry = *capacity.nodes[node];
    if (entry.kind == fba::CapacityElement::Target &&
        seen.insert(entry.id).second) {
      ports.push_back(entry.id);
    }
    for (const auto next : entry.next) {
      collectTargets(capacity, next, ports, seen);
    }
  }

  /// The flow model over every lattice.
  ///
  /// `unreached` collects the marker of every target that the model may leave
  /// unserved, `crowded` the marker of every ring port that a chain may find
  /// no room for when @p shortfalls allows that, and `penalty` the cost of one
  /// marker. The caller takes the markers back out of the objective: the
  /// penalty makes an unserved target a last resort, and left in, it would
  /// make the cost of a run that misses a target incomparable with the cost
  /// of one that reaches them all.
  [[nodiscard]] static std::vector<Arc>
  build(milp::Model& model, const CapacityPlanT& capacity,
        const std::vector<Lattice>& lattices, const InnerCircuit& circuit,
        const bool internal, const bool shortfalls,
        std::vector<milp::Var>& unreached, std::vector<Shortfall>& crowded,
        double& penalty) {
    const std::unordered_set<std::uint32_t> targets(circuit.targets.begin(),
                                                    circuit.targets.end());
    std::vector<Arc> arcs;
    // The variable of each directed edge, so that the constraints find it
    // without a search.
    std::vector<std::map<std::pair<std::size_t, std::size_t>, milp::Var>>
        byPair(lattices.size());

    double totalWeight = 0.0;
    for (std::size_t index = 0; index < lattices.size(); ++index) {
      const auto& lattice = lattices[index];
      for (std::size_t from = 0; from < lattice.nodes.size(); ++from) {
        for (const auto to : lattice.nodes[from].neighbors) {
          const auto weight = edgeWeight(lattice, from, to);
          totalWeight += weight;
          const auto variable = model.addBinary(
              std::format("arc_{}_{}_{}", index, from, to), weight);
          byPair[index].emplace(std::pair{from, to}, variable);
          arcs.push_back(
              {.lattice = index, .from = from, .to = to, .variable = variable});
        }
      }
    }
    const auto unreachedPenalty = totalWeight + 1.0;
    penalty = unreachedPenalty;

    const auto flow = [&](const std::size_t index, const std::size_t node,
                          const bool out) {
      milp::LinearExpr expression;
      for (const auto other : lattices[index].nodes[node].neighbors) {
        const auto key = out ? std::pair{node, other} : std::pair{other, node};
        expression.add(byPair[index].at(key), 1.0);
      }
      return expression;
    };

    // What the circuit sends through a port, over every lattice that carries
    // the port. A port is a node of every chain whose chamber it borders, so
    // a constraint on a port is a constraint on this sum and not on one
    // lattice. Per lattice, a bridge whose two ends lie in different chambers
    // would go unconstrained, and a wire could start at one of them from
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
        if (port.has_value()) {
          // A port is constrained over every lattice at once, below.
          continue;
        }
        // A plain crossing passes on what it takes, and carries at most one
        // wire.
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
          model.addLessOrEqual(std::format("once_{}_{}_{}", index, from, to),
                               milp::LinearExpr(byPair[index].at({from, to})) +
                                   byPair[index].at({to, from}),
                               1.0);
        }
      }
    }

    // Any port other than a target is where one wire starts or ends, and no
    // wire runs on through it. A wire that did would join two connections
    // into one component with three port ends, which names no connection. A
    // port that borders several chambers is a node of several lattices, and
    // it is still one port: it takes one wire over all of them.
    std::set<std::uint32_t> ends;
    for (const auto& lattice : lattices) {
      for (const auto& entry : lattice.nodeOfPort) {
        if (!targets.contains(entry.first)) {
          ends.insert(entry.first);
        }
      }
    }
    for (const auto port : ends) {
      model.addLessOrEqual(std::format("port_{}", port),
                           portFlow(port, false) + portFlow(port, true), 1.0);
    }

    // A ring port already has its wire: the assignment runs it to a launcher
    // or a feedline, and at the ring end of a bridge, that wire is the one
    // that surfaces through the bridge. An inner wire from a ring port would
    // give the port a second wire, and the target at its other end would
    // reach neither a launcher nor a feedline. So no inner wire starts, ends
    // or runs at a ring port, and every inner wire leaves the inner circuit
    // through the inner end of a ring bridge, perhaps after internal bridges.
    // The nodes stay in the lattice, so the picture still shows the ports.
    const std::unordered_set<std::uint32_t> inner(circuit.innerPorts.begin(),
                                                  circuit.innerPorts.end());
    for (const auto port : ends) {
      if (inner.contains(port)) {
        continue;
      }
      model.addEqual(std::format("ring_shut_in_{}", port),
                     portFlow(port, false), 0.0);
      model.addEqual(std::format("ring_shut_out_{}", port),
                     portFlow(port, true), 0.0);
    }

    // A target is where a wire ends: everything that arrives stays, and a
    // port takes one wire. The demand is on the port and not on a lattice
    // node. A chamber border runs between two chains, so a target beside one
    // is a node of both, and a demand per lattice would ask for a second wire
    // to a port that has one. That second wire takes supply the chip has, and
    // the target it was taken from is then reported unreached.
    //
    // Whether a target is reached is a variable and not a constraint. A chip
    // that cannot serve one of its targets is a fact about the chip and its
    // partitioning, and the stage has to say which target that was; a hard
    // constraint would only say "infeasible" and lose the rest of the
    // circuit. The penalty is above the cost of all edges together, so the
    // model leaves a target unreached only when no wire can reach it, and
    // the objective of a circuit that reaches every target is the one the
    // hard constraint would give.
    for (const auto target : circuit.targets) {
      const auto carried =
          std::ranges::any_of(lattices, [&](const Lattice& lattice) {
            return lattice.nodeOfPort.contains(target);
          });
      if (!carried) {
        // No lattice reaches it, so no variable can serve it.
        continue;
      }
      const auto missed = model.addBinary(std::format("unreached_{}", target),
                                          unreachedPenalty);
      unreached.push_back(missed);
      model.addEqual(std::format("target_in_{}", target),
                     portFlow(target, false) + missed, 1.0);
      model.addEqual(std::format("target_out_{}", target),
                     portFlow(target, true), 0.0);
    }

    // A wire leaves the inner circuit for good where a bridge surfaces on the
    // ring. What may leave there depends on the free space behind the ring
    // port, which is the chain capacity below, and not on the flow at the
    // far end, which lies on no lattice. So only a bridge with both ends
    // inside is treated here.
    std::unordered_set<std::uint32_t> surfaces;
    for (const auto& bridge : circuit.outerBridges) {
      surfaces.insert(bridge.to);
    }
    for (const auto& bridge : circuit.bridges) {
      if (surfaces.contains(bridge.to) || surfaces.contains(bridge.from)) {
        continue;
      }
      if (!internal) {
        // An internal crossing is not allowed, so both of its ports carry no
        // flow at all. A wire that ended at one of them would end inside a
        // component the ring never reaches, and no later stage could say
        // where it goes from there. The nodes stay in the lattice, so the
        // picture still shows the ports.
        for (const auto port : {bridge.from, bridge.to}) {
          model.addEqual(std::format("bridge_shut_in_{}", port),
                         portFlow(port, false), 0.0);
          model.addEqual(std::format("bridge_shut_out_{}", port),
                         portFlow(port, true), 0.0);
        }
        continue;
      }
      // What leaves one of its ports entered the other. The two ends usually
      // lie in different chambers and so in different lattices, so the flow
      // is taken over all of them.
      model.addEqual(std::format("bridge_{}_{}", bridge.from, bridge.to),
                     portFlow(bridge.from, true) - portFlow(bridge.to, false),
                     0.0);
      model.addEqual(std::format("bridge_back_{}_{}", bridge.from, bridge.to),
                     portFlow(bridge.to, true) - portFlow(bridge.from, false),
                     0.0);
      model.addLessOrEqual(
          std::format("bridge_once_{}_{}", bridge.from, bridge.to),
          portFlow(bridge.from, true), 1.0);
      model.addLessOrEqual(
          std::format("bridge_once_back_{}_{}", bridge.from, bridge.to),
          portFlow(bridge.to, true), 1.0);
    }

    // The free space that the wire has to cross to get there. A lattice edge
    // says where a wire may run; it does not say that the corridor around it
    // has room. The capacity chains measure that room. Without them the
    // circuit surfaces where the wire is shortest, typically at the coupler
    // it already stands on, and not where the chip can carry it out to a
    // launcher.
    addChainCapacity(
        model, capacity, circuit, portFlow,
        shortfalls ? std::optional{unreachedPenalty} : std::nullopt, crowded);

    model.setSense(milp::Sense::Minimize);
    return arcs;
  }

  /// One link of a chain as the flow model sees it: its plan node, its parent
  /// and its children, all by position in the tree and not in the plan.
  ///
  /// Two chains may share a subtree, and a shared link carries its own flow
  /// in each, so the walk puts the tree into fresh positions instead of
  /// indexing it by plan node.
  struct ChainLink {
    std::uint32_t node = 0;
    std::optional<std::size_t> parent;
    std::vector<std::size_t> children;
  };

  /// The links of one chain, rooted at @p root.
  [[nodiscard]] static std::vector<ChainLink>
  chainLinks(const CapacityPlanT& capacity, const std::uint32_t root) {
    std::vector<ChainLink> links{
        {.node = root, .parent = std::nullopt, .children = {}}};
    for (std::size_t at = 0; at < links.size(); ++at) {
      for (const auto next : capacity.nodes[links[at].node]->next) {
        links[at].children.push_back(links.size());
        links.push_back({.node = next, .parent = at, .children = {}});
      }
    }
    return links;
  }

  /// Ties the inner circuit to the free space it has to cross.
  ///
  /// Each chain of outer space carries an integer flow of its own: its
  /// launcher supplies it, each of its gates passes at most its capacity, and
  /// each of its targets draws one wire. Where a target is the outer end of a
  /// bridge, the chains that border it and that a launcher supplies together
  /// draw exactly what the inner circuit sends out through that bridge. An
  /// inner wire may therefore surface at an outer port only where a chain
  /// behind that port can carry it to a launcher.
  ///
  /// A chain that belongs to the inner circuit is left out. Its targets are
  /// what the circuit has to reach, or the ports it reaches them through,
  /// and a constraint of those against themselves says nothing.
  ///
  /// A plain ring port draws its wire through the gates of the chain like
  /// any other target. With a @p penalty, a port that the gates have no room
  /// for goes short at that cost and is marked in @p crowded, instead of the
  /// model finding no solution. The stage then names the port and counts it
  /// as a fail.
  template <typename PortFlow>
  static void
  addChainCapacity(milp::Model& model, const CapacityPlanT& capacity,
                   const InnerCircuit& circuit, const PortFlow& portFlow,
                   const std::optional<double> penalty,
                   std::vector<Shortfall>& crowded) {
    // The inner end of each bridge by its ring port, and the ports that the
    // inner circuit owns.
    std::unordered_map<std::uint32_t, std::uint32_t> insideOf;
    for (const auto& bridge : circuit.outerBridges) {
      insideOf.emplace(bridge.to, bridge.from);
    }
    std::unordered_set<std::uint32_t> claimed(circuit.targets.begin(),
                                              circuit.targets.end());
    std::unordered_set<std::uint32_t> outerSide;
    for (const auto& bridge : circuit.outerBridges) {
      claimed.insert(bridge.from);
      outerSide.insert(bridge.to);
    }
    for (const auto& bridge : circuit.bridges) {
      if (!outerSide.contains(bridge.to)) {
        // An internal bridge: both of its ends belong to the circuit.
        claimed.insert(bridge.from);
        claimed.insert(bridge.to);
      }
    }

    const auto isTarget = [&](const ChainLink& link) {
      return capacity.nodes[link.node]->kind == fba::CapacityElement::Target;
    };
    const auto portOf = [&](const ChainLink& link) {
      return capacity.nodes[link.node]->id;
    };

    // What each chain can reach: a launcher supplies it, and a gate without
    // room stops it. A target that the supply cannot reach draws nothing.
    struct Walked {
      std::size_t chain = 0;
      std::vector<ChainLink> links;
      std::vector<bool> supplied;
    };
    std::vector<Walked> walked;
    for (std::size_t chain = 0; chain < capacity.chains.size(); ++chain) {
      auto links = chainLinks(capacity, capacity.chains[chain]);
      if (std::ranges::any_of(links, [&](const ChainLink& link) {
            return isTarget(link) && claimed.contains(portOf(link));
          })) {
        continue;
      }
      auto supplied = suppliedLinks(capacity, links);
      walked.push_back({.chain = chain,
                        .links = std::move(links),
                        .supplied = std::move(supplied)});
    }

    // A ring end of a bridge may border several chains. The wire that
    // surfaces there leaves through any of them that a launcher supplies, so
    // the rows of the bridge sum over those chains and come with the last of
    // them. Only where no chain supplies the ring end does nothing surface
    // there, because a wire that came out there would have no free space to
    // leave through.
    std::unordered_map<std::uint32_t, std::size_t> suppliers;
    std::unordered_map<std::uint32_t, std::size_t> lastSupplier;
    for (std::size_t index = 0; index < walked.size(); ++index) {
      const auto& [chain, links, supplied] = walked[index];
      for (std::size_t at = 0; at < links.size(); ++at) {
        if (isTarget(links[at]) && supplied[at] &&
            insideOf.contains(portOf(links[at]))) {
          ++suppliers[portOf(links[at])];
          lastSupplier[portOf(links[at])] = index;
        }
      }
    }
    std::unordered_map<std::uint32_t, milp::LinearExpr> drawn;
    std::unordered_set<std::uint32_t> sealed;

    for (std::size_t index = 0; index < walked.size(); ++index) {
      const auto& [chain, links, supplied] = walked[index];

      const auto demand =
          static_cast<double>(std::ranges::count_if(links, isTarget));
      std::map<std::pair<std::size_t, std::size_t>, milp::Var> edges;
      for (std::size_t at = 0; at < links.size(); ++at) {
        for (const auto child : links[at].children) {
          edges.emplace(
              std::pair{at, child},
              model.addInteger(std::format("chain_{}_{}_{}", chain, at, child),
                               0.0, demand));
          edges.emplace(
              std::pair{child, at},
              model.addInteger(std::format("chain_{}_{}_{}", chain, child, at),
                               0.0, demand));
        }
      }

      // Everything a link sends or receives, or the same over its parent
      // edge alone, which is the edge that a gate limits.
      const auto around = [&](const std::size_t at, const bool out,
                              const bool parentOnly) {
        milp::LinearExpr expression;
        if (const auto parent = links[at].parent) {
          expression.add(
              edges.at(out ? std::pair{at, *parent} : std::pair{*parent, at}),
              1.0);
        }
        if (!parentOnly) {
          for (const auto child : links[at].children) {
            expression.add(
                edges.at(out ? std::pair{at, child} : std::pair{child, at}),
                1.0);
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
          model.addLessOrEqual(std::format("chain_{}_launcher_{}", chain, at),
                               out, demand);
          break;
        case fba::CapacityElement::Bottleneck:
          // A gate passes on what it takes, and no more than fits.
          model.addEqual(std::format("chain_{}_gate_{}", chain, at), in - out,
                         0.0);
          model.addLessOrEqual(std::format("chain_{}_gate_out_{}", chain, at),
                               around(at, true, true), element.capacity);
          model.addLessOrEqual(std::format("chain_{}_gate_in_{}", chain, at),
                               around(at, false, true), element.capacity);
          break;
        case fba::CapacityElement::Target: {
          const auto inside = insideOf.find(element.id);
          if (!supplied[at]) {
            // Cut off from every launcher of its chain. A demand here would
            // only make the model infeasible.
            model.addEqual(std::format("chain_{}_cut_in_{}", chain, at), in,
                           0.0);
            model.addEqual(std::format("chain_{}_cut_out_{}", chain, at), out,
                           0.0);
            if (inside != insideOf.end() && !suppliers.contains(element.id) &&
                sealed.insert(element.id).second) {
              model.addEqual(std::format("chain_{}_sealed_{}", chain, at),
                             portFlow(inside->second, true), 0.0);
            }
            break;
          }
          if (inside == insideOf.end()) {
            // A plain outer target draws the one wire it is there for, unless
            // the gates on the way have no room for it.
            auto drawnHere = in - out;
            if (penalty.has_value()) {
              const auto missing = model.addBinary(
                  std::format("chain_{}_short_{}", chain, at), *penalty);
              crowded.push_back({.marker = missing, .port = element.id});
              drawnHere.add(missing, 1.0);
            }
            model.addEqual(std::format("chain_{}_target_{}", chain, at),
                           drawnHere, 1.0);
            break;
          }
          // A target where the inner circuit can surface draws exactly what
          // the circuit sends out through the bridge, and never the reverse:
          // the wire leaves the inner region there and does not enter it.
          auto& total = drawn[element.id];
          total.add(in - out);
          if (suppliers.at(element.id) > 1) {
            // No chain hands a wire on to another one through the port.
            model.addGreaterOrEqual(
                std::format("chain_{}_bridge_draw_{}", chain, at), in - out,
                0.0);
          }
          if (lastSupplier.at(element.id) != index) {
            break;
          }
          model.addEqual(std::format("chain_{}_bridge_{}", chain, at),
                         total - portFlow(inside->second, true), 0.0);
          model.addLessOrEqual(std::format("chain_{}_bridge_cap_{}", chain, at),
                               total, 1.0);
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

  /// The links of a chain that a launcher can reach: the walk starts at every
  /// launcher of the chain and stops at a gate without room.
  [[nodiscard]] static std::vector<bool>
  suppliedLinks(const CapacityPlanT& capacity,
                const std::vector<ChainLink>& links) {
    std::vector<bool> supplied(links.size(), false);
    std::vector<std::size_t> pending;
    for (std::size_t at = 0; at < links.size(); ++at) {
      if (capacity.nodes[links[at].node]->kind ==
          fba::CapacityElement::Launcher) {
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
            (element.kind == fba::CapacityElement::Bottleneck &&
             element.capacity == 0)) {
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

  /// Reads the solved flow back into the artifact.
  ///
  /// @return The ports the circuit left through, and the length of each
  /// connection.
  [[nodiscard]] static Circuit readBack(GlobalRoutingT& routing,
                                        const std::vector<Lattice>& lattices,
                                        const std::vector<Arc>& arcs,
                                        const milp::Solution& solution,
                                        const InnerCircuit& circuit) {
    const std::unordered_set<std::uint32_t> targets(circuit.targets.begin(),
                                                    circuit.targets.end());

    // The lattices, with the selected edges marked, so that a picture of the
    // inner circuit can be drawn from the artifact alone.
    std::vector<std::map<std::pair<std::size_t, std::size_t>, std::uint32_t>>
        edgeIndex(lattices.size());
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
          edgeIndex[index].emplace(
              std::pair{from, to},
              static_cast<std::uint32_t>(entry->edges.size() / 2));
          entry->edges.push_back(static_cast<std::uint32_t>(from));
          entry->edges.push_back(static_cast<std::uint32_t>(to));
        }
      }
      routing.lattices.push_back(std::move(entry));
    }

    // The edges the solution selected, per lattice and node.
    std::vector<std::vector<std::vector<std::size_t>>> selected(
        lattices.size());
    Circuit solved;
    for (std::size_t index = 0; index < lattices.size(); ++index) {
      selected[index].assign(lattices[index].nodes.size(), {});
    }
    for (const auto& arc : arcs) {
      if (!solution.isSet(arc.variable)) {
        continue;
      }
      selected[arc.lattice][arc.from].push_back(arc.to);
      selected[arc.lattice][arc.to].push_back(arc.from);

      const auto key =
          std::pair{std::min(arc.from, arc.to), std::max(arc.from, arc.to)};
      if (const auto found = edgeIndex[arc.lattice].find(key);
          found != edgeIndex[arc.lattice].end()) {
        routing.lattices[arc.lattice]->selected.push_back(found->second);
      }
      // A port with an outgoing arc is where a wire of the inner circuit
      // leaves, which is what the ring has to keep.
      if (const auto port = lattices[arc.lattice].nodes[arc.from].port;
          port.has_value() && !targets.contains(*port)) {
        solved.sources.insert(*port);
      }
    }
    for (auto& lattice : routing.lattices) {
      std::ranges::sort(lattice->selected);
      const auto duplicates = std::ranges::unique(lattice->selected);
      lattice->selected.erase(duplicates.begin(), duplicates.end());
    }

    // Each connected component of the selected edges is one wire. Its two
    // port ends are the connection that the later stages route.
    for (std::size_t index = 0; index < lattices.size(); ++index) {
      std::vector<bool> seen(lattices[index].nodes.size(), false);
      for (std::size_t start = 0; start < lattices[index].nodes.size();
           ++start) {
        if (seen[start] || selected[index][start].empty()) {
          continue;
        }
        std::vector<std::uint32_t> ends;
        double length = 0.0;
        std::vector<std::size_t> pending{start};
        seen[start] = true;
        while (!pending.empty()) {
          const auto node = pending.back();
          pending.pop_back();
          if (const auto port = lattices[index].nodes[node].port;
              port.has_value()) {
            ends.push_back(*port);
          }
          for (const auto next : selected[index][node]) {
            if (node < next) {
              length += edgeWeight(lattices[index], node, next);
            }
            if (!seen[next]) {
              seen[next] = true;
              pending.push_back(next);
            }
          }
        }
        if (ends.size() != 2) {
          continue;
        }
        // The target end is where the wire ends; the other end is its source.
        // A wire between two bridges has no target end. The flow leaves its
        // source, and a port that is no target carries one wire, so the end
        // with an outgoing arc is the source.
        const auto targetFirst =
            targets.contains(ends[0]) ||
            (!targets.contains(ends[1]) && !solved.sources.contains(ends[0]) &&
             solved.sources.contains(ends[1]));
        auto connection = std::make_unique<fbd::ConnectionT>();
        connection->source =
            std::make_unique<fbd::PortRef>(targetFirst ? ends[1] : ends[0]);
        connection->target = fbd::PortRef(targetFirst ? ends[0] : ends[1]);
        connection->source_role = fbd::AssignedRole::ConventionalSource;
        connection->target_role = fbd::AssignedRole::ConventionalTarget;
        routing.connections.push_back(std::move(connection));
        solved.lengths.push_back(length);
      }
    }
    return solved;
  }

  /// Fills the ring that the assignment consumes, and the resonators it has
  /// to feed.
  ///
  /// The configured ring keeps its order. A port on the far side of a bridge
  /// stays only where the inner circuit surfaces there: such a port carries
  /// an inner wire, and only then does the outer assignment have to route it.
  static void fillRing(GlobalRoutingT& routing, const ChipT& chip,
                       const ConfigT& config, const InnerCircuit& circuit,
                       const std::unordered_set<std::uint32_t>& sources) {
    // The inner end of each ring port that a bridge reaches. A wire surfaces
    // at the ring port, but the inner port carries it, so that is the port
    // the solved flow names as a source.
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

    // The inner wire of a resonator may cross internal bridges before it
    // surfaces, so it can be several connections long. The connections and
    // the internal bridges join the ports into these wires.
    std::unordered_map<std::uint32_t, std::vector<std::uint32_t>> joined;
    const auto join = [&joined](const std::uint32_t a, const std::uint32_t b) {
      joined[a].push_back(b);
      joined[b].push_back(a);
    };
    for (const auto& connection : routing.connections) {
      if (connection->source != nullptr) {
        join(connection->source->index(), connection->target.index());
      }
    }
    for (const auto& bridge : circuit.bridges) {
      if (!surfaceOf.contains(bridge.from) && !surfaceOf.contains(bridge.to)) {
        join(bridge.from, bridge.to);
      }
    }

    // A resonator inside the ring is fed where its inner wire surfaces. The
    // assignment works on the ring, so the ring port that the wire came out
    // of is a resonator to it as well. The port of the qubit stays a
    // resonator too: it is the resonator the chip has, and the two are the
    // same wire seen from either end.
    std::unordered_set<std::uint32_t> resonators;
    for (std::uint32_t index = 0; index < chip.ports.size(); ++index) {
      if (chip.ports[index]->role == fbd::UnassignedRole::Resonator) {
        resonators.insert(index);
      }
    }
    std::vector<std::uint32_t> surfaced;
    for (const auto resonator : resonators) {
      std::unordered_set<std::uint32_t> seen{resonator};
      std::vector<std::uint32_t> pending{resonator};
      while (!pending.empty()) {
        const auto port = pending.back();
        pending.pop_back();
        if (const auto surface = surfaceOf.find(port);
            surface != surfaceOf.end()) {
          surfaced.push_back(surface->second);
        }
        if (const auto next = joined.find(port); next != joined.end()) {
          for (const auto other : next->second) {
            if (seen.insert(other).second) {
              pending.push_back(other);
            }
          }
        }
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
      throw std::invalid_argument(std::format(
          "the ring names '{}', which the chip does not carry", label));
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

  for (const auto& component :
       design::componentsOf(chip, config.ports->bridge_pairs)) {
    for (const auto& pair : component.bridges) {
      // A bridge runs from the inside out. When the ring carries one of the
      // two ports, the assignment sees that port, so it is the far side.
      const auto firstInRing = ring.contains(pair.first);
      const auto secondInRing = ring.contains(pair.second);
      if (firstInRing && secondInRing) {
        // With both ends outside, the wire has no inside to come from. The
        // ring then names a port where the inner circuit can never surface,
        // which is a ring that does not match the bridge rules rather than a
        // circuit that cannot be solved.
        throw std::invalid_argument(std::format(
            "the ring carries both ends of the bridge '{}' - '{}'",
            chip.ports[pair.first]->label, chip.ports[pair.second]->label));
      }
      const PortBridge bridge{.from = secondInRing ? pair.first : pair.second,
                              .to = secondInRing ? pair.second : pair.first};
      circuit.bridges.push_back(bridge);
      if (firstInRing || secondInRing) {
        circuit.outerBridges.push_back(bridge);
      }
    }
    // A port that no bridge claims ends a wire, and the circuit has to reach
    // an inner one. The ports of a qubit are why an inner qubit needs the
    // circuit, and the port of a coupler that is left over faces the qubit
    // pair the coupler joins.
    for (const auto port : component.ends) {
      if (inner.contains(port)) {
        circuit.targets.push_back(port);
      }
    }
  }
  // A port of no component has no bridge to claim it, so it ends a wire too.
  for (const auto port : circuit.innerPorts) {
    if (chip.ports[port]->component.empty()) {
      circuit.targets.push_back(port);
    }
  }
  std::ranges::sort(circuit.targets);
  return circuit;
}

std::unique_ptr<IGlobalRouter> makeHananMilpRouter() {
  return std::make_unique<HananMilpRouter>();
}

} // namespace mqt::scpd::pipeline
