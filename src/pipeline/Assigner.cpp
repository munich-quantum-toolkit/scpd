/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/pipeline/Assigner.hpp"

#include "mqt-scpd/flatbuffers/artifacts.hpp"
#include "mqt-scpd/flatbuffers/design.hpp"
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
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace mqt::scpd::pipeline {
namespace {

namespace fba = flatbuffers::artifacts;
namespace fbd = flatbuffers::design;

/// How much the distance of a ring node from its nearest launcher counts
/// against a crossing. The weight is small on purpose: the model minimizes
/// crossings first, and prefers the nearer launcher only between two
/// assignments with as many crossings.
constexpr double PROXIMITY_WEIGHT = 0.01;

/// The ring edges: one per pair of consecutive resonators, weighted by how
/// many conventional ports lie between them.
///
/// The ring is a cycle of every outer port, but only a resonator needs a
/// launcher, so only a resonator is an end of the assignment. The ports
/// between two resonators are the length of the chord that would join them,
/// and the objective pays for that length.
struct RingEdges {
  /// The ring nodes that are resonators, in ring order.
  std::vector<std::size_t> anchors;
  /// Each edge, as two positions in `anchors`.
  std::vector<std::pair<std::size_t, std::size_t>> edges;
  /// The weight of each edge.
  std::vector<double> weights;
  /// The edge that leaves each anchor, as an index into `edges`.
  std::vector<std::size_t> leaving;
  /// The position of each ring node in `anchors`, for the nodes that are in
  /// it. The launcher variable of a resonator is found through this and not
  /// counted along the walk, which is off by one whenever the ring opens on
  /// a conventional port.
  std::vector<std::size_t> positionOf;
};

RingEdges ringEdgesOf(const AssignmentInputs& inputs) {
  RingEdges ring;
  ring.positionOf.assign(inputs.ring.size(), 0);
  for (std::size_t index = 0; index < inputs.ring.size(); ++index) {
    if (inputs.isResonator[index]) {
      ring.positionOf[index] = ring.anchors.size();
      ring.anchors.push_back(index);
    }
  }
  if (ring.anchors.size() < 2) {
    return ring;
  }

  ring.leaving.assign(ring.anchors.size(), 0);
  for (std::size_t position = 0; position < ring.anchors.size(); ++position) {
    const auto from = ring.anchors[position];
    const auto to = ring.anchors[(position + 1) % ring.anchors.size()];
    // The ports between the two, walking forward around the cycle.
    const auto between =
        to > from ? to - from - 1 : inputs.ring.size() - from + to - 1;
    ring.leaving[position] = ring.edges.size();
    ring.edges.emplace_back(position, (position + 1) % ring.anchors.size());
    ring.weights.push_back(static_cast<double>(between));
  }
  return ring;
}

/// The label of a port, or "termination" for a chain end without one.
std::string endName(const ChipT& chip,
                    const std::unique_ptr<fbd::PortRef>& port) {
  return port != nullptr ? chip.ports[port->index()]->label : "termination";
}

/// The assigner of the first release.
class OrderedMilpAssigner final : public IAssigner {
public:
  [[nodiscard]] AssignmentT run(const ChipT& chip,
                                const CapacityPlanT& capacity,
                                const GlobalRoutingT& global,
                                const ConfigT& config,
                                const Report& report) const override {
    report.progress("working out the inputs", "", 0, 0);
    const auto inputs = assignmentInputs(chip, capacity, global);
    const auto ring = ringEdgesOf(inputs);
    const auto resonators =
        static_cast<std::size_t>(std::ranges::count(inputs.isResonator, true));
    if (report.wants(Detail::Steps)) {
      report.line(
          Detail::Steps,
          std::format(
              "{}{}{}{}{}{}{}", counted(inputs.ring.size(), "ring port"),
              FIGURE_SEPARATOR, counted(resonators, "resonator"),
              FIGURE_SEPARATOR,
              counted(inputs.ring.size() - resonators, "conventional port"),
              FIGURE_SEPARATOR,
              counted(inputs.launchers.size(), "launcher slot")));
    }

    AssignmentT assignment;
    assignment.objective = 0.0;
    for (const auto port : inputs.ring) {
      assignment.ring.emplace_back(port);
    }
    if (ring.anchors.size() < 2) {
      // A ring with fewer than two resonators has no chain to build. The
      // artifact is written with the ring alone, and every port of the ring
      // is reported as a fail, since none of them is given a launcher.
      report.result({{.text = counted(inputs.ring.size(), "port")},
                     {.text = "nothing to assign"}},
                    inputs.ring.size());
      return assignment;
    }

    const auto target = launcherTarget(config);
    if (report.wants(Detail::Steps)) {
      const auto& rules = *config.rules;
      report.line(Detail::Steps,
                  std::format("launcher target {}{}max feedline utilization "
                              "{}{}feedline terminations {}",
                              target, FIGURE_SEPARATOR,
                              rules.max_feedline_utilization, FIGURE_SEPARATOR,
                              rules.feedline_terminations));
    }
    report.progress("building the model", "", 0, 0);
    milp::Model model("assignment");
    const auto variables = build(model, inputs, ring, config, target);
    const auto solution =
        solveWith(model, config, report, "solving the assignment");
    if (!solution.hasValues()) {
      throw std::runtime_error(
          std::format("the assignment did not solve: {}{}",
                      milp::statusName(solution.status),
                      solution.message.empty() ? "" : ", " + solution.message));
    }
    assignment.objective = solution.objective;
    readBack(assignment, inputs, ring, variables, solution);

    if (report.wants(Detail::Items)) {
      for (std::size_t index = 0; index < assignment.chains.size(); ++index) {
        const auto& chain = *assignment.chains[index];
        report.entry(
            Detail::Items, std::format("chain {}", index + 1),
            {{.text = std::format("{} {} {}", endName(chip, chain.start), ARROW,
                                  endName(chip, chain.end))},
             {.text = counted(chain.nodes.size(), "resonator")}});
      }
    }
    report.line(Detail::Steps,
                counted(assignment.chains.size(), "feedline chain"));
    report.result(
        {{.text = std::format("{} on {}", counted(inputs.ring.size(), "port"),
                              counted(target, "launcher"))},
         {.text = std::format("objective {:.2f}", assignment.objective)},
         {.text = std::string(milp::statusName(solution.status))}},
        0U);
    return assignment;
  }

private:
  struct Variables {
    /// The launcher index each ring node maps to.
    std::vector<milp::Var> flow;
    /// Whether each anchor is given a launcher.
    std::vector<milp::Var> launcher;
    /// Whether each anchor ends a feedline at a termination instead.
    std::vector<milp::Var> termination;
    /// Whether each ring edge is used.
    std::vector<milp::Var> edge;
    /// How many wires the corridor at each anchor carries.
    std::vector<milp::Var> utilization;
    /// The rotation of the whole flow assignment.
    milp::Var offset;
  };

  [[nodiscard]] static Variables build(milp::Model& model,
                                       const AssignmentInputs& inputs,
                                       const RingEdges& ring,
                                       const ConfigT& config,
                                       const std::uint32_t target) {
    const auto launchers = static_cast<double>(inputs.launchers.size());
    const auto& rules = *config.rules;
    const auto utilizationCap =
        static_cast<double>(rules.max_feedline_utilization);
    const auto terminations = static_cast<double>(rules.feedline_terminations);

    Variables variables;
    // The potential that the ring is walked with: a place on the ring of
    // launchers. The value modulo the launcher count names the launcher, and
    // that is how the stage reads it back.
    //
    // Its range is one turn of that ring, one short of the launcher count.
    // The walk never rises and falls by at least one at every conventional
    // port, so one turn keeps two conventional ports off one launcher: they
    // would have to lie a whole launcher ring apart, and no walk of one turn
    // does that. It also leaves the launcher slots along the ring in their
    // cyclic order, with the single rise where the turn closes.
    //
    // A ring that carries more conventional ports than the chip has
    // launchers is therefore infeasible. That is the chip saying it has no
    // assignment, since one launcher feeds one conventional port.
    //
    // The range is one turn exactly, with no room above it. Room would let a
    // node skip to the launcher nearest to it rather than the next one along.
    // That is worth little to the proximity term and makes the solve of a
    // large chip much slower, and the crossing count, which is what this
    // stage is for, stays the same.
    const auto span = launchers - 1.0;
    variables.flow.reserve(inputs.ring.size());
    for (std::size_t index = 0; index < inputs.ring.size(); ++index) {
      variables.flow.push_back(
          model.addInteger(std::format("flow_{}", index), 0.0, span));
    }
    for (std::size_t index = 0; index < ring.anchors.size(); ++index) {
      variables.launcher.push_back(
          model.addBinary(std::format("launcher_{}", index)));
      variables.termination.push_back(
          model.addBinary(std::format("termination_{}", index)));
      variables.utilization.push_back(model.addInteger(
          std::format("utilization_{}", index), 0.0, utilizationCap));
    }
    for (std::size_t index = 0; index < ring.edges.size(); ++index) {
      variables.edge.push_back(
          model.addBinary(std::format("edge_{}", index), ring.weights[index]));
    }
    variables.offset = model.addInteger("offset", 0.0, launchers);

    // Every anchor has degree two on the ring: two chords, or one chord and
    // an end. An end is a launcher or a permitted termination.
    for (std::size_t index = 0; index < ring.anchors.size(); ++index) {
      milp::LinearExpr degree;
      for (std::size_t edge = 0; edge < ring.edges.size(); ++edge) {
        if (ring.edges[edge].first == index ||
            ring.edges[edge].second == index) {
          degree.add(variables.edge[edge], 1.0);
        }
      }
      degree.add(variables.launcher[index], 1.0);
      degree.add(variables.termination[index], 1.0);
      model.addEqual(std::format("degree_{}", index), degree, 2.0);
    }

    // The flow runs around the ring and never increases: a conventional port
    // always takes one step, and a resonator only where it takes a launcher.
    // That makes two assignments that cross also cross on the chip.
    //
    // The anchor that the step of a resonator charges is looked up, not
    // counted along the walk. A count starts at the first resonator the walk
    // meets, which is the second anchor whenever the ring opens on a
    // conventional port.
    model.addEqual("flow_start", milp::LinearExpr(variables.flow.front()),
                   span);
    for (std::size_t index = 1; index < inputs.ring.size(); ++index) {
      const auto step =
          milp::LinearExpr(variables.flow[index]) - variables.flow[index - 1];
      if (!inputs.isResonator[index]) {
        model.addLessOrEqual(std::format("flow_step_{}", index), step, -1.0);
        continue;
      }
      model.addLessOrEqual(std::format("flow_step_{}", index),
                           step + variables.launcher[ring.positionOf[index]],
                           0.0);
    }

    // The utilization of a corridor counts up from every launcher and resets
    // at the next one, which bounds how many wires one corridor carries. A
    // big-M pair per anchor switches between the two.
    const auto bigM = static_cast<double>(ring.anchors.size()) + 2.0;
    for (std::size_t index = 0; index < ring.anchors.size(); ++index) {
      const auto previous =
          variables.utilization[(index + 1) % ring.anchors.size()];
      const auto leaving = variables.edge[ring.leaving[index]];
      const auto starts = model.addBinary(std::format("starts_{}", index));

      // `starts` is one exactly where the anchor ends a chain and the chord
      // that leaves it is unused, which is where a new corridor begins.
      model.addLessOrEqual(std::format("starts_a_{}", index),
                           milp::LinearExpr(starts) -
                               variables.launcher[index] -
                               variables.termination[index],
                           0.0);
      model.addLessOrEqual(std::format("starts_b_{}", index),
                           milp::LinearExpr(starts) + leaving, 1.0);
      model.addGreaterOrEqual(std::format("starts_c_{}", index),
                              milp::LinearExpr(starts) -
                                  variables.launcher[index] -
                                  variables.termination[index] + leaving,
                              0.0);

      const auto here = milp::LinearExpr(variables.utilization[index]);
      model.addLessOrEqual(std::format("util_reset_up_{}", index),
                           here + (bigM * milp::LinearExpr(starts)),
                           1.0 + bigM);
      model.addGreaterOrEqual(std::format("util_reset_low_{}", index),
                              here - (bigM * milp::LinearExpr(starts)),
                              1.0 - bigM);
      model.addLessOrEqual(std::format("util_step_up_{}", index),
                           here - previous - (bigM * milp::LinearExpr(starts)),
                           1.0);
      model.addGreaterOrEqual(
          std::format("util_step_low_{}", index),
          here - previous + (bigM * milp::LinearExpr(starts)), 1.0);
    }

    milp::LinearExpr activeLaunchers;
    milp::LinearExpr activeTerminations;
    for (std::size_t index = 0; index < ring.anchors.size(); ++index) {
      activeLaunchers.add(variables.launcher[index], 1.0);
      activeTerminations.add(variables.termination[index], 1.0);
    }
    model.addEqual("launcher_target", activeLaunchers,
                   static_cast<double>(target));
    model.addEqual("termination_target", activeTerminations, terminations);

    // Between two assignments with as many crossings, the better one sends
    // each node to the launcher nearest to it. The ring of launchers is
    // cyclic, so the distance is too.
    for (std::size_t index = 0; index < inputs.ring.size(); ++index) {
      const auto wanted = static_cast<double>(inputs.nearestLauncher[index]);
      // How many turns of the launcher ring lie between the potential and the
      // launcher nearest to the node. The potential spans the whole ring, so
      // this reaches as far as the ring is long.
      const auto turns = std::ceil((span + launchers) / launchers);
      const auto wrap =
          model.addInteger(std::format("wrap_{}", index), -turns, 1.0);
      const auto gap = model.addContinuous(std::format("gap_{}", index), 0.0,
                                           launchers / 2.0, PROXIMITY_WEIGHT);
      const auto placed =
          milp::LinearExpr(variables.flow[index]) + variables.offset;
      model.addGreaterOrEqual(std::format("gap_up_{}", index),
                              milp::LinearExpr(gap) - placed -
                                  (launchers * milp::LinearExpr(wrap)),
                              -wanted);
      model.addGreaterOrEqual(std::format("gap_low_{}", index),
                              milp::LinearExpr(gap) + placed +
                                  (launchers * milp::LinearExpr(wrap)),
                              wanted);
    }

    model.setSense(milp::Sense::Minimize);
    return variables;
  }

  /// The number of launcher ends the model has to use. It is a figure of the
  /// chip with no default.
  [[nodiscard]] static std::uint32_t launcherTarget(const ConfigT& config) {
    if (config.stages != nullptr && config.stages->assignment != nullptr &&
        config.stages->assignment->launcher_target > 0) {
      return config.stages->assignment->launcher_target;
    }
    throw std::invalid_argument(
        "[stages.assignment] launcher_target is missing; it is a per-chip "
        "figure with no default");
  }

  /// Which ring node is the last anchor of a chain that ends at a launcher.
  ///
  /// This is the walk of `readBackChains`, run before the feeds are placed:
  /// where a resonator is fed depends on whether its chain ends on it. The
  /// feed of such a node has to lie before its launcher along the walk, or
  /// the chain runs past the launcher and has to come back to it.
  [[nodiscard]] static std::vector<bool>
  endsAChain(const AssignmentInputs& inputs, const RingEdges& ring,
             const Variables& variables, const milp::Solution& solution) {
    std::vector<bool> ends(inputs.ring.size(), false);
    const auto anchors = ring.anchors.size();
    if (anchors == 0) {
      return ends;
    }
    const auto used = [&](const std::size_t edge) {
      return std::llround(solution.valueOf(variables.edge[edge])) == 1;
    };
    const auto launcherAt = [&](const std::size_t position) {
      return std::llround(solution.valueOf(variables.launcher[position])) == 1;
    };
    const auto entering = [&](const std::size_t position) {
      return ring.leaving[(position + anchors - 1) % anchors];
    };
    std::vector<bool> taken(anchors, false);
    for (std::size_t first = 0; first < anchors; ++first) {
      if (taken[first] || used(entering(first))) {
        continue;
      }
      auto position = first;
      std::size_t count = 0;
      while (true) {
        taken[position] = true;
        ++count;
        if (!used(ring.leaving[position])) {
          break;
        }
        const auto next = (position + 1) % anchors;
        if (taken[next]) {
          break;
        }
        position = next;
      }
      // A chain of one anchor both starts and ends on it. The start wins: its
      // feed has to leave the launcher the chain starts at, and there is no
      // second launcher to arrive at.
      if (count > 1 && launcherAt(position)) {
        ends[ring.anchors[position]] = true;
      }
    }
    return ends;
  }

  static void readBack(AssignmentT& assignment, const AssignmentInputs& inputs,
                       const RingEdges& ring, const Variables& variables,
                       const milp::Solution& solution) {
    const auto launchers = inputs.launchers.size();
    const auto offset = static_cast<std::int64_t>(
        std::llround(solution.valueOf(variables.offset)));

    // The launcher the walk gave each ring node. The potential is a place on
    // the launcher ring, so the launcher is that place modulo the launcher
    // count.
    std::vector<std::size_t> slotOf(inputs.ring.size(), 0);
    assignment.launchers.reserve(inputs.ring.size());
    for (std::size_t index = 0; index < inputs.ring.size(); ++index) {
      const auto placed = static_cast<std::int64_t>(
          std::llround(solution.valueOf(variables.flow[index])));
      const auto count = static_cast<std::int64_t>(launchers);
      slotOf[index] = static_cast<std::size_t>(
          (((placed + offset) % count) + count) % count);
      assignment.launchers.emplace_back(inputs.launchers[slotOf[index]]);
    }

    // Where each node is fed from. A conventional port is fed at its launcher
    // slot, and no two of them share one. A resonator is fed at no launcher:
    // the wire that reaches it comes past the slot, so its feed moves onto
    // one of the two segments that its launcher lies between.
    //
    // The segment depends on where the chain of the resonator goes. The ring
    // is walked one way, and the potential falls along the walk, so the
    // launcher after a slot is the slot below it by index.
    //
    // - A resonator that its chain runs on past is fed on the segment that
    //   leaves its launcher. The chain reaches the launcher first and the
    //   feed after it, so the walk keeps one direction.
    // - A resonator that its chain ends on is fed on the segment that
    //   arrives at its launcher. The chain reaches the feed first and the
    //   launcher after it. Fed on the leaving segment, the chain would run
    //   past its own launcher and have to turn back to it.
    //
    // Where a launcher was given n resonators on one segment, they land at
    // 1/(n+1) ... n/(n+1) of it. The one the walk reaches first lands first
    // on the segment as well, so the wires of the group do not cross.
    const auto ends = endsAChain(inputs, ring, variables, solution);
    assignment.feeds.reserve(inputs.ring.size());
    std::vector<std::vector<std::size_t>> given(launchers);
    for (std::size_t index = 0; index < inputs.ring.size(); ++index) {
      assignment.feeds.push_back(inputs.launcherPosition[slotOf[index]]);
      given[slotOf[index]].push_back(index);
    }
    for (std::size_t slot = 0; slot < launchers; ++slot) {
      const auto& here = inputs.launcherPosition[slot];
      // The segment that leaves the slot along the walk, and the one that
      // arrives at it.
      const auto& after =
          inputs.launcherPosition[(slot + launchers - 1) % launchers];
      const auto& before = inputs.launcherPosition[(slot + 1) % launchers];
      for (const bool ending : {false, true}) {
        std::vector<std::size_t> group;
        for (const auto index : given[slot]) {
          if (inputs.isResonator[index] && ends[index] == ending) {
            group.push_back(index);
          }
        }
        if (group.empty()) {
          continue;
        }
        const auto& there = ending ? before : after;
        if (geometry::distance(here, there) == 0.0) {
          // The two slots are one point, and there is nothing to interpolate.
          continue;
        }
        // On the leaving segment the walk runs from the launcher outward, so
        // the first of the group in ring order sits nearest to it. On the
        // arriving segment the walk runs toward the launcher, so the first in
        // ring order sits farthest from it.
        const auto count = group.size();
        for (std::size_t taken = 0; taken < count; ++taken) {
          const auto step = ending ? (count - taken) : (taken + 1);
          const auto ratio =
              static_cast<double>(step) / static_cast<double>(count + 1);
          assignment.feeds[group[taken]] =
              geometry::Point(here.x() + (ratio * (there.x() - here.x())),
                              here.y() + (ratio * (there.y() - here.y())));
        }
      }
    }

    // One connection per ring node: the assignment reaches every port of the
    // ring. The source of a resonator is the coupler that a later stage
    // inserts, so it stays absent here; the assignment decides that the
    // resonator is fed, and by which launcher. A conventional port is fed
    // from that launcher itself, which is the end of a feedline chain.
    assignment.connections.reserve(inputs.ring.size());
    for (std::size_t index = 0; index < inputs.ring.size(); ++index) {
      auto connection = std::make_unique<fbd::ConnectionT>();
      connection->target = fbd::PortRef(inputs.ring[index]);
      if (inputs.isResonator[index]) {
        connection->source_role = fbd::AssignedRole::ResonatorSource;
        connection->target_role = fbd::AssignedRole::ResonatorTarget;
      } else {
        connection->source =
            std::make_unique<fbd::PortRef>(inputs.launchers[slotOf[index]]);
        connection->source_role = fbd::AssignedRole::FeedlineSource;
        connection->target_role = fbd::AssignedRole::FeedlineTarget;
      }
      assignment.connections.push_back(std::move(connection));
    }

    readBackChains(assignment, inputs, ring, variables, solution, slotOf);
  }

  /// The feedline chains the model chose, read off the chord, launcher and
  /// termination variables.
  ///
  /// Every anchor has degree two: a chord on either side, or a chord and an
  /// end. A chain is walked from an anchor whose entering chord is unused,
  /// along the used chords, to the anchor whose leaving chord is unused, so
  /// it runs in ring order. Its launchers are the slots that its two end
  /// anchors were given. The feed of the first anchor lies on the segment
  /// that leaves its slot, so the chain starts there. The feed of the last
  /// anchor lies on the segment that arrives at its slot, so the chain ends
  /// there without turning back; `endsAChain` places that feed and runs the
  /// same walk. An end at a termination has no launcher.
  static void readBackChains(AssignmentT& assignment,
                             const AssignmentInputs& inputs,
                             const RingEdges& ring, const Variables& variables,
                             const milp::Solution& solution,
                             const std::vector<std::size_t>& slotOf) {
    const auto anchors = ring.anchors.size();
    const auto used = [&](const std::size_t edge) {
      return std::llround(solution.valueOf(variables.edge[edge])) == 1;
    };
    const auto launcherAt = [&](const std::size_t position) {
      return std::llround(solution.valueOf(variables.launcher[position])) == 1;
    };
    // The chord that enters an anchor is the one that leaves the anchor
    // before it.
    const auto entering = [&](const std::size_t position) {
      return ring.leaving[(position + anchors - 1) % anchors];
    };
    std::vector<bool> taken(anchors, false);
    for (std::size_t first = 0; first < anchors; ++first) {
      if (taken[first] || used(entering(first))) {
        continue;
      }
      auto chain = std::make_unique<fba::FeedlineChainT>();
      auto position = first;
      while (true) {
        taken[position] = true;
        chain->nodes.push_back(
            static_cast<std::uint32_t>(ring.anchors[position]));
        if (!used(ring.leaving[position])) {
          break;
        }
        position = (position + 1) % anchors;
        if (taken[position]) {
          break;
        }
      }
      const auto last = position;
      if (launcherAt(first)) {
        chain->start = std::make_unique<fbd::PortRef>(
            inputs.launchers[slotOf[ring.anchors[first]]]);
      }
      if (launcherAt(last)) {
        chain->end = std::make_unique<fbd::PortRef>(
            inputs.launchers[slotOf[ring.anchors[last]]]);
      }
      assignment.chains.push_back(std::move(chain));
    }
    // A ring where every chord is used has no end at all, which the degree
    // constraints forbid. It is written as one chain from the first anchor
    // rather than dropped, so that no later stage is left without it.
    if (assignment.chains.empty() && anchors > 0) {
      auto chain = std::make_unique<fba::FeedlineChainT>();
      for (const auto anchor : ring.anchors) {
        chain->nodes.push_back(static_cast<std::uint32_t>(anchor));
      }
      assignment.chains.push_back(std::move(chain));
    }
  }
};

} // namespace

AssignmentInputs assignmentInputs(const ChipT& chip,
                                  const CapacityPlanT& capacity,
                                  const GlobalRoutingT& global) {
  if (capacity.launchers.empty()) {
    throw std::invalid_argument("the capacity plan carries no launcher");
  }
  if (global.outer_ring.empty()) {
    throw std::invalid_argument("the global routing carries no outer ring");
  }

  AssignmentInputs inputs;
  inputs.ring.reserve(global.outer_ring.size());
  for (const auto& port : global.outer_ring) {
    inputs.ring.push_back(port.index());
  }

  std::unordered_set<std::uint32_t> resonators;
  for (const auto& port : global.resonators) {
    resonators.insert(port.index());
  }
  inputs.isResonator.reserve(inputs.ring.size());
  for (const auto port : inputs.ring) {
    inputs.isResonator.push_back(resonators.contains(port));
  }

  inputs.launchers.reserve(capacity.launchers.size());
  inputs.launcherPosition.reserve(capacity.launchers.size());
  for (const auto& slot : capacity.launchers) {
    inputs.launchers.push_back(slot->port.index());
    inputs.launcherPosition.push_back(slot->position);
  }
  const auto& positions = inputs.launcherPosition;

  // The launcher nearest to each node, by straight distance. The first of
  // two equally near launchers wins.
  inputs.nearestLauncher.reserve(inputs.ring.size());
  for (const auto port : inputs.ring) {
    const auto& center = chip.ports[port]->center;
    std::size_t best = 0;
    auto shortest = geometry::distance(center, positions.front());
    for (std::size_t index = 1; index < positions.size(); ++index) {
      if (const auto gap = geometry::distance(center, positions[index]);
          gap < shortest) {
        shortest = gap;
        best = index;
      }
    }
    inputs.nearestLauncher.push_back(static_cast<std::uint32_t>(best));
  }

  return inputs;
}

std::unique_ptr<IAssigner> makeOrderedMilpAssigner() {
  return std::make_unique<OrderedMilpAssigner>();
}

} // namespace mqt::scpd::pipeline
