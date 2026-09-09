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
#include "mqt-scpd/pipeline/Solver.hpp"
#include "mqt-scpd/pipeline/Stages.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <memory>
#include <stdexcept>
#include <unordered_set>
#include <utility>
#include <vector>

namespace mqt::scpd::pipeline {
namespace {

namespace fbd = flatbuffers::design;

/// How much a ring node's distance from its own nearest launcher counts
/// against a crossing. It is small on purpose: the model minimizes crossings
/// first and only prefers the nearer launcher between two assignments that
/// cross equally often.
constexpr double PROXIMITY_WEIGHT = 0.01;

/// The ring edges: one per pair of consecutive resonators, weighted by how
/// many conventional ports lie between them.
///
/// The ring is a cycle of every outer port, but only a resonator needs a
/// launcher, so only a resonator is an endpoint of the assignment. What lies
/// between two of them is the length of the chord that would join them, and
/// that length is what the objective pays for.
struct RingEdges {
  std::vector<std::size_t> anchors;
  std::vector<std::pair<std::size_t, std::size_t>> edges;
  std::vector<double> weights;
  /// The edge that leaves each anchor, as an index into `edges`.
  std::vector<std::size_t> leaving;
  /// Where each ring node stands in `anchors`, for the nodes that are in it.
  /// A resonator's own launcher variable is found through this rather than
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
    auto between = to > from ? to - from - 1 : inputs.ring.size() - from + to - 1;
    ring.leaving[position] = ring.edges.size();
    ring.edges.emplace_back(position, (position + 1) % ring.anchors.size());
    ring.weights.push_back(static_cast<double>(between));
  }
  return ring;
}

/// The assigner of the first release.
class OrderedMilpAssigner final : public IAssigner {
public:
  [[nodiscard]] AssignmentT run(const ChipT& chip, const CapacityPlanT& capacity,
                                const GlobalRoutingT& global,
                                const ConfigT& config) const override {
    const auto inputs = assignmentInputs(chip, capacity, global);
    const auto ring = ringEdgesOf(inputs);

    AssignmentT assignment;
    assignment.objective = 0.0;
    for (const auto port : inputs.ring) {
      assignment.ring.emplace_back(port);
    }
    if (ring.anchors.size() < 2) {
      // A ring with fewer than two resonators has nothing to assign. The
      // artifact is written empty rather than the stage skipped.
      return assignment;
    }

    milp::Model model("assignment");
    const auto variables = build(model, inputs, ring, config);
    const auto solution = solveWith(model, config);
    if (!solution.hasValues()) {
      throw std::runtime_error(std::format("the assignment did not solve: {}{}",
                                           milp::statusName(solution.status),
                                           solution.message.empty() ? "" : ", " + solution.message));
    }
    assignment.objective = solution.objective;
    readBack(assignment, inputs, variables, solution);
    return assignment;
  }

private:
  struct Variables {
    /// The launcher index each ring node maps to.
    std::vector<milp::Var> flow;
    /// Whether each anchor is given a launcher.
    std::vector<milp::Var> launcher;
    /// Whether each anchor terminates a feedline instead.
    std::vector<milp::Var> termination;
    /// Whether each ring edge is used.
    std::vector<milp::Var> edge;
    /// How many wires the corridor at each anchor already carries.
    std::vector<milp::Var> utilization;
    /// The rotation of the whole flow assignment.
    milp::Var offset;
  };

  [[nodiscard]] static Variables build(milp::Model& model, const AssignmentInputs& inputs,
                                       const RingEdges& ring, const ConfigT& config) {
    const auto launchers = static_cast<double>(inputs.launchers.size());
    const auto& rules = *config.rules;
    const auto utilizationCap = static_cast<double>(rules.max_feedline_utilization);
    const auto terminations = static_cast<double>(rules.feedline_terminations);
    const auto target = launcherTarget(config);

    Variables variables;
    // The potential the ring is walked with: a place on the ring of launchers.
    // Which launcher a value names is the value modulo the launcher count,
    // which is how the stage reads it back.
    //
    // Its range is one turn of that ring, one short of the launcher count. The
    // walk never rises and falls at least once at every conventional port, so
    // one turn is what keeps two conventional ports off one launcher: they
    // would have to lie a whole launcher ring apart, and no walk of one turn
    // does that. It is also what leaves the launcher slots along the ring in
    // their cyclic order, with the single rise where the turn closes.
    //
    // A ring that carries more conventional ports than the chip has launchers
    // therefore comes out infeasible. That is the chip saying it has no
    // assignment, not the formulation capping the ring: one launcher feeds one
    // conventional port.
    //
    // One turn exactly, with no room above it. Room would let a node skip to
    // the launcher it is nearest to rather than the next one along, which is
    // worth a little to the proximity term; a whole launcher ring of it
    // improved that term by 0.05 on the 4-qubit chip and made the 45-qubit chip
    // take minutes instead of seconds. The crossing count, which is what this
    // stage is for, is the same either way.
    const auto span = launchers - 1.0;
    variables.flow.reserve(inputs.ring.size());
    for (std::size_t index = 0; index < inputs.ring.size(); ++index) {
      variables.flow.push_back(model.addInteger(std::format("flow_{}", index), 0.0, span));
    }
    for (std::size_t index = 0; index < ring.anchors.size(); ++index) {
      variables.launcher.push_back(model.addBinary(std::format("launcher_{}", index)));
      variables.termination.push_back(model.addBinary(std::format("termination_{}", index)));
      variables.utilization.push_back(
          model.addInteger(std::format("utilization_{}", index), 0.0, utilizationCap));
    }
    for (std::size_t index = 0; index < ring.edges.size(); ++index) {
      variables.edge.push_back(
          model.addBinary(std::format("edge_{}", index), ring.weights[index]));
    }
    variables.offset = model.addInteger("offset", 0.0, launchers);

    // Every anchor has degree two on the ring: two chords, or one chord and
    // an end. An end is a launcher, or a permitted extra termination.
    for (std::size_t index = 0; index < ring.anchors.size(); ++index) {
      milp::LinearExpr degree;
      for (std::size_t edge = 0; edge < ring.edges.size(); ++edge) {
        if (ring.edges[edge].first == index || ring.edges[edge].second == index) {
          degree.add(variables.edge[edge], 1.0);
        }
      }
      degree.add(variables.launcher[index], 1.0);
      degree.add(variables.termination[index], 1.0);
      model.addEqual(std::format("degree_{}", index), degree, 2.0);
    }

    // The flow runs around the ring and never increases: a conventional port
    // always consumes one step, a resonator only where it takes a launcher.
    // That is what makes two assignments that cross also cross on the chip.
    //
    // Which anchor a resonator's step charges is looked up, not counted along
    // the walk. Counting starts at the first resonator the walk meets, which is
    // the second anchor whenever the ring opens on a conventional port; the
    // constraint then held the wrong resonator's launcher on four of the eight
    // benchmark chips.
    model.addEqual("flow_start", milp::LinearExpr(variables.flow.front()), span);
    for (std::size_t index = 1; index < inputs.ring.size(); ++index) {
      const auto step = milp::LinearExpr(variables.flow[index]) - variables.flow[index - 1];
      if (!inputs.isResonator[index]) {
        model.addLessOrEqual(std::format("flow_step_{}", index), step, -1.0);
        continue;
      }
      model.addLessOrEqual(std::format("flow_step_{}", index),
                           step + variables.launcher[ring.positionOf[index]], 0.0);
    }

    // The utilization of a corridor counts up from every launcher and resets
    // at the next one, which is what bounds how many wires one corridor
    // carries. A big-M pair per anchor switches between the two.
    const auto bigM = static_cast<double>(ring.anchors.size()) + 2.0;
    for (std::size_t index = 0; index < ring.anchors.size(); ++index) {
      const auto previous = variables.utilization[(index + 1) % ring.anchors.size()];
      const auto leaving = variables.edge[ring.leaving[index]];
      const auto starts = model.addBinary(std::format("starts_{}", index));

      // `starts` is one exactly where the anchor ends a chain and the chord
      // leaving it is unused, which is where a new corridor begins.
      model.addLessOrEqual(std::format("starts_a_{}", index),
                           milp::LinearExpr(starts) - variables.launcher[index] -
                               variables.termination[index],
                           0.0);
      model.addLessOrEqual(std::format("starts_b_{}", index),
                           milp::LinearExpr(starts) + leaving, 1.0);
      model.addGreaterOrEqual(std::format("starts_c_{}", index),
                              milp::LinearExpr(starts) - variables.launcher[index] -
                                  variables.termination[index] + leaving,
                              0.0);

      const auto here = milp::LinearExpr(variables.utilization[index]);
      model.addLessOrEqual(std::format("util_reset_up_{}", index),
                           here + (bigM * milp::LinearExpr(starts)), 1.0 + bigM);
      model.addGreaterOrEqual(std::format("util_reset_low_{}", index),
                              here - (bigM * milp::LinearExpr(starts)), 1.0 - bigM);
      model.addLessOrEqual(std::format("util_step_up_{}", index),
                           here - previous - (bigM * milp::LinearExpr(starts)), 1.0);
      model.addGreaterOrEqual(std::format("util_step_low_{}", index),
                              here - previous + (bigM * milp::LinearExpr(starts)), 1.0);
    }

    milp::LinearExpr activeLaunchers;
    milp::LinearExpr activeTerminations;
    for (std::size_t index = 0; index < ring.anchors.size(); ++index) {
      activeLaunchers.add(variables.launcher[index], 1.0);
      activeTerminations.add(variables.termination[index], 1.0);
    }
    model.addEqual("launcher_target", activeLaunchers, static_cast<double>(target));
    model.addEqual("termination_target", activeTerminations, terminations);

    // Between two assignments that cross equally often, the one that sends
    // each node to the launcher it is already nearest to is the better one.
    // The ring of launchers is cyclic, so the distance is too.
    for (std::size_t index = 0; index < inputs.ring.size(); ++index) {
      const auto wanted = static_cast<double>(inputs.nearestLauncher[index]);
      // How many turns of the launcher ring separate the potential from the
      // launcher it is nearest to. The potential spans the whole ring, so this
      // reaches as far as the ring is long.
      const auto turns = std::ceil((span + launchers) / launchers);
      const auto wrap = model.addInteger(std::format("wrap_{}", index), -turns, 1.0);
      const auto gap = model.addContinuous(std::format("gap_{}", index), 0.0, launchers / 2.0,
                                           PROXIMITY_WEIGHT);
      const auto placed = milp::LinearExpr(variables.flow[index]) + variables.offset;
      model.addGreaterOrEqual(std::format("gap_up_{}", index),
                              milp::LinearExpr(gap) - placed - (launchers * milp::LinearExpr(wrap)),
                              -wanted);
      model.addGreaterOrEqual(std::format("gap_low_{}", index),
                              milp::LinearExpr(gap) + placed + (launchers * milp::LinearExpr(wrap)),
                              wanted);
    }

    model.setSense(milp::Sense::Minimize);
    return variables;
  }

  [[nodiscard]] static std::uint32_t launcherTarget(const ConfigT& config) {
    if (config.stages != nullptr && config.stages->assignment != nullptr &&
        config.stages->assignment->launcher_target > 0) {
      return config.stages->assignment->launcher_target;
    }
    throw std::invalid_argument(
        "[stages.assignment] launcher_target is missing; it is a per-chip figure with no default");
  }

  static void readBack(AssignmentT& assignment, const AssignmentInputs& inputs,
                       const Variables& variables, const milp::Solution& solution) {
    const auto launchers = inputs.launchers.size();
    const auto offset = std::llround(solution.valueOf(variables.offset));

    // Which launcher the walk gave each ring node. The potential is a place on
    // the launcher ring, so the launcher is that place modulo the launcher
    // count.
    std::vector<std::size_t> slotOf(inputs.ring.size(), 0);
    assignment.launchers.reserve(inputs.ring.size());
    for (std::size_t index = 0; index < inputs.ring.size(); ++index) {
      const auto placed = std::llround(solution.valueOf(variables.flow[index]));
      const auto count = static_cast<long long>(launchers);
      slotOf[index] = static_cast<std::size_t>((((placed + offset) % count) + count) % count);
      assignment.launchers.emplace_back(inputs.launchers[slotOf[index]]);
    }

    // Where each node is fed from. A conventional port is fed at its launcher
    // slot, and no two of them share one. A resonator is fed at no launcher at
    // all: the wire that reaches it comes past the slot, so it moves onto the
    // segment that runs from its own launcher to the next launcher along.
    //
    // The next launcher along is the slot *below* by index, because the
    // potential falls as the ring is walked. Where a launcher was given n
    // resonators they land at 1/(n+1) ... n/(n+1) of that segment, in ring
    // order, so the one the walk reaches first is the one nearest its own
    // launcher and the wires of the group do not cross each other.
    assignment.feeds.reserve(inputs.ring.size());
    std::vector<std::vector<std::size_t>> given(launchers);
    for (std::size_t index = 0; index < inputs.ring.size(); ++index) {
      assignment.feeds.push_back(inputs.launcherPosition[slotOf[index]]);
      given[slotOf[index]].push_back(index);
    }
    for (std::size_t slot = 0; slot < launchers; ++slot) {
      const auto resonators = static_cast<std::size_t>(std::ranges::count_if(
          given[slot], [&](const std::size_t index) { return inputs.isResonator[index]; }));
      if (resonators == 0) {
        continue;
      }
      const auto& here = inputs.launcherPosition[slot];
      const auto& there = inputs.launcherPosition[(slot + launchers - 1) % launchers];
      if (geometry::distance(here, there) == 0.0) {
        // The two slots are one point and there is nothing to interpolate.
        continue;
      }
      std::size_t taken = 0;
      for (const auto index : given[slot]) {
        if (!inputs.isResonator[index]) {
          continue;
        }
        const auto ratio = static_cast<double>(++taken) / static_cast<double>(resonators + 1);
        assignment.feeds[index] = geometry::Point(here.x() + (ratio * (there.x() - here.x())),
                                                  here.y() + (ratio * (there.y() - here.y())));
      }
    }

    // One connection per ring node: the assignment reaches every port the ring
    // carries. The source of a resonator is the coupler the Final stage
    // inserts, so it stays absent here; what the assignment decides is that the
    // resonator is fed, and by which launcher. A conventional port is fed from
    // that launcher itself, which is the end of a feedline chain.
    assignment.connections.reserve(inputs.ring.size());
    for (std::size_t index = 0; index < inputs.ring.size(); ++index) {
      auto connection = std::make_unique<fbd::ConnectionT>();
      connection->target = fbd::PortRef(inputs.ring[index]);
      if (inputs.isResonator[index]) {
        connection->source_role = fbd::AssignedRole::ResonatorSource;
        connection->target_role = fbd::AssignedRole::ResonatorTarget;
      } else {
        connection->source = std::make_unique<fbd::PortRef>(inputs.launchers[slotOf[index]]);
        connection->source_role = fbd::AssignedRole::FeedlineSource;
        connection->target_role = fbd::AssignedRole::FeedlineTarget;
      }
      assignment.connections.push_back(std::move(connection));
    }
  }
};

} // namespace

AssignmentInputs assignmentInputs(const ChipT& chip, const CapacityPlanT& capacity,
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

  const std::unordered_set<std::uint32_t> resonators = [&] {
    std::unordered_set<std::uint32_t> set;
    for (const auto& port : global.resonators) {
      set.insert(port.index());
    }
    return set;
  }();
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

  // The launcher each node would reach most cheaply. In the prototype this
  // ran a graph search over the capacity grid *during model construction*,
  // which is what made the model inseparable from the geometry engine. It is
  // a plain value here, worked out before the model exists.
  inputs.nearestLauncher.reserve(inputs.ring.size());
  for (const auto port : inputs.ring) {
    const auto& center = chip.ports[port]->center;
    std::size_t best = 0;
    auto shortest = geometry::distance(center, positions.front());
    for (std::size_t index = 1; index < positions.size(); ++index) {
      if (const auto gap = geometry::distance(center, positions[index]); gap < shortest) {
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
