/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#pragma once

#include "mqt-scpd/milp/Backend.hpp"
#include "mqt-scpd/milp/Model.hpp"
#include "mqt-scpd/pipeline/mqt_scpd_pipeline_export.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace mqt::scpd::pipeline {

/// A point of the chip, in cells, for the length of a way through a
/// capacity graph.
struct FlowPoint {
  double x = 0.0;
  double y = 0.0;
};

/// One edge of a capacity graph: a bottleneck, or a stretch of a feedline a
/// wire may cross, joining the chambers on its sides.
struct FlowEdge {
  /// The chambers it joins: two, or more where a bottleneck ends among
  /// several.
  std::vector<std::uint32_t> chambers;
  /// How many wires it takes.
  std::uint32_t capacity = 0;
  /// The demands that may use it, by their index; every demand when empty.
  std::optional<std::vector<std::uint32_t>> users;
  /// The line a way passes it on: the bottleneck's line, or the stretch from
  /// its first cell to its last.
  FlowPoint from;
  FlowPoint to;
};

/// One wire to carry: the chambers it may start in and the ones it may end
/// in, and the edges it has to cross on the way.
struct FlowDemand {
  std::vector<std::uint32_t> from;
  std::vector<std::uint32_t> to;
  /// One group per feedline edge the wire is prescribed to cross: the edges
  /// of the stretches of that feedline edge. The way takes exactly one edge
  /// of each group. A group with no edge leaves the demand no way. Empty
  /// for a wire that crosses no feedline edge.
  std::vector<std::vector<std::uint32_t>> crossings;
  /// How long its way may be, in cells, for a wire whose length is fixed (a
  /// resonator), against the least a way can run: from `start` to the line
  /// of the first edge it passes, from line to line, and from the last line
  /// to `end`. Nothing for a wire of any length.
  std::optional<double> longest;
  FlowPoint start;
  FlowPoint end;
};

/// How many groups of edges to cross one demand may have.
inline constexpr std::size_t MAX_CROSSINGS = 16;

/// What `checkCapacity` or `checkInTurn` found.
struct FlowCheck {
  /// What the solver made of the model: `Optimal` when the overflow is
  /// proven least, `Feasible` when a limit stopped it first. `checkInTurn`
  /// says `Optimal`.
  milp::SolveStatus status = milp::SolveStatus::Error;
  /// The edges each demand passes, in order from where it starts. Empty
  /// when it starts in a chamber it may end in; nothing when the graph has
  /// no way for it at all, whatever the capacities.
  std::vector<std::optional<std::vector<std::uint32_t>>> ways;
  /// How many demands each edge carries.
  std::vector<std::uint32_t> load;
  /// How many more than it takes.
  std::vector<std::uint32_t> overflow;
  /// The sum of the overflow: by how many wires the graph is short. Zero
  /// when every demand with a way has one within the capacities.
  std::uint32_t shortBy = 0;
  /// Per demand with a `longest`, from `checkInTurn`: the length of its way,
  /// or of the shortest way it has when none is short enough; 0 for every
  /// other demand.
  std::vector<double> lengths;
  /// Per demand from `checkInTurn`: whether it has no way only because
  /// every way it has is longer than `longest`.
  std::vector<bool> tooLong;
};

/// How many wires a gap of `cells` holds: its length over the wire
/// clearance, rounded down (user, 2026-10-07). The gap is measured between
/// the walls as they stand; copper that keeps a clearance of its own, a
/// feedline or a port run, has to be inflated by half of it before.
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT std::uint32_t
wiresThroughGap(double cells, double clearance);

/// Whether a capacity graph carries every demand at once, as an integer
/// multi-commodity flow.
///
/// Every demand is one unit from its start chambers to its end chambers,
/// on a single way, over the edges open to it. An edge is a node of the
/// network of its own, joined both ways to each of its chambers, so that a
/// bottleneck between three chambers is one edge and not three; the units
/// that enter it, in either direction, are its load. The load may exceed
/// the capacity, by an integer overflow, and the objective is the least
/// total overflow first and the fewest edges after it. A demand with no
/// way at all, found by a walk that ignores the capacities, is left out of
/// the model and has no way in the answer.
///
/// The flow does not hold a demand to its groups of edges to cross or to
/// its `longest`; `checkInTurn` does.
///
/// The answer is a necessary condition for routing and not a sufficient
/// one: the flow lets two wires change their order inside a chamber, which
/// two wires that may not cross cannot do.
///
/// @param chambers How many chambers there are; they are numbered from zero.
/// @throws std::invalid_argument when an edge or a demand names a chamber
/// out of range, an edge a demand that does not exist, or a demand an edge
/// to cross that does not exist or more than `MAX_CROSSINGS` groups.
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT FlowCheck checkCapacity(
    std::uint32_t chambers, const std::vector<FlowEdge>& edges,
    const std::vector<FlowDemand>& demands, const milp::SolveOptions& options);

/// Whether a capacity graph carries every demand, routed one after another
/// (user, 2026-10-08).
///
/// The demands are routed in their order, each on its way with the fewest
/// edges over the edges open to it that still take a wire: an edge takes
/// as many wires as its capacity, less the ways routed before that pass
/// it. A demand with groups of edges to cross takes exactly one edge of
/// each group. A demand that finds no such way takes its way with the
/// fewest edges whatever the capacities, and the edges it overfills have
/// overflow. A demand with no way at all has no way in the answer.
///
/// A demand with a `longest` takes its shortest way instead, measured as
/// the least a wire on it can run: straight from line to line of the edges
/// it passes (user, 2026-10-09). A way longer than `longest` is no way for
/// it: when even its shortest way whatever the capacities is longer, it has
/// no way and `tooLong` says so. A demand refused for its length cannot be
/// routed through those edges at all.
///
/// No overflow says that the graph carries every demand. Overflow says only
/// that this order does not: another order of the demands may carry them.
///
/// @param chambers How many chambers there are; they are numbered from zero.
/// @throws std::invalid_argument when an edge or a demand names a chamber
/// out of range, an edge a demand that does not exist, or a demand an edge
/// to cross that does not exist or more than `MAX_CROSSINGS` groups.
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT FlowCheck
checkInTurn(std::uint32_t chambers, const std::vector<FlowEdge>& edges,
            const std::vector<FlowDemand>& demands);

} // namespace mqt::scpd::pipeline
