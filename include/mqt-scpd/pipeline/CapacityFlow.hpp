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
  /// resonator), against the length of a way: from `start` to the centre of
  /// the line of the first edge it passes, from centre to centre, and from
  /// the last centre to `end`. Nothing for a wire of any length.
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
  /// How many sweeps `checkInTurn` made over the demands: one in their
  /// order, and one more each time a demand overfilled an edge and a sweep
  /// could still move one.
  std::uint32_t sweeps = 1;
};

/// How many wires a gap of `cells` holds: its length over the wire
/// clearance, rounded down (user, 2026-10-07). The gap is measured between
/// the walls as they stand; copper that keeps a clearance of its own, a
/// feedline or a port run, has to be inflated by half of it before.
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT std::uint32_t
wiresThroughGap(double cells, double clearance);

/// How `checkInTurn` routes, where a rule has an earlier form: the rules of
/// the rounds before the rip-up (round 22) and before the detour measured
/// over the gates that take a wire (round 25).
struct InTurnRules {
  /// How many sweeps over the demands at most: one routes every demand once
  /// and rips nothing up.
  std::uint32_t mostSweeps = 8;
  /// Whether a demand's detour is measured against its shortest way over
  /// the gates that take a wire at all; else over any gate, as before.
  bool detourOverTakers = true;
};

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
/// A demand with a `longest` takes its shortest way instead, measured
/// straight from centre to centre of the lines of the edges it passes (user,
/// 2026-10-13; from line to line before, 2026-10-09). A way longer than
/// `longest` is no way for it: when even its shortest way whatever the
/// capacities is longer, it has no way and `tooLong` says so. A demand
/// refused for its length cannot be routed through those edges at all.
///
/// With a `detour` above zero, a demand without a `longest` takes its
/// shortest way within the capacities instead, measured as above, and only
/// when it is no longer than `detour` times its shortest way over the
/// edges that take a wire at all, whatever their load (user, 2026-10-14);
/// else it takes that shortest way and overfills its edges. A gate that
/// takes no wire is no way to measure by; only a demand that cannot pass
/// without one is measured against, and sent over, its shortest way at
/// all. A wire whose short way is full does not go round the chip in the
/// check, as no routing takes it round the chip either.
///
/// After the first sweep the demands that overfill an edge are ripped up
/// and routed again, as the feedline routing does with its wires (user,
/// 2026-10-14): a sweep backward over the demands, then forward, and so on,
/// each taking every such demand off its way and routing it again within
/// the capacities. When it has no way within them, every demand on an edge
/// its way would overfill is taken off in turn and routed again within the
/// capacities off that edge, and then the demand once more. The sweeps end
/// when no demand overfills an edge, when a sweep moves nothing, or after
/// `rules.mostSweeps`, eight by default. `sweeps` counts them.
///
/// No overflow says that the graph carries every demand. Overflow says only
/// that these sweeps do not: another order of the demands may carry them.
///
/// @param chambers How many chambers there are; they are numbered from zero.
/// @param detour How many times its shortest way a demand's way within the
/// capacities may run; 0 takes the way with the fewest edges, as before.
/// @param rules The sweeps and the measure of the detour; the defaults are
/// the rules as they stand.
/// @throws std::invalid_argument when an edge or a demand names a chamber
/// out of range, an edge a demand that does not exist, or a demand an edge
/// to cross that does not exist or more than `MAX_CROSSINGS` groups.
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT FlowCheck
checkInTurn(std::uint32_t chambers, const std::vector<FlowEdge>& edges,
            const std::vector<FlowDemand>& demands, double detour = 0.0,
            InTurnRules rules = {});

} // namespace mqt::scpd::pipeline
