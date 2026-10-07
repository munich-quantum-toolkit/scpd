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

#include <cstdint>
#include <optional>
#include <vector>

namespace mqt::scpd::pipeline {

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
};

/// One wire to carry: the chambers it may start in and the ones it may end
/// in.
struct FlowDemand {
  std::vector<std::uint32_t> from;
  std::vector<std::uint32_t> to;
};

/// What `checkCapacity` found.
struct FlowCheck {
  /// What the solver made of the model: `Optimal` when the overflow is
  /// proven least, `Feasible` when a limit stopped it first.
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
/// The answer is a necessary condition for routing and not a sufficient
/// one: the flow lets two wires change their order inside a chamber, which
/// two wires that may not cross cannot do.
///
/// @param chambers How many chambers there are; they are numbered from zero.
/// @throws std::invalid_argument when an edge or a demand names a chamber
/// out of range, or an edge a demand that does not exist.
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT FlowCheck checkCapacity(
    std::uint32_t chambers, const std::vector<FlowEdge>& edges,
    const std::vector<FlowDemand>& demands, const milp::SolveOptions& options);

} // namespace mqt::scpd::pipeline
