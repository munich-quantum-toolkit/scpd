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

#include <string>
#include <string_view>
#include <vector>

namespace mqt::scpd::pipeline {

/// What `slotCheck` found.
struct SlotCheckResult {
  /// `SAT` when every wire has a way and no edge and no self-bottleneck
  /// takes more wires than it holds; `UNSAT` otherwise.
  std::string verdict;
  /// The wires with no way: a port in no chamber, or no way in the graph.
  std::vector<std::string> lost;
  /// The edges that take more wires than they hold, by name, and after them
  /// every self-bottleneck that does, as `<name> <load>/<capacity>`.
  std::vector<std::string> over;
  /// The capacity graph as the check saw it, as JSON in the form of
  /// `final-capacity-graph.json`: the slot edges in place of the stretches,
  /// a self-bottleneck edge (`S …`) per line that limits a group of slots,
  /// every edge with its load, every wire with its way, the slots of every
  /// feedline edge (`crossSlots`), `verdict` and `summary`.
  std::string graph;
};

/// The capacity check of a capacity graph by its crossing slots.
///
/// The input is the JSON of `final-capacity-graph.json`. The check keeps
/// the bottlenecks of the graph and replaces every stretch of a feedline
/// edge by the crossing slots that fit across the edge. A slot is a
/// rectangle across a straight piece of the edge, the crossing reach to
/// either side of its middle line and one clearance wide along it. It
/// covers only free cells and the edge's own wall, and no two slots
/// overlap. A slot is an edge of capacity one between the chambers on the
/// two sides of the feedline edge, open to the wires prescribed to cross
/// that feedline edge.
///
/// A bottleneck joins a chamber on one side of its line to one on the other
/// side, as samples a cell and a half off the line find them. A sample
/// whose step off the line crosses the line of another bottleneck counts
/// for no side.
///
/// Two kinds of self-bottleneck limit groups of slots:
///
/// - a free line between two places of one feedline edge, close in space
///   and far apart along the edge, past which more slots lie than the line
///   holds wires;
/// - a free line from a corner of a slot to the open end of a stub that
///   closes a room. A wire that crosses at a slot whose end lies in the
///   room passes the line, and so does a wire with one end in the room.
///
/// A wire passes a self-bottleneck when the number of its slots and its
/// ends behind the line is odd.
///
/// The wires are routed by negotiated congestion (PathFinder) for at most
/// 60 rounds. A way crosses each prescribed feedline edge exactly once over
/// one of its slots and crosses no other. A wire with a crossing
/// prescription passes no mouth between two chains. A plain wire whose
/// launcher lies between the launchers of a mouth's two terminal edges, and
/// those terminal edges, pass that mouth and no other. A bottleneck with an
/// end on a wire's own port run holds half a clearance more for that wire
/// when the run's middle line, carried on past its open end, meets the
/// bottleneck's line. A way whose length, pulled tight through its edges,
/// is more than 1.5 times that of the wire's shortest way over the edges
/// that take a wire is no way: the wire takes its shortest way.
///
/// @param graph The capacity graph as JSON.
/// @throws std::invalid_argument when the JSON is not valid, lacks a key the
/// check reads, names a terminal edge with no number, or describes a graph
/// larger than the search can index.
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT SlotCheckResult
slotCheck(std::string_view graph);

/// What `slotCheck` finds, as one JSON object: `verdict`, `lost`, `over`
/// and `graph`, the graph as an object.
///
/// @param graph The capacity graph as JSON.
/// @throws std::invalid_argument as `slotCheck` does.
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT std::string
slotCheckJson(std::string_view graph);

} // namespace mqt::scpd::pipeline
