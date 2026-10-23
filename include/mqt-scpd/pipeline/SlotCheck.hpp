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

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace mqt::scpd::pipeline {

/// The capacity graph the slot check judges, in memory: what the JSON of
/// `final-capacity-graph.json` carries of it, without the JSON. The coupler
/// insertion fills it from its own data; `slotGraphOf` reads it from JSON
/// for debugging and for Python.
struct SlotGraph {
  std::int64_t width = 0;
  std::int64_t height = 0;
  /// The clearance between two wires, in cells.
  double clearance = 0.0;
  /// The crossing reach of the graph: the router's reach plus one.
  double crossingReach = 0.0;
  /// How far a feedline or a port run is inflated into a wall, in cells.
  double inflation = 0.0;
  /// The settings of the check, `[stages.final]` of the configuration. The
  /// safety factor on the clearance a wire takes in a gap: a gap holds
  /// `floor(cells / (clearance * margin))` wires, from gaps of
  /// `capacityMarginFromWires` wires on; below that a gap is measured
  /// exactly on the grid.
  double capacityMargin = 1.06;
  std::int64_t capacityMarginFromWires = 3;
  /// The narrowest gap a resonator passes, as a share of the clearance.
  double resonatorPassage = 0.8;
  /// The cells closed at the ends of a feedline edge, where no slot lies.
  std::size_t slotPinCells = 10;
  /// The straight run after the escape turn of a slot's end and the
  /// clearance an escape keeps from a stub, in cells.
  std::int64_t escapeStraight = 5;
  double escapeStubMargin = 4.0;
  /// The names of the wall kinds; wall code k + 1 is kind k.
  std::vector<std::string> wallKinds;
  /// Per cell, row by row: 0 a cut line, 1 to 5 a wall (`wallKinds`),
  /// 6 + c chamber c. Empty when `raster` gives the codes.
  std::vector<std::uint16_t> codes;

  /// The codes of `codes` read from the buffers the coupler insertion holds,
  /// a cell at a time when the check asks for it, so that no raster of the
  /// whole chip is written per check. The buffers must outlive the check.
  struct RasterView {
    /// One bit per cell, row by row, 64 cells a word: whether it is a wall.
    const std::uint64_t* wallMask = nullptr;
    /// Per cell: the wall it belongs to, read where `wallMask` is set.
    const std::uint32_t* wallOf = nullptr;
    /// Per wall: its code, 1 to 5.
    const std::uint16_t* wallCodes = nullptr;
    std::size_t walls = 0;
    /// The code of a wall cell whose wall is not among `walls`.
    std::uint16_t otherWall = 0;
    /// Per cell: its chamber, or `noChamber`.
    const std::uint32_t* chamberOf = nullptr;
    std::uint32_t noChamber = 0;

    /// The code of a cell, as `codes` holds it.
    [[nodiscard]] std::uint16_t code(const std::size_t cell) const {
      if (((wallMask[cell >> 6U] >> (cell & 63U)) & 1U) != 0U) {
        const auto wall = wallOf[cell];
        return wall < walls ? wallCodes[wall] : otherWall;
      }
      const auto chamber = chamberOf[cell];
      return chamber != noChamber ? static_cast<std::uint16_t>(6U + chamber)
                                  : std::uint16_t{0};
    }
    bool operator==(const RasterView&) const = default;
  };
  /// Read in place of `codes` when `codes` is empty.
  std::optional<RasterView> raster;
  /// The cells whose code is the stub wall, row by row.
  std::vector<std::size_t> stubCells;
  /// The cells of the port runs' slots.
  std::vector<std::array<std::int64_t, 2>> portSlots;

  /// A port run or a feedline edge, by its middle line.
  struct Stroke {
    std::string name;
    std::string kind;
    std::vector<std::array<double, 2>> points;
    bool operator==(const Stroke&) const = default;
  };
  std::vector<Stroke> strokes;

  /// A bottleneck, or a stretch of a feedline edge the check by stretches
  /// crosses.
  struct Edge {
    bool stretch = false;
    std::string name;
    /// The feedline edge of a stretch.
    std::string feedline;
    std::vector<std::int64_t> chambers;
    std::int64_t capacity = 0;
    /// The wires that may use it; every wire when nothing. The slots of a
    /// stretch's feedline edge are open to the users of its stretches.
    std::optional<std::vector<std::string>> users;
    /// The first and the last point of the line.
    std::array<double, 2> from{};
    std::array<double, 2> to{};
    /// What the bottleneck's two ends lie on.
    std::vector<std::string> ends;
    bool mouth = false;
    double length = 0.0;
    /// The chambers on the two sides of the line, when the graph knows them.
    std::optional<std::array<std::vector<std::int64_t>, 2>> sides;
    bool operator==(const Edge&) const = default;
  };
  std::vector<Edge> edges;

  /// One end of a wire.
  struct End {
    std::array<double, 2> cell{};
    std::vector<std::int64_t> chambers;
    std::string label;
    bool operator==(const End&) const = default;
  };
  /// A wire to route.
  struct Wire {
    std::string name;
    End source;
    End target;
    std::vector<std::string> crosses;
    bool terminal = false;
    bool resonator = false;
    bool operator==(const Wire&) const = default;
  };
  std::vector<Wire> wires;

  /// How many chambers the graph has.
  std::size_t chambers = 0;

  /// Whether two graphs hold the same data, field by field.
  bool operator==(const SlotGraph&) const = default;
};

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
  /// The wires whose way passes an edge or a self-bottleneck that is over,
  /// in the order they were routed.
  std::vector<std::string> crowded;
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

/// The verdict of `slotCheck` on a graph in memory, without the graph as
/// JSON: `graph` of the result stays empty.
///
/// @throws std::invalid_argument when the graph has no feedline wall kind,
/// names a terminal edge with no number, or is larger than the search can
/// index.
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT SlotCheckResult
slotCheck(const SlotGraph& graph);

/// A capacity graph read from the JSON of `final-capacity-graph.json`.
///
/// @throws std::invalid_argument when the JSON is not valid or lacks a key
/// the check reads.
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT SlotGraph
slotGraphOf(std::string_view graph);

/// What `slotCheck` finds, as one JSON object: `verdict`, `lost`, `over`
/// and `graph`, the graph as an object.
///
/// @param graph The capacity graph as JSON.
/// @throws std::invalid_argument as `slotCheck` does.
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT std::string
slotCheckJson(std::string_view graph);

} // namespace mqt::scpd::pipeline
