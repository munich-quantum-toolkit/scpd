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
#include <map>
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
  /// Whether a gate with an end on a wire's own port run holds that wire
  /// half a clearance more, where the wire leaves the run straight through
  /// the gate: the wire keeps no clearance from its own run.
  bool ownRunBonus = false;
  /// The area budget of the chambers (`capacity_chamber_fill`): the share
  /// of a chamber's free cells the wires through it may cover, each piece
  /// of a way inside it its length times the clearance; 0 for none.
  double chamberFill = 0.0;
  /// The free cells of every chamber, by chamber; empty to count them from
  /// the raster.
  std::vector<std::uint32_t> chamberArea;
  /// The cells closed at the ends of a feedline edge, where no slot lies.
  std::size_t slotPinCells = 10;
  /// The straight run after the escape turn of a slot's end and the
  /// clearance an escape keeps from a stub, in cells.
  std::int64_t escapeStraight = 5;
  double escapeStubMargin = 4.0;
  /// The escape rounds (`capacity_escape_rounds`): after a SAT check, the
  /// ways out of the crossing slots the wires took narrow the gates they
  /// reach into, and the check runs again on the narrowed gates, at most
  /// this many times; 0 for none.
  std::int64_t escapeRounds = 0;
  /// What a way out takes of a gate it reaches into, in cells, flat
  /// (`capacity_escape_cells`); 0 measures it, L - L1 - L2.
  double escapeCells = 0.0;
  /// Whether the check merges the fans of gates (`capacity_merge_fans`):
  /// gates on one obstacle within two cells of each other at one end and on
  /// one obstacle at the other, with nothing between them, are one gate,
  /// the shortest.
  bool mergeFans = false;
  /// What a wire's turn out of its launcher takes of a gate that ends on
  /// that launcher's stub and that the wire passes, in cells, flat, under
  /// the escape rounds (`launcher_escape_cells`); 0 for none.
  double launcherEscapeCells = 0.0;
  /// Under the escape rounds (`capacity_escape_touch`, user, 2026-10-31):
  /// every gate that a way out of a slot a wire took touches, inflated like
  /// a wall, holds one wire less per such wire, once however many of its
  /// ways out touch it; a wire that passes the gate itself counts there as
  /// its load only.
  bool escapeTouch = false;
  /// Chords (`capacity_chords`, user, 2026-10-31): a line between two
  /// corner points of a chamber on different obstacles, through the chamber
  /// alone, parts no chambers but holds what a gate of its length holds, one
  /// wire less per wire whose way out of a slot touches it and does not
  /// cross it; the wires that come into the chamber on one side of it and
  /// leave or end on the other must fit, or the check is UNSAT.
  bool chords = false;
  /// Under `chords` (`capacity_chords_routing`): the SAT routing prices the
  /// chords in its way search and negotiates their overflow as it does a
  /// gate's; off, they are counted and only judged once it is done.
  bool chordsRouting = false;
  /// Whether no way of the SAT routing crosses a terminal edge on the line it
  /// is drawn on (`capacity_terminal_no_cross`, user, 2026-11-03), though it
  /// may share its gates: every wire but the terminal edges and the wires
  /// that may cross them (`Wire::mayCross`). A resonator or a free wire
  /// so stays on its side of the feedline's start and end and does not
  /// enter its partition round the end of the chain.
  bool terminalNoCross = false;
  /// Whether a crossing slot needs room past the crossing on both sides
  /// (`capacity_slot_room`, user, 2026-11-01): a free cell, reached
  /// straight from the slot's end, more than a clearance from every
  /// feedline edge and every coupler run. A wire that crosses keeps more
  /// than a clearance from the crossed edge once past it (the routed
  /// benchmarks never run one nearer), so a slot whose far side has no such
  /// cell — the closed end of a U the feedline makes round its coupler,
  /// 45q chain 2 — takes no wire the router can lay.
  bool slotRoom = false;
  /// Under `escapeTouch` (`capacity_escape_touch_routing`, user,
  /// 2026-11-01): the SAT routing counts, gate by gate, the wires whose way
  /// out of the slot they take touches the gate and that do not pass it, as
  /// it routes; a gate whose load and touches pass what it holds is over,
  /// and the history of an overflow so caused grows on the touching slots
  /// too, so that the touching wire tries another slot. The escape rounds
  /// then leave the touches alone. Off, the escape rounds narrow a touched
  /// gate after a SAT routing and keep it narrowed, which shuts out the
  /// wire that must pass and never moves the one that touches it (17q chain
  /// 1: 29 at f7 14 above 31's launcher shut 31's gate g78).
  bool escapeTouchRouting = false;
  /// The order rule of a local check (`ringOrder`): a wire with a place in
  /// the ring (`ring`, by name, of `ringSize` places) passes a gate whose two
  /// ends lie on wires of the ring (their stub, lead or pad) only when it lies
  /// between them in the ring, on the shorter way round. A wire that slips
  /// between the ends of wires it does not lie between would have to cross
  /// them; the gates alone, which count wires and not their order, let it
  /// round the back of a feedline into the partition its neighbours end in
  /// (user, 2026-10-27).
  bool ringOrder = false;
  std::map<std::string, std::int64_t> ring;
  std::int64_t ringSize = 0;
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

  /// A port run or a feedline edge, by its middle line; under `slotRoom`
  /// also the line a terminal edge is drawn on (kind `terminal`), which is
  /// no wall.
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
    /// Whether `capacity` is final: the check takes it as it is, with no
    /// margin and no passage rule (an escape round narrowed the gate).
    bool fixed = false;
    /// Under escape rounds: the wires the ways out of the slots take of the
    /// gate, and the capacity the check gave it before; for the page.
    double escapeShare = 0.0;
    std::int64_t capacityBefore = -1;
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
    /// Whether the wire stands and is not routed: a terminal edge of
    /// another chain in the local check of a chain. It names the mouth it
    /// stands at, as every terminal edge does, but takes no room and is
    /// neither lost nor shown. JSON key `standing`.
    bool standing = false;
    /// Of a terminal edge under `terminalNoCross`: the line it is drawn on,
    /// and the wires that may cross it (those that must, `bridgers`). JSON
    /// keys `path` and `mayCross`.
    std::vector<std::array<double, 2>> line;
    std::vector<std::string> mayCross;
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
  /// Every chord in `over` (`capacity_chords`), by its name there: the walls
  /// its two corners lie on and the middle of its line, which a chord has
  /// no edge of the graph to name (45q, 2026-10-31: the global SAT search
  /// found no chain end for a chord over and never moved one).
  struct ChordOver {
    std::vector<std::string> ends;
    std::array<double, 2> middle{};
  };
  std::map<std::string, ChordOver> chordsOver;
  /// The crossing slots the check placed across each feedline edge, by the
  /// edge's name.
  std::map<std::string, std::size_t> slots;
  /// Under escape rounds, after a SAT round: every gate the ways out of the
  /// slots the wires took narrow below the capacity the round gave it, by
  /// name, with the capacity left to it, the share the ways out take, the
  /// capacity before and the wires through it; `escapeOver` names those
  /// whose load passes what is left. Empty otherwise.
  std::map<std::string, std::int64_t> escapeCaps;
  std::map<std::string, double> escapeShares;
  std::map<std::string, std::int64_t> escapeBefore;
  std::map<std::string, std::vector<std::string>> escapeThrough;
  std::vector<std::string> escapeOver;
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
