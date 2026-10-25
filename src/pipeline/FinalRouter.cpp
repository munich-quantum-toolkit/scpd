/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/pipeline/FinalRouter.hpp"

#include "mqt-scpd/design/Bridges.hpp"
#include "mqt-scpd/design/Roles.hpp"
#include "mqt-scpd/flatbuffers/artifacts.hpp"
#include "mqt-scpd/flatbuffers/config.hpp"
#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/grid/Bottlenecks.hpp"
#include "mqt-scpd/grid/Chambers.hpp"
#include "mqt-scpd/grid/DistanceTransform.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"
#include "mqt-scpd/grid/PortBands.hpp"
#include "mqt-scpd/grid/ProfileCuts.hpp"
#include "mqt-scpd/grid/Rasterize.hpp"
#include "mqt-scpd/grid/RoutingBottlenecks.hpp"
#include "mqt-scpd/grid/Voronoi.hpp"
#include "mqt-scpd/pipeline/CapacityFlow.hpp"
#include "mqt-scpd/pipeline/CapacityPlanner.hpp"
#include "mqt-scpd/pipeline/CombinationOrder.hpp"
#include "mqt-scpd/pipeline/CouplerSession.hpp"
#include "mqt-scpd/pipeline/SlotCheck.hpp"
#include "mqt-scpd/pipeline/Stages.hpp"
#include "mqt-scpd/routing/AnalyticDubins.hpp"
#include "mqt-scpd/routing/ChainSearch.hpp"
#include "mqt-scpd/routing/CouplerInsertion.hpp"
#include "mqt-scpd/routing/CrossingConstraints.hpp"
#include "mqt-scpd/routing/DubinsRouter.hpp"
#include "mqt-scpd/routing/Heading.hpp"
#include "mqt-scpd/routing/MeanderInsertion.hpp"
#include "mqt-scpd/routing/Path.hpp"
#include "mqt-scpd/routing/PathGeometry.hpp"
#include "mqt-scpd/routing/Primitives.hpp"
#include "mqt-scpd/routing/RoomRules.hpp"
#include "mqt-scpd/routing/SearchScratch.hpp"
#include "mqt-scpd/routing/SelfIntersection.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <initializer_list>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <numbers>
#include <numeric>
#include <optional>
#include <queue>
#include <ranges>
#include <regex>
#include <set>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace mqt::scpd::pipeline {
namespace {

namespace fba = flatbuffers::artifacts;
namespace fbc = flatbuffers::config;
namespace fbd = flatbuffers::design;
namespace fbg = flatbuffers::geometry;

using routing::Heading;
using routing::MovePrimitives;
using routing::Path;
using routing::PathPoint;
using routing::RoutingObjective;

/// No wire holds this cell.
constexpr std::uint32_t NO_OWNER = std::numeric_limits<std::uint32_t>::max();

/// The bend radius of the primitives, in cells.
///
/// It is a property of the move set and not a knob: the primitive tables, the
/// swept-cell tries and the search state are all built for one radius, and the
/// design rule it has to clear is `min_bend_radius`, which is 50 layout units
/// against a router cell of about ten. The prototype uses five everywhere.
constexpr std::uint8_t BEND_RADIUS = 5;

/// The straight run of a resonator's lead off its coupler, in cells, after
/// the quarter turn out of the port: what the routing holds the resonator
/// to before its own search begins (`leadOf`). The coupler insertion lays
/// `coupler_lead_margin_cells` more, which the routing gives back.
constexpr std::uint32_t COUPLER_LEAD_STRAIGHT = 14;

/// What one direction change of a resonator's way costs against the price of
/// its cells, when the meander insertion picks the cheapest placement: the
/// prototype's `c_angle` of its strict insertion.
constexpr double MEANDER_BEND_PRICE = 100.0;

/// How wide a resonator's copper stands in the way of a feedline edge, in
/// cells, past the lead that carries the clearance (user, 2026-10-03).
///
/// Two cells says only one thing, and it is the thing that matters there: an
/// edge may lie beside a resonator far from any coupler — they have nothing
/// to do with each other, the feedline pass draws the resonator again, and
/// fencing the whole length at the clearance is what left edge f1 of chain 0
/// on 69q without a way — but it may not run **across** it.
constexpr std::uint32_t RESONATOR_COPPER = 2;

/// The most a cell of the wire proximity may hold.
///
/// The layer is sixteen bits now, but the search adds `PENALTY_SCALE * value`
/// into a `uint32_t` cost that also carries the whole way behind it, so the
/// value needs a roof. Four thousand is 400 000 a cell, about twenty-seven
/// bends on a 33-qubit grid, and even a way made entirely of such cells
/// stays an order of magnitude below where the cost would wrap.
constexpr std::uint32_t CEILING = 4000;

/// How many places the insertion will try before it gives a coupler up. The
/// first is the one the bias names; the rest are what a coupler whose every
/// orientation is refused there falls back on.

// --------------------------------------------------------------- The tuning

/// Every knob of the stage, already converted onto the grid it runs on.
///
/// Nothing here is a cell count that stands for a distance. What the
/// configuration carries is a length in layout units or a price, and the
/// conversion happens once, here, so that no call site can convert it a
/// second time or differently.
struct Tuning {
  /// What the stage keeps between two wires, in cells. The rule spans this
  /// many whole cells, so the search keeps a little more than the rule asks.
  std::uint32_t clearance = 0;
  /// The rule itself, in cells but not rounded. What the search keeps is the
  /// whole number above it; what a committed wire is judged by is this, which
  /// is the same figure the design-rule check uses.
  double spacing = 0.0;
  /// The straight run a wire leaves its source on, in cells.
  std::uint32_t straightStart = 0;
  /// The rule `min_straight_length` itself, in cells of the smaller cell
  /// side and not rounded: the length of a port run in the bottleneck
  /// analysis, the same along an axis and along a diagonal.
  double straightLength = 0.0;
  /// The same run, lengthened by `launcherLead`, in cells: what the coupler
  /// box keeps clear of a launcher. Never shorter than `straightStart`.
  std::uint32_t launcherStraight = 0;
  /// How far to either side of its own way a wire may be moved, in cells.
  std::uint32_t reach = 0;
  /// The same for both refinements: `refinement_corridor_spacings`.
  std::uint32_t refinementReach = 0;
  std::uint32_t innerReach = 0;
  std::uint32_t rounds = 0;
  std::uint32_t innerRounds = 0;
  std::uint32_t maxRelaxation = 0;
  std::uint32_t refinementRounds = 0;
  std::uint32_t feedlineRefinementRounds = 0;
  /// How long a resonator's way is made before the coupler is spliced in, in
  /// cells. Zero switches the meander off.
  double meanderLength = 0.0;
  /// The same in layout units, for the lines that report it.
  double meanderLengthUnits = 0.0;
  /// How long a resonator is when the stage is done: the design rule, in
  /// cells and in layout units, with the tolerance the rule allows. The
  /// coupler is spliced where the way left to the qubit is this long, and
  /// the way drawn again from the coupler is made this long exactly.
  double targetLength = 0.0;
  double targetLengthUnits = 0.0;
  double lengthTolerance = 0.0;
  double lengthToleranceUnits = 0.0;
  /// The coupler body, in cells: the run the resonator couples along and
  /// how far the feedline lies from it.
  std::uint32_t couplerLength = 0;
  std::uint32_t couplerHeight = 0;
  /// How many times a placed coupler may be turned to another of its
  /// options while the feedlines do not route.
  std::uint32_t repairTrials = 0;
  /// The targeted repair: how many ring members beside a segment are drawn
  /// again and how many local rip-up rounds it runs.
  std::uint32_t researchRingMembers = 2;
  std::uint32_t researchRounds = 2;
  /// The coupler insertion's knobs of `[stages.final]`, see `config.fbs`.
  /// Shares of `target_resonator_length`: where the coupler cuts the
  /// resonator, and the most it may leave it short by.
  double couplerBias = 0.98;
  double couplerMaxShortfall = 0.30;
  /// In cells: what a coupler keeps off the sides of the coupler box, how far
  /// an option is moved into the box, the lead's run beyond the rule, what a
  /// feedline edge keeps closed in front of a launcher beyond its run, and
  /// the slot a feedline edge keeps open at a coupler's port.
  std::int64_t couplerBoxMargin = 5;
  std::int64_t couplerShift = 20;
  std::uint32_t couplerLeadMargin = 5;
  std::int64_t edgeLauncherMargin = 6;
  std::uint32_t terminalSlot = 20;
  /// The offsets the option generator turns a pad by, one flag per eighth
  /// turn.
  std::array<bool, routing::NUM_HEADINGS> couplerOffsets{};
  /// The chain search's prices: the weight of an edge at a launcher, the
  /// bend price of a feedline edge as a factor on the outer routing's, and
  /// the price of one cell of length.
  double chainEndEdgeWeight = 0.1;
  double edgeBendFactor = 1.0;
  std::uint64_t chainLengthWeight = 0;
  /// Whether the insertion runs the capacity graph check at all, and how
  /// many steps the capacity rule may refuse before a run of options joins
  /// a chain.
  bool capacityCheck = true;
  std::uint32_t chainRuleRefusals = 1500;
  /// The capacity graph: the band beside a feedline edge where a wire may
  /// only cross at a right angle, in cells; the spacing of the crossing
  /// points in clearances; the detour a wire may take through a gap.
  int crossingReach = 10;
  double crossingSpacing = 1.4142135623730951;
  double detourFactor = 1.5;
  /// The bottleneck analysis: the widest gap, in wires, still a bottleneck,
  /// and the least rise of a narrowing, in cells.
  std::uint32_t bottleneckWires = 10;
  double bottleneckRise = 2.0;
  /// The room rules: the dead length at both ends of a straight run and the
  /// reach of the channel rule, in cells.
  std::uint32_t roomMargin = 10;
  std::size_t roomChannelReach = 80;
  /// The budgets of the global SAT search: option runs per segment,
  /// locally SAT states kept per chain, whole-chip combinations tried.
  std::uint32_t globalSatRuns = 8;
  std::size_t globalSatStates = 8;
  std::uint32_t globalSatTries = 200;
  /// The length-point clearance of the outer routing: its radius in cells
  /// (0 turns it off) and the half-width of the band as a share of the
  /// target length.
  std::uint32_t lengthPointRadius = 40;
  double lengthPointBand = 0.10;
  /// The feedline sweep's band of lengths as a multiple of the tolerance,
  /// the spacing of the meander's legs and the start of a resonator the
  /// meander leaves alone, in cells.
  double sweepLengthBand = 1.0;
  std::uint32_t meanderLegSpacing = 25;
  std::uint32_t meanderStartMargin = 0;
  /// How many ring neighbours to either side a wire is fenced against in
  /// the feedline pass and the refinements, and how far a terminal edge's
  /// run is guarded past the pad, in cells.
  std::uint32_t fencePairs = 3;
  std::uint32_t terminalStubExtra = 5;
  /// The halo of a wire let go of: its reach in clearances, and in the
  /// feedline pass the rising halo past the wire, up to `risePeak` times
  /// the flat price at `riseReach` clearances.
  std::uint32_t haloReach = 1;
  std::uint32_t riseReach = 3;
  std::uint32_t risePeak = 3;
  /// The slot check's settings, handed to it with every graph.
  double capacityMargin = 1.06;
  std::int64_t capacityMarginFromWires = 3;
  double resonatorPassage = 0.8;
  std::size_t slotPinCells = 10;
  std::int64_t escapeStraight = 5;
  double escapeStubMargin = 4.0;
  /// The last of the five phases to run, by the name of its snapshot, or
  /// empty for all five. A stop is for looking at one phase's work: the
  /// stage still counts what it came to and still writes its artifact.
  std::string stopAfter;
  /// The price of one eighth turn, in hundredths of a cell.
  std::uint16_t bendPenalty = 0;
  std::uint8_t wireProximityPenalty = 0;
  std::uint8_t staticProximityPenalty = 0;
  /// How far the price of running beside an obstacle reaches, in cells.
  std::uint32_t obstacleReach = 0;
  /// The approach of a terminal, in the geometry of a port's band: the
  /// straight length in steps and the half width in cells, along an axis and
  /// along a diagonal, where a step is longer and a strip narrower by the
  /// square root of two.
  std::uint32_t approachAxial = 0;
  std::uint32_t approachDiagonal = 0;
  std::uint32_t approachHalfAxial = 0;
  std::uint32_t approachHalfDiagonal = 0;
};

/// A price quoted against the extent of the grid, resolved to an absolute one.
///
/// A bend costs a fixed amount while the cost of a way grows with the grid, so
/// the same bend price is relatively cheaper on a bigger chip. The prototype
/// had to retune three penalties per chip until it quoted them this way, and
/// the spread it had hand-tuned then collapsed from 3.75 to about 2.
[[nodiscard]] std::uint32_t resolved(const double norm,
                                     const grid::GridMetrics& router) {
  const auto extent = static_cast<double>(router.width) + router.height;
  return static_cast<std::uint32_t>(std::lround(norm * extent));
}

/// A figure in cells along an axis, as cells along a heading.
///
/// Every figure of the tuning is a length in layout units converted once, on
/// the axis with the shorter cell step. A diagonal step is longer by the
/// square root of two, so the same length is fewer cells along it, and a
/// body laid out in axial cells on a diagonal would be half again as long as
/// the design rule asks.
[[nodiscard]] std::uint32_t cellsOn(const std::uint32_t axial,
                                    const routing::Heading heading) {
  if (!routing::isDiagonal(heading)) {
    return axial;
  }
  return std::max(1U, static_cast<std::uint32_t>(std::lround(
                          static_cast<double>(axial) / std::numbers::sqrt2)));
}

/// The offsets a comma list of eighth turns names; every offset but 3 and 4
/// for an empty list or one that names no offset.
[[nodiscard]] std::array<bool, routing::NUM_HEADINGS>
offsetsOf(const std::string& text) {
  std::array<bool, routing::NUM_HEADINGS> on{};
  on.fill(true);
  on.at(3) = false;
  on.at(4) = false;
  std::array<bool, routing::NUM_HEADINGS> listed{};
  bool any = false;
  std::string_view rest(text);
  while (!rest.empty()) {
    const auto comma = rest.find(',');
    auto item = rest.substr(0, comma);
    rest = comma == std::string_view::npos ? std::string_view{}
                                           : rest.substr(comma + 1);
    while (!item.empty() && item.front() == ' ') {
      item.remove_prefix(1);
    }
    while (!item.empty() && item.back() == ' ') {
      item.remove_suffix(1);
    }
    if (item.size() == 1 && item.front() >= '0' &&
        item.front() < static_cast<char>('0' + routing::NUM_HEADINGS)) {
      listed.at(static_cast<std::size_t>(item.front() - '0')) = true;
      any = true;
    }
  }
  return any ? listed : on;
}

[[nodiscard]] Tuning tuningOf(const ConfigT& config,
                              const grid::GridMetrics& router) {
  const fbc::FinalParamsT defaults;
  const auto& params =
      (config.stages != nullptr && config.stages->final != nullptr)
          ? *config.stages->final
          : defaults;
  const auto& rules = *config.rules;

  Tuning tuning;
  // The one conversion of the design rule. `cells_for` spans at least the
  // rule, so nineteen cells of about ten layout units is 190 against a rule of
  // 185; the prototype writes the nineteen out as a literal on every one of
  // its eight grids and records no reason for it.
  tuning.clearance =
      std::max(1U, grid::cellsFor(rules.min_wire_spacing, router));
  // The rule itself, unrounded. A committed way is judged by this and not by
  // the whole cells the search keeps, so that the verdict is the one the
  // design-rule check gives: `checkClearance` measures a pair in cells times
  // the smaller cell side, and the same side is the unit here.
  tuning.spacing =
      rules.min_wire_spacing / std::min(router.cellWidth, router.cellHeight);
  tuning.straightStart = grid::cellsFor(rules.min_straight_length, router);
  tuning.straightLength =
      rules.min_straight_length / std::min(router.cellWidth, router.cellHeight);
  tuning.launcherStraight = grid::cellsFor(
      rules.min_straight_length + std::max(0.0, params.launcher_lead), router);
  tuning.reach = std::max(1U, params.corridor_spacings * tuning.clearance);
  tuning.refinementReach =
      params.refinement_corridor_spacings == 0
          ? tuning.reach
          : std::max(1U,
                     params.refinement_corridor_spacings * tuning.clearance);
  tuning.innerReach =
      std::max(1U, params.inner_corridor_spacings * tuning.clearance);
  tuning.rounds = std::max(1U, params.rounds);
  tuning.innerRounds = std::max(1U, params.inner_rounds);
  tuning.maxRelaxation = params.max_relaxation;
  tuning.refinementRounds = params.refinement_rounds;
  tuning.feedlineRefinementRounds = params.feedline_refinement_rounds;
  // How long the outer routing makes a resonator's way is the design rule
  // itself, not a figure of its own. It used to be `meander_length`, carried
  // per chip and set to anything from 2500 to 6000 while every chip asked
  // for a 2500-unit resonator; what it actually decided was how much slack
  // the way had over the figure before the coupler cut it back. One number
  // for one thing: the way is made the length the rule asks for.
  tuning.meanderLength =
      rules.target_resonator_length <= 0.0
          ? 0.0
          : rules.target_resonator_length /
                std::min(router.cellWidth, router.cellHeight);
  tuning.meanderLengthUnits = std::max(0.0, rules.target_resonator_length);
  const auto unit = std::min(router.cellWidth, router.cellHeight);
  tuning.targetLength = std::max(0.0, rules.target_resonator_length) / unit;
  tuning.targetLengthUnits = std::max(0.0, rules.target_resonator_length);
  tuning.lengthTolerance =
      std::max(0.0, rules.resonator_length_tolerance) / unit;
  tuning.lengthToleranceUnits = std::max(0.0, rules.resonator_length_tolerance);
  tuning.couplerLength =
      std::max(1U, grid::cellsFor(params.coupler_length, router));
  tuning.couplerHeight =
      std::max(1U, grid::cellsFor(params.coupler_height, router));
  tuning.repairTrials = params.repair_trials;
  tuning.researchRingMembers = std::min(params.research_ring_members, 20U);
  tuning.researchRounds = std::clamp(params.research_rounds, 1U, 10U);
  tuning.couplerBias = std::clamp(params.coupler_bias, 0.5, 1.0);
  tuning.couplerMaxShortfall =
      std::clamp(params.coupler_max_shortfall, 0.0, 1.0);
  tuning.couplerBoxMargin =
      std::min<std::int64_t>(params.coupler_box_margin_cells, 500);
  tuning.couplerShift = std::min<std::int64_t>(params.coupler_shift_cells, 100);
  tuning.couplerLeadMargin = params.coupler_lead_margin_cells;
  tuning.edgeLauncherMargin =
      std::min<std::int64_t>(params.edge_launcher_margin_cells, 500);
  tuning.terminalSlot = std::min(params.terminal_slot_cells, 200U);
  tuning.couplerOffsets = offsetsOf(params.coupler_offsets);
  tuning.chainEndEdgeWeight =
      std::clamp(params.chain_end_edge_weight, 0.0, 1000.0);
  tuning.edgeBendFactor = std::clamp(params.edge_bend_factor, 0.01, 16.0);
  tuning.chainLengthWeight = std::min(params.chain_length_weight, 1000U);
  tuning.capacityCheck = params.capacity_check;
  tuning.chainRuleRefusals = params.chain_rule_refusals;
  tuning.crossingReach =
      std::clamp(static_cast<int>(params.crossing_reach_cells), 1, 200);
  tuning.crossingSpacing = std::clamp(params.crossing_spacing, 1.0, 4.0);
  tuning.detourFactor = std::clamp(params.detour_factor, 0.0, 100.0);
  tuning.bottleneckWires = std::clamp(params.bottleneck_wires, 1U, 100U);
  tuning.bottleneckRise = std::clamp(params.bottleneck_rise, 0.0, 1000.0);
  tuning.roomMargin = std::min(params.room_margin_cells, 200U);
  tuning.roomChannelReach = static_cast<std::size_t>(
      std::clamp(params.room_channel_reach_cells, 1U, 2000U));
  tuning.globalSatRuns = std::clamp(params.global_sat_runs, 1U, 1000U);
  tuning.globalSatStates =
      static_cast<std::size_t>(std::clamp(params.global_sat_states, 1U, 100U));
  tuning.globalSatTries = std::clamp(params.global_sat_tries, 1U, 100000U);
  tuning.lengthPointRadius = std::min(params.length_point_radius_cells, 200U);
  tuning.lengthPointBand =
      std::clamp(params.length_point_band_percent, 0.0, 100.0) / 100.0;
  tuning.sweepLengthBand = std::max(1.0, params.sweep_length_band);
  tuning.meanderLegSpacing =
      std::clamp(params.meander_leg_spacing_cells, 4U, 200U);
  // By default the meander leaves the lead's straight run alone and may
  // start on the first cell behind it.
  tuning.meanderStartMargin =
      params.meander_start_margin_cells < 0
          ? std::max(COUPLER_LEAD_STRAIGHT, tuning.straightStart)
          : static_cast<std::uint32_t>(params.meander_start_margin_cells);
  tuning.fencePairs = std::clamp(params.fence_pairs, 1U, 8U);
  tuning.terminalStubExtra = std::min(params.terminal_stub_extra_cells, 100U);
  tuning.haloReach = std::clamp(params.halo_reach, 1U, 12U);
  tuning.riseReach = std::clamp(params.rise_reach, 1U, 12U);
  tuning.risePeak = std::clamp(params.rise_peak, 1U, 50U);
  tuning.capacityMargin = std::clamp(params.capacity_margin, 1.0, 2.0);
  tuning.capacityMarginFromWires = static_cast<std::int64_t>(
      std::clamp(params.capacity_margin_from_wires, 1U, 100U));
  tuning.resonatorPassage = std::clamp(params.resonator_passage, 0.0, 1.0);
  tuning.slotPinCells = params.slot_pin_cells;
  tuning.escapeStraight =
      static_cast<std::int64_t>(std::min(params.escape_straight_cells, 100U));
  tuning.escapeStubMargin =
      static_cast<double>(std::min(params.escape_stub_margin_cells, 100U));
  // Where the run ends: `[run] stop_after` names a step, and the five phases
  // of this stage are steps.
  tuning.stopAfter = config.run != nullptr ? config.run->stop_after : "";
  tuning.bendPenalty = static_cast<std::uint16_t>(
      std::min<std::uint32_t>(std::numeric_limits<std::uint16_t>::max(),
                              resolved(params.bend_penalty_norm, router)));
  // The search adds a penalty per swept cell, so it has to stay well below
  // the 127 the router accepts or one cell of it would outweigh a whole bend.
  tuning.wireProximityPenalty =
      static_cast<std::uint8_t>(std::min<std::uint32_t>(
          127U, resolved(params.wire_proximity_penalty_norm, router)));
  tuning.staticProximityPenalty =
      static_cast<std::uint8_t>(std::min<std::uint32_t>(
          127U, resolved(params.static_proximity_penalty_norm, router)));
  tuning.obstacleReach = grid::cellsFor(params.obstacle_penalty_reach, router);
  tuning.approachAxial = grid::bandLength(rules.min_straight_length, router,
                                          grid::Step{.x = 1, .y = 0});
  tuning.approachDiagonal = grid::bandLength(rules.min_straight_length, router,
                                             grid::Step{.x = 1, .y = 1});
  tuning.approachHalfAxial = grid::bandHalfWidth(rules.min_wire_spacing, router,
                                                 grid::Step{.x = 1, .y = 0});
  tuning.approachHalfDiagonal = grid::bandHalfWidth(
      rules.min_wire_spacing, router, grid::Step{.x = 1, .y = 1});
  return tuning;
}

// ---------------------------------------------------------------- The scene

/// The grid the stage searches on, and what it may not enter.
///
/// None of it is an artifact. It is rebuilt from the chip and the
/// configuration, as every grid of a run is, which is what keeps a resumed run
/// equal to an uninterrupted one.
struct Scene {
  grid::GridMetrics router;
  /// The chip artwork, its obstacle keepout, the border and the ports' own
  /// approaches. Every cell a search may enter satisfies the obstacle rule
  /// because this mask already holds it.
  grid::BitGrid blocked;
  /// The artwork alone: the qubits, the couplers and the launchers as
  /// `rasterizeObstacles` lays them on the grid, with the launcher cells
  /// freed. None of what `blocked` adds on top of it — not the block over
  /// everything outside the launchers, not the approach band of every port.
  /// A feedline edge is drawn against this and the launcher stubs and
  /// nothing else.
  grid::BitGrid components;
  /// The cell each routable port is reached at, by port index.
  std::unordered_map<std::uint32_t, std::size_t> targetCell;
  /// The heading a wire has when it arrives at each port, by port index.
  std::unordered_map<std::uint32_t, Heading> arrival;
  /// The cell each launcher stands on, by port index: where a feedline
  /// chain starts and ends. Freed in the mask, because a wire has to be
  /// able to stand where it starts.
  std::unordered_map<std::uint32_t, std::size_t> launcherCell;
  /// The heading a wire leaves each launcher on, by port index. An outer
  /// wire is fed from a point on the ring and carries no port for its
  /// launcher, so this is the only place the launcher's own direction is
  /// kept.
  std::unordered_map<std::uint32_t, routing::Heading> launcherHeading;
};

/// Block everything between the edge of the chip and the ring the wires are
/// fed from.
///
/// Every wire starts on the ring of launcher slots, which is a rectangle set
/// in from the chip outline. What lies outside it is free space that no wire
/// has any business in: a wire that enters it comes back in somewhere else and
/// has gone *around* the sources of the wires beside it rather than past them,
/// which is a crossing the plan never allowed. The capacity grid keeps the
/// same strip clear through the configured launcher offset; here the strip is
/// derived, because where the sources are is a fact of the chip and not a
/// figure to tune.
void blockOutsideTheSources(const ChipT& chip, Scene& scene) {
  std::uint32_t lowX = scene.router.width - 1;
  std::uint32_t lowY = scene.router.height - 1;
  std::uint32_t highX = 0;
  std::uint32_t highY = 0;
  bool found = false;
  for (const auto& port : chip.ports) {
    if (port->role != fbd::UnassignedRole::Launcher) {
      continue;
    }
    const auto cell = scene.router.clampToCell(port->center);
    lowX = std::min(lowX, cell.x());
    highX = std::max(highX, cell.x());
    lowY = std::min(lowY, cell.y());
    highY = std::max(highY, cell.y());
    found = true;
  }
  if (!found || lowX > highX || lowY > highY) {
    return;
  }
  for (std::uint32_t y = 0; y < scene.router.height; ++y) {
    const bool outsideRow = y < lowY || y > highY;
    for (std::uint32_t x = 0; x < scene.router.width; ++x) {
      if (outsideRow || x < lowX || x > highX) {
        scene.blocked.setCell(x, y, true);
      }
    }
  }
}

[[nodiscard]] Scene sceneOf(const ChipT& chip, const ConfigT& config,
                            const grid::GridMetrics& capacity) {
  const auto& rules = *config.rules;

  Scene scene;
  scene.router = grid::routerGrid(capacity, config.grid->router_cell_size);

  const design::BridgeRules noRules;
  const auto& bridgeRules =
      config.ports != nullptr ? config.ports->bridge_pairs : noRules;
  const auto bridging = design::bridgingPorts(chip, bridgeRules);

  // The obstacle clearance cannot be expressed in a search step: the router
  // knows a wire only by its centre line and may enter any free cell. So it is
  // baked into the mask instead, as an exact distance in layout units from the
  // cell to the polygon edge rather than a dilation of the finished raster —
  // a dilation would carry the half cell the fill convention differs by, and
  // would widen the edge seam a second time.
  //
  // What the keepout would otherwise seal is the ports themselves: every port
  // sits at the foot of a small polygon of its own. Each keeps a corridor out
  // along its orientation, as long as the approach its band reserves.
  grid::RasterOptions options;
  options.keepout = rules.min_obstacle_spacing;
  for (std::uint32_t index = 0; index < chip.ports.size(); ++index) {
    const auto& port = *chip.ports[index];
    if (!design::isRoutable(port.role) &&
        port.role != fbd::UnassignedRole::Launcher) {
      continue;
    }
    const auto step = grid::orientationStep(port.orientation);
    if (step.x == 0 && step.y == 0) {
      continue;
    }
    const double reach =
        (bridging[index] ? 2.0 : 1.0) * rules.min_straight_length;
    const double length = step.diagonal() ? reach / std::sqrt(2.0) : reach;
    options.keepoutExemptions.push_back(
        {.from = port.center,
         .to = {port.center.x() + (step.x * length),
                port.center.y() + (step.y * length)},
         .halfWidth = 0.5 * rules.min_wire_spacing});
  }

  const auto raster = grid::rasterizeObstacles(chip, scene.router, options);
  scene.blocked = raster.blocked;
  // Kept before anything else is stamped into the mask: the artwork on its
  // own, which is all a feedline edge has to stay out of.
  scene.components = raster.blocked;
  blockOutsideTheSources(chip, scene);

  // A launcher's slot is the launcher, as the capacity scene has it: the
  // cell its centre rounds to, freed so that a feedline can stand on it. The
  // keepout exemption above keeps the run out of the pad open.
  for (std::uint32_t index = 0; index < chip.ports.size(); ++index) {
    const auto& port = *chip.ports[index];
    if (port.role != fbd::UnassignedRole::Launcher) {
      continue;
    }
    if (routing::headingOfOrientation(port.orientation) >=
        routing::NUM_HEADINGS) {
      continue;
    }
    const auto cell = scene.router.clampToCell(port.center);
    const auto slot = scene.router.index(cell.x(), cell.y());
    scene.blocked.set(slot, false);
    scene.components.set(slot, false);
    scene.launcherCell.emplace(index, slot);
    scene.launcherHeading.emplace(
        index,
        routing::reverse(routing::headingOfOrientation(port.orientation)));
  }

  // The approach of every routable port, as the prototype's
  // `add_fixed_port_obstacles` stamps it: a strip one wire spacing wide that
  // runs the minimum straight length out of the port and further back behind
  // it, so that a wire reaches the port along its orientation and no other
  // wire runs over the approach. The target is the cell just beyond it.
  for (std::uint32_t index = 0; index < chip.ports.size(); ++index) {
    const auto& port = *chip.ports[index];
    if (!design::isRoutable(port.role)) {
      continue;
    }
    const auto step = grid::orientationStep(port.orientation);
    if (step.x == 0 && step.y == 0) {
      continue;
    }
    const auto heading = routing::headingOfOrientation(port.orientation);
    if (heading >= routing::NUM_HEADINGS) {
      continue;
    }
    // The target is the first cell along the step whose centre lies the
    // straight length or more from the port itself — the port's own
    // position, not the centre of the cell it falls on, which can be half a
    // cell off — and the band ends on the cell before it. So the straight
    // run into the port exceeds the rule by less than one step: on a grid of
    // ten-unit cells by up to ten units along an axis and fourteen along a
    // diagonal, where a step count of whole band lengths put it twenty to
    // thirty units over.
    const auto reach =
        (bridging[index] ? 2.0 : 1.0) * rules.min_straight_length;
    const auto centre = scene.router.clampToCell(port.center);
    // Signed throughout: a step of -1 times an unsigned count wraps around,
    // and the walk would leave the grid on its first step.
    std::uint32_t steps = 0;
    for (std::int64_t k = 1; steps == 0; ++k) {
      const auto x = static_cast<std::int64_t>(centre.x()) + (step.x * k);
      const auto y = static_cast<std::int64_t>(centre.y()) + (step.y * k);
      if (x < 0 || y < 0 || x >= scene.router.width ||
          y >= scene.router.height) {
        break;
      }
      const auto at =
          scene.router.toLayout(static_cast<double>(x), static_cast<double>(y));
      if (std::hypot(at.x() - port.center.x(), at.y() - port.center.y()) >=
          reach) {
        steps = static_cast<std::uint32_t>(k);
      }
    }
    // The band reaches back into the component as well, so that a port on
    // an island of its own, not flush with the body behind it, leaves no
    // gap a wire could run into. A conventional port reaches back twice as
    // far, as the prototype's `expand_backward` does (user, 2026-10-14).
    const auto back =
        port.role == fbd::UnassignedRole::Conventional ? 6.0 : 3.0;
    const grid::PortBand band{
        .center = centre,
        .step = step,
        .forward = steps == 0 ? grid::bandLength(reach, scene.router, step)
                              : steps - 1,
        .backward = grid::bandLength(back * rules.min_straight_length,
                                     scene.router, step),
        .halfWidth =
            grid::bandHalfWidth(rules.min_wire_spacing, scene.router, step)};
    // The part behind the port seals the artwork too (user, 2026-10-14):
    // the feedline edges of the coupler insertion and the walls of the
    // capacity graph read `components`, and without it they see a passage
    // between the island and the body that no wire has.
    {
      auto seal = band;
      seal.forward = 0;
      static_cast<void>(grid::stampBand(scene.components, seal));
    }
    const auto stamped = grid::stampBand(scene.blocked, band);
    // The router grid keeps no mask from before the bands, so the target is
    // dug rather than placed: the walk frees the band cells it crosses, which
    // is the way out of the port's own approach. This is the one caller
    // `digTargetBeyondBand` was written for.
    if (const auto target =
            grid::digTargetBeyondBand(scene.blocked, stamped, step)) {
      scene.targetCell.emplace(index, *target);
      scene.arrival.emplace(index, heading);
    }
  }
  return scene;
}

// ------------------------------------------------------------ The clearance

/// The offsets of one clearance disc, and what enters it on each step.
///
/// Charging the disc around every cell of a wire costs the disc for every
/// cell, and the disc of nineteen cells holds eleven hundred of them. Two
/// consecutive cells of a path differ by one step, so all but a leading edge
/// of the disc is already charged: the full disc is stamped once, at the first
/// cell, and one edge per step after that. The prototype does the same, and
/// what it buys is a factor of thirty.
struct Stencil {
  using Offsets = std::vector<std::pair<std::int32_t, std::int32_t>>;
  Offsets full;
  /// The offsets that enter the disc on a step, indexed `[dy + 1][dx + 1]`.
  std::array<std::array<Offsets, 3>, 3> edge;
};

[[nodiscard]] Stencil stencilOf(const std::uint32_t radius) {
  Stencil stencil;
  const auto reach = static_cast<std::int32_t>(radius);
  const auto limit = reach * reach;
  const auto inside = [limit, reach](const std::int32_t dx,
                                     const std::int32_t dy) {
    return dx >= -reach && dx <= reach && dy >= -reach && dy <= reach &&
           ((dx * dx) + (dy * dy)) <= limit;
  };
  for (std::int32_t dy = -reach; dy <= reach; ++dy) {
    for (std::int32_t dx = -reach; dx <= reach; ++dx) {
      if (inside(dx, dy)) {
        stencil.full.emplace_back(dx, dy);
      }
    }
  }
  for (std::int32_t sy = -1; sy <= 1; ++sy) {
    for (std::int32_t sx = -1; sx <= 1; ++sx) {
      if (sx == 0 && sy == 0) {
        continue;
      }
      auto& edge = stencil.edge[static_cast<std::size_t>(sy + 1)]
                               [static_cast<std::size_t>(sx + 1)];
      for (const auto& [dx, dy] : stencil.full) {
        if (!inside(dx + sx, dy + sy)) {
          edge.emplace_back(dx, dy);
        }
      }
    }
  }
  return stencil;
}

/// The copper of the chip, and the room every wire keeps around it.
///
/// Two fields over the router grid. `owner` says which wire holds a cell;
/// `guard` counts how many wires keep their clearance over a cell. A wire
/// charges its guard when it is put down and gives it up when it is taken
/// off.
///
/// The field is what a wire is *judged* by, not what its search is fenced in
/// by. The search keeps the prototype's two ring neighbours, inflated by the
/// clearance, and nothing else (`Driver::fence`); the fails of a pass are
/// counted on this field against every other wire (`Driver::conflictsIn`).
/// Charged once per move, the count against two hundred wires costs what the
/// count against two cost.
class Field {
public:
  Field(const grid::GridMetrics& router, const std::uint32_t clearance)
      : stencil_(stencilOf(clearance)), width_(router.width),
        height_(router.height), guard_(router.cells(), 0),
        owner_(router.cells(), NO_OWNER), stamp_(router.cells(), 0) {}

  [[nodiscard]] bool guarded(const std::size_t cell) const {
    return guard_[cell] != 0;
  }
  [[nodiscard]] std::uint16_t guardCount(const std::size_t cell) const {
    return guard_[cell];
  }
  [[nodiscard]] std::uint32_t owner(const std::size_t cell) const {
    return owner_[cell];
  }

  /// Take the copper of a wire, without its room.
  void occupy(const Path& path, const std::uint32_t wire) {
    note(path, 0);
    for (const auto& point : path) {
      if (point.x < width_ && point.y < height_) {
        owner_[index(point.x, point.y)] = wire;
      }
    }
  }

  /// Give up the copper of a wire. A cell another wire has since taken is
  /// left alone, which cannot happen while no two wires share a cell but
  /// costs nothing to say.
  void vacate(const Path& path, const std::uint32_t wire) {
    note(path, 0);
    for (const auto& point : path) {
      if (point.x < width_ && point.y < height_) {
        const auto cell = index(point.x, point.y);
        if (owner_[cell] == wire) {
          owner_[cell] = NO_OWNER;
        }
      }
    }
  }

  /// Charge the room a wire keeps, or give it up again.
  void charge(const Path& path) { walk(path, 1); }
  void discharge(const Path& path) { walk(path, -1); }

  /// The same for the places a wire cannot be moved off.
  void chargeFixed(const Path& fixed) { walk(fixed, 1); }
  void dischargeFixed(const Path& fixed) { walk(fixed, -1); }

  /// Start to record the rows every later change writes, for
  /// `restoreRowsFrom`.
  void track() {
    reach_ = 0;
    for (const auto& offset : stencil_.full) {
      reach_ = std::max<std::int64_t>(reach_, std::abs(offset.second));
    }
    tracking_ = true;
    dirtyLo_ = height_;
    dirtyHi_ = -1;
  }

  /// Stop recording rows.
  void untrack() { tracking_ = false; }

  /// Copy back from `from` every row written since `track`, and the pass
  /// counter. When this field equalled `from` at `track`, it equals `from`
  /// again, cell for cell.
  void restoreRowsFrom(const Field& from) {
    if (dirtyHi_ >= dirtyLo_) {
      const auto begin = static_cast<std::ptrdiff_t>(dirtyLo_ * width_);
      const auto end = static_cast<std::ptrdiff_t>((dirtyHi_ + 1) * width_);
      std::copy(from.guard_.begin() + begin, from.guard_.begin() + end,
                guard_.begin() + begin);
      std::copy(from.owner_.begin() + begin, from.owner_.begin() + end,
                owner_.begin() + begin);
      std::copy(from.stamp_.begin() + begin, from.stamp_.begin() + end,
                stamp_.begin() + begin);
    }
    pass_ = from.pass_;
    track();
  }

  /// Whether two fields hold the same room, copper and stamps.
  [[nodiscard]] bool sameAs(const Field& other) const {
    return guard_ == other.guard_ && owner_ == other.owner_ &&
           stamp_ == other.stamp_ && pass_ == other.pass_;
  }

private:
  /// Record the rows a path writes, `reach` rows beyond it on each side.
  void note(const Path& path, const std::int64_t reach) {
    if (!tracking_) {
      return;
    }
    for (const auto& point : path) {
      const auto y = static_cast<std::int64_t>(point.y);
      dirtyLo_ = std::min(dirtyLo_, std::max<std::int64_t>(0, y - reach));
      dirtyHi_ = std::min(height_ - 1, std::max(dirtyHi_, y + reach));
    }
  }

  [[nodiscard]] std::size_t index(const std::uint32_t x,
                                  const std::uint32_t y) const {
    return (static_cast<std::size_t>(y) * static_cast<std::size_t>(width_)) + x;
  }

  /// Apply a change to every cell within the rule of the path, once each.
  ///
  /// The stamp is what makes it once each: the leading edges of the steps
  /// overlap wherever a path turns, and a path that comes back on itself
  /// covers the same cells twice. Charging a cell twice for one wire would
  /// leave it guarded after the wire is gone.
  void walk(const Path& path, const int by) {
    if (path.empty()) {
      return;
    }
    note(path, reach_);
    beginPass();
    bool started = false;
    std::int64_t lastX = 0;
    std::int64_t lastY = 0;
    for (const auto& point : path) {
      const auto x = static_cast<std::int64_t>(point.x);
      const auto y = static_cast<std::int64_t>(point.y);
      if (started) {
        const auto sx = x - lastX;
        const auto sy = y - lastY;
        if (sx == 0 && sy == 0) {
          continue;
        }
        if (sx >= -1 && sx <= 1 && sy >= -1 && sy <= 1) {
          stamp(stencil_.edge[static_cast<std::size_t>(sy + 1)]
                             [static_cast<std::size_t>(sx + 1)],
                x, y, by);
          lastX = x;
          lastY = y;
          continue;
        }
      }
      stamp(stencil_.full, x, y, by);
      started = true;
      lastX = x;
      lastY = y;
    }
  }

  void beginPass() {
    if (++pass_ == 0) {
      std::ranges::fill(stamp_, 0);
      pass_ = 1;
      dirtyLo_ = 0;
      dirtyHi_ = height_ - 1;
    }
  }

  void stamp(const Stencil::Offsets& offsets, const std::int64_t cx,
             const std::int64_t cy, const int by) {
    for (const auto& [dx, dy] : offsets) {
      const auto x = cx + dx;
      const auto y = cy + dy;
      if (x < 0 || y < 0 || x >= width_ || y >= height_) {
        continue;
      }
      const auto cell = static_cast<std::size_t>((y * width_) + x);
      if (stamp_[cell] == pass_) {
        continue;
      }
      stamp_[cell] = pass_;
      guard_[cell] = static_cast<std::uint16_t>(guard_[cell] + by);
    }
  }

  Stencil stencil_;
  std::int64_t width_;
  std::int64_t height_;
  std::vector<std::uint16_t> guard_;
  std::vector<std::uint32_t> owner_;
  std::vector<std::uint32_t> stamp_;
  std::uint32_t pass_ = 0;
  /// The rows written since `track`, and how far the stencil reaches.
  bool tracking_ = false;
  std::int64_t reach_ = 0;
  std::int64_t dirtyLo_ = 0;
  std::int64_t dirtyHi_ = -1;
};

// ----------------------------------------------------------------- The wires

/// One connection of the plan, and the way it has.
struct Wire {
  RoutingObjective objective;
  /// The way it has: the Detail stage's cells at first, joined on this grid
  /// and put down like any other way, its own after it has been routed. The
  /// corridor of every search is a band around this, and a wire whose search
  /// finds nothing keeps it.
  Path way;
  /// Whether the way was drawn by this stage. A way that was not is a seed:
  /// it holds copper and room on the canvas, but it is not curvature-
  /// constrained and knows nothing of this grid's keepout, so a wire still
  /// on it when the rounds end is unrouted and the artifact carries no cells
  /// for it.
  bool drawn = false;
  /// Whether it has been routed in the pass running now.
  bool routed = false;
  /// Whether its room is charged into the field.
  bool placed = false;
  /// Whether only its fixed places are charged, which is what a wire without
  /// a way of its own holds until it is drawn.
  bool endsOnly = false;
  /// The places it cannot be moved off: the straight run it leaves its source
  /// on, which its port's orientation fixes, and the cell of its target port.
  Path fixed;
  bool resonator = false;
  /// A resonator whose target is one end of a bridge: the inner wire beyond
  /// the bridge is the resonator and carries its length, and this one only
  /// feeds it, with the coupler on it and no length of its own.
  bool feedsABridge = false;
  /// An inner wire from one end of a bridge to a resonator port: the part
  /// of a resonator beyond the bridge, which carries the resonator's length.
  bool resonatorBeyondBridge = false;
  /// The run from the cell its way ends on to the port itself, in cells. The
  /// sampler measures a way to its last cell, and the wire runs on to the
  /// port from there; a resonator's length counts both.
  double anchorGap = 0.0;
  /// The same run in layout units, for the lines that report it.
  double anchorGapUnits = 0.0;
  /// Whether its way is shorter than a resonator's has to be: drawn, but
  /// without room for the meander that would make it long enough.
  bool tooShort = false;
  bool inner = false;
  /// Its place in the artifact: the connection of the assignment, or of the
  /// global stage for an inner wire.
  std::uint32_t slot = 0;
  /// Its place in the wire list, which is what the field records per cell.
  std::uint32_t key = 0;
  /// Whether both of its ends lie on the grid.
  bool feasible = false;
  /// The ports its two ends are, where they are ports at all. A resonator is
  /// fed on the segment between two launchers, so its source is a point and
  /// no port stands there.
  std::uint32_t sourcePort = NO_OWNER;
  std::uint32_t targetPort = NO_OWNER;
  /// Whether it is an edge of a feedline chain, and which.
  bool feedline = false;
  std::uint32_t edge = NO_OWNER;
  /// A feedline edge that starts or ends at a launcher: a hard obstacle for
  /// every wire.
  bool terminal = false;
  /// The couplers at its ends: a resonator's own, a feedline edge's two, by
  /// their index in the coupler list. Where two wires share one, the rule
  /// does not bind near it.
  std::uint32_t couplerAtSource = NO_OWNER;
  std::uint32_t couplerAtTarget = NO_OWNER;
  /// The straight run it leaves its source on and arrives at its target on,
  /// in cells, where the pass's own figure does not apply: a resonator
  /// drawn from its coupler runs the coupling length straight, a feedline
  /// edge runs through the coupler body it arrives at.
  std::optional<std::uint32_t> startStub;
  std::uint32_t endStub = 0;
  /// The head of a resonator drawn from its coupler: the coupling run from
  /// the anchor and the quarter turn after it, up to the source of the
  /// search. It is put in front of every way the search finds, and it is a
  /// fixed place.
  Path arc;
  /// The feedline edge a conventional wire may cross, at a right angle: the
  /// edge of the chain that spans its launcher. Everything else is fenced.
  std::uint32_t bridged = NO_OWNER;
  /// Whether the way is longer than a resonator may be, in the feedline
  /// phase, where no meander can help it.
  bool tooLong = false;
  /// Whether a neighbour's relaxation let go of it since it was last drawn:
  /// then it is drawn again even where its way holds, because letting it
  /// go was a promise that it moves.
  bool ripped = false;
  /// The sweep and the round, one based, in which the wire found no way at
  /// any level of the relaxation; 0 for none.
  std::uint64_t relaxedOut = 0;
};

/// What one pass of the driver does.
struct Pass {
  /// What to call it in the progress lines.
  std::string name;
  std::uint32_t rounds = 1;
  std::uint32_t maxRelaxation = 0;
  /// The band around a wire's own way, in cells.
  std::uint32_t reach = 0;
  /// The straight run out of the source, in cells.
  std::uint32_t straightStart = 0;
  /// Whether a wire that already has a drawn way is left alone.
  bool keepDrawn = true;
  /// Whether the feedlines constrain the pass: a wire crosses a feedline
  /// at a right angle only, the edges of the chains are fenced, and a
  /// resonator is made its target length exactly.
  bool feedlines = false;
  /// Whether the pass redraws only the members already flagged
  /// `routed = false` and leaves every other member on the way it holds,
  /// standing as the fences and lanes of the ones redrawn. Off, a pass under
  /// the feedline constraints clears `routed` on every member in
  /// `beginPass`, so every member is drawn again.
  ///
  /// The targeted repair's local rip-up-and-reroute is what it is for
  /// (2026-10-04): the window of a candidate run of coupler options is
  /// flagged, the **whole ring** is swept, and only the window and whatever
  /// the relaxation releases are redrawn — while the ±1/±2 fences and the
  /// lane see the true ring neighbours. A pass over a subset of `members`
  /// could not give that: `attempt` reads its neighbours off
  /// `members[slot ± 1]`, and the relaxation releases `members[slot ± level]`.
  /// Needs `keepDrawn`, which is what makes the sweep skip a routed member.
  bool onlyUnsettled = false;
  /// Whether this is a refinement pass — one attempt per wire, no rip-up,
  /// a rollback onto the way the wire had. `refine` sets nothing itself; the
  /// flag is what tells `lengthensIn` that `SCPD_REFINE_MEANDER` applies
  /// and the checks at the end of the pass which name to say.
  bool refinement = false;
};

// ------------------------------------------------------------ Joining cells

/// The heading of one step between two neighbouring cells, or the heading the
/// first cell has when the two are one place.
[[nodiscard]] Heading headingOfStep(const PathPoint& from,
                                    const PathPoint& to) {
  const int dx = (to.x > from.x) ? 1 : ((to.x < from.x) ? -1 : 0);
  const int dy = (to.y > from.y) ? 1 : ((to.y < from.y) ? -1 : 0);
  for (Heading heading = 0; heading < routing::NUM_HEADINGS; ++heading) {
    const auto step = routing::headingVector(heading);
    if (step.dx == dx && step.dy == dy) {
      return heading;
    }
  }
  return from.heading;
}

/// Extend a way to a cell by the eight-connected steps of the line between
/// its last cell and that one, so that no two consecutive cells of the way
/// are more than one step apart. A cell the way already ends on adds nothing.
void connect(Path& way, const PathPoint& to) {
  auto x = static_cast<std::int64_t>(way.back().x);
  auto y = static_cast<std::int64_t>(way.back().y);
  const auto targetX = static_cast<std::int64_t>(to.x);
  const auto targetY = static_cast<std::int64_t>(to.y);
  const auto dx = std::abs(targetX - x);
  const auto dy = -std::abs(targetY - y);
  const std::int64_t sx = x < targetX ? 1 : -1;
  const std::int64_t sy = y < targetY ? 1 : -1;
  auto error = dx + dy;
  while (x != targetX || y != targetY) {
    const auto doubled = 2 * error;
    if (doubled > dy) {
      error += dy;
      x += sx;
    }
    if (doubled < dx) {
      error += dx;
      y += sy;
    }
    way.push_back({.x = static_cast<std::uint32_t>(x),
                   .y = static_cast<std::uint32_t>(y),
                   .heading = 0,
                   .primitive = 0});
  }
}

// --------------------------------------------------------- The debug records

/// The five phases of the stage, in the order they run and by the names
/// their snapshots and `[run] stop_after` carry.
constexpr std::array<std::string_view, 5> PHASES{
    "pre-feedline-routing", "pre-feedline-refinement", "feedline-insertion",
    "feedline-constrained-routing", "feedline-constrained-refinement"};

/// The kinds of record the `[debug]` section asks for: "grid",
/// "search:<phase>", "coupler-options", "capacity-graph", "step-check",
/// "global-sat" and "phase". A run records nothing of a kind no page needs.
using DebugRecords = std::set<std::string>;

/// The record kinds the pages of `config.debug` need.
[[nodiscard]] DebugRecords recordsWantedBy(const ConfigT& config) {
  DebugRecords wanted;
  const auto* debug = config.debug.get();
  if (debug == nullptr) {
    return wanted;
  }
  const auto dashboard = [&](const bool on, const std::string_view phase) {
    if (on) {
      wanted.insert("grid");
      wanted.insert(std::format("search:{}", phase));
    }
  };
  dashboard(debug->pre_feedline_routing_dashboard, PHASES[0]);
  dashboard(debug->pre_feedline_refinement_dashboard, PHASES[1]);
  dashboard(debug->feedline_constrained_routing_dashboard, PHASES[3]);
  dashboard(debug->feedline_constrained_refinement_dashboard, PHASES[4]);
  if (debug->feedline_insertion) {
    wanted.insert("grid");
    wanted.insert("coupler-options");
    wanted.insert("capacity-graph");
  }
  if (debug->local_sat) {
    wanted.insert("step-check");
  }
  if (debug->global_sat) {
    wanted.insert("global-sat");
  }
  if (debug->html) {
    wanted.insert("phase");
  }
  return wanted;
}

/// Where a search stands, for the record of it.
struct DebugFrame {
  /// The pass, by its first word: "inner", "outer", "refinement".
  std::string pass;
  std::uint32_t round = 0;
  bool forward = true;
  /// "normal" for phase 1, "relax N" for the relaxation, "refine".
  std::string kind = "normal";
  /// The wires whose inflated ways fence the search.
  std::vector<std::uint32_t> fence;
  /// The wires let go of, which the search may cross.
  std::vector<std::uint32_t> ripped;
  /// The two ring neighbours, which bound the lane.
  std::uint32_t before = NO_OWNER;
  std::uint32_t after = NO_OWNER;
  const std::vector<Wire>* wires = nullptr;
};

/// The first word of a pass name, for the record of a search.
[[nodiscard]] std::string passTag(const std::string_view name) {
  const auto space = name.find(' ');
  return std::string(space == std::string_view::npos ? name
                                                     : name.substr(0, space));
}

/// A wire as its picture names it: the ring index, or "i" and the index of
/// the inner circuit.
[[nodiscard]] std::string wireId(const Wire& wire) {
  if (wire.feedline) {
    return std::format("f{}", wire.slot);
  }
  return wire.inner ? std::format("i{}", wire.slot)
                    : std::format("{}", wire.slot);
}

/// A port as the chip carries it, on the router grid: the cell its centre
/// falls on, the step its orientation takes, and its label. Not the cell a
/// wire is routed to, which lies beyond the port's band.
struct PortMark {
  std::int64_t x = 0;
  std::int64_t y = 0;
  /// The exact position, in fractional cells: the integer `i` is the centre
  /// of cell `i`, so `x` and `y` are these rounded.
  double fx = 0.0;
  double fy = 0.0;
  grid::Step step;
  std::uint32_t index = 0;
  std::string label;
};

[[nodiscard]] std::vector<PortMark> portMarksOf(const ChipT& chip,
                                                const Scene& scene) {
  std::vector<PortMark> marks;
  marks.reserve(chip.ports.size());
  for (std::uint32_t index = 0; index < chip.ports.size(); ++index) {
    const auto& port = *chip.ports[index];
    const auto cell = scene.router.clampToCell(port.center);
    const auto exact = scene.router.toCell(port.center);
    marks.push_back({.x = cell.x(),
                     .y = cell.y(),
                     .fx = exact.x(),
                     .fy = exact.y(),
                     .step = grid::orientationStep(port.orientation),
                     .index = index,
                     .label = port.label});
  }
  return marks;
}

/// A cell of the router grid, as `x` and `y`.
using Cell = std::pair<std::int64_t, std::int64_t>;

/// The cells of a way.
[[nodiscard]] std::vector<Cell> cellsOf(const Path& way) {
  std::vector<Cell> cells;
  cells.reserve(way.size());
  for (const auto& point : way) {
    cells.emplace_back(point.x, point.y);
  }
  return cells;
}

// -------------------------------------------------------------- The driver

/// The router grid of a run, and every search the stage makes on it.
///
/// One driver runs every routing phase. The prototype writes the loop out four
/// times — once for the inner circuit, once for the ring, once for the wires
/// under the feedline constraints and once for the refinement — and the four
/// differ only in their parameters.
class Driver {
public:
  Driver(const Scene& scene, const Tuning& tuning, Report report = {},
         Debug debug = {}, DebugRecords wanted = {})
      : scene_(scene), tuning_(tuning), report_(std::move(report)),
        debug_(std::move(debug)), wanted_(std::move(wanted)),
        primitives_(std::make_shared<const MovePrimitives>(BEND_RADIUS)),
        analytic_(*primitives_),
        scratch_(scene.router.width, scene.router.height),
        router_(primitives_, scratch_,
                {.startStraightLength = 0,
                 .endStraightLength = 0,
                 .minRadius = BEND_RADIUS,
                 .bendPenalty = tuning.bendPenalty}),
        field_(scene.router, tuning.clearance),
        corridor_(scene.router.width, scene.router.height),
        proximity_(scene.router.cells(), 0), seen_(scene.router.cells(), 0),
        bodies_(scene.router.width, scene.router.height),
        approaches_(scene.router.width, scene.router.height) {
    router_.attachObstacles(&scene_.blocked);
    // The room a coupler may sit in depends on the launchers alone, and the
    // outer routing already lays each resonator's meander inside it.
    couplerBox_ = couplerBoxOf();
    // The price of running beside an obstacle. It forbids nothing — the
    // keepout in the mask already does that — and keeps copper off the artwork
    // wherever there is room for it.
    router_.computeStaticProximity(tuning_.obstacleReach,
                                   tuning_.staticProximityPenalty);
    router_.attachWireProximity(&proximity_);
    // The distance field, not the octile distance. It costs a backward
    // Dijkstra over the corridor per search — 15 to 23 % of the time a
    // search takes — and it earns it back on a fenced corridor, where it
    // stops the search flooding and proves unreachable cells before they are
    // expanded. Measured both ways on the real chips: 17q 23.4 s with the
    // field against 41.3 s with octile, 9q 13.0 against 14.9, same angle
    // cost. On an *empty* grid the ranking reverses and the field is pure
    // overhead — which is why a synthetic benchmark says the opposite.
    router_.setHeuristic(routing::Heuristic::DistanceField);
  }

  [[nodiscard]] const Field& field() const { return field_; }
  [[nodiscard]] const Tuning& tuning() const { return tuning_; }

  /// Report one line of what the stage does, at the level of its steps.
  void say(const std::string& line) const { report_.line(Detail::Steps, line); }

  /// Report one line of detail: what one wire's search came to. Only at the
  /// level of the items, where a run says one line per search.
  void tell(const std::string& line) const {
    report_.line(Detail::Items, line);
  }

  /// Report one line of the coupler insertion's own reasoning: every option
  /// a coupler had, whether it survived being built and what the greedy
  /// priced it at. Only at the deepest level — it is 64 lines per coupler,
  /// which is one chip under a lens and not a run's log.
  void explain(const std::string& line) const {
    report_.line(Detail::Solver, line);
  }
  [[nodiscard]] bool explaining() const {
    return report_.wants(Detail::Solver);
  }

  /// Whether a page asks for records of `kind`.
  [[nodiscard]] bool wants(const std::string& kind) const {
    return debug_ && wanted_.contains(kind);
  }

  /// Hand `data` to the debug sink as a record of `kind`, when a page asks
  /// for that kind.
  void record(const std::string& kind, const nlohmann::json& data) const {
    if (wants(kind)) {
      debug_(kind, data.dump());
    }
  }

  /// The phase that runs from now on; the searches of a phase are recorded
  /// when its dashboard asks for them.
  void enterPhase(const std::string_view phase) { phase_ = phase; }

  /// The router grid, once: its size, the closed cells as runs per row, the
  /// ports as the chip carries them and every wire's two ends.
  void recordGrid(const std::vector<Wire>& wires) const {
    if (!wants("grid")) {
      return;
    }
    using nlohmann::json;
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto height = static_cast<std::int64_t>(scene_.router.height);
    json blocked = json::array();
    for (std::int64_t y = 0; y < height; ++y) {
      std::int64_t start = -1;
      for (std::int64_t x = 0; x <= width; ++x) {
        const bool shut =
            x < width &&
            scene_.blocked.test(static_cast<std::size_t>((y * width) + x));
        if (shut && start < 0) {
          start = x;
        } else if (!shut && start >= 0) {
          blocked.push_back({y, start, x - start});
          start = -1;
        }
      }
    }
    json ports = json::array();
    for (const auto& port : ports_) {
      ports.push_back({{"label", port.label}, {"x", port.x}, {"y", port.y}});
    }
    json ends = json::array();
    for (const auto& wire : wires) {
      if (!wire.feasible) {
        continue;
      }
      ends.push_back(
          {{"wire", wireId(wire)},
           {"inner", wire.inner},
           {"resonator", wire.resonator},
           {"source", {wire.objective.source.x, wire.objective.source.y}},
           {"target", {wire.objective.target.x, wire.objective.target.y}}});
    }
    record("grid", {{"width", width},
                    {"height", height},
                    {"cellSize", scene_.router.cellWidth},
                    {"clearance", tuning_.clearance},
                    {"blocked", std::move(blocked)},
                    {"ports", std::move(ports)},
                    {"wires", std::move(ends)}});
  }

  /// One search, right after it: the phase, the pass, the round, the wire,
  /// the kind of search, the wires let go of, what it found and the way as
  /// its bends. For a resonator the way carries its meander and the note
  /// says what the lengthening came to.
  void recordSearch(const Wire& wire, const Path& found,
                    const std::string& note) const {
    if (!wants(std::format("search:{}", phase_))) {
      return;
    }
    using nlohmann::json;
    json way = json::array();
    for (std::size_t at = 0; at < found.size(); ++at) {
      const bool bend = at == 0 || at + 1 == found.size() ||
                        found[at].heading != found[at - 1].heading;
      if (bend) {
        way.push_back({found[at].x, found[at].y});
      }
    }
    json ripped = json::array();
    for (const auto key : frame_.ripped) {
      if (frame_.wires != nullptr && key < frame_.wires->size()) {
        ripped.push_back(wireId((*frame_.wires)[key]));
      }
    }
    const nlohmann::json data{{"phase", phase_},
                              {"pass", frame_.pass},
                              {"round", frame_.round},
                              {"forward", frame_.forward},
                              {"wire", wireId(wire)},
                              {"kind", frame_.kind},
                              {"ripped", std::move(ripped)},
                              {"found", !found.empty()},
                              {"cells", found.size()},
                              {"note", note},
                              {"way", std::move(way)}};
    debug_("search", data.dump());
  }

  /// Every coupler with every option it may take, once, before the chain
  /// search: what `couplersJson` lists.
  void recordCouplerOptions(const std::vector<Wire>& wires) const {
    if (wants("coupler-options")) {
      record("coupler-options", couplersData(wires));
    }
  }

  /// What a phase came to: the wires it leaves drawn and the fails it
  /// leaves, counted as the end of the stage counts them.
  void recordPhase(const std::string_view phase, std::vector<Wire>& wires) {
    if (!wants("phase")) {
      return;
    }
    std::vector<std::uint32_t> every(wires.size());
    std::iota(every.begin(), every.end(), 0U);
    const auto fails = failsOf(wires, every);
    record("phase", {{"phase", phase},
                     {"wires", wires.size()},
                     {"drawn", fails.drawn},
                     {"unrouted", fails.unrouted},
                     {"open", fails.open},
                     {"short", fails.tooShort},
                     {"long", fails.tooLong},
                     {"crossing", fails.crossing},
                     {"loops", fails.loops},
                     {"bad", fails.broken}});
  }

  /// What the grid and the rules came to, once, before anything is drawn.
  void sayTheSetting() const {
    say(std::format(
        "grid {}x{} cells of {:.2f} layout units | clearance {} cells for a "
        "rule of {:.2f} | stub {} cells | band {} cells | bend {} | wire "
        "price {} | obstacle price {} over {} cells | {}",
        scene_.router.width, scene_.router.height,
        std::min(scene_.router.cellWidth, scene_.router.cellHeight),
        tuning_.clearance, tuning_.spacing, tuning_.straightStart,
        tuning_.reach, tuning_.bendPenalty, tuning_.wireProximityPenalty,
        tuning_.staticProximityPenalty, tuning_.obstacleReach,
        tuning_.meanderLength > 0.0
            ? std::format("resonators {:.0f} cells long", tuning_.meanderLength)
            : std::string("no meander")));
  }

  /// Whether a wire has to reach a length: a resonator, while the meander is
  /// switched on; every resonator once it is drawn from its coupler.
  /// Whether a wire is lengthened in this pass: a resonator that needs a
  /// length.
  [[nodiscard]] bool lengthensIn(const Wire& wire) const {
    return needsLength(wire);
  }

  [[nodiscard]] bool needsLength(const Wire& wire) const {
    return carriesLength(wire) && (exact_ || tuning_.meanderLength > 0.0);
  }

  /// Whether a wire is the part of a resonator that carries its length: a
  /// resonator that ends on its qubit, or the inner wire beyond the bridge
  /// of one that does not.
  [[nodiscard]] static bool carriesLength(const Wire& wire) {
    return (wire.resonator && !wire.feedsABridge) || wire.resonatorBeyondBridge;
  }

  /// How long a resonator's way has to be, in cells: `meander_length` in the
  /// outer routing and the target length once the coupler is in, each less
  /// the run from the cell the way ends on to the port itself, which the
  /// wire covers without cells of its own.
  ///
  /// In the outer routing the coupler is still to come, and it is spliced
  /// where what is left to the qubit is `leftAtCoupler`, inside the coupler
  /// box (`couplerPlace`). A way that carries its length outside the box is
  /// cut there short of it. So the way has to carry what the coupler leaves
  /// from where it enters the box (`couplerEntry`), and whatever runs before
  /// that comes on top.
  [[nodiscard]] double requiredLength(const Wire& wire, const Path& way) const {
    const auto figure = exact_ ? tuning_.targetLength : tuning_.meanderLength;
    const auto whole = std::max(0.0, figure - wire.anchorGap);
    if (exact_ || !wire.resonator || wire.feedsABridge) {
      return whole;
    }
    const auto entry = couplerEntry(way);
    if (!entry.has_value()) {
      return whole;
    }
    return std::max(whole, entry->lengthAt + leftAtCoupler(wire));
  }

  /// What the way of a resonator measures from its coupler to the qubit once
  /// the coupler is in, in cells: `target_resonator_length` times the bias,
  /// less the run to the port and the lead the component leaves the pad on.
  [[nodiscard]] double leftAtCoupler(const Wire& resonator) const {
    const double target =
        std::max(0.0, tuning_.targetLength - resonator.anchorGap);
    // The component's own lead runs in front of what is left, so the place
    // is the point where the lead **and** the rest together come to the
    // figure. Without this the way left is the target and the lead — a
    // quarter turn and fourteen cells, about 290 layout units — sits on top
    // of it: on 21q every resonator came out at 2746 to 2805 against a
    // target of 2500, over by exactly the lead, and nothing downstream
    // shortens a way.
    return std::max(0.0, (target * tuning_.couplerBias) - leadLength());
  }

  /// Where a way first comes to a cell a coupler may be centred on, and the
  /// rendered length of the way up to there, in cells.
  struct CouplerEntry {
    std::size_t index = 0;
    double lengthAt = 0.0;
  };

  /// The first cell of a way that lies far enough inside the coupler box
  /// for a coupler centred on it to keep its ports, their runs and its body
  /// inside, in any orientation: `coupler_box_margin_cells`, half the pad's
  /// run and the run off a port along it, half its depth across.
  [[nodiscard]] std::optional<CouplerEntry>
  couplerEntry(const Path& way) const {
    // A box narrower than one resonator's length has no room for a meander
    // behind the entry, and on such a chip the meander goes where it fits.
    if (way.size() < 2 || couplerBox_.empty() ||
        static_cast<double>(std::min(couplerBox_.maxX - couplerBox_.minX,
                                     couplerBox_.maxY - couplerBox_.minY)) <
            tuning_.targetLength) {
      return std::nullopt;
    }
    const auto inset =
        tuning_.couplerBoxMargin +
        static_cast<std::int64_t>(cellsOn(tuning_.couplerLength, 0) / 2) +
        edgeRunOffPort() +
        static_cast<std::int64_t>(cellsOn(tuning_.couplerHeight, 0) / 2) + 1;
    Path copy = way;
    std::vector<routing::PathSegment> segments;
    std::ignore =
        routing::samplePath(*primitives_, copy, copy.front(), segments);
    std::size_t cursor = 0;
    for (const auto& segment : segments) {
      for (std::size_t i = 0; i < segment.cells.size(); ++i) {
        const auto& cell = segment.cells[i];
        while (cursor < way.size() && !(way[cursor] == cell)) {
          ++cursor;
        }
        if (cursor >= way.size()) {
          return std::nullopt;
        }
        if (couplerBox_.holds(cell.x, cell.y, inset)) {
          return CouplerEntry{.index = cursor, .lengthAt = segment.lengthAt[i]};
        }
      }
    }
    return std::nullopt;
  }

  /// The figure a resonator is measured against, in layout units, and the
  /// tolerance around it: none in the outer routing, the rule's once the
  /// coupler is in.
  [[nodiscard]] double requiredUnits() const {
    return exact_ ? tuning_.targetLengthUnits : tuning_.meanderLengthUnits;
  }
  [[nodiscard]] double toleranceUnits() const {
    return exact_ ? tuning_.lengthToleranceUnits : 0.0;
  }

  /// Switch the length a resonator is made to: at least `meander_length`
  /// before the coupler, the target length exactly after it.
  void aimAtTarget(const bool exact) { exact_ = exact; }

  /// The length of a way as the sampler renders it, in cells.
  [[nodiscard]] double lengthOf(const Path& way) const {
    return routing::renderedLength(*primitives_, way);
  }

  /// The same in layout units, with the two sides of a cell told apart.
  [[nodiscard]] double lengthInUnits(const Path& way) const {
    if (way.empty()) {
      return 0.0;
    }
    Path copy = way;
    std::vector<routing::PathSegment> segments;
    const auto points =
        routing::samplePath(*primitives_, copy, copy.front(), segments);
    double total = 0.0;
    for (std::size_t at = 1; at < points.size(); ++at) {
      total += std::hypot(
          (points[at].x() - points[at - 1].x()) * scene_.router.cellWidth,
          (points[at].y() - points[at - 1].y()) * scene_.router.cellHeight);
    }
    return total;
  }

  /// What lengthening a way came to, and the words for it.
  struct Lengthened {
    bool reached = false;
    bool tooLong = false;
    std::string note;
  };

  /// Lengthen a resonator's way to what it needs: the prototype's meander
  /// insertion, the cheapest placement by the search's own price field and
  /// `MEANDER_BEND_PRICE` per direction change, as the prototype's strict
  /// insertion takes it. The meander may enter what the search could
  /// enter — the band less the fence — and nothing else, so it cannot cross
  /// a neighbour the way itself could not.
  /// @param band How wide the length band is, as a multiple of
  ///   `resonator_length_tolerance`: one in the refinement, which aims at
  ///   the length itself, and `sweepLengthBand` in the sweep, which aims
  ///   roughly. See `sweepLengthBand`.
  [[nodiscard]] Lengthened lengthen(const Wire& wire, Path& way,
                                    const double band = 1.0) {
    const auto required = requiredLength(wire, way);
    routing::MeanderOptions options{.width = scene_.router.width,
                                    .height = scene_.router.height,
                                    .box = {.minX = 0,
                                            .maxX = scene_.router.width - 1,
                                            .minY = 0,
                                            .maxY = scene_.router.height - 1},
                                    .minStraightLength =
                                        tuning_.meanderLegSpacing};
    // Before the coupler is in, the meander goes where the coupler keeps
    // it: inside the coupler box and behind the cell the way enters it on,
    // for the coupler is spliced there and what lies before it is cut off.
    // See `requiredLength`.
    const auto entry = exact_ || !wire.resonator || wire.feedsABridge
                           ? std::optional<CouplerEntry>{}
                           : couplerEntry(way);
    // The entry cell itself stays where it is, so that the way with its
    // meander enters the box on the same cell and is measured against the
    // same length.
    if (entry.has_value()) {
      options.keepBefore = entry->index + 1;
    }
    if (exact_) {
      // The prototype's strict insertion: the length exactly, within the
      // rule's tolerance, and never in the lead the resonator leaves its
      // coupler on.
      //
      // The prototype also holds each pair to the widest free strip along
      // it, because its depth comes from that strip's height. Here every
      // cell of a meander is tested against the corridor and the spliced
      // way against itself, and the strip only took room away: it ends at
      // the two cells of the pair, so a meander could not reach past them.
      options.exact = true;
      options.tolerance = band * tuning_.lengthTolerance;
      options.startMargin = tuning_.meanderStartMargin;
    }
    // Before the coupler is in, a resonator's meander stays inside the
    // coupler box when its way reaches the box far enough for a coupler.
    // A way that never does leaves the coupler no place inside the box in
    // any case, and a chip whose box is that small keeps its meanders
    // where they fit.
    const bool inBox = entry.has_value();
    const auto enterable = [this, inBox](const std::uint32_t x,
                                         const std::uint32_t y) {
      if (inBox && !couplerBox_.holds(x, y, tuning_.couplerBoxMargin)) {
        return false;
      }
      return x < scene_.router.width && y < scene_.router.height &&
             !corridor_.testCell(x, y);
    };
    options.bendPrice = MEANDER_BEND_PRICE;
    const routing::CellPrice price = [this](const std::uint32_t x,
                                            const std::uint32_t y) {
      const auto cell = (static_cast<std::size_t>(y) * scene_.router.width) + x;
      return static_cast<std::uint32_t>(router_.staticPenalty(cell)) +
             proximity_[cell];
    };
    const auto made = routing::insertMeander(*primitives_, way, required,
                                             enterable, options, price);
    if (made.inserted) {
      auto& kept = meanders_[wire.key];
      kept.emplace_back(way, made.meander);
      if (kept.size() > 4) {
        kept.erase(kept.begin());
      }
    }
    Lengthened result{.reached = made.reached, .tooLong = made.tooLong};
    if (made.tooLong) {
      result.note =
          std::format("too long for its target: {:.0f} of {:.0f} cells",
                      made.lengthBefore, required);
    } else if (!made.reached) {
      // What stood in the way, not only how many placements were counted: a
      // loop refused for want of room is a different problem from one no
      // pair of the way lies far enough apart to carry. See
      // `MeanderResult::Refusals`.
      std::string said;
      for (const auto& [reason, times] : made.refusals.said()) {
        said += std::format("{}{} {}", said.empty() ? "" : ", ", times, reason);
      }
      result.note = std::format(
          "no room for a meander: {:.0f} of {:.0f} cells, {} placements "
          "tried over {} straight cells less {} reserved, widest span {} of "
          "the {} a loop needs — {}",
          made.lengthBefore, required, made.candidates, made.straightCells,
          made.reserved, made.widestSpan,
          options.minStraightLength + (2U * primitives_->minRadius()), said);
    } else if (made.inserted) {
      result.note = std::format("meander: {:.0f} → {:.0f} cells for {:.0f}",
                                made.lengthBefore, made.lengthAfter, required);
    } else {
      result.note = std::format("long enough: {:.0f} of {:.0f} cells",
                                made.lengthBefore, required);
    }
    return result;
  }

  /// Put a wire's way down: its copper, and the room it keeps around it. The
  /// way may be one this stage drew or the seed the wire started on; both
  /// hold copper, because both are where the wire runs.
  void place(Wire& wire) {
    if (wire.way.empty()) {
      return;
    }
    if (wire.endsOnly) {
      field_.dischargeFixed(wire.fixed);
      wire.endsOnly = false;
    }
    if (!wire.placed) {
      field_.occupy(wire.way, wire.key);
      field_.charge(wire.way);
      wire.placed = true;
    }
  }

  /// Take a wire off the canvas altogether, for the length of its own search.
  /// Only the wire being drawn is let out of its own two places.
  void lift(Wire& wire) {
    if (wire.placed) {
      field_.discharge(wire.way);
      wire.placed = false;
    } else if (wire.endsOnly) {
      field_.dischargeFixed(wire.fixed);
    }
    wire.endsOnly = false;
    if (!wire.way.empty()) {
      field_.vacate(wire.way, wire.key);
    }
  }

  /// Sweep the wires, drawing each one again from the way it has.
  ///
  /// The sweep is a rip-up and re-route, and a rip-up needs something to rip:
  /// every wire of the pass starts on the way the Detail stage drew, put down
  /// with its copper and its room before the first search, and a wire is
  /// taken off the canvas, offered a way of its own, and put back on the way
  /// it had when it finds none. So no wire is ever without a way, and what a
  /// round sees of the wires it has not reached yet is where they run rather
  /// than empty space. The prototype starts the same way: its `global_paths`
  /// begin as the detailed routing's paths, and a wire that fails keeps its
  /// entry.
  ///
  /// Rounds alternate direction. A wire that has been routed is left alone.
  /// A round ends when every wire has been routed, not when no attempt
  /// failed. The two are not the same: a wire routed early in a round can be
  /// let go of by a wire further along, and letting it go is a promise that it
  /// will be drawn again. Stopping on the failure count alone leaves it on the
  /// canvas with its room uncharged, and the next wire that comes past settles
  /// inside it.
  ///
  /// What a round reports, and what the pass ends on, is its fails: the
  /// wires still on their seed, for which this stage has found no way; the
  /// wires with a way of their own that is not settled against every other
  /// wire; and the resonators whose way is too short for want of room for
  /// their meander. The first are unrouted, the second open, the third
  /// short, and all three count.
  ///
  /// @returns How many wires are unrouted, open or short when the pass ends.
  std::uint32_t sweep(std::vector<Wire>& wires,
                      const std::vector<std::uint32_t>& members,
                      const Pass& pass) {
    const auto total = static_cast<std::uint32_t>(members.size());
    if (total == 0) {
      // A pass with nothing to draw still ends on its summary, so that every
      // pass of every run reads the same way.
      sayFails(pass.name, {}, 0);
      return 0;
    }
    fixPlaces(wires, members, pass);
    seed(wires, members, pass);
    beginPass(wires, members, pass);
    rounds_.clear();
    bridgeRefusals_ = 0;
    auto fewest = std::numeric_limits<std::uint32_t>::max();
    std::uint32_t stale = 0;
    say(std::format("{}: {} wires, up to {} rounds, {} relaxations each way",
                    pass.name, total, pass.rounds, pass.maxRelaxation));
    ++sweepSerial_;
    for (std::uint32_t round = 0; round < pass.rounds; ++round) {
      const bool forward = (round % 2) == 0;
      const auto roundName = std::format("round {} {}", round + 1,
                                         forward ? "forward" : "backward");
      frame_.pass = passTag(pass.name);
      frame_.round = round;
      frame_.forward = forward;
      rounds_.push_back({.forward = forward});
      std::uint32_t tried = 0;
      std::uint32_t won = 0;
      for (std::uint32_t at = 0; at < total; ++at) {
        report_.progress(pass.name, roundName, at, total);
        const auto slot = forward ? at : (total - 1 - at);
        auto& wire = wires[members[slot]];
        if (!wire.feasible || (wire.routed && pass.keepDrawn)) {
          continue;
        }
        ++tried;
        won += attempt(wires, members, slot, forward, pass) ? 1 : 0;
      }
      // The ways the round leaves, for a picture of it (`roundsJson`); only
      // the last two rounds keep theirs.
      for (const auto member : members) {
        rounds_.back().ways.emplace_back(wireId(wires[member]),
                                         wires[member].way);
      }
      if (rounds_.size() > 2) {
        rounds_[rounds_.size() - 3].ways.clear();
      }
      // Unrouted is a wire with no way of its own yet; open is a wire whose
      // own way is not settled — let go of by a wire that relaxed past it and
      // not drawn again yet; short is a resonator that kept a way without
      // room for its meander. All three are fails, and a round with none
      // ends the pass.
      std::uint32_t unrouted = 0;
      std::uint32_t open = 0;
      std::uint32_t tooShort = 0;
      for (const auto member : members) {
        const auto& wire = wires[member];
        if (!wire.feasible) {
          continue;
        }
        unrouted += wire.drawn ? 0 : 1;
        tooShort += (wire.drawn && (wire.tooShort || wire.tooLong)) ? 1 : 0;
        open += (wire.drawn && !wire.tooShort && !wire.tooLong && !wire.routed)
                    ? 1
                    : 0;
      }
      const auto fails = unrouted + open + tooShort;
      report_.progress(pass.name, roundName, total, total);
      report_.entry(Detail::Steps, roundName,
                    {{.text = std::format("tried {}", tried)},
                     {.text = std::format("routed {}", won)},
                     verdict("unrouted", unrouted),
                     verdict("open", open),
                     verdict("short", tooShort)});
      if (fails == 0) {
        break;
      }
      // A round that leaves as many fails as the best round before it has
      // moved nothing that the next round can use. The rounds are there for
      // a wire to take room a later one has not claimed yet, and once that
      // has stopped happening they only cost searches: measured on the
      // 21-qubit chip, the count settles by the second round and the
      // remaining twenty-eight change no byte of the result.
      stale = fails < fewest ? 0 : stale + 1;
      fewest = std::min(fewest, fails);
      if (stale >= STALE_ROUNDS) {
        say(std::format("{}: {} rounds without progress, stopping", pass.name,
                        stale));
        break;
      }
    }

    // Everything goes back down, so that the field says what the canvas holds
    // and the next phase starts from the truth.
    for (const auto member : members) {
      place(wires[member]);
    }
    const auto fails = failsOf(wires, members);
    sayFails(pass.name, fails, total);
    if (pass.feedlines) {
      say(std::format("{}: {} way{} refused for not crossing the wire's "
                      "bridge",
                      pass.name, bridgeRefusals_,
                      bridgeRefusals_ == 1 ? "" : "s"));
    }
    // The second level of verbosity ends the pass on what every round came
    // to, wire by wire: how many found a way in phase 1, how many only after
    // relaxation, and which found none.
    if (report_.wants(Detail::Items)) {
      for (std::size_t round = 0; round < rounds_.size(); ++round) {
        const auto& record = rounds_[round];
        std::string failed;
        for (const auto& id : record.failed) {
          failed += (failed.empty() ? " " : ", ") + id;
        }
        say(std::format(
            "{} round {} {}: {} found a way in phase 1, {} after "
            "relaxation, {} failed{}{}",
            pass.name, round + 1, record.forward ? "forward" : "backward",
            record.normal, record.relaxed, record.failed.size(),
            record.failed.empty() ? "" : ":" + failed,
            record.tooShort == 0 ? ""
                                 : std::format(", {} kept a way too short for "
                                               "its meander",
                                               record.tooShort)));
      }
    }
    return fails.total();
  }

  /// What a set of wires comes to, counted with every wire down.
  struct Fails {
    /// Wires with a way of their own.
    std::uint32_t drawn = 0;
    /// Wires without one: still on their seed, or not on the grid at all.
    std::uint32_t unrouted = 0;
    /// Wires whose own way comes within the rule of another wire.
    std::uint32_t open = 0;
    /// Resonators whose way is shorter than the target length asks, or than
    /// the target length allows once the coupler is in.
    std::uint32_t tooShort = 0;
    /// Resonators whose way is longer than the target length allows, once
    /// the coupler is in.
    std::uint32_t tooLong = 0;
    /// Wires that cross a feedline other than at a right angle, or enter
    /// the room around a feedline's bend.
    std::uint32_t crossing = 0;
    /// Wires that cross or touch themselves: a resonator whose kept way
    /// runs back over the head its coupler put in front of it.
    std::uint32_t loops = 0;
    /// Of the unrouted, the edges of the feedline chains.
    std::uint32_t feedlinesUnrouted = 0;
    /// Wires with any of the above, each counted once.
    std::uint32_t failing = 0;
    /// Wires with no usable way: unrouted, open, crossing a feedline or
    /// meeting itself, each counted once.
    std::uint32_t broken = 0;
    /// Which ones, for the second level of verbosity.
    std::vector<std::string> unroutedIds;
    std::vector<std::string> openIds;
    std::vector<std::string> shortIds;
    std::vector<std::string> longIds;
    std::vector<std::string> crossingIds;
    std::vector<std::string> loopIds;
    /// The same three the targeted repair reads, by wire key rather than by
    /// name, so that a fail can be looked up without parsing its id.
    std::vector<std::uint32_t> unroutedKeys;
    std::vector<std::uint32_t> openKeys;
    std::vector<std::uint32_t> crossingKeys;
    /// The verdict against every wire of the list, by key, as the bits of
    /// `FinalVerdict`: zero where nothing is held against a wire, and for
    /// every wire outside the members counted. This is what the artifact
    /// carries on its end state, so a picture can mark the wires the stage
    /// left failing by the stage's own count.
    std::vector<std::uint8_t> verdicts;
    [[nodiscard]] std::uint32_t total() const { return failing; }
    /// The figure the targeted repair is judged by: unrouted, open and
    /// crossing, the lengths left out (user, 2026-10-04).
    [[nodiscard]] std::uint32_t bad() const {
      return unrouted + open + crossing;
    }
  };

  /// Count the fails of a set of wires, by the same test the design-rule
  /// check makes and with every wire down — so what it says is what the
  /// check will find, not what the sweep happened to leave open. Each wire
  /// is counted with itself lifted off the canvas, because a wire is always
  /// within the rule of itself. A resonator's length is measured here as
  /// well, by the sampler that measures it for the artifact, so that a way
  /// too short is a fail whatever the sweep believed about it.
  [[nodiscard]] Fails failsOf(std::vector<Wire>& wires,
                              const std::vector<std::uint32_t>& members) {
    Fails fails;
    fails.verdicts.assign(wires.size(), 0);
    const auto hold = [&fails](const Wire& wire, const fba::FinalVerdict bit) {
      if (wire.key < fails.verdicts.size()) {
        fails.verdicts[wire.key] |= static_cast<std::uint8_t>(bit);
      }
    };
    for (const auto member : members) {
      auto& wire = wires[member];
      if (!wire.drawn) {
        ++fails.unrouted;
        ++fails.failing;
        ++fails.broken;
        fails.feedlinesUnrouted += wire.feedline ? 1 : 0;
        fails.unroutedIds.push_back(wireId(wire));
        fails.unroutedKeys.push_back(wire.key);
        hold(wire, fba::FinalVerdict::Unrouted);
        continue;
      }
      ++fails.drawn;
      lift(wire);
      const bool open =
          conflictsOf(wire, wires) != 0 || crossesItsChain(wire, wires);
      if (open && report_.wants(Detail::Items)) {
        say(std::format("  {} is open in the room of {}", wireId(wire),
                        whoBlocks(wire, wires)));
      }
      place(wire);
      const auto length = needsLength(wire) ? lengthOf(wire.way) : 0.0;
      const auto required = requiredLength(wire, wire.way);
      const auto tolerance = exact_ ? tuning_.lengthTolerance : 0.0;
      wire.tooShort = needsLength(wire) && length < required - tolerance;
      wire.tooLong =
          needsLength(wire) && exact_ && length > required + tolerance;
      const bool crossing =
          feedlinePass_ &&
          (crossesAFeedline(wire, wires) ||
           breaksAFeedlineRule(wire, wire.way,
                               startStubOf(wire, tuning_.straightStart),
                               wires));
      if (crossing && report_.wants(Detail::Items)) {
        say(std::format("  {} crosses at {}", wireId(wire),
                        whereItCrosses(wire, wires)));
      }
      // A way that meets itself is rule 3 of the check; a search never
      // returns one, but a resonator that keeps the way its coupler cut
      // may hold one, and the count says so.
      const bool loop = routing::pathSelfIntersects(
          wire.way, scene_.router.width, scene_.router.height, loopScratch_);
      if (open) {
        ++fails.open;
        fails.openIds.push_back(wireId(wire));
        fails.openKeys.push_back(wire.key);
        hold(wire, fba::FinalVerdict::Open);
      }
      if (wire.tooShort) {
        ++fails.tooShort;
        fails.shortIds.push_back(wireId(wire));
        hold(wire, fba::FinalVerdict::Short);
      }
      if (wire.tooLong) {
        ++fails.tooLong;
        fails.longIds.push_back(wireId(wire));
        hold(wire, fba::FinalVerdict::Long);
      }
      if (crossing) {
        ++fails.crossing;
        fails.crossingIds.push_back(wireId(wire));
        fails.crossingKeys.push_back(wire.key);
        hold(wire, fba::FinalVerdict::Crossing);
      }
      if (loop) {
        ++fails.loops;
        fails.loopIds.push_back(wireId(wire));
        hold(wire, fba::FinalVerdict::Loop);
      }
      if (open || wire.tooShort || wire.tooLong || crossing || loop) {
        ++fails.failing;
      }
      if (open || crossing || loop) {
        ++fails.broken;
      }
    }
    return fails;
  }

  /// Whether a wire crosses a feedline other than at a right angle, by the
  /// test the search runs on every move: the constraints of every edge
  /// between two couplers, with the cells around the wire's own coupler
  /// left out, because the edges of its own chain pin there on purpose.
  [[nodiscard]] bool crossesAFeedline(const Wire& wire,
                                      const std::vector<Wire>& wires) const {
    (void)wires;
    return crossingCellOf(wire).has_value();
  }

  /// The first cell of a wire's way that breaks the crossing rule, outside
  /// the reach of its own coupler; nothing when the way holds the rule, when
  /// the rule is off, or when the wire is a feedline edge, which the rule is
  /// built from and never binds.
  [[nodiscard]] std::optional<PathPoint>
  crossingCellOf(const Wire& wire) const {
    if (wire.feedline || !wire.drawn) {
      return std::nullopt;
    }
    const auto& constraints = router_.crossingConstraints();
    if (constraints.empty()) {
      return std::nullopt;
    }
    const auto own =
        wire.couplerAtSource != NO_OWNER
            ? std::optional<PathPoint>(couplers_[wire.couplerAtSource].anchor)
            : std::nullopt;
    for (std::size_t at = 0; at < wire.way.size(); ++at) {
      const auto& point = wire.way[at];
      if (own.has_value() &&
          std::hypot(static_cast<double>(point.x) - own->x,
                     static_cast<double>(point.y) - own->y) <= couplerReach()) {
        continue;
      }
      // On the heading the way arrives with as well, which is what the
      // search tested: see `CrossingConstraints::allowedArriving`.
      const auto arrived = at == 0 ? point.heading : wire.way[at - 1].heading;
      if (!constraints.allowedArriving(point.x, point.y, point.heading,
                                       arrived)) {
        return point;
      }
    }
    return std::nullopt;
  }

  /// Where a wire first breaks the crossing rule, and which coupler stands
  /// nearest that cell — the question being whether the breaks gather at
  /// couplers of one kind.
  [[nodiscard]] std::string
  whereItCrosses(const Wire& wire, const std::vector<Wire>& wires) const {
    const auto& constraints = router_.crossingConstraints();
    const auto own =
        wire.couplerAtSource != NO_OWNER
            ? std::optional<PathPoint>(couplers_[wire.couplerAtSource].anchor)
            : std::nullopt;
    for (std::size_t at = 0; at < wire.way.size(); ++at) {
      const auto& point = wire.way[at];
      if (own.has_value() &&
          std::hypot(static_cast<double>(point.x) - own->x,
                     static_cast<double>(point.y) - own->y) <= couplerReach()) {
        continue;
      }
      const auto arrived = at == 0 ? point.heading : wire.way[at - 1].heading;
      if (constraints.allowedArriving(point.x, point.y, point.heading,
                                      arrived)) {
        continue;
      }
      // Which edge it is crossing: the drawn chain edge with a cell nearest
      // the offending one.
      const auto [nearest, span] = nearestEdgeTo(wires, point);
      if (nearest == edges_.size()) {
        return std::format("({},{})", point.x, point.y);
      }
      // The couplers at that edge's two ends, and how each stands.
      const auto& chain = chains_[edges_[nearest].chain];
      const auto describe = [this](const Waypoint& point) {
        if (point.coupler == NO_OWNER) {
          return std::string("launcher");
        }
        const auto& option =
            couplers_[point.coupler].options[couplers_[point.coupler].chosen];
        return std::format("coupler {} ({})", point.coupler,
                           (!option.way.empty() &&
                            routing::isDiagonal(option.way.front().heading))
                               ? "diagonal"
                               : "axial");
      };
      return std::format(
          "({},{}), {:.0f} cells from edge {}, which runs {} -> {}", point.x,
          point.y, span, wireId(wires[edges_[nearest].wire]),
          describe(chain[edges_[nearest].from]),
          describe(chain[edges_[nearest].to]));
    }
    return "nowhere";
  }

  /// How far a cell lies from a drawn chain edge: the least distance from
  /// the cell to any cell of the edge's way. Brute force over the way —
  /// a few hundred cells — which is what every caller can afford.
  [[nodiscard]] static double distanceToWay(const PathPoint& cell,
                                            const Path& way) {
    double least = std::numeric_limits<double>::max();
    for (const auto& at : way) {
      least = std::min(least, std::hypot(static_cast<double>(cell.x) - at.x,
                                         static_cast<double>(cell.y) - at.y));
    }
    return least;
  }

  /// The drawn chain edge with a cell nearest a cell, by edge index, and how
  /// far that cell is; `edges_.size()` when no edge is drawn. The first edge
  /// in edge order wins a tie, as the loop this was factored out of had it.
  [[nodiscard]] std::pair<std::size_t, double>
  nearestEdgeTo(const std::vector<Wire>& wires, const PathPoint& cell) const {
    std::size_t nearest = edges_.size();
    double span = std::numeric_limits<double>::max();
    for (std::size_t at = 0; at < edges_.size(); ++at) {
      const auto& edge = wires[edges_[at].wire];
      if (!edge.drawn) {
        continue;
      }
      const auto far = distanceToWay(cell, edge.way);
      if (far < span) {
        span = far;
        nearest = at;
      }
    }
    return {nearest, span};
  }

  /// Every drawn chain edge whose way comes within `reach` cells of a cell,
  /// by edge index with its distance, nearest first. Brute force over
  /// `edges_` × way — some 25 000 distances a query on 45q — which is
  /// nothing next to one search.
  [[nodiscard]] std::vector<std::pair<std::size_t, double>>
  edgesNear(const std::vector<Wire>& wires, const PathPoint& cell,
            const double reach) const {
    std::vector<std::pair<std::size_t, double>> near;
    for (std::size_t at = 0; at < edges_.size(); ++at) {
      const auto& edge = wires[edges_[at].wire];
      if (!edge.drawn) {
        continue;
      }
      const auto far = distanceToWay(cell, edge.way);
      if (far <= reach) {
        near.emplace_back(at, far);
      }
    }
    std::ranges::stable_sort(near, {},
                             [](const auto& entry) { return entry.second; });
    return near;
  }

  /// Which switches the outer routing runs under, said once before it.
  void sayOuterSettings() const {
    say(std::format(
        "outer routing settings: length-point clearance k {} cells, band "
        "±{:.0f}% of the target length; hard — a wire that cannot be routed "
        "with it fails, recovered without it at the end",
        tuning_.lengthPointRadius, tuning_.lengthPointBand * 100.0));
  }

  /// The recovery at the end of the outer routing: the wires the rounds left
  /// unrouted or open, searched once more in a pass over the ring with the
  /// length-point bands off. `beginPass` clears `routed` on no member of an
  /// outer pass and the sweep skips a routed member under `keepDrawn`, so
  /// only the failing wires are drawn; every other wire stands as the fence
  /// of the ones redrawn. Nothing happens when nothing failed.
  void recoverWithoutBands(std::vector<Wire>& wires,
                           const std::vector<std::uint32_t>& members,
                           const Pass& pass) {
    if (tuning_.lengthPointRadius == 0) {
      return;
    }
    std::uint32_t failing = 0;
    for (const auto member : members) {
      const auto& wire = wires[member];
      failing += (wire.feasible && (!wire.drawn || !wire.routed)) ? 1 : 0;
    }
    if (failing == 0) {
      say(std::format("{}: nothing to recover without the length-point "
                      "clearance",
                      pass.name));
      return;
    }
    Pass recovery = pass;
    recovery.name = "outer recovery";
    lengthPointActive_ = false;
    const auto left = sweep(wires, members, recovery);
    lengthPointActive_ = true;
    say(std::format("{}: recovery without the length-point clearance: {} "
                    "wire{} failing before it, {} after",
                    pass.name, failing, failing == 1 ? "" : "s", left));
  }

  /// The length-point report, the prototype's `[LENPT]` line: for every
  /// resonator the nearest other wire of the ring to its length point, the
  /// same-port siblings left out as the stamp leaves them out, capped at a
  /// hundred cells; the least, the mean, how many lie below `k`, and a
  /// histogram at fixed edges so that arms at different `k` and the `k = 0`
  /// baseline compare directly. One `tell` per resonator at `-v 1`.
  void sayLengthPoints(const std::vector<Wire>& wires,
                       const std::vector<std::uint32_t>& members) const {
    constexpr double CAP = 100.0;
    const auto k = tuning_.lengthPointRadius;
    const double guard = std::max(static_cast<double>(k) + 2.0,
                                  1.5 * static_cast<double>(tuning_.clearance));
    struct Hit {
      std::uint32_t wire = 0;
      double nearest = CAP;
      std::string names;
    };
    std::vector<Hit> hits;
    std::uint32_t unknown = 0;
    for (const auto member : members) {
      const auto& wire = wires[member];
      if (!wire.resonator || !wire.feasible) {
        continue;
      }
      const auto marks = lengthMarksOf(wire);
      if (!marks.valid) {
        ++unknown;
        continue;
      }
      const auto& point = wire.way[marks.pt];
      std::vector<std::pair<double, std::uint32_t>> near;
      for (const auto other : members) {
        if (other == member || wires[other].way.empty()) {
          continue;
        }
        const auto& o = wires[other];
        bool sibling = false;
        for (const auto* const end :
             {&o.objective.source, &o.objective.target}) {
          sibling = sibling ||
                    std::hypot(static_cast<double>(point.x) - end->x,
                               static_cast<double>(point.y) - end->y) <= guard;
        }
        if (sibling) {
          continue;
        }
        const auto far = distanceToWay(point, o.way);
        if (far < CAP) {
          near.emplace_back(far, other);
        }
      }
      std::ranges::sort(near);
      Hit hit{.wire = member,
              .nearest = near.empty() ? CAP : near.front().first,
              .names = {}};
      for (std::size_t n = 0; n < near.size() && n < 3; ++n) {
        hit.names += std::format("{}{} at {:.1f}", n == 0 ? "" : ", ",
                                 wireId(wires[near[n].second]), near[n].first);
      }
      if (report_.wants(Detail::Items)) {
        tell(std::format(
            "length point of {} at ({},{}), band {} cells, "
            "nearest: {}",
            wireId(wire), point.x, point.y, marks.hi - marks.lo + 1,
            hit.names.empty() ? std::string("none within 100") : hit.names));
      }
      hits.push_back(std::move(hit));
    }
    double sum = 0.0;
    double least = CAP;
    std::uint32_t below = 0;
    const std::array<double, 6> edges{20.0, 25.0, 30.0, 40.0, 60.0, 100.0};
    std::array<std::uint32_t, 7> buckets{};
    for (const auto& hit : hits) {
      sum += hit.nearest;
      least = std::min(least, hit.nearest);
      below += hit.nearest < static_cast<double>(k) ? 1 : 0;
      std::size_t b = 0;
      while (b < edges.size() && hit.nearest >= edges[b]) {
        ++b;
      }
      ++buckets[b];
    }
    say(std::format(
        "outer routing: LENPT k={} band=±{:.0f}%: {} length points ({} "
        "unknown), nearest other wire min {:.1f} mean {:.1f} cells, {} below "
        "k; histogram <20:{} 20-25:{} 25-30:{} 30-40:{} 40-60:{} 60-100:{} "
        ">=100:{}",
        k, tuning_.lengthPointBand * 100.0, hits.size(), unknown,
        hits.empty() ? -1.0 : least,
        hits.empty() ? -1.0 : sum / static_cast<double>(hits.size()), below,
        buckets[0], buckets[1], buckets[2], buckets[3], buckets[4], buckets[5],
        buckets[6]));
  }

  /// Say how long every resonator is, in layout units: its way as the
  /// sampler renders it, the run from the way's last cell to the port, and
  /// the two together against `meander_length`. One line per resonator, in
  /// the order of the ring.
  void sayResonatorLengths(const std::vector<Wire>& wires,
                           const std::vector<std::uint32_t>& members) const {
    if (tuning_.meanderLength <= 0.0) {
      return;
    }
    std::uint32_t resonators = 0;
    std::uint32_t short_ = 0;
    double shortest = std::numeric_limits<double>::max();
    double longest = 0.0;
    for (const auto member : members) {
      const auto& wire = wires[member];
      if (!carriesLength(wire) || !wire.feasible) {
        continue;
      }
      ++resonators;
      const auto label = wire.targetPort < ports_.size()
                             ? ports_[wire.targetPort].label
                             : std::string("?");
      if (!wire.drawn) {
        ++short_;
        say(std::format("resonator {} to {}: not drawn", wireId(wire), label));
        continue;
      }
      const auto way = lengthInUnits(wire.way);
      const auto total = way + wire.anchorGapUnits;
      const auto figure = requiredUnits();
      const auto tolerance = toleranceUnits();
      const bool enough = total >= figure - tolerance;
      const bool over = exact_ && total > figure + tolerance;
      short_ += (enough && !over) ? 0 : 1;
      shortest = std::min(shortest, total);
      longest = std::max(longest, total);
      say(std::format("resonator {} to {}: {:.0f} units of way + {:.0f} to "
                      "the port = {:.0f} units, {} {:.0f}{}",
                      wireId(wire), label, way, wire.anchorGapUnits, total,
                      over ? "over" : (enough ? "meets" : "short of"), figure,
                      exact_ ? std::format(" within {:.0f}", tolerance) : ""));
    }
    if (resonators == 0) {
      return;
    }
    say(std::format(
        "resonators: {} in all, {:.0f} to {:.0f} units, {} off "
        "{:.0f}{}",
        resonators, shortest, longest, short_, requiredUnits(),
        exact_ ? std::format(" by more than {:.0f}", toleranceUnits()) : ""));
  }

  /// The figure of how many wires have a way.
  [[nodiscard]] static Figure drawnFigure(const Fails& fails,
                                          const std::uint32_t total) {
    return {.text = std::format("drawn {}/{}", fails.drawn, total)};
  }

  /// The figures a pass ends on: how many wires have a way, and the verdicts
  /// against the rest. `short` and `long` count the resonators of the wrong
  /// length; `bad` counts the wires with no usable way (unrouted, open,
  /// crossing a feedline or meeting itself), each wire once.
  [[nodiscard]] std::vector<Figure>
  verdictFigures(const Fails& fails, const std::uint32_t total) const {
    std::vector<Figure> figures{drawnFigure(fails, total),
                                verdict("unrouted", fails.unrouted)};
    if (fails.feedlinesUnrouted != 0) {
      figures.push_back({.text = std::format("{} of them feedline edges",
                                             fails.feedlinesUnrouted)});
    }
    figures.push_back(verdict("open", fails.open));
    figures.push_back(verdict("short", fails.tooShort));
    if (exact_) {
      figures.push_back(verdict("long", fails.tooLong));
    }
    if (feedlinePass_) {
      figures.push_back(verdict("crossing", fails.crossing));
    }
    if (fails.loops != 0) {
      figures.push_back(verdict("meeting itself", fails.loops));
    }
    figures.push_back(verdict("bad", fails.broken));
    return figures;
  }

  /// The figures of the coupler insertion: its edges, their angle cost and
  /// the verdict of the capacity check on the chip, where the check judged
  /// it.
  [[nodiscard]] std::vector<Figure> insertionFigures() const {
    std::vector<Figure> figures{
        {.text = std::format("edges {}/{}", insertionDrawn_, edges_.size())},
        {.text = std::format("angle cost {}", insertionAngle_)}};
    if (sat_.has_value()) {
      figures.push_back({.text = *sat_ ? "SAT" : "UNSAT",
                         .tone = *sat_ ? Tone::Good : Tone::Bad});
    } else if (!tuning_.capacityCheck) {
      figures.push_back({.text = "capacity check off"});
    }
    return figures;
  }

  /// The figures of one phase of the stage, by its index in `PHASES`, over
  /// `members`.
  [[nodiscard]] std::vector<Figure>
  phaseFigures(const std::size_t phase, std::vector<Wire>& wires,
               const std::vector<std::uint32_t>& members) {
    if (phase == 2) {
      return insertionFigures();
    }
    // The count says nothing: the entry carries the figures.
    const auto loud = std::exchange(report_, Report{});
    const auto fails = failsOf(wires, members);
    report_ = loud;
    // Which resonators are off their length at the end of the phase, and by
    // how much, so that a phase is read by wire and not only by its count.
    if (report_.wants(Detail::Items)) {
      for (const auto key : members) {
        const auto& wire = wires[key];
        if (wire.tooShort || wire.tooLong) {
          say(std::format("  {} is {} at the end of {}: {:.0f} of {:.0f} cells",
                          wireId(wire), wire.tooShort ? "short" : "long",
                          PHASES[phase], lengthOf(wire.way),
                          requiredLength(wire, wire.way)));
        }
      }
    }
    const auto total = static_cast<std::uint32_t>(members.size());
    switch (phase) {
    case 0:
      return {drawnFigure(fails, total), verdict("short", fails.tooShort),
              verdict("bad", fails.broken)};
    case 1:
      return {verdict("short", fails.tooShort), verdict("long", fails.tooLong)};
    case 3:
      return {drawnFigure(fails, total), verdict("short", fails.tooShort),
              verdict("long", fails.tooLong),
              verdict("crossing", fails.crossing),
              verdict("bad", fails.broken)};
    default:
      return {verdict("short", fails.tooShort), verdict("long", fails.tooLong),
              verdict("bad", fails.broken)};
    }
  }

  /// Report the result of the stage, once: every wire of the plan, and as
  /// its fails the wires with no usable way. A resonator of the wrong length
  /// is a verdict of its own and not a fail: a run that ends with one still
  /// made a plan, and its exit code says so.
  void reportResult(const Fails& fails, const std::uint32_t total) const {
    std::vector<Figure> figures{
        drawnFigure(fails, total), verdict("short", fails.tooShort),
        verdict("long", fails.tooLong), verdict("bad", fails.broken)};
    if (insertionDone_) {
      const auto insertion = insertionFigures();
      figures.insert(figures.end(), insertion.begin() + 1, insertion.end());
    }
    report_.result(figures, fails.broken);
    tellFails("final routing", fails);
  }

  /// The entry a pass ends on, and at the level of the items the wires
  /// behind each of its verdicts.
  void sayFails(const std::string& name, const Fails& fails,
                const std::uint32_t total) const {
    report_.entry(Detail::Steps, name, verdictFigures(fails, total));
    tellFails(name, fails);
  }

  /// The wires behind each verdict, at the level of the items.
  void tellFails(const std::string& name, const Fails& fails) const {
    if (report_.wants(Detail::Items) && fails.total() != 0) {
      const auto join = [](const std::vector<std::string>& ids) {
        std::string joined;
        for (const auto& id : ids) {
          joined += (joined.empty() ? "" : ", ") + id;
        }
        return joined.empty() ? std::string("-") : joined;
      };
      say(std::format("{}: unrouted: {} · open: {} · short: {} · long: {} · "
                      "crossing: {} · meeting itself: {}",
                      name, join(fails.unroutedIds), join(fails.openIds),
                      join(fails.shortIds), join(fails.longIds),
                      join(fails.crossingIds), join(fails.loopIds)));
    }
  }

  /// Route every wire again with a price on the room it leaves, so that a
  /// wire that has room to spare moves into the middle of it. A wire that
  /// cannot be routed under the wider constraint keeps the way it had.
  void refine(std::vector<Wire>& wires,
              const std::vector<std::uint32_t>& members, const Pass& pass) {
    const auto total = static_cast<std::uint32_t>(members.size());
    if (pass.rounds != 0) {
      say(std::format("{}: {} wires, {} rounds", pass.name, total,
                      pass.rounds));
      if (pass.refinement && pass.feedlines) {
        say(std::format("{} settings: {} pair{} fenced", pass.name,
                        tuning_.fencePairs,
                        tuning_.fencePairs == 1 ? "" : "s"));
      }
    }
    // Has this resonator ever been at its length in this pass? The
    // prototype's `fitted`, and the one thing the fallback is gated on: a
    // resonator that has reached it rolls back onto a way that is still at
    // its length, so a second search buys nothing.
    std::vector<std::uint8_t> fitted(wires.size(), 0);
    ++sweepSerial_;
    for (std::uint32_t round = 0; round < pass.rounds; ++round) {
      std::uint32_t fallbacksTried = 0;
      std::uint32_t fallbacksTaken = 0;
      const bool forward = (round % 2) == 0;
      frame_.pass = passTag(pass.name);
      frame_.round = round;
      frame_.forward = forward;
      std::uint32_t moved = 0;
      for (std::uint32_t at = 0; at < total; ++at) {
        report_.progress(pass.name, std::format("round {}", round + 1), at,
                         total);
        const auto slot = forward ? at : (total - 1 - at);
        auto& wire = wires[members[slot]];
        if (!wire.drawn || !wire.feasible) {
          continue;
        }
        const auto had = wire.way;
        const auto& before = wires[members[(slot + total - 1) % total]];
        const auto& after = wires[members[(slot + 1) % total]];
        lift(wire);
        buildCorridor(wire, pass.reach);
        // The ring neighbours, `fencePairs` pairs of them: the pair
        // beside the wire first, so that one pair is the fence the phase was
        // written with, cell for cell.
        for (std::uint32_t pair = 1; pair <= tuning_.fencePairs; ++pair) {
          if (2 * pair >= total) {
            break;
          }
          fence(wire, {&wires[members[(slot + total - pair) % total]]});
          fence(wire, {&wires[members[(slot + pair) % total]]});
        }
        // The outer refinement keeps the length-point clearance too, or it
        // would level the room the rounds won (the prototype turned it back
        // on there for that reason, `FinalGrid.cpp:7628`). Nothing under
        // the feedline constraints.
        closeLengthBands(wire, {&before, &after}, pass);
        // The chamfer to the middle of the channel reads the corridor, so it
        // is built after everything that closes a cell of it — the fences of
        // the chain edges and the terminal edges' coupler runs included.
        constrainByFeedlines(wire, wires, pass);
        {
          priceRoom();
          // `priceRoom` fills the field, so the 5× discs around the chain
          // edges `constrainByFeedlines` stamped are gone with it; they go
          // back on top, which is the order the prototype stamps them in
          // (`compute_corridor_proximity_decay`, then
          // `compute_proximity_grid`).
          priceTheEdges(wire, wires, pass);
        }
        frame_.wires = &wires;
        frame_.before = before.key;
        frame_.after = after.key;
        frame_.kind = "refine";
        frame_.fence = {before.key, after.key};
        frame_.ripped.clear();
        auto found = search(wire, pass.straightStart, true, !lengthensIn(wire));
        std::string note;
        // Whether the way in hand is at this resonator's length, which is
        // what `fitted` records once the way is actually taken.
        bool atLength = false;
        if (lengthensIn(wire)) {
          // A resonator is lengthened here as it is in the relaxation, and
          // keeps the way it had when the wider way has no room for it.
          std::string said;
          bool reached = true;
          if (!found.empty()) {
            const auto made = lengthen(wire, found);
            said = made.note;
            note = " · " + said;
            reached = made.reached;
          }
          recordSearch(wire, found, said);
          if (!reached) {
            found.clear();
          }
          // The prototype's fallback: the way that was found is a detour
          // around the neighbours and a detour has no slack, so the search
          // is made once more with the room price taken off and the hard
          // corridor standing. Only while this resonator has never been at
          // its length.
          if (found.empty() && fitted[wire.key] == 0) {
            ++fallbacksTried;
            // Nothing reads the price again in this iteration, and the next
            // wire's `priceRoom` fills it from scratch, so it is zeroed
            // rather than saved and put back: a grid-sized copy per failed
            // resonator is 24 MB on the 69-qubit chip. The prototype does
            // not restore it either.
            std::ranges::fill(proximity_, 0);
            auto again = search(wire, pass.straightStart, true, false);
            std::string saidAgain("no way");
            bool reachedAgain = false;
            if (!again.empty()) {
              const auto made = lengthen(wire, again);
              saidAgain = made.note;
              reachedAgain = made.reached;
            }
            recordSearch(wire, again, saidAgain);
            if (reachedAgain) {
              found = std::move(again);
              ++fallbacksTaken;
            }
            note += std::format(" · without the room price: {}", saidAgain);
          }
          atLength = !found.empty();
        }
        tell(std::format("wire {} · round {} {} · refine: {}{}", wireId(wire),
                         round, forward ? "forward" : "backward",
                         found.empty()
                             ? std::string("kept its way")
                             : std::format("found {} cells", found.size()),
                         note));
        // The prototype's refinement takes what it finds, and falls back to
        // the way it had when the wider constraint does not route. Here a
        // way found is taken only when it is not worse than the way the
        // wire had, counted against every other wire with the wire lifted,
        // and when it crosses no feedline other than at a right angle:
        // under the feedline constraints a wider way is fenced by the two
        // neighbours and the edges alone, and a way that takes another
        // wire's room is not wider, it is wrong.
        if (!found.empty()) {
          const auto before = conflictsIn(had, wire, wires);
          const auto after = conflictsIn(found, wire, wires);
          bool crossing = false;
          if (feedlinePass_) {
            wire.way = found;
            crossing = crossesAFeedline(wire, wires);
          }
          if (after > before || crossing) {
            tell(std::format(
                "wire {} · refine: the way found is worse, {} against {} "
                "conflicting cells{}; kept its way",
                wireId(wire), after, before,
                crossing ? ", or crosses a feedline" : ""));
            found.clear();
          }
        }
        wire.way = found.empty() ? had : found;
        moved += found.empty() ? 0 : 1;
        // Only a way that is taken makes a resonator fitted. A way at its
        // length that the test above refused leaves the wire on the way it
        // had, which this pass has not measured, so the fallback is offered
        // again next round — one search more than the prototype pays, and
        // the conservative way round.
        if (atLength && !found.empty()) {
          fitted[wire.key] = 1;
        }
        wire.drawn = true;
        place(wire);
      }
      std::uint32_t crowded = 0;
      for (const auto member : members) {
        auto& wire = wires[member];
        if (!wire.drawn) {
          continue;
        }
        lift(wire);
        crowded += conflictsOf(wire, wires) == 0 ? 0 : 1;
        place(wire);
      }
      say(std::format("{} round {} {}: moved {} of {}, {} too close to "
                      "another{}",
                      pass.name, round, forward ? "forward " : "backward",
                      moved, total, crowded,
                      fallbacksTried == 0
                          ? std::string{}
                          : std::format(", {} of {} searches without the room "
                                        "price took their length",
                                        fallbacksTaken, fallbacksTried)));
    }
    // The guarantees of the insertion, said again on what the refinement
    // left: it moves feedline edges, and the insertion's lines were about
    // the edges as it drew them.
    if (pass.rounds != 0 && pass.feedlines && pass.refinement) {
      static_cast<void>(checkFeedlineRoom(wires, pass.name));
      static_cast<void>(checkCouplerCrossings(wires, pass.name));
      static_cast<void>(checkResonatorCrossings(wires, pass.name));
      static_cast<void>(checkFeedlineCrossings(wires, pass.name));
      static_cast<void>(checkChainCrossings(wires, pass.name));
    }
  }

  // ---------------------------------------------------------- The couplers

  /// One way a coupler can sit on its resonator: the prototype's
  /// `CouplerOption`, with the geometry this stage builds from it.
  ///
  /// The resonator's way is cut where the way left to the qubit is the
  /// target length and a dogleg is put in front: from the anchor the way
  /// runs the coupling length straight on the heading across the coupler's
  /// orientation, turns a quarter onto the orientation, runs the straight
  /// start and joins the way. The body spans the coupling run,
  /// `couplerHeight` cells across on the side away from the turn, so that
  /// the turn never crosses the feedline, and the feedline runs along its
  /// far edge, across the orientation too: along the ring, where the chains
  /// run, as the prototype's body lies. The feedline may run either way
  /// along it.
  struct CouplerOption {
    /// The orientation the pad takes: the heading the way holds at the
    /// place, turned by the offset.
    Heading orientation = 0;
    /// Which of the eight turns off that heading this option is.
    Heading offset = 0;

    /// The coupling run and the depth of the body, in cells on this
    /// orientation: the design figure in layout units, which along a diagonal
    /// is fewer cells than along an axis because a diagonal step is longer by
    /// the square root of two.
    std::uint32_t run = 0;
    std::uint32_t depth = 0;
    /// The resonator's way from the anchor to the qubit.
    Path way;
    /// The anchor: the resonator's port on the pad, where its mandatory
    /// quarter turn off the coupler begins.
    PathPoint anchor;
    /// The centre of the pad, which is the cell the placement rule names.
    PathPoint centre;
    /// Which of the two resonator ports the resonator leaves from.
    bool secondPort = false;
    /// The second dogleg, the prototype's `second_straight` /
    /// `second_reverse`: after the arc and its straight, one more quarter
    /// turn and one more straight before the lead meets the path. Zero for
    /// none. It is what lets a coupler slide sideways off the way it sits
    /// on — the same pad, reached from one side or the other.
    std::uint32_t secondStraight = 0;
    bool secondReverse = false;
    /// Which place of the resonator's way the option sits at, in the order
    /// `couplerPlace` offers them: 0 is the place nearest the target length.
    std::uint32_t place = 0;
    /// How many cells of `arc` the first quarter turn and its straight take;
    /// with a second dogleg the cell after them is the first lead's tip.
    std::size_t firstLead = 0;
    /// How far the option was moved into the box, in cells (see
    /// `couplerShift`). Zero when the lead's tip lands on the place.
    std::int64_t shiftX = 0;
    std::int64_t shiftY = 0;
    /// **The coupler's orientation**: the direction the straight run takes
    /// after the arc off the resonator port. Not the pad's long axis, which
    /// is `orientation` and is the axis the feedline runs along — this is
    /// the direction the coupler points the resonator in, and it is what the
    /// legality rule binds and what the layout is drawn on.
    Heading couplerOrientation = 0;
    /// The coupler's two resonator ports: the ends of the near edge,
    /// parallel to the feedline pair and opposite it. The resonator leaves
    /// the first with the clock and would leave the second against it.
    PathPoint resonatorIn;
    PathPoint resonatorOut;
    /// Where the quarter turn ends, on the orientation: the source of every
    /// search of the resonator from then on.
    PathPoint arcEnd;
    /// The cells of the coupling run and the turn, the arc end excluded.
    Path arc;
    /// The cells of the body.
    std::vector<std::size_t> body;
    /// Where the feedline arrives and where it leaves, with its heading.
    PathPoint in;
    PathPoint out;
    /// What the option cost the last time it was priced: the two edges to
    /// its chain neighbours.
    std::uint32_t cost = std::numeric_limits<std::uint32_t>::max();
    /// How many cells of the body and the run lie in the room of another
    /// wire. Not a rejection — every wire is drawn again under the feedline
    /// constraints — but a price.
    std::uint32_t guarded = 0;
  };

  /// A coupler: the resonator it belongs to and every way it can sit.
  struct Coupler {
    std::uint32_t wire = NO_OWNER;
    std::vector<CouplerOption> options;
    std::size_t chosen = 0;
    /// The anchor of the chosen option, for the meeting exemption.
    PathPoint anchor;
    /// The resonator's way before the cut, which every option is cut from.
    Path outerWay;
    /// Whether the options with a second dogleg are open to this coupler.
    ///
    /// They are not, to begin with. A jog slides the pad sideways and there
    /// are two of them per offset and port, so opening them everywhere
    /// trebles what the greedy has to route and hands it choices it cannot
    /// judge — its cost measures the feedline's turning and knows nothing of
    /// the resonator. So a coupler gets them only once it has shown it needs
    /// them: a pass in which not one of its plain options brought both of
    /// its feedline edges home unlocks them for the passes after.
    bool jogsUnlocked = false;
    /// The feedline edges into and out of it, by wire key.
    std::uint32_t edgeIn = NO_OWNER;
    std::uint32_t edgeOut = NO_OWNER;
  };

  /// A point of a feedline chain: a launcher, or a coupler with its options.
  struct Waypoint {
    bool fixed = false;
    std::uint32_t coupler = NO_OWNER;
    std::uint32_t port = NO_OWNER;
    /// The launcher as a source and as a target.
    PathPoint asSource;
    PathPoint asTarget;
  };

  /// One edge of a chain, and the feedline wire that draws it.
  struct Edge {
    std::uint32_t chain = 0;
    std::size_t from = 0;
    std::size_t to = 0;
    std::uint32_t wire = NO_OWNER;
    bool terminal = false;
    /// An edge of a chain the chain search has laid only up to some
    /// waypoint (`stateOf`). Such a chain is open at its end and separates
    /// nothing yet, so the capacity graph prescribes no crossing of it.
    bool open = false;
  };

  [[nodiscard]] const std::vector<Coupler>& couplers() const {
    return couplers_;
  }
  [[nodiscard]] const std::vector<Edge>& edges() const { return edges_; }

  /// What `reportSqueeze` found on one edge: the figures in words and the
  /// worst line it measured, from the edge's cell to the wall it hit.
  struct Squeeze {
    std::string note;
    PathPoint from;
    PathPoint to;
  };
  [[nodiscard]] const std::unordered_map<std::uint32_t, Squeeze>&
  squeezed() const {
    return squeezed_;
  }
  [[nodiscard]] const std::vector<std::vector<Waypoint>>& chains() const {
    return chains_;
  }

  /// The straight run a feedline holds along a coupler's body: the coupling
  /// run of the option that coupler stands on, which is fewer cells along a
  /// diagonal than along an axis. The straight start where there is no
  /// coupler, so that a launcher keeps the run the rule asks for.
  [[nodiscard]] std::uint32_t couplerRunOf(const std::uint32_t coupler) const {
    if (coupler == NO_OWNER || coupler >= couplers_.size()) {
      return tuning_.straightStart;
    }
    const auto& found = couplers_[coupler];
    return found.options[found.chosen].run;
  }

  /// The straight runs an edge has no choice about: the one it leaves its
  /// source on and the one it arrives at its target on. `routeEdge` hands
  /// these to the search and `corridorOfEdge` has to leave them open, so
  /// they are said once here rather than twice.
  [[nodiscard]] std::pair<std::uint32_t, std::uint32_t>
  runsOfEdge(const Edge& edge) const {
    const auto& chain = chains_[edge.chain];
    return {chain[edge.from].fixed ? tuning_.straightStart
                                   : couplerRunOf(chain[edge.from].coupler),
            chain[edge.to].fixed ? tuning_.straightStart
                                 : couplerRunOf(chain[edge.to].coupler)};
  }

  /// The source of a waypoint as the edge that leaves it sees it, and the
  /// target as the edge that reaches it sees it.
  [[nodiscard]] PathPoint sourceOf(const Waypoint& point) const {
    if (point.fixed) {
      return point.asSource;
    }
    const auto& coupler = couplers_[point.coupler];
    return coupler.options[coupler.chosen].out;
  }
  [[nodiscard]] PathPoint targetOf(const Waypoint& point) const {
    if (point.fixed) {
      return point.asTarget;
    }
    const auto& coupler = couplers_[point.coupler];
    // The end the chain arrives at, not the one it leaves on: between the two
    // it runs the length of the pad, which is what couples it.
    return coupler.options[coupler.chosen].in;
  }

  /// Phase 3: put a coupler on every resonator and route the feedline chains
  /// through them.
  ///
  /// The prototype's `run_optimized_cpw_coupler_insertion`. For every
  /// resonator the options are built: the pad along the way at the places the
  /// biased target names, rejected when the pad or the feedline's run along
  /// it leaves the grid or meets the artwork. Then
  /// every chain is optimized by a greedy local search: a coupler takes the
  /// option whose two feedline edges to its chain neighbours turn least,
  /// until no coupler improves or five passes are done. The chosen options
  /// are committed — the resonator cut back to its coupler, the body an
  /// obstacle, the edges drawn — and what came of it is said.
  void insertCouplers(std::vector<Wire>& wires, const AssignmentT& assignment,
                      const ChipT& chip) {
    const auto began = std::chrono::steady_clock::now();
    couplers_.clear();
    chains_.clear();
    edges_.clear();
    fastCapacity_.reset();
    profileMemo_.clear();
    budgetCutoffs_ = 0;
    stepBudget_ = routing::TRELLIS_UNREACHABLE;
    room_ = RoomStats{};
    bodies_ = grid::BitGrid(scene_.router.width, scene_.router.height);
    aimAtTarget(true);
    couplerBox_ = couplerBoxOf();
    closeMeanders(wires);
    say(std::format(
        "couplers sit inside x {}..{} y {}..{}, which is what the "
        "launcher stubs leave open at a run of {} cells inflated "
        "by {} ({} cells of lead over the stub itself); a place "
        "is taken where one of the eight orientations puts both "
        "feedline ports and their runs inside it; an edge between two "
        "couplers stays inside it as well",
        couplerBox_.minX, couplerBox_.maxX, couplerBox_.minY, couplerBox_.maxY,
        tuning_.launcherStraight, tuning_.clearance,
        tuning_.launcherStraight - tuning_.straightStart));
    if (tuning_.couplerBoxMargin > 0) {
      say(std::format(
          "a coupler keeps {} cells more off every side of that box "
          "(coupler_box_margin_cells): x {}..{} y {}..{}",
          tuning_.couplerBoxMargin, couplerBox_.minX + tuning_.couplerBoxMargin,
          couplerBox_.maxX - tuning_.couplerBoxMargin,
          couplerBox_.minY + tuning_.couplerBoxMargin,
          couplerBox_.maxY - tuning_.couplerBoxMargin));
    }
    if (tuning_.edgeLauncherMargin > 0) {
      say(std::format(
          "a feedline edge keeps {} cells closed in front of every launcher "
          "but its own: the run of {} cells, {} cells of margin "
          "(edge_launcher_margin_cells), and the clearance around them",
          static_cast<std::int64_t>(tuning_.straightStart) +
              tuning_.edgeLauncherMargin,
          static_cast<std::int64_t>(tuning_.straightStart),
          tuning_.edgeLauncherMargin));
    }

    // The approaches of every port, inflated by the clearance: no coupler
    // may sit in one and no feedline edge may run through one, because the
    // wire into that port has nowhere else to go. So the approach reaches
    // from the port out past the cell the search ends on by the straight
    // start it has to arrive on. A resonator's feed is a point on the ring
    // and not a port, so only its target counts.
    approaches_ = grid::BitGrid(scene_.router.width, scene_.router.height);
    {
      const auto& stencil = stencilFor(tuning_.clearance);
      const auto width = static_cast<std::int64_t>(scene_.router.width);
      // Only the straight run the wire has to arrive on. The crossing band
      // used to be added here as well, to keep a feedline far enough from a
      // port that the wire into it could still cross at a right angle — but
      // the insertion has no orthogonal crossing rule, so that band was a
      // rule from another phase enforced by geometry in this one.
      const auto beyond = tuning_.straightStart;
      for (const auto& wire : wires) {
        if (!wire.feasible || wire.feedline || wire.fixed.empty()) {
          continue;
        }
        Path places = wire.resonator ? Path{} : wire.fixed;
        // The cells before the target, where the search arrives on the
        // port's heading: back from the target against that heading.
        const auto v = routing::headingVector(wire.objective.target.heading);
        for (std::int64_t k = 0; k <= beyond; ++k) {
          // `k` is signed on purpose: a heading step is a signed byte, and an
          // unsigned `k` would compute `k * v.dx` unsigned, turning a step of
          // -1 into four billion and cutting the approach short after one cell
          // on every heading that runs left or down.
          const auto x =
              static_cast<std::int64_t>(wire.objective.target.x) - (k * v.dx);
          const auto y =
              static_cast<std::int64_t>(wire.objective.target.y) - (k * v.dy);
          if (x < 0 || y < 0 || x >= width || y >= scene_.router.height) {
            break;
          }
          places.push_back({.x = static_cast<std::uint32_t>(x),
                            .y = static_cast<std::uint32_t>(y),
                            .heading = wire.objective.target.heading,
                            .primitive = 0});
        }
        const auto in = router_.straightStub(wire.objective.target, true,
                                             tuning_.straightStart);
        places.insert(places.end(), in.begin(), in.end());
        alongDisc(
            places, stencil, [&](const std::int64_t x, const std::int64_t y) {
              approaches_.set(static_cast<std::size_t>((y * width) + x), true);
            });
      }
    }

    // Which wires have little room to give way: everything of the ring that
    // is not a resonator and not a feedline. Read by the option price.
    conventional_.assign(wires.size(), 0);
    for (const auto& wire : wires) {
      if (wire.key < conventional_.size()) {
        conventional_[wire.key] = static_cast<std::uint8_t>(
            !wire.resonator && !wire.feedline ? 1 : 0);
      }
    }

    // The couplers, one per drawn resonator, with their options.
    couplerOfWire_.clear();
    auto& couplerOfWire = couplerOfWire_;
    std::uint32_t withoutOptions = 0;
    for (auto& wire : wires) {
      if (!wire.resonator || !wire.feasible || !wire.drawn) {
        continue;
      }
      Coupler coupler;
      coupler.wire = wire.key;
      coupler.outerWay = wire.way;
      const auto offsets = tuning_.couplerOffsets;
      coupler.options = optionsOf(wire, offsets);
      if (coupler.options.empty() &&
          std::ranges::any_of(offsets, [](const bool on) { return !on; })) {
        // None of the allowed offsets fits this coupler: it takes every
        // offset, as without `couplerOffsets`.
        ++offsetFallbacks_;
        say(std::format("coupler insertion: coupler for resonator {}: no "
                        "option on the allowed offsets {} fits — every "
                        "offset is allowed for it",
                        wireId(wire), offsetsText(offsets)));
        std::array<bool, routing::NUM_HEADINGS> every{};
        every.fill(true);
        coupler.options = optionsOf(wire, every);
      }
      if (coupler.options.empty()) {
        ++withoutOptions;
        tell(std::format("coupler for resonator {}: no option fits",
                         wireId(wire)));
        continue;
      }
      // The coupler starts on its first option without a second dogleg,
      // where it has one.
      {
        const auto plain = std::ranges::find_if(
            coupler.options, [](const CouplerOption& option) {
              return option.secondStraight == 0;
            });
        if (plain != coupler.options.end()) {
          coupler.chosen =
              static_cast<std::size_t>(plain - coupler.options.begin());
        }
      }
      std::string where;
      for (Heading heading = 0; heading < routing::NUM_HEADINGS; ++heading) {
        if (std::ranges::any_of(coupler.options,
                                [heading](const CouplerOption& option) {
                                  return option.orientation == heading;
                                })) {
          where += (where.empty() ? "" : " ") + std::to_string(heading);
        }
      }
      tell(std::format("coupler for resonator {}: {} options, orientations {}",
                       wireId(wire), coupler.options.size(), where));
      couplerOfWire.emplace(wire.key,
                            static_cast<std::uint32_t>(couplers_.size()));
      couplers_.push_back(std::move(coupler));
    }

    // The chains, from the assignment: launcher, couplers in ring order,
    // launcher; an end that is a termination has no launcher.
    for (const auto& chain : assignment.chains) {
      std::vector<Waypoint> points;
      const auto launcher =
          [&](const fbd::PortRef& port) -> std::optional<Waypoint> {
        const auto index = port.index();
        const auto cell = scene_.launcherCell.find(index);
        if (index >= chip.ports.size() || cell == scene_.launcherCell.end()) {
          return std::nullopt;
        }
        const auto heading =
            routing::headingOfOrientation(chip.ports[index]->orientation);
        const auto place = scene_.router.cell(cell->second);
        Waypoint point;
        point.fixed = true;
        point.port = index;
        point.asSource = {.x = place.x(),
                          .y = place.y(),
                          .heading = routing::reverse(heading),
                          .primitive = 0};
        point.asTarget = {
            .x = place.x(), .y = place.y(), .heading = heading, .primitive = 0};
        return point;
      };
      if (chain->start != nullptr) {
        if (const auto point = launcher(*chain->start)) {
          points.push_back(*point);
        }
      }
      for (const auto node : chain->nodes) {
        const auto found = std::ranges::find_if(wires, [&](const Wire& wire) {
          return !wire.inner && wire.slot == node;
        });
        if (found == wires.end()) {
          continue;
        }
        const auto coupler = couplerOfWire.find(found->key);
        if (coupler == couplerOfWire.end()) {
          continue;
        }
        Waypoint point;
        point.coupler = coupler->second;
        points.push_back(point);
      }
      if (chain->end != nullptr) {
        if (const auto point = launcher(*chain->end)) {
          points.push_back(*point);
        }
      }
      if (points.size() >= 2) {
        // A feedline with a reflector has one launcher; the edge at its
        // reflector end runs between two couplers and is no terminal edge
        // (`atLauncher`).
        if (points.front().fixed != points.back().fixed) {
          say(std::format(
              "coupler insertion: chain {}: one launcher, a reflector at its "
              "{} end — its {} edge runs between couplers",
              chains_.size(), points.front().fixed ? "last" : "first",
              points.front().fixed ? "last" : "first"));
        }
        chains_.push_back(std::move(points));
      }
    }

    chainEdgePaths_.assign(chains_.size(), {});
    edgeWireOf_.assign(chains_.size(), {});
    for (std::size_t chain = 0; chain < chains_.size(); ++chain) {
      const auto edges = chains_[chain].size() - 1;
      chainEdgePaths_[chain].assign(edges, Path{});
      edgeWireOf_[chain].assign(edges, NO_OWNER);
    }
    liftedR1_.assign(chains_.size(), false);
    liftedR2_.assign(chains_.size(), false);
    liftedR3_.assign(chains_.size(), false);
    chainOfCoupler_.assign(couplers_.size(), {NO_OWNER, 0});
    for (std::uint32_t chain = 0; chain < chains_.size(); ++chain) {
      for (std::size_t at = 0; at < chains_[chain].size(); ++at) {
        const auto& point = chains_[chain][at];
        if (point.fixed) {
          continue;
        }
        if (chainOfCoupler_[point.coupler].first != NO_OWNER) {
          throw std::logic_error(std::format(
              "coupler {} stands in chains {} and {}", point.coupler,
              chainOfCoupler_[point.coupler].first, chain));
        }
        chainOfCoupler_[point.coupler] = {chain, at};
      }
    }

    // R0: the ring as the feedline pass will sweep it — every wire that is
    // not inner, in the order the assignment feeds them, which is the order
    // `wires` holds them in before the edges are appended — and for every
    // edge between two couplers the plain wires that must cross it.
    ring_.clear();
    ringPlace_.clear();
    for (const auto& wire : wires) {
      if (!wire.inner && !wire.feedline) {
        ringPlace_.emplace(wire.slot, static_cast<std::uint32_t>(ring_.size()));
        ring_.push_back(wire.key);
      }
    }
    bridgers_.assign(chains_.size(), {});
    for (std::uint32_t chain = 0; chain < chains_.size(); ++chain) {
      const auto edges = chains_[chain].size() - 1;
      bridgers_[chain].assign(edges, {});
      for (std::size_t at = 0; at < edges; ++at) {
        bridgers_[chain][at] = bridgersOf(wires, chain, at);
        if (!bridgers_[chain][at].empty()) {
          tell(std::format("coupler insertion: chain {} edge {}->{}: {} plain "
                           "wire{} must cross it ({})",
                           chain, at, at + 1, bridgers_[chain][at].size(),
                           bridgers_[chain][at].size() == 1 ? "" : "s",
                           namesOf(wires, bridgers_[chain][at])));
        }
      }
    }

    recordCouplerOptions(wires);

    say(std::format(
        "coupler insertion: settings: end edge weight {:g}, coupler offsets "
        "{}{}, shortfall {:.0f}%, moved into the box by up to {} cells; room: "
        "pitch {}, margin {}, channel reach {}",
        tuning_.chainEndEdgeWeight, offsetsText(tuning_.couplerOffsets),
        offsetFallbacks_ == 0
            ? std::string{}
            : std::format("; {} coupler{} on every offset", offsetFallbacks_,
                          offsetFallbacks_ == 1 ? "" : "s"),
        tuning_.couplerMaxShortfall * 100.0, tuning_.couplerShift, roomPitch(),
        tuning_.roomMargin, tuning_.roomChannelReach));

    // The search for the options: every chain on its own, by the prefix A*.
    const auto passes = optimizeChainsPrefix(wires);

    // The commit: the chosen options, then the edges.
    for (std::uint32_t index = 0; index < couplers_.size(); ++index) {
      applyOption(wires, index, couplers_[index].chosen);
    }
    for (std::uint32_t chain = 0; chain < chains_.size(); ++chain) {
      const auto& points = chains_[chain];
      for (std::size_t at = 0; at + 1 < points.size(); ++at) {
        Edge edge{.chain = chain,
                  .from = at,
                  .to = at + 1,
                  .wire = static_cast<std::uint32_t>(wires.size()),
                  .terminal = points[at].fixed || points[at + 1].fixed};
        Wire wire;
        wire.key = edge.wire;
        wire.slot = static_cast<std::uint32_t>(edges_.size());
        wire.feedline = true;
        wire.feasible = true;
        wire.terminal = edge.terminal;
        wire.edge = wire.slot;
        // Both ends the same way round: the edge that leaves a coupler runs
        // along its body exactly as the edge that arrives on it does, and
        // `couplerRunOf` is that run — the straight start where there is no
        // coupler, so a launcher keeps the run the rule asks for. The start
        // used to take the axial `straightStart` whatever stood there, which
        // made the two edges of one coupler disagree about its own body.
        wire.startStub = couplerRunOf(points[at].coupler);
        wire.endStub = points[at + 1].fixed
                           ? tuning_.straightStart
                           : couplerRunOf(points[at + 1].coupler);
        wire.couplerAtSource = points[at].coupler;
        wire.couplerAtTarget = points[at + 1].coupler;
        wire.sourcePort = points[at].port;
        wire.targetPort = points[at + 1].port;
        wire.objective.source = sourceOf(points[at]);
        wire.objective.target = targetOf(points[at + 1]);
        if (points[at].coupler != NO_OWNER) {
          couplers_[points[at].coupler].edgeOut = edge.wire;
        }
        if (points[at + 1].coupler != NO_OWNER) {
          couplers_[points[at + 1].coupler].edgeIn = edge.wire;
        }
        edgeWireOf_[chain][at] = edge.wire;
        wires.push_back(std::move(wire));
        edges_.push_back(edge);
      }
    }
    // The edges as the greedy chose them, where they still run between the
    // ports the couplers ended on; the rest routed now, against everything
    // committed before them.
    std::uint32_t drawn = 0;
    std::uint32_t angle = 0;
    for (const auto& edge : edges_) {
      auto& wire = wires[edge.wire];
      const auto& chosen = chainEdgePaths_[edge.chain][edge.from];
      // A path the greedy chose is kept only when it still runs between the
      // right two ports **and** still lies in the corridor that holds now.
      // The endpoints alone are not enough: the greedy drew it against the
      // chains as they stood then, with the edges of the coupler it was
      // weighing left out, and every edge committed since has closed ground
      // under it. Kept without this test, such a path ships straight through
      // a launcher's stub or another chain — visible in the debug picture as
      // a way crossing closed white, which no search would ever have
      // returned.
      // The endpoints are identity and are always asked. Whether the way
      // still lies in the corridor that holds now is the *rule*.
      const bool runsRight = !chosen.empty() &&
                             chosen.front().samePlace(wire.objective.source) &&
                             chosen.back().samePlace(wire.objective.target);
      const bool fits =
          runsRight && edgeWayStillOpen(wires, wire.objective, edge, chosen);
      wire.way = fits ? chosen : routeEdge(wires, wire.objective, edge);
      wire.drawn = !wire.way.empty();
      // **The other order, for the pair at either end of the chain**, as
      // the chain search tries it (user, 2026-10-11): a chain the search
      // left as it stood is committed front to back, and an end edge that
      // finds no way after its predecessor may find one laid first, with
      // the predecessor routed again against it.
      const auto lastEdge = chainEdgePaths_[edge.chain].size() - 1;
      if (!wire.drawn && edge.from >= 1 &&
          ((edge.from == 1 && atLauncher(edge.chain, 0)) ||
           (edge.from == lastEdge && edge.terminal))) {
        const auto before = std::ranges::find_if(edges_, [&](const Edge& one) {
          return one.chain == edge.chain && one.from + 1 == edge.from;
        });
        auto& prior = wires[before->wire];
        if (prior.drawn) {
          const auto priorWay = prior.way;
          const auto priorPath = chainEdgePaths_[edge.chain][before->from];
          lift(prior);
          prior.way.clear();
          prior.drawn = false;
          chainEdgePaths_[edge.chain][before->from].clear();
          auto mine = routeEdge(wires, wire.objective, edge);
          Path again;
          if (!mine.empty()) {
            wire.way = std::move(mine);
            wire.drawn = true;
            place(wire);
            again = routeEdge(wires, prior.objective, *before);
          }
          if (!again.empty()) {
            angle -= angleCostOf(priorWay, prior.objective.target.heading);
            angle += angleCostOf(again, prior.objective.target.heading);
            prior.way = std::move(again);
            chainEdgePaths_[edge.chain][before->from] = prior.way;
            say(std::format(
                "coupler insertion: edge f{} chain {} ({}->{}): no way after "
                "f{}, but both fit with f{} laid first — f{} routed again",
                wire.slot, edge.chain, edge.from, edge.to, prior.slot,
                wire.slot, prior.slot));
          } else {
            if (wire.drawn) {
              lift(wire);
            }
            wire.way.clear();
            wire.drawn = false;
            prior.way = priorWay;
            chainEdgePaths_[edge.chain][before->from] = priorPath;
          }
          prior.drawn = true;
          place(prior);
        }
      }
      // A way the greedy chose and the commit kept was drawn by no search of
      // this loop, so it has no picture of its own. Draw it against the
      // ground it stands on, or every line of the log would link the last
      // search the greedy happened to make.
      if (debug_ && fits) {
        corridorOfEdge(wires, wire.objective, edge);
        Wire pretend;
        pretend.objective = wire.objective;
        pretend.feedline = true;
        pretend.slot = wire.slot;
        frame_.wires = &wires;
        frame_.pass = "edge";
        frame_.round = edge.chain;
        frame_.forward = true;
        frame_.kind =
            std::format("c{}-{}to{}-kept", edge.chain, edge.from, edge.to);
        frame_.fence.clear();
        frame_.ripped.clear();
        frame_.before = NO_OWNER;
        frame_.after = NO_OWNER;
        recordSearch(pretend, wire.way,
                     std::format("chain {} {}->{}, as the greedy chose it",
                                 edge.chain, edge.from, edge.to));
      }
      if (wire.drawn) {
        ++drawn;
        angle += angleCostOf(wire.way, wire.objective.target.heading);
        place(wire);
      }
      tell(std::format(
          "coupler insertion: edge f{} chain {} ({}->{}{}): {}", wire.slot,
          edge.chain, edge.from, edge.to, wire.terminal ? ", terminal" : "",
          wire.drawn ? std::format(
                           "{} cells, angle {}{}", wire.way.size(),
                           angleCostOf(wire.way, wire.objective.target.heading),
                           fits ? ", as the greedy chose it" : ", routed again")
                     : std::string("NO WAY")));
    }
    std::vector<std::size_t> chosenAtCommit;
    for (const auto& coupler : couplers_) {
      chosenAtCommit.push_back(coupler.chosen);
    }
    if (tuning_.capacityCheck && searchGlobalSat(wires)) {
      // The edges of the couplers the search moved were drawn again.
      drawn = 0;
      angle = 0;
      for (const auto& edge : edges_) {
        const auto& wire = wires[edge.wire];
        if (wire.drawn) {
          ++drawn;
          angle += angleCostOf(wire.way, wire.objective.target.heading);
        }
      }
    }
    insertionDone_ = true;
    insertionDrawn_ = drawn;
    insertionAngle_ = angle;
    for (std::uint32_t index = 0; index < couplers_.size(); ++index) {
      sayCoupler(wires, index);
    }
    static_cast<void>(checkFeedlineRoom(wires));
    static_cast<void>(checkCouplerBodies(wires));
    static_cast<void>(checkCouplerCrossings(wires));
    static_cast<void>(checkResonatorCrossings(wires));
    static_cast<void>(checkFeedlineCrossings(wires));
    static_cast<void>(checkChainCrossings(wires));
    reportRoom(wires);
    reportSqueeze(wires);
    reportBottlenecks(wires);
    const auto seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - began)
            .count();
    const auto diagonal =
        std::ranges::count_if(couplers_, [](const Coupler& coupler) {
          return routing::isDiagonal(
              coupler.options[coupler.chosen].way.front().heading);
        });
    say(std::format(
        "coupler insertion: {} couplers on {} resonators ({} on a diagonal), "
        "{} chains, {} of {} edges drawn, feedline angle cost {}, {} {}, "
        "{} of {} option costs from the memo ({} remembered ways routed "
        "again under the prefix), {:.1f}s",
        couplers_.size(), couplers_.size() + withoutOptions, diagonal,
        chains_.size(), drawn, edges_.size(), angle, passes, "chains settled",
        memoHits_, memoHits_ + memoMisses_, memoStale_, seconds));
    // The two figures the insertion is judged by, on a line of their own so
    // that a sweep over settings can be read off the log without counting.
    // Whether the ways the insertion leaves behind are ways a search could
    // have returned. A resonator spliced onto its coupler's lead is the one
    // way in the stage that no router drew, so it is the one that can hold a
    // corner no primitive makes; `couplerLeadMargin` is what buys the join
    // room to be a turn instead.
    std::uint32_t kinked = 0;
    std::uint32_t kinks = 0;
    for (const auto& coupler : couplers_) {
      const auto found = kinksIn(wires[coupler.wire].way);
      kinked += found == 0 ? 0 : 1;
      kinks += found;
    }
    say(std::format("coupler insertion: lead {} cells straight, {} of margin, "
                    "cut at {:.2f} of the target length; {} of {} resonators "
                    "keep a way with a corner no primitive makes, {} corners "
                    "in all",
                    leadStraight(), tuning_.couplerLeadMargin,
                    tuning_.couplerBias, kinked, couplers_.size(), kinks));
    const auto missing = static_cast<std::uint32_t>(edges_.size()) - drawn;
    if (boundPairs_ > 0) {
      say(std::format(
          "coupler insertion: the layer graph took {} bound{} in {:.1f} ms "
          "({:.3f} us each), bound {}",
          boundPairs_, boundPairs_ == 1 ? "" : "s",
          static_cast<double>(boundNanos_) / 1e6,
          static_cast<double>(boundNanos_) / 1000.0 /
              static_cast<double>(boundPairs_),
          "analytic with the end runs"));
    }
    say(std::format("coupler insertion: the step budget cut off {} edge "
                    "searches",
                    budgetCutoffs_));
    {
      const auto chainsOf = [](const std::vector<std::uint32_t>& chains) {
        std::string out;
        for (const auto chain : chains) {
          out += (out.empty() ? "" : " ") + std::to_string(chain);
        }
        return out;
      };
      say(std::format(
          "coupler insertion: room rules — R1 refused {}/{} edges, R2 "
          "refused {}/{} channels, R3 refused {} places ({} couplers built "
          "without it, {} moved > 100 units); stood down on chains R2[{}] "
          "R1[{}] R3[{}]; committed: {} edges below capacity, {} of {} "
          "coupler channels tighter than (m+1)·pitch ({} by along-runners), "
          "{} of {} channels between chains, couplers with a plain wire "
          "within 0/5/10/19 cells: {}/{}/{}/{}",
          room_.r1Refused, room_.r1Asked, room_.r2Refused, room_.r2Asked,
          room_.r3Refused, room_.r3StoodDown, room_.r3Moved,
          chainsOf(room_.liftedR2), chainsOf(room_.liftedR1),
          chainsOf(room_.liftedR3), room_.committedBelowCapacity,
          room_.committedTightChannels, room_.couplerChannels,
          room_.committedTightAlong, room_.crossChainTight,
          room_.crossChainChannels, room_.r3Near[0], room_.r3Near[1],
          room_.r3Near[2], room_.r3Near[3]));
    }
    say(std::format("==> coupler insertion: {} feedline edge{} NOT drawn, "
                    "feedline angle cost {}, {:.1f}s",
                    missing, missing == 1 ? "" : "s", angle, seconds));
  }

  /// Every option of a coupler on a resonator's way: the prototype's sweep
  /// over the orientation offsets and the mirror, with the direction of the
  /// feedline and one second dogleg on top.
  [[nodiscard]] std::vector<CouplerOption>
  optionsOf(Wire& resonator,
            const std::array<bool, routing::NUM_HEADINGS>& offsets) {
    std::vector<CouplerOption> options;
    // The resonator's own room must not stand in the way of its own body.
    lift(resonator);
    // The place is settled **per orientation**, not once for all of them.
    // Every orientation gets the point of the way whose mismatch to the
    // biased target is least *among the points where that orientation
    // actually fits* — where its pad, its feedline run and its resonator
    // lead all stay inside the box. A point that is perfect on length but
    // puts the pad through the box edge is not the place for that
    // orientation; the next point along the way is tried, and the one after
    // that.
    //
    // Settling it once for all eight was what cost the options: one
    // orientation's perfect point is another's impossible one, and the
    // whole coupler was built at the first point that suited *anybody*.
    const auto places = couplerPlace(resonator);
    for (Heading offset = 0; offset < routing::NUM_HEADINGS; ++offset) {
      if (!offsets.at(offset)) {
        continue;
      }
      for (const bool secondPort : {false, true}) {
        // The second dogleg, as the prototype sweeps it: none, or one of
        // twenty turned each way. It slides the pad sideways off the way it
        // sits on without changing which way the coupler points, which is
        // the one freedom the offsets and the two ports do not give.
        for (const auto& [secondStraight, secondReverse] :
             {std::pair{0U, false}, std::pair{COUPLER_SECOND_STRAIGHT, true},
              std::pair{COUPLER_SECOND_STRAIGHT, false}}) {
          std::string why;
          std::size_t tried = 0;
          bool got = false;
          // Every place, in order of mismatch, until one fits. No cap: the
          // cap was what broke 4q, where the two dozen places nearest the
          // target length all put the pad through the box edge and the
          // coupler was given up on while places that fitted lay further
          // along the way.
          for (const auto& spot : places) {
            ++tried;
            // An option that leaves the box is moved by what its cell outside
            // lacks and built again, until it fits or the move passes
            // `couplerShift` on an axis. A move can push another cell out on
            // the other side; `COUPLER_SHIFT_ROUNDS` ends that.
            std::pair<std::int64_t, std::int64_t> miss{0, 0};
            std::int64_t shiftX = 0;
            std::int64_t shiftY = 0;
            auto option = makeOption(
                resonator, spot, secondPort, secondStraight, secondReverse,
                offset, explaining() ? &why : nullptr, 0, 0, &miss);
            for (std::uint32_t round = 0;
                 !option.has_value() && round < COUPLER_SHIFT_ROUNDS &&
                 (miss.first != 0 || miss.second != 0);
                 ++round) {
              shiftX += miss.first;
              shiftY += miss.second;
              if (std::abs(shiftX) > tuning_.couplerShift ||
                  std::abs(shiftY) > tuning_.couplerShift) {
                break;
              }
              miss = {0, 0};
              option = makeOption(
                  resonator, spot, secondPort, secondStraight, secondReverse,
                  offset, explaining() ? &why : nullptr, shiftX, shiftY, &miss);
            }
            if (!option.has_value()) {
              continue;
            }
            explain(std::format(
                "coupler insertion:   '{}' offset {} port {}{}: option {} at "
                "place {} of {} ({:.0f} of {:.0f} cells to the port), "
                "angle {}°, centre ({},{}), {} priced cells",
                wireId(resonator), offset, secondPort ? 2 : 1,
                (secondStraight == 0
                     ? std::string{}
                     : std::format(" jog {}{}", secondStraight,
                                   secondReverse ? " reversed" : "")) +
                    (shiftX == 0 && shiftY == 0
                         ? std::string{}
                         : std::format(" moved ({},{})", shiftX, shiftY)),
                options.size(), tried - 1, places.size(), spot.centreLength,
                spot.wanted, degreesOfHeading(option->couplerOrientation),
                option->centre.x, option->centre.y, option->guarded));
            option->place = static_cast<std::uint32_t>(tried - 1);
            options.push_back(std::move(*option));
            got = true;
            break;
          }
          if (!got && explaining()) {
            explain(std::format(
                "coupler insertion:   '{}' offset {} port {}{}: no place of "
                "{} tried fits — last refusal: {}",
                wireId(resonator), offset, secondPort ? 2 : 1,
                secondStraight == 0
                    ? std::string{}
                    : std::format(" jog {}{}", secondStraight,
                                  secondReverse ? " reversed" : ""),
                tried, why.empty() ? std::string("none offered") : why));
          }
        }
      }
    }
    if (places.empty() && explaining()) {
      explain(std::format(
          "resonator {}: no cell of its way carries the biased target",
          wireId(resonator)));
    }
    place(resonator);
    return options;
  }

  /// The rectangle a coupler may sit in.
  ///
  /// Every launcher closes a stub for the wire that leaves it: the cell, the
  /// straight run it needs and the clearance around both. A coupler placed
  /// beyond one of those stubs puts the chain's feedline on the far side of
  /// it, and the edge into that coupler has to come back around the stub —
  /// which is where the detours came from. The stubs of all the launchers
  /// together leave one open rectangle in the middle of the chip, and inside
  /// it no stub stands between a coupler and the ring.
  ///
  /// The rectangle is the innermost of the stub ends: of each launcher the
  /// point its stub reaches furthest in the direction it faces, projected
  /// onto the axis it bounds.
  struct CouplerBox {
    std::int64_t minX = 0;
    std::int64_t maxX = 0;
    std::int64_t minY = 0;
    std::int64_t maxY = 0;
    [[nodiscard]] bool holds(const std::int64_t x, const std::int64_t y,
                             const std::int64_t margin = 0) const {
      return x >= minX + margin && x <= maxX - margin && y >= minY + margin &&
             y <= maxY - margin;
    }
    [[nodiscard]] bool empty() const { return maxX < minX || maxY < minY; }
  };

  /// The room a chain's edge has no choice about beyond a coupler's feedline
  /// port: the straight run it is forced to make off it, in cells.
  [[nodiscard]] std::int64_t edgeRunOffPort() const {
    return static_cast<std::int64_t>(tuning_.straightStart);
  }

  /// Whether a coupler centred on a cell, in one orientation, keeps both of
  /// its feedline ports and the straight run the chain is forced to make off
  /// each of them inside the box.
  ///
  /// The real coordinates, not a margin around the centre: how far the ports
  /// lie from the centre and in which direction is the orientation's own
  /// business, and a pad lying along the box's long side reaches nowhere
  /// near as far across it as one lying athwart. A single figure for all
  /// eight is pessimistic by the difference, and it cost places that were
  /// perfectly good.
  ///
  /// The run along the far edge is a straight line and the box is convex, so
  /// its two ends decide it.
  [[nodiscard]] bool terminalsInBox(const std::int64_t cx,
                                    const std::int64_t cy,
                                    const Heading orientation,
                                    const Heading leaves) const {
    const std::int64_t halfRun =
        cellsOn(tuning_.couplerLength, orientation) / 2;
    const std::int64_t halfDepth =
        cellsOn(tuning_.couplerHeight, orientation) / 2;
    const auto along = routing::headingVector(orientation);
    // The edge the feedline runs along, picked as `makeOption` picks it: the
    // one away from the side the resonator leaves on.
    const auto leaving = routing::headingVector(leaves);
    auto across = routing::headingVector(routing::turned(orientation, 2));
    if (((across.dx * leaving.dx) + (across.dy * leaving.dy)) > 0) {
      across = routing::headingVector(routing::turned(orientation, -2));
    }
    const auto edgeX = cx + ((halfDepth + 1) * across.dx);
    const auto edgeY = cy + ((halfDepth + 1) * across.dy);
    // The straight run off the port, and the quarter turn the edge has to
    // make at the end of it: a run that ends on the box edge leaves the
    // curve after it nowhere but in a launcher's stub.
    //
    // **The run off a coupler port is not `straightStart`.** The two ends of
    // a chain's edge are forced straight by different figures, and
    // `runsOfEdge` is where they are said: `straightStart` at a launcher, the
    // pad's own run — `cellsOn(couplerLength, orientation)`, 21 cells axial
    // and 15 diagonal — at a coupler. `corridorOfEdge` then leaves
    // `max(tuning_.terminalSlot, run + BEND_RADIUS)` open at each terminal, 26
    // at a coupler against 20 at a launcher. A box drawn on `straightStart` is
    // ten cells short of the room the edge off this port has no choice about,
    // so the same figure is used here. See `edgeRunOffPort`.
    const std::int64_t reach = halfRun + edgeRunOffPort();
    for (const std::int64_t end : {-reach, reach}) {
      if (!couplerBox_.holds(edgeX + (end * along.dx), edgeY + (end * along.dy),
                             tuning_.couplerBoxMargin)) {
        return false;
      }
    }
    return true;
  }

  /// Whether any of the eight orientations fits at a cell. A place needs one
  /// — the greedy is free to take another and reach past the box, as it was
  /// before the box existed. All eight whatever `couplerOffsets` allows:
  /// this picks the places, and an allowed offset may fit a place that only
  /// another offset fits unmoved once the place is moved into the box, so
  /// the options of the allowed offsets are those they are without the
  /// switch.
  [[nodiscard]] bool anyOrientationFits(const std::int64_t cx,
                                        const std::int64_t cy,
                                        const Heading leaves) const {
    for (Heading offset = 0; offset < routing::NUM_HEADINGS; ++offset) {
      const auto orientation =
          static_cast<Heading>((leaves + offset) % routing::NUM_HEADINGS);
      if (terminalsInBox(cx, cy, orientation, leaves)) {
        return true;
      }
    }
    return false;
  }

  /// Close the box of every meander a resonator's way carries, grown by the
  /// wire clearance, to the feedline edges of the coupler insertion: an edge
  /// that runs into a meander's box takes the room the meander needs, and
  /// the resonator then loses its length when it is drawn again.
  void closeMeanders(const std::vector<Wire>& wires) {
    meanderKeepout_ = grid::BitGrid(scene_.router.width, scene_.router.height);
    const auto grow = static_cast<std::int64_t>(tuning_.clearance);
    const auto w = static_cast<std::int64_t>(scene_.router.width);
    const auto h = static_cast<std::int64_t>(scene_.router.height);
    std::uint32_t boxes = 0;
    std::uint64_t cells = 0;
    for (const auto& wire : wires) {
      if (!carriesLength(wire) || !wire.drawn) {
        continue;
      }
      const auto found = meanders_.find(wire.key);
      if (found == meanders_.end()) {
        continue;
      }
      for (const auto& [way, box] : found->second) {
        if (way != wire.way || box.maxX < box.minX) {
          continue;
        }
        ++boxes;
        // Never past the coupler box: what lies outside it is the room the
        // feedline edges need in front of the launchers.
        const auto lowY =
            std::max({std::int64_t{0}, box.minY - grow, couplerBox_.minY});
        const auto highY = std::min({h - 1, box.maxY + grow, couplerBox_.maxY});
        const auto lowX =
            std::max({std::int64_t{0}, box.minX - grow, couplerBox_.minX});
        const auto highX = std::min({w - 1, box.maxX + grow, couplerBox_.maxX});
        for (auto y = lowY; y <= highY; ++y) {
          for (auto x = lowX; x <= highX; ++x) {
            const auto cell = static_cast<std::size_t>((y * w) + x);
            cells += meanderKeepout_.test(cell) ? 0 : 1;
            meanderKeepout_.set(cell, true);
          }
        }
        break;
      }
    }
    say(std::format("the feedline edges keep clear of {} meander boxes, "
                    "grown by the clearance of {} cells: {} cells",
                    boxes, tuning_.clearance, cells));
  }

  /// The rectangle the launcher stubs leave open, from the scene's launchers.
  [[nodiscard]] CouplerBox couplerBoxOf() const {
    CouplerBox box{.minX = 0,
                   .maxX = static_cast<std::int64_t>(scene_.router.width) - 1,
                   .minY = 0,
                   .maxY = static_cast<std::int64_t>(scene_.router.height) - 1};
    // How far a stub reaches from its launcher: the straight run the wire
    // needs, the clearance the stub is inflated by, and one cell more.
    //
    // The one cell matters. The stub's places run to `straightStart` and the
    // clearance disc carries them `clearance` further, so `straightStart +
    // clearance` is the **last closed** cell, not the first open one. A box
    // drawn on it is tangent to every stub, and a coupler port on its edge
    // has its straight run grazing the disc — which is why a path left such
    // a port straight and then swung wide instead of running on.
    //
    // The run here is `launcherStraight`, not `straightStart`: the box keeps
    // the launchers a little more room than their own stubs take, so that an
    // edge into a coupler at the box edge has somewhere to run. See
    // `launcherLead`.
    //
    // And the launcher's run alone is not what the wire off it needs: it is
    // forced straight for `straightStart` and then has to *turn* to leave
    // the channel, which takes `BEND_RADIUS` more.
    const auto run = tuning_.launcherStraight;
    const auto reach = static_cast<std::int64_t>(run + tuning_.clearance) + 1;
    for (const auto& [port, slot] : scene_.launcherCell) {
      const auto found = scene_.launcherHeading.find(port);
      if (found == scene_.launcherHeading.end()) {
        continue;
      }
      const auto place = scene_.router.cell(slot);
      const auto v = routing::headingVector(found->second);
      const auto endX = static_cast<std::int64_t>(place.x()) + (reach * v.dx);
      const auto endY = static_cast<std::int64_t>(place.y()) + (reach * v.dy);
      // A stub that faces along an axis bounds that axis, on the side it
      // faces from. A diagonal one bounds both.
      if (v.dx > 0) {
        box.minX = std::max(box.minX, endX);
      } else if (v.dx < 0) {
        box.maxX = std::min(box.maxX, endX);
      }
      if (v.dy > 0) {
        box.minY = std::max(box.minY, endY);
      } else if (v.dy < 0) {
        box.maxY = std::min(box.maxY, endY);
      }
    }
    return box;
  }

  /// Where a coupler sits on a resonator: one cell of the way, and what the
  /// way still measures from there to the port.
  struct CouplerPlace {
    /// The index into the way the coupler's centre takes.
    std::size_t at = 0;
    /// That cell, for the box the coupler has to sit in, and the heading
    /// the way holds there, which decides which edge the feedline takes.
    std::int64_t x = 0;
    std::int64_t y = 0;
    Heading leaves = 0;
    /// What the way from there to the port came to, in cells, against what
    /// was asked of it.
    double centreLength = 0.0;
    double wanted = 0.0;
  };

  /// The place the coupler takes on a resonator: the point of the way where
  /// what is left to the qubit is `target_resonator_length` times the bias.
  ///
  /// This replaces the prototype's splice. There is no dogleg and no turn to
  /// build: the body is a pad centred on that one point, the way is cut
  /// there, and the orientation is the option's to choose. What is left of
  /// the way is short of the design figure by the bias, which is the room the
  /// meander needs to make it exact afterwards — the meander can only
  /// lengthen a way, never shorten one.
  [[nodiscard]] std::vector<CouplerPlace>
  couplerPlace(const Wire& resonator) const {
    if (resonator.way.size() < 2) {
      return {};
    }
    const auto segmented =
        routing::reconstructSegments(*primitives_, resonator.way);
    if (segmented.segments.empty() ||
        segmented.segments.back().lengthAt.empty()) {
      return {};
    }
    const auto overall = segmented.segments.back().lengthAt.back();
    const double wanted = leftAtCoupler(resonator);
    // Where each cell of the way sits, so a cell of a segment can be named as
    // an index into the way again.
    std::unordered_map<std::uint64_t, std::size_t> indexOf;
    indexOf.reserve(resonator.way.size() * 2);
    for (std::size_t r = 0; r < resonator.way.size(); ++r) {
      const auto key = (static_cast<std::uint64_t>(resonator.way[r].x) << 32U) |
                       resonator.way[r].y;
      indexOf.emplace(key, r);
    }
    std::vector<CouplerPlace> places;
    for (const auto& segment : segmented.segments) {
      for (std::size_t i = 0; i < segment.cells.size(); ++i) {
        const auto left = overall - segment.lengthAt[i];
        const auto key =
            (static_cast<std::uint64_t>(segment.cells[i].x) << 32U) |
            segment.cells[i].y;
        const auto found = indexOf.find(key);
        if (found == indexOf.end() ||
            found->second + 1 >= resonator.way.size()) {
          continue;
        }
        places.push_back(
            CouplerPlace{.at = found->second,
                         .x = static_cast<std::int64_t>(segment.cells[i].x),
                         .y = static_cast<std::int64_t>(segment.cells[i].y),
                         .leaves = segment.cells[i].heading,
                         .centreLength = left,
                         .wanted = wanted});
      }
    }
    // Nearest the asked-for length first, and nothing else. Which places
    // fit is no longer a property of the list: every orientation walks it
    // and stops at the first place *it* fits, so the order only has to say
    // which place is best on length. The list used to be sorted by whether
    // any of the eight orientations fitted, which put one orientation's
    // impossible point ahead of another's perfect one.
    std::ranges::sort(places, [](const CouplerPlace& a, const CouplerPlace& b) {
      const auto one = std::abs(a.centreLength - a.wanted);
      const auto two = std::abs(b.centreLength - b.wanted);
      return one != two ? one < two : a.at < b.at;
    });
    return places;
  }

  /// Where a resonator reaches its target length, on the way it has now: the
  /// index of that cell (`pt`) and the band of indices `lo..hi` whose
  /// remaining length to the target end lies within `tuning_.lengthPointBand`
  /// of the figure. The figure is the target length less the anchor gap, as the
  /// prototype's `meander_length - anchor` is — plainly ±10 % of the target
  /// length around that point, nothing derived (user, 2026-10-04). The pad
  /// the insertion then centres lies `leadLength()` nearer the target end,
  /// well inside the band. The length is measured as the insertion measures
  /// it: by `reconstructSegments` on a drawn way, over the cells' polyline on
  /// a seeded one, which has no primitives of this grid yet. Backwards from
  /// the target end, because the insertion scores every place by what is
  /// left to the qubit. A way shorter than the band's near end collapses
  /// the band onto the point, as the prototype collapses it onto the front.
  struct LengthMarks {
    bool valid = false;
    std::size_t lo = 0;
    std::size_t pt = 0;
    std::size_t hi = 0;
    double wanted = 0.0;
  };

  [[nodiscard]] LengthMarks lengthMarksOf(const Wire& wire) const {
    LengthMarks marks;
    if (!wire.resonator || wire.way.size() < 2) {
      return marks;
    }
    const auto& way = wire.way;
    std::vector<double> at(way.size(), 0.0);
    if (wire.drawn) {
      const auto segmented = routing::reconstructSegments(*primitives_, way);
      std::unordered_map<std::uint64_t, std::size_t> indexOf;
      indexOf.reserve(way.size() * 2);
      const auto keyOf = [](const PathPoint& point) {
        return (static_cast<std::uint64_t>(point.x) << 32U) | point.y;
      };
      for (std::size_t r = 0; r < way.size(); ++r) {
        indexOf.emplace(keyOf(way[r]), r);
      }
      std::vector<double> known(way.size(), -1.0);
      for (const auto& segment : segmented.segments) {
        for (std::size_t i = 0; i < segment.cells.size(); ++i) {
          const auto found = indexOf.find(keyOf(segment.cells[i]));
          if (found != indexOf.end()) {
            known[found->second] = segment.lengthAt[i];
          }
        }
      }
      // The cells an arc sweeps carry no length of their own; they take the
      // last length known before them.
      double last = 0.0;
      for (std::size_t r = 0; r < way.size(); ++r) {
        if (known[r] >= 0.0) {
          last = known[r];
        }
        at[r] = last;
      }
    } else {
      for (std::size_t r = 1; r < way.size(); ++r) {
        at[r] = at[r - 1] +
                std::hypot(static_cast<double>(way[r].x) - way[r - 1].x,
                           static_cast<double>(way[r].y) - way[r - 1].y);
      }
    }
    const double overall = at.back();
    marks.wanted = std::max(0.0, tuning_.targetLength - wire.anchorGap);
    const double pct = tuning_.lengthPointBand;
    const double low = marks.wanted * (1.0 - pct);
    const double high = marks.wanted * (1.0 + pct);
    std::size_t lo = way.size();
    std::size_t hi = 0;
    double nearest = std::numeric_limits<double>::max();
    for (std::size_t r = 0; r < way.size(); ++r) {
      const double left = overall - at[r];
      if (left >= low && left <= high) {
        lo = std::min(lo, r);
        hi = std::max(hi, r);
      }
      const double off = std::abs(left - marks.wanted);
      if (off < nearest) {
        nearest = off;
        marks.pt = r;
      }
    }
    if (lo == way.size()) {
      lo = marks.pt;
      hi = marks.pt;
    }
    marks.lo = lo;
    marks.hi = hi;
    marks.valid = true;
    return marks;
  }

  /// The straight run of the component's resonator lead, in cells. The
  /// coupler is drawn with fourteen; a wire still owes the design rule its
  /// own straight, so the lead is never shorter than that. On a grid of
  /// about ten layout units a cell the rule asks eleven.
  /// How long the component's lead is, in cells: the quarter turn and the
  /// straight after it. Built once on a cardinal heading; a lead on a
  /// diagonal is a little longer, and the place is picked by this one figure
  /// all the same, because the place is settled before the orientation.
  [[nodiscard]] double leadLength() const {
    if (leadLength_ < 0.0) {
      try {
        leadLength_ =
            routing::buildDogleg(*primitives_, 0, 1, leadStraight()).cost;
      } catch (const std::logic_error&) {
        leadLength_ = leadStraight();
      }
    }
    return leadLength_;
  }
  mutable double leadLength_ = -1.0;

  /// A heading in degrees, the way the prototype prints an angle: it
  /// counts eighths and multiplies by 45, so a log of ours can be read
  /// beside a log of its.
  [[nodiscard]] static int degreesOfHeading(const Heading heading) {
    return static_cast<int>(heading) * 45;
  }

  [[nodiscard]] std::uint32_t leadStraight() const {
    return std::max(COUPLER_LEAD_STRAIGHT, tuning_.straightStart) +
           tuning_.couplerLeadMargin;
  }

  /// How many corners of a way no primitive could have made: two cells in a
  /// row whose headings are more than one eighth apart. A way the router
  /// returned holds none.
  [[nodiscard]] static std::uint32_t kinksIn(const Path& way) {
    std::uint32_t kinks = 0;
    for (std::size_t at = 0; at + 1 < way.size(); ++at) {
      if (routing::headingDistance(way[at].heading, way[at + 1].heading) > 1) {
        ++kinks;
      }
    }
    return kinks;
  }

  /// The straight of the second dogleg, in cells. The prototype's twenty.
  static constexpr std::uint32_t COUPLER_SECOND_STRAIGHT = 20;

  /// How often an option that leaves the box is moved and built again (see
  /// `couplerShift`). A round puts the cell that was outside inside, but can
  /// push another one out; the cap ends a move that goes back and forth.
  static constexpr std::uint32_t COUPLER_SHIFT_ROUNDS = 8;

  /// One option at the place the coupler takes: the body as a pad centred on
  /// that cell, turned by one of the eight offsets from the heading the way
  /// holds there. Nothing, where it does not fit.
  ///
  /// `shiftX` and `shiftY` move the whole option, so that the lead's tip
  /// lands that far off the place (see `couplerShift`). When the option is
  /// refused because a cell leaves the box, `boxMiss` receives the move
  /// that would bring that cell inside it.
  [[nodiscard]] std::optional<CouplerOption>
  makeOption(const Wire& resonator, const CouplerPlace& place,
             const bool secondPort, const std::uint32_t secondStraight,
             const bool secondReverse, const Heading offset,
             std::string* why = nullptr, const std::int64_t shiftX = 0,
             const std::int64_t shiftY = 0,
             std::pair<std::int64_t, std::int64_t>* boxMiss = nullptr) const {
    const auto refuse =
        [&why](std::string reason) -> std::optional<CouplerOption> {
      if (why != nullptr) {
        *why = std::move(reason);
      }
      return std::nullopt;
    };
    // What a cell outside the box lacks on each axis to be inside it.
    const auto missAt = [&](const std::int64_t x, const std::int64_t y) {
      if (boxMiss == nullptr) {
        return;
      }
      const auto margin = tuning_.couplerBoxMargin;
      const auto lacks = [margin](const std::int64_t at, const std::int64_t low,
                                  const std::int64_t high) -> std::int64_t {
        if (at < low + margin) {
          return low + margin - at;
        }
        return at > high - margin ? high - margin - at : 0;
      };
      *boxMiss = {lacks(x, couplerBox_.minX, couplerBox_.maxX),
                  lacks(y, couplerBox_.minY, couplerBox_.maxY)};
    };
    CouplerOption option;
    option.offset = offset;
    option.shiftX = shiftX;
    option.shiftY = shiftY;
    // The way is cut at the place. Nothing is spliced in front of it: the
    // resonator simply runs out of the pad it ends on.
    option.way.assign(resonator.way.begin() +
                          static_cast<std::ptrdiff_t>(place.at),
                      resonator.way.end());
    if (option.way.size() < 2) {
      return refuse("what is left of the way is too short to draw");
    }
    // The place is where the **tip of the lead** has to land — the end of
    // the arc and the straight after it. Neither the pad's centre nor the
    // resonator's port is this cell any more; both are worked back from it
    // once the lead is built.
    // The orientation: the heading the way holds at the place, turned by the
    // offset. This is the one thing an option chooses, and it is what lets
    // the coupler face the chain rather than inherit the direction the
    // resonator happens to run in — without it the pad on a diagonal run
    // takes a diagonal approach the chain cannot make, and edges go
    // undrawn.
    // At offset 0 the coupler's own orientation — the direction the
    // straight after the mandatory arc runs — is the heading the nearest
    // launcher faces, so the resonator leaves the coupler pointing the way
    // that launcher points. The pad's axis is a quarter turn back from
    // that, because the arc turns a quarter from the port onto the
    // orientation.
    const auto facing =
        routing::turned(nearestLauncherHeading(place.x, place.y), -2);
    option.orientation =
        static_cast<Heading>((facing + offset) % routing::NUM_HEADINGS);
    option.run = cellsOn(tuning_.couplerLength, option.orientation);
    option.depth = cellsOn(tuning_.couplerHeight, option.orientation);
    // The body: a pad centred on the anchor, the run along the orientation
    // and the depth across it. In cells: a cell is in the body when its
    // projections onto the two step vectors lie within half the run and half
    // the depth. The feedline runs along the far edge, parallel to the run.
    const auto along = routing::headingVector(option.orientation);
    // Which long edge of the pad the feedline runs along: the one away from
    // the side the resonator leaves on, so that the resonator never has to
    // cross its own feedline to get to its qubit. The pad straddles the
    // centre, so both edges exist either way; what this picks is the pair of
    // feedline ports, and picking the near one put every resonator on the
    // same side as the chain that drives it.
    // The side the feedline takes is not guessed any more, it is derived.
    // It used to be picked by a heuristic — the edge away from the side the
    // resonator happened to leave on — and the legality rule below then
    // demanded one particular side, so five of the eight offsets were built
    // and thrown away again for no geometric reason: on 4q the rule alone
    // refused forty of sixty-four candidates, on 17q two hundred and forty
    // of four hundred and sixteen. Taking the side the rule asks for makes
    // every offset legal and leaves the rule an invariant.
    const auto acrossHeading = routing::turned(option.orientation, -2);
    const auto across = routing::headingVector(acrossHeading);

    // The coupler's orientation. Think of the pad as a rectangle with the
    // line between its two feedline ports along one long edge and the line
    // between its two resonator ports along the other: the orientation is
    // the perpendicular from the feedline line to the resonator line. It is
    // the direction the coupler points its resonator in, and the straight
    // run after the arc lies on it.
    option.couplerOrientation = routing::reverse(acrossHeading);

    // Which options are legal. Both feedline edges meet this coupler at the
    // same angle — one arrives on it, the other leaves on it — and that
    // angle is the pad's own axis, which both feedline ports carry. The
    // coupler has to point its resonator a quarter turn with it. Which long
    // edge the feedline takes is decided above from the heading the way
    // holds at the place, so half the offsets come out pointing the other
    // way: those are the options that would send the resonator across its
    // own feedline, and they are refused here rather than priced.
    if (const auto legal = routing::turned(option.orientation, 2);
        option.couplerOrientation != legal) {
      return refuse(std::format(
          "the coupler points its resonator on {} where the feedline angle "
          "{} makes {} the only legal one",
          option.couplerOrientation, option.orientation, legal));
    }
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto height = static_cast<std::int64_t>(scene_.router.height);
    const std::int64_t halfRun = option.run / 2;
    const std::int64_t halfDepth = option.depth / 2;

    // Where the coupler sits. What has to land on the resonator's path is
    // not the pad's centre but the **tip of the straight run after the
    // arc** — the insertion point is the end of the component's own lead,
    // not the middle of its pad. So the lead is built first, in a frame
    // whose origin is the resonator port, and the port is the place less
    // the whole lead; the pad then hangs off that port.
    //
    // The port the resonator leaves on, and which way its arc turns: the
    // two ports face opposite ways along the pad, so one reaches the
    // coupler's orientation with the clock and the other against it.
    option.secondPort = secondPort;
    const auto portHeading =
        secondPort ? routing::reverse(option.orientation) : option.orientation;
    option.secondStraight = secondStraight;
    option.secondReverse = secondReverse;
    routing::DoglegGeometry lead;
    routing::DoglegGeometry jog;
    Heading afterFirst = 0;
    try {
      const auto turnSign =
          routing::turned(portHeading, 2) == option.couplerOrientation ? 1 : -1;
      lead = routing::buildDogleg(*primitives_, portHeading, turnSign,
                                  leadStraight());
      afterFirst = lead.tip.heading;
      // The second dogleg, when the option asks for one: another quarter
      // turn off the orientation and another straight. It turns the lead a
      // second time, so the heading it finally meets the path on is **not**
      // the coupler's orientation any more — the orientation is what the
      // component points the resonator in, which is settled by the first
      // arc, and the second is a jog in front of it that moves where the
      // pad has to sit for that lead to land on the way. Reversed it jogs
      // one way, with the first turn the other.
      if (secondStraight > 0) {
        jog = routing::buildDogleg(*primitives_, afterFirst,
                                   secondReverse ? -turnSign : turnSign,
                                   secondStraight);
      }
    } catch (const std::logic_error&) {
      return refuse("the primitives hold no quarter turn for the lead");
    }
    // A moved option: the lead's tip lands off the place, and the resonator
    // runs from the tip to its way in a straight line, the connector. It
    // joins the way at the first cell at least `BEND_RADIUS` cells in front
    // of the tip on the heading the tip holds, so that it leaves the tip
    // forwards and not back along the lead. The way between the place and
    // that cell is dropped. The routing draws the resonator again from the
    // tip; this way is where it starts.
    std::size_t connector = 0;
    if (shiftX != 0 || shiftY != 0) {
      const auto tipX = place.x + shiftX;
      const auto tipY = place.y + shiftY;
      if (tipX < 0 || tipY < 0 ||
          tipX >= static_cast<std::int64_t>(scene_.router.width) ||
          tipY >= static_cast<std::int64_t>(scene_.router.height)) {
        return refuse("the moved tip of the lead leaves the grid");
      }
      const auto ahead = routing::headingVector(
          secondStraight > 0 ? jog.tip.heading : lead.tip.heading);
      const auto reach = static_cast<std::int64_t>(BEND_RADIUS) *
                         ((ahead.dx * ahead.dx) + (ahead.dy * ahead.dy));
      auto join = place.at;
      while (
          join + 1 < resonator.way.size() &&
          ((static_cast<std::int64_t>(resonator.way[join].x) - tipX) *
           ahead.dx) +
                  ((static_cast<std::int64_t>(resonator.way[join].y) - tipY) *
                   ahead.dy) <
              reach) {
        ++join;
      }
      if (join + 1 >= resonator.way.size()) {
        return refuse("no cell of the way lies in front of the moved tip");
      }
      Path way{{.x = static_cast<std::uint32_t>(tipX),
                .y = static_cast<std::uint32_t>(tipY),
                .heading = 0,
                .primitive = 0}};
      connect(way, resonator.way[join]);
      connector = way.size() - 1;
      for (std::size_t at = 0; at < connector; ++at) {
        way[at].heading = headingOfStep(way[at], way[at + 1]);
      }
      way.back() = resonator.way[join];
      way.insert(way.end(),
                 resonator.way.begin() + static_cast<std::ptrdiff_t>(join + 1),
                 resonator.way.end());
      option.way = std::move(way);
    }
    // A resonator that only feeds a bridge has no length to hold, so no
    // place is refused for what is left of it.
    if (!resonator.feedsABridge) {
      // What is left runs to the qubit. Only an overshoot is refused: nothing
      // spliced in later makes a way shorter, while a way short of the figure
      // is what the meander is for, and is priced by how much it has to add.
      const double target =
          std::max(0.0, tuning_.targetLength - resonator.anchorGap);
      // The resonator measures the lead as well as what is left, as the
      // shortfall test below says; without the lead a place up to one lead
      // too long passed.
      const auto left = lengthOf(option.way);
      if (left + leadLength() > target + tuning_.lengthTolerance) {
        return refuse("what is left of the way is longer than the target");
      }
      // **And it may not be far shorter either.**
      //
      // The figure to hold against is what the resonator will actually
      // measure: the lead the component leaves the pad on, what is left to the
      // qubit, and the run from the last cell to the port. `left` is the
      // middle term alone — the lead is spliced on at the end of this function
      // — so the lead has to be added back, and it matters that it is: it is
      // an absolute length, about 290 layout units, which is a twentieth of a
      // 6000-unit target and an eighth of a 2500-unit one. Measured without
      // it, the same share would refuse on the small chips the very place
      // `couplerPlace` aims at.
      const auto whole = left + leadLength() + resonator.anchorGap;
      const auto least =
          tuning_.targetLength * (1.0 - tuning_.couplerMaxShortfall);
      if (whole < least) {
        return refuse(std::format(
            "what is left of the resonator is {:.0f} against a target of "
            "{:.0f}, past the {:.0f}% a coupler may take off",
            whole, tuning_.targetLength, tuning_.couplerMaxShortfall * 100.0));
      }
      if (left < target - tuning_.lengthTolerance) {
        option.guarded +=
            static_cast<std::uint32_t>(target - tuning_.lengthTolerance - left);
      }
    }

    // Where the whole lead ends, as an offset from the port: the first
    // dogleg, and the second laid on its tip.
    const auto tipX =
        static_cast<std::int64_t>(static_cast<std::int32_t>(lead.tip.x)) +
        (secondStraight > 0
             ? static_cast<std::int64_t>(static_cast<std::int32_t>(jog.tip.x))
             : 0);
    const auto tipY =
        static_cast<std::int64_t>(static_cast<std::int32_t>(lead.tip.y)) +
        (secondStraight > 0
             ? static_cast<std::int64_t>(static_cast<std::int32_t>(jog.tip.y))
             : 0);
    const auto portX = place.x - tipX + shiftX;
    const auto portY = place.y - tipY + shiftY;
    // The port is one end of the near edge; the centre lies half a depth
    // across from it and half a run back along the pad, on the side the
    // other port is.
    const std::int64_t towards = secondPort ? 1 : -1;
    const auto originX =
        portX + ((halfDepth + 1) * across.dx) + (towards * halfRun * along.dx);
    const auto originY =
        portY + ((halfDepth + 1) * across.dy) + (towards * halfRun * along.dy);
    if (originX < 0 || originY < 0 || originX >= scene_.router.width ||
        originY >= scene_.router.height) {
      return refuse("the pad leaves the grid");
    }
    option.centre = {.x = static_cast<std::uint32_t>(originX),
                     .y = static_cast<std::uint32_t>(originY),
                     .heading = option.orientation,
                     .primitive = 0};
    const std::int64_t alongNorm =
        (along.dx * along.dx) + (along.dy * along.dy);
    const std::int64_t acrossNorm =
        (across.dx * across.dx) + (across.dy * across.dy);
    const auto reach = halfRun + halfDepth + 1;
    if (originX - reach < 0 || originY - reach < 0 ||
        originX + reach >= width || originY + reach >= height) {
      return refuse("the body leaves the grid");
    }
    const char* blocker = "";
    const auto free = [&](const std::size_t cell) {
      if (scene_.blocked.test(cell)) {
        blocker = "artwork";
        return false;
      }
      if (approaches_.test(cell)) {
        blocker = "a port's approach";
        return false;
      }
      // A body already placed is not a refusal, and neither is another wire's
      // copper: every wire is drawn again under the feedline constraints.
      // Both are priced far above a wire's room.
      option.guarded += bodies_.test(cell) ? 20 : 0;
      const auto owner = field_.owner(cell);
      option.guarded += (owner != NO_OWNER && owner != resonator.key) ? 20 : 0;
      option.guarded += field_.guarded(cell) ? 1 : 0;
      return true;
    };
    for (std::int64_t y = originY - reach; y <= originY + reach; ++y) {
      for (std::int64_t x = originX - reach; x <= originX + reach; ++x) {
        const auto dx = x - originX;
        const auto dy = y - originY;
        const auto onRun = (dx * along.dx) + (dy * along.dy);
        const auto onDepth = (dx * across.dx) + (dy * across.dy);
        if (std::abs(onRun) > halfRun * alongNorm ||
            std::abs(onDepth) > halfDepth * acrossNorm) {
          continue;
        }
        // The pad's own copper. Nothing tested this against the box. The
        // run is on the pad's far edge, so an axial body whose run is inside
        // is inside too — but a diagonal one turns its corners out, and the
        // rule is about the copper and not about the four axial cases.
        if (!couplerBox_.holds(x, y, tuning_.couplerBoxMargin)) {
          missAt(x, y);
          return refuse(
              std::format("the body leaves the box at ({},{})", x, y));
        }
        const auto cell = static_cast<std::size_t>((y * width) + x);
        if (!free(cell)) {
          return refuse(std::format("the body meets {}", blocker));
        }
        option.body.push_back(cell);
      }
    }
    if (option.body.empty()) {
      return refuse("the body has no cells");
    }
    const std::unordered_set<std::size_t> pad(option.body.begin(),
                                              option.body.end());

    // **What is left of the resonator may not run back through the pad.**
    //
    // The way left runs from the insertion point to the qubit, and the pad
    // hangs off the far end of the component's lead, so a way that re-enters
    // it has folded back over the coupler it is being given. Nothing refused
    // that until now: `free` above prices another wire's copper but exempts
    // the resonator's own (`owner != resonator.key`), and the
    // self-intersection test below asks whether the path meets *itself*, not
    // whether it meets the body. The option is not buildable, and it is the
    // two feedline edges that have to reach the pad which pay for it — the
    // pad they must arrive at has the resonator lying across it.
    if (std::ranges::any_of(option.way, [&](const PathPoint& cell) {
          return pad.contains(static_cast<std::size_t>(
              (static_cast<std::int64_t>(cell.y) * width) +
              static_cast<std::int64_t>(cell.x)));
        })) {
      return refuse("what is left of the resonator runs through the pad");
    }

    // Where the feedline meets it: the two ends of the far edge, a port pair
    // whose line is parallel to the pad. The chain arrives at one end, runs
    // the length of the pad — which is the coupling — and leaves at the
    // other. Both carry the pad's orientation, so the feedline runs straight
    // along it and the body drawn in the layout is parallel to the pair.
    const auto edgeX = originX + ((halfDepth + 1) * across.dx);
    const auto edgeY = originY + ((halfDepth + 1) * across.dy);
    const auto endOf = [&](const std::int64_t sign) {
      return PathPoint{
          .x = static_cast<std::uint32_t>(edgeX + (sign * halfRun * along.dx)),
          .y = static_cast<std::uint32_t>(edgeY + (sign * halfRun * along.dy)),
          .heading = option.orientation,
          .primitive = 0};
    };
    option.in = endOf(-1);
    option.out = endOf(1);
    // The pair itself, and the stub the chain runs straight on beyond each
    // end, have to be free: the edge that arrives has nowhere else to come
    // from and the edge that leaves nowhere else to go. These cells are the
    // feedline's terminal stubs, and the resonator's own head is let through
    // them below — they are what the coupling is made of.
    //
    // Both ports, the run the chain is forced to make off each of them and
    // the quarter turn at the end of that run have to be in the box. The
    // place was chosen so that at least one orientation manages it; the ones
    // that do not are refused here, or the greedy would take a port past a
    // launcher's stub and the edge into it would have to come back around —
    // the detour the box exists to stop. The run is a straight line and the
    // box is convex, so its two ends decide it, and these are the same two
    // ends `terminalsInBox` tested when the place was picked.
    {
      const std::int64_t beyond = halfRun + edgeRunOffPort();
      for (const std::int64_t end : {-beyond, beyond}) {
        const auto x = edgeX + (end * along.dx);
        const auto y = edgeY + (end * along.dy);
        if (!couplerBox_.holds(x, y, tuning_.couplerBoxMargin)) {
          missAt(x, y);
          return refuse(std::format(
              "the feedline's run along the pad leaves the box at ({},{})", x,
              y));
        }
      }
    }
    std::unordered_set<std::size_t> stubs;
    for (std::int64_t k = -halfRun - tuning_.straightStart;
         k <= halfRun + tuning_.straightStart; ++k) {
      const auto x = edgeX + (k * along.dx);
      const auto y = edgeY + (k * along.dy);
      if (x < 0 || y < 0 || x >= width || y >= height) {
        return refuse("the feedline's run along the pad leaves the grid");
      }
      const auto cell = static_cast<std::size_t>((y * width) + x);
      if (!free(cell)) {
        return refuse(
            std::format("the feedline's run along the pad meets {}", blocker));
      }
      stubs.insert(cell);
    }
    // The edge search holds the feedline to the pad's own run off each port
    // (`runsOfEdge`), which is longer than the stub tested above: 21 cells
    // against 11 on the benchmarks. An option whose forced run meets the
    // artwork or a port's approach there can never be routed.
    {
      const auto forced = static_cast<std::int64_t>(option.run);
      for (std::int64_t k =
               static_cast<std::int64_t>(tuning_.straightStart) + 1;
           k <= forced; ++k) {
        for (const std::int64_t sign : {-1, 1}) {
          const auto x = edgeX + (sign * (halfRun + k) * along.dx);
          const auto y = edgeY + (sign * (halfRun + k) * along.dy);
          if (x < 0 || y < 0 || x >= width || y >= height) {
            return refuse("the feedline's forced run leaves the grid");
          }
          const auto cell = static_cast<std::size_t>((y * width) + x);
          if (scene_.components.test(cell) || approaches_.test(cell)) {
            return refuse("the feedline's forced run meets the artwork or a "
                          "port's approach");
          }
        }
      }
    }
    // The connector of a moved option is a new line, not a routed way, so
    // nothing has kept it off the artwork, a port's approach or this
    // coupler's own feedline stubs.
    for (std::size_t at = 1; at <= connector; ++at) {
      const auto cell = static_cast<std::size_t>(
          (static_cast<std::int64_t>(option.way[at].y) * width) +
          static_cast<std::int64_t>(option.way[at].x));
      if (scene_.blocked.test(cell) || approaches_.test(cell) ||
          stubs.contains(cell)) {
        return refuse("the connector of the moved tip meets the artwork, a "
                      "port's approach or the feedline's run");
      }
    }

    // The resonator ports: the two ends of the near edge, parallel to the
    // feedline pair and opposite it, so a coupler carries four ports in
    // all. Which way the resonator turns off the pad is the difference
    // between them — with the clock off the first, against it off the
    // second. The two are mirror images: each faces outward along the pad,
    // away from the other, and each turns the way that brings its run to the
    // same heading — the first with the clock off the orientation, the
    // second against it off the reverse. Both therefore satisfy the rule
    // below, and which one a coupler takes is an option, not a constant. On
    // 9q one coupler of nine wants the second, and a fixed choice gave it
    // the first.
    const auto nearX = originX - ((halfDepth + 1) * across.dx);
    const auto nearY = originY - ((halfDepth + 1) * across.dy);
    const auto resonatorPort = [&](const std::int64_t sign) {
      return std::pair{nearX + (sign * halfRun * along.dx),
                       nearY + (sign * halfRun * along.dy)};
    };
    {
      const auto [firstX, firstY] = resonatorPort(1);
      option.resonatorIn = {.x = static_cast<std::uint32_t>(firstX),
                            .y = static_cast<std::uint32_t>(firstY),
                            .heading = option.orientation,
                            .primitive = 0};
      const auto [otherX, otherY] = resonatorPort(-1);
      option.resonatorOut = {.x = static_cast<std::uint32_t>(otherX),
                             .y = static_cast<std::uint32_t>(otherY),
                             .heading = routing::reverse(option.orientation),
                             .primitive = 0};
    }
    const auto& leaveFrom =
        secondPort ? option.resonatorOut : option.resonatorIn;

    // The head of the resonator: the lead of the component itself, already
    // built above because the pad was placed from it. It is fixed — it
    // belongs to the coupler and not to the routing — and the search that
    // draws the rest of the resonator starts on its end, which is the
    // insertion point on the path.
    // The rule that ties the coupler to its chain. Both feedline edges meet
    // this coupler at the same angle — the one edge arrives on it, the
    // other leaves on it — and that angle is what the feedline ports carry.
    // The straight run has to have come out on the orientation; if it did
    // not, the turn and the orientation disagree and the option is not the
    // component.
    if (lead.tip.heading != option.couplerOrientation) {
      return refuse("the first arc does not reach the coupler's "
                    "orientation");
    }
    // The tip is not part of the arc: the search begins on it and puts the
    // arc in front of what it finds, so a tip in both would put one cell in
    // the way twice and the self-intersection test would throw it away.
    //
    // Both pieces of the lead, walked as one: the first dogleg from the
    // port, and the second laid on its tip. The second's cells are offset
    // by the first's tip, which is how `buildDogleg` hands them back — in a
    // frame whose origin is the turn it starts on.
    std::vector<std::pair<PathPoint, bool>> leadCells;
    leadCells.reserve(lead.path.size() + jog.path.size());
    for (const auto& move : lead.path) {
      leadCells.emplace_back(move, false);
    }
    for (const auto& move : jog.path) {
      leadCells.emplace_back(move, true);
    }
    const auto jogX =
        static_cast<std::int64_t>(static_cast<std::int32_t>(lead.tip.x));
    const auto jogY =
        static_cast<std::int64_t>(static_cast<std::int32_t>(lead.tip.y));

    option.arc.clear();
    option.arc.reserve(leadCells.size());
    PathPoint tip{};
    for (std::size_t step = 0; step < leadCells.size(); ++step) {
      const auto& [move, onJog] = leadCells[step];
      const auto x =
          portX + (onJog ? jogX : 0) +
          static_cast<std::int64_t>(static_cast<std::int32_t>(move.x));
      const auto y =
          portY + (onJog ? jogY : 0) +
          static_cast<std::int64_t>(static_cast<std::int32_t>(move.y));
      if (x < 0 || y < 0 || x >= width || y >= height) {
        return refuse("the resonator's lead leaves the grid");
      }
      if (!couplerBox_.holds(x, y, tuning_.couplerBoxMargin)) {
        missAt(x, y);
        return refuse(std::format(
            "the resonator's lead leaves the box at ({},{})", x, y));
      }
      const auto cell = static_cast<std::size_t>((y * width) + x);
      // The lead runs off the pad's own near edge, so the pad is not in its
      // way; everything else is.
      if (!pad.contains(cell) && !free(cell)) {
        return refuse(std::format("the resonator's lead meets {}", blocker));
      }
      const PathPoint here{.x = static_cast<std::uint32_t>(x),
                           .y = static_cast<std::uint32_t>(y),
                           .heading = move.heading,
                           .primitive = move.primitive};
      if (step + 1 < leadCells.size()) {
        option.arc.push_back(here);
      } else {
        tip = here;
      }
    }
    if (option.arc.empty()) {
      return refuse("the lead has no cells");
    }
    // `lead.path` ends on the first tip, which the second dogleg starts on.
    option.firstLead = secondStraight > 0 && !lead.path.empty()
                           ? lead.path.size() - 1
                           : option.arc.size();
    option.anchor = leaveFrom;
    option.arcEnd = tip;
    // **The resonator's own turn.** The lead already carries the straight
    // run the rule asks for off the pad — it is the quarter turn and
    // `max(COUPLER_LEAD_STRAIGHT, straightStart)` — and the search begins
    // on its tip. The first thing that search may want to do is bend, and a
    // tip on the box edge leaves it no room to, exactly as a feedline port
    // there leaves its edge none. So the `BEND_RADIUS` cells beyond the tip,
    // on the heading the tip holds, are in the rule too.
    // The way the option carries is the lead **and what is left to the
    // qubit**: the arc, the straight after it, and the old route onward
    // from the insertion point.
    //
    // The two join without a jump, which they did not always. While the
    // pad sat on the path and the lead hung off it, the lead ended nowhere
    // near the route and stitching them made a way that read as drawn and
    // measured as nonsense; the way was the lead alone for that reason.
    // Now the way left starts on the lead's tip — the insertion point, or
    // for a moved option the moved tip — so the join is exact, and a
    // resonator that finds nothing under the feedline constraints keeps a
    // way that is a whole resonator rather than a stub.
    Path spliced = option.arc;
    spliced.push_back(tip);
    spliced.insert(spliced.end(), option.way.begin() + 1, option.way.end());
    if (routing::pathSelfIntersects(spliced, scene_.router.width,
                                    scene_.router.height, loopScratch_)) {
      return refuse("the spliced way meets itself");
    }
    option.way = std::move(spliced);
    return option;
  }

  /// The corridor of a feedline edge: everything free within the reach of
  /// the line between its two ends, less the bodies and the ways of the
  /// couplers it runs between. The prototype's `expand_path` of 150 cells
  /// around the two endpoints.
  /// The way an edge of a chain has now: its wire's, once it has one, or
  /// what the greedy last chose for it.
  [[nodiscard]] const Path& edgeWayOf(const std::vector<Wire>& wires,
                                      const std::uint32_t chain,
                                      const std::size_t at) const {
    const auto key = edgeWireOf_[chain][at];
    if (key != NO_OWNER && wires[key].drawn) {
      return wires[key].way;
    }
    return chainEdgePaths_[chain][at];
  }

  /// How far past the two ends of an edge its search may roam, in cells.
  static constexpr std::uint32_t EDGE_BOX_MARGIN = 250;

  /// How many edges at each end of a chain the prefix search routes again for
  /// every prefix rather than remembering by the pair of options at their two
  /// ends. The edges in between are remembered and tested against the prefix
  /// fence — see `Driver::solveChainAStar`.
  static constexpr std::size_t CHAIN_FRESH_EDGES = 2;

  /// Every edge of every chain but the one being routed stands in its way,
  /// inflated by the clearance — the edges that share a coupler with it
  /// included, right up to the port they share.
  ///
  /// Whether edge `at` of chain `chain` runs from or to a launcher. A
  /// feedline with a reflector has one launcher only (user, 2026-10-11):
  /// the edge at its reflector end runs between two couplers and is no
  /// terminal edge, though it is the first or the last of its chain.
  [[nodiscard]] bool atLauncher(const std::uint32_t chain,
                                const std::size_t at) const {
    const auto& points = chains_[chain];
    return at + 1 < points.size() && (points[at].fixed || points[at + 1].fixed);
  }

  /// Nothing is exempt. The direct neighbours of a path were once the one
  /// pair of feedlines nothing closed, and an edge weighed against them ran
  /// through them.
  ///
  /// What it fences is `edgeWayOf`: the edge's wire once it has one, else
  /// what the greedy last chose for it. So this is the one thing in
  /// `corridorOfEdge` that is **not** a function of the two endpoint options
  /// alone — it is what ties an edge's price to every other edge on the chip.
  void fenceCommittedEdges(const std::vector<Wire>& wires, const Edge& edge) {
    // While a chain is searching itself, its own edges may be asked to stand
    // out of the way. What they hold is the answer of the round before, and
    // fencing an edge with the last round's idea of its own neighbours is
    // what makes the rounds a walk that can circle rather than a descent.
    // The commit fences the chain fully and re-routes what no longer fits,
    // so nothing ships through a way this leaves open.
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto& stencil = stencilFor(tuning_.clearance);
    sameChainWays_.clear();
    std::size_t fenced = 0;
    std::size_t ways = 0;
    std::size_t neighbours = 0;
    for (std::uint32_t chain = 0; chain < chains_.size(); ++chain) {
      if (chain != edge.chain) {
        // Another chain's feedline stands open: an edge keeps clear of the
        // edges of its own feedline only. `fenceEverything_` fences the
        // edge's own chain whole and leaves the others open as well.
        continue;
      }
      for (std::size_t at = 0; at < chainEdgePaths_[chain].size(); ++at) {
        if (chain == edge.chain && at == edge.from) {
          continue;
        }
        // The segment the targeted repair is re-searching stands open: its
        // edges are the ones being laid again, and `edgeWayOf` would fence
        // them with the stale ways their wires still hold (2026-10-04).
        if (chain == openChain_ && at >= openLo_ && at < openHi_) {
          continue;
        }
        // An edge of this chain that comes after this one stands open, so
        // that the commit asks what the search answered — **except the one
        // next to it**, which shares a coupler with it (user, 2026-10-03).
        // Two edges that meet at a coupler may not cross each other, and
        // skipping every later edge skipped that one too: the edge leaving
        // a coupler and the edge arriving at it were drawn through one
        // another.
        if (chain == edge.chain && !fenceEverything_ && at > edge.from &&
            at != edge.from + 1) {
          continue;
        }
        // Counted only for the diagnostic: an edge that meets this one at a
        // coupler is fenced by the full clearance like every other feedline.
        // The disc that used to be left open there was the one place two
        // feedlines could come closer than the rule allows.
        const bool adjacent =
            chain == edge.chain && (at + 1 == edge.from || at == edge.from + 1);
        neighbours += adjacent ? 1 : 0;
        const auto& way = edgeWayOf(wires, chain, at);
        if (way.empty()) {
          continue;
        }
        ++ways;
        if (chain == edge.chain) {
          sameChainWays_.push_back(&way);
        }
        alongDisc(
            way, stencil, [&](const std::int64_t x, const std::int64_t y) {
              corridor_.set(static_cast<std::size_t>((y * width) + x), true);
              ++fenced;
            });
      }
    }
    tell(std::format("fence for edge c{}-{}to{}: {} ways ({} neighbours), "
                     "{} cells closed",
                     edge.chain, edge.from, edge.to, ways, neighbours, fenced));
  }

  /// The way a resonator stands on: its own once it has been drawn from its
  /// coupler, and what its coupler's chosen option carries before that.
  [[nodiscard]] const Path& wayOfResonator(const Wire& wire) const {
    const auto owner = couplerOfWire_.find(wire.key);
    if (owner != couplerOfWire_.end() &&
        !(wire.couplerAtSource == owner->second && wire.drawn)) {
      const auto& own = couplers_[owner->second];
      return own.options[own.chosen].way;
    }
    return wire.way;
  }

  /// The room of the coupler leads that do **not** hang off the chain being
  /// routed, as one grid of bits.
  ///
  /// Why it is built per chain rather than per edge: a lead moves when its
  /// coupler takes another option, and while a chain is solved the only
  /// couplers that move are its own. Everything else stands, so one grid
  /// serves every edge search of that solve — thousands of them on 69q.
  void ensureForeignRoom(const std::vector<Wire>& wires,
                         const std::uint32_t chain) {
    if (!foreignRoomStale_ && foreignRoomChain_ == chain &&
        foreignRoom_.size() == static_cast<std::size_t>(scene_.router.width) *
                                   scene_.router.height) {
      return;
    }
    foreignRoom_ = grid::BitGrid(scene_.router.width, scene_.router.height);
    // Which couplers belong to the chain being routed, so their resonators
    // can be left out.
    std::unordered_set<std::uint32_t> mine;
    if (chain < chains_.size()) {
      for (const auto& point : chains_[chain]) {
        if (!point.fixed) {
          mine.insert(point.coupler);
        }
      }
    }
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto& stencil = stencilFor(tuning_.clearance);
    for (std::uint32_t index = 0; index < couplers_.size(); ++index) {
      if (mine.contains(index)) {
        continue;
      }
      const auto& coupler = couplers_[index];
      const auto& option = coupler.options[coupler.chosen];
      const auto& way = wayOfResonator(wires[coupler.wire]);
      const auto close = [&](const std::int64_t x, const std::int64_t y) {
        foreignRoom_.set(static_cast<std::size_t>((y * width) + x), true);
      };
      // The lead carries the clearance.
      alongDisc(option.arc, stencil, close);
      // What follows it carries only its own copper: not crossable, but no
      // room kept. See `RESONATOR_COPPER`.
      alongDisc(way, stencilFor(RESONATOR_COPPER), close,
                option.arc.empty() ? 0 : option.arc.size() - 1);
      // And the pad itself, copper only.
      //
      // `bodies_` is empty for the whole of the option search — it is filled
      // by `applyOption`, and that runs at the commit — so the search was
      // solving against a chip with no coupler pads on it at all, while the
      // commit builds on one with every pad standing. On 45q that cost edge
      // f47 of chain 7: a tight turn between two neighbouring couplers that
      // the search found and the commit could not build, which is what
      // `stateFaults` was reporting as the one edge that would not survive
      // (user, 2026-10-02).
      for (const auto cell : option.body) {
        foreignRoom_.set(cell, true);
      }
    }
    // The run into every outer wire's target port, the rule's straight
    // length, carries the clearance too (user, 2026-10-13): no edge is
    // drawn over a port a wire has to reach. The runs do not move, so they
    // stand in the grid of every chain.
    for (const auto& wire : wires) {
      if (wire.feedline || wire.inner || !wire.feasible) {
        continue;
      }
      alongDisc(router_.straightStub(wire.objective.target, true,
                                     tuning_.straightStart),
                stencil, [&](const std::int64_t x, const std::int64_t y) {
                  foreignRoom_.set(static_cast<std::size_t>((y * width) + x),
                                   true);
                });
    }
    foreignRoomChain_ = chain;
    foreignRoomStale_ = false;
  }

  /// Every resonator stands in the way of an edge, inflated by the
  /// clearance: a feedline comes no closer to a resonator than the rule
  /// allows, except to the resonators of the two couplers the edge runs
  /// between, within the couplers' reach, where the coupling itself is the
  /// closeness. A resonator's way is the way it has once it is cut back to
  /// its coupler, or its coupler's chosen option while the options are
  /// still being weighed.
  void fenceResonators(const std::vector<Wire>& wires, const Edge& edge) {
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto& stencil = stencilFor(tuning_.clearance);
    const auto& chain = chains_[edge.chain];
    std::vector<std::pair<std::uint32_t, PathPoint>> exempt;
    for (const auto at : {edge.from, edge.to}) {
      if (!chain[at].fixed) {
        const auto& coupler = couplers_[chain[at].coupler];
        exempt.emplace_back(coupler.wire,
                            coupler.options[coupler.chosen].anchor);
      }
    }
    for (const auto& wire : wires) {
      if (!wire.resonator || !wire.feasible) {
        continue;
      }
      const Path* way = &wire.way;
      const auto coupler = couplerOfWire_.find(wire.key);
      if (coupler != couplerOfWire_.end() &&
          !(wire.couplerAtSource == coupler->second && wire.drawn)) {
        const auto& own = couplers_[coupler->second];
        way = &own.options[own.chosen].way;
      }
      const PathPoint* anchor = nullptr;
      for (const auto& [key, at] : exempt) {
        if (key == wire.key) {
          anchor = &at;
        }
      }
      const auto reach = couplerReach();
      alongDisc(*way, stencil, [&](const std::int64_t x, const std::int64_t y) {
        if (anchor != nullptr &&
            std::hypot(static_cast<double>(x) - anchor->x,
                       static_cast<double>(y) - anchor->y) <= reach) {
          return;
        }
        corridor_.set(static_cast<std::size_t>((y * width) + x), true);
      });
    }
  }

  void corridorOfEdge(const std::vector<Wire>& wires,
                      const RoutingObjective& objective, const Edge& edge) {
    // The corridor of an edge holds nothing back but the artwork. A band
    // around the beeline, the bodies, the resonators, the committed edges and
    // every port's approach used to be closed here, and it was the band and
    // the fences that bent the edges into their shapes: the search took the
    // turns the corridor left it, not the ones it would have chosen, and
    // raising the bend penalty sixteenfold changed not one point of the angle
    // cost because there was no straighter way left open to move to.
    //
    // **The artwork has to be stamped in here.** The router holds an obstacle
    // grid of its own, but it reads it only to price proximity and to measure
    // a free strip; what actually closes a cell to a search is the corridor
    // and nothing else. Starting from an empty corridor therefore opens the
    // artwork, and the edges ran straight through the launcher pads.
    //
    // `components` and not `blocked`: the artwork alone. `blocked` carries
    // two more things an edge has no reason to obey — everything outside the
    // rectangle the launchers span, and the approach band of every port —
    // and obeying them is what bent the edges before.
    //
    // Every wire an edge passes is drawn again under the feedline constraints
    // afterwards, so none of them is closed either. Nor are the stubs at the
    // edge's own two ends, nor the cells a caller hands in as `more`: the
    // artwork and the launcher terminals below, and nothing else.
    ensureForeignRoom(wires, edge.chain);
    corridor_ = scene_.components;
    // A box around the two ends, and everything outside it closed.
    //
    // Not a band along a beeline — that was what bent the edges into shapes
    // the bend penalty could not move, and it is not coming back. This is a
    // rectangle spanning the source and the target with a wide margin, so
    // an edge may still take any shape inside it; what it may not do is
    // wander the whole chip. Without it the search explores a grid of two
    // million cells to draw an edge three hundred long, and on 17q one edge
    // search cost 51 ms — the insertion's whole runtime is these searches.
    // The prototype bounds the same search the same way, with `expand_path`
    // around the path it already has.
    const auto margin = static_cast<std::int64_t>(EDGE_BOX_MARGIN);
    const auto lowX =
        std::min<std::int64_t>(objective.source.x, objective.target.x);
    const auto highX =
        std::max<std::int64_t>(objective.source.x, objective.target.x);
    const auto lowY =
        std::min<std::int64_t>(objective.source.y, objective.target.y);
    const auto highY =
        std::max<std::int64_t>(objective.source.y, objective.target.y);
    box_ = {
        .minX = std::max<std::int64_t>(0, lowX - margin),
        .minY = std::max<std::int64_t>(0, lowY - margin),
        .maxX = std::min<std::int64_t>(
            static_cast<std::int64_t>(scene_.router.width) - 1, highX + margin),
        .maxY = std::min<std::int64_t>(
            static_cast<std::int64_t>(scene_.router.height) - 1,
            highY + margin)};
    openBox_ = box_;
    slotLeftBox_ = false;
    {
      const auto w = static_cast<std::int64_t>(scene_.router.width);
      const auto h = static_cast<std::int64_t>(scene_.router.height);
      // Every coupler body that stands and the room of every resonator
      // outside this chain, closed in the same pass. Two bit tests a cell on
      // a loop that already walks the grid, against two more walks of it.
      const grid::BitGrid* foreign =
          foreignRoom_.empty() ? nullptr : &foreignRoom_;
      // The meanders' boxes, but around the edge's own two ends: an edge
      // reaches its coupler however near the coupler sits to a meander.
      const grid::BitGrid* meanders =
          meanderKeepout_.empty() ? nullptr : &meanderKeepout_;
      const auto around =
          couplerReach() + static_cast<double>(tuning_.clearance);
      const auto nearAnEnd = [&](const std::int64_t x, const std::int64_t y) {
        return std::hypot(static_cast<double>(x) - objective.source.x,
                          static_cast<double>(y) - objective.source.y) <=
                   around ||
               std::hypot(static_cast<double>(x) - objective.target.x,
                          static_cast<double>(y) - objective.target.y) <=
                   around;
      };
      // The coupler box, for an edge between two couplers.
      const bool inBox = !edge.terminal && !couplerBox_.empty();
      // Everything outside the box closed a word at a time; the cells
      // inside it one by one.
      const auto row = [w](const std::int64_t y, const std::int64_t x) {
        return static_cast<std::size_t>((y * w) + x);
      };
      corridor_.setRange(0, row(box_.minY, 0));
      corridor_.setRange(row(box_.maxY + 1, 0), row(h, 0));
      for (std::int64_t y = box_.minY; y <= box_.maxY; ++y) {
        corridor_.setRange(row(y, 0), row(y, box_.minX));
        corridor_.setRange(row(y, box_.maxX + 1), row(y + 1, 0));
        for (std::int64_t x = box_.minX; x <= box_.maxX; ++x) {
          const auto cell = row(y, x);
          if ((inBox && !couplerBox_.holds(x, y)) || bodies_.test(cell) ||
              (foreign != nullptr && foreign->test(cell)) ||
              (meanders != nullptr && meanders->test(cell) &&
               !nearAnEnd(x, y))) {
            corridor_.set(cell, true);
          }
        }
      }
    }
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto height = static_cast<std::int64_t>(scene_.router.height);
    const auto& chain = chains_[edge.chain];

    // Every feedline that stands now is an obstacle to the one being drawn.
    // This is the one fence the corridor kept when the rest were opened: two
    // edges of the same chain are wires like any other pair and owe each
    // other the clearance, and an edge drawn through a chain that is already
    // there is not a way the stage can keep. The edges that meet this one at
    // a coupler are the exception, and only within that coupler's reach,
    // where they pin on the same cell on purpose.
    fenceCommittedEdges(wires, edge);

    // The edges the caller is standing on, which nothing above can close.
    //
    // `fenceCommittedEdges` reaches only what an earlier round committed,
    // and inside one solve that is the *other* assignment's ways — so
    // without this two edges of the chain being searched never see each
    // other, the search prices a pair that cannot both be built as if it
    // could, and the commit is where one of them is lost. This is that hole
    // closed: the ways the caller has already laid, fenced at the full
    // clearance like every other feedline. `prefixFence_` says which they
    // are.
    if (prefixFence_ != nullptr) {
      for (const Path* laid : *prefixFence_) {
        if (laid == nullptr || laid->empty()) {
          continue;
        }
        alongDisc(*laid, stencilFor(tuning_.clearance),
                  [&](const std::int64_t x, const std::int64_t y) {
                    corridor_.set(static_cast<std::size_t>((y * width) + x),
                                  true);
                  });
      }
    }

    // The leads of this chain's own couplers, the two at this edge's ends
    // included.
    //
    // `ensureForeignRoom` holds every other coupler's lead and leaves out
    // this chain's, because those move while it is solved. Between the two,
    // **every** lead on the chip stands in the way of every edge, inflated
    // by the clearance, with no exemption anywhere — see `terminalSlot`,
    // which is the one thing that opens again and the only thing that does.
    {
      const auto& stencil = stencilFor(tuning_.clearance);
      for (std::size_t at = 0; at < chain.size(); ++at) {
        if (chain[at].fixed) {
          continue;
        }
        const auto& coupler = couplers_[chain[at].coupler];
        const auto& option = coupler.options[coupler.chosen];
        const auto& way = wayOfResonator(wires[coupler.wire]);
        const auto close = [&](const std::int64_t x, const std::int64_t y) {
          corridor_.set(static_cast<std::size_t>((y * width) + x), true);
        };
        alongDisc(option.arc, stencil, close);
        alongDisc(way, stencilFor(RESONATOR_COPPER), close,
                  option.arc.empty() ? 0 : option.arc.size() - 1);
        // The pad with it, copper only, and every pad of the chain rather
        // than the two at this edge's ends: these are the ones `bodies_`
        // does not hold while the chain is still searching itself.
        for (const auto cell : option.body) {
          corridor_.set(cell, true);
        }
      }
    }

    // The launcher terminals: the one thing an edge may not run through. A
    // wire leaves its launcher on the launcher's heading and has nowhere else
    // to go, so the cell, the straight run it needs off it and the clearance
    // around the whole of it are closed.
    //
    // **Every** launcher that carries a wire, not only the ends of the
    // chains: on 4q the chains use two of sixteen, and the other fourteen are
    // the sources of the conventional wires. Closing only the chain ends let
    // the edges run right past those fourteen.
    //
    // Every launcher but the one this edge is being routed to. A chain edge
    // has at most one launcher end — the other is a coupler — so this is the
    // one terminal it has to be able to reach, and every other launcher on
    // the chip is closed to it with its clearance disc like any obstacle.
    // Opening both ends was the wider rule and had nothing to open: an edge
    // between two couplers has no launcher end at all, and one with two
    // would be a chain without a coupler, which the insertion never builds.
    std::unordered_set<std::uint32_t> mine;
    if (chain[edge.to].fixed) {
      mine.insert(chain[edge.to].port);
    } else if (chain[edge.from].fixed) {
      mine.insert(chain[edge.from].port);
    }
    const auto& stencil = stencilFor(tuning_.clearance);
    // The straight run the wire off a launcher is forced to make, and the
    // margin of `edgeLauncherMargin` beyond.
    const auto launcherRun = static_cast<std::int64_t>(tuning_.straightStart) +
                             tuning_.edgeLauncherMargin;
    const auto close = [&](const PathPoint& at) {
      // The straight run off the terminal, on the heading the wire leaves it
      // on, and the clearance around it.
      const auto v = routing::headingVector(at.heading);
      Path places;
      for (std::int64_t k = 0; k <= launcherRun; ++k) {
        const auto x = static_cast<std::int64_t>(at.x) + (k * v.dx);
        const auto y = static_cast<std::int64_t>(at.y) + (k * v.dy);
        if (x < 0 || y < 0 || x >= width || y >= height) {
          break;
        }
        places.push_back({.x = static_cast<std::uint32_t>(x),
                          .y = static_cast<std::uint32_t>(y),
                          .heading = at.heading,
                          .primitive = 0});
      }
      alongDisc(
          places, stencil, [&](const std::int64_t x, const std::int64_t y) {
            corridor_.set(static_cast<std::size_t>((y * width) + x), true);
          });
    };
    for (const auto& [port, slot] : scene_.launcherCell) {
      if (mine.contains(port)) {
        continue;
      }
      const auto found = scene_.launcherHeading.find(port);
      if (found == scene_.launcherHeading.end()) {
        continue;
      }
      const auto place = scene_.router.cell(slot);
      close({.x = place.x(),
             .y = place.y(),
             .heading = found->second,
             .primitive = 0});
    }

    // The runs the terminal edges of every chain already settled have to
    // make at their couplers — into the first coupler, out of the last —
    // and `terminalStubExtra` cells beyond, inflated by the clearance: an
    // edge of a later chain drawn
    // across them is a crossing of two chains the feedline pass cannot
    // undo, because a terminal edge that finds no way keeps its way. 69q's
    // f5 and f6 crossed at the insertion that way. See `terminalStubGuard`.
    guardSettledTerminalRuns(edge);
    guardOwnTerminalRuns(edge);

    // **The slot**, and the only thing that opens again — last, so that it
    // is the last word.
    //
    // Everything within the clearance of a resonator is closed above, the
    // coupler's lead with it, and that seals the corridor at the port the
    // edge has to dock on. A straight run of `terminalSlot` cells is freed
    // at each of the two terminals and nothing else is: no coupling zone,
    // no exemption around an anchor. The prototype's `free_terminal_stub`
    // is this, down to the twenty.
    //
    // The artwork stays closed. A forced run that lies on copper is an edge
    // that cannot be built, and opening it would hide that rather than mend
    // it.
    //
    {
      // At least the slot, and never less than the run the search is held
      // to plus the room one turn needs.
      //
      // In the prototype the slot and the run are one number: it holds the
      // router to 20 and frees 20. Here the run at a coupler is the pad's
      // own length, because the feedline lies along the whole pad — 21
      // cells on 45q — and after it the edge still has to bend away. A slot
      // of 20 left it standing at the end of the pad inside the resonator's
      // clearance, and six edges of 45q and fourteen of 57q came back with
      // no way at all.
      //
      // **Measured on 45q**, slot against edges drawn: 20 -> 46 of 52, 25 ->
      // 52, and 30, 35, 40, 60, 100 all bit-identical to 25. So what is
      // missing at 20 is a handful of cells to turn in, not room to route
      // in, and `BEND_RADIUS` is that figure by name.
      const auto [startRun, endRun] = runsOfEdge(edge);
      const auto turn = static_cast<std::uint32_t>(BEND_RADIUS);
      const auto slotFrom = std::max(tuning_.terminalSlot, startRun + turn);
      const auto slotTo = std::max(tuning_.terminalSlot, endRun + turn);
      std::size_t opened = 0;
      const auto reopen = [&](const Path& run) {
        for (const auto& point : run) {
          if (point.x >= scene_.router.width ||
              point.y >= scene_.router.height) {
            continue;
          }
          const auto cell = scene_.router.index(point.x, point.y);
          if (scene_.components.test(cell) || !corridor_.test(cell)) {
            continue;
          }
          corridor_.set(cell, false);
          ++opened;
          slotLeftBox_ = slotLeftBox_ ||
                         static_cast<std::int64_t>(point.x) < box_.minX ||
                         static_cast<std::int64_t>(point.x) > box_.maxX ||
                         static_cast<std::int64_t>(point.y) < box_.minY ||
                         static_cast<std::int64_t>(point.y) > box_.maxY;
        }
      };
      reopen(router_.straightStub(objective.source, false, slotFrom));
      reopen(router_.straightStub(objective.target, true, slotTo));
      if (opened != 0) {
        tell(std::format("edge c{}-{}to{}: {} cells of its slots ({} and {} "
                         "cells) were closed and are open again",
                         edge.chain, edge.from, edge.to, opened, slotFrom,
                         slotTo));
      }
    }

    // **No slot opens an edge of the same chain.** The slots open the
    // clearance around everything, and so also around an earlier edge of
    // this chain that lies in the run off a port; the edge being drawn could
    // then cross it. The copper of every such way, one cell to each side so
    // that two diagonal steps cannot cross at a cell corner either, closes
    // again here. Edges of other chains may cross in the insertion.
    //
    // A terminal edge keeps the whole clearance from its neighbour, the edge
    // that meets it at its coupler, in the slots too. Any other edge keeps
    // it outside the meeting the feedline routing grants the two.
    const auto& points = chains_[edge.chain];
    struct Meeting {
      /// The neighbour's port at the shared coupler.
      PathPoint port;
      /// This edge's port at the shared coupler.
      PathPoint own;
      /// The shared coupler's anchor.
      PathPoint anchor;
    };
    std::vector<Meeting> meetings;
    const bool whole = edge.terminal;
    {
      const auto anchorOf = [&](const Waypoint& point) {
        const auto& coupler = couplers_[point.coupler];
        return coupler.options[coupler.chosen].anchor;
      };
      if (!points[edge.from].fixed) {
        meetings.push_back({.port = targetOf(points[edge.from]),
                            .own = objective.source,
                            .anchor = anchorOf(points[edge.from])});
      }
      if (!points[edge.to].fixed) {
        meetings.push_back({.port = sourceOf(points[edge.to]),
                            .own = objective.target,
                            .anchor = anchorOf(points[edge.to])});
      }
    }
    const auto meetingOf = [&](const Path& way) -> const Meeting* {
      for (const auto& meeting : meetings) {
        const auto near = [&](const PathPoint& end) {
          return std::abs(static_cast<std::int64_t>(end.x) -
                          static_cast<std::int64_t>(meeting.port.x)) <= 2 &&
                 std::abs(static_cast<std::int64_t>(end.y) -
                          static_cast<std::int64_t>(meeting.port.y)) <= 2;
        };
        if (near(way.front()) || near(way.back())) {
          return &meeting;
        }
      }
      return nullptr;
    };
    // Whether two cells of the edges that meet at a coupler pass the rule
    // where its middle lies, as `meetAt` judges a pair in the feedline
    // routing: within half the coupling length and a clearance of the
    // anchor, or within one and a half clearances of either port when the
    // two ports stand within the rule of each other.
    const auto link = static_cast<double>(tuning_.clearance);
    const auto reach =
        (0.5 * static_cast<double>(tuning_.couplerLength)) + link;
    const auto meets = [&](const Meeting& meeting, const double x,
                           const double y) {
      const auto from = [&](const PathPoint& at) {
        return std::hypot(x - static_cast<double>(at.x),
                          y - static_cast<double>(at.y));
      };
      if (from(meeting.anchor) <= reach) {
        return true;
      }
      const auto apart = std::hypot(static_cast<double>(meeting.port.x) -
                                        static_cast<double>(meeting.own.x),
                                    static_cast<double>(meeting.port.y) -
                                        static_cast<double>(meeting.own.y));
      return apart <= link && (from(meeting.port) <= 1.5 * link ||
                               from(meeting.own) <= 1.5 * link);
    };
    const auto closeCopper = [&](const Path& way) {
      const auto* meeting = meetingOf(way);
      const auto& stencil = meeting != nullptr && whole
                                ? stencilFor(tuning_.clearance)
                                : stencilFor(1);
      alongDisc(way, stencil, [&](const std::int64_t x, const std::int64_t y) {
        corridor_.set(static_cast<std::size_t>((y * width) + x), true);
      });
      if (meeting == nullptr || whole) {
        return;
      }
      for (const auto& point : way) {
        for (const auto& [dx, dy] : stencilFor(tuning_.clearance).full) {
          const auto x = static_cast<std::int64_t>(point.x) + dx;
          const auto y = static_cast<std::int64_t>(point.y) + dy;
          if (x < 0 || y < 0 || x >= width || y >= height ||
              meets(*meeting, static_cast<double>(point.x) + (0.5 * dx),
                    static_cast<double>(point.y) + (0.5 * dy))) {
            continue;
          }
          corridor_.set(static_cast<std::size_t>((y * width) + x), true);
        }
      }
    };
    for (const Path* way : sameChainWays_) {
      closeCopper(*way);
    }
    if (prefixFence_ != nullptr) {
      for (const Path* laid : *prefixFence_) {
        if (laid != nullptr && !laid->empty()) {
          closeCopper(*laid);
        }
      }
    }
  }

  /// What a bend costs an edge of a chain. A feedline is judged by how much
  /// it turns — that is the whole objective the greedy optimises — so an edge
  /// may pay more for a bend than a wire that is only trying to get there.
  /// Read from the environment so a sweep over it costs a rebuild of nothing.
  ///
  /// At 1.0 this **is** the outer routing's own figure, 7125 on 17q and
  /// 27000 on 69q; the insertion adds nothing to it. The prototype routes
  /// its coupler edges at a flat 4000 (`FinalGrid.cpp:4963`), so ours is
  /// between two and seven times what it pays, and the factor may now go
  /// below one to meet it (user, 2026-10-03).
  [[nodiscard]] std::uint16_t edgeBendPenalty() const {
    const double factor = tuning_.edgeBendFactor;
    // `RoutingConfig::bendPenalty` is 16 bits; 69q's 27000 times a factor
    // above 2.43 does not fit, and the cast of a value out of range is
    // undefined.
    return static_cast<std::uint16_t>(
        std::clamp(factor * tuning_.bendPenalty, 1.0, 65535.0));
  }

  /// The forced straight run at a waypoint's port for a named option, as
  /// `runsOfEdge` reads it off the chosen one.
  [[nodiscard]] std::uint32_t forcedRunOf(const std::uint32_t chain,
                                          const std::size_t at,
                                          const std::size_t option) const {
    const auto& point = chains_[chain][at];
    if (point.fixed) {
      return tuning_.straightStart;
    }
    return couplers_[point.coupler].options[option].run;
  }

  /// The analytic bound with the forced end runs folded in
  /// (`SCPD_CHAIN_BOUND=4`). Every way has to run `first` cells straight out
  /// of the source and `last` straight into the target, so the rest of it
  /// covers the displacement less those two runs. Both points are lifted by
  /// a constant so the shifted target stays inside the unsigned
  /// coordinates; the bound reads differences only.
  [[nodiscard]] std::uint32_t stubbedTurns(const PathPoint& from,
                                           const PathPoint& to,
                                           const std::uint32_t first,
                                           const std::uint32_t last) const {
    constexpr std::int64_t LIFT = 1LL << 20U;
    const auto vs = routing::headingVector(from.heading & 7U);
    const auto vt = routing::headingVector(to.heading & 7U);
    auto a = from;
    auto b = to;
    a.x = static_cast<std::uint32_t>(static_cast<std::int64_t>(a.x) + LIFT);
    a.y = static_cast<std::uint32_t>(static_cast<std::int64_t>(a.y) + LIFT);
    b.x =
        static_cast<std::uint32_t>(static_cast<std::int64_t>(b.x) + LIFT -
                                   ((static_cast<std::int64_t>(first) * vs.dx) +
                                    (static_cast<std::int64_t>(last) * vt.dx)));
    b.y =
        static_cast<std::uint32_t>(static_cast<std::int64_t>(b.y) + LIFT -
                                   ((static_cast<std::int64_t>(first) * vs.dy) +
                                    (static_cast<std::int64_t>(last) * vt.dy)));
    return analytic_.minTurns(a, b);
  }

  /// A round's total, or the word for a round holding a chain that nothing
  /// joins — which is not a number and must not print as one.
  [[nodiscard]] static std::string spell(const std::uint64_t total) {
    return total == routing::TRELLIS_UNREACHABLE ? std::string("unreachable")
                                                 : std::to_string(total);
  }

  /// What the bound cost and how sharp it was, over the whole insertion.
  std::uint64_t boundPairs_ = 0;
  /// The step budget of the chain A*: what the step being priced may cost,
  /// handed from the search to `routeEdge`; how many searches it cut off,
  /// and whether the last one was.
  std::uint64_t stepBudget_ = routing::TRELLIS_UNREACHABLE;
  std::uint64_t budgetCutoffs_ = 0;
  bool lastCutOff_ = false;
  std::uint64_t boundNanos_ = 0;

  /// The ways the caller has already laid, for the corridor to close along
  /// with everything committed, or null. Set around a single `routeEdge`
  /// call and cleared again.
  ///
  /// `fenceCommittedEdges` can only close what an earlier round committed,
  /// so inside one solve it holds the *other* assignment's ways. This is
  /// the hole that leaves: the ways of the edges the caller is standing on
  /// right now. The second-order trellis puts one way in it — its
  /// predecessor's — and `solveChainAStar` puts every edge of the prefix it
  /// is extending.
  const std::vector<const Path*>* prefixFence_ = nullptr;
  /// The ways of the edge's own chain that `fenceCommittedEdges` fenced for
  /// the corridor being built, closed again after the slots open.
  std::vector<const Path*> sameChainWays_;
  /// The wires of the feedline pass, for the checks `search` makes on a way
  /// it found (`breaksAFeedlineRule`); null outside the feedline pass.
  const std::vector<Wire>* feedlineWires_ = nullptr;
  /// The wires a relaxation level has let go of, while
  /// `constrainByFeedlines` builds the search of that level: a feedline
  /// edge among them is no fence; its room is still priced.
  std::vector<std::uint32_t> letGo_;

  /// How often a chain search says how far it has come: every
  /// `SCPD_CHAIN_PROGRESS_STEPS` priced steps, 100 by default, a
  /// `[Chain Progress]` line with the steps priced, the edge searches, the
  /// prefixes expanded, the open list, the deepest layer reached and the
  /// cheapest complete run so far. Zero says nothing. The search has no time
  /// limit: it ends on the optimum or when it has seen every run of options,
  /// so its answer does not depend on the speed of the machine. These lines
  /// show a search that grows without end while it runs.
  static constexpr std::uint32_t CHAIN_PROGRESS_STEPS = 100;

  // ---------------------------------------------- The length-point clearance
  //
  // The coupler insertion puts a resonator's pad where what is left of the
  // way to the qubit is the target length (`couplerPlace`). The pad and the
  // lead stick out beside the way there and need room, and a plain wire
  // that the outer routing laid too close to that place is what the
  // insertion and the feedline pass then fail on, at a spot nobody can
  // clean up any more (the room rules of 2026-10-04: R3, the lane pairs
  // leaning on a pad). The prototype's answer is its RRR-Lenpoint-Constraint
  // (`FinalGrid.cpp:6193-6600`, `:7620-7760`): in the outer routing and its
  // refinement every fence of a resonator inflates the stretch of its way
  // around that point by a larger radius, and a resonator being routed keeps
  // the same radius from the stretch of each neighbour that runs alongside
  // its own point. This is that mechanism, for the outer routing and the
  // outer refinement **only** — under the feedline constraints the couplers
  // stand and the lead and pad are fenced themselves — and **hard**: a wire
  // that cannot be routed with the clearance fails (user, 2026-10-04); the
  // prototype's second try without it is not built. Plan:
  // `plan-resonator-lenpoint.md`.

  /// The radius a wire keeps from a resonator's length-point band, in cells:
  /// `SCPD_LENPOINT_K`, **40**, the prototype's flat figure on a grid of the
  /// same pitch (user, 2026-10-04: as the prototype has it, no derivation
  /// from the pad's geometry); 0 is off. A first version took
  /// `couplerHeight + clearance + BEND_RADIUS` = 27 on the benchmarks;
  /// `artifacts/logs/lenpoint-k27` holds that arm for 4q–45q.
  ///
  /// **Measured 2026-10-04** at 40 (`artifacts/logs/lenpoint40` against
  /// `bridge-check`, eight chips, `stop_after = "feedlines"`, no repair):
  /// `bad` after the feedline routing 71 → 56 (33q 4 → 0, 45q 8 → 5, 69q
  /// 28 → 16), the stage 2400 s → 1354 s; but the outer routing no longer
  /// ends at 0 fails everywhere — 17q leaves resonator 30 unrouted (bad
  /// 8 → 12), 57q ends with 23 open and loses edge f4 of chain 0 — and the
  /// feedline angle moves with the layout (4q 8 → 12, 45q 111 → 109). See
  /// *The length-point clearance* in handover-feedline-routing.md.
  // --------------------------------------------------- The targeted repair
  //
  // Phase 4 leaves wires open that no sweep closes: lane-trading pairs of
  // plain wires leaning on a coupler pad, a resonator whose lead the
  // insertion laid one cell from a neighbour, terminal edges of two chains
  // crossing. The room rules measured on 2026-10-04 that no local geometric
  // rule at the coupler separates these from healthy couplers; what reaches
  // them is a trial that changes coupler options and routes again. The
  // targeted repair is that trial done with aim: every fail is traced to the
  // chain segments that can be its cause (`blameFails`), each segment is
  // re-searched with the prefix search on its own — both ends fixed, the
  // rest of the chip frozen — and every complete run of options the search
  // reaches is tested by a local rip-up-and-reroute of the wires the segment
  // touches before it is taken. Plan: `plan-coupler-repair-search.md`.

  /// The allowed offsets as a comma list.
  [[nodiscard]] static std::string
  offsetsText(const std::array<bool, routing::NUM_HEADINGS>& offsets) {
    std::string text;
    for (std::size_t offset = 0; offset < offsets.size(); ++offset) {
      if (offsets.at(offset)) {
        text += std::format("{}{}", text.empty() ? "" : ",", offset);
      }
    }
    return text;
  }

  /// How many couplers took every offset because none of the allowed ones
  /// fits them (`couplerOffsets`).
  std::uint32_t offsetFallbacks_ = 0;

  /// The weight of edge `at` of chain `chain` in the chain searches:
  /// `chainEndEdgeWeight` for an edge with a launcher at one end, 1 else.
  [[nodiscard]] double edgeWeight(const std::uint32_t chain,
                                  const std::size_t at) const {
    return atLauncher(chain, at) ? tuning_.chainEndEdgeWeight : 1.0;
  }

  /// A price or a bound of the chain searches times `weight`, rounded to
  /// the nearest whole number, halves away from zero (`std::llround`). The
  /// rounding never lowers a larger value below a smaller one, so a bound
  /// weighted alike stays a lower bound. TRELLIS_UNREACHABLE and a weight
  /// of 1 leave the value as it is.
  [[nodiscard]] static std::uint64_t weighted(const std::uint64_t value,
                                              const double weight) {
    if (value == routing::TRELLIS_UNREACHABLE || weight == 1.0) {
      return value;
    }
    return static_cast<std::uint64_t>(
        std::llround(weight * static_cast<double>(value)));
  }

  /// The turn budget of an edge of weight `weight` from the step budget of
  /// its prefix (`ChainProblem::stepBudget`, in weighted price): its way may
  /// cost up to `(budget + 0.5) / weight` unweighted and still round to no
  /// more than the budget, so the router is told that, rounded up. A weight
  /// of 0 leaves the edge without a budget: no way of it can cost more.
  [[nodiscard]] static std::uint64_t weightedBudget(const std::uint64_t budget,
                                                    const double weight) {
    if (budget == routing::TRELLIS_UNREACHABLE || weight == 1.0) {
      return budget;
    }
    if (weight == 0.0) {
      return routing::TRELLIS_UNREACHABLE;
    }
    const auto most = std::ceil((static_cast<double>(budget) + 0.5) / weight);
    return most >= static_cast<double>(routing::TRELLIS_UNREACHABLE)
               ? routing::TRELLIS_UNREACHABLE
               : static_cast<std::uint64_t>(most);
  }

  // ------------------------------------------------------- The room rules
  //
  // The insertion routes every feedline edge against the artwork, the pads,
  // the leads and the resonator tails, and never against the ring wires that
  // must cross it or run beside it afterwards. The feedline pass then draws
  // the terminal edges again but keeps every edge between two couplers as
  // the insertion laid it, so a pinch between two such edges — the two edges
  // of one coupler bending back the same way with 44 cells between them
  // where two plain wires need three times the pitch — can be mended here or
  // nowhere. The room rules measure the room an option leaves on the
  // geometry `routing/RoomRules.hpp` holds, and each refuses hard: a refused
  // option or edge does not exist for the search, nothing is priced (user,
  // 2026-10-03). Every rule has a switch, and the calibration lines
  // `reportRoom` prints at the chosen options say what each would have
  // refused whether or not it is on.

  /// The centre-to-centre pitch of two plain wires running side by side, in
  /// cells: `SCPD_ROOM_PITCH`, the clearance plus one by default — 20 on
  /// every benchmark. The design rule is 18.65 cells, so 20 is conservative
  /// by one to two cells a lane. Read by every room rule and by the
  /// calibration lines.
  [[nodiscard]] std::uint32_t roomPitch() const {
    return tuning_.clearance + 1;
  }

  /// How far `wallsAfterInsertion` inflates a port run or a feedline
  /// edge: half the wire clearance, in cells (user, 2026-10-07).
  [[nodiscard]] double wallInflation() const {
    return 0.5 * static_cast<double>(tuning_.clearance);
  }
  /// The cells within `wallInflation` of a cell, as offsets.
  [[nodiscard]] std::vector<std::pair<std::int64_t, std::int64_t>>
  inflationStencil() const {
    const auto radius = wallInflation();
    const auto reach = static_cast<std::int64_t>(std::floor(radius));
    std::vector<std::pair<std::int64_t, std::int64_t>> offsets;
    for (std::int64_t dy = -reach; dy <= reach; ++dy) {
      for (std::int64_t dx = -reach; dx <= reach; ++dx) {
        if (static_cast<double>((dx * dx) + (dy * dy)) <= radius * radius) {
          offsets.emplace_back(dx, dy);
        }
      }
    }
    return offsets;
  }

  /// What the room rules did in one insertion, for the summary line.
  struct RoomStats {
    std::uint32_t r1Asked = 0;
    std::uint32_t r1Refused = 0;
    std::uint32_t r2Asked = 0;
    std::uint32_t r2Refused = 0;
    /// Places the option's own-room rule refused.
    std::uint32_t r3Refused = 0;
    /// Couplers built without the own-room rule, having too few options
    /// under it.
    std::uint32_t r3StoodDown = 0;
    /// Couplers that ended more than 100 units off the design figure.
    std::uint32_t r3Moved = 0;
    /// The chains each rule was lifted for.
    std::vector<std::uint32_t> liftedR1;
    std::vector<std::uint32_t> liftedR2;
    std::vector<std::uint32_t> liftedR3;
    /// At the commit: edges whose straights carry fewer lanes than wires
    /// must cross them, and coupler channels narrower than the wires in
    /// them need.
    std::uint32_t committedBelowCapacity = 0;
    std::uint32_t committedTightChannels = 0;
    /// The same, counting only the wires that run along the channel.
    std::uint32_t committedTightAlong = 0;
    std::uint32_t couplerChannels = 0;
    /// At the commit: couplers whose chosen option has a plain wire's copper
    /// within 0 / 5 / 10 / 19 cells of its pad, run or lead — what R3 would
    /// refuse at each reach.
    std::array<std::uint32_t, 4> r3Near{};
    /// The channels between two chains, and how many are too narrow.
    std::uint32_t crossChainChannels = 0;
    std::uint32_t crossChainTight = 0;
  };
  RoomStats room_;
  /// The edges `reportSqueeze` found leaving too little room, by wire key,
  /// with the figures in words — what the artifact carries as `Squeezed`
  /// and `note` on every snapshot from the coupler insertion on.
  std::unordered_map<std::uint32_t, Squeeze> squeezed_;

  /// The outer ring in ring order — every wire that is not inner, in the
  /// order the assignment feeds them — and the ring place of every slot.
  /// What `assignBridges` walks, held here so the insertion can walk it
  /// before the chains are settled.
  std::vector<std::uint32_t> ring_;
  std::unordered_map<std::uint32_t, std::uint32_t> ringPlace_;
  /// R0: for every chain and edge, the plain ring wires that must cross
  /// that edge. Empty for a terminal edge. See `bridgersOf`.
  std::vector<std::vector<std::vector<std::uint32_t>>> bridgers_;
  /// The chain and waypoint every coupler stands at.
  std::vector<std::pair<std::uint32_t, std::size_t>> chainOfCoupler_;
  /// Which room rules are lifted for a chain: the stand-down ladder's
  /// state, one flag per chain. Nothing but the rules reads them.
  std::vector<bool> liftedR1_;
  std::vector<bool> liftedR2_;
  std::vector<bool> liftedR3_;

  /// R0: the plain ring wires that must cross one edge of a chain.
  ///
  /// The walk `assignBridges` makes, done once here so the room rules can
  /// ask before the chain is settled: the ring wires that are not
  /// resonators, are feasible, and whose slot lies strictly between the
  /// slots of the two couplers' resonators, walking the ring forward from
  /// the `from` resonator. A terminal edge has none. Both arcs are walked
  /// and the shorter kept — a chain whose nodes ran against ring order
  /// would otherwise count nearly the whole ring — and since
  /// `assignBridges` itself only walks forward, the line says when the two
  /// differ; `checkBridgers` in phase 4 must then report 0.
  [[nodiscard]] std::vector<std::uint32_t>
  bridgersOf(const std::vector<Wire>& wires, const std::uint32_t chain,
             const std::size_t at) const {
    std::vector<std::uint32_t> none;
    const auto& points = chains_[chain];
    if (at + 1 >= points.size() || points[at].fixed || points[at + 1].fixed ||
        ring_.empty()) {
      return none;
    }
    const auto ring = static_cast<std::uint32_t>(ring_.size());
    const auto from =
        ringPlace_.find(wires[couplers_[points[at].coupler].wire].slot);
    const auto to =
        ringPlace_.find(wires[couplers_[points[at + 1].coupler].wire].slot);
    if (from == ringPlace_.end() || to == ringPlace_.end()) {
      return none;
    }
    const auto walk = [&](const std::uint32_t a, const std::uint32_t b) {
      std::vector<std::uint32_t> keys;
      for (auto p = (a + 1) % ring; p != b; p = (p + 1) % ring) {
        const auto& wire = wires[ring_[p]];
        if (!wire.resonator && wire.feasible) {
          keys.push_back(wire.key);
        }
      }
      return keys;
    };
    auto forward = walk(from->second, to->second);
    auto backward = walk(to->second, from->second);
    if (backward.size() < forward.size()) {
      tell(std::format("coupler insertion: chain {} edge {}->{}: the ring "
                       "runs the other way round it ({} wires forward, {} "
                       "back); the shorter arc is taken",
                       chain, at, at + 1, forward.size(), backward.size()));
      return backward;
    }
    return forward;
  }

  /// The names of a list of wires, for a line.
  [[nodiscard]] static std::string
  namesOf(const std::vector<Wire>& wires,
          const std::vector<std::uint32_t>& keys) {
    std::string out;
    for (const auto key : keys) {
      out += (out.empty() ? "" : " ") + wireId(wires[key]);
    }
    return out.empty() ? std::string("-") : out;
  }

  /// After `assignBridges`: whether the edge every plain wire was told to
  /// cross is the edge the insertion counted it against (R0). A difference
  /// means the room rules were fed a wrong count; it must be 0 on every
  /// chip.
  void checkBridgers(const std::vector<Wire>& wires) const {
    std::unordered_map<std::uint32_t, std::uint32_t> expected;
    for (std::size_t chain = 0; chain < bridgers_.size(); ++chain) {
      for (std::size_t at = 0; at < bridgers_[chain].size(); ++at) {
        for (const auto key : bridgers_[chain][at]) {
          expected[key] = edgeWireOf_[chain][at];
        }
      }
    }
    std::uint32_t differ = 0;
    std::string named;
    for (const auto key : ring_) {
      const auto& wire = wires[key];
      if (wire.resonator || !wire.feasible) {
        continue;
      }
      const auto found = expected.find(key);
      const auto want = found == expected.end() ? NO_OWNER : found->second;
      if (want == wire.bridged) {
        continue;
      }
      ++differ;
      if (differ <= 12) {
        named += std::format(
            "{}{} crosses {} where the insertion counted it on {}",
            named.empty() ? ": " : " · ", wireId(wire),
            wire.bridged == NO_OWNER ? std::string("no edge")
                                     : wireId(wires[wire.bridged]),
            want == NO_OWNER ? std::string("no edge") : wireId(wires[want]));
      }
    }
    say(std::format("feedline routing: CHECK bridges — {} wire{} whose "
                    "bridged edge differs from the insertion's count{}",
                    differ, differ == 1 ? "" : "s",
                    differ == 0 ? "; the check is GREEN" : named));
  }

  /// The channel a coupler's two edges leave behind its pad, measured.
  struct CouplerChannel {
    routing::Channel channel;
    /// The plain wires whose ways lie in the channel, by key.
    std::vector<std::uint32_t> inside;
    /// Those of them that run along the channel rather than across it.
    std::vector<std::uint32_t> along;
    /// Whether the arms come closer than where they leave their pad runs.
    /// Arms that run on or apart leave no channel to measure.
    bool narrows = false;
    /// Whether the resonator's own way runs through the channel.
    bool resonatorInside = false;
    /// Whether both arms lie on the same side of the pad.
    bool sameSide = false;
    /// How many wires must pass through: `m`.
    std::uint32_t need = 0;
  };

  /// R2's measurement: how close the edge arriving at a pad and the edge
  /// leaving it come to each other behind the pad, and how many plain wires
  /// lie between them there.
  ///
  /// Both pad runs are skipped (`run + 1` cells); the in-edge is followed
  /// `reach` cells back and the out-edge twice as far forward. `m` is read
  /// off the field: for every cell of the in-arm, the cells on the line to
  /// its nearest out-arm cell name their owners, and every distinct plain
  /// ring wire among them counts once — plus one when the resonator's own
  /// way past its lead lies on one of those lines.
  [[nodiscard]] CouplerChannel channelAt(const std::vector<Wire>& wires,
                                         const CouplerOption& option,
                                         const std::uint32_t resonatorKey,
                                         const Path& in, const Path& out,
                                         const std::size_t reach) const {
    CouplerChannel found;
    const std::size_t skip = option.run + 1;
    found.channel =
        routing::channelBetween(in, skip, reach, out, skip, 2 * reach);
    if (!found.channel.measured) {
      return found;
    }
    found.narrows = found.channel.gap < found.channel.startGap - 0.5;
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto height = static_cast<std::int64_t>(scene_.router.height);
    const auto aEnd = in.size() - skip;
    const auto aBegin = aEnd > reach ? aEnd - reach : std::size_t{0};
    const auto bBegin = skip;
    const auto bEnd = std::min(out.size(), bBegin + (2 * reach));
    std::unordered_set<std::size_t> lineCells;
    std::set<std::uint32_t> owners;
    // A wire runs along the channel when its cells on the lines keep within
    // an eighth of the arm's heading or its reverse; one that crosses the
    // channel meets the lines at a right angle. Half a pitch of such cells
    // makes a wire an along-runner.
    std::unordered_map<std::uint32_t, std::unordered_map<std::size_t, Heading>>
        headingOf;
    std::unordered_map<std::uint32_t, std::uint32_t> parallel;
    for (std::size_t i = aBegin; i < aEnd; ++i) {
      std::size_t nearest = bBegin;
      auto least = std::numeric_limits<double>::infinity();
      for (std::size_t j = bBegin; j < bEnd; ++j) {
        const auto dx =
            static_cast<double>(in[i].x) - static_cast<double>(out[j].x);
        const auto dy =
            static_cast<double>(in[i].y) - static_cast<double>(out[j].y);
        const auto d = (dx * dx) + (dy * dy);
        if (d < least) {
          least = d;
          nearest = j;
        }
      }
      routing::supercoverLine(
          in[i].x, in[i].y, out[nearest].x, out[nearest].y,
          [&](const std::int64_t x, const std::int64_t y) {
            if (x < 0 || y < 0 || x >= width || y >= height) {
              return;
            }
            const auto cell = static_cast<std::size_t>((y * width) + x);
            lineCells.insert(cell);
            const auto owner = field_.owner(cell);
            if (owner != NO_OWNER && owner != resonatorKey &&
                owner < conventional_.size() && conventional_[owner] != 0) {
              owners.insert(owner);
              auto known = headingOf.find(owner);
              if (known == headingOf.end()) {
                std::unordered_map<std::size_t, Heading> cells;
                for (const auto& point : wires[owner].way) {
                  if (point.x < scene_.router.width &&
                      point.y < scene_.router.height) {
                    cells[scene_.router.index(point.x, point.y)] =
                        point.heading;
                  }
                }
                known = headingOf.emplace(owner, std::move(cells)).first;
              }
              const auto heading = known->second.find(cell);
              if (heading != known->second.end() &&
                  routing::headingDistance(heading->second, in[i].heading) !=
                      2) {
                ++parallel[owner];
              }
            }
          });
    }
    found.inside.assign(owners.begin(), owners.end());
    for (const auto owner : found.inside) {
      const auto count = parallel.find(owner);
      if (count != parallel.end() && count->second >= roomPitch() / 2) {
        found.along.push_back(owner);
      }
    }
    for (std::size_t at = option.arc.size(); at < option.way.size(); ++at) {
      const auto& cell = option.way[at];
      if (cell.x < scene_.router.width && cell.y < scene_.router.height &&
          lineCells.contains(scene_.router.index(cell.x, cell.y))) {
        found.resonatorInside = true;
        break;
      }
    }
    // Which side of the pad each arm lies on, read `K` cells past the stubs
    // — the far-edge side is where `across` points.
    const auto across =
        routing::headingVector(routing::turned(option.orientation, -2));
    const auto K = std::min<std::size_t>(reach, 30);
    const auto aK = aEnd > K ? std::max(aBegin, aEnd - K) : aBegin;
    const auto bK = std::min(bEnd - 1, bBegin + K - 1);
    found.sameSide = routing::sameSide(option.centre, across, in[aK], out[bK]);
    found.need = static_cast<std::uint32_t>(owners.size()) +
                 (found.resonatorInside ? 1U : 0U);
    return found;
  }

  /// The crossing capacity of one edge's way with the pad runs and stubs at
  /// its two ends dropped: `max(tuning_.terminalSlot, run + BEND_RADIUS)` at
  /// each end, the figures `corridorOfEdge` and `checkFeedlineRoom` use.
  [[nodiscard]] routing::CrossingCapacity
  capacityOfEdge(const Edge& edge, const Path& way) const {
    const auto [startRun, endRun] = runsOfEdge(edge);
    const auto turn = static_cast<std::uint32_t>(BEND_RADIUS);
    return routing::crossingCapacity(
        way, std::max(tuning_.terminalSlot, startRun + turn),
        std::max(tuning_.terminalSlot, endRun + turn), roomPitch(),
        tuning_.roomMargin);
  }

  /// What a list of run lengths reads as.
  [[nodiscard]] static std::string
  runsOf(const std::vector<std::uint32_t>& runs) {
    std::string out;
    for (const auto run : runs) {
      out += (out.empty() ? "" : " ") + std::to_string(run);
    }
    return out.empty() ? std::string("-") : out;
  }

  /// The room figures at the chosen options, after the commit: what R1
  /// would say of every edge between two couplers and what R2 would say of
  /// every coupler's channel, one line each, and the channels between
  /// chains (R4). Nothing here refuses; it is how the rules are calibrated
  /// before one of them is live, and what the summary line counts.
  void reportRoom(const std::vector<Wire>& wires) {
    const auto pitch = roomPitch();
    // R1 at the chosen options.
    for (const auto& edge : edges_) {
      if (edge.terminal || edge.wire >= wires.size()) {
        continue;
      }
      const auto& wire = wires[edge.wire];
      if (!wire.drawn) {
        continue;
      }
      const auto& need = bridgers_[edge.chain][edge.from];
      const auto capacity = capacityOfEdge(edge, wire.way);
      const bool below = capacity.lanes < need.size();
      if (below) {
        ++room_.committedBelowCapacity;
      }
      // The same runs at no margin, which is the figure for the regime
      // without the orthogonal crossing rule, where a wire may cross its
      // bridged edge anywhere.
      std::uint32_t loose = 0;
      for (const auto run : capacity.runs) {
        loose += routing::lanesOf(run, pitch, 0);
      }
      tell(std::format(
          "coupler insertion: room R1 chain {} edge {}->{} (f{}): {} plain "
          "wire{} must cross it ({}); its straights carry {} lane{} over {} "
          "run{} of {} cells (pitch {}, margin {}; {} at margin 0) — {}",
          edge.chain, edge.from, edge.to, wire.slot, need.size(),
          need.size() == 1 ? "" : "s", namesOf(wires, need), capacity.lanes,
          capacity.lanes == 1 ? "" : "s", capacity.runs.size(),
          capacity.runs.size() == 1 ? "" : "s", runsOf(capacity.runs), pitch,
          tuning_.roomMargin, loose,
          below ? "R1 would refuse" : "room enough"));
    }
    // R2 at the chosen options.
    for (std::uint32_t index = 0; index < couplers_.size(); ++index) {
      const auto& coupler = couplers_[index];
      if (coupler.edgeIn == NO_OWNER || coupler.edgeOut == NO_OWNER) {
        continue;
      }
      const auto& in = wires[coupler.edgeIn];
      const auto& out = wires[coupler.edgeOut];
      if (!in.drawn || !out.drawn) {
        continue;
      }
      const auto& option = coupler.options[coupler.chosen];
      const auto [chain, at] = chainOfCoupler_[index];
      if (chain == NO_OWNER) {
        continue;
      }
      const auto found = channelAt(wires, option, coupler.wire, in.way, out.way,
                                   tuning_.roomChannelReach);
      if (!found.channel.measured) {
        continue;
      }
      ++room_.couplerChannels;
      if (!found.narrows) {
        tell(std::format(
            "coupler insertion: room R2 chain {} coupler '{}' option {}: "
            "the arms of f{} and f{} do not come closer than at the pad "
            "({:.1f} cells, {}) — silent",
            chain, wireId(wires[coupler.wire]), coupler.chosen, in.slot,
            out.slot, found.channel.startGap,
            found.sameSide ? "same side" : "other sides"));
        continue;
      }
      const auto needs = (found.need + 1) * pitch;
      const bool tight = found.need >= 1 && found.channel.gap < needs;
      const auto alongNeed = static_cast<std::uint32_t>(found.along.size()) +
                             (found.resonatorInside ? 1U : 0U);
      const bool tightAlong =
          alongNeed >= 1 && found.channel.gap < (alongNeed + 1) * pitch;
      if (tight) {
        ++room_.committedTightChannels;
      }
      if (tightAlong) {
        ++room_.committedTightAlong;
      }
      tell(std::format(
          "coupler insertion: room R2 chain {} coupler '{}' option {}: "
          "channel between f{} and f{} narrows to {:.1f} cells at ({},{}), "
          "{} cells past the pad (from {:.1f} at the pad), {}; {} wire{} in "
          "it ({}{}), {} running along it ({}); need {} — {}; along-runners "
          "alone need {} — {}",
          chain, wireId(wires[coupler.wire]), coupler.chosen, in.slot, out.slot,
          found.channel.gap, in.way[found.channel.at].x,
          in.way[found.channel.at].y, found.channel.aDepth,
          found.channel.startGap, found.sameSide ? "same side" : "other sides",
          found.need, found.need == 1 ? "" : "s", namesOf(wires, found.inside),
          found.resonatorInside ? " + its own resonator" : "",
          found.along.size(), namesOf(wires, found.along), needs,
          tight ? "R2 would refuse" : "room enough", (alongNeed + 1) * pitch,
          tightAlong ? "would refuse" : "room enough"));
    }
    // R3 at the chosen options: how close the nearest plain wire's copper
    // lies to the pad, the feedline's run along it and the lead.
    {
      const std::uint32_t farthest = 19;
      const auto& stencil = stencilFor(farthest);
      const auto width = static_cast<std::int64_t>(scene_.router.width);
      const auto height = static_cast<std::int64_t>(scene_.router.height);
      for (std::uint32_t index = 0; index < couplers_.size(); ++index) {
        const auto& coupler = couplers_[index];
        const auto [chain, at] = chainOfCoupler_[index];
        if (chain == NO_OWNER) {
          continue;
        }
        const auto& option = coupler.options[coupler.chosen];
        // The option's own cells, each with the part it belongs to.
        std::vector<std::pair<std::size_t, const char*>> own;
        for (const auto cell : option.body) {
          own.emplace_back(cell, "pad");
        }
        for (const auto& cell : option.arc) {
          if (cell.x < scene_.router.width && cell.y < scene_.router.height) {
            own.emplace_back(scene_.router.index(cell.x, cell.y), "lead");
          }
        }
        const auto along = routing::headingVector(option.orientation);
        const auto reach = static_cast<std::int64_t>(option.run / 2) +
                           static_cast<std::int64_t>(tuning_.straightStart);
        const std::int64_t midX =
            (static_cast<std::int64_t>(option.in.x) + option.out.x) / 2;
        const std::int64_t midY =
            (static_cast<std::int64_t>(option.in.y) + option.out.y) / 2;
        for (std::int64_t k = -reach; k <= reach; ++k) {
          const auto x = midX + (k * along.dx);
          const auto y = midY + (k * along.dy);
          if (x >= 0 && y >= 0 && x < width && y < height) {
            own.emplace_back(static_cast<std::size_t>((y * width) + x), "run");
          }
        }
        // The nearest plain wire to the pad and the lead, and apart from it
        // the nearest to the feedline's run along the pad: a plain wire
        // may cross the run's stubs — that is where it crosses its bridged
        // edge — so a wire at the run says nothing about the room, and the
        // counts below are over the pad and the lead alone.
        struct Nearest {
          double distance = std::numeric_limits<double>::infinity();
          std::uint32_t who = NO_OWNER;
          const char* part = "";
          std::int64_t x = 0;
          std::int64_t y = 0;
        };
        Nearest own_;
        Nearest run_;
        for (const auto& [cell, name] : own) {
          Nearest& best = name[0] == 'r' ? run_ : own_;
          const auto cx = static_cast<std::int64_t>(cell % scene_.router.width);
          const auto cy = static_cast<std::int64_t>(cell / scene_.router.width);
          for (const auto& [dx, dy] : stencil.full) {
            const auto d =
                std::sqrt(static_cast<double>((dx * dx) + (dy * dy)));
            if (d >= best.distance) {
              continue;
            }
            const auto x = cx + dx;
            const auto y = cy + dy;
            if (x < 0 || y < 0 || x >= width || y >= height) {
              continue;
            }
            const auto owner =
                field_.owner(static_cast<std::size_t>((y * width) + x));
            if (owner == NO_OWNER || owner == coupler.wire ||
                owner >= conventional_.size() || conventional_[owner] == 0) {
              continue;
            }
            best = {
                .distance = d, .who = owner, .part = name, .x = cx, .y = cy};
          }
        }
        const auto said = [&](const Nearest& found, const char* what) {
          if (found.who == NO_OWNER) {
            return std::format("no plain wire within {} cells of its {}",
                               farthest, what);
          }
          return std::format("plain wire {} lies {:.1f} cells from its {} at "
                             "({},{})",
                             wireId(wires[found.who]), found.distance,
                             found.part, found.x, found.y);
        };
        if (own_.who != NO_OWNER) {
          room_.r3Near[0] += own_.distance <= 0.0 ? 1 : 0;
          room_.r3Near[1] += own_.distance <= 5.0 ? 1 : 0;
          room_.r3Near[2] += own_.distance <= 10.0 ? 1 : 0;
          room_.r3Near[3] += own_.distance <= 19.0 ? 1 : 0;
        }
        tell(std::format(
            "coupler insertion: room R3 chain {} coupler '{}' option {}: {}"
            "{}; {}",
            chain, wireId(wires[coupler.wire]), coupler.chosen,
            said(own_, "pad or lead"),
            own_.who == NO_OWNER
                ? std::string{}
                : std::format(
                      " — R3 would refuse at a reach of {} or more",
                      static_cast<std::uint32_t>(std::ceil(own_.distance))),
            said(run_, "run")));
      }
    }
    checkChainChannels(wires);
  }

  /// The room a way leaves beside it, measured as `reportSqueeze` says:
  /// the worst of the straight lines from every `squeezeStep`-th cell past
  /// `skipStart` and before `skipEnd`, to either side at a right angle, to
  /// the first artwork cell inside the launcher rectangle. `selfKey` is the
  /// wire whose own copper is not a wire in the channel. Returns the
  /// shortfall in cells, zero when nothing is short, and what was found.
  [[nodiscard]] std::pair<double, Squeeze>
  measureSqueeze(const std::vector<Wire>& wires, const Path& way,
                 const std::size_t skipStart, const std::size_t skipEnd,
                 const std::uint32_t selfKey) const {
    double worst = 0.0;
    Squeeze found;
    if (way.size() < 3 || skipStart + skipEnd >= way.size()) {
      return {worst, found};
    }
    const auto pitch = roomPitch();
    const auto clearance = tuning_.clearance;
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto height = static_cast<std::int64_t>(scene_.router.height);
    const auto reach = SQUEEZE_REACH;
    // The rectangle the launcher cells span: the artwork inside it is the
    // circuit, the artwork outside it the launcher pads.
    std::int64_t inMinX = width;
    std::int64_t inMinY = height;
    std::int64_t inMaxX = -1;
    std::int64_t inMaxY = -1;
    for (const auto& [port, slot] : scene_.launcherCell) {
      const auto place = scene_.router.cell(slot);
      inMinX = std::min<std::int64_t>(inMinX, place.x());
      inMaxX = std::max<std::int64_t>(inMaxX, place.x());
      inMinY = std::min<std::int64_t>(inMinY, place.y());
      inMaxY = std::max<std::int64_t>(inMaxY, place.y());
    }
    const auto inside = [&](const std::int64_t x, const std::int64_t y) {
      return x > inMinX && x < inMaxX && y > inMinY && y < inMaxY;
    };
    // A resonator whose coupler is not applied yet holds its whole outer
    // way in the field, the tail past the anchor included — copper the cut
    // takes away. Measured at search time that tail counted as a wire in
    // the channel, and on 17q a chain whose edges were squeezed by nothing
    // at the end paid 40000 to keep clear of its own resonators' tails
    // (user, 2026-10-05). So such a resonator counts only where the cell
    // lies on its way as the chosen option cuts it, `wayOfResonator`.
    std::unordered_map<std::uint32_t, std::unordered_set<std::size_t>> cut;
    const auto onCutWay = [&](const std::uint32_t owner,
                              const std::size_t index) {
      const auto coupler = couplerOfWire_.find(owner);
      if (coupler == couplerOfWire_.end()) {
        return true;
      }
      const auto& wire = wires[owner];
      if (wire.couplerAtSource == coupler->second && wire.drawn) {
        return true;
      }
      auto found = cut.find(owner);
      if (found == cut.end()) {
        std::unordered_set<std::size_t> cells;
        for (const auto& point : wayOfResonator(wire)) {
          if (point.x < scene_.router.width && point.y < scene_.router.height) {
            cells.insert(scene_.router.index(point.x, point.y));
          }
        }
        found = cut.emplace(owner, std::move(cells)).first;
      }
      return found->second.contains(index);
    };
    for (std::size_t at = skipStart; at + skipEnd < way.size();
         at += SQUEEZE_STEP) {
      const auto& cell = way[at];
      for (const int side : {2, -2}) {
        const auto v =
            routing::headingVector(routing::turned(cell.heading, side));
        if (v.dx == 0 && v.dy == 0) {
          continue;
        }
        const bool diagonal = v.dx != 0 && v.dy != 0;
        std::unordered_set<std::uint32_t> owners;
        bool wall = false;
        std::int64_t x = cell.x;
        std::int64_t y = cell.y;
        std::uint32_t steps = 0;
        const auto noteOwner = [&](const std::int64_t ox,
                                   const std::int64_t oy) {
          const auto index = static_cast<std::size_t>((oy * width) + ox);
          const auto owner = field_.owner(index);
          if (owner != NO_OWNER && owner != selfKey && owner < wires.size() &&
              onCutWay(owner, index)) {
            owners.insert(owner);
          }
        };
        for (std::uint32_t s = 1; s <= reach; ++s) {
          x += v.dx;
          y += v.dy;
          if (!inside(x, y)) {
            break;
          }
          const auto index = static_cast<std::size_t>((y * width) + x);
          if (scene_.components.test(index) && !bodies_.test(index)) {
            wall = true;
            steps = s;
            break;
          }
          noteOwner(x, y);
          if (diagonal) {
            // A way may pass between two diagonal cells without standing
            // on either; the two axis neighbours of the step catch it.
            noteOwner(x - v.dx, y);
            noteOwner(x, y - v.dy);
          }
        }
        if (!wall || owners.empty()) {
          continue;
        }
        const auto k = static_cast<std::uint32_t>(owners.size());
        const double length =
            static_cast<double>(steps) * (diagonal ? std::numbers::sqrt2 : 1.0);
        const double need = static_cast<double>(clearance) +
                            static_cast<double>(k) * static_cast<double>(pitch);
        const double shortfall = need - length;
        if (shortfall > worst) {
          worst = shortfall;
          std::vector<std::uint32_t> keys(owners.begin(), owners.end());
          std::ranges::sort(keys);
          found.from = cell;
          found.to = {.x = static_cast<std::uint32_t>(x),
                      .y = static_cast<std::uint32_t>(y),
                      .heading = cell.heading,
                      .primitive = 0};
          found.note = std::format(
              "squeezed at ({},{}): {} wire{} ({}) in {:.0f} cells to the "
              "artwork, need {:.0f} ({} + {} x {})",
              cell.x, cell.y, k, k == 1 ? "" : "s", namesOf(wires, keys),
              length, need, clearance, k, pitch);
        }
      }
    }
    return {worst, found};
  }

  /// The room every edge leaves beside it, measured at the end of the
  /// insertion (user, 2026-10-05): from every `squeezeStep`-th cell of the
  /// edge, past its two runs at the couplers, a straight line to either
  /// side at a right angle to the edge's heading, as far as the first cell
  /// of the artwork **inside the chip** — the wall of the channel: a qubit
  /// or a tunable coupler, never a launcher pad, which lies outside the
  /// rectangle the launcher cells span, and never a CPW coupler body the
  /// insertion put there (user, 2026-10-05: the room in question is the
  /// room between the feedlines and the circuit, not the room at the
  /// border). A line that leaves that rectangle measures nothing. Every
  /// distinct wire whose copper the line crosses is a wire that has to pass
  /// through that channel, and `k` of them need `clearance + k · pitch`
  /// cells: the feedline's own exclusion zone and one pitch each. A line
  /// that finds no wall within `squeezeReach`, or no wire, measures nothing.
  /// The worst line of an edge is its figure; an edge whose worst line is
  /// short of what its wires need is `Squeezed`, said at `-v 1` and kept in
  /// `squeezed_` for the artifact. This is the bottleneck the feedline pass
  /// fails in afterwards, read before it has run and without the repair's
  /// trials. Report only: no search refuses a way for it.
  static constexpr std::uint32_t SQUEEZE_STEP = 5;
  static constexpr std::uint32_t SQUEEZE_REACH = 300;
  void reportSqueeze(const std::vector<Wire>& wires) {
    squeezed_.clear();
    std::vector<std::string> marked;
    for (const auto& edge : edges_) {
      if (edge.wire >= wires.size()) {
        continue;
      }
      const auto& wire = wires[edge.wire];
      if (!wire.drawn) {
        continue;
      }
      const auto [worst, found] = measureSqueeze(
          wires, wire.way,
          static_cast<std::size_t>(startStubOf(wire, tuning_.straightStart) +
                                   BEND_RADIUS),
          static_cast<std::size_t>(wire.endStub + BEND_RADIUS), wire.key);
      if (worst > 0.0) {
        marked.push_back(std::format("f{} ({})", wire.slot, found.note));
        tell(std::format("coupler insertion: squeeze chain {} edge {}->{} "
                         "(f{}): {}, short by {:.0f}, the line ends at ({},{})",
                         edge.chain, edge.from, edge.to, wire.slot, found.note,
                         worst, found.to.x, found.to.y));
        squeezed_[wire.key] = found;
      }
    }
    std::string list;
    for (const auto& one : marked) {
      list += (list.empty() ? "" : " · ") + one;
    }
    say(std::format("coupler insertion: SQUEEZE — {} of {} edges leave too "
                    "little room beside them for the wires that have to "
                    "pass (pitch {}, clearance {}, step {}, reach {}){}{}",
                    marked.size(), edges_.size(), roomPitch(),
                    tuning_.clearance, SQUEEZE_STEP, SQUEEZE_REACH,
                    marked.empty() ? "" : ": ", list));
  }

  /// What an obstacle of `reportBottlenecks` is.
  enum class WallKind : std::uint8_t {
    Border,
    Artwork,
    Coupler,
    Stub,
    Feedline
  };

  [[nodiscard]] static std::string_view kindName(const WallKind kind) {
    switch (kind) {
    case WallKind::Border:
      return "border";
    case WallKind::Artwork:
      return "artwork";
    case WallKind::Coupler:
      return "coupler pad";
    case WallKind::Stub:
      return "stub";
    case WallKind::Feedline:
      return "feedline";
    }
    return "?";
  }

  /// One obstacle of `reportBottlenecks`: what it is, how the log names it,
  /// and the wire it belongs to — the feedline edge, the resonator of a pad,
  /// the wire a stub or a lead is the end of — or NO_OWNER.
  struct Wall {
    WallKind kind = WallKind::Artwork;
    std::string name;
    std::uint32_t wire = NO_OWNER;
    /// A flank of a resonator's lead: the stub at its coupler.
    bool lead = false;
    /// A flank of the port run at a wire's target, not its source.
    bool target = false;
    /// The port run a flank belongs to, by the index of its left flank;
    /// `NO_OWNER` for a wall that is no port run.
    std::uint32_t run = NO_OWNER;
    /// A flank of the run a terminal edge keeps as a wire of the capacity
    /// graph (`terminalDemands`): the insertion has a hand in it, as in a
    /// pad or a lead.
    bool terminalRun = false;
  };

  /// A port run of the bottleneck analysis: the middle line of the copper
  /// a wire has no choice about at one end, in fractional cells, from the
  /// port to the terminal — the end the free routing starts from.
  using PortRun = std::vector<std::pair<double, double>>;

  /// The run a wire leaves its source on. A plain wire's is the rule's
  /// straight length from the point the assignment feeds it at. A
  /// resonator's is the lead it already has — the coupling run and the
  /// quarter turn off its coupler, to where its search starts — with
  /// nothing added.
  [[nodiscard]] PortRun sourceRunOf(const Wire& wire) const {
    const auto& source = wire.objective.source;
    if (wire.couplerAtSource != NO_OWNER && !wire.arc.empty()) {
      PortRun run;
      for (const auto& point : wire.arc) {
        run.emplace_back(point.x, point.y);
      }
      if (!wire.arc.back().samePlace(source)) {
        run.emplace_back(source.x, source.y);
      }
      if (run.size() == 1) {
        run.push_back(run.front());
      }
      return run;
    }
    return straightRun(static_cast<double>(source.x),
                       static_cast<double>(source.y), source.heading);
  }

  /// The run a wire arrives at its target port on: the rule's straight
  /// length from the port itself. The router's target cell already lies
  /// past the port's band, that length out, so the run is measured from
  /// the port and not from that cell.
  [[nodiscard]] PortRun targetRunOf(const Wire& wire) const {
    const auto& target = wire.objective.target;
    const auto out = routing::reverse(target.heading);
    if (wire.targetPort < ports_.size()) {
      return straightRun(ports_[wire.targetPort].fx, ports_[wire.targetPort].fy,
                         out);
    }
    return straightRun(static_cast<double>(target.x),
                       static_cast<double>(target.y), out);
  }

  /// `straightLength` from a point on a heading.
  [[nodiscard]] PortRun straightRun(const double x, const double y,
                                    const Heading heading) const {
    return straightRun(x, y, heading, tuning_.straightLength);
  }

  /// A run of `length` cells from a point on a heading.
  [[nodiscard]] static PortRun straightRun(const double x, const double y,
                                           const Heading heading,
                                           const double length) {
    const auto v = routing::headingVector(heading);
    const auto norm =
        std::hypot(static_cast<double>(v.dx), static_cast<double>(v.dy));
    return {{x, y},
            {x + (static_cast<double>(v.dx) / norm * length),
             y + (static_cast<double>(v.dy) / norm * length)}};
  }

  /// The run a terminal edge keeps at one of its ends, as a wire of the
  /// capacity graph (`terminalDemands`): the straight the edge search holds
  /// it to there (`runsOfEdge`), the coupler's own run at a coupler and the
  /// rule's straight start at a launcher (user, 2026-10-14). `target` picks
  /// the end the edge arrives at.
  [[nodiscard]] PortRun terminalRunOf(const Edge& edge, const Wire& wire,
                                      const bool target) const {
    const auto [atSource, atTarget] = runsOfEdge(edge);
    // The runs are steps along the heading, a diagonal step √2 cells long
    // (user, 2026-10-14: a diagonal run of 15 steps came out 11 steps
    // long).
    const auto length = [&](const std::uint32_t steps, const Heading heading) {
      const auto v = routing::headingVector(heading);
      return static_cast<double>(steps) *
             std::hypot(static_cast<double>(v.dx), static_cast<double>(v.dy));
    };
    if (target) {
      const auto& end = wire.objective.target;
      const auto out = routing::reverse(end.heading);
      return straightRun(static_cast<double>(end.x), static_cast<double>(end.y),
                         out, length(atTarget, out));
    }
    const auto& start = wire.objective.source;
    return straightRun(static_cast<double>(start.x),
                       static_cast<double>(start.y), start.heading,
                       length(atSource, start.heading));
  }

  /// The obstacles `reportBottlenecks` builds the medial axis over: the
  /// mask, the obstacle every blocked cell belongs to, and the lines of
  /// the stubs, leads and feedlines for the picture, which a mask one cell
  /// wide shows too thin to read. With them the slots of the port runs —
  /// the free cells between the two walls of each — and per outer wire the
  /// cell its terminal lies on at each end, source first: a cell of the
  /// slot.
  struct Walls {
    grid::BitGrid mask;
    std::vector<std::uint32_t> of;
    std::vector<Wall> list;
    std::vector<std::pair<std::uint32_t, std::vector<Cell>>> strokes;
    std::vector<std::size_t> slots;
    /// Every cell of every slot, also those a feedline edge took after:
    /// where a wire runs on its port run.
    std::vector<std::size_t> runSlots;
    std::unordered_map<std::uint32_t, std::array<std::size_t, 2>> terminals;
    /// The cells whose label this pass wrote: every cell of a wall but the
    /// border and the artwork (`baseWalls`).
    grid::BitGrid drawn;
    /// The option every coupler's pad stands on, by coupler, when the walls
    /// are those of a state (`InsertionState::options`); empty when the
    /// pads stand on the options the couplers hold.
    std::vector<std::size_t> padOptions;
  };

  /// One bottleneck as `reportBottlenecks` measured it: its length in
  /// cells, how many wires it holds by the rule `reportSqueeze` uses, and
  /// which wires cross it on the ways they have now.
  struct MeasuredBottleneck {
    grid::Bottleneck gate;
    double cells = 0.0;
    std::uint32_t holds = 0;
    std::vector<std::uint32_t> crossing;
    /// A mouth between two chains closed by a line (`mouthGates`).
    bool mouth = false;
    /// The mouth of the slot of a terminal edge's run (`slotGates`): one
    /// wire, the edge's own.
    bool slot = false;
  };

  /// The obstacles of the chip as the insertion leaves it (user,
  /// 2026-10-07): everything outside the rectangle the launcher cells span
  /// and the outermost ring of cells (the border), the artwork inside that
  /// rectangle, the coupler pads, the port runs every outer wire has no
  /// choice about, and every feedline edge drawn. A port run
  /// (`sourceRunOf`, `targetRunOf`) is the rule's straight length from a
  /// plain wire's feed point or from a target port, and a resonator's lead
  /// as it stands. The port runs and the feedline edges are copper that
  /// keeps a clearance of its own, so each keeps half the clearance to
  /// either side (user, 2026-10-07); a gap between two of them has already
  /// given up a whole clearance. A port run is the two sides of a band with
  /// square ends, two walls one cell thick with a slot between them where
  /// its terminal lies (user, 2026-10-08); a feedline edge is inflated by a
  /// disc. The room a later obstacle keeps takes only free cells, a
  /// feedline's own line any cell. The ways of the other wires are
  /// no obstacle: they are what has to pass through. The feedline edge
  /// whose wire is `without` is left out.
  [[nodiscard]] Walls
  wallsAfterInsertion(const std::vector<Wire>& wires,
                      const std::uint32_t without = NO_OWNER) const {
    return wallsOf(wires, edges_, nullptr, without);
  }

  /// What a port run takes of the grid, from its geometry alone: the
  /// cells of its left and right walls and of its slot, in the order
  /// `wallsOf` takes them, and its terminal. A cell another wall already
  /// holds is skipped when the stamp is laid, as the walls are built.
  enum class RunCell : std::uint8_t { Left, Right, Slot };
  struct RunStamp {
    PortRun run;
    std::vector<std::pair<std::size_t, RunCell>> cells;
    std::vector<Cell> line;
    std::size_t terminal = 0;
  };
  /// The stamps of the port runs, by wire and end, kept while the run's
  /// line stays the same: the capacity check builds the walls thousands of
  /// times, and only the leads change.
  mutable std::unordered_map<std::uint64_t, RunStamp> runStamps_;

  [[nodiscard]] const RunStamp& runStampOf(const PortRun& run,
                                           const std::uint64_t key) const {
    if (const auto found = runStamps_.find(key);
        found != runStamps_.end() && found->second.run == run) {
      return found->second;
    }
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto height = static_cast<std::int64_t>(scene_.router.height);
    const auto half = wallInflation();
    RunStamp stamp{.run = run, .cells = {}, .line = {}, .terminal = 0};
    double total = 0.0;
    double x0 = run.front().first;
    double x1 = x0;
    double y0 = run.front().second;
    double y1 = y0;
    for (std::size_t k = 0; k < run.size(); ++k) {
      const auto [px, py] = run[k];
      if (k > 0) {
        total += std::hypot(px - run[k - 1].first, py - run[k - 1].second);
      }
      x0 = std::min(x0, px);
      x1 = std::max(x1, px);
      y0 = std::min(y0, py);
      y1 = std::max(y1, py);
    }
    // Where each cell of a box around the band lies. The box reaches a
    // cell past the band on every side, so every neighbour of a band cell
    // is in it; it is not clipped to the grid.
    enum class Place : std::uint8_t {
      Aside,
      BeyondPort,
      BeyondEnd,
      Left,
      Right
    };
    const auto reach = half + 1.0;
    const auto fromX = static_cast<std::int64_t>(std::floor(x0 - reach));
    const auto fromY = static_cast<std::int64_t>(std::floor(y0 - reach));
    const auto boxWidth =
        static_cast<std::int64_t>(std::ceil(x1 + reach)) - fromX + 1;
    const auto boxHeight =
        static_cast<std::int64_t>(std::ceil(y1 + reach)) - fromY + 1;
    std::vector<Place> places(static_cast<std::size_t>(boxWidth * boxHeight),
                              Place::Aside);
    const auto placeAt = [&](const std::int64_t x, const std::int64_t y) {
      return places[static_cast<std::size_t>(((y - fromY) * boxWidth) +
                                             (x - fromX))];
    };
    for (std::size_t at = 0; total > 0.0 && at < places.size(); ++at) {
      const auto cx = static_cast<double>(
          fromX + (static_cast<std::int64_t>(at) % boxWidth));
      const auto cy = static_cast<double>(
          fromY + (static_cast<std::int64_t>(at) / boxWidth));
      // The nearest point of the line: how far, on which side, and
      // whether it is an end seen from beyond.
      double best = std::numeric_limits<double>::max();
      double side = 0.0;
      auto beyond = Place::Aside;
      for (std::size_t k = 0; k + 1 < run.size(); ++k) {
        const auto ax = run[k].first;
        const auto ay = run[k].second;
        const auto dx = run[k + 1].first - ax;
        const auto dy = run[k + 1].second - ay;
        const auto squared = (dx * dx) + (dy * dy);
        const auto raw = squared > 0.0
                             ? (((cx - ax) * dx) + ((cy - ay) * dy)) / squared
                             : 0.0;
        const auto t = std::clamp(raw, 0.0, 1.0);
        const auto distance =
            std::hypot(cx - (ax + (t * dx)), cy - (ay + (t * dy)));
        if (distance < best) {
          best = distance;
          side = (dx * (cy - ay)) - (dy * (cx - ax));
          beyond = k == 0 && raw < 0.0                ? Place::BeyondPort
                   : k + 2 == run.size() && raw > 1.0 ? Place::BeyondEnd
                                                      : Place::Aside;
        }
      }
      if (beyond != Place::Aside) {
        places[at] = beyond;
      } else if (best <= half) {
        places[at] = side < 0.0 ? Place::Right : Place::Left;
      }
    }
    for (std::int64_t y = fromY + 1; y + 1 < fromY + boxHeight; ++y) {
      for (std::int64_t x = fromX + 1; x + 1 < fromX + boxWidth; ++x) {
        const auto place = placeAt(x, y);
        if ((place != Place::Left && place != Place::Right) || x < 0 || y < 0 ||
            x >= width || y >= height) {
          continue;
        }
        const auto cell = static_cast<std::size_t>((y * width) + x);
        bool edge = false;
        for (std::int64_t dy = -1; dy <= 1 && !edge; ++dy) {
          for (std::int64_t dx = -1; dx <= 1 && !edge; ++dx) {
            const auto next = placeAt(x + dx, y + dy);
            edge = next == Place::Aside || next == Place::BeyondPort;
          }
        }
        stamp.cells.emplace_back(cell, !edge                   ? RunCell::Slot
                                       : place == Place::Right ? RunCell::Right
                                                               : RunCell::Left);
      }
    }
    for (const auto& [px, py] : run) {
      stamp.line.emplace_back(std::lround(px), std::lround(py));
    }
    // The terminal: on the line, a cell short of its end, in the slot
    // between the two walls.
    const auto& end = run.back();
    const auto& before = run[run.size() - 2];
    const auto length =
        std::max(1.0e-9, std::hypot(end.first - before.first,
                                    end.second - before.second));
    const auto tx = std::clamp<std::int64_t>(
        std::lround(end.first - ((end.first - before.first) / length)), 0,
        width - 1);
    const auto ty = std::clamp<std::int64_t>(
        std::lround(end.second - ((end.second - before.second) / length)), 0,
        height - 1);
    stamp.terminal = static_cast<std::size_t>((ty * width) + tx);
    return runStamps_.insert_or_assign(key, std::move(stamp)).first->second;
  }

  /// The border (everything outside the rectangle the launcher cells span,
  /// and the outermost ring of cells) and the artwork inside it, labelled
  /// as the walls 0 and 1 of `wallsOf`. They depend on the chip alone and
  /// are built once.
  struct BaseWalls {
    std::vector<std::uint32_t> of;
    grid::BitGrid mask;
  };
  mutable std::optional<BaseWalls> baseWalls_;

  [[nodiscard]] const BaseWalls& baseWalls() const {
    if (baseWalls_) {
      return *baseWalls_;
    }
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto height = static_cast<std::int64_t>(scene_.router.height);
    BaseWalls walls{
        .of = std::vector<std::uint32_t>(scene_.router.cells(), NO_OWNER),
        .mask = grid::BitGrid(scene_.router.width, scene_.router.height)};
    const auto mark = [&](const std::int64_t x, const std::int64_t y,
                          const std::uint32_t wall) {
      const auto cell = static_cast<std::size_t>((y * width) + x);
      walls.mask.set(cell);
      walls.of[cell] = wall;
    };
    std::int64_t inMinX = 0;
    std::int64_t inMinY = 0;
    std::int64_t inMaxX = width - 1;
    std::int64_t inMaxY = height - 1;
    if (!scene_.launcherCell.empty()) {
      inMinX = width;
      inMinY = height;
      inMaxX = -1;
      inMaxY = -1;
      for (const auto& [port, slot] : scene_.launcherCell) {
        const auto place = scene_.router.cell(slot);
        inMinX = std::min<std::int64_t>(inMinX, place.x());
        inMaxX = std::max<std::int64_t>(inMaxX, place.x());
        inMinY = std::min<std::int64_t>(inMinY, place.y());
        inMaxY = std::max<std::int64_t>(inMaxY, place.y());
      }
    }
    constexpr std::uint32_t border = 0;
    for (std::int64_t y = 0; y < height; ++y) {
      for (std::int64_t x = 0; x < width; ++x) {
        if (x <= 0 || y <= 0 || x >= width - 1 || y >= height - 1 ||
            x < inMinX || x > inMaxX || y < inMinY || y > inMaxY) {
          mark(x, y, border);
        }
      }
    }

    // The artwork inside the launcher cells is one obstacle: the qubits and
    // the couplers between them touch. `endName` names a cell of it by the
    // nearest port.
    constexpr std::uint32_t artwork = 1;
    for (std::size_t cell = 0; cell < walls.of.size(); ++cell) {
      if (scene_.components.test(cell) && walls.of[cell] == NO_OWNER) {
        walls.mask.set(cell);
        walls.of[cell] = artwork;
      }
    }
    baseWalls_ = std::move(walls);
    return *baseWalls_;
  }

  /// The obstacles of `wallsAfterInsertion` for a state of the insertion:
  /// the feedline edges `edges` (their ways in `wires`), and the pads of
  /// the couplers `placed` marks, every coupler when it is null.
  ///
  /// With `staticOnly` the walls are those that stay through an insertion:
  /// the border, the artwork and every port run but a resonator's lead,
  /// with no pad and no feedline edge. With `terminalsOpen` the terminal
  /// edges are no walls either: the check of the whole chip takes them as
  /// wires (`terminalDemands`). Without it a terminal edge is a wall one
  /// cell to either side of its line, not inflated.
  [[nodiscard]] Walls
  wallsOf(const std::vector<Wire>& wires, const std::vector<Edge>& edges,
          const std::vector<bool>* placed, const std::uint32_t without,
          const bool staticOnly = false, const bool terminalsOpen = false,
          const std::vector<std::size_t>* options = nullptr) const {
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto height = static_cast<std::int64_t>(scene_.router.height);
    Walls walls{.mask =
                    grid::BitGrid(scene_.router.width, scene_.router.height),
                .of = std::move(spareWallOf_),
                .list = {},
                .strokes = {}};
    const auto add = [&walls](const WallKind kind, std::string name,
                              const std::uint32_t wire) {
      walls.list.push_back(
          {.kind = kind, .name = std::move(name), .wire = wire});
      return static_cast<std::uint32_t>(walls.list.size() - 1);
    };
    const auto mark = [&](const std::int64_t x, const std::int64_t y,
                          const std::uint32_t wall) {
      if (x < 0 || y < 0 || x >= width || y >= height) {
        return;
      }
      const auto cell = static_cast<std::size_t>((y * width) + x);
      walls.mask.set(cell);
      walls.of[cell] = wall;
      walls.drawn.set(cell);
    };
    // A line through cells as an obstacle, inflated by `wallInflation`: the
    // line takes its cells whatever holds them, the disc around each cell
    // only the free ones.
    const auto disc = inflationStencil();
    // A terminal edge that stays a wall (not `terminalsOpen`) is drawn one
    // cell to either side of its line and no more: the feedline routing
    // lays it again, so it closes the chain's pocket but keeps no room.
    const std::vector<std::pair<std::int64_t, std::int64_t>> thin{
        {0, 0}, {-1, 0}, {1, 0}, {0, -1}, {0, 1}};
    const auto markThick = [&](const std::vector<Cell>& points,
                               const std::uint32_t wall,
                               const bool terminal = false) {
      std::vector<std::size_t> line;
      for (std::size_t at = 0; at < points.size(); ++at) {
        const auto [x, y] = points[at];
        const auto [px, py] = at == 0 ? points[at] : points[at - 1];
        for (const auto cell : grid::lineCells(
                 px, py, x, y, scene_.router.width, scene_.router.height)) {
          line.push_back(cell);
        }
      }
      for (const auto cell : line) {
        const auto cx = static_cast<std::int64_t>(cell) % width;
        const auto cy = static_cast<std::int64_t>(cell) / width;
        for (const auto& [dx, dy] : terminal ? thin : disc) {
          const auto x = cx + dx;
          const auto y = cy + dy;
          if (x >= 0 && y >= 0 && x < width && y < height &&
              walls.of[static_cast<std::size_t>((y * width) + x)] == NO_OWNER) {
            mark(x, y, wall);
          }
        }
      }
      for (const auto cell : line) {
        walls.mask.set(cell);
        walls.of[cell] = wall;
        walls.drawn.set(cell);
      }
      walls.strokes.emplace_back(wall, points);
    };

    // The border and the artwork inside the launcher cells, as
    // `baseWalls` keeps them: the walls 0 and 1.
    add(WallKind::Border, "border", NO_OWNER);
    add(WallKind::Artwork, "artwork", NO_OWNER);
    {
      const auto& base = baseWalls();
      walls.of.assign(base.of.begin(), base.of.end());
      walls.mask = base.mask;
      walls.drawn = grid::BitGrid(scene_.router.width, scene_.router.height);
    }

    // A state names an option for every coupler; anything else leaves the
    // pads on the options the couplers hold.
    if (options != nullptr && options->size() == couplers_.size()) {
      walls.padOptions = *options;
    }
    for (std::size_t index = 0; index < couplers_.size(); ++index) {
      const auto& coupler = couplers_[index];
      const auto option =
          walls.padOptions.empty() ? coupler.chosen : walls.padOptions[index];
      if (staticOnly || option >= coupler.options.size() ||
          coupler.wire >= wires.size() ||
          (placed != nullptr && !(*placed)[index])) {
        continue;
      }
      const auto pad = add(
          WallKind::Coupler,
          std::format("pad of {}", wireId(wires[coupler.wire])), coupler.wire);
      for (const auto cell : coupler.options[option].body) {
        walls.mask.set(cell);
        walls.of[cell] = pad;
        walls.drawn.set(cell);
      }
    }

    // The port runs (user, 2026-10-08). A run keeps the band of half the
    // clearance to either side of its middle line, with square ends: a
    // cell whose nearest point on the line is one of its ends from beyond
    // is not in it. The run is the edge of that band and nothing more: two
    // walls one cell thick, left and right of the line, made of the cells
    // of the band with a neighbour, diagonal ones included, outside it to
    // the side or beyond the port end. So each wall is 4-connected and lies
    // where the band's edge lies, and a gap between two runs keeps its
    // length. The end at the port is closed because the port's own copper
    // lies there; open, the space between the walls would join the room
    // behind the run through the cell or two between the band and the port
    // (on 17q a row of cells between every launcher run and the border).
    // The end at the terminal is open. The cells between the two walls are
    // free and are the slot the terminal lies in: the medial axis runs into
    // it, and no bottleneck crosses it. A wall takes only free cells, and a
    // run of no length keeps no room.
    const auto markRun = [&](const PortRun& run, const std::string& name,
                             const std::uint32_t wire, const bool lead,
                             const bool target) {
      const auto left = add(WallKind::Stub, name + ", left", wire);
      const auto right = add(WallKind::Stub, name + ", right", wire);
      for (const auto flank : {left, right}) {
        walls.list[flank].lead = lead;
        walls.list[flank].target = target;
        walls.list[flank].run = left;
      }
      const auto& stamp = runStampOf(
          run, (static_cast<std::uint64_t>(wire) << 1U) | (target ? 1U : 0U));
      for (const auto& [cell, place] : stamp.cells) {
        if (walls.of[cell] != NO_OWNER) {
          continue;
        }
        if (place == RunCell::Slot) {
          walls.slots.push_back(cell);
        } else {
          walls.mask.set(cell);
          walls.of[cell] = place == RunCell::Right ? right : left;
          walls.drawn.set(cell);
        }
      }
      walls.strokes.emplace_back(left, stamp.line);
      return stamp.terminal;
    };
    for (const auto& wire : wires) {
      if (!wire.feasible || wire.inner || wire.feedline) {
        continue;
      }
      const auto resonator =
          wire.couplerAtSource != NO_OWNER && !wire.arc.empty();
      if (staticOnly && wire.resonator) {
        const auto target = markRun(
            targetRunOf(wire), std::format("{} at its target", wireId(wire)),
            wire.key, false, true);
        walls.terminals.emplace(wire.key, std::array{target, target});
        continue;
      }
      const auto source =
          markRun(sourceRunOf(wire),
                  resonator ? std::format("lead of {}", wireId(wire))
                            : std::format("{} at its source", wireId(wire)),
                  wire.key, resonator, false);
      const auto target = markRun(targetRunOf(wire),
                                  std::format("{} at its target", wireId(wire)),
                                  wire.key, false, true);
      walls.terminals.emplace(wire.key, std::array{source, target});
    }
    // A terminal edge taken as a wire (`terminalsOpen`) keeps its port
    // runs as every other wire does: at the launcher and at the coupler's
    // port (user, 2026-10-14).
    if (terminalsOpen) {
      for (const auto& edge : edges) {
        if (!edge.terminal || edge.wire >= wires.size() ||
            !wires[edge.wire].feasible) {
          continue;
        }
        const auto& wire = wires[edge.wire];
        const auto& points = chains_[edge.chain];
        const auto source = markRun(
            terminalRunOf(edge, wire, false),
            std::format("{} at its {}", wireId(wire),
                        points[edge.from].fixed ? "launcher" : "coupler"),
            wire.key, false, false);
        const auto target =
            markRun(terminalRunOf(edge, wire, true),
                    std::format("{} at its {}", wireId(wire),
                                points[edge.to].fixed ? "launcher" : "coupler"),
                    wire.key, false, true);
        // Both flanks of both runs: the four walls `markRun` just added.
        for (auto flank = walls.list.size() - 4; flank < walls.list.size();
             ++flank) {
          walls.list[flank].terminalRun = true;
        }
        walls.terminals.emplace(wire.key, std::array{source, target});
      }
    }

    for (const auto& edge : edges) {
      if (edge.wire >= wires.size()) {
        continue;
      }
      const auto& wire = wires[edge.wire];
      if (staticOnly || !wire.drawn || wire.way.empty() ||
          edge.wire == without || (terminalsOpen && edge.terminal)) {
        continue;
      }
      markThick(cellsOf(wire.way),
                add(WallKind::Feedline, wireId(wire), edge.wire),
                edge.terminal);
    }
    // A feedline's own line takes any cell, so a slot keeps only the cells
    // still free.
    walls.runSlots = walls.slots;
    std::erase_if(walls.slots, [&walls](const std::size_t cell) {
      return walls.mask.test(cell);
    });
    return walls;
  }

  /// How the log names the obstacle at one end of a bottleneck: by its
  /// wall, and a cell of the artwork by the nearest port that is not a
  /// launcher, since the artwork is one piece.
  [[nodiscard]] std::string endName(const Walls& walls,
                                    const std::size_t cell) const {
    const auto& wall = walls.list[walls.of[cell]];
    if (wall.kind != WallKind::Artwork) {
      return wall.name;
    }
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto x = static_cast<std::int64_t>(cell) % width;
    const auto y = static_cast<std::int64_t>(cell) / width;
    const PortMark* nearest = nullptr;
    std::int64_t best = std::numeric_limits<std::int64_t>::max();
    for (const auto& port : ports_) {
      if (scene_.launcherCell.contains(port.index)) {
        continue;
      }
      const auto dx = port.x - x;
      const auto dy = port.y - y;
      const auto squared = (dx * dx) + (dy * dy);
      if (squared < best) {
        best = squared;
        nearest = &port;
      }
    }
    return nearest == nullptr ? wall.name
                              : std::format("artwork near {}", nearest->label);
  }

  /// Whether the check rips up and reroutes the wires that overfill an edge
  /// (round 22) and measures a detour over the gates that take a wire
  /// (round 25); `SCPD_RIP_UP`, `SCPD_DETOUR_TAKERS`.
  [[nodiscard]] static InTurnRules inTurnRules() {
    return {.mostSweeps = 8U, .detourOverTakers = true};
  }

  /// The bottlenecks the coupler insertion has a hand in (user,
  /// 2026-10-08): those with an end on a feedline edge, on a coupler pad,
  /// on the lead a resonator leaves its coupler on, or on the run of a
  /// terminal edge. A bottleneck between other obstacles — the artwork,
  /// the border, the port runs of the plain wires — is there whatever the
  /// insertion does, and is left out, so the chambers on either side of it
  /// are one. No bottleneck joins the two walls of one port run:
  /// its line would cross the slot between them, and
  /// `grid::findRoutingBottlenecks` refuses every line that crosses a slot. The
  /// mouths between the chains, closed (user, 2026-10-14). With the terminal
  /// edges as wires (`terminalDemands`) every chain stands open at its launcher
  /// ends, and the room outside one chain runs into the room inside the next
  /// round their ends; the edges between couplers then have one chamber on both
  /// sides and no stretch. So along the border, between the ending terminal
  /// edge of one chain and the starting terminal edge of the next, the outer
  /// lines between the slots of the two edges' runs at their couplers close the
  /// mouth (`mouthLinesOf`): two where the runs stand end to end, one where
  /// they stand side by side. Wherever a line runs free, between two walls it
  /// meets or from a flank's tip to the first, it is a bottleneck holding its
  /// length over the clearance; the walls it crosses close the mouth there
  /// themselves. The strip between the two lines, with the two slots the
  /// terminal edges end in, is a chamber of its own: a terminal edge enters it
  /// over the outer line from the launcher's side, a plain wire from a launcher
  /// between the two chains passes both lines on its way in. The ends of the
  /// chains are taken in the order of their launchers round the centre of the
  /// chip; two ends of one chain next to each other enclose that chain's
  /// own pocket and close no mouth.
  /// The gates of the mouths, and one note for every mouth that could not
  /// be closed and why: the check's summary carries the notes, as a mouth
  /// left open joins the room inside the chains to the room outside.
  struct MouthGates {
    std::vector<grid::Bottleneck> gates;
    std::vector<std::string> notes;
  };

  [[nodiscard]] MouthGates mouthGates(const std::vector<Wire>& wires,
                                      const std::vector<Edge>& edges,
                                      const Walls& walls) const {
    MouthGates out;
    struct End {
      bool starts = false;
      std::uint32_t chain = NO_OWNER;
      std::uint32_t wire = NO_OWNER;
      std::uint32_t coupler = NO_OWNER;
      const Edge* edge = nullptr;
      /// The chain ends at a reflector here: the coupler at that end, not a
      /// terminal edge's run, is this side of the mouth.
      bool reflector = false;
    };
    // The starting edge and the ending edge of every chain, by chain.
    std::vector<std::optional<End>> startOf(chains_.size());
    std::vector<std::optional<End>> endOf(chains_.size());
    for (const auto& edge : edges) {
      if (!edge.terminal || edge.wire >= wires.size() ||
          !wires[edge.wire].feasible || edge.chain >= chains_.size()) {
        continue;
      }
      const auto& points = chains_[edge.chain];
      const bool starts = points[edge.from].fixed && !points[edge.to].fixed;
      const bool stops = points[edge.to].fixed && !points[edge.from].fixed;
      if (!starts && !stops) {
        continue;
      }
      const auto coupler =
          starts ? points[edge.to].coupler : points[edge.from].coupler;
      if (coupler >= couplers_.size()) {
        out.notes.push_back(std::format(
            "no mouth at {}: no coupler at the end of chain {} it {}",
            wireId(wires[edge.wire]), edge.chain, starts ? "starts" : "ends"));
        continue;
      }
      (starts ? startOf : endOf)[edge.chain] = End{.starts = starts,
                                                   .chain = edge.chain,
                                                   .wire = edge.wire,
                                                   .coupler = coupler,
                                                   .edge = &edge};
    }
    // A chain with a reflector at one end has no terminal edge there; its
    // coupler at that end is that side of the mouth to the next chain (45q,
    // user, 2026-10-15). Without it the room between the two chains stays
    // open, and the edges of the chain beside it have one chamber on both
    // sides.
    for (std::uint32_t chain = 0; chain < chains_.size(); ++chain) {
      const auto& points = chains_[chain];
      if (points.size() < 2) {
        continue;
      }
      for (const bool first : {true, false}) {
        auto& slot = first ? startOf[chain] : endOf[chain];
        const auto at = first ? std::size_t{0} : points.size() - 1;
        if (slot.has_value() || points[at].fixed ||
            points[at].coupler >= couplers_.size()) {
          continue;
        }
        const auto found = std::ranges::find_if(edges, [&](const Edge& edge) {
          return edge.chain == chain &&
                 (first ? edge.from == at : edge.to == at);
        });
        if (found == edges.end()) {
          continue;
        }
        slot = End{.starts = first,
                   .chain = chain,
                   .wire = found->wire,
                   .coupler = points[at].coupler,
                   .edge = &*found,
                   .reflector = true};
      }
    }
    // The outer tips of the two flanks of an edge's run at its coupler: the
    // cell of each wall farthest from the port. The run is the target run
    // of a starting edge and the source run of an ending one.
    const auto tipsOf = [&](const End& end) {
      return runTipsOf(*end.edge, wires[end.wire], end.starts, walls);
    };
    // The two tips of the pad of a coupler at a reflector end, seen from the
    // other side of the mouth: the cells of the pad farthest out to either
    // side across the way to that side, so that the lines of the mouth pass
    // on both sides of the pad as they pass the flanks of a run.
    const auto padTipsOf = [&](const End& end,
                               const std::array<std::size_t, 2>& other)
        -> std::optional<std::array<std::size_t, 2>> {
      const auto& coupler = couplers_[end.coupler];
      const auto option = walls.padOptions.empty()
                              ? coupler.chosen
                              : walls.padOptions[end.coupler];
      if (option >= coupler.options.size()) {
        return std::nullopt;
      }
      const auto& body = coupler.options[option].body;
      if (body.empty()) {
        return std::nullopt;
      }
      const auto width = static_cast<std::int64_t>(scene_.router.width);
      const auto xOf = [&](const std::size_t cell) {
        return static_cast<double>(static_cast<std::int64_t>(cell) % width);
      };
      const auto yOf = [&](const std::size_t cell) {
        return static_cast<double>(static_cast<std::int64_t>(cell) / width);
      };
      double cx = 0.0;
      double cy = 0.0;
      for (const auto cell : body) {
        cx += xOf(cell);
        cy += yOf(cell);
      }
      cx /= static_cast<double>(body.size());
      cy /= static_cast<double>(body.size());
      const auto dx = ((xOf(other[0]) + xOf(other[1])) / 2.0) - cx;
      const auto dy = ((yOf(other[0]) + yOf(other[1])) / 2.0) - cy;
      std::optional<std::size_t> low;
      std::optional<std::size_t> high;
      double lowest = 0.0;
      double highest = 0.0;
      for (const auto cell : body) {
        const auto across = (-dy * (xOf(cell) - cx)) + (dx * (yOf(cell) - cy));
        if (!low || across < lowest) {
          lowest = across;
          low = cell;
        }
        if (!high || across > highest) {
          highest = across;
          high = cell;
        }
      }
      if (!low || !high || *low == *high) {
        return std::nullopt;
      }
      return std::array{*low, *high};
    };
    for (std::uint32_t chain = 0; chain < chains_.size(); ++chain) {
      // A mouth lies between a chain's ending edge and the next chain's
      // starting edge, in the order the assignment lays the chains round
      // the chip; at a reflector the coupler there stands for the edge.
      const auto& one = endOf[chain];
      const auto& next = startOf[(chain + 1) % chains_.size()];
      if (!one.has_value() || !next.has_value() || one->chain == next->chain) {
        continue;
      }
      if (one->reflector && next->reflector) {
        continue;
      }
      auto tipsA = one->reflector ? std::nullopt : tipsOf(*one);
      auto tipsB = next->reflector ? std::nullopt : tipsOf(*next);
      if (one->reflector && tipsB.has_value()) {
        tipsA = padTipsOf(*one, *tipsB);
      }
      if (next->reflector && tipsA.has_value()) {
        tipsB = padTipsOf(*next, *tipsA);
      }
      if (!tipsA.has_value() || !tipsB.has_value()) {
        out.notes.push_back(std::format(
            "no mouth between {} and {}: no tips at the run of {}",
            wireId(wires[one->wire]), wireId(wires[next->wire]),
            wireId(wires[tipsA.has_value() ? next->wire : one->wire])));
        continue;
      }
      const auto lines = mouthLinesOf(*tipsA, *tipsB);
      // A line is a gate wherever it runs free: from a tip, or from the
      // cell it leaves a wall on, to the cell it meets the next wall on.
      // The walls it crosses close the mouth there themselves. The lead of
      // the resonator of either coupler runs beside the edge's run, and a
      // gate between the run and its own lead would cut the slot off from
      // the mouth (user, 2026-10-14): no gate joins those two. A line from
      // the other run's tip may end on that lead (33q, user, 2026-10-15):
      // the lead stands between the two runs, and the lead, the pad and
      // the run's flanks carry the mouth on to the slot.
      const auto couplerOfLead = [&](const std::size_t cell) {
        const auto wall = walls.of[cell];
        if (wall >= walls.list.size() || !walls.list[wall].lead) {
          return NO_OWNER;
        }
        if (walls.list[wall].wire == couplers_[one->coupler].wire) {
          return one->coupler;
        }
        if (walls.list[wall].wire == couplers_[next->coupler].wire) {
          return next->coupler;
        }
        return NO_OWNER;
      };
      const auto couplerOfRun = [&](const std::size_t cell) {
        const auto wall = walls.of[cell];
        if (wall >= walls.list.size() ||
            walls.list[wall].kind != WallKind::Stub || walls.list[wall].lead) {
          return NO_OWNER;
        }
        if (walls.list[wall].wire == wires[one->wire].key) {
          return one->coupler;
        }
        if (walls.list[wall].wire == wires[next->wire].key) {
          return next->coupler;
        }
        return NO_OWNER;
      };
      const auto ownLead = [&](const std::size_t from, const std::size_t to) {
        const auto lead = couplerOfLead(from);
        const auto other = couplerOfLead(to);
        return (lead != NO_OWNER && couplerOfRun(to) == lead) ||
               (other != NO_OWNER && couplerOfRun(from) == other);
      };
      const auto before = out.gates.size();
      for (const auto& [a, b] : lines) {
        cutAtWalls(a, b, walls, ownLead, out.gates);
      }
      if (out.gates.size() == before) {
        out.notes.push_back(std::format(
            "no mouth between {} and {}: {} line{} with no free stretch",
            wireId(wires[one->wire]), wireId(wires[next->wire]), lines.size(),
            lines.size() == 1 ? "" : "s"));
      }
    }
    return out;
  }

  /// The lines that close the mouth between two runs at their couplers,
  /// each from a tip of the one run's slot to a tip of the other's. A line
  /// is an outer one when the other two tips lie on one side of it, or on
  /// it, so that no two lines cross. An outer line that runs within half a
  /// clearance of one of the other two tips, past its ends, runs across
  /// the front of a slot (user, 2026-10-14): where the four tips lie nearly
  /// on one line, as they do when the two runs stand side by side, every
  /// line but the one between the two nearest tips does, and that one
  /// alone closes the mouth. When no outer line is left, the shortest line
  /// closes it.
  [[nodiscard]] std::vector<std::pair<std::size_t, std::size_t>>
  mouthLinesOf(const std::array<std::size_t, 2>& tipsA,
               const std::array<std::size_t, 2>& tipsB) const {
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto x = [width](const std::size_t cell) {
      return static_cast<double>(static_cast<std::int64_t>(cell) % width);
    };
    const auto y = [width](const std::size_t cell) {
      return static_cast<double>(static_cast<std::int64_t>(cell) / width);
    };
    const auto apart = [&](const std::size_t a, const std::size_t b) {
      return std::hypot(x(a) - x(b), y(a) - y(b));
    };
    // How far `at` lies off the line through `from` and `to`, in cells,
    // signed by the side it lies on.
    const auto beside = [&](const std::size_t from, const std::size_t to,
                            const std::size_t at) {
      const auto dx = x(to) - x(from);
      const auto dy = y(to) - y(from);
      const auto length = std::hypot(dx, dy);
      return length > 0.0
                 ? (((x(at) - x(from)) * dy) - ((y(at) - y(from)) * dx)) /
                       length
                 : 0.0;
    };
    // How far `at` lies from the line between `from` and `to`, in cells.
    const auto awayFrom = [&](const std::size_t from, const std::size_t to,
                              const std::size_t at) {
      const auto dx = x(to) - x(from);
      const auto dy = y(to) - y(from);
      const auto squared = (dx * dx) + (dy * dy);
      const auto along = squared > 0.0 ? std::clamp((((x(at) - x(from)) * dx) +
                                                     ((y(at) - y(from)) * dy)) /
                                                        squared,
                                                    0.0, 1.0)
                                       : 0.0;
      return std::hypot(x(at) - (x(from) + (along * dx)),
                        y(at) - (y(from) + (along * dy)));
    };
    const auto half = static_cast<double>(tuning_.clearance) / 2.0;
    std::vector<std::pair<std::size_t, std::size_t>> lines;
    std::pair<std::size_t, std::size_t> shortest{tipsA[0], tipsB[0]};
    for (const std::size_t a : {0U, 1U}) {
      for (const std::size_t b : {0U, 1U}) {
        const auto from = tipsA[a];
        const auto to = tipsB[b];
        const auto otherA = tipsA[1 - a];
        const auto otherB = tipsB[1 - b];
        if (apart(from, to) < apart(shortest.first, shortest.second)) {
          shortest = {from, to};
        }
        const auto sideA = beside(from, to, otherA);
        const auto sideB = beside(from, to, otherB);
        const bool outer = std::abs(sideA) <= 1.0 || std::abs(sideB) <= 1.0 ||
                           (sideA > 0.0) == (sideB > 0.0);
        if (outer && awayFrom(from, to, otherA) >= half &&
            awayFrom(from, to, otherB) >= half) {
          lines.emplace_back(from, to);
        }
      }
    }
    if (lines.empty()) {
      lines.push_back(shortest);
    }
    return lines;
  }

  /// The outer tips of the two flanks of a terminal edge's run at one end
  /// (`terminalRunOf`): the cell of each wall farthest from the port, left
  /// then right; nothing when a flank has no cell of its own.
  [[nodiscard]] std::optional<std::array<std::size_t, 2>>
  runTipsOf(const Edge& edge, const Wire& wire, const bool target,
            const Walls& walls) const {
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto run = terminalRunOf(edge, wire, target);
    if (run.empty()) {
      return std::nullopt;
    }
    const auto& stamp = runStampOf(
        run, (static_cast<std::uint64_t>(wire.key) << 1U) | (target ? 1U : 0U));
    std::array<std::optional<std::size_t>, 2> tip;
    std::array<double, 2> far{-1.0, -1.0};
    for (const auto& [cell, place] : stamp.cells) {
      if (place == RunCell::Slot) {
        continue;
      }
      const auto wall = walls.of[cell];
      if (wall >= walls.list.size() ||
          walls.list[wall].kind != WallKind::Stub ||
          walls.list[wall].wire != wire.key) {
        continue;
      }
      const auto side = place == RunCell::Left ? 0U : 1U;
      const auto away = std::hypot(
          static_cast<double>(static_cast<std::int64_t>(cell) % width) -
              run.front().first,
          static_cast<double>(static_cast<std::int64_t>(cell) / width) -
              run.front().second);
      if (away > far[side]) {
        far[side] = away;
        tip[side] = cell;
      }
    }
    if (!tip[0].has_value() || !tip[1].has_value()) {
      return std::nullopt;
    }
    return std::array{*tip[0], *tip[1]};
  }

  /// The mouths of the slots of the terminal edges' runs, closed (user,
  /// 2026-10-14): at both ends of every terminal edge, a line from the tip
  /// of one flank of its run to the tip of the other. It is a gate for one
  /// wire, the edge's own (`capacityProblemOf`): the room in front of a
  /// slot is where the edge leaves it, and a wire that passed there would
  /// cross the edge. On 21q a wire ran between the two flanks of a terminal
  /// edge's run at its coupler, over the slot's mouth, and met the edge.
  [[nodiscard]] std::vector<grid::Bottleneck>
  slotGates(const std::vector<Wire>& wires, const std::vector<Edge>& edges,
            const Walls& walls) const {
    std::vector<grid::Bottleneck> slots;
    for (const auto& edge : edges) {
      if (!edge.terminal || edge.wire >= wires.size() ||
          !wires[edge.wire].feasible) {
        continue;
      }
      for (const bool target : {false, true}) {
        const auto tips = runTipsOf(edge, wires[edge.wire], target, walls);
        if (tips.has_value() && (*tips)[0] != (*tips)[1]) {
          slots.push_back({.first = (*tips)[0],
                           .second = (*tips)[1],
                           .saddle = (*tips)[0]});
        }
      }
    }
    return slots;
  }

  /// The gates a line from wall cell `a` to wall cell `b` makes: one for
  /// every stretch of it that runs free, from the wall it leaves to the
  /// wall it meets. A line that crosses a wall is no gate across it: the
  /// chambers on either side of that wall are apart, whatever the line
  /// says (user, 2026-10-14: a cut from a wire's target run to a lead ran
  /// through the run of a terminal edge and let a wire through it). No gate
  /// joins the two walls of one port run: that stretch of the line crosses
  /// the run's slot, which is closed at the port. A stretch whose two ends
  /// `skip` refuses together makes no gate either.
  template <typename Skip>
  void cutAtWalls(const std::size_t a, const std::size_t b, const Walls& walls,
                  const Skip& skip, std::vector<grid::Bottleneck>& out,
                  const unsigned depth = 0) const {
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto lineOf = [&](const std::size_t from, const std::size_t to) {
      return grid::lineCells(static_cast<std::int64_t>(from) % width,
                             static_cast<std::int64_t>(from) / width,
                             static_cast<std::int64_t>(to) % width,
                             static_cast<std::int64_t>(to) / width,
                             scene_.router.width, scene_.router.height);
    };
    const auto cells = lineOf(a, b);
    const auto runOf = [&](const std::size_t cell) {
      const auto wall = walls.of[cell];
      return wall < walls.list.size() && walls.list[wall].kind == WallKind::Stub
                 ? walls.list[wall].run
                 : NO_OWNER;
    };
    // A stretch is a gate by its own line, from its ends: the grid walks
    // that line through other cells than the longer line it was cut from,
    // and where those are walls the stretch is cut again.
    const auto gate = [&](const std::size_t from, const std::size_t to) {
      const auto own = lineOf(from, to);
      const bool clean = own.size() < 3 || std::ranges::none_of(
                                               own.begin() + 1, own.end() - 1,
                                               [&](const std::size_t cell) {
                                                 return walls.mask.test(cell);
                                               });
      // Every step moves the end at least one cell nearer the wall the own
      // line meets first, so the steps are few; the bound is a guard.
      if (clean || depth >= 32) {
        out.push_back({.first = from, .second = to, .saddle = from});
        return;
      }
      cutAtWalls(from, to, walls, skip, out, depth + 1);
    };
    std::size_t lastWall = a;
    bool free = false;
    for (const auto cell : cells) {
      if (!walls.mask.test(cell)) {
        free = true;
        continue;
      }
      if (free &&
          (runOf(lastWall) == NO_OWNER || runOf(lastWall) != runOf(cell)) &&
          !skip(lastWall, cell)) {
        gate(lastWall, cell);
      }
      free = false;
      lastWall = cell;
    }
  }

  /// Drops every gate that meets another in the middle of either
  /// (`grid::uncrossedBottlenecks`, user, 2026-10-14): a slot's gate stays
  /// over a mouth's, a mouth's over a cut, and the shorter of two cuts over
  /// the longer. `mouthsFrom` and `slotsFrom` move to where the mouths and
  /// the slots start among the gates that stay; one past the end stays
  /// past the end.
  /// Gives gates whose ends lie within a cell and a half of each other one
  /// end: the end of the gate that ranks first (slot, then mouth, then cut;
  /// the shorter within a rank). Such gates start at one place, as a fan
  /// from one tip does, and `uncrossedBottlenecks` keeps them both; ends a
  /// cell apart would make two lines at a shallow angle cross a few cells
  /// in, and the sliver between them would let a way pass the narrower one
  /// (21q: the mouth line from f15's coupler and the cut from the run of 38
  /// at the run of f16's coupler). `counts` holds how many of `gates` are
  /// cuts, mouths and slots, in that order.
  void snapSharedEnds(std::vector<grid::Bottleneck>& gates,
                      const std::array<std::size_t, 3>& counts,
                      const Walls& walls) const {
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto xOf = [&](const std::size_t cell) {
      return static_cast<double>(static_cast<std::int64_t>(cell) % width);
    };
    const auto yOf = [&](const std::size_t cell) {
      return static_cast<double>(static_cast<std::int64_t>(cell) / width);
    };
    const auto rankOf = [&](const std::size_t index) {
      return index >= counts[0] + counts[1] ? 2U : index >= counts[0] ? 1U : 0U;
    };
    const auto lengthOf = [&](const grid::Bottleneck& gate) {
      return std::hypot(xOf(gate.second) - xOf(gate.first),
                        yOf(gate.second) - yOf(gate.first));
    };
    std::vector<std::size_t> order(gates.size());
    std::iota(order.begin(), order.end(), std::size_t{0});
    std::ranges::stable_sort(order,
                             [&](const std::size_t i, const std::size_t j) {
                               if (rankOf(i) != rankOf(j)) {
                                 return rankOf(i) > rankOf(j);
                               }
                               return lengthOf(gates[i]) < lengthOf(gates[j]);
                             });
    // Whether the line between two cells crosses no wall between its ends.
    const auto free = [&](const std::size_t from, const std::size_t to) {
      const auto cells =
          grid::lineCells(static_cast<std::int64_t>(xOf(from)),
                          static_cast<std::int64_t>(yOf(from)),
                          static_cast<std::int64_t>(xOf(to)),
                          static_cast<std::int64_t>(yOf(to)),
                          scene_.router.width, scene_.router.height);
      for (std::size_t k = 1; k + 1 < cells.size(); ++k) {
        if (walls.of[cells[k]] != NO_OWNER) {
          return false;
        }
      }
      return true;
    };
    constexpr double SHARED = 1.5;
    std::vector<std::size_t> ends;
    for (const auto index : order) {
      auto& gate = gates[index];
      for (auto* end : {&gate.first, &gate.second}) {
        const auto far = end == &gate.first ? gate.second : gate.first;
        for (const auto other : ends) {
          if (other != *end &&
              std::hypot(xOf(other) - xOf(*end), yOf(other) - yOf(*end)) <
                  SHARED &&
              free(other, far)) {
            *end = other;
            break;
          }
        }
      }
      ends.push_back(gate.first);
      ends.push_back(gate.second);
    }
  }

  void dropCrossingGates(std::vector<grid::Bottleneck>& gates,
                         std::size_t& mouthsFrom, std::size_t& slotsFrom,
                         const Walls& walls) const {
    std::vector<std::uint32_t> rank(gates.size());
    for (std::size_t index = 0; index < gates.size(); ++index) {
      rank[index] = index >= slotsFrom ? 2U : index >= mouthsFrom ? 1U : 0U;
    }
    const auto stays =
        grid::uncrossedBottlenecks(gates, rank, scene_.router.width);
    std::vector<grid::Bottleneck> kept;
    kept.reserve(gates.size());
    std::array<std::size_t, 3> counts{};
    for (std::size_t index = 0; index < gates.size(); ++index) {
      if (stays[index]) {
        kept.push_back(gates[index]);
        ++counts[rank[index]];
      }
    }
    snapSharedEnds(kept, counts, walls);
    if (mouthsFrom <= gates.size()) {
      mouthsFrom = counts[0];
    }
    if (slotsFrom <= gates.size()) {
      slotsFrom = counts[0] + counts[1];
    }
    gates = std::move(kept);
  }

  /// Every gate cut at the walls its line crosses (`cutAtWalls`): a gate
  /// whose line runs free from end to end stays as it is.
  [[nodiscard]] std::vector<grid::Bottleneck>
  cutGatesAtWalls(const std::vector<grid::Bottleneck>& gates,
                  const Walls& walls) const {
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    std::vector<grid::Bottleneck> cut;
    cut.reserve(gates.size());
    const auto keep = [](const std::size_t /*from*/, const std::size_t /*to*/) {
      return false;
    };
    for (const auto& gate : gates) {
      const auto cells =
          grid::lineCells(static_cast<std::int64_t>(gate.first) % width,
                          static_cast<std::int64_t>(gate.first) / width,
                          static_cast<std::int64_t>(gate.second) % width,
                          static_cast<std::int64_t>(gate.second) / width,
                          scene_.router.width, scene_.router.height);
      const bool crosses =
          cells.size() >= 3 &&
          std::ranges::any_of(
              cells.begin() + 1, cells.end() - 1,
              [&](const std::size_t cell) { return walls.mask.test(cell); });
      if (!crosses) {
        cut.push_back(gate);
        continue;
      }
      cutAtWalls(gate.first, gate.second, walls, keep, cut);
    }
    return cut;
  }

  [[nodiscard]] static std::vector<grid::Bottleneck>
  couplerCuts(const std::vector<Wall>& list,
              const std::span<const std::uint32_t> wallOf,
              std::vector<grid::Bottleneck> gates) {
    // A terminal edge's run counts as the edge does (user, 2026-10-14: a
    // gate between the run of f21 at its coupler and the target run of
    // wire 56 was left out, and a wire went through the run).
    const auto counts = [&](const std::size_t cell) {
      const auto& wall = list[wallOf[cell]];
      return wall.kind == WallKind::Feedline ||
             wall.kind == WallKind::Coupler || wall.lead || wall.terminalRun;
    };
    std::erase_if(gates, [&](const grid::Bottleneck& gate) {
      return !counts(gate.first) && !counts(gate.second);
    });
    return gates;
  }

  /// The options `reportBottlenecks` searches the bottlenecks with: gaps up
  /// to `bottleneckWires` wires, one narrowing per wire pitch, the rise of
  /// `bottleneckRise`, and the slots and walls of `wallsAfterInsertion`.
  [[nodiscard]] grid::RoutingBottleneckOptions
  bottleneckOptionsOf(const std::span<const std::size_t> slots,
                      const std::span<const std::uint32_t> wallOf) const {
    const auto pitch = static_cast<double>(roomPitch());
    const auto clearance = static_cast<double>(tuning_.clearance);
    const auto half =
        static_cast<double>(tuning_.bottleneckWires) * clearance / 2.0;
    const auto step =
        std::min(scene_.router.cellWidth, scene_.router.cellHeight);
    return {.maximumSquaredClearance =
                static_cast<std::uint32_t>(std::floor(half * half)) + 1,
            .sameNarrowing = pitch * step,
            .minimumRise = tuning_.bottleneckRise,
            .slots = slots,
            .wallOf = wallOf};
  }

  /// The bottlenecks of the chip the insertion leaves behind (user,
  /// 2026-10-07), report only. The medial axis is built over the obstacles
  /// of `wallsAfterInsertion` — a Voronoi diagram over their boundary
  /// cells, as the capacity stage builds it — and
  /// `grid::findRoutingBottlenecks` finds the cells of the axis where the
  /// clearance has a local minimum or the edge of a plateau of one, and traces
  /// each of them down to the two obstacle cells across it. Every such line is
  /// a bottleneck, and the two obstacles at its ends say what it lies between:
  /// a feedline and a launcher stub, a feedline and a qubit, two feedlines.
  ///
  /// Only gaps up to what `bottleneckWires` wires need are reported, and a
  /// local minimum counts only where the clearance rises by
  /// `bottleneckRise` cells on both sides: the steps of a raster wall are
  /// minima a cell deep. Per bottleneck the length gives how many wires it
  /// holds — `wiresThroughGap`, the length between the inflated walls over
  /// the wire clearance — and the ways the wires have now give which cross
  /// it.
  /// Those ways are the Detail stage's or the insertion's and the feedline
  /// pass moves them, so the count says where to look, not what will fail.
  void reportBottlenecks(const std::vector<Wire>& wires) {
    analyseCapacity(wires);
  }

  /// The analysis of `reportBottlenecks`, whatever `SCPD_BOTTLENECKS`
  /// says: the bottlenecks, the capacity graph and its check, with their
  /// lines, pictures and data.
  void analyseCapacity(const std::vector<Wire>& wires) {
    const auto began = std::chrono::steady_clock::now();
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    // The time of every step, for the line that says what the analysis
    // cost.
    auto lapStart = began;
    std::string laps;
    const auto lap = [&](const std::string_view what) {
      const auto now = std::chrono::steady_clock::now();
      laps +=
          std::format("{}{} {:.3f}", laps.empty() ? "" : ", ", what,
                      std::chrono::duration<double>(now - lapStart).count());
      lapStart = now;
    };
    // The terminal edges are no walls when the check takes them as wires
    // (`terminalDemands`).
    const auto walls = wallsOf(wires, edges_, nullptr, NO_OWNER, false, true);
    lap("walls");
    // The report shows the bottlenecks the check of the chain search finds,
    // by profiles, with no medial axis to draw.
    grid::MedialAxis axis;
    axis.cells.assign(scene_.router.cells(), grid::AxisCell::None);
    const auto found = fastGates(wires, walls);
    const auto clearance = static_cast<double>(tuning_.clearance);
    const auto widest =
        static_cast<double>(tuning_.bottleneckWires) * clearance;
    auto gates =
        cutGatesAtWalls(couplerCuts(walls.list, walls.of, found), walls);
    auto mouthsFrom = gates.size();
    {
      const auto mouths = mouthGates(wires, edges_, walls);
      for (const auto& mouth : mouths.gates) {
        gates.push_back(mouth);
      }
      for (const auto& note : mouths.notes) {
        say("capacity: " + note);
      }
    }
    auto slotsFrom = gates.size();
    for (const auto& slot : slotGates(wires, edges_, walls)) {
      gates.push_back(slot);
    }
    dropCrossingGates(gates, mouthsFrom, slotsFrom, walls);
    lap("profiles");

    std::vector<MeasuredBottleneck> measured;
    measured.reserve(gates.size());
    std::map<std::string, std::uint32_t> pairs;
    std::uint32_t overfull = 0;
    for (std::size_t index = 0; index < gates.size(); ++index) {
      const auto& gate = gates[index];
      MeasuredBottleneck one{.gate = gate,
                             .mouth = index >= mouthsFrom && index < slotsFrom,
                             .slot = index >= slotsFrom};
      const auto x0 = static_cast<std::int64_t>(gate.first) % width;
      const auto y0 = static_cast<std::int64_t>(gate.first) / width;
      const auto x1 = static_cast<std::int64_t>(gate.second) % width;
      const auto y1 = static_cast<std::int64_t>(gate.second) / width;
      one.cells = std::hypot(static_cast<double>(x1 - x0),
                             static_cast<double>(y1 - y0));
      const auto& a = walls.list[walls.of[gate.first]];
      const auto& b = walls.list[walls.of[gate.second]];
      one.holds = wiresThroughGap(one.cells, clearance);

      // The wires on the line between the two ends, the ends' own wires
      // aside. A way may pass between two cells of the line that touch at a
      // corner, so a diagonal step looks at the two cells beside it too.
      std::set<std::uint32_t> owners;
      const auto note = [&](const std::size_t cell) {
        const auto owner = field_.owner(cell);
        if (owner < wires.size() && !wires[owner].feedline && owner != a.wire &&
            owner != b.wire) {
          owners.insert(owner);
        }
      };
      const auto line = grid::lineCells(x0, y0, x1, y1, scene_.router.width,
                                        scene_.router.height);
      for (std::size_t at = 1; at + 1 < line.size(); ++at) {
        note(line[at]);
        const auto px = static_cast<std::int64_t>(line[at - 1]) % width;
        const auto py = static_cast<std::int64_t>(line[at - 1]) / width;
        const auto cx = static_cast<std::int64_t>(line[at]) % width;
        const auto cy = static_cast<std::int64_t>(line[at]) / width;
        if (px != cx && py != cy) {
          note(static_cast<std::size_t>((py * width) + cx));
          note(static_cast<std::size_t>((cy * width) + px));
        }
      }
      one.crossing.assign(owners.begin(), owners.end());
      if (one.crossing.size() > one.holds) {
        ++overfull;
      }
      auto low = kindName(a.kind);
      auto high = kindName(b.kind);
      if (high < low) {
        std::swap(low, high);
      }
      ++pairs[std::format("{}–{}", low, high)];
      measured.push_back(std::move(one));
    }

    // Every bottleneck alike, the tightest first: the fewest wires to
    // spare.
    std::vector<std::size_t> order(measured.size());
    std::iota(order.begin(), order.end(), std::size_t{0});
    const auto spare = [&measured](const std::size_t at) {
      return static_cast<std::int64_t>(measured[at].holds) -
             static_cast<std::int64_t>(measured[at].crossing.size());
    };
    std::ranges::stable_sort(
        order, [&](const std::size_t left, const std::size_t right) {
          return spare(left) != spare(right)
                     ? spare(left) < spare(right)
                     : measured[left].cells < measured[right].cells;
        });
    for (const auto at : order) {
      const auto& one = measured[at];
      tell(std::format(
          "bottleneck: {} · {}: {:.1f} cells from ({},{}) to ({},{}), holds "
          "{}, "
          "crossed now by {} ({})",
          endName(walls, one.gate.first), endName(walls, one.gate.second),
          one.cells, static_cast<std::int64_t>(one.gate.first) % width,
          static_cast<std::int64_t>(one.gate.first) / width,
          static_cast<std::int64_t>(one.gate.second) % width,
          static_cast<std::int64_t>(one.gate.second) / width, one.holds,
          one.crossing.size(), namesOf(wires, one.crossing)));
    }

    std::string paired;
    for (const auto& [pair, count] : pairs) {
      paired +=
          std::format("{}{} {}", paired.empty() ? "" : " · ", pair, count);
    }
    std::array<std::uint32_t, 5> kinds{};
    for (const auto& wall : walls.list) {
      ++kinds[static_cast<std::size_t>(wall.kind)];
    }
    std::size_t axisCells = 0;
    for (const auto cell : axis.cells) {
      axisCells += cell == grid::AxisCell::None ? 0 : 1;
    }
    say(std::format(
        "coupler insertion: BOTTLENECKS — {} in all ({}){}; {} "
        "of them hold fewer wires than cross them now; walls: {} feedline "
        "edges, {} coupler pads, {} stubs and leads, the artwork, the border; "
        "axis {} cells; gaps up to {} wires ({:.0f} cells = {} x {:.0f}), "
        "a gap holding its length over the clearance, port runs walled "
        "and feedlines inflated {:.1f} cells out; rise {:.1f} cells; {:.2f}s "
        "({})",
        measured.size(), paired.empty() ? "-" : paired,
        std::format(", {} more left out with no end on a feedline edge, a "
                    "coupler pad or a resonator lead",
                    found.size() - gates.size()),
        overfull, kinds[static_cast<std::size_t>(WallKind::Feedline)],
        kinds[static_cast<std::size_t>(WallKind::Coupler)],
        kinds[static_cast<std::size_t>(WallKind::Stub)], axisCells,
        tuning_.bottleneckWires, widest, tuning_.bottleneckWires, clearance,
        wallInflation(), tuning_.bottleneckRise,
        std::chrono::duration<double>(std::chrono::steady_clock::now() - began)
            .count(),
        (lap("measure"), laps)));
    reportCapacityGraph(wires, walls, gates, measured);
  }

  /// One edge of the capacity graph of `reportCapacityGraph`: a bottleneck
  /// between the chambers beside it, or a stretch of a feedline edge that a
  /// wire may cross between the chambers on its two sides.
  struct GraphEdge {
    bool crossing = false;
    /// The bottleneck by its place in the measured list, or the feedline
    /// edge by its wire key.
    std::uint32_t id = 0;
    std::vector<std::uint32_t> chambers;
    /// How many wires it takes: the bottleneck's `holds`, or how many
    /// crossings a wire pitch apart the stretch has room for.
    std::uint32_t capacity = 0;
    /// The wires that may use it. Every wire may pass a bottleneck; a
    /// stretch of an edge between couplers only the wires that bridge it.
    std::optional<std::vector<std::uint32_t>> users;
    /// A stretch: its first and last cell on the edge's way, and its length
    /// in cells.
    std::size_t first = 0;
    std::size_t last = 0;
    double length = 0.0;
    /// The wires whose way through the graph takes it.
    std::vector<std::uint32_t> through;
    /// A bottleneck with an end on a terminal feedline edge, or a stretch of
    /// one: it takes every wire (`capacityProblemOf`, `crossingStretches`).
    bool terminal = false;
    /// A stretch: the cells of its crossings, one per wire it takes.
    std::vector<std::size_t> points;
  };

  /// One outer wire of the capacity graph: the cells its two ends leave
  /// from, as `portsOf` gives them, the chambers each lies in, and its
  /// demand in the flow; none when a port lies in no chamber.
  struct GraphWire {
    std::uint32_t wire = 0;
    std::array<PathPoint, 2> ends{};
    std::vector<std::uint32_t> from;
    std::vector<std::uint32_t> to;
    std::optional<std::uint32_t> demand;
    /// The feedline edges it is prescribed to cross, by their wire key.
    std::vector<std::uint32_t> crosses;
    /// A terminal edge taken as a wire (`terminalDemands`), with the chip
    /// port at either end where it has one; `NO_OWNER` at a coupler.
    bool terminal = false;
    std::uint32_t sourcePort = NO_OWNER;
    std::uint32_t targetPort = NO_OWNER;
  };

  /// The stretches of every drawn feedline edge where a wire may cross it,
  /// with the chambers on either side, by the rule the feedline pass holds
  /// a wire to (user, 2026-10-07). The rule is built as
  /// `rebuildCrossingRule` builds it — the same edges, `crossingReach` —
  /// and a cell of an edge is a place to cross when the straight line
  /// across it at a right angle, `crossingReach` + 1 cells to each side,
  /// is one `CrossingConstraints::allowed` lets a wire run and runs into no
  /// obstacle but the edge itself. So a crossing keeps clear of the edge's
  /// bends and of its first and last ten cells, as the search does.
  ///
  /// Consecutive places with the same two chambers are one stretch. No
  /// place lies within `crossingReach` of the
  /// edge's ends, along the edge, as the crossing rule closes them, and a
  /// straight run of `length` cells takes `(length - crossingReach) /
  /// clearance + 1` crossings, none when it is shorter than
  /// `crossingReach` (user, 2026-10-13): the first crossing needs the band
  /// of the crossing rule, every further one a clearance more. Without it a
  /// stretch takes `length / pitch + 1`. A plain wire crosses
  /// the one edge it is prescribed and no other (user, 2026-10-07): an edge
  /// between couplers is open only to the plain wires that bridge it
  /// (`bridgers_`, which `checkBridgers` holds equal to `assignBridges`).
  /// A terminal edge — the edge from or to a launcher — has no stretch: no
  /// wire is prescribed to cross it, so no wire crosses it (user,
  /// 2026-10-13). No resonator crosses an edge
  /// between couplers.
  /// An edge of a chain with a reflector end keeps its stretches where both
  /// sides are one chamber, as stretches from that chamber back into it.
  [[nodiscard]] std::vector<GraphEdge>
  crossingStretches(const std::vector<Wire>& wires,
                    const std::vector<Edge>& edges, const Walls& walls,
                    const grid::Chambers& chambers) const {
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto height = static_cast<std::int64_t>(scene_.router.height);
    std::vector<Path> crossable;
    for (const auto& edge : edges) {
      const auto& wire = wires[edge.wire];
      if (wire.drawn && !edge.terminal) {
        crossable.push_back(wire.way);
      }
    }
    routing::CrossingConstraints rule;
    rule.build(scene_.router.width, scene_.router.height, crossable, {},
               tuning_.crossingReach);
    // The obstacle every drawn edge is, to let a crossing line run over its
    // own edge.
    std::unordered_map<std::uint32_t, std::uint32_t> wallOfEdge;
    for (std::uint32_t wall = 0; wall < walls.list.size(); ++wall) {
      if (walls.list[wall].kind == WallKind::Feedline) {
        wallOfEdge.emplace(walls.list[wall].wire, wall);
      }
    }
    const auto reach = static_cast<std::int64_t>(tuning_.crossingReach) + 1;
    // No wire crosses a feedline edge on a port run, its own included (user,
    // 2026-10-13): a place whose crossing line runs through a slot is none.
    grid::BitGrid onRun(scene_.router.width, scene_.router.height);
    for (const auto cell : walls.runSlots) {
      onRun.set(cell, true);
    }
    // Every cell within the wire clearance of a resonator's lead, the run
    // off its coupler, or of the run a terminal edge keeps at either end
    // as a wire of the graph: no crossing is counted there (user,
    // 2026-10-14). The feedline routing keeps the clearance from the
    // resonators and from the terminal edges' straights, so a wire crosses
    // an edge no nearer to them than that.
    grid::BitGrid nearLead(scene_.router.width, scene_.router.height);
    {
      const auto r = static_cast<std::int64_t>(tuning_.clearance);
      std::vector<std::pair<std::int64_t, std::int64_t>> disc;
      for (std::int64_t dy = -r; dy <= r; ++dy) {
        for (std::int64_t dx = -r; dx <= r; ++dx) {
          if ((dx * dx) + (dy * dy) < r * r) {
            disc.emplace_back(dx, dy);
          }
        }
      }
      for (const auto& [wall, line] : walls.strokes) {
        if (wall >= walls.list.size()) {
          continue;
        }
        const auto& named = walls.list[wall];
        const bool terminalRun = named.kind == WallKind::Stub &&
                                 named.wire < wires.size() &&
                                 wires[named.wire].feedline;
        if (!named.lead && !terminalRun) {
          continue;
        }
        for (const auto& [cx, cy] : line) {
          for (const auto& [dx, dy] : disc) {
            const auto x = cx + dx;
            const auto y = cy + dy;
            if (x >= 0 && y >= 0 && x < width && y < height) {
              nearLead.set(static_cast<std::size_t>((y * width) + x), true);
            }
          }
        }
      }
    }
    // How far past the band a side may still be the edge's own inflation.
    const auto beyond =
        static_cast<std::int64_t>(std::ceil(wallInflation())) + 1;

    std::vector<GraphEdge> stretches;
    for (const auto& edge : edges) {
      const auto& wire = wires[edge.wire];
      const auto own = wallOfEdge.find(edge.wire);
      if (!wire.drawn || wire.way.empty() || own == wallOfEdge.end() ||
          edge.terminal) {
        continue;
      }
      const std::optional<std::vector<std::uint32_t>> users =
          edge.terminal ? std::nullopt
                        : std::optional<std::vector<std::uint32_t>>(
                              bridgers_[edge.chain][edge.from]);
      // A chain with a reflector stays open at that end when it stands
      // whole, so an edge of it may have one chamber on both sides. A wire
      // prescribed to cross it still has to find room on it: such a place
      // is a stretch from the chamber back into it.
      const auto& ends = chains_[edge.chain];
      const bool reflected =
          !edge.open && (!ends.front().fixed || !ends.back().fixed);
      const auto straight = routing::straightCells(wire.way);

      // The two chambers a crossing at a cell joins, or nothing where no
      // wire may cross there.
      // How far along the edge every cell lies, for the distance a crossing
      // keeps from the couplers at its ends.
      std::vector<double> along(wire.way.size(), 0.0);
      for (std::size_t at = 1; at < wire.way.size(); ++at) {
        along[at] =
            along[at - 1] +
            std::hypot(static_cast<double>(wire.way[at].x) - wire.way[at - 1].x,
                       static_cast<double>(wire.way[at].y) -
                           wire.way[at - 1].y);
      }
      const auto total = along.empty() ? 0.0 : along.back();
      const auto clearance = static_cast<double>(tuning_.clearance);
      // The ends are closed as the crossing rule closes them around a
      // bend: the reach of its band.
      const auto endZone = static_cast<double>(tuning_.crossingReach);
      // How many wires cross a straight run of `length` cells side by side:
      // the first needs the band of the crossing rule, every further one a
      // clearance more (user, 2026-10-13).
      const auto crossingsOn = [&](const double length) -> std::uint32_t {
        if (length < endZone) {
          return 0;
        }
        return static_cast<std::uint32_t>(
                   std::floor((length - endZone) / clearance)) +
               1;
      };
      // The crossing line through every straight cell of the edge, the
      // band's reach to either side: whether it is free of other walls and
      // of port runs, and the rule allows a crossing on it.
      std::vector<bool> lineClear(wire.way.size(), false);
      for (std::size_t at = 0; at < wire.way.size(); ++at) {
        if (!straight[at]) {
          continue;
        }
        const auto& point = wire.way[at];
        const auto across = routing::turned(point.heading, 2);
        const auto v = routing::headingVector(across);
        bool clear = true;
        for (std::int64_t t = -reach; t <= reach && clear; ++t) {
          const auto x = static_cast<std::int64_t>(point.x) + (t * v.dx);
          const auto y = static_cast<std::int64_t>(point.y) + (t * v.dy);
          if (x < 0 || y < 0 || x >= width || y >= height) {
            clear = false;
            break;
          }
          const auto cell = static_cast<std::size_t>((y * width) + x);
          clear = !((walls.mask.test(cell) && walls.of[cell] != own->second) ||
                    onRun.test(cell)) &&
                  rule.allowed(static_cast<std::uint32_t>(x),
                               static_cast<std::uint32_t>(y), across);
        }
        lineClear[at] = clear;
      }
      const auto sidesAt = [&](const std::size_t at)
          -> std::optional<std::pair<std::uint32_t, std::uint32_t>> {
        const auto& point = wire.way[at];
        if (!lineClear[at]) {
          return std::nullopt;
        }
        if (along[at] < endZone || total - along[at] < endZone) {
          return std::nullopt;
        }
        // A crossing takes a rectangle, not a line (user, 2026-10-14): the
        // band across the edge and a clearance along it, half to either
        // side, and it may touch no other wall. So the crossing lines of
        // the cells half a clearance along the edge either way have to be
        // clear as well — in steps of the edge, a diagonal step √2 long.
        {
          const auto step = routing::headingVector(point.heading);
          const auto norm = std::hypot(static_cast<double>(step.dx),
                                       static_cast<double>(step.dy));
          const auto half =
              static_cast<std::size_t>(std::ceil(clearance / 2.0 / norm));
          if (at < half || at + half >= wire.way.size()) {
            return std::nullopt;
          }
          for (std::size_t k = at - half; k <= at + half; ++k) {
            if (!lineClear[k]) {
              return std::nullopt;
            }
          }
        }
        const auto across = routing::turned(point.heading, 2);
        const auto v = routing::headingVector(across);
        const auto chamberAt = [&](const std::int64_t side) {
          for (std::int64_t t = reach; t <= reach + beyond; ++t) {
            const auto x =
                static_cast<std::int64_t>(point.x) + (side * t * v.dx);
            const auto y =
                static_cast<std::int64_t>(point.y) + (side * t * v.dy);
            if (x < 0 || y < 0 || x >= width || y >= height) {
              break;
            }
            const auto cell = static_cast<std::size_t>((y * width) + x);
            if (chambers.of[cell] != grid::NO_CHAMBER) {
              return chambers.of[cell];
            }
            if (walls.mask.test(cell) && walls.of[cell] != own->second) {
              break;
            }
          }
          return grid::NO_CHAMBER;
        };
        const auto left = chamberAt(1);
        const auto right = chamberAt(-1);
        if (left == grid::NO_CHAMBER || right == grid::NO_CHAMBER ||
            (left == right && !reflected)) {
          return std::nullopt;
        }
        return std::pair{std::min(left, right), std::max(left, right)};
      };

      std::optional<std::pair<std::uint32_t, std::uint32_t>> open;
      std::size_t first = 0;
      const auto close = [&](const std::size_t last) {
        double length = 0.0;
        for (std::size_t at = first + 1; at <= last; ++at) {
          length += std::hypot(
              static_cast<double>(wire.way[at].x) - wire.way[at - 1].x,
              static_cast<double>(wire.way[at].y) - wire.way[at - 1].y);
        }
        stretches.push_back(
            {.crossing = true,
             .id = edge.wire,
             .chambers = {open->first, open->second},
             .capacity = edge.terminal
                             ? static_cast<std::uint32_t>(wires.size())
                             : crossingsOn(length),
             .users = users,
             .first = first,
             .last = last,
             .length = length,
             .through = {},
             .terminal = edge.terminal});
      };
      const auto mine = stretches.size();
      for (std::size_t at = 0; at < wire.way.size(); ++at) {
        const auto sides = sidesAt(at);
        if (open.has_value() && sides != open) {
          close(at - 1);
          open.reset();
        }
        if (sides.has_value() && !open.has_value()) {
          open = sides;
          first = at;
        }
      }
      if (open.has_value()) {
        close(wire.way.size() - 1);
      }
      // The crossings an edge takes, as points (user, 2026-10-14): from its
      // start to its end, a point on every place to cross that lies a lane
      // spacing or more from every point before it, straight across. A
      // place is a straight cell whose crossing line the rule allows, which
      // keeps the rule's distance from the ends; a stretch takes the points
      // on it.
      if (!edge.terminal && stretches.size() > mine) {
        const auto spacing = clearance * tuning_.crossingSpacing;
        std::vector<std::size_t> points;
        const auto place = [&](const std::size_t at) {
          return std::ranges::any_of(
              std::ranges::subrange(stretches.begin() +
                                        static_cast<std::ptrdiff_t>(mine),
                                    stretches.end()),
              [at](const GraphEdge& piece) {
                return piece.first <= at && at <= piece.last;
              });
        };
        for (std::size_t at = 0; at < wire.way.size(); ++at) {
          if (!place(at)) {
            continue;
          }
          const auto& cell = wire.way[at];
          if (nearLead.test((static_cast<std::size_t>(cell.y) *
                             static_cast<std::size_t>(width)) +
                            cell.x)) {
            continue;
          }
          const bool apart = std::ranges::all_of(points, [&](const auto k) {
            return std::hypot(static_cast<double>(cell.x) - wire.way[k].x,
                              static_cast<double>(cell.y) - wire.way[k].y) >=
                   spacing;
          });
          if (apart) {
            points.push_back(at);
          }
        }
        for (auto piece = mine; piece < stretches.size(); ++piece) {
          auto& stretch = stretches[piece];
          stretch.points.clear();
          for (const auto at : points) {
            if (stretch.first <= at && at <= stretch.last) {
              stretch.points.push_back(
                  (static_cast<std::size_t>(wire.way[at].y) *
                   static_cast<std::size_t>(width)) +
                  wire.way[at].x);
            }
          }
          stretch.capacity = static_cast<std::uint32_t>(stretch.points.size());
        }
        continue;
      }
      // A straight run the chambers beside it cut into several stretches
      // takes as many crossings as fit into the whole run: each stretch
      // what fits into it, and the rest the longest one.
      if (!edge.terminal) {
        for (auto begin = mine; begin < stretches.size();) {
          auto end = begin + 1;
          while (end < stretches.size() &&
                 stretches[end].first == stretches[end - 1].last + 1) {
            ++end;
          }
          const auto run =
              along[stretches[end - 1].last] - along[stretches[begin].first];
          const auto fits = crossingsOn(run);
          std::uint32_t given = 0;
          auto longest = begin;
          for (auto piece = begin; piece < end; ++piece) {
            given += stretches[piece].capacity;
            if (stretches[piece].length > stretches[longest].length) {
              longest = piece;
            }
          }
          if (fits > given) {
            stretches[longest].capacity += fits - given;
          }
          begin = end;
        }
      }
    }
    return stretches;
  }

  /// The cells a wire's terminals lie on, source first — cells of the slots
  /// between the walls of its port runs — each with the way on from there.
  [[nodiscard]] std::array<std::pair<PathPoint, routing::HeadingVector>, 2>
  portsOf(const Walls& walls, const Wire& wire) const {
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto found = walls.terminals.find(wire.key);
    const auto onward = [](const PortRun& run) {
      const auto& end = run.back();
      const auto& before = run[run.size() - 2];
      return routing::HeadingVector{
          .dx =
              static_cast<std::int8_t>((end.first > before.first + 0.5)   ? 1
                                       : (end.first < before.first - 0.5) ? -1
                                                                          : 0),
          .dy = static_cast<std::int8_t>((end.second > before.second + 0.5) ? 1
                                         : (end.second < before.second - 0.5)
                                             ? -1
                                             : 0)};
    };
    const auto pointOf = [width](const std::size_t cell) {
      return PathPoint{.x = static_cast<std::uint32_t>(
                           static_cast<std::int64_t>(cell) % width),
                       .y = static_cast<std::uint32_t>(
                           static_cast<std::int64_t>(cell) / width),
                       .heading = 0,
                       .primitive = 0};
    };
    if (found == walls.terminals.end()) {
      return {std::pair{wire.objective.source, routing::HeadingVector{}},
              std::pair{wire.objective.target, routing::HeadingVector{}}};
    }
    return {std::pair{pointOf(found->second[0]), onward(sourceRunOf(wire))},
            std::pair{pointOf(found->second[1]), onward(targetRunOf(wire))}};
  }

  /// The chambers a terminal lies in. A terminal lies in the slot between
  /// the two walls of its port run, and the slot in one chamber: the room
  /// in front of the run's open end, which the bottlenecks from the ends of
  /// the two walls close off (user, 2026-10-07). A terminal on the line of a
  /// bottleneck lies in every chamber beside it; one on an obstacle is taken on
  /// along its way, and then to the nearest cell around it, a few cells at
  /// most; no chamber when there is none.
  [[nodiscard]] std::vector<std::uint32_t>
  chambersOfPort(const grid::Chambers& chambers, const Walls& walls,
                 const PathPoint& port,
                 const routing::HeadingVector& on) const {
    // Past the inflation of the port run, and a few cells more.
    const auto LOOK = static_cast<std::int64_t>(std::ceil(wallInflation())) + 6;
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto height = static_cast<std::int64_t>(scene_.router.height);
    const auto inside = [&](const std::int64_t x, const std::int64_t y) {
      return x >= 0 && y >= 0 && x < width && y < height;
    };
    const auto cellAt = [width](const std::int64_t x, const std::int64_t y) {
      return static_cast<std::size_t>((y * width) + x);
    };
    // The chambers of a free cell: its own, or those of the cells around it
    // when it lies on a line.
    const auto of = [&](const std::int64_t x, const std::int64_t y) {
      std::vector<std::uint32_t> found;
      if (!inside(x, y) || walls.mask.test(cellAt(x, y))) {
        return found;
      }
      if (const auto own = chambers.of[cellAt(x, y)]; own != grid::NO_CHAMBER) {
        found.push_back(own);
        return found;
      }
      for (std::size_t step = 0; step < grid::STEP_DX.size(); ++step) {
        const auto nx = x + grid::STEP_DX[step];
        const auto ny = y + grid::STEP_DY[step];
        if (inside(nx, ny) && chambers.of[cellAt(nx, ny)] != grid::NO_CHAMBER) {
          found.push_back(chambers.of[cellAt(nx, ny)]);
        }
      }
      std::ranges::sort(found);
      const auto repeated = std::ranges::unique(found);
      found.erase(repeated.begin(), repeated.end());
      return found;
    };
    const auto px = static_cast<std::int64_t>(port.x);
    const auto py = static_cast<std::int64_t>(port.y);
    for (std::int64_t k = 0; k <= LOOK; ++k) {
      if (auto found = of(px + (k * on.dx), py + (k * on.dy)); !found.empty()) {
        return found;
      }
    }
    for (std::int64_t ring = 1; ring <= LOOK; ++ring) {
      for (std::int64_t dy = -ring; dy <= ring; ++dy) {
        for (std::int64_t dx = -ring; dx <= ring; ++dx) {
          if (std::max(std::abs(dx), std::abs(dy)) != ring) {
            continue;
          }
          if (auto found = of(px + dx, py + dy); !found.empty()) {
            return found;
          }
        }
      }
    }
    return {};
  }

  /// The flow problem of `reportCapacityGraph`: the graph's edges, the
  /// demands and what the report and the picture need of both.
  struct CapacityProblem {
    /// The bottleneck edges first, then the stretches.
    std::vector<GraphEdge> graph;
    std::size_t gateEdges = 0;
    /// The bottlenecks with one chamber on both sides.
    std::uint32_t roundabout = 0;
    std::vector<FlowEdge> flowEdges;
    std::vector<FlowDemand> demands;
    /// The wire of every demand.
    std::vector<std::uint32_t> wireOf;
    /// The outer wires with a terminal in no chamber.
    std::vector<std::uint32_t> walled;
    std::uint32_t outer = 0;
    /// The terminal edges that are wires of the graph (`terminalDemands`).
    std::uint32_t terminals = 0;
    std::vector<std::uint32_t> portsIn;
    std::vector<std::pair<PathPoint, std::uint32_t>> portMarks;
    std::vector<GraphWire> outerWires;
  };

  /// The edges and demands of the capacity graph over the chambers the
  /// bottlenecks cut: see `reportCapacityGraph`. With `terminalsAsWires`
  /// the terminal edges are demands, one wire each from the launcher to
  /// the coupler (`terminalDemands`), and `walls` holds none of them.
  [[nodiscard]] CapacityProblem
  capacityProblemOf(const std::vector<Wire>& wires,
                    const std::vector<Edge>& edges, const Walls& walls,
                    const std::vector<MeasuredBottleneck>& measured,
                    const grid::Chambers& chambers,
                    const bool terminalsAsWires = false) const {
    CapacityProblem problem;
    auto& graph = problem.graph;
    // A bottleneck with an end on a terminal feedline edge — the edge from
    // or to a launcher — takes every wire: the feedline routing lays the
    // terminal edges again later, so a gap at one is no limit yet, and the
    // check of the whole chip takes them as wires (`terminalDemands`). Only
    // a check that keeps them as walls, that of a chain on its own, has
    // such bottlenecks; there the terminal edges close the chain's pocket
    // and nothing more.
    std::unordered_set<std::uint32_t> terminalEdges;
    for (const auto& edge : edges) {
      if (edge.terminal) {
        terminalEdges.insert(edge.wire);
      }
    }
    const auto onTerminal = [&](const std::size_t cell) {
      const auto wall = walls.of[cell];
      return wall < walls.list.size() &&
             walls.list[wall].kind == WallKind::Feedline &&
             terminalEdges.contains(walls.list[wall].wire);
    };
    // A bottleneck too narrow for any wire still lets one wire through when
    // an end of it is a flank of that wire's own port run (user,
    // 2026-10-13): the wire leaves its run there, and its own copper keeps
    // no clearance from itself, so the half clearance the run is inflated
    // by is room for it. A wire that leaves its launcher sideways, between
    // the end of its run and a feedline that passes in front of it, takes
    // such a gap. No other wire may use it.
    const auto runOwner = [&](const std::size_t cell) {
      const auto wall = walls.of[cell];
      return (wall < walls.list.size() && walls.list[wall].run != NO_OWNER)
                 ? walls.list[wall].wire
                 : NO_OWNER;
    };
    const auto clearance = static_cast<double>(tuning_.clearance);
    const auto everyWire = static_cast<std::uint32_t>(wires.size());
    for (std::uint32_t gate = 0; gate < measured.size(); ++gate) {
      if (chambers.beside[gate].size() < 2) {
        ++problem.roundabout;
        continue;
      }
      const auto& ends = measured[gate].gate;
      const bool terminal = onTerminal(ends.first) || onTerminal(ends.second);
      auto capacity = terminal ? everyWire : measured[gate].holds;
      std::optional<std::vector<std::uint32_t>> users;
      if (capacity == 0 &&
          wiresThroughGap(measured[gate].cells + wallInflation(), clearance) >
              0) {
        std::vector<std::uint32_t> owners;
        for (const auto cell : {ends.first, ends.second}) {
          if (const auto owner = runOwner(cell);
              owner != NO_OWNER &&
              std::ranges::find(owners, owner) == owners.end()) {
            owners.push_back(owner);
          }
        }
        if (!owners.empty()) {
          capacity = 1;
          users = std::move(owners);
        }
      }
      // The mouth of the slot of a terminal edge's run (`slotGates`): the
      // edge alone passes it, into its slot.
      if (measured[gate].slot) {
        capacity = 0;
        for (const auto cell : {ends.first, ends.second}) {
          if (const auto owner = runOwner(cell); owner != NO_OWNER) {
            capacity = 1;
            users = std::vector<std::uint32_t>{owner};
            break;
          }
        }
      }
      // A gate of a mouth line at a flank of a terminal edge's own run
      // (user, 2026-10-14): however narrow the gap beside the flank, the
      // edge enters its slot along the run's own line, so the gate lets
      // the edge through and no other wire.
      if (capacity == 0 && measured[gate].mouth) {
        std::vector<std::uint32_t> owners;
        for (const auto cell : {ends.first, ends.second}) {
          if (const auto owner = runOwner(cell);
              owner != NO_OWNER && owner < wires.size() &&
              wires[owner].feedline &&
              std::ranges::find(owners, owner) == owners.end()) {
            owners.push_back(owner);
          }
        }
        if (!owners.empty()) {
          capacity = 1;
          users = std::move(owners);
        }
      }
      graph.push_back({.crossing = false,
                       .id = gate,
                       .chambers = chambers.beside[gate],
                       .capacity = capacity,
                       .users = std::move(users),
                       .first = 0,
                       .last = 0,
                       .length = measured[gate].cells,
                       .through = {},
                       .terminal = terminal});
    }
    problem.gateEdges = graph.size();
    // The slot of a terminal edge's run is reached through the mouth of
    // the slot alone (user, 2026-10-14): the chamber a slot gate closes
    // off, the one its run's terminal lies in, is beside no other gate.
    // The mouth lines between the chains end on the flanks' tips as the
    // slot gates do, and a chamber that touches a line at its end counts
    // as beside it; so the slot would be open from the mouth line too.
    if (terminalsAsWires) {
      std::unordered_set<std::uint32_t> slotChambers;
      for (const auto& edge : graph) {
        if (edge.crossing || !measured[edge.id].slot) {
          continue;
        }
        const auto owner = runOwner(measured[edge.id].gate.first);
        const auto found = owner == NO_OWNER ? walls.terminals.end()
                                             : walls.terminals.find(owner);
        if (found == walls.terminals.end()) {
          continue;
        }
        for (const auto cell : found->second) {
          const auto chamber = chambers.of[cell];
          if (chamber != grid::NO_CHAMBER &&
              std::ranges::find(edge.chambers, chamber) !=
                  edge.chambers.end()) {
            slotChambers.insert(chamber);
          }
        }
      }
      for (auto& edge : graph) {
        if (edge.crossing || measured[edge.id].slot) {
          continue;
        }
        std::erase_if(edge.chambers, [&](const std::uint32_t chamber) {
          return slotChambers.contains(chamber);
        });
      }
    }
    for (auto& stretch : crossingStretches(wires, edges, walls, chambers)) {
      graph.push_back(std::move(stretch));
    }

    // The feedline edges each plain wire is prescribed to cross (user,
    // 2026-10-08): every drawn edge between two couplers that it bridges
    // (`bridgers_`). Its way takes exactly one stretch of each and no other;
    // an edge with no stretch leaves it no way. An edge of an open chain is
    // prescribed to no wire (user, 2026-10-10): with its end open, both
    // sides of the edge are one chamber, so the edge has no stretch, and a
    // wire that will cross it once the chain is whole may now pass round
    // the open end. Its stretches, where it has any, stay open to the wires
    // that bridge it.
    std::unordered_set<std::uint32_t> drawnFeedlines;
    for (const auto& wall : walls.list) {
      if (wall.kind == WallKind::Feedline) {
        drawnFeedlines.insert(wall.wire);
      }
    }
    std::unordered_map<std::uint32_t, std::vector<std::uint32_t>> prescribed;
    for (const auto& edge : edges) {
      if (edge.terminal || edge.open || !drawnFeedlines.contains(edge.wire)) {
        continue;
      }
      for (const auto key : bridgers_[edge.chain][edge.from]) {
        prescribed[key].push_back(edge.wire);
      }
    }
    std::unordered_map<std::uint32_t, std::vector<std::uint32_t>> stretchesOf;
    for (std::uint32_t index = 0; index < graph.size(); ++index) {
      if (graph[index].crossing) {
        stretchesOf[graph[index].id].push_back(index);
      }
    }

    // A terminal that a feedline edge comes within the rule of lies in no
    // chamber (user, 2026-10-13): no wire leaves the port without coming
    // within the rule of the edge. The edge's wall is its line inflated by
    // `wallInflation`, so the edge is too near when its wall lies within
    // the clearance less that of the terminal. Without the narrow meeting, an
    // edge of a resonator's own coupler is left out at its source, the end
    // of its lead, where the two meet (`meetAt`).
    const auto near =
        std::max(0.0, static_cast<double>(tuning_.clearance) - wallInflation());
    const auto reachNear = static_cast<std::int64_t>(std::floor(near));
    // The lead end of a resonator against the lines of its own coupler's
    // edges: those meet the lead at the pad, so their walls and the lead's
    // flanks lie between the two, and only the distance to the line itself
    // says that the edge comes within the rule of the lead end.
    const auto nearOwnEdge = [&](const Wire& wire, const PathPoint& at) {
      const auto mine = couplerOfWire_.find(wire.key);
      if (mine == couplerOfWire_.end()) {
        return false;
      }
      const auto limit = static_cast<double>(tuning_.clearance);
      return std::ranges::any_of(edges, [&](const Edge& edge) {
        const auto& points = chains_[edge.chain];
        const bool own = std::ranges::any_of(
            std::array{edge.from, edge.to}, [&](const std::size_t point) {
              return !points[point].fixed &&
                     points[point].coupler == mine->second;
            });
        if (!own || edge.wire >= wires.size() || !wires[edge.wire].drawn) {
          return false;
        }
        return std::ranges::any_of(
            wires[edge.wire].way, [&](const PathPoint& cell) {
              return std::hypot(static_cast<double>(cell.x) - at.x,
                                static_cast<double>(cell.y) - at.y) < limit;
            });
      });
    };
    const auto covered = [&](const Wire& wire, const PathPoint& at,
                             const bool source) {
      if (source && wire.resonator && nearOwnEdge(wire, at)) {
        return true;
      }
      const auto width = static_cast<std::int64_t>(scene_.router.width);
      const auto height = static_cast<std::int64_t>(scene_.router.height);
      for (std::int64_t dy = -reachNear; dy <= reachNear; ++dy) {
        for (std::int64_t dx = -reachNear; dx <= reachNear; ++dx) {
          const auto x = static_cast<std::int64_t>(at.x) + dx;
          const auto y = static_cast<std::int64_t>(at.y) + dy;
          if (x < 0 || y < 0 || x >= width || y >= height ||
              static_cast<double>((dx * dx) + (dy * dy)) >= near * near) {
            continue;
          }
          const auto wall = walls.of[static_cast<std::size_t>((y * width) + x)];
          // A terminal edge kept as a wall covers no port: it is laid again.
          if (wall >= walls.list.size() ||
              walls.list[wall].kind != WallKind::Feedline ||
              terminalEdges.contains(walls.list[wall].wire)) {
            continue;
          }
          return true;
        }
      }
      return false;
    };

    std::unordered_map<std::uint32_t, std::uint32_t> demandOf;
    problem.portsIn.assign(chambers.count, 0);
    const auto pointOf = [](const PathPoint& point) {
      return FlowPoint{.x = static_cast<double>(point.x),
                       .y = static_cast<double>(point.y)};
    };
    // The terminal edges as wires (user, 2026-10-14): one demand each,
    // from the chamber in front of the launcher to the chamber at the
    // coupler's port, or the other way round, crossing nothing and of any
    // length. They come first, before the outer wires: `checkInTurn`
    // routes the demands in order, and a feedline edge between two fixed
    // ports has the least room to give.
    if (terminalsAsWires) {
      for (const auto& edge : edges) {
        if (!edge.terminal || edge.wire >= wires.size() ||
            !wires[edge.wire].feasible) {
          continue;
        }
        const auto& points = chains_[edge.chain];
        // In the slots of its port runs, as every other wire.
        const auto ends = portsOf(walls, wires[edge.wire]);
        const auto& start = ends[0].first;
        const auto& end = ends[1].first;
        auto from = chambersOfPort(chambers, walls, start, ends[0].second);
        auto to = chambersOfPort(chambers, walls, end, ends[1].second);
        ++problem.terminals;
        problem.portMarks.emplace_back(start, edge.wire);
        problem.portMarks.emplace_back(end, edge.wire);
        problem.outerWires.push_back(
            {.wire = edge.wire,
             .ends = {start, end},
             .from = from,
             .to = to,
             .demand = std::nullopt,
             .crosses = {},
             .terminal = true,
             .sourcePort =
                 points[edge.from].fixed ? points[edge.from].port : NO_OWNER,
             .targetPort =
                 points[edge.to].fixed ? points[edge.to].port : NO_OWNER});
        if (from.empty() || to.empty()) {
          problem.walled.push_back(edge.wire);
          continue;
        }
        ++problem.portsIn[from.front()];
        ++problem.portsIn[to.front()];
        problem.outerWires.back().demand =
            static_cast<std::uint32_t>(problem.demands.size());
        demandOf.emplace(edge.wire,
                         static_cast<std::uint32_t>(problem.demands.size()));
        problem.wireOf.push_back(edge.wire);
        problem.demands.push_back({.from = std::move(from),
                                   .to = std::move(to),
                                   .crossings = {},
                                   .longest = std::nullopt,
                                   .start = pointOf(start),
                                   .end = pointOf(end)});
      }
    }

    // One demand per outer wire whose two ports lie in chambers.
    for (const auto& wire : wires) {
      if (!wire.feasible || wire.inner || wire.feedline) {
        continue;
      }
      ++problem.outer;
      const auto ends = portsOf(walls, wire);
      auto from =
          covered(wire, ends[0].first, true)
              ? std::vector<std::uint32_t>{}
              : chambersOfPort(chambers, walls, ends[0].first, ends[0].second);
      auto to =
          covered(wire, ends[1].first, false)
              ? std::vector<std::uint32_t>{}
              : chambersOfPort(chambers, walls, ends[1].first, ends[1].second);
      problem.portMarks.emplace_back(ends[0].first, wire.key);
      problem.portMarks.emplace_back(ends[1].first, wire.key);
      const auto mine = prescribed.find(wire.key);
      problem.outerWires.push_back(
          {.wire = wire.key,
           .ends = {ends[0].first, ends[1].first},
           .from = from,
           .to = to,
           .demand = std::nullopt,
           .crosses = mine != prescribed.end() ? mine->second
                                               : std::vector<std::uint32_t>{}});
      if (from.empty() || to.empty()) {
        problem.walled.push_back(wire.key);
        continue;
      }
      ++problem.portsIn[from.front()];
      ++problem.portsIn[to.front()];
      problem.outerWires.back().demand =
          static_cast<std::uint32_t>(problem.demands.size());
      demandOf.emplace(wire.key,
                       static_cast<std::uint32_t>(problem.demands.size()));
      problem.wireOf.push_back(wire.key);
      std::vector<std::vector<std::uint32_t>> crossings;
      for (const auto feedline : problem.outerWires.back().crosses) {
        const auto found = stretchesOf.find(feedline);
        crossings.push_back(found != stretchesOf.end()
                                ? found->second
                                : std::vector<std::uint32_t>{});
      }
      problem.demands.push_back({.from = std::move(from),
                                 .to = std::move(to),
                                 .crossings = std::move(crossings),
                                 .start = pointOf(ends[0].first),
                                 .end = pointOf(ends[1].first)});
    }
    problem.flowEdges.reserve(graph.size());
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    for (const auto& edge : graph) {
      FlowEdge flow{
          .chambers = edge.chambers, .capacity = edge.capacity, .users = {}};
      // The line a way passes the edge on: the bottleneck's line, or the
      // stretch from its first cell to its last.
      if (edge.crossing) {
        const auto& way = wires[edge.id].way;
        flow.from = {.x = static_cast<double>(way[edge.first].x),
                     .y = static_cast<double>(way[edge.first].y)};
        flow.to = {.x = static_cast<double>(way[edge.last].x),
                   .y = static_cast<double>(way[edge.last].y)};
      } else {
        const auto& gate = measured[edge.id].gate;
        const auto a = static_cast<std::int64_t>(gate.first);
        const auto b = static_cast<std::int64_t>(gate.second);
        flow.from = {.x = static_cast<double>(a % width),
                     .y = static_cast<double>(a / width)};
        flow.to = {.x = static_cast<double>(b % width),
                   .y = static_cast<double>(b / width)};
      }
      if (edge.users.has_value()) {
        flow.users = std::vector<std::uint32_t>{};
        for (const auto key : *edge.users) {
          if (const auto found = demandOf.find(key); found != demandOf.end()) {
            flow.users->push_back(found->second);
          }
        }
      }
      problem.flowEdges.push_back(std::move(flow));
    }
    return problem;
  }

  /// The capacity graph of the chip the insertion leaves behind, and whether
  /// it carries every outer wire at once (user, 2026-10-07), report only.
  ///
  /// The bottlenecks cut the free space into chambers, as they cut it in
  /// the capacity stage: a walk over the cells that never steps across a
  /// bottleneck's line (`grid::chambersOf`). A chamber is a node. A
  /// bottleneck between two or more chambers is an edge every wire may
  /// take, and `holds` wires — `wiresThroughGap` — fit
  /// through; one with the same chamber on both sides has a way round it
  /// and separates nothing. The feedlines are walls, and the stretches of
  /// `crossingStretches` are the edges across them, each open to the wires
  /// the feedline pass lets cross there.
  ///
  /// Every outer wire is a demand from the chamber its source terminal
  /// lies in to the chamber of its target terminal, each in the slot of
  /// its port run, crossing exactly the feedline edges it is prescribed.
  /// `checkInTurn` routes them one after another in wire order, each over
  /// the edges that still take a wire (user, 2026-10-08). An overflow of
  /// zero says the graph carries every wire; an edge with overflow is where
  /// it does not, and the wires sent through it are the ones that compete
  /// there. A wire the graph has no way for at all is said apart. The check
  /// is necessary and not sufficient: the graph knows nothing of the order
  /// of wires inside a chamber, of turns or of lengths.
  void reportCapacityGraph(const std::vector<Wire>& wires, const Walls& walls,
                           const std::vector<grid::Bottleneck>& gates,
                           const std::vector<MeasuredBottleneck>& measured) {
    const auto began = std::chrono::steady_clock::now();
    auto lapStart = began;
    std::string laps;
    const auto lap = [&](const std::string_view what) {
      const auto now = std::chrono::steady_clock::now();
      laps +=
          std::format("{}{} {:.3f}", laps.empty() ? "" : ", ", what,
                      std::chrono::duration<double>(now - lapStart).count());
      lapStart = now;
    };
    const auto chambers = grid::chambersOf(walls.mask, gates, scene_.router);
    lap("chambers");

    auto problem =
        capacityProblemOf(wires, edges_, walls, measured, chambers, true);
    auto& graph = problem.graph;
    const auto roundabout = problem.roundabout;
    const auto gateEdges = problem.gateEdges;
    const auto& demands = problem.demands;
    const auto& wireOf = problem.wireOf;
    const auto& walled = problem.walled;
    const auto outer = problem.outer;
    for (std::size_t index = gateEdges; index < graph.size(); ++index) {
      const auto& stretch = graph[index];
      const auto& way = wires[stretch.id].way;
      tell(std::format(
          "capacity: stretch {} cells {}..{} from ({},{}) to ({},{}), "
          "{:.0f} cells, chambers c{} and c{}, takes {}, open to {}",
          wireId(wires[stretch.id]), stretch.first, stretch.last,
          way[stretch.first].x, way[stretch.first].y, way[stretch.last].x,
          way[stretch.last].y, stretch.length, stretch.chambers[0],
          stretch.chambers[1], stretch.capacity,
          stretch.users.has_value() ? namesOf(wires, *stretch.users)
                                    : std::string("every wire")));
    }
    lap("stretches and ports");
    const auto check = checkInTurn(chambers.count, problem.flowEdges, demands,
                                   tuning_.detourFactor, inTurnRules());
    lap("in turn");

    std::vector<std::uint32_t> noWay;
    for (std::uint32_t demand = 0; demand < demands.size(); ++demand) {
      if (!check.ways[demand].has_value()) {
        noWay.push_back(wireOf[demand]);
        continue;
      }
      for (const auto index : *check.ways[demand]) {
        graph[index].through.push_back(wireOf[demand]);
      }
    }

    const auto nameOf = [&](const GraphEdge& edge) {
      if (!edge.crossing) {
        const auto& gate = gates[edge.id];
        return std::format("g{} {} · {}", edge.id, endName(walls, gate.first),
                           endName(walls, gate.second));
      }
      return std::format("× {} cells {}..{}", wireId(wires[edge.id]),
                         edge.first, edge.last);
    };
    std::vector<std::vector<std::uint32_t>> crossesOf(demands.size());
    for (const auto& one : problem.outerWires) {
      if (one.demand.has_value()) {
        crossesOf[*one.demand] = one.crosses;
      }
    }
    for (std::uint32_t demand = 0; demand < demands.size(); ++demand) {
      const auto& way = check.ways[demand];
      std::string said;
      if (way.has_value()) {
        for (const auto index : *way) {
          const auto& edge = graph[index];
          said += std::format("{}{} ({}/{})", said.empty() ? "" : " → ",
                              nameOf(edge), edge.through.size(), edge.capacity);
        }
      }
      const auto& feedlines = crossesOf[demand];
      tell(std::format(
          "capacity: {}{}: {}", wireId(wires[wireOf[demand]]),
          feedlines.empty()
              ? std::string()
              : std::format(", crossing {}", namesOf(wires, feedlines)),
          !way.has_value()
              ? (demand < check.tooLong.size() && check.tooLong[demand]
                     ? std::format("no way within its length: the shortest "
                                   "runs {:.0f} cells, it may run {:.0f}",
                                   check.lengths[demand],
                                   demands[demand].longest.value_or(0.0))
                     : std::string("no way from its source chamber to its "
                                   "target chamber"))
          : way->empty() ? std::string("source and target in one chamber")
                         : said));
    }

    std::uint32_t crossings = 0;
    std::set<std::uint32_t> crossed;
    std::string overList;
    std::uint32_t overEdges = 0;
    for (std::uint32_t index = 0; index < graph.size(); ++index) {
      const auto& edge = graph[index];
      if (edge.crossing) {
        crossings += edge.capacity;
        crossed.insert(edge.id);
      }
      if (index < check.overflow.size() && check.overflow[index] > 0) {
        ++overEdges;
        const auto one = std::format(
            "{} +{} ({}/{}: {})", nameOf(edge), check.overflow[index],
            edge.through.size(), edge.capacity, namesOf(wires, edge.through));
        tell(std::format("capacity: over: {}", one));
        overList += (overList.empty() ? "" : " · ") + one;
      }
    }
    const auto counted = std::format(
        "coupler insertion: CAPACITY GRAPH — {} chambers, {} bottlenecks "
        "between two or more of them ({} with a way round them), {} stretches "
        "to cross on {} feedline edges ({} crossings in all)",
        chambers.count, gateEdges, roundabout, graph.size() - gateEdges,
        crossed.size(), crossings);
    say(counted);
    // The verdict. SAT: every outer wire has a way and the ways keep every
    // edge within what it takes. UNSAT: a wire has no way at all, a port
    // lies in no chamber, the wires routed one after another overfill an
    // edge.
    const bool lost = !noWay.empty() || !walled.empty();
    const auto verdict = lost ? "UNSAT" : check.shortBy == 0 ? "SAT" : "UNSAT";
    const auto resonators = static_cast<std::uint32_t>(
        std::ranges::count_if(wires, [](const Wire& wire) {
          return wire.feasible && !wire.inner && !wire.feedline &&
                 wire.resonator;
        }));
    std::string why;
    if (!noWay.empty()) {
      // The wires refused only for their length apart from the rest.
      std::vector<std::uint32_t> tooLong;
      std::vector<std::uint32_t> nowhere;
      for (std::uint32_t demand = 0; demand < demands.size(); ++demand) {
        if (check.ways[demand].has_value()) {
          continue;
        }
        (demand < check.tooLong.size() && check.tooLong[demand] ? tooLong
                                                                : nowhere)
            .push_back(wireOf[demand]);
      }
      if (!nowhere.empty()) {
        why += std::format("; no way in the graph for {}",
                           namesOf(wires, nowhere));
      }
      if (!tooLong.empty()) {
        why += std::format("; no way within its length for {}",
                           namesOf(wires, tooLong));
      }
    }
    if (!walled.empty()) {
      why +=
          std::format("; a port in no chamber for {}", namesOf(wires, walled));
    }
    if (check.shortBy > 0) {
      why += std::format(
          "; short by {} wire{} on {} edge{}, routed one after another in "
          "wire order: {}",
          check.shortBy, check.shortBy == 1 ? "" : "s", overEdges,
          overEdges == 1 ? "" : "s", overList);
    }
    // The check by crossing slots gives the verdict, and this line names the
    // verdict of the stretches apart.
    const auto judged = std::format(
        "coupler insertion: CAPACITY GRAPH {}{} — {} outer wires, {} plain "
        "and {} "
        "resonators from their coupler port, each plain wire crossing "
        "exactly the feedline edges it is prescribed{}{}; {:.2f}s ({})",
        "by the stretches, before the slot check: ", verdict, outer,
        outer - resonators, resonators,
        problem.terminals > 0
            ? std::format(", and {} terminal edges as wires from their "
                          "launcher to their coupler",
                          problem.terminals)
            : std::string(),
        !why.empty()
            ? why
            : std::format(
                  "; every one has a way within the capacities, "
                  "routed one after another in wire order{}",
                  check.sweeps > 1
                      ? std::format(" and again in {} sweeps", check.sweeps)
                      : std::string()),
        std::chrono::duration<double>(std::chrono::steady_clock::now() - began)
            .count(),
        (lap("report"), laps));
    say(judged);
    writeCapacityGraph(wires, walls, gates, measured, chambers, graph,
                       problem.outerWires, check, verdict, {counted, judged},
                       !graphOnly_);
  }

  /// Say the verdict of the check by crossing slots as
  /// the verdict of the chip, beside the verdict of the stretches.
  void sayBySlots(const SlotCheckResult& result,
                  const std::string_view previous,
                  const std::chrono::steady_clock::time_point began) {
    const auto listed = [](const std::vector<std::string>& names) {
      std::string list;
      for (const auto& name : names) {
        list += (list.empty() ? "" : ", ") + name;
      }
      return list;
    };
    std::string why;
    if (!result.lost.empty()) {
      why += std::format("; no way for {}", listed(result.lost));
    }
    if (!result.over.empty()) {
      why +=
          std::format("; {} edge{} over: {}", result.over.size(),
                      result.over.size() == 1 ? "" : "s", listed(result.over));
    }
    say(std::format(
        "coupler insertion: CAPACITY GRAPH {} — by the crossing slots of the "
        "feedline edges{}; the stretches said {}; {:.2f}s",
        result.verdict,
        why.empty() ? std::string("; every one has a way within the "
                                  "capacities")
                    : why,
        previous,
        std::chrono::duration<double>(std::chrono::steady_clock::now() - began)
            .count()));
  }

  /// The page of the capacity graph `data`, as `writeCapacityGraph` writes
  /// it, as the check by crossing slots saw it: its slots, loads and ways,
  /// and the verdict of the check before as `previousVerdict`. Only for the
  /// debug output; the verdict itself comes from the graph in memory. When
  /// the check cannot read the graph, `data` stays as it is.
  [[nodiscard]] static nlohmann::json judgedBySlots(nlohmann::json data) {
    const auto previous = data.value("verdict", std::string("UNKNOWN"));
    try {
      auto graph = nlohmann::json::parse(slotCheck(data.dump()).graph);
      graph["previousVerdict"] = previous;
      return graph;
    } catch (const std::invalid_argument&) {
      return data;
    }
  }

  /// One quiet run of the whole capacity analysis and the time of each
  /// step, in the order of `CAPACITY_STEPS`.
  struct CapacityPass {
    Walls walls;
    std::vector<grid::Bottleneck> gates;
    /// Where the mouths between the chains start in `gates`
    /// (`mouthGates`), and where the mouths of the terminal edges' slots
    /// start after them (`slotGates`); past the end when there are none.
    std::size_t mouthsFrom = std::numeric_limits<std::size_t>::max();
    std::size_t slotsFrom = std::numeric_limits<std::size_t>::max();
    /// Why a mouth between two chains could not be closed (`mouthGates`).
    std::vector<std::string> notes;
    grid::Chambers chambers;
    CapacityProblem problem;
    FlowCheck check;
    /// The outer wires with no way: a terminal in no chamber, or no way in
    /// the graph (alone, or at all under the flow).
    std::uint32_t unrouted = 0;
    std::array<double, 9> seconds{};
  };
  static constexpr std::array<std::string_view, 9> CAPACITY_STEPS{
      "walls",   "distance", "voronoi",  "axis",
      "saddles", "measure",  "chambers", "stretches and ports",
      "check"};

  /// The open end of a port run's flank, for `grid::profileCuts`.
  struct FastMouth {
    std::size_t tip = 0;
    double towardX = 0.0;
    double towardY = 0.0;
    double staticClearance = 0.0;
  };

  /// What `fastGates` keeps through an insertion: the nearest cell of the
  /// walls inside the feedline ring (the artwork and the port runs at the
  /// targets) and of those outside it (the border and the port runs at the
  /// sources), and the mouth of every such run's flank, by wire, end and
  /// side.
  struct FastCapacity {
    grid::NearestBlocked inside;
    grid::NearestBlocked outside;
    std::map<std::tuple<std::uint32_t, bool, bool>, FastMouth> mouths;
  };
  /// Built by `fastCapacityOf` at the first check of an insertion.
  mutable std::shared_ptr<const FastCapacity> fastCapacity_;
  /// The time of every step of the passes with `fastGates` since the last
  /// chain said it, and how many passes.
  mutable std::array<double, 9> fastLaps_{};
  mutable std::uint32_t fastPasses_ = 0;
  /// The widest gap a bottleneck spans, in cells: `bottleneckWires` wires.
  [[nodiscard]] double widestGap() const {
    return static_cast<double>(tuning_.bottleneckWires) *
           static_cast<double>(tuning_.clearance);
  }

  /// The mouth of the slot a terminal lies in, at one flank of its run:
  /// the slot cells beside a free cell outside it are the mouth, the flank
  /// cell nearest their middle is the tip, and the way from the middle of
  /// the slot to the middle of the mouth points out.
  [[nodiscard]] std::optional<FastMouth>
  mouthOf(const Walls& walls, const grid::BitGrid& slots,
          const std::uint32_t flank, const std::size_t terminal) const {
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto height = static_cast<std::int64_t>(scene_.router.height);
    const auto inside = [&](const std::int64_t x, const std::int64_t y) {
      return x >= 0 && y >= 0 && x < width && y < height;
    };
    // The slot cell the terminal stands on, or one beside it.
    std::optional<std::size_t> start;
    const auto tx = static_cast<std::int64_t>(terminal) % width;
    const auto ty = static_cast<std::int64_t>(terminal) / width;
    for (std::int64_t ring = 0; ring <= 2 && !start; ++ring) {
      for (std::int64_t dy = -ring; dy <= ring && !start; ++dy) {
        for (std::int64_t dx = -ring; dx <= ring && !start; ++dx) {
          const auto x = tx + dx;
          const auto y = ty + dy;
          if (inside(x, y) &&
              slots.test(static_cast<std::size_t>((y * width) + x))) {
            start = static_cast<std::size_t>((y * width) + x);
          }
        }
      }
    }
    if (!start) {
      return std::nullopt;
    }
    constexpr std::size_t LARGEST_SLOT = 20000;
    std::vector<std::size_t> pending{*start};
    std::set<std::size_t> seen{*start};
    double slotX = 0.0;
    double slotY = 0.0;
    double mouthX = 0.0;
    double mouthY = 0.0;
    std::size_t inSlot = 0;
    std::size_t inMouth = 0;
    while (!pending.empty() && seen.size() < LARGEST_SLOT) {
      const auto cell = pending.back();
      pending.pop_back();
      const auto x = static_cast<std::int64_t>(cell) % width;
      const auto y = static_cast<std::int64_t>(cell) / width;
      slotX += static_cast<double>(x);
      slotY += static_cast<double>(y);
      ++inSlot;
      bool mouth = false;
      for (std::size_t step = 0; step < 4; ++step) {
        const auto nx = x + grid::STEP_DX[step];
        const auto ny = y + grid::STEP_DY[step];
        if (!inside(nx, ny)) {
          continue;
        }
        const auto next = static_cast<std::size_t>((ny * width) + nx);
        if (slots.test(next)) {
          if (seen.insert(next).second) {
            pending.push_back(next);
          }
        } else if (!walls.mask.test(next)) {
          mouth = true;
        }
      }
      if (mouth) {
        mouthX += static_cast<double>(x);
        mouthY += static_cast<double>(y);
        ++inMouth;
      }
    }
    if (inMouth == 0) {
      return std::nullopt;
    }
    slotX /= static_cast<double>(inSlot);
    slotY /= static_cast<double>(inSlot);
    mouthX /= static_cast<double>(inMouth);
    mouthY /= static_cast<double>(inMouth);
    // The tip: the flank's cell nearest the middle of the mouth.
    const auto reach =
        static_cast<std::int64_t>(std::ceil(wallInflation())) + 8;
    const auto mx = std::llround(mouthX);
    const auto my = std::llround(mouthY);
    std::optional<std::size_t> tip;
    double nearest = std::numeric_limits<double>::max();
    for (auto y = my - reach; y <= my + reach; ++y) {
      for (auto x = mx - reach; x <= mx + reach; ++x) {
        if (!inside(x, y) ||
            walls.of[static_cast<std::size_t>((y * width) + x)] != flank) {
          continue;
        }
        const auto d = std::hypot(static_cast<double>(x) - mouthX,
                                  static_cast<double>(y) - mouthY);
        if (d < nearest) {
          nearest = d;
          tip = static_cast<std::size_t>((y * width) + x);
        }
      }
    }
    const auto length = std::hypot(mouthX - slotX, mouthY - slotY);
    if (!tip || length <= 0.0) {
      return std::nullopt;
    }
    return FastMouth{.tip = *tip,
                     .towardX = (mouthX - slotX) / length,
                     .towardY = (mouthY - slotY) / length,
                     .staticClearance = widestGap()};
  }

  /// The walls that stay through an insertion, read once: see
  /// `FastCapacity`. Every resonator's target run is a wall, standing or
  /// not, so that the fields need not change when a coupler is placed.
  [[nodiscard]] const FastCapacity&
  fastCapacityOf(const std::vector<Wire>& wires) const {
    if (fastCapacity_ != nullptr) {
      return *fastCapacity_;
    }
    auto all = wires;
    for (auto& wire : all) {
      if (wire.resonator && !wire.inner && !wire.feedline) {
        wire.feasible = true;
      }
    }
    const std::vector<bool> nonePlaced(couplers_.size(), false);
    const auto walls = wallsOf(all, {}, &nonePlaced, NO_OWNER, true);
    auto fast = std::make_shared<FastCapacity>();
    grid::BitGrid inside(scene_.router.width, scene_.router.height);
    grid::BitGrid outside(scene_.router.width, scene_.router.height);
    // The fields hold the static walls alone: the artwork inside, the
    // border outside. The search walks every other wall as dynamic and
    // takes no field cell of one as a partner, so such a cell would only
    // hide the static wall behind it: a target run at the tip of a lead hid
    // that lead from the coupler run beside it (21q, the lead of Qb20 and
    // the run of f16 at its coupler).
    for (std::size_t cell = 0; cell < walls.of.size(); ++cell) {
      if (walls.of[cell] == NO_OWNER) {
        continue;
      }
      const auto& wall = walls.list[walls.of[cell]];
      if (wall.lead) {
        continue;
      }
      if (wall.kind == WallKind::Artwork) {
        inside.set(cell);
      } else if (wall.kind == WallKind::Border) {
        outside.set(cell);
      }
    }
    fast->inside = grid::nearestBlocked(inside);
    fast->outside = grid::nearestBlocked(outside);
    grid::BitGrid slots(scene_.router.width, scene_.router.height);
    for (const auto cell : walls.slots) {
      slots.set(cell);
    }
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto height = static_cast<std::int64_t>(scene_.router.height);
    const auto widest = widestGap();
    for (std::uint32_t index = 0; index < walls.list.size(); ++index) {
      const auto& wall = walls.list[index];
      const auto ends = walls.terminals.find(wall.wire);
      if (wall.kind != WallKind::Stub || ends == walls.terminals.end()) {
        continue;
      }
      auto mouth =
          mouthOf(walls, slots, index, ends->second[wall.target ? 1 : 0]);
      if (!mouth) {
        continue;
      }
      // How far ahead the nearest static wall of another run lies.
      const auto tx = static_cast<std::int64_t>(mouth->tip) % width;
      const auto ty = static_cast<std::int64_t>(mouth->tip) / width;
      const auto reach = static_cast<std::int64_t>(std::ceil(widest));
      for (auto y = std::max<std::int64_t>(0, ty - reach);
           y <= std::min(height - 1, ty + reach); ++y) {
        for (auto x = std::max<std::int64_t>(0, tx - reach);
             x <= std::min(width - 1, tx + reach); ++x) {
          const auto other =
              walls.of[static_cast<std::size_t>((y * width) + x)];
          if (other == NO_OWNER || walls.list[other].run == wall.run) {
            continue;
          }
          const auto dx = static_cast<double>(x - tx);
          const auto dy = static_cast<double>(y - ty);
          if ((dx * mouth->towardX) + (dy * mouth->towardY) <= 0.0) {
            continue;
          }
          mouth->staticClearance =
              std::min(mouth->staticClearance, std::hypot(dx, dy));
        }
      }
      fast->mouths[{wall.wire, wall.target, index != wall.run}] = *mouth;
    }
    fastCapacity_ = std::move(fast);
    return *fastCapacity_;
  }

  /// The bottlenecks of a state of the insertion by `grid::profileCuts`:
  /// the walls the insertion moves (feedline edges, coupler pads, leads)
  /// are walked along; the others are read off `fastCapacityOf`. A
  /// feedline edge is its line inflated by `wallInflation`. Every port
  /// run's slot is closed at its mouth.
  [[nodiscard]] std::vector<grid::Bottleneck>
  fastGates(const std::vector<Wire>& wires, const Walls& walls) const {
    const auto& fast = fastCapacityOf(wires);
    std::vector<grid::ProfileRole> roles;
    std::vector<std::uint32_t> runs;
    roles.reserve(walls.list.size());
    runs.reserve(walls.list.size());
    for (const auto& wall : walls.list) {
      // The flanks of the port runs are walked as well: from a feedline
      // edge the static fields show only the
      // nearest static wall on either side, so a flank behind another, the
      // second flank of a run beside an island say, had no cut to the edge
      // (user, 2026-10-14).
      const bool moves = wall.kind == WallKind::Feedline ||
                         wall.kind == WallKind::Coupler || wall.lead ||
                         wall.kind == WallKind::Stub;
      roles.push_back(moves ? grid::ProfileRole::Dynamic
                            : grid::ProfileRole::Static);
      runs.push_back(wall.run);
    }
    grid::BitGrid slots(scene_.router.width, scene_.router.height);
    for (const auto cell : walls.slots) {
      slots.set(cell);
    }
    std::vector<grid::ProfileLine> lines;
    for (const auto& [wall, points] : walls.strokes) {
      if (walls.list[wall].kind != WallKind::Feedline) {
        continue;
      }
      grid::ProfileLine line{
          .wall = wall, .points = {}, .radius = wallInflation()};
      line.points.reserve(points.size());
      for (const auto& [x, y] : points) {
        line.points.emplace_back(static_cast<double>(x),
                                 static_cast<double>(y));
      }
      lines.push_back(std::move(line));
    }
    std::vector<grid::ProfileMouth> mouths;
    for (std::uint32_t index = 0; index < walls.list.size(); ++index) {
      const auto& wall = walls.list[index];
      if (wall.kind != WallKind::Stub) {
        continue;
      }
      std::optional<FastMouth> mouth;
      if (wall.lead) {
        const auto ends = walls.terminals.find(wall.wire);
        if (ends != walls.terminals.end()) {
          mouth = mouthOf(walls, slots, index, ends->second[0]);
        }
      } else if (const auto found = fast.mouths.find(
                     {wall.wire, wall.target, index != wall.run});
                 found != fast.mouths.end() &&
                 walls.of[found->second.tip] == index) {
        mouth = found->second;
      }
      if (mouth) {
        mouths.push_back({.flank = index,
                          .tip = mouth->tip,
                          .towardX = mouth->towardX,
                          .towardY = mouth->towardY,
                          .staticClearance = mouth->staticClearance});
      }
    }
    const std::array<const grid::NearestBlocked*, 2> fields{&fast.inside,
                                                            &fast.outside};
    const grid::ProfileScene profileScene{.width = scene_.router.width,
                                          .height = scene_.router.height,
                                          .wallOf = walls.of,
                                          .roles = roles,
                                          .runs = runs,
                                          .fields = fields,
                                          .slots = &slots,
                                          .walls = &walls.drawn,
                                          .lines = lines,
                                          .mouths = mouths};
    const grid::ProfileOptions profileOptions{
        .widest = widestGap(),
        .minimumRise = 2.0 * tuning_.bottleneckRise,
        .sameNarrowing = static_cast<double>(roomPitch()),
        .narrowest = FAST_NARROWEST,
        .clearance = static_cast<double>(tuning_.clearance),
        .dropCrossing = true,
        .keepEdgeWedges = true};
    return grid::profileCuts(profileScene, profileOptions, profileMemo_);
  }
  /// A gap narrower than this many cells is a contact between two walls,
  /// not a bottleneck, for `fastGates`.
  static constexpr double FAST_NARROWEST = 3.0;

  /// The traces, lines and gaps `fastGates` keeps between passes.
  mutable grid::ProfileMemo profileMemo_;

  /// The analysis of `reportBottlenecks` and `reportCapacityGraph` without
  /// a line of log, with the feedline edge of wire `without` left out.
  [[nodiscard]] CapacityPass capacityPass(const std::vector<Wire>& wires,
                                          const std::uint32_t without) const {
    return capacityPass(wires, edges_, nullptr, without);
  }

  /// `capacityPass` for a state of the insertion, as `wallsOf` takes it.
  ///
  /// With `fast` the bottlenecks are `fastGates`: the profiles along the
  /// walls the insertion moves, against fields of the walls it does not,
  /// with no distance transform and no medial axis of the chip. With
  /// `options` (`InsertionState::options`) every pad stands on the option
  /// the state names for its coupler.
  [[nodiscard]] CapacityPass
  capacityPass(const std::vector<Wire>& wires, const std::vector<Edge>& edges,
               const std::vector<bool>* placed, const std::uint32_t without,
               const bool fast = false, const bool terminalsAsWires = false,
               const std::vector<std::size_t>* options = nullptr) const {
    CapacityPass pass;
    auto mark = std::chrono::steady_clock::now();
    std::size_t step = 0;
    const auto lap = [&] {
      const auto now = std::chrono::steady_clock::now();
      pass.seconds.at(step++) =
          std::chrono::duration<double>(now - mark).count();
      mark = now;
    };
    pass.walls = wallsOf(wires, edges, placed, without, false, terminalsAsWires,
                         options);
    lap();
    // No distance transform, Voronoi diagram or axis: their laps stay
    // empty, and the profiles count as the saddles.
    lap();
    lap();
    lap();
    pass.gates = couplerCuts(pass.walls.list, pass.walls.of,
                             fastGates(wires, pass.walls));
    pass.gates = cutGatesAtWalls(pass.gates, pass.walls);
    if (terminalsAsWires) {
      pass.mouthsFrom = pass.gates.size();
      const auto mouths = mouthGates(wires, edges, pass.walls);
      for (const auto& mouth : mouths.gates) {
        pass.gates.push_back(mouth);
      }
      pass.notes = mouths.notes;
      pass.slotsFrom = pass.gates.size();
      for (const auto& slot : slotGates(wires, edges, pass.walls)) {
        pass.gates.push_back(slot);
      }
    }
    dropCrossingGates(pass.gates, pass.mouthsFrom, pass.slotsFrom, pass.walls);
    lap();
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    std::vector<MeasuredBottleneck> measured;
    measured.reserve(pass.gates.size());
    for (std::size_t index = 0; index < pass.gates.size(); ++index) {
      const auto& gate = pass.gates[index];
      MeasuredBottleneck one{.gate = gate,
                             .mouth = index >= pass.mouthsFrom &&
                                      index < pass.slotsFrom,
                             .slot = index >= pass.slotsFrom};
      one.cells = std::hypot(
          static_cast<double>((static_cast<std::int64_t>(gate.second) % width) -
                              (static_cast<std::int64_t>(gate.first) % width)),
          static_cast<double>((static_cast<std::int64_t>(gate.second) / width) -
                              (static_cast<std::int64_t>(gate.first) / width)));
      one.holds =
          wiresThroughGap(one.cells, static_cast<double>(tuning_.clearance));
      measured.push_back(std::move(one));
    }
    lap();
    pass.chambers.of = std::move(spareChambers_);
    grid::chambersOf(pass.walls.mask, pass.gates, scene_.router, pass.chambers);
    lap();
    pass.problem = capacityProblemOf(wires, edges, pass.walls, measured,
                                     pass.chambers, terminalsAsWires);
    lap();
    pass.check =
        checkInTurn(pass.chambers.count, pass.problem.flowEdges,
                    pass.problem.demands, tuning_.detourFactor, inTurnRules());
    lap();
    if (fast) {
      for (std::size_t step = 0; step < fastLaps_.size(); ++step) {
        fastLaps_.at(step) += pass.seconds.at(step);
      }
      ++fastPasses_;
    }
    pass.unrouted = static_cast<std::uint32_t>(
        pass.problem.walled.size() +
        static_cast<std::size_t>(
            std::ranges::count_if(pass.check.ways, [](const auto& way) {
              return !way.has_value();
            })));
    return pass;
  }

  /// R4, report only: the channels between the terminal edges of two
  /// chains. Walking the ring, every maximal run of plain wires between two
  /// resonators whose couplers belong to different chains lies between the
  /// last edge of the one chain and the first edge of the other; the gap
  /// between those two committed ways, both pad runs excluded, is measured
  /// against what the run's wires need.
  void checkChainChannels(const std::vector<Wire>& wires) {
    const auto pitch = roomPitch();
    const auto n = ring_.size();
    std::string named;
    const auto couplerOf =
        [&](const Wire& wire) -> std::optional<std::uint32_t> {
      if (!wire.resonator) {
        return std::nullopt;
      }
      const auto found = couplerOfWire_.find(wire.key);
      if (found == couplerOfWire_.end()) {
        return std::nullopt;
      }
      return found->second;
    };
    std::size_t start = n;
    for (std::size_t p = 0; p < n; ++p) {
      if (couplerOf(wires[ring_[p]]).has_value()) {
        start = p;
        break;
      }
    }
    if (start < n) {
      std::vector<std::uint32_t> between;
      auto last = start;
      for (std::size_t k = 1; k <= n; ++k) {
        const auto p = (start + k) % n;
        const auto& wire = wires[ring_[p]];
        const auto b = couplerOf(wire);
        if (!b.has_value()) {
          if (!wire.resonator && wire.feasible) {
            between.push_back(wire.key);
          }
          continue;
        }
        const auto a = *couplerOf(wires[ring_[last]]);
        const auto [chainA, atA] = chainOfCoupler_[a];
        const auto [chainB, atB] = chainOfCoupler_[*b];
        if (chainA != NO_OWNER && chainB != NO_OWNER && chainA != chainB &&
            !between.empty()) {
          const auto outKey = couplers_[a].edgeOut;
          const auto inKey = couplers_[*b].edgeIn;
          if (outKey != NO_OWNER && inKey != NO_OWNER && wires[outKey].drawn &&
              wires[inKey].drawn) {
            ++room_.crossChainChannels;
            // Reversed, so that the pad run of each lies where
            // `channelBetween` skips: the out-edge's at its start, the
            // in-edge's at its end.
            const Path first(wires[outKey].way.rbegin(),
                             wires[outKey].way.rend());
            const Path second(wires[inKey].way.rbegin(),
                              wires[inKey].way.rend());
            const auto runA = couplers_[a].options[couplers_[a].chosen].run;
            const auto runB = couplers_[*b].options[couplers_[*b].chosen].run;
            const auto channel = routing::channelBetween(
                first, runA + 1, first.size(), second, runB + 1, second.size());
            const auto needs =
                (static_cast<std::uint32_t>(between.size()) + 1) * pitch;
            if (channel.measured && channel.gap < needs) {
              ++room_.crossChainTight;
              if (room_.crossChainTight <= 12) {
                named += std::format(
                    "{}chain {} f{} / chain {} f{}: {} wire{} ({}), gap "
                    "{:.1f} cells at ({},{}) against {}",
                    named.empty() ? ": " : " · ", chainA, wires[outKey].slot,
                    chainB, wires[inKey].slot, between.size(),
                    between.size() == 1 ? "" : "s", namesOf(wires, between),
                    channel.gap, first[channel.at].x, first[channel.at].y,
                    needs);
              }
            }
          }
        }
        between.clear();
        last = p;
      }
    }
    say(std::format(
        "coupler insertion: CHECK chain channels — {} of {} channel{} between "
        "chains tighter than (m+1)·pitch, {} of {} coupler channel{} tighter{}",
        room_.crossChainTight, room_.crossChainChannels,
        room_.crossChainChannels == 1 ? "" : "s", room_.committedTightChannels,
        room_.couplerChannels, room_.couplerChannels == 1 ? "" : "s",
        (room_.crossChainTight == 0 && room_.committedTightChannels == 0)
            ? "; the check is GREEN"
            : named));
  }

  /// The chain being searched on its own, or NO_OWNER. Set around a single
  /// `solveChainAStar` call and cleared again, so nothing the commit does
  /// ever sees it.
  std::uint32_t soloChain_ = NO_OWNER;
  /// Whether `analyseCapacity` writes only the data and draws no picture:
  /// set by `capacityGraphJson`.
  bool graphOnly_ = false;
  /// The chains standing while the capacity check judges every step of a
  /// chain's search (none: each chain on its own chip); null when the check
  /// is off. With `stepRefuses_` a step whose edge closes a wire is refused.
  const std::vector<bool>* stepSettled_ = nullptr;
  /// The wires the slot check of a step routes and judges while a chain is
  /// searched, by name, or nothing for every wire.
  const std::set<std::string>* localWires_ = nullptr;
  bool stepRefuses_ = false;
  /// The room of the resonators outside one chain, and which chain that is.
  /// `foreignRoomStale_` is set wherever a resonator can have moved.
  grid::BitGrid foreignRoom_;
  std::uint32_t foreignRoomChain_ = NO_OWNER;
  bool foreignRoomStale_ = true;
  /// Set around `stateFaults`, and around a segment search of the targeted
  /// repair under `researchSeesChains`: fence every edge of the chain,
  /// the later ones too.
  bool fenceEverything_ = false;
  /// The segment the targeted repair is re-searching — chain and the edges
  /// `[openLo_, openHi_)` — which `fenceCommittedEdges` leaves open, or
  /// NO_OWNER. Set around `researchSegment` and cleared again.
  std::uint32_t openChain_ = NO_OWNER;
  std::size_t openLo_ = 0;
  std::size_t openHi_ = 0;

  /// What the local rip-up-and-reroute of one feedline came to in the
  /// comparison (`ripFeedline`).
  struct FeedlineRip {
    bool clean = true;
    /// The wires it left unrouted, open or crossing, and what each was.
    std::string failing;
    /// Who each open wire meets.
    std::string blocked;
    /// Every round of the pass: its direction, how many wires found a way
    /// at once and how many only by the relaxation, and which found none.
    nlohmann::json rounds = nlohmann::json::array();
    double seconds = 0.0;
  };

  /// What the comparison of the local slot check with the local
  /// rip-up-and-reroute has counted (`compareLocally`), and whether a
  /// segment's search is that comparison rather than the repair.
  struct Agreement {
    bool on = false;
    /// Over every feedline of every run: SAT and clean, SAT and failing,
    /// UNSAT and clean, UNSAT and failing.
    std::array<std::uint32_t, 4> whole{};
    /// Per feedline its wires (`feedlineWindow`) and the free wires beside
    /// it (`freeWiresBeside`), by key and by name, and what the slot check
    /// and the rip-up said of the chip as it began.
    std::vector<std::set<std::uint32_t>> windows;
    std::vector<std::set<std::string>> windowNames;
    std::vector<bool> baseSat;
    std::vector<FeedlineRip> baseRip;
    std::uint32_t unjudged = 0;
    double slotSeconds = 0.0;
    double rerouteSeconds = 0.0;
    /// The runs judged, one record each, for the local SAT benchmark.
    nlohmann::json records = nlohmann::json::array();
    std::uint32_t written = 0;
  };
  Agreement agreement_;

  /// Whether every cell of a way the greedy chose is still open in the
  /// corridor this edge would be searched in now.
  ///
  /// The corridor is rebuilt for the test, which is what makes it honest:
  /// it holds the artwork, the launcher terminals and every feedline that
  /// stands, inflated by the wire clearance. A way that fails this is a way
  /// the search would refuse, and keeping it would put a crossing in the
  /// layout that nothing downstream looks at again.
  [[nodiscard]] bool edgeWayStillOpen(const std::vector<Wire>& wires,
                                      const RoutingObjective& objective,
                                      const Edge& edge, const Path& way) {
    corridorOfEdge(wires, objective, edge);
    return std::ranges::none_of(way, [this](const PathPoint& point) {
      return point.x >= scene_.router.width ||
             point.y >= scene_.router.height ||
             corridor_.test(scene_.router.index(point.x, point.y));
    });
  }

  /// The analytic bound of the step the chain search is pricing
  /// (`pairBound`, at weight 1), or TRELLIS_UNREACHABLE.
  std::uint64_t stepBound_ = routing::TRELLIS_UNREACHABLE;

  /// What one edge search of `routeEdge` returned: the way, whether the
  /// turn bound dropped a move, what the router paid for the way, and the
  /// turn bounds under which the search runs the same
  /// (`routing::DubinsRouter::lastBoundWindow`).
  struct EdgeSearch {
    Path way;
    bool cutOff = false;
    std::uint32_t cost = 0;
    routing::DubinsRouter::BoundWindow window;
  };

  /// The edge searches of `routeEdge`, by the key of
  /// their input (`edgeSearchKeyOf`); cleared for every chain the A* search
  /// solves.
  ///
  /// **Why a stored search answers exactly.** The router reads the
  /// following and nothing else: the two ends with their headings; the
  /// start and end runs and the bend penalty (`setParams`); the turn bound
  /// and the analytic turn switch; the corridor; the static proximity; and
  /// the wire proximity. The heuristic, the bend lower bound and the static
  /// proximity are set once when the driver is built. The wire proximity is
  /// `zeroProximity_`, all zeroes. The free search reads no crossing
  /// constraint, no crossing exemption and no single crossing feedline.
  /// `corridorOfEdge` closes every cell outside `box_`, so the search and
  /// its distance field never enter one, and the static proximity of a
  /// closed cell is never read. The one exception is a slot cell outside
  /// the box (`slotLeftBox_`), and such a search is not stored. The open
  /// list is cleared before every search and pops in the same order for the
  /// same pushes. So two searches with the same ends, runs, bend penalty,
  /// switch, box and corridor cells inside the box run move for move the
  /// same under the same turn bound. A search under another turn bound runs
  /// the same when the bound lies in the stored window
  /// (`routing::DubinsRouter::lastBoundWindow`).
  std::map<std::pair<std::uint64_t, std::uint64_t>, std::vector<EdgeSearch>>
      edgeSearches_;

  /// The key of `edgeSearches_` for the corridor `corridorOfEdge` just
  /// built: two 64-bit hashes over the two ends with their headings, the
  /// runs at them, the bend penalty, the analytic turn switch, the box and
  /// the corridor words that hold the cells of the box, row by row.
  [[nodiscard]] std::pair<std::uint64_t, std::uint64_t>
  edgeSearchKeyOf(const RoutingObjective& objective, const Edge& edge) const {
    const auto [startRun, endRun] = runsOfEdge(edge);
    std::uint64_t a = 0x9E3779B97F4A7C15ULL;
    std::uint64_t b = 0xD1B54A32D192ED03ULL;
    const auto mix = [&a, &b](const std::uint64_t word) {
      a ^= word;
      a ^= a >> 33U;
      a *= 0xFF51AFD7ED558CCDULL;
      a ^= a >> 33U;
      a *= 0xC4CEB9FE1A85EC53ULL;
      a ^= a >> 33U;
      b += word + 0x9E3779B97F4A7C15ULL;
      b ^= b >> 30U;
      b *= 0xBF58476D1CE4E5B9ULL;
      b ^= b >> 27U;
      b *= 0x94D049BB133111EBULL;
      b ^= b >> 31U;
    };
    for (const auto& point : {objective.source, objective.target}) {
      mix((static_cast<std::uint64_t>(point.x) << 32U) | point.y);
      mix(static_cast<std::uint64_t>(point.heading));
    }
    mix((static_cast<std::uint64_t>(startRun) << 32U) | endRun);
    mix((static_cast<std::uint64_t>(edgeBendPenalty()) << 1U) | 1U);
    for (const auto side : {box_.minX, box_.minY, box_.maxX, box_.maxY}) {
      mix(static_cast<std::uint64_t>(side));
    }
    const auto& words = corridor_.words();
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    for (auto y = box_.minY; y <= box_.maxY; ++y) {
      const auto first = static_cast<std::size_t>((y * width) + box_.minX);
      const auto last = static_cast<std::size_t>((y * width) + box_.maxX);
      for (auto word = first >> 6U; word <= (last >> 6U); ++word) {
        mix(words[word]);
      }
    }
    return {a, b};
  }

  /// A stored search of `key` that runs the same under the turn bound
  /// `maxTurns` (`-1` for none), or null.
  [[nodiscard]] const EdgeSearch*
  replayOf(const std::pair<std::uint64_t, std::uint64_t>& key,
           const std::int16_t maxTurns) const {
    const auto found = edgeSearches_.find(key);
    if (found == edgeSearches_.end()) {
      return nullptr;
    }
    for (const auto& one : found->second) {
      // An empty way needs the cut-off to match as well; see
      // `lastBoundWindow`.
      const auto from = one.way.empty() ? one.window.passed : one.window.kept;
      const bool serves =
          maxTurns < 0
              ? one.window.below == std::numeric_limits<std::uint32_t>::max()
              : from <= static_cast<std::uint32_t>(maxTurns) &&
                    static_cast<std::uint32_t>(maxTurns) < one.window.below;
      if (serves) {
        return &one;
      }
    }
    return nullptr;
  }

  /// Route one feedline edge, free of any crossing rule, as the prototype
  /// routes its chain edges: the room of every wire is priced, nothing but
  /// the artwork and the couplers is in the way. `corridorReady` says the
  /// caller has just built the corridor of this edge (`segmentWayOf`).
  [[nodiscard]] Path routeEdge(const std::vector<Wire>& wires,
                               const RoutingObjective& objective,
                               const Edge& edge,
                               const bool corridorReady = false) {
    if (!corridorReady) {
      corridorOfEdge(wires, objective, edge);
    }
    // Nothing another wire holds is priced here. An edge is drawn against
    // the artwork and the couplers alone; every wire it passes is drawn
    // again under the feedline constraints afterwards, and the price on its
    // copper and its room only bought the edge a detour it did not need.
    // The static penalty on the artwork stays, which is what `usePenalty`
    // still carries into the search.
    //
    // The field is all zeroes, and it used to be rewritten — a byte per cell
    // of the whole grid — before every edge search. On 69q that is 29
    // million bytes a search and about 160 GB over an insertion, to say each
    // time what it said before. A field that is always zero is filled once.
    if (zeroProximity_.size() != proximity_.size()) {
      zeroProximity_.assign(proximity_.size(), 0);
    }

    // The approaches of every port used to cost the most a cell can, because
    // a feedline beside a port leaves the wire into it no way to cross at a
    // right angle. That is the crossing rule of the feedline phase, and the
    // insertion does not carry it: the corridor already keeps an edge out of
    // an approach, and nothing here prices an angle.
    const auto& chain = chains_[edge.chain];
    const auto [startRun, endRun] = runsOfEdge(edge);
    router_.setParams({.startStraightLength = startRun,
                       .endStraightLength = endRun,
                       .minRadius = BEND_RADIUS,
                       .bendPenalty = edgeBendPenalty()});
    router_.setSingleCrossingFeedline(nullptr, 0, 1);
    router_.attachCorridor(&corridor_);
    router_.attachWireProximity(&zeroProximity_);
    // The turns this way may make and still let its prefix win, when the
    // chain search said (the step budget).
    const std::int16_t maxTurns =
        stepBudget_ != routing::TRELLIS_UNREACHABLE
            ? static_cast<std::int16_t>(
                  std::min<std::uint64_t>(stepBudget_ / bendUnit(), 255ULL))
            : std::int16_t{-1};
    if (maxTurns >= 0) {
      router_.setMaxTurns(maxTurns);
    }
    router_.setTurnLowerBound(true);
    // The search, or its answer from `edgeSearches_`.
    const auto key = (!debug_ && !slotLeftBox_)
                         ? std::optional(edgeSearchKeyOf(objective, edge))
                         : std::nullopt;
    const EdgeSearch* replay = key ? replayOf(*key, maxTurns)
                                   : static_cast<const EdgeSearch*>(nullptr);
    Path found;
    bool cutOff = false;
    if (replay != nullptr) {
      found = replay->way;
      cutOff = replay->cutOff;
    } else {
      found = router_.route(objective, true);
      cutOff = router_.lastSearchCutOff();
      if (key && router_.lastSearchExpansions() > 0) {
        edgeSearches_[*key].push_back(
            EdgeSearch{.way = found,
                       .cutOff = cutOff,
                       .cost = router_.lastSearchCost(),
                       .window = router_.lastBoundWindow()});
      }
    }
    router_.setTurnLowerBound(false);
    lastCutOff_ =
        stepBudget_ != routing::TRELLIS_UNREACHABLE && found.empty() && cutOff;
    budgetCutoffs_ += lastCutOff_ ? 1 : 0;
    router_.setMaxTurns(-1);
    // One picture per search, as the outer routing draws one: what the
    // corridor left open, what the search paid for and the way it took. The
    // ones that found nothing are the point of it, but a way that came out
    // crooked is only readable beside the ground it was found on.
    if (debug_) {
      Wire pretend;
      pretend.objective = objective;
      pretend.feedline = true;
      pretend.slot = static_cast<std::uint32_t>(edge.from);
      frame_.wires = &wires;
      frame_.pass = "edge";
      frame_.round = edge.chain;
      frame_.forward = true;
      frame_.kind = std::format("c{}-{}to{}", edge.chain, edge.from, edge.to);
      frame_.fence.clear();
      frame_.ripped.clear();
      frame_.before = NO_OWNER;
      frame_.after = NO_OWNER;
      recordSearch(pretend, found,
                   std::format("chain {} {}->{}{}", edge.chain, edge.from,
                               edge.to, found.empty() ? ", no way" : ""));
    }
    if (found.empty() && report_.wants(Detail::Items)) {
      const auto describe = [&](const PathPoint& point, const bool isTarget) {
        const auto moved = router_.sanitize(point, isTarget);
        const bool inGrid =
            moved.x < scene_.router.width && moved.y < scene_.router.height;
        const auto cell = inGrid ? scene_.router.index(moved.x, moved.y) : 0;
        return std::format(
            "({}, {}) heading {} -> ({}, {}){}", point.x, point.y,
            point.heading, moved.x, moved.y,
            !inGrid ? " off the grid"
            : corridor_.test(cell)
                ? " outside the corridor"
                : (scene_.blocked.test(cell) ? " on artwork" : " free"));
      };
      std::string ways;
      for (const auto at : {edge.from, edge.to}) {
        const auto& point = chain[at];
        if (point.fixed) {
          continue;
        }
        const auto& coupler = couplers_[point.coupler];
        const auto& option = coupler.options[coupler.chosen];
        const auto& wire = wires[coupler.wire];
        ways += std::format(
            " | waypoint {} way ({},{})..({},{}) {} cells, outer "
            "({},{})..({},{}) {} cells, target ({},{})",
            at, option.way.front().x, option.way.front().y, option.way.back().x,
            option.way.back().y, option.way.size(), coupler.outerWay.front().x,
            coupler.outerWay.front().y, coupler.outerWay.back().x,
            coupler.outerWay.back().y, coupler.outerWay.size(),
            wire.objective.target.x, wire.objective.target.y);
      }
      tell(std::format(
          "edge of chain {} from {} to {}: no way; source {}, target {}{}",
          edge.chain, edge.from, edge.to, describe(objective.source, false),
          describe(objective.target, true), ways));
    }
    return found;
  }

  [[nodiscard]] std::uint64_t wayCostOf(const Path& way,
                                        const Heading targetHeading) const {
    if (way.empty()) {
      return 0;
    }
    return (bendUnit() * angleCostOf(way, targetHeading)) +
           (tuning_.chainLengthWeight * static_cast<std::uint64_t>(way.size()));
  }

  /// What one eighth turn costs the chain searches: 10000.
  [[nodiscard]] static constexpr std::uint64_t bendUnit() { return 10000ULL; }

  /// The bound of a pair of options in the chain searches: `bendUnit` times
  /// the fewest eighth turns between the two ports (`stubbedTurns` under
  /// `SCPD_CHAIN_BOUND=4`, `boundTurns` else).
  [[nodiscard]] std::uint64_t pairBound(const PathPoint& leaving,
                                        const PathPoint& arriving,
                                        const std::uint32_t firstRun,
                                        const std::uint32_t lastRun) const {
    const auto turns = stubbedTurns(leaving, arriving, firstRun, lastRun);
    return bendUnit() * turns;
  }

  /// How much a way turns, in eighths, its arrival at the target counted:
  /// the prototype's `compute_segment_angle_cost`, which is what its coupler
  /// optimizer minimizes.
  [[nodiscard]] std::uint32_t angleCostOf(const Path& way,
                                          const Heading target) const {
    if (way.empty()) {
      return 0;
    }
    const auto segmented = routing::reconstructSegments(*primitives_, way);
    std::uint32_t cost = 0;
    Heading last = way.front().heading;
    bool first = true;
    for (const auto& segment : segmented.segments) {
      if (!first) {
        cost += routing::headingDistance(last, segment.heading);
      }
      first = false;
      last = segment.heading;
    }
    cost += routing::headingDistance(last, target);
    return cost;
  }

  /// What one option of a coupler costs: its two edges routed, each priced
  /// by how much it turns, plus their lengths and the difference between
  /// them, plus a penalty for a second dogleg. An option whose edge finds
  /// no way, or whose body meets a neighbour's, costs everything.
  [[nodiscard]] std::uint32_t
  localCost(const std::vector<Wire>& wires, const std::uint32_t chain,
            const std::size_t at, const std::size_t option,
            Path* firstOut = nullptr, Path* secondOut = nullptr,
            std::uint32_t* lostOut = nullptr) {

    constexpr auto INFINITE = std::numeric_limits<std::uint32_t>::max();
    auto& points = chains_[chain];
    auto& coupler = couplers_[points[at].coupler];
    const auto kept = coupler.chosen;
    coupler.chosen = option;
    const auto& candidate = coupler.options[option];
    std::uint32_t cost = 0;
    std::uint32_t before = 0;
    std::uint32_t after = 0;
    Path first;
    const auto overlaps = [&](const Waypoint& other) {
      if (other.fixed) {
        return false;
      }
      const auto& body = couplers_[other.coupler]
                             .options[couplers_[other.coupler].chosen]
                             .body;
      std::unordered_set<std::size_t> cells(body.begin(), body.end());
      return std::ranges::any_of(candidate.body, [&](const std::size_t cell) {
        return cells.contains(cell);
      });
    };
    const auto restore = [&]() { coupler.chosen = kept; };
    // How many of the two edges found no way. Kept apart from the cost on
    // purpose: an edge that finds nothing is an empty way, and an empty way
    // turns nowhere — `angleCostOf` scores it zero, the cheapest an edge can
    // be. Weighed on one number, an option that loses an edge beats every
    // option that keeps one, and the chain ships with a coupler nothing
    // reaches. The greedy compares the pair instead.
    std::uint32_t lost = 0;
    const auto report = [&](const std::uint32_t value) {
      if (lostOut != nullptr) {
        *lostOut = lost;
      }
      return value;
    };
    const bool hasBefore = at > 0;
    const bool hasAfter = at + 1 < points.size();
    if ((hasBefore && overlaps(points[at - 1])) ||
        (hasAfter && overlaps(points[at + 1]))) {
      restore();
      lost = 2;
      return report(INFINITE);
    }

    const auto edgeOf = [&](const std::size_t from, const std::size_t to) {
      return Edge{.chain = chain,
                  .from = from,
                  .to = to,
                  .terminal = points[from].fixed || points[to].fixed};
    };
    const auto objectiveOf = [&](const std::size_t from, const std::size_t to) {
      return RoutingObjective{.source = sourceOf(points[from]),
                              .target = targetOf(points[to])};
    };

    // One edge of this coupler, routed on its own.
    //
    // It used to take the sibling edge's freshly found way and fence it —
    // copper two cells wide, the clearance on top, the clearance dropped
    // again when that left no way. None of it ever reached the search:
    // `corridorOfEdge` discarded the cells it was handed. What actually
    // keeps the two edges of one coupler apart is `fenceCommittedEdges`,
    // which closes the sibling's last chosen way like every other edge on
    // the chip. So the price of an edge is a function of the two options at
    // its ends and the frozen state of every other edge — never of the
    // order the two edges of one coupler are routed in.
    const auto routeOne = [&](const std::size_t from, const std::size_t to) {
      return routeEdge(wires, objectiveOf(from, to), edgeOf(from, to));
    };

    // Both edges of this coupler. Neither sees the other, so there is no
    // order to choose: the two searches were tried both ways round for a
    // long time and returned the same two paths every time, because the
    // corridor each is built in does not depend on the other.
    struct Attempt {
      Path first;
      Path second;
      std::uint32_t lost = 0;
      std::uint32_t cost = 0;
    };
    const auto routeBoth = [&]() {
      Attempt out;
      if (hasBefore) {
        out.first = routeOne(at - 1, at);
      }
      if (hasAfter) {
        out.second = routeOne(at, at + 1);
      }
      if (hasBefore) {
        out.lost += out.first.empty() ? 1 : 0;
        out.cost +=
            wayCostOf(out.first, objectiveOf(at - 1, at).target.heading);
      }
      if (hasAfter) {
        out.lost += out.second.empty() ? 1 : 0;
        out.cost +=
            wayCostOf(out.second, objectiveOf(at, at + 1).target.heading);
      }
      return out;
    };

    // Both edges already known for this pair of options: nothing to route.
    // This is where the greedy stops repeating itself — it weighs the same
    // pair once per pass and per neighbour, and the answer cannot have
    // changed while neither coupler moved.
    const auto remembered = [&](const std::size_t from, Path& into) {
      const auto found = edgeMemo_.find(edgeMemoKey(chain, from));
      if (found == edgeMemo_.end()) {
        return false;
      }
      into = found->second;
      return true;
    };
    {
      Attempt known;
      bool all = true;
      if (hasBefore) {
        all = all && remembered(at - 1, known.first);
      }
      if (hasAfter) {
        all = all && remembered(at, known.second);
      }
      if (all) {
        ++memoHits_;
        if (hasBefore) {
          known.lost += known.first.empty() ? 1 : 0;
          known.cost +=
              wayCostOf(known.first, objectiveOf(at - 1, at).target.heading);
        }
        if (hasAfter) {
          known.lost += known.second.empty() ? 1 : 0;
          known.cost +=
              wayCostOf(known.second, objectiveOf(at, at + 1).target.heading);
        }
        first = std::move(known.first);
        lost = known.lost;
        cost = known.cost;
        before = static_cast<std::uint32_t>(first.size());
        after = static_cast<std::uint32_t>(known.second.size());
        if (secondOut != nullptr) {
          *secondOut = known.second;
        }
        if (firstOut != nullptr) {
          *firstOut = first;
        }
        restore();
        return report(cost);
      }
    }
    ++memoMisses_;

    auto taken = routeBoth();
    if (hasBefore) {
      edgeMemo_[edgeMemoKey(chain, at - 1)] = taken.first;
    }
    if (hasAfter) {
      edgeMemo_[edgeMemoKey(chain, at)] = taken.second;
    }
    first = std::move(taken.first);
    lost = taken.lost;
    cost = taken.cost;
    before = static_cast<std::uint32_t>(first.size());
    after = static_cast<std::uint32_t>(taken.second.size());
    if (secondOut != nullptr) {
      *secondOut = taken.second;
    }
    if (firstOut != nullptr) {
      *firstOut = first;
    }
    // cost += before + after;
    // cost += before > after ? before - after : after - before;
    // cost += 100 * candidate.guarded;
    restore();
    return report(cost);
  }

  /// What one edge of a chain costs with a named option at each of its ends,
  /// routed if it has to be. The price the greedy puts on it, for the one
  /// edge: ten thousand an eighth turn, and everything if there is no way.
  ///
  /// It stands the two couplers on the options asked about for as long as it
  /// takes to answer, because everything the question needs — the two ports,
  /// the end stub, the two resonators the corridor closes — is read off
  /// `chosen`. Both are put back before it returns.
  ///
  /// `fence` holds the ways the caller has already laid, which the corridor
  /// closes on top of everything committed. `remember` is false for a caller
  /// whose fence is a whole prefix: a way found against one prefix is worth
  /// nothing under another, and the memo key cannot tell them apart.
  [[nodiscard]] std::uint64_t
  edgeCost(const std::vector<Wire>& wires, const std::uint32_t chain,
           const std::size_t from, const std::size_t optionFrom,
           const std::size_t optionTo, const std::size_t aheadOption = 0,
           const std::vector<const Path*>* fence = nullptr,
           Path* wayOut = nullptr, const bool remember = true) {
    auto& points = chains_[chain];
    const auto stand = [&](const std::size_t at, const std::size_t option) {
      if (points[at].fixed) {
        return std::size_t{0};
      }
      auto& coupler = couplers_[points[at].coupler];
      const auto kept = coupler.chosen;
      coupler.chosen = option;
      return kept;
    };
    const auto keptFrom = stand(from, optionFrom);
    const auto keptTo = stand(from + 1, optionTo);

    const RoutingObjective objective{.source = sourceOf(points[from]),
                                     .target = targetOf(points[from + 1])};
    const Edge edge{.chain = chain,
                    .from = from,
                    .to = from + 1,
                    .terminal = points[from].fixed || points[from + 1].fixed};

    const auto key = edgeMemoKey(chain, from, aheadOption);
    const auto found = remember ? edgeMemo_.find(key) : edgeMemo_.end();
    // The memo key holds the pair of options but not the fence, so a way
    // remembered under one fence is tested against this one before it is
    // taken, and routed again when it runs into it.
    bool stale = false;
    if (found != edgeMemo_.end() && fence != nullptr &&
        !found->second.empty()) {
      prefixFence_ = fence;
      stale = !edgeWayStillOpen(wires, objective, edge, found->second);
      prefixFence_ = nullptr;
      memoStale_ += stale ? 1 : 0;
    }
    Path way;
    if (found != edgeMemo_.end() && !stale) {
      ++memoHits_;
      lastCutOff_ = false;
      way = found->second;
    } else {
      ++memoMisses_;
      prefixFence_ = fence;
      way = routeEdge(wires, objective, edge);
      prefixFence_ = nullptr;
      // An edge with no way under a fence may have one under another fence.
      if (remember && !lastCutOff_ && (fence == nullptr || !way.empty())) {
        edgeMemo_[key] = way;
      }
    }

    stand(from, keptFrom);
    stand(from + 1, keptTo);
    if (wayOut != nullptr) {
      *wayOut = way;
    }
    if (way.empty()) {
      return routing::TRELLIS_UNREACHABLE;
    }
    return wayCostOf(way, objective.target.heading);
  }

  /// What the search made of one chain: the options it would have it stand
  /// on, and the way it found for each of its edges. Nothing is written
  /// while a search runs.
  struct ChainAnswer {
    bool solved = false;
    std::uint64_t cost = routing::TRELLIS_UNREACHABLE;
    std::uint32_t evaluations = 0;
    /// Of those, the ones that were not already remembered — the searches.
    std::uint32_t routed = 0;
    /// How many prefixes the prefix search grew children from. Zero on the
    /// trellis, which has no prefixes.
    std::uint32_t expansions = 0;
    /// Placeholders the search dropped unpriced against its incumbent.
    std::uint32_t pruned = 0;
    /// Whether the search proved this the cheapest run of options there is.
    /// False when the prefix search reordered a pair of edges at an end of
    /// the chain; this is then only the cheapest run it reached.
    bool optimal = false;
    /// Whether a pair of edges at an end of the chain had to be laid the
    /// other way round. That is the reason `optimal` is false.
    bool relaid = false;
    /// The steps the capacity rule refused before a run of options joined
    /// the chain, and whether they reached `chainRuleRefusals`, which ended
    /// the search.
    std::uint32_t refusedBeforeRun = 0;
    bool refusalLimit = false;
    /// What the run costs with every edge at weight 1
    /// (`chainEndEdgeWeight`); TRELLIS_UNREACHABLE where the search does not
    /// say.
    std::uint64_t unweighted = routing::TRELLIS_UNREACHABLE;
    std::uint64_t pairs = 0;
    std::vector<std::size_t> chosen;
    std::vector<Path> ways;
  };

  /// The copper of every feedline that stands, bar one chain's: the cells a
  /// coupler body may not be laid across.
  ///
  /// The chain being solved is left out because its own edges are about to
  /// be routed again — what they hold is the answer of the round before, and
  /// closing options against it would hold this round to the last one's
  /// shape. Built once per solve, not once per layer: a pass over the ways
  /// of the chip against a grid of bits.
  [[nodiscard]] grid::BitGrid
  feedlineCellsBut(const std::uint32_t chain) const {
    grid::BitGrid cells(scene_.router.width, scene_.router.height);
    for (std::uint32_t other = 0; other < chainEdgePaths_.size(); ++other) {
      if (other == chain) {
        continue;
      }
      for (const auto& way : chainEdgePaths_[other]) {
        for (const auto& point : way) {
          if (point.x < scene_.router.width && point.y < scene_.router.height) {
            cells.set(scene_.router.index(point.x, point.y), true);
          }
        }
      }
    }
    return cells;
  }

  /// Which options a waypoint offers a search: a launcher offers the one it
  /// is, a coupler its plain options, and its jogs only once it has shown it
  /// needs them.
  ///
  /// And, where `feedlines` is given, only the options whose body is clear
  /// of every feedline that stands. Where that
  /// leaves nothing the coupler keeps all of them: a layer with no node has
  /// no answer, and the chain would be lost rather than moved.
  [[nodiscard]] std::vector<std::uint32_t>
  openOptionsOf(const std::uint32_t chain, const std::size_t at,
                const grid::BitGrid* feedlines = nullptr) const {
    std::vector<std::uint32_t> open;
    const auto& point = chains_[chain][at];
    if (point.fixed) {
      open.push_back(0);
      return open;
    }
    const auto& coupler = couplers_[point.coupler];
    std::vector<std::uint32_t> all;
    for (std::size_t option = 0; option < coupler.options.size(); ++option) {
      if (coupler.options[option].secondStraight > 0 && !coupler.jogsUnlocked) {
        continue;
      }
      all.push_back(static_cast<std::uint32_t>(option));
      if (feedlines != nullptr &&
          std::ranges::any_of(
              coupler.options[option].body,
              [&](const std::size_t cell) { return feedlines->test(cell); })) {
        continue;
      }
      open.push_back(static_cast<std::uint32_t>(option));
    }
    if (open.empty() && !all.empty()) {
      tell(std::format("coupler insertion:   chain {} waypoint {}: every "
                       "option lies on a feedline; the guard stands down",
                       chain, at));
      return all;
    }
    if (feedlines != nullptr && open.size() != all.size()) {
      tell(std::format("coupler insertion:   chain {} waypoint {}: {} of {} "
                       "options closed, their body on a feedline",
                       chain, at, all.size() - open.size(), all.size()));
    }
    return open;
  }

  /// The port an edge leaves a waypoint by, or arrives at it by, with the
  /// coupler standing there on a named option. A launcher has one port each
  /// way whatever is asked.
  [[nodiscard]] PathPoint portOfOption(const std::uint32_t chain,
                                       const std::size_t at,
                                       const std::size_t option,
                                       const bool leaving) const {
    const auto& point = chains_[chain][at];
    if (point.fixed) {
      return leaving ? point.asSource : point.asTarget;
    }
    const auto& chosen = couplers_[point.coupler].options[option];
    return leaving ? chosen.out : chosen.in;
  }

  /// The cheapest run of coupler options along one chain, exactly, with
  /// every edge routed against the ways the options before it have laid.
  ///
  /// `solveChain` above is far cheaper and rests on an assumption that is
  /// false here: that the price of a step turns on the option at each of its
  /// two ends and on nothing else. Two feedline edges that meet at a coupler
  /// block each other, so the trellis prices a pair that cannot both be
  /// built as though it could, `stateFaults` counts the difference, and the
  /// commit is where an edge is lost. Carrying the predecessor in the node
  /// closes that for the two edges that **share** a coupler — on 17q six
  /// undrawn edges to none — and it does not reach 57q and 69q, where what
  /// is lost is blocked by edges that are not neighbours. No width of node
  /// reaches those; only pricing a step against the whole prefix does.
  ///
  /// So a node here is a **prefix**, and `routing::solveChainAStar` walks
  /// the prefixes best-first. Its heuristic is the cheapest run of analytic
  /// bounds from a layer to the end of the chain — the same relaxation the
  /// trellis fills its price array from, run backwards. It never exceeds
  /// what a real edge costs under any prefix, so the answer is the true
  /// optimum against a fence that is real.
  ///
  /// **Little is remembered.** A way is worth only what the prefix it was
  /// routed against makes it worth, and with no merging no prefix comes
  /// round twice. The inner edges of a chain use `edgeMemo_` all the same,
  /// for time, and every remembered way is tested against the prefix fence
  /// before it is priced; a way that stays open can still be dearer than
  /// the best way under this prefix, so the answer is the optimum of the
  /// prices the search saw, not a proof.
  /// What one step of one prefix of the chain search laid.
  struct Laid {
    /// The way of the edge into the prefix's last layer.
    Path way;
    /// A way for the edge **before** it, when the two would only both fit
    /// with this one laid first — see the reorder in `solveChainAStar`'s
    /// `step`. Empty otherwise. It hangs off this key and not the shorter one
    /// because the shorter prefix is shared by every choice that extends it,
    /// and only this choice needed its predecessor moved.
    Path before;
    /// The prices of `way` and of `before` at weight 1, whatever
    /// `chainEndEdgeWeight` says.
    std::uint64_t price = 0;
    std::uint64_t beforePrice = 0;
  };

  /// What the chain line calls a chain's answer: `optimum` when the search
  /// proved it, `best found, an end pair laid the other way round` when a
  /// relaid pair cost the proof, and `best found` else.
  [[nodiscard]] static std::string answerWord(const ChainAnswer& answer) {
    if (answer.optimal) {
      return "optimum";
    }
    return answer.relaid ? "best found, an end pair laid the other way round"
                         : "best found";
  }

  /// `, <cost> at weight 1` for a chain's answer, when the end edges weigh
  /// otherwise (`chainEndEdgeWeight`) and the search says what the run costs
  /// at weight 1.
  [[nodiscard]] std::string atWeightOne(const ChainAnswer& answer) const {
    return tuning_.chainEndEdgeWeight == 1.0 ||
                   answer.unweighted == routing::TRELLIS_UNREACHABLE
               ? std::string{}
               : std::format(", {} at weight 1", answer.unweighted);
  }

  /// What a complete run of choices costs at weight 1: the prices of the
  /// ways `waysOfRun` reads, each at weight 1.
  [[nodiscard]] static std::uint64_t
  unweightedOfRun(const std::map<std::vector<std::uint32_t>, Laid>& laid,
                  const std::span<const std::uint32_t> run) {
    const std::vector<std::uint32_t> whole(run.begin(), run.end());
    const auto upto = [&](const std::size_t take) {
      return std::vector<std::uint32_t>(
          whole.begin(), whole.begin() + static_cast<std::ptrdiff_t>(take));
    };
    std::uint64_t total = 0;
    for (std::size_t at = 0; at + 1 < whole.size(); ++at) {
      if (at + 3 <= whole.size()) {
        const auto moved = laid.find(upto(at + 3));
        if (moved != laid.end() && !moved->second.before.empty()) {
          total += moved->second.beforePrice;
          continue;
        }
      }
      const auto found = laid.find(upto(at + 2));
      if (found != laid.end()) {
        total += found->second.price;
      }
    }
    return total;
  }

  /// The ways of a complete run of choices, read out of what the search laid
  /// under the prefix each edge was priced under — which is the run itself.
  /// Where a pair at an end of the chain was reordered, the earlier edge's
  /// way is the replacement the later one left behind. One way per edge,
  /// empty where nothing was laid.
  [[nodiscard]] static std::vector<Path>
  waysOfRun(const std::map<std::vector<std::uint32_t>, Laid>& laid,
            const std::span<const std::uint32_t> run) {
    std::vector<Path> ways(run.empty() ? 0 : run.size() - 1);
    const std::vector<std::uint32_t> whole(run.begin(), run.end());
    const auto upto = [&](const std::size_t take) {
      return std::vector<std::uint32_t>(
          whole.begin(), whole.begin() + static_cast<std::ptrdiff_t>(take));
    };
    for (std::size_t at = 0; at + 1 < whole.size(); ++at) {
      if (at + 3 <= whole.size()) {
        const auto moved = laid.find(upto(at + 3));
        if (moved != laid.end() && !moved->second.before.empty()) {
          ways[at] = moved->second.before;
          continue;
        }
      }
      const auto found = laid.find(upto(at + 2));
      if (found != laid.end()) {
        ways[at] = found->second.way;
      }
    }
    return ways;
  }

  [[nodiscard]] ChainAnswer solveChainAStar(const std::vector<Wire>& wires,
                                            const std::uint32_t chain) {
    ChainAnswer out;
    const auto routedBefore = memoMisses_;
    edgeSearches_.clear();
    auto& points = chains_[chain];
    const auto layers = points.size();
    if (layers < 2) {
      return out;
    }

    std::vector<std::vector<std::uint32_t>> open;
    routing::ChainSolution answer;
    // What every step the search has priced laid (`Laid`), by the prefix it
    // was priced under: the key is the run of node indices from layer 0.
    // This is what `step` reads a prefix's fence out of, and it is the one
    // thing a node of the search needs that the search itself knows nothing
    // about.
    //
    // `std::map` for two reasons: the key is a vector, and its values never
    // move, so the fence may hold pointers into it.
    std::map<std::vector<std::uint32_t>, Laid> laid;

    // The capacity check of every step (`capacityStep`): the verdict on the
    // chip each priced step leaves, under the same key as `laid`, with the
    // wires it closes against the prefix before it; and the verdict on the
    // chip with none of this chain standing, which a first edge is compared
    // with. The feedline edges are named as the commit names them.
    struct StepCheck {
      CapacityVerdict verdict;
      std::vector<std::uint32_t> closed;
    };
    std::map<std::vector<std::uint32_t>, StepCheck> checked;
    CapacityVerdict unchained;
    std::uint32_t stepChecks = 0;
    std::uint32_t stepsCarrying = 0;
    std::uint32_t stepsClosing = 0;
    std::uint32_t stepsRefused = 0;
    // The steps the capacity rule refused while no run of options had joined
    // the chain, over both goes, for `chainRuleRefusals`; and whether a run
    // has joined.
    std::uint32_t refusedBeforeRun = 0;
    bool joined = false;
    // The checks left out because no verdict could change the answer.
    std::uint32_t stepsUnread = 0;
    double stepSeconds = 0.0;
    std::uint32_t firstSlot = 0;
    for (std::uint32_t before = 0; before < chain; ++before) {
      firstSlot += chains_[before].empty()
                       ? 0U
                       : static_cast<std::uint32_t>(chains_[before].size() - 1);
    }
    if (stepSettled_ != nullptr) {
      unchained = capacityVerdictOf(stateOf(wires, *stepSettled_));
    }

    // Two goes at most: the plain options, then the jogs opened at the wall
    // if the plain ones did not reach.
    const auto feedlines = feedlineCellsBut(chain);
    for (int go = 0; go < 2; ++go) {
      open.clear();
      for (std::size_t at = 0; at < layers; ++at) {
        open.push_back(openOptionsOf(chain, at, &feedlines));
      }
      laid.clear();
      checked.clear();

      // The bounds, built before the search as the trellis builds its price
      // array. The analytic answer depends on the pair of options and
      // nothing else, so the whole graph costs microseconds against the tens
      // of milliseconds one real edge search costs.
      const auto beganBound = std::chrono::steady_clock::now();
      std::vector<std::vector<std::uint64_t>> turns(layers - 1);
      for (std::size_t at = 0; at + 1 < layers; ++at) {
        const auto n = open[at].size();
        const auto m = open[at + 1].size();
        turns[at].resize(n * m);
        for (std::size_t c = 0; c < n; ++c) {
          const auto leaving = portOfOption(chain, at, open[at][c], true);
          for (std::size_t d = 0; d < m; ++d) {
            const auto arriving =
                portOfOption(chain, at + 1, open[at + 1][d], false);
            turns[at][(c * m) + d] = pairBound(
                leaving, arriving, forcedRunOf(chain, at, open[at][c]),
                forcedRunOf(chain, at + 1, open[at + 1][d]));
          }
        }
        boundPairs_ += n * m;
      }
      boundNanos_ += static_cast<std::uint64_t>(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::steady_clock::now() - beganBound)
              .count());

      routing::ChainProblem problem;
      for (const auto& choices : open) {
        problem.width.push_back(static_cast<std::uint32_t>(choices.size()));
      }
      problem.bound = [&](const std::size_t at, const std::uint32_t i,
                          const std::uint32_t j) {
        auto bound =
            turns[at][(static_cast<std::size_t>(i) * open[at + 1].size()) + j];
        if (const auto weight = edgeWeight(chain, at); weight != 1.0) {
          return weighted(bound, weight);
        }
        return bound;
      };
      // Every time a run of options comes back with all of its feedline
      // edges drawn: say when one arrives and what it costs. A chain that
      // never prints this line is one the search never joined.
      problem.found = [&](const std::span<const std::uint32_t> run,
                          const std::uint64_t cost) {
        std::string options;
        for (std::size_t at = 0; at < run.size(); ++at) {
          options +=
              std::format("{}{}", at == 0 ? "" : " ",
                          points[at].fixed ? std::string("launcher")
                                           : std::to_string(open[at][run[at]]));
        }
        tell(std::format("coupler insertion:   chain {}: every feedline edge "
                         "drawn on options {} -> cost {}",
                         chain, options, cost));
      };
      // How far the search has come, every `CHAIN_PROGRESS_STEPS` priced
      // steps.
      const auto goBegan = std::chrono::steady_clock::now();
      problem.progressEvery = CHAIN_PROGRESS_STEPS;
      problem.progress = [&](const routing::ChainSolution& sofar,
                             const std::size_t queued) {
        say(std::format(
            "chain progress: chain {} go {}{}: {} steps priced, {} edge "
            "searches, {} prefixes expanded, {} left unpriced, {} open, "
            "deepest layer {} of {}, {}; {:.1f}s",
            chain, go, stepRefuses_ ? " under the capacity rule" : "",
            sofar.routed, memoMisses_ - routedBefore, sofar.expansions,
            sofar.pruned, queued, sofar.reached, layers - 1,
            sofar.solved ? std::format("cheapest run so far {}", sofar.cost)
                         : std::string("no run yet"),
            std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                          goBegan)
                .count()));
      };
      // The ways of the edges of a prefix laid so far, by the prefix's key:
      // what `step` reads as `wayOfEdge`.
      const auto wayOfEdgeIn = [&](const std::vector<std::uint32_t>& key,
                                   const std::size_t e) -> const Path* {
        const auto upto = [&](const std::size_t take) {
          return std::vector<std::uint32_t>(
              key.begin(), key.begin() + static_cast<std::ptrdiff_t>(take));
        };
        if (e + 3 <= key.size()) {
          const auto moved = laid.find(upto(e + 3));
          if (moved != laid.end() && !moved->second.before.empty()) {
            return &moved->second.before;
          }
        }
        const auto found = laid.find(upto(e + 2));
        return (found != laid.end() && !found->second.way.empty())
                   ? &found->second.way
                   : nullptr;
      };
      // The capacity check of one step: the chip the chains settled before
      // this one leave, with the prefix `key` and the option `j` after it
      // standing on top — its couplers up to this edge's far end and its
      // edges up to this one, `way` the edge's own and `before` the one
      // before it where the pair was laid the other way round. Under
      // `capacityRule` a step whose edge closes a wire is refused: whether
      // it is.
      const auto checkStep = [&](const std::vector<std::uint32_t>& key,
                                 const std::uint32_t j, const Path& way,
                                 const Path& before) {
        const auto at = key.size() - 1;
        const auto edges = layers - 1;
        const auto began = std::chrono::steady_clock::now();
        std::vector<std::size_t> options(layers, 0);
        std::string named;
        for (std::size_t layer = 0; layer <= at + 1; ++layer) {
          options[layer] = open[layer][layer <= at ? key[layer] : j];
          named +=
              std::format("{}{}", layer == 0 ? "" : " ",
                          points[layer].fixed ? std::string("launcher")
                                              : std::to_string(options[layer]));
        }
        std::vector<const Path*> ways(edges, nullptr);
        for (std::size_t e = 0; e < at; ++e) {
          ways[e] = wayOfEdgeIn(key, e);
        }
        if (!before.empty()) {
          ways[at - 1] = &before;
        }
        ways[at] = &way;
        const auto state =
            stateOf(wires, *stepSettled_, chain, &options, &ways, at + 2);
        auto verdict = capacityVerdictOf(state);
        const auto parent = checked.find(key);
        const auto& was = (at == 0 || parent == checked.end())
                              ? unchained
                              : parent->second.verdict;
        std::vector<std::uint32_t> closed;
        std::ranges::set_difference(verdict.lost, was.lost,
                                    std::back_inserter(closed));
        const auto spent = std::chrono::steady_clock::now() - began;
        const auto seconds = std::chrono::duration<double>(spent).count();
        ++stepChecks;
        stepsCarrying += verdict.lost.empty() ? 1U : 0U;
        stepsClosing += closed.empty() ? 0U : 1U;
        stepSeconds += seconds;
        const bool refused = stepRefuses_ && !closed.empty();
        stepsRefused += refused ? 1U : 0U;
        // A step that closes a wire names the feedline edges its edge
        // meets.
        std::string meets;
        if (!closed.empty()) {
          const auto met = feedlinesMet(state, chain, at);
          if (!met.empty()) {
            meets =
                std::format("; its edge meets {}", namesOf(state.wires, met));
          }
        }
        const std::vector<std::uint32_t> lost(verdict.lost.begin(),
                                              verdict.lost.end());
        tell(std::format(
            "capacity step: chain {} f{} ({}->{}) on options {}: {}{}; "
            "{}{}{}; {:.3f}s",
            chain, firstSlot + at, at, at + 1, named,
            lost.empty() ? std::string("SAT") : std::string("UNSAT"),
            lost.empty()
                ? std::string()
                : std::format(" — {} wire{} without a way ({}), short by {}",
                              lost.size(), lost.size() == 1 ? "" : "s",
                              namesOf(wires, lost), verdict.shortBy),
            closed.empty()
                ? std::string("this edge closes no wire")
                : std::format("this edge closes {}", namesOf(wires, closed)),
            meets, refused ? " — refused by the capacity rule" : "", seconds));
        if (wants("step-check")) {
          const auto ids = [&wires](const std::vector<std::uint32_t>& keys) {
            nlohmann::json named = nlohmann::json::array();
            for (const auto key : keys) {
              named.push_back(wireId(wires[key]));
            }
            return named;
          };
          record("step-check", {{"chain", chain},
                                {"step", at},
                                {"edge", firstSlot + at},
                                {"options", options},
                                {"verdict", lost.empty() ? "SAT" : "UNSAT"},
                                {"lost", ids(lost)},
                                {"closed", ids(closed)},
                                {"refused", refused},
                                {"seconds", seconds}});
        }
        if (refused) {
          return true;
        }
        auto mine = key;
        mine.push_back(j);
        checked[std::move(mine)] = StepCheck{.verdict = std::move(verdict),
                                             .closed = std::move(closed)};
        return false;
      };
      problem.step = [&](const std::span<const std::uint32_t> prefix,
                         const std::uint32_t j, std::int64_t& redone) {
        redone = 0;
        const auto at = prefix.size() - 1;
        const auto edges = layers - 1;
        std::vector<std::uint32_t> key(prefix.begin(), prefix.end());
        const auto upto = [&](const std::size_t take) {
          return std::vector<std::uint32_t>(
              key.begin(), key.begin() + static_cast<std::ptrdiff_t>(take));
        };
        // The way edge `e` of this prefix stands on. Every edge of a prefix
        // has one: the search prices a prefix's own step before it ever
        // grows a child from it. A pair that was reordered left the
        // replacement for its earlier edge on the later one's key, so that
        // is what counts where it exists.
        const auto wayOfEdge = [&](const std::size_t e) -> const Path* {
          if (e + 3 <= key.size()) {
            const auto moved = laid.find(upto(e + 3));
            if (moved != laid.end() && !moved->second.before.empty()) {
              return &moved->second.before;
            }
          }
          const auto found = laid.find(upto(e + 2));
          return (found != laid.end() && !found->second.way.empty())
                     ? &found->second.way
                     : nullptr;
        };
        std::vector<const Path*> fence;
        fence.reserve(at);
        for (std::size_t e = 0; e < at; ++e) {
          // The chain's first edge is an obstacle to every edge of the chain.
          // The last edge cannot appear here at all: this fence holds the
          // edges *before* the one being priced.
          if (const Path* laidWay = wayOfEdge(e); laidWay != nullptr) {
            fence.push_back(laidWay);
          }
        }

        // Whether this edge is routed again for every prefix, or remembered
        // by the pair of options at its two ends. The two edges at each end
        // of a chain are always routed against the prefix that stands: an
        // edge at a launcher has one way off the terminal and the edges
        // beside it crowd the same room (`CHAIN_FRESH_EDGES`, user
        // 2026-09-28). An edge in between is remembered, but `edgeCost`
        // tests a remembered way against the prefix fence and routes it again
        // when it runs into it, so no step is priced on a way the prefix
        // blocks. A way that stays open may still be dearer than the best way
        // under this prefix; that is the price of the memo. Without it, 45q
        // ran chain 3 out of its time.
        const bool fresh =
            (at < CHAIN_FRESH_EDGES && points.front().fixed) ||
            (at + CHAIN_FRESH_EDGES >= edges && points.back().fixed);
        //
        // A step that may lay its end pair the other way round (below) gets
        // no step budget: a budget cut-off is no proof that the edge has no
        // way, and taking it for one reordered pairs that had a way and
        // marked the answer as not optimal.
        const bool pairAtAnEnd =
            (at == 1 && atLauncher(chain, 0)) ||
            (at + 1 == edges && at >= 1 && atLauncher(chain, at));
        const bool mayReorder = pairAtAnEnd && !fence.empty();
        Path way;
        // An end edge is priced at its weight, so its turn budget is the
        // step budget unweighted (`weightedBudget`).
        const auto weight = edgeWeight(chain, at);
        stepBudget_ = mayReorder ? routing::TRELLIS_UNREACHABLE
                                 : weightedBudget(problem.stepBudget, weight);
        stepBound_ =
            turns[at]
                 [(static_cast<std::size_t>(prefix[at]) * open[at + 1].size()) +
                  j];
        auto real =
            edgeCost(wires, chain, at, open[at][prefix[at]], open[at + 1][j], 0,
                     fence.empty() ? nullptr : &fence, &way, !fresh);
        stepBudget_ = routing::TRELLIS_UNREACHABLE;
        stepBound_ = routing::TRELLIS_UNREACHABLE;

        // **The other order, for the pair at either end of the chain.**
        //
        // The prefix lays its edges front to back, so the edge before this
        // one took the room first and this one has to work around it. At the
        // ends of a chain that order decides whether both fit at all: an
        // edge at a launcher leaves the terminal on one heading and cannot
        // yield, and the edge beside it wants the same ground. Laid the
        // other way round — this one first, then its predecessor again
        // against it — the pair can come home where it could not (user,
        // 2026-09-28).
        //
        // The predecessor's new way hangs off *this* key, because the prefix
        // it belongs to is shared by every choice that extends it and only
        // this choice needed it moved. And the price of the prefix changes
        // with it, which is what `redone` carries back.
        Path before;
        std::uint64_t beforePrice = 0;
        if (real == routing::TRELLIS_UNREACHABLE && mayReorder) {
          const Path* stood = fence.back();
          const auto wasTurning = wayCostOf(
              *stood,
              portOfOption(chain, at, open[at][prefix[at]], false).heading);
          std::vector<const Path*> without(fence.begin(), fence.end() - 1);
          Path mine;
          const auto first =
              edgeCost(wires, chain, at, open[at][prefix[at]], open[at + 1][j],
                       0, without.empty() ? nullptr : &without, &mine, false);
          if (first != routing::TRELLIS_UNREACHABLE) {
            auto against = without;
            against.push_back(&mine);
            Path again;
            const auto nowTurning =
                edgeCost(wires, chain, at - 1, open[at - 1][prefix[at - 1]],
                         open[at][prefix[at]], 0, &against, &again, false);
            if (nowTurning != routing::TRELLIS_UNREACHABLE) {
              real = first;
              way = std::move(mine);
              before = std::move(again);
              beforePrice = nowTurning;
              // The predecessor at its own weight, laid either way.
              redone = static_cast<std::int64_t>(
                           weighted(nowTurning, edgeWeight(chain, at - 1))) -
                       static_cast<std::int64_t>(
                           weighted(wasTurning, edgeWeight(chain, at - 1)));
              tell(std::format(
                  "coupler insertion:   chain {} edge {}: no way after edge "
                  "{}, but both fit with edge {} laid first — edge {} redone "
                  "at {} instead of {}",
                  chain, at, at - 1, at, at - 1, nowTurning, wasTurning));
            }
          }
        }

        if (real == routing::TRELLIS_UNREACHABLE) {
          // The prefix blocks itself here, in either order. Nothing that
          // extends it is a chain, and the search takes another combination
          // — which is the whole of what the trellis cannot do.
          return real;
        }
        // The capacity check of this step (`checkStep`).
        //
        // Not when no verdict can change the answer. The search set
        // `stepBudget` to `U - 1 - floor`, where
        // `floor` is the price of the prefix plus the run of bounds after
        // this step, so the step's `f` is `floor` plus its weighted price,
        // and `f` is above `U` exactly when that price is above the budget
        // plus one.
        const bool unread = [&] {
          if (stepSettled_ == nullptr ||
              problem.stepBudget == routing::TRELLIS_UNREACHABLE ||
              redone != 0) {
            return false;
          }
          const auto priced = weighted(real, weight);
          const auto atU = problem.stepBudget + 1;
          if (at + 1 < edges) {
            return priced >= atU;
          }
          return priced > atU ||
                 (priced == atU && problem.stepNode > problem.winnerNode);
        }();
        if (unread) {
          ++stepsUnread;
          problem.stepTested = false;
        } else if (stepSettled_ != nullptr && checkStep(key, j, way, before)) {
          // Refused by the capacity rule. Before a run joins the chain, the
          // refusals count toward `chainRuleRefusals`.
          if (!joined && ++refusedBeforeRun == tuning_.chainRuleRefusals) {
            problem.stop = true;
          }
          return routing::TRELLIS_UNREACHABLE;
        }
        joined = joined || at + 1 == edges;
        key.push_back(j);
        laid[std::move(key)] = Laid{.way = std::move(way),
                                    .before = std::move(before),
                                    .price = real,
                                    .beforePrice = beforePrice};
        return weighted(real, weight);
      };

      answer = routing::solveChainAStar(problem);
      if (answer.solved || answer.stopped) {
        break;
      }
      // The wall: the deepest layer a prefix whose every step is real ever
      // got to. The step out of it is what shut the chain, so those are the
      // two couplers whose choice is widened — never the whole chain, which
      // trebles what every round has to route and was measured worse.
      bool widened = false;
      const auto unlock = [&](const std::size_t at) {
        if (at >= layers || points[at].fixed) {
          return;
        }
        auto& coupler = couplers_[points[at].coupler];
        if (coupler.jogsUnlocked) {
          return;
        }
        coupler.jogsUnlocked = true;
        widened = true;
        tell(std::format(
            "coupler insertion:   '{}' chain {}: no prefix reaches past it "
            "on plain options — opening the second-dogleg options",
            wireId(wires[coupler.wire]), chain));
      };
      unlock(answer.reached);
      unlock(answer.reached + 1);
      if (!widened) {
        break;
      }
    }

    // On this path a step is priced exactly when it is routed, so the two
    // figures the trellis keeps apart are one and the same.
    out.evaluations = answer.routed;
    out.routed = static_cast<std::uint32_t>(memoMisses_ - routedBefore);
    out.expansions = answer.expansions;
    out.pruned = answer.pruned;
    out.refusedBeforeRun = refusedBeforeRun;
    out.refusalLimit = answer.stopped;
    if (stepSettled_ != nullptr) {
      const std::vector<std::uint32_t> lost(unchained.lost.begin(),
                                            unchained.lost.end());
      say(std::format(
          "capacity step: chain {}: {} steps checked in {:.1f}s ({:.3f}s "
          "each), {} of them SAT, {} closed a wire, {} refused for it; "
          "without the chain {} wire{} without a way ({}){}",
          chain, stepChecks, stepSeconds,
          stepChecks == 0 ? 0.0 : stepSeconds / stepChecks, stepsCarrying,
          stepsClosing, stepsRefused, lost.size(), lost.size() == 1 ? "" : "s",
          namesOf(wires, lost),
          std::format("; {} checks left out, their verdict unread",
                      stepsUnread)));
      if (fastPasses_ > 0) {
        std::string laps;
        for (std::size_t step = 0; step < fastLaps_.size(); ++step) {
          if (step == 1 || step == 2 || step == 3) {
            continue;
          }
          laps += std::format("{}{} {:.4f}", laps.empty() ? "" : ", ",
                              CAPACITY_STEPS.at(step),
                              fastLaps_.at(step) / fastPasses_);
        }
        say(std::format("capacity check: chain {}: {} passes, seconds per "
                        "pass: {}",
                        chain, fastPasses_, laps));
        fastLaps_ = {};
        fastPasses_ = 0;
      }
      // Every feedline edge of the run the chain settles on, with what the
      // check said when the search laid it.
      for (std::size_t e = 0; answer.solved && e + 1 < layers; ++e) {
        const std::vector<std::uint32_t> run(
            answer.chosen.begin(),
            answer.chosen.begin() + static_cast<std::ptrdiff_t>(e + 2));
        const auto found = checked.find(run);
        if (found == checked.end()) {
          say(std::format("capacity step: chain {} f{}: not checked", chain,
                          firstSlot + e));
          continue;
        }
        const auto& one = found->second;
        const std::vector<std::uint32_t> edgeLost(one.verdict.lost.begin(),
                                                  one.verdict.lost.end());
        say(std::format(
            "capacity step: chain {} f{}: {}{}", chain, firstSlot + e,
            edgeLost.empty()
                ? std::string("SAT")
                : std::format("UNSAT — {} wire{} without a way ({}), short "
                              "by {}",
                              edgeLost.size(), edgeLost.size() == 1 ? "" : "s",
                              namesOf(wires, edgeLost), one.verdict.shortBy),
            one.closed.empty()
                ? std::string(", closes no wire")
                : std::format(", closes {}", namesOf(wires, one.closed))));
      }
    }
    for (std::size_t at = 0; at + 1 < layers; ++at) {
      out.pairs +=
          static_cast<std::uint64_t>(open[at].size()) * open[at + 1].size();
    }
    if (!answer.solved) {
      return out;
    }
    out.solved = true;
    out.optimal = answer.optimal;
    out.relaid = answer.relaid;
    out.cost = answer.cost;
    out.chosen.assign(layers, 0);
    for (std::size_t at = 0; at < layers; ++at) {
      out.chosen[at] = points[at].fixed ? 0 : open[at][answer.chosen[at]];
    }
    // The winner's ways, read out of `laid` under the prefix each was routed
    // against — which is the run of choices the winner itself is. Where a
    // pair at an end of the chain was reordered, the earlier edge's way is
    // the replacement the later one left behind.
    out.ways = waysOfRun(laid, answer.chosen);
    out.unweighted = unweightedOfRun(laid, answer.chosen);
    return out;
  }

  /// Stand a whole chain on a run of options and hand back the run it was
  /// standing on. Used to read a chain's ways out of the memo without
  /// leaving anything moved.
  [[nodiscard]] std::vector<std::size_t>
  standOn(const std::uint32_t chain, const std::vector<std::size_t>& options) {
    auto& points = chains_[chain];
    std::vector<std::size_t> kept(points.size(), 0);
    for (std::size_t at = 0; at < points.size(); ++at) {
      if (points[at].fixed) {
        continue;
      }
      auto& coupler = couplers_[points[at].coupler];
      kept[at] = coupler.chosen;
      coupler.chosen = options[at];
    }
    return kept;
  }

  /// A rectangle that may hang off the grid on either side, which
  /// `routing::CellBox` cannot: both of the things compared below are the
  /// grown span of something near an edge of the chip.
  struct Span {
    std::int64_t minX = 0;
    std::int64_t minY = 0;
    std::int64_t maxX = 0;
    std::int64_t maxY = 0;

    [[nodiscard]] bool meets(const Span& other) const {
      return other.minX <= maxX && other.maxX >= minX && other.minY <= maxY &&
             other.maxY >= minY;
    }
  };

  /// The box an edge is searched in, for a named option at each of its ends
  /// — the same rectangle `corridorOfEdge` closes everything outside of.
  [[nodiscard]] Span edgeBox(const std::uint32_t chain, const std::size_t from,
                             const std::size_t optionFrom,
                             const std::size_t optionTo) const {
    const auto source = portOfOption(chain, from, optionFrom, true);
    const auto target = portOfOption(chain, from + 1, optionTo, false);
    const std::int64_t margin = EDGE_BOX_MARGIN;
    return {.minX = std::min<std::int64_t>(source.x, target.x) - margin,
            .minY = std::min<std::int64_t>(source.y, target.y) - margin,
            .maxX = std::max<std::int64_t>(source.x, target.x) + margin,
            .maxY = std::max<std::int64_t>(source.y, target.y) + margin};
  }

  /// Forget every remembered edge whose corridor one of these ways can have
  /// changed, and keep the rest.
  ///
  /// A round's answer only moves an edge's price where it moves a way that
  /// edge could see. Everything outside an edge's box is closed to it
  /// whatever happens, so a way that changed matters to it only if what the
  /// way closes — itself, spread by the clearance — reaches into that box.
  /// The test is on bounding boxes, so it forgets more than it has to and
  /// never less.
  ///
  /// Throwing the whole memo away at the head of every round is the safe
  /// thing and it is what the rounds cost: on 17q three of four rounds
  /// re-priced chains that had not moved and could not move.
  void forgetEdgesNear(const std::vector<Span>& moved) {
    if (moved.empty()) {
      return;
    }
    std::erase_if(edgeMemo_, [&](const auto& entry) {
      const auto key = entry.first;
      const auto chain = static_cast<std::uint32_t>((key >> 56U) & 0xFFU);
      const auto from = static_cast<std::size_t>((key >> 48U) & 0xFFU);
      const auto encodedFrom = static_cast<std::size_t>((key >> 16U) & 0xFFFFU);
      const auto encodedTo = static_cast<std::size_t>(key & 0xFFFFU);
      if (chain >= chains_.size() || from + 1 >= chains_[chain].size()) {
        return true;
      }
      const auto box =
          edgeBox(chain, from, encodedFrom == 0 ? 0 : encodedFrom - 1,
                  encodedTo == 0 ? 0 : encodedTo - 1);
      return std::ranges::any_of(
          moved, [&](const Span& what) { return what.meets(box); });
    });
  }

  /// Where a way that changed reaches, the clearance it keeps included.
  [[nodiscard]] Span reachOf(const Path& way) const {
    const std::int64_t spread = tuning_.clearance;
    Span box{.minX = std::numeric_limits<std::int64_t>::max(),
             .minY = std::numeric_limits<std::int64_t>::max(),
             .maxX = std::numeric_limits<std::int64_t>::min(),
             .maxY = std::numeric_limits<std::int64_t>::min()};
    for (const auto& point : way) {
      box.minX = std::min<std::int64_t>(box.minX, point.x);
      box.minY = std::min<std::int64_t>(box.minY, point.y);
      box.maxX = std::max<std::int64_t>(box.maxX, point.x);
      box.maxY = std::max<std::int64_t>(box.maxY, point.y);
    }
    box.minX -= spread;
    box.minY -= spread;
    box.maxX += spread;
    box.maxY += spread;
    return box;
  }

  /// How many edges of the chip would not survive the state it is standing
  /// on: their way is empty, or it no longer lies in the corridor that holds
  /// once every other edge of that same state is fenced.
  ///
  /// This is the commit's own test — `edgeWayStillOpen` — asked of a whole
  /// state at once, and it is what a round has to be judged by. A round
  /// whose answer scores zero here is one the commit will keep entire; a
  /// round that scores above zero will have edges routed again from
  /// scratch, and those are the ones that come back with no way at all.
  ///
  /// It builds a corridor per edge and searches for nothing, so it costs a
  /// few dozen corridor builds and not one A*.
  [[nodiscard]] std::uint32_t stateFaults(const std::vector<Wire>& wires) {
    std::uint32_t faults = 0;
    for (std::uint32_t chain = 0; chain < chains_.size(); ++chain) {
      faults += chainFaults(wires, chain);
    }
    return faults;
  }

  /// `chainFaults` of a chain standing on a run of options and ways for a
  /// moment; the options and ways it stood on before are put back.
  [[nodiscard]] std::uint32_t runFaults(const std::vector<Wire>& wires,
                                        const std::uint32_t chain,
                                        const std::vector<std::size_t>& options,
                                        const std::vector<const Path*>& ways) {
    const auto& points = chains_[chain];
    std::vector<std::size_t> kept(points.size(), 0);
    for (std::size_t at = 0; at < points.size(); ++at) {
      if (!points[at].fixed) {
        auto& coupler = couplers_[points[at].coupler];
        kept[at] = coupler.chosen;
        coupler.chosen = options[at];
      }
    }
    auto keptWays = chainEdgePaths_[chain];
    for (std::size_t at = 0; at < ways.size(); ++at) {
      chainEdgePaths_[chain][at] = ways[at] != nullptr ? *ways[at] : Path{};
    }
    const auto faults = chainFaults(wires, chain);
    chainEdgePaths_[chain] = std::move(keptWays);
    for (std::size_t at = 0; at < points.size(); ++at) {
      if (!points[at].fixed) {
        couplers_[points[at].coupler].chosen = kept[at];
      }
    }
    return faults;
  }

  /// The edges of one chain that would not survive the state, as
  /// `stateFaults` counts them.
  [[nodiscard]] std::uint32_t chainFaults(const std::vector<Wire>& wires,
                                          const std::uint32_t chain) {
    // The one place that fences every chain against every other whatever
    // the searches do. This is the commit's test asked of a whole state,
    // and the commit lays all the chains on one chip — so with the fence
    // open it would answer zero by construction and say nothing.
    const bool fencedBefore = fenceEverything_;
    fenceEverything_ = true;
    std::uint32_t faults = 0;
    const auto& points = chains_[chain];
    for (std::size_t at = 0; at + 1 < points.size(); ++at) {
      const auto& way = chainEdgePaths_[chain][at];
      if (way.empty()) {
        ++faults;
        continue;
      }
      const RoutingObjective objective{.source = sourceOf(points[at]),
                                       .target = targetOf(points[at + 1])};
      const Edge edge{.chain = chain,
                      .from = at,
                      .to = at + 1,
                      .terminal = points[at].fixed || points[at + 1].fixed};
      if (!way.front().samePlace(objective.source) ||
          !way.back().samePlace(objective.target) ||
          !edgeWayStillOpen(wires, objective, edge, way)) {
        ++faults;
      }
    }
    fenceEverything_ = fencedBefore;
    return faults;
  }

  /// The options of a chain state, each after a space.
  [[nodiscard]] static std::string
  optionsText(const std::vector<std::size_t>& state) {
    std::string text;
    for (const auto option : state) {
      text += std::format(" {}", option);
    }
    return text;
  }

  /// What the slot check of the whole chip finds against it, as the
  /// insertion's check judges it (`insertionCheckJson`): every chain
  /// standing, the terminal edges as wires. The wires it loses and the edges
  /// over their capacity; both empty when the chip is SAT. `touching`, when
  /// given, gets the feedline edges (`f<k>`) that an edge over its capacity
  /// lies on or ends at; `overGates`, when given, the middle of the line of
  /// every bottleneck over its capacity.
  [[nodiscard]] std::vector<std::string>
  wholeChipLost(const std::vector<Wire>& wires,
                std::set<std::string>* touching = nullptr,
                std::vector<std::array<double, 2>>* overGates = nullptr) {
    auto first = static_cast<std::uint32_t>(wires.size());
    for (const auto& edge : edges_) {
      first = std::min(first, edge.wire);
    }
    const std::vector<Wire> before(
        wires.begin(), wires.begin() + static_cast<std::ptrdiff_t>(first));
    auto state = stateOf(before, std::vector<bool>(chains_.size(), true));
    state.terminalsAsWires = true;
    localWires_ = nullptr;
    std::set<std::uint32_t> lost;
    std::optional<SlotCheckResult> typed;
    SlotGraph input;
    capacityCheckOf(state, lost, &typed, &input);
    if (typed.has_value()) {
      if (touching != nullptr) {
        const std::regex feedline(R"(\bf\d+\b)");
        const auto take = [&](const std::string& text) {
          for (auto it =
                   std::sregex_iterator(text.begin(), text.end(), feedline);
               it != std::sregex_iterator(); ++it) {
            touching->insert(it->str());
          }
        };
        // An edge over by its name; a self-bottleneck by its name and load.
        for (const auto& over : typed->over) {
          if (overGates != nullptr) {
            for (const auto& edge : input.edges) {
              if (!edge.stretch && over == edge.name) {
                overGates->push_back({(edge.from[0] + edge.to[0]) / 2.0,
                                      (edge.from[1] + edge.to[1]) / 2.0});
              }
            }
          }
          take(over);
          for (const auto& edge : input.edges) {
            if (over == edge.name || over.starts_with(edge.name + " ")) {
              take(edge.feedline);
              for (const auto& end : edge.ends) {
                take(end);
              }
            }
          }
        }
      }
      auto against = typed->lost;
      against.insert(against.end(), typed->over.begin(), typed->over.end());
      if (against.empty() && typed->verdict != "SAT") {
        against.emplace_back(typed->verdict);
      }
      return against;
    }
    std::vector<std::string> names;
    for (const auto key : lost) {
      names.push_back(wireId(state.wires[key]));
    }
    return names;
  }

  /// The locally SAT states of every chain for `searchGlobalSat`, after the
  /// commit: the chain as it stands first, then the runs of options of the
  /// segments at its two ends (`researchSegment` handing them to
  /// `segmentCollect_`) that put its first or last coupler on another option
  /// and whose chain the slot check of that chain alone finds SAT, end
  /// options not kept yet first. The first and last couplers are where a
  /// chain meets its neighbours. A chain with a feedline edge at an edge over
  /// its capacity (`focus`: per chain, the waypoints at the ends of such
  /// edges) is searched on every segment that holds such a waypoint as well
  /// and keeps every locally SAT state, its ends moved or not, without the
  /// limit of `globalSatStates`; those that move the coupler of such a
  /// waypoint come first.
  ///
  /// Collects the chains of `order`, in that order, into `chainStates_`,
  /// which the caller sizes. `spent` holds the seconds every earlier call
  /// of the same search collected and gets this call's seconds added; it
  /// is reported and decides nothing.
  void collectChainStates(
      std::vector<Wire>& wires,
      const std::map<std::uint32_t, std::set<std::size_t>>& focus,
      const std::vector<std::uint32_t>& order,
      std::chrono::duration<double>& spent) {
    const auto chains = static_cast<std::uint32_t>(chains_.size());
    auto firstEdge = static_cast<std::uint32_t>(wires.size());
    for (const auto& edge : edges_) {
      firstEdge = std::min(firstEdge, edge.wire);
    }
    const std::vector<Wire> before(
        wires.begin(), wires.begin() + static_cast<std::ptrdiff_t>(firstEdge));
    const auto began = std::chrono::steady_clock::now();
    for (const auto chain : order) {
      const auto& points = chains_[chain];
      std::size_t first = points.size();
      std::size_t last = 0;
      for (std::size_t at = 0; at < points.size(); ++at) {
        if (!points[at].fixed) {
          first = std::min(first, at);
          last = std::max(last, at);
        }
      }
      if (first > last || points.size() < 3) {
        continue;
      }
      std::vector<std::size_t> current(points.size(), 0);
      for (std::size_t at = 0; at < points.size(); ++at) {
        if (!points[at].fixed) {
          current[at] = couplers_[points[at].coupler].chosen;
        }
      }
      auto& kept = chainStates_[chain];
      kept.push_back(current);
      std::set<std::vector<std::size_t>> seen{current};
      std::set<std::pair<std::size_t, std::size_t>> ends{
          {current[first], current[last]}};
      std::vector<std::vector<std::size_t>> others;
      std::uint32_t runs = 0;
      std::uint32_t sat = 0;
      std::chrono::duration<double> checking{0};
      const auto chainBegan = std::chrono::steady_clock::now();
      const auto own = localCheckWires(before, chain);
      const std::vector<bool> every(chains, true);
      std::vector<bool> only(chains, false);
      only[chain] = true;
      std::uint32_t fitting = 0;
      const auto span = std::min<std::size_t>(3, points.size() - 1);
      std::set<std::size_t> starts{0, points.size() - 1 - span};
      const auto focused = focus.find(chain);
      std::set<std::size_t> marked;
      if (focused != focus.end()) {
        for (const auto at : focused->second) {
          if (!points[at].fixed) {
            marked.insert(at);
          }
        }
        for (std::size_t lo = 0; lo + span < points.size(); ++lo) {
          if (std::ranges::any_of(marked, [&](const std::size_t at) {
                return at >= lo && at <= lo + span;
              })) {
            starts.insert(lo);
          }
        }
      }
      // A chain at an edge over keeps every locally SAT state, its ends
      // moved or not; first those that move the coupler of a marked
      // waypoint.
      const bool atOver = focused != focus.end();
      std::vector<std::vector<std::size_t>> moving;
      // The states SAT only with the chain alone, for a chain at an edge
      // over; after those SAT with the other chains standing.
      std::vector<std::vector<std::size_t>> aloneMoving;
      std::vector<std::vector<std::size_t>> aloneOthers;
      for (const auto lo : starts) {
        const RunCollector collect = [&](const std::vector<std::size_t>& layer,
                                         const std::vector<Path>& ways) {
          ++runs;
          auto options = current;
          for (std::size_t k = 0; k < layer.size(); ++k) {
            if (!points[lo + k].fixed) {
              options[lo + k] = layer[k];
            }
          }
          const bool moves =
              std::ranges::any_of(marked, [&](const std::size_t at) {
                return options[at] != current[at];
              });
          if (seen.contains(options) ||
              (!atOver && options[first] == current[first] &&
               options[last] == current[last])) {
            return false;
          }
          seen.insert(options);
          std::vector<const Path*> paths;
          for (std::size_t at = 0; at + 1 < points.size(); ++at) {
            paths.push_back(at >= lo && at < lo + ways.size()
                                ? &ways[at - lo]
                                : &chainEdgePaths_[chain][at]);
          }
          // A chain with an edge that has no way is no state of it.
          if (std::ranges::any_of(
                  paths, [](const Path* way) { return way->empty(); })) {
            return false;
          }
          // Two local checks; a state either finds SAT is kept. First as the
          // comparison of the local check with the rip-up makes it
          // (`slotCheckNow`): every other chain standing as the commit left
          // it, the terminal edges as wires, and only the chain's own wires,
          // the free wires beside them and its terminal edges routed and
          // judged. Such a
          // state can stand on its own. Then with the chain alone, its own
          // wires and the free wires beside them routed: such a state may
          // need its neighbours to move as well.
          auto state = stateOf(before, every, chain, &options, &paths);
          state.terminalsAsWires = true;
          std::set<std::string> judged = own;
          for (const auto& edge : state.edges) {
            if (edge.chain == chain && edge.terminal) {
              judged.insert(wireId(state.wires[edge.wire]));
            }
          }
          localWires_ = &judged;
          const auto checkBegan = std::chrono::steady_clock::now();
          const auto verdict = capacityVerdictOf(state);
          checking += std::chrono::steady_clock::now() - checkBegan;
          localWires_ = nullptr;
          std::vector<std::string> ownLost;
          for (const auto key : verdict.lost) {
            if (const auto name = wireId(state.wires[key]);
                judged.contains(name)) {
              ownLost.push_back(name);
            }
          }
          const bool fits = ownLost.empty();
          bool alone = false;
          if (!fits) {
            localWires_ = &own;
            const auto aloneBegan = std::chrono::steady_clock::now();
            alone = capacityVerdictOf(
                        stateOf(before, only, chain, &options, &paths))
                        .lost.empty();
            checking += std::chrono::steady_clock::now() - aloneBegan;
            localWires_ = nullptr;
          }
          if (!fits && !alone) {
            if (atOver) {
              std::string named;
              for (const auto option : options) {
                named += std::format(" {}", option);
              }
              std::string lostText;
              for (const auto& name : ownLost) {
                lostText += " " + name;
              }
              tell(std::format("global SAT: chain {} segment {}: locally "
                               "UNSAT on options{} (lost{})",
                               chain, lo, named, lostText));
            }
            return false;
          }
          ++sat;
          fitting += fits ? 1U : 0U;
          if (atOver) {
            std::string named;
            for (const auto option : options) {
              named += std::format(" {}", option);
            }
            tell(std::format("global SAT: chain {} segment {}: locally SAT "
                             "on options{}{}",
                             chain, lo, named,
                             fits ? "" : " with the chain alone"));
            ends.insert({options[first], options[last]});
            if (fits) {
              (moves ? moving : others).push_back(options);
            } else {
              (moves ? aloneMoving : aloneOthers).push_back(options);
            }
          } else if (ends.insert({options[first], options[last]}).second) {
            kept.push_back(options);
          } else {
            others.push_back(options);
          }
          return false;
        };
        segmentCollect_ = &collect;
        std::uint32_t badNow = 0;
        static_cast<void>(researchSegment(wires, {}, {}, Pass{},
                                          {.chain = chain,
                                           .lo = lo,
                                           .hi = lo + span,
                                           .fails = {},
                                           .because = {}},
                                          tuning_.globalSatRuns, badNow));
        segmentCollect_ = nullptr;
      }
      if (atOver) {
        kept.insert(kept.end(), std::make_move_iterator(moving.begin()),
                    std::make_move_iterator(moving.end()));
        kept.insert(kept.end(), std::make_move_iterator(others.begin()),
                    std::make_move_iterator(others.end()));
        kept.insert(kept.end(), std::make_move_iterator(aloneMoving.begin()),
                    std::make_move_iterator(aloneMoving.end()));
        kept.insert(kept.end(), std::make_move_iterator(aloneOthers.begin()),
                    std::make_move_iterator(aloneOthers.end()));
        others.clear();
      }
      for (auto& other : others) {
        if (kept.size() >= tuning_.globalSatStates) {
          break;
        }
        kept.push_back(std::move(other));
      }
      if (!atOver && kept.size() > tuning_.globalSatStates) {
        kept.resize(tuning_.globalSatStates);
      }
      say(std::format("global SAT: chain {}: {} runs of {} segment{}, {} "
                      "locally SAT on other options ({} with the other chains "
                      "standing){}; {} state{} kept, {} end pair{}; {:.1f}s, "
                      "{:.1f}s of it in slot checks",
                      chain, runs, starts.size(), starts.size() == 1 ? "" : "s",
                      sat, fitting,
                      moving.empty() ? std::string{}
                                     : std::format(", {} moving a waypoint at "
                                                   "an edge over",
                                                   moving.size()),
                      kept.size(), kept.size() == 1 ? "" : "s", ends.size(),
                      ends.size() == 1 ? "" : "s",
                      std::chrono::duration<double>(
                          std::chrono::steady_clock::now() - chainBegan)
                          .count(),
                      checking.count()));
    }
    spent += std::chrono::steady_clock::now() - began;
  }

  /// **The search for a globally SAT state** (`globalSatSearch`), after the
  /// commit. When the slot check of the whole chip finds a wire lost or an
  /// edge over, every segment of every chain is searched as the targeted
  /// repair searches it (`researchSegment`), and every run of options it
  /// reaches whose chain the local check finds SAT is a state of that chain;
  /// a chain keeps up to `globalSatStates`, end options (its first and last
  /// coupler) not kept yet first. Then whole-chip states, every chain on one
  /// of its states and most on other ends, are tried: the couplers moved as
  /// the coupler session moves them (`setCouplerOption`), every edge drawn,
  /// and the chip checked as the insertion checks it. Stage 1 tries every
  /// state of a chain at an edge over on its own; stage 2 starts from the
  /// failed combination with the fewest lost wires and edges over and moves
  /// one more chain at its edges over per round. The order of the
  /// combinations is fixed and nothing in it depends on time or chance;
  /// `globalSatTries` bounds how many are checked. The first SAT state
  /// stays; when there is none, every coupler goes back to its option.
  /// Under `globalSatLazy` a chain is collected only when a stage first
  /// needs its states. Whether the search moved a coupler.
  bool searchGlobalSat(std::vector<Wire>& wires) {
    const auto chains = static_cast<std::uint32_t>(chains_.size());
    const auto recordNoSearch = [this](const std::string_view why) {
      record("global-sat",
             {{"event", "result"}, {"searched", false}, {"why", why}});
    };
    if (chains < 2) {
      recordNoSearch("one chain: nothing to combine");
      return false;
    }
    // A chip with an edge that has no way is no SAT state, whatever the
    // check of the edges that stand says.
    const bool complete = std::ranges::all_of(
        edges_, [&](const Edge& edge) { return wires[edge.wire].drawn; });
    std::set<std::string> touching;
    std::vector<std::array<double, 2>> gatesAtCommit;
    auto lost = wholeChipLost(wires, &touching, &gatesAtCommit);
    if (lost.empty() && complete) {
      sat_ = true;
      say("global SAT: the chip is SAT after the commit; nothing to search");
      recordNoSearch("SAT after the commit");
      return false;
    }
    if (!complete) {
      lost.emplace_back("an edge without a way");
    }
    const auto began = std::chrono::steady_clock::now();
    std::string named;
    for (const auto& name : lost) {
      named += (named.empty() ? "" : " ") + name;
    }
    say(std::format("global SAT: the chip is UNSAT ({}); searching locally "
                    "SAT chain states",
                    named));
    record("global-sat", {{"event", "start"}, {"lost", lost}});
    // The waypoints at the ends of the feedline edges at an edge over.
    std::map<std::uint32_t, std::set<std::size_t>> focus;
    for (const auto& edge : edges_) {
      if (!edge.terminal && touching.contains(wireId(wires[edge.wire]))) {
        focus[edge.chain].insert(edge.from);
        focus[edge.chain].insert(edge.to);
      }
    }
    std::string focusText;
    for (const auto& [chain, at] : focus) {
      focusText +=
          std::format("{}chain {} at", focusText.empty() ? "" : "; ", chain);
      for (const auto point : at) {
        focusText += std::format(" {}", point);
      }
    }
    if (!focusText.empty()) {
      say(std::format("global SAT: the edges over lie on {}", focusText));
    }
    // The chains at an edge over are collected one at a time, and the states
    // of each are tried on their own before the next chain is collected;
    // another chain is collected only when the conflict-directed stage first
    // needs its states.
    chainStates_.assign(chains, {});
    std::chrono::duration<double> collecting{0};
    std::vector<std::size_t> original(couplers_.size());
    std::optional<Snapshot> taken;
    std::vector<std::vector<Path>> paths;
    std::optional<Field> fieldAtCommit;
    // Every combination starts from the chip as the commit left it: a
    // combination that failed leaves edges drawn elsewhere, and the next one
    // would start on them.
    const auto settle = [&]() {
      for (std::size_t index = 0; index < couplers_.size(); ++index) {
        original[index] = couplers_[index].chosen;
      }
      taken = snapshot(wires);
      paths = chainEdgePaths_;
    };
    const auto fullBack = [&]() {
      restore(wires, *taken);
      chainEdgePaths_ = paths;
      foreignRoomStale_ = true;
    };
    // `restore` builds the field again from every wire. It is built once
    // (`restoredWires`, `restoredField`); after that, the search puts back
    // only what the last `standOn` changed: the wires of the couplers it
    // moved and of their edges (`touchedWires`), the ways of their chains
    // (`touchedChains`), the rows of the field it wrote, the pads and the
    // couplers. `onRestored` says that the chip is that copy with one
    // `standOn` on top. `restore` reads nothing but the snapshot, so the
    // chip is the one a full `restore` would give, cell for cell.
    std::optional<std::vector<Wire>> restoredWires;
    std::optional<Field> restoredField;
    bool onRestored = false;
    std::set<std::uint32_t> touchedWires;
    std::set<std::uint32_t> touchedChains;
    const auto backToStart = [&]() {
      if (!restoredWires.has_value()) {
        fullBack();
        restoredWires = wires;
        restoredField = field_;
      } else {
        if (onRestored) {
          for (const auto key : touchedWires) {
            wires[key] = (*restoredWires)[key];
          }
          field_.restoreRowsFrom(*restoredField);
          for (const auto chain : touchedChains) {
            chainEdgePaths_[chain] = paths[chain];
          }
        } else {
          wires = *restoredWires;
          field_ = *restoredField;
          chainEdgePaths_ = paths;
        }
        bodies_ = taken->bodies;
        for (std::size_t index = 0; index < couplers_.size(); ++index) {
          couplers_[index].chosen = taken->chosen[index];
          couplers_[index].anchor = taken->anchors[index];
        }
        foreignRoomStale_ = true;
      }
      touchedWires.clear();
      touchedChains.clear();
      field_.track();
      onRestored = true;
    };
    // The chip exactly as the commit left it, the field as the commit built
    // it and not built again: where the next collection starts after a
    // chain's states were tried. The insertion runs outside a feedline pass,
    // so there is no crossing rule to put back.
    const auto backToCommit = [&]() {
      wires = taken->wires;
      bodies_ = taken->bodies;
      for (std::size_t index = 0; index < couplers_.size(); ++index) {
        couplers_[index].chosen = taken->chosen[index];
        couplers_[index].anchor = taken->anchors[index];
      }
      field_ = *fieldAtCommit;
      chainEdgePaths_ = paths;
      foreignRoomStale_ = true;
      onRestored = false;
    };
    // Moves the couplers to a whole-chip state; whether every edge of the
    // chip is drawn, those of an earlier state's couplers included.
    const auto standOn = [&](const std::vector<std::size_t>& options) {
      for (std::size_t index = 0; index < couplers_.size(); ++index) {
        if (couplers_[index].chosen != options[index]) {
          const auto& coupler = couplers_[index];
          touchedWires.insert(coupler.wire);
          for (const auto key : {coupler.edgeIn, coupler.edgeOut}) {
            if (key < wires.size()) {
              touchedWires.insert(key);
              touchedChains.insert(edges_[wires[key].slot].chain);
            }
          }
          static_cast<void>(setCouplerOption(
              wires, static_cast<std::uint32_t>(index), options[index]));
        }
      }
      return std::ranges::all_of(
          edges_, [&](const Edge& edge) { return wires[edge.wire].drawn; });
    };
    std::set<std::vector<std::size_t>> tried;
    std::uint32_t checks = 0;
    // The seconds the combinations take and where they go, for the report
    // alone.
    std::chrono::duration<double> combined{0};
    std::chrono::duration<double> restoring{0};
    std::chrono::duration<double> moving{0};
    std::chrono::duration<double> judging{0};
    std::uint32_t repeated = 0;
    std::string satMoved;
    // The chains a failed check names (`namedChains`): those whose feedline
    // edges an edge over lies on or ends at, terminal edges included; those
    // whose feedline holds a lost wire (`chainWires`); for a feedline edge
    // that has no way its own chain and every chain with a way that reaches
    // into the box the edge is searched in (`edgeBox`, `reachOf`); and when
    // none of these names a chain, the chain of the coupler nearest each
    // bottleneck over its capacity.
    ChipChains chip;
    for (const auto& edge : edges_) {
      chip.chainOfFeedline.emplace(wireId(wires[edge.wire]), edge.chain);
    }
    for (std::uint32_t chain = 0; chain < chains; ++chain) {
      chip.wiresOfChain.push_back(chainWires(wires, chain));
    }
    const auto chainsAt = [&](const std::set<std::string>& names,
                              const std::vector<std::string>& against,
                              const std::vector<std::array<double, 2>>& gates) {
      CheckFinding finding{.touching = names,
                           .against = against,
                           .overGates = gates,
                           .named = {}};
      for (const auto& edge : edges_) {
        if (wires[edge.wire].drawn) {
          continue;
        }
        finding.named.insert(edge.chain);
        const auto& points = chains_[edge.chain];
        const auto optionAt = [&](const std::size_t at) {
          return points[at].fixed ? std::size_t{0}
                                  : couplers_[points[at].coupler].chosen;
        };
        const auto box = edgeBox(edge.chain, edge.from, optionAt(edge.from),
                                 optionAt(edge.to));
        for (const auto& other : edges_) {
          const auto& way = wires[other.wire].way;
          if (other.chain != edge.chain && wires[other.wire].drawn &&
              !way.empty() && reachOf(way).meets(box)) {
            finding.named.insert(other.chain);
          }
        }
      }
      // The pads as the combination stands them.
      chip.couplers.clear();
      for (std::uint32_t chain = 0; chain < chains; ++chain) {
        for (const auto& point : chains_[chain]) {
          if (!point.fixed) {
            const auto& coupler = couplers_[point.coupler];
            const auto& centre = coupler.options[coupler.chosen].centre;
            chip.couplers.push_back(
                {{static_cast<double>(centre.x), static_cast<double>(centre.y)},
                 chain});
          }
        }
      }
      return namedChains(finding, chip);
    };
    // The combinations that were UNSAT since the last round of the
    // conflict-directed stage (`nextCombinationRound`).
    std::vector<FailedCombination> failed;
    // The commit is the first failed combination: the chains at its edges
    // over are the focus chains.
    failed.push_back({.pick = std::vector<std::size_t>(chains, 0),
                      .against = complete
                                     ? lost.size()
                                     : std::numeric_limits<std::size_t>::max(),
                      .named = chainsAt(touching, lost, gatesAtCommit),
                      .check = 0});
    const auto pickText = [&](const std::vector<std::size_t>& pick) {
      std::string moved;
      for (std::uint32_t chain = 0; chain < chains; ++chain) {
        if (pick[chain] != 0) {
          moved += std::format("{}chain {} state {}", moved.empty() ? "" : "; ",
                               chain, pick[chain]);
        }
      }
      return moved;
    };
    // Tries one combination: every chain on the state the combination
    // names. Whether it is SAT; the SAT combination stays on the chip, and
    // a combination that is not goes into `failed`.
    const auto tryPick = [&](const std::vector<std::size_t>& pick,
                             const std::string_view stage) {
      const auto tryBegan = std::chrono::steady_clock::now();
      ++checks;
      report_.progress("global SAT search",
                       std::format("combination {}", checks), checks,
                       tuning_.globalSatTries);
      backToStart();
      const auto moveBegan = std::chrono::steady_clock::now();
      restoring += moveBegan - tryBegan;
      auto options = original;
      for (std::uint32_t chain = 0; chain < chains; ++chain) {
        const auto& points = chains_[chain];
        if (pick[chain] == 0) {
          continue;
        }
        const auto& state = chainStates_[chain][pick[chain]];
        for (std::size_t at = 0; at < points.size(); ++at) {
          if (!points[at].fixed) {
            options[points[at].coupler] = state[at];
          }
        }
      }
      const bool drawn = standOn(options);
      moving += std::chrono::steady_clock::now() - moveBegan;
      const auto checkBegan = std::chrono::steady_clock::now();
      std::set<std::string> over;
      std::vector<std::array<double, 2>> gates;
      const auto against =
          drawn ? wholeChipLost(wires, &over, &gates)
                : std::vector<std::string>{"an edge without a way"};
      judging += std::chrono::steady_clock::now() - checkBegan;
      combined += std::chrono::steady_clock::now() - tryBegan;
      const auto moved = pickText(pick);
      record("global-sat", {{"event", "combination"},
                            {"index", checks},
                            {"stage", stage},
                            {"states", pick},
                            {"verdict", against.empty() ? "SAT" : "UNSAT"},
                            {"lost", against}});
      if (against.empty()) {
        for (std::uint32_t chain = 0; chain < chains; ++chain) {
          if (pick[chain] != 0) {
            tell(std::format("global SAT: chain {} state {} stands on "
                             "options{}",
                             chain, pick[chain],
                             optionsText(chainStates_[chain][pick[chain]])));
          }
        }
        satMoved = moved;
        return true;
      }
      std::string lostText;
      for (const auto& name : against) {
        lostText += (lostText.empty() ? "" : " ") + name;
      }
      tell(std::format("global SAT: combination {} ({}{}): UNSAT ({})", checks,
                       stage, moved, lostText));
      failed.push_back(
          {.pick = pick,
           .against =
               drawn ? against.size() : std::numeric_limits<std::size_t>::max(),
           .named = chainsAt(over, against, gates),
           .check = checks});
      return false;
    };
    // Tries the combinations in `picks`, in that order, until one is SAT or
    // the tries run out; a combination tried before is skipped.
    const auto tryAll = [&](const std::vector<std::vector<std::size_t>>& picks,
                            const std::string_view stage) {
      for (const auto& pick : picks) {
        if (checks >= tuning_.globalSatTries) {
          return false;
        }
        if (!tried.insert(pick).second) {
          ++repeated;
          continue;
        }
        if (tryPick(pick, stage)) {
          return true;
        }
      }
      return false;
    };
    const auto saySat = [&]() {
      sat_ = true;
      const auto seconds = std::chrono::duration<double>(
                               std::chrono::steady_clock::now() - began)
                               .count();
      say(std::format("global SAT: SAT after {} combination{}: {}; {:.1f}s",
                      checks, checks == 1 ? "" : "s", satMoved, seconds));
      record("global-sat", {{"event", "result"},
                            {"searched", true},
                            {"sat", true},
                            {"combinations", checks},
                            {"moved", satMoved},
                            {"seconds", seconds}});
    };
    // The summary of the combinations; the field stops recording rows.
    const auto sayCombined = [&]() {
      field_.untrack();
      say(std::format(
          "global SAT: {} combination{} in {:.1f}s: {:.1f}s back to the "
          "commit, {:.1f}s moving couplers, {:.1f}s in whole-chip checks; {} "
          "combination{} skipped as tried before",
          checks, checks == 1 ? "" : "s", combined.count(), restoring.count(),
          moving.count(), judging.count(), repeated, repeated == 1 ? "" : "s"));
    };
    const auto sayCollected = [&]() {
      say(std::format("global SAT: states collected in {:.1f}s",
                      collecting.count()));
    };
    const auto sayNone = [&]() {
      sat_ = false;
      backToStart();
      sayCombined();
      const auto seconds = std::chrono::duration<double>(
                               std::chrono::steady_clock::now() - began)
                               .count();
      say(std::format("global SAT: no SAT combination in {} checked; the "
                      "chip as the commit left it; {:.1f}s",
                      checks, seconds));
      record("global-sat", {{"event", "result"},
                            {"searched", true},
                            {"sat", false},
                            {"combinations", checks},
                            {"seconds", seconds}});
    };

    // Which chains have their states: a chain is collected only when a stage
    // first needs its states, from the chip as the commit left it.
    std::vector<bool> collected(chains, false);
    const auto collect = [&](const std::uint32_t chain) {
      if (collected[chain]) {
        return;
      }
      if (onRestored) {
        backToCommit();
      }
      collected[chain] = true;
      collectChainStates(wires, focus, {chain}, collecting);
    };
    settle();
    fieldAtCommit = field_;

    // **Stage 1**: every state of a chain at an edge over on its own, the
    // other chains as the commit left them, chain by chain and in the order
    // each chain keeps its states.
    for (const auto& entry : focus) {
      const auto chain = entry.first;
      collect(chain);
      const auto& kept = chainStates_[chain];
      record("global-sat",
             {{"event", "states"}, {"chain", chain}, {"states", kept.size()}});
      std::vector<std::vector<std::size_t>> picks;
      for (std::size_t k = 1; k < kept.size(); ++k) {
        picks.emplace_back(chains, 0)[chain] = k;
      }
      say(std::format("global SAT: stage 1: chain {} alone on its {} "
                      "state{}",
                      chain, picks.size(), picks.size() == 1 ? "" : "s"));
      if (tryAll(picks, "stage 1: ")) {
        sayCollected();
        sayCombined();
        saySat();
        return true;
      }
      if (checks >= tuning_.globalSatTries) {
        sayCollected();
        say("global SAT: out of combinations after stage 1");
        sayNone();
        return true;
      }
      if (onRestored) {
        backToCommit();
      }
    }

    // **Stage 2**, conflict-directed. A failed combination names chains
    // (`chainsAt`, `namedChains`): the chains at its edges over, terminal
    // edges included; the chains whose feedline holds one of its lost
    // wires; for an edge that has no way its own chain and the chains whose
    // ways reach into its box; and when none of these names a chain, the
    // chain of the pad nearest each gate over. The base of a round is the
    // failed combination of the round before (stage 1 and the commit for the
    // first round) with the fewest lost wires and edges over, the earliest on a
    // tie, that names a chain it does not move yet and that has a state to move
    // to. The round moves one such chain onto one of its states on top of the
    // base. The order is fixed: fewer moved chains first, then the smaller sum
    // of the state numbers, then the chain and the state with the smaller
    // index. Every round moves one chain more than the round before. The search
    // ends at a SAT combination, when no failed combination of the last round
    // names a chain to move, or after `globalSatTries` combinations.
    for (std::uint32_t round = 1;; ++round) {
      const auto next = nextCombinationRound(
          std::exchange(failed, {}),
          [&](const std::uint32_t chain) {
            collect(chain);
            return chainStates_[chain].size();
          },
          tried);
      if (!next.has_value()) {
        sayCollected();
        say(std::format("global SAT: stage 2 round {}: no failed combination "
                        "names a chain to move",
                        round));
        sayNone();
        return true;
      }
      const auto& base = next->base;
      const auto& picks = next->picks;
      std::string chainList;
      for (const auto chain : next->movable) {
        chainList += std::format("{}{}", chainList.empty() ? "" : " ", chain);
      }
      say(std::format("global SAT: stage 2 round {}: on combination {} ({}; "
                      "{} against it), moving one of chains {}; {} "
                      "combination{}",
                      round, base.check,
                      base.check == 0 ? std::string("the commit")
                                      : pickText(base.pick),
                      base.against == std::numeric_limits<std::size_t>::max()
                          ? std::string("an edge without a way")
                          : std::to_string(base.against),
                      chainList, picks.size(), picks.size() == 1 ? "" : "s"));
      if (tryAll(picks, std::format("stage 2 round {}: ", round))) {
        sayCollected();
        sayCombined();
        saySat();
        return true;
      }
      if (checks >= tuning_.globalSatTries) {
        sayCollected();
        say(std::format("global SAT: out of combinations in stage 2 round {}",
                        round));
        sayNone();
        return true;
      }
    }
  }

  /// The prefix search over every chain, once: a chain's answer is optimal
  /// against the fence that stands when it is searched, and the fence of a
  /// later chain holds the earlier ones' answers.
  [[nodiscard]] std::uint32_t optimizeChainsPrefix(std::vector<Wire>& wires) {
    std::uint32_t settled = 0;
    // Every step is checked with no other chain standing: each chain on its
    // own chip, judged by its own feedline's wires.
    const std::vector<bool> none(chains_.size(), false);
    for (std::uint32_t chain = 0; chain < chains_.size(); ++chain) {
      report_.progress("coupler insertion", std::format("chain {}", chain + 1),
                       chain, chains_.size());
      soloChain_ = chain;
      const auto own = localCheckWires(wires, chain);
      localWires_ = &own;
      stepSettled_ = tuning_.capacityCheck ? &none : nullptr;
      stepRefuses_ = tuning_.capacityCheck;
      auto answer = solveChainAStar(wires, chain);
      if (!answer.solved && stepRefuses_) {
        say(answer.refusalLimit
                ? std::format("coupler insertion: chain {}: the capacity rule "
                              "refused {} step{} before a run of options "
                              "joined the chain (chain_rule_refusals, {} "
                              "steps routed) — searched again without the "
                              "capacity rule",
                              chain, answer.refusedBeforeRun,
                              answer.refusedBeforeRun == 1 ? "" : "s",
                              answer.routed)
                : std::format("coupler insertion: chain {}: no run of options "
                              "joins the chain without closing a wire ({} "
                              "steps routed) — searched again without the "
                              "capacity rule",
                              chain, answer.routed));
        stepRefuses_ = false;
        stepSettled_ = nullptr;
        answer = solveChainAStar(wires, chain);
      }
      stepSettled_ = nullptr;
      stepRefuses_ = false;
      soloChain_ = NO_OWNER;
      localWires_ = nullptr;
      auto& points = chains_[chain];
      if (!answer.solved) {
        // **The chain is left as it stands**, and the edges it cannot draw
        // are reported by the commit.
        say(std::format("coupler insertion: chain {}: no run of options "
                        "joins the chain — leaving it as it stands ({} steps "
                        "routed)",
                        chain, answer.routed));
        continue;
      }
      ++settled;
      for (std::size_t at = 0; at < points.size(); ++at) {
        if (!points[at].fixed) {
          couplers_[points[at].coupler].chosen = answer.chosen[at];
        }
      }
      for (std::size_t at = 0; at + 1 < points.size(); ++at) {
        if (!answer.ways[at].empty()) {
          chainEdgePaths_[chain][at] = answer.ways[at];
        }
      }
      say(std::format("coupler insertion: chain {}: {} {} over {} layers, "
                      "{} prefixes expanded, {} steps routed",
                      chain, answerWord(answer), answer.cost, points.size(),
                      answer.expansions, answer.routed) +
          atWeightOne(answer));
    }
    // What the commit is about to find, said once. It decides nothing — with
    // one pass there is no second state to prefer — but it is the number
    // this whole approach is judged by, so it belongs in the log: an edge
    // counted here is an edge the commit will route again from scratch.
    const auto faults = stateFaults(wires);
    say(std::format(
        "coupler insertion: {} of {} chains settled{}{}, {} edge{} "
        "would not survive this state",
        settled, chains_.size(), " each on its own chip",
        tuning_.capacityCheck ? ", each checked against itself alone" : "",
        faults, faults == 1 ? "" : "s"));
    return settled;
  }

  /// The free wires beside every feedline, by name. A free wire is a ring
  /// wire that belongs to no feedline (`owned`, per chain). It joins the
  /// feedlines that own the nearest owned ring wire before it and the
  /// nearest after it, so the free wires in the ring between two feedlines
  /// join both. No feedline's check would judge them otherwise, and a
  /// feedline could take the room they need unseen.
  [[nodiscard]] std::vector<std::set<std::string>>
  freeWiresBeside(const std::vector<Wire>& wires,
                  const std::vector<std::set<std::string>>& owned) const {
    const auto total = ring_.size();
    std::vector<std::string> names(total);
    std::vector<std::vector<std::uint32_t>> owners(total);
    bool any = false;
    for (std::size_t at = 0; at < total; ++at) {
      if (ring_[at] >= wires.size()) {
        continue;
      }
      names[at] = wireId(wires[ring_[at]]);
      for (std::uint32_t chain = 0; chain < owned.size(); ++chain) {
        if (owned[chain].contains(names[at])) {
          owners[at].push_back(chain);
          any = true;
        }
      }
    }
    std::vector<std::set<std::string>> beside(owned.size());
    if (!any) {
      return beside;
    }
    for (std::size_t at = 0; at < total; ++at) {
      if (names[at].empty() || !owners[at].empty()) {
        continue;
      }
      for (const std::size_t back : {std::size_t{1}, total - 1}) {
        for (std::size_t step = 1, next = (at + back) % total; step < total;
             ++step, next = (next + back) % total) {
          if (!owners[next].empty()) {
            for (const auto chain : owners[next]) {
              beside[chain].insert(names[at]);
            }
            break;
          }
        }
      }
    }
    return beside;
  }

  /// The wires a local check of one chain routes and judges: its own
  /// feedline's (`chainWires`) and the free wires beside it
  /// (`freeWiresBeside`).
  [[nodiscard]] std::set<std::string>
  localCheckWires(const std::vector<Wire>& wires,
                  const std::uint32_t chain) const {
    std::vector<std::set<std::string>> owned;
    owned.reserve(chains_.size());
    for (std::uint32_t other = 0; other < chains_.size(); ++other) {
      owned.push_back(chainWires(wires, other));
    }
    auto names = std::move(owned[chain]);
    names.merge(freeWiresBeside(wires, owned)[chain]);
    return names;
  }

  /// The wires of one chain's feedline in the insertion, by name: for a
  /// chain with a launcher at both ends the ring from the resonator of its
  /// first coupler to that of its last, every plain wire and resonator
  /// between them, as the feedline routing's ring places its terminal edges
  /// next to those resonators; for a chain with a reflector end the
  /// resonators of its couplers and the wires that bridge one of its edges.
  [[nodiscard]] std::set<std::string>
  chainWires(const std::vector<Wire>& wires, const std::uint32_t chain) const {
    std::set<std::string> names;
    const auto& points = chains_[chain];
    std::vector<std::uint32_t> resonators;
    for (const auto& point : points) {
      if (!point.fixed && couplers_[point.coupler].wire < wires.size()) {
        resonators.push_back(couplers_[point.coupler].wire);
      }
    }
    const auto total = ring_.size();
    std::vector<std::size_t> at;
    for (const auto key : resonators) {
      const auto found = std::ranges::find(ring_, key);
      if (found != ring_.end()) {
        at.push_back(static_cast<std::size_t>(found - ring_.begin()));
      }
    }
    if (points.size() >= 2 && points.front().fixed && points.back().fixed &&
        at.size() >= 2 && total > 0) {
      // The ring from the first resonator to the last, one way round or the
      // other: the way that passes every resonator of the chain, the shorter
      // when both do.
      const auto span = [&](const std::size_t from, const std::size_t to) {
        return (to + total - from) % total;
      };
      const auto passes = [&](const std::size_t from, const std::size_t to) {
        return std::ranges::all_of(
            at, [&](const auto k) { return span(from, k) <= span(from, to); });
      };
      auto from = at.front();
      auto to = at.back();
      if (!passes(from, to) ||
          (passes(to, from) && span(to, from) < span(from, to))) {
        std::swap(from, to);
      }
      for (std::size_t step = 0; step <= span(from, to); ++step) {
        names.insert(wireId(wires[ring_[(from + step) % total]]));
      }
      return names;
    }
    for (const auto key : resonators) {
      names.insert(wireId(wires[key]));
    }
    if (chain < bridgers_.size()) {
      for (const auto& over : bridgers_[chain]) {
        for (const auto key : over) {
          names.insert(wireId(wires[key]));
        }
      }
    }
    return names;
  }

  /// A state of the insertion as the capacity analysis reads it: the
  /// wires, with a feedline wire for every edge drawn; those edges; and the
  /// couplers that stand.
  struct InsertionState {
    std::vector<Wire> wires;
    std::vector<Edge> edges;
    std::vector<bool> placed;
    /// Whether the terminal edges are wires of the capacity graph and not
    /// walls (`terminalDemands`): the check of the whole chip.
    bool terminalsAsWires = false;
    /// The option every coupler stands on in this state, by coupler: a
    /// placed coupler's pad stands there, as its lead does.
    std::vector<std::size_t> options;
  };

  /// The feedline edges of a state that edge `from` of chain `chain`
  /// meets: it takes a cell of one, or steps diagonally between two cells
  /// of one. The edges beside it in its own chain are left out, since they
  /// meet it at their coupler.
  [[nodiscard]] std::vector<std::uint32_t>
  feedlinesMet(const InsertionState& state, const std::uint32_t chain,
               const std::size_t from) const {
    const auto width = static_cast<std::size_t>(scene_.router.width);
    const auto own = std::ranges::find_if(state.edges, [&](const Edge& edge) {
      return edge.chain == chain && edge.from == from;
    });
    if (own == state.edges.end()) {
      return {};
    }
    std::unordered_map<std::size_t, std::uint32_t> ownerOf;
    for (const auto& edge : state.edges) {
      const bool beside = edge.chain == chain &&
                          (edge.from + 1 >= from && edge.from <= from + 1);
      if (beside || !state.wires[edge.wire].drawn) {
        continue;
      }
      for (const auto& point : state.wires[edge.wire].way) {
        ownerOf.emplace((static_cast<std::size_t>(point.y) * width) + point.x,
                        edge.wire);
      }
    }
    std::set<std::uint32_t> met;
    const auto& way = state.wires[own->wire].way;
    const auto at = [&](const std::uint32_t x, const std::uint32_t y) {
      const auto found =
          ownerOf.find((static_cast<std::size_t>(y) * width) + x);
      return found == ownerOf.end() ? NO_OWNER : found->second;
    };
    for (std::size_t k = 0; k < way.size(); ++k) {
      if (const auto owner = at(way[k].x, way[k].y); owner != NO_OWNER) {
        met.insert(owner);
      }
      if (k + 1 < way.size() && way[k].x != way[k + 1].x &&
          way[k].y != way[k + 1].y) {
        const auto a = at(way[k + 1].x, way[k].y);
        if (a != NO_OWNER && a == at(way[k].x, way[k + 1].y)) {
          met.insert(a);
        }
      }
    }
    return {met.begin(), met.end()};
  }

  /// The insertion with the chains `in` standing on the options and ways
  /// they settled on, and chain `chain`, when `options` is given, on
  /// `options` (one per waypoint) and `ways` (one per edge; an edge with no
  /// way is not drawn). Of that chain only the first `standing` waypoints
  /// stand, as a prefix of the chain search leaves it. A standing
  /// coupler has its pad and its resonator the lead `applyOption` gives
  /// it; a resonator whose coupler does not stand yet is left out, since
  /// its lead is not known. The feedline edges are numbered as the commit
  /// numbers them, so they carry the names the commit gives them.
  [[nodiscard]] InsertionState
  stateOf(const std::vector<Wire>& wires, const std::vector<bool>& in,
          const std::uint32_t chain = NO_OWNER,
          const std::vector<std::size_t>* options = nullptr,
          const std::vector<const Path*>* ways = nullptr,
          const std::size_t standing =
              std::numeric_limits<std::size_t>::max()) const {
    InsertionState state{.wires = wires,
                         .edges = {},
                         .placed = std::vector<bool>(couplers_.size(), false)};
    state.options.reserve(couplers_.size());
    for (const auto& coupler : couplers_) {
      state.options.push_back(coupler.chosen);
    }
    std::uint32_t slot = 0;
    for (std::uint32_t at = 0; at < chains_.size(); ++at) {
      const auto& points = chains_[at];
      const bool given = at == chain && options != nullptr;
      if (!in[at] && !given) {
        slot +=
            points.empty() ? 0U : static_cast<std::uint32_t>(points.size() - 1);
        continue;
      }
      for (std::size_t point = 0; point < points.size(); ++point) {
        if (points[point].fixed || (given && point >= standing)) {
          continue;
        }
        const auto index = points[point].coupler;
        const auto& coupler = couplers_[index];
        const auto& option =
            coupler.options[given ? (*options)[point] : coupler.chosen];
        state.placed[index] = true;
        state.options[index] = given ? (*options)[point] : coupler.chosen;
        // The lead as `applyOption` cuts it.
        auto& wire = state.wires[coupler.wire];
        auto lead = leadOf(option);
        wire.arc = std::move(lead.arc);
        wire.objective.source = lead.source;
        wire.couplerAtSource = index;
      }
      for (std::size_t from = 0; from + 1 < points.size(); ++from) {
        const Path* way = given && ways != nullptr ? (*ways)[from]
                                                   : &chainEdgePaths_[at][from];
        Wire feed;
        feed.key = static_cast<std::uint32_t>(state.wires.size());
        feed.slot = slot++;
        feed.feedline = true;
        feed.feasible = true;
        feed.terminal = points[from].fixed || points[from + 1].fixed;
        // Its two ends, for the port runs a terminal edge gets as a wire
        // of the capacity graph (`terminalDemands`): the launcher's cell
        // and the coupler's port on the chosen option.
        feed.objective.source = sourceOf(points[from]);
        feed.objective.target = targetOf(points[from + 1]);
        if (way != nullptr) {
          feed.way = *way;
        }
        feed.drawn = !feed.way.empty();
        state.edges.push_back({.chain = at,
                               .from = from,
                               .to = from + 1,
                               .wire = feed.key,
                               .terminal = feed.terminal,
                               .open = given && standing < points.size()});
        state.wires.push_back(std::move(feed));
      }
    }
    for (std::size_t index = 0; index < couplers_.size(); ++index) {
      if (!state.placed[index] && couplers_[index].wire < wires.size()) {
        state.wires[couplers_[index].wire].feasible = false;
      }
    }
    return state;
  }

  /// What the capacity graph of a state says: the outer wires it has no
  /// way for, and by how many wires its edges are short.
  struct CapacityVerdict {
    std::set<std::uint32_t> lost;
    std::uint32_t shortBy = 0;
    /// The edges the slot check finds over their capacity, by name.
    std::vector<std::string> over;
    double seconds = 0.0;
  };

  /// The outer wires the capacity graph of a state has no way for: a
  /// terminal in no chamber, no way at all, or a way over an edge with
  /// overflow, routed in turn (`checkInTurn`) or, under
  /// `SCPD_CAPACITY_FLOW`, by the flow.
  [[nodiscard]] std::set<std::uint32_t>
  lostWires(const InsertionState& state) const {
    return capacityVerdictOf(state).lost;
  }

  /// `lostWires`, and the shortfall of the edges with it.
  [[nodiscard]] CapacityVerdict
  capacityVerdictOf(const InsertionState& state) const {
    CapacityVerdict verdict;
    const auto began = std::chrono::steady_clock::now();
    auto pass = capacityPass(state.wires, state.edges, &state.placed, NO_OWNER,
                             true, state.terminalsAsWires, &state.options);
    verdict.seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - began)
            .count();
    verdict.shortBy = pass.check.shortBy;
    verdict.lost = lostOf(pass);
    {
      const auto checking = std::chrono::steady_clock::now();
      try {
        auto& graph = scratchGraph_;
        fillSlotGraphOfPass(graph, state.wires, pass, true);
        if (localWires_ != nullptr) {
          // The wires of the chain's feedline and the free wires beside it
          // alone (`localCheckWires`), and every terminal edge, which names
          // the mouths between the chains.
          std::erase_if(graph.wires, [&](const SlotGraph::Wire& wire) {
            return !wire.terminal && !localWires_->contains(wire.name);
          });
        }
        const auto result = slotCheck(graph);
        std::unordered_map<std::string, std::uint32_t> keyOf;
        for (const auto& wire : state.wires) {
          keyOf.emplace(wireId(wire), wire.key);
        }
        std::set<std::uint32_t> lost;
        for (const auto* names : {&result.lost, &result.crowded}) {
          for (const auto& name : *names) {
            if (const auto found = keyOf.find(name); found != keyOf.end()) {
              lost.insert(found->second);
            }
          }
        }
        verdict.lost = std::move(lost);
        verdict.shortBy = static_cast<std::uint32_t>(result.over.size());
        verdict.over = result.over;
      } catch (const std::invalid_argument& error) {
        tell(std::format("capacity step: the slot check failed ({}); the "
                         "check by stretches decides",
                         error.what()));
      }
      verdict.seconds += std::chrono::duration<double>(
                             std::chrono::steady_clock::now() - checking)
                             .count();
    }
    giveBack(pass);
    return verdict;
  }

  /// The capacity graph of a state of the insertion as the JSON
  /// `writeCapacityGraph` writes, with `said` as the first line of its
  /// summary. Nothing is said in the log. `lostOut`, when given, receives
  /// the wires of `lostWires`.
  [[nodiscard]] std::string
  capacityGraphJsonOf(const InsertionState& state, const std::string& said,
                      std::set<std::uint32_t>* lostOut = nullptr,
                      std::optional<SlotCheckResult>* slotOut = nullptr,
                      SlotGraph* slotGraphOut = nullptr) {
    auto pass = capacityPass(state.wires, state.edges, &state.placed, NO_OWNER,
                             true, state.terminalsAsWires, &state.options);
    if (slotOut != nullptr) {
      auto input = slotGraphOfPass(state.wires, pass);
      *slotOut = slotCheck(input);
      if (slotGraphOut != nullptr) {
        *slotGraphOut = std::move(input);
      }
    }
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    std::vector<MeasuredBottleneck> measured;
    measured.reserve(pass.gates.size());
    for (std::size_t index = 0; index < pass.gates.size(); ++index) {
      const auto& gate = pass.gates[index];
      MeasuredBottleneck one{.gate = gate,
                             .mouth = index >= pass.mouthsFrom &&
                                      index < pass.slotsFrom,
                             .slot = index >= pass.slotsFrom};
      one.cells = std::hypot(
          static_cast<double>((static_cast<std::int64_t>(gate.second) % width) -
                              (static_cast<std::int64_t>(gate.first) % width)),
          static_cast<double>((static_cast<std::int64_t>(gate.second) / width) -
                              (static_cast<std::int64_t>(gate.first) / width)));
      one.holds =
          wiresThroughGap(one.cells, static_cast<double>(tuning_.clearance));
      measured.push_back(std::move(one));
    }
    auto& graph = pass.problem.graph;
    std::vector<std::uint32_t> noWay;
    for (std::uint32_t demand = 0; demand < pass.problem.demands.size();
         ++demand) {
      if (!pass.check.ways[demand].has_value()) {
        noWay.push_back(pass.problem.wireOf[demand]);
        continue;
      }
      for (const auto index : *pass.check.ways[demand]) {
        graph[index].through.push_back(pass.problem.wireOf[demand]);
      }
    }
    const auto lost = lostOf(pass);
    if (lostOut != nullptr) {
      *lostOut = lost;
    }
    const std::vector<std::uint32_t> lostList(lost.begin(), lost.end());
    std::vector<std::string> summary{said};
    summary.push_back(std::format(
        "{} chambers, {} bottlenecks between two or more of them, {} "
        "stretches to cross",
        pass.chambers.count, pass.problem.gateEdges,
        graph.size() - pass.problem.gateEdges));
    for (const auto& note : pass.notes) {
      summary.push_back(note);
    }
    if (!noWay.empty()) {
      summary.push_back(std::format("no way in the graph for {}",
                                    namesOf(state.wires, noWay)));
    }
    if (!pass.problem.walled.empty()) {
      summary.push_back(std::format("a port in no chamber for {}",
                                    namesOf(state.wires, pass.problem.walled)));
    }
    if (pass.check.shortBy > 0) {
      summary.push_back(std::format("short by {} wire{}", pass.check.shortBy,
                                    pass.check.shortBy == 1 ? "" : "s"));
    }
    if (pass.check.sweeps > 1) {
      summary.push_back(std::format(
          "routed in {} sweeps, the wires that overfilled an edge taken off "
          "and routed again",
          pass.check.sweeps));
    }
    // The graph is captured, not recorded: it is the caller's answer.
    const bool wasGraphOnly = graphOnly_;
    graphOnly_ = true;
    capturedGraph_.clear();
    writeCapacityGraph(state.wires, pass.walls, pass.gates, measured,
                       pass.chambers, graph, pass.problem.outerWires,
                       pass.check, lostList.empty() ? "SAT" : "UNSAT", summary);
    graphOnly_ = wasGraphOnly;
    giveBack(pass);
    return std::move(capturedGraph_);
  }

  /// What `capacityGraphJsonOf` gives its caller besides the JSON, without
  /// building the JSON: the wires of `lostWires` and, when `slotOut` is
  /// given, the slot check of the graph and, in `slotGraphOut`, the graph it
  /// read. The global SAT search checks the whole chip once per combination,
  /// and on 69q the JSON it threw away was 2.8 MB each time.
  void capacityCheckOf(const InsertionState& state,
                       std::set<std::uint32_t>& lostOut,
                       std::optional<SlotCheckResult>* slotOut = nullptr,
                       SlotGraph* slotGraphOut = nullptr) {
    auto pass = capacityPass(state.wires, state.edges, &state.placed, NO_OWNER,
                             true, state.terminalsAsWires, &state.options);
    if (slotOut != nullptr) {
      auto input = slotGraphOfPass(state.wires, pass);
      *slotOut = slotCheck(input);
      if (slotGraphOut != nullptr) {
        *slotGraphOut = std::move(input);
      }
    }
    lostOut = lostOf(pass);
    giveBack(pass);
  }

  /// The labels of a pass's walls and chambers, a cell each, kept for the
  /// next pass: the chain search checks thousands of steps, and on a grid
  /// of millions of cells fresh memory costs more than the labelling.
  mutable std::vector<std::uint32_t> spareWallOf_;
  mutable std::vector<std::uint32_t> spareChambers_;
  void giveBack(CapacityPass& pass) const {
    spareWallOf_ = std::move(pass.walls.of);
    spareChambers_ = std::move(pass.chambers.of);
  }

  /// The graph every check of `capacityVerdictOf` fills (`fillSlotGraph`),
  /// kept between checks: a fresh graph a step allocates and clears a cell
  /// raster the size of the grid.
  mutable SlotGraph scratchGraph_;
  /// The code of every wall of the last view (`SlotGraph::RasterView`).
  mutable std::vector<std::uint16_t> viewWallCodes_;
  /// The outer wires a pass has no way for: a terminal in no chamber, no
  /// way at all, or a way over an edge with overflow.
  [[nodiscard]] static std::set<std::uint32_t>
  lostOf(const CapacityPass& pass) {
    std::set<std::uint32_t> lost(pass.problem.walled.begin(),
                                 pass.problem.walled.end());
    for (std::size_t demand = 0; demand < pass.check.ways.size(); ++demand) {
      const auto& way = pass.check.ways[demand];
      const bool over =
          way.has_value() &&
          std::ranges::any_of(*way, [&](const std::uint32_t index) {
            return pass.check.overflow[index] > 0;
          });
      if (!way.has_value() || over) {
        lost.insert(pass.problem.wireOf[demand]);
      }
    }
    return lost;
  }

  /// The pad of an option as the four corners of the rectangle its body
  /// cells span along its orientation, half a cell out.
  [[nodiscard]] std::vector<std::array<double, 2>>
  padCorners(const CouplerOption& option) const {
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto v = routing::headingVector(option.orientation);
    const auto norm =
        std::hypot(static_cast<double>(v.dx), static_cast<double>(v.dy));
    const auto ax = static_cast<double>(v.dx) / norm;
    const auto ay = static_cast<double>(v.dy) / norm;
    const auto bx = -ay;
    const auto by = ax;
    auto alongLow = std::numeric_limits<double>::max();
    auto alongHigh = std::numeric_limits<double>::lowest();
    auto acrossLow = std::numeric_limits<double>::max();
    auto acrossHigh = std::numeric_limits<double>::lowest();
    for (const auto cell : option.body) {
      const auto x =
          static_cast<double>(static_cast<std::int64_t>(cell) % width);
      const auto y =
          static_cast<double>(static_cast<std::int64_t>(cell) / width);
      alongLow = std::min(alongLow, (x * ax) + (y * ay));
      alongHigh = std::max(alongHigh, (x * ax) + (y * ay));
      acrossLow = std::min(acrossLow, (x * bx) + (y * by));
      acrossHigh = std::max(acrossHigh, (x * bx) + (y * by));
    }
    if (option.body.empty()) {
      return {};
    }
    std::vector<std::array<double, 2>> corners;
    for (const auto& [a, b] : {std::pair{alongLow - 0.5, acrossLow - 0.5},
                               std::pair{alongHigh + 0.5, acrossLow - 0.5},
                               std::pair{alongHigh + 0.5, acrossHigh + 0.5},
                               std::pair{alongLow - 0.5, acrossHigh + 0.5}}) {
      corners.push_back({(a * ax) + (b * bx), (a * ay) + (b * by)});
    }
    return corners;
  }

  /// The couplers as they stand, with every option of each, as JSON for the
  /// page that moves couplers (`CouplerSession`). Per coupler: its index,
  /// its resonator, its chain and waypoint, the option it stands on and its
  /// two feedline edges. Per option: the pad as four corners, the two
  /// feedline ports, the lead, the length of resonator left in layout
  /// units, and whether the chain search offers it (`openOptionsOf`: the
  /// shortfall rule, the body guard, the jogs).
  [[nodiscard]] std::string couplersJson(const std::vector<Wire>& wires) const {
    return couplersData(wires).dump();
  }

  /// The data of `couplersJson`.
  [[nodiscard]] nlohmann::json
  couplersData(const std::vector<Wire>& wires) const {
    using nlohmann::json;
    const auto point = [](const PathPoint& at) {
      return json::array({at.x, at.y});
    };
    const auto edgeName = [&wires](const std::uint32_t key) {
      return key < wires.size() ? json(wireId(wires[key])) : json(nullptr);
    };
    json list = json::array();
    for (std::uint32_t chain = 0; chain < chains_.size(); ++chain) {
      const auto& points = chains_[chain];
      const auto feedlines = feedlineCellsBut(chain);
      for (std::size_t at = 0; at < points.size(); ++at) {
        if (points[at].fixed) {
          continue;
        }
        const auto index = points[at].coupler;
        const auto& coupler = couplers_[index];
        const auto open = openOptionsOf(chain, at, &feedlines);
        json options = json::array();
        for (std::size_t k = 0; k < coupler.options.size(); ++k) {
          const auto& option = coupler.options[k];
          json lead = json::array();
          for (const auto& cell : option.arc) {
            lead.push_back(point(cell));
          }
          options.push_back(
              {{"index", k},
               {"open", std::ranges::find(
                            open, static_cast<std::uint32_t>(k)) != open.end()},
               {"orientation", option.orientation},
               {"offset", option.offset},
               {"secondPort", option.secondPort},
               {"jog", option.secondStraight},
               {"moved", {option.shiftX, option.shiftY}},
               {"centre", point(option.centre)},
               {"anchor", point(option.anchor)},
               {"in", point(option.in)},
               {"out", point(option.out)},
               {"pad", padCorners(option)},
               {"lead", lead},
               {"resonator", std::lround(lengthInUnits(option.way))}});
        }
        list.push_back({{"index", index},
                        {"resonator", wireId(wires[coupler.wire])},
                        {"chain", chain},
                        {"at", at},
                        {"chosen", coupler.chosen},
                        {"jogs", coupler.jogsUnlocked},
                        {"edgeIn", edgeName(coupler.edgeIn)},
                        {"edgeOut", edgeName(coupler.edgeOut)},
                        {"options", options}});
      }
    }
    return json{{"couplers", list}};
  }

  /// Put a coupler on another option after the insertion and draw its two
  /// feedline edges again, for the page that moves couplers
  /// (`CouplerSession`). The option is applied as the commit applies it
  /// (`applyOption`); both edges stand down first, so that neither is
  /// routed around the other's way to the old ports, and are then routed
  /// as the commit routes an edge, against every edge of its chain
  /// (`fenceEverything_`). Returns
  /// JSON: per edge whether it was drawn, its length in cells and its way.
  ///
  /// @throws std::invalid_argument when the coupler or the option does not
  /// exist.
  [[nodiscard]] std::string setCouplerOption(std::vector<Wire>& wires,
                                             const std::uint32_t index,
                                             const std::size_t option) {
    if (index >= couplers_.size() ||
        option >= couplers_[index].options.size()) {
      throw std::invalid_argument(
          std::format("coupler {} has no option {}", index, option));
    }
    applyOption(wires, index, option);
    const auto& coupler = couplers_[index];
    std::vector<std::uint32_t> keys;
    for (const auto key : {coupler.edgeIn, coupler.edgeOut}) {
      if (key < wires.size()) {
        keys.push_back(key);
      }
    }
    for (const auto key : keys) {
      auto& wire = wires[key];
      const auto& edge = edges_[wire.slot];
      if (wire.drawn) {
        lift(wire);
      }
      wire.drawn = false;
      wire.way.clear();
      chainEdgePaths_[edge.chain][edge.from].clear();
    }
    using nlohmann::json;
    json drawn = json::array();
    for (const auto key : keys) {
      auto& wire = wires[key];
      const auto edge = edges_[wire.slot];
      const auto& points = chains_[edge.chain];
      wire.objective.source = sourceOf(points[edge.from]);
      wire.objective.target = targetOf(points[edge.to]);
      wire.startStub = couplerRunOf(points[edge.from].coupler);
      wire.endStub = points[edge.to].fixed
                         ? tuning_.straightStart
                         : couplerRunOf(points[edge.to].coupler);
      const bool fencedBefore = fenceEverything_;
      fenceEverything_ = true;
      auto way = routeEdge(wires, wire.objective, edge);
      fenceEverything_ = fencedBefore;
      wire.way = std::move(way);
      wire.drawn = !wire.way.empty();
      if (wire.drawn) {
        place(wire);
      }
      chainEdgePaths_[edge.chain][edge.from] = wire.way;
      // An edge that comes closer to a neighbour than the feedline routing
      // grants the two at their coupler is no edge the feedline routing can
      // keep: it does not draw the edge again and finds both open. Such an
      // edge stands down, and the chip is not complete.
      std::string refused;
      if (wire.drawn) {
        for (const auto at : {edge.from, edge.to}) {
          // The edge before meets this one at its source, the edge after at
          // its target; a chain that ends at a reflector has none there.
          const auto& ways = chainEdgePaths_[edge.chain];
          if (points[at].fixed || (at == edge.from && edge.from == 0) ||
              (at == edge.to && edge.to >= ways.size())) {
            continue;
          }
          const auto other = at == edge.from ? edge.from - 1 : edge.to;
          const auto& theirs = ways[other];
          const auto& shared = couplers_[points[at].coupler];
          if (const auto pair =
                  tooClose(wire.way, theirs, shared.options[shared.chosen]);
              pair.has_value()) {
            refused = std::format(
                "NO WAY: ({},{}) is {:.1f} cells from ({},{}) of the edge "
                "it meets at coupler {}",
                pair->first.x, pair->first.y,
                std::hypot(static_cast<double>(pair->first.x) - pair->second.x,
                           static_cast<double>(pair->first.y) - pair->second.y),
                pair->second.x, pair->second.y, points[at].coupler);
            break;
          }
        }
      }
      if (!refused.empty()) {
        lift(wire);
        wire.drawn = false;
        wire.way.clear();
        chainEdgePaths_[edge.chain][edge.from].clear();
      }
      json laid = json::array();
      for (const auto& cell : wire.way) {
        laid.push_back(json::array({cell.x, cell.y}));
      }
      drawn.push_back({{"edge", wireId(wire)},
                       {"drawn", wire.drawn},
                       {"cells", wire.way.size()},
                       {"way", laid}});
      say(std::format("coupler session: '{}' on option {}: edge {} {}",
                      wireId(wires[coupler.wire]), option, wireId(wire),
                      wire.drawn
                          ? std::format("drawn, {} cells", wire.way.size())
                      : refused.empty() ? std::string("NO WAY")
                                        : refused));
    }
    return json{{"coupler", index}, {"option", option}, {"edges", drawn}}
        .dump();
  }

  /// The capacity graph of the chip as it stands, as the JSON
  /// `writeCapacityGraph` writes, without a picture
  /// (`CouplerSession`). The lines of the analysis are said as ever.
  [[nodiscard]] std::string capacityGraphJson(const std::vector<Wire>& wires) {
    capturedGraph_.clear();
    graphOnly_ = true;
    try {
      analyseCapacity(wires);
    } catch (...) {
      graphOnly_ = false;
      throw;
    }
    graphOnly_ = false;
    return std::move(capturedGraph_);
  }

  /// The capacity check of the coupler insertion (`capacityRule`) on the
  /// chip as it stands, with every chain on the options and ways it has now
  /// (`CouplerSession`). The check sees what the chain search sees once the
  /// last step of every chain stands: the wires as they were before the
  /// commit appended the feedline edges, and an edge per way of
  /// `chainEdgePaths_`. Returns JSON: `verdict`, `lost` (the outer wires
  /// with no way) and `graph`, the capacity graph the check saw, as
  /// `writeCapacityGraph` writes it. The check by crossing slots judges that
  /// graph: `verdict`, `lost`, `over` (the edges
  /// over) and `graph` are its own, and `previousVerdict` and
  /// `previousLost` keep those of the check before.
  [[nodiscard]] std::string insertionCheckJson(const std::vector<Wire>& wires) {
    auto first = static_cast<std::uint32_t>(wires.size());
    for (const auto& edge : edges_) {
      first = std::min(first, edge.wire);
    }
    const std::vector<Wire> before(
        wires.begin(), wires.begin() + static_cast<std::ptrdiff_t>(first));
    using nlohmann::json;
    const auto namesOfLost = [](const std::vector<Wire>& named,
                                const std::set<std::uint32_t>& lost) {
      json names = json::array();
      for (const auto key : lost) {
        names.push_back(wireId(named[key]));
      }
      return names;
    };
    // Every chain on its own first (user, 2026-10-14): the chip with that
    // chain's edges standing and no other's, its terminal edges as walls,
    // as the chain search checked it.
    json local = json::array();
    for (std::uint32_t chain = 0; chain < chains_.size(); ++chain) {
      std::vector<bool> only(chains_.size(), false);
      only[chain] = true;
      const auto alone = stateOf(before, only);
      const auto own = localCheckWires(before, chain);
      localWires_ = &own;
      const auto verdict = capacityVerdictOf(alone);
      localWires_ = nullptr;
      json stood = json::array();
      for (const auto& edge : alone.edges) {
        if (edge.chain == chain && edge.wire < alone.wires.size()) {
          stood.push_back(wireId(alone.wires[edge.wire]));
        }
      }
      local.push_back({{"chain", chain},
                       {"verdict", verdict.lost.empty() ? "SAT" : "UNSAT"},
                       {"lost", namesOfLost(alone.wires, verdict.lost)},
                       {"shortBy", verdict.shortBy},
                       {"edges", std::move(stood)}});
    }
    auto state = stateOf(before, std::vector<bool>(chains_.size(), true));
    state.terminalsAsWires = true;
    std::set<std::uint32_t> lost;
    std::optional<SlotCheckResult> typed;
    auto graph = capacityGraphJsonOf(
        state,
        state.terminalsAsWires
            ? "the capacity check of the coupler insertion, every chain "
              "standing, the terminal edges as wires"
            : "the capacity check of the coupler insertion, every chain "
              "standing",
        &lost, &typed);
    const auto verdict = lost.empty() ? "SAT" : "UNSAT";
    {
      try {
        // The verdict from the graph in memory; the JSON is the page.
        auto judged = json::parse(slotCheck(graph).graph);
        judged["previousVerdict"] = verdict;
        const auto& result = *typed;
        return json{{"verdict", result.verdict},
                    {"lost", result.lost},
                    {"over", result.over},
                    {"previousVerdict", verdict},
                    {"previousLost", namesOfLost(state.wires, lost)},
                    {"local", std::move(local)},
                    {"graph", std::move(judged)}}
            .dump();
      } catch (const std::invalid_argument& error) {
        say(std::format("coupler session: the slot check failed ({}); the "
                        "verdict of the stretches stands",
                        error.what()));
      }
    }
    return json{{"verdict", verdict},
                {"lost", namesOfLost(state.wires, lost)},
                {"local", std::move(local)},
                {"graph", json::parse(graph)}}
        .dump();
  }

  /// Hand the pictures to `debug` from now on (`CouplerSession`).
  void setDebug(Debug debug) { debug_ = std::move(debug); }

  /// What every round of the last sweep came to, as JSON: per round its
  /// direction, how many wires found a way in phase 1 and after relaxation,
  /// which failed, and for the last two rounds every member's way.
  [[nodiscard]] nlohmann::json roundsJson() const {
    auto list = nlohmann::json::array();
    for (std::size_t round = 0; round < rounds_.size(); ++round) {
      const auto& record = rounds_[round];
      auto ways = nlohmann::json::object();
      for (const auto& [name, way] : record.ways) {
        auto path = nlohmann::json::array();
        for (std::size_t k = 0; k < way.size(); ++k) {
          // Only the ends and the cells where the way turns.
          if (k == 0 || k + 1 == way.size() ||
              way[k].x - way[k - 1].x != way[k + 1].x - way[k].x ||
              way[k].y - way[k - 1].y != way[k + 1].y - way[k].y) {
            path.push_back(nlohmann::json::array({way[k].x, way[k].y}));
          }
        }
        ways[name] = std::move(path);
      }
      list.push_back({{"round", round},
                      {"forward", record.forward},
                      {"normal", record.normal},
                      {"relaxed", record.relaxed},
                      {"tooShort", record.tooShort},
                      {"failed", record.failed},
                      {"ways", std::move(ways)}});
    }
    return list;
  }

  /// What the wires come to (`CouplerSession`): the fails of every wire as
  /// `failsOf` counts them, every wire's way, and `rounds`, as JSON. A way
  /// keeps only its ends and the cells where it changes direction.
  [[nodiscard]] std::string routedJson(std::vector<Wire>& wires,
                                       const nlohmann::json& rounds = {}) {
    std::vector<std::uint32_t> every(wires.size());
    std::iota(every.begin(), every.end(), 0U);
    const auto fails = failsOf(wires, every);
    using nlohmann::json;
    json list = json::array();
    for (const auto& wire : wires) {
      json path = json::array();
      const auto& way = wire.way;
      const auto step = [&way](const std::size_t from, const bool alongX) {
        const auto at = [&way, alongX](const std::size_t k) {
          return static_cast<std::int64_t>(alongX ? way[k].x : way[k].y);
        };
        return at(from + 1) - at(from);
      };
      for (std::size_t k = 0; k < way.size(); ++k) {
        const bool turns = k == 0 || k + 1 == way.size() ||
                           step(k - 1, true) != step(k, true) ||
                           step(k - 1, false) != step(k, false);
        if (turns) {
          path.push_back(json::array({way[k].x, way[k].y}));
        }
      }
      list.push_back({{"name", wireId(wire)},
                      {"kind", wire.feedline    ? "feedline"
                               : wire.inner     ? "inner"
                               : wire.resonator ? "resonator"
                                                : "plain"},
                      {"drawn", wire.drawn},
                      {"verdict", fails.verdicts[wire.key]},
                      {"path", std::move(path)}});
    }
    return json{{"bad", fails.bad()},      {"unrouted", fails.unroutedIds},
                {"open", fails.openIds},   {"crossing", fails.crossingIds},
                {"short", fails.shortIds}, {"long", fails.longIds},
                {"loops", fails.loopIds},  {"rounds", rounds},
                {"wires", std::move(list)}}
        .dump();
  }

  /// The lead a resonator is held to in the capacity graph and the feedline
  /// routing, and where its search starts: the option's lead, or with a
  /// second dogleg the first quarter turn and its straight alone, less the
  /// margin the insertion needed and the routing does not.
  /// What is cut off goes back to the search, which starts where the kept
  /// head ends.
  struct Lead {
    Path arc;
    PathPoint source;
  };
  [[nodiscard]] Lead leadOf(const CouplerOption& option) const {
    Lead out{.arc = option.arc, .source = option.arcEnd};
    if (option.secondStraight > 0 && option.firstLead < option.arc.size()) {
      out.source = option.arc[option.firstLead];
      out.source.heading = option.couplerOrientation;
      out.arc.resize(option.firstLead);
    }
    if (out.arc.size() > tuning_.couplerLeadMargin + 1) {
      out.arc.resize(out.arc.size() - tuning_.couplerLeadMargin);
      out.source = {.x = out.arc.back().x,
                    .y = out.arc.back().y,
                    .heading = out.source.heading,
                    .primitive = out.arc.back().primitive};
    }
    return out;
  }

  /// Whether a coupler's option keeps a clearance from some feedline
  /// ways: its body, its two feedline ports and its lead. The segment
  /// search lays the edges of a run one after the other with the interior
  /// pads hidden, so an edge may run where the option of a later coupler
  /// then stands, or a later edge where an earlier coupler's option stands.
  /// On 17q, chain 0 on options launcher 31 1 0, f0 ran across the pad of
  /// resonator 11's coupler on option 1, and the insertion could not draw f1
  /// into it.
  [[nodiscard]] bool optionClearOf(const CouplerOption& option,
                                   const std::vector<const Path*>& ways) const {
    const auto clear = static_cast<double>(tuning_.clearance);
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    std::vector<std::pair<std::int64_t, std::int64_t>> cells;
    cells.reserve(option.body.size() + option.arc.size() + 2);
    for (const auto cell : option.body) {
      cells.emplace_back(static_cast<std::int64_t>(cell) % width,
                         static_cast<std::int64_t>(cell) / width);
    }
    for (const auto& point : leadOf(option).arc) {
      cells.emplace_back(point.x, point.y);
    }
    cells.emplace_back(option.in.x, option.in.y);
    cells.emplace_back(option.out.x, option.out.y);
    for (const Path* way : ways) {
      for (const auto& point : *way) {
        for (const auto& [x, y] : cells) {
          const auto dx = static_cast<double>(x) - point.x;
          const auto dy = static_cast<double>(y) - point.y;
          if ((dx * dx) + (dy * dy) < clear * clear) {
            return false;
          }
        }
      }
    }
    return true;
  }

  /// Put a coupler's option in place: the resonator's way is cut back to
  /// the coupler, its search from now on starts at the end of the turn on
  /// the coupling run, and the body becomes an obstacle. The body it had
  /// before is cleared first, which the prototype needs
  /// `remove_cpw_coupler_obstacles` for.
  void applyOption(std::vector<Wire>& wires, const std::uint32_t index,
                   const std::size_t option) {
    auto& coupler = couplers_[index];
    auto& wire = wires[coupler.wire];
    if (coupler.anchor.x != 0 || coupler.anchor.y != 0 ||
        wire.couplerAtSource == index) {
      const auto& before = coupler.options[coupler.chosen];
      for (const auto cell : before.body) {
        bodies_.set(cell, false);
      }
    }
    coupler.chosen = option;
    const auto& chosen = coupler.options[option];
    lift(wire);
    // The lead the routing holds the resonator to (`leadOf`).
    auto lead = leadOf(chosen);
    wire.arc = std::move(lead.arc);
    wire.objective.source = lead.source;
    // The run is a length in layout units, and `straightStart` counts the
    // cells that span it **along an axis** (`cellsFor`, which says so). The
    // router walks that many cells along the heading, and a diagonal step
    // spans the square root of two, so a resonator leaving a diagonal
    // coupler was held to a run half again as long as the rule asks — 155
    // units against 100 on 17q. `cellsOn` is the conversion the coupler's
    // own run and body already go through.
    //
    // There is no such run: the lead is already the rule's length, and the
    // prototype forces nothing beyond it.
    wire.startStub = 0U;
    wire.fixed.clear();
    wire.couplerAtSource = index;
    wire.routed = false;
    wire.tooShort = false;
    wire.tooLong = false;
    coupler.anchor = chosen.anchor;
    wire.way = chosen.way;
    wire.drawn = true;
    foreignRoomStale_ = true;
    for (const auto cell : chosen.body) {
      bodies_.set(cell, true);
    }
    place(wire);
  }

  /// Say where a coupler sits and what is left of its resonator.
  void sayCoupler(const std::vector<Wire>& wires,
                  const std::uint32_t index) const {
    const auto& coupler = couplers_[index];
    const auto& wire = wires[coupler.wire];
    const auto& option = coupler.options[coupler.chosen];
    const auto label = wire.targetPort < ports_.size()
                           ? ports_[wire.targetPort].label
                           : std::string("?");
    const auto way = lengthInUnits(wire.way);
    tell(std::format(
        "coupler insertion: Coupler for '{} -> {}' inserted at ({}, {}) "
        "with angle {}°, resonator leaves on {}° ({}), search starts on "
        "{}°, offset {} of the nearest launcher, resonator "
        "port {} at ({},{}), "
        "guarded {}, {:.0f} units of way + "
        "{:.0f} to the port = {:.0f}, target {:.0f}",
        wireId(wire), label, option.centre.x, option.centre.y,
        degreesOfHeading(option.couplerOrientation),
        option.way.empty() ? 0 : degreesOfHeading(option.way.front().heading),
        (!option.way.empty() && routing::isDiagonal(option.way.front().heading))
            ? "diagonal"
            : "axial",
        degreesOfHeading(option.arcEnd.heading), option.offset,
        option.secondPort ? 2 : 1, option.anchor.x, option.anchor.y,
        option.guarded, way, wire.anchorGapUnits, way + wire.anchorGapUnits,
        tuning_.targetLengthUnits));
  }

  // --------------------------------------------- Under the feedline
  // constraints

  /// The sweep of a pass under the feedline constraints, in ring order: the
  /// edge of a chain that arrives at a coupler comes right before that
  /// coupler's resonator, the edge that leaves it right after, as the
  /// prototype builds its ring of objectives.
  [[nodiscard]] std::vector<std::uint32_t>
  ringWithEdges(const std::vector<Wire>& wires,
                const std::vector<std::uint32_t>& outer) const {
    std::vector<std::uint32_t> members;
    for (const auto key : outer) {
      const auto& wire = wires[key];
      const auto coupler = wire.couplerAtSource;
      if (coupler != NO_OWNER && couplers_[coupler].edgeIn != NO_OWNER &&
          wires[couplers_[coupler].edgeIn].terminal) {
        members.push_back(couplers_[coupler].edgeIn);
      }
      members.push_back(key);
      if (coupler != NO_OWNER && couplers_[coupler].edgeOut != NO_OWNER &&
          wires[couplers_[coupler].edgeOut].terminal) {
        members.push_back(couplers_[coupler].edgeOut);
      }
    }
    return members;
  }

  /// Tell every conventional wire which edge of a chain it may cross: the
  /// edge between the two couplers whose resonators stand on either side of
  /// it in the ring. It crosses that one at a right angle and no other.
  void assignBridges(std::vector<Wire>& wires,
                     const std::vector<std::uint32_t>& outer) const {
    const auto ring = static_cast<std::uint32_t>(outer.size());
    if (ring == 0) {
      return;
    }
    std::unordered_map<std::uint32_t, std::uint32_t> placeOf;
    for (std::uint32_t at = 0; at < ring; ++at) {
      placeOf.emplace(wires[outer[at]].slot, at);
    }
    for (const auto& edge : edges_) {
      if (edge.terminal) {
        continue;
      }
      const auto& points = chains_[edge.chain];
      const auto from =
          placeOf.find(wires[couplers_[points[edge.from].coupler].wire].slot);
      const auto to =
          placeOf.find(wires[couplers_[points[edge.to].coupler].wire].slot);
      if (from == placeOf.end() || to == placeOf.end()) {
        continue;
      }
      for (auto at = (from->second + 1) % ring; at != to->second;
           at = (at + 1) % ring) {
        auto& wire = wires[outer[at]];
        if (!wire.resonator) {
          wire.bridged = edge.wire;
        }
      }
    }
  }

  /// What a pass under the feedline constraints starts with: every wire of
  /// it drawn again, and the crossing rule built from the edges between
  /// couplers.
  void beginPass(std::vector<Wire>& wires,
                 const std::vector<std::uint32_t>& members, const Pass& pass) {
    // Every wire is placed again here, so a resonator may stand anywhere.
    foreignRoomStale_ = true;
    feedlinePass_ = pass.feedlines;
    if (!pass.feedlines) {
      router_.clearOrthogonalConstraints();
      return;
    }
    if (!pass.onlyUnsettled) {
      for (const auto member : members) {
        wires[member].routed = false;
      }
    }
    rebuildCrossingRule(wires);
    say(std::format("feedline routing settings: the terminal edges' coupler "
                    "runs guarded by {} cells more, {} pair{} fenced",
                    tuning_.terminalStubExtra, tuning_.fencePairs,
                    tuning_.fencePairs == 1 ? "" : "s"));
    if (feedlinePass_ && report_.wants(Detail::Items)) {
      std::string order;
      for (const auto member : members) {
        const auto& wire = wires[member];
        order += (order.empty() ? "" : " ") + wireId(wire) +
                 (wire.feedline
                      ? std::format("[{}{}]", startsAtLauncher(wire) ? "F" : "",
                                    endsAtLauncher(wire) ? "L" : "")
                      : (wire.resonator ? "(r)" : ""));
      }
      say(std::format("the ring the feedline pass sweeps: {}", order));
    }
  }

  /// The crossing rule from the edges between couplers as they are drawn
  /// now. Built when a pass under the feedline constraints starts and
  /// whenever a snapshot is put back, so that the rule the count and the
  /// searches use is never the one of a trial thrown away.
  void rebuildCrossingRule(const std::vector<Wire>& wires) {
    // Every edge of a chain, the terminal ones included: a wire that may
    // pass a terminal edge has to cross it at a right angle like any other,
    // or the crossing is one no air bridge can be built over.
    std::vector<Path> crossable;
    for (const auto& edge : edges_) {
      const auto& wire = wires[edge.wire];
      // The prototype builds the rule from the edges between couplers and
      // leaves the terminal ones out of it (`is_first_last_feedline`,
      // `FinalGrid.cpp:10677`): a terminal edge runs from the border to the
      // first coupler, and a rule over it closes the plane from outside in.
      if (edge.terminal) {
        continue;
      }
      if (wire.drawn) {
        crossable.push_back(wire.way);
      }
    }
    router_.buildOrthogonalConstraints(crossable, {}, tuning_.crossingReach);
  }

  /// Fence a search by the feedlines and price the room around them, as
  /// the prototype's feedline routing does: every edge inflated by the
  /// clearance stands in the way, except the edge a conventional wire
  /// bridges, which it may cross once at a right angle, and the edges of a
  /// resonator's own coupler, which the meeting leaves open. The room
  /// around every edge costs five times the wire price.
  /// Whether the relaxation level being built has let go of this wire.
  [[nodiscard]] bool isLetGo(const std::uint32_t key) const {
    return std::ranges::find(letGo_, key) != letGo_.end();
  }

  void constrainByFeedlines(const Wire& wire, const std::vector<Wire>& wires,
                            const Pass& pass) {
    feedlineWires_ = pass.feedlines ? &wires : nullptr;
    if (!pass.feedlines) {
      router_.setSingleCrossingFeedline(nullptr, 0, 1);
      return;
    }
    // A resonator crosses no feedline edge between couplers, its own
    // coupler's included (user, 2026-10-13): the copper of every such edge,
    // one cell to each side so that two diagonal steps cannot cross at a
    // cell corner either, is closed. The terminal edges stay open to it.
    if (wire.resonator) {
      const auto width = static_cast<std::int64_t>(scene_.router.width);
      for (const auto& edge : edges_) {
        if (edge.terminal || edge.wire >= wires.size() ||
            !wires[edge.wire].drawn) {
          continue;
        }
        alongDisc(wires[edge.wire].way, stencilFor(1),
                  [&](const std::int64_t x, const std::int64_t y) {
                    corridor_.set(static_cast<std::size_t>((y * width) + x),
                                  true);
                  });
      }
    }
    for (const auto& edge : edges_) {
      const auto& other = wires[edge.wire];
      // The run a terminal edge has to make at its coupler, closed whether
      // or not the edge is drawn: see `terminalStubGuard`.
      if (edge.terminal && other.key != wire.key) {
        guardTerminalStub(wire, other);
      }
      // An edge the relaxation has let go of closes nothing more, like every
      // wire let go of: its room is priced (`priceTheEdges`), and it is
      // drawn again afterwards.
      if (isLetGo(other.key)) {
        continue;
      }
      if (!other.drawn || other.key == wire.key || other.key == wire.bridged) {
        continue;
      }
      // The first and last edge of a chain run from a launcher on the border
      // to the first coupler, which is deep in the fan-in. Fenced, such an
      // edge cuts the plane in two from the border inward, and every wire on
      // the far side of it has to go around the coupler at its end.
      //
      // The prototype leaves them out of the fence of **every** wire, not
      // only of the ones that may cross them: `forbidden_feedline_paths` is
      // built with `if (feedline.is_first_last_feedline) continue;` in both
      // its branches (`FinalGrid.cpp:3730`, `:3744`), the resonators' branch
      // included. Fencing a resonator by them was ours.
      //
      // A wire that is no edge fences them
      // after all (user, 2026-10-13): no wire is prescribed to cross a
      // terminal edge, and the count holds every wire to the rule against
      // it, so a search that may run over one finds ways the count refuses.
      const bool fencesTerminals = !wire.feedline;
      if (edge.terminal && !fencesTerminals) {
        continue;
      }
      fence(wire, {&other});
    }
    // Two edges of one chain may come within the rule of each other at the
    // coupler they share (`meetAt`), but may not cross there (user,
    // 2026-10-13). The copper of every other edge of the wire's chain, one
    // cell to each side so that two diagonal steps cannot cross at a cell
    // corner either, is closed, as `routeEdge` closes it in the insertion.
    if (wire.feedline && wire.edge < edges_.size()) {
      const auto width = static_cast<std::int64_t>(scene_.router.width);
      const auto chain = edges_[wire.edge].chain;
      for (const auto& edge : edges_) {
        if (edge.chain != chain || edge.wire >= wires.size()) {
          continue;
        }
        const auto& other = wires[edge.wire];
        if (other.key == wire.key || !other.drawn) {
          continue;
        }
        alongDisc(other.way, stencilFor(1),
                  [&](const std::int64_t x, const std::int64_t y) {
                    corridor_.set(static_cast<std::size_t>((y * width) + x),
                                  true);
                  });
      }
    }
    // A feedline edge keeps the clearance from every resonator; the meeting
    // at its own coupler stays open.
    if (wire.feedline) {
      for (const auto& other : wires) {
        if (other.resonator && other.drawn && other.key != wire.key) {
          fence(wire, {&other});
        }
      }
    }
    // The bridged edge is crossed under the crossing rule alone, at a right
    // angle within its band, as the prototype crosses it: the one-crossing
    // overlay the router offers asks for a straight run on the far side
    // that a port beside the feedline cannot give.
    router_.setSingleCrossingFeedline(nullptr, 0, 1);
    if (wire.feedline) {
      openTheRunsOut(wire, wires);
    }
    priceTheEdges(wire, wires, pass);
  }

  /// The room around an edge, five times the price of the room around any
  /// other wire, as the prototype prices `all_feedline_paths`
  /// (`FinalGrid.cpp:11733`). Inflated by the clearance, which is the
  /// radius `compute_proximity_grid` is handed there.
  ///
  /// **The terminal edges are priced with the rest.** The prototype leaves
  /// them out of this list and closes them hard instead
  /// (`forbidden_paths_start_end`, `FinalGrid.cpp:11747`), so under it they
  /// need no price. Here they are drawn again like any other edge, and a
  /// wire that is neither fenced by them nor made to pay for their room
  /// treats a launcher run as free ground (user, 2026-09-30).
  ///
  /// Its own method because the refinement builds the room price after the
  /// constraints and so has to stamp these again on top of it.
  void priceTheEdges(const Wire& wire, const std::vector<Wire>& wires,
                     const Pass& pass) {
    if (!pass.feedlines) {
      return;
    }
    const auto strong = static_cast<std::uint8_t>(
        std::min<std::uint32_t>(127U, 5U * tuning_.wireProximityPenalty));
    for (const auto& edge : edges_) {
      const auto& other = wires[edge.wire];
      if (other.drawn && other.key != wire.key) {
        stampDisc(other.way, tuning_.clearance, strong);
      }
    }
  }

  /// Close, in the corridor of an edge search, the coupler-end runs of the
  /// terminal edges of every **other** chain whose edges are committed:
  /// the run into the first coupler and the run out of the last, each with
  /// `terminalStubExtra` cells beyond it and the clearance around the
  /// whole. The chain being searched is left out — its own terminal edges
  /// are fenced by `fenceCommittedEdges` and `prefixFence_` as the rules
  /// there say, and its couplers may still move. See `terminalStubGuard`.
  void guardSettledTerminalRuns(const Edge& edge) {
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto& stencil = stencilFor(tuning_.clearance);
    const auto close = [&](const Path& cells) {
      alongDisc(
          cells, stencil, [&](const std::int64_t x, const std::int64_t y) {
            corridor_.set(static_cast<std::size_t>((y * width) + x), true);
          });
    };
    for (std::uint32_t other = 0; other < chains_.size(); ++other) {
      if (other == edge.chain || other >= chainEdgePaths_.size()) {
        continue;
      }
      const auto& points = chains_[other];
      const auto& ways = chainEdgePaths_[other];
      if (points.size() < 3 || ways.size() + 1 != points.size()) {
        continue;
      }
      // The first edge, committed, from a launcher into the first coupler.
      if (points.front().fixed && !points[1].fixed && !ways.front().empty()) {
        const auto run = couplerRunOf(points[1].coupler);
        close(router_.straightStub(targetOf(points[1]), true,
                                   run + tuning_.terminalStubExtra));
      }
      // The last edge, committed, out of the last coupler to a launcher.
      const auto last = points.size() - 2;
      if (points.back().fixed && !points[last].fixed && !ways.back().empty()) {
        const auto run = couplerRunOf(points[last].coupler);
        close(router_.straightStub(sourceOf(points[last]), false,
                                   run + tuning_.terminalStubExtra));
      }
    }
  }

  /// Close, in the corridor of an edge search, the coupler-end run of the
  /// terminal edge of the same chain that meets the edge at one of its
  /// couplers: the run into the first coupler for the edge after it, the
  /// run out of the last coupler for the edge before it, each with
  /// `terminalStubExtra` cells beyond it and the clearance around the
  /// whole. The couplers stand on the options under test, so the runs are
  /// those the terminal edge will have to make. The edge's own run at the
  /// coupler opens again with its slot.
  void guardOwnTerminalRuns(const Edge& edge) {
    const auto& points = chains_[edge.chain];
    if (edge.terminal || points.size() < 3) {
      return;
    }
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto& stencil = stencilFor(tuning_.clearance);
    const auto close = [&](const Path& cells) {
      alongDisc(
          cells, stencil, [&](const std::int64_t x, const std::int64_t y) {
            corridor_.set(static_cast<std::size_t>((y * width) + x), true);
          });
    };
    // The edge after, from the coupler this edge ends at to a launcher.
    if (edge.to + 1 < points.size() && points[edge.to + 1].fixed &&
        !points[edge.to].fixed) {
      const auto run = couplerRunOf(points[edge.to].coupler);
      close(router_.straightStub(sourceOf(points[edge.to]), false,
                                 run + tuning_.terminalStubExtra));
    }
    // The edge before, from a launcher into the coupler this edge starts at.
    if (edge.from >= 1 && points[edge.from - 1].fixed &&
        !points[edge.from].fixed) {
      const auto run = couplerRunOf(points[edge.from].coupler);
      close(router_.straightStub(targetOf(points[edge.from]), true,
                                 run + tuning_.terminalStubExtra));
    }
  }

  /// Close the straight run a terminal edge makes at its coupler — into the
  /// first coupler for an edge that starts at a launcher, out of the last
  /// for one that ends at a launcher — and `terminalStubExtra` cells beyond
  /// it, inflated by the clearance, to the search of `wire`. The launcher
  /// end needs nothing: the port band walls it. See `terminalStubGuard`.
  void guardTerminalStub(const Wire& wire, const Wire& edge) {
    Path cells;
    if (startsAtLauncher(edge)) {
      const auto in =
          router_.straightStub(edge.objective.target, true,
                               edge.endStub + tuning_.terminalStubExtra);
      cells.insert(cells.end(), in.begin(), in.end());
    }
    if (endsAtLauncher(edge)) {
      const auto out = router_.straightStub(
          edge.objective.source, false,
          startStubOf(edge, tuning_.straightStart) + tuning_.terminalStubExtra);
      cells.insert(cells.end(), out.begin(), out.end());
    }
    if (cells.empty()) {
      return;
    }
    closeRoomOf(wire, edge, cells);
    openFixedPlaces(wire);
  }

  // ------------------------------------------------------------- The repair

  /// The state a trial of the repair changes, and puts back.
  struct Snapshot {
    std::vector<Wire> wires;
    std::vector<std::size_t> chosen;
    std::vector<PathPoint> anchors;
    grid::BitGrid bodies;
  };

  [[nodiscard]] Snapshot snapshot(const std::vector<Wire>& wires) const {
    Snapshot taken{
        .wires = wires, .chosen = {}, .anchors = {}, .bodies = bodies_};
    for (const auto& coupler : couplers_) {
      taken.chosen.push_back(coupler.chosen);
      taken.anchors.push_back(coupler.anchor);
    }
    return taken;
  }

  /// Put a snapshot back, and the field with it: everything is charged
  /// again from what the wires hold.
  void restore(std::vector<Wire>& wires, const Snapshot& taken) {
    wires = taken.wires;
    bodies_ = taken.bodies;
    for (std::size_t index = 0; index < couplers_.size(); ++index) {
      couplers_[index].chosen = taken.chosen[index];
      couplers_[index].anchor = taken.anchors[index];
    }
    field_ = Field(scene_.router, tuning_.clearance);
    for (auto& wire : wires) {
      wire.placed = false;
      wire.endsOnly = false;
      if (!wire.way.empty()) {
        place(wire);
      } else if (!wire.fixed.empty()) {
        field_.chargeFixed(wire.fixed);
        wire.endsOnly = true;
      }
    }
    if (feedlinePass_) {
      rebuildCrossingRule(wires);
    }
  }

  /// Turn a placed coupler to another of its options and draw its two
  /// edges again. Nothing is changed when an edge finds no way.
  [[nodiscard]] bool turnCoupler(std::vector<Wire>& wires,
                                 const std::uint32_t index,
                                 const std::size_t option) {
    auto& coupler = couplers_[index];
    const auto taken = snapshot(wires);
    for (const auto key : {coupler.edgeIn, coupler.edgeOut}) {
      if (key != NO_OWNER) {
        lift(wires[key]);
        wires[key].way.clear();
        wires[key].drawn = false;
      }
    }
    applyOption(wires, index, option);
    for (const auto key : {coupler.edgeIn, coupler.edgeOut}) {
      if (key == NO_OWNER) {
        continue;
      }
      auto& wire = wires[key];
      const auto& edge = edges_[wire.edge];
      const auto& points = chains_[edge.chain];
      wire.objective.source = sourceOf(points[edge.from]);
      wire.objective.target = targetOf(points[edge.to]);
      wire.fixed.clear();
      wire.way = routeEdge(wires, wire.objective, edge);
      if (wire.way.empty()) {
        restore(wires, taken);
        return false;
      }
      wire.drawn = true;
      wire.routed = false;
      place(wire);
    }
    return true;
  }

  /// The wires a turned coupler unsettles: its resonator, its two edges,
  /// and every wire of the ring between it and the next resonator on
  /// either side. The prototype's dirty walk.
  [[nodiscard]] std::vector<std::uint32_t>
  unsettledBy(const std::vector<Wire>& wires,
              const std::vector<std::uint32_t>& members,
              const std::uint32_t index) const {
    const auto& coupler = couplers_[index];
    std::vector<std::uint32_t> dirty{coupler.wire};
    for (const auto key : {coupler.edgeIn, coupler.edgeOut}) {
      if (key != NO_OWNER) {
        dirty.push_back(key);
      }
    }
    const auto total = members.size();
    const auto found = std::ranges::find(members, coupler.wire);
    if (found == members.end() || total < 2) {
      return dirty;
    }
    const auto at = static_cast<std::size_t>(found - members.begin());
    for (const int direction : {1, -1}) {
      for (std::size_t step = 1; step < total; ++step) {
        const auto place =
            (at + total + (static_cast<std::size_t>(direction) * step)) % total;
        const auto& wire = wires[members[place]];
        if (wire.resonator) {
          break;
        }
        dirty.push_back(wire.key);
      }
    }
    return dirty;
  }

  // --------------------------------------------------- The targeted repair

  /// One segment of one chain the targeted repair re-searches: waypoints
  /// `lo..hi` of `chains_[chain]`, `lo < hi`, the two ends **fixed** on the
  /// option they stand on (a launcher is fixed anyway), the interior
  /// `lo+1..hi-1` re-searched and the edges `lo..hi-1` re-routed. A segment
  /// with no interior (`hi == lo + 1`) re-routes one edge and re-searches
  /// nothing — allowed, it is the cheapest trial there is.
  struct Segment {
    std::uint32_t chain = 0;
    std::size_t lo = 0;
    std::size_t hi = 0;
    /// The fails it was built from, by wire key, and what each was.
    std::vector<std::uint32_t> fails;
    std::vector<std::string> because;
  };

  /// A coupler as the rest of the log names it: by its resonator, in
  /// quotes, as the `room` lines and `sayCoupler` do — never by its index.
  [[nodiscard]] std::string couplerName(const std::vector<Wire>& wires,
                                        const std::uint32_t coupler) const {
    if (coupler == NO_OWNER || coupler >= couplers_.size()) {
      return "launcher";
    }
    return std::format("'{}'", wireId(wires[couplers_[coupler].wire]));
  }

  [[nodiscard]] std::string segmentName(const std::vector<Wire>& wires,
                                        const Segment& segment) const {
    std::string interior;
    for (std::size_t at = segment.lo + 1; at < segment.hi; ++at) {
      const auto& point = chains_[segment.chain][at];
      interior += std::format("{}{}", interior.empty() ? "" : " ",
                              point.fixed ? std::string("launcher")
                                          : couplerName(wires, point.coupler));
    }
    return std::format("chain {} waypoints {}..{} ({} edge{}, interior "
                       "coupler{} {})",
                       segment.chain, segment.lo, segment.hi,
                       segment.hi - segment.lo,
                       segment.hi - segment.lo == 1 ? "" : "s",
                       segment.hi - segment.lo <= 2 ? "" : "s",
                       interior.empty() ? "none" : interior);
  }

  /// **The triage**: every fail of the feedline routing traced to the chain
  /// edges and couplers that can be its cause, and those grouped into the
  /// segments of one chain the re-search works on. Nothing here decides
  /// anything about geometry; it only says where to look.
  ///
  /// - An **open** wire names its partners through `conflictsIn`, the own
  ///   cell of each conflict included. Blamed: the wire's bridged edge and
  ///   each partner's (and the edge itself where either is a feedline),
  ///   every edge whose way comes within `couplerReach` of the conflict
  ///   cell, and every coupler whose lead or pad lies within the clearance
  ///   of it — the lane pairs of 45q and 57q lean on a pad, 69q's resonator
  ///   9 lies one cell from its own lead (handover-cpw-coupler-insertion,
  ///   *The room rules*).
  /// - A **crossing** wire names the cell where it breaks the rule; blamed
  ///   is every edge within `crossingReach` + 1 of it, which is the edge
  ///   whose rule it broke.
  /// - An **unrouted** wire blames its bridged edge and the couplers of the
  ///   two resonators that flank it in the ring, as the blind repair's
  ///   `near` did; a resonator blames its own coupler, an edge itself.
  /// - Two **edges of different chains that cross** (`feedlineCrossingPairs`)
  ///   blame both edges, each counted as a fail of its own wire.
  /// - Every open, crossing or unrouted wire also blames the edges within
  ///   the clearance of its **search start**: a wire whose first cells are
  ///   closed is dead on arrival in every round, whatever else the fail
  ///   says.
  ///
  /// A blamed edge marks its two waypoints, a blamed coupler its one. Per
  /// chain the maximal runs of marked waypoints are widened by
  /// `researchGrow` at each end, clamped to the chain, merged where they
  /// touch, and become the segments; a segment carries the fails that marked
  /// its waypoints. Segments are ordered by fails carried, then by chain.
  /// Cross-chain fails yield one segment per chain, searched one after the
  /// other, so the second sees the first's result.
  ///
  /// One `tell` per fail and per segment, one `say` per chip.
  [[nodiscard]] std::vector<Segment>
  blameFails(std::vector<Wire>& wires,
             const std::vector<std::uint32_t>& members, const Fails& fails) {
    std::vector<std::vector<std::vector<std::uint32_t>>> marks(chains_.size());
    for (std::uint32_t chain = 0; chain < chains_.size(); ++chain) {
      marks[chain].assign(chains_[chain].size(), {});
    }
    std::map<std::uint32_t, std::string> because;
    std::set<std::uint32_t> unblamed;
    const auto note = [&](const std::uint32_t key, const std::string& what) {
      auto& text = because[key];
      text += (text.empty() ? "" : "; ") + what;
    };
    const auto mark = [&](const std::uint32_t chain, const std::size_t at,
                          const std::uint32_t fail) {
      auto& list = marks[chain][at];
      if (std::ranges::find(list, fail) == list.end()) {
        list.push_back(fail);
      }
    };
    const auto add = [](std::string& named, const std::string& name) {
      if (named.find(name) == std::string::npos) {
        named += (named.empty() ? "" : ", ") + name;
      }
    };
    const auto blameEdge = [&](const std::size_t index,
                               const std::uint32_t fail, std::string& named) {
      if (index >= edges_.size()) {
        return;
      }
      const auto& edge = edges_[index];
      mark(edge.chain, edge.from, fail);
      mark(edge.chain, edge.to, fail);
      add(named, std::format("{} (chain {} edge {})", wireId(wires[edge.wire]),
                             edge.chain, edge.from));
    };
    const auto blameCoupler = [&](const std::uint32_t coupler,
                                  const std::uint32_t fail,
                                  std::string& named) {
      if (coupler == NO_OWNER || coupler >= chainOfCoupler_.size()) {
        return;
      }
      const auto [chain, at] = chainOfCoupler_[coupler];
      if (chain == NO_OWNER) {
        return;
      }
      mark(chain, at, fail);
      add(named, std::format("coupler {} (chain {} waypoint {})",
                             couplerName(wires, coupler), chain, at));
    };
    // The edge a wire key stands for, or none.
    const auto edgeOfWire = [&](const std::uint32_t key) {
      return (key != NO_OWNER && key < wires.size() && wires[key].feedline)
                 ? static_cast<std::size_t>(wires[key].edge)
                 : edges_.size();
    };
    // Every coupler whose lead or pad lies within the clearance of a cell.
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto couplersNear = [&](const PathPoint& cell,
                                  const std::uint32_t fail,
                                  std::string& named) {
      const auto limit = static_cast<double>(tuning_.clearance);
      for (std::uint32_t index = 0; index < couplers_.size(); ++index) {
        const auto& option = couplers_[index].options[couplers_[index].chosen];
        bool near = distanceToWay(cell, option.arc) <= limit;
        for (std::size_t at = 0; !near && at < option.body.size(); ++at) {
          const auto body = static_cast<std::int64_t>(option.body[at]);
          near =
              std::hypot(static_cast<double>(body % width) - cell.x,
                         static_cast<double>(body / width) - cell.y) <= limit;
        }
        if (near) {
          blameCoupler(index, fail, named);
        }
      }
    };
    // The edges within the clearance of a wire's search start — the source
    // moved along its heading by the stub, where the router begins. A wire
    // whose first cells are closed is dead on arrival in every round, so an
    // edge standing there is its cause whatever else the fail says: 17q's
    // wire 31 crosses f9 and is blamed on f9, but it finds no way because
    // f8 lies 19 cells from its start (`deadOnArrival`, 2026-10-04).
    const auto blameAtTheStart = [&](const Wire& wire, const std::uint32_t fail,
                                     std::string& named) {
      const auto stub =
          static_cast<std::int64_t>(startStubOf(wire, tuning_.straightStart));
      const auto& source = wire.objective.source;
      const auto v = routing::headingVector(source.heading);
      const auto x = static_cast<std::int64_t>(source.x) + (v.dx * stub);
      const auto y = static_cast<std::int64_t>(source.y) + (v.dy * stub);
      if (x < 0 || y < 0 || x >= width ||
          y >= static_cast<std::int64_t>(scene_.router.height)) {
        return;
      }
      const PathPoint start{.x = static_cast<std::uint32_t>(x),
                            .y = static_cast<std::uint32_t>(y),
                            .heading = source.heading,
                            .primitive = 0};
      for (const auto& [edge, far] : edgesNear(
               wires, start, static_cast<double>(tuning_.clearance) + 1.0)) {
        if (edges_[edge].wire != wire.key) {
          blameEdge(edge, fail, named);
        }
      }
    };
    const auto said = [&](const std::uint32_t key, const std::string& what,
                          const std::string& named) {
      if (named.empty()) {
        unblamed.insert(key);
      }
      tell(std::format(
          "targeted repair: fail {} ({}) blames {}", wireId(wires[key]), what,
          named.empty() ? std::string("nothing within reach") : named));
    };

    for (const auto key : fails.openKeys) {
      auto& wire = wires[key];
      std::vector<std::uint32_t> blockers;
      std::vector<PathPoint> where;
      lift(wire);
      conflictsIn(wire.way, wire, wires, &blockers, &where);
      place(wire);
      std::string named;
      std::string with;
      if (wire.feedline) {
        blameEdge(wire.edge, key, named);
      }
      blameEdge(edgeOfWire(wire.bridged), key, named);
      for (std::size_t at = 0; at < blockers.size(); ++at) {
        const auto& partner = wires[blockers[at]];
        with +=
            std::format("{}{}", with.empty() ? "" : " and ", wireId(partner));
        if (partner.feedline) {
          blameEdge(partner.edge, key, named);
        }
        blameEdge(edgeOfWire(partner.bridged), key, named);
        if (at < where.size()) {
          const auto& cell = where[at];
          with += std::format(" at ({},{})", cell.x, cell.y);
          for (const auto& [edge, far] :
               edgesNear(wires, cell, couplerReach())) {
            blameEdge(edge, key, named);
          }
          couplersNear(cell, key, named);
        }
      }
      blameAtTheStart(wire, key, named);
      const auto what = std::format("open with {}", with.empty() ? "-" : with);
      note(key, what);
      said(key, what, named);
    }
    for (const auto key : fails.crossingKeys) {
      const auto& wire = wires[key];
      std::string named;
      std::string what = "crossing nowhere";
      if (const auto cell = crossingCellOf(wire); cell.has_value()) {
        what = std::format("crossing at ({},{})", cell->x, cell->y);
        // The rule's halo is a disc of `crossingReach` cells around each
        // straight cell, and a cell of a curve's zone lies one cell beyond a
        // curve: one cell more finds the edge in either.
        for (const auto& [edge, far] :
             edgesNear(wires, *cell,
                       static_cast<double>(tuning_.crossingReach) + 2.0)) {
          blameEdge(edge, key, named);
        }
      }
      blameAtTheStart(wire, key, named);
      note(key, what);
      said(key, what, named);
    }
    for (const auto key : fails.unroutedKeys) {
      const auto& wire = wires[key];
      std::string named;
      if (wire.feedline) {
        blameEdge(wire.edge, key, named);
      } else if (wire.couplerAtSource != NO_OWNER) {
        blameCoupler(wire.couplerAtSource, key, named);
      } else {
        blameEdge(edgeOfWire(wire.bridged), key, named);
        const auto found = std::ranges::find(members, key);
        if (found != members.end()) {
          const auto at = static_cast<std::size_t>(found - members.begin());
          const auto total = members.size();
          for (const int direction : {1, -1}) {
            for (std::size_t step = 1; step < total; ++step) {
              const auto place =
                  (at + total + (static_cast<std::size_t>(direction) * step)) %
                  total;
              const auto& other = wires[members[place]];
              if (other.resonator) {
                blameCoupler(other.couplerAtSource, key, named);
                break;
              }
            }
          }
        }
      }
      blameAtTheStart(wire, key, named);
      note(key, "unrouted");
      said(key, "unrouted", named);
    }
    for (const auto& [i, j] : feedlineCrossingPairs(wires)) {
      for (const auto [mine, other] : {std::pair{i, j}, std::pair{j, i}}) {
        const auto key = edges_[mine].wire;
        std::string named;
        blameEdge(mine, key, named);
        blameEdge(other, key, named);
        const auto what =
            std::format("crosses {} of chain {}",
                        wireId(wires[edges_[other].wire]), edges_[other].chain);
        note(key, what);
        said(key, what, named);
      }
    }

    // The segments: maximal runs of marked waypoints per chain, widened,
    // clamped, merged where they touch after widening.
    const auto grow = RESEARCH_GROW;
    std::vector<Segment> segments;
    for (std::uint32_t chain = 0; chain < chains_.size(); ++chain) {
      const auto size = chains_[chain].size();
      std::vector<std::pair<std::size_t, std::size_t>> runs;
      for (std::size_t at = 0; at < size;) {
        if (marks[chain][at].empty()) {
          ++at;
          continue;
        }
        auto end = at;
        while (end + 1 < size && !marks[chain][end + 1].empty()) {
          ++end;
        }
        const auto lo = at >= grow ? at - grow : 0;
        const auto hi = std::min(size - 1, end + grow);
        if (!runs.empty() && runs.back().second >= lo) {
          runs.back().second = std::max(runs.back().second, hi);
        } else {
          runs.emplace_back(lo, hi);
        }
        at = end + 1;
      }
      for (const auto& [lo, hi] : runs) {
        Segment segment{
            .chain = chain, .lo = lo, .hi = hi, .fails = {}, .because = {}};
        for (std::size_t at = lo; at <= hi; ++at) {
          for (const auto fail : marks[chain][at]) {
            if (std::ranges::find(segment.fails, fail) == segment.fails.end()) {
              segment.fails.push_back(fail);
              segment.because.push_back(because[fail]);
            }
          }
        }
        if (hi <= lo) {
          tell(std::format("targeted repair: chain {} waypoint {} is blamed "
                           "alone and the segment is widened by nothing — no "
                           "edge to route, dropped",
                           chain, lo));
          continue;
        }
        segments.push_back(std::move(segment));
      }
    }
    std::ranges::stable_sort(segments, [](const Segment& a, const Segment& b) {
      if (a.fails.size() != b.fails.size()) {
        return a.fails.size() > b.fails.size();
      }
      if (a.chain != b.chain) {
        return a.chain < b.chain;
      }
      return a.lo < b.lo;
    });
    std::string chainsNamed;
    for (const auto& segment : segments) {
      std::string from;
      for (std::size_t at = 0; at < segment.fails.size(); ++at) {
        from +=
            std::format("{}{} ({})", at == 0 ? "" : ", ",
                        wireId(wires[segment.fails[at]]), segment.because[at]);
      }
      tell(std::format("targeted repair: segment {} from fails {}",
                       segmentName(wires, segment), from));
      chainsNamed +=
          std::format("{}{}", chainsNamed.empty() ? "" : " ", segment.chain);
    }
    say(std::format("targeted repair: {} fail{} → {} segment{} (chains {}), {} "
                    "fail{} with no edge or coupler within reach",
                    because.size(), because.size() == 1 ? "" : "s",
                    segments.size(), segments.size() == 1 ? "" : "s",
                    chainsNamed.empty() ? "-" : chainsNamed, unblamed.size(),
                    unblamed.size() == 1 ? "" : "s"));
    return segments;
  }

  /// Phase 4's repair, dispatched: the targeted re-search of coupler options
  /// under `repairSearch`, the blind `repair` of before without it. Says
  /// which switches are in force first, as the insertion says its own.
  /// How many waypoints a segment of the targeted repair is widened by to
  /// either side.
  static constexpr std::size_t RESEARCH_GROW = 1;
  /// Every wire of the feedline pass is drawn again at least once: the way
  /// a resonator holds when the pass begins is the coupler's lead spliced
  /// onto the outer routing's way, a kink no search would return.
  static constexpr bool FORCE_REROUTE = true;

  void repairFeedlines(std::vector<Wire>& wires,
                       const std::vector<std::uint32_t>& members,
                       const std::vector<std::uint32_t>& every,
                       const Pass& pass) {
    say(std::format("targeted repair settings: {} ring member{} redrawn, "
                    "segments grown by {}, {} round{}, budget {} legality "
                    "tests (repair_trials)",
                    tuning_.researchRingMembers,
                    tuning_.researchRingMembers == 1 ? "" : "s", RESEARCH_GROW,
                    tuning_.researchRounds,
                    tuning_.researchRounds == 1 ? "" : "s",
                    tuning_.repairTrials));
    researchSegments(wires, members, every, pass);
  }

  /// Hide or show the pads of some couplers in `bodies_`, at the option each
  /// stands on. A segment's interior couplers are hidden for the length of
  /// its search: `corridorOfEdge` closes `bodies_` as hard obstacles, and
  /// the pads that will not be there once the couplers move would otherwise
  /// make every candidate edge route around them — the symptom being a cost
  /// that never beats the current run.
  void setPads(const std::vector<std::uint32_t>& couplers,
               const bool standing) {
    for (const auto index : couplers) {
      const auto& coupler = couplers_[index];
      for (const auto cell : coupler.options[coupler.chosen].body) {
        bodies_.set(cell, standing);
      }
    }
  }

  /// Every drawn edge's cells but a segment's own, for the option guard of
  /// `openOptionsOf`: an option whose pad lies on a feedline that stands is
  /// closed. `feedlineCellsBut` reads the insertion's ways; here the edges
  /// have wires, and the pass may have moved the terminal ones.
  [[nodiscard]] grid::BitGrid standingCellsBut(const std::vector<Wire>& wires,
                                               const std::uint32_t chain,
                                               const std::size_t lo,
                                               const std::size_t hi) const {
    grid::BitGrid cells(scene_.router.width, scene_.router.height);
    for (const auto& edge : edges_) {
      if (edge.chain == chain && edge.from >= lo && edge.from < hi) {
        continue;
      }
      const auto& way = edgeWayOf(wires, edge.chain, edge.from);
      for (const auto& point : way) {
        if (point.x < scene_.router.width && point.y < scene_.router.height) {
          cells.set(scene_.router.index(point.x, point.y), true);
        }
      }
    }
    return cells;
  }

  /// What one segment's re-search came to.
  struct Research {
    bool accepted = false;
    /// Legality tests run — the budget's unit.
    std::uint32_t trials = 0;
    /// Runs refused after a test, and runs refused without one because they
    /// were the current run with the same ways.
    std::uint32_t rejected = 0;
    std::uint32_t skippedCurrent = 0;
    /// Edge searches of the prefix search.
    std::uint32_t routed = 0;
    std::uint32_t runs = 0;
    bool outOfTrials = false;
    bool exhausted = false;
  };

  /// **The re-search of one segment**: the prefix search on the sub-chain
  /// `lo..hi`, both ends fixed on the option they stand on, the interior
  /// couplers open, and every complete run of options it reaches tested by
  /// the local rip-up-and-reroute of `tryRun` before it is taken. Built as
  /// `solveChainAStar` is, with what differs said here:
  ///
  /// - **No `edgeMemo_` and no reorder.** `edgeMemoKey` knows nothing of the
  ///   fence and the phase-3 entries are stale in phase 4, so `edgeMemo_` is
  ///   not read; a step's way comes from its own search or from the call's
  ///   `SegmentMemo`, which keys it by the whole corridor. The end-pair
  ///   reorder would fire at the segment ends and break the cost order the
  ///   enumeration rests on — a `redone` price makes a prefix cheaper than
  ///   its parent, and then the runs no longer pop cheapest first.
  /// - **The fence.** `prefixFence_` holds the segment's own laid ways, every
  ///   one of them; `fenceCommittedEdges` holds the rest of the chip, with
  ///   the segment's own edges left open (`openChain_`) and, under
  ///   `researchSeesChains`, every other chain's edge and this chain's
  ///   terminal and later edges fenced whatever the insertion's switches
  ///   say. `soloChain_` stays at NO_OWNER.
  /// - **What the step sees.** `edgeCost` stands the two ends on the options
  ///   asked about, and `sourceOf`/`targetOf`, `runsOfEdge`, `portOfOption`
  ///   and the chain's own leads and pads in `corridorOfEdge` read `chosen`,
  ///   so the candidate's lead and pad are fenced. Three things are the
  ///   **applied** state: the interior couplers' committed pads in `bodies_`
  ///   are hidden for the search (`setPads`); the resonator tail stamped
  ///   along `wayOfResonator` is the applied, cut-back way, harmless at
  ///   copper width; and an interior coupler that is not this step's end
  ///   stands on its committed option — the approximation the insertion
  ///   lives with too.
  /// - **The current run** is enumerated like any other and refused without
  ///   a test when its edges come out on the ways they already hold; with
  ///   other ways it is a candidate like any other — the same couplers with
  ///   the edges laid against the frozen chip, which is what a terminal pair
  ///   that crosses its neighbour chain needs.
  ///
  /// `badNow` is the chip's figure before the segment and is moved to the
  /// accepted candidate's. Leaves the accepted run applied, or the chip as
  /// it was.
  [[nodiscard]] Research
  researchSegment(std::vector<Wire>& wires,
                  const std::vector<std::uint32_t>& members,
                  const std::vector<std::uint32_t>& every, const Pass& pass,
                  const Segment& segment, const std::uint32_t trialsLeft,
                  std::uint32_t& badNow) {
    Research out;
    const auto chain = segment.chain;
    auto& points = chains_[chain];
    const auto lo = segment.lo;
    const auto hi = segment.hi;
    const auto layers = hi - lo + 1;
    const auto edges = hi - lo;
    const auto name = segmentName(wires, segment);

    // The run the segment stands on, by option index per layer.
    std::vector<std::uint32_t> current(layers, 0);
    std::vector<std::uint32_t> interior;
    for (std::size_t k = 0; k < layers; ++k) {
      const auto& point = points[lo + k];
      if (point.fixed) {
        continue;
      }
      current[k] = static_cast<std::uint32_t>(couplers_[point.coupler].chosen);
      if (k > 0 && k + 1 < layers) {
        interior.push_back(point.coupler);
      }
    }

    // The options each layer offers: the ends their current option, the
    // interior everything the guard leaves open.
    const auto standing = standingCellsBut(wires, chain, lo, hi);
    std::vector<std::vector<std::uint32_t>> open(layers);
    for (std::size_t k = 0; k < layers; ++k) {
      if (k == 0 || k + 1 == layers) {
        open[k] = {current[k]};
      } else {
        open[k] = openOptionsOf(chain, lo + k, &standing);
      }
    }

    // The bounds, as the insertion builds them, indices shifted by `lo`.
    std::vector<std::vector<std::uint64_t>> turns(edges);
    std::uint64_t pairs = 0;
    for (std::size_t e = 0; e < edges; ++e) {
      const auto n = open[e].size();
      const auto m = open[e + 1].size();
      turns[e].resize(n * m);
      for (std::size_t c = 0; c < n; ++c) {
        const auto leaving = portOfOption(chain, lo + e, open[e][c], true);
        for (std::size_t d = 0; d < m; ++d) {
          const auto arriving =
              portOfOption(chain, lo + e + 1, open[e + 1][d], false);
          turns[e][(c * m) + d] = pairBound(
              leaving, arriving, forcedRunOf(chain, lo + e, open[e][c]),
              forcedRunOf(chain, lo + e + 1, open[e + 1][d]));
        }
      }
      pairs += n * m;
    }

    routing::ChainProblem problem;
    for (const auto& choices : open) {
      problem.width.push_back(static_cast<std::uint32_t>(choices.size()));
    }
    problem.bound = [&](const std::size_t e, const std::uint32_t i,
                        const std::uint32_t j) {
      return weighted(
          turns[e][(static_cast<std::size_t>(i) * open[e + 1].size()) + j],
          edgeWeight(chain, lo + e));
    };
    // How far the search has come, every `CHAIN_PROGRESS_STEPS` priced steps.
    const auto began = std::chrono::steady_clock::now();
    problem.progressEvery = CHAIN_PROGRESS_STEPS;
    problem.progress = [&](const routing::ChainSolution& sofar,
                           const std::size_t queued) {
      say(std::format(
          "chain progress: {}: {} steps priced, {} prefixes expanded, {} "
          "left unpriced, {} open, deepest layer {} of {}, {} runs reached, "
          "{} refused; {:.1f}s",
          name, sofar.routed, sofar.expansions, sofar.pruned, queued,
          sofar.reached, layers - 1, out.runs, sofar.rejected,
          std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                        began)
              .count()));
    };
    // One more than the tests left: the current run may be refused once
    // without a test, and that refusal counts in the search's own tally.
    problem.maxAccepts = trialsLeft + 1;

    std::map<std::vector<std::uint32_t>, Laid> laid;
    const auto runText = [&](const std::span<const std::uint32_t> run) {
      std::string options;
      for (std::size_t k = 0; k < run.size(); ++k) {
        options +=
            std::format("{}{}", k == 0 ? "" : " ",
                        points[lo + k].fixed ? std::string("launcher")
                                             : std::to_string(open[k][run[k]]));
      }
      return options;
    };
    problem.found = [&](const std::span<const std::uint32_t> run,
                        const std::uint64_t cost) {
      ++out.runs;
      tell(std::format("targeted repair:   {}: every edge drawn on options {} "
                       "-> cost {}",
                       name, runText(run), cost));
    };
    problem.step = [&](const std::span<const std::uint32_t> prefix,
                       const std::uint32_t j, std::int64_t& redone) {
      redone = 0;
      const auto at = prefix.size() - 1;
      std::vector<std::uint32_t> key(prefix.begin(), prefix.end());
      // Every way the prefix has laid is fenced, no exemption: the terminal
      // exemptions of the insertion's fence are chain-relative, and a
      // segment is not a chain.
      std::vector<const Path*> fence;
      fence.reserve(at);
      for (std::size_t e = 0; e < at; ++e) {
        const auto found = laid.find(std::vector<std::uint32_t>(
            key.begin(), key.begin() + static_cast<std::ptrdiff_t>(e + 2)));
        if (found != laid.end() && !found->second.way.empty()) {
          fence.push_back(&found->second.way);
        }
      }
      // The option the step ends on keeps a clearance from every way laid
      // before it (`optionClearOf`).
      if (const auto& next = points[lo + at + 1];
          !next.fixed &&
          !optionClearOf(couplers_[next.coupler].options[open[at + 1][j]],
                         fence)) {
        return routing::TRELLIS_UNREACHABLE;
      }
      Path way;
      const auto real =
          edgeCost(wires, chain, lo + at, open[at][prefix[at]], open[at + 1][j],
                   0, fence.empty() ? nullptr : &fence, &way, false);
      if (real == routing::TRELLIS_UNREACHABLE) {
        return real;
      }
      // And the way keeps a clearance from the options of the couplers
      // before the one it leaves.
      for (std::size_t k = 0; k < at; ++k) {
        const auto& point = points[lo + k];
        if (!point.fixed &&
            !optionClearOf(couplers_[point.coupler].options[open[k][prefix[k]]],
                           {&way})) {
          return routing::TRELLIS_UNREACHABLE;
        }
      }
      key.push_back(j);
      laid[std::move(key)] = Laid{
          .way = std::move(way), .before = {}, .price = real, .beforePrice = 0};
      return weighted(real, edgeWeight(chain, lo + at));
    };

    // The state the segment starts from, with the interior pads hidden: a
    // refused candidate is put back to exactly the search's own ground.
    setPads(interior, false);
    const auto taken = snapshot(wires);
    fenceEverything_ = true;
    openChain_ = chain;
    openLo_ = lo;
    openHi_ = hi;
    std::uint32_t trials = 0;

    problem.accept = [&](const std::span<const std::uint32_t> run,
                         const std::uint64_t cost) {
      const auto ways = waysOfRun(laid, run);
      if (segmentCollect_ != nullptr) {
        std::vector<std::size_t> options(layers, 0);
        for (std::size_t k = 0; k < layers; ++k) {
          options[k] = open[k][run[k]];
        }
        static_cast<void>((*segmentCollect_)(options, ways));
        return false;
      }
      // The current run on the ways it already holds: nothing to test.
      bool same = true;
      for (std::size_t k = 0; same && k < layers; ++k) {
        same = open[k][run[k]] == current[k];
      }
      for (std::size_t e = 0; same && e < edges; ++e) {
        const auto& held = wires[edgeWireOf_[chain][lo + e]].way;
        same = held.size() == ways[e].size() &&
               std::ranges::equal(held, ways[e],
                                  [](const PathPoint& a, const PathPoint& b) {
                                    return a.samePlace(b);
                                  });
      }
      if (same && !agreement_.on) {
        ++out.skippedCurrent;
        tell(std::format("targeted repair:   {} run {} (cost {}): the current "
                         "run on the ways it holds, not tested",
                         name, runText(run), cost));
        return false;
      }
      if (trials >= trialsLeft) {
        out.outOfTrials = true;
        return false;
      }
      ++trials;
      if (agreement_.on) {
        agreeRun(wires, members, every, pass, segment, open, run, ways, cost,
                 taken);
        return false;
      }
      const bool kept = tryRun(wires, members, every, pass, segment, open, run,
                               ways, cost, taken, badNow);
      if (!kept) {
        ++out.rejected;
        setPads(interior, false);
      }
      return kept;
    };

    const auto answer = routing::solveChainAStar(problem);

    openChain_ = NO_OWNER;
    openLo_ = 0;
    openHi_ = 0;
    fenceEverything_ = false;
    prefixFence_ = nullptr;
    // The pads back — the accepted run's, or the ones the chip stood on.
    setPads(interior, true);
    foreignRoomStale_ = true;

    out.trials = trials;
    out.routed = answer.routed;
    out.accepted = answer.solved;
    out.outOfTrials = out.outOfTrials || answer.outOfTrials;
    out.exhausted = !answer.solved && answer.optimal;
    say(std::format(
        "targeted repair: {} — {} after {} run{} reached, {} tested, {} "
        "refused, {} the current run; {} edge searches over {} option "
        "pair{}{}",
        name,
        answer.solved     ? std::format("ACCEPTED options {} at cost {}",
                                        runText(answer.chosen), answer.cost)
        : out.outOfTrials ? "nothing accepted, out of legality tests"
        : out.runs > 0    ? "nothing accepted, every run refused"
                          : "nothing accepted, no run of options joins it",
        out.runs, out.runs == 1 ? "" : "s", trials, out.rejected,
        out.skippedCurrent, answer.routed, pairs, pairs == 1 ? "" : "s",
        answer.expansions == 0
            ? std::string{}
            : std::format(", {} prefixes expanded", answer.expansions)));
    return out;
  }

  /// Put a segment on a run of options: the interior couplers on their
  /// options, the segment's edges on `ways` with their objectives and stubs
  /// again, and the crossing rule rebuilt (`tryRun`, step 1).
  void applyRun(std::vector<Wire>& wires, const Segment& segment,
                const std::vector<std::vector<std::uint32_t>>& open,
                const std::span<const std::uint32_t> run,
                const std::vector<Path>& ways) {
    const auto chain = segment.chain;
    const auto& points = chains_[chain];
    const auto lo = segment.lo;
    const auto edges = segment.hi - segment.lo;
    for (std::size_t k = 1; k + 1 < run.size(); ++k) {
      if (!points[lo + k].fixed) {
        applyOption(wires, points[lo + k].coupler, open[k][run[k]]);
      }
    }
    for (std::size_t e = 0; e < edges; ++e) {
      auto& wire = wires[edgeWireOf_[chain][lo + e]];
      const auto& edge = edges_[wire.edge];
      lift(wire);
      wire.way = ways[e];
      wire.objective.source = sourceOf(points[edge.from]);
      wire.objective.target = targetOf(points[edge.to]);
      wire.fixed.clear();
      const auto [startRun, endRun] = runsOfEdge(edge);
      wire.startStub = startRun;
      wire.endStub = endRun;
      wire.drawn = !wire.way.empty();
      wire.routed = wire.drawn;
      wire.tooShort = false;
      wire.tooLong = false;
      place(wire);
    }
    rebuildCrossingRule(wires);
  }

  /// The wires a run of options for a segment puts at stake, by key
  /// (`tryRun`, step 2). With `enclosedOnly` the feedline and the wires it
  /// encloses (user, 2026-10-19): the ring from the segment's starting edge
  /// to its ending edge, every plain wire and resonator between them, and
  /// the wires the fails named; not the members near a new edge way or
  /// across one, and not the ring members beyond.
  [[nodiscard]] std::set<std::uint32_t>
  windowOfRun(const std::vector<Wire>& wires,
              const std::vector<std::uint32_t>& members, const Segment& segment,
              const std::span<const std::uint32_t> run,
              const bool enclosedOnly = false) {
    const auto chain = segment.chain;
    const auto& points = chains_[chain];
    const auto lo = segment.lo;
    const auto edges = segment.hi - segment.lo;
    std::set<std::uint32_t> window;
    for (std::size_t e = 0; e < edges; ++e) {
      if (lo + e < bridgers_[chain].size()) {
        for (const auto key : bridgers_[chain][lo + e]) {
          window.insert(key);
        }
      }
      const auto key = edgeWireOf_[chain][lo + e];
      if (edges_[wires[key].edge].terminal) {
        window.insert(key);
      }
    }
    for (std::size_t k = 0; k < run.size(); ++k) {
      if (!points[lo + k].fixed) {
        window.insert(couplers_[points[lo + k].coupler].wire);
      }
    }
    for (const auto key : segment.fails) {
      window.insert(key);
    }
    if (enclosedOnly) {
      std::set<std::uint32_t> enclosed(segment.fails.begin(),
                                       segment.fails.end());
      const auto total = members.size();
      std::vector<std::size_t> at;
      for (std::size_t e = 0; e < edges; ++e) {
        const auto found =
            std::ranges::find(members, edgeWireOf_[chain][lo + e]);
        if (found != members.end()) {
          at.push_back(static_cast<std::size_t>(found - members.begin()));
        }
      }
      if (at.empty()) {
        return window;
      }
      // The ring from the first edge to the last one way round or the
      // other: the way that passes every edge of the segment, the shorter
      // when both do.
      const auto span = [&](const std::size_t from, const std::size_t to) {
        return (to + total - from) % total;
      };
      const auto passes = [&](const std::size_t from, const std::size_t to) {
        return std::ranges::all_of(
            at, [&](const auto k) { return span(from, k) <= span(from, to); });
      };
      auto from = at.front();
      auto to = at.back();
      if (!passes(from, to) ||
          (passes(to, from) && span(to, from) < span(from, to))) {
        std::swap(from, to);
      }
      for (std::size_t step = 0; step <= span(from, to); ++step) {
        enclosed.insert(members[(from + step) % total]);
      }
      return enclosed;
    }
    {
      const std::unordered_set<std::uint32_t> isMember(members.begin(),
                                                       members.end());
      const auto& stencil = stencilFor(tuning_.clearance);
      const auto width = static_cast<std::int64_t>(scene_.router.width);
      for (std::size_t e = 0; e < edges; ++e) {
        const auto& way = wires[edgeWireOf_[chain][lo + e]].way;
        alongDisc(way, stencil,
                  [&](const std::int64_t x, const std::int64_t y) {
                    const auto owner =
                        field_.owner(static_cast<std::size_t>((y * width) + x));
                    if (owner != NO_OWNER && owner < wires.size() &&
                        isMember.contains(owner)) {
                      window.insert(owner);
                    }
                  });
        for (const auto member : members) {
          if (!window.contains(member) && !wires[member].way.empty() &&
              waysCross(way, wires[member].way)) {
            window.insert(member);
          }
        }
      }
    }
    {
      const auto k = tuning_.researchRingMembers;
      const auto total = members.size();
      std::vector<std::uint32_t> more;
      for (std::size_t at = 0; at < total; ++at) {
        if (!window.contains(members[at])) {
          continue;
        }
        for (std::size_t step = 1; step <= k && step < total; ++step) {
          more.push_back(members[(at + step) % total]);
          more.push_back(members[(at + total - step) % total]);
        }
      }
      window.insert(more.begin(), more.end());
    }
    return window;
  }

  /// What the local rip-up-and-reroute of a window came to.
  struct WindowPass {
    /// The window's wires and the members whose way changed.
    std::uint32_t flagged = 0;
    std::uint32_t moved = 0;
    /// The wires the pass touched — the window, and what the relaxation
    /// released and redrew — that end unrouted, open or crossing.
    std::uint32_t unrouted = 0;
    std::uint32_t open = 0;
    std::uint32_t crossing = 0;
    std::set<std::uint32_t> failing;
    std::string named;
    /// The wires that found no way in the last round of the pass.
    std::string noWay;
    Fails after;

    [[nodiscard]] bool clean() const { return unrouted + open + crossing == 0; }
  };

  /// Rip up the wires of `window` and route them again in one silenced
  /// sweep over the whole ring (`tryRun`, steps 3 and 4): every other
  /// member is held where it is. With `ignoreOutside` the sweep runs over
  /// the window and the feedline edges alone, in ring order, and every
  /// other member is off the canvas for its length: no obstacle, never
  /// released by the relaxation, and not judged.
  [[nodiscard]] WindowPass rerouteWindow(
      std::vector<Wire>& wires, const std::vector<std::uint32_t>& members,
      const std::vector<std::uint32_t>& every, const Pass& pass,
      const std::set<std::uint32_t>& window, const bool ignoreOutside = false) {
    WindowPass out;
    std::vector<std::uint32_t> ring;
    std::vector<std::uint32_t> ignored;
    for (const auto member : members) {
      if (!ignoreOutside || window.contains(member) || wires[member].feedline) {
        ring.push_back(member);
      } else {
        lift(wires[member]);
        ignored.push_back(member);
      }
    }
    std::vector<std::uint32_t> flagged;
    std::vector<Path> had;
    had.reserve(ring.size());
    for (const auto member : ring) {
      auto& wire = wires[member];
      had.push_back(wire.way);
      if (window.contains(member)) {
        wire.routed = false;
        wire.ripped = true;
        flagged.push_back(member);
      } else {
        wire.routed = true;
      }
    }

    // The local pass, silenced unless asked to speak.
    Pass local = pass;
    local.rounds = tuning_.researchRounds;
    local.name = "targeted repair";
    local.onlyUnsettled = true;
    const auto loud = std::exchange(report_, Report{});
    sweep(wires, ring, local);
    out.after = failsOf(wires, ignoreOutside ? ring : every);
    report_ = loud;
    // Which window wires found no way in the last round of the local pass
    // and kept what they had: a wire that fails the window because it was
    // never redrawn reads differently from one redrawn into a fail.
    if (!rounds_.empty()) {
      for (const auto& id : rounds_.back().failed) {
        out.noWay += (out.noWay.empty() ? "" : ", ") + id;
      }
    }

    // What it came to among the wires it touched.
    std::set<std::uint32_t> touched(flagged.begin(), flagged.end());
    for (std::size_t at = 0; at < ring.size(); ++at) {
      const auto& now = wires[ring[at]].way;
      const bool same =
          now.size() == had[at].size() &&
          std::ranges::equal(now, had[at],
                             [](const PathPoint& a, const PathPoint& b) {
                               return a.samePlace(b);
                             });
      if (!same) {
        ++out.moved;
        touched.insert(ring[at]);
      }
    }
    const auto count = [&](const std::vector<std::uint32_t>& keys,
                           std::uint32_t& figure, const char* what) {
      for (const auto key : keys) {
        if (touched.contains(key)) {
          ++figure;
          out.failing.insert(key);
          out.named += std::format("{}{} {}", out.named.empty() ? "" : ", ",
                                   wireId(wires[key]), what);
        }
      }
    };
    count(out.after.unroutedKeys, out.unrouted, "unrouted");
    count(out.after.openKeys, out.open, "open");
    count(out.after.crossingKeys, out.crossing, "crossing");
    out.flagged = static_cast<std::uint32_t>(flagged.size());
    for (const auto key : ignored) {
      if (wires[key].drawn) {
        place(wires[key]);
      }
    }
    return out;
  }

  /// **The legality test** of one candidate run of options for a segment: a
  /// local rip-up-and-reroute, judged on the figure.
  ///
  /// 1. The interior couplers take the run's options (`applyOption`, as
  ///    `turnCoupler` does, but for every coupler before any edge is laid);
  ///    the segment's edges take the ways the search found, objectives from
  ///    `sourceOf`/`targetOf`, stubs **recomputed** from `runsOfEdge` — they
  ///    are set once at creation and `turnCoupler` never recomputed them, a
  ///    latent bug this does not inherit; the crossing rule is rebuilt.
  /// 2. The window: the plain wires bridged to a segment edge
  ///    (`bridgers_`), the resonators of its waypoints, a terminal edge among
  ///    its edges, every wire the fails named, every member whose way lies
  ///    within the clearance of a new edge way or crosses one — which
  ///    `isLegal` cannot see, because `conflictsIn` exempts plain wire
  ///    against feedline both ways — and `researchK` ring members beyond
  ///    each of those. Flagged `routed = false, ripped = true`; every other
  ///    member is held `routed = true`, the fails outside the window
  ///    included, so the window is what the pass redraws and nothing else.
  /// 3. One silenced `sweep` over the **whole ring** under
  ///    `Pass::onlyUnsettled`, `researchRounds` rounds, so the fences and the
  ///    lane see the true ring neighbours. Terminal edges in the window are
  ///    drawn by the sweep's `search`, as the pass draws them.
  /// 4. The verdict from `failsOf` over every wire, not from the round
  ///    tallies: the window is clean when no wire the pass touched — the
  ///    window and whatever the relaxation released and redrew — is
  ///    unrouted, open or crossing; the run is kept when the window is clean
  ///    and `bad()` over every wire is strictly below what it was before the
  ///    segment. Lengths are no criterion (user, 2026-10-04).
  /// 5. Refused: `restore` to the segment's start — the whole wire list,
  ///    `bodies_`, `chosen`, the anchors, the field rebuilt, the crossing
  ///    rule; not `chainEdgePaths_`, `edgeMemo_`, `jogsUnlocked`. Kept: the
  ///    edges' ways go into `chainEdgePaths_` and `badNow` moves.
  [[nodiscard]] bool tryRun(std::vector<Wire>& wires,
                            const std::vector<std::uint32_t>& members,
                            const std::vector<std::uint32_t>& every,
                            const Pass& pass, const Segment& segment,
                            const std::vector<std::vector<std::uint32_t>>& open,
                            const std::span<const std::uint32_t> run,
                            const std::vector<Path>& ways,
                            const std::uint64_t cost, const Snapshot& taken,
                            std::uint32_t& badNow) {
    const auto chain = segment.chain;
    const auto& points = chains_[chain];
    const auto lo = segment.lo;
    const auto hi = segment.hi;
    const auto edges = hi - lo;
    std::string options;
    for (std::size_t k = 0; k < run.size(); ++k) {
      options +=
          std::format("{}{}", k == 0 ? "" : " ",
                      points[lo + k].fixed ? std::string("launcher")
                                           : std::to_string(open[k][run[k]]));
    }
    const auto label = std::format("targeted repair:   {} run {} (cost {})",
                                   segmentName(wires, segment), options, cost);

    // 1. Apply.
    applyRun(wires, segment, open, run, ways);

    // 2. The window.
    const auto window = windowOfRun(wires, members, segment, run);
    // 3. The local pass, and 4. what it came to in the window.
    const auto local = rerouteWindow(wires, members, every, pass, window);
    const auto& after = local.after;
    const bool clean = local.clean();
    const bool better = after.bad() < badNow;
    tell(std::format(
        "{}: window {} wire{}, {} moved{}; in it {} unrouted, {} "
        "open, {} crossing{}; bad {} -> {} ({} unrouted, {} open, "
        "{} crossing): {}",
        label, local.flagged, local.flagged == 1 ? "" : "s", local.moved,
        local.noWay.empty()
            ? std::string{}
            : ", no way for " + local.noWay + " in the last round",
        local.unrouted, local.open, local.crossing,
        local.named.empty() ? std::string{} : " (" + local.named + ")", badNow,
        after.bad(), after.unrouted, after.open, after.crossing,
        clean && better ? "ACCEPTED"
        : clean         ? "rejected, the chip is not better"
        : better        ? "rejected, the window is not clean"
                        : "rejected"));
    if (clean && better) {
      badNow = after.bad();
      for (std::size_t e = 0; e < edges; ++e) {
        chainEdgePaths_[chain][lo + e] = wires[edgeWireOf_[chain][lo + e]].way;
      }
      return true;
    }

    // 5. Refused.
    restore(wires, taken);
    foreignRoomStale_ = true;
    return false;
  }

  /// The wires of one feedline for the comparison. A feedline with a
  /// launcher at both ends: the ring from its starting edge to its ending
  /// edge, every plain wire and resonator between them (`windowOfRun` with
  /// `enclosedOnly`). The ring holds the terminal edges alone, so a
  /// feedline with a reflector end has one edge in it: its wires are the
  /// resonators of its couplers, the wires that bridge one of its edges,
  /// and its terminal edge.
  [[nodiscard]] std::set<std::uint32_t>
  feedlineWindow(const std::vector<Wire>& wires,
                 const std::vector<std::uint32_t>& members,
                 const std::uint32_t chain) {
    const auto& points = chains_[chain];
    if (points.size() < 2) {
      return {};
    }
    if (points.front().fixed && points.back().fixed) {
      return windowOfRun(wires, members,
                         {.chain = chain,
                          .lo = 0,
                          .hi = points.size() - 1,
                          .fails = {},
                          .because = {}},
                         {}, true);
    }
    std::set<std::uint32_t> window;
    for (std::size_t at = 0; at + 1 < points.size(); ++at) {
      if (at < bridgers_[chain].size()) {
        window.insert(bridgers_[chain][at].begin(), bridgers_[chain][at].end());
      }
      const auto key = edgeWireOf_[chain][at];
      if (edges_[wires[key].edge].terminal) {
        window.insert(key);
      }
    }
    for (const auto& point : points) {
      if (!point.fixed) {
        window.insert(couplers_[point.coupler].wire);
      }
    }
    return window;
  }

  /// The slot checks of the chip as it stands, on the graph the insertion
  /// builds (the terminal edges as wires when `terminalDemands` says so,
  /// the bottlenecks found as it finds them): once with every wire, and
  /// once per feedline with its wires alone (user, 2026-10-19), so that a
  /// feedline's check routes only its own wires. The terminal edges of the
  /// other feedlines stay in the graph: they name the mouths between the
  /// chains, and without them a feedline's own terminal edge lost its way
  /// (17q, f6 and f0). Nothing when the check cannot read the graph.
  struct SlotChecks {
    SlotCheckResult whole;
    std::vector<std::optional<SlotCheckResult>> feedlines;
  };
  [[nodiscard]] std::optional<SlotChecks>
  slotCheckNow(const std::vector<Wire>& wires,
               const std::vector<std::set<std::string>>& windows,
               const std::string& label) {
    auto capacity = capacityPass(wires, edges_, nullptr, NO_OWNER, true, true);
    std::optional<SlotChecks> out;
    try {
      auto graph = slotGraphOfPass(wires, capacity);
      SlotChecks checks{.whole = slotCheck(graph), .feedlines = {}};
      const auto every = std::move(graph.wires);
      for (const auto& window : windows) {
        if (window.empty()) {
          checks.feedlines.emplace_back();
          continue;
        }
        graph.wires.clear();
        for (const auto& wire : every) {
          if (window.contains(wire.name) || wire.terminal) {
            graph.wires.push_back(wire);
          }
        }
        checks.feedlines.emplace_back(slotCheck(graph));
      }
      out = std::move(checks);
    } catch (const std::invalid_argument& error) {
      tell(std::format("{}: the slot check failed ({})", label, error.what()));
    }
    giveBack(capacity);
    return out;
  }

  /// The wires of a window that a slot check loses or crowds, by name.
  [[nodiscard]] static std::vector<std::string>
  lostIn(const SlotCheckResult& result, const std::set<std::string>& window) {
    std::set<std::string> named(result.lost.begin(), result.lost.end());
    named.insert(result.crowded.begin(), result.crowded.end());
    std::vector<std::string> out;
    for (const auto& name : named) {
      if (window.contains(name)) {
        out.push_back(name);
      }
    }
    return out;
  }

  /// The local rip-up-and-reroute of one feedline's window, every wire
  /// outside it ignored (`rerouteWindow` with `ignoreOutside`).
  [[nodiscard]] FeedlineRip
  ripFeedline(std::vector<Wire>& wires,
              const std::vector<std::uint32_t>& members,
              const std::vector<std::uint32_t>& every, const Pass& pass,
              const std::set<std::uint32_t>& window) {
    const auto began = std::chrono::steady_clock::now();
    const auto local = rerouteWindow(wires, members, every, pass, window, true);
    FeedlineRip out;
    out.clean = local.clean();
    out.failing = local.named;
    for (const auto key : local.after.openKeys) {
      if (!local.failing.contains(key)) {
        continue;
      }
      auto& wire = wires[key];
      lift(wire);
      out.blocked += std::format("{}{} is open in the room of {}",
                                 out.blocked.empty() ? "" : "; ", wireId(wire),
                                 whoBlocks(wire, wires));
      place(wire);
    }
    for (std::size_t round = 0; round < rounds_.size(); ++round) {
      const auto& record = rounds_[round];
      out.rounds.push_back({{"round", round},
                            {"forward", record.forward},
                            {"normal", record.normal},
                            {"relaxed", record.relaxed},
                            {"failed", record.failed}});
    }
    out.seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - began)
            .count();
    return out;
  }

  /// **One run of the comparison** of the local slot check with the local
  /// rip-up-and-reroute, feedline by feedline (user, 2026-10-19). The run
  /// is applied as `tryRun` applies it. Then, for every feedline, on its
  /// wires (`feedlineWindow`):
  ///
  /// 1. The slot check of the chip as it now stands with the feedline's
  ///    wires alone: SAT for the feedline when none of them is lost or
  ///    crowded. The check with every wire is kept beside it.
  /// 2. The local rip-up-and-reroute of its wires alone, every other wire
  ///    ignored: clean when none of them ends unrouted, open or crossing.
  ///    It runs again for the feedline of the segment, for every feedline
  ///    that shares a wire with it and for every feedline whose verdict
  ///    differs from the chip as it began; any other feedline keeps the
  ///    rip-up of the chip as it began.
  ///
  /// One `[Local Agreement]` line says both for every feedline; the chip is
  /// put back.
  void agreeRun(std::vector<Wire>& wires,
                const std::vector<std::uint32_t>& members,
                const std::vector<std::uint32_t>& every, const Pass& pass,
                const Segment& segment,
                const std::vector<std::vector<std::uint32_t>>& open,
                const std::span<const std::uint32_t> run,
                const std::vector<Path>& ways, const std::uint64_t cost,
                const Snapshot& taken) {
    const auto& points = chains_[segment.chain];
    std::string options;
    for (std::size_t k = 0; k < run.size(); ++k) {
      options += std::format("{}{}", k == 0 ? "" : " ",
                             points[segment.lo + k].fixed
                                 ? std::string("launcher")
                                 : std::to_string(open[k][run[k]]));
    }
    const auto label = std::format("local agreement: {} run {} (cost {})",
                                   segmentName(wires, segment), options, cost);
    applyRun(wires, segment, open, run, ways);
    const auto chains = static_cast<std::uint32_t>(chains_.size());
    const auto& windows = agreement_.windows;

    // 1. The slot checks: every wire, and every feedline's wires alone.
    const auto began = std::chrono::steady_clock::now();
    const auto checks = slotCheckNow(wires, agreement_.windowNames, label);
    const auto slotSeconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - began)
            .count();
    agreement_.slotSeconds += slotSeconds;
    if (!checks.has_value()) {
      ++agreement_.unjudged;
      restore(wires, taken);
      foreignRoomStale_ = true;
      return;
    }
    const auto& whole = checks->whole;
    std::vector<std::vector<std::string>> lost(chains);
    std::vector<std::vector<std::string>> lostWhole(chains);
    for (std::uint32_t chain = 0; chain < chains; ++chain) {
      if (checks->feedlines[chain].has_value()) {
        lost[chain] =
            lostIn(*checks->feedlines[chain], agreement_.windowNames[chain]);
      }
      lostWhole[chain] = lostIn(whole, agreement_.windowNames[chain]);
    }

    // 2. The rip-up of every feedline the run may touch, the segment's own
    // last, so that the chip it leaves is the one the page draws.
    std::vector<std::uint32_t> again;
    for (std::uint32_t chain = 0; chain < chains; ++chain) {
      if (chain == segment.chain || windows[chain].empty()) {
        continue;
      }
      const bool shares =
          std::ranges::any_of(windows[chain], [&](const auto key) {
            return windows[segment.chain].contains(key);
          });
      if (shares || lost[chain].empty() != agreement_.baseSat[chain]) {
        again.push_back(chain);
      }
    }
    again.push_back(segment.chain);
    std::vector<std::optional<FeedlineRip>> rips(chains);
    for (std::size_t at = 0; at < again.size(); ++at) {
      if (at > 0) {
        restore(wires, taken);
        applyRun(wires, segment, open, run, ways);
      }
      const auto chain = again[at];
      rips[chain] = ripFeedline(wires, members, every, pass, windows[chain]);
      agreement_.rerouteSeconds += rips[chain]->seconds;
    }

    // What each feedline came to, and the count of the comparison.
    using nlohmann::json;
    json feedlines = json::array();
    std::string said;
    std::uint32_t disagree = 0;
    for (std::uint32_t chain = 0; chain < chains; ++chain) {
      if (windows[chain].empty()) {
        continue;
      }
      const bool reused = !rips[chain].has_value();
      const auto& rip = reused ? agreement_.baseRip[chain] : *rips[chain];
      const bool sat = lost[chain].empty();
      ++agreement_.whole.at((sat ? 0U : 2U) + (rip.clean ? 0U : 1U));
      disagree += sat == rip.clean ? 0U : 1U;
      std::string lostText;
      for (const auto& name : lost[chain]) {
        lostText += (lostText.empty() ? "" : ", ") + name;
      }
      said += std::format(
          "; feedline {} {}{} rip-up {}{}{}", chain, sat ? "SAT" : "UNSAT",
          lostText.empty() ? std::string{} : " (" + lostText + ")",
          rip.clean ? "clean" : "fails",
          rip.failing.empty() ? std::string{} : " (" + rip.failing + ")",
          reused ? " as begun" : "");
      json names = json::array();
      for (const auto& name : agreement_.windowNames[chain]) {
        names.push_back(name);
      }
      feedlines.push_back({{"chain", chain},
                           {"window", std::move(names)},
                           {"sat", sat},
                           {"lost", lost[chain]},
                           {"wholeSat", lostWhole[chain].empty()},
                           {"wholeLost", lostWhole[chain]},
                           {"clean", rip.clean},
                           {"failing", rip.failing},
                           {"blocked", rip.blocked},
                           {"rounds", rip.rounds},
                           {"reused", reused},
                           {"seconds", rip.seconds}});
    }
    say(std::format("{}: {}{}; {:.2f}s in the slot check", label,
                    disagree == 0 ? std::string("AGREE")
                                  : std::format("DISAGREE on {}", disagree),
                    said, slotSeconds));

    {
      const auto id = std::format("c{}-w{}-r{}", segment.chain, segment.lo,
                                  agreement_.written++);
      json pads = json::array();
      for (std::uint32_t index = 0; index < couplers_.size(); ++index) {
        const auto& coupler = couplers_[index];
        if (coupler.options.empty()) {
          continue;
        }
        pads.push_back({{"index", index},
                        {"resonator", wireId(wires[coupler.wire])},
                        {"pad", padCorners(coupler.options[coupler.chosen])}});
      }
      // `routedJson` counts the fails again; its lines stay out of the log.
      const auto loud = std::exchange(report_, Report{});
      auto routed = json::parse(routedJson(wires));
      report_ = loud;
      // The run as moves of couplers onto options, so that a benchmark can
      // put the chip on the same state again.
      json moves = json::array();
      for (std::size_t k = 0; k < run.size(); ++k) {
        if (!points[segment.lo + k].fixed) {
          moves.push_back({{"coupler", points[segment.lo + k].coupler},
                           {"option", open[k][run[k]]}});
        }
      }
      json out{{"id", id},
               {"chain", segment.chain},
               {"lo", segment.lo},
               {"hi", segment.hi},
               {"segment", segmentName(wires, segment)},
               {"options", options},
               {"moves", std::move(moves)},
               {"cost", cost},
               {"slot",
                {{"verdict", whole.verdict},
                 {"lost", whole.lost},
                 {"crowded", whole.crowded},
                 {"over", whole.over}}},
               {"feedlines", std::move(feedlines)},
               {"disagree", disagree},
               {"seconds", {{"slot", slotSeconds}}},
               {"routed", std::move(routed)},
               {"pads", std::move(pads)}};
      agreement_.records.push_back(std::move(out));
    }

    restore(wires, taken);
    foreignRoomStale_ = true;
  }

  /// **The comparison** of the local slot check with the local
  /// rip-up-and-reroute, in place of the repair (`localAgreementRuns`):
  /// every segment of `localAgreementSpan` edges of every chain, its ends
  /// fixed and the couplers between them open, is searched as the targeted
  /// repair searches it, and up to `localAgreementRuns` runs it reaches are
  /// judged feedline by feedline (`agreeRun`) and put back. First the chip
  /// as it began is judged the same way. The chip ends as it began.
  ///
  /// @returns What the comparison counted, as JSON: `segments`, `classes`
  /// (the four classes over every feedline judged), `unjudged`, the seconds
  /// in the slot check and in the rerouting, `begun` (what the chip as it
  /// began came to, per feedline) and `runs` (one record per run judged).
  [[nodiscard]] nlohmann::json
  compareLocally(std::vector<Wire>& wires,
                 const std::vector<std::uint32_t>& members,
                 const std::vector<std::uint32_t>& every, const Pass& pass,
                 const std::uint32_t runs, const std::size_t spanAsked) {
    const auto began = std::chrono::steady_clock::now();
    say(std::format("local agreement: {} run{} a segment of {} edges", runs,
                    runs == 1 ? "" : "s", spanAsked));
    agreement_ = Agreement{.on = true};
    const auto chains = static_cast<std::uint32_t>(chains_.size());
    // The chip as it began, feedline by feedline.
    agreement_.windows.resize(chains);
    agreement_.windowNames.resize(chains);
    agreement_.baseSat.assign(chains, true);
    agreement_.baseRip.resize(chains);
    for (std::uint32_t chain = 0; chain < chains; ++chain) {
      agreement_.windows[chain] = feedlineWindow(wires, members, chain);
      for (const auto key : agreement_.windows[chain]) {
        agreement_.windowNames[chain].insert(wireId(wires[key]));
      }
    }
    // The free wires beside each feedline join its window.
    std::unordered_map<std::string, std::uint32_t> keyOf;
    for (const auto member : members) {
      keyOf.emplace(wireId(wires[member]), member);
    }
    const auto beside = freeWiresBeside(wires, agreement_.windowNames);
    for (std::uint32_t chain = 0; chain < chains; ++chain) {
      if (agreement_.windows[chain].empty()) {
        continue;
      }
      for (const auto& name : beside[chain]) {
        if (const auto found = keyOf.find(name); found != keyOf.end()) {
          agreement_.windows[chain].insert(found->second);
          agreement_.windowNames[chain].insert(name);
        }
      }
    }
    if (const auto checks = slotCheckNow(wires, agreement_.windowNames,
                                         "local agreement: as begun");
        checks.has_value()) {
      for (std::uint32_t chain = 0; chain < chains; ++chain) {
        agreement_.baseSat[chain] =
            !checks->feedlines[chain].has_value() ||
            lostIn(*checks->feedlines[chain], agreement_.windowNames[chain])
                .empty();
      }
    }
    {
      const auto taken = snapshot(wires);
      for (std::uint32_t chain = 0; chain < chains; ++chain) {
        if (agreement_.windows[chain].empty()) {
          continue;
        }
        agreement_.baseRip[chain] =
            ripFeedline(wires, members, every, pass, agreement_.windows[chain]);
        restore(wires, taken);
        foreignRoomStale_ = true;
        say(std::format("local agreement: as begun: feedline {} {}, rip-up "
                        "{}{}",
                        chain, agreement_.baseSat[chain] ? "SAT" : "UNSAT",
                        agreement_.baseRip[chain].clean ? "clean" : "fails",
                        agreement_.baseRip[chain].failing.empty()
                            ? std::string{}
                            : " (" + agreement_.baseRip[chain].failing + ")"));
      }
    }
    auto badNow = failsOf(wires, every).bad();
    std::uint32_t segments = 0;
    for (std::uint32_t chain = 0; chain < chains; ++chain) {
      const auto& points = chains_[chain];
      if (points.size() < 3) {
        continue;
      }
      const auto span = std::min(spanAsked, points.size() - 1);
      for (std::size_t lo = 0; lo + span < points.size(); ++lo) {
        ++segments;
        static_cast<void>(researchSegment(wires, members, every, pass,
                                          {.chain = chain,
                                           .lo = lo,
                                           .hi = lo + span,
                                           .fails = {},
                                           .because = {}},
                                          runs, badNow));
      }
    }
    const auto& w = agreement_.whole;
    const auto seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - began)
            .count();
    say(std::format(
        "==> local agreement: {} segments; feedlines judged: {} SAT and "
        "clean, {} SAT but failing, {} UNSAT but clean, {} UNSAT and "
        "failing; {} not judged; {:.0f}s in the slot check, {:.0f}s "
        "rerouting, {:.0f}s in all",
        segments, w[0], w[1], w[2], w[3], agreement_.unjudged,
        agreement_.slotSeconds, agreement_.rerouteSeconds, seconds));
    nlohmann::json begun = nlohmann::json::array();
    for (std::uint32_t chain = 0; chain < chains; ++chain) {
      if (agreement_.windows[chain].empty()) {
        continue;
      }
      begun.push_back({{"chain", chain},
                       {"sat", agreement_.baseSat[chain]},
                       {"clean", agreement_.baseRip[chain].clean},
                       {"failing", agreement_.baseRip[chain].failing}});
    }
    nlohmann::json out{{"segments", segments},
                       {"runsPerSegment", runs},
                       {"span", spanAsked},
                       {"classes",
                        {{"satClean", w[0]},
                         {"satFailing", w[1]},
                         {"unsatClean", w[2]},
                         {"unsatFailing", w[3]}}},
                       {"unjudged", agreement_.unjudged},
                       {"slotSeconds", agreement_.slotSeconds},
                       {"rerouteSeconds", agreement_.rerouteSeconds},
                       {"seconds", seconds},
                       {"begun", std::move(begun)},
                       {"runs", std::move(agreement_.records)}};
    agreement_ = Agreement{};
    return out;
  }

  /// **The targeted repair over one chip**, in place of the blind `repair`:
  /// the triage, then every segment re-searched in order under the budget
  /// of `repair_trials` legality tests. After an accept the fails are read
  /// again and the triage rebuilt on the new state — a segment whose fails
  /// an earlier accept cleared is then not there to be searched, and one
  /// whose fails remain is searched against the chip as it now stands. The
  /// figure can only fall with an accept, so the loop ends. Then, under
  /// `researchFinalSweep`, one full pass over the ring, kept if the figure
  /// does not rise. One `==>` line says what came of it.
  void researchSegments(std::vector<Wire>& wires,
                        const std::vector<std::uint32_t>& members,
                        const std::vector<std::uint32_t>& every,
                        const Pass& pass) {
    const auto began = std::chrono::steady_clock::now();
    auto fails = failsOf(wires, every);
    const auto before = fails;
    auto segments = blameFails(wires, members, fails);
    const auto built = segments.size();
    auto badNow = fails.bad();
    std::uint32_t trials = 0;
    std::uint32_t accepted = 0;
    std::uint32_t searched = 0;
    std::uint32_t rejected = 0;
    std::uint32_t skipped = 0;
    std::uint32_t routed = 0;
    std::uint32_t retriaged = 0;
    // The segments searched without an accept, by chain, span and the fails
    // they carried. After an accept the triage is rebuilt, and a segment
    // that comes back unchanged would be searched again against a chip that
    // changed somewhere else — on 33q that spent a second 20 s on chain 0
    // for the same three refusals. Skipped, and said; a segment whose fails
    // changed is searched again.
    std::set<std::tuple<std::uint32_t, std::size_t, std::size_t,
                        std::vector<std::uint32_t>>>
        fruitless;
    std::size_t at = 0;
    while (at < segments.size() && badNow > 0) {
      if (trials >= tuning_.repairTrials) {
        say(std::format(
            "targeted repair: the budget of {} legality test{} is "
            "spent, {} segment{} left unsearched",
            tuning_.repairTrials, tuning_.repairTrials == 1 ? "" : "s",
            segments.size() - at, segments.size() - at == 1 ? "" : "s"));
        break;
      }
      const Segment segment = segments[at];
      const auto key =
          std::tuple{segment.chain, segment.lo, segment.hi, segment.fails};
      if (fruitless.contains(key)) {
        tell(std::format("targeted repair: {} searched before without an "
                         "accept and unchanged since, skipped",
                         segmentName(wires, segment)));
        ++at;
        continue;
      }
      ++searched;
      const auto result =
          researchSegment(wires, members, every, pass, segment,
                          tuning_.repairTrials - trials, badNow);
      trials += result.trials;
      rejected += result.rejected;
      skipped += result.skippedCurrent;
      routed += result.routed;
      if (!result.accepted) {
        fruitless.insert(key);
        ++at;
        continue;
      }
      ++accepted;
      ++retriaged;
      // The triage again on the new state, its fail lines silenced; the
      // summary says what remains.
      const auto loud = std::exchange(report_, Report{});
      fails = failsOf(wires, every);
      segments = blameFails(wires, members, fails);
      report_ = loud;
      at = 0;
    }
    // Everything down again, so that the field says what the canvas holds.
    for (auto& wire : wires) {
      place(wire);
    }
    const auto loud = std::exchange(report_, Report{});
    const auto after = failsOf(wires, every);
    report_ = loud;
    // The fourth guarantee as the stage leaves it: the pass redraws the
    // terminal edges and the repair moves couplers, so the insertion's line
    // is not the last word on it. Only here, under the switch, so that
    // `SCPD_REPAIR_SEARCH=0` reproduces the stage's lines exactly.
    static_cast<void>(checkFeedlineCrossings(wires, "feedline routing"));
    static_cast<void>(checkChainCrossings(wires, "feedline routing"));
    // And the first guarantee on what the sweep left, so that a pair the
    // refinement's own line reports can be told from one the sweep made:
    // the insertion's line is about the edges as the insertion drew them,
    // and every pass after it draws them again.
    static_cast<void>(checkFeedlineRoom(wires, "feedline routing"));
    const auto seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - began)
            .count();
    say(std::format(
        "==> targeted repair: {} segment{} built, {} searched, {} accepted ({} "
        "re-triage{}); {} legality test{} of {} ({} refused, {} the current "
        "run), {} edge searches; unrouted {} -> {}, open {} -> {}, crossing {} "
        "-> {} (bad {} -> {}), {:.1f}s",
        built, built == 1 ? "" : "s", searched, accepted, retriaged,
        retriaged == 1 ? "" : "s", trials, trials == 1 ? "" : "s",
        tuning_.repairTrials, rejected, skipped, routed, before.unrouted,
        after.unrouted, before.open, after.open, before.crossing,
        after.crossing, before.bad(), after.bad(), seconds));
  }

private:
  /// How many rounds without progress end a pass.
  static constexpr std::uint32_t STALE_ROUNDS = 4;

  /// How many ways the pass running now refused because they did not cross
  /// the wire's bridge. Reset per sweep, said at its end.
  std::uint32_t bridgeRefusals_ = 0;

  /// Whether the length-point bands are stamped at all. Cleared for the
  /// recovery pass at the end of the outer routing (`recoverWithoutBands`)
  /// and set again after it.
  bool lengthPointActive_ = true;

  /// Charge the places no wire may be moved off, before anything is drawn.
  ///
  /// A wire has more of them than its two ends. The straight run it leaves its
  /// source on is fixed by the orientation of the port that feeds it: the
  /// search never sees those cells, because the router adds the run to the way
  /// after it has searched. So a wire drawn across another wire's run is a
  /// violation the search could not have refused and no later round can undo.
  /// They are charged here, before the first wire is drawn, and only the wire
  /// being searched for is let out of its own.
  void fixPlaces(std::vector<Wire>& wires,
                 const std::vector<std::uint32_t>& members, const Pass& pass) {
    for (const auto member : members) {
      auto& wire = wires[member];
      if (!wire.feasible || !wire.fixed.empty()) {
        continue;
      }
      const auto stub = startStubOf(wire, pass.straightStart);
      wire.fixed = wire.arc;
      const auto out = router_.straightStub(wire.objective.source, false, stub);
      wire.fixed.insert(wire.fixed.end(), out.begin(), out.end());
      if (wire.endStub > 0) {
        const auto in =
            router_.straightStub(wire.objective.target, true, wire.endStub);
        wire.fixed.insert(wire.fixed.end(), in.begin(), in.end());
      } else {
        wire.fixed.push_back(wire.objective.target);
      }
      if (!wire.placed) {
        field_.chargeFixed(wire.fixed);
        wire.endsOnly = true;
      }
    }
  }

  /// Put every wire of a pass down on the way the Detail stage drew, before
  /// anything is searched.
  ///
  /// What a wire starts on is the Detail stage's cells joined on this grid,
  /// with the straight run out of its source spliced in front: that run is
  /// part of every way the search will ever return, and the room around it
  /// has to stand from the first search on or a neighbour settles across it.
  /// A wire that has a way of its own from an earlier pass, or that is down
  /// already, is left alone; a wire the Detail stage did not draw keeps only
  /// its fixed places charged, as `fixPlaces` left them.
  void seed(std::vector<Wire>& wires, const std::vector<std::uint32_t>& members,
            const Pass& pass) {
    std::uint32_t seeded = 0;
    std::uint32_t feasible = 0;
    for (const auto member : members) {
      auto& wire = wires[member];
      if (!wire.feasible) {
        continue;
      }
      ++feasible;
      if (wire.drawn || wire.placed || wire.way.empty()) {
        continue;
      }
      wire.way = seededWay(wire);
      place(wire);
      ++seeded;
    }
    say(std::format("{}: {} of {} wires start on the way the Detail stage "
                    "drew",
                    pass.name, seeded, feasible));
  }

  /// The Detail stage's cells as a way on this grid.
  ///
  /// The cells come from a grid three to four times coarser, so consecutive
  /// ones lie apart; each pair is joined by the eight-connected steps of the
  /// line between them, which is what makes the result a way the field can
  /// hold like any other. In front of it runs the straight stub out of the
  /// source, which `fixed` holds; the seed cells the stub already covers are
  /// skipped, and the way ends on the target cell whatever the last seed cell
  /// was. The headings are those of the steps.
  [[nodiscard]] static Path seededWay(const Wire& wire) {
    Path way;
    if (wire.fixed.size() >= 2) {
      way.assign(wire.fixed.begin(), wire.fixed.end() - 1);
    } else {
      way.push_back(wire.objective.source);
    }
    const auto& source = wire.objective.source;
    const auto covered = static_cast<double>(way.size() - 1);
    auto first = wire.way.begin();
    while (first != wire.way.end() &&
           std::hypot(static_cast<double>(first->x) - source.x,
                      static_cast<double>(first->y) - source.y) <= covered) {
      ++first;
    }
    for (auto point = first; point != wire.way.end(); ++point) {
      connect(way, *point);
    }
    connect(way, wire.objective.target);
    for (std::size_t at = 0; at + 1 < way.size(); ++at) {
      way[at].heading = headingOfStep(way[at], way[at + 1]);
      way[at].primitive = 0;
    }
    way.back().heading = wire.objective.target.heading;
    way.back().primitive = 0;
    return way;
  }

  /// One wire's turn: the prototype's two phases, and nothing else.
  ///
  /// Phase 1 offers the wire a way inside the band around the way it has,
  /// with its two ring neighbours standing in the way as obstacles inflated
  /// by the clearance. Phase 2 is the relaxation, along the sweep as the
  /// prototype's: each level lets go of one more of the wires ahead — a wire
  /// let go of is no obstacle at all, because it is drawn again afterwards —
  /// and fences the search with the last wire let go of and the neighbour on
  /// the other side. The lane between the two ring neighbours is free and
  /// everything outside it priced, and the wires let go of are priced at
  /// growing distances, so the way is pushed away from where they run rather
  /// than drawn over it. A way found is taken as it is; when none is found,
  /// the wires let go of go back to what they were.
  ///
  /// A resonator's way found is not a way taken until it is long enough.
  /// The meander goes in after each search, the cheapest placement that
  /// fits, and a way without room for it counts as no way, so the
  /// relaxation goes on; when nothing comes of
  /// it the wire keeps that way, short as it is, as the prototype keeps it.
  ///
  /// The rule against every other wire is not what the search keeps. It is
  /// what the fails are counted by when the round is over.
  bool attempt(std::vector<Wire>& wires,
               const std::vector<std::uint32_t>& members,
               const std::uint32_t slot, const bool forward, const Pass& pass) {
    const auto total = static_cast<std::uint32_t>(members.size());
    auto& wire = wires[members[slot]];
    const auto& before = wires[members[(slot + total - 1) % total]];
    const auto& after = wires[members[(slot + 1) % total]];

    // Under the feedline constraints a wire starts on the way it has, and a
    // way that holds every rule already is settled: the rule against every
    // other wire, the crossing rule, and a resonator's length. Only a wire
    // whose way does not hold would be searched for; every wire is drawn
    // again at least once, which draws the kink out of a spliced way: a
    // kinked way passes `isLegal`.
    // No lane until one is priced: the frame outlives a wire, and a picture
    // of the next one must not show the last one's polygon.
    const bool ripped = wire.ripped;
    wire.ripped = false;
    std::string broken;
    const bool holds =
        pass.feedlines && wire.drawn && !ripped &&
        isLegal(wire, wires, report_.wants(Detail::Items) ? &broken : nullptr);
    if (holds && !FORCE_REROUTE) {
      wire.routed = true;
      wire.tooShort = false;
      wire.tooLong = false;
      place(wire);
      tell(std::format("wire {} · round {} {}: keeps its way, which holds",
                       wireId(wire), frame_.round,
                       forward ? "forward" : "backward"));
      return true;
    }
    if (!broken.empty()) {
      tell(std::format("wire {} · round {} {}: its way does not hold, {}",
                       wireId(wire), frame_.round,
                       forward ? "forward" : "backward", broken));
    }
    // The way a resonator has when it holds every rule, its length
    // included: the outer routing lays the meander where the coupler keeps
    // it, so the way the coupler leaves is often at its length already. It
    // is drawn again all the same, but a way found with no room for a
    // meander does not replace it.
    const Path held = holds && lengthensIn(wire) ? wire.way : Path{};
    // A resonator whose way holds every rule but its length: what the
    // coupler left is often a little short. When the way found has no room
    // for a meander, the meander goes into this way instead.
    const Path shortOf = !holds && pass.feedlines && wire.drawn && !ripped &&
                                 lengthensIn(wire) &&
                                 isLegal(wire, wires, nullptr, false)
                             ? wire.way
                             : Path{};

    // The wire is lifted off the canvas for the length of its own search, so
    // that what is put down afterwards is the one way it has.
    lift(wire);

    // Phase 1: the band around the way it has, fenced by its two neighbours,
    // and nothing priced — except, under the feedline constraints, the
    // chains: fenced, and priced around.
    buildCorridor(wire, pass.reach);
    fence(wire, {&before, &after});
    // The prototype's feedline pass fences the second pair as well
    // (`additional_paths_1`, `FinalGrid.cpp:4262`), where its outer routing
    // fences only the first. A ring whose order the chain edges have cut is
    // no longer planar, and one neighbour either side no longer says where
    // a wire may run.
    if (pass.feedlines && total > 4) {
      // And the pairs beyond it, as `fencePairs` says.
      for (std::uint32_t k = 2; k <= tuning_.fencePairs && 2 * k < total; ++k) {
        fence(wire, {&wires[members[(slot + total - k) % total]],
                     &wires[members[(slot + k) % total]]});
      }
    }
    // The length-point clearance of the outer routing: see
    // `tuning_.lengthPointRadius`.
    closeLengthBands(wire, {&before, &after}, pass);
    std::ranges::fill(proximity_, 0);
    constrainByFeedlines(wire, wires, pass);
    frame_.wires = &wires;
    frame_.before = before.key;
    frame_.after = after.key;
    frame_.kind = "normal";
    frame_.fence = {before.key, after.key};
    frame_.ripped.clear();
    const bool lengthened = lengthensIn(wire);
    auto found = search(wire, pass.straightStart, true, !lengthened);
    const auto id = wireId(wire);
    const auto where = std::format("round {} {}", frame_.round,
                                   forward ? "forward" : "backward");
    RoundRecord* record = rounds_.empty() ? nullptr : &rounds_.back();
    // A wire that has to cross a chain edge has to cross **its** edge; a way
    // that comes home on the wrong side of the feedline is refused as no
    // way, so that the wire fails here rather than standing in the way of
    // every wire drawn after it. See `bridgeCheck`.
    const auto crossesItsBridge = [&](const Path& way) {
      if (!pass.feedlines || wire.bridged == NO_OWNER ||
          wire.bridged >= wires.size()) {
        return true;
      }
      const auto& bridge = wires[wire.bridged];
      return !bridge.drawn || bridge.way.empty() || waysCross(way, bridge.way);
    };
    const auto refuseTheDetour = [&](const std::string& phase) {
      if (found.empty() || crossesItsBridge(found)) {
        return;
      }
      tell(std::format("wire {} · {} · {}: found {} cells that do not cross "
                       "its bridge {} — refused as no way",
                       id, where, phase, found.size(),
                       wireId(wires[wire.bridged])));
      ++bridgeRefusals_;
      found.clear();
    };
    refuseTheDetour("normal");
    // The last way found that could not be made long enough, and what the
    // lengthening said. A way found is kept only when it is long enough.
    Path plain;
    bool plainTooLong = false;
    std::string note;
    const auto lengthenFound = [&]() {
      note.clear();
      if (!lengthened) {
        return;
      }
      std::string said;
      if (!found.empty()) {
        const auto made = lengthen(
            wire, found, pass.refinement ? 1.0 : tuning_.sweepLengthBand);
        said = made.note;
        note = " · " + said;
        if (!made.reached) {
          plain = std::move(found);
          plainTooLong = made.tooLong;
          found.clear();
        }
      }
      // The picture of the search, taken here rather than in `search` so
      // that it shows the way with its meander, or the way that had no room
      // for one.
      recordSearch(wire, found.empty() && !said.empty() ? plain : found, said);
    };
    // What a search came to, in words: the way found and what the
    // lengthening made of it, or a way found and left short, or nothing.
    const auto outcome = [&]() {
      if (!found.empty()) {
        return std::format("found {} cells{}", found.size(), note);
      }
      if (!note.empty()) {
        return std::format("found {} cells{}", plain.size(), note);
      }
      return std::string("no way");
    };
    lengthenFound();
    if (!found.empty()) {
      tell(std::format("wire {} · {} · normal: {}", id, where, outcome()));
      if (record != nullptr) {
        ++record->normal;
      }
    } else {
      // Asked before `whatBlocks`, which puts the corridor back without the
      // second pair of neighbours and would misreport an open start.
      const auto arrival = report_.wants(Detail::Items)
                               ? deadOnArrival(wire, wires, pass.straightStart)
                               : std::string{};
      tell(std::format("wire {} · {} · normal: {}, relaxing{}{}", id, where,
                       outcome(),
                       report_.wants(Detail::Items) && pass.feedlines
                           ? whatBlocks(wire, wires, before, after, pass)
                           : std::string{},
                       arrival));
    }

    // A resonator whose way was found but had no room for its meander keeps
    // that way and lets no neighbour go to make room for one (user,
    // 2026-10-13): the relaxation would draw the meander into the room of
    // the wires it lets go of, and those then find no way back. The wire
    // is tried again in the next round, when the wires around it may have
    // moved.
    if (found.empty() && !plain.empty() && !held.empty()) {
      wire.way = held;
      wire.drawn = true;
      wire.tooShort = false;
      wire.tooLong = false;
      place(wire);
      wire.routed = true;
      tell(std::format("wire {} · {}: no room for its meander; keeps the way "
                       "it had, which holds",
                       id, where));
      return true;
    }
    if (found.empty() && !plain.empty() && !shortOf.empty()) {
      Path again = shortOf;
      const auto made = lengthen(
          wire, again, pass.refinement ? 1.0 : tuning_.sweepLengthBand);
      if (made.reached) {
        const Path had = std::exchange(wire.way, std::move(again));
        if (isLegal(wire, wires)) {
          wire.drawn = true;
          wire.tooShort = false;
          wire.tooLong = false;
          place(wire);
          wire.routed = true;
          tell(std::format("wire {} · {}: no room for its meander; the way it "
                           "had takes one · {}",
                           id, where, made.note));
          return true;
        }
        wire.way = had;
      }
      tell(std::format("wire {} · {}: the way it had takes no meander either "
                       "· {}",
                       id, where, made.note));
    }
    if (found.empty() && !plain.empty()) {
      wire.way = std::move(plain);
      wire.drawn = true;
      wire.tooShort = !plainTooLong;
      wire.tooLong = plainTooLong;
      place(wire);
      wire.routed = false;
      tell(std::format("wire {} · {}: no room for its meander; keeps the way "
                       "it found and lets no neighbour go",
                       id, where));
      if (record != nullptr) {
        record->failed.push_back(id);
        record->tooShort += wire.tooShort ? 1 : 0;
      }
      return false;
    }

    // Phase 2: the relaxation.
    std::vector<std::pair<std::uint32_t, bool>> released;
    for (std::uint32_t level = 1;
         found.empty() && total > 1 && level <= pass.maxRelaxation; ++level) {
      const auto step = forward ? (slot + level) % total
                                : (slot + total - (level % total)) % total;
      if (step == slot) {
        continue;
      }
      auto& ripped = wires[members[step]];
      released.emplace_back(ripped.key, ripped.routed);
      ripped.routed = false;

      buildCorridor(wire, pass.reach);
      // What the fence actually closed, so that the picture and the log can
      // say it. They used to name `{ripped, before/after}` whatever the
      // fence did, and since the prototype's rule came in that is wrong on
      // both counts: the wire let go of is **not** fenced — it is the one
      // thing meant to stay crossable — and two of the four that are fenced
      // were never named. A reader then sees a closed band drawn over a
      // crossable wire and no band over two closed ones (user, 2026-10-01).
      std::vector<std::uint32_t> closed;
      if (pass.feedlines && total > 4) {
        // The prototype fences the wire *after* the one it let go of, not
        // the one it let go of — that one is meant to be crossable — and the
        // second pair with it (`FinalGrid.cpp:4430`).
        const auto onward = [&](const std::uint32_t at) {
          return forward ? (at + 1) % total : (at + total - 1) % total;
        };
        const auto backward = [&](const std::uint32_t at) {
          return forward ? (at + total - 1) % total : (at + 1) % total;
        };
        // `fencePairs` pairs: the k-th beyond the released wire and
        // the k-th behind this one, stopping before a pair wraps round.
        auto on = step;
        auto back = slot;
        for (std::uint32_t k = 1; k <= tuning_.fencePairs; ++k) {
          on = onward(on);
          back = backward(back);
          if (on == slot || back == step || on == back) {
            break;
          }
          // Never a wire let go of at this level or one before: those are
          // the ones meant to stay crossable (user, 2026-10-05).
          for (const auto at : {on, back}) {
            const auto& other = wires[members[at]];
            const bool ripped = std::ranges::any_of(
                released, [&](const auto& r) { return r.first == other.key; });
            if (ripped || other.key == wire.key) {
              continue;
            }
            fence(wire, {&other});
            if (std::ranges::find(closed, other.key) == closed.end()) {
              closed.push_back(other.key);
            }
          }
        }
      } else {
        fence(wire, {&ripped, forward ? &before : &after});
        closed = {ripped.key, (forward ? before : after).key};
        // Only the neighbour on the far side keeps its length-point band at
        // a relaxation level. The wire let go of at this level is fenced by
        // its way and nothing more: it is drawn again afterwards and its
        // length point moves with it, so its band is let go of with it
        // (user, 2026-10-04) — which is also what the prototype ends up
        // doing, its stamp of that wire's band landing before the corridor
        // buffer is reset (`FinalGrid.cpp:7095-7102`). The wires let go of
        // at the levels before are crossable and get no band either.
        closeLengthBands(wire, {forward ? &before : &after}, pass);
      }
      fenceFixedPlaces(wire, wires, released);
      priceLane(wires, members, slot, forward, level);
      letGo_.clear();
      {
        for (const auto& [key, routed] : released) {
          letGo_.push_back(key);
        }
      }
      constrainByFeedlines(wire, wires, pass);
      letGo_.clear();
      frame_.kind = std::format("relax {}", level);
      frame_.fence = closed;
      frame_.ripped.push_back(ripped.key);
      found = search(wire, pass.straightStart, true, !lengthened);
      refuseTheDetour(std::format("relax {}", level));
      lengthenFound();
      std::string fenced;
      for (const auto key : closed) {
        fenced += (fenced.empty() ? "" : ", ") + wireId(wires[key]);
      }
      tell(std::format("wire {} · relax {}: let go of {}, fence {} · {}{}", id,
                       level, wireId(ripped),
                       fenced.empty() ? std::string("nothing") : fenced,
                       outcome(),
                       found.empty() && report_.wants(Detail::Items)
                           ? deadOnArrival(wire, wires, pass.straightStart)
                           : std::string{}));
    }

    // Both neighbours at once (`relaxBoth`), when the round before, the other
    // way round, ended for this wire with no way at any level either.
    const auto thisRound =
        (static_cast<std::uint64_t>(sweepSerial_) << 32U) | (frame_.round + 1U);
    if (found.empty() && total > 4 && wire.relaxedOut != 0 &&
        wire.relaxedOut + 1 == thisRound) {
      auto& ahead = wires[members[(slot + 1) % total]];
      auto& behind = wires[members[(slot + total - 1) % total]];
      for (Wire* const other : {&ahead, &behind}) {
        if (std::ranges::none_of(released, [&](const auto& r) {
              return r.first == other->key;
            })) {
          released.emplace_back(other->key, other->routed);
          other->routed = false;
        }
      }
      const auto& farAhead = wires[members[(slot + 2) % total]];
      const auto& farBehind = wires[members[(slot + total - 2) % total]];
      buildCorridor(wire, pass.reach);
      fence(wire, {&farAhead, &farBehind});
      fenceFixedPlaces(wire, wires, released);
      letGo_.clear();
      {
        for (const auto& [key, routed] : released) {
          letGo_.push_back(key);
        }
      }
      constrainByFeedlines(wire, wires, pass);
      letGo_.clear();
      const auto level = pass.maxRelaxation + 1;
      frame_.kind = std::format("relax {}", level);
      frame_.fence = {farAhead.key, farBehind.key};
      frame_.ripped.push_back(ahead.key);
      frame_.ripped.push_back(behind.key);
      found = search(wire, pass.straightStart, true, !lengthened);
      refuseTheDetour(std::format("relax {}", level));
      lengthenFound();
      tell(std::format("wire {} · relax {}: let go of {} and {}, both "
                       "neighbours, fence {}, {} · {}",
                       id, level, wireId(behind), wireId(ahead),
                       wireId(farBehind), wireId(farAhead), outcome()));
    }
    wire.relaxedOut = found.empty() ? thisRound : 0;

    if (found.empty()) {
      // The rollback: every wire let go of is exactly what it was.
      std::string names;
      for (const auto& [key, routed] : released) {
        wires[key].routed = routed;
        names += (names.empty() ? "" : ", ") + wireId(wires[key]);
      }
      // A resonator whose way was found but could not be made long enough
      // keeps that way: it is what the wires around it see, and the wire is
      // tried again in the next round.
      if (!plain.empty()) {
        wire.way = std::move(plain);
        wire.drawn = true;
        wire.tooShort = !plainTooLong;
        wire.tooLong = plainTooLong;
      }
      place(wire);
      wire.routed = false;
      tell(std::format("wire {} · {}: no way after {} relaxations; {} back as "
                       "they were{}",
                       id, where, released.size(),
                       names.empty() ? std::string("nothing") : names,
                       wire.tooShort ? "; keeps a way too short for its meander"
                                     : ""));
      if (record != nullptr) {
        record->failed.push_back(id);
        record->tooShort += wire.tooShort ? 1 : 0;
      }
      return false;
    }
    if (record != nullptr && !released.empty()) {
      ++record->relaxed;
    }
    for (const auto& [key, routed] : released) {
      wires[key].ripped = true;
    }
    wire.way = found;
    wire.drawn = true;
    wire.routed = true;
    wire.tooShort = false;
    wire.tooLong = false;
    place(wire);
    return true;
  }

  /// What stands in the way of a wire that found nothing under the feedline
  /// constraints, for the second level of verbosity: the search again with
  /// one constraint left out at a time — the ring neighbours, the feedline
  /// fences, the crossing rule, the coupler bodies — and with all of them
  /// left out. Restores what the search had.
  [[nodiscard]] std::string whatBlocks(const Wire& wire,
                                       std::vector<Wire>& wires,
                                       const Wire& before, const Wire& after,
                                       const Pass& pass) {
    const auto attempt = [&](const bool neighbours, const bool feedlines,
                             const bool crossing, const bool bodies) {
      const auto kept = bodies_;
      if (!bodies) {
        bodies_ = grid::BitGrid(scene_.router.width, scene_.router.height);
      }
      buildCorridor(wire, pass.reach);
      if (neighbours) {
        fence(wire, {&before, &after});
      }
      std::ranges::fill(proximity_, 0);
      if (feedlines) {
        for (const auto& edge : edges_) {
          const auto& other = wires[edge.wire];
          if (other.drawn && other.key != wire.key &&
              other.key != wire.bridged) {
            fence(wire, {&other});
          }
        }
      }
      if (!crossing) {
        router_.clearOrthogonalConstraints();
      }
      const bool found = !search(wire, pass.straightStart, true, false).empty();
      if (!crossing) {
        // Only the rule goes back. `beginPass` would do it, but it also
        // rebuilds the field and puts **every** wire down again — the one
        // this attempt has lifted included — and the attempt goes on
        // believing it is still off the canvas. That made the second level
        // of verbosity change the result it was there to explain: the same
        // settings came to 6 open at `-v` and 4 at `-v 1`.
        rebuildCrossingRule(wires);
      }
      bodies_ = kept;
      return found;
    };
    std::string blocked;
    const auto note = [&](const std::string& what, const bool found) {
      if (found) {
        blocked += (blocked.empty() ? "" : ", ") + what;
      }
    };
    note("the neighbours", attempt(false, true, true, true));
    note("the feedlines", attempt(true, false, true, true));
    note("the crossing rule", attempt(true, true, false, true));
    note("the coupler bodies", attempt(true, true, true, false));
    if (blocked.empty()) {
      blocked = attempt(false, false, false, false) ? "several together"
                                                    : "the band itself";
    }
    // What the search had, put back.
    buildCorridor(wire, pass.reach);
    fence(wire, {&before, &after});
    std::ranges::fill(proximity_, 0);
    constrainByFeedlines(wire, wires, pass);
    return " · in the way: " + blocked;
  }

  /// Whether a search was over before it began, for the log of one that
  /// found nothing: the cell it starts on — the source moved along its
  /// heading by the straight run — and the cells a first step can reach,
  /// which of them are closed, and what lies within the clearance of each.
  /// Empty when the start is open.
  ///
  /// A search that fails this way fails whatever else is loosened, and it is
  /// what 45q's wire 4 did in every round: its start stood 17 cells from wire
  /// 3, inside 3's clearance, with its own seed way open end to end and a
  /// strip twenty cells wide beside it (2026-10-03). The `in the way` line
  /// cannot see this — it never fences the second pair of neighbours — and
  /// the pictures draw the closed cells white and unnamed.
  [[nodiscard]] std::string deadOnArrival(const Wire& wire,
                                          const std::vector<Wire>& wires,
                                          const std::uint32_t straightStart) {
    const auto stub =
        static_cast<std::int64_t>(startStubOf(wire, straightStart));
    const auto& source = wire.objective.source;
    const auto v = routing::headingVector(source.heading);
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto height = static_cast<std::int64_t>(scene_.router.height);
    const std::int64_t sx = static_cast<std::int64_t>(source.x) + (v.dx * stub);
    const std::int64_t sy = static_cast<std::int64_t>(source.y) + (v.dy * stub);
    // The start, the cell straight ahead and the two beside that one: the
    // least any first move sweeps.
    const std::int64_t px = -v.dy;
    const std::int64_t py = v.dx;
    const std::array<std::pair<std::int64_t, std::int64_t>, 4> first{
        {{sx, sy},
         {sx + v.dx, sy + v.dy},
         {sx + v.dx + px, sy + v.dy + py},
         {sx + v.dx - px, sy + v.dy - py}}};
    std::string closed;
    std::uint32_t shut = 0;
    for (const auto& [x, y] : first) {
      if (x < 0 || y < 0 || x >= width || y >= height) {
        ++shut;
        closed += std::format("{}({},{}) off the grid",
                              closed.empty() ? "" : "; ", x, y);
        continue;
      }
      if (!corridor_.test(static_cast<std::size_t>((y * width) + x))) {
        continue;
      }
      ++shut;
      closed += std::format("{}({},{}) closed, within the clearance of {}",
                            closed.empty() ? "" : "; ", x, y,
                            whatIsNear(wire, wires, x, y));
    }
    if (shut == 0) {
      return {};
    }
    return std::format(" · dead on arrival: the search starts at ({},{}) "
                       "heading ({},{}) after a stub of {}, and {} of its "
                       "first {} cells are closed: {}",
                       sx, sy, v.dx, v.dy, stub, shut, first.size(), closed);
  }

  /// What lies within the clearance of a cell, by name: the artwork, a
  /// coupler body, and every wire whose way or fixed places reach it. Not
  /// every wire named is fenced in the search at hand; the line says what
  /// is there, the fences say what is closed.
  [[nodiscard]] std::string whatIsNear(const Wire& wire,
                                       const std::vector<Wire>& wires,
                                       const std::int64_t x,
                                       const std::int64_t y) const {
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto cell = static_cast<std::size_t>((y * width) + x);
    std::string names;
    if (scene_.blocked.test(cell)) {
      names = "artwork or keepout";
    }
    if (bodies_.test(cell)) {
      names += std::string(names.empty() ? "" : ", ") + "a coupler body";
    }
    const auto limit = static_cast<double>(tuning_.clearance);
    for (const auto& other : wires) {
      if (other.key == wire.key) {
        continue;
      }
      auto least = std::numeric_limits<double>::max();
      const char* what = "";
      for (const Path* const cells : {&other.way, &other.fixed}) {
        for (const auto& point : *cells) {
          const auto far =
              std::hypot(static_cast<double>(point.x) - static_cast<double>(x),
                         static_cast<double>(point.y) - static_cast<double>(y));
          if (far < least) {
            least = far;
            what = cells == &other.way ? "way" : "fixed places";
          }
        }
      }
      if (least <= limit) {
        names +=
            std::format("{}{} ({} at {:.1f} cells)", names.empty() ? "" : ", ",
                        wireId(other), what, least);
      }
    }
    return names.empty()
               ? std::string("nothing named: the band, a port band or a body")
               : names;
  }

  /// Whether the way a wire has holds every rule the pass judges by: the
  /// rule against every other wire, the crossing rule, and the length of a
  /// resonator.
  /// @param why Where to say which rule the way breaks, when given.
  /// @param length Whether a resonator's length is one of the rules.
  [[nodiscard]] bool isLegal(Wire& wire, const std::vector<Wire>& wires,
                             std::string* why = nullptr,
                             const bool length = true) {
    const auto broken = [why](std::string rule) {
      if (why != nullptr) {
        *why = std::move(rule);
      }
      return false;
    };
    if (!wire.drawn || wire.way.empty()) {
      return broken("it has no way");
    }
    const bool placed = wire.placed;
    lift(wire);
    const bool open =
        conflictsOf(wire, wires) != 0 || crossesItsChain(wire, wires);
    if (open && why != nullptr) {
      *why = std::format("it lies in the room of {}", whoBlocks(wire, wires));
    }
    if (placed) {
      place(wire);
    }
    if (open) {
      return false;
    }
    if (crossesAFeedline(wire, wires)) {
      return broken("it crosses a feedline");
    }
    if (feedlinePass_ &&
        breaksAFeedlineRule(wire, wire.way,
                            startStubOf(wire, tuning_.straightStart), wires)) {
      return broken("it breaks a feedline rule");
    }
    if (length && needsLength(wire)) {
      const auto measured = lengthOf(wire.way);
      const auto required = requiredLength(wire, wire.way);
      const auto tolerance = exact_ ? tuning_.lengthTolerance : 0.0;
      if (measured < required - tolerance ||
          (exact_ && measured > required + tolerance)) {
        return broken(std::format("it is {:.0f} cells long against {:.0f}",
                                  measured, required));
      }
    }
    return true;
  }

  /// How many cells of a way lie in the room of another wire.
  ///
  /// The wire itself has to be off the canvas when this runs. A cell where
  /// the two wires meet is not counted, because the rule does not bind there
  /// and the design-rule check makes the same exemption.
  [[nodiscard]] std::uint32_t conflictsOf(const Wire& wire,
                                          const std::vector<Wire>& wires) {
    return conflictsIn(wire.way, wire, wires);
  }

  /// Whether two wires meet at a place, so that the rule does not bind there.
  ///
  /// They meet when a terminal of one sits within the rule of a terminal of
  /// the other: two ports that close together have to be reached by two
  /// wires whose approaches converge, and no arrangement holds them apart.
  /// What belongs to the meeting is everything within one and a half rules
  /// of either terminal. This is the design-rule check's own test, and it is
  /// geometry alone: that two wires end on one component says nothing about
  /// where its ports are — the two ports of a qubit can sit nine hundred
  /// units apart, and a wire passing the other's approach there is as much a
  /// violation as anywhere else.
  [[nodiscard]] bool meetAt(const Wire& one, const Wire& two, const double x,
                            const double y, const bool onLead = true) const {
    const auto link = static_cast<double>(tuning_.clearance);
    const auto reach = 1.5 * link;
    // A resonator and the feedline edges of its own coupler run beside each
    // other along the coupler on purpose: that is the coupling. Everything
    // within the coupler's reach of its anchor belongs to that meeting, as
    // far as `meetingReach` lets it.
    if (const auto shared = sharedCoupler(one, two); shared != NO_OWNER) {
      const auto& anchor = couplers_[shared].anchor;
      if (std::hypot(x - anchor.x, y - anchor.y) <=
          meetingReach(one, two, onLead)) {
        return true;
      }
      // An edge and the resonator of its coupler meet along the edge's run
      // out of its port on the pad (`runOutOf`), so that the resonator's
      // clearance does not close the edge's stub and first moves; anywhere
      // else the edge keeps the rule from the resonator, its lead's end
      // included (user, 2026-10-13).
      if (feedlinePass_ && one.feedline != two.feedline &&
          !(one.resonator && onLead)) {
        const auto& edge = one.feedline ? one : two;
        if (const auto run = runOutOf(edge, shared);
            run.has_value() && run->distanceTo(x, y) <= link) {
          return true;
        }
      }
    }
    const std::array<const PathPoint*, 2> mine{&one.objective.source,
                                               &one.objective.target};
    const std::array<const PathPoint*, 2> theirs{&two.objective.source,
                                                 &two.objective.target};
    for (const auto* const a : mine) {
      for (const auto* const b : theirs) {
        const auto apart = std::hypot(static_cast<double>(a->x) - b->x,
                                      static_cast<double>(a->y) - b->y);
        if (apart > link) {
          continue;
        }
        if (std::hypot(x - a->x, y - a->y) <= reach ||
            std::hypot(x - b->x, y - b->y) <= reach) {
          return true;
        }
      }
    }
    return false;
  }

  /// The straight run of an edge out of its port at a coupler: from the
  /// port along the heading the edge leaves it on, through its stub and a
  /// clearance beyond.
  struct RunOut {
    double x = 0.0;
    double y = 0.0;
    std::int64_t dx = 0;
    std::int64_t dy = 0;
    /// In steps of (`dx`, `dy`).
    std::int64_t steps = 0;
    /// The distance from a point to the run.
    [[nodiscard]] double distanceTo(const double px, const double py) const {
      const auto ex = static_cast<double>(dx * steps);
      const auto ey = static_cast<double>(dy * steps);
      const auto squared = (ex * ex) + (ey * ey);
      const auto t =
          squared > 0.0
              ? std::clamp((((px - x) * ex) + ((py - y) * ey)) / squared, 0.0,
                           1.0)
              : 0.0;
      return std::hypot(px - (x + (t * ex)), py - (y + (t * ey)));
    }
  };

  /// The run of `runOutOf` of edge `edge` at coupler `coupler`; nothing
  /// when the edge has no port there.
  [[nodiscard]] std::optional<RunOut>
  runOutOf(const Wire& edge, const std::uint32_t coupler) const {
    if (!edge.feedline || coupler == NO_OWNER) {
      return std::nullopt;
    }
    const bool source = edge.couplerAtSource == coupler;
    if (!source && edge.couplerAtTarget != coupler) {
      return std::nullopt;
    }
    const auto& port = source ? edge.objective.source : edge.objective.target;
    const auto heading =
        source ? port.heading : routing::turned(port.heading, 4);
    const auto v = routing::headingVector(heading);
    const auto stub =
        source ? edge.startStub.value_or(tuning_.straightStart) : edge.endStub;
    return RunOut{.x = static_cast<double>(port.x),
                  .y = static_cast<double>(port.y),
                  .dx = v.dx,
                  .dy = v.dy,
                  .steps = static_cast<std::int64_t>(stub) +
                           static_cast<std::int64_t>(tuning_.clearance)};
  }

  /// Open the straight run out of each port of an edge at a coupler (user,
  /// 2026-10-13): along the heading the edge leaves the port on, the cells
  /// within the clearance of that coupler's resonator, and the cell to
  /// either side of each, as far as the resonator's clearance reaches. The
  /// resonator's fence closes them, and the search of an edge starts only
  /// after its stub, so without this an edge whose resonator runs beside
  /// its port has no first move. Walls and copper stay closed, and the run
  /// stops at the first cell of either.
  void openTheRunsOut(const Wire& edge, const std::vector<Wire>& wires) {
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto height = static_cast<std::int64_t>(scene_.router.height);
    const auto limit = static_cast<double>(tuning_.clearance);
    for (const auto coupler : {edge.couplerAtSource, edge.couplerAtTarget}) {
      if (coupler == NO_OWNER || coupler >= couplers_.size() ||
          couplers_[coupler].wire >= wires.size()) {
        continue;
      }
      const auto& resonator = wires[couplers_[coupler].wire];
      const auto run = runOutOf(edge, coupler);
      if (!run.has_value() || resonator.way.empty()) {
        continue;
      }
      const auto nearest = [&](const std::int64_t px, const std::int64_t py) {
        double best = std::numeric_limits<double>::max();
        for (const auto& cell : resonator.way) {
          best = std::min(best, std::hypot(static_cast<double>(cell.x - px),
                                           static_cast<double>(cell.y - py)));
        }
        return best;
      };
      const auto open = [&](const std::int64_t px, const std::int64_t py) {
        if (px < 0 || py < 0 || px >= width || py >= height) {
          return false;
        }
        const auto cell = static_cast<std::size_t>((py * width) + px);
        if (scene_.blocked.test(cell) || field_.owner(cell) != NO_OWNER) {
          return false;
        }
        corridor_.set(cell, false);
        return true;
      };
      // As far as the resonator's clearance reaches, and never shorter than
      // the run: a cell past the run is opened only while it is still
      // within that clearance.
      const auto most = run->steps + static_cast<std::int64_t>(limit);
      for (std::int64_t k = 0; k <= most; ++k) {
        const auto px = static_cast<std::int64_t>(run->x) + (k * run->dx);
        const auto py = static_cast<std::int64_t>(run->y) + (k * run->dy);
        const bool within = nearest(px, py) < limit;
        if (!within) {
          if (k > run->steps) {
            break;
          }
          continue;
        }
        if (!open(px, py)) {
          break;
        }
        static_cast<void>(open(px - run->dy, py + run->dx));
        static_cast<void>(open(px + run->dy, py - run->dx));
      }
    }
  }

  /// How far from the anchor of the coupler two wires share the rule does
  /// not bind between them (`meetAt`). Under `narrowMeeting`, in the
  /// feedline routing (user, 2026-10-13):
  ///
  /// - Two edges of one coupler meet on the pad only: half the coupling
  ///   length and a clearance. Beyond it they keep the rule, as the
  ///   insertion holds them to it.
  /// - A resonator meets the edges of its own coupler on its lead only
  ///   (`onLead`). Past the lead it keeps the rule from them, the terminal
  ///   one included.
  /// - An edge meets its resonator only along the edge's run out of its
  ///   port on the pad (`meetAt`, `runOutOf`), so that the resonator's
  ///   clearance does not close the edge's stub and first moves.
  [[nodiscard]] double meetingReach(const Wire& one, const Wire& two,
                                    const bool onLead) const {
    if (!feedlinePass_) {
      return couplerReach();
    }
    if (one.feedline && two.feedline) {
      return (0.5 * static_cast<double>(tuning_.couplerLength)) +
             static_cast<double>(tuning_.clearance);
    }
    if (one.resonator && two.feedline && onLead) {
      return couplerReach();
    }
    return 0.0;
  }

  /// The first pair of cells of two feedline edges that meet at a coupler on
  /// `option` that the feedline routing holds against them: closer than the
  /// rule, with the middle of the pair outside the meeting `meetAt` grants
  /// two edges there — half the coupling length and a clearance from the
  /// anchor, or one and a half clearances from either port where the two
  /// ports stand within the rule of each other. Nothing when every pair
  /// passes or either way is empty.
  [[nodiscard]] std::optional<std::pair<PathPoint, PathPoint>>
  tooClose(const Path& one, const Path& two,
           const CouplerOption& option) const {
    const auto limit = static_cast<double>(tuning_.spacing * tuning_.spacing);
    const auto link = static_cast<double>(tuning_.clearance);
    const auto reach =
        (0.5 * static_cast<double>(tuning_.couplerLength)) + link;
    const auto from = [](const double x, const double y, const PathPoint& at) {
      return std::hypot(x - static_cast<double>(at.x),
                        y - static_cast<double>(at.y));
    };
    const bool portsMeet = from(option.in.x, option.in.y, option.out) <= link;
    for (const auto& a : one) {
      for (const auto& b : two) {
        const auto dx = static_cast<double>(a.x) - static_cast<double>(b.x);
        const auto dy = static_cast<double>(a.y) - static_cast<double>(b.y);
        if ((dx * dx) + (dy * dy) > limit) {
          continue;
        }
        const auto x = 0.5 * (static_cast<double>(a.x) + b.x);
        const auto y = 0.5 * (static_cast<double>(a.y) + b.y);
        if (from(x, y, option.anchor) <= reach ||
            (portsMeet && (from(x, y, option.in) <= 1.5 * link ||
                           from(x, y, option.out) <= 1.5 * link))) {
          continue;
        }
        return std::pair{a, b};
      }
    }
    return std::nullopt;
  }

  /// The coupler two wires share, or none: a resonator and an edge of its
  /// own coupler's chain, or the two edges that meet at one coupler.
  [[nodiscard]] static std::uint32_t sharedCoupler(const Wire& one,
                                                   const Wire& two) {
    for (const auto a : {one.couplerAtSource, one.couplerAtTarget}) {
      if (a == NO_OWNER) {
        continue;
      }
      if (a == two.couplerAtSource || a == two.couplerAtTarget) {
        return a;
      }
    }
    return NO_OWNER;
  }

  /// How far the meeting at a coupler reaches from its anchor, in cells:
  /// the body and the clearance around it.
  /// How far from a coupler two edges of its own chain may be within the
  /// rule of each other.
  ///
  /// They pin on one cell — the coupler's feedline port — so around that
  /// cell the rule cannot bind, and the disc of the rule itself is what it
  /// takes to let them both stand there. `couplerReach` is three times as
  /// far and is the wrong figure here: it is the reach of the coupler's
  /// *geometry*, the run and the depth and the turns, and on 17q it left
  /// sixty-two cells of every neighbouring edge fenced by two cells of
  /// copper instead of nineteen of clearance — the thin fence visible at
  /// the start of every edge in the pictures.
  /// The heading the launcher nearest a cell faces.
  ///
  /// What a coupler's orientation is counted from. The heading the
  /// resonator's way happens to hold at the insertion point is a property
  /// of the routing, not of the chip: it changes whenever the outer routing
  /// changes, and offset 0 then means something different on every coupler.
  /// The nearest launcher's own heading is a fact of the chip, so offset 0
  /// is the same thing everywhere — the resonator leaves its coupler the
  /// way the launcher that drives it points.
  [[nodiscard]] Heading nearestLauncherHeading(const std::int64_t x,
                                               const std::int64_t y) const {
    // A tie goes to the lower port number, so that the answer does not
    // depend on the order the hash map holds its launchers in.
    Heading heading = 0;
    double nearest = std::numeric_limits<double>::max();
    std::uint32_t nearestPort = std::numeric_limits<std::uint32_t>::max();
    for (const auto& [port, slot] : scene_.launcherCell) {
      const auto found = scene_.launcherHeading.find(port);
      if (found == scene_.launcherHeading.end()) {
        continue;
      }
      const auto place = scene_.router.cell(slot);
      const auto distance = std::hypot(static_cast<double>(place.x()) - x,
                                       static_cast<double>(place.y()) - y);
      if (distance < nearest || (distance == nearest && port < nearestPort)) {
        nearest = distance;
        nearestPort = port;
        heading = found->second;
      }
    }
    return heading;
  }

  [[nodiscard]] double couplerMeeting() const {
    return static_cast<double>(tuning_.clearance);
  }

  [[nodiscard]] double couplerReach() const {
    return static_cast<double>(tuning_.couplerLength + tuning_.couplerHeight +
                               (2 * BEND_RADIUS)) +
           (1.5 * static_cast<double>(tuning_.clearance));
  }

  /// The same for a way the wire does not have yet.
  ///
  /// The guard is only the first question. What the search keeps is nineteen
  /// whole cells and what the rule asks for is 185 layout units, which is
  /// 18.5 of them, so a cell the search refuses is not always a cell the rule
  /// refuses. A committed wire is judged by the rule, in layout units, which
  /// is the figure the design-rule check uses — otherwise the rounds chase
  /// encounters that nothing will ever report.
  [[nodiscard]] std::uint32_t
  conflictsIn(const Path& way, const Wire& wire, const std::vector<Wire>& wires,
              std::vector<std::uint32_t>* blockers = nullptr,
              std::vector<PathPoint>* where = nullptr) {
    // An edge of a chain is crossed by the wires between its couplers on
    // purpose, so the rule does not bind between it and a conventional or
    // an inner wire. Against a resonator it binds: a feedline keeps the
    // clearance from every resonator but its own coupler's, and there only
    // within the coupler's reach. Against another edge it binds too, so
    // that no feedline is ever drawn along another.
    const bool crossable = wire.feedline;
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto height = static_cast<std::int64_t>(scene_.router.height);
    const auto limit = tuning_.spacing * tuning_.spacing;
    const auto& stencil = stencilFor(tuning_.clearance);
    std::uint32_t found = 0;
    for (std::size_t at = 0; at < way.size(); ++at) {
      const auto& point = way[at];
      // A resonator's way begins with its lead (`applyOption`).
      const bool onLead = !wire.resonator || at < wire.arc.size();
      if (point.x >= scene_.router.width || point.y >= scene_.router.height) {
        continue;
      }
      const auto cell =
          (static_cast<std::size_t>(point.y) * scene_.router.width) + point.x;
      if (!field_.guarded(cell)) {
        continue;
      }
      // Somebody guards it. Whether the rule is broken is a question of how
      // far away that wire's copper really is, and of whether the two meet
      // there — which is the question the design-rule check asks, pair by
      // pair, and it has to be asked the same way here.
      for (const auto& [dx, dy] : stencil.full) {
        if (static_cast<double>((dx * dx) + (dy * dy)) > limit) {
          continue;
        }
        const auto x = static_cast<std::int64_t>(point.x) + dx;
        const auto y = static_cast<std::int64_t>(point.y) + dy;
        if (x < 0 || y < 0 || x >= width || y >= height) {
          continue;
        }
        const auto owner =
            field_.owner(static_cast<std::size_t>((y * width) + x));
        if (owner == NO_OWNER || owner == wire.key || owner >= wires.size()) {
          continue;
        }
        const auto& other = wires[owner];
        // A plain wire is crossed by the edge it bridges, under the
        // crossing rule, and keeps the rule from every other edge (user,
        // 2026-10-13); under `wireFeedlineClearance` off, from none.
        const auto crossedBy = [](const Wire& plain, const Wire& edge) {
          return plain.inner || plain.bridged == edge.key;
        };
        if ((crossable && !other.resonator && !other.feedline &&
             crossedBy(other, wire)) ||
            (other.feedline && !wire.resonator && !wire.feedline &&
             crossedBy(wire, other))) {
          continue;
        }
        if (meetAt(wire, wires[owner], 0.5 * (point.x + x), 0.5 * (point.y + y),
                   onLead)) {
          continue;
        }
        ++found;
        if (blockers != nullptr &&
            std::ranges::find(*blockers, owner) == blockers->end()) {
          blockers->push_back(owner);
          if (where != nullptr) {
            where->push_back(point);
          }
        }
        break;
      }
    }
    return found;
  }

  /// Whether a way breaks a feedline rule that the search cannot hold by
  /// itself (user, 2026-10-13).
  ///
  /// The stubs of a wire, the straight runs at its source and its target
  /// that the router lays without searching them (`assemble`), and at least
  /// the rule's straight length at either end, cross no feedline edge, at a
  /// right angle or not. A resonator's source is the
  /// end of its lead, which meets the edges of its own coupler on purpose,
  /// so only its target stub counts. A resonator crosses no feedline edge
  /// between couplers, its own coupler's included, beyond its lead; only
  /// the terminal edges, which the feedline routing lays again, stay open to
  /// it.
  [[nodiscard]] bool breaksAFeedlineRule(const Wire& wire, const Path& way,
                                         const std::uint32_t startStub,
                                         const std::vector<Wire>& wires) const {
    if (wire.feedline || wire.inner || way.empty()) {
      return false;
    }
    const auto run = [&way](const std::size_t from, const std::size_t to) {
      return Path(way.begin() + static_cast<std::ptrdiff_t>(from),
                  way.begin() + static_cast<std::ptrdiff_t>(to));
    };
    const auto cells = way.size();
    // How many points of the way, from one end, span `length` cells.
    const auto span = [&way, cells](const double length, const bool fromEnd) {
      double walked = 0.0;
      std::size_t taken = 1;
      while (taken < cells && walked < length) {
        const auto& a = way[fromEnd ? cells - taken : taken - 1];
        const auto& b = way[fromEnd ? cells - taken - 1 : taken];
        walked += std::hypot(static_cast<double>(a.x) - b.x,
                             static_cast<double>(a.y) - b.y);
        ++taken;
      }
      return taken;
    };
    // A stub is at least the rule's straight length, the port run of the
    // capacity graph, also where the router is not held to one.
    const auto rule = static_cast<double>(tuning_.straightStart);
    const auto head =
        wire.resonator
            ? Path{}
            : run(0,
                  span(std::max(rule, static_cast<double>(startStub)), false));
    const auto tail = run(
        cells - span(std::max(rule, static_cast<double>(wire.endStub)), true),
        cells);
    const auto searched =
        wire.resonator ? run(std::min(cells, wire.arc.size()), cells) : Path{};
    for (const auto& edge : edges_) {
      if (edge.wire >= wires.size()) {
        continue;
      }
      const auto& other = wires[edge.wire];
      if (!other.drawn || other.way.empty()) {
        continue;
      }
      if ((!head.empty() && waysCross(head, other.way)) ||
          (!tail.empty() && waysCross(tail, other.way)) ||
          (!edge.terminal && !searched.empty() &&
           waysCross(searched, other.way))) {
        return true;
      }
    }
    return false;
  }

  /// Whether a feedline edge crosses another edge of its own chain
  /// (`waysCross`). The rule does not bind between two edges at the coupler
  /// they share (`meetAt`), so `conflictsOf` does not see such a crossing.
  [[nodiscard]] bool crossesItsChain(const Wire& wire,
                                     const std::vector<Wire>& wires) const {
    if (!wire.feedline || !wire.drawn || wire.edge >= edges_.size()) {
      return false;
    }
    const auto chain = edges_[wire.edge].chain;
    return std::ranges::any_of(edges_, [&](const Edge& edge) {
      if (edge.chain != chain || edge.wire >= wires.size()) {
        return false;
      }
      const auto& other = wires[edge.wire];
      return other.key != wire.key && other.drawn &&
             waysCross(wire.way, other.way);
    });
  }

  /// Whether two ways cross: they share a cell, or they are two diagonal
  /// steps through one unit square the other way round.
  ///
  /// The second case shares no cell at all — a step from (x,y) to
  /// (x+1,y+1) and one from (x+1,y) to (x,y+1) pass through each other
  /// without ever standing on the same pixel — and a test that only
  /// compares cells reports such a pair as clear.
  [[nodiscard]] static bool waysCross(const Path& one, const Path& two) {
    std::unordered_set<std::uint64_t> cells;
    cells.reserve(one.size() * 2);
    const auto cellKey = [](const PathPoint& at) {
      return (static_cast<std::uint64_t>(at.x) << 32U) | at.y;
    };
    for (const auto& at : one) {
      cells.insert(cellKey(at));
    }
    for (const auto& at : two) {
      if (cells.contains(cellKey(at))) {
        return true;
      }
    }
    // The diagonals of each unit square a way steps across, by the square's
    // lower-left corner and which way the step leans.
    const auto diagonals = [](const Path& way) {
      std::unordered_set<std::uint64_t> seen;
      for (std::size_t at = 0; at + 1 < way.size(); ++at) {
        const auto fromX = static_cast<std::int64_t>(way[at].x);
        const auto fromY = static_cast<std::int64_t>(way[at].y);
        const auto dx = static_cast<std::int64_t>(way[at + 1].x) - fromX;
        const auto dy = static_cast<std::int64_t>(way[at + 1].y) - fromY;
        if (dx == 0 || dy == 0 || dx * dx != 1 || dy * dy != 1) {
          continue;
        }
        const auto cornerX = std::min(fromX, fromX + dx);
        const auto cornerY = std::min(fromY, fromY + dy);
        const std::uint64_t lean = (dx * dy > 0) ? 1U : 0U;
        seen.insert((static_cast<std::uint64_t>(cornerX) << 33U) |
                    (static_cast<std::uint64_t>(cornerY) << 1U) | lean);
      }
      return seen;
    };
    const auto mine = diagonals(one);
    if (mine.empty()) {
      return false;
    }
    for (const auto key : diagonals(two)) {
      if (mine.contains(key ^ 1U)) {
        return true;
      }
    }
    return false;
  }

  /// Whether any feedline edge crosses a resonator.
  ///
  /// Past its lead a resonator keeps no room from a feedline — see
  /// `RESONATOR_COPPER` — but it is not crossable, and that is what this
  /// counts. The lead itself is left out: an edge docks on its coupler
  /// there, through the terminal slot, and `checkFeedlineRoom` is what has
  /// something to say about that stretch.
  [[nodiscard]] std::uint32_t
  checkResonatorCrossings(const std::vector<Wire>& wires,
                          const std::string_view stage = "coupler insertion") {
    std::uint32_t pairs = 0;
    std::string named;
    for (const auto& edge : edges_) {
      if (edge.wire >= wires.size()) {
        continue;
      }
      const auto& feedline = wires[edge.wire];
      if (!feedline.feedline || feedline.way.empty()) {
        continue;
      }
      for (const auto& coupler : couplers_) {
        const auto& option = coupler.options[coupler.chosen];
        const auto& way = wayOfResonator(wires[coupler.wire]);
        if (way.size() <= option.arc.size()) {
          continue;
        }
        const Path tail(way.begin() +
                            static_cast<std::ptrdiff_t>(option.arc.size()),
                        way.end());
        if (!waysCross(feedline.way, tail)) {
          continue;
        }
        ++pairs;
        if (pairs <= 12) {
          named += std::format("{}f{} x {}", named.empty() ? "" : " · ",
                               feedline.slot, wireId(wires[coupler.wire]));
        }
      }
    }
    say(std::format("{}: CHECK resonator crossings — {} "
                    "feedline{} cross a resonator{}{}",
                    stage, pairs, pairs == 1 ? "" : "s",
                    pairs == 0 ? "; the check is GREEN" : ": ", named));
    return pairs;
  }

  /// Whether any two edges that meet at a coupler cross each other.
  ///
  /// They pin on the same coupler on purpose, so no clearance binds between
  /// them there and `conflictsIn` says nothing about the pair. Crossing is
  /// still not allowed, and with the later edges open it became possible:
  /// the skip that opens the later edges of a chain opened the neighbour
  /// with them, and the edge leaving a coupler was drawn through the edge
  /// arriving at it (user, 2026-10-03).
  [[nodiscard]] std::uint32_t
  checkCouplerCrossings(const std::vector<Wire>& wires,
                        const std::string_view stage = "coupler insertion") {
    std::uint32_t pairs = 0;
    std::string named;
    for (std::uint32_t chain = 0; chain < chains_.size(); ++chain) {
      const auto& edges = edgeWireOf_[chain];
      for (std::size_t at = 0; at + 1 < edges.size(); ++at) {
        if (edges[at] >= wires.size() || edges[at + 1] >= wires.size()) {
          continue;
        }
        const auto& one = wires[edges[at]];
        const auto& two = wires[edges[at + 1]];
        if (one.way.empty() || two.way.empty() ||
            !waysCross(one.way, two.way)) {
          continue;
        }
        ++pairs;
        if (pairs <= 12) {
          named +=
              std::format("{}f{} x f{} (chain {})", named.empty() ? "" : " · ",
                          one.slot, two.slot, chain);
        }
      }
    }
    say(std::format("{}: CHECK coupler crossings — {} pair{} "
                    "of edges meeting at a coupler cross{}{}",
                    stage, pairs, pairs == 1 ? "" : "s",
                    pairs == 0 ? "; the check is GREEN" : ": ", named));
    return pairs;
  }

  /// Whether every committed coupler lies wholly inside the box, measured on
  /// all three pieces of copper it puts on the chip.
  ///
  /// **The body alone is the wrong half.** The feedline runs along the pad's
  /// *far* edge, `halfDepth + 1` cells across the centre, so the body always
  /// lies inside the run and a check on the body says GREEN while the run
  /// sits on the box edge. On 17q that is five couplers of seventeen — 3 and
  /// 4 with their run on `maxX` 1347, 8 on `minY` 77, 12 on `minX` 77, 16
  /// four cells off `maxY` — and the edges f2, f8, f12/f13 and f17 then run
  /// the length of the chip along that line. So the check measures:
  ///
  /// - the **body**, `couplerLength` along the orientation by
  ///   `couplerHeight` across it, as `option.body` holds it;
  /// - the **run**, the pad's two feedline ports and the straight run the
  ///   chain is forced to make off each of them plus its turn — the pad's own
  ///   run at a coupler and not `straightStart`, see `edgeRunOffPort`;
  /// - the **lead**, the resonator's own quarter turn off the near edge and
  ///   the straight run after it, `option.arc`;
  /// - the **turn**, the `BEND_RADIUS` cells beyond the lead's tip on the
  ///   heading it holds — the room the resonator's search needs to bend at
  ///   all, which is the same thing the run's straight stubs need.
  ///
  /// Per piece: how many couplers reach outside, the worst overshoot in
  /// cells, and — so that the line carries a measured figure when it is
  /// green — how far the piece that comes nearest a side of the box still
  /// stays off it. A slack of 0 is a piece *on* the box edge: legal, and one
  /// cell from not being. The box side is the first cell a launcher's stub
  /// and its clearance leave open, so a coupler there keeps the design rule
  /// from that stub by exactly one cell and leaves the wire off that
  /// launcher nothing to turn in.
  ///
  /// Said once, at the end of the insertion: a coupler does not move again —
  /// the feedline passes redraw edges and resonators, never a pad — so a
  /// second line in the refinement would re-print this one rather than
  /// re-measure it.
  [[nodiscard]] std::uint32_t
  checkCouplerBodies(const std::vector<Wire>& wires,
                     const std::string_view stage = "coupler insertion") {
    /// One piece of copper, measured against the box.
    struct Piece {
      std::string_view what;
      std::uint32_t outside = 0;
      std::int64_t worst = 0;
      std::string named;
      std::int64_t tightest = std::numeric_limits<std::int64_t>::max();
      std::string closest;
    };
    std::array<Piece, 4> pieces{Piece{.what = "body"}, Piece{.what = "run"},
                                Piece{.what = "lead"}, Piece{.what = "turn"}};
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    // How far a cell reaches past a side of the box; negative inside. The
    // slack reported is to the box side itself, which is the figure a
    // picture shows: zero is a piece *on* the edge, which the rule allows.
    const auto past = [this](const std::int64_t x, const std::int64_t y) {
      return std::max({couplerBox_.minX - x, x - couplerBox_.maxX,
                       couplerBox_.minY - y, y - couplerBox_.maxY});
    };
    const auto measure =
        [&](Piece& piece, const std::string& id,
            const std::vector<std::pair<std::int64_t, std::int64_t>>& at) {
          std::int64_t over = 0;
          std::int64_t overX = 0;
          std::int64_t overY = 0;
          for (const auto& [x, y] : at) {
            const auto out = past(x, y);
            if (out > over) {
              over = out;
              overX = x;
              overY = y;
            }
            if (-past(x, y) < piece.tightest) {
              piece.tightest = -past(x, y);
              piece.closest = std::format("'{}' at ({},{})", id, x, y);
            }
          }
          if (over <= 0) {
            return;
          }
          ++piece.outside;
          piece.worst = std::max(piece.worst, over);
          if (piece.outside <= 8) {
            piece.named +=
                std::format("{}'{}' by {} cell{} at ({},{})",
                            piece.named.empty() ? "" : " · ", id, over,
                            over == 1 ? "" : "s", overX, overY);
          }
        };

    for (const auto& coupler : couplers_) {
      if (coupler.wire >= wires.size() ||
          coupler.chosen >= coupler.options.size()) {
        continue;
      }
      const auto& option = coupler.options[coupler.chosen];
      const auto id = wireId(wires[coupler.wire]);

      std::vector<std::pair<std::int64_t, std::int64_t>> at;
      at.reserve(option.body.size());
      for (const auto cell : option.body) {
        at.emplace_back(static_cast<std::int64_t>(cell) % width,
                        static_cast<std::int64_t>(cell) / width);
      }
      measure(pieces[0], id, at);

      // The run: the line through the two feedline ports, carried
      // `straightStart` past each of them on the orientation. Built the way
      // `makeOption` builds it, off the ports rather than off the centre, so
      // that a diagonal orientation is measured where its copper is.
      at.clear();
      const auto along = routing::headingVector(option.orientation);
      const auto fromX = static_cast<std::int64_t>(option.in.x);
      const auto fromY = static_cast<std::int64_t>(option.in.y);
      // The span from port to port, read off the two ports themselves.
      // **Not `option.run`**: that is `cellsOn(couplerLength, orientation)`
      // and the ports sit at `± cellsOn(...) / 2` of the centre, so on an odd
      // run the two differ by a cell and the check reported a cell of
      // overshoot that `makeOption` had never allowed (17q coupler '40').
      const auto span =
          std::max(std::abs(static_cast<std::int64_t>(option.out.x) - fromX),
                   std::abs(static_cast<std::int64_t>(option.out.y) - fromY));
      // The same figure `terminalsInBox` and `corridorOfEdge` use, which at
      // a coupler port is the pad's own run and not `straightStart` — see
      // `edgeRunOffPort`.
      const auto off = edgeRunOffPort();
      for (std::int64_t k = -off; k <= span + off; ++k) {
        at.emplace_back(fromX + (k * along.dx), fromY + (k * along.dy));
      }
      measure(pieces[1], id, at);

      at.clear();
      at.reserve(option.arc.size());
      for (const auto& step : option.arc) {
        at.emplace_back(static_cast<std::int64_t>(step.x),
                        static_cast<std::int64_t>(step.y));
      }
      measure(pieces[2], id, at);

      // The quarter turn the resonator's search may want off the lead's tip.
      at.clear();
      const auto onward = routing::headingVector(option.arcEnd.heading);
      const auto beyondTip = static_cast<std::int64_t>(BEND_RADIUS);
      for (std::int64_t k = 1; k <= beyondTip; ++k) {
        at.emplace_back(
            static_cast<std::int64_t>(option.arcEnd.x) + (k * onward.dx),
            static_cast<std::int64_t>(option.arcEnd.y) + (k * onward.dy));
      }
      measure(pieces[3], id, at);
    }

    std::uint32_t outside = 0;
    std::string line;
    for (auto& piece : pieces) {
      outside += piece.outside;
      if (piece.closest.empty()) {
        piece.tightest = 0;
        piece.closest = "nothing";
      }
      line += std::format("{}{} {} outside (worst {}, tightest {} off, {}){}",
                          line.empty() ? "" : "; ", piece.outside, piece.what,
                          piece.worst, piece.tightest, piece.closest,
                          piece.named.empty() ? "" : ": " + piece.named);
    }
    say(std::format("{}: CHECK coupler bodies — of {} coupler{} against the "
                    "box x {}..{} y {}..{}: {}{}",
                    stage, couplers_.size(), couplers_.size() == 1 ? "" : "s",
                    couplerBox_.minX, couplerBox_.maxX, couplerBox_.minY,
                    couplerBox_.maxY, line,
                    outside == 0 ? "; the check is GREEN" : ""));
    return outside;
  }

  /// The pairs of drawn edges of **different** chains whose ways cross, by
  /// edge index, in edge order. Two edges of one chain are
  /// `checkCouplerCrossings`'s business where they meet at a coupler and the
  /// clearance rule's elsewhere; two edges of different chains have no
  /// business meeting at all. Prefiltered by the boxes the two ways span, so
  /// 69q's 81 edges are 3240 box tests and a handful of `waysCross`.
  [[nodiscard]] std::vector<std::pair<std::size_t, std::size_t>>
  feedlineCrossingPairs(const std::vector<Wire>& wires) const {
    std::vector<std::pair<std::size_t, std::size_t>> pairs;
    std::vector<Span> boxes(edges_.size());
    std::vector<bool> drawn(edges_.size(), false);
    for (std::size_t at = 0; at < edges_.size(); ++at) {
      const auto key = edges_[at].wire;
      if (key < wires.size() && !wires[key].way.empty()) {
        boxes[at] = reachOf(wires[key].way);
        drawn[at] = true;
      }
    }
    for (std::size_t i = 0; i < edges_.size(); ++i) {
      if (!drawn[i]) {
        continue;
      }
      for (std::size_t j = i + 1; j < edges_.size(); ++j) {
        if (!drawn[j] || edges_[j].chain == edges_[i].chain ||
            !boxes[i].meets(boxes[j]) ||
            !waysCross(wires[edges_[i].wire].way, wires[edges_[j].wire].way)) {
          continue;
        }
        pairs.emplace_back(i, j);
      }
    }
    return pairs;
  }

  /// **The fourth guarantee** (2026-10-04): no feedline crosses a feedline
  /// of another chain. The three checks before it look at leads, couplers
  /// and resonators, and `final routing: … crossing a feedline` counts plain
  /// wires only, so on 69q the terminal edges of four pairs of neighbouring
  /// chains — f32/f33, f46/f47, f60/f61, f73/f74 — crossed each other twice
  /// each and no line said so (`artifacts/logs/fcross.py`, R4 of the room
  /// rules). `SCPD_EDGE_SEES_CHAINS` is off, so the insertion never routes
  /// an edge against another chain; this is where that shows. The same
  /// `waysCross` the coupler check uses, which finds the two diagonals
  /// through one unit square as well as a shared cell.
  ///
  /// Said twice: at the end of the insertion, where the four 69q pairs
  /// stand, and at the end of phase 4 after the repair (`feedline routing:`),
  /// where the pass has redrawn the terminal edges — the line a chip is
  /// judged by is the second.
  ///
  /// @returns How many pairs of edges of different chains cross.
  [[nodiscard]] std::uint32_t checkFeedlineCrossings(
      const std::vector<Wire>& wires,
      const std::string_view stage = "coupler insertion") const {
    const auto pairs = feedlineCrossingPairs(wires);
    std::string named;
    for (std::size_t k = 0; k < pairs.size() && k < 12; ++k) {
      const auto& one = edges_[pairs[k].first];
      const auto& two = edges_[pairs[k].second];
      named += std::format("{}{} (chain {}) x {} (chain {})",
                           named.empty() ? "" : " · ", wireId(wires[one.wire]),
                           one.chain, wireId(wires[two.wire]), two.chain);
    }
    say(std::format("{}: CHECK feedline crossings — {} pair{} of edges of "
                    "different chains cross{}{}",
                    stage, pairs.size(), pairs.size() == 1 ? "" : "s",
                    pairs.empty() ? "; the check is GREEN" : ": ", named));
    return static_cast<std::uint32_t>(pairs.size());
  }

  /// **No feedline crosses an edge of its own chain** (user, 2026-10-10).
  /// `checkFeedlineCrossings` looks at different chains and
  /// `checkCouplerCrossings` at the two edges that meet at a coupler; this
  /// is every other pair of one chain. On 69q chain 2's f13 and f15 shared
  /// two cells while the terminal edges were fenced only for their
  /// neighbours, and no line said so. The same `waysCross` as the check of
  /// different chains, prefiltered by the boxes the two ways span.
  ///
  /// @returns How many pairs of edges of one chain cross.
  [[nodiscard]] std::uint32_t checkChainCrossings(
      const std::vector<Wire>& wires,
      const std::string_view stage = "coupler insertion") const {
    std::string named;
    std::size_t pairs = 0;
    for (std::size_t i = 0; i < edges_.size(); ++i) {
      const auto& one = edges_[i];
      if (one.wire >= wires.size() || wires[one.wire].way.empty()) {
        continue;
      }
      for (std::size_t j = i + 1; j < edges_.size(); ++j) {
        const auto& two = edges_[j];
        if (two.chain != one.chain || two.wire >= wires.size() ||
            wires[two.wire].way.empty() || one.to == two.from ||
            two.to == one.from ||
            !reachOf(wires[one.wire].way).meets(reachOf(wires[two.wire].way)) ||
            !waysCross(wires[one.wire].way, wires[two.wire].way)) {
          continue;
        }
        ++pairs;
        if (pairs <= 12) {
          named += std::format(
              "{}{} x {} (chain {})", named.empty() ? "" : " · ",
              wireId(wires[one.wire]), wireId(wires[two.wire]), one.chain);
        }
      }
    }
    say(std::format("{}: CHECK chain crossings — {} pair{} of edges of one "
                    "chain cross{}{}",
                    stage, pairs, pairs == 1 ? "" : "s",
                    pairs == 0 ? "; the check is GREEN" : ": ", named));
    return static_cast<std::uint32_t>(pairs);
  }

  /// **The check the coupler insertion is handed over on**: no feedline may
  /// come closer to what the corridor fences than the design rule allows.
  ///
  /// It measures **what is actually routed against**, not a rule of its own
  /// (user, 2026-10-03). That is three things and nothing else, the same
  /// three `corridorOfEdge` builds:
  ///
  /// 1. What is fenced — every coupler's lead.
  /// 2. Inflated by the design rule, taken unrounded in layout units as
  ///    `conflictsIn` takes it, so a cell the whole-cell stencil refuses is
  ///    not always a cell the rule refuses.
  /// 3. Less the terminal slots, which are the one thing the corridor opens
  ///    again and are opened here by the same figures — see `terminalSlot`.
  ///
  /// So a green line means the insertion built what it said it would, and a
  /// red one means an edge sits somewhere the corridor had closed. It is no
  /// longer a second opinion about the design rule; `checkClearance` in the
  /// DRC is that, and it has the whole chip to look at rather than this
  /// stage's own fence.
  ///
  /// @returns How many feedline/coupler pairs break it.
  [[nodiscard]] std::uint32_t
  checkFeedlineRoom(std::vector<Wire>& wires,
                    const std::string_view stage = "coupler insertion") {
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto height = static_cast<std::int64_t>(scene_.router.height);
    // What the corridor closes, by coupler, so a pair can be named.
    std::unordered_map<std::size_t, std::uint32_t> fenced;
    for (std::uint32_t index = 0; index < couplers_.size(); ++index) {
      const auto& coupler = couplers_[index];
      const auto& option = coupler.options[coupler.chosen];
      const Path& path = option.arc;
      for (const auto& point : path) {
        if (point.x < scene_.router.width && point.y < scene_.router.height) {
          fenced.emplace(scene_.router.index(point.x, point.y), index);
        }
      }
    }
    const auto limit = tuning_.spacing * tuning_.spacing;
    const auto& stencil = stencilFor(tuning_.clearance);
    const auto turn = static_cast<std::uint32_t>(BEND_RADIUS);
    std::uint32_t pairs = 0;
    std::string named;
    for (const auto& edge : edges_) {
      if (edge.wire >= wires.size()) {
        continue;
      }
      const auto& wire = wires[edge.wire];
      if (!wire.feedline || wire.way.empty()) {
        continue;
      }
      // The slots this edge docks in, by the figures the corridor used.
      std::unordered_set<std::size_t> slot;
      const auto [startRun, endRun] = runsOfEdge(edge);
      const auto keep = [&](const Path& run) {
        for (const auto& point : run) {
          if (point.x < scene_.router.width && point.y < scene_.router.height) {
            slot.insert(scene_.router.index(point.x, point.y));
          }
        }
      };
      keep(router_.straightStub(
          wire.objective.source, false,
          std::max(tuning_.terminalSlot, startRun + turn)));
      keep(router_.straightStub(wire.objective.target, true,
                                std::max(tuning_.terminalSlot, endRun + turn)));
      // One pair per coupler, however many cells of the way reach it.
      std::unordered_set<std::uint32_t> told;
      for (const auto& point : wire.way) {
        if (point.x >= scene_.router.width || point.y >= scene_.router.height) {
          continue;
        }
        if (slot.contains(scene_.router.index(point.x, point.y))) {
          continue;
        }
        for (const auto& [dx, dy] : stencil.full) {
          if (static_cast<double>((dx * dx) + (dy * dy)) > limit) {
            continue;
          }
          const auto x = static_cast<std::int64_t>(point.x) + dx;
          const auto y = static_cast<std::int64_t>(point.y) + dy;
          if (x < 0 || y < 0 || x >= width || y >= height) {
            continue;
          }
          const auto found =
              fenced.find(static_cast<std::size_t>((y * width) + x));
          if (found == fenced.end() || !told.insert(found->second).second) {
            continue;
          }
          ++pairs;
          if (pairs <= 12) {
            named += std::format(
                "{}f{} vs {} at ({},{})", named.empty() ? "" : " · ", wire.slot,
                wireId(wires[couplers_[found->second].wire]), point.x, point.y);
          }
        }
      }
    }
    say(std::format("{}: CHECK feedline room — {} feedline/{} pair{} "
                    "closer than the rule{}{}",
                    stage, pairs, "lead", pairs == 1 ? "" : "s",
                    pairs == 0 ? "; the check is GREEN" : ": ", named));
    return pairs;
  }

  /// Which wires an open wire lies in the room of, by name. The wire has to
  /// be off the canvas when this runs, as `conflictsIn` asks.
  [[nodiscard]] std::string whoBlocks(const Wire& wire,
                                      const std::vector<Wire>& wires) {
    std::vector<std::uint32_t> blockers;
    std::vector<PathPoint> where;
    conflictsIn(wire.way, wire, wires, &blockers, &where);
    std::string named;
    for (std::size_t at = 0; at < blockers.size(); ++at) {
      const auto& other = wires[blockers[at]];
      std::string note;
      if (at < where.size()) {
        const auto& cell = where[at];
        const auto far = [&cell](const PathPoint& end) {
          return std::hypot(static_cast<double>(cell.x) - end.x,
                            static_cast<double>(cell.y) - end.y);
        };
        const auto fromSource = far(wire.objective.source);
        const auto fromTarget = far(wire.objective.target);
        const auto stub = static_cast<double>(
            std::max(startStubOf(wire, tuning_.straightStart), wire.endStub));
        note = std::format(
            " at ({},{}), {:.0f} from its source and {:.0f} from its target, "
            "stub {:.0f}{}",
            cell.x, cell.y, fromSource, fromTarget, stub,
            std::min(fromSource, fromTarget) <= stub ? " — IN ITS OWN STUB"
                                                     : "");
      }
      named += (named.empty() ? "" : "; ") + wireId(other) + note;
    }
    return named.empty() ? std::string("-") : named;
  }

  /// The search itself, with the stubs of this pass. Its picture is taken
  /// here unless the caller has more to put into it.
  [[nodiscard]] Path search(const Wire& wire, const std::uint32_t straightStart,
                            const bool usePenalty, const bool draw = true) {
    router_.setParams({.startStraightLength = startStubOf(wire, straightStart),
                       .endStraightLength = wire.endStub,
                       .minRadius = BEND_RADIUS,
                       .bendPenalty = tuning_.bendPenalty});
    // The field this search is priced by, said every time rather than once.
    //
    // `routeEdge` hands the router a field of zeroes — an edge of a chain is
    // searched without a price — and it was never handed back. The router
    // keeps a pointer, so from the first edge of the coupler insertion
    // onward every later search read zeroes: the lane, the halos, the bands
    // along the feedlines and the toll were all written into a field nobody
    // looked at. It is why four sweeps over the prices moved nothing and why
    // a toll of twenty-seven bends a cell left the ways exactly where they
    // were (user, 2026-10-01).
    router_.attachWireProximity(&proximity_);
    router_.attachCorridor(&corridor_);
    // The crossing rule does not bind around a resonator's own coupler,
    // where the edges of its chain pin and run beside it on purpose.
    std::vector<std::uint32_t> exempt;
    if (feedlinePass_ && wire.couplerAtSource != NO_OWNER) {
      const auto& anchor = couplers_[wire.couplerAtSource].anchor;
      const auto reach = static_cast<std::int64_t>(std::ceil(couplerReach()));
      const auto width = static_cast<std::int64_t>(scene_.router.width);
      for (std::int64_t dy = -reach; dy <= reach; ++dy) {
        for (std::int64_t dx = -reach; dx <= reach; ++dx) {
          const auto x = static_cast<std::int64_t>(anchor.x) + dx;
          const auto y = static_cast<std::int64_t>(anchor.y) + dy;
          if (x < 0 || y < 0 || x >= width || y >= scene_.router.height ||
              std::hypot(static_cast<double>(dx), static_cast<double>(dy)) >
                  couplerReach()) {
            continue;
          }
          exempt.push_back(static_cast<std::uint32_t>((y * width) + x));
        }
      }
    }
    router_.setCrossingExemption(exempt);
    // A wire under the feedline constraints crosses a feedline at a right
    // angle only; a feedline edge itself is searched free, as the
    // prototype searches it.
    auto found = feedlinePass_ && !wire.feedline
                     ? router_.routeOrthogonal(wire.objective, usePenalty)
                     : router_.route(wire.objective, usePenalty);
    // A resonator drawn from its coupler leaves it on a fixed quarter turn
    // that no search sees; the way it has is the turn and what was found.
    if (!found.empty() && !wire.arc.empty()) {
      Path whole = wire.arc;
      whole.insert(whole.end(), found.begin(), found.end());
      // The search never saw the head, so a way that comes back over it is
      // a way that meets itself, which the router's own guard would have
      // refused: no way, by the same test.
      if (routing::pathSelfIntersects(whole, scene_.router.width,
                                      scene_.router.height, loopScratch_)) {
        found.clear();
      } else {
        found = std::move(whole);
      }
    }
    // The stubs are laid, not searched, and the closed copper keeps a
    // resonator off most edges but not off every corner: what the search
    // cannot hold by itself is refused here.
    if (feedlinePass_ && feedlineWires_ != nullptr && !found.empty() &&
        breaksAFeedlineRule(wire, found, startStubOf(wire, straightStart),
                            *feedlineWires_)) {
      found.clear();
    }
    if (draw) {
      recordSearch(wire, found, {});
    }
    return found;
  }

  /// The straight run a wire leaves its source on, in cells: its own figure
  /// where it has one, the pass's for a wire fed from the ring, and none
  /// for a resonator fed from the segment between two launchers.
  [[nodiscard]] static std::uint32_t
  startStubOf(const Wire& wire, const std::uint32_t straightStart) {
    if (wire.startStub.has_value()) {
      return *wire.startStub;
    }
    return wire.resonator ? 0U : straightStart;
  }

  /// The cells a search may enter: the free space within `reach` steps of the
  /// way the wire has.
  ///
  /// This is the prototype's `expand_path`: a walk over the cells that are not
  /// artwork, so the band follows the way rather than boxing it and does not
  /// reach around an obstacle. It knows nothing of other wires. What fences a
  /// search in is `fence`, and nothing else.
  void buildCorridor(const Wire& wire, const std::uint32_t asked) {
    // The prototype widens the band of the feedline pass by one band per
    // round (`FinalGrid.cpp:4254`), where its outer routing holds it at one.
    // A wire that found nothing in round 0 is offered more room in round 1
    // rather than the same room again, which is what stops the sweep from
    // repeating itself.
    const std::uint32_t reach =
        feedlinePass_ ? (frame_.round + 1U) * asked : asked;
    blockOnBodies_ = !wire.feedline;
    corridor_.fill(true);
    box_ = {};
    openBox_ = {};
    if (wire.way.empty()) {
      return;
    }
    // The band cannot leave the box the way spans grown by the reach, so
    // neither the walk nor anything that reads the band afterwards has to
    // touch the rest of the grid. On the 21-qubit chip a short wire's box is
    // a fortieth of it.
    box_ = boxOf(wire, reach);
    ++pass_;
    if (pass_ == 0) {
      std::ranges::fill(seen_, 0);
      pass_ = 1;
    }
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    queue_.clear();
    depth_.clear();

    const auto visit = [&](const std::int64_t x, const std::int64_t y,
                           const std::uint32_t depth) {
      if (x < box_.minX || y < box_.minY || x > box_.maxX || y > box_.maxY) {
        return;
      }
      const auto cell = static_cast<std::size_t>((y * width) + x);
      // A coupler body stands in the way of a wire that has to go around it,
      // but not of a feedline: the feedline is what the coupler is there to
      // meet, and it runs along the body on purpose.
      if (seen_[cell] == pass_ || scene_.blocked.test(cell) ||
          (blockOnBodies_ && bodies_.test(cell))) {
        return;
      }
      seen_[cell] = pass_;
      queue_.push_back(cell);
      depth_.push_back(depth);
      corridor_.set(cell, false);
      stretchOpenBox(x, y);
    };

    for (const auto& point : wire.way) {
      visit(point.x, point.y, 0);
    }
    for (std::size_t head = 0; head < queue_.size(); ++head) {
      const auto depth = depth_[head];
      if (depth >= reach) {
        continue;
      }
      const auto cell = queue_[head];
      const auto x = static_cast<std::int64_t>(cell % scene_.router.width);
      const auto y = static_cast<std::int64_t>(cell / scene_.router.width);
      for (std::int64_t dy = -1; dy <= 1; ++dy) {
        for (std::int64_t dx = -1; dx <= 1; ++dx) {
          if (dx != 0 || dy != 0) {
            visit(x + dx, y + dy, depth + 1);
          }
        }
      }
    }
    openFixedPlaces(wire);
  }

  /// Fence a search in: the ways of these wires, inflated by the clearance,
  /// are closed to it.
  ///
  /// This is the prototype's `mark_obstacles` with `min_dist_wires`: the disc
  /// of the clearance around every cell of a way, stamped by its leading
  /// edge. Where the wire and a fence wire meet — a terminal of one within
  /// the rule of a terminal of the other — the disc leaves the meeting open,
  /// which is the design-rule check's own
  /// exemption in its narrow form: the fence wire's copper itself stays
  /// closed. The wire's own fixed places are opened again last, because it
  /// has to be able to stand on them.
  void fence(const Wire& wire, std::initializer_list<const Wire*> others) {
    for (const Wire* const other : others) {
      if (other == &wire) {
        continue;
      }
      closeRoomOf(wire, *other, other->way);
      // The fixed places with it, where they are not in the way: the lead
      // and the straight run of a resonator the insertion has just spliced,
      // which its next search has to make and no neighbour may take. See
      // `fenceFixed`.
      closeRoomOf(wire, *other, other->fixed);
    }
    openFixedPlaces(wire);
  }

  /// Keep the fixed places of the wires let go of closed while everything
  /// else of them is crossable — the prototype's `ripped_heads`, for every
  /// wire.
  void fenceFixedPlaces(
      const Wire& wire, const std::vector<Wire>& wires,
      const std::vector<std::pair<std::uint32_t, bool>>& released) {
    for (const auto& [key, routed] : released) {
      closeRoomOf(wire, wires[key], wires[key].fixed);
    }
    openFixedPlaces(wire);
  }

  /// Close the clearance around some cells of another wire — its way or its
  /// fixed places — except where the two wires meet.
  void closeRoomOf(const Wire& wire, const Wire& other, const Path& cells) {
    if (cells.empty()) {
      return;
    }
    const auto& stencil = stencilFor(tuning_.clearance);
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto height = static_cast<std::int64_t>(scene_.router.height);
    const bool meeting = couldMeet(wire, other);
    // What the search draws of a resonator starts where its lead ends.
    const bool onLead = !wire.resonator;
    if (!meeting) {
      alongDisc(cells, stencil,
                [&](const std::int64_t x, const std::int64_t y) {
                  const auto cell = static_cast<std::size_t>((y * width) + x);
                  if (meeting && field_.owner(cell) != other.key &&
                      meetAt(wire, other, static_cast<double>(x),
                             static_cast<double>(y), onLead)) {
                    return;
                  }
                  corridor_.set(cell, true);
                });
      return;
    }
    // Where the two may meet, a cell stays open only when every pair it
    // forms with a cell of the other wire within the rule meets, judged at
    // the middle of the pair as `conflictsIn` judges the laid way: the
    // search may then draw no way the count holds against it.
    for (const auto& point : cells) {
      for (const auto& [dx, dy] : stencil.full) {
        const auto x = static_cast<std::int64_t>(point.x) + dx;
        const auto y = static_cast<std::int64_t>(point.y) + dy;
        if (x < 0 || y < 0 || x >= width || y >= height) {
          continue;
        }
        const auto cell = static_cast<std::size_t>((y * width) + x);
        if (field_.owner(cell) != other.key &&
            meetAt(wire, other, static_cast<double>(point.x) + (0.5 * dx),
                   static_cast<double>(point.y) + (0.5 * dy), onLead)) {
          continue;
        }
        corridor_.set(cell, true);
      }
    }
  }

  /// Close some cells of another wire by the length-point radius, except
  /// where the two meet and except within reach of the routed wire's own
  /// **fixed places** — its ends, the straight run out of its source and the
  /// run into its target. The prototype guards the two ends alone
  /// (`lenpt_covers_terminal_of`, `k + 2`) and falls back to a search without
  /// the constraint when a wire fails; here the constraint is hard, and the
  /// first version with the ends alone left 17q's wires 2 and 31 dead on
  /// arrival in every round of the outer routing: the length point of the
  /// resonator beside them lies 24 cells from their launcher stub, and a
  /// band cell 30 cells from the source still closed, inflated by `k`, the
  /// cells the search has to step onto behind the stub. A wire's fixed
  /// places are where it has to be; no band may close them or the cells a
  /// first move out of them sweeps.
  void closeBandCells(const Wire& wire, const Wire& other, const Path& cells,
                      const std::uint32_t k) {
    if (cells.empty() || k == 0) {
      return;
    }
    const double guard = std::max(static_cast<double>(k) + 2.0,
                                  1.5 * static_cast<double>(tuning_.clearance));
    const auto nearAnEnd = [&](const PathPoint& cell) {
      for (const auto* const end :
           {&wire.objective.source, &wire.objective.target}) {
        if (std::hypot(static_cast<double>(cell.x) - end->x,
                       static_cast<double>(cell.y) - end->y) <= guard) {
          return true;
        }
      }
      return !wire.fixed.empty() && distanceToWay(cell, wire.fixed) <= guard;
    };
    Path kept;
    kept.reserve(cells.size());
    for (const auto& cell : cells) {
      if (!nearAnEnd(cell)) {
        kept.push_back(cell);
      }
    }
    if (kept.empty()) {
      return;
    }
    const auto& stencil = stencilFor(k);
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const bool meeting = couldMeet(wire, other);
    alongDisc(kept, stencil, [&](const std::int64_t x, const std::int64_t y) {
      const auto cell = static_cast<std::size_t>((y * width) + x);
      if (meeting && field_.owner(cell) != other.key &&
          meetAt(wire, other, static_cast<double>(x), static_cast<double>(y))) {
        return;
      }
      corridor_.set(cell, true);
    });
  }

  /// The length-point clearance of the outer routing, for one search: for
  /// every fenced neighbour that is a resonator, its band inflated by
  /// `tuning_.lengthPointRadius` instead of the clearance; and when the wire
  /// being routed is itself a resonator, the stretch of each fenced neighbour
  /// that runs alongside its own band — its two marks projected onto the
  /// neighbour's way, the way between the projections — inflated by the
  /// same radius. Nothing under the feedline constraints, nothing at `k = 0`.
  /// The wire's own fixed places are opened again afterwards, as `fence`
  /// opens them.
  void closeLengthBands(const Wire& wire,
                        std::initializer_list<const Wire*> others,
                        const Pass& pass) {
    const auto k = tuning_.lengthPointRadius;
    if (pass.feedlines || k == 0 || !lengthPointActive_) {
      return;
    }
    for (const Wire* const other : others) {
      if (other == nullptr || other == &wire || other->way.empty()) {
        continue;
      }
      // One zone per neighbour (user, 2026-10-04): a resonator neighbour is
      // stamped with its own band and nothing else; the stretch alongside
      // the routed resonator's band is stamped only on a neighbour that has
      // no band of its own. A plain wire beside a routed resonator used to
      // carry both and showed two inflated zones in the pictures.
      if (other->resonator) {
        const auto marks = lengthMarksOf(*other);
        if (marks.valid) {
          closeBandCells(
              wire, *other,
              Path(other->way.begin() + static_cast<std::ptrdiff_t>(marks.lo),
                   other->way.begin() +
                       static_cast<std::ptrdiff_t>(marks.hi + 1)),
              k);
        }
        continue;
      }
      if (wire.resonator) {
        const auto marks = lengthMarksOf(wire);
        if (marks.valid) {
          const auto& lo = wire.way[marks.lo];
          const auto& hi = wire.way[marks.hi];
          std::size_t atLo = 0;
          std::size_t atHi = 0;
          double bestLo = std::numeric_limits<double>::max();
          double bestHi = std::numeric_limits<double>::max();
          for (std::size_t n = 0; n < other->way.size(); ++n) {
            const auto& cell = other->way[n];
            const auto dLo = std::hypot(static_cast<double>(cell.x) - lo.x,
                                        static_cast<double>(cell.y) - lo.y);
            const auto dHi = std::hypot(static_cast<double>(cell.x) - hi.x,
                                        static_cast<double>(cell.y) - hi.y);
            if (dLo < bestLo) {
              bestLo = dLo;
              atLo = n;
            }
            if (dHi < bestHi) {
              bestHi = dHi;
              atHi = n;
            }
          }
          const auto from = std::min(atLo, atHi);
          const auto to = std::max(atLo, atHi);
          closeBandCells(
              wire, *other,
              Path(other->way.begin() + static_cast<std::ptrdiff_t>(from),
                   other->way.begin() + static_cast<std::ptrdiff_t>(to + 1)),
              k);
        }
      }
    }
    openFixedPlaces(wire);
  }

  /// A wire's own fixed places are always enterable. They are where the wire
  /// has to be, and the search is the one pass that may stand on them.
  void openFixedPlaces(const Wire& wire) {
    for (const auto& point : wire.fixed) {
      if (point.x < scene_.router.width && point.y < scene_.router.height) {
        corridor_.setCell(point.x, point.y, false);
        stretchOpenBox(point.x, point.y);
      }
    }
  }

  /// Grow `openBox_` to hold a cell the corridor opens.
  void stretchOpenBox(const std::int64_t x, const std::int64_t y) {
    if (openBox_.empty()) {
      openBox_ = {.minX = x, .minY = y, .maxX = x, .maxY = y};
      return;
    }
    openBox_.minX = std::min(openBox_.minX, x);
    openBox_.minY = std::min(openBox_.minY, y);
    openBox_.maxX = std::max(openBox_.maxX, x);
    openBox_.maxY = std::max(openBox_.maxY, y);
  }

  /// Whether two wires meet anywhere: a terminal of one within the rule of a
  /// terminal of the other. The cheap half of `meetAt`, asked once per fence
  /// wire before every cell is asked.
  [[nodiscard]] bool couldMeet(const Wire& one, const Wire& two) const {
    // Two wires on one coupler meet there whatever their terminals do.
    //
    // This is the cheap gate `fence` asks once per fence wire before it asks
    // `meetAt` of every cell, and it used to ask about terminals alone. So
    // the coupler exemption `meetAt` carries was never reached from the
    // fence: the resonator of a coupler had its whole clearance disc closed
    // against the edge that leaves that same coupler, the edge's own start
    // stub included, and the edge had no first move to make. On 33q it cost
    // every one of the seven last edges — f5, f11, f17, f22, f27, f33, f39 —
    // which failed their search in all six rounds, each one fenced by the
    // resonator standing beside it in the ring (user, 2026-10-01).
    //
    // The rule is not being given up: `meetAt` still decides cell by cell,
    // and it opens only what lies within the coupler's reach of its anchor.
    if (sharedCoupler(one, two) != NO_OWNER) {
      return true;
    }
    const auto link = static_cast<double>(tuning_.clearance);
    for (const auto* const a : {&one.objective.source, &one.objective.target}) {
      for (const auto* const b :
           {&two.objective.source, &two.objective.target}) {
        if (std::hypot(static_cast<double>(a->x) - b->x,
                       static_cast<double>(a->y) - b->y) <= link) {
          return true;
        }
      }
    }
    return false;
  }

  /// Apply something to every cell of a disc around every cell of a way, once
  /// per step: the full disc at the first cell and the leading edge of the
  /// disc at each step after it, as the clearance field charges its own.
  /// Cells off the grid are skipped.
  template <typename Apply>
  void alongDisc(const Path& way, const Stencil& stencil, Apply&& apply,
                 const std::size_t from = 0) const {
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto height = static_cast<std::int64_t>(scene_.router.height);
    const auto stamp = [&](const Stencil::Offsets& offsets,
                           const std::int64_t cx, const std::int64_t cy) {
      for (const auto& [dx, dy] : offsets) {
        const auto x = cx + dx;
        const auto y = cy + dy;
        if (x >= 0 && y >= 0 && x < width && y < height) {
          apply(x, y);
        }
      }
    };
    bool started = false;
    std::int64_t lastX = 0;
    std::int64_t lastY = 0;
    for (std::size_t step = from; step < way.size(); ++step) {
      const auto& point = way[step];
      const auto x = static_cast<std::int64_t>(point.x);
      const auto y = static_cast<std::int64_t>(point.y);
      if (started) {
        const auto sx = x - lastX;
        const auto sy = y - lastY;
        if (sx == 0 && sy == 0) {
          continue;
        }
        if (sx >= -1 && sx <= 1 && sy >= -1 && sy <= 1) {
          stamp(stencil.edge[static_cast<std::size_t>(sy + 1)]
                            [static_cast<std::size_t>(sx + 1)],
                x, y);
          lastX = x;
          lastY = y;
          continue;
        }
      }
      stamp(stencil.full, x, y);
      started = true;
      lastX = x;
      lastY = y;
    }
  }

  /// The price of leaving the lane between a wire's two ring neighbours.
  ///
  /// This is the prototype's `compute_corridor_polygon_proximity`, and it is
  /// what steers the relaxation. The lane is the closed polygon that runs from
  /// the wire's own source along the way of the neighbour before it, back to
  /// the wire's own target, and along the way of the neighbour after it. Every
  /// cell outside that polygon is priced; nothing is forbidden. So a wire that
  /// has to leave its lane may, and a wire that need not, does not.
  ///
  /// On top of it the wires further ahead along the sweep are priced at
  /// growing distances, so that a wire let go of is pushed away rather than
  /// walked over.
  ///
  /// And on top of that, not in the prototype, the approaches of the wires
  /// around this one cost ten times the price. A wire let go of is
  /// crossable, but the straight run it leaves its source on and the run it
  /// arrives at its target on are the two places it cannot be drawn anywhere
  /// else: a way through either takes them for good, and the wire let go of
  /// has no way back. The ring neighbours are fenced already; their
  /// approaches are priced all the same, so that a way that has to pass them
  /// passes wide.
  void priceLane(const std::vector<Wire>& wires,
                 const std::vector<std::uint32_t>& members,
                 const std::uint32_t slot, const bool forward,
                 const std::uint32_t level) {
    const auto total = static_cast<std::uint32_t>(members.size());
    const auto& wire = wires[members[slot]];
    const auto& before = wires[members[(slot + total - 1) % total]];
    const auto& after = wires[members[(slot + 1) % total]];

    // The prototype's feedline pass builds the polygon from other ways than
    // the two neighbours (`Driver::laneOf`), prices the discs of the wires it
    // let go of at one clearance rather than one per level
    // (`FinalGrid.cpp:4612` against `:3151`), and knows nothing of the
    // ten-fold price on the approaches, which is ours.
    const bool proto = feedlinePass_;

    const auto price = tuning_.wireProximityPenalty;
    std::ranges::fill(proximity_, 0);
    if (proto) {
      const auto lane = laneOf(wires, members, slot);
      if (!lane.rule.empty()) {
        tell(std::format("    lane {} · {}before {}, after {}, own {} cells",
                         wireId(wire), lane.rule, lane.before.size(),
                         lane.after.size(), lane.current.size()));
      }
      fillLane(lane.current, lane.before, lane.after, price, 0);
    } else {
      // The outer routing prices the room outside the lane polygon between
      // the two neighbours' ways: the ground first, the lane freed after, so
      // a degenerate polygon leaves everything priced.
      std::ranges::fill(proximity_, price);
      fillLane(wire.way, before.way, after.way, price, 0);
    }

    const auto neighbour =
        forward ? (slot + 1) % total : (slot + total - 1) % total;
    // The wires let go of keep a priced halo: in the feedline pass a flat
    // halo of `tuning_.haloReach`, in the outer routing one clearance more per
    // level.
    for (std::uint32_t step = 0; step <= level; ++step) {
      const auto at = forward ? (neighbour + step) % total
                              : (neighbour + total - step) % total;
      const auto& released = wires[members[at]].way;
      stampHalo(released,
                proto ? tuning_.haloReach * tuning_.clearance
                      : (step + 1) * tuning_.clearance,
                static_cast<std::uint8_t>(std::min<std::uint32_t>(127U, price)),
                wire.way, proto);
    }

    // The approaches of the wires around the one being drawn are priced at
    // ten times the wire price in the outer routing; `priceLane` returns
    // before this under the feedline constraints.
    if (proto) {
      return;
    }

    const auto strong =
        static_cast<std::uint8_t>(std::min<std::uint32_t>(127U, 10U * price));
    std::vector<std::uint32_t> around{before.key, after.key};
    for (std::uint32_t step = 0; step <= level; ++step) {
      const auto at = forward ? (neighbour + step) % total
                              : (neighbour + total - step) % total;
      around.push_back(wires[members[at]].key);
    }
    std::ranges::sort(around);
    const auto twice = std::ranges::unique(around);
    around.erase(twice.begin(), twice.end());
    for (const auto key : around) {
      if (key != wire.key) {
        priceApproaches(wires[key], strong);
      }
    }
  }

  /// Price the two approaches of a wire: the straight run out of its source
  /// along the heading it leaves on, and the run into its target along the
  /// heading it arrives on, each as long as a port's band and twice as wide
  /// as the wire clearance, in the band's own geometry.
  void priceApproaches(const Wire& other, const std::uint16_t price) {
    if (!other.feasible) {
      return;
    }
    priceApproach(other.objective.source, other.objective.source.heading, true,
                  price);
    priceApproach(other.objective.target, other.objective.target.heading, false,
                  price);
  }

  /// One approach: from `end` along `heading` where the wire leaves there,
  /// against it where the wire arrives there. A price only ever rises.
  void priceApproach(const PathPoint& end, const Heading heading,
                     const bool leaving, const std::uint16_t price) {
    const auto v = routing::headingVector(heading);
    if (v.dx == 0 && v.dy == 0) {
      return;
    }
    const std::int64_t dirX = leaving ? v.dx : -v.dx;
    const std::int64_t dirY = leaving ? v.dy : -v.dy;
    const bool diagonal = v.dx != 0 && v.dy != 0;
    const auto length =
        diagonal ? tuning_.approachDiagonal : tuning_.approachAxial;
    // Twice the band's half width: two clearances across, not one.
    const auto half =
        2 * static_cast<std::int64_t>(diagonal ? tuning_.approachHalfDiagonal
                                               : tuning_.approachHalfAxial);
    const std::int64_t perpX = -dirY;
    const std::int64_t perpY = dirX;
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto height = static_cast<std::int64_t>(scene_.router.height);
    const auto strip = [&](const std::int64_t cx, const std::int64_t cy) {
      for (std::int64_t t = -half; t <= half; ++t) {
        const auto x = cx + (t * perpX);
        const auto y = cy + (t * perpY);
        if (x < 0 || y < 0 || x >= width || y >= height) {
          continue;
        }
        auto& cell = proximity_[static_cast<std::size_t>((y * width) + x)];
        cell = std::max(cell, price);
      }
    };
    std::int64_t prevY = end.y;
    for (std::uint32_t k = 0; k <= length; ++k) {
      const auto cx = static_cast<std::int64_t>(end.x) + (dirX * k);
      const auto cy = static_cast<std::int64_t>(end.y) + (dirY * k);
      // A diagonal run touches its last cell only at a corner; the joining
      // cell closes the gap, as the band's stamp closes its own.
      if (diagonal && k > 0) {
        strip(cx, prevY);
      }
      strip(cx, cy);
      prevY = cy;
    }
  }

  /// The three ways the lane polygon is built from.
  struct Lane {
    Path before;
    Path after;
    Path current;
    /// Which of the prototype's rules fired, for the log.
    std::string rule;
  };

  /// Whether a feedline edge has a launcher at its source, or at its target.
  /// The prototype's `first_feedlines` and `last_feedlines`.
  [[nodiscard]] bool startsAtLauncher(const Wire& wire) const {
    if (!wire.feedline || wire.edge >= edges_.size()) {
      return false;
    }
    const auto& edge = edges_[wire.edge];
    return edge.chain < chains_.size() &&
           edge.from < chains_[edge.chain].size() &&
           chains_[edge.chain][edge.from].fixed;
  }
  [[nodiscard]] bool endsAtLauncher(const Wire& wire) const {
    if (!wire.feedline || wire.edge >= edges_.size()) {
      return false;
    }
    const auto& edge = edges_[wire.edge];
    return edge.chain < chains_.size() &&
           edge.to < chains_[edge.chain].size() &&
           chains_[edge.chain][edge.to].fixed;
  }

  /// Keep the stretch of a way that runs alongside another: from the cell
  /// nearest that one's first to the cell nearest its last, ordered so that
  /// the kept stretch runs the same way round.
  static void trimAlong(Path& way, const Path& current) {
    if (way.size() < 2 || current.empty()) {
      return;
    }
    const auto nearest = [&way](const PathPoint& to) {
      std::size_t best = 0;
      auto least = std::numeric_limits<std::uint32_t>::max();
      for (std::size_t at = 0; at < way.size(); ++at) {
        const auto span = static_cast<std::uint32_t>(
            std::sqrt(std::pow(static_cast<double>(way[at].x) - to.x, 2) +
                      std::pow(static_cast<double>(way[at].y) - to.y, 2)));
        if (span < least) {
          least = span;
          best = at;
        }
      }
      return best;
    };
    const auto head = nearest(current.front());
    const auto tail = nearest(current.back());
    const auto lo = std::min(head, tail);
    const auto hi = std::max(head, tail);
    if (hi <= lo) {
      return;
    }
    Path kept(way.begin() + static_cast<std::ptrdiff_t>(lo),
              way.begin() + static_cast<std::ptrdiff_t>(hi) + 1);
    if (head > tail) {
      std::ranges::reverse(kept);
    }
    way = std::move(kept);
  }

  /// Cut a way after the cell of it that lies nearest a point.
  static void cutAfterNearest(Path& way, const PathPoint& to) {
    if (way.empty()) {
      return;
    }
    // The prototype truncates: `uint32_t dist = std::sqrt(...)`
    // (`FinalGrid.cpp:4484`). Whole cells, and the first of a tie wins,
    // which is not the same index a real-valued distance picks.
    std::size_t best = 0;
    std::uint32_t least = std::numeric_limits<std::uint32_t>::max();
    for (std::size_t at = 0; at < way.size(); ++at) {
      const auto span = static_cast<std::uint32_t>(
          std::sqrt(std::pow(static_cast<double>(way[at].x) - to.x, 2) +
                    std::pow(static_cast<double>(way[at].y) - to.y, 2)));
      if (span < least) {
        least = span;
        best = at;
      }
    }
    way.erase(way.begin() + static_cast<std::ptrdiff_t>(best) + 1, way.end());
  }

  /// What the lane polygon is drawn between, as the prototype draws it
  /// (`FinalGrid.cpp:4437`-`:4592`).
  ///
  /// The polygon of the outer routing runs from the wire's first cell along
  /// its predecessor, to its last cell, and back along its successor. That
  /// is the room the wire has — and it is the right room only while the two
  /// wires beside it are conventional wires of the ring, which is what the
  /// outer routing's members always are.
  ///
  /// In the feedline pass they need not be. A feedline edge runs *across*
  /// the ring rather than beside it, so a polygon closed on one bounds
  /// nothing; and a resonator beside a resonator is not a wall either,
  /// because it is the thing the wire is trying to get past. Either way the
  /// fill then frees a region that is not the wire's room, and the search is
  /// steered into somebody else's.
  ///
  /// So the walk goes outward until it finds a way that does bound the room:
  /// past a feedline to the wire beyond it, past a run of resonators to the
  /// first conventional wire after them — that one cut at the cell nearest
  /// the resonator's own source, because only the part of it that runs
  /// alongside is a wall.
  [[nodiscard]] Lane laneOf(const std::vector<Wire>& wires,
                            const std::vector<std::uint32_t>& members,
                            const std::uint32_t slot) const {
    const auto total = static_cast<std::int64_t>(members.size());
    const auto step = [&](const std::int64_t k) -> const Wire& {
      const auto at =
          ((static_cast<std::int64_t>(slot) + k) % total + total) % total;
      return wires[members[static_cast<std::size_t>(at)]];
    };
    Lane lane{
        .before = step(-1).way, .after = step(1).way, .current = step(0).way};
    const auto& wire = step(0);

    // A feedline edge of its own. A terminal edge takes the way beyond the
    // neighbour it would be bounded by, and a last edge is run the other way
    // round so that the polygon closes the way it was drawn.
    if (wire.feedline) {
      if (startsAtLauncher(wire)) {
        lane.after = step(2).way;
        lane.rule += "first feedline: after <- +2; ";
      }
      if (endsAtLauncher(wire)) {
        lane.before = step(-2).way;
        std::ranges::reverse(lane.current);
        lane.rule += "last feedline: before <- -2, own way reversed; ";
      }
      return lane;
    }

    // The successor.
    const auto& next = step(1);
    if (next.feedline) {
      if (wire.resonator) {
        lane.after = step(2).way;
        lane.rule += "after is a feedline, this a resonator: after <- +2; ";
      } else {
        const auto& beyond = step(2).way;
        lane.after.insert(lane.after.end(), beyond.begin(), beyond.end());
        lane.rule += "after is a feedline, this conventional: after += +2; ";
      }
    } else if (next.resonator) {
      // The prototype walks on from `i + 2` while it stands on a resonator
      // and stops on the first that is not one — or on the wire itself,
      // having gone the whole way round, and it uses that one too
      // (`FinalGrid.cpp:4465`). No guard, so none here.
      std::int64_t k = 2;
      while (k < total && step(k).resonator) {
        ++k;
      }
      auto beyond = step(k).way;
      if (endsAtLauncher(step(k))) {
        std::ranges::reverse(beyond);
      } else {
        cutAfterNearest(beyond, next.objective.source);
      }
      lane.after.insert(lane.after.begin(), beyond.begin(), beyond.end());
      lane.rule += std::format("after is a resonator: +{} prepended{}; ", k,
                               endsAtLauncher(step(k)) ? " reversed" : " cut");
    }

    // The predecessor, the same the other way round.
    const auto& prev = step(-1);
    if (prev.feedline) {
      if (wire.resonator) {
        lane.before = step(-2).way;
        lane.rule += "before is a feedline, this a resonator: before <- -2; ";
      } else {
        std::ranges::reverse(lane.before);
        const auto& beyond = step(-2).way;
        lane.before.insert(lane.before.end(), beyond.begin(), beyond.end());
        lane.rule +=
            "before is a feedline, this conventional: reversed, += -2; ";
      }
    } else if (prev.resonator) {
      std::int64_t k = -2;
      while (k > -total && step(k).resonator) {
        --k;
      }
      // The before side never reverses, whatever it stops on: the
      // prototype's `last_feedlines` test is on the after side only
      // (`FinalGrid.cpp:4497` against `:4568`).
      auto beyond = step(k).way;
      cutAfterNearest(beyond, prev.objective.source);
      lane.before.insert(lane.before.begin(), beyond.begin(), beyond.end());
      lane.rule += std::format("before is a resonator: {} prepended cut; ", k);
    }

    // The case the prototype has no rule for: a resonator with a plain wire
    // on either side. Neither branch above fired, so both neighbours are
    // still whole, and the polygon would close over the two long jumps from
    // the coupler and from the qubit: a resonator begins at its coupler, out
    // in the fan-in, and ends at its qubit, while its neighbours begin and
    // end at their own ports. Cut to the stretch between the cells nearest
    // the resonator's two ends, the lane is the corridor the wire runs in.
    if (wire.resonator && !next.feedline && !next.resonator && !prev.feedline &&
        !prev.resonator) {
      const auto was = lane.before.size() + lane.after.size();
      trimAlong(lane.before, lane.current);
      trimAlong(lane.after, lane.current);
      lane.rule += std::format(
          "resonator between two plain wires: both cut to its own span, "
          "{} -> {} cells; ",
          was, lane.before.size() + lane.after.size());
    }
    return lane;
  }

  /// Clear the price inside the lane, by a scanline fill of its polygon.
  /// `outside` is the price the ground takes, `inside` the price the lane
  /// keeps. The ground is painted **here**, not by the caller, and only once
  /// the polygon is known to be one: the prototype's
  /// `compute_corridor_polygon_proximity` returns before its own
  /// `std::fill` on an empty way or a degenerate polygon
  /// (`FinalGrid.cpp:15756`, `:15773`), which leaves the grid as the caller
  /// zeroed it. Painting first and freeing after inverts that case — every
  /// cell priced where the prototype prices none.
  void fillLane(const Path& current, const Path& before, const Path& after,
                const std::uint16_t outside, const std::uint16_t inside) {
    if (current.empty() || outside == 0) {
      return;
    }
    std::vector<std::pair<double, double>> polygon;
    polygon.reserve(before.size() + after.size() + 2);
    const auto add = [&polygon](const PathPoint& point) {
      polygon.emplace_back(static_cast<double>(point.x) + 0.5,
                           static_cast<double>(point.y) + 0.5);
    };
    add(current.front());
    for (const auto& point : before) {
      add(point);
    }
    add(current.back());
    for (const auto& point : std::ranges::reverse_view(after)) {
      add(point);
    }
    if (polygon.size() < 3) {
      return;
    }
    std::ranges::fill(proximity_, outside);

    double minY = polygon.front().second;
    double maxY = minY;
    for (const auto& [x, y] : polygon) {
      minY = std::min(minY, y);
      maxY = std::max(maxY, y);
    }
    const auto height = static_cast<std::int64_t>(scene_.router.height);
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto firstRow =
        std::max<std::int64_t>(0, static_cast<std::int64_t>(std::floor(minY)));
    const auto lastRow = std::min<std::int64_t>(
        height - 1, static_cast<std::int64_t>(std::ceil(maxY)));

    std::vector<double> crossings;
    for (std::int64_t y = firstRow; y <= lastRow; ++y) {
      const auto scan = static_cast<double>(y) + 0.5;
      crossings.clear();
      for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size();
           j = i++) {
        const auto [xi, yi] = polygon[i];
        const auto [xj, yj] = polygon[j];
        if ((yi > scan) != (yj > scan)) {
          crossings.push_back(xi + ((scan - yi) * (xj - xi) / (yj - yi)));
        }
      }
      std::ranges::sort(crossings);
      for (std::size_t at = 0; at + 1 < crossings.size(); at += 2) {
        const auto from = std::max<std::int64_t>(
            0, static_cast<std::int64_t>(std::ceil(crossings[at] - 0.5)));
        const auto to = std::min<std::int64_t>(
            width - 1,
            static_cast<std::int64_t>(std::floor(crossings[at + 1] - 0.5)));
        for (std::int64_t x = from; x <= to; ++x) {
          proximity_[static_cast<std::size_t>((y * width) + x)] = inside;
        }
      }
    }
  }

  /// Price a disc of a radius around every cell of a way, added to what the
  /// cell already holds and clamped at `CEILING`.
  ///
  /// Adding keeps the ground the lane fill laid and raises it where
  /// something else is near, so the halo of a released wire is dearer than
  /// open ground and dearer again where it meets a feedline. A stamp that
  /// replaced the price left a cell outside the lane and a cell in the halo
  /// at the same figure, and the search had nothing to tell them apart.
  void stampDisc(const Path& way, const std::uint32_t radius,
                 const std::uint16_t price) {
    if (way.empty() || radius == 0 || price == 0) {
      return;
    }
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    alongDisc(way, stencilFor(radius),
              [&](const std::int64_t x, const std::int64_t y) {
                auto& cell =
                    proximity_[static_cast<std::size_t>((y * width) + x)];
                cell = static_cast<std::uint16_t>(std::min<std::uint32_t>(
                    CEILING, static_cast<std::uint32_t>(cell) + price));
              });
  }

  /// The halo of a released way: a flat disc of `reach` at `peak`, and in
  /// the feedline pass the rising halo past the way on top
  /// (`tuning_.riseReach`, `tuning_.risePeak`). The outer routing before the
  /// insertion keeps the flat disc alone, so the one-sided halo does not move
  /// the couplers the feedline pass starts from.
  void stampHalo(const Path& way, const std::uint32_t reach,
                 const std::uint16_t peak, const Path& own = {},
                 const bool feedlinePass = false) {
    if (way.empty() || reach == 0 || peak == 0) {
      return;
    }
    stampDisc(way, reach, peak);
    if (feedlinePass) {
      stampRisingHalo(way, tuning_.riseReach * tuning_.clearance,
                      static_cast<std::uint16_t>(tuning_.risePeak * peak), own,
                      peak);
    }
  }

  /// A rising halo beyond a released way: nothing on the
  /// way and nothing on the side of it the wire being drawn lies on (`own`,
  /// its way before); on the far side a price growing with the distance from
  /// the way, from `base` at the way to `peak` at the reach, nothing past it.
  /// So a way pays only once it has crossed the released wire, and the more the
  /// farther it runs from it there; a cell past either end of the released way
  /// is no crossing and pays nothing. The side of a cell is the side of the way
  /// at the way's nearest cell, from a feature transform over the box the halo
  /// covers. Without a way before there is no own side, and nothing is priced.
  void stampRisingHalo(const Path& way, const std::uint32_t reach,
                       const std::uint16_t peak, const Path& own,
                       const std::uint16_t base) {
    if (way.size() < 2 || own.empty()) {
      return;
    }
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto height = static_cast<std::int64_t>(scene_.router.height);
    const auto r = static_cast<std::int64_t>(reach);
    std::int64_t minX = width;
    std::int64_t minY = height;
    std::int64_t maxX = -1;
    std::int64_t maxY = -1;
    for (const auto& point : way) {
      minX = std::min<std::int64_t>(minX, point.x);
      minY = std::min<std::int64_t>(minY, point.y);
      maxX = std::max<std::int64_t>(maxX, point.x);
      maxY = std::max<std::int64_t>(maxY, point.y);
    }
    minX = std::max<std::int64_t>(0, minX - r);
    minY = std::max<std::int64_t>(0, minY - r);
    maxX = std::min<std::int64_t>(width - 1, maxX + r);
    maxY = std::min<std::int64_t>(height - 1, maxY + r);
    const auto boxW = static_cast<std::uint32_t>(maxX - minX + 1);
    const auto boxH = static_cast<std::uint32_t>(maxY - minY + 1);
    const auto cells = static_cast<std::size_t>(boxW) * boxH;
    grid::BitGrid on(boxW, boxH);
    // The direction of the way at each of its cells, and its two end cells.
    std::vector<std::pair<std::int8_t, std::int8_t>> heading(cells, {0, 0});
    std::vector<bool> end(cells, false);
    const auto local = [&](const PathPoint& point) {
      return static_cast<std::size_t>(
          ((static_cast<std::int64_t>(point.y) - minY) * boxW) +
          (static_cast<std::int64_t>(point.x) - minX));
    };
    for (std::size_t at = 1; at < way.size(); ++at) {
      const auto& from = way[at - 1];
      const auto& to = way[at];
      const auto dx = static_cast<std::int64_t>(to.x) - from.x;
      const auto dy = static_cast<std::int64_t>(to.y) - from.y;
      if (dx == 0 && dy == 0) {
        continue;
      }
      const std::pair<std::int8_t, std::int8_t> step{
          static_cast<std::int8_t>((dx > 0) - (dx < 0)),
          static_cast<std::int8_t>((dy > 0) - (dy < 0))};
      for (const auto cell : grid::lineCells(
               static_cast<std::int64_t>(from.x) - minX,
               static_cast<std::int64_t>(from.y) - minY,
               static_cast<std::int64_t>(to.x) - minX,
               static_cast<std::int64_t>(to.y) - minY, boxW, boxH)) {
        on.set(cell);
        heading[cell] = step;
      }
    }
    end[local(way.front())] = true;
    end[local(way.back())] = true;
    const auto nearest = grid::nearestBlocked(on);
    // The side of a cell of the box: +1 left of the way, -1 right, 0 on it,
    // past an end of it or out of reach.
    const auto sideOf = [&](const std::int64_t x, const std::int64_t y) {
      const auto here = static_cast<std::size_t>((y * boxW) + x);
      const auto q = nearest.cell[here];
      if (q == grid::NO_NEAREST || end[q] ||
          nearest.squared[here] > static_cast<std::uint32_t>(r * r) ||
          nearest.squared[here] == 0) {
        return 0;
      }
      const auto qx = static_cast<std::int64_t>(q % boxW);
      const auto qy = static_cast<std::int64_t>(q / boxW);
      const auto cross =
          (heading[q].first * (y - qy)) - (heading[q].second * (x - qx));
      return static_cast<int>((cross > 0) - (cross < 0));
    };
    // The own side: where most of the wire's way before lies.
    int vote = 0;
    for (const auto& point : own) {
      const auto x = static_cast<std::int64_t>(point.x) - minX;
      const auto y = static_cast<std::int64_t>(point.y) - minY;
      if (x >= 0 && y >= 0 && x < boxW && y < boxH) {
        vote += sideOf(x, y);
      }
    }
    if (vote == 0) {
      return;
    }
    const auto ownSide = vote > 0 ? 1 : -1;
    const auto limit = static_cast<double>(reach);
    for (std::int64_t y = 0; y < boxH; ++y) {
      for (std::int64_t x = 0; x < boxW; ++x) {
        const auto side = sideOf(x, y);
        if (side == 0 || side == ownSide) {
          continue;
        }
        const auto d = std::sqrt(static_cast<double>(
            nearest.squared[static_cast<std::size_t>((y * boxW) + x)]));
        const auto price = static_cast<std::int32_t>(std::lround(
            static_cast<double>(base) +
            (static_cast<double>(peak - std::min(peak, base)) * d / limit)));
        if (price <= 0) {
          continue;
        }
        auto& cell = proximity_[static_cast<std::size_t>(((y + minY) * width) +
                                                         x + minX)];
        const auto was = static_cast<std::int32_t>(cell);
        cell = static_cast<std::uint16_t>(
            std::clamp(was + price, 0, static_cast<std::int32_t>(CEILING)));
      }
    }
  }

  [[nodiscard]] const Stencil& stencilFor(const std::uint32_t radius) {
    const auto found = stencils_.find(radius);
    if (found != stencils_.end()) {
      return found->second;
    }
    return stencils_.emplace(radius, stencilOf(radius)).first->second;
  }

  /// The price that pulls a wire into the middle of the room it has.
  ///
  /// Two chamfer passes give every free cell of the corridor its distance to
  /// the nearest wall; the cells that are a local maximum of that distance are
  /// the middle of the channel, and two more passes give every cell its
  /// distance to the middle. What a cell costs is then the share of the way
  /// from the middle to the wall it stands at, so the middle is free and the
  /// wall is the full price. This is the prototype's
  /// `compute_corridor_proximity_decay`, which is what its refinement steers
  /// by.
  void priceRoom() {
    const auto price = tuning_.wireProximityPenalty;
    std::ranges::fill(proximity_, price);
    if (price == 0) {
      return;
    }
    if (box_.empty() || openBox_.empty()) {
      return;
    }
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    constexpr std::int32_t FAR = 1000000;
    // Every pass below reads and writes this box alone, and every cell
    // outside it keeps the full price of the fill above, so the two fields
    // are sized once and set inside the box only.
    wall_.resize(scene_.router.cells());
    middle_.resize(scene_.router.cells());

    // The open cells grown by one, within the band's box. A cell of the box
    // outside it is a wall, and the passes carry no distance through a wall,
    // so the field inside is the field the whole box would give.
    const auto lowX = std::max(box_.minX, openBox_.minX - 1);
    const auto lowY = std::max(box_.minY, openBox_.minY - 1);
    const auto highX = std::min(box_.maxX, openBox_.maxX + 1);
    const auto highY = std::min(box_.maxY, openBox_.maxY + 1);
    for (std::int64_t y = lowY; y <= highY; ++y) {
      for (std::int64_t x = lowX; x <= highX; ++x) {
        const auto cell = static_cast<std::size_t>((y * width) + x);
        const bool border = y == lowY || x == lowX || y == highY || x == highX;
        wall_[cell] = (corridor_.test(cell) || border) ? 0 : FAR;
        middle_[cell] = FAR;
      }
    }
    const auto step = static_cast<std::size_t>(width);
    for (std::int64_t y = lowY + 1; y < highY; ++y) {
      for (std::int64_t x = lowX + 1; x < highX; ++x) {
        const auto cell = static_cast<std::size_t>((y * width) + x);
        if (wall_[cell] == 0) {
          continue;
        }
        wall_[cell] = std::min(
            {wall_[cell], wall_[cell - step - 1] + 1, wall_[cell - step] + 1,
             wall_[cell - step + 1] + 1, wall_[cell - 1] + 1});
      }
    }
    for (std::int64_t y = highY - 1; y > lowY; --y) {
      for (std::int64_t x = highX - 1; x > lowX; --x) {
        const auto cell = static_cast<std::size_t>((y * width) + x);
        if (wall_[cell] == 0) {
          continue;
        }
        wall_[cell] = std::min(
            {wall_[cell], wall_[cell + 1] + 1, wall_[cell + step - 1] + 1,
             wall_[cell + step] + 1, wall_[cell + step + 1] + 1});
      }
    }

    for (std::int64_t y = lowY + 1; y < highY; ++y) {
      for (std::int64_t x = lowX + 1; x < highX; ++x) {
        const auto cell = static_cast<std::size_t>((y * width) + x);
        if (wall_[cell] == 0) {
          continue;
        }
        bool peak = true;
        for (std::int64_t dy = -1; dy <= 1 && peak; ++dy) {
          for (std::int64_t dx = -1; dx <= 1; ++dx) {
            const auto other = static_cast<std::size_t>(
                static_cast<std::int64_t>(cell) + (dy * width) + dx);
            if (wall_[other] > wall_[cell]) {
              peak = false;
              break;
            }
          }
        }
        if (peak) {
          middle_[cell] = 0;
        }
      }
    }
    for (std::int64_t y = lowY + 1; y < highY; ++y) {
      for (std::int64_t x = lowX + 1; x < highX; ++x) {
        const auto cell = static_cast<std::size_t>((y * width) + x);
        if (wall_[cell] == 0) {
          continue;
        }
        middle_[cell] =
            std::min({middle_[cell], middle_[cell - step - 1] + 1,
                      middle_[cell - step] + 1, middle_[cell - step + 1] + 1,
                      middle_[cell - 1] + 1});
      }
    }
    for (std::int64_t y = highY - 1; y > lowY; --y) {
      for (std::int64_t x = highX - 1; x > lowX; --x) {
        const auto cell = static_cast<std::size_t>((y * width) + x);
        if (wall_[cell] == 0) {
          continue;
        }
        middle_[cell] = std::min(
            {middle_[cell], middle_[cell + 1] + 1, middle_[cell + step - 1] + 1,
             middle_[cell + step] + 1, middle_[cell + step + 1] + 1});
      }
    }

    for (std::int64_t y = lowY + 1; y < highY; ++y) {
      for (std::int64_t x = lowX + 1; x < highX; ++x) {
        const auto cell = static_cast<std::size_t>((y * width) + x);
        const auto dw = wall_[cell];
        const auto dc = middle_[cell];
        if (dw == 0 || dc >= FAR) {
          continue;
        }
        proximity_[cell] = static_cast<std::uint8_t>(
            ((dc * price) + ((dc + dw) / 2)) / (dc + dw));
      }
    }
  }

  /// The box a band can reach, so that nothing sweeps the whole grid.
  struct Box {
    std::int64_t minX = 0;
    std::int64_t minY = 0;
    std::int64_t maxX = -1;
    std::int64_t maxY = -1;
    [[nodiscard]] bool empty() const { return maxX < minX || maxY < minY; }
  };

  [[nodiscard]] Box boxOf(const Wire& wire, const std::uint32_t reach) const {
    Box box{.minX = std::numeric_limits<std::int64_t>::max(),
            .minY = std::numeric_limits<std::int64_t>::max(),
            .maxX = std::numeric_limits<std::int64_t>::min(),
            .maxY = std::numeric_limits<std::int64_t>::min()};
    const auto stretch = [&box](const PathPoint& point) {
      box.minX = std::min(box.minX, static_cast<std::int64_t>(point.x));
      box.maxX = std::max(box.maxX, static_cast<std::int64_t>(point.x));
      box.minY = std::min(box.minY, static_cast<std::int64_t>(point.y));
      box.maxY = std::max(box.maxY, static_cast<std::int64_t>(point.y));
    };
    for (const auto& point : wire.way) {
      stretch(point);
    }
    stretch(wire.objective.source);
    stretch(wire.objective.target);
    for (const auto& point : wire.fixed) {
      stretch(point);
    }
    const auto grown = static_cast<std::int64_t>(reach);
    box.minX = std::max<std::int64_t>(0, box.minX - grown);
    box.minY = std::max<std::int64_t>(0, box.minY - grown);
    box.maxX =
        std::min<std::int64_t>(scene_.router.width - 1, box.maxX + grown);
    box.maxY =
        std::min<std::int64_t>(scene_.router.height - 1, box.maxY + grown);
    return box;
  }

public:
  /// A picture of the whole router grid before anything is drawn: the
  /// artwork, the price of running beside it, every port as the chip carries
  /// it, every wire's two ends with the heading it leaves and arrives on, and
  /// the way the Detail stage drew.
  void attachPorts(std::vector<PortMark> ports) { ports_ = std::move(ports); }

  /// The capacity graph of `reportCapacityGraph` as data, for the page that
  /// `plan --debug` builds from it (`final-capacity-graph.html`). Cells are
  /// grid cells, y up. It holds
  ///
  /// - the chambers and the walls as one raster, run-length coded row by
  ///   row: 0 a free cell in no chamber (the line of a bottleneck), 1 to 5
  ///   a wall by its kind (`wallKinds`), 6 + c a cell of chamber c;
  /// - the lines of the port runs and the feedline edges (`strokes`);
  /// - a node per chamber, and every edge with the chambers it joins, what
  ///   it takes, the wires the flow sends through it and its overflow; a
  ///   bottleneck also with its two ends and the wires its line crosses
  ///   now, a stretch with its feedline and the wires it is open to;
  /// - the cells of the slots between the walls of the port runs, where the
  ///   terminals lie;
  /// - the bottlenecks with one chamber on both sides (`aside`);
  /// - every outer wire with its source and its target port, the cells its
  ///   ends leave from, the chambers they lie in, its way through the graph
  ///   as edge indices and the way it has now.
  ///
  /// With `bySlots` the check by crossing slots judges the data, with or
  /// without a debug sink, and the graph it saw is written instead
  /// (`judgedBySlots`).
  void writeCapacityGraph(const std::vector<Wire>& wires, const Walls& walls,
                          const std::vector<grid::Bottleneck>& gates,
                          const std::vector<MeasuredBottleneck>& measured,
                          const grid::Chambers& chambers,
                          const std::vector<GraphEdge>& graph,
                          const std::vector<GraphWire>& outer,
                          const FlowCheck& check,
                          const std::string_view verdict,
                          const std::vector<std::string>& summary,
                          const bool bySlots = false) {
    if (bySlots) {
      // The verdict of the chip, from the graph in memory; the JSON below
      // is only the page.
      const auto began = std::chrono::steady_clock::now();
      try {
        sayBySlots(slotCheck(slotGraphOf(wires, walls, gates, measured,
                                         chambers, graph, outer)),
                   verdict, began);
      } catch (const std::invalid_argument& error) {
        say(std::format("coupler insertion: the slot check failed ({}); the "
                        "verdict of the stretches stands: CAPACITY GRAPH {}",
                        error.what(), verdict));
      }
    }
    if (!graphOnly_ && !wants("capacity-graph")) {
      return;
    }
    using nlohmann::json;
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto cellAt = [width](const std::size_t cell) {
      return json::array({static_cast<std::int64_t>(cell) % width,
                          static_cast<std::int64_t>(cell) / width});
    };
    const auto names = [&wires](const std::vector<std::uint32_t>& keys) {
      auto list = json::array();
      for (const auto key : keys) {
        list.push_back(wireId(wires[key]));
      }
      return list;
    };
    // A way as the cells it turns at, its two ends included.
    const auto corners = [](const Path& way, const std::size_t first,
                            const std::size_t last) {
      auto list = json::array();
      for (auto at = first; at <= last && at < way.size(); ++at) {
        if (at != first && at != last) {
          const auto& before = way[at - 1];
          const auto& here = way[at];
          const auto& after = way[at + 1];
          const auto inX = static_cast<std::int64_t>(here.x) - before.x;
          const auto inY = static_cast<std::int64_t>(here.y) - before.y;
          const auto outX = static_cast<std::int64_t>(after.x) - here.x;
          const auto outY = static_cast<std::int64_t>(after.y) - here.y;
          if (inX == outX && inY == outY) {
            continue;
          }
        }
        list.push_back(json::array({way[at].x, way[at].y}));
      }
      return list;
    };

    auto raster = json::array();
    {
      int previous = -1;
      std::size_t run = 0;
      for (std::size_t cell = 0; cell < chambers.of.size(); ++cell) {
        int code = 0;
        if (walls.mask.test(cell)) {
          code = walls.of[cell] < walls.list.size()
                     ? 1 + static_cast<int>(walls.list[walls.of[cell]].kind)
                     : 1 + static_cast<int>(WallKind::Artwork);
        } else if (chambers.of[cell] != grid::NO_CHAMBER) {
          code = 6 + static_cast<int>(chambers.of[cell]);
        }
        if (code != previous && run > 0) {
          raster.push_back(previous);
          raster.push_back(run);
          run = 0;
        }
        previous = code;
        ++run;
      }
      if (run > 0) {
        raster.push_back(previous);
        raster.push_back(run);
      }
    }
    auto wallKinds = json::array();
    for (const auto kind :
         {WallKind::Border, WallKind::Artwork, WallKind::Coupler,
          WallKind::Stub, WallKind::Feedline}) {
      wallKinds.push_back(kindName(kind));
    }
    auto strokes = json::array();
    for (const auto& [wall, points] : walls.strokes) {
      auto line = json::array();
      for (const auto& [x, y] : points) {
        line.push_back(json::array({x, y}));
      }
      strokes.push_back({{"name", walls.list[wall].name},
                         {"kind", kindName(walls.list[wall].kind)},
                         {"points", std::move(line)}});
    }

    auto slotCells = json::array();
    for (const auto cell : walls.slots) {
      slotCells.push_back(cellAt(cell));
    }

    std::vector<std::size_t> size(chambers.count, 0);
    for (const auto chamber : chambers.of) {
      if (chamber != grid::NO_CHAMBER) {
        ++size[chamber];
      }
    }
    const auto node = chamberNodes(chambers);
    auto nodes = json::array();
    for (std::uint32_t chamber = 0; chamber < chambers.count; ++chamber) {
      nodes.push_back(
          {{"chamber", chamber},
           {"at", json::array({node[chamber].first, node[chamber].second})},
           {"cells", size[chamber]}});
    }

    auto edges = json::array();
    for (std::size_t index = 0; index < graph.size(); ++index) {
      const auto& edge = graph[index];
      json one{{"index", index},
               {"kind", edge.crossing ? "stretch" : "bottleneck"},
               {"chambers", edge.chambers},
               {"capacity", edge.capacity},
               {"load", edge.through.size()},
               {"overflow",
                index < check.overflow.size() ? check.overflow[index] : 0U},
               {"length", edge.length},
               {"through", names(edge.through)},
               {"users", edge.users.has_value() ? names(*edge.users) : json()},
               {"terminal", edge.terminal}};
      if (edge.crossing) {
        one["name"] = std::format("× {} cells {}..{}", wireId(wires[edge.id]),
                                  edge.first, edge.last);
        one["feedline"] = wireId(wires[edge.id]);
        one["line"] = corners(wires[edge.id].way, edge.first, edge.last);
        auto points = json::array();
        for (const auto cell : edge.points) {
          points.push_back(cellAt(cell));
        }
        one["points"] = std::move(points);
      } else {
        const auto& gate = gates[edge.id];
        one["name"] = std::format("g{}", edge.id);
        one["ends"] = json::array(
            {endName(walls, gate.first), endName(walls, gate.second)});
        one["line"] = json::array({cellAt(gate.first), cellAt(gate.second)});
        one["crossingNow"] = names(measured[edge.id].crossing);
        one["mouth"] = measured[edge.id].mouth;
        one["slot"] = measured[edge.id].slot;
      }
      edges.push_back(std::move(one));
    }
    auto aside = json::array();
    for (std::size_t gate = 0; gate < gates.size(); ++gate) {
      if (chambers.beside[gate].size() >= 2) {
        continue;
      }
      aside.push_back(
          {{"name", std::format("g{}", gate)},
           {"chambers", chambers.beside[gate]},
           {"ends", json::array({endName(walls, gates[gate].first),
                                 endName(walls, gates[gate].second)})},
           {"line", json::array({cellAt(gates[gate].first),
                                 cellAt(gates[gate].second)})},
           {"length", measured[gate].cells},
           {"capacity", measured[gate].holds}});
    }

    // The launcher a plain wire starts at, which its wire does not name:
    // the launcher port nearest the start of its way.
    const auto launcherAt = [this](const PathPoint& end) -> std::uint32_t {
      std::uint32_t nearest = NO_OWNER;
      std::int64_t best = std::numeric_limits<std::int64_t>::max();
      for (const auto& port : ports_) {
        if (!scene_.launcherCell.contains(port.index)) {
          continue;
        }
        const auto dx = port.x - static_cast<std::int64_t>(end.x);
        const auto dy = port.y - static_cast<std::int64_t>(end.y);
        if ((dx * dx) + (dy * dy) < best) {
          best = (dx * dx) + (dy * dy);
          nearest = port.index;
        }
      }
      return nearest;
    };
    // A port: what the chip calls it, where it is, the way a wire leaves it
    // or arrives at it, and the cell the graph places it by.
    const auto portOf = [&](const std::uint32_t index, const PathPoint& end,
                            const PathPoint& leaves,
                            const std::vector<std::uint32_t>& in,
                            const std::string_view otherwise) {
      const auto v = routing::headingVector(end.heading);
      json port{{"at", json::array({end.x, end.y})},
                {"heading", json::array({v.dx, v.dy})},
                {"cell", json::array({leaves.x, leaves.y})},
                {"chambers", in}};
      if (index < ports_.size()) {
        port["label"] = ports_[index].label;
        port["port"] = json::array({ports_[index].fx, ports_[index].fy});
      } else {
        port["label"] = otherwise;
        port["port"] = json::array({end.x, end.y});
      }
      return port;
    };
    auto outerWires = json::array();
    for (const auto& one : outer) {
      const auto& wire = wires[one.wire];
      json entry{{"name", wireId(wire)},
                 {"resonator", wire.resonator},
                 {"terminal", one.terminal},
                 {"path", wire.way.empty()
                              ? json::array()
                              : corners(wire.way, 0, wire.way.size() - 1)}};
      if (one.terminal) {
        // A terminal edge as a wire: its launcher port at one end, the
        // coupler's port at the other.
        entry["source"] = portOf(one.sourcePort, one.ends[0], one.ends[0],
                                 one.from, "coupler port");
        entry["target"] = portOf(one.targetPort, one.ends[1], one.ends[1],
                                 one.to, "coupler port");
      } else {
        entry["source"] =
            wire.couplerAtSource != NO_OWNER
                ? portOf(wire.sourcePort, wire.objective.source, one.ends[0],
                         one.from, "coupler port")
                : portOf(wire.sourcePort < ports_.size()
                             ? wire.sourcePort
                             : launcherAt(wire.objective.source),
                         wire.objective.source, one.ends[0], one.from, "");
        entry["target"] = portOf(wire.targetPort, wire.objective.target,
                                 one.ends[1], one.to, "");
        if (wire.couplerAtSource == NO_OWNER &&
            wire.sourcePort >= ports_.size()) {
          // The launcher port names the start; the wire starts at its cell.
          entry["source"]["port"] = entry["source"]["at"];
        }
      }
      entry["crosses"] = json::array();
      for (const auto feedline : one.crosses) {
        entry["crosses"].push_back(wireId(wires[feedline]));
      }
      if (!one.demand.has_value()) {
        entry["status"] = "port in no chamber";
        entry["way"] = json();
      } else if (const auto& way = check.ways[*one.demand]; !way.has_value()) {
        entry["status"] = "no way";
        entry["way"] = json();
      } else {
        entry["status"] = way->empty() ? "one chamber" : "routed";
        entry["way"] = *way;
      }
      outerWires.push_back(std::move(entry));
    }

    json data{{"width", scene_.router.width},
              {"height", scene_.router.height},
              {"cellSize", scene_.router.cellWidth},
              {"clearance", tuning_.clearance},
              {"straightLength", tuning_.straightLength},
              {"pitch", roomPitch()},
              {"crossingReach", tuning_.crossingReach + 1},
              {"capacityMargin", tuning_.capacityMargin},
              {"capacityMarginFromWires", tuning_.capacityMarginFromWires},
              {"resonatorPassage", tuning_.resonatorPassage},
              {"slotPinCells", tuning_.slotPinCells},
              {"escapeStraight", tuning_.escapeStraight},
              {"escapeStubMargin", tuning_.escapeStubMargin},
              {"inflation", wallInflation()},
              {"verdict", verdict},
              {"summary", summary},
              {"wallKinds", std::move(wallKinds)},
              {"raster", std::move(raster)},
              {"strokes", std::move(strokes)},
              {"slots", std::move(slotCells)},
              {"nodes", std::move(nodes)},
              {"edges", std::move(edges)},
              {"aside", std::move(aside)},
              {"wires", std::move(outerWires)}};
    if (bySlots) {
      data = judgedBySlots(std::move(data));
    }
    if (graphOnly_) {
      capturedGraph_ = data.dump();
    } else {
      record("capacity-graph", data);
    }
  }

  /// The capacity graph of a pass of the insertion's check in memory
  /// (`slotGraphOf`), with the mouths where the pass put them.
  [[nodiscard]] SlotGraph slotGraphOfPass(const std::vector<Wire>& wires,
                                          const CapacityPass& pass) const {
    SlotGraph out;
    fillSlotGraphOfPass(out, wires, pass);
    return out;
  }

  /// `slotGraphOfPass` into `out`, whose buffers keep their capacity. With
  /// `asView` the raster is a view over the pass's buffers
  /// (`SlotGraph::raster`); the pass must then outlive the check.
  void fillSlotGraphOfPass(SlotGraph& out, const std::vector<Wire>& wires,
                           const CapacityPass& pass,
                           const bool asView = false) const {
    std::vector<MeasuredBottleneck> measured;
    measured.reserve(pass.gates.size());
    for (std::size_t index = 0; index < pass.gates.size(); ++index) {
      measured.push_back(
          {.gate = pass.gates[index],
           .mouth = index >= pass.mouthsFrom && index < pass.slotsFrom,
           .slot = index >= pass.slotsFrom});
    }
    fillSlotGraph(out, wires, pass.walls, pass.gates, measured, pass.chambers,
                  pass.problem.graph, pass.problem.outerWires, asView);
  }

  /// The capacity graph of `writeCapacityGraph` in memory, as the slot
  /// check reads it (`SlotGraph`): the same cells, port runs, edges and
  /// wires, without the JSON.
  [[nodiscard]] SlotGraph
  slotGraphOf(const std::vector<Wire>& wires, const Walls& walls,
              const std::vector<grid::Bottleneck>& gates,
              const std::vector<MeasuredBottleneck>& measured,
              const grid::Chambers& chambers,
              const std::vector<GraphEdge>& graph,
              const std::vector<GraphWire>& outer) const {
    SlotGraph out;
    fillSlotGraph(out, wires, walls, gates, measured, chambers, graph, outer);
    return out;
  }

  /// `slotGraphOf` into `out`. Every field is set again; the vectors are
  /// cleared and refilled, so a graph used again keeps its buffers, the
  /// cell raster above all, which is a cell for every cell of the grid.
  /// With `asView` no raster is written: `raster` points at `walls` and
  /// `chambers`, and only the stub cells are collected, from the words of
  /// the wall mask that hold a wall.
  void fillSlotGraph(SlotGraph& out, const std::vector<Wire>& wires,
                     const Walls& walls,
                     const std::vector<grid::Bottleneck>& gates,
                     const std::vector<MeasuredBottleneck>& measured,
                     const grid::Chambers& chambers,
                     const std::vector<GraphEdge>& graph,
                     const std::vector<GraphWire>& outer,
                     const bool asView = false) const {
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto cellAt = [width](const std::size_t cell) {
      return std::array{
          static_cast<double>(static_cast<std::int64_t>(cell) % width),
          static_cast<double>(static_cast<std::int64_t>(cell) / width)};
    };
    const auto names = [&wires](const std::vector<std::uint32_t>& keys) {
      std::vector<std::string> list;
      list.reserve(keys.size());
      for (const auto key : keys) {
        list.push_back(wireId(wires[key]));
      }
      return list;
    };
    out.width = width;
    out.height = static_cast<std::int64_t>(scene_.router.height);
    out.clearance = static_cast<double>(tuning_.clearance);
    out.crossingReach = static_cast<double>(tuning_.crossingReach + 1);
    out.inflation = wallInflation();
    out.capacityMargin = tuning_.capacityMargin;
    out.capacityMarginFromWires = tuning_.capacityMarginFromWires;
    out.resonatorPassage = tuning_.resonatorPassage;
    out.slotPinCells = tuning_.slotPinCells;
    out.escapeStraight = tuning_.escapeStraight;
    out.escapeStubMargin = tuning_.escapeStubMargin;
    out.wallKinds.clear();
    out.stubCells.clear();
    out.portSlots.clear();
    out.strokes.clear();
    out.edges.clear();
    out.wires.clear();
    for (const auto kind :
         {WallKind::Border, WallKind::Artwork, WallKind::Coupler,
          WallKind::Stub, WallKind::Feedline}) {
      out.wallKinds.emplace_back(kindName(kind));
    }
    const auto stub =
        static_cast<std::uint16_t>(1 + static_cast<int>(WallKind::Stub));
    // The raster 64 cells at a time, by the words of the wall mask: a word
    // with no wall cell holds chambers and cut lines only. A cell of a wall
    // takes its kind's code; one of no listed wall is artwork.
    std::vector<std::uint16_t> kindCode(walls.list.size());
    for (std::size_t wall = 0; wall < walls.list.size(); ++wall) {
      kindCode[wall] = static_cast<std::uint16_t>(
          1 + static_cast<int>(walls.list[wall].kind));
    }
    const auto artwork =
        static_cast<std::uint16_t>(1 + static_cast<int>(WallKind::Artwork));
    const auto cells = chambers.of.size();
    const auto& maskWords = walls.mask.words();
    if (asView) {
      out.codes.clear();
      viewWallCodes_ = std::move(kindCode);
      out.raster = SlotGraph::RasterView{.wallMask = maskWords.data(),
                                         .wallOf = walls.of.data(),
                                         .wallCodes = viewWallCodes_.data(),
                                         .walls = viewWallCodes_.size(),
                                         .otherWall = artwork,
                                         .chamberOf = chambers.of.data(),
                                         .noChamber = grid::NO_CHAMBER};
      // The stubs are walls the pass drew (`Walls::drawn`): the border and
      // the artwork hold none.
      const auto& drawnWords = walls.drawn.words().size() == maskWords.size()
                                   ? walls.drawn.words()
                                   : maskWords;
      for (std::size_t first = 0; first < cells; first += 64) {
        auto mask = drawnWords[first / 64];
        while (mask != 0) {
          const auto cell =
              first + static_cast<std::size_t>(std::countr_zero(mask));
          mask &= mask - 1;
          if (out.raster->code(cell) == stub) {
            out.stubCells.push_back(cell);
          }
        }
      }
    } else {
      out.raster.reset();
      out.codes.resize(cells);
    }
    for (std::size_t first = 0; !asView && first < cells; first += 64) {
      const auto last = std::min(first + 64, cells);
      const auto mask = maskWords[first / 64];
      for (std::size_t cell = first; cell < last; ++cell) {
        if (((mask >> (cell - first)) & 1U) == 0) {
          const auto chamber = chambers.of[cell];
          out.codes[cell] = chamber != grid::NO_CHAMBER
                                ? static_cast<std::uint16_t>(6 + chamber)
                                : std::uint16_t{0};
          continue;
        }
        const auto wall = walls.of[cell];
        const auto code = wall < kindCode.size() ? kindCode[wall] : artwork;
        out.codes[cell] = code;
        if (code == stub) {
          out.stubCells.push_back(cell);
        }
      }
    }
    for (const auto& [wall, points] : walls.strokes) {
      SlotGraph::Stroke one{.name = walls.list[wall].name,
                            .kind =
                                std::string(kindName(walls.list[wall].kind)),
                            .points = {}};
      one.points.reserve(points.size());
      for (const auto& [x, y] : points) {
        one.points.push_back({static_cast<double>(x), static_cast<double>(y)});
      }
      out.strokes.push_back(std::move(one));
    }
    for (const auto cell : walls.slots) {
      out.portSlots.push_back({static_cast<std::int64_t>(cell) % width,
                               static_cast<std::int64_t>(cell) / width});
    }
    for (const auto& edge : graph) {
      SlotGraph::Edge one;
      one.stretch = edge.crossing;
      one.chambers.assign(edge.chambers.begin(), edge.chambers.end());
      one.capacity = edge.capacity;
      if (edge.crossing) {
        one.feedline = wireId(wires[edge.id]);
        one.name = std::format("× {} cells {}..{}", one.feedline, edge.first,
                               edge.last);
      } else {
        const auto& gate = gates[edge.id];
        one.name = std::format("g{}", edge.id);
        one.ends = {endName(walls, gate.first), endName(walls, gate.second)};
        one.from = cellAt(gate.first);
        one.to = cellAt(gate.second);
        one.mouth = measured[edge.id].mouth;
        one.length = edge.length;
      }
      if (edge.users.has_value()) {
        one.users = names(*edge.users);
      }
      out.edges.push_back(std::move(one));
    }
    // The launcher a plain wire starts at, which its wire does not name:
    // the launcher port nearest the start of its way.
    const auto launcherAt = [this](const PathPoint& end) -> std::uint32_t {
      std::uint32_t nearest = NO_OWNER;
      std::int64_t best = std::numeric_limits<std::int64_t>::max();
      for (const auto& port : ports_) {
        if (!scene_.launcherCell.contains(port.index)) {
          continue;
        }
        const auto dx = port.x - static_cast<std::int64_t>(end.x);
        const auto dy = port.y - static_cast<std::int64_t>(end.y);
        if ((dx * dx) + (dy * dy) < best) {
          best = (dx * dx) + (dy * dy);
          nearest = port.index;
        }
      }
      return nearest;
    };
    // A port as the graph gives it: the cell it is placed by, the chambers
    // it lies in, and what the chip calls it.
    const auto endOf = [&](const std::uint32_t index, const PathPoint& leaves,
                           const std::vector<std::uint32_t>& in,
                           const std::string_view otherwise) {
      return SlotGraph::End{.cell = {static_cast<double>(leaves.x),
                                     static_cast<double>(leaves.y)},
                            .chambers = {in.begin(), in.end()},
                            .label = index < ports_.size()
                                         ? ports_[index].label
                                         : std::string(otherwise)};
    };
    for (const auto& one : outer) {
      const auto& wire = wires[one.wire];
      SlotGraph::Wire entry{.name = wireId(wire),
                            .source = {},
                            .target = {},
                            .crosses = names(one.crosses),
                            .terminal = one.terminal,
                            .resonator = wire.resonator};
      if (one.terminal) {
        entry.source =
            endOf(one.sourcePort, one.ends[0], one.from, "coupler port");
        entry.target =
            endOf(one.targetPort, one.ends[1], one.to, "coupler port");
      } else {
        entry.source =
            wire.couplerAtSource != NO_OWNER
                ? endOf(wire.sourcePort, one.ends[0], one.from, "coupler port")
                : endOf(wire.sourcePort < ports_.size()
                            ? wire.sourcePort
                            : launcherAt(wire.objective.source),
                        one.ends[0], one.from, "");
        entry.target = endOf(wire.targetPort, one.ends[1], one.to, "");
      }
      out.wires.push_back(std::move(entry));
    }
    out.chambers = chambers.count;
  }

  /// Where the capacity graph puts the node of each chamber: the cell of
  /// it nearest the mean of its cells.
  [[nodiscard]] std::vector<Cell>
  chamberNodes(const grid::Chambers& chambers) const {
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    std::vector<double> sumX(chambers.count, 0.0);
    std::vector<double> sumY(chambers.count, 0.0);
    std::vector<double> cells(chambers.count, 0.0);
    for (std::size_t cell = 0; cell < chambers.of.size(); ++cell) {
      const auto chamber = chambers.of[cell];
      if (chamber != grid::NO_CHAMBER) {
        sumX[chamber] +=
            static_cast<double>(static_cast<std::int64_t>(cell) % width);
        sumY[chamber] +=
            static_cast<double>(static_cast<std::int64_t>(cell) / width);
        cells[chamber] += 1.0;
      }
    }
    std::vector<Cell> node(chambers.count, {0, 0});
    std::vector<double> best(chambers.count,
                             std::numeric_limits<double>::max());
    for (std::size_t cell = 0; cell < chambers.of.size(); ++cell) {
      const auto chamber = chambers.of[cell];
      if (chamber == grid::NO_CHAMBER) {
        continue;
      }
      const auto x = static_cast<std::int64_t>(cell) % width;
      const auto y = static_cast<std::int64_t>(cell) / width;
      const auto dx = static_cast<double>(x) - (sumX[chamber] / cells[chamber]);
      const auto dy = static_cast<double>(y) - (sumY[chamber] / cells[chamber]);
      const auto apart = (dx * dx) + (dy * dy);
      if (apart < best[chamber]) {
        best[chamber] = apart;
        node[chamber] = {x, y};
      }
    }
    return node;
  }

private:
  const Scene& scene_;
  Tuning tuning_;
  Report report_;
  /// What the coupler insertion came to, for the figures of its phase and of
  /// the stage: whether it ran, how many of its edges it drew and at what
  /// angle cost, and whether the chip is SAT by the capacity check, nothing
  /// while the check has not judged it.
  bool insertionDone_ = false;
  std::uint32_t insertionDrawn_ = 0;
  std::uint32_t insertionAngle_ = 0;
  std::optional<bool> sat_;
  Debug debug_;
  /// The record kinds the `[debug]` section asks for.
  DebugRecords wanted_;
  /// The phase that runs, for the search records and the dashboards.
  std::string phase_;
  /// The capacity graph as data, kept for `capacityGraphJson` while
  /// `graphOnly_` holds.
  std::string capturedGraph_;
  DebugFrame frame_;
  /// Per chain, the options of every waypoint of its locally SAT states,
  /// the state it stands on first (`collectChainStates`).
  std::vector<std::vector<std::vector<std::size_t>>> chainStates_;
  /// A run of options of a segment and the ways of its edges, handed to
  /// `collectChainStates` by the segment search instead of being tested.
  using RunCollector = std::function<bool(const std::vector<std::size_t>&,
                                          const std::vector<Path>&)>;
  const RunCollector* segmentCollect_ = nullptr;
  /// Counts the sweeps, so that a wire's `relaxedOut` names one.
  std::uint32_t sweepSerial_ = 0;
  /// The ports as the chip carries them, for the pictures.
  std::vector<PortMark> ports_;
  /// What each round of the pass running now came to, per wire.
  struct RoundRecord {
    bool forward = true;
    std::uint32_t normal = 0;
    std::uint32_t relaxed = 0;
    /// Of the failed, the resonators that kept a way too short.
    std::uint32_t tooShort = 0;
    std::vector<std::string> failed;
    /// Every member's way as the round left it, by name; kept for the last
    /// two rounds of a sweep only.
    std::vector<std::pair<std::string, Path>> ways;
  };
  std::vector<RoundRecord> rounds_;
  std::shared_ptr<const MovePrimitives> primitives_;
  routing::AnalyticDubins analytic_;
  routing::SearchScratch scratch_;
  routing::DubinsRouter router_;
  Field field_;
  grid::BitGrid corridor_;
  std::vector<std::uint16_t> proximity_;
  std::vector<std::uint32_t> seen_;
  std::vector<std::size_t> queue_;
  std::vector<std::uint32_t> depth_;
  std::vector<std::int32_t> wall_;
  std::vector<std::int32_t> middle_;
  std::unordered_map<std::uint32_t, Stencil> stencils_;
  Box box_;
  /// Whether `corridorOfEdge` opened a slot cell outside `box_`. Every
  /// other cell outside the box stays closed.
  bool slotLeftBox_ = false;
  /// The cells of the corridor opened since `buildCorridor` began it, as a
  /// box: every open cell of the corridor lies inside it. See `priceRoom`.
  Box openBox_;
  std::uint32_t pass_ = 0;
  mutable routing::PathLoopScratch loopScratch_;
  /// Whether a resonator is made the target length exactly, once its
  /// coupler is in, rather than at least `meander_length`.
  bool exact_ = false;
  /// Whether the pass running now is under the feedline constraints.
  bool feedlinePass_ = false;
  /// The cells the coupler bodies take.
  grid::BitGrid bodies_;
  /// Whether the corridor being built closes the coupler bodies. A feedline
  /// runs along a body on purpose, so for a feedline it does not.
  bool blockOnBodies_ = true;
  /// One byte per wire: whether it is a conventional wire, which the option
  /// price weighs heavier than a resonator.
  std::vector<std::uint8_t> conventional_;
  /// A price field of nothing, for the searches that price nothing. Filled
  /// once and never written again.
  std::vector<std::uint16_t> zeroProximity_;

  /// The way one edge of a chain was found to take, kept by the pair of
  /// coupler options at its two ends.
  ///
  /// What an edge runs between is those two options and nothing else: move
  /// either coupler and the edge is a different question, leave both and it
  /// is the same one. The greedy asks it again and again — every option of
  /// every coupler is weighed against both neighbours once per pass, and a
  /// pair it has already routed comes back on the next pass and the next.
  ///
  /// The pair is the whole key. What else could bear on the answer — the
  /// other chains, fencing this one — is deliberately left out: taking it
  /// in would make every entry stale on every move and there would be
  /// nothing left to keep. The price is that an entry can outlive the
  /// ground it was found on; the fails say what that costs.
  /// The greedy leaves the staleness above standing. The exact search does
  /// not: at the head of every round it forgets the entries whose corridor
  /// a way that moved could have changed, and keeps the rest — see
  /// `forgetEdgesNear`.
  std::unordered_map<std::uint64_t, Path> edgeMemo_;
  std::uint64_t memoHits_ = 0;
  std::uint64_t memoMisses_ = 0;
  /// Remembered ways the prefix fence ran into, routed again.
  std::uint64_t memoStale_ = 0;

  /// The option a waypoint stands on; a launcher has none and counts zero.
  [[nodiscard]] std::size_t optionAt(const std::uint32_t chain,
                                     const std::size_t at) const {
    const auto& point = chains_[chain][at];
    return point.fixed ? 0 : couplers_[point.coupler].chosen + 1;
  }

  /// `ahead` is the option the waypoint **before** this edge stands on, plus
  /// one, and zero for an edge that has no predecessor or is being priced
  /// first order. It belongs in the key because the second-order search
  /// fences the way that predecessor takes, so one pair of endpoint options
  /// can have two different ways under two different predecessors. Zero is
  /// therefore the first-order slot, which is what the greedy uses and what
  /// the second order reads its own fence out of.
  [[nodiscard]] std::uint64_t edgeMemoKey(const std::uint32_t chain,
                                          const std::size_t from,
                                          const std::size_t ahead = 0) const {
    return (static_cast<std::uint64_t>(chain & 0xFFU) << 56U) |
           (static_cast<std::uint64_t>(from & 0xFFU) << 48U) |
           (static_cast<std::uint64_t>(ahead & 0xFFFFU) << 32U) |
           (static_cast<std::uint64_t>(optionAt(chain, from) & 0xFFFFU)
            << 16U) |
           static_cast<std::uint64_t>(optionAt(chain, from + 1) & 0xFFFFU);
  }

  /// The approaches of every port, inflated by the clearance, where no
  /// coupler may sit.
  grid::BitGrid approaches_;
  /// The rectangle the launcher stubs leave open, which a coupler sits in.
  CouplerBox couplerBox_;
  /// The meanders the resonators' ways carry, by wire: the way as it came out
  /// of the insertion and the box its meander spans, the last few per wire,
  /// so that a way a refinement went back to still finds its own.
  std::unordered_map<std::uint32_t,
                     std::vector<std::pair<Path, routing::CellBox>>>
      meanders_;
  /// The cells the feedline edges keep clear of in the coupler insertion:
  /// the box of every meander a resonator's way carries, grown by the wire
  /// clearance.
  grid::BitGrid meanderKeepout_;
  std::vector<Coupler> couplers_;
  /// The coupler of every resonator that has one, by wire key.
  std::unordered_map<std::uint32_t, std::uint32_t> couplerOfWire_;
  std::vector<std::vector<Waypoint>> chains_;
  std::vector<Edge> edges_;
  /// The way of every edge of every chain as the greedy last chose it, per
  /// chain in edge order, until the edge has a wire of its own. What is
  /// committed stands in the way of every edge routed after it.
  std::vector<std::vector<Path>> chainEdgePaths_;
  /// The wire of every edge of every chain once it is committed, `NO_OWNER`
  /// before.
  std::vector<std::vector<std::uint32_t>> edgeWireOf_;
};

// -------------------------------------------------------- Building the wires

/// The cell of the router grid a cell of the detail grid falls on.
[[nodiscard]] PathPoint onRouter(const grid::GridMetrics& detail,
                                 const grid::GridMetrics& router,
                                 const fbg::DCoord& cell) {
  const auto place = detail.toLayout(cell.x(), cell.y());
  const auto found = router.clampToCell(place);
  return {.x = found.x(), .y = found.y(), .heading = 0, .primitive = 0};
}

/// The way the Detail stage drew, on the grid this stage draws on.
///
/// It is a seed and nothing else. The Detail stage's cells are a centre line
/// on a grid three to four times coarser, so what comes back is a chain of
/// cells with gaps; every use of it here is a set of places to spread from or
/// a polygon to price against, and neither needs the chain to be unbroken.
[[nodiscard]] Path seedOf(const grid::GridMetrics& detail,
                          const grid::GridMetrics& router,
                          const std::vector<fbg::DCoord>& path) {
  Path way;
  way.reserve(path.size());
  for (const auto& cell : path) {
    const auto point = onRouter(detail, router, cell);
    if (way.empty() || !point.samePlace(way.back())) {
      way.push_back(point);
    }
  }
  return way;
}

[[nodiscard]] grid::GridMetrics metricsOf(const fba::GridExtentT& extent) {
  return {.width = extent.width,
          .height = extent.height,
          .origin = extent.origin,
          .cellWidth = extent.cell_width,
          .cellHeight = extent.cell_height};
}

/// Every wire the stage has to draw: the ring first, in the order the
/// assignment holds it, and the inner circuit after it.
[[nodiscard]] std::vector<Wire> wiresOf(const ChipT& chip,
                                        const GlobalRoutingT& global,
                                        const AssignmentT& assignment,
                                        const DetailRoutingT& detail,
                                        const Scene& scene) {
  const auto detailGrid = metricsOf(*detail.grid);
  std::vector<Wire> wires;
  wires.reserve(assignment.connections.size() + global.connections.size());

  const auto place =
      [&](const std::uint32_t port, const bool leaving,
          PathPoint& into) { // NOLINT(bugprone-easily-swappable-parameters)
        const auto cell = scene.targetCell.find(port);
        const auto heading = scene.arrival.find(port);
        if (cell == scene.targetCell.end() || heading == scene.arrival.end()) {
          return false;
        }
        const auto found = scene.router.cell(cell->second);
        into = {.x = found.x(),
                .y = found.y(),
                .heading = leaving ? routing::reverse(heading->second)
                                   : heading->second,
                .primitive = 0};
        return true;
      };

  for (std::uint32_t index = 0; index < assignment.connections.size();
       ++index) {
    const auto& connection = *assignment.connections[index];
    Wire wire;
    wire.slot = index;
    wire.key = static_cast<std::uint32_t>(wires.size());
    wire.resonator =
        connection.target_role == fbd::AssignedRole::ResonatorTarget;
    if (index < detail.wires.size()) {
      wire.way = seedOf(detailGrid, scene.router, detail.wires[index]->path);
    }

    // A wire starts where the assignment feeds it, which is a point on the
    // ring and not a port: a resonator is fed on the segment between two
    // launchers, so no port stands there. The heading it leaves with is the
    // reverse of the one a wire arrives at its launcher with.
    const bool fed =
        index < assignment.feeds.size() && index < assignment.launchers.size();
    if (fed) {
      const auto launcher = assignment.launchers[index].index();
      if (launcher < chip.ports.size()) {
        const auto heading =
            routing::headingOfOrientation(chip.ports[launcher]->orientation);
        const auto cell = scene.router.clampToCell(assignment.feeds[index]);
        wire.objective.source = {.x = cell.x(),
                                 .y = cell.y(),
                                 .heading = routing::reverse(heading),
                                 .primitive = 0};
        wire.feasible = heading < routing::NUM_HEADINGS;
      }
    }
    wire.targetPort = connection.target.index();
    if (!place(connection.target.index(), false, wire.objective.target)) {
      wire.feasible = false;
    }
    wire.feedsABridge = wire.resonator &&
                        connection.target.index() < chip.ports.size() &&
                        chip.ports[connection.target.index()]->role ==
                            fbd::UnassignedRole::BridgePair;
    // A resonator's length runs to the port itself, and the way ends on the
    // cell beyond the port's band, so the gap between the two is part of
    // the length. The port's exact position is in fractional cells, as the
    // sampler measures.
    if (wire.resonator && wire.feasible &&
        connection.target.index() < chip.ports.size()) {
      const auto exact =
          scene.router.toCell(chip.ports[connection.target.index()]->center);
      const auto dx = static_cast<double>(wire.objective.target.x) - exact.x();
      const auto dy = static_cast<double>(wire.objective.target.y) - exact.y();
      wire.anchorGap = std::hypot(dx, dy);
      wire.anchorGapUnits =
          std::hypot(dx * scene.router.cellWidth, dy * scene.router.cellHeight);
    }
    wires.push_back(std::move(wire));
  }

  for (std::uint32_t index = 0; index < global.connections.size(); ++index) {
    const auto& connection = *global.connections[index];
    Wire wire;
    wire.inner = true;
    wire.slot = index;
    wire.key = static_cast<std::uint32_t>(wires.size());
    if (index < detail.inner.size()) {
      wire.way = seedOf(detailGrid, scene.router, detail.inner[index]->path);
    }
    wire.targetPort = connection.target.index();
    if (connection.source != nullptr) {
      wire.sourcePort = connection.source->index();
    }
    wire.feasible =
        connection.source != nullptr &&
        place(connection.source->index(), true, wire.objective.source) &&
        place(connection.target.index(), false, wire.objective.target);
    const auto roleOf = [&chip](const std::uint32_t port) {
      return port < chip.ports.size() ? chip.ports[port]->role
                                      : fbd::UnassignedRole::Unset;
    };
    wire.resonatorBeyondBridge =
        connection.source != nullptr &&
        roleOf(connection.source->index()) == fbd::UnassignedRole::BridgePair &&
        roleOf(connection.target.index()) == fbd::UnassignedRole::Resonator;
    // Its length runs to the port itself, as a resonator's does.
    if (wire.resonatorBeyondBridge && wire.feasible) {
      const auto exact =
          scene.router.toCell(chip.ports[connection.target.index()]->center);
      const auto dx = static_cast<double>(wire.objective.target.x) - exact.x();
      const auto dy = static_cast<double>(wire.objective.target.y) - exact.y();
      wire.anchorGap = std::hypot(dx, dy);
      wire.anchorGapUnits =
          std::hypot(dx * scene.router.cellWidth, dy * scene.router.cellHeight);
    }
    wires.push_back(std::move(wire));
  }
  return wires;
}

// ------------------------------------------------------------- The artifact

/// How long a way is, in layout units.
using Measure = std::function<double(const Path&)>;

/// What the insertion says about an edge, by wire key: the `Squeezed` bit,
/// the note behind it and the line it measured. See `Driver::reportSqueeze`.
using Notes = std::unordered_map<std::uint32_t, Driver::Squeeze>;

[[nodiscard]] std::unique_ptr<fba::FinalWireT>
wireOf(const Path& way, const double length, const std::uint8_t verdict = 0,
       const Driver::Squeeze* squeeze = nullptr) {
  auto drawn = std::make_unique<fba::FinalWireT>();
  drawn->path.reserve(way.size());
  for (const auto& point : way) {
    drawn->path.emplace_back(point.x, point.y, point.heading);
  }
  drawn->length = length;
  drawn->verdict = static_cast<fba::FinalVerdict>(verdict);
  if (squeeze != nullptr) {
    drawn->note = squeeze->note;
    drawn->marks.emplace_back(squeeze->from.x, squeeze->from.y,
                              squeeze->from.heading);
    drawn->marks.emplace_back(squeeze->to.x, squeeze->to.y,
                              squeeze->to.heading);
  }
  return drawn;
}

/// The couplers as the artifact carries them.
using CouplerList = std::vector<std::unique_ptr<fbd::CpwCouplerT>>;

[[nodiscard]] CouplerList copyOf(const CouplerList& couplers) {
  CouplerList copy;
  copy.reserve(couplers.size());
  for (const auto& coupler : couplers) {
    copy.push_back(std::make_unique<fbd::CpwCouplerT>(*coupler));
  }
  return copy;
}

/// The state of every wire, as one phase left it. `verdicts` is what the
/// stage holds against each wire by key, as `failsOf` counted it, and only
/// the end state is handed one: a phase snapshot is not judged and carries
/// zero on every wire.
[[nodiscard]] std::unique_ptr<fba::FinalPhaseT>
snapshotOf(std::string name, const std::vector<Wire>& wires,
           const std::uint32_t ring, const std::size_t inner,
           const Measure& measure, const std::vector<std::uint32_t>& edges = {},
           const CouplerList& couplers = {},
           const std::vector<std::uint8_t>* verdicts = nullptr,
           const Notes* notes = nullptr) {
  const auto verdictOf = [&](const std::uint32_t key) -> std::uint8_t {
    std::uint8_t bits =
        verdicts != nullptr && key < verdicts->size() ? (*verdicts)[key] : 0;
    if (notes != nullptr && notes->contains(key)) {
      bits |= static_cast<std::uint8_t>(fba::FinalVerdict::Squeezed);
    }
    return bits;
  };
  const auto noteOf =
      [notes](const std::uint32_t key) -> const Driver::Squeeze* {
    if (notes == nullptr) {
      return nullptr;
    }
    const auto found = notes->find(key);
    return found == notes->end() ? nullptr : &found->second;
  };
  const auto entryOf = [&](const Wire& wire) {
    return wire.drawn ? wireOf(wire.way, measure(wire.way), verdictOf(wire.key),
                               noteOf(wire.key))
                      : wireOf({}, 0.0, verdictOf(wire.key), noteOf(wire.key));
  };
  auto phase = std::make_unique<fba::FinalPhaseT>();
  phase->name = std::move(name);
  for (const auto key : edges) {
    phase->feedlines.push_back(entryOf(wires[key]));
  }
  phase->couplers = copyOf(couplers);
  phase->wires.reserve(ring);
  for (std::uint32_t at = 0; at < ring; ++at) {
    phase->wires.push_back(wireOf({}, 0.0));
  }
  phase->inner.reserve(inner);
  for (std::size_t at = 0; at < inner; ++at) {
    phase->inner.push_back(wireOf({}, 0.0));
  }
  for (const auto& wire : wires) {
    if (wire.feedline) {
      continue;
    }
    auto& into = wire.inner ? phase->inner : phase->wires;
    if (wire.slot < into.size()) {
      // An undrawn wire stays an empty entry, as before, and carries the
      // verdict against it — `Unrouted`, when the end state is judged.
      into[wire.slot] = entryOf(wire);
    }
  }
  return phase;
}

/// The degrees a heading stands for, the inverse of `headingOfOrientation`.
[[nodiscard]] double degreesOf(const Heading heading) {
  switch (heading & 7U) {
  case 2:
    return 0.0;
  case 1:
    return 45.0;
  case 0:
    return 90.0;
  case 7:
    return 135.0;
  case 6:
    return 180.0;
  case 5:
    return 225.0;
  case 4:
    return 270.0;
  default:
    return 315.0;
  }
}

/// The port a waypoint of a chain stands for: the launcher, or the port
/// the coupler created, which follows the chip's own ports in coupler order.
[[nodiscard]] fbd::PortRef portOf(const Driver& driver, const ChipT& chip,
                                  const std::uint32_t chain,
                                  const std::size_t at) {
  const auto& point = driver.chains()[chain][at];
  if (point.fixed) {
    return fbd::PortRef(point.port);
  }
  return fbd::PortRef(static_cast<std::uint32_t>(chip.ports.size()) +
                      point.coupler);
}

/// The couplers as they stand, for the artifact: the connection each
/// completes, the port it creates, and its body in layout units.
[[nodiscard]] CouplerList couplersOf(const Driver& driver,
                                     const std::vector<Wire>& wires,
                                     const ChipT& chip, const Scene& scene) {
  CouplerList list;
  for (std::uint32_t index = 0; index < driver.couplers().size(); ++index) {
    const auto& coupler = driver.couplers()[index];
    const auto& option = coupler.options[coupler.chosen];
    const auto& wire = wires[coupler.wire];
    auto made = std::make_unique<fbd::CpwCouplerT>();
    made->connection = fbd::ConnectionRef(wire.slot);
    made->port = std::make_unique<fbd::PortT>();
    made->port->label =
        std::format("{}.coupler", wire.targetPort < chip.ports.size()
                                      ? chip.ports[wire.targetPort]->label
                                      : std::to_string(index));
    made->port->center =
        scene.router.toLayout(option.anchor.x, option.anchor.y);
    // A wire arrives at the port against the heading the resonator leaves
    // its anchor on.
    made->port->orientation =
        degreesOf(routing::reverse(option.way.front().heading));
    made->port->role = fbd::UnassignedRole::Coupler;
    // The body is a pad centred on the anchor and turned to the option's
    // orientation — not to the heading the resonator happens to arrive on.
    // Drawing it on the arrival heading is what put the couplers at the
    // wrong angle in the picture while their cells sat right.
    const auto along = routing::headingVector(option.orientation);
    const auto across =
        routing::headingVector(routing::turned(option.orientation, 2));
    (void)across;
    // On the pad's own centre, which is the place the coupler was put at.
    // Not on the anchor: that is the resonator port on the pad's near edge,
    // and a body drawn on it hangs half a pad off where the coupler sits.
    made->center = scene.router.toLayout(option.centre.x, option.centre.y);
    // The rectangle is drawn on its own long axis, which is the axis the
    // feedline runs along. The coupler's orientation stands perpendicular to
    // it and is not what a rectangle is rotated by.
    made->rotation = static_cast<fbd::Rotation>(
        1 + static_cast<int>(degreesOf(option.orientation) / 45.0));
    // In layout units, by the step the cells are counted in: a diagonal step
    // is the diagonal of a cell, so a body that is fewer cells along a
    // diagonal is the length the design rule asks for all the same.
    const auto step = [&scene](const routing::HeadingVector v) {
      return std::hypot(v.dx * scene.router.cellWidth,
                        v.dy * scene.router.cellHeight);
    };
    made->length = option.run * step(along);
    made->height =
        option.depth *
        step(routing::headingVector(routing::turned(option.orientation, 2)));
    list.push_back(std::move(made));
  }
  return list;
}

/// The final router of the first release.
/// The pass of phase 1, the inner circuit.
[[nodiscard]] Pass innerPassOf(const Tuning& tuning) {
  return {.name = "inner routing",
          .rounds = tuning.innerRounds,
          .maxRelaxation = tuning.maxRelaxation,
          .reach = tuning.innerReach,
          .straightStart = 0,
          .keepDrawn = true};
}

/// The pass of phase 2, the ring.
[[nodiscard]] Pass outerPassOf(const Tuning& tuning) {
  return {.name = "outer routing",
          .rounds = tuning.rounds,
          .maxRelaxation = tuning.maxRelaxation,
          .reach = tuning.reach,
          .straightStart = tuning.straightStart,
          .keepDrawn = true};
}

/// The refinement that ends phase 2.
[[nodiscard]] Pass refinementPassOf(const Tuning& tuning) {
  return {.name = "refinement",
          .rounds = tuning.refinementRounds,
          .maxRelaxation = 0,
          .reach = tuning.refinementReach,
          .straightStart = tuning.straightStart,
          .keepDrawn = false,
          .refinement = true};
}

/// The ring, against the inner circuit and against itself: the second half
/// of the first phase.
void routeTheRing(Driver& driver, std::vector<Wire>& wires,
                  const std::vector<std::uint32_t>& outer,
                  const Tuning& tuning) {
  driver.sayOuterSettings();
  const auto outerPass = outerPassOf(tuning);
  driver.sweep(wires, outer, outerPass);
  driver.recoverWithoutBands(wires, outer, outerPass);
}

/// The second phase: the refinement of the ring before the feedlines.
void refineTheRing(Driver& driver, std::vector<Wire>& wires,
                   const std::vector<std::uint32_t>& outer,
                   const Tuning& tuning) {
  driver.refine(wires, outer, refinementPassOf(tuning));
  driver.sayLengthPoints(wires, outer);
}

/// The pass of the feedline routing: every wire of the ring with the edges
/// of the chains, under the feedline constraints.
[[nodiscard]] Pass feedlinePassOf(const Tuning& tuning) {
  return {.name = "feedline routing",
          .rounds = tuning.rounds,
          .maxRelaxation = tuning.maxRelaxation,
          .reach = tuning.reach,
          .straightStart = tuning.straightStart,
          .keepDrawn = true,
          .feedlines = true};
}

/// Phase 4 of the stage: every wire again under the feedline constraints,
/// in the ring with the edges of the chains, then the inner circuit; and the
/// repair, which turns a coupler while the feedlines leave fails. Returns the
/// ring with the edges of the chains, which the refinement takes.
/// `rounds`, when given, receives what every round of the feedline sweep
/// came to (`Driver::roundsJson`).
[[nodiscard]] std::vector<std::uint32_t>
routeTheFeedlines(Driver& driver, std::vector<Wire>& wires,
                  const std::vector<std::uint32_t>& outer,
                  const std::vector<std::uint32_t>& inner, const Tuning& tuning,
                  nlohmann::json* rounds = nullptr) {
  driver.assignBridges(wires, outer);
  driver.checkBridgers(wires);
  auto members = driver.ringWithEdges(wires, outer);
  const auto feedlinePass = feedlinePassOf(tuning);
  driver.sweep(wires, members, feedlinePass);
  if (rounds != nullptr) {
    *rounds = driver.roundsJson();
  }
  driver.sweep(wires, inner,
               {.name = "inner routing under the feedlines",
                .rounds = tuning.innerRounds,
                .maxRelaxation = tuning.maxRelaxation,
                .reach = tuning.innerReach,
                .straightStart = 0,
                .keepDrawn = true,
                .feedlines = true});
  std::vector<std::uint32_t> every(wires.size());
  std::iota(every.begin(), every.end(), 0U);
  driver.repairFeedlines(wires, members, every, feedlinePass);
  return members;
}

class DubinsFinalRouter final : public IFinalRouter {
public:
  [[nodiscard]] FinalRoutingT
  run(const ChipT& chip, const CapacityPlanT& /*capacity*/,
      const GlobalRoutingT& global, const AssignmentT& assignment,
      const DetailRoutingT& detail, const ConfigT& config, const Report& report,
      const Debug& debug) const override {
    if (config.grid == nullptr || config.rules == nullptr) {
      throw std::invalid_argument(
          "the configuration carries no grid section or no design rules");
    }
    if (detail.grid == nullptr) {
      throw std::invalid_argument("the detail routing carries no grid");
    }
    const auto scene = buildScene(chip, config);
    const auto tuning =
        tuningOf(config, grid::routerGrid(scene.capacity,
                                          config.grid->router_cell_size));
    const auto field = sceneOf(chip, config, scene.capacity);

    auto wires = wiresOf(chip, global, assignment, detail, field);
    const auto ring = static_cast<std::uint32_t>(assignment.connections.size());
    std::vector<std::uint32_t> outer;
    std::vector<std::uint32_t> inner;
    for (const auto& wire : wires) {
      (wire.inner ? inner : outer).push_back(wire.key);
    }

    Driver driver(field, tuning, report, debug, recordsWantedBy(config));
    driver.attachPorts(portMarksOf(chip, field));
    driver.recordGrid(wires);
    const Measure measure = [&driver](const Path& way) {
      return driver.lengthInUnits(way);
    };

    FinalRoutingT routing;
    auto extent = std::make_unique<fba::GridExtentT>();
    extent->width = field.router.width;
    extent->height = field.router.height;
    extent->origin = field.router.origin;
    extent->cell_width = field.router.cellWidth;
    extent->cell_height = field.router.cellHeight;
    routing.grid = std::move(extent);

    // Where the run ends. `[run] stop_after` names the last phase to run,
    // by the name of its snapshot; what follows it is not run at all. An
    // unknown name runs every phase, because the loader has already refused
    // it.
    std::size_t lastPhase = PHASES.size() - 1;
    for (std::size_t at = 0; at < PHASES.size(); ++at) {
      if (tuning.stopAfter == PHASES[at]) {
        lastPhase = at;
      }
    }
    const auto runs = [&lastPhase](const std::size_t phase) {
      return phase <= lastPhase;
    };

    // Phase 1, the routing before the feedlines. The inner circuit runs
    // inside the unit cells, where the room is narrow and there is nothing to
    // steer around, so it takes no straight stubs and a band a fraction of
    // the ring's. Then the ring, against the inner circuit and against
    // itself.
    // Every wire of the plan, counted from the list as it is at the time,
    // because the coupler insertion appends the edges of the chains to it.
    const auto everyWire = [&wires]() {
      std::vector<std::uint32_t> all(wires.size());
      std::iota(all.begin(), all.end(), 0U);
      return all;
    };
    // Each phase ends on one entry of the summary.
    const auto endPhase = [&](const std::size_t phase) {
      report.entry(Detail::Summary, PHASES[phase],
                   driver.phaseFigures(phase, wires, everyWire()));
    };

    driver.sayTheSetting();
    driver.enterPhase(PHASES[0]);
    driver.sweep(wires, inner, innerPassOf(tuning));
    routeTheRing(driver, wires, outer, tuning);
    routing.phases.push_back(snapshotOf(std::string(PHASES[0]), wires, ring,
                                        global.connections.size(), measure));
    driver.recordPhase(PHASES[0], wires);
    endPhase(0);

    // Phase 2. The refinement of the ring.
    if (runs(1)) {
      driver.enterPhase(PHASES[1]);
      refineTheRing(driver, wires, outer, tuning);
      routing.phases.push_back(snapshotOf(std::string(PHASES[1]), wires, ring,
                                          global.connections.size(), measure));
      driver.recordPhase(PHASES[1], wires);
      endPhase(1);
    }

    // What the tail writes, whatever the run stops after: the edges of the
    // chains and the couplers as they stand.
    std::vector<std::uint32_t> edges;
    const auto couplersNow = [&]() {
      return couplersOf(driver, wires, chip, field);
    };

    // Phase 3. The feedline insertion: the couplers, and the feedline chains
    // through them.
    if (runs(2)) {
      driver.enterPhase(PHASES[2]);
      driver.insertCouplers(wires, assignment, chip);
      for (const auto& edge : driver.edges()) {
        edges.push_back(edge.wire);
      }
      routing.phases.push_back(snapshotOf(
          std::string(PHASES[2]), wires, ring, global.connections.size(),
          measure, edges, couplersNow(), nullptr, &driver.squeezed()));
      driver.recordPhase(PHASES[2], wires);
      endPhase(2);
    }

    // Phase 4. Every wire again under the feedline constraints, in the ring
    // with the edges of the chains, then the inner circuit; and the repair,
    // which turns a coupler while the feedlines leave fails.
    if (runs(3)) {
      driver.enterPhase(PHASES[3]);
      const auto members =
          routeTheFeedlines(driver, wires, outer, inner, tuning);
      routing.phases.push_back(snapshotOf(
          std::string(PHASES[3]), wires, ring, global.connections.size(),
          measure, edges, couplersNow(), nullptr, &driver.squeezed()));
      driver.recordPhase(PHASES[3], wires);
      endPhase(3);

      // Phase 5. The refinement of the feedline routing.
      if (runs(4)) {
        driver.enterPhase(PHASES[4]);
        driver.refine(wires, members,
                      {.name = "feedline refinement",
                       .rounds = tuning.feedlineRefinementRounds,
                       .maxRelaxation = 0,
                       .reach = tuning.refinementReach,
                       .straightStart = tuning.straightStart,
                       .keepDrawn = false,
                       .feedlines = true,
                       .refinement = true});
        routing.phases.push_back(snapshotOf(
            std::string(PHASES[4]), wires, ring, global.connections.size(),
            measure, edges, couplersNow(), nullptr, &driver.squeezed()));
        driver.recordPhase(PHASES[4], wires);
        endPhase(4);
      }
    }

    // What the stage came to, over every wire of the plan and counted with
    // every wire down, so that the last line of a run says what the
    // design-rule check will find.
    if (lastPhase + 1 < PHASES.size()) {
      driver.say(std::format("the stage stops after {}, as stop_after asks",
                             PHASES[lastPhase]));
    }
    driver.sayResonatorLengths(wires, everyWire());
    const auto fails = driver.failsOf(wires, everyWire());
    driver.reportResult(fails, static_cast<std::uint32_t>(wires.size()));

    // The end state carries the verdict against every wire, so that a
    // picture of the run marks what the last line counted.
    auto last =
        snapshotOf("", wires, ring, global.connections.size(), measure, edges,
                   couplersNow(), &fails.verdicts, &driver.squeezed());
    routing.wires = std::move(last->wires);
    routing.inner = std::move(last->inner);
    routing.feedlines = std::move(last->feedlines);
    routing.couplers = std::move(last->couplers);
    for (const auto& edge : driver.edges()) {
      auto described = std::make_unique<fba::FeedlineEdgeT>();
      described->chain = edge.chain;
      described->from = portOf(driver, chip, edge.chain, edge.from);
      described->to = portOf(driver, chip, edge.chain, edge.to);
      described->terminal = edge.terminal;
      routing.feedline_edges.push_back(std::move(described));
    }
    // A `ConnectionRef` indexes the connections of the assignment, so only a
    // wire of the ring can be named here. An inner wire that was not drawn is
    // an empty entry in `inner`, which is what a reader counts.
    for (const auto& wire : wires) {
      if (!wire.drawn && !wire.inner) {
        routing.unresolved.emplace_back(wire.slot);
      }
    }
    return routing;
  }
};

} // namespace

std::unique_ptr<IFinalRouter> makeDubinsFinalRouter() {
  return std::make_unique<DubinsFinalRouter>();
}

/// What a `CouplerSession` holds: the scene, the tuning and the wires the
/// driver works on, and the driver, which refers to the first two.
struct CouplerSession::State {
  State(Scene scene, Tuning tuning)
      : field(std::move(scene)), tuning(std::move(tuning)) {}
  Scene field;
  Tuning tuning;
  std::vector<Wire> wires;
  std::vector<std::uint32_t> outer;
  std::vector<std::uint32_t> inner;
  std::unique_ptr<Driver> driver;
};

CouplerSession::CouplerSession(const ChipT& chip, const GlobalRoutingT& global,
                               const AssignmentT& assignment,
                               const DetailRoutingT& detail,
                               const ConfigT& config, const Report& report) {
  if (config.grid == nullptr || config.rules == nullptr) {
    throw std::invalid_argument(
        "the configuration carries no grid section or no design rules");
  }
  if (detail.grid == nullptr) {
    throw std::invalid_argument("the detail routing carries no grid");
  }
  const auto scene = buildScene(chip, config);
  state_ = std::make_unique<State>(
      sceneOf(chip, config, scene.capacity),
      tuningOf(config, grid::routerGrid(scene.capacity,
                                        config.grid->router_cell_size)));
  auto& wires = state_->wires;
  wires = wiresOf(chip, global, assignment, detail, state_->field);
  auto& outer = state_->outer;
  auto& inner = state_->inner;
  for (const auto& wire : wires) {
    (wire.inner ? inner : outer).push_back(wire.key);
  }
  state_->driver = std::make_unique<Driver>(
      state_->field, state_->tuning, report, Debug{}, recordsWantedBy(config));
  auto& driver = *state_->driver;
  driver.attachPorts(portMarksOf(chip, state_->field));
  driver.sayTheSetting();
  driver.enterPhase(PHASES[0]);
  driver.sweep(wires, inner, innerPassOf(state_->tuning));
  routeTheRing(driver, wires, outer, state_->tuning);
  driver.enterPhase(PHASES[1]);
  refineTheRing(driver, wires, outer, state_->tuning);
  driver.enterPhase(PHASES[2]);
  driver.insertCouplers(wires, assignment, chip);
}

CouplerSession::~CouplerSession() = default;

std::string CouplerSession::couplers() const {
  return state_->driver->couplersJson(state_->wires);
}

std::string CouplerSession::setOption(const std::uint32_t coupler,
                                      const std::uint32_t option) {
  return state_->driver->setCouplerOption(state_->wires, coupler, option);
}

std::string CouplerSession::graph() {
  return state_->driver->capacityGraphJson(state_->wires);
}

std::string CouplerSession::insertionCheck() {
  return state_->driver->insertionCheckJson(state_->wires);
}

void CouplerSession::setDebug(Debug debug) {
  state_->driver->setDebug(std::move(debug));
}

std::string CouplerSession::compareLocally(const std::uint32_t runs,
                                           const std::size_t span) {
  auto& driver = *state_->driver;
  auto& wires = state_->wires;
  driver.enterPhase(PHASES[3]);
  const auto members = routeTheFeedlines(driver, wires, state_->outer,
                                         state_->inner, state_->tuning);
  std::vector<std::uint32_t> every(wires.size());
  std::iota(every.begin(), every.end(), 0U);
  return driver
      .compareLocally(wires, members, every, feedlinePassOf(state_->tuning),
                      runs, span)
      .dump();
}

std::string CouplerSession::routeFeedlines() {
  nlohmann::json rounds;
  state_->driver->enterPhase(PHASES[3]);
  static_cast<void>(routeTheFeedlines(*state_->driver, state_->wires,
                                      state_->outer, state_->inner,
                                      state_->tuning, &rounds));
  return state_->driver->routedJson(state_->wires, rounds);
}

} // namespace mqt::scpd::pipeline
