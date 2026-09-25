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

#include "DebugSvg.hpp"
#include "mqt-scpd/design/Bridges.hpp"
#include "mqt-scpd/design/Roles.hpp"
#include "mqt-scpd/flatbuffers/artifacts.hpp"
#include "mqt-scpd/flatbuffers/config.hpp"
#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"
#include "mqt-scpd/grid/PortBands.hpp"
#include "mqt-scpd/grid/Rasterize.hpp"
#include "mqt-scpd/pipeline/CapacityPlanner.hpp"
#include "mqt-scpd/pipeline/Stages.hpp"
#include "mqt-scpd/routing/CouplerInsertion.hpp"
#include "mqt-scpd/routing/DubinsRouter.hpp"
#include "mqt-scpd/routing/Heading.hpp"
#include "mqt-scpd/routing/MeanderInsertion.hpp"
#include "mqt-scpd/routing/Path.hpp"
#include "mqt-scpd/routing/PathGeometry.hpp"
#include "mqt-scpd/routing/Primitives.hpp"
#include "mqt-scpd/routing/SearchScratch.hpp"
#include "mqt-scpd/routing/SelfIntersection.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <initializer_list>
#include <limits>
#include <memory>
#include <numbers>
#include <numeric>
#include <optional>
#include <ranges>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
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

/// How much of `target_resonator_length` is left to run from the centre of
/// the coupler to the qubit. Short of the design figure on purpose: the
/// meander is what makes the way exact afterwards, and it can only lengthen
/// a way, never shorten one. The tenth held back is what it has to work in.
constexpr double COUPLER_BIAS = 1.0;

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
  /// How far to either side of its own way a wire may be moved, in cells.
  std::uint32_t reach = 0;
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
  tuning.reach = std::max(1U, params.corridor_spacings * tuning.clearance);
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
  tuning.stopAfter = params.stop_after;
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
  tuning.approachAxial =
      grid::bandLength(rules.min_straight_length, router, false);
  tuning.approachDiagonal =
      grid::bandLength(rules.min_straight_length, router, true);
  tuning.approachHalfAxial =
      grid::bandHalfWidth(rules.min_wire_spacing, router, false);
  tuning.approachHalfDiagonal =
      grid::bandHalfWidth(rules.min_wire_spacing, router, true);
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
        index, routing::reverse(routing::headingOfOrientation(port.orientation)));
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
    const grid::PortBand band{
        .center = centre,
        .step = step,
        .forward = steps == 0
                       ? grid::bandLength(reach, scene.router, step.diagonal())
                       : steps - 1,
        .backward = grid::bandLength(3.0 * rules.min_straight_length,
                                     scene.router, step.diagonal()),
        .halfWidth = grid::bandHalfWidth(rules.min_wire_spacing, scene.router,
                                         step.diagonal())};
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

private:
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

// -------------------------------------------------------- The debug pictures

/// Where a search stood when its picture was taken.
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

/// The first word of a pass name, for a file name.
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

/// A value of a price field as one of eight levels, zero for none.
[[nodiscard]] int levelOf(const int value, const int most) {
  if (value <= 0) {
    return 0;
  }
  return 1 + std::min(7, (7 * value) / std::max(1, most));
}

/// The look of the debug pictures: one class per thing, so that a layer can
/// be switched off in the file. Every fill is translucent, so that what lies
/// under it stays visible.
constexpr std::string_view DEBUG_STYLE =
    ".bg{fill:#fff}"
    ".ob{fill:#37474f;fill-opacity:.85}"
    ".co{fill:#43a047;fill-opacity:.22}"
    ".s1{fill:#ffb300;fill-opacity:.12}.s2{fill:#ffb300;fill-opacity:.16}"
    ".s3{fill:#ffb300;fill-opacity:.20}.s4{fill:#ffb300;fill-opacity:.24}"
    ".s5{fill:#ffb300;fill-opacity:.28}.s6{fill:#ffb300;fill-opacity:.32}"
    ".s7{fill:#ffb300;fill-opacity:.36}.s8{fill:#ffb300;fill-opacity:.40}"
    ".p1{fill:#e53935;fill-opacity:.10}.p2{fill:#e53935;fill-opacity:.15}"
    ".p3{fill:#e53935;fill-opacity:.20}.p4{fill:#e53935;fill-opacity:.25}"
    ".p5{fill:#e53935;fill-opacity:.30}.p6{fill:#e53935;fill-opacity:.35}"
    ".p7{fill:#e53935;fill-opacity:.40}.p8{fill:#e53935;fill-opacity:.45}"
    ".fz{fill:none;stroke:#fb8c00;stroke-opacity:.30;stroke-linecap:round;"
    "stroke-linejoin:round}"
    ".fw{fill:none;stroke:#e65100;stroke-width:1.5}"
    ".fwg{fill:none;stroke:#e65100;stroke-width:1;stroke-opacity:.35;"
    "stroke-dasharray:4 4}"
    ".nb{fill:none;stroke:#1e88e5;stroke-width:1.2}"
    ".rp{fill:none;stroke:#8e24aa;stroke-width:1.5;stroke-dasharray:6 4}"
    ".ow{fill:none;stroke:#616161;stroke-width:1;stroke-dasharray:3 3}"
    ".fd{fill:none;stroke:#2e7d32;stroke-width:2.5}"
    ".sd{fill:none;stroke:#757575;stroke-width:.8;stroke-opacity:.8}"
    ".src{fill:#1565c0;stroke:#fff;stroke-width:.4}"
    ".tgt{fill:#c62828;stroke:#fff;stroke-width:.4}"
    ".prt{fill:none;stroke:#00897b;stroke-width:1.2}"
    ".prx{fill:#00897b;stroke:#fff;stroke-width:.3}"
    ".dst{fill:none;stroke:#00897b;stroke-width:.8;stroke-dasharray:2 2}"
    ".cbx{fill:none;stroke:#6a1b9a;stroke-width:1.4;stroke-dasharray:6 4}"
    ".opt{fill:none;stroke:#8e24aa;stroke-width:.7;stroke-opacity:.45}"
    ".opl{fill:none;stroke:#8e24aa;stroke-width:.7;stroke-opacity:.35}"
    ".ops{fill:#8e24aa;fill-opacity:.5;stroke:none}"
    ".ocp{fill:none;stroke:#d81b60;stroke-width:2}"
    ".ocl{fill:none;stroke:#d81b60;stroke-width:2}"
    ".ocs{fill:#d81b60;stroke:#fff;stroke-width:.4}"
    ".pra{fill:none;stroke:#00897b;stroke-width:1}"
    ".prl{font-family:monospace;fill:#00897b}"
    ".ar{stroke:#000;stroke-width:1;fill:none}"
    ".lane{fill:none;stroke:#e53935;stroke-width:1;stroke-dasharray:8 4;"
    "stroke-opacity:.9}"
    ".lb{font-family:monospace;fill:#111}"
    ".lg{fill:#fff;fill-opacity:.88;stroke:#9e9e9e;stroke-width:.5}";

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

/// The cells of a way, for the painter.
[[nodiscard]] std::vector<debug::Cell> cellsOf(const Path& way) {
  std::vector<debug::Cell> cells;
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
  Driver(const Scene& scene, const Tuning& tuning, Progress progress = {},
         Debug debug = {}, const std::uint32_t verbosity = 0)
      : scene_(scene), tuning_(tuning), progress_(std::move(progress)),
        debug_(std::move(debug)), verbosity_(verbosity),
        primitives_(std::make_shared<const MovePrimitives>(BEND_RADIUS)),
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
    // The price of running beside an obstacle. It forbids nothing — the
    // keepout in the mask already does that — and keeps copper off the artwork
    // wherever there is room for it.
    router_.computeStaticProximity(tuning_.obstacleReach,
                                   tuning_.staticProximityPenalty);
    router_.attachWireProximity(&proximity_);
    router_.setHeuristic(routing::Heuristic::DistanceField);
  }

  [[nodiscard]] const Field& field() const { return field_; }
  [[nodiscard]] const Tuning& tuning() const { return tuning_; }

  /// Say one line, when anybody is listening. The seconds since the driver
  /// was built are on every line, so a slow round is visible as one.
  void say(const std::string& line) const {
    if (!progress_) {
      return;
    }
    const auto seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - began_)
            .count();
    progress_(std::format("[final] {:8.2f}s  {}", seconds, line));
  }

  /// Say one line of detail: what one wire's search came to. Only at the
  /// second level of verbosity, where a run says one line per search.
  void tell(const std::string& line) const {
    if (verbosity_ >= 1) {
      say("  " + line);
    }
  }

  /// Say one line of the coupler insertion's own reasoning: every option a
  /// coupler had, whether it survived being built and what the greedy
  /// priced it at. Only at the third level of verbosity — it is 64 lines
  /// per coupler, which is one chip under a lens and not a run's log.
  void explain(const std::string& line) const {
    if (verbosity_ >= 2) {
      say("    " + line);
    }
  }
  [[nodiscard]] bool explaining() const { return verbosity_ >= 2; }

  /// The picture of the last search, for the line about it, when there is one.
  [[nodiscard]] std::string picture() const {
    return lastPicture_.empty() ? std::string{} : " · " + lastPicture_;
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
  [[nodiscard]] bool needsLength(const Wire& wire) const {
    return wire.resonator && (exact_ || tuning_.meanderLength > 0.0);
  }

  /// How long a resonator's way has to be, in cells: `meander_length` in the
  /// outer routing and the target length once the coupler is in, each less
  /// the run from the cell the way ends on to the port itself, which the
  /// wire covers without cells of its own.
  [[nodiscard]] double requiredLength(const Wire& wire) const {
    const auto figure = exact_ ? tuning_.targetLength : tuning_.meanderLength;
    return std::max(0.0, figure - wire.anchorGap);
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
  /// insertion, the first placement that fits in phase 1 and the cheapest by
  /// the search's own price field in the relaxation and the refinement. The
  /// meander may enter what the search could enter — the band less the
  /// fence — and nothing else, so it cannot cross a neighbour the way itself
  /// could not.
  [[nodiscard]] Lengthened lengthen(const Wire& wire, Path& way,
                                    const bool priced) {
    const auto required = requiredLength(wire);
    routing::MeanderOptions options{.width = scene_.router.width,
                                    .height = scene_.router.height,
                                    .box = {.minX = 0,
                                            .maxX = scene_.router.width - 1,
                                            .minY = 0,
                                            .maxY = scene_.router.height - 1}};
    if (exact_) {
      // The prototype's strict insertion: the length exactly, within the
      // rule's tolerance, in the widest free strip along the pair, and
      // never in the run the resonator couples along.
      options.exact = true;
      options.tolerance = tuning_.lengthTolerance;
      options.startMargin = tuning_.couplerLength + tuning_.straightStart + 4;
      options.boxFor = [this](const PathPoint& a, const PathPoint& b) {
        return router_.freeStripAlong(a, b);
      };
    }
    const auto enterable = [this](const std::uint32_t x,
                                  const std::uint32_t y) {
      return x < scene_.router.width && y < scene_.router.height &&
             !corridor_.testCell(x, y);
    };
    routing::CellPrice price;
    if (priced) {
      price = [this](const std::uint32_t x, const std::uint32_t y) {
        const auto cell =
            (static_cast<std::size_t>(y) * scene_.router.width) + x;
        return static_cast<std::uint32_t>(router_.staticPenalty(cell)) +
               proximity_[cell];
      };
    }
    const auto made = routing::insertMeander(*primitives_, way, required,
                                             enterable, options, price);
    Lengthened result{.reached = made.reached, .tooLong = made.tooLong};
    if (made.tooLong) {
      result.note =
          std::format("too long for its target: {:.0f} of {:.0f} cells",
                      made.lengthBefore, required);
    } else if (!made.reached) {
      result.note = std::format(
          "no room for a meander: {:.0f} of {:.0f} cells, {} placements tried",
          made.lengthBefore, required, made.candidates);
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
    auto fewest = std::numeric_limits<std::uint32_t>::max();
    std::uint32_t stale = 0;
    say(std::format("{}: {} wires, up to {} rounds, {} relaxations each way",
                    pass.name, total, pass.rounds, pass.maxRelaxation));
    for (std::uint32_t round = 0; round < pass.rounds; ++round) {
      const bool forward = (round % 2) == 0;
      frame_.pass = passTag(pass.name);
      frame_.round = round;
      frame_.forward = forward;
      rounds_.push_back({.forward = forward});
      std::uint32_t tried = 0;
      std::uint32_t won = 0;
      for (std::uint32_t at = 0; at < total; ++at) {
        const auto slot = forward ? at : (total - 1 - at);
        auto& wire = wires[members[slot]];
        if (!wire.feasible || (wire.routed && pass.keepDrawn)) {
          continue;
        }
        ++tried;
        won += attempt(wires, members, slot, forward, pass) ? 1 : 0;
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
      say(std::format("{} round {} {}: tried {}, routed {}, unrouted {}, "
                      "open {}, short {} | Fails: {}",
                      pass.name, round, forward ? "forward " : "backward",
                      tried, won, unrouted, open, tooShort, fails));
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
    // The second level of verbosity ends the pass on what every round came
    // to, wire by wire: how many found a way in phase 1, how many only after
    // relaxation, and which found none.
    if (verbosity_ >= 1) {
      for (std::size_t round = 0; round < rounds_.size(); ++round) {
        const auto& record = rounds_[round];
        std::string failed;
        for (const auto& id : record.failed) {
          failed += (failed.empty() ? " " : ", ") + id;
        }
        say(std::format(
            "{} round {} {}: {} found a way in phase 1, {} after "
            "relaxation, {} failed{}{}",
            pass.name, round, record.forward ? "forward " : "backward",
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
    /// Resonators whose way is shorter than `meander_length` asks, or than
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
    /// Which ones, for the second level of verbosity.
    std::vector<std::string> unroutedIds;
    std::vector<std::string> openIds;
    std::vector<std::string> shortIds;
    std::vector<std::string> longIds;
    std::vector<std::string> crossingIds;
    std::vector<std::string> loopIds;
    [[nodiscard]] std::uint32_t total() const { return failing; }
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
    for (const auto member : members) {
      auto& wire = wires[member];
      if (!wire.drawn) {
        ++fails.unrouted;
        ++fails.failing;
        fails.feedlinesUnrouted += wire.feedline ? 1 : 0;
        fails.unroutedIds.push_back(wireId(wire));
        continue;
      }
      ++fails.drawn;
      lift(wire);
      const bool open = conflictsOf(wire, wires) != 0;
      place(wire);
      const auto length = needsLength(wire) ? lengthOf(wire.way) : 0.0;
      const auto required = requiredLength(wire);
      const auto tolerance = exact_ ? tuning_.lengthTolerance : 0.0;
      wire.tooShort = needsLength(wire) && length < required - tolerance;
      wire.tooLong =
          needsLength(wire) && exact_ && length > required + tolerance;
      const bool crossing = feedlinePass_ && crossesAFeedline(wire, wires);
      // A way that meets itself is rule 3 of the check; a search never
      // returns one, but a resonator that keeps the way its coupler cut
      // may hold one, and the count says so.
      const bool loop = routing::pathSelfIntersects(
          wire.way, scene_.router.width, scene_.router.height, loopScratch_);
      if (open) {
        ++fails.open;
        fails.openIds.push_back(wireId(wire));
      }
      if (wire.tooShort) {
        ++fails.tooShort;
        fails.shortIds.push_back(wireId(wire));
      }
      if (wire.tooLong) {
        ++fails.tooLong;
        fails.longIds.push_back(wireId(wire));
      }
      if (crossing) {
        ++fails.crossing;
        fails.crossingIds.push_back(wireId(wire));
      }
      if (loop) {
        ++fails.loops;
        fails.loopIds.push_back(wireId(wire));
      }
      if (open || wire.tooShort || wire.tooLong || crossing || loop) {
        ++fails.failing;
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
    if (wire.feedline || !wire.drawn) {
      return false;
    }
    const auto& constraints = router_.crossingConstraints();
    if (constraints.empty()) {
      return false;
    }
    const auto own =
        wire.couplerAtSource != NO_OWNER
            ? std::optional<PathPoint>(couplers_[wire.couplerAtSource].anchor)
            : std::nullopt;
    for (const auto& point : wire.way) {
      if (own.has_value() &&
          std::hypot(static_cast<double>(point.x) - own->x,
                     static_cast<double>(point.y) - own->y) <= couplerReach()) {
        continue;
      }
      if (!constraints.allowed(point.x, point.y, point.heading)) {
        return true;
      }
    }
    (void)wires;
    return false;
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
      if (!wire.resonator || !wire.feasible) {
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

  /// The summary line of a pass, or of the stage.
  void sayFails(const std::string& name, const Fails& fails,
                const std::uint32_t total) const {
    say(std::format(
        "{}: {} of {} drawn, {} unrouted{}, {} open, {} short{}{}{} "
        "| Fails: {}",
        name, fails.drawn, total, fails.unrouted,
        fails.feedlinesUnrouted == 0
            ? std::string{}
            : std::format(" ({} feedline edges)", fails.feedlinesUnrouted),
        fails.open, fails.tooShort,
        exact_ ? std::format(", {} long", fails.tooLong) : std::string{},
        feedlinePass_ ? std::format(", {} crossing a feedline", fails.crossing)
                      : std::string{},
        fails.loops == 0 ? std::string{}
                         : std::format(", {} meeting itself", fails.loops),
        fails.total()));
    if (verbosity_ >= 1 && fails.total() != 0) {
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
    }
    for (std::uint32_t round = 0; round < pass.rounds; ++round) {
      const bool forward = (round % 2) == 0;
      frame_.pass = passTag(pass.name);
      frame_.round = round;
      frame_.forward = forward;
      std::uint32_t moved = 0;
      for (std::uint32_t at = 0; at < total; ++at) {
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
        fence(wire, {&before, &after});
        priceRoom();
        constrainByFeedlines(wire, wires, pass);
        frame_.wires = &wires;
        frame_.before = before.key;
        frame_.after = after.key;
        frame_.kind = "refine";
        frame_.fence = {before.key, after.key};
        frame_.ripped.clear();
        auto found = search(wire, pass.straightStart, true, !needsLength(wire));
        std::string note;
        if (needsLength(wire)) {
          // A resonator is lengthened here as it is in the relaxation, and
          // keeps the way it had when the wider way has no room for it.
          std::string said;
          bool reached = true;
          if (!found.empty()) {
            const auto made = lengthen(wire, found, true);
            said = made.note;
            note = " · " + said;
            reached = made.reached;
          }
          drawSearch(wire, found, said);
          if (!reached) {
            found.clear();
          }
        }
        tell(std::format("wire {} · round {} {} · refine: {}{}{}", wireId(wire),
                         round, forward ? "forward" : "backward",
                         found.empty()
                             ? std::string("kept its way")
                             : std::format("found {} cells", found.size()),
                         note, picture()));
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
            tell(std::format("wire {} · refine: the way found is worse, {} against {} "
                             "conflicting cells{}; kept its way",
                             wireId(wire), after, before,
                             crossing ? ", or crosses a feedline" : ""));
            found.clear();
          }
        }
        wire.way = found.empty() ? had : found;
        moved += found.empty() ? 0 : 1;
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
      say(std::format("{} round {} {}: moved {} of {}, {} too close to another",
                      pass.name, round, forward ? "forward " : "backward",
                      moved, total, crowded));
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
  };

  [[nodiscard]] const std::vector<Coupler>& couplers() const {
    return couplers_;
  }
  [[nodiscard]] const std::vector<Edge>& edges() const { return edges_; }
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
    looseEdges_ = 0;
    bodies_ = grid::BitGrid(scene_.router.width, scene_.router.height);
    aimAtTarget(true);
    couplerBox_ = couplerBoxOf();
    say(std::format("couplers sit inside x {}..{} y {}..{}, which is what the "
                    "launcher stubs leave open; a place is taken where one of "
                    "the eight orientations puts both feedline ports and "
                    "their runs inside it",
                    couplerBox_.minX, couplerBox_.maxX, couplerBox_.minY,
                    couplerBox_.maxY));

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
          const auto x = static_cast<std::int64_t>(wire.objective.target.x) - (k * v.dx);
          const auto y = static_cast<std::int64_t>(wire.objective.target.y) - (k * v.dy);
          if (x < 0 || y < 0 || x >= width || y >= scene_.router.height) {
            break;
          }
          places.push_back({.x = static_cast<std::uint32_t>(x),
                            .y = static_cast<std::uint32_t>(y),
                            .heading = wire.objective.target.heading,
                            .primitive = 0});
        }
        const auto in = router_.straightStub(wire.objective.target, true, tuning_.straightStart);
        places.insert(places.end(), in.begin(), in.end());
        alongDisc(places, stencil, [&](const std::int64_t x, const std::int64_t y) {
          approaches_.set(static_cast<std::size_t>((y * width) + x), true);
        });
      }
    }

    // The resonators as they will stand: a coupler cuts every resonator back
    // to the target length, so what survives of another resonator's way is
    // its tail — the run from the qubit back along the way until the target
    // length and the tolerance are spent. No coupler body and no coupler
    // head may cut through that, whoever it belongs to. The head is priced
    // by what lies under it and the body was too; both are refused here,
    // because neither ever moves again once the option is taken.
    tailOwner_.assign(scene_.router.cells(), NO_OWNER);
    for (const auto& wire : wires) {
      if (!wire.resonator || !wire.feasible || !wire.drawn || wire.way.empty()) {
        continue;
      }
      const auto keep = std::max(0.0, tuning_.targetLength - wire.anchorGap) +
                        tuning_.lengthTolerance;
      double run = 0.0;
      for (auto at = wire.way.size(); at-- > 0;) {
        const auto& point = wire.way[at];
        if (point.x < scene_.router.width && point.y < scene_.router.height) {
          tailOwner_[scene_.router.index(point.x, point.y)] = wire.key;
        }
        if (at == 0) {
          break;
        }
        const auto& step = wire.way[at - 1];
        run += (step.x != point.x && step.y != point.y) ? std::numbers::sqrt2
                                                        : 1.0;
        if (run > keep) {
          break;
        }
      }
    }

    // Which wires have little room to give way: everything of the ring that
    // is not a resonator and not a feedline. Read by the option price.
    conventional_.assign(wires.size(), 0);
    for (const auto& wire : wires) {
      if (wire.key < conventional_.size()) {
        conventional_[wire.key] =
            static_cast<std::uint8_t>(!wire.resonator && !wire.feedline ? 1 : 0);
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
      coupler.options = optionsOf(wire);
      if (coupler.options.empty()) {
        ++withoutOptions;
        tell(std::format("coupler for resonator {}: no option fits",
                         wireId(wire)));
        continue;
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

    drawCouplerOptions(wires);

    // The greedy local search over every chain.
    std::uint32_t passes = 0;
    for (std::uint32_t chain = 0; chain < chains_.size(); ++chain) {
      passes = std::max(passes, optimizeChain(wires, chain));
    }

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
        wire.startStub = tuning_.straightStart;
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
      lastPicture_.clear();
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
      const bool fits =
          !chosen.empty() &&
          chosen.front().samePlace(wire.objective.source) &&
          chosen.back().samePlace(wire.objective.target) &&
          edgeWayStillOpen(wires, wire.objective, edge, chosen);
      wire.way = fits ? chosen : routeEdge(wires, wire.objective, edge, {}, false);
      if (wire.way.empty()) {
        // The edges beside it may hold what the greedy chose for another
        // option of the coupler between them; once more without them.
        wire.way = routeEdge(wires, wire.objective, edge, {}, true);
      }
      wire.drawn = !wire.way.empty();
      // A way the greedy chose and the commit kept was drawn by no search of
      // this loop, so it has no picture of its own. Draw it against the
      // ground it stands on, or every line of the log would link the last
      // search the greedy happened to make.
      if (debug_ && fits) {
        corridorOfEdge(wires, wire.objective, edge, {}, false);
        Wire pretend;
        pretend.objective = wire.objective;
        pretend.feedline = true;
        pretend.slot = wire.slot;
        frame_.wires = &wires;
        frame_.pass = "edge";
        frame_.round = edge.chain;
        frame_.forward = true;
        frame_.kind = std::format("c{}-{}to{}-kept", edge.chain, edge.from,
                                  edge.to);
        frame_.fence.clear();
        frame_.ripped.clear();
        frame_.before = NO_OWNER;
        frame_.after = NO_OWNER;
        drawSearch(pretend, wire.way,
                   std::format("chain {} {}->{}, as the greedy chose it",
                               edge.chain, edge.from, edge.to));
      }
      if (wire.drawn) {
        ++drawn;
        angle += angleCostOf(wire.way, wire.objective.target.heading);
        place(wire);
      }
      tell(std::format(
          "[Coupler Insertion] edge f{} chain {} ({}->{}): {}{}", wire.slot,
          edge.chain, edge.from, edge.to,
          wire.drawn
              ? std::format("{} cells, angle {}{}", wire.way.size(),
                            angleCostOf(wire.way, wire.objective.target.heading),
                            fits ? ", as the greedy chose it" : ", routed again")
              : std::string("NO WAY"),
          picture()));
    }
    for (std::uint32_t index = 0; index < couplers_.size(); ++index) {
      sayCoupler(wires, index);
    }
    const auto seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - began)
            .count();
    const auto diagonal = std::ranges::count_if(
        couplers_, [](const Coupler& coupler) {
          return routing::isDiagonal(
              coupler.options[coupler.chosen].way.front().heading);
        });
    say(std::format(
        "coupler insertion: {} couplers on {} resonators ({} on a diagonal), "
        "{} chains, {} of {} edges drawn, feedline angle cost {}, {} greedy "
        "passes, {} weighed without the clearance to their neighbour, "
        "{} of {} option costs from the memo, {:.1f}s",
        couplers_.size(), couplers_.size() + withoutOptions, diagonal,
        chains_.size(), drawn, edges_.size(), angle, passes, looseEdges_,
        memoHits_, memoHits_ + memoMisses_, seconds));
    // The two figures the insertion is judged by, on a line of their own so
    // that a sweep over settings can be read off the log without counting.
    const auto missing = static_cast<std::uint32_t>(edges_.size()) - drawn;
    say(std::format("==> coupler insertion: {} feedline edge{} NOT drawn, "
                    "feedline angle cost {}, {:.1f}s",
                    missing, missing == 1 ? "" : "s", angle, seconds));
  }

  /// Every option of a coupler on a resonator's way: the prototype's sweep
  /// over the orientation offsets and the mirror, with the direction of the
  /// feedline and one second dogleg on top.
  [[nodiscard]] std::vector<CouplerOption> optionsOf(Wire& resonator) {
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
      for (const bool secondPort : {false, true}) {
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
          auto option = makeOption(resonator, spot, secondPort, offset,
                                   explaining() ? &why : nullptr);
          if (!option.has_value()) {
            continue;
          }
          explain(std::format(
              "resonator {} · offset {} · resonator port {}: option {} at "
              "place {} of {} ({:.0f} of {:.0f} cells to the port), "
              "orientation {}, centre ({},{}), feedline at ({},{}) on "
              "heading {}, {} priced cells",
              wireId(resonator), offset, secondPort ? 2 : 1, options.size(),
              tried - 1, places.size(), spot.centreLength, spot.wanted,
              option->orientation, option->centre.x, option->centre.y,
              option->out.x, option->out.y, option->out.heading,
              option->guarded));
          options.push_back(std::move(*option));
          got = true;
          break;
        }
        if (!got && explaining()) {
          explain(std::format(
              "resonator {} · offset {} · resonator port {}: no place of {} "
              "tried fits — last refusal: {}",
              wireId(resonator), offset, secondPort ? 2 : 1, tried,
              why.empty() ? std::string("none offered") : why));
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
      return x >= minX + margin && x <= maxX - margin &&
             y >= minY + margin && y <= maxY - margin;
    }
    [[nodiscard]] bool empty() const { return maxX < minX || maxY < minY; }
  };

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
    const std::int64_t reach =
        halfRun + static_cast<std::int64_t>(tuning_.straightStart);
    for (const std::int64_t end : {-reach, reach}) {
      if (!couplerBox_.holds(edgeX + (end * along.dx),
                             edgeY + (end * along.dy))) {
        return false;
      }
    }
    return true;
  }

  /// Whether any of the eight orientations fits at a cell. A place needs one
  /// — the greedy is free to take another and reach past the box, as it was
  /// before the box existed.
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
    const auto reach = static_cast<std::int64_t>(tuning_.straightStart +
                                                 tuning_.clearance) +
                       1;
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
    const double target =
        std::max(0.0, tuning_.targetLength - resonator.anchorGap);
    // The component's own lead runs in front of what is left, so the place
    // is the point where the lead **and** the rest together come to the
    // figure. Without this the way left is the target and the lead — a
    // quarter turn and fourteen cells, about 290 layout units — sits on top
    // of it: on 21q every resonator came out at 2746 to 2805 against a
    // target of 2500, over by exactly the lead, and nothing downstream
    // shortens a way.
    const double wanted =
        std::max(0.0, (target * COUPLER_BIAS) - leadLength());
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
        if (found == indexOf.end() || found->second + 1 >= resonator.way.size()) {
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
      return std::abs(a.centreLength - a.wanted) <
             std::abs(b.centreLength - b.wanted);
    });
    return places;
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
    return std::max(COUPLER_LEAD_STRAIGHT, tuning_.straightStart);
  }
  static constexpr std::uint32_t COUPLER_LEAD_STRAIGHT = 14;

  /// One option at the place the coupler takes: the body as a pad centred on
  /// that cell, turned by one of the eight offsets from the heading the way
  /// holds there. Nothing, where it does not fit.
  [[nodiscard]] std::optional<CouplerOption>
  makeOption(const Wire& resonator, const CouplerPlace& place,
             const bool secondPort,
             const Heading offset, std::string* why = nullptr) const {
    const auto refuse = [&why](std::string reason)
        -> std::optional<CouplerOption> {
      if (why != nullptr) {
        *why = std::move(reason);
      }
      return std::nullopt;
    };
    CouplerOption option;
    option.offset = offset;
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
    const auto facing = routing::turned(
        nearestLauncherHeading(place.x, place.y), -2);
    option.orientation =
        static_cast<Heading>((facing + offset) % routing::NUM_HEADINGS);
    option.run = cellsOn(tuning_.couplerLength, option.orientation);
    option.depth = cellsOn(tuning_.couplerHeight, option.orientation);
    // What is left runs to the qubit. Only an overshoot is refused: nothing
    // spliced in later makes a way shorter, while a way short of the figure
    // is what the meander is for, and is priced by how much it has to add.
    const double target =
        std::max(0.0, tuning_.targetLength - resonator.anchorGap);
    const auto left = lengthOf(option.way);
    if (left > target + tuning_.lengthTolerance) {
      return refuse("what is left of the way is longer than the target");
    }
    if (left < target - tuning_.lengthTolerance) {
      option.guarded +=
          static_cast<std::uint32_t>(target - tuning_.lengthTolerance - left);
    }

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
    const auto portHeading = secondPort
                                 ? routing::reverse(option.orientation)
                                 : option.orientation;
    routing::DoglegGeometry lead;
    try {
      const auto turnSign =
          routing::turned(portHeading, 2) == option.couplerOrientation ? 1 : -1;
      lead = routing::buildDogleg(*primitives_, portHeading, turnSign,
                                  leadStraight());
    } catch (const std::logic_error&) {
      return refuse("the primitives hold no quarter turn for the lead");
    }
    const auto portX =
        place.x -
        static_cast<std::int64_t>(static_cast<std::int32_t>(lead.tip.x));
    const auto portY =
        place.y -
        static_cast<std::int64_t>(static_cast<std::int32_t>(lead.tip.y));
    // The port is one end of the near edge; the centre lies half a depth
    // across from it and half a run back along the pad, on the side the
    // other port is.
    const std::int64_t towards = secondPort ? 1 : -1;
    const auto originX = portX + ((halfDepth + 1) * across.dx) +
                         (towards * halfRun * along.dx);
    const auto originY = portY + ((halfDepth + 1) * across.dy) +
                         (towards * halfRun * along.dy);
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
    std::unordered_set<std::size_t> stubs;
    for (std::int64_t k = -halfRun - tuning_.straightStart;
         k <= halfRun + tuning_.straightStart; ++k) {
      const auto x = edgeX + (k * along.dx);
      const auto y = edgeY + (k * along.dy);
      if (x < 0 || y < 0 || x >= width || y >= height) {
        return refuse("the feedline's run along the pad leaves the grid");
      }
      // Both ports and the run the chain is forced to make off each of them
      // have to be in the box. The place was chosen so that at least one
      // orientation manages it; the ones that do not are refused here, or
      // the greedy would take a port past a launcher's stub and the edge
      // into it would have to come back around — the detour the box exists
      // to stop.
      if (!couplerBox_.holds(x, y)) {
        return refuse(std::format(
            "the feedline's run along the pad leaves the box at ({},{})", x,
            y));
      }
      const auto cell = static_cast<std::size_t>((y * width) + x);
      if (!free(cell)) {
        return refuse(
            std::format("the feedline's run along the pad meets {}", blocker));
      }
      stubs.insert(cell);
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
    const auto& leaveFrom = secondPort ? option.resonatorOut
                                       : option.resonatorIn;

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
      return refuse("the arc does not reach the coupler's orientation");
    }
    // The tip is not part of the arc: the search begins on it and puts the
    // arc in front of what it finds, so a tip in both would put one cell in
    // the way twice and the self-intersection test would throw it away.
    option.arc.clear();
    option.arc.reserve(lead.path.size());
    PathPoint tip{};
    for (std::size_t step = 0; step < lead.path.size(); ++step) {
      const auto& move = lead.path[step];
      const auto x =
          portX + static_cast<std::int64_t>(static_cast<std::int32_t>(move.x));
      const auto y =
          portY + static_cast<std::int64_t>(static_cast<std::int32_t>(move.y));
      if (x < 0 || y < 0 || x >= width || y >= height) {
        return refuse("the resonator's lead leaves the grid");
      }
      if (!couplerBox_.holds(x, y)) {
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
      if (step + 1 < lead.path.size()) {
        option.arc.push_back(here);
      } else {
        tip = here;
      }
    }
    if (option.arc.empty()) {
      return refuse("the lead has no cells");
    }
    option.anchor = leaveFrom;
    option.arcEnd = tip;
    // The way the option carries is the lead **and what is left to the
    // qubit**: the arc, the straight after it, and the old route onward
    // from the insertion point.
    //
    // The two join without a jump, which they did not always. While the
    // pad sat on the path and the lead hung off it, the lead ended nowhere
    // near the route and stitching them made a way that read as drawn and
    // measured as nonsense; the way was the lead alone for that reason.
    // Now the lead's tip *is* the insertion point, so the join is exact —
    // and a resonator that finds nothing under the feedline constraints
    // keeps a way that is a whole resonator rather than a stub.
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

  /// Every edge already committed stands in the way of the edge about to be
  /// routed, inflated by the clearance; the edges that share a coupler with
  /// it meet it at the coupler's port and keep no clearance from it, but
  /// their copper, two cells wide, is closed to it all the same, so that
  /// the edge into a coupler and the edge out of it never cross.
  ///
  /// @param ignoreAdjacent Leave out the edges that share a coupler with
  /// this one: while the options are weighed, what they hold is stale and
  /// the caller passes the fresh one.
  /// How far past the two ends of an edge its search may roam, in cells.
  static constexpr std::uint32_t EDGE_BOX_MARGIN = 250;

  static constexpr std::size_t NO_PIVOT =
      std::numeric_limits<std::size_t>::max();

  void fenceCommittedEdges(const std::vector<Wire>& wires, const Edge& edge,
                           const bool ignoreAdjacent,
                           const std::size_t pivot = NO_PIVOT) {
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto& stencil = stencilFor(tuning_.clearance);
    std::size_t fenced = 0;
    std::size_t ways = 0;
    std::size_t neighbours = 0;
    for (std::uint32_t chain = 0; chain < chains_.size(); ++chain) {
      for (std::size_t at = 0; at < chainEdgePaths_[chain].size(); ++at) {
        if (chain == edge.chain && at == edge.from) {
          continue;
        }
        // No exemption of any kind. An edge that meets this one at a
        // coupler is fenced by the full clearance like every other feedline,
        // right up to the port they share. The disc that used to be left
        // open there was the one place two feedlines could come closer than
        // the rule allows.
        const bool adjacent = chain == edge.chain &&
                              (at + 1 == edge.from || at == edge.from + 1);
        neighbours += adjacent ? 1 : 0;
        // Nothing is left out any more. An edge that meets this one at a
        // coupler is still an active feedline, and it is fenced like every
        // other — at the clearance away from the coupler, open only within
        // the reach where the two pin on the same cell on purpose. The
        // caller that replaces one of them hands the fresh path in as
        // `more`, which is fenced the same way, so the stale copy costs the
        // search only the ground it actually holds.
        //
        // What used to happen instead: the direct neighbours of a path were
        // the one pair of feedlines nothing closed, and an edge weighed
        // against them ran through them.
        (void)ignoreAdjacent;
        (void)pivot;
        const auto& way = edgeWayOf(wires, chain, at);
        if (way.empty()) {
          continue;
        }
        ++ways;
        alongDisc(way, stencil, [&](const std::int64_t x, const std::int64_t y) {
          corridor_.set(static_cast<std::size_t>((y * width) + x), true);
          ++fenced;
        });
      }
    }
    tell(std::format("fence for edge c{}-{}to{}: {} ways ({} neighbours), "
                     "{} cells closed",
                     edge.chain, edge.from, edge.to, ways, neighbours,
                     fenced));
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
        exempt.emplace_back(coupler.wire, coupler.options[coupler.chosen].anchor);
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
                      const RoutingObjective& objective, const Edge& edge,
                      const std::vector<std::size_t>& more,
                      const bool ignoreAdjacent,
                      const std::size_t pivot = NO_PIVOT) {
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
    const auto lowX = std::min<std::int64_t>(objective.source.x, objective.target.x);
    const auto highX = std::max<std::int64_t>(objective.source.x, objective.target.x);
    const auto lowY = std::min<std::int64_t>(objective.source.y, objective.target.y);
    const auto highY = std::max<std::int64_t>(objective.source.y, objective.target.y);
    box_ = {.minX = std::max<std::int64_t>(0, lowX - margin),
            .minY = std::max<std::int64_t>(0, lowY - margin),
            .maxX = std::min<std::int64_t>(
                static_cast<std::int64_t>(scene_.router.width) - 1, highX + margin),
            .maxY = std::min<std::int64_t>(
                static_cast<std::int64_t>(scene_.router.height) - 1, highY + margin)};
    {
      const auto w = static_cast<std::int64_t>(scene_.router.width);
      const auto h = static_cast<std::int64_t>(scene_.router.height);
      for (std::int64_t y = 0; y < h; ++y) {
        const bool outsideRow = y < box_.minY || y > box_.maxY;
        for (std::int64_t x = 0; x < w; ++x) {
          if (outsideRow || x < box_.minX || x > box_.maxX) {
            corridor_.set(static_cast<std::size_t>((y * w) + x), true);
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
    fenceCommittedEdges(wires, edge, ignoreAdjacent, pivot);

    // The box is **off**. It was a hard bound on where an edge may run — no
    // cell outside the rectangle the launcher stubs leave open — with the
    // starting and ending edges of a chain exempt. `closeOutsideBox` is
    // still here and has no caller; put this back to switch it on again.

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
    const auto close = [&](const PathPoint& at) {
      // The straight run off the terminal, on the heading the wire leaves it
      // on, and the clearance around it.
      const auto v = routing::headingVector(at.heading);
      Path places;
      for (std::int64_t k = 0; k <= tuning_.straightStart*2.0; ++k) {
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
      alongDisc(places, stencil,
                [&](const std::int64_t x, const std::int64_t y) {
                  corridor_.set(static_cast<std::size_t>((y * width) + x),
                                true);
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

    (void)more;
    (void)objective;
  }

  /// What a bend costs an edge of a chain. A feedline is judged by how much
  /// it turns — that is the whole objective the greedy optimises — so an edge
  /// may pay more for a bend than a wire that is only trying to get there.
  /// Read from the environment so a sweep over it costs a rebuild of nothing.
  [[nodiscard]] std::uint16_t edgeBendPenalty() const {
    static const double factor = [] {
      const char* set = std::getenv("SCPD_EDGE_BEND_FACTOR");
      return set != nullptr ? std::max(1.0, std::atof(set)) : 1.0;
    }();
    return std::max(static_cast<std::uint16_t>(1),
                     static_cast<std::uint16_t>(factor * tuning_.bendPenalty));
  }

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
    corridorOfEdge(wires, objective, edge, {}, false);
    return std::ranges::none_of(way, [this](const PathPoint& point) {
      return point.x >= scene_.router.width || point.y >= scene_.router.height ||
             corridor_.test(scene_.router.index(point.x, point.y));
    });
  }

  /// Route one feedline edge, free of any crossing rule, as the prototype
  /// routes its chain edges: the room of every wire is priced, nothing but
  /// the artwork and the couplers is in the way.
  [[nodiscard]] Path routeEdge(const std::vector<Wire>& wires,
                               const RoutingObjective& objective,
                               const Edge& edge,
                               const std::vector<std::size_t>& more,
                               const bool ignoreAdjacent,
                               const std::size_t pivot = NO_PIVOT) {
    lastPicture_.clear();
    corridorOfEdge(wires, objective, edge, more, ignoreAdjacent, pivot);
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
    router_.setParams({.startStraightLength = tuning_.straightStart,
                       .endStraightLength =
                           chain[edge.to].fixed
                               ? tuning_.straightStart
                               : couplerRunOf(chain[edge.to].coupler),
                       .minRadius = BEND_RADIUS,
                       .bendPenalty = edgeBendPenalty()});
    router_.setSingleCrossingFeedline(nullptr, 0, 1);
    router_.attachCorridor(&corridor_);
    router_.attachWireProximity(&zeroProximity_);
    auto found = router_.route(objective, true);
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
      drawSearch(pretend, found,
                 std::format("chain {} {}->{}{}", edge.chain, edge.from,
                             edge.to, found.empty() ? ", no way" : ""));
    }
    if (found.empty() && verbosity_ >= 1) {
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
  [[nodiscard]] std::uint32_t localCost(const std::vector<Wire>& wires,
                                        const std::uint32_t chain,
                                        const std::size_t at,
                                        const std::size_t option,
                                        Path* firstOut = nullptr,
                                        Path* secondOut = nullptr,
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

    // One edge, routed against a way already found — its copper two cells
    // wide so the two never cross, and its clearance on top of that. The
    // clearance goes before the copper but never instead of an edge: a chain
    // with an edge missing is worse than two edges that run close. On 69q
    // the clearance alone left two of eighty-one edges undrawn.
    const auto routeAgainst = [&](const std::size_t from, const std::size_t to,
                                  const Path& against) {
      const auto edge = edgeOf(from, to);
      const auto objective = objectiveOf(from, to);
      if (against.empty()) {
        return routeEdge(wires, objective, edge, {}, true, at);
      }
      const auto width = static_cast<std::int64_t>(scene_.router.width);
      std::vector<std::size_t> copper;
      for (const auto& cell : against) {
        for (std::int64_t dy = -2; dy <= 2; ++dy) {
          for (std::int64_t dx = -2; dx <= 2; ++dx) {
            const auto x = static_cast<std::int64_t>(cell.x) + dx;
            const auto y = static_cast<std::int64_t>(cell.y) + dy;
            if (x >= 0 && y >= 0 && x < width && y < scene_.router.height) {
              copper.push_back(static_cast<std::size_t>((y * width) + x));
            }
          }
        }
      }
      auto taken = copper;
      const auto& stencil = stencilFor(tuning_.clearance);
      alongDisc(against, stencil,
                [&](const std::int64_t x, const std::int64_t y) {
                  taken.push_back(static_cast<std::size_t>((y * width) + x));
                });
      auto found = routeEdge(wires, objective, edge, taken, true, at);
      if (found.empty()) {
        found = routeEdge(wires, objective, edge, copper, true, at);
        looseEdges_ += found.empty() ? 0 : 1;
      }
      return found;
    };

    // Both edges, in one order. The one routed first is free of the other
    // and the second has to get past it, so which goes first decides what
    // is left for the other: a way that takes the only gap blocks its
    // neighbour, where the reverse order fits both. The second order is
    // tried only when the first loses an edge, which is what makes it
    // cheap — on a chain that routes, it never runs at all.
    struct Attempt {
      Path first;
      Path second;
      std::uint32_t lost = 0;
      std::uint32_t cost = 0;
    };
    const auto tryOrder = [&](const bool afterFirst) {
      Attempt out;
      if (afterFirst) {
        if (hasAfter) {
          out.second = routeAgainst(at, at + 1, {});
        }
        if (hasBefore) {
          out.first = routeAgainst(at - 1, at, out.second);
        }
      } else {
        if (hasBefore) {
          out.first = routeAgainst(at - 1, at, {});
        }
        if (hasAfter) {
          out.second = routeAgainst(at, at + 1, out.first);
        }
      }
      if (hasBefore) {
        out.lost += out.first.empty() ? 1 : 0;
        out.cost += 10000 * angleCostOf(out.first,
                                        objectiveOf(at - 1, at).target.heading);
      }
      if (hasAfter) {
        out.lost += out.second.empty() ? 1 : 0;
        out.cost += 10000 * angleCostOf(out.second,
                                        objectiveOf(at, at + 1).target.heading);
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
          known.cost += 10000 * angleCostOf(
              known.first, objectiveOf(at - 1, at).target.heading);
        }
        if (hasAfter) {
          known.lost += known.second.empty() ? 1 : 0;
          known.cost += 10000 * angleCostOf(
              known.second, objectiveOf(at, at + 1).target.heading);
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

    auto taken = tryOrder(false);
    if (taken.lost > 0 && hasBefore && hasAfter) {
      const auto other = tryOrder(true);
      if (other.lost < taken.lost ||
          (other.lost == taken.lost && other.cost < taken.cost)) {
        taken = other;
      }
    }
    // The better of the two orders is what the pair is remembered by.
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


  /// What a cell of the head or the body costs an option, in the units the
  /// greedy multiplies by a hundred — where one turn of a feedline edge
  /// costs ten thousand.
  ///
  /// The head and the body never move again once an option is taken: the
  /// head is the resonator's fixed places and the body is an obstacle to
  /// every search after it. So whatever stands beside them is what has to
  /// give way, and it is the wire beside them that fails when it cannot.
  /// Measured on the 17-qubit chip: every pair of wires the check found too
  /// close was a resonator and its ring neighbour, within a dozen cells of
  /// that resonator's own coupler.
  ///
  /// A conventional wire costs more than a resonator under them, because a
  /// resonator is drawn again from its coupler and may meander its way
  /// around, while a conventional wire runs from a fixed port to a fixed
  /// port and has only the band around its way.
  static constexpr std::uint32_t UNDER_RESONATOR = 20;
  static constexpr std::uint32_t UNDER_CONVENTIONAL = 60;
  static constexpr std::uint32_t IN_THE_ROOM = 1;


  /// The greedy local search over one chain: the prototype's
  /// `optimize_chain`. Up to five passes; in each, every coupler of the
  /// chain takes the cheapest of its options, its own included, with the
  /// neighbours as they stand.
  ///
  /// **Every** option is routed. The prototype pre-screens them by an
  /// estimate of what their edges will turn and routes only the cheapest
  /// two dozen; here the estimate is gone, because an estimate that ranks
  /// an option it never routes can only drop the one that would have won.
  /// It costs what it costs: a coupler weighs up to sixty-four options and
  /// each is two routed edges.
  ///
  /// @returns How many passes improved something.
  [[nodiscard]] std::uint32_t optimizeChain(const std::vector<Wire>& wires,
                                            const std::uint32_t chain) {
    constexpr std::uint32_t PASSES = 5;
    constexpr auto INFINITE = std::numeric_limits<std::uint32_t>::max();
    auto& points = chains_[chain];
    std::uint32_t improved = 0;
    for (std::uint32_t pass = 0; pass < PASSES; ++pass) {
      bool moved = false;
      for (std::size_t at = 0; at < points.size(); ++at) {
        if (points[at].fixed) {
          continue;
        }
        auto& coupler = couplers_[points[at].coupler];
        if (coupler.options.size() <= 1) {
          Path first;
          Path second;
          std::uint32_t lost = 0;
          coupler.options.front().cost =
              localCost(wires, chain, at, 0, &first, &second, &lost);
          if (at > 0) {
            chainEdgePaths_[chain][at - 1] = first;
          }
          if (at + 1 < points.size()) {
            chainEdgePaths_[chain][at] = second;
          }
          continue;
        }
        const auto current = coupler.chosen;
        auto best = current;
        Path bestFirst;
        Path bestSecond;
        // An option is eligible only when both its edges found a way.
        // One that loses an edge is discarded outright, never weighed: an
        // empty way turns nowhere, so `angleCostOf` scores it zero and it
        // would otherwise look cheaper than every option that keeps both.
        // Among the eligible ones the cheapest wins, and any eligible option
        // beats a coupler sitting on one that lost an edge.
        std::uint32_t leastLost = 0;
        auto least = localCost(wires, chain, at, current, &bestFirst,
                               &bestSecond, &leastLost);
        coupler.options[current].cost = least;
        explain(std::format(
            "[Coupler Insertion]   '{}' chain {} pass {}: holds option "
            "{}/{} -> {}",
            wireId(wires[coupler.wire]), chain, pass, current + 1,
            coupler.options.size(),
            leastLost > 0 ? std::format("{} edge(s) unroutable", leastLost)
                          : std::format("cost {}", least)));
        for (std::size_t option = 0; option < coupler.options.size();
             ++option) {
          if (option == current) {
            continue;
          }
          Path first;
          Path second;
          std::uint32_t lost = 0;
          const auto cost =
              localCost(wires, chain, at, option, &first, &second, &lost);
          coupler.options[option].cost = cost;
          if (explaining()) {
            const auto& candidate = coupler.options[option];
            explain(std::format(
                "[Coupler Insertion]   try '{}' option {}/{}: angle {}° "
                "(offset {}, resonator port {}) at ({},{}) -> {}{}",
                wireId(wires[coupler.wire]), option + 1,
                coupler.options.size(),
                degreesOfHeading(candidate.couplerOrientation),
                candidate.offset, candidate.secondPort ? 2 : 1,
                candidate.centre.x, candidate.centre.y,
                lost > 0 ? std::format("{} edge(s) unroutable", lost)
                         : std::format("cost {}", cost),
                picture()));
          }
          const bool better =
              lost == 0 &&
              (leastLost > 0 || (cost != INFINITE && cost <= least));
          if (better) {
            leastLost = lost;
            least = cost;
            best = option;
            bestFirst = std::move(first);
            bestSecond = std::move(second);
          }
        }
        // What the coupler's edges are from now on, for every edge routed
        // after them.
        if (least != INFINITE) {
          if (at > 0 && !bestFirst.empty()) {
            chainEdgePaths_[chain][at - 1] = bestFirst;
          }
          if (at + 1 < points.size() && !bestSecond.empty()) {
            chainEdgePaths_[chain][at] = bestSecond;
          }
        }
        if (best != current) {
          coupler.chosen = best;
          moved = true;
          const auto& won = coupler.options[best];
          tell(std::format(
              "[Coupler Insertion] ACCEPT '{}' chain {} pass {}: angle {}° "
              "(offset {}, resonator port {}) at ({},{}) -> cost {}",
              wireId(wires[coupler.wire]), chain, pass,
              degreesOfHeading(won.couplerOrientation), won.offset,
              won.secondPort ? 2 : 1, won.centre.x, won.centre.y, least));
        }
      }
      if (!moved) {
        break;
      }
      ++improved;
    }
    return improved;
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
    wire.arc = chosen.arc;
    wire.objective.source = chosen.arcEnd;
    wire.startStub = tuning_.straightStart;
    wire.fixed.clear();
    wire.couplerAtSource = index;
    wire.routed = false;
    wire.tooShort = false;
    wire.tooLong = false;
    coupler.anchor = chosen.anchor;
    wire.way = chosen.way;
    wire.drawn = true;
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
        "[Coupler Insertion] Coupler for '{} -> {}' inserted at ({}, {}) "
        "with angle {}°, offset {} of the nearest launcher, resonator "
        "port {} at ({},{}), "
        "guarded {}, {:.0f} units of way + "
        "{:.0f} to the port = {:.0f}, target {:.0f}",
        wireId(wire), label, option.centre.x, option.centre.y,
        degreesOfHeading(option.couplerOrientation), option.offset,
        option.secondPort ? 2 : 1, option.anchor.x, option.anchor.y,
        option.guarded, way,
        wire.anchorGapUnits, way + wire.anchorGapUnits,
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
    feedlinePass_ = pass.feedlines;
    if (!pass.feedlines) {
      router_.clearOrthogonalConstraints();
      return;
    }
    for (const auto member : members) {
      wires[member].routed = false;
    }
    rebuildCrossingRule(wires);
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
      if (wire.drawn) {
        crossable.push_back(wire.way);
      }
    }
    router_.buildOrthogonalConstraints(crossable, {}, CROSSING_REACH);
  }

  /// Fence a search by the feedlines and price the room around them, as
  /// the prototype's feedline routing does: every edge inflated by the
  /// clearance stands in the way, except the edge a conventional wire
  /// bridges, which it may cross once at a right angle, and the edges of a
  /// resonator's own coupler, which the meeting leaves open. The room
  /// around every edge costs five times the wire price.
  void constrainByFeedlines(const Wire& wire, const std::vector<Wire>& wires,
                            const Pass& pass) {
    if (!pass.feedlines) {
      router_.setSingleCrossingFeedline(nullptr, 0, 1);
      return;
    }
    for (const auto& edge : edges_) {
      const auto& other = wires[edge.wire];
      if (!other.drawn || other.key == wire.key || other.key == wire.bridged) {
        continue;
      }
      // The first and last edge of a chain run from a launcher on the border
      // to the first coupler, which is deep in the fan-in. Fenced, such an
      // edge cuts the plane in two from the border inward, and every wire on
      // the far side of it has to go around the coupler at its end. The
      // prototype leaves them out of its fence and out of its crossing rule
      // (`is_first_last_feedline`, FinalGrid.cpp:10677, :10788, :10802), and
      // so do we for the wires that may cross them. A resonator may not —
      // that is the rule — and neither may another edge.
      if (edge.terminal && !wire.resonator && !wire.feedline) {
        continue;
      }
      fence(wire, {&other});
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
    const auto strong = static_cast<std::uint8_t>(
        std::min<std::uint32_t>(127U, 5U * tuning_.wireProximityPenalty));
    for (const auto& edge : edges_) {
      const auto& other = wires[edge.wire];
      if (other.drawn && other.key != wire.key) {
        stampDisc(other.way, tuning_.clearance, strong);
      }
    }
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
      wire.way = routeEdge(wires, wire.objective, edge, {}, false);
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

  /// Phase 4's repair: while the feedlines leave fails, turn a coupler near
  /// them to another of its options and route what that unsettles again;
  /// keep the turn that leaves strictly fewer fails. The prototype's
  /// `run_final_routing_feedline_choices_parallel`, one trial at a time.
  ///
  /// @returns The fails the repair ends on.
  std::uint32_t repair(std::vector<Wire>& wires,
                       const std::vector<std::uint32_t>& members,
                       const std::vector<std::uint32_t>& every,
                       const Pass& pass) {
    auto current = failsOf(wires, every);
    std::uint32_t trials = 0;
    std::set<std::pair<std::uint32_t, std::size_t>> tried;
    Pass again = pass;
    again.rounds = 1;
    again.name = "feedline repair";
    while (current.total() > 0 && trials < tuning_.repairTrials) {
      // The couplers near the fails: of a failing resonator or edge, and
      // of the nearest resonators on either side of a failing wire, with
      // their chain neighbours.
      std::vector<std::uint32_t> candidates;
      const auto add = [&](const std::uint32_t index) {
        if (index != NO_OWNER &&
            std::ranges::find(candidates, index) == candidates.end()) {
          candidates.push_back(index);
        }
      };
      const auto near = [&](const Wire& wire) {
        if (wire.feedline) {
          add(wire.couplerAtSource);
          add(wire.couplerAtTarget);
          return;
        }
        if (wire.couplerAtSource != NO_OWNER) {
          add(wire.couplerAtSource);
          return;
        }
        const auto found = std::ranges::find(members, wire.key);
        if (found == members.end()) {
          return;
        }
        const auto at = static_cast<std::size_t>(found - members.begin());
        const auto total = members.size();
        for (const int direction : {1, -1}) {
          for (std::size_t step = 1; step < total; ++step) {
            const auto place =
                (at + total + (static_cast<std::size_t>(direction) * step)) %
                total;
            const auto& other = wires[members[place]];
            if (other.resonator) {
              add(other.couplerAtSource);
              break;
            }
          }
        }
      };
      for (const auto key : every) {
        const auto& wire = wires[key];
        if (!wire.feasible) {
          continue;
        }
        const bool failing =
            !wire.drawn || !wire.routed || wire.tooShort || wire.tooLong;
        if (failing) {
          near(wire);
        }
      }
      const auto direct = candidates;
      for (const auto index : direct) {
        for (const auto& points : chains_) {
          for (std::size_t at = 0; at < points.size(); ++at) {
            if (points[at].coupler != index) {
              continue;
            }
            if (at > 0) {
              add(points[at - 1].coupler);
            }
            if (at + 1 < points.size()) {
              add(points[at + 1].coupler);
            }
          }
        }
      }
      // A wave: up to two untried options per candidate, the cheapest first.
      struct Trial {
        std::uint32_t coupler = 0;
        std::size_t option = 0;
        std::uint32_t fails = 0;
      };
      std::vector<Trial> wave;
      for (const auto index : candidates) {
        const auto& coupler = couplers_[index];
        std::vector<std::size_t> options;
        for (std::size_t option = 0; option < coupler.options.size();
             ++option) {
          if (option != coupler.chosen && !tried.contains({index, option})) {
            options.push_back(option);
          }
        }
        std::ranges::stable_sort(options, {}, [&](const std::size_t option) {
          return coupler.options[option].cost;
        });
        for (std::size_t k = 0; k < 2 && k < options.size(); ++k) {
          wave.push_back({.coupler = index,
                          .option = options[k],
                          .fails = current.total()});
        }
      }
      if (wave.empty()) {
        say("feedline repair: nothing left to try");
        break;
      }
      const auto taken = snapshot(wires);
      for (auto& trial : wave) {
        if (trials >= tuning_.repairTrials) {
          break;
        }
        ++trials;
        tried.insert({trial.coupler, trial.option});
        if (!turnCoupler(wires, trial.coupler, trial.option)) {
          trial.fails = std::numeric_limits<std::uint32_t>::max();
          tell(std::format(
              "repair trial {}: coupler {} option {}: its edges find no way",
              trials, trial.coupler, trial.option));
          continue;
        }
        for (const auto key : unsettledBy(wires, members, trial.coupler)) {
          wires[key].routed = false;
        }
        const auto verbosity = verbosity_;
        verbosity_ = 0;
        const auto quiet = progress_;
        progress_ = {};
        sweep(wires, members, again);
        progress_ = quiet;
        verbosity_ = verbosity;
        trial.fails = failsOf(wires, every).total();
        tell(std::format(
            "repair trial {}: coupler {} option {}: {} fails against {}",
            trials, trial.coupler, trial.option, trial.fails, current.total()));
        restore(wires, taken);
      }
      const auto best = std::ranges::min_element(wave, {}, &Trial::fails);
      if (best == wave.end() || best->fails >= current.total()) {
        say(std::format(
            "feedline repair: {} trials, none with fewer than {} fails", trials,
            current.total()));
        if (trials >= tuning_.repairTrials) {
          break;
        }
        continue;
      }
      if (!turnCoupler(wires, best->coupler, best->option)) {
        continue;
      }
      for (const auto key : unsettledBy(wires, members, best->coupler)) {
        wires[key].routed = false;
      }
      sweep(wires, members, again);
      current = failsOf(wires, every);
      say(std::format(
          "feedline repair: coupler {} turned to option {} after {} trials | "
          "Fails: {}",
          best->coupler, best->option, trials, current.total()));
    }
    // Everything down again, so that the field says what the canvas holds.
    for (auto& wire : wires) {
      place(wire);
    }
    return current.total();
  }

private:
  /// How many rounds without progress end a pass.
  static constexpr std::uint32_t STALE_ROUNDS = 4;

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
  /// The meander goes in after each search, the first placement that fits in
  /// phase 1 and the cheapest in the relaxation, and a way without room for
  /// it counts as no way, so the relaxation goes on; when nothing comes of
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
    // whose way does not hold is searched for.
    const bool ripped = wire.ripped;
    wire.ripped = false;
    if (pass.feedlines && wire.drawn && !ripped && isLegal(wire, wires)) {
      wire.routed = true;
      wire.tooShort = false;
      wire.tooLong = false;
      place(wire);
      tell(std::format("wire {} · round {} {}: keeps its way, which holds",
                       wireId(wire), frame_.round,
                       forward ? "forward" : "backward"));
      return true;
    }

    // The wire is lifted off the canvas for the length of its own search, so
    // that what is put down afterwards is the one way it has.
    lift(wire);

    // Phase 1: the band around the way it has, fenced by its two neighbours,
    // and nothing priced — except, under the feedline constraints, the
    // chains: fenced, and priced around.
    buildCorridor(wire, pass.reach);
    fence(wire, {&before, &after});
    std::ranges::fill(proximity_, 0);
    constrainByFeedlines(wire, wires, pass);
    frame_.wires = &wires;
    frame_.before = before.key;
    frame_.after = after.key;
    frame_.kind = "normal";
    frame_.fence = {before.key, after.key};
    frame_.ripped.clear();
    const bool lengthened = needsLength(wire);
    auto found = search(wire, pass.straightStart, true, !lengthened);
    const auto id = wireId(wire);
    const auto where = std::format("round {} {}", frame_.round,
                                   forward ? "forward" : "backward");
    RoundRecord* record = rounds_.empty() ? nullptr : &rounds_.back();
    // The last way found that could not be made long enough, and what the
    // lengthening said. A way found is kept only when it is long enough.
    Path plain;
    bool plainTooLong = false;
    std::string note;
    const auto lengthenFound = [&](const bool priced) {
      note.clear();
      if (!lengthened) {
        return;
      }
      std::string said;
      if (!found.empty()) {
        const auto made = lengthen(wire, found, priced);
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
      drawSearch(wire, found.empty() && !said.empty() ? plain : found, said);
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
    lengthenFound(false);
    if (!found.empty()) {
      tell(std::format("wire {} · {} · normal: {}{}", id, where, outcome(),
                       picture()));
      if (record != nullptr) {
        ++record->normal;
      }
    } else {
      tell(std::format("wire {} · {} · normal: {}, relaxing{}{}", id, where,
                       outcome(), picture(),
                       verbosity_ >= 1 && pass.feedlines
                           ? whatBlocks(wire, wires, members, before, after, pass)
                           : std::string{}));
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
      fence(wire, {&ripped, forward ? &before : &after});
      priceLane(wires, members, slot, forward, level);
      constrainByFeedlines(wire, wires, pass);
      frame_.kind = std::format("relax {}", level);
      frame_.fence = {ripped.key, (forward ? before : after).key};
      frame_.ripped.push_back(ripped.key);
      found = search(wire, pass.straightStart, true, !lengthened);
      lengthenFound(true);
      tell(std::format(
          "wire {} · relax {}: let go of {}, fence {} and {} · {}{}", id, level,
          wireId(ripped), wireId(ripped), wireId(forward ? before : after),
          outcome(), picture()));
    }

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
  [[nodiscard]] std::string whatBlocks(const Wire& wire, std::vector<Wire>& wires,
                                       const std::vector<std::uint32_t>& members,
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
          if (other.drawn && other.key != wire.key && other.key != wire.bridged) {
            fence(wire, {&other});
          }
        }
      }
      if (!crossing) {
        router_.clearOrthogonalConstraints();
      }
      const bool found = !search(wire, pass.straightStart, true, false).empty();
      if (!crossing) {
        beginPass(wires, members, pass);
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
      blocked = attempt(false, false, false, false) ? "several together" : "the band itself";
    }
    // What the search had, put back.
    buildCorridor(wire, pass.reach);
    fence(wire, {&before, &after});
    std::ranges::fill(proximity_, 0);
    constrainByFeedlines(wire, wires, pass);
    return " · in the way: " + blocked;
  }

  /// Whether the way a wire has holds every rule the pass judges by: the
  /// rule against every other wire, the crossing rule, and the length of a
  /// resonator.
  [[nodiscard]] bool isLegal(Wire& wire, const std::vector<Wire>& wires) {
    if (!wire.drawn || wire.way.empty()) {
      return false;
    }
    const bool placed = wire.placed;
    lift(wire);
    const bool open = conflictsOf(wire, wires) != 0;
    if (placed) {
      place(wire);
    }
    if (open || crossesAFeedline(wire, wires)) {
      return false;
    }
    if (needsLength(wire)) {
      const auto length = lengthOf(wire.way);
      const auto required = requiredLength(wire);
      const auto tolerance = exact_ ? tuning_.lengthTolerance : 0.0;
      if (length < required - tolerance ||
          (exact_ && length > required + tolerance)) {
        return false;
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
                            const double y) const {
    const auto link = static_cast<double>(tuning_.clearance);
    const auto reach = 1.5 * link;
    // A resonator and the feedline edges of its own coupler run beside each
    // other along the coupler on purpose: that is the coupling. Everything
    // within the coupler's reach of its anchor belongs to that meeting.
    if (const auto shared = sharedCoupler(one, two); shared != NO_OWNER) {
      const auto& anchor = couplers_[shared].anchor;
      if (std::hypot(x - anchor.x, y - anchor.y) <= couplerReach()) {
        return true;
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
  /// Close everything outside the box the launcher stubs leave open.
  ///
  /// A hard bound on where a wire may run at all, not a price: the room
  /// between the launcher stubs and the edge of the chip is the room the
  /// stubs need, and a way through it is a way around the sources of the
  /// wires beside it. The starting and ending edges of a chain are the one
  /// exception — they come from a launcher on the border and have to cross
  /// that room to reach the first coupler.
  void closeOutsideBox() {
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto height = static_cast<std::int64_t>(scene_.router.height);
    for (std::int64_t y = 0; y < height; ++y) {
      const bool outsideRow = y < couplerBox_.minY || y > couplerBox_.maxY;
      for (std::int64_t x = 0; x < width; ++x) {
        if (outsideRow || x < couplerBox_.minX || x > couplerBox_.maxX) {
          corridor_.set(static_cast<std::size_t>((y * width) + x), true);
        }
      }
    }
  }

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
    Heading heading = 0;
    double nearest = std::numeric_limits<double>::max();
    for (const auto& [port, slot] : scene_.launcherCell) {
      const auto found = scene_.launcherHeading.find(port);
      if (found == scene_.launcherHeading.end()) {
        continue;
      }
      const auto place = scene_.router.cell(slot);
      const auto distance = std::hypot(static_cast<double>(place.x()) - x,
                                       static_cast<double>(place.y()) - y);
      if (distance < nearest) {
        nearest = distance;
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
  [[nodiscard]] std::uint32_t conflictsIn(const Path& way, const Wire& wire,
                                          const std::vector<Wire>& wires) {
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
    for (const auto& point : way) {
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
        if ((crossable && !other.resonator && !other.feedline) ||
            (other.feedline && !wire.resonator && !wire.feedline)) {
          continue;
        }
        if (meetAt(wire, wires[owner], 0.5 * (point.x + x),
                   0.5 * (point.y + y))) {
          continue;
        }
        ++found;
        break;
      }
    }
    return found;
  }

  /// The search itself, with the stubs of this pass. Its picture is taken
  /// here unless the caller has more to put into it.
  [[nodiscard]] Path search(const Wire& wire, const std::uint32_t straightStart,
                            const bool usePenalty, const bool draw = true) {
    router_.setParams({.startStraightLength = startStubOf(wire, straightStart),
                       .endStraightLength = wire.endStub,
                       .minRadius = BEND_RADIUS,
                       .bendPenalty = tuning_.bendPenalty});
    router_.attachCorridor(&corridor_);
    lastPicture_.clear();
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
    if (draw) {
      drawSearch(wire, found, {});
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
  void buildCorridor(const Wire& wire, const std::uint32_t reach) {
    blockOnBodies_ = !wire.feedline;
    corridor_.fill(true);
    box_ = {};
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
    const auto& stencil = stencilFor(tuning_.clearance);
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    for (const Wire* const other : others) {
      if (other == &wire || other->way.empty()) {
        continue;
      }
      const bool meeting = couldMeet(wire, *other);
      alongDisc(other->way, stencil,
                [&](const std::int64_t x, const std::int64_t y) {
                  const auto cell = static_cast<std::size_t>((y * width) + x);
                  if (meeting && field_.owner(cell) != other->key &&
                      meetAt(wire, *other, static_cast<double>(x),
                             static_cast<double>(y))) {
                    return;
                  }
                  corridor_.set(cell, true);
                });
    }
    openFixedPlaces(wire);
  }

  /// A wire's own fixed places are always enterable. They are where the wire
  /// has to be, and the search is the one pass that may stand on them.
  void openFixedPlaces(const Wire& wire) {
    for (const auto& point : wire.fixed) {
      if (point.x < scene_.router.width && point.y < scene_.router.height) {
        corridor_.setCell(point.x, point.y, false);
      }
    }
  }

  /// Whether two wires meet anywhere: a terminal of one within the rule of a
  /// terminal of the other. The cheap half of `meetAt`, asked once per fence
  /// wire before every cell is asked.
  [[nodiscard]] bool couldMeet(const Wire& one, const Wire& two) const {
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
  void alongDisc(const Path& way, const Stencil& stencil, Apply&& apply) const {
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
    for (const auto& point : way) {
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

    const auto price = tuning_.wireProximityPenalty;
    std::ranges::fill(proximity_, price);
    fillLane(wire, before, after, 0);

    const auto neighbour =
        forward ? (slot + 1) % total : (slot + total - 1) % total;
    for (std::uint32_t step = 0; step <= level; ++step) {
      const auto at = forward ? (neighbour + step) % total
                              : (neighbour + total - step) % total;
      stampDisc(wires[members[at]].way, (step + 1) * tuning_.clearance, price);
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
  void priceApproaches(const Wire& other, const std::uint8_t price) {
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
                     const bool leaving, const std::uint8_t price) {
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

  /// Clear the price inside the lane, by a scanline fill of its polygon.
  void fillLane(const Wire& wire, const Wire& before, const Wire& after,
                const std::uint8_t inside) {
    if (wire.way.empty()) {
      return;
    }
    std::vector<std::pair<double, double>> polygon;
    polygon.reserve(before.way.size() + after.way.size() + 2);
    const auto add = [&polygon](const PathPoint& point) {
      polygon.emplace_back(static_cast<double>(point.x) + 0.5,
                           static_cast<double>(point.y) + 0.5);
    };
    add(wire.way.front());
    for (const auto& point : before.way) {
      add(point);
    }
    add(wire.way.back());
    for (const auto& point : std::ranges::reverse_view(after.way)) {
      add(point);
    }
    if (polygon.size() < 3) {
      return;
    }

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

  /// Price a disc of a radius around every cell of a way.
  void stampDisc(const Path& way, const std::uint32_t radius,
                 const std::uint8_t price) {
    if (way.empty() || radius == 0 || price == 0) {
      return;
    }
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    alongDisc(way, stencilFor(radius),
              [&](const std::int64_t x, const std::int64_t y) {
                proximity_[static_cast<std::size_t>((y * width) + x)] = price;
              });
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
    if (box_.empty()) {
      return;
    }
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    constexpr std::int32_t FAR = 1000000;
    wall_.assign(scene_.router.cells(), 0);
    middle_.assign(scene_.router.cells(), FAR);

    const auto lowX = box_.minX;
    const auto lowY = box_.minY;
    const auto highX = box_.maxX;
    const auto highY = box_.maxY;
    for (std::int64_t y = lowY; y <= highY; ++y) {
      for (std::int64_t x = lowX; x <= highX; ++x) {
        const auto cell = static_cast<std::size_t>((y * width) + x);
        const bool border = y == lowY || x == lowX || y == highY || x == highX;
        wall_[cell] = (corridor_.test(cell) || border) ? 0 : FAR;
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

    for (std::size_t cell = 0; cell < proximity_.size(); ++cell) {
      const auto dw = wall_[cell];
      const auto dc = middle_[cell];
      if (dw == 0 || dc >= FAR) {
        proximity_[cell] = price;
        continue;
      }
      proximity_[cell] = static_cast<std::uint8_t>(
          ((dc * price) + ((dc + dw) / 2)) / (dc + dw));
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

  // --- The debug pictures --------------------------------------------------

  /// The font and the markers of a picture, from how wide it is, so that a
  /// picture of a whole chip and one of a single band both read.
  struct Scale {
    double font = 8.0;
    double marker = 3.0;
  };
  [[nodiscard]] static Scale scaleOf(const debug::View& view) {
    const auto width = static_cast<double>(view.width());
    return {.font = std::clamp(width / 70.0, 4.0, 28.0),
            .marker = std::clamp(width / 160.0, 1.5, 8.0)};
  }

  /// The room a legend of so many lines needs above the view.
  [[nodiscard]] static double headroomFor(const Scale& scale,
                                          const std::size_t lines) {
    return scale.font * ((static_cast<double>(lines) * 1.4) + 1.6);
  }

  /// The two ends of a wire: a marker each, the heading it leaves or arrives
  /// on as a line the length of the clearance, and its name.
  void drawEnds(debug::Svg& svg, const Wire& wire, const Scale& scale) const {
    const auto reach = static_cast<double>(tuning_.clearance);
    const auto& source = wire.objective.source;
    const auto& target = wire.objective.target;
    const auto leaving = routing::headingVector(source.heading);
    const auto arriving = routing::headingVector(target.heading);
    svg.line(svg.centreX(source.x), svg.centreY(source.y),
             svg.centreX(source.x) + (leaving.dx * reach),
             svg.centreY(source.y) - (leaving.dy * reach), "ar");
    svg.line(svg.centreX(target.x) - (arriving.dx * reach),
             svg.centreY(target.y) + (arriving.dy * reach),
             svg.centreX(target.x), svg.centreY(target.y), "ar");
    svg.circle(source.x, source.y, scale.marker, "src");
    svg.circle(target.x, target.y, scale.marker, "tgt");
    const auto id = wireId(wire);
    svg.text(svg.centreX(source.x) + scale.marker,
             svg.centreY(source.y) - scale.marker, id, "lb", scale.font);
    svg.text(svg.centreX(target.x) + scale.marker,
             svg.centreY(target.y) - scale.marker, id, "lb", scale.font);
  }

  /// A legend in the headroom above the view, so that it covers no cell: a
  /// title, then one line per thing drawn, with the swatch it is drawn in.
  void drawLegend(
      debug::Svg& svg, const Scale& scale,
      const std::vector<std::string>& title,
      const std::vector<std::pair<std::string, std::string>>& entries) const {
    const auto& view = svg.view();
    const auto x = debug::Svg::left(view.minX) + scale.font;
    auto y = svg.ceiling() + scale.font;
    const auto rows = static_cast<double>(title.size() + entries.size());
    svg.box(x - (0.5 * scale.font), y - (0.5 * scale.font), scale.font * 60.0,
            scale.font * ((rows * 1.4) + 0.6), "lg");
    for (const auto& line : title) {
      svg.text(x, y + scale.font, line, "lb", scale.font);
      y += 1.4 * scale.font;
    }
    for (const auto& [cls, what] : entries) {
      svg.box(x, y + (0.15 * scale.font), scale.font, scale.font, cls);
      svg.text(x + (1.6 * scale.font), y + scale.font, what, "lb", scale.font);
      y += 1.4 * scale.font;
    }
  }

  /// Every port whose cell lies in the view, as the chip carries it: the
  /// square on the port's cell, a dot at its exact position, the line along
  /// its orientation, the index beside it and the label as a tooltip.
  void drawPorts(debug::Svg& svg, const Scale& scale) const {
    const auto& view = svg.view();
    const auto reach = static_cast<double>(tuning_.clearance);
    for (const auto& port : ports_) {
      if (port.x < view.minX || port.x > view.maxX || port.y < view.minY ||
          port.y > view.maxY) {
        continue;
      }
      svg.square(port.x, port.y, scale.marker, "prt",
                 std::format("port {} · {} · exactly at cell ({:.2f}, {:.2f})",
                             port.index, port.label, port.fx, port.fy));
      svg.dot(debug::Svg::atX(port.fx), svg.atY(port.fy), 0.6 * scale.marker,
              "prx");
      if (port.step.x != 0 || port.step.y != 0) {
        svg.line(debug::Svg::atX(port.fx), svg.atY(port.fy),
                 debug::Svg::atX(port.fx) + (port.step.x * reach),
                 svg.atY(port.fy) - (port.step.y * reach), "pra");
      }
      svg.text(svg.centreX(port.x) - scale.marker,
               svg.centreY(port.y) + (2.2 * scale.marker),
               std::format("p{}", port.index), "prl", 0.8 * scale.font);
    }
  }

  /// For every wire whose end lies in the view: a dashed line from the exact
  /// position of its port to the cell it is routed to, beyond the port's
  /// band, and the distance between the two in layout units and in cells.
  void drawPortDistances(debug::Svg& svg, const Scale& scale,
                         const std::vector<Wire>& wires) const {
    const auto& view = svg.view();
    const auto inside = [&view](const PathPoint& point) {
      return point.x >= view.minX && point.x <= view.maxX &&
             point.y >= view.minY && point.y <= view.maxY;
    };
    const auto annotate = [&](const PathPoint& end, const std::uint32_t port) {
      if (port >= ports_.size() || !inside(end)) {
        return;
      }
      const auto& mark = ports_[port];
      const auto dx = static_cast<double>(end.x) - mark.fx;
      const auto dy = static_cast<double>(end.y) - mark.fy;
      const auto cells = std::hypot(dx, dy);
      const auto units = std::hypot(dx * scene_.router.cellWidth,
                                    dy * scene_.router.cellHeight);
      const auto x0 = debug::Svg::atX(mark.fx);
      const auto y0 = svg.atY(mark.fy);
      const auto x1 = svg.centreX(end.x);
      const auto y1 = svg.centreY(end.y);
      svg.line(x0, y0, x1, y1, "dst");
      svg.text((0.5 * (x0 + x1)) + scale.marker,
               (0.5 * (y0 + y1)) - scale.marker,
               std::format("{:.1f} u · {:.1f} cells", units, cells), "prl",
               0.8 * scale.font);
    };
    for (const auto& wire : wires) {
      if (!wire.feasible) {
        continue;
      }
      annotate(wire.objective.target, wire.targetPort);
      if (wire.inner) {
        annotate(wire.objective.source, wire.sourcePort);
      }
    }
  }

  /// The obstacles and the price of running beside them, over a view.
  /// The rectangle the launcher stubs leave open, which is where a coupler
  /// may sit. Drawn on every search picture, because an edge that detours is
  /// usually an edge whose coupler sits outside it.
  void drawCouplerBox(debug::Svg& svg) const {
    if (couplerBox_.empty()) {
      return;
    }
    const auto x0 = debug::Svg::atX(static_cast<double>(couplerBox_.minX) - 0.5);
    const auto x1 = debug::Svg::atX(static_cast<double>(couplerBox_.maxX) + 0.5);
    const auto y0 = svg.atY(static_cast<double>(couplerBox_.maxY) + 0.5);
    const auto y1 = svg.atY(static_cast<double>(couplerBox_.minY) - 0.5);
    svg.box(x0, y0, x1 - x0, y1 - y0, "cbx");
  }

  void drawGround(debug::Svg& svg, const debug::View& view) const {
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto cellOf = [width](const std::int64_t x, const std::int64_t y) {
      return static_cast<std::size_t>((y * width) + x);
    };
    const auto& halo = router_.staticProximity();
    const int haloMost = std::max<int>(1, tuning_.staticProximityPenalty);
    static_cast<void>(view);
    svg.runs(
        [&](const std::int64_t x, const std::int64_t y) {
          return halo.empty() ? 0 : levelOf(halo[cellOf(x, y)], haloMost);
        },
        [](const int level) { return std::format("s{}", level); });
    svg.runs(
        [&](const std::int64_t x, const std::int64_t y) {
          return scene_.blocked.test(cellOf(x, y)) ? 1 : 0;
        },
        [](const int) { return std::string("ob"); });
  }

  /// A picture of one search: what it may enter, what fences it, what it
  /// pays, the wires around it, and what it found. Taken right after the
  /// search, so it shows exactly what the router was given; for a resonator
  /// the way carries its meander and the note says what the lengthening
  /// came to.
  void drawSearch(const Wire& wire, const Path& found,
                  const std::string& note) {
    if (!debug_ || box_.empty() || frame_.wires == nullptr) {
      return;
    }
    const auto& wires = *frame_.wires;
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto cellOf = [width](const std::int64_t x, const std::int64_t y) {
      return static_cast<std::size_t>((y * width) + x);
    };
    const debug::View view{.minX = box_.minX,
                           .minY = box_.minY,
                           .maxX = box_.maxX,
                           .maxY = box_.maxY};
    const auto scale = scaleOf(view);
    debug::Svg svg(view, scene_.router.height, headroomFor(scale, 15));
    svg.style(DEBUG_STYLE);

    // The ground: the halo, the artwork, what the search may enter, and
    // what it pays for entering it.
    drawGround(svg, view);
    drawCouplerBox(svg);
    {
      std::size_t shut = 0;
      std::size_t art = 0;
      for (std::int64_t y = box_.minY; y <= box_.maxY; ++y) {
        for (std::int64_t x = box_.minX; x <= box_.maxX; ++x) {
          const auto cell = cellOf(x, y);
          shut += corridor_.test(cell) ? 1 : 0;
          art += scene_.components.test(cell) ? 1 : 0;
        }
      }
      tell(std::format("picture {}: {} cells closed in the corridor, {} of "
                       "them artwork",
                       frame_.kind, shut, art));
    }
    svg.runs(
        [&](const std::int64_t x, const std::int64_t y) {
          return corridor_.test(cellOf(x, y)) ? 0 : 1;
        },
        [](const int) { return std::string("co"); });
    // The levels are scaled to the dearest cell in the box, so that a price
    // ten times the ordinary one is drawn ten times as dark and not the same.
    int priceMost = 1;
    for (std::int64_t y = box_.minY; y <= box_.maxY; ++y) {
      for (std::int64_t x = box_.minX; x <= box_.maxX; ++x) {
        priceMost = std::max<int>(priceMost, proximity_[cellOf(x, y)]);
      }
    }
    svg.runs(
        [&](const std::int64_t x, const std::int64_t y) {
          return levelOf(proximity_[cellOf(x, y)], priceMost);
        },
        [](const int level) { return std::format("p{}", level); });

    // The couplers and the feedlines as they stand right now, whatever the
    // search is. A picture of one edge used to show the ground and that
    // edge alone, so what the edge had to get past was not in it: the pads
    // it runs between and the chains already drawn. Both are drawn under
    // the wires of the search, so they never hide them.
    for (const auto& coupler : couplers_) {
      if (coupler.chosen >= coupler.options.size()) {
        continue;
      }
      const auto& option = coupler.options[coupler.chosen];
      const auto along = routing::headingVector(option.orientation);
      const auto across =
          routing::headingVector(routing::turned(option.orientation, -2));
      const std::int64_t halfRun = option.run / 2;
      const std::int64_t halfDepth = option.depth / 2;
      const auto cx = static_cast<std::int64_t>(option.centre.x);
      const auto cy = static_cast<std::int64_t>(option.centre.y);
      std::vector<debug::Cell> pad;
      for (const auto& [a, d] : {std::pair{-1, -1}, std::pair{1, -1},
                                 std::pair{1, 1}, std::pair{-1, 1}}) {
        pad.emplace_back(
            cx + (a * halfRun * along.dx) + (d * halfDepth * across.dx),
            cy + (a * halfRun * along.dy) + (d * halfDepth * across.dy));
      }
      svg.polygon(pad, "ocp");
      if (!option.arc.empty()) {
        auto lead = cellsOf(option.arc);
        lead.emplace_back(option.arcEnd.x, option.arcEnd.y);
        svg.polyline(lead, "ocl");
      }
    }
    // Over the chains and not over `edges_`: that list is filled at the
    // commit, so during the greedy — which is where most of these pictures
    // are taken — it is empty, and the feedlines went undrawn in every one
    // of them. `edgeWayOf` gives the drawn wire's way once there is one and
    // what the greedy last chose before that, so this holds in both.
    for (std::uint32_t chain = 0; chain < chains_.size(); ++chain) {
      for (std::size_t at = 0; at < chainEdgePaths_[chain].size(); ++at) {
        const auto key = edgeWireOf_[chain][at];
        if (key == wire.key) {
          continue;
        }
        const auto& way = edgeWayOf(wires, chain, at);
        if (way.empty()) {
          continue;
        }
        // A drawn edge and a path the greedy chose are not the same thing
        // and must not look the same. Once the commit has been past an edge
        // and it found nothing, what is left in `chainEdgePaths_` is a way
        // that will never be built — and drawn like the rest it shows a
        // chip that never existed. On 17q this put two feedlines crossing in
        // the picture of an edge whose own way crosses nothing: the crossing
        // was against the abandoned path of an edge that had already failed.
        const bool drawn = key != NO_OWNER && key < wires.size() &&
                           wires[key].drawn;
        svg.polyline(cellsOf(way), drawn ? "fw" : "fwg");
      }
    }

    // The wires around it: the lane between the ring neighbours, the
    // neighbours, the fence, the wires let go of, and its own way.
    const auto nameOf = [&wires](const std::uint32_t key) {
      return key < wires.size() ? wireId(wires[key]) : std::string("-");
    };
    const auto wayOf = [&wires](const std::uint32_t key) -> const Path& {
      static const Path none;
      return key < wires.size() ? wires[key].way : none;
    };
    if (frame_.kind != "normal" && !wire.way.empty()) {
      std::vector<debug::Cell> lane;
      lane.emplace_back(wire.way.front().x, wire.way.front().y);
      for (const auto& point : wayOf(frame_.before)) {
        lane.emplace_back(point.x, point.y);
      }
      lane.emplace_back(wire.way.back().x, wire.way.back().y);
      const auto& after = wayOf(frame_.after);
      for (const auto& point : std::ranges::reverse_view(after)) {
        lane.emplace_back(point.x, point.y);
      }
      svg.polygon(lane, "lane");
    }
    for (const auto key : {frame_.before, frame_.after}) {
      svg.polyline(cellsOf(wayOf(key)), "nb");
    }
    const auto band =
        std::format("stroke-width=\"{}\"", (2 * tuning_.clearance) + 1);
    std::string fenced;
    for (const auto key : frame_.fence) {
      svg.polyline(cellsOf(wayOf(key)), "fz", band);
      svg.polyline(cellsOf(wayOf(key)), "fw");
      fenced += (fenced.empty() ? "" : ",") + nameOf(key);
    }
    std::string ripped;
    for (const auto key : frame_.ripped) {
      svg.polyline(cellsOf(wayOf(key)), "rp");
      ripped += (ripped.empty() ? "" : ",") + nameOf(key);
    }
    svg.polyline(cellsOf(wire.way), "ow");
    svg.polyline(cellsOf(found), "fd");
    drawPorts(svg, scale);
    drawPortDistances(svg, scale, wires);
    drawEnds(svg, wire, scale);

    const auto result = found.empty()
                            ? std::string("no way")
                            : std::format("found {} cells", found.size());
    drawLegend(
        svg, scale,
        {std::format("{} · round {} {} · wire {} · {} · {}{}", frame_.pass,
                     frame_.round, frame_.forward ? "forward" : "backward",
                     wireId(wire), frame_.kind, result,
                     note.empty() ? "" : " · " + note),
         std::format("fence {} · ripped {} · lane between {} and {} · box x "
                     "{}..{} y {}..{}",
                     fenced.empty() ? "-" : fenced,
                     ripped.empty() ? "-" : ripped, nameOf(frame_.before),
                     nameOf(frame_.after), box_.minX, box_.maxX, box_.minY,
                     box_.maxY),
         std::format("clearance {} cells · band {} cells · wire price {} · "
                     "obstacle price {} over {} cells · bend {}",
                     tuning_.clearance, tuning_.reach,
                     tuning_.wireProximityPenalty,
                     tuning_.staticProximityPenalty, tuning_.obstacleReach,
                     tuning_.bendPenalty)},
        {{"ob", "artwork and keepout: closed"},
         {"s8", "obstacle halo: priced, darker is dearer"},
         {"co", "band: what the search may enter"},
         {"p8", "wire price: outside the lane, around the wires let go of, "
                "ten times on the approaches of the wires around"},
         {"fz", "fence: the clearance around the fence wires, closed"},
         {"nb", "ring neighbours"},
         {"rp", "let go of: drawn again afterwards; crossable unless it is "
                "the fence"},
         {"ow", "the way it had"},
         {"fd", "the way it found"},
         {"src", "source, line = heading it leaves on"},
         {"tgt", "target, line = heading it arrives on"},
         {"prt", "port as the chip carries it: square = its cell, dot = exact "
                 "position, dashed = distance to the target beyond its band"},
         {"cbx", "where a coupler may sit: what the launcher stubs leave open"},
         {"ocp", "the couplers as they stand: pad and resonator lead"},
         {"fw", "the feedlines as they stand: drawn"},
         {"fwg", "what the greedy chose for an edge that has no way: dashed, "
                 "and never built"}});

    std::string kind = frame_.kind;
    std::erase(kind, ' ');
    lastPicture_ =
        debug_(std::format("final-{:05}-{}-r{}{}-w{}-{}.svg", ++pictures_,
                           frame_.pass, frame_.round,
                           frame_.forward ? 'f' : 'b', wireId(wire), kind),
               svg.finish());
  }

public:
  /// A picture of the whole router grid before anything is drawn: the
  /// artwork, the price of running beside it, every port as the chip carries
  /// it, every wire's two ends with the heading it leaves and arrives on, and
  /// the way the Detail stage drew.
  void attachPorts(std::vector<PortMark> ports) { ports_ = std::move(ports); }

  /// One picture of every coupler option there is, for reading with the
  /// eye what the numbers only count: the outer wires as they stand, and on
  /// every coupler each option it may take — the pad, the resonator lead
  /// off it and the feedline port it offers. The option the coupler starts
  /// on is drawn over the others in a second colour, so that "is the right
  /// one there at all, and is it the one we begin with" is one glance.
  ///
  /// Drawn once, before the greedy runs.
  void drawCouplerOptions(const std::vector<Wire>& wires) {
    if (!debug_) {
      return;
    }
    const debug::View view{
        .minX = 0,
        .minY = 0,
        .maxX = static_cast<std::int64_t>(scene_.router.width) - 1,
        .maxY = static_cast<std::int64_t>(scene_.router.height) - 1};
    const auto scale = scaleOf(view);
    debug::Svg svg(view, scene_.router.height, headroomFor(scale, 8));
    svg.style(DEBUG_STYLE);
    drawGround(svg, view);
    drawCouplerBox(svg);

    // Every outer wire as it stands going in.
    for (const auto& wire : wires) {
      if (wire.feasible && !wire.inner && !wire.feedline && !wire.way.empty()) {
        svg.polyline(cellsOf(wire.way), "sd");
      }
    }

    // The pad of one option, as the four corners of its rectangle.
    const auto padOf = [this](const CouplerOption& option) {
      const auto along = routing::headingVector(option.orientation);
      const auto across =
          routing::headingVector(routing::turned(option.orientation, -2));
      const std::int64_t halfRun = option.run / 2;
      const std::int64_t halfDepth = option.depth / 2;
      const auto cx = static_cast<std::int64_t>(option.centre.x);
      const auto cy = static_cast<std::int64_t>(option.centre.y);
      std::vector<debug::Cell> corners;
      for (const auto& [a, d] : {std::pair{-1, -1}, std::pair{1, -1},
                                 std::pair{1, 1}, std::pair{-1, 1}}) {
        corners.emplace_back(
            cx + (a * halfRun * along.dx) + (d * halfDepth * across.dx),
            cy + (a * halfRun * along.dy) + (d * halfDepth * across.dy));
      }
      return corners;
    };

    std::uint32_t options = 0;
    for (const auto& coupler : couplers_) {
      for (std::size_t at = 0; at < coupler.options.size(); ++at) {
        const auto& option = coupler.options[at];
        const bool starts = at == coupler.chosen;
        ++options;
        svg.polygon(padOf(option), starts ? "ocp" : "opt");
        if (!option.arc.empty()) {
          auto lead = cellsOf(option.arc);
          lead.emplace_back(option.arcEnd.x, option.arcEnd.y);
          svg.polyline(lead, starts ? "ocl" : "opl");
        }
        svg.circle(option.out.x, option.out.y, starts ? 2.2 : 1.2,
                   starts ? "ocs" : "ops");
      }
    }

    drawPorts(svg, scale);
    drawLegend(
        svg, scale,
        {std::format("coupler options · {} couplers · {} options in all · "
                     "before the greedy",
                     couplers_.size(), options),
         std::format("pad {}x{} cells · lead {} straight · clearance {} cells",
                     tuning_.couplerLength, tuning_.couplerHeight,
                     leadStraight(), tuning_.clearance)},
        {{"ob", "artwork and keepout"},
         {"sd", "the outer wires as they stand"},
         {"cbx", "the box the launcher stubs leave open"},
         {"opt", "an option's pad"},
         {"opl", "that option's resonator lead, port to insertion point"},
         {"ops", "that option's feedline port"},
         {"ocp", "the option the coupler starts on: pad, lead and port"},
         {"prt", "port as the chip carries it"}});
    const auto where = debug_("final-coupler-options.svg", svg.finish());
    if (!where.empty()) {
      say(std::format("the coupler options: {}", where));
    }
  }

  void drawGrid(const std::vector<Wire>& wires) {
    if (!debug_) {
      return;
    }
    const debug::View view{
        .minX = 0,
        .minY = 0,
        .maxX = static_cast<std::int64_t>(scene_.router.width) - 1,
        .maxY = static_cast<std::int64_t>(scene_.router.height) - 1};
    const auto scale = scaleOf(view);
    debug::Svg svg(view, scene_.router.height, headroomFor(scale, 8));
    svg.style(DEBUG_STYLE);
    drawGround(svg, view);
    for (const auto& wire : wires) {
      if (wire.feasible) {
        svg.polyline(cellsOf(wire.way), "sd");
      }
    }
    // The ports themselves, before the wire ends, which lie a band beyond
    // them.
    drawPorts(svg, scale);
    for (const auto& wire : wires) {
      if (wire.feasible) {
        drawEnds(svg, wire, scale);
      }
    }
    drawLegend(
        svg, scale,
        {std::format(
             "final routing · grid {}x{} cells of {:.2f} layout "
             "units · {} wires",
             scene_.router.width, scene_.router.height,
             std::min(scene_.router.cellWidth, scene_.router.cellHeight),
             wires.size()),
         std::format("clearance {} cells · stub {} cells · band {} cells · "
                     "wire price {} · obstacle price {} over {} cells · "
                     "bend {}",
                     tuning_.clearance, tuning_.straightStart, tuning_.reach,
                     tuning_.wireProximityPenalty,
                     tuning_.staticProximityPenalty, tuning_.obstacleReach,
                     tuning_.bendPenalty)},
        {{"ob", "artwork and keepout: closed to every search"},
         {"s8", "obstacle halo: priced, darker is dearer"},
         {"sd", "the way the Detail stage drew, the seed of every search"},
         {"prt", "port as the chip carries it: square = its cell, dot = exact "
                 "position, line = orientation, p<n> = its index, label on "
                 "hover"},
         {"src", "source, line = heading it leaves on, number = wire"},
         {"tgt",
          "target beyond the port's band, line = heading it arrives on"}});
    const auto where = debug_("final-grid.svg", svg.finish());
    if (!where.empty()) {
      say(std::format("the grid: {}", where));
    }
  }

private:
  const Scene& scene_;
  Tuning tuning_;
  Progress progress_;
  Debug debug_;
  std::uint32_t verbosity_ = 0;
  DebugFrame frame_;
  std::uint32_t pictures_ = 0;
  /// The ports as the chip carries them, for the pictures.
  std::vector<PortMark> ports_;
  /// Where the picture of the last search went, for the line about it.
  std::string lastPicture_;
  /// What each round of the pass running now came to, per wire.
  struct RoundRecord {
    bool forward = true;
    std::uint32_t normal = 0;
    std::uint32_t relaxed = 0;
    /// Of the failed, the resonators that kept a way too short.
    std::uint32_t tooShort = 0;
    std::vector<std::string> failed;
  };
  std::vector<RoundRecord> rounds_;
  std::chrono::steady_clock::time_point began_ =
      std::chrono::steady_clock::now();
  std::shared_ptr<const MovePrimitives> primitives_;
  routing::SearchScratch scratch_;
  routing::DubinsRouter router_;
  Field field_;
  grid::BitGrid corridor_;
  std::vector<std::uint8_t> proximity_;
  std::vector<std::uint32_t> seen_;
  std::vector<std::size_t> queue_;
  std::vector<std::uint32_t> depth_;
  std::vector<std::int32_t> wall_;
  std::vector<std::int32_t> middle_;
  std::unordered_map<std::uint32_t, Stencil> stencils_;
  Box box_;
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
  /// One entry per cell: the resonator whose surviving way holds it, or
  /// nobody. No coupler may be built over one.
  std::vector<std::uint32_t> tailOwner_;
  /// How often an edge of the greedy had to be weighed against its chain
  /// neighbour's copper alone, because it found no way that kept the
  /// clearance from it. The fallback is what keeps a chain whole; the count
  /// is what makes two runs comparable when it fires in one and not the
  /// other.
  std::uint32_t looseEdges_ = 0;
  /// A price field of nothing, for the searches that price nothing. Filled
  /// once and never written again.
  std::vector<std::uint8_t> zeroProximity_;

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
  std::unordered_map<std::uint64_t, Path> edgeMemo_;
  std::uint64_t memoHits_ = 0;
  std::uint64_t memoMisses_ = 0;

  /// The option a waypoint stands on; a launcher has none and counts zero.
  [[nodiscard]] std::size_t optionAt(const std::uint32_t chain,
                                     const std::size_t at) const {
    const auto& point = chains_[chain][at];
    return point.fixed ? 0 : couplers_[point.coupler].chosen + 1;
  }

  [[nodiscard]] std::uint64_t edgeMemoKey(const std::uint32_t chain,
                                          const std::size_t from) const {
    return (static_cast<std::uint64_t>(chain) << 48U) |
           (static_cast<std::uint64_t>(from) << 32U) |
           (static_cast<std::uint64_t>(optionAt(chain, from)) << 16U) |
           static_cast<std::uint64_t>(optionAt(chain, from + 1));
  }

  /// The approaches of every port, inflated by the clearance, where no
  /// coupler may sit.
  grid::BitGrid approaches_;
  /// The rectangle the launcher stubs leave open, which a coupler sits in.
  CouplerBox couplerBox_;
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
  /// How far the crossing rule reaches from a feedline's straight run, in
  /// cells: the prototype's `expand_radius`.
  static constexpr int CROSSING_REACH = 10;
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
    wires.push_back(std::move(wire));
  }
  return wires;
}

// ------------------------------------------------------------- The artifact

/// How long a way is, in layout units.
using Measure = std::function<double(const Path&)>;

[[nodiscard]] std::unique_ptr<fba::FinalWireT> wireOf(const Path& way,
                                                      const double length) {
  auto drawn = std::make_unique<fba::FinalWireT>();
  drawn->path.reserve(way.size());
  for (const auto& point : way) {
    drawn->path.emplace_back(point.x, point.y, point.heading);
  }
  drawn->length = length;
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

/// The state of every wire, as one phase left it.
[[nodiscard]] std::unique_ptr<fba::FinalPhaseT>
snapshotOf(std::string name, const std::vector<Wire>& wires,
           const std::uint32_t ring, const std::size_t inner,
           const Measure& measure, const std::vector<std::uint32_t>& edges = {},
           const CouplerList& couplers = {}) {
  auto phase = std::make_unique<fba::FinalPhaseT>();
  phase->name = std::move(name);
  for (const auto key : edges) {
    const auto& wire = wires[key];
    phase->feedlines.push_back(wire.drawn ? wireOf(wire.way, measure(wire.way))
                                          : wireOf({}, 0.0));
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
    if (!wire.drawn || wire.feedline) {
      continue;
    }
    auto& into = wire.inner ? phase->inner : phase->wires;
    if (wire.slot < into.size()) {
      into[wire.slot] = wireOf(wire.way, measure(wire.way));
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
class DubinsFinalRouter final : public IFinalRouter {
public:
  [[nodiscard]] FinalRoutingT
  run(const ChipT& chip, const CapacityPlanT& /*capacity*/,
      const GlobalRoutingT& global, const AssignmentT& assignment,
      const DetailRoutingT& detail, const ConfigT& config,
      const Progress& progress, const Debug& debug,
      const std::uint32_t verbosity) const override {
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

    Driver driver(field, tuning, progress, debug, verbosity);
    driver.attachPorts(portMarksOf(chip, field));
    driver.drawGrid(wires);
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

    // Where the run ends. `stop_after` names the last phase to run, by the
    // name of its snapshot; what follows it is not run at all, which is what
    // a stop is worth over a picture of a finished run. An unknown name runs
    // every phase, because the loader has already refused it.
    static constexpr std::array<std::string_view, 5> PHASES{
        "inner", "outer", "couplers", "feedlines", "refined"};
    std::size_t lastPhase = PHASES.size() - 1;
    for (std::size_t at = 0; at < PHASES.size(); ++at) {
      if (tuning.stopAfter == PHASES[at]) {
        lastPhase = at;
      }
    }
    const auto runs = [&lastPhase](const std::size_t phase) {
      return phase <= lastPhase;
    };

    // Phase 1. The inner circuit runs inside the unit cells, where the room is
    // narrow and there is nothing to steer around, so it takes no straight
    // stubs and a band a fraction of the ring's.
    driver.sayTheSetting();
    driver.sweep(wires, inner,
                 {.name = "inner routing",
                  .rounds = tuning.innerRounds,
                  .maxRelaxation = tuning.maxRelaxation,
                  .reach = tuning.innerReach,
                  .straightStart = 0,
                  .keepDrawn = true});
    routing.phases.push_back(
        snapshotOf("inner", wires, ring, global.connections.size(), measure));

    // Phase 2. The ring, against the inner circuit and against itself.
    if (runs(1)) {
      driver.sweep(wires, outer,
                   {.name = "outer routing",
                    .rounds = tuning.rounds,
                    .maxRelaxation = tuning.maxRelaxation,
                    .reach = tuning.reach,
                    .straightStart = tuning.straightStart,
                    .keepDrawn = true});
      driver.refine(wires, outer,
                    {.name = "refinement",
                     .rounds = tuning.refinementRounds,
                     .maxRelaxation = 0,
                     .reach = tuning.reach,
                     .straightStart = tuning.straightStart,
                     .keepDrawn = false});
      routing.phases.push_back(
          snapshotOf("outer", wires, ring, global.connections.size(), measure));
    }

    // What the tail writes, whatever the run stops after: the edges of the
    // chains and the couplers as they stand. Every wire is counted from the
    // list as it is at the time, because the coupler insertion appends the
    // edges of the chains to it.
    std::vector<std::uint32_t> edges;
    const auto couplersNow = [&]() {
      return couplersOf(driver, wires, chip, field);
    };
    const auto everyWire = [&wires]() {
      std::vector<std::uint32_t> all(wires.size());
      std::iota(all.begin(), all.end(), 0U);
      return all;
    };

    // Phase 3. The couplers, and the feedline chains through them.
    if (runs(2)) {
      driver.insertCouplers(wires, assignment, chip);
      for (const auto& edge : driver.edges()) {
        edges.push_back(edge.wire);
      }
      routing.phases.push_back(snapshotOf("couplers", wires, ring,
                                          global.connections.size(), measure,
                                          edges, couplersNow()));
    }

    // Phase 4. Every wire again under the feedline constraints, in the ring
    // with the edges of the chains, then the inner circuit; and the repair,
    // which turns a coupler while the feedlines leave fails.
    if (runs(3)) {
      driver.assignBridges(wires, outer);
      const auto members = driver.ringWithEdges(wires, outer);
      const Pass feedlinePass{.name = "feedline routing",
                              .rounds = tuning.rounds,
                              .maxRelaxation = tuning.maxRelaxation,
                              .reach = tuning.reach,
                              .straightStart = tuning.straightStart,
                              .keepDrawn = true,
                              .feedlines = true};
      driver.sweep(wires, members, feedlinePass);
      driver.sweep(wires, inner,
                   {.name = "inner routing under the feedlines",
                    .rounds = tuning.innerRounds,
                    .maxRelaxation = tuning.maxRelaxation,
                    .reach = tuning.innerReach,
                    .straightStart = 0,
                    .keepDrawn = true,
                    .feedlines = true});
      driver.repair(wires, members, everyWire(), feedlinePass);
      routing.phases.push_back(snapshotOf("feedlines", wires, ring,
                                          global.connections.size(), measure,
                                          edges, couplersNow()));

      // Phase 5. The refinement of the feedline routing.
      if (runs(4)) {
        driver.refine(wires, members,
                      {.name = "feedline refinement",
                       .rounds = tuning.feedlineRefinementRounds,
                       .maxRelaxation = 0,
                       .reach = tuning.reach,
                       .straightStart = tuning.straightStart,
                       .keepDrawn = false,
                       .feedlines = true});
        routing.phases.push_back(snapshotOf("refined", wires, ring,
                                            global.connections.size(), measure,
                                            edges, couplersNow()));
      }
    }

    // What the stage came to, over every wire of the plan and counted with
    // every wire down, so that the last line of a run says what the
    // design-rule check will find.
    if (lastPhase + 1 < PHASES.size()) {
      driver.say(std::format("the stage stops after {}, as stop_after asks",
                             PHASES[lastPhase]));
    }
    driver.sayResonatorLengths(wires, outer);
    driver.sayFails("final routing", driver.failsOf(wires, everyWire()),
                    static_cast<std::uint32_t>(wires.size()));

    auto last = snapshotOf("", wires, ring, global.connections.size(), measure,
                           edges, couplersNow());
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

} // namespace mqt::scpd::pipeline
