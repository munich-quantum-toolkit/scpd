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
#include "mqt-scpd/routing/DubinsRouter.hpp"
#include "mqt-scpd/routing/Heading.hpp"
#include "mqt-scpd/routing/Path.hpp"
#include "mqt-scpd/routing/PathGeometry.hpp"
#include "mqt-scpd/routing/Primitives.hpp"
#include "mqt-scpd/routing/SearchScratch.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <memory>
#include <numeric>
#include <ranges>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
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
  /// How long a resonator's way is made before the coupler is spliced in, in
  /// cells. Zero switches the meander off.
  double meanderLength = 0.0;
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
  tuning.meanderLength =
      params.meander_length <= 0.0
          ? 0.0
          : params.meander_length /
                std::min(router.cellWidth, router.cellHeight);
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
  /// The cell each routable port is reached at, by port index.
  std::unordered_map<std::uint32_t, std::size_t> targetCell;
  /// The heading a wire has when it arrives at each port, by port index.
  std::unordered_map<std::uint32_t, Heading> arrival;
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
  blockOutsideTheSources(chip, scene);

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
        proximity_(scene.router.cells(), 0), seen_(scene.router.cells(), 0) {
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

  /// The picture of the last search, for the line about it, when there is one.
  [[nodiscard]] std::string picture() const {
    return lastPicture_.empty() ? std::string{} : " · " + lastPicture_;
  }

  /// What the grid and the rules came to, once, before anything is drawn.
  void sayTheSetting() const {
    say(std::format(
        "grid {}x{} cells of {:.2f} layout units | clearance {} cells for a "
        "rule of {:.2f} | stub {} cells | band {} cells | bend {} | wire "
        "price {} | obstacle price {} over {} cells",
        scene_.router.width, scene_.router.height,
        std::min(scene_.router.cellWidth, scene_.router.cellHeight),
        tuning_.clearance, tuning_.spacing, tuning_.straightStart,
        tuning_.reach, tuning_.bendPenalty, tuning_.wireProximityPenalty,
        tuning_.staticProximityPenalty, tuning_.obstacleReach));
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
  /// wires still on their seed, for which this stage has found no way, and
  /// the wires with a way of their own that is not settled against every
  /// other wire. The first are unrouted, the second open, and both count.
  ///
  /// @returns How many wires are unrouted or open when the pass ends.
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
      // not drawn again yet. Both are fails, and a round with none ends the
      // pass.
      std::uint32_t unrouted = 0;
      std::uint32_t open = 0;
      for (const auto member : members) {
        const auto& wire = wires[member];
        if (!wire.feasible) {
          continue;
        }
        unrouted += wire.drawn ? 0 : 1;
        open += (wire.drawn && !wire.routed) ? 1 : 0;
      }
      const auto fails = unrouted + open;
      say(std::format("{} round {} {}: tried {}, routed {}, unrouted {}, "
                      "open {} | Fails: {}",
                      pass.name, round, forward ? "forward " : "backward",
                      tried, won, unrouted, open, fails));
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
        say(std::format("{} round {} {}: {} found a way in phase 1, {} after "
                        "relaxation, {} failed{}",
                        pass.name, round,
                        record.forward ? "forward " : "backward", record.normal,
                        record.relaxed, record.failed.size(),
                        record.failed.empty() ? "" : ":" + failed));
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
    /// Which ones, for the second level of verbosity.
    std::vector<std::string> unroutedIds;
    std::vector<std::string> openIds;
    [[nodiscard]] std::uint32_t total() const { return unrouted + open; }
  };

  /// Count the fails of a set of wires, by the same test the design-rule
  /// check makes and with every wire down — so what it says is what the
  /// check will find, not what the sweep happened to leave open. Each wire
  /// is counted with itself lifted off the canvas, because a wire is always
  /// within the rule of itself.
  [[nodiscard]] Fails failsOf(std::vector<Wire>& wires,
                              const std::vector<std::uint32_t>& members) {
    Fails fails;
    for (const auto member : members) {
      auto& wire = wires[member];
      if (!wire.drawn) {
        ++fails.unrouted;
        fails.unroutedIds.push_back(wireId(wire));
        continue;
      }
      ++fails.drawn;
      lift(wire);
      if (conflictsOf(wire, wires) != 0) {
        ++fails.open;
        fails.openIds.push_back(wireId(wire));
      }
      place(wire);
    }
    return fails;
  }

  /// The summary line of a pass, or of the stage.
  void sayFails(const std::string& name, const Fails& fails,
                const std::uint32_t total) const {
    say(std::format("{}: {} of {} drawn, {} unrouted, {} open | Fails: {}",
                    name, fails.drawn, total, fails.unrouted, fails.open,
                    fails.total()));
    if (verbosity_ >= 1 && fails.total() != 0) {
      const auto join = [](const std::vector<std::string>& ids) {
        std::string joined;
        for (const auto& id : ids) {
          joined += (joined.empty() ? "" : ", ") + id;
        }
        return joined.empty() ? std::string("-") : joined;
      };
      say(std::format("{}: unrouted: {} · open: {}", name,
                      join(fails.unroutedIds), join(fails.openIds)));
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
        frame_.wires = &wires;
        frame_.before = before.key;
        frame_.after = after.key;
        frame_.kind = "refine";
        frame_.fence = {before.key, after.key};
        frame_.ripped.clear();
        const auto found = search(wire, pass.straightStart, true);
        tell(std::format("wire {} · round {} {} · refine: {}{}", wireId(wire),
                         round, forward ? "forward" : "backward",
                         found.empty()
                             ? std::string("kept its way")
                             : std::format("found {} cells", found.size()),
                         picture()));
        // The prototype's refinement takes what it finds, and falls back to
        // the way it had when the wider constraint does not route.
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
      const auto stub = wire.resonator ? 0U : pass.straightStart;
      wire.fixed = router_.straightStub(wire.objective.source, false, stub);
      wire.fixed.push_back(wire.objective.target);
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
  /// The rule against every other wire is not what the search keeps. It is
  /// what the fails are counted by when the round is over.
  bool attempt(std::vector<Wire>& wires,
               const std::vector<std::uint32_t>& members,
               const std::uint32_t slot, const bool forward, const Pass& pass) {
    const auto total = static_cast<std::uint32_t>(members.size());
    auto& wire = wires[members[slot]];
    const auto& before = wires[members[(slot + total - 1) % total]];
    const auto& after = wires[members[(slot + 1) % total]];

    // The wire is lifted off the canvas for the length of its own search, so
    // that what is put down afterwards is the one way it has.
    lift(wire);

    // Phase 1: the band around the way it has, fenced by its two neighbours,
    // and nothing priced.
    buildCorridor(wire, pass.reach);
    fence(wire, {&before, &after});
    std::ranges::fill(proximity_, 0);
    frame_.wires = &wires;
    frame_.before = before.key;
    frame_.after = after.key;
    frame_.kind = "normal";
    frame_.fence = {before.key, after.key};
    frame_.ripped.clear();
    auto found = search(wire, pass.straightStart, true);
    const auto id = wireId(wire);
    const auto where = std::format("round {} {}", frame_.round,
                                   forward ? "forward" : "backward");
    RoundRecord* record = rounds_.empty() ? nullptr : &rounds_.back();
    if (!found.empty()) {
      tell(std::format("wire {} · {} · normal: found {} cells{}", id, where,
                       found.size(), picture()));
      if (record != nullptr) {
        ++record->normal;
      }
    } else {
      tell(std::format("wire {} · {} · normal: no way, relaxing{}", id, where,
                       picture()));
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
      frame_.kind = std::format("relax {}", level);
      frame_.fence = {ripped.key, (forward ? before : after).key};
      frame_.ripped.push_back(ripped.key);
      found = search(wire, pass.straightStart, true);
      tell(std::format(
          "wire {} · relax {}: let go of {}, fence {} and {} · {}{}", id, level,
          wireId(ripped), wireId(ripped), wireId(forward ? before : after),
          found.empty() ? std::string("no way")
                        : std::format("found {} cells", found.size()),
          picture()));
    }

    if (found.empty()) {
      // The rollback: every wire let go of is exactly what it was.
      std::string names;
      for (const auto& [key, routed] : released) {
        wires[key].routed = routed;
        names += (names.empty() ? "" : ", ") + wireId(wires[key]);
      }
      place(wire);
      wire.routed = false;
      tell(std::format("wire {} · {}: no way after {} relaxations; {} back as "
                       "they were",
                       id, where, released.size(),
                       names.empty() ? std::string("nothing") : names));
      if (record != nullptr) {
        record->failed.push_back(id);
      }
      return false;
    }
    if (record != nullptr && !released.empty()) {
      ++record->relaxed;
    }
    wire.way = found;
    wire.drawn = true;
    wire.routed = true;
    place(wire);
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

  /// The search itself, with the stubs of this pass.
  [[nodiscard]] Path search(const Wire& wire, const std::uint32_t straightStart,
                            const bool usePenalty) {
    router_.setParams(
        {.startStraightLength = wire.resonator ? 0U : straightStart,
         .endStraightLength = 0,
         .minRadius = BEND_RADIUS,
         .bendPenalty = tuning_.bendPenalty});
    router_.attachCorridor(&corridor_);
    lastPicture_.clear();
    auto found = router_.route(wire.objective, usePenalty);
    if (debug_) {
      drawSearch(wire, found);
    }
    return found;
  }

  /// The cells a search may enter: the free space within `reach` steps of the
  /// way the wire has.
  ///
  /// This is the prototype's `expand_path`: a walk over the cells that are not
  /// artwork, so the band follows the way rather than boxing it and does not
  /// reach around an obstacle. It knows nothing of other wires. What fences a
  /// search in is `fence`, and nothing else.
  void buildCorridor(const Wire& wire, const std::uint32_t reach) {
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
      if (seen_[cell] == pass_ || scene_.blocked.test(cell)) {
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
  /// pays, the wires around it, and what it found. Taken from inside
  /// `search`, so it shows exactly what the router was given.
  void drawSearch(const Wire& wire, const Path& found) {
    if (box_.empty() || frame_.wires == nullptr) {
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
        {std::format("{} · round {} {} · wire {} · {} · {}", frame_.pass,
                     frame_.round, frame_.forward ? "forward" : "backward",
                     wireId(wire), frame_.kind, result),
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
                 "position, dashed = distance to the target beyond its band"}});

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

[[nodiscard]] std::unique_ptr<fba::FinalWireT> wireOf(const Path& way) {
  auto drawn = std::make_unique<fba::FinalWireT>();
  drawn->path.reserve(way.size());
  for (const auto& point : way) {
    drawn->path.emplace_back(point.x, point.y, point.heading);
  }
  return drawn;
}

/// The state of every wire, as one phase left it.
[[nodiscard]] std::unique_ptr<fba::FinalPhaseT>
snapshotOf(std::string name, const std::vector<Wire>& wires,
           const std::uint32_t ring, const std::size_t inner) {
  auto phase = std::make_unique<fba::FinalPhaseT>();
  phase->name = std::move(name);
  phase->wires.reserve(ring);
  for (std::uint32_t at = 0; at < ring; ++at) {
    phase->wires.push_back(wireOf({}));
  }
  phase->inner.reserve(inner);
  for (std::size_t at = 0; at < inner; ++at) {
    phase->inner.push_back(wireOf({}));
  }
  for (const auto& wire : wires) {
    if (!wire.drawn) {
      continue;
    }
    auto& into = wire.inner ? phase->inner : phase->wires;
    if (wire.slot < into.size()) {
      into[wire.slot] = wireOf(wire.way);
    }
  }
  return phase;
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

    FinalRoutingT routing;
    auto extent = std::make_unique<fba::GridExtentT>();
    extent->width = field.router.width;
    extent->height = field.router.height;
    extent->origin = field.router.origin;
    extent->cell_width = field.router.cellWidth;
    extent->cell_height = field.router.cellHeight;
    routing.grid = std::move(extent);

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
        snapshotOf("inner", wires, ring, global.connections.size()));

    // Phase 2. The ring, against the inner circuit and against itself.
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
        snapshotOf("outer", wires, ring, global.connections.size()));

    // The three phases that follow are not implemented yet. Their snapshots
    // are written empty rather than left out, so that a reader and a renderer
    // see the same five phases whatever a build carries.
    for (const auto* const name : {"couplers", "feedlines", "refined"}) {
      routing.phases.push_back(
          snapshotOf(name, wires, ring, global.connections.size()));
    }

    // What the stage came to, over every wire of the plan and counted with
    // every wire down, so that the last line of a run says what the
    // design-rule check will find.
    std::vector<std::uint32_t> every(wires.size());
    std::iota(every.begin(), every.end(), 0U);
    driver.sayFails("final routing", driver.failsOf(wires, every),
                    static_cast<std::uint32_t>(wires.size()));

    auto last = snapshotOf("", wires, ring, global.connections.size());
    routing.wires = std::move(last->wires);
    routing.inner = std::move(last->inner);
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
