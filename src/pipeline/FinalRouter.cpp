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
#include <limits>
#include <memory>
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
    const grid::PortBand band{
        .center = scene.router.clampToCell(port.center),
        .step = step,
        .forward = grid::bandLength((bridging[index] ? 2.0 : 1.0) *
                                        rules.min_straight_length,
                                    scene.router, step.diagonal()),
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
/// Two fields over the router grid. `owner` says which wire holds a cell, so
/// no two wires ever share one; `guard` counts how many wires keep their
/// clearance over a cell, so a search that has to hold the rule refuses any
/// cell with a count above zero. A wire charges its guard when it is put down
/// and gives it up when it is taken off, which makes "letting a wire go"
/// exactly "its room stops standing in the way, its copper stays".
///
/// The prototype instead cuts a disc around the two wires beside this one in
/// the ring, per search, and knows nothing about the other two hundred. That
/// is why its own pictures show wires touching. Cutting the disc for every
/// wire per search would cost the whole chip's copper per attempt; a field
/// charged once per move costs the rule against two hundred wires what the
/// rule against two cost.
class Field {
public:
  Field(const grid::GridMetrics& router, const std::uint32_t clearance)
      : stencil_(stencilOf(clearance)), width_(router.width),
        height_(router.height), guard_(router.cells(), 0),
        owner_(router.cells(), NO_OWNER),
        guardOwner_(router.cells(), NO_COMPONENT), stamp_(router.cells(), 0) {}

  [[nodiscard]] bool guarded(const std::size_t cell) const {
    return guard_[cell] != 0;
  }
  [[nodiscard]] std::uint16_t guardCount(const std::size_t cell) const {
    return guard_[cell];
  }
  [[nodiscard]] std::uint32_t owner(const std::size_t cell) const {
    return owner_[cell];
  }

  /// Whether every wire that guards a cell ends on one of these components.
  ///
  /// A cell guarded by wires of two different components, or by a wire that
  /// ends on no component at all, is never exempt. The mark is not restored
  /// when one of two components goes away again, so the answer is only ever
  /// too careful, never too generous.
  [[nodiscard]] bool
  guardedOnlyBy(const std::size_t cell,
                const std::array<std::uint32_t, 2>& components) const {
    const auto here = guardOwner_[cell];
    if (here == NO_COMPONENT || here == MIXED_COMPONENT) {
      return false;
    }
    return here == components[0] || here == components[1];
  }

  /// No component, and more than one.
  static constexpr std::uint32_t NO_COMPONENT = 0xFFFFFFFFU;
  static constexpr std::uint32_t MIXED_COMPONENT = 0xFFFFFFFEU;

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
  void charge(const Path& path, const std::uint32_t component) {
    walk(path, 1, component);
  }
  void discharge(const Path& path, const std::uint32_t component) {
    walk(path, -1, component);
  }

  /// The same for the places a wire cannot be moved off.
  void chargeFixed(const Path& fixed, const std::uint32_t component) {
    walk(fixed, 1, component);
  }
  void dischargeFixed(const Path& fixed, const std::uint32_t component) {
    walk(fixed, -1, component);
  }

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
  void walk(const Path& path, const int by, const std::uint32_t component) {
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
                x, y, by, component);
          lastX = x;
          lastY = y;
          continue;
        }
      }
      stamp(stencil_.full, x, y, by, component);
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
             const std::int64_t cy, const int by,
             const std::uint32_t component) {
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
      if (guard_[cell] == 0) {
        guardOwner_[cell] = NO_COMPONENT;
      } else if (by > 0) {
        guardOwner_[cell] = (guardOwner_[cell] == NO_COMPONENT ||
                             guardOwner_[cell] == component)
                                ? component
                                : MIXED_COMPONENT;
      }
    }
  }

  Stencil stencil_;
  std::int64_t width_;
  std::int64_t height_;
  std::vector<std::uint16_t> guard_;
  std::vector<std::uint32_t> owner_;
  std::vector<std::uint32_t> guardOwner_;
  std::vector<std::uint32_t> stamp_;
  std::uint32_t pass_ = 0;
};

// ----------------------------------------------------------------- The wires

/// One connection of the plan, and the way it has.
struct Wire {
  RoutingObjective objective;
  /// The way it has: the Detail stage's cells at first, its own after it has
  /// been routed. The corridor of every search is a band around this.
  Path way;
  /// Whether the way was drawn by this stage. A way that was not is a seed.
  bool drawn = false;
  /// Whether it has been routed in the pass running now.
  bool routed = false;
  /// Whether its room is charged into the field.
  bool placed = false;
  /// Whether only its fixed places are charged, which is what a wire that has
  /// been let go of keeps.
  bool endsOnly = false;
  /// The places it cannot be moved off: the straight run it leaves its source
  /// on, which its port's orientation fixes, and the cell of its target port.
  Path fixed;
  /// The cells where the rule does not bind, because two wires meet there.
  ///
  /// Where a terminal of this wire and a terminal of another sit within one
  /// wire spacing of each other, the two are one junction: they end at two
  /// ports of the same component, or a feedline lands on the coupler its
  /// resonator runs from. Their approaches converge there by construction and
  /// no arrangement can hold them apart, so the design rule exempts the
  /// neighbourhood of the junction and this stage does not enforce it either.
  /// The prototype's own clearance check makes the same exemption, with the
  /// same two figures: a link of one spacing and a radius of one and a half.
  std::vector<std::size_t> junction;
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
  /// The components its two ends sit on, as dense numbers.
  std::array<std::uint32_t, 2> components{Field::NO_COMPONENT,
                                          Field::NO_COMPONENT};
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

// -------------------------------------------------------------- The driver

/// The router grid of a run, and every search the stage makes on it.
///
/// One driver runs every routing phase. The prototype writes the loop out four
/// times — once for the inner circuit, once for the ring, once for the wires
/// under the feedline constraints and once for the refinement — and the four
/// differ only in their parameters.
class Driver {
public:
  Driver(const Scene& scene, const Tuning& tuning, Progress progress = {})
      : scene_(scene), tuning_(tuning), progress_(std::move(progress)),
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

  /// What the grid and the rules came to, once, before anything is drawn.
  void sayTheSetting() const {
    say(std::format(
        "grid {}x{} cells of {:.2f} layout units | clearance {} cells | "
        "stub {} cells | band {} cells | bend {} | wire price {} | "
        "obstacle price {} over {} cells",
        scene_.router.width, scene_.router.height,
        std::min(scene_.router.cellWidth, scene_.router.cellHeight),
        tuning_.clearance, tuning_.straightStart, tuning_.reach,
        tuning_.bendPenalty, tuning_.wireProximityPenalty,
        tuning_.staticProximityPenalty, tuning_.obstacleReach));
  }

  /// Put a wire's way down: its copper, and the room it keeps around it.
  void place(Wire& wire) {
    if (wire.way.empty() || !wire.drawn) {
      return;
    }
    if (wire.endsOnly) {
      field_.dischargeFixed(wire.fixed, wire.components[1]);
      wire.endsOnly = false;
    }
    if (!wire.placed) {
      field_.occupy(wire.way, wire.key);
      field_.charge(wire.way, wire.components[1]);
      wire.placed = true;
    }
  }

  /// Take a wire's room away and leave its copper. This is what the prototype
  /// means by ripping a wire: the wire is marked as one still to be drawn and
  /// its clearance stops standing in the way, but nothing is drawn over it, so
  /// no two wires ever share a cell while the sweep is running.
  ///
  /// The two places it cannot be moved off keep their room, because a wire
  /// drawn beside a place another wire has to come back to is a violation no
  /// later round can undo: neither of the two can move the place.
  void letGo(Wire& wire) {
    if (!wire.placed) {
      return;
    }
    field_.discharge(wire.way, wire.components[1]);
    field_.chargeFixed(wire.fixed, wire.components[1]);
    wire.placed = false;
    wire.endsOnly = true;
  }

  /// Take a wire off the canvas altogether, for the length of its own search.
  /// Only the wire being drawn is let out of its own two places.
  void lift(Wire& wire) {
    if (wire.placed) {
      field_.discharge(wire.way, wire.components[1]);
      wire.placed = false;
    } else if (wire.endsOnly) {
      field_.dischargeFixed(wire.fixed, wire.components[1]);
    }
    wire.endsOnly = false;
    if (wire.drawn) {
      field_.vacate(wire.way, wire.key);
    }
  }

  /// Sweep the wires, drawing the ones that have no way of their own yet.
  ///
  /// Rounds alternate direction. A wire that has been routed is left alone; a
  /// wire that finds nothing keeps the way it had, which is always legal
  /// because it was legal when it was put down.
  ///
  /// A round ends when every wire has been routed, not when no attempt
  /// failed. The two are not the same: a wire routed early in a round can be
  /// let go of by a wire further along, and letting it go is a promise that it
  /// will be drawn again. Stopping on the failure count alone leaves it on the
  /// canvas with its room uncharged, and the next wire that comes past settles
  /// inside it.
  ///
  /// @returns How many wires were left without a way of their own.
  std::uint32_t sweep(std::vector<Wire>& wires,
                      const std::vector<std::uint32_t>& members,
                      const Pass& pass) {
    const auto total = static_cast<std::uint32_t>(members.size());
    if (total == 0) {
      return 0;
    }
    fixPlaces(wires, members, pass);
    auto fewest = std::numeric_limits<std::uint32_t>::max();
    std::uint32_t stale = 0;
    say(std::format("{}: {} wires, up to {} rounds, {} relaxations each way",
                    pass.name, total, pass.rounds, pass.maxRelaxation));
    for (std::uint32_t round = 0; round < pass.rounds; ++round) {
      const bool forward = (round % 2) == 0;
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
      const auto open = static_cast<std::uint32_t>(
          std::ranges::count_if(members, [&wires](const std::uint32_t member) {
            const auto& wire = wires[member];
            return wire.feasible && !wire.routed;
          }));
      std::uint32_t blank = 0;
      for (const auto member : members) {
        blank += (wires[member].feasible && !wires[member].drawn) ? 1 : 0;
      }
      say(std::format("{} round {} {}: tried {}, routed {}, still open {}, "
                      "undrawn {}",
                      pass.name, round, forward ? "forward " : "backward",
                      tried, won, open, blank));
      if (open == 0) {
        break;
      }
      // A round that leaves as many wires open as the best round before it
      // has moved nothing that the next round can use. The rounds are there
      // for a wire to take room a later one has not claimed yet, and once
      // that has stopped happening they only cost searches: measured on the
      // 21-qubit chip, the count settles by the second round and the
      // remaining twenty-eight change no byte of the result.
      stale = open < fewest ? 0 : stale + 1;
      fewest = std::min(fewest, open);
      if (stale >= STALE_ROUNDS) {
        say(std::format("{}: {} rounds without progress, stopping", pass.name,
                        stale));
        break;
      }
    }

    // What the rounds ran out on: a wire with no way at all is offered one
    // last time, and takes whatever the free copper allows even where that
    // breaks the rule. A connection that is not drawn cannot be repaired
    // later; a connection drawn too close to another is a finding the check
    // reports and a person can look at.
    for (std::uint32_t at = 0; at < total; ++at) {
      auto& wire = wires[members[at]];
      if (wire.feasible && !wire.drawn) {
        say(std::format("{}: wire {} has no way; taking one without the rule",
                        pass.name, wire.slot));
        static_cast<void>(attempt(wires, members, at, true, pass, true));
      }
    }

    // Everything goes back down, so that the field says what the canvas holds
    // and the next phase starts from the truth.
    std::uint32_t undrawn = 0;
    std::uint32_t crowded = 0;
    for (const auto member : members) {
      auto& wire = wires[member];
      place(wire);
      undrawn += wire.drawn ? 0 : 1;
    }
    // Counted with every wire down, so what it says is what the design-rule
    // check will find and not what the sweep happened to leave open.
    for (const auto member : members) {
      auto& wire = wires[member];
      if (!wire.drawn) {
        continue;
      }
      lift(wire);
      crowded += conflictsOf(wire, wires) == 0 ? 0 : 1;
      place(wire);
    }
    say(std::format("{}: {} of {} drawn, {} of them too close to another wire",
                    pass.name, total - undrawn, total, crowded));
    return undrawn;
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
      std::uint32_t moved = 0;
      for (std::uint32_t at = 0; at < total; ++at) {
        const auto slot = forward ? at : (total - 1 - at);
        auto& wire = wires[members[slot]];
        if (!wire.drawn || !wire.feasible) {
          continue;
        }
        const auto before = wire.way;
        lift(wire);
        buildCorridor(wire, pass.reach);
        priceRoom();
        const auto found = search(wire, pass.straightStart, true);
        // A wider way is only wider if it is still legal. The search keeps the
        // rule against every wire that is down, but the stubs the router adds
        // to a way after it has searched are not part of what it searched, so
        // the result is judged like any other before it is kept.
        const auto keep = !found.empty() && conflictsIn(found, wire, wires) <=
                                                conflictsIn(before, wire, wires);
        wire.way = keep ? found : before;
        moved += keep ? 1 : 0;
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
                      pass.name, round, forward ? "forward " : "backward", moved,
                      total, crowded));
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
        field_.chargeFixed(wire.fixed, wire.components[1]);
        wire.endsOnly = true;
      }
    }
    markJunctions(wires, members);
  }

  /// The neighbourhood of every junction, per wire.
  ///
  /// A terminal of one wire within one wire spacing of a terminal of another
  /// is a junction. Everything within one and a half spacings of the middle
  /// of the two belongs to it. The zone is where the search may enter a cell
  /// another wire guards; it may still not enter a cell another wire holds,
  /// so two wires never share copper even here.
  void markJunctions(std::vector<Wire>& wires,
                     const std::vector<std::uint32_t>& members) {
    struct Terminal {
      std::uint32_t key = 0;
      std::array<std::uint32_t, 2> components{Field::NO_COMPONENT,
                                              Field::NO_COMPONENT};
      std::int64_t x = 0;
      std::int64_t y = 0;
    };
    std::vector<Terminal> terminals;
    for (const auto& wire : wires) {
      if (!wire.feasible) {
        continue;
      }
      terminals.push_back({wire.key, wire.components, wire.objective.source.x,
                           wire.objective.source.y});
      terminals.push_back({wire.key, wire.components, wire.objective.target.x,
                           wire.objective.target.y});
    }
    const auto link = static_cast<std::int64_t>(tuning_.clearance) *
                      static_cast<std::int64_t>(tuning_.clearance);
    const auto reach = (3 * tuning_.clearance) / 2;
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto height = static_cast<std::int64_t>(scene_.router.height);
    for (const auto member : members) {
      auto& wire = wires[member];
      if (!wire.feasible || !wire.junction.empty()) {
        continue;
      }
      for (const auto& end : {wire.objective.source, wire.objective.target}) {
        for (const auto& other : terminals) {
          if (other.key == wire.key) {
            continue;
          }
          const auto dx = static_cast<std::int64_t>(end.x) - other.x;
          const auto dy = static_cast<std::int64_t>(end.y) - other.y;
          const bool close = ((dx * dx) + (dy * dy)) <= link;
          const bool shared = wire.components[1] != Field::NO_COMPONENT &&
                              (wire.components[1] == other.components[0] ||
                               wire.components[1] == other.components[1]);
          if (!close && !shared) {
            continue;
          }
          const std::array<std::pair<std::int64_t, std::int64_t>, 2> centres{
              {{static_cast<std::int64_t>(end.x),
                static_cast<std::int64_t>(end.y)},
               {other.x, other.y}}};
          for (const auto& [cx, cy] : centres) {
            for (const auto& [ox, oy] : stencilFor(reach).full) {
              const auto x = cx + ox;
              const auto y = cy + oy;
              if (x >= 0 && y >= 0 && x < width && y < height) {
                wire.junction.push_back(
                    static_cast<std::size_t>((y * width) + x));
              }
            }
          }
        }
      }
      std::ranges::sort(wire.junction);
      const auto duplicates = std::ranges::unique(wire.junction);
      wire.junction.erase(duplicates.begin(), duplicates.end());
    }
  }

  /// One wire's turn: the band around its own way first, then the relaxation.
  ///
  /// Whatever was let go of stands in the way again before the wire is put
  /// down, so what it is put down on is judged against every wire on the chip
  /// and not against the field the search was given. A wire counts as routed
  /// only when the way it found holds the rule against all of them; a way that
  /// only exists because a neighbour was lifted leaves the wire open, and the
  /// next round tries it again. Without that a wire keeps a way it took from a
  /// neighbour, the neighbour cannot get it back, and no later round undoes it.
  bool attempt(std::vector<Wire>& wires,
               const std::vector<std::uint32_t>& members,
               const std::uint32_t slot, const bool forward, const Pass& pass,
               const bool rescue = false) {
    const auto total = static_cast<std::uint32_t>(members.size());
    auto& wire = wires[members[slot]];
    const auto had = wire.way;
    const auto hadDrawn = wire.drawn;

    // The wire is lifted off the canvas for the length of its own search, its
    // own fixed places included: it has to be able to reach them.
    lift(wire);

    // The band around the way it has, and nothing priced.
    buildCorridor(wire, pass.reach);
    std::ranges::fill(proximity_, 0);
    auto found = search(wire, pass.straightStart, false);

    // The relaxation. Each level lets go of one more of the wires beside it
    // along the sweep, so that a wire which found no way through can take the
    // room a later one has not claimed yet, and prices the way outside the
    // lane between its two ring neighbours rather than forbidding it.
    //
    // It runs along the sweep first and then against it. The prototype only
    // ever goes one way, and a wire whose way is blocked by the wire behind it
    // then has no move at all.
    std::vector<std::uint32_t> released;
    const auto relax = [&](const bool along) {
      for (std::uint32_t level = 1; level <= pass.maxRelaxation; ++level) {
        const auto step = (forward == along)
                              ? (slot + level) % total
                              : (slot + total - (level % total)) % total;
        if (step == slot) {
          continue;
        }
        auto& neighbour = wires[members[step]];
        released.push_back(neighbour.key);
        letGo(neighbour);

        buildCorridor(wire, pass.reach);
        priceLane(wires, members, slot, forward == along, level);
        found = search(wire, pass.straightStart, true);
        if (!found.empty()) {
          return;
        }
      }
    };
    if (found.empty() && total > 1) {
      relax(true);
    }
    if (found.empty() && total > 1) {
      relax(false);
    }

    // The last resort: the wires actually in the way. The sweep names its
    // neighbours by their place in the ring, and the wire that stands in the
    // way of this one need not be a neighbour at all. So the wire is routed
    // once with the room of the others ignored — their copper is still solid,
    // so what comes back is a way that exists — and every wire whose room lies
    // over that way is let go of.
    Path greedy;
    if (found.empty()) {
      buildCorridor(wire, pass.reach, false);
      priceGuarded();
      greedy = search(wire, pass.straightStart, true);
      if (!greedy.empty()) {
        for (const auto blocker : blockersOf(greedy, wire.key)) {
          released.push_back(blocker);
          letGo(wires[blocker]);
        }
        if (!released.empty()) {
          buildCorridor(wire, pass.reach);
          std::ranges::fill(proximity_, 0);
          found = search(wire, pass.straightStart, false);
        }
      }
    }

    // A wire that has never been drawn takes the way it found without the
    // rule rather than none at all — but only once the rounds are over. It is
    // a way in every other sense: it runs over no obstacle and shares no cell
    // with another wire, so what it costs is a clearance the check then
    // reports, against a connection that would otherwise be missing
    // altogether. Doing it earlier costs more than it buys: in the first round
    // no wire is drawn yet, so every wire that finds nothing would settle for
    // a way that ignores the rule, and the rounds that follow cannot take it
    // back.
    if (found.empty() && rescue && !hadDrawn && !greedy.empty()) {
      found = greedy;
    }

    // Everything that was let go of stands in the way again. A wire that was
    // lifted for this search is one the next round has to look at, because the
    // way it has was drawn when this one was not there.
    for (const auto key : released) {
      auto& neighbour = wires[key];
      place(neighbour);
      neighbour.routed = false;
    }

    // The verdict, taken while the wire is still off the canvas: how many
    // cells of a way lie in the room of another wire. Its own room is not
    // charged here, which is the whole reason for counting before putting it
    // down — a wire is always within the rule of itself.
    //
    // A way found while a neighbour was lifted is not automatically better
    // than the way the wire already had: the neighbour comes back. So the two
    // are compared, and the new one is taken when it is not worse. Not worse
    // rather than better: the corridor stage measured that a strict test
    // blocks the lateral moves the next round needs.
    std::uint32_t conflicts = 0;
    if (!found.empty()) {
      const auto now = conflictsIn(found, wire, wires);
      // A wire with no way yet is not thereby entitled to a way that breaks
      // the rule. Its baseline is none: a way that conflicts is worse, and it
      // stays undrawn so that the rounds can try again with a fuller picture
      // and the rescue can draw it at the end, where the price keeps it as far
      // from its neighbours as it can get. Taking a conflicting way here is
      // what puts three consecutive wires one cell apart: each of them routes
      // through the room of the one before it in the first round, and no later
      // round can undo three at once.
      const auto before = hadDrawn ? conflictsIn(had, wire, wires) : 0U;
      if (rescue || now <= before) {
        wire.way = found;
        wire.drawn = true;
        conflicts = now;
      } else {
        conflicts = before;
        found.clear();
      }
    } else if (hadDrawn) {
      conflicts = conflictsIn(had, wire, wires);
    }
    place(wire);
    if (conflicts != 0) {
      // The wires it is too close to have to move as well, so both sides of
      // the encounter are open when the next round comes past.
      for (const auto blocker : blockersOf(wire.way, wire.key)) {
        wires[blocker].routed = false;
      }
    }
    // Settled means the way it has holds the rule against every other wire —
    // not that this attempt found a new one. A wire whose search fails and
    // whose way was legal all along is finished; re-trying it every round
    // costs a search and moves nothing.
    wire.routed = wire.drawn && conflicts == 0;
    return wire.routed;
  }

  /// How many cells of a way lie in the room of another wire.
  ///
  /// The wire itself has to be off the canvas when this runs. A cell inside a
  /// junction this wire shares is not counted, because the rule does not bind
  /// there and the design-rule check makes the same exemption.
  [[nodiscard]] std::uint32_t conflictsOf(const Wire& wire,
                                          const std::vector<Wire>& wires) {
    return conflictsIn(wire.way, wire, wires);
  }

  /// Whether two wires meet at a place, so that the rule does not bind there.
  ///
  /// They meet when a terminal of one sits within the rule of a terminal of
  /// the other, or when the two end on one component — the ports of a
  /// component sit closer together than the wire spacing and each has to be
  /// reached, so the approaches converge. What belongs to the meeting is
  /// everything within one and a half rules of either terminal. This is the
  /// design-rule check's own test.
  [[nodiscard]] bool meetAt(const Wire& one, const Wire& two, const double x,
                            const double y) const {
    const auto shares = one.components[1] != Field::NO_COMPONENT &&
                        (one.components[1] == two.components[0] ||
                         one.components[1] == two.components[1]);
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
        if (!shares && apart > link) {
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
      if (field_.guardedOnlyBy(cell, wire.components) &&
          std::ranges::binary_search(wire.junction, cell)) {
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

  /// The wires whose room lies over a way, nearest first.
  ///
  /// A cell of the way is looked at through the same disc the clearance is
  /// charged through, and every wire that holds copper inside it is named. The
  /// scan is a failure path and costs the disc per cell of one way, which is
  /// nothing against the searches it saves.
  [[nodiscard]] std::vector<std::uint32_t>
  blockersOf(const Path& way, const std::uint32_t self) {
    const auto& stencil = stencilFor(tuning_.clearance);
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto height = static_cast<std::int64_t>(scene_.router.height);
    std::vector<std::uint32_t> blockers;
    for (const auto& point : way) {
      for (const auto& [dx, dy] : stencil.full) {
        const auto x = static_cast<std::int64_t>(point.x) + dx;
        const auto y = static_cast<std::int64_t>(point.y) + dy;
        if (x < 0 || y < 0 || x >= width || y >= height) {
          continue;
        }
        const auto owner =
            field_.owner(static_cast<std::size_t>((y * width) + x));
        if (owner != NO_OWNER && owner != self) {
          blockers.push_back(owner);
        }
      }
    }
    std::ranges::sort(blockers);
    const auto duplicates = std::ranges::unique(blockers);
    blockers.erase(duplicates.begin(), duplicates.end());
    return blockers;
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
    return router_.route(wire.objective, usePenalty);
  }

  /// The cells a search may enter: the free space within `reach` steps of the
  /// way the wire has, less every cell another wire holds or guards.
  ///
  /// The walk is the prototype's `expand_path`, which spreads over free cells
  /// only and therefore does not reach around an obstacle. What it produces is
  /// a band that follows the way rather than a box around it.
  void buildCorridor(const Wire& wire, const std::uint32_t reach,
                     const bool holdClearance = true) {
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
      const bool free = holdClearance ? !field_.guarded(cell)
                                      : field_.owner(cell) == NO_OWNER;
      if (free &&
          (field_.owner(cell) == NO_OWNER || field_.owner(cell) == wire.key)) {
        corridor_.set(cell, false);
      }
    };

    for (const auto& point : wire.way) {
      visit(point.x, point.y, 0);
    }
    // A wire's own fixed places are always enterable. They are where the wire
    // has to be, and the search is the one pass that may stand on them.
    for (const auto& point : wire.fixed) {
      if (point.x < scene_.router.width && point.y < scene_.router.height) {
        corridor_.setCell(point.x, point.y, false);
      }
    }
    // Inside a junction the rule does not bind against the wire it is shared
    // with. Two conditions have to hold for a guarded cell to open: the cell
    // belongs to a junction of this wire, and every wire guarding it ends on
    // a component this wire also ends on. A cell another wire holds stays
    // shut either way — the exemption is of the clearance, never of the
    // copper.
    for (const auto cell : wire.junction) {
      if (scene_.blocked.test(cell) ||
          (field_.owner(cell) != NO_OWNER && field_.owner(cell) != wire.key)) {
        continue;
      }
      if (!field_.guarded(cell) ||
          field_.guardedOnlyBy(cell, wire.components)) {
        corridor_.set(cell, false);
      }
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

  /// Price a disc of a radius around every cell of a way, by the leading edge
  /// of the disc as the clearance field charges its own.
  void stampDisc(const Path& way, const std::uint32_t radius,
                 const std::uint8_t price) {
    if (way.empty() || radius == 0 || price == 0) {
      return;
    }
    const auto& stencil = stencilFor(radius);
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    const auto height = static_cast<std::int64_t>(scene_.router.height);
    const auto stamp = [&](const Stencil::Offsets& offsets,
                           const std::int64_t cx, const std::int64_t cy) {
      for (const auto& [dx, dy] : offsets) {
        const auto x = cx + dx;
        const auto y = cy + dy;
        if (x >= 0 && y >= 0 && x < width && y < height) {
          proximity_[static_cast<std::size_t>((y * width) + x)] = price;
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

  [[nodiscard]] const Stencil& stencilFor(const std::uint32_t radius) {
    const auto found = stencils_.find(radius);
    if (found != stencils_.end()) {
      return found->second;
    }
    return stencils_.emplace(radius, stencilOf(radius)).first->second;
  }

  /// A price on every cell another wire guards.
  ///
  /// The search that ignores the rule uses it, so that the way it finds keeps
  /// what room there is even where it cannot keep the rule. Forbidding those
  /// cells is what failed; pricing them is what is left.
  ///
  /// The price is the highest one the router takes, not the one the relaxation
  /// steers with. A wire spacing is worth more than any number of bends: with
  /// the ordinary price of four the search hugs the wire beside it to save one
  /// corner, and what comes out is a bundle of ways one cell apart.
  void priceGuarded() {
    std::ranges::fill(proximity_, 0);
    constexpr std::uint8_t price = 127;
    if (box_.empty()) {
      return;
    }
    const auto width = static_cast<std::int64_t>(scene_.router.width);
    for (std::int64_t y = box_.minY; y <= box_.maxY; ++y) {
      for (std::int64_t x = box_.minX; x <= box_.maxX; ++x) {
        const auto cell = static_cast<std::size_t>((y * width) + x);
        if (field_.guarded(cell)) {
          proximity_[cell] = price;
        }
      }
    }
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

  const Scene& scene_;
  Tuning tuning_;
  Progress progress_;
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

/// The components of a chip as dense numbers, so that "these two ports sit on
/// one component" is a comparison of two integers rather than of two strings.
/// A port with no component keeps `Field::NO_COMPONENT`.
[[nodiscard]] std::vector<std::uint32_t> componentsOf(const ChipT& chip) {
  std::unordered_map<std::string, std::uint32_t> numbers;
  std::vector<std::uint32_t> of(chip.ports.size(), Field::NO_COMPONENT);
  for (std::size_t index = 0; index < chip.ports.size(); ++index) {
    const auto& name = chip.ports[index]->component;
    if (name.empty()) {
      continue;
    }
    const auto found =
        numbers.emplace(name, static_cast<std::uint32_t>(numbers.size()));
    of[index] = found.first->second;
  }
  return of;
}

/// Every wire the stage has to draw: the ring first, in the order the
/// assignment holds it, and the inner circuit after it.
[[nodiscard]] std::vector<Wire> wiresOf(const ChipT& chip,
                                        const GlobalRoutingT& global,
                                        const AssignmentT& assignment,
                                        const DetailRoutingT& detail,
                                        const Scene& scene) {
  const auto components = componentsOf(chip);
  const auto componentAt = [&components](const std::uint32_t port) {
    return port < components.size() ? components[port] : Field::NO_COMPONENT;
  };
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
    wire.components = {Field::NO_COMPONENT, componentAt(wire.targetPort)};
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
    wire.components = {componentAt(wire.sourcePort),
                       componentAt(wire.targetPort)};
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
      const Progress& progress) const override {
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

    Driver driver(field, tuning, progress);

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
