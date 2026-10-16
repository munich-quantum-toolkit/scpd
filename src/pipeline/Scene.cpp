/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/design/Bridges.hpp"
#include "mqt-scpd/design/Roles.hpp"
#include "mqt-scpd/flatbuffers/config.hpp"
#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/geometry/Geometry.hpp"
#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"
#include "mqt-scpd/grid/Partitions.hpp"
#include "mqt-scpd/grid/PortBands.hpp"
#include "mqt-scpd/grid/Rasterize.hpp"
#include "mqt-scpd/pipeline/CapacityPlanner.hpp"
#include "mqt-scpd/pipeline/Stages.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace mqt::scpd::pipeline {
namespace {

using flatbuffers::design::UnassignedRole;

/// How far a port band reaches back against the direction a wire leaves the
/// port, as a multiple of the forward reach.
///
/// The band exists so that no other wire runs through the approach of a port.
/// It reaches further backward than forward because backward is into the
/// artwork of the component, where no wire belongs. One factor holds for every
/// port: a longer band would only cover artwork that the obstacle raster
/// blocks already.
constexpr double BACKWARD_BAND_FACTOR = 3.0;

/// How much further forward the band of a bridge port reaches than that of an
/// ordinary port.
///
/// A wire does not stop at a bridge port: it crosses the component and leaves
/// through the paired port. Its approach therefore has to clear the artwork of
/// the component, which lies one component width ahead, instead of merely
/// leaving the port. With a single reach the target lands on the edge of the
/// artwork, in a pocket the surrounding bands close off, and a target that no
/// free space surrounds takes no part in the capacity chains.
constexpr double BRIDGE_FORWARD_FACTOR = 2.0;

/// The capacity grid a configuration asks for.
grid::GridMetrics capacityGrid(const geometry::BoundingBox& box,
                               const flatbuffers::config::GridParamsT& params) {
  if (params.capacity_cells_y == 0) {
    return grid::GridMetrics::fitWidth(box, params.capacity_cells_x);
  }
  return grid::GridMetrics::fit(box, params.capacity_cells_x,
                                params.capacity_cells_y);
}

/// The grid section of a configuration. A configuration leaves out [grid]
/// when every value in it is a default, so an absent section is one of
/// defaults.
const flatbuffers::config::GridParamsT& gridParamsOf(const ConfigT& config) {
  static const flatbuffers::config::GridParamsT DEFAULTS;
  return config.grid != nullptr ? *config.grid : DEFAULTS;
}

/// The capacity grid and the detail grid a configuration asks for.
std::pair<grid::GridMetrics, grid::GridMetrics> gridsOf(const ChipT& chip,
                                                        const ConfigT& config) {
  const auto& params = gridParamsOf(config);
  if (params.detail_factor == 0) {
    throw std::invalid_argument(
        "a detail grid of zero cells per capacity cell has no cells");
  }
  if (auto problem = detailGridProblem(chip, config)) {
    throw std::invalid_argument(*std::move(problem));
  }
  const auto capacity = capacityGrid(grid::chipBounds(chip), params);
  return {capacity, capacity.refined(params.detail_factor)};
}

} // namespace

bool hasLauncherSlot(const flatbuffers::design::PortT& port) {
  const auto step = grid::orientationStep(port.orientation);
  return port.role == UnassignedRole::Launcher && (step.x != 0 || step.y != 0);
}

std::optional<std::string> detailGridProblem(const ChipT& chip,
                                             const ConfigT& config) {
  const auto& params = gridParamsOf(config);
  if (params.capacity_cells_x == 0 || params.detail_factor == 0) {
    return std::nullopt;
  }
  // The figures stay in floating point, so that no product of two 32-bit
  // figures wraps around. They are whole numbers, and every one at or below
  // the limit is exact.
  const double capacityWidth = params.capacity_cells_x;
  double capacityHeight = params.capacity_cells_y;
  if (params.capacity_cells_y == 0) {
    geometry::BoundingBox box;
    try {
      box = grid::chipBounds(chip);
    } catch (const std::invalid_argument&) {
      return std::nullopt;
    }
    const auto ratio = box.height() / box.width();
    if (!std::isfinite(ratio) || !(ratio > 0.0)) {
      return std::nullopt;
    }
    // GridMetrics::fitWidth() rounds the same way.
    capacityHeight = std::max(std::round(capacityWidth * ratio), 2.0);
  }
  const double factor = params.detail_factor;
  const auto width = capacityWidth * factor;
  const auto height = capacityHeight * factor;
  const auto cells = width * height;
  if (cells <= static_cast<double>(MAX_DETAIL_CELLS)) {
    return std::nullopt;
  }
  return std::format("[grid] asks for a detail grid of {:.0f} × {:.0f} = "
                     "{:.0f} cells, a capacity grid of {:.0f} × {:.0f} cells "
                     "times detail_factor {:.0f}; the limit is {} cells",
                     width, height, cells, capacityWidth, capacityHeight,
                     factor, MAX_DETAIL_CELLS);
}

std::vector<geometry::Point> launcherSlotPositions(const ChipT& chip,
                                                   const ConfigT& config) {
  const auto detail = gridsOf(chip, config).second;
  std::vector<geometry::Point> positions;
  for (const auto& port : chip.ports) {
    if (hasLauncherSlot(*port)) {
      const auto cell = detail.clampToCell(port->center);
      positions.push_back(detail.toLayout(cell.x(), cell.y()));
    }
  }
  return positions;
}

double crossingPitch(const ConfigT& config) {
  if (config.stages != nullptr && config.stages->capacity != nullptr) {
    return config.stages->capacity->crossing_pitch;
  }
  return flatbuffers::config::CapacityParamsT{}.crossing_pitch;
}

std::vector<grid::Point> crossingSlots(const grid::PartitionBorder& border,
                                       const CapacityScene& scene,
                                       const double pitch) {
  // The rectangle runs through the centres of the outermost launcher cells.
  auto minX = std::numeric_limits<double>::max();
  auto minY = std::numeric_limits<double>::max();
  auto maxX = std::numeric_limits<double>::lowest();
  auto maxY = std::numeric_limits<double>::lowest();
  for (const auto cell : scene.launcherCell) {
    const auto coordinate = scene.detail.cell(cell);
    const auto x = static_cast<double>(coordinate.x()) + 0.5;
    const auto y = static_cast<double>(coordinate.y()) + 0.5;
    minX = std::min(minX, x);
    minY = std::min(minY, y);
    maxX = std::max(maxX, x);
    maxY = std::max(maxY, y);
  }
  // Launchers on one line span no interval along that axis. The chip bounds
  // hold the port centres, so such a line lies on the edge of the chip, and
  // the grid bounds the slots along that axis instead.
  const auto width = static_cast<double>(scene.detail.width);
  const auto height = static_cast<double>(scene.detail.height);
  if (minX == maxX) {
    minX = 0.0;
    maxX = width;
  }
  if (minY == maxY) {
    minY = 0.0;
    maxY = height;
  }
  // A launcher sits on the edge of the grid nearest to it. Where no launcher
  // sits on an edge, no launcher stands behind a slot near it, and the grid
  // bounds the slots on that side too. Without that, launchers on two
  // opposite edges would cut every slot beyond the outermost of them.
  std::array<bool, 4> sides{}; // left, right, bottom, top
  for (const auto cell : scene.launcherCell) {
    const auto coordinate = scene.detail.cell(cell);
    const auto x = static_cast<double>(coordinate.x()) + 0.5;
    const auto y = static_cast<double>(coordinate.y()) + 0.5;
    const std::array<double, 4> away{x, width - x, y, height - y};
    sides.at(static_cast<std::size_t>(std::ranges::min_element(away) -
                                      away.begin())) = true;
  }
  minX = sides[0] ? minX : 0.0;
  maxX = sides[1] ? maxX : width;
  minY = sides[2] ? minY : 0.0;
  maxY = sides[3] ? maxY : height;
  // The samples outside the rectangle are dropped before the slots are
  // picked. A slot picked outside and dropped afterwards would still keep
  // every sample within one pitch of it from becoming a slot.
  auto inside = border;
  std::erase_if(inside.samples, [&](const grid::Point& sample) {
    return sample.x() <= minX || sample.x() >= maxX || sample.y() <= minY ||
           sample.y() >= maxY;
  });
  return grid::borderSlots(inside, scene.detail, pitch);
}

CapacityScene buildScene(const ChipT& chip, const ConfigT& config) {
  if (config.rules == nullptr) {
    throw std::invalid_argument("the configuration carries no design rules");
  }
  const auto& rules = *config.rules;

  CapacityScene scene;
  std::tie(scene.capacity, scene.detail) = gridsOf(chip, config);

  // The mask carries no obstacle keepout. The keepout makes the clearance a
  // hard constraint on a search, and nothing searches on this grid: the stage
  // partitions the free space and counts what fits through it. A keepout
  // would narrow every corridor by a fraction of a cell and move the medial
  // axis, which is what this grid exists to find. The routing grids of the
  // later stages carry it.
  //
  // No border of the grid stays blocked: the launchers sit on the edge of the
  // grid, and a blocked border would wall them in.
  const auto raster =
      grid::rasterizeObstacles(chip, scene.detail, {.keepoutExemptions = {}});
  scene.blocked = raster.blocked;
  scene.reserved = grid::BitGrid(scene.detail.width, scene.detail.height);
  scene.keepout = grid::BitGrid(scene.detail.width, scene.detail.height);

  // The mask before any band, so that a target can be placed on a cell that
  // was free of chip artwork even where a band has covered it since.
  const auto beforeBands = scene.blocked;

  // Which ports carry a wire on across their component rather than end it.
  const design::BridgeRules noRules;
  const auto& bridgeRules =
      config.ports != nullptr ? config.ports->bridge_pairs : noRules;
  const auto bridging = design::bridgingPorts(chip, bridgeRules);

  for (std::uint32_t index = 0; index < chip.ports.size(); ++index) {
    const auto& port = *chip.ports[index];
    if (!design::isRoutable(port.role)) {
      continue;
    }
    const auto step = grid::orientationStep(port.orientation);
    if (step.x == 0 && step.y == 0) {
      continue;
    }
    const auto center = scene.detail.clampToCell(port.center);
    const grid::PortBand band{
        .center = center,
        .step = step,
        .forward =
            grid::bandLength((bridging[index] ? BRIDGE_FORWARD_FACTOR : 1.0) *
                                 rules.min_straight_length,
                             scene.detail, step),
        .backward =
            grid::bandLength(BACKWARD_BAND_FACTOR * rules.min_straight_length,
                             scene.detail, step),
        .halfWidth =
            grid::bandHalfWidth(rules.min_wire_spacing, scene.detail, step)};

    const auto stamped = grid::stampBand(scene.blocked, band);
    for (std::size_t position = 0; position < stamped.cells.size();
         ++position) {
      // A cell the band blocked that was free before is reserved, not
      // artwork. A bottleneck that ends on one divides the space around a
      // port rather than meets a wall.
      if (!stamped.wasBlocked[position]) {
        scene.reserved.set(stamped.cells[position], true);
        scene.keepout.set(stamped.cells[position], true);
      }
    }

    const auto target =
        grid::placeTargetBeyondBand(scene.blocked, beforeBands, stamped, step);
    if (target.has_value()) {
      scene.reserved.set(*target, false);
      scene.targetCell.push_back(*target);
      scene.targetPort.push_back(index);
    }
  }

  // The slot of a launcher is the cell its center rounds to, and nothing is
  // blocked around it. The cell is freed, because a wire has to be able to
  // stand where it starts, and it is the only cell this loop touches.
  for (std::uint32_t index = 0; index < chip.ports.size(); ++index) {
    const auto& port = *chip.ports[index];
    if (!hasLauncherSlot(port)) {
      continue;
    }
    const auto center = scene.detail.clampToCell(port.center);
    const auto slot = scene.detail.index(center.x(), center.y());
    scene.blocked.set(slot, false);
    scene.reserved.set(slot, false);
    scene.launcherCell.push_back(slot);
    scene.launcherPort.push_back(index);
  }

  if (scene.launcherCell.empty()) {
    throw std::invalid_argument(
        "the chip carries no launcher a wire could be routed to");
  }

  // A target has to be reachable, and the bands are stamped one port at a
  // time: the band of a port can cover a target that an earlier port placed,
  // and the two ports of a coupler bridge sit close enough together that it
  // happens regularly. A target buried like that starts no chain, and the
  // whole region behind it drops out of the partitioning. Only band cells are
  // freed; a target on chip artwork stays blocked, because opening artwork
  // would let a wire run through a component.
  for (const auto cell : scene.targetCell) {
    if (scene.reserved.test(cell)) {
      scene.blocked.set(cell, false);
      scene.reserved.set(cell, false);
    }
  }
  return scene;
}

} // namespace mqt::scpd::pipeline
