/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/design/Roles.hpp"
#include "mqt-scpd/flatbuffers/config.hpp"
#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/geometry/Geometry.hpp"
#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"
#include "mqt-scpd/design/Bridges.hpp"
#include "mqt-scpd/grid/PortBands.hpp"
#include "mqt-scpd/grid/Rasterize.hpp"
#include "mqt-scpd/pipeline/CapacityPlanner.hpp"

#include <cstddef>
#include <cstdint>
#include <format>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace mqt::scpd::pipeline {
namespace {

using flatbuffers::design::UnassignedRole;

/// How far a port band reaches back against the direction a wire leaves the
/// port, as a multiple of the forward reach.
///
/// The band exists so that no other wire runs through a port's approach. It
/// reaches further backward than forward because backward is into the
/// component's own artwork, where a wire has no business being at all.
///
/// The prototype used three, and six for a qubit's second routing port,
/// which it selected by testing the port's label for a leading `Q` and a
/// trailing `1`. That test is a component model recovered from a name, which
/// this port does not have and does not want; the longer band covered
/// artwork the obstacle raster already blocks, so one factor is used for
/// every port. A chip that needs more gets a documented override rather than
/// a label test.
constexpr double BACKWARD_BAND_FACTOR = 3.0;

/// How much further forward a bridge port's band reaches than an ordinary
/// port's.
///
/// A wire does not stop at a bridge port: it crosses the component and leaves
/// through the port's partner. Its approach therefore has to clear the
/// component's own artwork, which lies exactly one component width ahead,
/// instead of merely leaving the port. A single reach puts the target on the
/// artwork's edge, in a pocket the surrounding bands close off, and a target
/// that no free space surrounds takes no part in the capacity chains.
constexpr double BRIDGE_FORWARD_FACTOR = 2.0;

/// How far a launcher's sweep reaches along its orientation, in cells of the
/// detail grid.
constexpr std::uint32_t LAUNCHER_SWEEP_CELLS = 6;

/// The capacity grid a configuration asks for.
grid::GridMetrics capacityGrid(const geometry::BoundingBox& box,
                               const flatbuffers::config::GridParamsT& params) {
  if (params.capacity_cells_y == 0) {
    return grid::GridMetrics::fitWidth(box, params.capacity_cells_x);
  }
  return grid::GridMetrics::fit(box, params.capacity_cells_x, params.capacity_cells_y);
}

} // namespace

CapacityScene buildScene(const ChipT& chip, const ConfigT& config) {
  if (config.grid == nullptr) {
    throw std::invalid_argument("the configuration carries no grid section");
  }
  if (config.rules == nullptr) {
    throw std::invalid_argument("the configuration carries no design rules");
  }
  const auto& params = *config.grid;
  const auto& rules = *config.rules;
  if (params.detail_factor == 0) {
    throw std::invalid_argument("a detail grid of zero cells per capacity cell has no cells");
  }

  CapacityScene scene;
  const auto box = grid::chipBounds(chip);
  scene.capacity = capacityGrid(box, params);
  scene.detail = scene.capacity.refined(params.detail_factor);

  // The keepout is baked into the mask rather than checked afterwards, so
  // every cell a later search may enter already satisfies the obstacle
  // clearance. The corridors keep the ports themselves reachable.
  //
  // The configured launcher offset is a border of detail cells that stays
  // blocked along each axis. It is what keeps a launcher's wires off the very
  // edge of the grid, and without it the free space runs right to the border
  // and every corridor near it looks wider than it is.
  // No obstacle keepout here. The keepout is what makes the clearance a hard
  // constraint on a *search*, and nothing searches on the capacity grid: the
  // stage partitions free space and counts what fits through it. Baking a
  // clearance in would narrow every corridor by a fraction of a cell and move
  // the medial axis, which is the one thing this grid exists to find. The
  // routing grids of the later stages carry it.
  const auto raster = grid::rasterizeObstacles(chip, scene.detail,
                                               {.borderX = params.launcher_offset_x,
                                                .borderY = params.launcher_offset_y});
  scene.blocked = raster.blocked;
  scene.reserved = grid::BitGrid(scene.detail.width, scene.detail.height);
  scene.keepout = grid::BitGrid(scene.detail.width, scene.detail.height);

  // The mask before any band, so that a target can be placed on a cell that
  // was free of chip artwork even where a band has since covered it.
  const auto beforeBands = scene.blocked;

  // Which ports carry a wire on across their component rather than ending it.
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
        .forward = grid::bandLength(
            (bridging[index] ? BRIDGE_FORWARD_FACTOR : 1.0) * rules.min_straight_length,
            scene.detail, step.diagonal()),
        .backward = grid::bandLength(BACKWARD_BAND_FACTOR * rules.min_straight_length,
                                     scene.detail, step.diagonal()),
        .halfWidth =
            grid::bandHalfWidth(rules.min_wire_spacing, scene.detail, step.diagonal())};

    const auto stamped = grid::stampBand(scene.blocked, band);
    for (std::size_t position = 0; position < stamped.cells.size(); ++position) {
      // A cell the band blocked that was free before is reserved, not
      // artwork. A bottleneck that ends on one is a division of the space
      // around a port rather than a wall.
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

  const auto beforeSweeps = scene.blocked;
  for (std::uint32_t index = 0; index < chip.ports.size(); ++index) {
    const auto& port = *chip.ports[index];
    if (port.role != UnassignedRole::Launcher) {
      continue;
    }
    const auto step = grid::orientationStep(port.orientation);
    if (step.x == 0 && step.y == 0) {
      continue;
    }
    const auto center = scene.detail.clampToCell(port.center);
    const auto halfWidth =
        grid::bandHalfWidth(rules.min_wire_spacing, scene.detail, step.diagonal());
    const auto slot = grid::stampLauncherSweep(scene.blocked, center, step,
                                               LAUNCHER_SWEEP_CELLS, halfWidth);
    scene.launcherCell.push_back(slot);
    scene.launcherPort.push_back(index);
  }

  // What the sweeps took from the free space, for the picture. A sweep, unlike
  // a band, does not say which cells it changed, so the mask is compared with
  // the one before the whole loop; nothing else writes to it in between.
  for (std::size_t cell = 0; cell < scene.blocked.size(); ++cell) {
    if (scene.blocked.test(cell) && !beforeSweeps.test(cell)) {
      scene.keepout.set(cell, true);
    }
  }

  if (scene.launcherCell.empty()) {
    throw std::invalid_argument("the chip carries no launcher a wire could be routed to");
  }

  // A target has to be reachable, and the bands are stamped one port at a
  // time: a port's band can cover a target that an earlier port already
  // placed, and the two ports of a coupler bridge sit close enough together
  // that it regularly does. A target buried like that starts no chain at all,
  // so the whole region behind it drops out of the partitioning.
  //
  // Only band cells are freed. A target that landed on chip artwork is a
  // different problem and is left alone, because opening artwork would let a
  // wire route through a component.
  for (const auto cell : scene.targetCell) {
    if (scene.reserved.test(cell)) {
      scene.blocked.set(cell, false);
      scene.reserved.set(cell, false);
    }
  }
  for (const auto cell : scene.launcherCell) {
    if (scene.reserved.test(cell)) {
      scene.blocked.set(cell, false);
      scene.reserved.set(cell, false);
    }
  }
  return scene;
}

} // namespace mqt::scpd::pipeline
