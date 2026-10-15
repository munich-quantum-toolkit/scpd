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

#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"
#include "mqt-scpd/grid/Partitions.hpp"
#include "mqt-scpd/pipeline/Stages.hpp"
#include "mqt-scpd/pipeline/mqt_scpd_pipeline_export.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace mqt::scpd::pipeline {

/**
 * @brief The grids and masks the capacity stage works on.
 *
 * None of it is an artifact. It is rebuilt from the chip and the configuration
 * whenever a stage needs it, which keeps the artifacts small and makes a
 * resumed run equal to an uninterrupted one. The Corridor stage rebuilds the
 * same scene.
 */
struct MQT_SCPD_PIPELINE_EXPORT CapacityScene {
  /// The coarse grid the wire budgets are counted on.
  grid::GridMetrics capacity;
  /// The fine grid the partitioning runs on.
  grid::GridMetrics detail;
  /// The obstacle mask on the detail grid, with the border and the port bands
  /// in it.
  grid::BitGrid blocked;
  /// The cells a port band blocked that were free before. They are obstacles
  /// but not chip artwork, and a bottleneck that ends on one is budgeted
  /// differently. Where the stage took a band back to place the target of its
  /// port beyond artwork, the cells of the band are free again but stay
  /// reserved: they belong to no partition.
  grid::BitGrid reserved;
  /// Every cell the approaches of the ports took from the free space: the
  /// bands of the routable ports, before the targets were dug back out. It
  /// decides nothing and exists so that a picture can show what the ports
  /// block.
  grid::BitGrid keepout;
  /// The detail cell each target is reached at.
  std::vector<std::size_t> targetCell;
  /// The port of each target cell, parallel to targetCell.
  std::vector<std::uint32_t> targetPort;
  /// The detail cell each launcher feeds the chip from.
  std::vector<std::size_t> launcherCell;
  /// The port of each launcher cell, parallel to launcherCell.
  std::vector<std::uint32_t> launcherPort;
};

/**
 * @brief Returns how far apart the places sit where a wire may cross a
 * partition border.
 *
 * The Capacity stage counts them into the budget of a border and the Corridor
 * stage crosses at them, so both read the figure here.
 *
 * @param config The run configuration.
 * @return The crossing pitch, in layout units.
 */
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT double
crossingPitch(const ConfigT& config);

/**
 * @brief Returns the crossing slots of a partition border that a wire may
 * take.
 *
 * These are the slots of grid::borderSlots() at the crossing pitch, picked
 * from the samples that lie strictly inside the rectangle the launcher cells
 * span. Every wire starts on the launcher ring, so a slot on or outside that
 * rectangle sits beside or behind a launcher, and a wire that crossed there
 * would run along the back of the ring. Where the launchers span no interval
 * along an axis, as on one line of launchers or for a single launcher, the
 * edges of the grid bound the slots along that axis instead. The chip bounds
 * hold the port centres, so such launchers sit on the edge of the chip. The
 * same holds for each edge of the grid that no launcher sits on, where a
 * launcher sits on the edge nearest to it. The Capacity stage counts these
 * slots into the budget of a border, and the Corridor stage routes across the
 * same slots.
 *
 * @param border The border, in cell coordinates of the detail grid.
 * @param scene The scene of the run, which holds the launcher cells.
 * @param pitch The crossing pitch, in layout units.
 * @return The slots, in cell coordinates, in the order of
 * grid::borderSlots().
 * @throws std::invalid_argument When @p pitch is not positive.
 */
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT std::vector<grid::Point>
crossingSlots(const grid::PartitionBorder& border, const CapacityScene& scene,
              double pitch);

/**
 * @brief Returns whether the Capacity stage gives a port a launcher slot.
 *
 * A port gets a slot when it is a launcher with a direction. The Capacity
 * stage and the checks of the Assignment stage read the launchers through
 * this one rule.
 *
 * @param port The port of the classified chip.
 * @return True when the port gets a launcher slot.
 */
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT bool
hasLauncherSlot(const flatbuffers::design::PortT& port);

/**
 * @brief Returns where the Capacity stage places the slot of each launcher.
 *
 * A launcher with a direction gets a slot at the center of the detail cell
 * its own center rounds to. The positions match the slots of buildScene()
 * without building the masks.
 *
 * @param chip The classified chip.
 * @param config The run configuration, whose grid section places the cells.
 * @return The slot positions in layout units, in port order.
 * @throws std::invalid_argument When the detail grid of the configuration has
 * no cells or more than MAX_DETAIL_CELLS.
 */
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT std::vector<geometry::Point>
launcherSlotPositions(const ChipT& chip, const ConfigT& config);

/**
 * @brief The most cells a detail grid may have.
 *
 * The Capacity stage holds about 120 bytes per detail cell at its peak, so the
 * limit stands for about 3 GB. The finest benchmark chip, 69q, plans on 1500 ×
 * 1500 cells, less than a tenth of the limit. A typo that adds a digit to
 * `detail_factor`, or to `capacity_cells_x` where the aspect ratio sets the
 * cells along y, multiplies the cells by a hundred. On every benchmark chip
 * from 17q up, that goes past the limit. The limit also keeps every axis far
 * below the 32-bit range of a cell coordinate.
 */
inline constexpr std::uint64_t MAX_DETAIL_CELLS = 25'000'000;

/**
 * @brief Checks the size of the detail grid that a configuration asks for.
 *
 * The detail grid has `capacity_cells_x` times `detail_factor` cells along x,
 * and `capacity_cells_y` times `detail_factor` along y. With
 * `capacity_cells_y` at zero, the aspect ratio of the chip sets the capacity
 * cells along y, as buildScene() sets them. A configuration without a grid
 * section takes the grid defaults of the schema.
 *
 * @param chip The classified chip, whose box sets the aspect ratio.
 * @param config The run configuration.
 * @return The problem when the detail grid has more than MAX_DETAIL_CELLS
 * cells. None when it has fewer, and none when the configuration or the chip
 * gives no grid at all; buildScene() refuses those.
 */
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT std::optional<std::string>
detailGridProblem(const ChipT& chip, const ConfigT& config);

/**
 * @brief Builds the grids and masks of a run.
 *
 * A configuration without a grid section takes the grid defaults of the
 * schema.
 *
 * @param chip The classified chip.
 * @param config The run configuration.
 * @return The scene.
 * @throws std::invalid_argument When the configuration carries no design
 * rules, when its detail grid has no cells or more than MAX_DETAIL_CELLS, or
 * when the chip has no launcher.
 */
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT CapacityScene
buildScene(const ChipT& chip, const ConfigT& config);

/**
 * @brief Creates the capacity planner of the first release.
 *
 * The planner rasterizes the chip, takes the distance transform of the free
 * space, finds the bottlenecks along its medial axis, walks them outward from
 * each target into the capacity chains, carves the chambers the chains cross
 * into partitions, and grows the rest of the free space from the middle of
 * every clear capacity cell.
 *
 * A bottleneck of a chain usually parts two partitions, so the Corridor stage
 * meets a border there. A bottleneck between a wall and an island is the
 * exception when the free space also runs around the other side of the
 * island and the chain dropped the gate there: both sides of the bottleneck
 * then lie in one partition. The Global stage still budgets the wires of the
 * chain through the bottleneck, which counts less room than the island
 * leaves, and the Corridor stage meets no border at it.
 *
 * A corridor that runs at an angle other than a grid axis or a diagonal gives
 * a row of bottlenecks along its length (see grid::findBottlenecks()). Where
 * they follow one another in a chain, the chain keeps the one with the smallest
 * budget, so the raster picks the cut that sets the budget of the corridor,
 * and that cut can count fewer wires than the rest of the corridor carries.
 * The medial axis lies off the middle of a corridor by up to one cell (see
 * grid::rasterizeMedialAxis()), so the opening beside a rectangular obstacle
 * can get more than one bottleneck too, and two mirrored openings different
 * ones.
 *
 * @return The planner.
 */
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT std::unique_ptr<ICapacityPlanner>
makeWatershedPlanner();

} // namespace mqt::scpd::pipeline
