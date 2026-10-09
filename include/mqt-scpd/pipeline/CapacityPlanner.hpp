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

#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"
#include "mqt-scpd/pipeline/Stages.hpp"
#include "mqt-scpd/pipeline/mqt_scpd_pipeline_export.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
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
  /// differently.
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
 * @brief Builds the grids and masks of a run.
 *
 * A configuration without a grid section takes the grid defaults of the
 * schema.
 *
 * @param chip The classified chip.
 * @param config The run configuration.
 * @return The scene.
 * @throws std::invalid_argument When the configuration carries no design
 * rules, when its detail grid has no cells, or when the chip has no launcher.
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
 * @return The planner.
 */
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT std::unique_ptr<ICapacityPlanner>
makeWatershedPlanner();

} // namespace mqt::scpd::pipeline
