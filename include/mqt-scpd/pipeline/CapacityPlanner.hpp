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

/// The grids and masks the capacity stage works on.
///
/// None of it is an artifact: it is rebuilt from the chip and the
/// configuration whenever a stage needs it, which is what keeps the artifacts
/// small and what makes a resumed run equal to an uninterrupted one. It is
/// public because the later routing stages rebuild the same thing, and
/// because a test that wants to check one step of the stage has to be able to
/// stand where the stage stands.
struct MQT_SCPD_PIPELINE_EXPORT CapacityScene {
  /// The coarse grid the wire budgets are counted on.
  grid::GridMetrics capacity;
  /// The fine grid the partitioning runs on.
  grid::GridMetrics detail;
  /// The obstacle mask on the detail grid, with the keepout, the port bands
  /// and the launcher sweeps already in it.
  grid::BitGrid blocked;
  /// The cells a port band reserved. They are obstacles, but not chip
  /// artwork, and a bottleneck that ends on one is budgeted differently.
  grid::BitGrid reserved;
  /// Every cell the ports' own approaches took from the free space: the
  /// bands of the routable ports and the squares the launchers sweep.
  ///
  /// This is `reserved` before the targets and the launcher slots were dug
  /// back out of it, plus the launcher sweeps, which `reserved` never
  /// carried. It decides nothing — what a gate is budgeted at reads
  /// `reserved` — and exists so that the picture can show what the ports
  /// themselves block.
  grid::BitGrid keepout;
  /// The detail cell each target is reached at.
  std::vector<std::size_t> targetCell;
  /// The port of each target cell, parallel to `targetCell`.
  std::vector<std::uint32_t> targetPort;
  /// The detail cell each launcher feeds the chip from.
  std::vector<std::size_t> launcherCell;
  /// The port of each launcher cell, parallel to `launcherCell`.
  std::vector<std::uint32_t> launcherPort;
};

/// Build the grids and masks of a run.
///
/// @throws std::invalid_argument when the configuration describes no usable
/// grid, or the chip has neither obstacles nor ports.
/// How far apart the places a wire may cross a partition border sit.
///
/// The Capacity stage counts them into `PartitionBorder.budget` and the
/// Corridor stage crosses at them, so both read the figure here rather than
/// each deciding for itself.
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT double crossingPitch(const ConfigT& config);

[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT CapacityScene buildScene(const ChipT& chip,
                                                                const ConfigT& config);

/// The capacity planner of the first release.
///
/// It rasterizes the chip, takes the distance transform of the free space,
/// finds the bottlenecks along its medial axis, grows one partition from the
/// middle of every clear capacity cell, and walks the bottlenecks outward
/// from each target to build the chains the later stages route along.
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT std::unique_ptr<ICapacityPlanner> makeWatershedPlanner();

} // namespace mqt::scpd::pipeline
