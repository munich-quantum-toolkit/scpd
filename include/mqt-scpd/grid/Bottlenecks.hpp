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
#include "mqt-scpd/grid/Voronoi.hpp"
#include "mqt-scpd/grid/mqt_scpd_grid_export.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace mqt::scpd::grid {

/**
 * @brief The narrowest place of a corridor: the two obstacle cells that face
 * each other across it, and the cell of the medial axis between them.
 *
 * A bottleneck is a line, and its length decides how many wires fit through
 * the corridor. The later stages treat it as a gate that a wire has to pass.
 */
struct Bottleneck {
  /// The obstacle cell on one side.
  std::size_t first = 0;
  /// The obstacle cell on the other side.
  std::size_t second = 0;
  /// The cell of the medial axis the two were found from.
  std::size_t saddle = 0;
};

/// How bottlenecks are found.
struct BottleneckOptions {
  /// A cell of the axis is a candidate only when its squared distance to the
  /// nearest wall, in cells, is below this value. A place with room to spare
  /// is not a bottleneck, however local a minimum it is. The default admits a
  /// clearance below about 12.2 cells.
  std::uint32_t maximumSquaredClearance = 150;
  /// How far apart two cuts of one narrowing may end and still be one gate,
  /// in layout units.
  ///
  /// The search reports a candidate wherever the clearance stops falling, so
  /// a narrowing whose clearance stays flat over a few cells comes back as
  /// several lines that share a wall and end a cell or two apart on the other.
  /// They are one place. The wire pitch is the length to give here, being the
  /// distance within which the count of wires can change at all. Zero keeps
  /// every candidate.
  double sameNarrowing = 0.0;
  /// Cells that no bottleneck may cross, because the space around a port is
  /// not a wall that capacity divides around: the targets of the run.
  std::span<const std::size_t> targets;
};

/**
 * @brief Finds the bottlenecks of a rasterized chip.
 *
 * A candidate is a cell of the medial axis whose clearance is a local minimum
 * along the axis, or one edge of a plateau of minima. From it the free space
 * splits into shores, the connected groups of its off-axis neighbors, and a
 * bottleneck exists where there are two. Each shore is walked downhill in the
 * distance transform until it reaches a wall, and the two walls reached are
 * the ends of the bottleneck. The shores are kept apart during the walk, so
 * the two ends lie on different sides of the corridor.
 *
 * @param blocked The obstacle mask.
 * @param axis The rasterized medial axis of the mask.
 * @param squaredDistance The squared distance transform of the mask.
 * @param grid The metrics of the grid of the mask.
 * @param options How the bottlenecks are found.
 * @return The bottlenecks, ordered by the cell of their candidate, one per
 * narrowing.
 * @throws std::invalid_argument When the mask, the axis and the distance
 * transform do not describe one grid.
 */
[[nodiscard]] MQT_SCPD_GRID_EXPORT std::vector<Bottleneck>
findBottlenecks(const BitGrid& blocked, const MedialAxis& axis,
                std::span<const std::uint32_t> squaredDistance,
                const GridMetrics& grid, const BottleneckOptions& options = {});

/**
 * @brief Counts the wires that fit through a bottleneck.
 *
 * The gap is the length of the bottleneck in layout units, and every wire
 * takes one pitch: a wire spacing plus an obstacle spacing. A gap no longer
 * than the obstacle spacing carries nothing, and a gap shorter than a wire
 * spacing carries no whole wire either.
 *
 * @param bottleneck The bottleneck.
 * @param grid The metrics of the grid it was found on.
 * @param wireSpacing The clearance two wires keep.
 * @param obstacleSpacing The clearance a wire keeps to an obstacle.
 * @param roundDown Whether the count rounds down rather than up, which it does
 * when an end of the bottleneck sits on a cell reserved for a port rather than
 * on chip artwork.
 * @return The number of wires.
 */
[[nodiscard]] MQT_SCPD_GRID_EXPORT std::uint32_t
bottleneckCapacity(const Bottleneck& bottleneck, const GridMetrics& grid,
                   double wireSpacing, double obstacleSpacing, bool roundDown);

} // namespace mqt::scpd::grid
