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
  /// The raster can break one narrowing into several minima of the clearance
  /// a cell or two apart, and their lines share a wall and end a cell or two
  /// apart on the other. They are one place. The wire pitch is the length to
  /// give here, being the distance within which the count of wires can change
  /// at all. Zero keeps every candidate.
  double sameNarrowing = 0.0;
  /// Cells that no bottleneck may cross, because the space around a port is
  /// not a wall that capacity divides around: the targets of the run.
  std::span<const std::size_t> targets;
};

/**
 * @brief Finds the bottlenecks of a rasterized chip.
 *
 * A candidate is a stretch of one arm of the medial axis with constant
 * clearance whose bounding cells on both sides are at least as wide, so that a
 * corridor of constant width gives one candidate and a corridor that widens
 * step by step gives one at its narrow end. The middle cell of the stretch
 * stands for it. A junction or a dead end of the axis is never a candidate.
 *
 * A corridor of constant width gives one candidate only when it runs along a
 * grid axis or a diagonal. At any other angle the raster makes the clearance
 * step between two values, such as 9 and 8 squared cells, and every lower
 * step is a candidate: about one every two cells along the corridor. The
 * cuts of these candidates differ in length by a cell or so, and
 * BottleneckOptions::sameNarrowing does not merge them, because the
 * staircase of the wall gives no two of them a wall cell in common.
 *
 * Around the middle cell the free space splits into shores, the connected
 * groups of its off-axis neighbors, and a bottleneck exists where there are
 * two. Each shore is walked downhill in the distance transform until it
 * reaches a wall, and the two walls reached are the ends of the bottleneck.
 * The shores are kept apart during the walk, so the two ends lie on different
 * sides of the corridor.
 *
 * Cells outside the grid are no wall. A passage between the artwork and an
 * open edge of the grid is therefore not found, and the caller blocks a border
 * of cells where the edge has to act as a wall.
 *
 * @param blocked The obstacle mask.
 * @param axis The rasterized medial axis of the mask.
 * @param squaredDistance The squared distance transform of the mask.
 * @param grid The metrics of the grid of the mask.
 * @param options How the bottlenecks are found.
 * @return The bottlenecks, ordered by the cell of their candidate.
 * @throws std::invalid_argument When the mask, the axis and the distance
 * transform do not describe one grid.
 */
[[nodiscard]] MQT_SCPD_GRID_EXPORT std::vector<Bottleneck>
findBottlenecks(const BitGrid& blocked, const MedialAxis& axis,
                std::span<const std::uint32_t> squaredDistance,
                const GridMetrics& grid, const BottleneckOptions& options = {});

/**
 * @brief Counts the wires that a bottleneck lets through.
 *
 * The gap is the length of the bottleneck in layout units, with each axis
 * counted in its own cell step. Every wire takes one pitch: a wire spacing
 * plus an obstacle spacing. A gap no longer than the obstacle spacing carries
 * nothing, and a gap shorter than a wire spacing carries no whole wire either.
 *
 * Between two ends on chip artwork, the count is the gap in pitches rounded
 * up, as in the research prototype, so it can be one wire more than the gap
 * holds. Where an end sits on a cell reserved for a port, the count rounds
 * down. The crossing slots of the Corridor stage, not this count, decide how
 * many wires cross a partition border.
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
