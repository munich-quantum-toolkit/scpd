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

#include "mqt-scpd/geometry/Geometry.hpp"
#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"
#include "mqt-scpd/grid/mqt_scpd_grid_export.hpp"

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace mqt::scpd::grid {

/// A segment of the medial axis, in cell coordinates.
struct MedialEdge {
  /// The x coordinate of the first end.
  double x0 = 0.0;
  /// The y coordinate of the first end.
  double y0 = 0.0;
  /// The x coordinate of the second end.
  double x1 = 0.0;
  /// The y coordinate of the second end.
  double y1 = 0.0;
};

/**
 * @brief Computes the medial axis of the free space of a mask.
 *
 * The medial axis is the set of points that lie equally far from two
 * different walls, which a Voronoi diagram over the boundary cells of the
 * obstacles gives. Every corridor of the chip runs along it, and where the
 * clearance along it has a local minimum, the corridor has a bottleneck.
 *
 * Two filters make the result usable. An edge between two sites of the same
 * wall is an artifact of the raster and is dropped; two sites belong to one
 * wall when a walk over the boundary cells reaches one from the other within
 * four steps. An edge that passes through a blocked cell is dropped as well,
 * because the axis of the free space cannot run through an obstacle. The grid
 * border counts as a wall.
 *
 * @param blocked The obstacle mask.
 * @return The edges that survive both filters, in cell coordinates, each
 * once.
 */
[[nodiscard]] MQT_SCPD_GRID_EXPORT std::vector<MedialEdge>
medialAxis(const BitGrid& blocked);

/// What a cell of the rasterized medial axis is.
enum class AxisCell : std::uint8_t {
  /// The cell is not on the axis.
  None = 0,
  /// The cell is on the axis and has one or two neighbors there.
  Path = 1,
  /// The cell is on the axis where three or more arms meet.
  Junction = 2,
  /// The cell is on the axis and has exactly one neighbor there.
  Endpoint = 4,
};

/// The medial axis on the grid: which cells it covers and how they connect.
struct MedialAxis {
  /// One entry per cell of the grid, saying what the cell is on the axis.
  std::vector<AxisCell> cells;
  /// The cells adjacent along the axis, for every cell on it.
  std::unordered_map<std::size_t, std::vector<std::size_t>> neighbors;

  /**
   * @brief Checks whether a cell lies on the axis.
   * @param index The row-major index of the cell.
   * @return @c true when the cell is on the grid and on the axis.
   */
  [[nodiscard]] bool onAxis(const std::size_t index) const {
    return index < cells.size() && cells[index] != AxisCell::None;
  }
};

/**
 * @brief Rasterizes the medial axis onto the grid.
 *
 * Each edge is walked as a four-connected line through the cells its ends
 * truncate to, and a blocked cell is never marked. Each cell is joined to the
 * cell before it on the same edge, so the result is a graph over the grid. A
 * cell with three or more neighbors is a junction, and a cell with one
 * neighbor is an endpoint.
 *
 * @param blocked The obstacle mask the edges were computed from.
 * @param grid The metrics of the grid of the mask.
 * @param edges The edges of the medial axis, in cell coordinates.
 * @return The axis on the grid.
 * @throws std::invalid_argument When the mask does not have the size of the
 * grid.
 */
[[nodiscard]] MQT_SCPD_GRID_EXPORT MedialAxis
rasterizeMedialAxis(const BitGrid& blocked, const GridMetrics& grid,
                    const std::vector<MedialEdge>& edges);

} // namespace mqt::scpd::grid
