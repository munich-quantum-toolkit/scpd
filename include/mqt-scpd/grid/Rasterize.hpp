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
#include "mqt-scpd/flatbuffers/geometry.hpp"
#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"
#include "mqt-scpd/grid/mqt_scpd_grid_export.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace mqt::scpd::grid {

/**
 * @brief A strip of layout space that the obstacle keepout leaves open.
 *
 * The strip holds every point within @c halfWidth of the segment from @c from
 * to @c to. A corridor keeps a port reachable: it runs from the port outward
 * along the port's orientation, through the small polygons every port carries
 * at its foot.
 */
struct Corridor {
  /// One end of the center line of the strip, in layout units.
  Point from;
  /// The other end of the center line of the strip, in layout units.
  Point to;
  /// The largest distance from the center line that the strip holds, in layout
  /// units.
  double halfWidth = 0.0;
};

/**
 * @brief The options that control how the obstacle polygons are rasterized.
 */
struct RasterOptions {
  /// The keepout distance, in layout units.
  ///
  /// Cells whose center lies within this distance of an edge of an obstacle
  /// are blocked as well. The keepout puts the
  /// obstacle clearance into the mask, so every free cell outside a corridor
  /// keeps the clearance. The distance is exact: it is measured from the cell
  /// center to the edge, not by a dilation of the mask. A value of zero or less
  /// disables the keepout.
  double keepout = 0.0;
  /// The corridors in which keepout cells stay free. A corridor never frees a
  /// cell of a polygon itself.
  std::vector<Corridor> keepoutExemptions;
  /// The number of columns blocked at the left edge and at the right edge.
  ///
  /// The border keeps wires off the edge of the grid. It is a number of cells
  /// of the rasterized grid, set per axis, because a chip that is not square
  /// needs a different border along each axis.
  uint32_t borderX = 0;
  /// The number of rows blocked at the bottom edge and at the top edge.
  uint32_t borderY = 0;
};

/**
 * @brief The obstacle mask of a chip on a grid, with counts of what the
 * keepout did.
 */
struct RasterizedObstacles {
  /// The mask, with every blocked cell set.
  BitGrid blocked;
  /// The number of cells the keepout blocked beyond the polygons themselves,
  /// counted before the corridors released any of them.
  std::size_t keepoutCells = 0;
  /// The number of keepout cells a corridor released again.
  std::size_t exemptedCells = 0;
};

/**
 * @brief Rasterizes the obstacle polygons of a chip onto a grid.
 *
 * Every polygon of the chip is artwork. The function fills each one at the
 * cell centers and outlines it with Bresenham lines through the rounded vertex
 * cells, so that no edge has a gap.
 * A blocked cell with at least seven free neighbors is an artifact of the
 * rasterization, and the function frees it again. Then the keepout and the
 * border of @p options apply, in this order.
 *
 * @param chip The chip whose obstacles to rasterize.
 * @param grid The grid to rasterize onto.
 * @param options The keepout, its corridors and the border.
 * @return The mask on @p grid, and the numbers of cells the keepout blocked
 * and the corridors released.
 */
[[nodiscard]] MQT_SCPD_GRID_EXPORT RasterizedObstacles
rasterizeObstacles(const flatbuffers::design::ChipT& chip,
                   const GridMetrics& grid, const RasterOptions& options = {});

/**
 * @brief Blocks the cells of a polygon.
 *
 * The function blocks every cell whose center lies inside the polygon. It also
 * blocks the cells of the Bresenham line between the rounded cells of every
 * two consecutive vertices, the closing edge included. These lines keep the
 * outline free of gaps where the center test misses a thin polygon. A polygon
 * with fewer than three vertices blocks only the cells of its edges. Cells off
 * the grid are skipped.
 *
 * @param mask The mask to block the cells in.
 * @param grid The grid that converts layout units into cells.
 * @param polygon The polygon, in layout units.
 * @pre @p mask has the width and the height of @p grid.
 * @post Every cell that was blocked before the call is still blocked.
 */
MQT_SCPD_GRID_EXPORT void
fillPolygon(BitGrid& mask, const GridMetrics& grid,
            const flatbuffers::geometry::PolygonT& polygon);

/**
 * @brief Lists the cells of the Bresenham line between two cells.
 * @param x0 The column of the first cell.
 * @param y0 The row of the first cell.
 * @param x1 The column of the last cell.
 * @param y1 The row of the last cell.
 * @param width The number of columns of the grid to clip the line to.
 * @param height The number of rows of the grid to clip the line to.
 * @return The row-major indices of the cells of the line, from the first cell
 * to the last, both ends included. Cells off the grid are left out, so either
 * end may lie off the grid.
 */
[[nodiscard]] MQT_SCPD_GRID_EXPORT std::vector<std::size_t>
lineCells(int64_t x0, int64_t y0, int64_t x1, int64_t y1, uint32_t width,
          uint32_t height);

/**
 * @brief Frees every blocked cell with at least seven free neighbors of its
 * eight.
 *
 * The function counts the neighbors on the mask as it was before the call, so
 * a cell it frees does not change the count of another cell. Cells on the
 * border of the grid are left alone. A mask narrower than three cells along an
 * axis stays unchanged.
 *
 * @param mask The mask to change in place.
 */
MQT_SCPD_GRID_EXPORT void removeIslands(BitGrid& mask);

/**
 * @brief Blocks every cell within a given number of cells of the grid border,
 * counted separately along each axis.
 * @param mask The mask to change in place.
 * @param alongX The number of columns to block at the left edge and at the
 * right edge.
 * @param alongY The number of rows to block at the bottom edge and at the top
 * edge.
 */
MQT_SCPD_GRID_EXPORT void blockBorder(BitGrid& mask, uint32_t alongX,
                                      uint32_t alongY);

/**
 * @brief Computes the distance from a point to a segment.
 * @param point The point, in layout units.
 * @param from One end of the segment, in layout units.
 * @param to The other end of the segment, in layout units.
 * @return The Euclidean distance from @p point to the nearest point of the
 * segment, in layout units. A segment of almost no length counts as the
 * point @p from.
 */
[[nodiscard]] MQT_SCPD_GRID_EXPORT double
distanceToSegment(Point point, Point from, Point to);

} // namespace mqt::scpd::grid
