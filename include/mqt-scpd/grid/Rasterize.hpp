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
 * @brief A port corridor: a strip of layout space that the obstacle keepout
 * leaves open.
 *
 * The strip holds every point within @c halfWidth of the segment from @c from
 * to @c to. A port corridor keeps a port reachable: it runs from the port
 * outward along the port's orientation, through the small polygons every port
 * carries at its foot.
 */
struct PortCorridor {
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
  /// are blocked as well. The keepout puts the obstacle spacing into the mask,
  /// so every free cell outside a port corridor keeps the obstacle spacing.
  /// The distance is measured from the cell center to the edge, not by a
  /// dilation of the mask. rasterizeObstacles() compares the distance that
  /// distanceToSegment() computes with the keepout, and a center whose
  /// computed distance equals the keepout counts as within. The computed
  /// distance is the same on every target. It differs from the exact distance
  /// by about 2 ulp when the nearest point is an end of the edge. Otherwise it
  /// differs by up to about 1e-16 times the largest coordinate. The error goes
  /// in both directions, so a center within that error of the keepout can
  /// count as within or as outside. For an edge whose two ends both lie very
  /// far off the grid, the error is no longer small against the distance, as
  /// distanceToSegment() describes. A value of zero or less disables the
  /// keepout.
  double keepout = 0.0;
  /// The port corridors in which keepout cells stay free. A port corridor
  /// never frees a cell that the polygons block after the island rule of
  /// rasterizeObstacles(). A cell that the island rule frees is no longer a
  /// polygon cell: when the keepout blocks it again, it is a keepout cell, and
  /// a port corridor can free it.
  std::vector<PortCorridor> keepoutExemptions;
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
  /// counted before the port corridors released any of them.
  std::size_t keepoutCells = 0;
  /// The number of keepout cells a port corridor released again.
  std::size_t exemptedCells = 0;
};

/**
 * @brief Rasterizes the obstacle polygons of a chip onto a grid.
 *
 * Every polygon of the chip is artwork. The function fills each one at the
 * cell centers and outlines it with Bresenham lines through the rounded vertex
 * cells, so that no edge has a gap (fillPolygon()).
 *
 * The island rule follows (removeIslands()): a blocked cell with at least
 * seven free neighbors is an artifact of the rasterization, and the function
 * frees it again. So a line one cell wide loses one cell at each end, and a
 * feature of one or two cells disappears. A cell that the island rule frees
 * is no longer a polygon cell.
 *
 * Then the keepout and the border of @p options apply, in this order. The
 * keepout is measured from the polygon edges, not from the mask, so it can
 * block a cell that the island rule freed. Such a cell is a keepout cell, and
 * a port corridor can free it.
 *
 * @param chip The chip whose obstacles to rasterize.
 * @param grid The grid to rasterize onto.
 * @param options The keepout, its port corridors and the border.
 * @return The mask on @p grid, and the numbers of cells the keepout blocked
 * and the port corridors released.
 * @throws std::invalid_argument If a vertex of an obstacle is not finite, as
 * fillPolygon() throws.
 */
[[nodiscard]] MQT_SCPD_GRID_EXPORT RasterizedObstacles
rasterizeObstacles(const flatbuffers::design::ChipT& chip,
                   const GridMetrics& grid, const RasterOptions& options = {});

/**
 * @brief Blocks the cells of a polygon.
 *
 * The function blocks every cell whose center lies inside the polygon by the
 * even-odd rule along the row of the center. An edge crosses the row when one
 * of its ends lies above the row and the other does not. The center lies
 * inside when an odd number of these crossings lie right of it. So a center
 * on the outline lies inside when the polygon covers the points just right of
 * the center and a little above its row.
 *
 * The function also blocks the cells of the Bresenham line between the
 * rounded cells of every two consecutive vertices, the closing edge included.
 * A vertex rounds to the nearest cell, and a vertex halfway between two cells
 * rounds away from cell zero, as std::llround() does. These lines keep the
 * outline free of gaps where the center test misses a thin polygon. A polygon
 * with fewer than three vertices blocks only the cells of its edges. Cells off
 * the grid are skipped, so a vertex may lie far off the grid. An end of an
 * edge more than 2^59 cells off the grid first moves along the edge to that
 * distance, so that its cell fits into an integer. The center test computes
 * where an edge crosses a row from the vertices in double precision, so a
 * vertex very far off the grid makes the crossings of its edges inexact. The
 * line of an edge is inexact as well when both of its ends lie that far off
 * the grid.
 *
 * @param mask The mask to block the cells in.
 * @param grid The grid that converts layout units into cells.
 * @param polygon The polygon, in layout units.
 * @pre @p mask has the width and the height of @p grid.
 * @post Every cell that was blocked before the call is still blocked.
 * @throws std::invalid_argument If a vertex, in layout units or converted into
 * cells, is not finite. The function then leaves @p mask unchanged.
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
 * end may lie off the grid. The function starts the walk at the first cell on
 * the grid, in the state that the walk from the first end reaches there, and
 * stops at the last cell on the grid.
 * @throws std::invalid_argument If the two cells lie 2^61 or more cells apart
 * along an axis.
 */
[[nodiscard]] MQT_SCPD_GRID_EXPORT std::vector<std::size_t>
lineCells(int64_t x0, int64_t y0, int64_t x1, int64_t y1, uint32_t width,
          uint32_t height);

/**
 * @brief Frees every blocked cell with at least seven free neighbors of its
 * eight.
 *
 * The function counts the neighbors on the mask as it was before the call, so
 * a cell it frees does not change the count of another cell. A line one cell
 * wide therefore loses one cell at each end, and a group of one or two cells
 * disappears. Cells on the border of the grid are left alone. A mask narrower
 * than three cells along an axis stays unchanged.
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
 *
 * The function computes the point of the segment nearest to @p point, starting
 * from the end of the segment that the nearest point lies closer to. It
 * returns the square root of the sum of the squared coordinate differences
 * from @p point to the nearest point. Every step is a basic operation or
 * std::sqrt, and IEEE 754 rounds each of them correctly. For a distance from
 * about 1.5e-154 to 1.3e154, the result is therefore the same on every
 * platform. In that range, the result is the exact distance when the computed
 * nearest point is the exact nearest point, and the coordinate differences,
 * their squares, the sum of the squares and its square root are all doubles.
 * Otherwise the result differs from the exact distance by about 2 ulp when the
 * nearest point is an end of the segment, and by up to about 1e-16 times the
 * largest coordinate when the nearest point lies inside the segment. The
 * squares round before they are added, so the result is not always the exact
 * distance rounded to a double.
 *
 * The nearest point comes from the coordinates of the ends in double
 * precision. When both ends lie very far from @p point, the rounding error of
 * the nearest point is no longer small against the distance, and the result
 * is inexact. For the segment from (-1e18, 50) to (1e18, 50), the point
 * (37, 47) gets the distance 37.1 instead of 3.
 *
 * @param point The point, in layout units.
 * @param from One end of the segment, in layout units.
 * @param to The other end of the segment, in layout units.
 * @return The Euclidean distance from @p point to the nearest point of the
 * segment, in layout units. A segment of almost no length counts as the
 * point @p from. A segment whose squared length does not fit into a double
 * gives the distance too, as long as its ends are finite.
 */
[[nodiscard]] MQT_SCPD_GRID_EXPORT double
distanceToSegment(Point point, Point from, Point to);

} // namespace mqt::scpd::grid
