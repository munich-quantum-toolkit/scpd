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

/// A strip of layout space that the obstacle keepout leaves open: from a port
/// outward along its orientation, with a half width, so that the port stays
/// reachable through the small polygons every port carries at its foot.
struct Corridor {
  Point from;
  Point to;
  double halfWidth = 0.0;
};

/// How the obstacle polygons are rasterized.
struct RasterOptions {
  /// Cells whose center lies within this layout distance of an obstacle edge
  /// are blocked as well. This bakes the obstacle clearance into the mask, so
  /// every cell a router may enter satisfies the rule by construction. The
  /// distance is measured exactly, in layout units, from the cell center to
  /// the edge; it is not a dilation of the mask. Zero disables the keepout.
  double keepout = 0.0;
  /// Keepout cells inside these corridors stay free.
  std::vector<Corridor> keepoutExemptions;
  /// Cells within this many cells of the left and right edge are blocked.
  ///
  /// The border keeps a launcher's wires off the very edge of the grid, which
  /// is what the configured launcher offset asks for. It is a count of detail
  /// cells and is stated per axis, because a chip that is not square wants a
  /// different reach along each.
  uint32_t borderX = 0;
  /// Cells within this many cells of the top and bottom edge are blocked.
  uint32_t borderY = 0;
};

/// The obstacle mask of a chip on a grid, with what the keepout did.
struct RasterizedObstacles {
  BitGrid blocked;
  /// Cells the keepout blocked beyond the polygons themselves.
  std::size_t keepoutCells = 0;
  /// Keepout cells a corridor released again.
  std::size_t exemptedCells = 0;
};

/// Rasterize the obstacle polygons of a chip onto a grid.
///
/// The first polygon is the chip outline and is skipped. Every other polygon
/// is filled at the cell centers and outlined with Bresenham lines through
/// the rounded vertex cells, so that no edge has a gap. A blocked cell with
/// at least seven free neighbors is an artifact of the rasterization and is
/// freed again. Then the keepout and the border of the options apply.
[[nodiscard]] MQT_SCPD_GRID_EXPORT RasterizedObstacles
rasterizeObstacles(const flatbuffers::design::ChipT& chip,
                   const GridMetrics& grid, const RasterOptions& options = {});

/// Block the cells whose center lies inside the polygon, and the cells of
/// its edges. The polygon is given in layout units.
MQT_SCPD_GRID_EXPORT void fillPolygon(BitGrid& mask, const GridMetrics& grid,
                                      const flatbuffers::geometry::PolygonT& polygon);

/// The cells of the Bresenham line between two cells, both ends included,
/// clipped to a grid of width by height cells.
[[nodiscard]] MQT_SCPD_GRID_EXPORT std::vector<std::size_t>
lineCells(int64_t x0, int64_t y0, int64_t x1, int64_t y1, uint32_t width,
          uint32_t height);

/// Free every blocked cell with at least seven free neighbors of its eight.
/// Cells on the grid border are left alone.
MQT_SCPD_GRID_EXPORT void removeIslands(BitGrid& mask);

/// Block every cell within the given number of cells of the grid border,
/// counted separately along each axis.
MQT_SCPD_GRID_EXPORT void blockBorder(BitGrid& mask, uint32_t alongX, uint32_t alongY);

/// The distance from a point to a segment, all in layout units.
[[nodiscard]] MQT_SCPD_GRID_EXPORT double
distanceToSegment(Point point, Point from, Point to);

} // namespace mqt::scpd::grid
