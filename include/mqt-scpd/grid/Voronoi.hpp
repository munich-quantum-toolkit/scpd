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

/// A segment of the medial axis, in cell coordinates as doubles.
struct MedialEdge {
  double x0 = 0.0;
  double y0 = 0.0;
  double x1 = 0.0;
  double y1 = 0.0;
};

/// The medial axis of the free space of a mask.
///
/// The axis is the set of points equidistant from two different walls, which
/// is what a Voronoi diagram over the boundary cells of the obstacles gives.
/// Every corridor of the chip runs along it, and where the clearance along it
/// has a local minimum, the corridor has a bottleneck. That is the whole
/// reason the capacity stage builds it.
///
/// Two filters do the work that makes the result usable. An edge between two
/// sites of the *same* wall is an artifact of discretizing that wall and is
/// dropped; two sites belong to the same wall when a walk over the boundary
/// cells reaches one from the other within a few steps. An edge that passes
/// through a blocked cell is dropped as well, because the axis of free space
/// cannot run through an obstacle.
///
/// @param blocked The obstacle mask.
/// @returns The surviving edges, in cell coordinates.
[[nodiscard]] MQT_SCPD_GRID_EXPORT std::vector<MedialEdge> medialAxis(const BitGrid& blocked);

/// What a cell of the rasterized medial axis is.
enum class AxisCell : std::uint8_t {
  /// Not on the axis.
  None = 0,
  /// On the axis, with one or two neighbors.
  Path = 1,
  /// On the axis, where three or more arms meet.
  Junction = 2,
  /// On the axis, with exactly one neighbor: a dead end.
  Endpoint = 4,
};

/// The medial axis on the grid: which cells it covers, and how they connect.
struct MedialAxis {
  /// One entry per cell, saying what the cell is on the axis.
  std::vector<AxisCell> cells;
  /// The cells adjacent along the axis, for every cell that is on it.
  std::unordered_map<std::size_t, std::vector<std::size_t>> neighbors;

  /// Whether a cell lies on the axis.
  [[nodiscard]] bool onAxis(const std::size_t index) const {
    return index < cells.size() && cells[index] != AxisCell::None;
  }
};

/// Rasterize the medial axis onto the grid.
///
/// Each edge is walked as a Bresenham line through the cells its endpoints
/// truncate to, and a cell that carries an obstacle is never marked. Cells
/// are joined to the cell before them on the same edge, so the result is a
/// graph over the grid rather than a set of pixels; a cell with three or more
/// such neighbors is a junction and one with a single neighbor a dead end.
///
/// @throws std::invalid_argument when the mask does not fit the grid.
[[nodiscard]] MQT_SCPD_GRID_EXPORT MedialAxis
rasterizeMedialAxis(const BitGrid& blocked, const GridMetrics& grid,
                    const std::vector<MedialEdge>& edges);

} // namespace mqt::scpd::grid
