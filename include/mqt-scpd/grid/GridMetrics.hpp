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
#include "mqt-scpd/geometry/Geometry.hpp"
#include "mqt-scpd/grid/mqt_scpd_grid_export.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>

namespace mqt::scpd::grid {

using geometry::BoundingBox;
using geometry::Point;
using flatbuffers::geometry::DCoord;
using flatbuffers::geometry::GCoord;
using flatbuffers::geometry::RCoord;

/// A grid laid over the chip: its size in cells and the layout-space box it
/// covers.
///
/// Cell (0, 0) is centered on the minimum corner of the box and cell
/// (width - 1, height - 1) on the maximum corner, so every point of the box
/// rounds to a cell of the grid, and one cell step is the extent of the box
/// divided by the number of steps. Every conversion between layout units and
/// cells goes through this type. The same type describes the capacity grid,
/// the detail grid and the router grid; the coordinate structs of the schema
/// say which grid a cell belongs to.
struct MQT_SCPD_GRID_EXPORT GridMetrics {
  uint32_t width = 0;
  uint32_t height = 0;
  /// The layout position of cell (0, 0).
  Point origin{};
  /// Layout units per cell step along each axis.
  double cellWidth = 0.0;
  double cellHeight = 0.0;

  /// A grid of width by height cells over the box.
  ///
  /// @throws std::invalid_argument when a count is below two or the box has
  /// no extent along an axis, because such a grid has no cell step.
  [[nodiscard]] static GridMetrics fit(const BoundingBox& box, uint32_t width,
                                       uint32_t height);

  /// A grid of width cells across the box whose height follows the aspect
  /// ratio of the box, as the capacity grid does when the configuration
  /// gives no height.
  [[nodiscard]] static GridMetrics fitWidth(const BoundingBox& box,
                                            uint32_t width);

  /// The grid with factor times as many cells along each axis over the same
  /// box: the detail grid of a capacity grid.
  [[nodiscard]] GridMetrics refined(uint32_t factor) const;

  /// The number of cells.
  [[nodiscard]] std::size_t cells() const {
    return static_cast<std::size_t>(width) * height;
  }

  /// Whether a cell index pair lies on the grid.
  [[nodiscard]] bool contains(int64_t x, int64_t y) const {
    return x >= 0 && y >= 0 && x < static_cast<int64_t>(width) &&
           y < static_cast<int64_t>(height);
  }

  /// The row-major index of a cell.
  [[nodiscard]] std::size_t index(uint32_t x, uint32_t y) const {
    return (static_cast<std::size_t>(y) * width) + x;
  }

  /// The cell of a row-major index.
  [[nodiscard]] DCoord cell(std::size_t index) const {
    return {static_cast<uint32_t>(index % width),
            static_cast<uint32_t>(index / width)};
  }

  /// The box the grid covers.
  [[nodiscard]] BoundingBox box() const;

  /// A layout point in cell units, not rounded.
  [[nodiscard]] Point toCell(Point point) const;

  /// The layout point of a cell, or of a fractional position between cells.
  [[nodiscard]] Point toLayout(double x, double y) const;

  /// The cell a layout point rounds to, or nothing when the point lies
  /// outside the box.
  [[nodiscard]] std::optional<DCoord> roundToCell(Point point) const;

  /// The cell a layout point rounds to, clamped onto the grid.
  [[nodiscard]] DCoord clampToCell(Point point) const;

  [[nodiscard]] bool operator==(const GridMetrics&) const = default;
};

/// The router grid of a capacity grid: every capacity cell is divided into as
/// many router cells as it holds steps of about unitDivision layout units,
/// rounded up, so a router cell is at most unitDivision units wide.
///
/// @throws std::invalid_argument when unitDivision is not positive.
[[nodiscard]] MQT_SCPD_GRID_EXPORT GridMetrics
routerGrid(const GridMetrics& capacity, double unitDivision);

/// Cells on the grid that span at least distance layout units, along the axis
/// with the smaller cell step. The one conversion from a design rule, which
/// is a length in layout units, to a cell count.
[[nodiscard]] MQT_SCPD_GRID_EXPORT uint32_t cellsFor(double distance,
                                                     const GridMetrics& grid);

/// The smallest box containing every obstacle vertex and every port center of
/// the chip: the box every grid of a run covers.
///
/// @throws std::invalid_argument when the chip has neither vertices nor
/// ports.
[[nodiscard]] MQT_SCPD_GRID_EXPORT BoundingBox
chipBounds(const flatbuffers::design::ChipT& chip);

} // namespace mqt::scpd::grid
