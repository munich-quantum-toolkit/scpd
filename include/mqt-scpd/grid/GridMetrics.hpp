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
#include <utility>

namespace mqt::scpd::grid {

using flatbuffers::geometry::DCoord;
using geometry::BoundingBox;
using geometry::Point;

/**
 * @brief A grid laid over the chip: its size in cells and the layout-space box
 * it covers.
 *
 * Cell (0, 0) is centered on the minimum corner of the box, and cell
 * (@c width - 1, @c height - 1) is centered on the maximum corner. One cell
 * step is therefore the extent of the box divided by the number of steps,
 * which is one less than the number of cells. Every conversion between layout
 * units and cells goes through this type. The same type describes the
 * capacity grid, the detail grid and the router grid. The functions that
 * return a cell return a DCoord on each of these grids, so the type of a cell
 * does not say which grid the cell belongs to. The caller knows which grid a
 * GridMetrics describes.
 *
 * This is a node convention: a cell is a point, and the outermost cells sit
 * exactly on the edges of the box. A raster convention would make a cell an
 * area: cell @c k would cover [k, k + 1) of a grid @c width steps across and
 * would be tested at its center. The two conventions differ by half a cell.
 * The grid uses the node convention for two reasons. A cell of the router grid
 * is a search state and not an area. And every point of the box, its edges
 * included, rounds onto a cell.
 */
struct MQT_SCPD_GRID_EXPORT GridMetrics {
  /// The number of cells along x.
  uint32_t width = 0;
  /// The number of cells along y.
  uint32_t height = 0;
  /// The layout position of cell (0, 0).
  Point origin;
  /// The layout units per cell step along x.
  double cellWidth = 0.0;
  /// The layout units per cell step along y.
  double cellHeight = 0.0;

  /**
   * @brief Fits a grid of a given number of cells over a box.
   * @param box The box the grid covers.
   * @param width The number of cells along x.
   * @param height The number of cells along y.
   * @return The grid, with cell (0, 0) on the minimum corner of @p box and the
   * last cell on the maximum corner.
   * @throws std::invalid_argument If @p width or @p height is less than two, or
   * if @p box has no positive extent along an axis, because such a grid has no
   * cell step.
   */
  [[nodiscard]] static GridMetrics fit(const BoundingBox& box, uint32_t width,
                                       uint32_t height);

  /**
   * @brief Fits a grid over a box, with a height that follows the aspect ratio
   * of the box.
   *
   * The number of cells along y is @p width times the height of @p box divided
   * by the width of @p box, rounded to the nearest whole number and at least
   * two.
   *
   * @param box The box the grid covers.
   * @param width The number of cells along x.
   * @return The grid, as fit() returns it for the derived number of cells
   * along y.
   * @throws std::invalid_argument If @p width is less than two, or if @p box
   * has no positive extent along an axis.
   * @throws std::length_error If the number of cells along y exceeds
   * 2^32 - 1.
   */
  [[nodiscard]] static GridMetrics fitWidth(const BoundingBox& box,
                                            uint32_t width);

  /**
   * @brief Refines the grid by a whole factor over the same box.
   *
   * The refined grid has @p factor times as many cells along each axis.
   * Refining a capacity grid gives its detail grid.
   *
   * @param factor The factor by which the number of cells grows along each
   * axis.
   * @return The refined grid, as fit() returns it for the same box.
   * @throws std::invalid_argument If @p factor is zero, or if this grid has
   * fewer than two cells or no positive extent along an axis.
   * @throws std::length_error If the refined grid has more than 2^32 - 1
   * cells along an axis.
   */
  [[nodiscard]] GridMetrics refined(uint32_t factor) const;

  /**
   * @brief Counts the cells of the grid.
   * @return @c width times @c height.
   */
  [[nodiscard]] std::size_t cells() const {
    return static_cast<std::size_t>(width) * height;
  }

  /**
   * @brief Checks whether a pair of cell indices lies on the grid.
   * @param x The column, which may lie off the grid on either side.
   * @param y The row, which may lie off the grid on either side.
   * @return @c true when @p x lies in [0, @c width) and @p y lies in
   * [0, @c height).
   */
  [[nodiscard]] bool contains(int64_t x, int64_t y) const {
    return x >= 0 && y >= 0 && std::cmp_less(x, width) &&
           std::cmp_less(y, height);
  }

  /**
   * @brief Computes the row-major index of a cell.
   * @param x The column of the cell.
   * @param y The row of the cell.
   * @pre The cell lies on the grid.
   * @return @p y times @c width plus @p x.
   */
  [[nodiscard]] std::size_t index(uint32_t x, uint32_t y) const {
    return (static_cast<std::size_t>(y) * width) + x;
  }

  /**
   * @brief Computes the cell of a row-major index.
   * @param index The row-major index of the cell.
   * @pre @p index is less than cells().
   * @return The column and the row of the cell.
   */
  [[nodiscard]] DCoord cell(std::size_t index) const {
    return {static_cast<uint32_t>(index % width),
            static_cast<uint32_t>(index / width)};
  }

  /**
   * @brief Computes the box the grid covers.
   * @return The box from the layout position of cell (0, 0) to the layout
   * position of the last cell.
   */
  [[nodiscard]] BoundingBox box() const;

  /**
   * @brief Converts a layout point into cell units.
   * @param point The layout point.
   * @return The position in cell units, not rounded. Cell (x, y) sits at the
   * whole numbers @c x and @c y.
   */
  [[nodiscard]] Point toCell(Point point) const;

  /**
   * @brief Converts a position in cell units into a layout point.
   * @param x The column, which may lie between two cells.
   * @param y The row, which may lie between two cells.
   * @return The layout point of the cell, or of the position between cells.
   * The conversion is the inverse of toCell().
   */
  [[nodiscard]] Point toLayout(double x, double y) const;

  /**
   * @brief Rounds a layout point to the nearest cell.
   * @param point The layout point.
   * @return The nearest cell, or @c std::nullopt when that cell lies off the
   * grid or a coordinate of @p point is not finite. A point less than half a
   * cell step outside the box still rounds onto a cell on the edge of the grid.
   */
  [[nodiscard]] std::optional<DCoord> roundToCell(Point point) const;

  /**
   * @brief Rounds a layout point to the nearest cell and clamps that cell onto
   * the grid.
   * @param point The layout point, which may lie outside the box.
   * @return The nearest cell, with each coordinate clamped onto the grid
   * separately. An infinite coordinate clamps onto the first or the last cell
   * of its axis.
   * @throws std::invalid_argument If the grid has no cell, that is if
   * @c width or @c height is zero, or if a coordinate of @p point is not a
   * number.
   */
  [[nodiscard]] DCoord clampToCell(Point point) const;

  /**
   * @brief Compares two grids member by member.
   * @return @c true when the sizes, the origins and the cell steps are equal.
   */
  [[nodiscard]] bool operator==(const GridMetrics&) const = default;
};

/**
 * @brief Divides every cell of a capacity grid into router cells.
 *
 * The function takes the size of a capacity cell as the extent of the box
 * divided by the number of capacity cells. It divides every capacity cell into
 * as many router cells as the capacity cell holds steps of @p unitDivision
 * layout units, rounded up and at least one. The rounding has the tolerance of
 * cellsFor(), so a capacity cell that holds a whole number of steps keeps that
 * number. The function then fits the router grid over the same box with
 * GridMetrics::fit(). That function puts the outermost cells on the box, so
 * the router cell step is the extent of the box divided by one less than the
 * number of router cells. The step can therefore exceed @p unitDivision
 * slightly.
 *
 * @param capacity The capacity grid to divide.
 * @param unitDivision The router cell size to aim for, in layout units.
 * @pre @p capacity has at least two cells and a positive extent along each
 * axis, as every grid from GridMetrics::fit() has.
 * @return The router grid over the box of @p capacity.
 * @throws std::invalid_argument If @p unitDivision is not positive.
 * @throws std::length_error If the router grid has more than 2^32 - 1 cells
 * along an axis.
 */
[[nodiscard]] MQT_SCPD_GRID_EXPORT GridMetrics
routerGrid(const GridMetrics& capacity, double unitDivision);

/**
 * @brief Converts a length in layout units into a number of cells.
 *
 * Every design rule is a length in layout units, and this function is the one
 * conversion of such a length into a number of cells. It measures along the
 * axis with the smaller cell step, so the number of cells spans at least
 * @p distance along both axes.
 *
 * @param distance The length, in layout units.
 * @param grid The grid whose cell step applies.
 * @pre @p grid has a positive cell step along each axis.
 * @return The smallest number of cell steps that spans at least @p distance,
 * or zero when @p distance is not positive. A length that exceeds a whole
 * number of steps by less than a relative 1e-12 counts as that whole number,
 * so a rounding error in the cell step does not add a cell.
 * @throws std::length_error If the number of cells exceeds 2^32 - 1.
 */
[[nodiscard]] MQT_SCPD_GRID_EXPORT uint32_t cellsFor(double distance,
                                                     const GridMetrics& grid);

/**
 * @brief Computes the layout-space box of a chip.
 *
 * The box is the smallest one that contains every obstacle vertex and every
 * port center of the chip.
 *
 * @param chip The chip to cover.
 * @return The box.
 * @throws std::invalid_argument If @p chip has neither an obstacle vertex nor a
 * port.
 */
[[nodiscard]] MQT_SCPD_GRID_EXPORT BoundingBox
chipBounds(const flatbuffers::design::ChipT& chip);

} // namespace mqt::scpd::grid
