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

#include "mqt-scpd/flatbuffers/geometry.hpp"
#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"
#include "mqt-scpd/grid/mqt_scpd_grid_export.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace mqt::scpd::grid {

/// The unit step of a port orientation on the grid. Each component is -1, 0
/// or 1.
struct Step {
  /// The step along x: -1, 0 or 1.
  int x = 0;
  /// The step along y: -1, 0 or 1.
  int y = 0;

  /**
   * @brief Checks whether the step runs along a diagonal.
   * @return @c true when both components are not zero.
   */
  [[nodiscard]] bool diagonal() const { return x != 0 && y != 0; }

  /**
   * @brief Compares two steps component by component.
   * @return @c true when both components are equal.
   */
  [[nodiscard]] bool operator==(const Step&) const = default;
};

/**
 * @brief Returns the step of an orientation.
 * @param orientationDegrees The orientation in degrees, as the chip input
 * carries it.
 * @return The signs of the cosine and the sine of the orientation, where a
 * component within 0.05 of zero counts as zero.
 */
[[nodiscard]] MQT_SCPD_GRID_EXPORT Step
orientationStep(double orientationDegrees);

/**
 * @brief Returns the half width of a band that keeps a spacing free on both
 * sides of a port.
 *
 * The spacing runs across the band. It is rounded up to whole cells and to
 * an odd count, so that the band is centered on the cell of the port, and then
 * halved. A diagonal band, whose width runs along the other diagonal, is
 * narrower by the square root of two. On a grid whose cells are not square,
 * the spacing is counted in the cell step across the band.
 *
 * @param spacing The spacing, in layout units.
 * @param grid The grid, whose cell steps the spacing is counted in.
 * @param step The step the band runs along.
 * @return The half width, in cells, or zero for a zero step.
 */
[[nodiscard]] MQT_SCPD_GRID_EXPORT uint32_t
bandHalfWidth(double spacing, const GridMetrics& grid, Step step);

/**
 * @brief Returns a length as whole steps along a band.
 * @param length The length, in layout units.
 * @param grid The grid, whose cell steps give the length of one step.
 * @param step The step, whose length in layout units counts both of its
 * components.
 * @return The length in whole steps, rounded down, or zero for a zero step.
 */
[[nodiscard]] MQT_SCPD_GRID_EXPORT uint32_t bandLength(double length,
                                                       const GridMetrics& grid,
                                                       Step step);

/// The keep-out band of a port: a strip across the direction the wire leaves
/// the port, repeated forward along that direction and backward against it,
/// so that no other wire runs through the approach of the port.
struct PortBand {
  /// The cell of the port.
  DCoord center{0, 0};
  /// The direction the wire leaves the port.
  Step step;
  /// The cells the band reaches beyond the center along the step.
  uint32_t forward = 0;
  /// The cells the band reaches against the step.
  uint32_t backward = 0;
  /// The cells each strip reaches to either side of its center.
  uint32_t halfWidth = 0;
};

/// What stamping a band changed.
struct StampedBand {
  /// The cells the band covers, without duplicates.
  std::vector<std::size_t> cells;
  /// Whether each of those cells was blocked before the band.
  std::vector<bool> wasBlocked;
  /// The center of the last strip along the step, clamped onto the grid.
  DCoord lastForwardCenter{0, 0};
  /// Whether that center lies on the grid without the clamping.
  bool lastForwardOnGrid = true;
};

/**
 * @brief Blocks the cells of a band.
 *
 * A diagonal band also blocks the cell that joins two diagonally adjacent
 * strips, so that the band is four-connected and has no gaps.
 *
 * @param mask The mask the band is stamped into.
 * @param band The band.
 * @return What the band changed.
 */
[[nodiscard]] MQT_SCPD_GRID_EXPORT StampedBand stampBand(BitGrid& mask,
                                                         const PortBand& band);

/**
 * @brief Places the cell where a wire meets the band of its port, on the
 * grid of @p mask.
 *
 * The ideal target is the first cell beyond the band along the step. When that
 * cell lies on the grid and was free before the bands, it is freed and taken.
 * When it lies off the grid or was blocked,
 * the band is undone, and a walk along the step takes the first cell, or the
 * first eight-neighbor of a cell, that was free before the bands. When the
 * walk leaves the grid first, the nearest such cell by breadth-first search
 * from the ideal target is taken. The target is freed.
 *
 * @param mask The mask with the band stamped in.
 * @param before The mask before any band of the pass.
 * @param band The stamped band.
 * @param step The direction the wire leaves the port.
 * @return The target cell, or nothing when no cell was free before the bands.
 */
[[nodiscard]] MQT_SCPD_GRID_EXPORT std::optional<std::size_t>
placeTargetBeyondBand(BitGrid& mask, const BitGrid& before,
                      const StampedBand& band, Step step);

} // namespace mqt::scpd::grid
