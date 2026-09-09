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

/// The unit step a port's orientation takes on the grid: each component is
/// -1, 0 or 1.
struct Step {
  int8_t x = 0;
  int8_t y = 0;
  [[nodiscard]] bool diagonal() const { return x != 0 && y != 0; }
  [[nodiscard]] bool operator==(const Step&) const = default;
};

/// The step of an orientation in degrees, as the chip input carries it: the
/// signs of its cosine and sine, where a component within a few hundredths
/// of zero counts as zero.
[[nodiscard]] MQT_SCPD_GRID_EXPORT Step orientationStep(double orientationDegrees);

/// The half width in cells of a band that keeps a wire spacing free on both
/// sides of a port: the spacing rounded up to an odd number of cells, halved.
/// A diagonal band, whose width runs along the other diagonal, is narrower by
/// the square root of two.
[[nodiscard]] MQT_SCPD_GRID_EXPORT uint32_t bandHalfWidth(double spacing,
                                                          const GridMetrics& grid,
                                                          bool diagonal);

/// A length in layout units as whole cells along a step: shorter by the
/// square root of two along a diagonal step.
[[nodiscard]] MQT_SCPD_GRID_EXPORT uint32_t bandLength(double length,
                                                       const GridMetrics& grid,
                                                       bool diagonal);

/// The keep-out band of a port: a strip perpendicular to the direction the
/// wire leaves the port, extended forward along that direction and backward
/// against it, so that no other wire runs through the port's approach.
struct PortBand {
  /// The cell of the port.
  DCoord center{0, 0};
  /// The direction the wire leaves the port.
  Step step;
  /// Cells the band extends beyond the center along the step.
  uint32_t forward = 0;
  /// Cells the band extends against the step.
  uint32_t backward = 0;
  uint32_t halfWidth = 0;
};

/// What stamping a band changed.
struct StampedBand {
  /// The cells the band covers, on the grid, without duplicates.
  std::vector<std::size_t> cells;
  /// Whether each of those cells was blocked before the band.
  std::vector<bool> wasBlocked;
  /// The center of the last strip along the step.
  DCoord lastForwardCenter{0, 0};
};

/// Block the cells of a band. A diagonal band also blocks the cell that
/// joins two diagonally adjacent strips, so that the band is four-connected
/// and has no gaps.
[[nodiscard]] MQT_SCPD_GRID_EXPORT StampedBand stampBand(BitGrid& mask,
                                                         const PortBand& band);

/// Undo a stamped band: every cell returns to what it was.
MQT_SCPD_GRID_EXPORT void unstampBand(BitGrid& mask, const StampedBand& band);

/// The cell where a wire meets a port's band, for the router grid.
///
/// The target is the first cell beyond the band along the step. From there
/// the walk continues along the step, freeing every cell of the band it
/// crosses, until it leaves the band's own cells; that cell and its eight
/// neighbors are freed and the cell becomes the target. When the walk leaves
/// the grid instead, the nearest free cell to the ideal target, by breadth
/// first search over the eight-neighborhood, is the target.
///
/// @returns The target cell, or nothing when the grid has no free cell.
[[nodiscard]] MQT_SCPD_GRID_EXPORT std::optional<std::size_t>
digTargetBeyondBand(BitGrid& mask, const StampedBand& band, Step step);

/// The cell where a wire meets a port's band, for the capacity grid, which
/// keeps the obstacles that were there before the bands.
///
/// The ideal target is the first cell beyond the band along the step. When
/// that cell was free before the bands, it is freed and taken. When it was
/// blocked, the band is undone, and the walk along the step looks for the
/// first cell, or eight-neighbor of a cell, that was free before the bands;
/// failing that, the nearest such cell by breadth-first search. The chosen
/// cell is freed.
///
/// @param before The mask before any band of this pass.
/// @returns The target cell, or nothing when the grid has no free cell.
[[nodiscard]] MQT_SCPD_GRID_EXPORT std::optional<std::size_t>
placeTargetBeyondBand(BitGrid& mask, const BitGrid& before,
                      const StampedBand& band, Step step);

/// Block the squares a launcher sweeps along its orientation on the capacity
/// grid, and free the cell beyond the sweep as the launcher's slot.
///
/// @param length Cells the sweep extends along the step.
/// @param halfWidth The half side of the swept square.
/// @returns The slot cell, clamped onto the grid.
[[nodiscard]] MQT_SCPD_GRID_EXPORT std::size_t
stampLauncherSweep(BitGrid& mask, DCoord center, Step step, uint32_t length,
                   uint32_t halfWidth);

} // namespace mqt::scpd::grid
