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
#include "mqt-scpd/grid/mqt_scpd_grid_export.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace mqt::scpd::grid {

/// The squared distance of a cell on a grid without any blocked cell.
inline constexpr uint32_t DISTANCE_UNBOUNDED = 1'000'000;

/// The squared Euclidean distance, in cells, of every cell to the nearest
/// blocked cell. Blocked cells hold zero. The transform is exact and linear
/// in the number of cells: a pass along the rows finds the nearest blocked
/// cell in each row, and a pass down the columns takes the lower envelope of
/// the rows' parabolas (Felzenszwalb and Huttenlocher). A cell whose row has
/// no blocked cell holds at most `DISTANCE_UNBOUNDED`.
[[nodiscard]] MQT_SCPD_GRID_EXPORT std::vector<uint32_t>
squaredDistanceTransform(const BitGrid& blocked);

/// A cell with no blocked cell anywhere on the grid.
inline constexpr std::size_t NO_NEAREST = static_cast<std::size_t>(-1);

/// The distance transform with the cell it measures to: for every cell, the
/// squared Euclidean distance to the nearest blocked cell and that cell.
struct NearestBlocked {
  /// The squared distance, as `squaredDistanceTransform` gives it.
  std::vector<uint32_t> squared;
  /// The nearest blocked cell, or `NO_NEAREST`. Of two cells at the same
  /// distance, either may be named.
  std::vector<std::size_t> cell;
};

/// The exact distance to the nearest blocked cell, and which cell it is (a
/// feature transform), in time linear in the number of cells: the passes of
/// `squaredDistanceTransform`, each carrying the cell its minimum comes
/// from. Unlike that transform, a cell whose row has no blocked cell is
/// measured exactly as well.
[[nodiscard]] MQT_SCPD_GRID_EXPORT NearestBlocked
nearestBlocked(const BitGrid& blocked);

} // namespace mqt::scpd::grid
