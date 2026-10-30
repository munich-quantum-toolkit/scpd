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

#include "mqt-scpd/grid/mqt_scpd_grid_export.hpp"

#include <array>
#include <cstdint>
#include <utility>
#include <vector>

namespace mqt::scpd::grid {

/// The offsets of one disc of cells, and what enters it on each step.
///
/// Charging the disc around every cell of a path costs the disc for every
/// cell, and a disc of nineteen cells holds eleven hundred of them. Two
/// consecutive cells of a path differ by one step, so all but a leading edge
/// of the disc is already charged: the full disc is stamped once, at the first
/// cell, and one edge per step after that.
struct Stencil {
  using Offsets = std::vector<std::pair<std::int32_t, std::int32_t>>;
  /// Every offset of the disc.
  Offsets full;
  /// The offsets that enter the disc on a step, indexed `[dy + 1][dx + 1]`.
  std::array<std::array<Offsets, 3>, 3> edge;
};

/**
 * @brief The disc of the offsets within a distance of the centre.
 * @param reach How far the disc may reach along an axis, in whole cells.
 * @param radius The distance, in cells. An offset at exactly this distance is
 * in the disc. Not above `reach`.
 * @return The offsets in the square `reach` cells to either side whose
 * distance from the centre is at most `radius`, and the edge of every step.
 */
[[nodiscard]] MQT_SCPD_GRID_EXPORT Stencil stencilOf(std::uint32_t reach,
                                                     double radius);

} // namespace mqt::scpd::grid
