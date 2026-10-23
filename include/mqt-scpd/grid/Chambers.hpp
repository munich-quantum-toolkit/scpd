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
#include "mqt-scpd/grid/Bottlenecks.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"
#include "mqt-scpd/grid/mqt_scpd_grid_export.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace mqt::scpd::grid {

/// The x part of the eight steps around a cell, straight steps first. The
/// index of a step is the direction of a BlockedMove.
inline constexpr std::array<int, 8> STEP_DX = {0, 0, 1, -1, 1, -1, 1, -1};
/// The y part of the eight steps around a cell, in the order of STEP_DX.
inline constexpr std::array<int, 8> STEP_DY = {1, -1, 0, 0, 1, 1, -1, -1};

/// One move a gate refuses: leaving a cell in one of the eight directions.
struct BlockedMove {
  /// The cell the move leaves.
  std::size_t cell = 0;
  /// The direction of the move, as an index into STEP_DX and STEP_DY.
  std::size_t direction = 0;
};

/**
 * @brief Lists the moves each bottleneck refuses.
 *
 * A gate is a cut and not a wall: it stops a wire from crossing the line
 * between its two obstacle cells. It refuses the move between two cells whose
 * center-to-center segment crosses the line, in both directions. A cell whose
 * own center lies on the line, other than the two ends, is enclosed: every
 * move out of it and into it is refused, because a wire there is already on
 * the wrong side of every crossing. Along an axis every cell between the two
 * ends has its center on the line, so such a gate encloses all of them; a
 * gate at another angle encloses only the cells its line meets at a center.
 *
 * Blocking every cell the line passes through instead would take more free
 * space from the chambers on both sides, and a line of cells is
 * eight-connected, so a walk that moves diagonally would step through it.
 *
 * @param bottlenecks The bottlenecks.
 * @param grid The metrics of the grid they were found on.
 * @return For each bottleneck, the moves it refuses. Every move comes with
 * its reverse.
 */
[[nodiscard]] MQT_SCPD_GRID_EXPORT std::vector<std::vector<BlockedMove>>
bottleneckMoves(const std::vector<Bottleneck>& bottlenecks,
                const GridMetrics& grid);

/// The chamber of a cell that lies in none: a blocked cell, or a cell on the
/// line of a bottleneck.
inline constexpr std::uint32_t NO_CHAMBER =
    std::numeric_limits<std::uint32_t>::max();

/// The free space of a mask, cut into chambers by bottlenecks.
struct Chambers {
  /// The chamber of every cell, or `NO_CHAMBER`.
  std::vector<std::uint32_t> of;
  /// How many chambers there are. They are numbered from zero, in the order
  /// of the lowest cell each holds.
  std::uint32_t count = 0;
  /// For every bottleneck, the chambers on either side of it, sorted. Two
  /// where it separates two chambers; one where the free space runs around
  /// it, so that it separates nothing; more where its line ends among
  /// several.
  std::vector<std::vector<std::uint32_t>> beside;
};

/// Cut the free space of a mask into chambers: the groups of free cells a
/// walk reaches from one another without crossing a bottleneck.
///
/// The walk takes the eight steps around a cell, as the capacity stage's
/// does, and refuses three kinds: a step onto a blocked cell, a diagonal
/// step between two blocked cells, which is a passage the raster invented,
/// and a step `bottleneckMoves` refuses. A cell on the line of a bottleneck
/// belongs to no chamber.
///
/// @throws std::invalid_argument when the mask does not fit the grid.
[[nodiscard]] MQT_SCPD_GRID_EXPORT Chambers
chambersOf(const BitGrid& blocked, const std::vector<Bottleneck>& bottlenecks,
           const GridMetrics& grid);

/// `chambersOf` into chambers already there, so that a caller that labels
/// one grid many times keeps the memory of the labels from call to call.
MQT_SCPD_GRID_EXPORT void chambersOf(const BitGrid& blocked,
                                     const std::vector<Bottleneck>& bottlenecks,
                                     const GridMetrics& grid, Chambers& into);

} // namespace mqt::scpd::grid
