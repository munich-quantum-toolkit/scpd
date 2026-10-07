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

/// The eight steps around a cell, straight ones first: the numbering of
/// `BlockedMove::direction`.
inline constexpr std::array<int, 8> STEP_DX = {0, 0, 1, -1, 1, -1, 1, -1};
inline constexpr std::array<int, 8> STEP_DY = {1, -1, 0, 0, 1, 1, -1, -1};

/// One move a gate refuses: leaving a cell in one of the eight directions.
struct BlockedMove {
  std::size_t cell = 0;
  std::size_t direction = 0;
};

/// The moves each bottleneck refuses.
///
/// A gate is a **cut**, not a wall: it stops a wire from crossing the line
/// between its two obstacle cells, and it consumes no free space of its own.
/// What it blocks is therefore the move between two cells whose centre-to-
/// centre segment crosses it, in both directions. A cell whose own centre lies
/// on the line is enclosed entirely, because a wire there is already on the
/// wrong side of every crossing.
///
/// Blocking the line's *cells* instead would be simpler and is wrong twice
/// over: it takes free space away from the chambers on both sides, so the
/// chambers come out smaller and more numerous than they are; and a Bresenham
/// line is eight-connected, so a walk that also moves diagonally steps
/// straight through it anyway.
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

} // namespace mqt::scpd::grid
