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
#include <unordered_map>
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

/// `bottleneckMoves` for one bottleneck, into a list already there.
MQT_SCPD_GRID_EXPORT void bottleneckMovesOf(const Bottleneck& bottleneck,
                                            const GridMetrics& grid,
                                            std::vector<BlockedMove>& into);

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

/// What `chambersOfCached` keeps from one labelling of a grid to the next:
/// the moves of the gates it has seen lately, and the steps the gates of the
/// last labelling refuse out of each cell.
struct ChamberCache {
  /// The moves of one gate, and the box of their cells.
  struct Entry {
    std::vector<BlockedMove> moves;
    std::int64_t minX = 0;
    std::int64_t minY = 0;
    std::int64_t maxX = -1;
    std::int64_t maxY = -1;
    /// The call that last used the gate.
    std::uint64_t used = 0;
  };
  /// The gates by their two ends.
  std::unordered_map<std::uint64_t, Entry> entries;
  /// The gates whose moves `refused` holds, sorted.
  std::vector<std::uint64_t> applied;
  /// The directions the applied gates refuse out of each cell, one bit each,
  /// and the cells with any as a bit each.
  std::vector<std::uint8_t> refused;
  std::vector<std::uint64_t> refusedBits;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  /// The number of calls so far.
  std::uint64_t clock = 0;
  /// How many calls ago a gate may last have been used and still be kept.
  std::uint64_t keep = 4;
  /// The gates whose moves were taken from the cache and computed, and the
  /// gates added to and taken from the refused steps, over all calls.
  std::size_t reused = 0;
  std::size_t computed = 0;
  std::size_t added = 0;
  std::size_t removed = 0;
};

/// The refused steps of `cache` brought from the gates of its last call to
/// `bottlenecks`, as `chambersOfCached` does before it labels, and the moves
/// of every bottleneck, in order, pointing into the cache: they stay valid
/// until the next call.
///
/// @throws std::invalid_argument when the cache's grid does not fit `grid`
/// after a reset.
MQT_SCPD_GRID_EXPORT void
chamberCacheUpdate(const std::vector<Bottleneck>& bottlenecks,
                   const GridMetrics& grid, ChamberCache& cache,
                   std::vector<const std::vector<BlockedMove>*>& moves);

/// `chambersOf` with the moves of the gates and the refused steps kept from
/// call to call: a gate already seen is not traced again, and only the steps
/// of the gates that came or went since the last call are changed. The
/// chambers are those `chambersOf` gives, cell for cell, in the same order.
///
/// @throws std::invalid_argument when the mask does not fit the grid.
MQT_SCPD_GRID_EXPORT void
chambersOfCached(const BitGrid& blocked,
                 const std::vector<Bottleneck>& bottlenecks,
                 const GridMetrics& grid, Chambers& into, ChamberCache& cache);

} // namespace mqt::scpd::grid
