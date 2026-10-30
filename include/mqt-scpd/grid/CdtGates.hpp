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
#include "mqt-scpd/grid/mqt_scpd_grid_export.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace mqt::scpd::grid {

/// What `cdtGates` needs to know of one wall, by the label its cells carry.
struct CdtWall {
  /// The obstacle the wall belongs to: two walls of one group (the two
  /// flanks of a port run) never form a bottleneck with each other.
  std::uint32_t group = 0;
  /// A wall that stands through the insertion: the border, the artwork, a
  /// coupler pad. Two static walls never form a bottleneck.
  bool fixed = false;
  /// A flank of the run at a wire's target: it forms no bottleneck with a
  /// static wall either.
  bool target = false;
};

/// The settings of `cdtGates`.
struct CdtGateOptions {
  /// The wire clearance, in cells.
  double clearance = 1.0;
  /// A gap of this many wires or more is room, not a bottleneck.
  std::uint32_t widest = 10;
  /// The capacity margin of the slot check (`wiresIn`).
  double margin = 1.06;
  std::uint32_t marginFrom = 3;
  /// How far the outline of a wall may leave its cells when it is simplified,
  /// in cells.
  double simplify = 0.75;
  /// How far from a corner of the outline the nearest point of another wall
  /// is added to the triangulation, in clearances. Within it a gate measures
  /// the exact width of its gap.
  double footReach = 3.0;
  /// Whether the gates are canonical (`capacity_cdt_canonical`): the same
  /// for the same walls whatever order the triangulation meets their points
  /// in. Every point is moved by a deterministic amount below 1e-4 cells,
  /// drawn from its own coordinates, so that no four points are cocircular
  /// and the triangulation is unique; the widths are measured on the points
  /// as they were. Two candidates of one width are told apart by their ends,
  /// not by the order they were found in, and the gates come out sorted.
  bool canonical = false;
  /// For tests: a seed that shuffles the points before they are
  /// triangulated; 0 leaves them in the order of the outline.
  std::uint32_t shuffle = 0;
};

/// What one call of `cdtGates` came to, for the log.
struct CdtGateStats {
  std::size_t vertices = 0;
  std::size_t constraints = 0;
  std::size_t triangles = 0;
  std::size_t candidates = 0;
  std::size_t gates = 0;
  std::size_t contacts = 0;
  /// The seconds of the steps: the outline, its loops, the foot points, the
  /// triangulation, and the candidates with their basins.
  std::array<double, 5> seconds{};
};

/**
 * @brief The bottlenecks of a wall raster from one constrained Delaunay
 * triangulation of its free space.
 *
 * The outline of every wall (the edges between its cells and free cells,
 * simplified and cut to pieces no longer than a clearance) is the set of
 * constraints, with the foot points of every corner on the outlines of the
 * other walls within `footReach` clearances added. Every edge of the
 * triangulation between two triangles of the free space whose ends lie on
 * two different obstacles is a candidate, with the wires its length holds
 * across it. The candidates between one pair of obstacles form a channel, two
 * of them neighbours when they share a triangle. A candidate of one to
 * `widest - 1` wires is a gate when it is the narrowest of its basin: the
 * candidates of its channel reached from it over candidates that hold at
 * least one wire and no more than it does. So a channel between parallel
 * walls keeps one gate, and one that narrows twice keeps two. The candidates
 * of no wire close a channel where two walls almost touch; of each run of
 * them the narrowest is a gate as well. Two static walls, a target run and a
 * static wall, and two walls of one group form no candidate.
 *
 * A gate's two cells are the wall cells of its two obstacles nearest its
 * ends, as `profileCuts` gives them.
 *
 * @param owner The wall label of every cell, row-major; a free cell carries
 * `free`.
 * @param walls The walls by label. A label past its end is the border.
 * @param border The label the cells outside the grid take.
 */
[[nodiscard]] MQT_SCPD_GRID_EXPORT std::vector<Bottleneck>
cdtGates(std::uint32_t width, std::uint32_t height, const BitGrid& blocked,
         std::span<const std::uint32_t> owner, std::span<const CdtWall> walls,
         std::uint32_t border, const CdtGateOptions& options,
         CdtGateStats* stats = nullptr);

/// What `cdtGatesCached` keeps from call to call: the wall rasters of the
/// last few calls, each with the gates found on it.
struct CdtGateCache {
  struct Entry {
    /// Per cell: 0 free, 1 a static wall, 2 a wall that moves, 3 a target
    /// run.
    std::vector<std::uint8_t> kind;
    std::vector<Bottleneck> gates;
    std::uint64_t used = 0;
  };
  std::vector<Entry> entries;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  /// How many rasters it keeps.
  std::size_t capacity = 4;
  std::uint64_t clock = 0;
  /// What the calls came to: the same raster again, a window around what
  /// changed, or the whole raster.
  std::size_t reused = 0;
  std::size_t windowed = 0;
  std::size_t whole = 0;
  /// The cells of the windows triangulated, in all.
  std::uint64_t windowCells = 0;
};

/**
 * @brief `cdtGates`, incrementally: against the raster of an earlier call
 * that differs least from this one, only a window around the cells that
 * differ is triangulated again.
 *
 * The cells that differ span a box; the inner box is that box grown by the
 * widest gap (`widest` wires of clearance with the margin), and the window
 * the inner box grown by as much again. The gates whose middle lies in the
 * inner box are those of the window, the others those of the earlier call.
 * Where nothing differs the earlier gates are the answer; where the window
 * would cover most of the grid, the whole raster is triangulated.
 */
[[nodiscard]] MQT_SCPD_GRID_EXPORT std::vector<Bottleneck>
cdtGatesCached(std::uint32_t width, std::uint32_t height, const BitGrid& blocked,
               std::span<const std::uint32_t> owner, std::span<const CdtWall> walls,
               std::uint32_t border, const CdtGateOptions& options,
               CdtGateCache& cache, CdtGateStats* stats = nullptr);

} // namespace mqt::scpd::grid
