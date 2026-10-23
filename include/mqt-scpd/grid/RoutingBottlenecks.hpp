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
#include "mqt-scpd/grid/Voronoi.hpp"
#include "mqt-scpd/grid/mqt_scpd_grid_export.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace mqt::scpd::grid {

/// How bottlenecks are found.
struct RoutingBottleneckOptions {
  /// A cell of the axis is a candidate only when its squared distance to the
  /// nearest wall, in cells, is below this. A place with room to spare is
  /// not a bottleneck however local a minimum it is.
  ///
  /// The value is in the squared cells the distance transform produces, so
  /// the default of 150 admits a clearance below about 12.2 cells.
  std::uint32_t maximumSquaredClearance = 150;
  /// How far apart two cuts of one narrowing may end and still be one gate,
  /// in layout units.
  ///
  /// The saddle search reports a candidate wherever the clearance stops
  /// falling, so a narrowing whose clearance is flat over a few cells comes
  /// back as several lines that share a wall and end a cell or two apart on
  /// the other. They are one place, and the model reads them as several
  /// corridors: a chamber that two of them lead out of is credited with twice
  /// the room the chip has. The wire pitch is the length to give here, being
  /// the distance in which the count of wires can change at all.
  ///
  /// Zero keeps every candidate the search found.
  double sameNarrowing = 0.0;
  /// How far the clearance must rise on both sides of a candidate, in
  /// cells, before it falls below the candidate's own.
  ///
  /// A wall that does not run along an axis is a staircase on the raster,
  /// and every step of it is a local minimum of the clearance a cell deep.
  /// On a whole chip those steps outnumber the narrowings by far. Walking
  /// the axis away from a candidate, a side counts once the clearance has
  /// risen by this much; it fails where it first falls below the
  /// candidate's, because then a lower minimum stands for the same
  /// narrowing, and where the axis forks or ends before either. This is
  /// the persistence of the minimum. Where the clearance holds its minimum
  /// over a stretch of the axis, as between two parallel walls, the
  /// stretch is one narrowing and one cut is kept, from its middle.
  ///
  /// Zero keeps every local minimum and every cell of such a stretch.
  double minimumRise = 0.0;
  /// Cells that no bottleneck may cross, because a port is not a wall that
  /// capacity divides around. The targets of the run.
  std::span<const std::size_t> targets;
  /// The free cells of the slots terminals lie in: a slot is a notch in the
  /// wall a terminal stands in front of, open towards the free space, and
  /// the medial axis runs into it. Walking the axis away from a candidate,
  /// a fork one of whose arms runs into a slot counts as the rise: the
  /// narrowing in front of a terminal ends at its slot, so it splits into
  /// one on either side of the terminal. Every other arm of that fork, up
  /// to the next fork, is cut at its narrowest cell when no cut of the
  /// search lies on it — next to the fork where the fork itself is the
  /// narrowest — so that every side of a terminal is closed. No bottleneck
  /// crosses a slot.
  std::span<const std::size_t> slots;
  /// The wall every blocked cell belongs to, by a label of the caller's, or
  /// empty. Two cuts are one narrowing (`sameNarrowing`) only when their
  /// ends lie on the same two walls, so the two flanks of a slot keep a
  /// cut each; and two cuts between the same two walls whose ends lie
  /// within `sameNarrowing` of each other on both are one narrowing even
  /// where they share no cell.
  std::span<const std::uint32_t> wallOf;
};

/// The bottlenecks of a rasterized chip.
///
/// A candidate is a cell of the medial axis whose clearance is a local
/// minimum along the axis, or the edge of a plateau of one. From it the free
/// space splits into shores — the connected groups of off-axis neighbors —
/// and a bottleneck exists where there are two. Each shore is then walked
/// downhill in the distance transform until it reaches a wall, and the two
/// walls it reaches are the ends of the bottleneck.
///
/// The shores are kept apart during the walk, so the two ends are always on
/// different sides of the corridor. Without that the descent from one shore
/// can round a corner and land on the wall the other shore came from, which
/// yields a line along a corridor rather than across it.
///
/// @param blocked The obstacle mask.
/// @param axis The rasterized medial axis of the same mask.
/// @param squaredDistance The squared distance transform of the same mask.
/// @throws std::invalid_argument when the three do not describe one grid.
[[nodiscard]] MQT_SCPD_GRID_EXPORT std::vector<Bottleneck>
findRoutingBottlenecks(const BitGrid& blocked, const MedialAxis& axis,
                       std::span<const std::uint32_t> squaredDistance,
                       const GridMetrics& grid,
                       const RoutingBottleneckOptions& options = {});

/// Which bottlenecks stay when none may meet another in the middle of either
/// (user, 2026-10-14). Two meet so when their lines cross more than `near`
/// cells from the ends of both, or when one ends within `near` cells of the
/// other's middle: on the raster they share cells or run a cell apart, and
/// the chambers they cut lie beside both, so that a wire passes the one
/// through the other unpaid. The one of the lower `rank` goes; of equal rank
/// the longer, and of equal length the later. Each bottleneck is set against
/// the ones that stay, in that order, so one that meets only a bottleneck
/// that went itself stays.
///
/// @param rank One rank per bottleneck; the higher stays over the lower.
/// @param width The width of the grid the cells index.
/// @param near How many cells from an end the middle of a line begins.
/// @return Per bottleneck, whether it stays.
/// @throws std::invalid_argument when `rank` has not one entry per
/// bottleneck.
[[nodiscard]] MQT_SCPD_GRID_EXPORT std::vector<bool>
uncrossedBottlenecks(const std::vector<Bottleneck>& bottlenecks,
                     std::span<const std::uint32_t> rank, std::uint32_t width,
                     double near = 1.5);

} // namespace mqt::scpd::grid
