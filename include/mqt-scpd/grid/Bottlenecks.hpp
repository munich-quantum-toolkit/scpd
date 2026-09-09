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
#include "mqt-scpd/grid/GridMetrics.hpp"
#include "mqt-scpd/grid/Voronoi.hpp"
#include "mqt-scpd/grid/mqt_scpd_grid_export.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace mqt::scpd::grid {

/// The narrowest place in a corridor: the two obstacle cells that face each
/// other across it, and the point on the medial axis between them.
///
/// A bottleneck is a line, and how many wires fit through the corridor is
/// what that line's length allows. Every later stage treats it as the gate a
/// wire has to pass, which is what makes the capacity model a tree of gates
/// rather than a distance.
struct Bottleneck {
  /// The obstacle cell on one side.
  std::size_t first = 0;
  /// The obstacle cell on the other.
  std::size_t second = 0;
  /// The cell of the medial axis the two were found from.
  std::size_t saddle = 0;
};

/// How bottlenecks are found.
struct BottleneckOptions {
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
  /// Cells that no bottleneck may cross, because a port is not a wall that
  /// capacity divides around. The targets of the run.
  std::span<const std::size_t> targets;
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
findBottlenecks(const BitGrid& blocked, const MedialAxis& axis,
                std::span<const std::uint32_t> squaredDistance, const GridMetrics& grid,
                const BottleneckOptions& options = {});

/// How many wires fit through a bottleneck.
///
/// The gap is the length of the bottleneck in layout units, and a wire needs
/// its own width plus the clearance to the next one. A gap that does not even
/// reach one wire spacing carries nothing.
///
/// @param wireSpacing The clearance two wires must keep.
/// @param obstacleSpacing The clearance a wire must keep to an obstacle.
/// @param roundDown Whether the count rounds down rather than up, which it
/// does when either end of the bottleneck sits on a cell reserved for a port
/// rather than on chip artwork.
[[nodiscard]] MQT_SCPD_GRID_EXPORT std::uint32_t
bottleneckCapacity(const Bottleneck& bottleneck, const GridMetrics& grid, double wireSpacing,
                   double obstacleSpacing, bool roundDown);

} // namespace mqt::scpd::grid
