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

#include "mqt-scpd/geometry/Geometry.hpp"
#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"
#include "mqt-scpd/grid/Watershed.hpp"
#include "mqt-scpd/grid/mqt_scpd_grid_export.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace mqt::scpd::grid {

using geometry::Point;

/// The outline of one partition, in cell coordinates. A partition may have
/// more than one, when the watershed left it in pieces or with a hole.
struct PartitionOutline {
  PartitionLabel label = LABEL_NONE;
  /// The corners of the ring, closed: the last point is the first.
  std::vector<Point> ring;
};

/// Where two partitions meet.
///
/// The border is what the capacity model budgets: every wire that leaves one
/// partition for the other crosses it, so its length in layout units decides
/// how many may. The samples are the individual cell edges it is made of,
/// kept so that a renderer can draw the border as it actually runs rather
/// than as a straight line between two partitions.
struct PartitionBorder {
  PartitionLabel first = LABEL_NONE;
  PartitionLabel second = LABEL_NONE;
  /// The cell edges the border consists of. Its length in cells is their
  /// count.
  std::vector<Point> samples;

  /// The mean of the samples: one point that stands for the border.
  [[nodiscard]] MQT_SCPD_GRID_EXPORT Point center() const;
};

/// The geometry the watershed labels induce.
struct Partitions {
  std::vector<PartitionOutline> outlines;
  std::vector<PartitionBorder> borders;
  /// The distinct border sample points, which are the candidate positions the
  /// global router builds its lattice from.
  std::vector<Point> lattice;
};

/// Trace the partition geometry out of a labeled grid.
///
/// A cell contributes an outline edge wherever its neighbor carries another
/// label, and a border sample wherever that neighbor carries another *real*
/// label rather than an obstacle. The outlines are then assembled by walking
/// the edges of each label until they close, which is why a partition with a
/// hole yields two rings rather than one bad one.
///
/// Coordinates are in cells, with the corner of cell `(0, 0)` at the origin,
/// so a border sample at `(4.5, 7.0)` is on the edge between the cells
/// `(4, 6)` and `(4, 7)`. `toLayout` on the grid converts them.
///
/// @throws std::invalid_argument when the labels or the mask do not fit the
/// grid.
[[nodiscard]] MQT_SCPD_GRID_EXPORT Partitions
extractPartitions(const BitGrid& blocked, std::span<const PartitionLabel> labels,
                  const GridMetrics& grid);

/// The seed cells of the watershed: the center of every capacity cell that is
/// entirely free.
///
/// A cell with an obstacle in it is not a place a partition should grow from,
/// because the partition would then straddle the obstacle. Taking only the
/// clear ones puts one seed in the middle of every clear region and none
/// anywhere else, and taking them in row-major order makes the labels the
/// same on every run.
///
/// @param detail The detail grid the mask is on.
/// @param coarse The capacity grid whose cells are tested.
/// @throws std::invalid_argument when the detail grid is not a whole refinement
/// of the coarse one.
[[nodiscard]] MQT_SCPD_GRID_EXPORT std::vector<std::size_t>
freeCellSeeds(const BitGrid& blocked, const GridMetrics& detail, const GridMetrics& coarse);

/// How many wires may cross each edge of a capacity cell.
struct CellCapacity {
  std::uint16_t south = 0;
  std::uint16_t north = 0;
  std::uint16_t west = 0;
  std::uint16_t east = 0;
};

/// The wire budget of every capacity cell's four edges.
///
/// An edge's budget is what its free fraction allows: the cell's extent
/// divided by the pitch a wire occupies, scaled by how much of that edge is
/// not obstructed. It is a coarse figure on purpose — the detail router
/// decides where a wire actually goes, and this only has to keep the
/// assignment from committing more wires to a corridor than it can hold.
///
/// @param wirePitch The layout distance one wire occupies across a corridor.
/// @throws std::invalid_argument when the detail grid is not a whole refinement
/// of the coarse one, or the pitch is not positive.
[[nodiscard]] MQT_SCPD_GRID_EXPORT std::vector<CellCapacity>
cellCapacities(const BitGrid& blocked, const GridMetrics& detail, const GridMetrics& coarse,
               double wirePitch);

} // namespace mqt::scpd::grid
