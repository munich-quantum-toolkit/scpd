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
#include <span>
#include <vector>

namespace mqt::scpd::grid {

/// One outline of a partition, in cell coordinates. A partition has more than
/// one when it has a hole or falls apart into pieces.
struct PartitionOutline {
  /// The label of the partition.
  PartitionLabel label = LABEL_NONE;
  /// The corners of the ring, closed: the last point is the first.
  std::vector<Point> ring;
};

/**
 * @brief Where two partitions meet.
 *
 * Every wire that leaves one partition for the other crosses the border, so
 * its length decides how many may. The samples are the cell edges the border
 * is made of, so that a picture can draw the border as it runs.
 */
struct PartitionBorder {
  /// The lower of the two labels.
  PartitionLabel first = LABEL_NONE;
  /// The higher of the two labels.
  PartitionLabel second = LABEL_NONE;
  /// The cell edges of the border, as their midpoints in cell coordinates.
  /// The length of the border in cells is their count.
  std::vector<Point> samples;

  /**
   * @brief Returns one point that stands for the border.
   * @return The mean of the samples, or the origin when there is none.
   */
  [[nodiscard]] MQT_SCPD_GRID_EXPORT Point center() const;
};

/// The geometry that the labels of a grid induce.
struct Partitions {
  /// The outlines, grouped by label in ascending order.
  std::vector<PartitionOutline> outlines;
  /// The borders, ordered by their pair of labels.
  std::vector<PartitionBorder> borders;
};

/**
 * @brief Traces the partitions out of a labeled grid.
 *
 * A cell contributes an outline edge wherever its neighbor carries another
 * label, and a border sample wherever that neighbor carries another label of a
 * partition rather than an obstacle. The outline edges of a label are joined
 * into rings by walking them until they close, so a partition with a hole
 * gives two rings.
 *
 * Coordinates are in cells, with the corner of cell (0, 0) at the origin: a
 * border sample at (4.5, 7.0) lies on the edge between the cells (4, 6) and
 * (4, 7). GridMetrics::toLayout() converts them.
 *
 * @param blocked The obstacle mask. A blocked cell counts as no label.
 * @param labels The label of every cell, row-major. A label below
 * FIRST_PARTITION_LABEL counts as no label, so a reserved cell is part of no
 * partition.
 * @param grid The metrics of the grid.
 * @return The outlines and the borders.
 * @throws std::invalid_argument When the mask or the labels do not have the
 * size of the grid.
 */
[[nodiscard]] MQT_SCPD_GRID_EXPORT Partitions extractPartitions(
    const BitGrid& blocked, std::span<const PartitionLabel> labels,
    const GridMetrics& grid);

/**
 * @brief Returns the seed cells of the watershed.
 *
 * The detail grid is cut into blocks of factor by factor cells, one block per
 * capacity cell, in row-major order. A block sits close to its capacity cell
 * but not on it, because a refined grid does not nest in the grid it refines
 * (see GridMetrics::refined()); the blocks still tile the detail grid
 * exactly. A block whose cells are all free contributes its middle cell, so
 * every clear region gets a seed in its middle and no seed sits next to an
 * obstacle. The row-major order keeps the labels the same on every run.
 *
 * @param blocked The obstacle mask, on the detail grid.
 * @param detail The detail grid.
 * @param coarse The capacity grid.
 * @return The seed cells, as indices of the detail grid.
 * @throws std::invalid_argument When the detail grid does not have a whole
 * number of cells per capacity cell along each axis, or when the mask does not
 * have the size of the detail grid.
 */
[[nodiscard]] MQT_SCPD_GRID_EXPORT std::vector<std::size_t>
freeCellSeeds(const BitGrid& blocked, const GridMetrics& detail,
              const GridMetrics& coarse);

/**
 * @brief Fills the cells of every partition back in from its outlines.
 *
 * A label grid is derived state and no artifact carries it, but a stage that
 * routes inside a partition needs one. The outlines run along cell edges, and
 * a hole is a ring of its own, so filling the rings of one label by the
 * even-odd rule gives the cells the label had.
 *
 * @param outlines The rings, in cell coordinates, as extractPartitions()
 * returns them.
 * @param grid The metrics of the grid.
 * @return The label of every cell, row-major. A cell that no ring encloses
 * keeps LABEL_NONE.
 */
[[nodiscard]] MQT_SCPD_GRID_EXPORT std::vector<PartitionLabel>
rasterizePartitions(std::span<const PartitionOutline> outlines,
                    const GridMetrics& grid);

/**
 * @brief Returns the crossing slots of a border: the places a wire may cross
 * it.
 *
 * Two wires that cross one border keep a spacing apart, so the border is
 * walked from sample to nearest sample and a sample becomes a slot once it
 * lies one spacing past the last slot. A border in several pieces starts a
 * new slot on each piece, and a border shorter than one spacing still carries
 * one wire, so the first sample is always a slot.
 *
 * @param border The border.
 * @param grid The metrics of the grid, which convert the samples to layout
 * units.
 * @param spacing The distance two crossing wires keep, in layout units.
 * @return The slots, in cell coordinates, in the order of the walk.
 * @throws std::invalid_argument When @p spacing is not positive.
 */
[[nodiscard]] MQT_SCPD_GRID_EXPORT std::vector<Point>
borderSlots(const PartitionBorder& border, const GridMetrics& grid,
            double spacing);

} // namespace mqt::scpd::grid
