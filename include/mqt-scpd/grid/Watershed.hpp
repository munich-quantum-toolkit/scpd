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
#include <span>
#include <vector>

namespace mqt::scpd::grid {

/**
 * @brief The label of the partition a cell belongs to.
 *
 * Zero means no partition yet, and one marks a cell reserved for a port. Every
 * partition proper has a label of two or more.
 */
using PartitionLabel = uint16_t;

/**
 * @brief The label of a cell that belongs to no partition yet.
 */
inline constexpr PartitionLabel LABEL_NONE = 0;

/**
 * @brief The label of a cell reserved for a port.
 *
 * runWatershed() treats a reserved cell as a barrier.
 */
inline constexpr PartitionLabel LABEL_RESERVED = 1;

/**
 * @brief The lowest label of a partition proper.
 */
inline constexpr PartitionLabel FIRST_PARTITION_LABEL = 2;

/**
 * @brief Grows one partition from every seed over the free, unlabeled cells.
 *
 * The growth is a fast marching over the four-neighborhood. Every cell joins
 * the partition whose front reaches the cell first. The function solves the
 * arrival time of a cell from the eikonal equation on the finalized neighbors
 * of the cell. When two fronts arrive within a tolerance of each other, the
 * lower seed index decides and the rounding error of the arrival times does
 * not. The labels are therefore deterministic on a symmetric layout.
 *
 * The function skips a seed on a blocked cell, on a cell that carries a label,
 * or off the grid. A skipped seed takes no label, so the accepted seeds get
 * consecutive labels in seed order. Cells that carry a label from before this
 * call, reserved cells included, are barriers. Free, unlabeled cells that no
 * front reaches keep LABEL_NONE. When @p labels has fewer entries than
 * @p blocked has cells, the function changes nothing.
 *
 * @param blocked The obstacle mask. No front enters a blocked cell.
 * @param seeds Row-major cell indices, one partition each, in order.
 * @param labels The label of every cell, row-major. The function grows the
 * partitions into it in place.
 * @param nextLabel The label of the first partition this call creates.
 * @pre @p nextLabel is at least FIRST_PARTITION_LABEL, and every label already
 * in @p labels is below @p nextLabel.
 * @return The label after the last one this call assigned. It equals
 * @p nextLabel when the call accepts no seed.
 * @throws std::length_error If a seed to accept would take the largest value
 * of PartitionLabel. The label after it, which the function returns, would
 * not fit into PartitionLabel. The function checks every seed before it
 * changes @p labels, so @p labels stays unchanged.
 */
[[nodiscard]] MQT_SCPD_GRID_EXPORT PartitionLabel
runWatershed(const BitGrid& blocked, std::span<const std::size_t> seeds,
             std::vector<PartitionLabel>& labels, PartitionLabel nextLabel);

/**
 * @brief Smooths the borders between the partitions of one watershed run by a
 * majority vote.
 *
 * A cell of the run lies on a border when one of its free four-neighbors
 * carries another label or does not belong to the run. Such a cell takes the
 * label that holds a clear majority, more than half of the counted cells, in
 * the square window of radius @p radius around it. The vote counts only the
 * free cells of this run, that is cells with a label of at least
 * @p firstLabel. Only a clear majority moves a cell, so the vote keeps real
 * corners.
 *
 * A seed cell keeps its label, so a partition with a seed keeps at least that
 * cell. The vote can still move every other cell of the partition, and it can
 * split a partition into pieces that do not touch. A partition without a seed
 * can lose all of its cells.
 *
 * Every pass reads the labels as they were before the pass. The result
 * therefore does not depend on the order in which a pass visits the cells, and
 * the mirror image of the input gives the mirror image of the result. On some
 * labels, such passes move cells back and forth on every pass and never
 * settle. The result then depends on whether @p iterations is odd or even. The
 * pass repeats at most @p iterations times. It stops early after a pass that
 * changes nothing. When @p labels has fewer entries than @p blocked has cells,
 * the function changes nothing.
 *
 * A pass counts the (2 @p radius + 1)^2 cells of the window for every border
 * cell. The window ends at the edge of the grid, so a radius larger than the
 * grid costs no more than a window that just covers the grid.
 *
 * @param blocked The obstacle mask. Blocked cells neither vote nor change.
 * @param seeds Row-major cell indices whose labels do not change. Pass the
 * seeds of the run. The function ignores a seed off the grid.
 * @param labels The label of every cell, row-major, smoothed in place.
 * @param firstLabel The first label of the run. Cells with a lower label
 * neither vote nor change.
 * @param radius The radius of the window, in cells. The window is
 * 2 @p radius + 1 cells on a side.
 * @param iterations The largest number of passes.
 * @throws std::invalid_argument If @p radius or @p iterations is negative.
 */
MQT_SCPD_GRID_EXPORT void
smoothPartitionBorders(const BitGrid& blocked,
                       std::span<const std::size_t> seeds,
                       std::vector<PartitionLabel>& labels,
                       PartitionLabel firstLabel, int radius, int iterations);

} // namespace mqt::scpd::grid
