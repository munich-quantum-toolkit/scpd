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

/// The partition a cell belongs to. Zero is no partition yet, one is a cell
/// reserved for a port, and every partition proper has a label of two or
/// more.
using PartitionLabel = uint16_t;

inline constexpr PartitionLabel LABEL_NONE = 0;
inline constexpr PartitionLabel LABEL_RESERVED = 1;
inline constexpr PartitionLabel FIRST_PARTITION_LABEL = 2;

/// Grow one partition from every seed over the free, unlabeled cells.
///
/// The growth is a fast marching over the four-neighborhood: every cell
/// joins the partition whose front reaches it first, with the arrival time
/// solved from the eikonal equation on its finalized neighbors. Two fronts
/// that arrive within a tolerance of each other are decided by the lower seed
/// index, never by the rounding noise of the arrival times, so the labels
/// are deterministic on a symmetric layout. A seed on a blocked cell or on a
/// cell that carries a label is skipped. Cells that carry a label from
/// before this call, reserved cells included, are barriers.
///
/// @param blocked The obstacle mask.
/// @param seeds Row-major cell indices, one partition each, in order.
/// @param labels The label of every cell; grown in place.
/// @param nextLabel The label of the first partition this call creates.
/// @returns The label after the last one this call assigned.
[[nodiscard]] MQT_SCPD_GRID_EXPORT PartitionLabel
runWatershed(const BitGrid& blocked, std::span<const std::size_t> seeds,
             std::vector<PartitionLabel>& labels, PartitionLabel nextLabel);

/// Smooth the borders between the partitions of one watershed run by a
/// majority vote.
///
/// A cell on a border takes the label that holds a clear majority in the
/// window of the given radius around it, considering only the cells of this
/// run, that is with a label of at least firstLabel. The pass repeats up to
/// the given number of iterations and stops early once nothing changes.
MQT_SCPD_GRID_EXPORT void
smoothPartitionBorders(const BitGrid& blocked, std::vector<PartitionLabel>& labels,
                       PartitionLabel firstLabel, int radius, int iterations);

} // namespace mqt::scpd::grid
