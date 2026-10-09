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

#include "mqt-scpd/grid/Bottlenecks.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"
#include "mqt-scpd/grid/mqt_scpd_grid_export.hpp"

#include <array>
#include <cstddef>
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
 * between its two obstacle cells, and it takes no free space of its own. It
 * refuses the move between two cells whose center-to-center segment crosses
 * the line, in both directions. A cell whose own center lies on the line,
 * other than the two ends, is enclosed: every move out of it and into it is
 * refused, because a wire there is already on the wrong side of every
 * crossing.
 *
 * Blocking the cells of the line instead would take free space from the
 * chambers on both sides, and a line of cells is eight-connected, so a walk
 * that moves diagonally would step through it.
 *
 * @param bottlenecks The bottlenecks.
 * @param grid The metrics of the grid they were found on.
 * @return For each bottleneck, the moves it refuses. Every move comes with
 * its reverse.
 */
[[nodiscard]] MQT_SCPD_GRID_EXPORT std::vector<std::vector<BlockedMove>>
bottleneckMoves(const std::vector<Bottleneck>& bottlenecks,
                const GridMetrics& grid);

} // namespace mqt::scpd::grid
