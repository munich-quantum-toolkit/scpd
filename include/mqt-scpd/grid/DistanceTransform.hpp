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

#include <cstdint>
#include <vector>

namespace mqt::scpd::grid {

/// The squared distance of a cell on a grid without any blocked cell.
inline constexpr uint32_t DISTANCE_UNBOUNDED = 1'000'000;

/// The squared Euclidean distance, in cells, of every cell to the nearest
/// blocked cell. Blocked cells hold zero. The transform is exact: a pass
/// along the rows finds the nearest blocked cell in each row, and a pass down
/// the columns combines the rows, scanning no further than the best distance
/// so far allows.
[[nodiscard]] MQT_SCPD_GRID_EXPORT std::vector<uint32_t>
squaredDistanceTransform(const BitGrid& blocked);

} // namespace mqt::scpd::grid
