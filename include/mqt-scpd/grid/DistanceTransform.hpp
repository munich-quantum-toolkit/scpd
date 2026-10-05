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

/**
 * @brief The squared distance every cell holds on a grid without any blocked
 * cell.
 *
 * The value is 1000 squared. squaredDistanceTransform() is exact for every
 * squared distance below this value and returns this value for every larger
 * one.
 */
inline constexpr uint32_t DISTANCE_UNBOUNDED = 1'000'000;

/**
 * @brief Computes the squared Euclidean distance, in cells, from every cell to
 * the nearest blocked cell.
 *
 * A pass along the rows finds the nearest blocked cell in each row. A pass down
 * the columns then combines the rows: for every cell it takes the minimum over
 * the rows of the squared row distance plus the squared row offset. The pass
 * builds the lower envelope of one parabola per row (Felzenszwalb and
 * Huttenlocher), so its time is linear in the number of cells. The transform
 * is exact for every squared distance below DISTANCE_UNBOUNDED, that is for
 * every distance below 1000 cells.
 *
 * @param blocked The obstacle mask.
 * @return The squared distance of every cell, row-major. Blocked cells hold
 * zero. A cell whose squared distance is DISTANCE_UNBOUNDED or more holds
 * DISTANCE_UNBOUNDED. On a grid without any blocked cell, every cell holds
 * DISTANCE_UNBOUNDED.
 */
[[nodiscard]] MQT_SCPD_GRID_EXPORT std::vector<uint32_t>
squaredDistanceTransform(const BitGrid& blocked);

} // namespace mqt::scpd::grid
