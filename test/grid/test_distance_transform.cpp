/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "../SplitMix.hpp"
#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/grid/DistanceTransform.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace {

using namespace mqt::scpd::grid;
using mqt::scpd::test::SplitMix;

std::vector<uint32_t> bruteForce(const BitGrid& blocked) {
  std::vector<uint32_t> distance(blocked.size(), DISTANCE_UNBOUNDED);
  for (uint32_t y = 0; y < blocked.height(); ++y) {
    for (uint32_t x = 0; x < blocked.width(); ++x) {
      uint32_t best = DISTANCE_UNBOUNDED;
      for (uint32_t by = 0; by < blocked.height(); ++by) {
        for (uint32_t bx = 0; bx < blocked.width(); ++bx) {
          if (!blocked.testCell(bx, by)) {
            continue;
          }
          const int64_t dx = static_cast<int64_t>(x) - bx;
          const int64_t dy = static_cast<int64_t>(y) - by;
          best = std::min(best, static_cast<uint32_t>((dx * dx) + (dy * dy)));
        }
      }
      distance[(static_cast<std::size_t>(y) * blocked.width()) + x] = best;
    }
  }
  return distance;
}

TEST(DistanceTransform, MatchesBruteForceOnRandomSparseAndDenseGrids) {
  SplitMix random(7);
  const std::array<double, 4> densities = {0.003, 0.03, 0.3, 0.8};
  int grids = 0;
  for (int round = 0; round < 12; ++round) {
    for (const double density : densities) {
      const auto width = static_cast<uint32_t>(random.between(1, 40));
      const auto height = static_cast<uint32_t>(random.between(1, 30));
      // Four columns and three rows stay without a blocked cell, so that
      // every grid has columns without a blocked cell.
      const auto bandX = static_cast<uint32_t>(random.between(0, width - 1));
      const auto bandY = static_cast<uint32_t>(random.between(0, height - 1));
      BitGrid blocked(width, height);
      for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
          const bool inBand =
              (x >= bandX && x < bandX + 4) || (y >= bandY && y < bandY + 3);
          if (!inBand && random.unit() < density) {
            blocked.setCell(x, y);
          }
        }
      }
      EXPECT_EQ(squaredDistanceTransform(blocked), bruteForce(blocked))
          << width << "x" << height << " at density " << density;
      ++grids;
    }
  }
  EXPECT_EQ(grids, 48);
}

TEST(DistanceTransform, AColumnWithoutBlockedCellsTakesTheNearestRow) {
  // One blocked cell at the bottom left: every other column has no blocked
  // cell, and the cells of the top row take their distance across both axes.
  BitGrid blocked(30, 25);
  blocked.setCell(0, 0);
  EXPECT_EQ(squaredDistanceTransform(blocked), bruteForce(blocked));
  const std::vector<uint32_t> distance = squaredDistanceTransform(blocked);
  EXPECT_EQ(distance[(24 * 30) + 29], (29U * 29U) + (24U * 24U));

  // A single row and a single column.
  BitGrid row(40, 1);
  row.setCell(17, 0);
  EXPECT_EQ(squaredDistanceTransform(row), bruteForce(row));
  BitGrid column(1, 40);
  column.setCell(0, 3);
  column.setCell(0, 31);
  EXPECT_EQ(squaredDistanceTransform(column), bruteForce(column));
}

TEST(DistanceTransform, IsExactAlongRowsAndDiagonals) {
  BitGrid blocked(9, 9);
  blocked.setCell(4, 4);
  const std::vector<uint32_t> distance = squaredDistanceTransform(blocked);
  EXPECT_EQ(distance[(4 * 9) + 4], 0U);
  EXPECT_EQ(distance[(4 * 9) + 8], 16U);
  EXPECT_EQ(distance[(0 * 9) + 0], 32U);
  EXPECT_EQ(distance[(1 * 9) + 6], 13U);
}

TEST(DistanceTransform, AFarCellHoldsTheUnboundedValue) {
  // A row distance of 65536 squares to 2^32, which wraps to zero in 32 bits.
  // Every cell holds its squared distance up to the bound and the bound
  // beyond it, along a row and down a column alike.
  constexpr uint32_t length = 65537;
  BitGrid row(length, 1);
  row.setCell(0, 0);
  BitGrid column(1, length);
  column.setCell(0, 0);
  const std::vector<uint32_t> alongRow = squaredDistanceTransform(row);
  const std::vector<uint32_t> downColumn = squaredDistanceTransform(column);
  ASSERT_EQ(alongRow.size(), length);
  ASSERT_EQ(downColumn.size(), length);
  for (uint32_t i = 0; i < length; ++i) {
    const uint32_t expected = (i < 1000) ? i * i : DISTANCE_UNBOUNDED;
    ASSERT_EQ(alongRow[i], expected) << i;
    ASSERT_EQ(downColumn[i], expected) << i;
  }
}

TEST(DistanceTransform, AGridWithoutObstaclesIsUnbounded) {
  const BitGrid blocked(5, 4);
  for (const uint32_t value : squaredDistanceTransform(blocked)) {
    EXPECT_EQ(value, DISTANCE_UNBOUNDED);
  }
}

TEST(DistanceTransform, AGridWithoutCellsHasNoDistances) {
  EXPECT_TRUE(squaredDistanceTransform(BitGrid()).empty());
  EXPECT_TRUE(squaredDistanceTransform(BitGrid(0, 4)).empty());
  EXPECT_TRUE(squaredDistanceTransform(BitGrid(4, 0)).empty());
}

} // namespace
