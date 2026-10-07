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
#include <cstdlib>
#include <utility>
#include <vector>

namespace {

using namespace mqt::scpd::grid;
using mqt::scpd::test::SplitMix;

/// The blocked cells of a mask, as columns and rows.
std::vector<std::pair<int64_t, int64_t>> blockedCells(const BitGrid& blocked) {
  std::vector<std::pair<int64_t, int64_t>> cells;
  for (uint32_t y = 0; y < blocked.height(); ++y) {
    for (uint32_t x = 0; x < blocked.width(); ++x) {
      if (blocked.testCell(x, y)) {
        cells.emplace_back(x, y);
      }
    }
  }
  return cells;
}

/// The squared distance from every cell to the nearest blocked cell, as the
/// minimum over every blocked cell, and DISTANCE_UNBOUNDED for every larger
/// value.
std::vector<uint32_t> bruteForce(const BitGrid& blocked) {
  const std::vector<std::pair<int64_t, int64_t>> cells = blockedCells(blocked);
  std::vector<uint32_t> distance(blocked.size(), DISTANCE_UNBOUNDED);
  for (uint32_t y = 0; y < blocked.height(); ++y) {
    for (uint32_t x = 0; x < blocked.width(); ++x) {
      int64_t best = DISTANCE_UNBOUNDED;
      for (const auto& [bx, by] : cells) {
        const int64_t dx = static_cast<int64_t>(x) - bx;
        const int64_t dy = static_cast<int64_t>(y) - by;
        best = std::min(best, (dx * dx) + (dy * dy));
      }
      distance[(static_cast<std::size_t>(y) * blocked.width()) + x] =
          static_cast<uint32_t>(best);
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

TEST(DistanceTransform, TheColumnPassCombinesSaturatedRowsExactly) {
  // Grids of more than 1000 cells along both axes with a few blocked cells.
  // The row pass saturates a cell at DISTANCE_UNBOUNDED when its row holds no
  // blocked cell, or holds one only 1000 or more cells away. The column pass
  // then combines the saturated rows with the rows that hold a blocked cell
  // nearby. Such a cell can still lie less than 1000 cells from a blocked cell
  // in another row, and the result is its exact squared distance. Every cell
  // 1000 or more cells from all blocked cells holds DISTANCE_UNBOUNDED. The
  // first grid is fixed: its row 0 holds a blocked cell at column 0, and the
  // blocked cell (1200, 600) lies 600 cells above the end of that row.
  SplitMix random(31);
  std::size_t emptyRows = 0;
  std::size_t farRows = 0;
  std::size_t unbounded = 0;
  for (int round = 0; round < 3; ++round) {
    const auto width =
        static_cast<uint32_t>(round == 0 ? 1201 : random.between(1001, 1300));
    const auto height =
        static_cast<uint32_t>(round == 0 ? 1201 : random.between(1001, 1300));
    BitGrid blocked(width, height);
    if (round == 0) {
      blocked.setCell(0, 0);
      blocked.setCell(1200, 600);
    } else {
      const int64_t count = random.between(1, 6);
      for (int64_t i = 0; i < count; ++i) {
        blocked.setCell(static_cast<uint32_t>(random.between(0, width - 1)),
                        static_cast<uint32_t>(random.between(0, height - 1)));
      }
    }
    const std::vector<uint32_t> expected = bruteForce(blocked);
    ASSERT_EQ(squaredDistanceTransform(blocked), expected)
        << width << "x" << height << " in round " << round;

    // Count the cells of each kind.
    const std::vector<std::pair<int64_t, int64_t>> cells =
        blockedCells(blocked);
    for (uint32_t y = 0; y < height; ++y) {
      std::vector<int64_t> columns;
      for (const auto& [bx, by] : cells) {
        if (std::cmp_equal(by, y)) {
          columns.push_back(bx);
        }
      }
      for (uint32_t x = 0; x < width; ++x) {
        const uint32_t value =
            expected[(static_cast<std::size_t>(y) * width) + x];
        if (value == DISTANCE_UNBOUNDED) {
          ++unbounded;
        } else if (columns.empty()) {
          ++emptyRows;
        } else if (std::ranges::all_of(columns, [&](const int64_t bx) {
                     return std::abs(bx - static_cast<int64_t>(x)) >= 1000;
                   })) {
          ++farRows;
        }
      }
    }
  }
  EXPECT_GT(emptyRows, 0U);
  EXPECT_GT(farRows, 0U);
  EXPECT_GT(unbounded, 0U);
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
