/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/grid/BitGrid.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace {

using mqt::scpd::grid::BitGrid;

TEST(BitGrid, StoresOneBitPerCell) {
  BitGrid grid(70, 3);
  EXPECT_EQ(grid.size(), 210U);
  EXPECT_EQ(grid.words().size(), 4U);
  EXPECT_EQ(grid.count(), 0U);

  grid.setCell(69, 2);
  grid.setCell(0, 0);
  grid.set(5);
  EXPECT_TRUE(grid.testCell(69, 2));
  EXPECT_TRUE(grid.test(209));
  EXPECT_TRUE(grid.testCell(0, 0));
  EXPECT_TRUE(grid.testCell(5, 0));
  EXPECT_FALSE(grid.testCell(1, 0));
  EXPECT_EQ(grid.count(), 3U);

  grid.set(5, false);
  EXPECT_FALSE(grid.test(5));
  EXPECT_EQ(grid.count(), 2U);
}

TEST(BitGrid, FillCountsExactly) {
  BitGrid grid(70, 3, true);
  EXPECT_EQ(grid.count(), 210U);
  grid.fill(false);
  EXPECT_EQ(grid.count(), 0U);
  grid.fill(true);
  EXPECT_EQ(grid.count(), 210U);
  EXPECT_EQ(grid, BitGrid(70, 3, true));
  EXPECT_NE(grid, BitGrid(70, 3));
}

/// `setRange` sets the same cells as `set` cell by cell, within one word,
/// across words, up to the last cell and for an empty range.
TEST(BitGrid, SetRangeSetsTheCellsSetSets) {
  const std::vector<std::pair<std::size_t, std::size_t>> ranges{
      {0, 0}, {3, 9}, {0, 64}, {63, 65}, {10, 200}, {128, 192}, {150, 210}};
  for (const auto& [first, last] : ranges) {
    BitGrid ranged(70, 3);
    BitGrid single(70, 3);
    ranged.set(1);
    single.set(1);
    ranged.setRange(first, last);
    for (auto cell = first; cell < last; ++cell) {
      single.set(cell);
    }
    EXPECT_EQ(ranged, single) << first << ".." << last;
  }
}

} // namespace
