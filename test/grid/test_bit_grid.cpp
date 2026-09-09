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

#include <cstdint>
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

} // namespace
