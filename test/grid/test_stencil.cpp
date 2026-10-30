/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/grid/Stencil.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <set>
#include <utility>
#include <vector>

namespace {

using namespace mqt::scpd::grid;

using Cell = std::pair<std::int32_t, std::int32_t>;

[[nodiscard]] bool holds(const Stencil& stencil, const std::int32_t dx,
                         const std::int32_t dy) {
  return std::ranges::find(stencil.full, Cell{dx, dy}) != stencil.full.end();
}

/// A rule of 185 layout units on cells of 9.95 is 18.6 cells. The disc
/// reaches the 19 cells the rule rounds up to, but holds only the cells
/// within the rule: a wire 19 cells beside another along an axis keeps it.
TEST(Stencil, HoldsTheCellsWithinTheRadiusAndNoOthers) {
  const auto stencil = stencilOf(19, 185.0 / 9.95);
  EXPECT_TRUE(holds(stencil, 0, 0));
  EXPECT_TRUE(holds(stencil, 18, 0));
  EXPECT_TRUE(holds(stencil, 0, -18));
  EXPECT_TRUE(holds(stencil, 13, 13));
  EXPECT_FALSE(holds(stencil, 19, 0));
  EXPECT_FALSE(holds(stencil, 0, -19));
  EXPECT_FALSE(holds(stencil, 14, 13));
  for (const auto& [dx, dy] : stencil.full) {
    EXPECT_LE((dx * dx) + (dy * dy), 345) << dx << ", " << dy;
  }
}

/// A whole radius holds the cell at exactly that distance.
TEST(Stencil, HoldsTheCellsAtAWholeRadius) {
  const auto stencil = stencilOf(5, 5.0);
  EXPECT_TRUE(holds(stencil, 5, 0));
  EXPECT_TRUE(holds(stencil, -3, 4));
  EXPECT_FALSE(holds(stencil, 4, 4));
  EXPECT_FALSE(holds(stencil, 6, 0));
}

/// The full disc at the first cell of a path and the edge of every step
/// after it cover what the full disc at every cell covers, through a turn
/// and along a diagonal.
TEST(Stencil, TheEdgesOfThePathsStepsCoverItsDiscs) {
  const auto stencil = stencilOf(7, 6.4);
  const std::vector<Cell> path{{0, 0},  {0, -1}, {0, -2}, {1, -3},
                               {2, -4}, {3, -4}, {4, -4}, {4, -3}};
  std::set<Cell> everyDisc;
  for (const auto& [x, y] : path) {
    for (const auto& [dx, dy] : stencil.full) {
      everyDisc.emplace(x + dx, y + dy);
    }
  }
  std::set<Cell> edges;
  for (const auto& [dx, dy] : stencil.full) {
    edges.emplace(path.front().first + dx, path.front().second + dy);
  }
  for (std::size_t at = 1; at < path.size(); ++at) {
    const auto sx = path[at].first - path[at - 1].first;
    const auto sy = path[at].second - path[at - 1].second;
    for (const auto& [dx, dy] : stencil.edge[static_cast<std::size_t>(
             sy + 1)][static_cast<std::size_t>(sx + 1)]) {
      edges.emplace(path[at].first + dx, path[at].second + dy);
    }
  }
  EXPECT_EQ(edges, everyDisc);
}

} // namespace
