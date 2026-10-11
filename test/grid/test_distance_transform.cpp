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
#include "mqt-scpd/grid/DistanceTransform.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <random>
#include <vector>

namespace {

using namespace mqt::scpd::grid;

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

TEST(DistanceTransform, MatchesBruteForceOnARandomGrid) {
  std::mt19937 rng(7);
  std::bernoulli_distribution coin(0.08);
  BitGrid blocked(37, 23);
  for (std::size_t i = 0; i < blocked.size(); ++i) {
    if (coin(rng)) {
      blocked.set(i);
    }
  }
  EXPECT_EQ(squaredDistanceTransform(blocked), bruteForce(blocked));
}

TEST(DistanceTransform, MatchesBruteForceWithFewObstacles) {
  // Most rows have no blocked cell, and in a column the nearest blocked
  // cell lies many rows away: the envelope of the rows decides alone.
  for (const std::uint32_t seed : {1U, 2U, 3U, 4U, 5U}) {
    std::mt19937 rng(seed);
    std::bernoulli_distribution coin(0.004);
    BitGrid blocked(61, 47);
    for (std::size_t i = 0; i < blocked.size(); ++i) {
      if (coin(rng)) {
        blocked.set(i);
      }
    }
    EXPECT_EQ(squaredDistanceTransform(blocked), bruteForce(blocked)) << seed;
  }
}

TEST(DistanceTransform, IsExactAlongRowsAndDiagonals) {
  BitGrid blocked(9, 9);
  blocked.setCell(4, 4);
  const std::vector<uint32_t> distance = squaredDistanceTransform(blocked);
  EXPECT_EQ(distance[4 * 9 + 4], 0U);
  EXPECT_EQ(distance[4 * 9 + 8], 16U);
  EXPECT_EQ(distance[0 * 9 + 0], 32U);
  EXPECT_EQ(distance[1 * 9 + 6], 13U);
}

TEST(DistanceTransform, AGridWithoutObstaclesIsUnbounded) {
  const BitGrid blocked(5, 4);
  for (const uint32_t value : squaredDistanceTransform(blocked)) {
    EXPECT_EQ(value, DISTANCE_UNBOUNDED);
  }
}

TEST(NearestBlocked, NamesABlockedCellAtTheExactDistance) {
  for (const std::uint32_t seed : {7U, 8U, 9U}) {
    std::mt19937 rng(seed);
    std::bernoulli_distribution coin(seed == 9U ? 0.004 : 0.08);
    BitGrid blocked(41, 29);
    for (std::size_t i = 0; i < blocked.size(); ++i) {
      if (coin(rng)) {
        blocked.set(i);
      }
    }
    const auto nearest = nearestBlocked(blocked);
    const auto exact = bruteForce(blocked);
    for (std::size_t i = 0; i < blocked.size(); ++i) {
      ASSERT_EQ(nearest.squared[i], exact[i]) << seed << " cell " << i;
      if (exact[i] == DISTANCE_UNBOUNDED) {
        continue;
      }
      const auto site = nearest.cell[i];
      ASSERT_NE(site, NO_NEAREST);
      ASSERT_TRUE(blocked.test(site));
      const auto dx =
          static_cast<int64_t>(i % 41) - static_cast<int64_t>(site % 41);
      const auto dy =
          static_cast<int64_t>(i / 41) - static_cast<int64_t>(site / 41);
      EXPECT_EQ(static_cast<uint32_t>((dx * dx) + (dy * dy)), exact[i]);
    }
  }
}

TEST(NearestBlocked, NamesNoCellOnAnEmptyGrid) {
  const auto nearest = nearestBlocked(BitGrid(6, 3));
  for (std::size_t i = 0; i < nearest.cell.size(); ++i) {
    EXPECT_EQ(nearest.cell[i], NO_NEAREST);
    EXPECT_EQ(nearest.squared[i], DISTANCE_UNBOUNDED);
  }
}

} // namespace
