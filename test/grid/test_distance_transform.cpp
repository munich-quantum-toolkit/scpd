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

} // namespace
