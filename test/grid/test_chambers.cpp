/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// Chambers: the free space cut by bottlenecks, on masks whose rooms the test
// knows.

#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/grid/Bottlenecks.hpp"
#include "mqt-scpd/grid/Chambers.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace mqt::scpd::grid {
namespace {

constexpr std::uint32_t WIDTH = 61;
constexpr std::uint32_t HEIGHT = 41;
constexpr std::uint32_t MIDDLE = HEIGHT / 2;

GridMetrics unitGrid() {
  return GridMetrics::fit({.minX = 0.0,
                           .minY = 0.0,
                           .maxX = static_cast<double>(WIDTH - 1),
                           .maxY = static_cast<double>(HEIGHT - 1)},
                          WIDTH, HEIGHT);
}

std::size_t at(const std::uint32_t x, const std::uint32_t y) {
  return (static_cast<std::size_t>(y) * WIDTH) + x;
}

/// A corridor along x, free on the rows MIDDLE - 5 to MIDDLE + 5.
BitGrid corridor() {
  BitGrid mask(WIDTH, HEIGHT);
  for (std::uint32_t y = 0; y < HEIGHT; ++y) {
    for (std::uint32_t x = 0; x < WIDTH; ++x) {
      if (y + 5 < MIDDLE || y > MIDDLE + 5) {
        mask.setCell(x, y);
      }
    }
  }
  return mask;
}

TEST(Chambers, AGateAcrossACorridorCutsItInTwo) {
  const auto mask = corridor();
  const std::vector<Bottleneck> gates = {{.first = at(30, MIDDLE - 6),
                                          .second = at(30, MIDDLE + 6),
                                          .saddle = at(30, MIDDLE)}};
  const auto chambers = chambersOf(mask, gates, unitGrid());

  EXPECT_EQ(chambers.count, 2U);
  const auto left = chambers.of[at(10, MIDDLE)];
  const auto right = chambers.of[at(50, MIDDLE)];
  EXPECT_NE(left, NO_CHAMBER);
  EXPECT_NE(right, NO_CHAMBER);
  EXPECT_NE(left, right);
  ASSERT_EQ(chambers.beside.size(), 1U);
  EXPECT_EQ(chambers.beside[0],
            (std::vector<std::uint32_t>{std::min(left, right),
                                        std::max(left, right)}));
}

TEST(Chambers, AGateWithAWayAroundItCutsNothing) {
  // An island in the middle of the corridor, and a gate from it to the
  // upper wall: the free space still runs around the island below.
  auto mask = corridor();
  for (std::uint32_t x = 28; x <= 32; ++x) {
    for (std::uint32_t y = MIDDLE - 1; y <= MIDDLE + 1; ++y) {
      mask.setCell(x, y);
    }
  }
  const std::vector<Bottleneck> gates = {{.first = at(30, MIDDLE + 1),
                                          .second = at(30, MIDDLE + 6),
                                          .saddle = at(30, MIDDLE + 3)}};
  const auto chambers = chambersOf(mask, gates, unitGrid());

  EXPECT_EQ(chambers.count, 1U);
  ASSERT_EQ(chambers.beside.size(), 1U);
  EXPECT_EQ(chambers.beside[0].size(), 1U);
}

TEST(Chambers, WithoutGatesEveryPieceOfFreeSpaceIsAChamber) {
  // A wall across the corridor splits it without any gate.
  auto mask = corridor();
  for (std::uint32_t y = 0; y < HEIGHT; ++y) {
    mask.setCell(30, y);
  }
  const auto chambers = chambersOf(mask, {}, unitGrid());

  EXPECT_EQ(chambers.count, 2U);
  EXPECT_EQ(chambers.of[at(30, MIDDLE)], NO_CHAMBER);
  EXPECT_NE(chambers.of[at(10, MIDDLE)], chambers.of[at(50, MIDDLE)]);
  EXPECT_TRUE(chambers.beside.empty());
}

TEST(Chambers, ADiagonalStepBetweenTwoWallCornersIsNoPassage) {
  // Two walls that touch only at a corner close the corridor.
  auto mask = corridor();
  for (std::uint32_t y = 0; y <= MIDDLE; ++y) {
    mask.setCell(30, y);
  }
  for (std::uint32_t y = MIDDLE + 1; y < HEIGHT; ++y) {
    mask.setCell(31, y);
  }
  const auto chambers = chambersOf(mask, {}, unitGrid());

  EXPECT_EQ(chambers.count, 2U);
}

} // namespace
} // namespace mqt::scpd::grid
