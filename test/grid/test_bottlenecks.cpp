/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// Bottleneck detection, on masks with one narrow place put where the test
// knows it is.

#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/grid/Bottlenecks.hpp"
#include "mqt-scpd/grid/DistanceTransform.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"
#include "mqt-scpd/grid/Voronoi.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace mqt::scpd::grid {
namespace {

constexpr std::uint32_t WIDTH = 61;
constexpr std::uint32_t HEIGHT = 41;

GridMetrics unitGrid(const std::uint32_t width = WIDTH, const std::uint32_t height = HEIGHT) {
  return GridMetrics::fit({.minX = 0.0,
                           .minY = 0.0,
                           .maxX = static_cast<double>(width - 1),
                           .maxY = static_cast<double>(height - 1)},
                          width, height);
}

/// A corridor with a pinch: two walls that step towards each other over a few
/// columns in the middle and back out again.
BitGrid pinchedCorridor(const std::uint32_t pinchHalfHeight) {
  BitGrid mask(WIDTH, HEIGHT);
  constexpr std::uint32_t OPEN_HALF = 15;
  constexpr std::uint32_t MIDDLE = HEIGHT / 2;
  constexpr std::uint32_t PINCH_FROM = 28;
  constexpr std::uint32_t PINCH_TO = 32;

  for (std::uint32_t x = 0; x < WIDTH; ++x) {
    const auto half = (x >= PINCH_FROM && x <= PINCH_TO) ? pinchHalfHeight : OPEN_HALF;
    for (std::uint32_t y = 0; y < HEIGHT; ++y) {
      if (y + half < MIDDLE || y > MIDDLE + half) {
        mask.setCell(x, y);
      }
    }
  }
  return mask;
}

/// Everything a bottleneck search needs, derived from one mask.
struct Scene {
  BitGrid mask;
  GridMetrics grid;
  MedialAxis axis;
  std::vector<std::uint32_t> distance;
};

Scene sceneOf(BitGrid mask) {
  auto grid = unitGrid(mask.width(), mask.height());
  auto distance = squaredDistanceTransform(mask);
  auto axis = rasterizeMedialAxis(mask, grid, medialAxis(mask));
  return {std::move(mask), grid, std::move(axis), std::move(distance)};
}

TEST(Bottlenecks, FindsThePinchOfACorridorAndSpansIt) {
  const auto scene = sceneOf(pinchedCorridor(3));
  const auto found = findBottlenecks(scene.mask, scene.axis, scene.distance, scene.grid);
  ASSERT_FALSE(found.empty());

  // Every bottleneck runs between two obstacle cells, and across the
  // corridor rather than along it: the two ends differ in y far more than
  // in x.
  bool spansThePinch = false;
  for (const auto& bottleneck : found) {
    EXPECT_TRUE(scene.mask.test(bottleneck.first));
    EXPECT_TRUE(scene.mask.test(bottleneck.second));

    const auto x0 = static_cast<int>(bottleneck.first % WIDTH);
    const auto y0 = static_cast<int>(bottleneck.first / WIDTH);
    const auto x1 = static_cast<int>(bottleneck.second % WIDTH);
    const auto y1 = static_cast<int>(bottleneck.second / WIDTH);
    if (std::abs(y1 - y0) > std::abs(x1 - x0) && x0 >= 26 && x0 <= 34) {
      spansThePinch = true;
    }
  }
  EXPECT_TRUE(spansThePinch);
}

TEST(Bottlenecks, FindsNothingWhereEveryPlaceIsWide) {
  // A corridor of constant width has no place narrower than the admission
  // threshold, so nothing is a bottleneck.
  const auto scene = sceneOf(pinchedCorridor(15));
  const auto found = findBottlenecks(scene.mask, scene.axis, scene.distance, scene.grid);
  EXPECT_TRUE(found.empty());
}

TEST(Bottlenecks, IsTheSameOnASecondRun) {
  const auto scene = sceneOf(pinchedCorridor(3));
  const auto first = findBottlenecks(scene.mask, scene.axis, scene.distance, scene.grid);
  const auto second = findBottlenecks(scene.mask, scene.axis, scene.distance, scene.grid);

  ASSERT_EQ(first.size(), second.size());
  for (std::size_t index = 0; index < first.size(); ++index) {
    EXPECT_EQ(first[index].first, second[index].first);
    EXPECT_EQ(first[index].second, second[index].second);
    EXPECT_EQ(first[index].saddle, second[index].saddle);
  }
}

TEST(Bottlenecks, DropsOneThatWouldRunOverATarget) {
  const auto scene = sceneOf(pinchedCorridor(3));
  const auto without = findBottlenecks(scene.mask, scene.axis, scene.distance, scene.grid);
  ASSERT_FALSE(without.empty());

  // Putting a target on every cell of the first bottleneck's line removes
  // exactly that bottleneck: a port is not a wall that capacity divides
  // around.
  std::vector<std::size_t> targets;
  const auto& gone = without.front();
  const auto x0 = static_cast<int>(gone.first % WIDTH);
  const auto y0 = static_cast<int>(gone.first / WIDTH);
  const auto x1 = static_cast<int>(gone.second % WIDTH);
  const auto y1 = static_cast<int>(gone.second / WIDTH);
  targets.push_back(static_cast<std::size_t>(((y0 + y1) / 2) * WIDTH) +
                    static_cast<std::size_t>((x0 + x1) / 2));

  const auto with = findBottlenecks(scene.mask, scene.axis, scene.distance, scene.grid,
                                    {.targets = targets});
  EXPECT_LT(with.size(), without.size());
}

TEST(Bottlenecks, RefusesInputsThatDoNotDescribeOneGrid) {
  const auto scene = sceneOf(pinchedCorridor(3));
  EXPECT_THROW(static_cast<void>(
                   findBottlenecks(scene.mask, scene.axis, scene.distance, unitGrid(10, 10))),
               std::invalid_argument);

  const std::vector<std::uint32_t> shortDistance(4, 0);
  EXPECT_THROW(
      static_cast<void>(findBottlenecks(scene.mask, scene.axis, shortDistance, scene.grid)),
      std::invalid_argument);
}

TEST(BottleneckCapacity, CountsTheWiresThatFitTheGap) {
  // Ten cells apart on a grid of ten layout units per cell is a gap of 100.
  const auto grid = GridMetrics::fit({.minX = 0.0, .minY = 0.0, .maxX = 990.0, .maxY = 990.0},
                                     100, 100);
  ASSERT_DOUBLE_EQ(grid.cellWidth, 10.0);

  const Bottleneck across{.first = 0, .second = 10, .saddle = 5};
  // A gap of 100 against a pitch of 30 holds three wires, rounded up to four
  // where neither end sits on a reserved cell.
  EXPECT_EQ(bottleneckCapacity(across, grid, 25.0, 5.0, false), 4U);
  EXPECT_EQ(bottleneckCapacity(across, grid, 25.0, 5.0, true), 3U);
}

TEST(BottleneckCapacity, CarriesNothingThroughAGapBelowTheObstacleClearance) {
  const auto grid = GridMetrics::fit({.minX = 0.0, .minY = 0.0, .maxX = 990.0, .maxY = 990.0},
                                     100, 100);
  const Bottleneck touching{.first = 0, .second = 1, .saddle = 0};
  EXPECT_EQ(bottleneckCapacity(touching, grid, 185.0, 25.0, false), 0U);
}

} // namespace
} // namespace mqt::scpd::grid
