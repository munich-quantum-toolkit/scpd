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
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <format>
#include <iterator>
#include <stdexcept>
#include <utility>
#include <vector>

namespace mqt::scpd::grid {
namespace {

constexpr std::uint32_t WIDTH = 61;
constexpr std::uint32_t HEIGHT = 41;

GridMetrics unitGrid(const std::uint32_t width = WIDTH,
                     const std::uint32_t height = HEIGHT) {
  return GridMetrics::fit({.minX = 0.0,
                           .minY = 0.0,
                           .maxX = static_cast<double>(width - 1),
                           .maxY = static_cast<double>(height - 1)},
                          width, height);
}

/// A corridor with a pinch: two walls that step towards each other over the
/// columns from @p pinchFrom to @p pinchTo and back out again.
BitGrid pinchedCorridor(const std::uint32_t pinchHalfHeight,
                        const std::uint32_t pinchFrom = 28,
                        const std::uint32_t pinchTo = 32) {
  BitGrid mask(WIDTH, HEIGHT);
  constexpr std::uint32_t openHalf = 15;
  constexpr std::uint32_t middle = HEIGHT / 2;

  for (std::uint32_t x = 0; x < WIDTH; ++x) {
    const auto half =
        (x >= pinchFrom && x <= pinchTo) ? pinchHalfHeight : openHalf;
    for (std::uint32_t y = 0; y < HEIGHT; ++y) {
      if (y + half < middle || y > middle + half) {
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
  return {.mask = std::move(mask),
          .grid = grid,
          .axis = std::move(axis),
          .distance = std::move(distance)};
}

TEST(Bottlenecks, FindsThePinchOfACorridorAndSpansIt) {
  const auto scene = sceneOf(pinchedCorridor(3));
  const auto found =
      findBottlenecks(scene.mask, scene.axis, scene.distance, scene.grid);
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

/// A corridor that is wide on the left, narrows to a half height of two at
/// column 10 and widens again by one cell every six columns to the right.
BitGrid funnel() {
  BitGrid mask(WIDTH, HEIGHT);
  constexpr std::uint32_t middle = HEIGHT / 2;
  for (std::uint32_t x = 0; x < WIDTH; ++x) {
    const auto half = x < 10 ? 15U : 2U + ((x - 10) / 6);
    for (std::uint32_t y = 0; y < HEIGHT; ++y) {
      if (y + half < middle || y > middle + half) {
        mask.setCell(x, y);
      }
    }
  }
  return mask;
}

TEST(Bottlenecks, ReportsAPinchOfConstantWidthOnce) {
  // The pinch keeps one width over five columns, 28 to 32. Every column of it
  // is as narrow as the next, and it is one gate across the middle one.
  const auto scene = sceneOf(pinchedCorridor(3));
  const auto found =
      findBottlenecks(scene.mask, scene.axis, scene.distance, scene.grid);

  ASSERT_EQ(found.size(), 1U);
  EXPECT_EQ(found.front().saddle % WIDTH, 30U);
  EXPECT_EQ(found.front().first % WIDTH, 30U);
  EXPECT_EQ(found.front().second % WIDTH, 30U);
}

TEST(Bottlenecks, FindsOnlyTheNarrowEndOfAFunnel) {
  // Each step of the funnel keeps one width over six columns, and every step
  // but the first has a narrower one beside it.
  const auto scene = sceneOf(funnel());
  const auto found =
      findBottlenecks(scene.mask, scene.axis, scene.distance, scene.grid);

  ASSERT_EQ(found.size(), 1U);
  const auto column = found.front().saddle % WIDTH;
  EXPECT_GE(column, 10U);
  EXPECT_LE(column, 15U);
}

TEST(Bottlenecks, FindsAPinchAtTheEdgeOfTheGrid) {
  for (const auto& [from, to] : {std::pair{0U, 3U}, std::pair{57U, 60U}}) {
    SCOPED_TRACE(std::format("pinch from x = {} to x = {}", from, to));
    const auto scene = sceneOf(pinchedCorridor(3, from, to));
    const auto found =
        findBottlenecks(scene.mask, scene.axis, scene.distance, scene.grid);
    ASSERT_FALSE(found.empty());

    bool spansThePinch = false;
    for (const auto& bottleneck : found) {
      EXPECT_TRUE(scene.mask.test(bottleneck.first));
      EXPECT_TRUE(scene.mask.test(bottleneck.second));
      const auto x0 = bottleneck.first % WIDTH;
      const auto y0 = bottleneck.first / WIDTH;
      const auto y1 = bottleneck.second / WIDTH;
      if (y0 != y1 && x0 + 2 >= from && x0 <= to + 2) {
        spansThePinch = true;
      }
    }
    EXPECT_TRUE(spansThePinch);
  }
}

TEST(Bottlenecks, FindsNothingWhereEveryPlaceIsWide) {
  // A corridor of constant width has no place narrower than the admission
  // threshold, so nothing is a bottleneck.
  const auto scene = sceneOf(pinchedCorridor(15));
  const auto found =
      findBottlenecks(scene.mask, scene.axis, scene.distance, scene.grid);
  EXPECT_TRUE(found.empty());
}

TEST(Bottlenecks, IsTheSameOnASecondRun) {
  const auto scene = sceneOf(pinchedCorridor(3));
  const auto first =
      findBottlenecks(scene.mask, scene.axis, scene.distance, scene.grid);
  const auto second =
      findBottlenecks(scene.mask, scene.axis, scene.distance, scene.grid);

  ASSERT_EQ(first.size(), second.size());
  for (std::size_t index = 0; index < first.size(); ++index) {
    EXPECT_EQ(first[index].first, second[index].first);
    EXPECT_EQ(first[index].second, second[index].second);
    EXPECT_EQ(first[index].saddle, second[index].saddle);
  }
}

/// A wide corridor with a spike that hangs from its upper wall down to one
/// cell, column 30 and row 30, five rows above the lower wall.
BitGrid spike() {
  BitGrid mask(WIDTH, HEIGHT);
  constexpr int middle = HEIGHT / 2;
  for (std::uint32_t x = 0; x < WIDTH; ++x) {
    const auto offset = std::abs(static_cast<int>(x) - 30);
    for (std::uint32_t y = 0; y < HEIGHT; ++y) {
      const auto row = static_cast<int>(y);
      if (row + 15 < middle || row > middle + 15 ||
          row <= middle + 10 - (2 * offset)) {
        mask.setCell(x, y);
      }
    }
  }
  return mask;
}

/// A corridor with a pillar of one cell in its middle, at column 30 and row
/// 20, and an opening of six cells above and below it.
BitGrid pillar() {
  BitGrid mask(WIDTH, HEIGHT);
  constexpr std::uint32_t middle = HEIGHT / 2;
  for (std::uint32_t x = 0; x < WIDTH; ++x) {
    for (std::uint32_t y = 0; y < HEIGHT; ++y) {
      if (y + 6 < middle || y > middle + 6 || (x == 30 && y == middle)) {
        mask.setCell(x, y);
      }
    }
  }
  return mask;
}

/// The bottlenecks that end on one cell.
std::vector<Bottleneck> endingOn(const std::vector<Bottleneck>& found,
                                 const std::size_t cell) {
  std::vector<Bottleneck> ending;
  std::ranges::copy_if(
      found, std::back_inserter(ending), [cell](const Bottleneck& bottleneck) {
        return bottleneck.first == cell || bottleneck.second == cell;
      });
  return ending;
}

TEST(Bottlenecks, KeepsTheShortestCutOfANarrowingThatTheRasterSplits) {
  // The raster gives two cuts from the tip of the spike to the lower wall,
  // three cells apart there. With a reach of six cells they are one narrowing,
  // and the shorter cut stands for it.
  const auto scene = sceneOf(spike());
  const auto tip = scene.grid.index(30, 30);

  const auto apart =
      findBottlenecks(scene.mask, scene.axis, scene.distance, scene.grid);
  ASSERT_EQ(endingOn(apart, tip).size(), 2U);

  const auto merged =
      endingOn(findBottlenecks(scene.mask, scene.axis, scene.distance,
                               scene.grid, {.sameNarrowing = 6.0}),
               tip);
  ASSERT_EQ(merged.size(), 1U);
  const auto length = [&scene](const Bottleneck& cut) {
    const auto dx = static_cast<double>(scene.grid.cell(cut.first).x()) -
                    static_cast<double>(scene.grid.cell(cut.second).x());
    const auto dy = static_cast<double>(scene.grid.cell(cut.first).y()) -
                    static_cast<double>(scene.grid.cell(cut.second).y());
    return std::hypot(dx, dy);
  };
  for (const auto& cut : endingOn(apart, tip)) {
    EXPECT_LE(length(merged.front()), length(cut));
  }
}

TEST(Bottlenecks, KeepsTheTwoOpeningsBesideAPillarApart) {
  // Both openings end on the pillar, but their other ends lie fourteen cells
  // apart, so they are two narrowings under a reach of six cells.
  const auto scene = sceneOf(pillar());
  const auto found = findBottlenecks(scene.mask, scene.axis, scene.distance,
                                     scene.grid, {.sameNarrowing = 6.0});
  EXPECT_EQ(endingOn(found, scene.grid.index(30, 20)).size(), 2U);
}

TEST(Bottlenecks, DropsOneThatWouldRunOverATarget) {
  const auto scene = sceneOf(pinchedCorridor(3));
  const auto without =
      findBottlenecks(scene.mask, scene.axis, scene.distance, scene.grid);
  ASSERT_FALSE(without.empty());

  // A target on the middle cell of the first bottleneck's line removes that
  // bottleneck: a port is not a wall that capacity divides around.
  std::vector<std::size_t> targets;
  const auto& gone = without.front();
  const auto x0 = static_cast<int>(gone.first % WIDTH);
  const auto y0 = static_cast<int>(gone.first / WIDTH);
  const auto x1 = static_cast<int>(gone.second % WIDTH);
  const auto y1 = static_cast<int>(gone.second / WIDTH);
  targets.push_back(static_cast<std::size_t>(((y0 + y1) / 2) * WIDTH) +
                    static_cast<std::size_t>((x0 + x1) / 2));

  const auto with = findBottlenecks(scene.mask, scene.axis, scene.distance,
                                    scene.grid, {.targets = targets});
  EXPECT_LT(with.size(), without.size());
  EXPECT_TRUE(std::ranges::none_of(with, [&gone](const Bottleneck& kept) {
    return kept.first == gone.first && kept.second == gone.second;
  }));
}

TEST(Bottlenecks, RefusesInputsThatDoNotDescribeOneGrid) {
  const auto scene = sceneOf(pinchedCorridor(3));
  EXPECT_THROW(static_cast<void>(findBottlenecks(
                   scene.mask, scene.axis, scene.distance, unitGrid(10, 10))),
               std::invalid_argument);

  const std::vector<std::uint32_t> shortDistance(4, 0);
  EXPECT_THROW(static_cast<void>(findBottlenecks(scene.mask, scene.axis,
                                                 shortDistance, scene.grid)),
               std::invalid_argument);
}

TEST(BottleneckCapacity, CountsTheWiresThatFitTheGap) {
  // Ten cells apart on a grid of ten layout units per cell is a gap of 100.
  const auto grid = GridMetrics::fit(
      {.minX = 0.0, .minY = 0.0, .maxX = 990.0, .maxY = 990.0}, 100, 100);
  ASSERT_DOUBLE_EQ(grid.cellWidth, 10.0);

  const Bottleneck across{.first = 0, .second = 10, .saddle = 5};
  // A gap of 100 against a pitch of 30 holds three wires, rounded up to four
  // where neither end sits on a reserved cell.
  EXPECT_EQ(bottleneckCapacity(across, grid, 25.0, 5.0, false), 4U);
  EXPECT_EQ(bottleneckCapacity(across, grid, 25.0, 5.0, true), 3U);
}

TEST(BottleneckCapacity, CountsEachAxisInItsOwnCellStep) {
  // Cells ten layout units wide and five high.
  const auto grid = GridMetrics::fit(
      {.minX = 0.0, .minY = 0.0, .maxX = 990.0, .maxY = 990.0}, 100, 199);
  ASSERT_DOUBLE_EQ(grid.cellWidth, 10.0);
  ASSERT_DOUBLE_EQ(grid.cellHeight, 5.0);

  // Ten cells along x are 100 units: four wires at a pitch of 30.
  const Bottleneck acrossX{.first = 0, .second = 10, .saddle = 5};
  EXPECT_EQ(bottleneckCapacity(acrossX, grid, 25.0, 5.0, false), 4U);
  // Ten rows up, at cell 1000 of rows 100 cells wide, are 50 units: two
  // wires.
  const Bottleneck acrossY{.first = 0, .second = 1000, .saddle = 500};
  EXPECT_EQ(bottleneckCapacity(acrossY, grid, 25.0, 5.0, false), 2U);
}

TEST(BottleneckCapacity, CarriesNoWireThroughAGapBelowTheWireSpacing) {
  const auto grid = GridMetrics::fit(
      {.minX = 0.0, .minY = 0.0, .maxX = 990.0, .maxY = 990.0}, 100, 100);
  // A gap of 20 is wider than the obstacle clearance of 5 and narrower than
  // the wire spacing of 25.
  const Bottleneck narrow{.first = 0, .second = 2, .saddle = 1};
  EXPECT_EQ(bottleneckCapacity(narrow, grid, 25.0, 5.0, false), 0U);
}

TEST(BottleneckCapacity, CarriesNothingThroughAGapBelowTheObstacleClearance) {
  const auto grid = GridMetrics::fit(
      {.minX = 0.0, .minY = 0.0, .maxX = 990.0, .maxY = 990.0}, 100, 100);
  const Bottleneck touching{.first = 0, .second = 1, .saddle = 0};
  EXPECT_EQ(bottleneckCapacity(touching, grid, 185.0, 25.0, false), 0U);
}

} // namespace
} // namespace mqt::scpd::grid
