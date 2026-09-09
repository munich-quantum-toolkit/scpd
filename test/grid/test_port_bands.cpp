/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/flatbuffers/geometry.hpp"
#include "mqt-scpd/geometry/Geometry.hpp"
#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"
#include "mqt-scpd/grid/PortBands.hpp"

#include <gtest/gtest.h>

#include <cstddef>

namespace {

using namespace mqt::scpd::grid;

const GridMetrics TEN_UNITS = GridMetrics::fit(
    BoundingBox{.minX = 0.0, .minY = 0.0, .maxX = 1000.0, .maxY = 1000.0}, 101, 101);

TEST(PortBands, OrientationsStepAlongTheAxesAndDiagonals) {
  EXPECT_EQ(orientationStep(0.0), (Step{.x = 1, .y = 0}));
  EXPECT_EQ(orientationStep(90.0), (Step{.x = 0, .y = 1}));
  EXPECT_EQ(orientationStep(180.0), (Step{.x = -1, .y = 0}));
  EXPECT_EQ(orientationStep(270.0), (Step{.x = 0, .y = -1}));
  EXPECT_EQ(orientationStep(45.0), (Step{.x = 1, .y = 1}));
  EXPECT_EQ(orientationStep(495.0), (Step{.x = -1, .y = 1}));
  EXPECT_TRUE(orientationStep(225.0).diagonal());
  EXPECT_FALSE(orientationStep(90.0).diagonal());
}

TEST(PortBands, WidthsAndLengthsComeFromTheRules) {
  // 185 units on 10 unit cells: 19 cells, an odd square, half 9.
  EXPECT_EQ(bandHalfWidth(185.0, TEN_UNITS, false), 9U);
  EXPECT_EQ(bandHalfWidth(185.0, TEN_UNITS, true), 6U);
  EXPECT_EQ(bandHalfWidth(200.0, TEN_UNITS, false), 10U);
  EXPECT_EQ(bandLength(100.0, TEN_UNITS, false), 10U);
  EXPECT_EQ(bandLength(100.0, TEN_UNITS, true), 7U);
}

TEST(PortBands, AStraightBandCoversItsStripsAndCanBeUndone) {
  BitGrid mask(50, 50);
  const PortBand band{.center = DCoord(25, 25),
                      .step = {.x = 0, .y = 1},
                      .forward = 5,
                      .backward = 3,
                      .halfWidth = 2};
  const StampedBand stamped = stampBand(mask, band);
  EXPECT_EQ(stamped.cells.size(), 9U * 5U);
  EXPECT_EQ(mask.count(), 45U);
  EXPECT_EQ(stamped.lastForwardCenter, DCoord(25, 30));
  EXPECT_TRUE(mask.testCell(23, 30));
  EXPECT_TRUE(mask.testCell(27, 22));
  EXPECT_FALSE(mask.testCell(28, 25));
  EXPECT_FALSE(mask.testCell(25, 31));
  EXPECT_FALSE(mask.testCell(25, 21));

  unstampBand(mask, stamped);
  EXPECT_EQ(mask.count(), 0U);
}

TEST(PortBands, ADiagonalBandHasNoGaps) {
  BitGrid mask(60, 60);
  const PortBand band{.center = DCoord(20, 20),
                      .step = {.x = 1, .y = 1},
                      .forward = 10,
                      .backward = 0,
                      .halfWidth = 1};
  const StampedBand stamped = stampBand(mask, band);
  EXPECT_EQ(stamped.lastForwardCenter, DCoord(30, 30));
  // Every strip center and the joining cell between diagonal neighbors.
  for (uint32_t d = 0; d <= 10; ++d) {
    EXPECT_TRUE(mask.testCell(20 + d, 20 + d));
    if (d > 0) {
      EXPECT_TRUE(mask.testCell(20 + d, 20 + d - 1));
    }
  }
}

TEST(PortBands, DiggingFreesTheExitBeyondTheBand) {
  BitGrid mask(50, 50);
  const PortBand band{.center = DCoord(25, 25),
                      .step = {.x = 0, .y = 1},
                      .forward = 5,
                      .backward = 0,
                      .halfWidth = 2};
  const StampedBand stamped = stampBand(mask, band);
  const auto target = digTargetBeyondBand(mask, stamped, band.step);
  ASSERT_TRUE(target.has_value());
  // The walk starts one beyond the band and steps once more.
  EXPECT_EQ(*target, static_cast<std::size_t>(32) * 50 + 25);
  EXPECT_FALSE(mask.testCell(25, 32));
  EXPECT_FALSE(mask.testCell(24, 33));
  EXPECT_TRUE(mask.testCell(25, 30));
}

TEST(PortBands, PlacingKeepsObstaclesThatWereThereBefore) {
  BitGrid before(50, 50);
  BitGrid mask = before;
  const PortBand band{.center = DCoord(25, 25),
                      .step = {.x = 1, .y = 0},
                      .forward = 4,
                      .backward = 0,
                      .halfWidth = 1};
  StampedBand stamped = stampBand(mask, band);
  auto target = placeTargetBeyondBand(mask, before, stamped, band.step);
  ASSERT_TRUE(target.has_value());
  EXPECT_EQ(*target, static_cast<std::size_t>(25) * 50 + 30);
  EXPECT_EQ(mask.count(), stamped.cells.size());

  // An obstacle on the ideal target: the band is undone and the target
  // moves past the obstacle.
  before.setCell(30, 25);
  before.setCell(31, 25);
  mask = before;
  stamped = stampBand(mask, band);
  target = placeTargetBeyondBand(mask, before, stamped, band.step);
  ASSERT_TRUE(target.has_value());
  EXPECT_FALSE(before.test(*target));
  EXPECT_FALSE(mask.test(*target));
  EXPECT_FALSE(mask.testCell(27, 25));
  EXPECT_TRUE(mask.testCell(30, 25));
}

TEST(PortBands, ALauncherSweepLeavesASlotBeyondIt) {
  BitGrid mask(40, 40);
  const std::size_t slot =
      stampLauncherSweep(mask, DCoord(10, 20), Step{.x = 1, .y = 0}, 6, 2);
  EXPECT_EQ(slot, static_cast<std::size_t>(20) * 40 + 19);
  EXPECT_FALSE(mask.test(slot));
  EXPECT_TRUE(mask.testCell(16, 20));
  EXPECT_TRUE(mask.testCell(18, 22));
  EXPECT_FALSE(mask.testCell(18, 23));
}

} // namespace
