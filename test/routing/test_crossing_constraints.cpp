/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/routing/CrossingConstraints.hpp"
#include "mqt-scpd/routing/Path.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

namespace {

using namespace mqt::scpd::routing;

constexpr uint32_t WIDTH = 200;
constexpr uint32_t HEIGHT = 200;

/// A straight wire from (x, y0) to (x, y1 - 1), travelling toward positive y.
Path verticalWire(const uint32_t x, const uint32_t y0, const uint32_t y1) {
  Path wire;
  for (uint32_t y = y0; y < y1; ++y) {
    wire.push_back({.x = x, .y = y, .heading = 4, .primitive = 0});
  }
  return wire;
}

TEST(CrossingConstraints, WithoutFeedlinesEveryCellIsAllowed) {
  const CrossingConstraints constraints;
  EXPECT_TRUE(constraints.empty());
  EXPECT_TRUE(constraints.allowed(10, 10, 0));
  EXPECT_EQ(constraints.maskAt(10, 10), 0U);
}

TEST(CrossingConstraints, AStraightRunMayOnlyBeCrossedAtARightAngle) {
  CrossingConstraints constraints;
  constraints.build(WIDTH, HEIGHT, {verticalWire(100, 20, 180)}, {false}, 5);
  EXPECT_FALSE(constraints.empty());

  // On the wire and within the radius, the mask holds the wire's heading.
  EXPECT_EQ(constraints.maskAt(100, 100), 1U << 4U);
  EXPECT_EQ(constraints.maskAt(105, 100), 1U << 4U);
  // Travelling east or west crosses it at a right angle.
  EXPECT_TRUE(constraints.allowed(100, 100, 6));
  EXPECT_TRUE(constraints.allowed(100, 100, 2));
  // Along it, or at forty-five degrees, is refused.
  EXPECT_FALSE(constraints.allowed(100, 100, 4));
  EXPECT_FALSE(constraints.allowed(100, 100, 0));
  EXPECT_FALSE(constraints.allowed(100, 100, 5));
  // Beyond the radius nothing is constrained.
  EXPECT_EQ(constraints.maskAt(106, 100), 0U);
  EXPECT_TRUE(constraints.allowed(106, 100, 5));
}

TEST(CrossingConstraints, TheEndsOfAFeedlineCannotBeCrossed) {
  CrossingConstraints constraints;
  constraints.build(WIDTH, HEIGHT, {verticalWire(100, 20, 180)}, {false}, 5);
  // The first and the last ten cells, and their neighbors, are closed.
  EXPECT_EQ(constraints.maskAt(100, 25), CrossingConstraints::CURVE_ZONE);
  EXPECT_EQ(constraints.maskAt(101, 25), CrossingConstraints::CURVE_ZONE);
  EXPECT_EQ(constraints.maskAt(100, 175), CrossingConstraints::CURVE_ZONE);
  EXPECT_FALSE(constraints.allowed(100, 25, 6));
  EXPECT_FALSE(constraints.allowed(100, 175, 2));
}

TEST(CrossingConstraints, ABendCannotBeCrossed) {
  // East along y = 50, then south: the cell where the heading changes does
  // not step along its own heading, so it is a bend.
  Path wire;
  for (uint32_t x = 20; x <= 100; ++x) {
    wire.push_back({.x = x, .y = 50, .heading = 6, .primitive = 0});
  }
  for (uint32_t y = 51; y < 150; ++y) {
    wire.push_back({.x = 100, .y = y, .heading = 4, .primitive = 0});
  }
  CrossingConstraints constraints;
  constraints.build(WIDTH, HEIGHT, {wire}, {false}, 5);
  EXPECT_EQ(constraints.maskAt(100, 50), CrossingConstraints::CURVE_ZONE);
  EXPECT_EQ(constraints.maskAt(99, 49), CrossingConstraints::CURVE_ZONE);
  EXPECT_FALSE(constraints.allowed(100, 50, 0));
  // Away from the bend and the ends, both legs are straight runs.
  EXPECT_EQ(constraints.maskAt(60, 50), 1U << 6U);
  EXPECT_EQ(constraints.maskAt(100, 100), 1U << 4U);
}

TEST(CrossingConstraints, ASkippedFeedlineAddsNoConstraint) {
  CrossingConstraints constraints;
  constraints.build(WIDTH, HEIGHT,
                    {verticalWire(50, 20, 180), verticalWire(150, 20, 180)},
                    {true, false}, 5);
  EXPECT_EQ(constraints.maskAt(50, 100), 0U);
  EXPECT_EQ(constraints.maskAt(150, 100), 1U << 4U);
}

TEST(CrossingConstraints, AnEmptyFeedlineAddsNoConstraint) {
  CrossingConstraints constraints;
  constraints.build(WIDTH, HEIGHT, {Path{}, verticalWire(150, 20, 180)},
                    {false, false}, 5);
  EXPECT_EQ(constraints.maskAt(150, 100), 1U << 4U);
  EXPECT_EQ(constraints.maskAt(50, 100), 0U);
  EXPECT_TRUE(constraints.allowed(50, 100, 4));
}

TEST(CrossingConstraints,
     AFeedlineAlongTheBorderConstrainsTheCellsInsideTheGrid) {
  // The wire runs along the first column and starts in the corner, so both
  // the zone around its straight run and the zone around its end reach past
  // the edge of the grid.
  CrossingConstraints constraints;
  constraints.build(WIDTH, HEIGHT, {verticalWire(0, 0, 180)}, {false}, 5);
  EXPECT_EQ(constraints.maskAt(0, 100), 1U << 4U);
  EXPECT_EQ(constraints.maskAt(5, 100), 1U << 4U);
  EXPECT_EQ(constraints.maskAt(6, 100), 0U);
  EXPECT_EQ(constraints.maskAt(0, 0), CrossingConstraints::CURVE_ZONE);
  EXPECT_EQ(constraints.maskAt(1, 0), CrossingConstraints::CURVE_ZONE);
  EXPECT_TRUE(constraints.allowed(0, 100, 6));
  EXPECT_FALSE(constraints.allowed(0, 100, 4));
}

TEST(CrossingConstraints, ACellListedTwiceOnAStraightRunIsNoBend) {
  // The test for a straight run looks at the next cell somewhere else, so a
  // cell that appears twice in a row keeps its place on the run.
  Path wire = verticalWire(100, 20, 180);
  const PathPoint repeated = wire[80];
  wire.insert(wire.begin() + 80, repeated);
  CrossingConstraints constraints;
  constraints.build(WIDTH, HEIGHT, {wire}, {false}, 5);
  EXPECT_EQ(constraints.maskAt(100, 100), 1U << 4U);
  EXPECT_EQ(constraints.maskAt(101, 100), 1U << 4U);
  EXPECT_TRUE(constraints.allowed(100, 100, 6));
}

TEST(CrossingConstraints, ACellOutsideTheGridIsNotAllowed) {
  CrossingConstraints constraints;
  constraints.build(WIDTH, HEIGHT, {verticalWire(100, 20, 180)}, {false}, 5);
  EXPECT_FALSE(constraints.allowed(WIDTH, 10, 6));
  EXPECT_EQ(constraints.maskAt(WIDTH, 10), 0U);
}

TEST(CrossingConstraints, ClearingForgetsEveryConstraint) {
  CrossingConstraints constraints;
  constraints.build(WIDTH, HEIGHT, {verticalWire(100, 20, 180)}, {false}, 5);
  constraints.clear();
  EXPECT_TRUE(constraints.empty());
  EXPECT_TRUE(constraints.allowed(100, 100, 5));
}

} // namespace
