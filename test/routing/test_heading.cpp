/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/routing/Heading.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>

namespace {

using namespace mqt::scpd::routing;

TEST(Headings, StepsRunClockwiseFromNegativeY) {
  EXPECT_EQ(headingVector(0).dx, 0);
  EXPECT_EQ(headingVector(0).dy, -1);
  EXPECT_EQ(headingVector(2).dx, -1);
  EXPECT_EQ(headingVector(2).dy, 0);
  EXPECT_EQ(headingVector(4).dy, 1);
  EXPECT_EQ(headingVector(6).dx, 1);
  EXPECT_EQ(headingVector(1).dx, -1);
  EXPECT_EQ(headingVector(1).dy, -1);
  // The heading is masked, so an out-of-range value never reads past the
  // table.
  EXPECT_EQ(headingVector(8).dy, headingVector(0).dy);
}

TEST(Headings, DiagonalsAreTheOddOnes) {
  for (Heading h = 0; h < NUM_HEADINGS; ++h) {
    EXPECT_EQ(isDiagonal(h), (h % 2) == 1) << static_cast<int>(h);
  }
}

TEST(Headings, TurningAndReversingWrapAround) {
  EXPECT_EQ(reverse(0), 4);
  EXPECT_EQ(reverse(6), 2);
  EXPECT_EQ(turned(0, 2), 2);
  EXPECT_EQ(turned(0, -2), 6);
  EXPECT_EQ(turned(7, 3), 2);
  EXPECT_EQ(turned(1, -4), 5);
}

TEST(Headings, DistanceIsTheShorterWayRound) {
  EXPECT_EQ(headingDistance(0, 0), 0U);
  EXPECT_EQ(headingDistance(0, 1), 1U);
  EXPECT_EQ(headingDistance(0, 7), 1U);
  EXPECT_EQ(headingDistance(0, 4), 4U);
  EXPECT_EQ(headingDistance(6, 1), 3U);
  EXPECT_TRUE(isOrthogonal(0, 2));
  EXPECT_TRUE(isOrthogonal(0, 6));
  EXPECT_FALSE(isOrthogonal(0, 1));
  EXPECT_FALSE(isOrthogonal(0, 4));
}

TEST(Headings, OrientationsOfTheChipInputMapToHeadings) {
  // The orientation is where the port faces; the heading is the one a wire
  // has when it arrives there.
  EXPECT_EQ(headingOfOrientation(0.0), 2);
  EXPECT_EQ(headingOfOrientation(90.0), 0);
  EXPECT_EQ(headingOfOrientation(180.0), 6);
  EXPECT_EQ(headingOfOrientation(270.0), 4);
  EXPECT_EQ(headingOfOrientation(45.0), 1);
  EXPECT_EQ(headingOfOrientation(135.0), 7);
  // The benchmark inputs carry orientations beyond one turn and below zero.
  EXPECT_EQ(headingOfOrientation(495.0), 7);
  EXPECT_EQ(headingOfOrientation(-90.0), 4);
  EXPECT_EQ(headingOfOrientation(675.0), 3);
  // Anything off the eight directions has no heading.
  EXPECT_EQ(headingOfOrientation(30.0), NUM_HEADINGS);
}

TEST(Headings, OrientationsOfAnyMagnitudeFold) {
  // A quarter turn plus or minus 2^40 full turns is exact in a double.
  const double turns = std::ldexp(360.0, 40);
  EXPECT_EQ(headingOfOrientation(turns + 90.0), 0);
  EXPECT_EQ(headingOfOrientation(-turns + 90.0), 0);
  // At 1e20, subtracting 360 leaves the value as it is. 1e20 is no multiple
  // of 45, and 45 times 2^1000 is a multiple of 360.
  EXPECT_EQ(headingOfOrientation(1e20), NUM_HEADINGS);
  EXPECT_EQ(headingOfOrientation(-1e20), NUM_HEADINGS);
  EXPECT_EQ(headingOfOrientation(std::ldexp(45.0, 1000)), 2);
  EXPECT_EQ(headingOfOrientation(-std::ldexp(45.0, 1000)), 2);
  // A tiny negative value folds onto 360 after rounding and has no heading.
  EXPECT_EQ(headingOfOrientation(-1e-300), NUM_HEADINGS);
  EXPECT_EQ(headingOfOrientation(-0.0), 2);
  const double infinity = std::numeric_limits<double>::infinity();
  EXPECT_EQ(headingOfOrientation(infinity), NUM_HEADINGS);
  EXPECT_EQ(headingOfOrientation(-infinity), NUM_HEADINGS);
  EXPECT_EQ(headingOfOrientation(std::numeric_limits<double>::quiet_NaN()),
            NUM_HEADINGS);
}

TEST(Headings, EveryEighthTurnOfOrientationPointsAgainstThePort) {
  // A wire arrives at a port against the direction the port faces, so its
  // step is the opposite of the unit vector of the orientation.
  for (int k = 0; k < 8; ++k) {
    const double degrees = 45.0 * k;
    const double radians = degrees * std::numbers::pi / 180.0;
    const Heading heading = headingOfOrientation(degrees);
    ASSERT_LT(heading, NUM_HEADINGS) << degrees;
    EXPECT_EQ(headingVector(heading).dx, -std::lround(std::cos(radians)))
        << degrees;
    EXPECT_EQ(headingVector(heading).dy, -std::lround(std::sin(radians)))
        << degrees;
  }
}

} // namespace
