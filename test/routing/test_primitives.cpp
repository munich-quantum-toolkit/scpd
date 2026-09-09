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
#include "mqt-scpd/routing/Primitives.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <set>
#include <stdexcept>

namespace {

using namespace mqt::scpd::routing;

const MovePrimitives PRIMITIVES(5);

TEST(Primitives, EveryHeadingHasAStraightAndTurnsToEitherSide) {
  for (Heading h = 0; h < NUM_HEADINGS; ++h) {
    const auto moves = PRIMITIVES.of(h);
    ASSERT_FALSE(moves.empty()) << static_cast<int>(h);
    std::set<int> turns;
    for (const Primitive& p : moves) {
      const int eighths = ((static_cast<int>(p.exitHeading) - h + 12) % 8) - 4;
      turns.insert(eighths);
      EXPECT_FALSE(p.swept.empty()) << p.id;
      EXPECT_FALSE(p.samples.empty()) << p.id;
      EXPECT_LT(p.id, MovePrimitives::MAX_PRIMITIVE_ID);
    }
    // A straight step, an eighth turn each way and a quarter turn each way.
    EXPECT_EQ(turns, (std::set<int>{-2, -1, 0, 1, 2})) << static_cast<int>(h);
    EXPECT_TRUE(PRIMITIVES.isStraight(h, PRIMITIVES.straight(h)));
    EXPECT_DOUBLE_EQ(PRIMITIVES.cost(h, PRIMITIVES.straight(h)),
                     isDiagonal(h) ? std::numbers::sqrt2 : 1.0);
  }
}

TEST(Primitives, TheStraightStepFollowsItsHeading) {
  for (Heading h = 0; h < NUM_HEADINGS; ++h) {
    const Primitive* straight = PRIMITIVES.find(h, PRIMITIVES.straight(h));
    ASSERT_NE(straight, nullptr);
    EXPECT_EQ(straight->exitHeading, h);
    EXPECT_EQ(straight->dx, headingVector(h).dx);
    EXPECT_EQ(straight->dy, headingVector(h).dy);
  }
}

TEST(Primitives, AQuarterTurnIsAnArcOfTheBendRadius) {
  // From heading 0, which travels toward negative y, the quarter turn onto
  // heading 6, which travels toward positive x, ends one radius along each
  // axis and is a quarter of that circle long.
  const Primitive* quarter = nullptr;
  for (const Primitive& p : PRIMITIVES.of(0)) {
    if (p.exitHeading == 6) {
      quarter = &p;
    }
  }
  ASSERT_NE(quarter, nullptr);
  EXPECT_EQ(quarter->dx, 5);
  EXPECT_EQ(quarter->dy, -5);
  EXPECT_NEAR(quarter->cost, std::numbers::pi / 2.0 * 5.0, 0.01);

  // Its samples run from the origin to its end and stay on the circle
  // around the center at one radius to the side.
  ASSERT_FALSE(quarter->samples.empty());
  EXPECT_NEAR(quarter->samples.front().x(), 0.0, 1e-9);
  EXPECT_NEAR(quarter->samples.front().y(), 0.0, 1e-9);
  EXPECT_NEAR(quarter->samples.back().x(), 5.0, 0.5);
  EXPECT_NEAR(quarter->samples.back().y(), -5.0, 0.5);
}

TEST(Primitives, ADiagonalHeadingKeepsItsQuarterTurns) {
  // The rotation of the cardinal tables never lands on a diagonal heading,
  // so the quarter turns of a diagonal are built on their own.
  for (Heading h = 1; h < NUM_HEADINGS; h = static_cast<Heading>(h + 2)) {
    int quarters = 0;
    for (const Primitive& p : PRIMITIVES.of(h)) {
      if (headingDistance(h, p.exitHeading) == 2) {
        ++quarters;
        EXPECT_NEAR(p.cost, std::ceil(std::numbers::pi / 2.0 * 5.0), 1e-9);
        const double reach = std::hypot(static_cast<double>(p.dx), static_cast<double>(p.dy));
        EXPECT_NEAR(reach, 5.0 * std::numbers::sqrt2, 1.5);
      }
    }
    EXPECT_EQ(quarters, 2) << static_cast<int>(h);
  }
}

TEST(Primitives, SweptCellsStayWithinTheReachOfTheirMove) {
  for (Heading h = 0; h < NUM_HEADINGS; ++h) {
    for (const Primitive& p : PRIMITIVES.of(h)) {
      const int span = std::max(std::abs(p.dx), std::abs(p.dy));
      for (const CellOffset& c : p.swept) {
        EXPECT_LE(std::abs(c.dx), span + 1) << p.id;
        EXPECT_LE(std::abs(c.dy), span + 1) << p.id;
      }
      // The end of the move is swept or is the next cell after the sweep.
      const CellOffset& last = p.swept.back();
      EXPECT_LE(std::abs(last.dx - p.dx), 1) << p.id;
      EXPECT_LE(std::abs(last.dy - p.dy), 1) << p.id;
    }
  }
}

TEST(Primitives, LookupsOfAnUnknownMoveAreSafe) {
  EXPECT_EQ(PRIMITIVES.find(0, MovePrimitives::MAX_PRIMITIVE_ID + 5), nullptr);
  EXPECT_EQ(PRIMITIVES.find(0, 999), nullptr);
  EXPECT_DOUBLE_EQ(PRIMITIVES.cost(0, 999), 0.0);
  // An unknown move reads as heading zero, which is what the search's own
  // lookup did, so heading zero is the one that reports it as straight.
  EXPECT_TRUE(PRIMITIVES.isStraight(0, 999));
  EXPECT_FALSE(PRIMITIVES.isStraight(3, 999));
}

TEST(Primitives, TheBendRadiusMustBePositive) {
  EXPECT_THROW(static_cast<void>(MovePrimitives(0)), std::invalid_argument);
  EXPECT_NO_THROW(static_cast<void>(MovePrimitives(3)));
  EXPECT_NO_THROW(static_cast<void>(MovePrimitives(8)));
}

} // namespace
