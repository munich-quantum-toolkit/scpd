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

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <set>
#include <stdexcept>

namespace {

using namespace mqt::scpd::routing;

/// The move primitives of bend radius 5, built once for every test.
const MovePrimitives& primitives() {
  static const MovePrimitives PRIMITIVES(5);
  return PRIMITIVES;
}

/// The largest bend radius the router accepts.
constexpr uint32_t ROUTER_RADIUS_LIMIT = 23;

/// The direction of travel at the end of the curve of a move, in degrees.
/// The last two chords of an evenly sampled arc turn by the same angle, so
/// the tangent at the end lies half that angle beyond the last chord.
double endDirection(const Primitive& p) {
  const auto& s = p.samples;
  const std::size_t n = s.size();
  const double last =
      std::atan2(s[n - 1].y() - s[n - 2].y(), s[n - 1].x() - s[n - 2].x());
  const double before =
      std::atan2(s[n - 2].y() - s[n - 3].y(), s[n - 2].x() - s[n - 3].x());
  const double radians =
      last + (std::remainder(last - before, 2.0 * std::numbers::pi) / 2.0);
  return radians * 180.0 / std::numbers::pi;
}

/// The direction of travel of a heading, in degrees.
double headingDirection(const Heading h) {
  const HeadingVector v = headingVector(h);
  return std::atan2(static_cast<double>(v.dy), static_cast<double>(v.dx)) *
         180.0 / std::numbers::pi;
}

TEST(Primitives, EveryHeadingHasAStraightAndTurnsToEitherSide) {
  for (Heading h = 0; h < NUM_HEADINGS; ++h) {
    const auto moves = primitives().of(h);
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
    EXPECT_TRUE(primitives().isStraight(h, primitives().straight(h)));
    EXPECT_DOUBLE_EQ(primitives().cost(h, primitives().straight(h)),
                     isDiagonal(h) ? std::numbers::sqrt2 : 1.0);
  }
}

TEST(Primitives, TheStraightStepFollowsItsHeading) {
  for (Heading h = 0; h < NUM_HEADINGS; ++h) {
    const Primitive* straight = primitives().find(h, primitives().straight(h));
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
  for (const Primitive& p : primitives().of(0)) {
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
    for (const Primitive& p : primitives().of(h)) {
      if (headingDistance(h, p.exitHeading) == 2) {
        ++quarters;
        EXPECT_NEAR(p.cost, std::ceil(std::numbers::pi / 2.0 * 5.0), 1e-9);
        const double reach =
            std::hypot(static_cast<double>(p.dx), static_cast<double>(p.dy));
        EXPECT_NEAR(reach, 5.0 * std::numbers::sqrt2, 1.5);
      }
    }
    EXPECT_EQ(quarters, 2) << static_cast<int>(h);
  }
}

TEST(Primitives, EveryHeadingHoldsQuarterTurnsThatEndOnTheirExitHeading) {
  // At some radii a diagonal heading also holds a shorter arc to the
  // quarter-turn heading. Each side still has a quarter turn whose curve
  // ends along its exit heading.
  for (uint32_t radius = 1; radius <= ROUTER_RADIUS_LIMIT; ++radius) {
    const MovePrimitives table(radius);
    for (Heading h = 0; h < NUM_HEADINGS; ++h) {
      for (const int side : {-2, 2}) {
        const Heading exit = turned(h, side);
        bool tangent = false;
        for (const Primitive& p : table.of(h)) {
          if (p.exitHeading == exit && p.samples.size() >= 3) {
            const double error =
                std::remainder(endDirection(p) - headingDirection(exit), 360.0);
            tangent = tangent || std::abs(error) <= 1.0;
          }
        }
        EXPECT_TRUE(tangent)
            << radius << " " << static_cast<int>(h) << " " << side;
      }
    }
  }
}

TEST(Primitives, SweptCellsStayWithinTheReachOfTheirMove) {
  for (Heading h = 0; h < NUM_HEADINGS; ++h) {
    for (const Primitive& p : primitives().of(h)) {
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
  EXPECT_EQ(primitives().find(0, MovePrimitives::MAX_PRIMITIVE_ID + 5),
            nullptr);
  EXPECT_EQ(primitives().find(0, 999), nullptr);
  EXPECT_DOUBLE_EQ(primitives().cost(0, 999), 0.0);
  // An unknown move reads as heading zero, which is what the search's own
  // lookup did, so heading zero is the one that reports it as straight.
  EXPECT_TRUE(primitives().isStraight(0, 999));
  EXPECT_FALSE(primitives().isStraight(3, 999));
}

TEST(Primitives, TheBendRadiusMustBePositive) {
  EXPECT_THROW(static_cast<void>(MovePrimitives(0)), std::invalid_argument);
  EXPECT_NO_THROW(static_cast<void>(MovePrimitives(3)));
  EXPECT_NO_THROW(static_cast<void>(MovePrimitives(8)));
}

TEST(Primitives,
     EveryRadiusGivesEachHeadingAStraightStepAndAQuarterTurnEachWay) {
  for (uint32_t radius = 1; radius <= 40; ++radius) {
    const MovePrimitives table(radius);
    EXPECT_EQ(table.minRadius(), radius);
    for (Heading h = 0; h < NUM_HEADINGS; ++h) {
      const auto moves = table.of(h);
      ASSERT_FALSE(moves.empty()) << radius << " " << static_cast<int>(h);
      bool clockwise = false;
      bool counterClockwise = false;
      for (std::size_t i = 0; i < moves.size(); ++i) {
        const Primitive& p = moves[i];
        // The span lists the moves in ascending identifier order, and every
        // identifier fits the search state and finds its own move.
        if (i > 0) {
          EXPECT_GT(p.id, moves[i - 1].id) << radius;
        }
        EXPECT_LT(p.id, MovePrimitives::MAX_PRIMITIVE_ID) << radius;
        EXPECT_EQ(table.find(h, p.id), &p) << radius << " " << p.id;
        EXPECT_FALSE(p.swept.empty()) << radius << " " << p.id;
        ASSERT_FALSE(p.samples.empty()) << radius << " " << p.id;
        EXPECT_NEAR(p.samples.front().x(), 0.0, 1e-9) << radius << " " << p.id;
        EXPECT_NEAR(p.samples.front().y(), 0.0, 1e-9) << radius << " " << p.id;
        clockwise = clockwise || p.exitHeading == turned(h, 2);
        counterClockwise = counterClockwise || p.exitHeading == turned(h, -2);
      }
      EXPECT_TRUE(clockwise) << radius << " " << static_cast<int>(h);
      EXPECT_TRUE(counterClockwise) << radius << " " << static_cast<int>(h);

      const Primitive* straight = table.find(h, table.straight(h));
      ASSERT_NE(straight, nullptr) << radius << " " << static_cast<int>(h);
      EXPECT_EQ(straight->exitHeading, h) << radius;
      EXPECT_EQ(straight->dx, headingVector(h).dx) << radius;
      EXPECT_EQ(straight->dy, headingVector(h).dy) << radius;
    }
  }
}

TEST(Primitives, TheSamplesOfEveryMoveRunForwardToItsEnd) {
  for (uint32_t radius = 1; radius <= ROUTER_RADIUS_LIMIT; ++radius) {
    const MovePrimitives table(radius);
    for (Heading h = 0; h < NUM_HEADINGS; ++h) {
      for (const Primitive& p : table.of(h)) {
        const auto& s = p.samples;
        ASSERT_GE(s.size(), 2U) << radius << " " << p.id;
        EXPECT_NEAR(s.front().x(), 0.0, 1e-9) << radius << " " << p.id;
        EXPECT_NEAR(s.front().y(), 0.0, 1e-9) << radius << " " << p.id;
        // No sample steps back against the direction from start to end.
        for (std::size_t k = 1; k < s.size(); ++k) {
          const double along = ((s[k].x() - s[k - 1].x()) * p.dx) +
                               ((s[k].y() - s[k - 1].y()) * p.dy);
          EXPECT_GE(along, -1e-9) << radius << " " << static_cast<int>(h) << " "
                                  << p.id << " " << k;
        }
        // The curve reaches the end by a step of the sample spacing, not by
        // a jump.
        const std::size_t n = s.size();
        EXPECT_LE(std::hypot(s[n - 1].x() - s[n - 2].x(),
                             s[n - 1].y() - s[n - 2].y()),
                  2.0 * MovePrimitives::SAMPLE_SPACING)
            << radius << " " << static_cast<int>(h) << " " << p.id;
        if (isDiagonal(h) && (p.id == 900 || p.id == 901)) {
          // The arc of an exact diagonal quarter turn ends at no whole cell.
          EXPECT_EQ(std::lround(s.back().x()), p.dx) << radius << " " << p.id;
          EXPECT_EQ(std::lround(s.back().y()), p.dy) << radius << " " << p.id;
        } else {
          EXPECT_NEAR(s.back().x(), p.dx, 1e-9)
              << radius << " " << static_cast<int>(h) << " " << p.id;
          EXPECT_NEAR(s.back().y(), p.dy, 1e-9)
              << radius << " " << static_cast<int>(h) << " " << p.id;
        }
      }
    }
  }
}

TEST(Primitives, AMoveThatLeavesACardinalHeadingCostsTheLengthOfItsCurve) {
  for (uint32_t radius = 1; radius <= ROUTER_RADIUS_LIMIT; ++radius) {
    const MovePrimitives table(radius);
    for (Heading h = 0; h < NUM_HEADINGS; h = static_cast<Heading>(h + 2)) {
      for (const Primitive& p : table.of(h)) {
        double length = 0.0;
        for (std::size_t k = 1; k < p.samples.size(); ++k) {
          length += std::hypot(p.samples[k].x() - p.samples[k - 1].x(),
                               p.samples[k].y() - p.samples[k - 1].y());
        }
        EXPECT_NEAR(p.cost, length, 0.02)
            << radius << " " << static_cast<int>(h) << " " << p.id;
      }
    }
  }
}

TEST(Primitives, ARadiusTooLargeForTheIdentifiersIsRefused) {
  // The search state packs a move into ten bits. The largest radius whose
  // moves fit builds; a larger one is refused before any table is built,
  // even the largest radius the type holds.
  EXPECT_NO_THROW(
      static_cast<void>(MovePrimitives(MovePrimitives::MAX_BEND_RADIUS)));
  EXPECT_THROW(
      static_cast<void>(MovePrimitives(MovePrimitives::MAX_BEND_RADIUS + 1)),
      std::invalid_argument);
  EXPECT_THROW(static_cast<void>(MovePrimitives(1000)), std::invalid_argument);
  EXPECT_THROW(
      static_cast<void>(MovePrimitives(std::numeric_limits<uint32_t>::max())),
      std::invalid_argument);
}

} // namespace
