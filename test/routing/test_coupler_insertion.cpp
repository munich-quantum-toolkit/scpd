/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/routing/CouplerInsertion.hpp"
#include "mqt-scpd/routing/Heading.hpp"
#include "mqt-scpd/routing/Path.hpp"
#include "mqt-scpd/routing/PathGeometry.hpp"
#include "mqt-scpd/routing/Primitives.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <vector>

namespace {

using namespace mqt::scpd::routing;

const MovePrimitives PRIMITIVES(5);

/// A straight run of a heading from a cell, one step per cell.
Path straightRun(const uint32_t x0, const uint32_t y0, const Heading heading,
                 const uint32_t steps) {
  Path path;
  const HeadingVector v = headingVector(heading);
  const uint16_t id = PRIMITIVES.straight(heading);
  for (uint32_t i = 0; i <= steps; ++i) {
    path.push_back({.x = static_cast<uint32_t>(x0 + (v.dx * i)),
                    .y = static_cast<uint32_t>(y0 + (v.dy * i)),
                    .heading = heading,
                    .primitive = id});
  }
  return path;
}

TEST(CouplerInsertion, ADoglegTurnsOnceAndThenRunsStraight) {
  const DoglegGeometry dogleg = buildDogleg(PRIMITIVES, 6, -1, 12);
  // A quarter turn against the clock from heading 6 ends on heading 4.
  EXPECT_EQ(dogleg.tip.heading, 4);
  EXPECT_NEAR(dogleg.cost, (std::numbers::pi / 2.0 * 5.0) + 12.0, 0.5);
  ASSERT_FALSE(dogleg.path.empty());
  EXPECT_EQ(dogleg.path.back().x, dogleg.tip.x);
  EXPECT_EQ(dogleg.path.back().y, dogleg.tip.y);
  // The straight run leaves along the exit heading.
  const HeadingVector v = headingVector(4);
  const PathPoint& afterTurn = dogleg.path[dogleg.path.size() - 13];
  EXPECT_EQ(static_cast<int64_t>(dogleg.tip.x) - afterTurn.x, v.dx * 12);
  EXPECT_EQ(static_cast<int64_t>(dogleg.tip.y) - afterTurn.y, v.dy * 12);

  const DoglegGeometry clockwise = buildDogleg(PRIMITIVES, 6, 1, 0);
  EXPECT_EQ(clockwise.tip.heading, 0);
  EXPECT_THROW(static_cast<void>(buildDogleg(PRIMITIVES, 6, 0, 4)), std::invalid_argument);
}

TEST(CouplerInsertion, TheSpliceLandsWhereTheRestOfThePathHitsTheTarget) {
  // A long straight resonator: the coupler is spliced so that the path from
  // it to the far end is the target length.
  Path path = straightRun(400, 300, 6, 300);
  const double before = reconstructSegments(PRIMITIVES, path).nominalLength;
  ASSERT_GT(before, 250.0);

  const std::optional<CouplerSplice> splice =
      spliceCouplerDogleg(PRIMITIVES, 150.0, path, 0);
  ASSERT_TRUE(splice.has_value());
  EXPECT_TRUE(splice->inAllowedArea);
  EXPECT_EQ(path.front().x, splice->anchor.x);
  EXPECT_EQ(path.front().y, splice->anchor.y);

  // The path now runs from the coupler to the unchanged far end, and its
  // length is the target within the step the splice can place it on.
  EXPECT_EQ(path.back().x, 700U);
  EXPECT_EQ(path.back().y, 300U);
  const double after = reconstructSegments(PRIMITIVES, path).nominalLength;
  EXPECT_NEAR(after, 150.0, 3.0);
}

TEST(CouplerInsertion, TheSplicePrefersAnUndershoot) {
  Path path = straightRun(400, 300, 6, 300);
  const std::optional<CouplerSplice> splice =
      spliceCouplerDogleg(PRIMITIVES, 150.0, path, 0);
  ASSERT_TRUE(splice.has_value());
  const double after = reconstructSegments(PRIMITIVES, path).nominalLength;
  // A resonator that is a little short can be lengthened by a meander; one
  // that is too long cannot be shortened, so the splice never overshoots by
  // more than the step it had to choose between.
  EXPECT_LE(after, 151.0);
}

TEST(CouplerInsertion, AnAllowedAreaMovesTheCouplerAlongItsResonator) {
  Path unrestricted = straightRun(400, 300, 6, 300);
  const std::optional<CouplerSplice> free =
      spliceCouplerDogleg(PRIMITIVES, 150.0, unrestricted, 0);
  ASSERT_TRUE(free.has_value());

  // The window excludes where the coupler would otherwise land, so it
  // slides along the resonator instead of the placement being lost.
  Path restricted = straightRun(400, 300, 6, 300);
  const uint32_t forbidden = free->anchor.x;
  const std::optional<CouplerSplice> moved = spliceCouplerDogleg(
      PRIMITIVES, 150.0, restricted, 0, {},
      [&](const uint32_t x, const uint32_t) { return x < forbidden - 20 || x > forbidden + 20; });
  ASSERT_TRUE(moved.has_value());
  EXPECT_TRUE(moved->inAllowedArea);
  EXPECT_TRUE(moved->anchor.x < forbidden - 20 || moved->anchor.x > forbidden + 20);
}

TEST(CouplerInsertion, AnUnreachableWindowStillPlacesTheCoupler) {
  Path path = straightRun(400, 300, 6, 300);
  const std::optional<CouplerSplice> splice = spliceCouplerDogleg(
      PRIMITIVES, 150.0, path, 0, {}, [](uint32_t, uint32_t) { return false; });
  // The best collision-free placement is used anyway, and the caller is
  // told that it is outside the window, which is a named miss rather than a
  // failed run.
  ASSERT_TRUE(splice.has_value());
  EXPECT_FALSE(splice->inAllowedArea);
}

TEST(CouplerInsertion, ASecondDoglegOffersAnotherPlacement) {
  Path single = straightRun(400, 300, 6, 300);
  const std::optional<CouplerSplice> one =
      spliceCouplerDogleg(PRIMITIVES, 150.0, single, 0, {.straightLength = 14});
  ASSERT_TRUE(one.has_value());

  // The S-jog turns back on itself, so it reaches across the resonator and
  // puts the coupler somewhere the single dogleg cannot reach.
  Path jogged = straightRun(400, 300, 6, 300);
  const std::optional<CouplerSplice> jog = spliceCouplerDogleg(
      PRIMITIVES, 150.0, jogged, 0,
      {.straightLength = 14, .secondStraightLength = 10, .secondTurnReverse = true});
  ASSERT_TRUE(jog.has_value());
  EXPECT_NE(jog->anchor.x, one->anchor.x);
  EXPECT_NEAR(reconstructSegments(PRIMITIVES, jogged).nominalLength, 150.0, 3.0);

  // The hook turns twice the same way, so its second straight run lies
  // along the resonator: on a straight wire it lands the coupler exactly
  // where the single dogleg does, only further along the wire.
  Path hooked = straightRun(400, 300, 6, 300);
  const std::optional<CouplerSplice> hook = spliceCouplerDogleg(
      PRIMITIVES, 150.0, hooked, 0,
      {.straightLength = 14, .secondStraightLength = 10, .secondTurnReverse = false});
  ASSERT_TRUE(hook.has_value());
  EXPECT_EQ(hook->anchor.x, one->anchor.x);
  EXPECT_EQ(hook->anchor.y, one->anchor.y);
}

TEST(CouplerInsertion, TheMirroredDoglegTurnsTheOtherWay) {
  Path plain = straightRun(400, 300, 6, 300);
  Path mirrored = straightRun(400, 300, 6, 300);
  const std::optional<CouplerSplice> a =
      spliceCouplerDogleg(PRIMITIVES, 150.0, plain, 0, {.mirrored = false});
  const std::optional<CouplerSplice> b =
      spliceCouplerDogleg(PRIMITIVES, 150.0, mirrored, 0, {.mirrored = true});
  ASSERT_TRUE(a.has_value());
  ASSERT_TRUE(b.has_value());
  // The two doglegs leave the resonator on opposite sides.
  EXPECT_NE(a->anchor.y, b->anchor.y);
}

TEST(CouplerInsertion, ThereIsNothingToSpliceOntoAnEmptyPath) {
  Path empty;
  EXPECT_FALSE(spliceCouplerDogleg(PRIMITIVES, 150.0, empty, 0).has_value());
  Path path = straightRun(400, 300, 6, 10);
  EXPECT_THROW(static_cast<void>(spliceCouplerDogleg(PRIMITIVES, 150.0, path, 9)),
               std::invalid_argument);
}

} // namespace
