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
#include "mqt-scpd/routing/Primitives.hpp"
#include "mqt-scpd/routing/RoomRules.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <set>
#include <utility>
#include <vector>

namespace {

using namespace mqt::scpd::routing;

const MovePrimitives PRIMITIVES(5);

/// A straight run of a heading from a cell: `steps` steps, `steps + 1`
/// points, so `steps` of them step along the heading and are straight.
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

/// The cells of a dogleg laid with the start of its turn at a cell.
Path placed(const DoglegGeometry& dogleg, const int64_t x, const int64_t y) {
  Path path;
  for (const PathPoint& move : dogleg.path) {
    path.push_back(
        {.x = static_cast<uint32_t>(x + static_cast<int32_t>(move.x)),
         .y = static_cast<uint32_t>(y + static_cast<int32_t>(move.y)),
         .heading = move.heading,
         .primitive = move.primitive});
  }
  return path;
}

void append(Path& to, const Path& more) {
  to.insert(to.end(), more.begin(), more.end());
}

TEST(RoomRules, AStraightRunIsStraightEverywhereButItsLastCell) {
  const Path path = straightRun(100, 100, 6, 10);
  const auto straight = straightCells(path);
  ASSERT_EQ(straight.size(), 11U);
  for (std::size_t at = 0; at < 10; ++at) {
    EXPECT_TRUE(straight[at]) << at;
  }
  EXPECT_FALSE(straight[10]);
}

TEST(RoomRules, ACellListedTwiceAtABendIsNoStraightOfItsOwn) {
  // The router lists a cell again where its heading changes on it: the
  // first listing steps off its own heading and the second steps along the
  // new one.
  Path path = straightRun(100, 100, 6, 3);
  path.push_back({.x = 103, .y = 100, .heading = 4, .primitive = 0});
  append(path, straightRun(103, 101, 4, 2));
  const auto straight = straightCells(path);
  // 100, 101, 102 step east; 103 listed east steps south — a bend; 103
  // listed south steps south — straight; then 101, 102 south; 103 is last.
  const std::vector<bool> expected{true, true, true, false,
                                   true, true, true, false};
  EXPECT_EQ(straight, expected);
}

TEST(RoomRules, TheLanesOfARunAtTheRulesFigures) {
  EXPECT_EQ(lanesOf(21, 20, 10), 1U);
  EXPECT_EQ(lanesOf(41, 20, 10), 2U);
  EXPECT_EQ(lanesOf(61, 20, 10), 3U);
  EXPECT_EQ(lanesOf(20, 20, 10), 0U);
  EXPECT_EQ(lanesOf(0, 20, 10), 0U);
  EXPECT_EQ(lanesOf(21, 20, 0), 2U);
  EXPECT_EQ(lanesOf(5, 20, 0), 1U);
  EXPECT_EQ(lanesOf(50, 19, 10), 2U);
  EXPECT_EQ(lanesOf(60, 19, 10), 3U);
}

TEST(RoomRules, ASixtyOneCellStraightCarriesThreeLanes) {
  const Path path = straightRun(100, 100, 6, 61);
  const auto capacity = crossingCapacity(path, 0, 0, 20, 10);
  EXPECT_EQ(capacity.lanes, 3U);
  ASSERT_EQ(capacity.runs.size(), 1U);
  EXPECT_EQ(capacity.runs[0], 61U);
}

TEST(RoomRules, TwoStraightsAroundAQuarterTurnCarryOneEach) {
  // Twenty-four cells on heading 6, a quarter turn with the clock onto
  // heading 0, and twenty-four cells on that. Each arm is one lane at a
  // margin of 10 and nothing at a margin of 15. The first cells of the turn
  // still step along the entry heading and count as straight, as the
  // crossing rule counts them, so an arm may run a few cells longer than
  // its straight.
  const DoglegGeometry turn = buildDogleg(PRIMITIVES, 6, 1, 0);
  ASSERT_EQ(turn.tip.heading, 0);
  Path path = straightRun(100, 200, 6, 23);
  const Path arc = placed(turn, 124, 200);
  append(path, arc);
  const PathPoint& tip = arc.back();
  const HeadingVector onward = headingVector(0);
  append(path, straightRun(tip.x + onward.dx, tip.y + onward.dy, 0, 23));

  const auto loose = crossingCapacity(path, 0, 0, 20, 10);
  EXPECT_EQ(loose.lanes, 2U);
  ASSERT_GE(loose.runs.size(), 2U);
  EXPECT_GE(loose.runs.front(), 23U);
  EXPECT_LE(loose.runs.front(), 31U);
  EXPECT_GE(loose.runs.back(), 23U);
  EXPECT_LE(loose.runs.back(), 31U);

  const auto tight = crossingCapacity(path, 0, 0, 20, 15);
  EXPECT_EQ(tight.lanes, 0U);
}

TEST(RoomRules, TheSkipsTakeTheStubsOutOfTheCount) {
  const Path path = straightRun(100, 100, 6, 81);
  EXPECT_EQ(crossingCapacity(path, 0, 0, 20, 10).lanes, 4U);
  const auto less = crossingCapacity(path, 20, 20, 20, 10);
  EXPECT_EQ(less.lanes, 2U);
  ASSERT_EQ(less.runs.size(), 1U);
  EXPECT_EQ(less.runs[0], 42U);
  // Skips that swallow the way leave nothing.
  const auto nothing = crossingCapacity(path, 50, 50, 20, 10);
  EXPECT_EQ(nothing.lanes, 0U);
  EXPECT_TRUE(nothing.runs.empty());
}

TEST(RoomRules, AnArcAloneCarriesNothing) {
  // A quarter turn of radius 5 sweeps a handful of cells, and the first of
  // them step along the entry heading; no run of them reaches the margin.
  const DoglegGeometry turn = buildDogleg(PRIMITIVES, 0, -1, 0);
  const Path arc = placed(turn, 300, 300);
  ASSERT_FALSE(arc.empty());
  const auto capacity = crossingCapacity(arc, 0, 0, 20, 10);
  EXPECT_EQ(capacity.lanes, 0U);
  for (const auto run : capacity.runs) {
    EXPECT_LT(run, 8U);
  }
}

TEST(RoomRules, TwoParallelStraightsAreTheirDistanceApart) {
  // One arm runs east along y = 100 and ends at (200, 100); the other
  // leaves (200, 144) westwards. Nothing skipped, everything taken.
  const Path a = straightRun(100, 100, 6, 100);
  const Path b = straightRun(200, 144, 2, 100);
  const auto channel = channelBetween(a, 0, a.size(), b, 0, b.size());
  ASSERT_TRUE(channel.measured);
  EXPECT_DOUBLE_EQ(channel.gap, 44.0);
  EXPECT_DOUBLE_EQ(channel.startGap, 44.0);
  EXPECT_EQ(a[channel.at].y, 100U);
  EXPECT_EQ(b[channel.bt].y, 144U);
  EXPECT_EQ(a[channel.at].x, b[channel.bt].x);
  EXPECT_EQ(channel.aDepth, a.size() - 1 - channel.at);
  EXPECT_EQ(channel.bDepth, channel.bt);
}

TEST(RoomRules, TheSkipsAndTheTakesBoundTheArms) {
  // The arms converge towards the pad: a runs north-east into the pad run,
  // b runs south-east out of it, so their nearest cells are at the skips.
  Path a = straightRun(100, 160, 7, 40);
  const PathPoint aTip = a.back();
  const HeadingVector east = headingVector(6);
  append(a, straightRun(aTip.x + east.dx, aTip.y + east.dy, 6, 21));
  Path b = straightRun(a.back().x + 22, a.back().y, 6, 21);
  const PathPoint bTip = b.back();
  const HeadingVector southEast = headingVector(5);
  append(b, straightRun(bTip.x + southEast.dx, bTip.y + southEast.dy, 5, 40));

  const auto whole = channelBetween(a, 22, 100, b, 22, 200);
  ASSERT_TRUE(whole.measured);
  // The nearest pair is the first cell of each arm past its skip, so the
  // arms never come closer than where they start: no channel narrows.
  EXPECT_EQ(whole.aDepth, 0U);
  EXPECT_EQ(whole.bDepth, 0U);
  EXPECT_DOUBLE_EQ(whole.gap, whole.startGap);
  EXPECT_EQ(whole.at, a.size() - 23);
  EXPECT_EQ(whole.bt, 22U);
  // Taking fewer cells of the arms cannot find a nearer pair, and taking
  // none of the second arm measures nothing.
  const auto some = channelBetween(a, 22, 5, b, 22, 5);
  ASSERT_TRUE(some.measured);
  EXPECT_DOUBLE_EQ(some.gap, whole.gap);
  EXPECT_FALSE(channelBetween(a, 22, 5, b, b.size(), 5).measured);
  EXPECT_FALSE(channelBetween(a, a.size(), 5, b, 22, 5).measured);
}

TEST(RoomRules, AUThatFoldsBackOverThePadNarrowsBehindIt) {
  // The shape the rule exists for: the in-edge comes west above the pad,
  // turns down onto the pad's axis and runs onto the in port; the out-edge
  // leaves the out port, turns up and runs back west higher up. Behind the
  // pad the two run stacked, thirty cells apart, where they left their pad
  // runs the pad's span apart.
  const DoglegGeometry westToDown = buildDogleg(PRIMITIVES, 2, -1, 0);
  ASSERT_EQ(westToDown.tip.heading, 0);
  const DoglegGeometry downToEast = buildDogleg(PRIMITIVES, 0, -1, 0);
  ASSERT_EQ(downToEast.tip.heading, 6);
  const DoglegGeometry eastToUp = buildDogleg(PRIMITIVES, 6, -1, 0);
  ASSERT_EQ(eastToUp.tip.heading, 4);
  const DoglegGeometry upToWest = buildDogleg(PRIMITIVES, 4, -1, 0);
  ASSERT_EQ(upToWest.tip.heading, 2);
  const auto step = [](const PathPoint& from, const Heading heading) {
    const HeadingVector v = headingVector(heading);
    return PathPoint{.x = static_cast<uint32_t>(from.x + v.dx),
                     .y = static_cast<uint32_t>(from.y + v.dy),
                     .heading = heading};
  };

  Path a = straightRun(400, 330, 2, 60);
  Path arc = placed(westToDown, a.back().x, a.back().y);
  append(a, arc);
  PathPoint next = step(arc.back(), 0);
  append(a, straightRun(next.x, next.y, 0, 17));
  arc = placed(downToEast, a.back().x, a.back().y);
  append(a, arc);
  next = step(arc.back(), 6);
  append(a, straightRun(next.x, next.y, 6, 21));
  const PathPoint in = a.back();

  Path b = straightRun(in.x + 22, in.y, 6, 21);
  const PathPoint out = b.back();
  arc = placed(eastToUp, out.x, out.y);
  append(b, arc);
  next = step(arc.back(), 4);
  append(b, straightRun(next.x, next.y, 4, 49));
  arc = placed(upToWest, b.back().x, b.back().y);
  append(b, arc);
  next = step(arc.back(), 2);
  append(b, straightRun(next.x, next.y, 2, 100));

  const auto channel = channelBetween(a, 22, 80, b, 22, 160);
  ASSERT_TRUE(channel.measured);
  EXPECT_NEAR(channel.startGap, 65.0, 2.0);
  EXPECT_LT(channel.gap, channel.startGap);
  EXPECT_NEAR(channel.gap, static_cast<double>(b.back().y) - 330.0, 1.0);
  EXPECT_GT(channel.aDepth, 0U);
  EXPECT_GT(channel.bDepth, 0U);
  // Both arms lie on the far-edge side of the pad: `across` points from
  // the pad's centre to the feedline edge.
  const PathPoint centre{.x = in.x + 11, .y = in.y - 2, .heading = 6};
  const HeadingVector across = headingVector(4);
  EXPECT_TRUE(sameSide(centre, across, a[channel.at], b[channel.bt]));
  EXPECT_TRUE(sameSide(centre, across, a.front(), b.back()));
}

TEST(RoomRules, TheSideOfAPointHasASign) {
  const PathPoint centre{.x = 100, .y = 100, .heading = 0};
  const HeadingVector north = headingVector(0);
  EXPECT_EQ(sideOf(centre, north, {.x = 100, .y = 50}), 1);
  EXPECT_EQ(sideOf(centre, north, {.x = 100, .y = 150}), -1);
  EXPECT_EQ(sideOf(centre, north, {.x = 130, .y = 100}), 0);
  EXPECT_TRUE(sameSide(centre, north, {.x = 90, .y = 50}, {.x = 110, .y = 60}));
  EXPECT_FALSE(
      sameSide(centre, north, {.x = 90, .y = 50}, {.x = 110, .y = 150}));
  EXPECT_FALSE(
      sameSide(centre, north, {.x = 130, .y = 100}, {.x = 110, .y = 50}));
}

TEST(RoomRules, ASupercoverLineVisitsEveryCellItTouches) {
  std::vector<std::pair<int64_t, int64_t>> cells;
  const auto collect = [&](const int64_t x, const int64_t y) {
    cells.emplace_back(x, y);
  };
  // Along an axis: exactly the cells of the row.
  supercoverLine(0, 0, 5, 0, collect);
  ASSERT_EQ(cells.size(), 6U);
  for (int64_t x = 0; x <= 5; ++x) {
    EXPECT_EQ(cells[static_cast<std::size_t>(x)],
              std::make_pair(x, int64_t{0}));
  }
  // Through the corners exactly: both cells beside every corner.
  cells.clear();
  supercoverLine(0, 0, 2, 2, collect);
  const std::set<std::pair<int64_t, int64_t>> diagonal(cells.begin(),
                                                       cells.end());
  EXPECT_EQ(diagonal.size(), 7U);
  EXPECT_TRUE(diagonal.contains({1, 0}));
  EXPECT_TRUE(diagonal.contains({0, 1}));
  EXPECT_TRUE(diagonal.contains({1, 1}));
  EXPECT_TRUE(diagonal.contains({2, 2}));
  // A shallow line in every direction: both ends in, every step a
  // neighbour of the one before, and at least one cell per column.
  for (const auto [x1, y1] :
       {std::pair{7, 3}, std::pair{-7, 3}, std::pair{7, -3}, std::pair{-7, -3},
        std::pair{3, 7}, std::pair{-3, 7}, std::pair{3, -7},
        std::pair{-3, -7}}) {
    cells.clear();
    supercoverLine(0, 0, x1, y1, collect);
    ASSERT_GE(cells.size(),
              static_cast<std::size_t>(std::max(std::abs(x1), std::abs(y1))) +
                  1);
    EXPECT_EQ(cells.front(), std::make_pair(int64_t{0}, int64_t{0}));
    EXPECT_EQ(cells.back(), std::make_pair(int64_t{x1}, int64_t{y1}));
    for (std::size_t at = 1; at < cells.size(); ++at) {
      EXPECT_LE(std::abs(cells[at].first - cells[at - 1].first), 1);
      EXPECT_LE(std::abs(cells[at].second - cells[at - 1].second), 1);
    }
  }
}

} // namespace
