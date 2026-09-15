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
#include "mqt-scpd/routing/MeanderInsertion.hpp"
#include "mqt-scpd/routing/Path.hpp"
#include "mqt-scpd/routing/PathGeometry.hpp"
#include "mqt-scpd/routing/Primitives.hpp"
#include "mqt-scpd/routing/SelfIntersection.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <cstdlib>

namespace {

using namespace mqt::scpd::routing;

const MovePrimitives PRIMITIVES(5);
constexpr uint32_t WIDTH = 400;
constexpr uint32_t HEIGHT = 400;

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

/// The whole grid, with nothing in the way.
MeanderOptions everywhere() {
  return {.width = WIDTH,
          .height = HEIGHT,
          .box = {.minX = 0, .maxX = WIDTH - 1, .minY = 0, .maxY = HEIGHT - 1}};
}

bool anyCell(uint32_t /*x*/, uint32_t /*y*/) { return true; }

/// A path a reader can take: every step moves by at most one cell along each
/// axis, every cell names a move of its heading, the rendering measures what
/// the result says, and the path does not meet itself.
void expectWellFormed(const Path& path, const MeanderResult& result) {
  for (std::size_t at = 0; at < path.size(); ++at) {
    const PathPoint& point = path[at];
    EXPECT_NE(PRIMITIVES.find(point.heading, point.primitive), nullptr)
        << "cell " << at << " is tagged with a move its heading lacks";
    if (at == 0) {
      continue;
    }
    const PathPoint& before = path[at - 1];
    EXPECT_LE(std::abs(static_cast<int64_t>(point.x) - before.x), 1) << at;
    EXPECT_LE(std::abs(static_cast<int64_t>(point.y) - before.y), 1) << at;
    EXPECT_FALSE(point.samePlace(before)) << "cell " << at << " repeats";
  }
  EXPECT_NEAR(renderedLength(PRIMITIVES, path), result.lengthAfter, 1e-9);
  PathLoopScratch scratch;
  EXPECT_FALSE(pathSelfIntersects(path, WIDTH, HEIGHT, scratch));
}

TEST(MeanderInsertion, LengthensAStraightRunToWhatIsRequired) {
  Path path = straightRun(100, 200, 6, 150);
  const Path before = path;
  const auto result =
      insertMeander(PRIMITIVES, path, 400.0, anyCell, everywhere());

  EXPECT_TRUE(result.reached);
  EXPECT_TRUE(result.inserted);
  EXPECT_NEAR(result.lengthBefore, 150.0, 0.01);
  EXPECT_GE(result.lengthAfter, 400.0);
  // One loop: the length it adds is what was missing, within the rounding
  // of its depth and the turns onto and off the axis.
  EXPECT_LT(result.lengthAfter, 400.0 + 40.0);
  EXPECT_GT(result.fits, 0U);
  EXPECT_EQ(path.front(), before.front());
  EXPECT_TRUE(path.back().samePlace(before.back()));
  expectWellFormed(path, result);
}

TEST(MeanderInsertion, LeavesALongEnoughPathAlone) {
  Path path = straightRun(100, 200, 6, 150);
  const Path before = path;
  const auto result =
      insertMeander(PRIMITIVES, path, 120.0, anyCell, everywhere());

  EXPECT_TRUE(result.reached);
  EXPECT_FALSE(result.inserted);
  EXPECT_EQ(result.candidates, 0U);
  EXPECT_EQ(path, before);
}

TEST(MeanderInsertion, MakesALoopOfASmallDeficitAsWell) {
  // A deficit of ten cells is lengthened by the margin, so that the loop has
  // legs at all; what matters is that the requirement is met.
  Path path = straightRun(100, 200, 6, 150);
  const auto result =
      insertMeander(PRIMITIVES, path, 160.0, anyCell, everywhere());

  EXPECT_TRUE(result.reached);
  EXPECT_TRUE(result.inserted);
  EXPECT_GE(result.lengthAfter, 160.0);
  expectWellFormed(path, result);
}

TEST(MeanderInsertion, FindsNoRoomWhereNoCellMayBeEntered) {
  Path path = straightRun(100, 200, 6, 150);
  const Path before = path;
  const auto onTheRunOnly = [](const uint32_t /*x*/, const uint32_t y) {
    return y == 200;
  };
  const auto result =
      insertMeander(PRIMITIVES, path, 400.0, onTheRunOnly, everywhere());

  EXPECT_FALSE(result.reached);
  EXPECT_FALSE(result.inserted);
  EXPECT_GT(result.candidates, 0U);
  EXPECT_EQ(result.fits, 0U);
  EXPECT_EQ(path, before);
}

TEST(MeanderInsertion, KeepsTheLoopOnTheSideItMayUse) {
  for (const bool below : {true, false}) {
    Path path = straightRun(100, 200, 6, 150);
    const auto oneSide = [below](const uint32_t /*x*/, const uint32_t y) {
      return below ? y >= 200 : y <= 200;
    };
    const auto result =
        insertMeander(PRIMITIVES, path, 400.0, oneSide, everywhere());

    ASSERT_TRUE(result.reached) << (below ? "below" : "above");
    for (const PathPoint& point : path) {
      EXPECT_TRUE(oneSide(point.x, point.y))
          << "(" << point.x << ", " << point.y << ")";
    }
    expectWellFormed(path, result);
  }
}

TEST(MeanderInsertion, WorksAlongEveryHeading) {
  for (Heading heading = 0; heading < NUM_HEADINGS; ++heading) {
    const HeadingVector v = headingVector(heading);
    const uint32_t x0 = v.dx < 0 ? 300 : 100;
    const uint32_t y0 = v.dy < 0 ? 300 : 100;
    Path path = straightRun(x0, y0, heading, 120);
    const Path before = path;
    const auto result =
        insertMeander(PRIMITIVES, path, 350.0, anyCell, everywhere());

    EXPECT_TRUE(result.reached) << "heading " << static_cast<int>(heading);
    EXPECT_GE(result.lengthAfter, 350.0) << static_cast<int>(heading);
    EXPECT_EQ(path.front(), before.front()) << static_cast<int>(heading);
    EXPECT_TRUE(path.back().samePlace(before.back()))
        << static_cast<int>(heading);
    expectWellFormed(path, result);
  }
}

TEST(MeanderInsertion, StaysInsideTheBox) {
  Path path = straightRun(100, 200, 6, 150);
  MeanderOptions options = everywhere();
  options.box = {.minX = 90, .maxX = 260, .minY = 150, .maxY = 400};
  const auto result = insertMeander(PRIMITIVES, path, 400.0, anyCell, options);

  ASSERT_TRUE(result.reached);
  for (const PathPoint& point : path) {
    EXPECT_GE(point.y, 150U + options.safety);
    EXPECT_LE(point.x, 260U);
  }
}

TEST(MeanderInsertion, ThePricedInsertionTakesTheCheaperSide) {
  for (const bool dearAbove : {true, false}) {
    Path path = straightRun(100, 200, 6, 150);
    const auto price = [dearAbove](const uint32_t /*x*/, const uint32_t y) {
      return static_cast<uint32_t>((dearAbove ? y < 200 : y > 200) ? 100 : 0);
    };
    const auto result =
        insertMeander(PRIMITIVES, path, 400.0, anyCell, everywhere(), price);

    ASSERT_TRUE(result.reached);
    EXPECT_GT(result.fits, 1U);
    for (const PathPoint& point : path) {
      EXPECT_EQ(price(point.x, point.y), 0U)
          << "(" << point.x << ", " << point.y << ")";
    }
    expectWellFormed(path, result);
  }
}

TEST(MeanderInsertion, PutsTheLoopAsNearTheEndAsItFits) {
  // With nothing in its way the loop takes the end of the run: the first
  // hundred cells stay as they were and every cell off the run lies beyond
  // them.
  Path path = straightRun(100, 200, 6, 150);
  const auto result =
      insertMeander(PRIMITIVES, path, 400.0, anyCell, everywhere());

  ASSERT_TRUE(result.reached);
  for (std::size_t at = 0; at < 100; ++at) {
    EXPECT_EQ(path[at].x, 100 + at);
    EXPECT_EQ(path[at].y, 200U);
  }
  uint32_t leftmostOffTheRun = 250;
  for (const PathPoint& point : path) {
    if (point.y != 200) {
      leftmostOffTheRun = std::min(leftmostOffTheRun, point.x);
    }
  }
  EXPECT_GE(leftmostOffTheRun, 200U);

  // Where the end of the run is closed, the loop moves back along it.
  Path blocked = straightRun(100, 200, 6, 150);
  const auto notNearTheEnd = [](const uint32_t x, const uint32_t y) {
    return y == 200 || x < 180;
  };
  const auto moved =
      insertMeander(PRIMITIVES, blocked, 400.0, notNearTheEnd, everywhere());
  ASSERT_TRUE(moved.reached);
  for (const PathPoint& point : blocked) {
    EXPECT_TRUE(notNearTheEnd(point.x, point.y))
        << "(" << point.x << ", " << point.y << ")";
  }
}

TEST(MeanderInsertion, LeavesTheFirstCellsOfThePathAlone) {
  Path path = straightRun(100, 200, 6, 150);
  MeanderOptions options = everywhere();
  options.startMargin = 20;
  const auto result = insertMeander(PRIMITIVES, path, 400.0, anyCell, options);

  ASSERT_TRUE(result.reached);
  for (std::size_t at = 0; at < 20; ++at) {
    EXPECT_EQ(path[at].x, 100 + at);
    EXPECT_EQ(path[at].y, 200U);
  }

  // A margin as wide as the run leaves no pair to try.
  Path untouched = straightRun(100, 200, 6, 150);
  options.startMargin = 150;
  const auto none =
      insertMeander(PRIMITIVES, untouched, 400.0, anyCell, options);
  EXPECT_FALSE(none.reached);
  EXPECT_EQ(none.candidates, 0U);
}

} // namespace
