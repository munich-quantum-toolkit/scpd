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

/// How many cells two boxes share.
uint64_t sharedCells(const CellBox& a, const CellBox& b) {
  const auto low = [](const uint32_t p, const uint32_t q) {
    return std::max(p, q);
  };
  const auto high = [](const uint32_t p, const uint32_t q) {
    return std::min(p, q);
  };
  if (low(a.minX, b.minX) > high(a.maxX, b.maxX) ||
      low(a.minY, b.minY) > high(a.maxY, b.maxY)) {
    return 0;
  }
  return static_cast<uint64_t>(high(a.maxX, b.maxX) - low(a.minX, b.minX) + 1) *
         (high(a.maxY, b.maxY) - low(a.minY, b.minY) + 1);
}

uint64_t cellsOf(const CellBox& box) {
  return static_cast<uint64_t>(box.maxX - box.minX + 1) *
         (box.maxY - box.minY + 1);
}

TEST(MeanderInsertion, OffersAlternativesThatLieApartFromTheMeander) {
  const auto free = [](const uint32_t /*x*/, const uint32_t /*y*/) {
    return 0U;
  };
  Path plain = straightRun(20, 200, 6, 360);
  const auto taken =
      insertMeander(PRIMITIVES, plain, 500.0, anyCell, everywhere(), free);
  ASSERT_TRUE(taken.inserted);
  EXPECT_TRUE(taken.alternatives.empty());

  Path path = straightRun(20, 200, 6, 360);
  const Path before = path;
  MeanderOptions options = everywhere();
  options.alternatives = 3;
  const auto result =
      insertMeander(PRIMITIVES, path, 500.0, anyCell, options, free);

  ASSERT_TRUE(result.inserted);
  // The meander taken is the one taken without alternatives.
  EXPECT_EQ(path, plain);
  ASSERT_EQ(result.alternatives.size(), 3U);
  // At most a quarter of the smaller box is shared with the meander taken
  // or with another alternative.
  const auto apart = [](const CellBox& a, const CellBox& b) {
    return 4 * sharedCells(a, b) <= std::min(cellsOf(a), cellsOf(b));
  };
  // One of them is the loop on the other side of the run.
  const bool takenAbove = result.meander.minY < 200;
  bool otherSide = false;
  for (std::size_t at = 0; at < result.alternatives.size(); ++at) {
    const auto& alternative = result.alternatives[at];
    EXPECT_GE(alternative.lengthAfter, 500.0);
    EXPECT_NEAR(renderedLength(PRIMITIVES, alternative.path),
                alternative.lengthAfter, 1e-9);
    EXPECT_EQ(alternative.path.front(), before.front());
    EXPECT_EQ(alternative.path.back(), before.back());
    EXPECT_TRUE(apart(alternative.meander, result.meander)) << at;
    for (std::size_t other = 0; other < at; ++other) {
      EXPECT_TRUE(
          apart(alternative.meander, result.alternatives[other].meander))
          << at << " and " << other;
    }
    otherSide |= (alternative.meander.minY < 200) != takenAbove;
    PathLoopScratch scratch;
    EXPECT_FALSE(pathSelfIntersects(alternative.path, WIDTH, HEIGHT, scratch));
  }
  EXPECT_TRUE(otherSide);
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

TEST(MeanderInsertion, LeavesThePartBeforeTheKeptIndexAlone) {
  // The loop goes behind the index, where a cut further along keeps it.
  Path path = straightRun(100, 200, 6, 300);
  const Path before = path;
  MeanderOptions options = everywhere();
  options.keepBefore = 120;
  const auto result = insertMeander(PRIMITIVES, path, 450.0, anyCell, options);

  ASSERT_TRUE(result.reached);
  for (std::size_t at = 0; at < 120; ++at) {
    EXPECT_EQ(path[at], before[at]) << "cell " << at;
  }
  expectWellFormed(path, result);

  // A kept part that covers the whole run leaves no pair to try.
  Path untouched = straightRun(100, 200, 6, 300);
  options.keepBefore = 300;
  const auto none =
      insertMeander(PRIMITIVES, untouched, 450.0, anyCell, options);
  EXPECT_FALSE(none.reached);
  EXPECT_EQ(none.candidates, 0U);
}

TEST(MeanderInsertion, MeetsASmallDeficitWithinTheToleranceInTheExactMode) {
  // The flattest loop adds more than these deficits, but not more than the
  // tolerance allows past the required length: it is the loop to take.
  for (const double deficit : {11.0, 14.0, 18.0}) {
    Path path = straightRun(100, 200, 6, 150);
    MeanderOptions options = everywhere();
    options.exact = true;
    options.tolerance = 10.0;
    const double required = 150.0 + deficit;
    const auto result =
        insertMeander(PRIMITIVES, path, required, anyCell, options);

    EXPECT_TRUE(result.reached) << "deficit " << deficit;
    EXPECT_TRUE(result.inserted) << "deficit " << deficit;
    EXPECT_NEAR(result.lengthAfter, required, options.tolerance)
        << "deficit " << deficit;
    expectWellFormed(path, result);
  }
}

TEST(MeanderInsertion, KeepsTheLoopOnTheAxesOfADiagonalWay) {
  // A diagonal way with room for a loop along the axes takes that loop: a
  // loop along the diagonals is the last resort.
  for (const Heading heading : {1, 3, 5, 7}) {
    for (const bool exact : {false, true}) {
      Path path = straightRun(200, 200, heading, 100);
      MeanderOptions options = everywhere();
      options.exact = exact;
      options.tolerance = 10.0;
      const auto result =
          insertMeander(PRIMITIVES, path, 300.0, anyCell, options);

      ASSERT_TRUE(result.reached) << "heading " << int{heading};
      EXPECT_TRUE(result.inserted);
      if (exact) {
        EXPECT_NEAR(result.lengthAfter, 300.0, options.tolerance);
      } else {
        EXPECT_GE(result.lengthAfter, 300.0);
      }
      EXPECT_TRUE(std::ranges::any_of(
          path,
          [](const PathPoint& point) { return !isDiagonal(point.heading); }))
          << "heading " << int{heading};
      expectWellFormed(path, result);
    }
  }
}

TEST(MeanderInsertion, SplitsADeepLoopIntoFingersWhereTheBoxIsFlat) {
  // One loop for the missing length would reach far past the box; several
  // shallower fingers side by side stay inside it.
  Path path = straightRun(50, 200, 6, 300);
  MeanderOptions options = everywhere();
  options.box = {.minX = 0, .maxX = WIDTH - 1, .minY = 150, .maxY = 250};
  for (const bool exact : {false, true}) {
    Path way = path;
    options.exact = exact;
    options.tolerance = 10.0;
    const auto result = insertMeander(PRIMITIVES, way, 600.0, anyCell, options);

    ASSERT_TRUE(result.reached) << (exact ? "exact" : "at least");
    for (const PathPoint& point : way) {
      EXPECT_GE(point.y, 150U);
      EXPECT_LE(point.y, 250U);
    }
    expectWellFormed(way, result);
  }
}

TEST(MeanderInsertion, FindsRoomOnAShortDiagonalAfterAnAxisRun) {
  // A run along an axis the start margin keeps, then a diagonal too short
  // for a loop along an axis: the loop goes across the diagonal.
  Path path = straightRun(150, 100, 2, 25);
  const auto turn =
      std::ranges::find_if(PRIMITIVES.of(2), [](const Primitive& move) {
        return move.exitHeading == 3;
      });
  ASSERT_NE(turn, PRIMITIVES.of(2).end());
  const PathPoint corner = path.back();
  path.back().primitive = turn->id;
  for (const CellOffset& offset : turn->swept) {
    if ((offset.dx == 0 && offset.dy == 0) ||
        (offset.dx == turn->dx && offset.dy == turn->dy)) {
      continue;
    }
    path.push_back({.x = static_cast<uint32_t>(corner.x + offset.dx),
                    .y = static_cast<uint32_t>(corner.y + offset.dy),
                    .heading = 2,
                    .primitive = turn->id});
  }
  const Path diagonal =
      straightRun(static_cast<uint32_t>(corner.x + turn->dx),
                  static_cast<uint32_t>(corner.y + turn->dy), 3, 40);
  path.insert(path.end(), diagonal.begin(), diagonal.end());
  MeanderOptions options = everywhere();
  options.exact = true;
  options.tolerance = 10.0;
  options.startMargin = 25;
  const double before = renderedLength(PRIMITIVES, path);
  const auto result =
      insertMeander(PRIMITIVES, path, before + 120.0, anyCell, options);

  ASSERT_TRUE(result.reached);
  EXPECT_NEAR(result.lengthAfter, before + 120.0, options.tolerance);
  expectWellFormed(path, result);
}

} // namespace
