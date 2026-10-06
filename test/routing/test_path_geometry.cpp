/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "../SplitMix.hpp"
#include "mqt-scpd/flatbuffers/geometry.hpp"
#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/routing/CouplerInsertion.hpp"
#include "mqt-scpd/routing/DubinsRouter.hpp"
#include "mqt-scpd/routing/Heading.hpp"
#include "mqt-scpd/routing/Path.hpp"
#include "mqt-scpd/routing/PathGeometry.hpp"
#include "mqt-scpd/routing/Primitives.hpp"
#include "mqt-scpd/routing/SearchScratch.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <numbers>
#include <utility>
#include <vector>

namespace {

using mqt::scpd::test::SplitMix;
using namespace mqt::scpd;
using namespace mqt::scpd::routing;

/// The move primitives of bend radius 5, built once for every test.
const MovePrimitives& primitives() {
  static const MovePrimitives PRIMITIVES(5);
  return PRIMITIVES;
}

/// A straight run of a heading from a cell, one step per cell.
Path straightRun(const uint32_t x0, const uint32_t y0, const Heading heading,
                 const uint32_t steps) {
  return routing::straightRun(primitives(),
                              {.x = x0, .y = y0, .heading = heading}, steps);
}

/// Appends a turn in the format of Path. The turn takes over the last point
/// of the path, which becomes the start of its arc, and lists the other
/// cells it sweeps. A last swept cell on the end of the arc is left out,
/// because the next move starts there, as in a routed path.
/// @return The end of the arc.
PathPoint appendTurn(Path& path, const Primitive& turn) {
  path.back().primitive = turn.id;
  const PathPoint start = path.back();
  for (std::size_t k = 0; k < turn.swept.size(); ++k) {
    const CellOffset& c = turn.swept[k];
    const bool atEnd = c.dx == turn.dx && c.dy == turn.dy;
    if (c == CellOffset{} || (atEnd && k + 1 == turn.swept.size())) {
      continue;
    }
    path.push_back({.x = static_cast<uint32_t>(start.x + c.dx),
                    .y = static_cast<uint32_t>(start.y + c.dy),
                    .heading = start.heading,
                    .primitive = turn.id});
  }
  return {.x = static_cast<uint32_t>(start.x + turn.dx),
          .y = static_cast<uint32_t>(start.y + turn.dy),
          .heading = turn.exitHeading,
          .primitive = primitives().straight(turn.exitHeading)};
}

/// The first move of a heading that turns by a number of eighths.
const Primitive* turnOf(const Heading heading, const uint32_t eighths) {
  for (const Primitive& p : primitives().of(heading)) {
    if (headingDistance(heading, p.exitHeading) == eighths) {
      return &p;
    }
  }
  return nullptr;
}

/// An eighth turn of heading 6 whose last swept cell lies beyond the end of
/// its arc.
const Primitive* eighthBeyondItsEnd() {
  for (const Primitive& p : primitives().of(6)) {
    if (headingDistance(6, p.exitHeading) == 1 &&
        p.swept.back() != CellOffset{.dx = p.dx, .dy = p.dy}) {
      return &p;
    }
  }
  return nullptr;
}

/// The corners of a rendering, between consecutive pieces of the polyline
/// that are at least 0.05 cell long. A kink turns by more than 60 degrees
/// (cosine below 0.5), a reversal by more than 120 degrees (cosine below
/// -0.5). A reversal is also a kink.
struct Corners {
  int kinks = 0;
  int reversals = 0;
};

Corners cornersOf(const std::vector<Point>& points) {
  constexpr double minPiece = 0.05;
  Corners corners;
  bool havePrevious = false;
  double previousX = 0.0;
  double previousY = 0.0;
  for (std::size_t k = 1; k < points.size(); ++k) {
    const double dx = points[k].x() - points[k - 1].x();
    const double dy = points[k].y() - points[k - 1].y();
    const double length = std::hypot(dx, dy);
    if (length < minPiece) {
      continue;
    }
    if (havePrevious) {
      const double cosine = ((dx * previousX) + (dy * previousY)) / length;
      corners.kinks += cosine < 0.5 ? 1 : 0;
      corners.reversals += cosine < -0.5 ? 1 : 0;
    }
    previousX = dx / length;
    previousY = dy / length;
    havePrevious = true;
  }
  return corners;
}

/// The rendering of a path from its first point.
std::vector<Point> rendering(Path path) {
  std::vector<PathSegment> segments;
  const PathPoint start = path.front();
  return samplePath(primitives(), path, start, segments);
}

/// The next number of a fixed sequence, so that random requests are the

TEST(PathGeometry, APointKnowsItsSearchState) {
  // The search state of a point is its cell and its heading. The primitive
  // names the move the point starts and is no part of the state.
  const PathPoint point{.x = 12, .y = 34, .heading = 5, .primitive = 7};
  const mqt::scpd::flatbuffers::geometry::RCoord state = point.state();
  EXPECT_EQ(state.x(), 12U);
  EXPECT_EQ(state.y(), 34U);
  EXPECT_EQ(state.heading(), 5U);
  const PathPoint other{.x = 12, .y = 34, .heading = 5, .primitive = 3};
  EXPECT_EQ(other.state(), state);
}

TEST(PathGeometry, AStraightRunIsOneSegment) {
  const Path path = straightRun(100, 100, 6, 20);
  const SegmentedPath segmented = reconstructSegments(primitives(), path);
  ASSERT_EQ(segmented.segments.size(), 1U);
  const PathSegment& segment = segmented.segments.front();
  EXPECT_TRUE(segment.straight());
  EXPECT_EQ(segment.heading, 6);
  EXPECT_EQ(segment.steps(), 21U);
  EXPECT_EQ(segment.lengthAt.size(), 21U);
  EXPECT_DOUBLE_EQ(segmented.nominalLength, 21.0);
}

TEST(PathGeometry, AHeadingChangeStartsANewSegment) {
  Path path = straightRun(100, 100, 6, 10);
  const Path second = straightRun(111, 100, 4, 10);
  path.insert(path.end(), second.begin(), second.end());
  const SegmentedPath segmented = reconstructSegments(primitives(), path);
  ASSERT_EQ(segmented.segments.size(), 2U);
  EXPECT_EQ(segmented.segments[0].heading, 6);
  EXPECT_EQ(segmented.segments[1].heading, 4);
  EXPECT_TRUE(segmented.segments[0].straight());
  EXPECT_TRUE(segmented.segments[1].straight());
  // The lengths accumulate across the segments.
  EXPECT_GT(segmented.segments[1].lengthAt.back(),
            segmented.segments[0].lengthAt.back());
}

TEST(PathGeometry, ATurnPrimitiveStaysOneStepAtTheEndOfItsArc) {
  // A turn ends at its first point plus the end offset of its primitive. The
  // last swept cell of this eighth turn lies beyond that end.
  const Primitive* eighth = eighthBeyondItsEnd();
  ASSERT_NE(eighth, nullptr);
  Path path = straightRun(100, 100, 6, 5);
  const PathPoint end = appendTurn(path, *eighth);
  EXPECT_EQ(end.x, static_cast<uint32_t>(105 + eighth->dx));
  EXPECT_EQ(end.y, static_cast<uint32_t>(100 + eighth->dy));
  const Path after = straightRun(end.x, end.y, end.heading, 5);
  path.insert(path.end(), after.begin(), after.end());

  const SegmentedPath segmented = reconstructSegments(primitives(), path);
  ASSERT_EQ(segmented.segments.size(), 3U);
  const PathSegment& turn = segmented.segments[1];
  EXPECT_FALSE(turn.straight());
  EXPECT_EQ(turn.steps(), 1U);
  EXPECT_TRUE(turn.cells[0].samePlace(end));

  // Without a state after it, the turn still ends at the end of its arc.
  path.resize(path.size() - after.size());
  const SegmentedPath cut = reconstructSegments(primitives(), path);
  ASSERT_EQ(cut.segments.size(), 2U);
  EXPECT_TRUE(cut.segments[1].cells[0].samePlace(end));
}

TEST(PathGeometry, ATurnFromTheLastStubCellCountsItsMoveOnce) {
  // Where the search of a routed path begins with a turn, the start of the
  // arc is the last cell of the source stub, which keeps the straight tag
  // (see Path). The move of that cell is the turn. The nominal length is
  // therefore the same as with the turn's tag on that cell, and the same as
  // for the route in the other direction, whose search ends with the turn.
  auto shared = std::make_shared<const MovePrimitives>(5);
  constexpr uint32_t width = 300;
  constexpr uint32_t height = 200;
  SearchScratch scratch(width, height);
  const grid::BitGrid corridor(width, height);
  DubinsRouter router(shared, scratch,
                      {.startStraightLength = 10,
                       .endStraightLength = 10,
                       .minRadius = 5,
                       .bendPenalty = 500});
  router.attachCorridor(&corridor);
  const Path routed = router.route(
      {.source = {.x = 30, .y = 100, .heading = 0, .primitive = 0},
       .target = {.x = 260, .y = 100, .heading = 6, .primitive = 0}});
  ASSERT_GT(routed.size(), 12U);
  // Ten steps of stub, then the turn.
  const PathPoint& stubEnd = routed[10];
  const PathPoint& turn = routed[11];
  ASSERT_TRUE(shared->isStraight(stubEnd.heading, stubEnd.primitive));
  ASSERT_FALSE(shared->isStraight(turn.heading, turn.primitive));
  ASSERT_EQ(stubEnd.heading, turn.heading);

  Path retagged = routed;
  retagged[10].primitive = turn.primitive;
  const SegmentedPath segmented = reconstructSegments(*shared, routed);
  EXPECT_NEAR(segmented.nominalLength,
              reconstructSegments(*shared, retagged).nominalLength, 1e-9);
  const Path reverse = router.route(
      {.source = {.x = 260, .y = 100, .heading = 2, .primitive = 0},
       .target = {.x = 30, .y = 100, .heading = 4, .primitive = 0}});
  ASSERT_FALSE(reverse.empty());
  EXPECT_NEAR(segmented.nominalLength,
              reconstructSegments(*shared, reverse).nominalLength, 1e-9);

  // The stub keeps its cells, and its last cell adds nothing.
  const PathSegment& stub = segmented.segments.front();
  ASSERT_EQ(stub.steps(), 11U);
  EXPECT_DOUBLE_EQ(stub.lengthAt[10], stub.lengthAt[9]);
}

TEST(PathGeometry, NoTurnSweepsAStepAgainstItsExitHeadingFirst) {
  // reconstructSegments() tells a turn whose arc starts on the straight point
  // before it from a turn in the regular form by the point after the turn.
  // The two forms would meet if the first cell a turn sweeps after its start
  // lay one step against its exit heading.
  for (uint32_t radius = 1; radius <= 23; ++radius) {
    const MovePrimitives table(radius);
    for (Heading heading = 0; heading < NUM_HEADINGS; ++heading) {
      for (const Primitive& p : table.of(heading)) {
        if (p.exitHeading == heading) {
          continue;
        }
        const auto first = std::ranges::find_if(
            p.swept, [](const CellOffset& c) { return c != CellOffset{}; });
        ASSERT_NE(first, p.swept.end());
        const HeadingVector exit = headingVector(p.exitHeading);
        EXPECT_FALSE(first->dx == -exit.dx && first->dy == -exit.dy)
            << "radius " << radius << ", heading " << static_cast<int>(heading)
            << ", primitive " << p.id;
      }
    }
  }
}

TEST(PathGeometry, AStraightStepFromTheEndOfAnArcRendersWithoutACorner) {
  // A path can go on with a straight step from the end of an arc, so that
  // the end is no point of the path. The first point after this eighth turn
  // lies one step past its end.
  const Primitive* eighth = eighthBeyondItsEnd();
  ASSERT_NE(eighth, nullptr);
  Path path = straightRun(100, 100, 6, 5);
  const PathPoint end = appendTurn(path, *eighth);
  const HeadingVector v = headingVector(end.heading);
  const Path after = straightRun(end.x + v.dx, end.y + v.dy, end.heading, 5);
  path.insert(path.end(), after.begin(), after.end());

  const SegmentedPath segmented = reconstructSegments(primitives(), path);
  ASSERT_EQ(segmented.segments.size(), 3U);
  EXPECT_TRUE(segmented.segments[1].cells[0].samePlace(end));

  const std::vector<Point> points = rendering(path);
  const Corners corners = cornersOf(points);
  EXPECT_EQ(corners.kinks, 0);
  EXPECT_EQ(corners.reversals, 0);
  EXPECT_DOUBLE_EQ(points.back().x(), static_cast<double>(path.back().x));
  EXPECT_DOUBLE_EQ(points.back().y(), static_cast<double>(path.back().y));
  // Five steps east, the arc, and six diagonal steps.
  EXPECT_NEAR(polylineLength(points),
              5.0 + polylineLength(eighth->samples) +
                  (6.0 * std::numbers::sqrt2),
              0.01);
}

TEST(PathGeometry, ATurnStraightAfterATurnRendersWithoutACorner) {
  // A dogleg lists each move from its start, the start of an arc included,
  // and the next move starts at the end of the move before it. A move that
  // starts on the last point of the path takes that point over. Here an
  // eighth turn whose last swept cell lies beyond its end is followed
  // directly by a second eighth turn, and then by straight steps.
  const Primitive* first = eighthBeyondItsEnd();
  ASSERT_NE(first, nullptr);
  const Primitive* second = turnOf(first->exitHeading, 1);
  ASSERT_NE(second, nullptr);
  Path path = straightRun(100, 100, 6, 5);
  const auto appendMove = [&](const PathPoint start, const Primitive& move) {
    for (std::size_t k = 0; k < move.swept.size(); ++k) {
      const PathPoint point{
          .x = static_cast<uint32_t>(start.x + move.swept[k].dx),
          .y = static_cast<uint32_t>(start.y + move.swept[k].dy),
          .heading = start.heading,
          .primitive = move.id};
      if (k == 0 && path.back().samePlace(point)) {
        path.back() = point;
      } else {
        path.push_back(point);
      }
    }
    return PathPoint{.x = static_cast<uint32_t>(start.x + move.dx),
                     .y = static_cast<uint32_t>(start.y + move.dy),
                     .heading = move.exitHeading,
                     .primitive = 0};
  };
  const PathPoint firstEnd = appendMove(path.back(), *first);
  ASSERT_FALSE(path.back().samePlace(firstEnd));
  const PathPoint secondEnd = appendMove(firstEnd, *second);
  PathPoint at = secondEnd;
  const Primitive* straight = primitives().find(
      secondEnd.heading, primitives().straight(secondEnd.heading));
  ASSERT_NE(straight, nullptr);
  for (int k = 0; k < 5; ++k) {
    at = appendMove(at, *straight);
  }

  const SegmentedPath segmented = reconstructSegments(primitives(), path);
  ASSERT_EQ(segmented.segments.size(), 4U);
  EXPECT_TRUE(segmented.segments[1].cells[0].samePlace(firstEnd));
  EXPECT_TRUE(segmented.segments[2].cells[0].samePlace(secondEnd));

  const std::vector<Point> points = rendering(path);
  const Corners corners = cornersOf(points);
  EXPECT_EQ(corners.kinks, 0);
  EXPECT_EQ(corners.reversals, 0);
  // Five steps east, the two arcs, and five straight steps. The samples of
  // the turn off a diagonal heading end a tenth of a cell short of its end
  // cell, and the rendering pulls them onto it.
  EXPECT_NEAR(polylineLength(points),
              5.0 + polylineLength(first->samples) +
                  polylineLength(second->samples) +
                  (5.0 * polylineLength(straight->samples)),
              0.15);
}

TEST(PathGeometry, RoutedTurnsRenderWithoutKinksOrReversals) {
  // Every routed turn ends on the state its move reached, which is the first
  // point after the turn, also where the search begins with the turn. The
  // rendering turns smoothly through it. The first request needs an eighth
  // turn that sweeps a cell beyond its end.
  auto shared = std::make_shared<const MovePrimitives>(5);
  constexpr uint32_t width = 300;
  constexpr uint32_t height = 200;
  SearchScratch scratch(width, height);
  const grid::BitGrid corridor(width, height);
  DubinsRouter router(shared, scratch,
                      {.startStraightLength = 10,
                       .endStraightLength = 10,
                       .minRadius = 5,
                       .bendPenalty = 500});
  router.attachCorridor(&corridor);

  std::vector<RoutingObjective> requests{
      {.source = {.x = 30, .y = 100, .heading = 6, .primitive = 0},
       .target = {.x = 260, .y = 139, .heading = 6, .primitive = 0}}};
  SplitMix random(1);
  const auto below = [&](const uint32_t n) {
    return static_cast<uint32_t>(random.next() % n);
  };
  for (int i = 0; i < 60; ++i) {
    RoutingObjective request;
    request.source = {.x = 20 + below(60),
                      .y = 20 + below(160),
                      .heading = static_cast<Heading>(below(8)),
                      .primitive = 0};
    request.target = {.x = 220 + below(60),
                      .y = 20 + below(160),
                      .heading = static_cast<Heading>(below(8)),
                      .primitive = 0};
    requests.push_back(request);
  }

  int eighths = 0;
  int quarters = 0;
  int atTheSearchStart = 0;
  for (std::size_t r = 0; r < requests.size(); ++r) {
    const Path path = router.route(requests[r]);
    ASSERT_FALSE(path.empty()) << r;
    const SegmentedPath segmented = reconstructSegments(*shared, path);
    std::size_t next = 0;
    for (const PathSegment& segment : segmented.segments) {
      // Step past the points of this segment, to the first point after it.
      const std::size_t first = next;
      while (next < path.size() && path[next].heading == segment.heading &&
             path[next].primitive == segment.primitive) {
        ++next;
      }
      const Primitive* move = shared->find(segment.heading, segment.primitive);
      ASSERT_NE(move, nullptr) << r;
      const uint32_t eighthsTurned =
          headingDistance(segment.heading, move->exitHeading);
      if (eighthsTurned == 0) {
        continue;
      }
      eighths += eighthsTurned == 1 ? 1 : 0;
      quarters += eighthsTurned == 2 ? 1 : 0;
      // The source stub has ten cells, so the search starts on point ten. A
      // turn there starts on point eleven, the next cell its arc sweeps.
      atTheSearchStart += first == 11 ? 1 : 0;
      ASSERT_LT(next, path.size()) << r;
      EXPECT_TRUE(segment.cells[0].samePlace(path[next])) << r;
    }
    const Corners corners = cornersOf(rendering(path));
    EXPECT_EQ(corners.kinks, 0) << r;
    EXPECT_EQ(corners.reversals, 0) << r;
  }
  EXPECT_GT(eighths, 0);
  EXPECT_GT(quarters, 0);
  EXPECT_GT(atTheSearchStart, 0);
}

TEST(PathGeometry, SamplingRendersTheStraightRunAtItsRealLength) {
  Path path = straightRun(100, 100, 6, 20);
  std::vector<PathSegment> segments;
  const std::vector<Point> samples =
      samplePath(primitives(), path, path.front(), segments);
  ASSERT_FALSE(samples.empty());
  EXPECT_DOUBLE_EQ(samples.front().x(), 100.0);
  EXPECT_DOUBLE_EQ(samples.back().x(), 120.0);
  EXPECT_DOUBLE_EQ(samples.back().y(), 100.0);
  EXPECT_NEAR(polylineLength(samples), 20.0, 1e-9);
  // The rendered length is what the segments report, so a caller that
  // targets a length reads the real one.
  EXPECT_NEAR(segments.back().lengthAt.back(), 20.0, 1e-9);
}

TEST(PathGeometry, SamplingSnapsEveryStepOntoItsGridCell) {
  const Primitive* quarter = nullptr;
  for (const Primitive& p : primitives().of(0)) {
    if (p.exitHeading == 6) {
      quarter = &p;
      break;
    }
  }
  ASSERT_NE(quarter, nullptr);
  Path path = straightRun(50, 80, 0, 10);
  const PathPoint end = appendTurn(path, *quarter);
  const Path after = straightRun(end.x, end.y, end.heading, 10);
  path.insert(path.end(), after.begin(), after.end());

  std::vector<PathSegment> segments;
  const std::vector<Point> samples =
      samplePath(primitives(), path, path.front(), segments);
  ASSERT_FALSE(samples.empty());
  // The rendering ends exactly on the last grid cell, so it never drifts.
  EXPECT_NEAR(samples.back().x(), static_cast<double>(path.back().x), 1e-9);
  EXPECT_NEAR(samples.back().y(), static_cast<double>(path.back().y), 1e-9);
  // The two straight runs plus the arc, which renders at its true length
  // rather than as the chord across its ends.
  const double length = polylineLength(samples);
  EXPECT_NEAR(length, 20.0 + (std::numbers::pi / 2.0 * 5.0), 0.05);
}

TEST(PathGeometry, SamplingFromTheSecondPointLeavesTheCallersPathAlone) {
  Path path = straightRun(100, 100, 6, 20);
  std::vector<PathSegment> a;
  std::vector<PathSegment> b;
  std::vector<std::pair<std::size_t, bool>> bounds;
  Path copy = path;
  const std::vector<Point> full =
      samplePath(primitives(), copy, copy.front(), a);
  const std::vector<Point> second =
      samplePathFromSecond(primitives(), path, path.front(), b, &bounds);
  // The first point is the start the rendering is measured from, so both
  // forms render the same polyline; this one does not touch the path it is
  // given, which is what a caller that reads the path afterwards needs.
  EXPECT_EQ(path.size(), 21U);
  EXPECT_NEAR(polylineLength(full), polylineLength(second), 1e-9);
  ASSERT_FALSE(bounds.empty());
  EXPECT_EQ(bounds.size(), b.size());
  EXPECT_TRUE(bounds.front().second);
}

TEST(PathGeometry, SamplingFromTheSecondPointKeepsTheStartOfAFirstTurn) {
  // A spliced path without a lead begins with the quarter turn of its dogleg,
  // and its first point is the start of the arc. The rendering needs that
  // point to find the end of the arc, so the point stays, and the polyline is
  // the one samplePath() draws.
  Path path = straightRun(400, 300, 6, 300);
  ASSERT_TRUE(spliceCouplerDogleg(primitives(), 150.0, path, 1200, 1200, 0)
                  .has_value());
  ASSERT_FALSE(
      primitives().isStraight(path.front().heading, path.front().primitive));
  Path copy = path;
  std::vector<PathSegment> a;
  std::vector<PathSegment> b;
  const std::vector<Point> full =
      samplePath(primitives(), copy, copy.front(), a);
  const std::vector<Point> second =
      samplePathFromSecond(primitives(), path, path.front(), b);
  ASSERT_EQ(second.size(), full.size());
  for (std::size_t i = 0; i < full.size(); ++i) {
    EXPECT_NEAR(second[i].x(), full[i].x(), 1e-9) << i;
    EXPECT_NEAR(second[i].y(), full[i].y(), 1e-9) << i;
  }
}

TEST(PathGeometry, BendsAreHeadingChanges) {
  EXPECT_EQ(countBends(straightRun(10, 10, 6, 20)), 0U);
  EXPECT_EQ(countBends(Path{}), 0U);
  Path path = straightRun(10, 10, 6, 10);
  const Path second = straightRun(21, 10, 4, 10);
  path.insert(path.end(), second.begin(), second.end());
  EXPECT_EQ(countBends(path), 1U);

  // A repeated cell on the same heading is no bend.
  Path repeated = straightRun(10, 10, 6, 10);
  repeated.insert(repeated.begin() + 5, repeated[5]);
  EXPECT_EQ(countBends(repeated), 0U);

  // A quarter turn is one bend, however many cells it sweeps.
  const Primitive* quarter = turnOf(6, 2);
  ASSERT_NE(quarter, nullptr);
  ASSERT_GT(quarter->swept.size(), 2U);
  Path turning = straightRun(100, 100, 6, 5);
  const PathPoint turned = appendTurn(turning, *quarter);
  const Path out = straightRun(turned.x, turned.y, turned.heading, 5);
  turning.insert(turning.end(), out.begin(), out.end());
  EXPECT_EQ(countBends(turning), 1U);

  // Two eighth turns in a row are two bends.
  const Primitive* first = turnOf(6, 1);
  ASSERT_NE(first, nullptr);
  const Primitive* then = turnOf(first->exitHeading, 1);
  ASSERT_NE(then, nullptr);
  Path twice = straightRun(100, 100, 6, 5);
  twice.push_back(appendTurn(twice, *first));
  const PathPoint secondEnd = appendTurn(twice, *then);
  const Path last = straightRun(secondEnd.x, secondEnd.y, secondEnd.heading, 5);
  twice.insert(twice.end(), last.begin(), last.end());
  EXPECT_EQ(countBends(twice), 2U);
}

TEST(PathGeometry, AnEmptyPathRendersNothing) {
  // Each call replaces the segments and the bounds an earlier call left.
  std::vector<PathSegment> segments;
  std::vector<std::pair<std::size_t, bool>> bounds;
  Path path = straightRun(100, 100, 6, 5);
  ASSERT_FALSE(samplePath(primitives(), path, path.front(), segments).empty());
  ASSERT_FALSE(segments.empty());
  Path empty;
  EXPECT_TRUE(samplePath(primitives(), empty, PathPoint{}, segments).empty());
  EXPECT_TRUE(segments.empty());

  // Without its first point, a one-point path has nothing left to render.
  ASSERT_FALSE(
      samplePathFromSecond(primitives(), path, path.front(), segments, &bounds)
          .empty());
  ASSERT_FALSE(bounds.empty());
  const Path single = straightRun(100, 100, 6, 0);
  EXPECT_TRUE(samplePathFromSecond(primitives(), single, single.front(),
                                   segments, &bounds)
                  .empty());
  EXPECT_TRUE(segments.empty());
  EXPECT_TRUE(bounds.empty());
}

TEST(PathGeometry, ABrokenPathJumpsToTheCellOfTheStep) {
  // The last step lands fifty-five cells from where a straight step ends, far
  // more than rounding explains, so the rendering jumps there instead of
  // stretching the step over the gap.
  Path path = straightRun(100, 100, 6, 5);
  path.push_back({.x = 160,
                  .y = 100,
                  .heading = 6,
                  .primitive = primitives().straight(6)});
  std::vector<PathSegment> segments;
  const std::vector<Point> samples =
      samplePath(primitives(), path, path.front(), segments);
  ASSERT_GE(samples.size(), 2U);
  EXPECT_DOUBLE_EQ(samples.back().x(), 160.0);
  EXPECT_DOUBLE_EQ(samples.back().y(), 100.0);
  EXPECT_DOUBLE_EQ(samples[samples.size() - 2].x(), 105.0);
  EXPECT_DOUBLE_EQ(samples[samples.size() - 2].y(), 100.0);
  EXPECT_NEAR(polylineLength(samples), 60.0, 1e-9);
  EXPECT_NEAR(segments.back().lengthAt.back(), 60.0, 1e-9);
}

TEST(PathGeometry, AStepWithoutAMoveInTheTablesStillEndsOnItsCell) {
  // Identifier 999 names no move of the tables, so the step has no curve to
  // render; the rendering still reaches its cell and measures what it drew.
  Path path = straightRun(100, 100, 6, 5);
  path.push_back({.x = 108, .y = 103, .heading = 6, .primitive = 999});
  std::vector<PathSegment> segments;
  const std::vector<Point> samples =
      samplePath(primitives(), path, path.front(), segments);
  ASSERT_FALSE(samples.empty());
  EXPECT_DOUBLE_EQ(samples.back().x(), 108.0);
  EXPECT_DOUBLE_EQ(samples.back().y(), 103.0);
  ASSERT_FALSE(segments.empty());
  EXPECT_NEAR(segments.back().lengthAt.back(), polylineLength(samples), 1e-9);
}

TEST(PathGeometry, TheRenderedLengthFollowsTheArcs) {
  EXPECT_DOUBLE_EQ(renderedLength(primitives(), Path{}), 0.0);
  EXPECT_NEAR(renderedLength(primitives(), straightRun(100, 100, 6, 20)), 20.0,
              1e-9);

  // Ten cells north, a quarter turn onto east, ten cells east: the arc
  // counts at its true length, not as the cells it sweeps.
  const Primitive* quarter = nullptr;
  for (const Primitive& p : primitives().of(0)) {
    if (p.exitHeading == 6) {
      quarter = &p;
      break;
    }
  }
  ASSERT_NE(quarter, nullptr);
  Path path = straightRun(50, 80, 0, 10);
  const PathPoint end = appendTurn(path, *quarter);
  const Path after = straightRun(end.x, end.y, end.heading, 10);
  path.insert(path.end(), after.begin(), after.end());
  const double length = renderedLength(primitives(), path);
  EXPECT_NEAR(length, 20.0 + (std::numbers::pi / 2.0 * 5.0), 0.05);

  // It is the length of the polyline samplePath() draws from the first point.
  Path copy = path;
  std::vector<PathSegment> segments;
  EXPECT_DOUBLE_EQ(length, polylineLength(samplePath(primitives(), copy,
                                                     copy.front(), segments)));
}

TEST(PathGeometry, ARepeatedCellRendersNothing) {
  Path path = straightRun(100, 100, 6, 5);
  // The same cell twice in a row, under another tag the second time.
  path.push_back({.x = 105,
                  .y = 100,
                  .heading = 7,
                  .primitive = primitives().straight(7)});
  const Path after = straightRun(106, 99, 7, 5);
  path.insert(path.end(), after.begin(), after.end());
  std::vector<PathSegment> segments;
  const std::vector<Point> samples =
      samplePath(primitives(), path, path.front(), segments);
  EXPECT_NEAR(samples.back().x(), static_cast<double>(path.back().x), 1e-9);
  EXPECT_NEAR(samples.back().y(), static_cast<double>(path.back().y), 1e-9);
  EXPECT_NEAR(polylineLength(samples), 5.0 + (6.0 * std::numbers::sqrt2), 0.01);
}

} // namespace
