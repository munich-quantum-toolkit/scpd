/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/routing/CrossingConstraints.hpp"
#include "mqt-scpd/routing/DubinsRouter.hpp"
#include "mqt-scpd/routing/Heading.hpp"
#include "mqt-scpd/routing/Path.hpp"
#include "mqt-scpd/routing/PathGeometry.hpp"
#include "mqt-scpd/routing/Primitives.hpp"
#include "mqt-scpd/routing/SearchScratch.hpp"
#include "mqt-scpd/routing/SelfIntersection.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <numbers>
#include <optional>
#include <span>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

namespace {

using namespace mqt::scpd;
using namespace mqt::scpd::routing;

constexpr uint32_t WIDTH = 300;
constexpr uint32_t HEIGHT = 200;

/// A router over an empty grid, with its scratch and its grids.
struct Fixture {
  std::shared_ptr<const MovePrimitives> primitives =
      std::make_shared<const MovePrimitives>(5);
  SearchScratch scratch{WIDTH, HEIGHT};
  grid::BitGrid obstacles{WIDTH, HEIGHT};
  /// The corridor mask: a set bit marks a cell outside the corridor.
  grid::BitGrid outsideCorridor{WIDTH, HEIGHT};
  std::vector<uint8_t> wire =
      std::vector<uint8_t>(static_cast<std::size_t>(WIDTH) * HEIGHT, 0);
  DubinsRouter router{primitives,
                      scratch,
                      {.startStraightLength = 10,
                       .endStraightLength = 10,
                       .minRadius = 5,
                       .bendPenalty = 500}};

  Fixture() {
    router.attachObstacles(&obstacles);
    router.attachCorridor(&outsideCorridor);
    router.attachWireProximity(&wire);
  }

  /// Puts a rectangle outside the corridor and makes it an obstacle, both
  /// bounds included.
  void block(const uint32_t x0, const uint32_t y0, const uint32_t x1,
             const uint32_t y1) {
    for (uint32_t y = y0; y <= y1; ++y) {
      for (uint32_t x = x0; x <= x1; ++x) {
        outsideCorridor.setCell(x, y);
        obstacles.setCell(x, y);
      }
    }
    router.attachCorridor(&outsideCorridor);
  }
};

/// Whether every cell of a path is clear of a mask.
bool avoids(const Path& path, const grid::BitGrid& mask) {
  return std::ranges::none_of(path, [&](const PathPoint& point) {
    return point.x < mask.width() && point.y < mask.height() &&
           mask.testCell(point.x, point.y);
  });
}

/// The row-major index of a cell of the fixture grid.
uint32_t cellIndex(const uint32_t x, const uint32_t y) {
  return (y * WIDTH) + x;
}

/// A straight run toward positive y along one column of the whole grid.
Path columnRun(const MovePrimitives& primitives, const uint32_t x) {
  Path run;
  for (uint32_t y = 0; y < HEIGHT; ++y) {
    run.push_back(
        {.x = x, .y = y, .heading = 4, .primitive = primitives.straight(4)});
  }
  return run;
}

/// A straight run toward positive x along one row of the whole grid.
Path rowRun(const MovePrimitives& primitives, const uint32_t y) {
  Path run;
  for (uint32_t x = 0; x < WIDTH; ++x) {
    run.push_back(
        {.x = x, .y = y, .heading = 6, .primitive = primitives.straight(6)});
  }
  return run;
}

/// Raises a ridge of penalty along the straight path of ACROSS.
void raiseRidge(std::vector<uint8_t>& penalty) {
  for (uint32_t x = 100; x < 200; ++x) {
    for (uint32_t y = 95; y <= 105; ++y) {
      penalty[cellIndex(x, y)] = 60;
    }
  }
}

/// Whether a path leaves the ridge of raiseRidge() where the ridge is widest.
bool leavesTheRidge(const Path& path) {
  return std::ranges::any_of(path, [](const PathPoint& point) {
    return point.x > 120 && point.x < 180 && (point.y < 95 || point.y > 105);
  });
}

/// Whether a path runs on a diagonal heading anywhere.
bool runsDiagonally(const Path& path) {
  return std::ranges::any_of(
      path, [](const PathPoint& point) { return isDiagonal(point.heading); });
}

/// The most cells that one move covers, with its start and end cell.
std::size_t widestMove(const MovePrimitives& primitives) {
  std::size_t widest = 0;
  for (uint32_t heading = 0; heading < NUM_HEADINGS; ++heading) {
    for (const Primitive& move : primitives.of(static_cast<Heading>(heading))) {
      std::size_t cells = move.swept.size();
      if (move.swept.empty() || move.swept.front() != CellOffset{}) {
        ++cells;
      }
      if (move.swept.empty() ||
          move.swept.back() != CellOffset{.dx = move.dx, .dy = move.dy}) {
        ++cells;
      }
      widest = std::max(widest, cells);
    }
  }
  return widest;
}

/// The rendering of a path from its first point.
std::vector<Point> rendering(const MovePrimitives& primitives, Path path) {
  std::vector<PathSegment> segments;
  const PathPoint start = path.front();
  return samplePath(primitives, path, start, segments);
}

/// The angles, in degrees, by which a rendering misses a right angle where it
/// crosses the vertical line at @p column. Each piece of the polyline that
/// passes from one side of the line to the other gives one angle.
std::vector<double> crossingSkews(const std::vector<Point>& points,
                                  const double column) {
  std::vector<double> skews;
  for (std::size_t i = 1; i < points.size(); ++i) {
    const Point& a = points[i - 1];
    const Point& b = points[i];
    if ((a.x() < column) == (b.x() < column)) {
      continue;
    }
    skews.push_back(
        std::atan2(std::abs(b.y() - a.y()), std::abs(b.x() - a.x())) * 180.0 /
        std::numbers::pi);
  }
  return skews;
}

/// Whether a point of a path lies on a turn. The points of a turn carry its
/// tag, and the point after a turn is the end of its arc. Where the search
/// began with a turn, the arc starts on the last cell of the source stub,
/// which keeps the straight tag of the stub. That cell is on the turn when
/// the point after it carries a turn tag and the arc ends at that cell plus
/// the offset of the move. A search that begins with a straight step of one
/// cell and then turns gives the same tags, so the offset decides.
bool onTurn(const MovePrimitives& primitives, const Path& path,
            const std::size_t i, const std::size_t stubEnd) {
  const auto turns = [&](const PathPoint& point) {
    return !primitives.isStraight(point.heading, point.primitive);
  };
  if (turns(path[i]) || (i > 0 && turns(path[i - 1]))) {
    return true;
  }
  if (i != stubEnd || i + 1 >= path.size() || !turns(path[i + 1])) {
    return false;
  }
  const PathPoint& first = path[i + 1];
  std::size_t arcEnd = i + 1;
  while (arcEnd < path.size() && path[arcEnd].heading == first.heading &&
         path[arcEnd].primitive == first.primitive) {
    ++arcEnd;
  }
  const Primitive* move = primitives.find(first.heading, first.primitive);
  return move != nullptr && arcEnd < path.size() &&
         static_cast<int64_t>(path[arcEnd].x) - path[i].x == move->dx &&
         static_cast<int64_t>(path[arcEnd].y) - path[i].y == move->dy;
}

/// The points of a path that fail the crossing test that the orthogonal
/// search asks there: the test of a turn for a point on a turn, and else the
/// test of the straight step into the point. The router tests the first
/// point as if a straight step on its own heading entered it.
std::vector<std::size_t> crossingFailures(const DubinsRouter& router,
                                          const Path& path) {
  const std::size_t stubEnd = router.params().startStraightLength;
  std::vector<std::size_t> failures;
  for (std::size_t i = 0; i < path.size(); ++i) {
    bool passes = true;
    if (onTurn(router.primitives(), path, i, stubEnd)) {
      passes = router.turnAllowedOrthogonal(path[i].x, path[i].y);
    } else {
      const Heading step = path[i > 0 ? i - 1 : 0].heading;
      passes = router.crossingAllowedOrthogonal(path[i].x, path[i].y, step);
    }
    if (!passes) {
      failures.push_back(i);
    }
  }
  return failures;
}

const RoutingObjective ACROSS{
    .source = {.x = 30, .y = 100, .heading = 6, .primitive = 0},
    .target = {.x = 260, .y = 100, .heading = 6, .primitive = 0}};

TEST(DubinsRouter, RoutesFromTheSourceToTheTarget) {
  Fixture f;
  const Path path = f.router.route(ACROSS);
  ASSERT_FALSE(path.empty());
  EXPECT_EQ(path.front().x, ACROSS.source.x);
  EXPECT_EQ(path.front().y, ACROSS.source.y);
  EXPECT_EQ(path.front().heading, ACROSS.source.heading);
  EXPECT_EQ(path.back().x, ACROSS.target.x);
  EXPECT_EQ(path.back().y, ACROSS.target.y);
  EXPECT_EQ(path.back().heading, ACROSS.target.heading);
  // A clear grid gives the straight line, so no bend is paid for.
  EXPECT_EQ(countBends(path), 0U);
}

TEST(DubinsRouter, TheStubsOutOfTheEndsAreStraight) {
  Fixture f;
  // The two ends face each other across a wall with one gap, so the path
  // has to bend in the middle while both ends still leave straight.
  f.block(140, 0, 150, 80);
  f.block(140, 120, 150, HEIGHT - 1);
  const Path path = f.router.route(ACROSS);
  ASSERT_FALSE(path.empty());
  for (uint32_t i = 0; i <= 10; ++i) {
    EXPECT_EQ(path[i].heading, ACROSS.source.heading) << i;
    EXPECT_EQ(path[i].y, ACROSS.source.y) << i;
  }
  const std::size_t n = path.size();
  for (std::size_t i = n - 11; i < n; ++i) {
    EXPECT_EQ(path[i].heading, ACROSS.target.heading) << i;
    EXPECT_EQ(path[i].y, ACROSS.target.y) << i;
  }
}

TEST(DubinsRouter, ThePathStaysInTheCorridor) {
  Fixture f;
  f.block(120, 60, 180, 140);
  const Path path = f.router.route(ACROSS);
  ASSERT_FALSE(path.empty());
  EXPECT_TRUE(avoids(path, f.outsideCorridor));
  EXPECT_GT(countBends(path), 0U);
}

TEST(DubinsRouter, AnUnreachableTargetGivesAnEmptyPath) {
  Fixture f;
  for (uint32_t y = 0; y < HEIGHT; ++y) {
    f.outsideCorridor.setCell(150, y);
  }
  f.router.attachCorridor(&f.outsideCorridor);
  EXPECT_TRUE(f.router.route(ACROSS).empty());
}

TEST(DubinsRouter, AStubThatLeavesTheGridIsNotRoutable) {
  Fixture f;
  // The source sits closer to the edge than its own straight stub.
  const RoutingObjective offGrid{
      .source = {.x = 3, .y = 100, .heading = 2, .primitive = 0},
      .target = {.x = 260, .y = 100, .heading = 6, .primitive = 0}};
  EXPECT_TRUE(f.router.route(offGrid).empty());
}

TEST(DubinsRouter, EveryTurnOfThePathIsAnArcOfThePrimitives) {
  Fixture f;
  f.block(120, 0, 130, 120);
  const Path path = f.router.route(ACROSS);
  ASSERT_FALSE(path.empty());
  const SegmentedPath segmented = reconstructSegments(*f.primitives, path);
  ASSERT_GE(segmented.segments.size(), 3U);
  // Every segment is a move of the primitive table, and its exit heading is
  // the heading of the next segment. A path can therefore only turn by
  // running an arc of the bend radius; there is no corner anywhere.
  for (std::size_t i = 0; i + 1 < segmented.segments.size(); ++i) {
    const PathSegment& segment = segmented.segments[i];
    const Primitive* move =
        f.primitives->find(segment.heading, segment.primitive);
    ASSERT_NE(move, nullptr) << i;
    EXPECT_EQ(move->exitHeading, segmented.segments[i + 1].heading) << i;
  }
  // A turn spans at least the bend radius along the axis it turns into, and
  // counts as one bend, whatever cells it sweeps.
  uint32_t turns = 0;
  for (const PathSegment& segment : segmented.segments) {
    if (segment.straight()) {
      continue;
    }
    const Primitive* move =
        f.primitives->find(segment.heading, segment.primitive);
    ASSERT_NE(move, nullptr);
    if (move->exitHeading == segment.heading) {
      continue;
    }
    ++turns;
    EXPECT_GE(std::max(std::abs(move->dx), std::abs(move->dy)), 4);
  }
  EXPECT_GT(turns, 0U);
  EXPECT_EQ(countBends(path), turns);
}

TEST(DubinsRouter, TheOctileHeuristicFindsAPathWithAsManyBends) {
  Fixture f;
  f.block(120, 60, 180, 140);
  const Path field = f.router.route(ACROSS);
  f.router.setHeuristic(Heuristic::Octile);
  const Path octile = f.router.route(ACROSS);
  ASSERT_FALSE(field.empty());
  ASSERT_FALSE(octile.empty());
  EXPECT_EQ(countBends(field), countBends(octile));
  EXPECT_EQ(f.router.heuristic(), Heuristic::Octile);
}

TEST(DubinsRouter, BothHeuristicsWeaveThroughAFieldOfPosts) {
  Fixture f;
  // Short posts stand in rows across the grid, one row on the straight path.
  // The distance field runs between them, so it no longer equals the octile
  // distance.
  for (uint32_t y = 20; y <= 180; y += 20) {
    for (uint32_t x = 60; x <= 240; x += 20) {
      f.outsideCorridor.setCell(x, y);
      f.outsideCorridor.setCell(x + 1, y);
    }
  }
  f.router.attachCorridor(&f.outsideCorridor);
  const auto weaves = [&](const Path& path) {
    return !path.empty() && avoids(path, f.outsideCorridor) &&
           path.back().samePlace(ACROSS.target);
  };
  EXPECT_TRUE(weaves(f.router.route(ACROSS)));
  f.router.setHeuristic(Heuristic::Octile);
  EXPECT_TRUE(weaves(f.router.route(ACROSS)));
}

TEST(DubinsRouter, TheBendLowerBoundKeepsTheCheapestPath) {
  // The turning a state still owes is a lower bound on what it has left to
  // pay. On these objectives, adding it to the estimate changes how many
  // states the search expands, not the cost of the path it finds. The
  // comparison with a brute-force search in test_route_optimality.cpp pins
  // how much dearer the path can get on other grids.
  const std::vector<RoutingObjective> objectives{
      ACROSS,
      {.source = {.x = 30, .y = 100, .heading = 6, .primitive = 0},
       .target = {.x = 260, .y = 100, .heading = 2, .primitive = 0}},
      {.source = {.x = 40, .y = 30, .heading = 4, .primitive = 0},
       .target = {.x = 250, .y = 170, .heading = 6, .primitive = 0}},
      {.source = {.x = 250, .y = 40, .heading = 1, .primitive = 0},
       .target = {.x = 50, .y = 160, .heading = 5, .primitive = 0}},
  };
  Fixture f;
  f.block(120, 60, 180, 140);
  for (const RoutingObjective& objective : objectives) {
    f.router.setBendLowerBound(true);
    const Path bounded = f.router.route(objective);
    f.router.setBendLowerBound(false);
    const Path plain = f.router.route(objective);
    ASSERT_FALSE(bounded.empty());
    ASSERT_FALSE(plain.empty());
    EXPECT_EQ(countBends(bounded), countBends(plain));
    EXPECT_NEAR(reconstructSegments(*f.primitives, bounded).nominalLength,
                reconstructSegments(*f.primitives, plain).nominalLength, 1e-9);
  }
}

TEST(DubinsRouter, WireProximitySteersThePathAway) {
  Fixture f;
  const Path plain = f.router.route(ACROSS, true);
  ASSERT_FALSE(plain.empty());
  EXPECT_EQ(countBends(plain), 0U);

  // A ridge of penalty along the straight line makes the search go round it.
  for (uint32_t x = 100; x < 200; ++x) {
    for (uint32_t y = 95; y <= 105; ++y) {
      f.wire[(static_cast<std::size_t>(y) * WIDTH) + x] = 60;
    }
  }
  const Path steered = f.router.route(ACROSS, true);
  ASSERT_FALSE(steered.empty());
  bool leftTheRidge = false;
  for (const PathPoint& point : steered) {
    if (point.x > 120 && point.x < 180 && (point.y < 95 || point.y > 105)) {
      leftTheRidge = true;
    }
  }
  EXPECT_TRUE(leftTheRidge);
}

TEST(DubinsRouter, RoutingWithPenaltiesNeedsTheGrids) {
  Fixture f;
  f.router.attachWireProximity(nullptr);
  EXPECT_THROW(static_cast<void>(f.router.route(ACROSS, true)),
               std::logic_error);
  EXPECT_THROW(static_cast<void>(f.router.routeOrthogonal(ACROSS, true)),
               std::logic_error);
  f.router.attachCorridor(nullptr);
  EXPECT_THROW(static_cast<void>(f.router.route(ACROSS)), std::logic_error);
  EXPECT_THROW(static_cast<void>(f.router.routeOrthogonal(ACROSS)),
               std::logic_error);
}

TEST(DubinsRouter, TheEndsOfARouteNeedAHeadingOfTheGrid) {
  Fixture f;
  // A port that faces 30 degrees has no heading of the grid.
  RoutingObjective noSourceHeading = ACROSS;
  noSourceHeading.source.heading = headingOfOrientation(30.0);
  ASSERT_EQ(noSourceHeading.source.heading, NUM_HEADINGS);
  RoutingObjective noTargetHeading = ACROSS;
  noTargetHeading.target.heading = NUM_HEADINGS;
  for (const RoutingObjective& objective : {noSourceHeading, noTargetHeading}) {
    EXPECT_THROW(static_cast<void>(f.router.route(objective)),
                 std::invalid_argument);
    EXPECT_THROW(static_cast<void>(f.router.routeOrthogonal(objective)),
                 std::invalid_argument);
  }
}

TEST(DubinsRouter, GridsMustMatchTheRouterGrid) {
  Fixture f;
  const grid::BitGrid wrong(10, 10);
  EXPECT_THROW(f.router.attachObstacles(&wrong), std::invalid_argument);
  EXPECT_THROW(f.router.attachCorridor(&wrong), std::invalid_argument);
  // A transposed grid has as many cells, but a cell (x, y) of the router
  // grid is not the cell (x, y) of the transposed one.
  const grid::BitGrid transposed(HEIGHT, WIDTH);
  EXPECT_THROW(f.router.attachObstacles(&transposed), std::invalid_argument);
  EXPECT_THROW(f.router.attachCorridor(&transposed), std::invalid_argument);
  EXPECT_THROW(f.router.attachCorridorUnpacked(&transposed),
               std::invalid_argument);
  EXPECT_EQ(f.router.obstacles(), &f.obstacles);
  EXPECT_EQ(f.router.corridor(), &f.outsideCorridor);
  const std::vector<uint8_t> small(100, 0);
  EXPECT_THROW(f.router.attachWireProximity(&small), std::invalid_argument);
  EXPECT_THROW(f.router.setStaticProximity(small), std::invalid_argument);
  const std::vector<uint8_t> tooLarge(f.router.cells(), 200);
  EXPECT_THROW(f.router.setStaticProximity(tooLarge), std::invalid_argument);
}

TEST(DubinsRouter, TheStaticProximityDecaysAwayFromAnObstacle) {
  Fixture f;
  f.obstacles.setCell(150, 100);
  f.router.attachObstacles(&f.obstacles);
  f.router.computeStaticProximity(10, 40);
  const std::vector<uint8_t>& penalty = f.router.staticProximity();
  const auto at = [&](const uint32_t x, const uint32_t y) {
    return penalty[(static_cast<std::size_t>(y) * WIDTH) + x];
  };
  EXPECT_EQ(at(150, 100), 40);
  EXPECT_GT(at(151, 100), 0);
  EXPECT_LT(at(151, 100), 40);
  EXPECT_LT(at(155, 100), at(151, 100));
  EXPECT_EQ(at(200, 100), 0);
  // The growth is four-connected, so the penalty spreads by Manhattan steps.
  EXPECT_EQ(at(153, 102), at(155, 100));
}

TEST(DubinsRouter, TheStaticProximityHoldsForTheLargestDistances) {
  // Distances far beyond the grid: the decay over the distance is nearly
  // flat, so every cell of the grid carries almost the full penalty.
  for (const uint32_t distance :
       {40'000'000U, 0xFFFFFFFFU - 100U, 0xFFFFFFFFU}) {
    Fixture f;
    f.obstacles.setCell(150, 100);
    f.router.attachObstacles(&f.obstacles);
    f.router.computeStaticProximity(distance, 127);
    const std::vector<uint8_t>& penalty = f.router.staticProximity();
    EXPECT_EQ(penalty[cellIndex(150, 100)], 127) << distance;
    EXPECT_EQ(penalty[cellIndex(151, 100)], 126) << distance;
    EXPECT_EQ(penalty[cellIndex(WIDTH - 1, HEIGHT - 1)], 126) << distance;
    EXPECT_EQ(std::ranges::min(penalty), 126) << distance;
  }
}

TEST(DubinsRouter, TheWindowedProximityMatchesTheFullOne) {
  Fixture f;
  for (uint32_t x = 100; x < 110; ++x) {
    f.obstacles.setCell(x, 100);
  }
  f.router.attachObstacles(&f.obstacles);
  f.router.computeStaticProximity(8, 30);
  const std::vector<uint8_t> full = f.router.staticProximity();

  // Recomputing a window around the obstacle reproduces the same values.
  f.router.computeStaticProximityWindow(
      8, 30, {.minX = 90, .maxX = 120, .minY = 90, .maxY = 110});
  const std::vector<uint8_t>& windowed = f.router.staticProximity();
  for (uint32_t y = 90; y <= 110; ++y) {
    for (uint32_t x = 90; x <= 120; ++x) {
      const std::size_t index = (static_cast<std::size_t>(y) * WIDTH) + x;
      EXPECT_EQ(full[index], windowed[index]) << x << "," << y;
    }
  }
}

TEST(DubinsRouter, ACrossingOfAStraightWireMustBeOrthogonal) {
  Fixture f;
  // A wire running toward positive y across the middle of the grid.
  Path wire;
  for (uint32_t y = 20; y < 180; ++y) {
    wire.push_back({.x = 150,
                    .y = y,
                    .heading = 4,
                    .primitive = f.primitives->straight(4)});
  }
  f.router.buildOrthogonalConstraints({wire}, {false}, 6);

  // A route traveling east crosses it at a right angle and is allowed.
  EXPECT_TRUE(f.router.crossingAllowedOrthogonal(150, 100, 6));
  EXPECT_TRUE(f.router.crossingAllowedOrthogonal(150, 100, 2));
  // A route traveling at 45 degrees is not.
  EXPECT_FALSE(f.router.crossingAllowedOrthogonal(150, 100, 5));
  EXPECT_FALSE(f.router.crossingAllowedOrthogonal(150, 100, 4));
  EXPECT_EQ(f.router.constraintMaskAt(150, 100), 1U << 4U);
  // Away from the wire nothing is constrained.
  EXPECT_TRUE(f.router.crossingAllowedOrthogonal(50, 50, 5));
  EXPECT_EQ(f.router.constraintMaskAt(50, 50), 0U);
  // The ends of the wire may not be crossed at all.
  EXPECT_FALSE(f.router.crossingAllowedOrthogonal(150, 21, 6));

  f.router.clearOrthogonalConstraints();
  EXPECT_TRUE(f.router.crossingAllowedOrthogonal(150, 100, 5));
}

TEST(DubinsRouter, TheOrthogonalSearchCrossesAWireAtARightAngle) {
  Fixture f;
  Path wire;
  for (uint32_t y = 20; y < 180; ++y) {
    wire.push_back({.x = 150,
                    .y = y,
                    .heading = 4,
                    .primitive = f.primitives->straight(4)});
  }
  f.router.buildOrthogonalConstraints({wire}, {false}, 6);
  // The ends lie on different rows, so the free route crosses the column of
  // the wire on a diagonal.
  const RoutingObjective slanted{
      .source = {.x = 30, .y = 40, .heading = 6, .primitive = 0},
      .target = {.x = 260, .y = 160, .heading = 6, .primitive = 0}};
  const Path free = f.router.route(slanted);
  ASSERT_FALSE(free.empty());
  EXPECT_TRUE(std::ranges::any_of(free, [](const PathPoint& point) {
    return point.x == 150 && isDiagonal(point.heading);
  }));

  const Path path = f.router.routeOrthogonal(slanted);
  ASSERT_FALSE(path.empty());
  // The path crosses the column of the wire between its ends, and only on
  // heading 2 or 6, at a right angle to the wire.
  std::size_t onColumn = 0;
  for (const PathPoint& point : path) {
    if (point.x != 150) {
      continue;
    }
    ++onColumn;
    EXPECT_GT(point.y, 20U);
    EXPECT_LT(point.y, 179U);
    EXPECT_TRUE(point.heading == 2 || point.heading == 6) << point.heading;
  }
  EXPECT_GT(onColumn, 0U);
  // Every point passes the search's own test: the test of a turn where a
  // turn touches it, and else the test of the straight step into it.
  EXPECT_TRUE(crossingFailures(f.router, path).empty());
  // The rendered wire crosses the column at a right angle.
  const std::vector<double> skews =
      crossingSkews(rendering(*f.primitives, path), 150.0);
  EXPECT_FALSE(skews.empty());
  for (const double skew : skews) {
    EXPECT_LT(skew, 1e-6);
  }
}

TEST(DubinsRouter, AStubCrossesAWireAtARightAngleOnly) {
  Fixture f;
  f.router.setParams({.startStraightLength = 20,
                      .endStraightLength = 20,
                      .minRadius = 5,
                      .bendPenalty = 500});
  Path wire;
  for (uint32_t y = 20; y < 180; ++y) {
    wire.push_back({.x = 150,
                    .y = y,
                    .heading = 4,
                    .primitive = f.primitives->straight(4)});
  }
  f.router.buildOrthogonalConstraints({wire}, {false}, 6);
  // The source stub runs on a diagonal across the wire. The search cannot
  // change a stub, so no orthogonal route exists.
  const RoutingObjective diagonalSource{
      .source = {.x = 140, .y = 90, .heading = 5, .primitive = 0},
      .target = {.x = 260, .y = 160, .heading = 6, .primitive = 0}};
  EXPECT_FALSE(f.router.route(diagonalSource).empty());
  EXPECT_TRUE(f.router.routeOrthogonal(diagonalSource).empty());
  // The same holds for the target stub.
  const RoutingObjective diagonalTarget{
      .source = {.x = 30, .y = 40, .heading = 6, .primitive = 0},
      .target = {.x = 160, .y = 110, .heading = 5, .primitive = 0}};
  EXPECT_FALSE(f.router.route(diagonalTarget).empty());
  EXPECT_TRUE(f.router.routeOrthogonal(diagonalTarget).empty());

  // A stub that crosses the wire at a right angle passes, and so does every
  // other point of the path.
  const RoutingObjective straightSource{
      .source = {.x = 140, .y = 90, .heading = 6, .primitive = 0},
      .target = {.x = 260, .y = 160, .heading = 6, .primitive = 0}};
  const Path path = f.router.routeOrthogonal(straightSource);
  ASSERT_FALSE(path.empty());
  EXPECT_TRUE(path[10].samePlace({.x = 150, .y = 90}));
  EXPECT_TRUE(crossingFailures(f.router, path).empty());

  // Exempt cells pass the test of a stub too.
  std::vector<uint32_t> stubCells;
  for (uint32_t step = 1; step <= 20; ++step) {
    stubCells.push_back(cellIndex(140 + step, 90 + step));
  }
  f.router.setCrossingExemption(stubCells);
  EXPECT_FALSE(f.router.routeOrthogonal(diagonalSource).empty());
}

TEST(DubinsRouter, BothEndsOfAnOrthogonalRouteMeetTheCrossingTest) {
  // A pin lies in the zone of a wire. Heading 3 runs at 45 degrees to the
  // wire, and heading 2 at a right angle. The source meets the test that
  // the last step into the target meets, so a route may leave the pin on a
  // heading only where it may arrive on the reverse heading.
  for (const uint32_t stub : {0U, 3U}) {
    Fixture f;
    f.router.setParams({.startStraightLength = stub,
                        .endStraightLength = stub,
                        .minRadius = 5,
                        .bendPenalty = 500});
    f.router.buildOrthogonalConstraints({columnRun(*f.primitives, 150)},
                                        {false}, 6);
    const PathPoint pin{.x = 144, .y = 100, .heading = 3, .primitive = 0};
    ASSERT_FALSE(f.router.crossingAllowedOrthogonal(pin.x, pin.y, 3));
    const PathPoint distant{.x = 60, .y = 160, .heading = 3, .primitive = 0};
    const RoutingObjective leaving{.source = pin, .target = distant};
    const RoutingObjective arriving{
        .source = {.x = distant.x,
                   .y = distant.y,
                   .heading = 7,
                   .primitive = 0},
        .target = {.x = pin.x, .y = pin.y, .heading = 7, .primitive = 0}};
    // The stubs end outside the zone, so the pin alone decides. The free
    // search, which ignores the constraints, finds both paths.
    EXPECT_FALSE(f.router.route(leaving).empty()) << stub;
    EXPECT_FALSE(f.router.route(arriving).empty()) << stub;
    EXPECT_TRUE(f.router.routeOrthogonal(leaving).empty()) << stub;
    EXPECT_TRUE(f.router.routeOrthogonal(arriving).empty()) << stub;

    const RoutingObjective leavingAcross{
        .source = {.x = pin.x, .y = pin.y, .heading = 2, .primitive = 0},
        .target = {
            .x = distant.x, .y = distant.y, .heading = 2, .primitive = 0}};
    const RoutingObjective arrivingAcross{
        .source = {.x = distant.x,
                   .y = distant.y,
                   .heading = 6,
                   .primitive = 0},
        .target = {.x = pin.x, .y = pin.y, .heading = 6, .primitive = 0}};
    const Path out = f.router.routeOrthogonal(leavingAcross);
    const Path back = f.router.routeOrthogonal(arrivingAcross);
    ASSERT_FALSE(out.empty()) << stub;
    ASSERT_FALSE(back.empty()) << stub;
    EXPECT_TRUE(crossingFailures(f.router, out).empty()) << stub;
    EXPECT_TRUE(crossingFailures(f.router, back).empty()) << stub;
  }
}

TEST(DubinsRouter, AnOrthogonalRouteDoesNotTurnAcrossAWire) {
  Fixture f;
  f.router.setParams({.startStraightLength = 0,
                      .endStraightLength = 0,
                      .minRadius = 5,
                      .bendPenalty = 500});
  // A wire runs along column 53, and its zone reaches one cell to either
  // side.
  f.router.buildOrthogonalConstraints({columnRun(*f.primitives, 53)}, {false},
                                      1);
  const RoutingObjective quarter{
      .source = {.x = 50, .y = 50, .heading = 6, .primitive = 0},
      .target = {.x = 55, .y = 45, .heading = 0, .primitive = 0}};
  // The free path is one quarter turn. Every cell it sweeps is entered on
  // heading 6, but the arc crosses the column far off a right angle.
  const Path free = f.router.route(quarter);
  ASSERT_FALSE(free.empty());
  const std::vector<double> freeSkews =
      crossingSkews(rendering(*f.primitives, free), 53.0);
  ASSERT_EQ(freeSkews.size(), 1U);
  EXPECT_GT(freeSkews.front(), 30.0);
  // The wire has to cross the zone straight along row 50. It then reaches
  // the target on heading 0 only from the east, across its own first run,
  // so no orthogonal path exists.
  EXPECT_TRUE(f.router.routeOrthogonal(quarter).empty());

  // With the target 15 rows further on, the wire crosses straight, turns
  // beyond the zone and comes back to the column of the target.
  const RoutingObjective further{
      .source = quarter.source,
      .target = {.x = 55, .y = 30, .heading = 0, .primitive = 0}};
  const std::vector<double> furtherFreeSkews =
      crossingSkews(rendering(*f.primitives, f.router.route(further)), 53.0);
  ASSERT_EQ(furtherFreeSkews.size(), 1U);
  EXPECT_GT(furtherFreeSkews.front(), 30.0);
  const Path path = f.router.routeOrthogonal(further);
  ASSERT_FALSE(path.empty());
  EXPECT_TRUE(path.back().samePlace(further.target));
  const std::vector<double> skews =
      crossingSkews(rendering(*f.primitives, path), 53.0);
  EXPECT_FALSE(skews.empty());
  for (const double skew : skews) {
    EXPECT_LT(skew, 1e-6);
  }
}

TEST(DubinsRouter, AnOrthogonalRouteDoesNotTurnFromAConstrainedStart) {
  Fixture f;
  f.router.setParams({.startStraightLength = 0,
                      .endStraightLength = 0,
                      .minRadius = 5,
                      .bendPenalty = 500});
  f.router.buildOrthogonalConstraints({columnRun(*f.primitives, 53)}, {false},
                                      1);
  // The search starts on the last cell of the zone, and the free path turns
  // there at once. Every other cell of that turn lies outside the zone.
  const RoutingObjective fromTheZone{
      .source = {.x = 54, .y = 100, .heading = 6, .primitive = 0},
      .target = {.x = 59, .y = 60, .heading = 0, .primitive = 0}};
  // Whether a piece of the rendering leaves row 100 from a point in the
  // zone, whose last column ends at x = 54.5.
  const auto curvesInTheZone = [&](const Path& path) {
    const std::vector<Point> points = rendering(*f.primitives, path);
    for (std::size_t i = 1; i < points.size(); ++i) {
      if (points[i - 1].x() < 54.5 && points[i].y() != 100.0) {
        return true;
      }
    }
    return false;
  };
  const Path free = f.router.route(fromTheZone);
  ASSERT_FALSE(free.empty());
  EXPECT_TRUE(curvesInTheZone(free));

  // The arc would start in the zone, so the wire runs straight out of it
  // first.
  const Path path = f.router.routeOrthogonal(fromTheZone);
  ASSERT_FALSE(path.empty());
  EXPECT_TRUE(path.back().samePlace(fromTheZone.target));
  EXPECT_FALSE(curvesInTheZone(path));
}

TEST(DubinsRouter, ACheckSeesATurnThatStartsOnTheLastCellOfTheStub) {
  Fixture f;
  f.router.buildOrthogonalConstraints({columnRun(*f.primitives, 63)}, {false},
                                      1);
  // The source stub ends on the last column of the zone. The free path turns
  // there at once, so its arc starts on a cell with the straight tag of the
  // stub. The check asks the test of a turn there, as the search does.
  const RoutingObjective fromTheZone{
      .source = {.x = 54, .y = 100, .heading = 6, .primitive = 0},
      .target = {.x = 69, .y = 50, .heading = 0, .primitive = 0}};
  const Path free = f.router.route(fromTheZone);
  ASSERT_GT(free.size(), 11U);
  EXPECT_TRUE(free[10].samePlace({.x = 64, .y = 100}));
  EXPECT_TRUE(f.primitives->isStraight(free[10].heading, free[10].primitive));
  EXPECT_EQ(crossingFailures(f.router, free), std::vector<std::size_t>{10});

  // The orthogonal search does not turn there, and the check finds nothing.
  const Path path = f.router.routeOrthogonal(fromTheZone);
  ASSERT_FALSE(path.empty());
  EXPECT_TRUE(crossingFailures(f.router, path).empty());
}

TEST(DubinsRouter, ASingleCrossingWireCannotComeBack) {
  Fixture f;
  // The feedline splits the grid. A wall on the left forces the wire across
  // to the right, and a wall on the right with a gap near the feedline
  // invites it back to the left. The far end of the right wall is open too.
  Path feedline;
  for (uint32_t y = 0; y < HEIGHT; ++y) {
    feedline.push_back({.x = 150,
                        .y = y,
                        .heading = 4,
                        .primitive = f.primitives->straight(4)});
  }
  f.block(0, 68, 152, 72);
  f.block(147, 128, 279, 132);
  const RoutingObjective down{
      .source = {.x = 30, .y = 40, .heading = 6, .primitive = 0},
      .target = {.x = 260, .y = 170, .heading = 6, .primitive = 0}};
  const auto crossingsOf = [](const Path& path) {
    std::size_t crossings = 0;
    for (std::size_t i = 1; i < path.size(); ++i) {
      if ((path[i - 1].x < 150) != (path[i].x < 150)) {
        ++crossings;
      }
    }
    return crossings;
  };
  const Path free = f.router.routeOrthogonal(down);
  ASSERT_FALSE(free.empty());
  EXPECT_GT(crossingsOf(free), 1U);

  // With the rule, the wire crosses once and goes round the far end.
  f.router.setSingleCrossingFeedline(&feedline, 19, 1);
  const Path path = f.router.routeOrthogonal(down);
  ASSERT_FALSE(path.empty());
  EXPECT_EQ(crossingsOf(path), 1U);
  EXPECT_TRUE(avoids(path, f.outsideCorridor));

  // The rule exists only while the search runs. With the feedline still
  // set, no cell keeps a rule after the search.
  EXPECT_TRUE(f.router.crossingAllowedOrthogonal(160, 100, 4));
  std::size_t ruled = 0;
  for (uint32_t y = 0; y < HEIGHT; ++y) {
    for (uint32_t x = 0; x < WIDTH; ++x) {
      ruled += f.router.turnAllowedOrthogonal(x, y) ? 0U : 1U;
    }
  }
  EXPECT_EQ(ruled, 0U);
}

TEST(DubinsRouter, TheSingleCrossingRuleBindsTheStubsToo) {
  Fixture f;
  f.router.setParams({.startStraightLength = 20,
                      .endStraightLength = 20,
                      .minRadius = 5,
                      .bendPenalty = 500});
  const Path feedline = columnRun(*f.primitives, 150);
  f.router.setSingleCrossingFeedline(&feedline, 5, 1);
  // The source stub runs on a diagonal through the far side of the feedline,
  // where a straight step may only leave the feedline at a right angle. The
  // search would start beyond the zone.
  const RoutingObjective diagonal{
      .source = {.x = 140, .y = 90, .heading = 5, .primitive = 0},
      .target = {.x = 260, .y = 160, .heading = 6, .primitive = 0}};
  EXPECT_TRUE(f.router.routeOrthogonal(diagonal).empty());

  // The rule ends with the refusal too: with the feedline still set, the
  // cells of the stub keep no rule after the call.
  EXPECT_TRUE(f.router.crossingAllowedOrthogonal(155, 105, 5));
  // Without the feedline the stub passes.
  f.router.setSingleCrossingFeedline(nullptr, 0, 0);
  EXPECT_FALSE(f.router.routeOrthogonal(diagonal).empty());
}

TEST(DubinsRouter, ARoutedPathNeverCrossesItself) {
  Fixture f;
  // A corridor with two pockets, where a search over states could return to
  // a cell it already used on another heading.
  f.block(80, 0, 90, 150);
  f.block(150, 50, 160, HEIGHT - 1);
  f.block(220, 0, 230, 150);
  const Path path = f.router.route(ACROSS, true);
  ASSERT_FALSE(path.empty());
  PathLoopScratch scratch;
  EXPECT_FALSE(pathSelfIntersects(path, WIDTH, HEIGHT, scratch));
  // Nothing was rejected here, but the guard is the reason a caller may
  // trust the result.
  EXPECT_EQ(f.router.loopGuardRejections(), 0U);
}

TEST(DubinsRouter, TheSanitizedEndsAreTheStraightStubsOwnEnds) {
  const Fixture f;
  const PathPoint source{.x = 100, .y = 100, .heading = 6, .primitive = 0};
  const PathPoint moved = f.router.sanitize(source, false);
  EXPECT_EQ(moved.x, 110U);
  EXPECT_EQ(moved.y, 100U);
  const Path stub = f.router.straightStub(source, false, 10);
  ASSERT_EQ(stub.size(), 11U);
  EXPECT_EQ(stub.front().x, 100U);
  EXPECT_EQ(stub.back().x, 110U);

  const PathPoint target{.x = 200, .y = 100, .heading = 6, .primitive = 0};
  const PathPoint back = f.router.sanitize(target, true);
  EXPECT_EQ(back.x, 190U);
  const Path tail = f.router.straightStub(target, true, 10);
  ASSERT_EQ(tail.size(), 11U);
  EXPECT_EQ(tail.front().x, 200U);
  EXPECT_EQ(tail.back().x, 190U);
}

TEST(DubinsRouter, TheFreeStripGrowsToTheNearestObstacle) {
  Fixture f;
  for (uint32_t x = 50; x < 250; ++x) {
    f.obstacles.setCell(x, 60);
    f.obstacles.setCell(x, 140);
  }
  f.router.attachObstacles(&f.obstacles);
  const CellBox box = f.router.freeStripAlong(
      {.x = 100, .y = 100, .heading = 6, .primitive = 0},
      {.x = 200, .y = 100, .heading = 6, .primitive = 0});
  EXPECT_GT(box.minY, 60U);
  EXPECT_LT(box.maxY, 140U);
  EXPECT_LE(box.minX, 100U);
  EXPECT_GE(box.maxX, 200U);
}

TEST(DubinsRouter, ABendPenaltyChangeRebuildsTheTables) {
  Fixture f;
  f.block(120, 0, 130, 120);
  const Path cheap = f.router.route(ACROSS);
  f.router.setParams({.startStraightLength = 10,
                      .endStraightLength = 10,
                      .minRadius = 5,
                      .bendPenalty = 20000});
  const Path expensive = f.router.route(ACROSS);
  ASSERT_FALSE(cheap.empty());
  ASSERT_FALSE(expensive.empty());
  EXPECT_EQ(f.router.params().bendPenalty, 20000);
  // A high bend penalty never buys more bends than a low one.
  EXPECT_LE(countBends(expensive), countBends(cheap));
}

TEST(DubinsRouter, ThePackedAndTheUnpackedGridsAgree) {
  // The packed working grid folds the corridor mask and the static proximity
  // into one byte per cell so that the search touches one cache line per
  // swept cell. It is a cache, so the two forms must route alike.
  Fixture f;
  f.block(120, 60, 180, 140);
  f.obstacles.setCell(200, 90);
  f.router.attachObstacles(&f.obstacles);
  f.router.computeStaticProximity(12, 30);

  f.router.attachCorridor(&f.outsideCorridor);
  const Path packed = f.router.route(ACROSS, true);
  f.router.attachCorridorUnpacked(&f.outsideCorridor);
  const Path unpacked = f.router.route(ACROSS, true);
  ASSERT_FALSE(packed.empty());
  EXPECT_EQ(packed, unpacked);

  // Without the penalties the two forms agree as well.
  f.router.attachCorridor(&f.outsideCorridor);
  const Path packedPlain = f.router.route(ACROSS);
  f.router.attachCorridorUnpacked(&f.outsideCorridor);
  const Path unpackedPlain = f.router.route(ACROSS);
  ASSERT_FALSE(packedPlain.empty());
  EXPECT_EQ(packedPlain, unpackedPlain);

  // A corridor of a band of rows gives a mask whose words of 64 cells are
  // all clear, all set or mixed, and the grid ends in a part word.
  f.outsideCorridor.fill(true);
  for (uint32_t y = 85; y <= 115; ++y) {
    for (uint32_t x = 0; x < WIDTH; ++x) {
      f.outsideCorridor.setCell(x, y, false);
    }
  }
  f.router.attachCorridor(&f.outsideCorridor);
  const Path packedBand = f.router.route(ACROSS, true);
  f.router.attachCorridorUnpacked(&f.outsideCorridor);
  const Path unpackedBand = f.router.route(ACROSS, true);
  ASSERT_FALSE(packedBand.empty());
  EXPECT_TRUE(avoids(packedBand, f.outsideCorridor));
  EXPECT_EQ(packedBand, unpackedBand);
}

TEST(DubinsRouter, AnUnpackedCorridorStaysUnpackedWhenTheProximityChanges) {
  // Every search reads an unpacked corridor as it is at the time of the
  // search, also after a change of the static proximity.
  for (const bool computed : {true, false}) {
    Fixture f;
    // The octile estimate does not read the corridor, so only the search
    // can stop at the wall.
    f.router.setHeuristic(Heuristic::Octile);
    f.router.attachCorridorUnpacked(&f.outsideCorridor);
    if (computed) {
      f.obstacles.setCell(100, 20);
      f.router.computeStaticProximity(4, 20);
    } else {
      f.router.setStaticProximity(std::vector<uint8_t>(f.router.cells(), 0));
    }
    // A wall across the whole grid, drawn after the change.
    for (uint32_t y = 0; y < HEIGHT; ++y) {
      f.outsideCorridor.setCell(150, y);
    }
    EXPECT_TRUE(f.router.route(ACROSS).empty()) << computed;
    EXPECT_TRUE(f.router.route(ACROSS, true).empty()) << computed;
  }
}

TEST(DubinsRouter, ARouterOutlivesWhateverBuiltItsPrimitives) {
  // The router shares ownership of its primitive table, so a router that
  // outlives the scope it was built in still routes.
  SearchScratch scratch(WIDTH, HEIGHT);
  const grid::BitGrid outsideCorridor(WIDTH, HEIGHT);
  auto router = [&] {
    auto primitives = std::make_shared<const MovePrimitives>(5);
    return DubinsRouter(primitives, scratch,
                        {.startStraightLength = 10,
                         .endStraightLength = 10,
                         .minRadius = 5,
                         .bendPenalty = 500});
  }();
  router.attachCorridor(&outsideCorridor);
  EXPECT_EQ(router.primitives().minRadius(), 5U);
  EXPECT_FALSE(router.route(ACROSS).empty());
}

TEST(DubinsRouter, ARouterIsNeitherCopiedNorMoved) {
  // A copy would share the scratch of the original, and a moved-from router
  // would keep the state of tables it no longer holds.
  static_assert(!std::is_copy_constructible_v<DubinsRouter>);
  static_assert(!std::is_copy_assignable_v<DubinsRouter>);
  static_assert(!std::is_move_constructible_v<DubinsRouter>);
  static_assert(!std::is_move_assignable_v<DubinsRouter>);
}

TEST(DubinsRouter, TheBendRadiusMustMatchThePrimitives) {
  auto primitives = std::make_shared<const MovePrimitives>(5);
  SearchScratch scratch(WIDTH, HEIGHT);
  const SearchParams wrong{.startStraightLength = 10,
                           .endStraightLength = 10,
                           .minRadius = 8,
                           .bendPenalty = 500};
  // A radius that disagrees with the table would search over arcs of one
  // radius while the caller believes it asked for another.
  EXPECT_THROW(static_cast<void>(DubinsRouter(primitives, scratch, wrong)),
               std::invalid_argument);
  DubinsRouter router(primitives, scratch);
  EXPECT_THROW(router.setParams(wrong), std::invalid_argument);
}

TEST(DubinsRouter, TheRouterGridMustFitTheSearchState) {
  auto primitives = std::make_shared<const MovePrimitives>(5);
  SearchScratch scratch(WIDTH, HEIGHT);
  EXPECT_THROW(static_cast<void>(DubinsRouter(nullptr, scratch)),
               std::invalid_argument);
  // A grid whose states do not fit a search index is refused where it is
  // allocated, not later in a search.
  EXPECT_THROW(static_cast<void>(SearchScratch(40000, 40000)),
               std::invalid_argument);
  // Eight states per cell of a grid of 2^61 cells make 2^64 states, which
  // wraps a 64-bit count to zero.
  EXPECT_THROW(static_cast<void>(SearchScratch(1U << 31U, 1U << 30U)),
               std::invalid_argument);
}

TEST(DubinsRouter, ARouterGridHasOneTo65535CellsPerAxis) {
  auto primitives = std::make_shared<const MovePrimitives>(5);
  SearchScratch empty(0, 10);
  EXPECT_THROW(static_cast<void>(DubinsRouter(primitives, empty)),
               std::invalid_argument);
  SearchScratch tooWide(65536, 1);
  EXPECT_THROW(static_cast<void>(DubinsRouter(primitives, tooWide)),
               std::invalid_argument);
  SearchScratch widest(65535, 1);
  const DubinsRouter router(primitives, widest);
  EXPECT_EQ(router.width(), 65535U);
}

TEST(DubinsRouter, AMoveThatCoversTooManyCellsDoesNotFitTheTables) {
  // The search tables hold a move of at most 60 cells with its start and end
  // cell. The arcs grow with the bend radius, so a wide radius does not fit.
  SearchScratch scratch(64, 64);
  bool refused = false;
  for (uint32_t radius = 1; radius <= 30; ++radius) {
    auto primitives = std::make_shared<const MovePrimitives>(radius);
    const SearchParams params{.minRadius = static_cast<uint8_t>(radius)};
    if (widestMove(*primitives) > 60) {
      refused = true;
      EXPECT_THROW(static_cast<void>(DubinsRouter(primitives, scratch, params)),
                   std::invalid_argument)
          << radius;
    } else {
      EXPECT_NO_THROW(
          static_cast<void>(DubinsRouter(primitives, scratch, params)))
          << radius;
    }
  }
  EXPECT_TRUE(refused);
}

TEST(DubinsRouter, AStaticProximitySetByHandSteersThePath) {
  Fixture f;
  std::vector<uint8_t> ridge(f.router.cells(), 0);
  raiseRidge(ridge);
  // The corridor is attached already, so the working grid is packed again
  // with the new penalties.
  f.router.setStaticProximity(ridge);
  EXPECT_EQ(f.router.staticProximity(), ridge);
  EXPECT_EQ(f.router.staticPenalty(cellIndex(150, 100)), 60);
  const Path steered = f.router.route(ACROSS, true);
  ASSERT_FALSE(steered.empty());
  EXPECT_TRUE(leavesTheRidge(steered));
}

TEST(DubinsRouter, NoCellCarriesAProximityWithoutObstaclesOrGrowth) {
  Fixture f;
  f.obstacles.setCell(150, 100);
  f.router.attachObstacles(&f.obstacles);
  const auto penalized = [&] {
    return std::ranges::any_of(f.router.staticProximity(),
                               [](const uint8_t value) { return value != 0; });
  };
  f.router.computeStaticProximity(10, 40);
  ASSERT_TRUE(penalized());
  f.router.computeStaticProximity(0, 40);
  EXPECT_FALSE(penalized());

  f.router.computeStaticProximity(10, 40);
  f.router.computeStaticProximity(10, 0);
  EXPECT_FALSE(penalized());

  f.router.computeStaticProximity(10, 40);
  f.router.attachObstacles(nullptr);
  f.router.computeStaticProximity(10, 40);
  EXPECT_FALSE(penalized());
}

TEST(DubinsRouter, AProximityPenaltyAbove127IsRefusedWhenItSpreads) {
  Fixture f;
  f.obstacles.setCell(150, 100);
  f.router.attachObstacles(&f.obstacles);
  const CellBox window{.minX = 140, .maxX = 160, .minY = 90, .maxY = 110};
  f.router.computeStaticProximity(10, 40);
  const std::vector<uint8_t> before = f.router.staticProximity();
  EXPECT_THROW(f.router.computeStaticProximity(10, 128), std::invalid_argument);
  EXPECT_THROW(f.router.computeStaticProximityWindow(10, 128, window),
               std::invalid_argument);
  // A refused call changes no penalty.
  EXPECT_EQ(f.router.staticProximity(), before);

  // Without growth, without a cell to recompute or without obstacles, the
  // penalty spreads nowhere.
  EXPECT_NO_THROW(f.router.computeStaticProximity(0, 200));
  EXPECT_NO_THROW(f.router.computeStaticProximityWindow(0, 200, window));
  EXPECT_NO_THROW(f.router.computeStaticProximityWindow(
      10, 200, {.minX = 160, .maxX = 140, .minY = 90, .maxY = 110}));
  f.router.attachObstacles(nullptr);
  EXPECT_NO_THROW(f.router.computeStaticProximity(10, 200));
  EXPECT_NO_THROW(f.router.computeStaticProximityWindow(10, 200, window));
}

TEST(DubinsRouter, AWindowedProximityRecomputesTheWindowOnly) {
  Fixture f;
  for (uint32_t x = 100; x < 110; ++x) {
    f.obstacles.setCell(x, 100);
  }
  f.router.attachObstacles(&f.obstacles);
  f.router.computeStaticProximity(8, 30);
  const std::vector<uint8_t> full = f.router.staticProximity();
  const CellBox window{.minX = 100, .maxX = 104, .minY = 95, .maxY = 105};

  // Without obstacles the window keeps its penalties.
  f.router.attachObstacles(nullptr);
  f.router.computeStaticProximityWindow(8, 30, window);
  EXPECT_EQ(f.router.staticProximity(), full);

  // An empty window holds no cell to recompute.
  f.router.attachObstacles(&f.obstacles);
  f.router.computeStaticProximityWindow(
      8, 30, {.minX = 104, .maxX = 100, .minY = 95, .maxY = 105});
  EXPECT_EQ(f.router.staticProximity(), full);

  // Without growth the window gets what a computation without growth gives
  // it, which is no penalty, and every other cell keeps its penalty.
  f.router.computeStaticProximityWindow(0, 30, window);
  std::vector<uint8_t> expected = full;
  for (uint32_t y = window.minY; y <= window.maxY; ++y) {
    for (uint32_t x = window.minX; x <= window.maxX; ++x) {
      expected[cellIndex(x, y)] = 0;
    }
  }
  EXPECT_EQ(f.router.staticProximity(), expected);
  EXPECT_GT(f.router.staticPenalty(cellIndex(106, 100)), 0);
}

TEST(DubinsRouter, AWindowOutsideTheGridChangesNoPenalty) {
  Fixture f;
  f.router.computeStaticProximity(3, 60);
  const std::vector<uint8_t> before = f.router.staticProximity();
  // An obstacle on the last column and on the last row. A window past them
  // holds no cell of the grid, so it must not recompute them.
  f.obstacles.setCell(WIDTH - 1, 100);
  f.obstacles.setCell(150, HEIGHT - 1);
  f.router.computeStaticProximityWindow(
      3, 60,
      {.minX = WIDTH + 100, .maxX = WIDTH + 120, .minY = 90, .maxY = 110});
  f.router.computeStaticProximityWindow(
      3, 60,
      {.minX = 0, .maxX = WIDTH - 1, .minY = HEIGHT + 50, .maxY = HEIGHT + 60});
  EXPECT_EQ(f.router.staticProximity(), before);
}

TEST(DubinsRouter, TheWindowedProximitySeesTheObstaclesBeyondTheWindow) {
  Fixture f;
  // One obstacle sits in the corner of the grid, one just beyond the window.
  f.obstacles.setCell(0, 0);
  f.obstacles.setCell(60, 50);
  f.router.attachObstacles(&f.obstacles);
  f.router.computeStaticProximity(8, 30);
  const std::vector<uint8_t> full = f.router.staticProximity();
  EXPECT_EQ(f.router.staticPenalty(cellIndex(0, 0)), 30);
  EXPECT_GT(f.router.staticPenalty(cellIndex(55, 50)), 0);

  // The window reaches past the grid, which clamps it.
  f.router.computeStaticProximityWindow(
      8, 30, {.minX = 0, .maxX = 55, .minY = 0, .maxY = 1000});
  EXPECT_EQ(f.router.staticProximity(), full);
}

TEST(DubinsRouter, ARouteReadsAWindowedProximityBeforeTheGridIsPacked) {
  Fixture f;
  f.router.computeStaticProximity(12, 30);
  const Path straight = f.router.route(ACROSS, true);
  ASSERT_FALSE(straight.empty());
  EXPECT_EQ(countBends(straight), 0U);

  // New obstacles on the straight path raise penalties inside the window.
  for (uint32_t x = 140; x <= 160; ++x) {
    f.obstacles.setCell(x, 100);
  }
  f.router.computeStaticProximityWindow(
      12, 30, {.minX = 120, .maxX = 180, .minY = 80, .maxY = 120});
  // The working grid is out of date, so the route reads the corridor mask and
  // the new penalties directly, and agrees with the route on the packed grid.
  const Path unpacked = f.router.route(ACROSS, true);
  f.router.attachCorridor(&f.outsideCorridor);
  const Path packed = f.router.route(ACROSS, true);
  ASSERT_FALSE(unpacked.empty());
  EXPECT_GT(countBends(unpacked), 0U);
  EXPECT_EQ(unpacked, packed);
}

TEST(DubinsRouter, OnlyStraightMovesNeverEndOnADiagonalHeading) {
  Fixture f;
  // The target lies off the line of the source, so the cheapest path runs
  // diagonally.
  const RoutingObjective offset{
      .source = {.x = 30, .y = 40, .heading = 6, .primitive = 0},
      .target = {.x = 260, .y = 160, .heading = 6, .primitive = 0}};
  const Path free = f.router.route(offset);
  const Path orthogonal = f.router.routeOrthogonal(offset);
  ASSERT_FALSE(free.empty());
  ASSERT_FALSE(orthogonal.empty());
  EXPECT_TRUE(runsDiagonally(free));
  EXPECT_TRUE(runsDiagonally(orthogonal));

  const Path freeStraight = f.router.route(offset, false, true);
  const Path orthogonalStraight = f.router.routeOrthogonal(offset, false, true);
  ASSERT_FALSE(freeStraight.empty());
  ASSERT_FALSE(orthogonalStraight.empty());
  EXPECT_FALSE(runsDiagonally(freeStraight));
  EXPECT_FALSE(runsDiagonally(orthogonalStraight));
  EXPECT_EQ(freeStraight.back(), free.back());
  EXPECT_EQ(orthogonalStraight.back(), orthogonal.back());
}

TEST(DubinsRouter, APathThatCrossesItselfIsRejectedAndCounted) {
  Fixture f;
  // The target stub runs across the source stub, so every path between the
  // two stubs closes a loop.
  const RoutingObjective looped{
      .source = {.x = 100, .y = 100, .heading = 6, .primitive = 0},
      .target = {.x = 105, .y = 95, .heading = 0, .primitive = 0}};
  EXPECT_TRUE(f.router.route(looped).empty());
  EXPECT_EQ(f.router.loopGuardRejections(), 1U);
  EXPECT_TRUE(f.router.routeOrthogonal(looped).empty());
  EXPECT_EQ(f.router.loopGuardRejections(), 2U);
  // A search that finds nothing rejects nothing.
  f.block(150, 0, 150, HEIGHT - 1);
  EXPECT_TRUE(f.router.route(ACROSS).empty());
  EXPECT_EQ(f.router.loopGuardRejections(), 2U);
}

TEST(DubinsRouter, ManySearchesInARowRouteAlike) {
  // The distance field marks the cells of each search with a stamp, and the
  // stamps run out and start over. No search may read what an earlier search
  // left behind, however many searches came before it.
  constexpr uint32_t width = 80;
  constexpr uint32_t height = 40;
  SearchScratch scratch(width, height);
  grid::BitGrid outsideCorridor(width, height);
  for (uint32_t y = 0; y < 25; ++y) {
    outsideCorridor.setCell(40, y);
  }
  DubinsRouter router(std::make_shared<const MovePrimitives>(5), scratch,
                      {.startStraightLength = 10,
                       .endStraightLength = 10,
                       .minRadius = 5,
                       .bendPenalty = 500});
  router.attachCorridor(&outsideCorridor);
  const RoutingObjective east{
      .source = {.x = 5, .y = 10, .heading = 6, .primitive = 0},
      .target = {.x = 75, .y = 10, .heading = 6, .primitive = 0}};
  const RoutingObjective west{
      .source = {.x = 75, .y = 32, .heading = 2, .primitive = 0},
      .target = {.x = 5, .y = 32, .heading = 2, .primitive = 0}};
  const Path firstEast = router.route(east);
  const Path firstWest = router.route(west);
  ASSERT_FALSE(firstEast.empty());
  ASSERT_FALSE(firstWest.empty());
  EXPECT_GT(countBends(firstEast), 0U);
  for (int i = 0; i < 600; ++i) {
    ASSERT_EQ(router.route(east), firstEast) << i;
    ASSERT_EQ(router.route(west), firstWest) << i;
  }
}

TEST(DubinsRouter, TheOrthogonalSearchStaysInTheCorridor) {
  Fixture f;
  // A wall leaves a gap along the edge at y = 0 only.
  f.block(120, 8, 180, HEIGHT - 1);
  EXPECT_TRUE(f.router.corridorBlocked(150, 100));
  EXPECT_FALSE(f.router.corridorBlocked(150, 3));
  const Path path = f.router.routeOrthogonal(ACROSS);
  ASSERT_FALSE(path.empty());
  EXPECT_TRUE(avoids(path, f.outsideCorridor));
  EXPECT_TRUE(std::ranges::all_of(path, [](const PathPoint& point) {
    return point.x < WIDTH && point.y < HEIGHT;
  }));
  EXPECT_EQ(path.front(), (PathPoint{.x = 30,
                                     .y = 100,
                                     .heading = 6,
                                     .primitive = f.primitives->straight(6)}));
  EXPECT_TRUE(path.back().samePlace(ACROSS.target));
}

TEST(DubinsRouter, TheOrthogonalSearchAddsTheProximityPenalties) {
  Fixture f;
  const Path plain = f.router.routeOrthogonal(ACROSS, true);
  ASSERT_FALSE(plain.empty());
  EXPECT_EQ(countBends(plain), 0U);

  std::vector<uint8_t> ridge(f.router.cells(), 0);
  raiseRidge(ridge);
  f.router.setStaticProximity(ridge);
  const Path aroundStatic = f.router.routeOrthogonal(ACROSS, true);
  ASSERT_FALSE(aroundStatic.empty());
  EXPECT_TRUE(leavesTheRidge(aroundStatic));
  // Without the penalties the ridge costs nothing.
  EXPECT_EQ(countBends(f.router.routeOrthogonal(ACROSS)), 0U);

  f.router.setStaticProximity(std::vector<uint8_t>(f.router.cells(), 0));
  raiseRidge(f.wire);
  const Path aroundWire = f.router.routeOrthogonal(ACROSS, true);
  ASSERT_FALSE(aroundWire.empty());
  EXPECT_TRUE(leavesTheRidge(aroundWire));
}

TEST(DubinsRouter, AnOrthogonalRouteIsEmptyWhenNoPathExists) {
  Fixture f;
  const RoutingObjective offGrid{
      .source = {.x = 3, .y = 100, .heading = 2, .primitive = 0},
      .target = {.x = 260, .y = 100, .heading = 6, .primitive = 0}};
  EXPECT_TRUE(f.router.routeOrthogonal(offGrid).empty());
  f.block(150, 0, 150, HEIGHT - 1);
  EXPECT_TRUE(f.router.routeOrthogonal(ACROSS).empty());
}

TEST(DubinsRouter, AFeedlineThatDoesNotSeparateTheEndsSetsNoRule) {
  Fixture f;
  // The feedline runs beside the straight path, with the source and the
  // target on the same side of it. A rule there would bar the straight path.
  const Path feedline = rowRun(*f.primitives, 95);
  f.router.setSingleCrossingFeedline(&feedline, 19, 1);
  const Path path = f.router.routeOrthogonal(ACROSS);
  ASSERT_FALSE(path.empty());
  EXPECT_EQ(countBends(path), 0U);
}

TEST(DubinsRouter, TheHeadOfTheFeedlineBlocksItsFarSide) {
  Fixture f;
  const Path feedline = columnRun(*f.primitives, 150);
  const RoutingObjective nearHead{
      .source = {.x = 30, .y = 20, .heading = 6, .primitive = 0},
      .target = {.x = 260, .y = 20, .heading = 6, .primitive = 0}};
  const Path free = f.router.routeOrthogonal(nearHead);
  ASSERT_FALSE(free.empty());
  EXPECT_EQ(countBends(free), 0U);

  // Within 40 cells of the first ten cells of the feedline, and beyond the 5
  // cells of its straight zone, the far side may not be entered at all.
  f.router.setSingleCrossingFeedline(&feedline, 5, 40);
  const Path path = f.router.routeOrthogonal(nearHead);
  ASSERT_FALSE(path.empty());
  EXPECT_TRUE(std::ranges::none_of(path, [](const PathPoint& point) {
    return point.x >= 160 && point.x <= 185 && point.y <= 45;
  }));
}

TEST(DubinsRouter, AWireLeavesTheFeedlineAtARightAngle) {
  Fixture f;
  const Path feedline = columnRun(*f.primitives, 150);
  const RoutingObjective diagonal{
      .source = {.x = 20, .y = 10, .heading = 5, .primitive = 0},
      .target = {.x = 200, .y = 190, .heading = 5, .primitive = 0}};
  // Without the feedline the path is one diagonal line.
  const Path free = f.router.routeOrthogonal(diagonal);
  ASSERT_FALSE(free.empty());
  EXPECT_EQ(countBends(free), 0U);

  // Within ten cells beyond the feedline, the wire runs straight on the
  // heading that leaves the feedline at a right angle, so its rendering
  // stays on one row there.
  f.router.setSingleCrossingFeedline(&feedline, 10, 1);
  const Path path = f.router.routeOrthogonal(diagonal);
  ASSERT_FALSE(path.empty());
  EXPECT_GT(countBends(path), 0U);
  std::optional<double> row;
  for (const Point& point : rendering(*f.primitives, path)) {
    if (point.x() > 150.0 && point.x() <= 160.0) {
      row = row.value_or(point.y());
      EXPECT_EQ(point.y(), *row) << point.x();
    }
  }
  EXPECT_TRUE(row.has_value());
}

TEST(DubinsRouter, TheStraightZonesOfAFeedlineCornerBlockWhereTheyMeet) {
  Fixture f;
  // The feedline runs toward positive y from the edge at y = 0, turns onto
  // heading 6 on a quarter turn, and runs to the edge at x = WIDTH - 1, so it
  // fences in the corner of the grid at x = WIDTH - 1 and y = 0.
  const auto moves = f.primitives->of(4);
  const auto turn = std::ranges::find_if(
      moves, [](const Primitive& move) { return move.exitHeading == 6; });
  ASSERT_NE(turn, moves.end());
  Path feedline;
  for (uint32_t y = 0; y < 100; ++y) {
    feedline.push_back({.x = 150,
                        .y = y,
                        .heading = 4,
                        .primitive = f.primitives->straight(4)});
  }
  for (const CellOffset& cell : turn->swept) {
    feedline.push_back({.x = static_cast<uint32_t>(150 + cell.dx),
                        .y = static_cast<uint32_t>(100 + cell.dy),
                        .heading = 4,
                        .primitive = turn->id});
  }
  const auto cornerX = static_cast<uint32_t>(150 + turn->dx);
  const auto cornerY = static_cast<uint32_t>(100 + turn->dy);
  for (uint32_t x = cornerX + 1; x < WIDTH; ++x) {
    feedline.push_back({.x = x,
                        .y = cornerY,
                        .heading = 6,
                        .primitive = f.primitives->straight(6)});
  }
  // The straight zones reach ten cells. The corner lies close enough to the
  // column of the first run for the two zones to overlap there.
  ASSERT_LE(cornerX, 160U);
  f.router.setSingleCrossingFeedline(&feedline, 10, 3);

  const RoutingObjective inward{
      .source = {.x = 40, .y = 150, .heading = 6, .primitive = 0},
      .target = {.x = 250, .y = 40, .heading = 6, .primitive = 0}};
  const Path path = f.router.routeOrthogonal(inward);
  ASSERT_FALSE(path.empty());
  std::size_t constrained = 0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const PathPoint& cell = path[i];
    const bool besideFirstRun = cell.x > 150 && cell.x <= 160;
    const bool besideLastRun = cell.y + 10 >= cornerY && cell.y < cornerY;
    if (besideFirstRun && besideLastRun) {
      ADD_FAILURE() << "entered both zones at " << cell.x << "," << cell.y;
    } else if (besideFirstRun && cell.y < cornerY) {
      ++constrained;
      EXPECT_FALSE(onTurn(*f.primitives, path, i, 10)) << i;
      EXPECT_EQ(path[i - 1].heading, 6) << i;
    } else if (besideLastRun && cell.x > 160) {
      ++constrained;
      EXPECT_FALSE(onTurn(*f.primitives, path, i, 10)) << i;
      EXPECT_EQ(path[i - 1].heading, 0) << i;
    }
  }
  EXPECT_GT(constrained, 0U);
}

TEST(DubinsRouter, AnExemptCellPassesTheCrossingTestOnEveryHeading) {
  Fixture f;
  const Path wire = columnRun(*f.primitives, 150);
  f.router.buildOrthogonalConstraints({wire}, {false}, 6);
  ASSERT_FALSE(f.router.crossingAllowedOrthogonal(150, 100, 5));

  // The exemption ignores an index outside the grid and a repeated index.
  f.router.setCrossingExemption({cellIndex(150, 100), cellIndex(150, 100),
                                 static_cast<uint32_t>(f.router.cells())});
  for (uint32_t heading = 0; heading < NUM_HEADINGS; ++heading) {
    EXPECT_TRUE(f.router.crossingAllowedOrthogonal(
        150, 100, static_cast<Heading>(heading)))
        << heading;
  }
  EXPECT_FALSE(f.router.crossingAllowedOrthogonal(150, 101, 5));
  // The constraints themselves do not know the exemption.
  EXPECT_EQ(f.router.constraintMaskAt(150, 100), 1U << 4U);

  // A new exemption replaces the old one, and an empty one removes it.
  f.router.setCrossingExemption({cellIndex(150, 101)});
  EXPECT_FALSE(f.router.crossingAllowedOrthogonal(150, 100, 5));
  EXPECT_TRUE(f.router.crossingAllowedOrthogonal(150, 101, 5));
  f.router.setCrossingExemption({});
  EXPECT_FALSE(f.router.crossingAllowedOrthogonal(150, 101, 5));
  // Without the exemption, the search still obeys the constraints.
  const RoutingObjective slanted{
      .source = {.x = 30, .y = 40, .heading = 6, .primitive = 0},
      .target = {.x = 260, .y = 160, .heading = 6, .primitive = 0}};
  const Path path = f.router.routeOrthogonal(slanted);
  ASSERT_FALSE(path.empty());
  EXPECT_TRUE(crossingFailures(f.router, path).empty());

  // A cell outside the grid is never allowed.
  EXPECT_FALSE(f.router.crossingAllowedOrthogonal(WIDTH, 100, 6));
  EXPECT_FALSE(f.router.crossingAllowedOrthogonal(100, HEIGHT, 6));
}

TEST(DubinsRouter, TheCrossingConstraintsAreTheOnesBuiltLast) {
  Fixture f;
  EXPECT_TRUE(f.router.crossingConstraints().empty());
  f.router.buildOrthogonalConstraints({columnRun(*f.primitives, 150)}, {false},
                                      6);
  f.router.buildOrthogonalConstraints({rowRun(*f.primitives, 100)}, {false}, 6);
  const CrossingConstraints& constraints = f.router.crossingConstraints();
  ASSERT_FALSE(constraints.empty());
  EXPECT_EQ(constraints.maskAt(150, 50), 0U);
  EXPECT_EQ(constraints.maskAt(50, 100), 1U << 6U);

  // The constraints do not know the exemption.
  f.router.setCrossingExemption({cellIndex(50, 100)});
  EXPECT_TRUE(f.router.crossingAllowedOrthogonal(50, 100, 5));
  EXPECT_FALSE(constraints.allowed(50, 100, 5));

  f.router.clearOrthogonalConstraints();
  EXPECT_TRUE(f.router.crossingConstraints().empty());
  EXPECT_EQ(f.router.crossingConstraints().heldBytes(), 0U);
}

TEST(DubinsRouter, AnExemptCellIsFreeOfTheSingleCrossingRule) {
  Fixture f;
  const Path feedline = columnRun(*f.primitives, 150);
  f.router.setSingleCrossingFeedline(&feedline, 5, 40);
  const RoutingObjective nearHead{
      .source = {.x = 30, .y = 20, .heading = 6, .primitive = 0},
      .target = {.x = 260, .y = 20, .heading = 6, .primitive = 0}};
  const Path detour = f.router.routeOrthogonal(nearHead);
  ASSERT_FALSE(detour.empty());
  EXPECT_GT(countBends(detour), 0U);

  // A band of exempt cells along the straight path opens it again.
  std::vector<uint32_t> band;
  for (uint32_t y = 15; y <= 25; ++y) {
    for (uint32_t x = 140; x <= 200; ++x) {
      band.push_back(cellIndex(x, y));
    }
  }
  f.router.setCrossingExemption(band);
  const Path straight = f.router.routeOrthogonal(nearHead);
  ASSERT_FALSE(straight.empty());
  EXPECT_EQ(countBends(straight), 0U);
}

TEST(DubinsRouter, TheFreeStripWithoutObstaclesIsTheWholeGrid) {
  Fixture f;
  f.router.attachObstacles(nullptr);
  const CellBox box = f.router.freeStripAlong(
      {.x = 100, .y = 100, .heading = 6, .primitive = 0},
      {.x = 200, .y = 100, .heading = 6, .primitive = 0});
  EXPECT_EQ(box.minX, 0U);
  EXPECT_EQ(box.maxX, WIDTH - 1);
  EXPECT_EQ(box.minY, 0U);
  EXPECT_EQ(box.maxY, HEIGHT - 1);
}

TEST(DubinsRouter, TheFreeStripOfOneCellIsThatCell) {
  const Fixture f;
  const PathPoint end{.x = 120, .y = 80, .heading = 6, .primitive = 0};
  const CellBox box = f.router.freeStripAlong(end, end);
  EXPECT_EQ(box.minX, 120U);
  EXPECT_EQ(box.maxX, 120U);
  EXPECT_EQ(box.minY, 80U);
  EXPECT_EQ(box.maxY, 80U);
}

TEST(DubinsRouter, TheFreeStripStopsAtTheGridEdge) {
  const Fixture f;
  // No obstacle stops the strip, so both sides grow to the edge of the grid.
  const CellBox box = f.router.freeStripAlong(
      {.x = 100, .y = 100, .heading = 6, .primitive = 0},
      {.x = 200, .y = 100, .heading = 6, .primitive = 0});
  EXPECT_EQ(box.minX, 100U);
  EXPECT_EQ(box.maxX, 200U);
  EXPECT_EQ(box.minY, 0U);
  EXPECT_EQ(box.maxY, HEIGHT - 1);
}

TEST(DubinsRouter, AFreeStripEndOffTheGridCountsAsTheNearestCell) {
  Fixture f;
  f.obstacles.setCell(150, 60);
  const auto sameBox = [](const CellBox& a, const CellBox& b) {
    return a.minX == b.minX && a.maxX == b.maxX && a.minY == b.minY &&
           a.maxY == b.maxY;
  };
  // A source two cells from the edge at x = 0 faces toward negative x, so
  // its search start lies off the grid, where the coordinate wraps.
  const PathPoint wrapped = f.router.sanitize(
      {.x = 2, .y = 100, .heading = 2, .primitive = 0}, false);
  ASSERT_GE(wrapped.x, WIDTH);
  const PathPoint inside{.x = 100, .y = 100, .heading = 2, .primitive = 0};
  EXPECT_TRUE(
      sameBox(f.router.freeStripAlong(wrapped, inside),
              f.router.freeStripAlong({.x = WIDTH - 1, .y = 100}, inside)));
  EXPECT_TRUE(
      sameBox(f.router.freeStripAlong(inside, wrapped),
              f.router.freeStripAlong(inside, {.x = WIDTH - 1, .y = 100})));
  // An end past the last row moves onto the last row.
  EXPECT_TRUE(
      sameBox(f.router.freeStripAlong({.x = 100, .y = HEIGHT + 5}, inside),
              f.router.freeStripAlong({.x = 100, .y = HEIGHT - 1}, inside)));
  // Two ends that meet on the nearest cell give that cell.
  const CellBox corner = f.router.freeStripAlong(
      {.x = WIDTH + 1, .y = HEIGHT + 1}, {.x = WIDTH + 7, .y = HEIGHT + 3});
  EXPECT_TRUE(sameBox(corner, {.minX = WIDTH - 1,
                               .maxX = WIDTH - 1,
                               .minY = HEIGHT - 1,
                               .maxY = HEIGHT - 1}));
}

TEST(DubinsRouter, TheFreeStripOfAColumnGrowsSideways) {
  Fixture f;
  for (uint32_t y = 0; y < HEIGHT; ++y) {
    f.obstacles.setCell(80, y);
    f.obstacles.setCell(130, y);
  }
  f.router.attachObstacles(&f.obstacles);
  const CellBox box = f.router.freeStripAlong(
      {.x = 100, .y = 50, .heading = 4, .primitive = 0},
      {.x = 100, .y = 150, .heading = 4, .primitive = 0});
  EXPECT_EQ(box.minX, 81U);
  EXPECT_EQ(box.maxX, 129U);
  EXPECT_EQ(box.minY, 50U);
  EXPECT_EQ(box.maxY, 150U);
}

TEST(DubinsRouter, TheFreeStripOfADiagonalStopsAtAThinObstacle) {
  // At 45 degrees, the edge of one offset does not touch every cell between
  // it and the edge of the offset before. Those cells still bound the strip.
  Fixture f;
  const PathPoint from{.x = 20, .y = 20, .heading = 5, .primitive = 0};
  const PathPoint to{.x = 40, .y = 40, .heading = 5, .primitive = 0};
  f.obstacles.setCell(30, 31);
  f.router.attachObstacles(&f.obstacles);
  // The side of the obstacle does not grow. The other side grows until its
  // edge reaches the row 0.
  const CellBox beside = f.router.freeStripAlong(from, to);
  EXPECT_EQ(beside.minX, 20U);
  EXPECT_EQ(beside.maxX, 60U);
  EXPECT_EQ(beside.minY, 0U);
  EXPECT_EQ(beside.maxY, 40U);

  // Walls on the diagonals next to the segment hold the strip to the
  // segment.
  for (uint32_t x = 0; x + 1 < HEIGHT; ++x) {
    f.obstacles.setCell(x, x + 1);
    f.obstacles.setCell(x + 1, x);
  }
  const CellBox walled = f.router.freeStripAlong(from, to);
  EXPECT_EQ(walled.minX, 20U);
  EXPECT_EQ(walled.maxX, 40U);
  EXPECT_EQ(walled.minY, 20U);
  EXPECT_EQ(walled.maxY, 40U);
}

TEST(DubinsRouter, EveryObstacleBesideASlopedSegmentNarrowsTheFreeStrip) {
  // Every cell next to the segment, off its line, lies in the strip of the
  // empty grid. An obstacle on any of them must narrow the strip, whatever
  // the slope of the segment.
  Fixture f;
  const auto sameBox = [](const CellBox& a, const CellBox& b) {
    return a.minX == b.minX && a.maxX == b.maxX && a.minY == b.minY &&
           a.maxY == b.maxY;
  };
  const std::vector<std::pair<PathPoint, PathPoint>> segments = {
      {{.x = 100, .y = 100}, {.x = 140, .y = 120}},  // slope 1/2
      {{.x = 100, .y = 100}, {.x = 160, .y = 115}},  // slope 1/4
      {{.x = 100, .y = 100}, {.x = 110, .y = 140}},  // slope 4
      {{.x = 100, .y = 100}, {.x = 130, .y = 130}},  // slope 1
      {{.x = 100, .y = 130}, {.x = 130, .y = 100}}}; // slope -1
  for (const auto& [from, to] : segments) {
    const CellBox empty = f.router.freeStripAlong(from, to);
    const double dx = static_cast<double>(to.x) - from.x;
    const double dy = static_cast<double>(to.y) - from.y;
    const double length = std::hypot(dx, dy);
    uint32_t tested = 0;
    for (uint32_t y = std::min(from.y, to.y) - 4;
         y <= std::max(from.y, to.y) + 4; ++y) {
      for (uint32_t x = std::min(from.x, to.x) - 4;
           x <= std::max(from.x, to.x) + 4; ++x) {
        const double rx = static_cast<double>(x) - from.x;
        const double ry = static_cast<double>(y) - from.y;
        const double along = ((rx * dx) + (ry * dy)) / (length * length);
        const double offset = std::abs((ry * dx) - (rx * dy)) / length;
        if (along < 0.0 || along > 1.0 || offset < 0.5 || offset > 3.0) {
          continue;
        }
        f.obstacles.setCell(x, y);
        EXPECT_FALSE(sameBox(f.router.freeStripAlong(from, to), empty))
            << from.x << "," << from.y << " to " << to.x << "," << to.y
            << ": obstacle " << x << "," << y;
        f.obstacles.setCell(x, y, false);
        ++tested;
      }
    }
    EXPECT_GT(tested, 100U);
  }
}

TEST(DubinsRouter, SearchEndsOnTheSamePoseJoinTheStubs) {
  // Thirty cells of stub on each side bring both ends to (40, 50) on heading
  // 6, so there is nothing to search; the path is the two stubs joined.
  Fixture f;
  f.router.setParams({.startStraightLength = 30,
                      .endStraightLength = 30,
                      .minRadius = 5,
                      .bendPenalty = 500});
  const RoutingObjective meeting{
      .source = {.x = 10, .y = 50, .heading = 6, .primitive = 0},
      .target = {.x = 70, .y = 50, .heading = 6, .primitive = 0}};
  const uint16_t straight = f.primitives->straight(6);
  for (const Path& path :
       {f.router.route(meeting), f.router.routeOrthogonal(meeting)}) {
    ASSERT_EQ(path.size(), 61U);
    for (uint32_t i = 0; i < path.size(); ++i) {
      EXPECT_EQ(path[i].x, 10 + i);
      EXPECT_EQ(path[i].y, 50U);
      EXPECT_EQ(path[i].heading, 6);
      EXPECT_EQ(path[i].primitive, straight);
    }
  }
  EXPECT_EQ(f.router.loopGuardRejections(), 0U);

  // One cell further apart, the ends no longer meet and the search runs.
  RoutingObjective apart = meeting;
  apart.target.x = 71;
  EXPECT_EQ(f.router.route(apart).size(), 62U);
}

TEST(DubinsRouter, ATurnAtTheSearchStartStartsOnTheLastCellOfTheStub) {
  // The source faces heading 0 and the target lies toward positive x, so the
  // search turns right away. The arc starts on the last cell of the source
  // stub, which keeps the tag of the stub. The first point of the turn is the
  // next cell the arc sweeps, and the point after the turn is the end of the
  // arc.
  Fixture f;
  const RoutingObjective turning{
      .source = {.x = 30, .y = 100, .heading = 0, .primitive = 0},
      .target = {.x = 260, .y = 100, .heading = 6, .primitive = 0}};
  const Path path = f.router.route(turning);
  ASSERT_GT(path.size(), 12U);
  for (uint32_t i = 0; i <= 10; ++i) {
    EXPECT_EQ(path[i].heading, 0) << i;
    EXPECT_EQ(path[i].primitive, f.primitives->straight(0)) << i;
  }
  EXPECT_TRUE(path[10].samePlace({.x = 30, .y = 90}));
  const Primitive* turn = f.primitives->find(0, path[11].primitive);
  ASSERT_NE(turn, nullptr);
  EXPECT_NE(turn->exitHeading, 0);
  ASSERT_EQ(turn->swept.front(), CellOffset{});
  EXPECT_TRUE(
      path[11].samePlace({.x = static_cast<uint32_t>(30 + turn->swept[1].dx),
                          .y = static_cast<uint32_t>(90 + turn->swept[1].dy)}));
  std::size_t after = 11;
  while (path[after].primitive == path[11].primitive &&
         path[after].heading == 0) {
    ++after;
  }
  EXPECT_TRUE(
      path[after].samePlace({.x = static_cast<uint32_t>(30 + turn->dx),
                             .y = static_cast<uint32_t>(90 + turn->dy)}));
  // The segments record the end of the arc for that turn too.
  const SegmentedPath segmented = reconstructSegments(*f.primitives, path);
  ASSERT_GE(segmented.segments.size(), 2U);
  EXPECT_TRUE(segmented.segments[1].cells[0].samePlace(path[after]));
}

TEST(DubinsRouter, AWallWhoseCellsShareEdgesStopsTheSearch) {
  // A staircase two cells thick along x + y = 150 cuts the grid in two. Its
  // cells share edges, so no step of either search passes it, diagonal steps
  // included.
  constexpr uint32_t side = 300;
  auto primitives = std::make_shared<const MovePrimitives>(5);
  SearchScratch scratch(side, side);
  grid::BitGrid outsideCorridor(side, side);
  DubinsRouter router(primitives, scratch,
                      {.startStraightLength = 10,
                       .endStraightLength = 10,
                       .minRadius = 5,
                       .bendPenalty = 500});
  for (uint32_t x = 0; x <= 151; ++x) {
    outsideCorridor.setCell(x, 151 - x);
    if (x <= 150) {
      outsideCorridor.setCell(x, 150 - x);
    }
  }
  router.attachCorridor(&outsideCorridor);
  const RoutingObjective across{
      .source = {.x = 20, .y = 20, .heading = 5, .primitive = 0},
      .target = {.x = 100, .y = 100, .heading = 5, .primitive = 0}};
  EXPECT_TRUE(router.route(across).empty());
  EXPECT_TRUE(router.routeOrthogonal(across).empty());
  router.setHeuristic(Heuristic::Octile);
  EXPECT_TRUE(router.route(across).empty());
}

TEST(DubinsRouter, ASearchStartOutsideTheCorridorGivesNoPath) {
  // The search starts on the cell ten cells east of the source of ACROSS. It
  // tests that cell as it tests every cell a move enters.
  for (const bool packed : {true, false}) {
    Fixture f;
    f.outsideCorridor.setCell(40, 100);
    if (packed) {
      f.router.attachCorridor(&f.outsideCorridor);
    } else {
      f.router.attachCorridorUnpacked(&f.outsideCorridor);
    }
    EXPECT_TRUE(f.router.route(ACROSS).empty()) << packed;
    EXPECT_TRUE(f.router.route(ACROSS, true).empty()) << packed;
    EXPECT_TRUE(f.router.routeOrthogonal(ACROSS).empty()) << packed;
    EXPECT_TRUE(f.router.routeOrthogonal(ACROSS, true).empty()) << packed;
  }
}

TEST(DubinsRouter, ASearchGoalOutsideTheCorridorGivesNoPathWithoutASearch) {
  // The search ends on the cell ten cells west of the target of ACROSS. No
  // move can reach a goal outside the corridor, so the call returns before
  // the search writes a single record to the scratch.
  for (const bool packed : {true, false}) {
    Fixture f;
    f.outsideCorridor.setCell(250, 100);
    if (packed) {
      f.router.attachCorridor(&f.outsideCorridor);
    } else {
      f.router.attachCorridorUnpacked(&f.outsideCorridor);
    }
    const auto records = [&] {
      const std::span<const SearchNode> nodes(f.scratch.data(),
                                              f.scratch.size());
      return std::ranges::count_if(nodes, [&](const SearchNode& node) {
        return node.iteration == f.scratch.iteration();
      });
    };
    for (const bool usePenalty : {false, true}) {
      EXPECT_TRUE(f.router.route(ACROSS, usePenalty).empty()) << packed;
      EXPECT_EQ(records(), 0) << packed << usePenalty;
      EXPECT_TRUE(f.router.routeOrthogonal(ACROSS, usePenalty).empty())
          << packed;
      EXPECT_EQ(records(), 0) << packed << usePenalty;
    }
  }
}

TEST(DubinsRouter, TheCellsOfTheStubsAreNotTested) {
  // The caller keeps the stubs free. A blocked cell under the source stub
  // or under the target stub, other than the search ends, is not seen.
  Fixture f;
  f.outsideCorridor.setCell(30, 100);
  f.outsideCorridor.setCell(35, 100);
  f.outsideCorridor.setCell(255, 100);
  f.router.attachCorridor(&f.outsideCorridor);
  for (const Path& path :
       {f.router.route(ACROSS), f.router.routeOrthogonal(ACROSS)}) {
    ASSERT_FALSE(path.empty());
    EXPECT_TRUE(path.front().samePlace(ACROSS.source));
    EXPECT_TRUE(path[5].samePlace({.x = 35, .y = 100}));
    EXPECT_TRUE(path.back().samePlace(ACROSS.target));
  }
}

} // namespace
