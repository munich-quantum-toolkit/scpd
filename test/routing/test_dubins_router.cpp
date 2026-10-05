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
#include "mqt-scpd/routing/DubinsRouter.hpp"
#include "mqt-scpd/routing/Heading.hpp"
#include "mqt-scpd/routing/Path.hpp"
#include "mqt-scpd/routing/PathGeometry.hpp"
#include "mqt-scpd/routing/Primitives.hpp"
#include "mqt-scpd/routing/SearchScratch.hpp"
#include "mqt-scpd/routing/SelfIntersection.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <stdexcept>
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
  grid::BitGrid corridor{WIDTH, HEIGHT};
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
    router.attachCorridor(&corridor);
    router.attachWireProximity(&wire);
  }

  /// Block a rectangle of the corridor, both bounds included.
  void block(const uint32_t x0, const uint32_t y0, const uint32_t x1,
             const uint32_t y1) {
    for (uint32_t y = y0; y <= y1; ++y) {
      for (uint32_t x = x0; x <= x1; ++x) {
        corridor.setCell(x, y);
        obstacles.setCell(x, y);
      }
    }
    router.attachCorridor(&corridor);
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

/// A straight run south down one column of the whole grid.
Path southRun(const MovePrimitives& primitives, const uint32_t x) {
  Path run;
  for (uint32_t y = 0; y < HEIGHT; ++y) {
    run.push_back(
        {.x = x, .y = y, .heading = 4, .primitive = primitives.straight(4)});
  }
  return run;
}

/// A straight run east along one row of the whole grid.
Path eastRun(const MovePrimitives& primitives, const uint32_t y) {
  Path run;
  for (uint32_t x = 0; x < WIDTH; ++x) {
    run.push_back(
        {.x = x, .y = y, .heading = 6, .primitive = primitives.straight(6)});
  }
  return run;
}

/// Raises a ridge of penalty along the straight way of ACROSS.
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

TEST(DubinsRouter, TheCorridorIsNeverEntered) {
  Fixture f;
  f.block(120, 60, 180, 140);
  const Path path = f.router.route(ACROSS);
  ASSERT_FALSE(path.empty());
  EXPECT_TRUE(avoids(path, f.corridor));
  EXPECT_GT(countBends(path), 0U);
}

TEST(DubinsRouter, AnUnreachableTargetGivesAnEmptyPath) {
  Fixture f;
  for (uint32_t y = 0; y < HEIGHT; ++y) {
    f.corridor.setCell(150, y);
  }
  f.router.attachCorridor(&f.corridor);
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
  // A turn spans at least the bend radius along the axis it turns into.
  for (const PathSegment& segment : segmented.segments) {
    if (segment.straight) {
      continue;
    }
    const Primitive* move =
        f.primitives->find(segment.heading, segment.primitive);
    ASSERT_NE(move, nullptr);
    EXPECT_GE(std::max(std::abs(move->dx), std::abs(move->dy)), 4);
  }
}

TEST(DubinsRouter, TheOctileHeuristicFindsAPathOfTheSameCost) {
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
  // Short posts stand in rows across the grid, one row on the straight way.
  // The distance field runs between them, so it no longer equals the octile
  // distance.
  for (uint32_t y = 20; y <= 180; y += 20) {
    for (uint32_t x = 60; x <= 240; x += 20) {
      f.corridor.setCell(x, y);
      f.corridor.setCell(x + 1, y);
    }
  }
  f.router.attachCorridor(&f.corridor);
  const auto weaves = [&](const Path& path) {
    return !path.empty() && avoids(path, f.corridor) &&
           path.back().samePlace(ACROSS.target);
  };
  EXPECT_TRUE(weaves(f.router.route(ACROSS)));
  f.router.setHeuristic(Heuristic::Octile);
  EXPECT_TRUE(weaves(f.router.route(ACROSS)));
}

TEST(DubinsRouter, TheBendLowerBoundKeepsTheCheapestWay) {
  // The turning a state still owes is a lower bound on what it has left to
  // pay, so adding it to the estimate changes how many states the search
  // expands, not the cost of the way it finds.
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

TEST(DubinsRouter, GridsMustMatchTheRouterGrid) {
  Fixture f;
  const grid::BitGrid wrong(10, 10);
  EXPECT_THROW(f.router.attachObstacles(&wrong), std::invalid_argument);
  EXPECT_THROW(f.router.attachCorridor(&wrong), std::invalid_argument);
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
  // A wire running north to south across the middle of the grid.
  Path wire;
  for (uint32_t y = 20; y < 180; ++y) {
    wire.push_back({.x = 150,
                    .y = y,
                    .heading = 4,
                    .primitive = f.primitives->straight(4)});
  }
  f.router.buildOrthogonalConstraints({wire}, {false}, 6);

  // A route travelling east crosses it at a right angle and is allowed.
  EXPECT_TRUE(f.router.crossingAllowedOrthogonal(150, 100, 6));
  EXPECT_TRUE(f.router.crossingAllowedOrthogonal(150, 100, 2));
  // A route travelling at 45 degrees is not.
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
  const Path path = f.router.routeOrthogonal(ACROSS);
  ASSERT_FALSE(path.empty());
  // Every step onto a constrained cell is one the search's own test admits.
  for (std::size_t i = 1; i < path.size(); ++i) {
    EXPECT_TRUE(f.router.crossingAllowedOrthogonal(path[i].x, path[i].y,
                                                   path[i - 1].heading))
        << i << " at " << path[i].x << "," << path[i].y;
  }
}

TEST(DubinsRouter, ASingleCrossingWireCannotComeBack) {
  Fixture f;
  // The feedline splits the grid; the wire has to cross it once.
  Path feedline;
  for (uint32_t y = 0; y < HEIGHT; ++y) {
    feedline.push_back({.x = 150,
                        .y = y,
                        .heading = 4,
                        .primitive = f.primitives->straight(4)});
  }
  f.router.setSingleCrossingFeedline(&feedline, 19, 1);
  const Path path = f.router.routeOrthogonal(ACROSS);
  ASSERT_FALSE(path.empty());

  // Once past the feedline the wire only moves away from it.
  std::size_t crossings = 0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    if ((path[i - 1].x < 150) != (path[i].x < 150)) {
      ++crossings;
    }
  }
  EXPECT_EQ(crossings, 1U);

  // The overlay is undone after the search, so the next route is free.
  f.router.setSingleCrossingFeedline(nullptr, 0, 0);
  EXPECT_TRUE(f.router.crossingAllowedOrthogonal(160, 100, 4));
}

TEST(DubinsRouter, ARoutedPathNeverCrossesItself) {
  Fixture f;
  // A corridor with two pockets, where a search over states could return to
  // a cell it already used on another heading.
  f.block(80, 0, 90, 150);
  f.block(150, 50, 160, HEIGHT - 1);
  f.block(220, 0, 230, 150);
  const Path path = f.router.route(ACROSS, true);
  if (!path.empty()) {
    PathLoopScratch scratch;
    EXPECT_FALSE(pathSelfIntersects(path, WIDTH, HEIGHT, scratch));
  }
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
  // The packed working grid folds the corridor and the static proximity
  // into one byte per cell so that the search touches one cache line per
  // swept cell. It is a cache, so the two forms must route alike.
  Fixture f;
  f.block(120, 60, 180, 140);
  f.obstacles.setCell(200, 90);
  f.router.attachObstacles(&f.obstacles);
  f.router.computeStaticProximity(12, 30);

  f.router.attachCorridor(&f.corridor);
  const Path packed = f.router.route(ACROSS, true);
  f.router.attachCorridorUnpacked(&f.corridor);
  const Path unpacked = f.router.route(ACROSS, true);
  ASSERT_FALSE(packed.empty());
  EXPECT_EQ(packed, unpacked);

  // Without the penalties the two forms agree as well.
  f.router.attachCorridor(&f.corridor);
  const Path packedPlain = f.router.route(ACROSS);
  f.router.attachCorridorUnpacked(&f.corridor);
  const Path unpackedPlain = f.router.route(ACROSS);
  ASSERT_FALSE(packedPlain.empty());
  EXPECT_EQ(packedPlain, unpackedPlain);
}

TEST(DubinsRouter, ARouterOutlivesWhateverBuiltItsPrimitives) {
  // The router shares ownership of its primitive table, so a router that
  // outlives the scope it was built in still routes.
  SearchScratch scratch(WIDTH, HEIGHT);
  grid::BitGrid corridor(WIDTH, HEIGHT);
  auto router = [&] {
    auto primitives = std::make_shared<const MovePrimitives>(5);
    DubinsRouter built(primitives, scratch,
                       {.startStraightLength = 10,
                        .endStraightLength = 10,
                        .minRadius = 5,
                        .bendPenalty = 500});
    built.attachCorridor(&corridor);
    return built;
  }();
  EXPECT_EQ(router.primitives().minRadius(), 5U);
  EXPECT_FALSE(router.route(ACROSS).empty());
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
  EXPECT_THROW(f.router.computeStaticProximity(10, 128), std::invalid_argument);
  EXPECT_THROW(f.router.computeStaticProximityWindow(10, 128, window),
               std::invalid_argument);

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

  // New obstacles on the straight way raise penalties inside the window.
  for (uint32_t x = 140; x <= 160; ++x) {
    f.obstacles.setCell(x, 100);
  }
  f.router.computeStaticProximityWindow(
      12, 30, {.minX = 120, .maxX = 180, .minY = 80, .maxY = 120});
  // The working grid is out of date, so the route reads the corridor and the
  // new penalties directly, and agrees with the route on the packed grid.
  const Path unpacked = f.router.route(ACROSS, true);
  f.router.attachCorridor(&f.corridor);
  const Path packed = f.router.route(ACROSS, true);
  ASSERT_FALSE(unpacked.empty());
  EXPECT_GT(countBends(unpacked), 0U);
  EXPECT_EQ(unpacked, packed);
}

TEST(DubinsRouter, OnlyStraightMovesNeverEndOnADiagonalHeading) {
  Fixture f;
  // The target lies off the line of the source, so the cheapest way runs
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
  // The target stub runs north across the source stub, so every way between
  // the two stubs closes a loop.
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
  grid::BitGrid corridor(width, height);
  for (uint32_t y = 0; y < 25; ++y) {
    corridor.setCell(40, y);
  }
  DubinsRouter router(std::make_shared<const MovePrimitives>(5), scratch,
                      {.startStraightLength = 10,
                       .endStraightLength = 10,
                       .minRadius = 5,
                       .bendPenalty = 500});
  router.attachCorridor(&corridor);
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
  // A wall leaves a gap along the top edge of the grid only.
  f.block(120, 8, 180, HEIGHT - 1);
  EXPECT_TRUE(f.router.corridorBlocked(150, 100));
  EXPECT_FALSE(f.router.corridorBlocked(150, 3));
  const Path path = f.router.routeOrthogonal(ACROSS);
  ASSERT_FALSE(path.empty());
  EXPECT_TRUE(avoids(path, f.corridor));
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
  // The feedline runs beside the straight way, with the source and the target
  // on the same side of it. A rule there would bar the straight way.
  const Path feedline = eastRun(*f.primitives, 95);
  f.router.setSingleCrossingFeedline(&feedline, 19, 1);
  const Path path = f.router.routeOrthogonal(ACROSS);
  ASSERT_FALSE(path.empty());
  EXPECT_EQ(countBends(path), 0U);
}

TEST(DubinsRouter, TheHeadOfTheFeedlineBlocksItsFarSide) {
  Fixture f;
  const Path feedline = southRun(*f.primitives, 150);
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
  const Path feedline = southRun(*f.primitives, 150);
  const RoutingObjective diagonal{
      .source = {.x = 20, .y = 10, .heading = 5, .primitive = 0},
      .target = {.x = 200, .y = 190, .heading = 5, .primitive = 0}};
  // Without the feedline the way is one diagonal line.
  const Path free = f.router.routeOrthogonal(diagonal);
  ASSERT_FALSE(free.empty());
  EXPECT_EQ(countBends(free), 0U);

  // Within ten cells beyond the feedline, every move starts on the heading
  // that leaves the feedline at a right angle.
  f.router.setSingleCrossingFeedline(&feedline, 10, 1);
  const Path path = f.router.routeOrthogonal(diagonal);
  ASSERT_FALSE(path.empty());
  EXPECT_GT(countBends(path), 0U);
  std::size_t beyond = 0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    if (path[i].x > 150 && path[i].x <= 160) {
      ++beyond;
      EXPECT_EQ(path[i - 1].heading, 6) << i;
    }
  }
  EXPECT_GT(beyond, 0U);
}

TEST(DubinsRouter, TheStraightZonesOfAFeedlineCornerBlockWhereTheyMeet) {
  Fixture f;
  // The feedline runs south from the top edge, turns east on a quarter turn,
  // and runs to the right edge, so it fences in the top right of the grid.
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
  // column of the south run for the two zones to overlap there.
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
    const bool besideSouthRun = cell.x > 150 && cell.x <= 160;
    const bool besideEastRun = cell.y + 10 >= cornerY && cell.y < cornerY;
    if (besideSouthRun && besideEastRun) {
      ADD_FAILURE() << "entered both zones at " << cell.x << "," << cell.y;
    } else if (besideSouthRun && cell.y < cornerY) {
      ++constrained;
      EXPECT_EQ(path[i - 1].heading, 6) << i;
    } else if (besideEastRun && cell.x > 160) {
      ++constrained;
      EXPECT_EQ(path[i - 1].heading, 0) << i;
    }
  }
  EXPECT_GT(constrained, 0U);
}

TEST(DubinsRouter, AnExemptCellPassesTheCrossingTestOnEveryHeading) {
  Fixture f;
  const Path wire = southRun(*f.primitives, 150);
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

  // A cell outside the grid is never allowed.
  EXPECT_FALSE(f.router.crossingAllowedOrthogonal(WIDTH, 100, 6));
  EXPECT_FALSE(f.router.crossingAllowedOrthogonal(100, HEIGHT, 6));
}

TEST(DubinsRouter, AnExemptCellIsFreeOfTheSingleCrossingRule) {
  Fixture f;
  const Path feedline = southRun(*f.primitives, 150);
  f.router.setSingleCrossingFeedline(&feedline, 5, 40);
  const RoutingObjective nearHead{
      .source = {.x = 30, .y = 20, .heading = 6, .primitive = 0},
      .target = {.x = 260, .y = 20, .heading = 6, .primitive = 0}};
  const Path detour = f.router.routeOrthogonal(nearHead);
  ASSERT_FALSE(detour.empty());
  EXPECT_GT(countBends(detour), 0U);

  // A band of exempt cells along the straight way opens it again.
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

} // namespace
