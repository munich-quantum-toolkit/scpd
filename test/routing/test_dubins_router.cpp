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
  std::vector<uint8_t> wire = std::vector<uint8_t>(
      static_cast<std::size_t>(WIDTH) * HEIGHT, 0);
  DubinsRouter router{primitives, scratch,
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
  void block(const uint32_t x0, const uint32_t y0, const uint32_t x1, const uint32_t y1) {
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
  for (const PathPoint& point : path) {
    if (point.x < mask.width() && point.y < mask.height() && mask.testCell(point.x, point.y)) {
      return false;
    }
  }
  return true;
}

const RoutingObjective ACROSS{{.x = 30, .y = 100, .heading = 6, .primitive = 0},
                              {.x = 260, .y = 100, .heading = 6, .primitive = 0}};

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
  const RoutingObjective offGrid{{.x = 3, .y = 100, .heading = 2, .primitive = 0},
                                 {.x = 260, .y = 100, .heading = 6, .primitive = 0}};
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
    const Primitive* move = f.primitives->find(segment.heading, segment.primitive);
    ASSERT_NE(move, nullptr) << i;
    EXPECT_EQ(move->exitHeading, segmented.segments[i + 1].heading) << i;
  }
  // A turn spans at least the bend radius along the axis it turns into.
  for (const PathSegment& segment : segmented.segments) {
    if (segment.straight) {
      continue;
    }
    const Primitive* move = f.primitives->find(segment.heading, segment.primitive);
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
  EXPECT_THROW(static_cast<void>(f.router.route(ACROSS, true)), std::logic_error);
  f.router.attachCorridor(nullptr);
  EXPECT_THROW(static_cast<void>(f.router.route(ACROSS)), std::logic_error);
  EXPECT_THROW(static_cast<void>(f.router.routeOrthogonal(ACROSS)), std::logic_error);
}

TEST(DubinsRouter, GridsMustMatchTheRouterGrid) {
  Fixture f;
  const grid::BitGrid wrong(10, 10);
  EXPECT_THROW(f.router.attachObstacles(&wrong), std::invalid_argument);
  EXPECT_THROW(f.router.attachCorridor(&wrong), std::invalid_argument);
  const std::vector<uint8_t> small(100, 0);
  EXPECT_THROW(f.router.attachWireProximity(&small), std::invalid_argument);
  EXPECT_THROW(f.router.setStaticProximity(small), std::invalid_argument);
  std::vector<uint8_t> tooLarge(f.router.cells(), 200);
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
  f.router.computeStaticProximityWindow(8, 30, {.minX = 90, .maxX = 120, .minY = 90, .maxY = 110});
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
    wire.push_back({.x = 150, .y = y, .heading = 4, .primitive = f.primitives->straight(4)});
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
    wire.push_back({.x = 150, .y = y, .heading = 4, .primitive = f.primitives->straight(4)});
  }
  f.router.buildOrthogonalConstraints({wire}, {false}, 6);
  const Path path = f.router.routeOrthogonal(ACROSS);
  ASSERT_FALSE(path.empty());
  // Every step onto a constrained cell is one the search's own test admits.
  for (std::size_t i = 1; i < path.size(); ++i) {
    EXPECT_TRUE(f.router.crossingAllowedOrthogonal(path[i].x, path[i].y, path[i - 1].heading))
        << i << " at " << path[i].x << "," << path[i].y;
  }
}

TEST(DubinsRouter, ASingleCrossingWireCannotComeBack) {
  Fixture f;
  // The feedline splits the grid; the wire has to cross it once.
  Path feedline;
  for (uint32_t y = 0; y < HEIGHT; ++y) {
    feedline.push_back({.x = 150, .y = y, .heading = 4, .primitive = f.primitives->straight(4)});
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
  Fixture f;
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
  const CellBox box = f.router.freeStripAlong({.x = 100, .y = 100, .heading = 6, .primitive = 0},
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
  f.router.setParams({.startStraightLength = 10, .endStraightLength = 10, .minRadius = 5,
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
}

TEST(DubinsRouter, ARouterOutlivesWhateverBuiltItsPrimitives) {
  // The prototype bound its primitives as a reference member to a caller's
  // stack local and worked around the dangling result in the callers. The
  // table is shared and owned instead, so a router that outlives the scope
  // it was built in still routes.
  SearchScratch scratch(WIDTH, HEIGHT);
  grid::BitGrid corridor(WIDTH, HEIGHT);
  auto router = [&] {
    auto primitives = std::make_shared<const MovePrimitives>(5);
    DubinsRouter built(primitives, scratch,
                       {.startStraightLength = 10, .endStraightLength = 10,
                        .minRadius = 5, .bendPenalty = 500});
    built.attachCorridor(&corridor);
    return built;
  }();
  EXPECT_EQ(router.primitives().minRadius(), 5U);
  EXPECT_FALSE(router.route(ACROSS).empty());
}

TEST(DubinsRouter, TheBendRadiusMustMatchThePrimitives) {
  auto primitives = std::make_shared<const MovePrimitives>(5);
  SearchScratch scratch(WIDTH, HEIGHT);
  const SearchParams wrong{.startStraightLength = 10, .endStraightLength = 10,
                           .minRadius = 8, .bendPenalty = 500};
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
  EXPECT_THROW(static_cast<void>(DubinsRouter(nullptr, scratch)), std::invalid_argument);
  // A grid whose states do not fit a search index is refused where it is
  // allocated, not later in a search.
  EXPECT_THROW(static_cast<void>(SearchScratch(40000, 40000)), std::invalid_argument);
}

} // namespace
