/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// The prototype's routing stress harness, as a test: a hundred routes over
// two grids of a thousand cells per side, each in its own cell of a lattice,
// with the source and the target facing opposite ways. The prototype ran it
// as a standalone program that wrote two pictures and printed the lengths;
// what it was actually checking is asserted here instead.

#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/routing/DubinsRouter.hpp"
#include "mqt-scpd/routing/Heading.hpp"
#include "mqt-scpd/routing/Path.hpp"
#include "mqt-scpd/routing/PathGeometry.hpp"
#include "mqt-scpd/routing/Primitives.hpp"
#include "mqt-scpd/routing/SearchScratch.hpp"
#include "mqt-scpd/routing/SelfIntersection.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace {

using namespace mqt::scpd;
using namespace mqt::scpd::routing;

constexpr uint32_t GRID_DIMENSION = 1000;
constexpr int TOTAL_ROUTES = 100;
constexpr int ROUTES_PER_GRID = 50;

/// One request of the harness: the cell of the lattice it lives in, and the
/// two ends inside that cell.
struct Request {
  RoutingObjective objective;
  uint32_t lowerX = 0;
  uint32_t lowerY = 0;
};

Request requestFor(const int t) {
  const int local = (t < ROUTES_PER_GRID) ? t : (t - ROUTES_PER_GRID);
  const uint32_t cellX = static_cast<uint32_t>(local % 5) * 200;
  const uint32_t cellY = static_cast<uint32_t>(local / 5) * 100;

  uint32_t startX = cellX + 30;
  uint32_t startY = cellY + 80;
  uint32_t endX = cellX + 170;
  uint32_t endY = cellY + 20;
  if (t % 4 == 1) {
    startX = cellX + 20;
    startY = cellY + 50;
    endX = cellX + 180;
    endY = cellY + 50;
  } else if (t % 4 == 2) {
    startX = cellX + 150;
    startY = cellY + 15;
    endX = cellX + 50;
    endY = cellY + 85;
  } else if (t % 4 == 3) {
    startX = cellX + 40;
    startY = cellY + 15;
    endX = cellX + 160;
    endY = cellY + 85;
  }
  const auto startHeading = static_cast<Heading>(t % 8);
  const auto endHeading = static_cast<Heading>((t + 4) % 8);
  return {.objective = {{.x = startX, .y = startY, .heading = startHeading, .primitive = 0},
                        {.x = endX, .y = endY, .heading = endHeading, .primitive = 0}},
          .lowerX = cellX,
          .lowerY = cellY};
}

TEST(RouteStress, AHundredRoutesOverTwoGrids) {
  auto primitives = std::make_shared<const MovePrimitives>(5);
  SearchScratch scratch(GRID_DIMENSION, GRID_DIMENSION);
  grid::BitGrid corridor(GRID_DIMENSION, GRID_DIMENSION);
  DubinsRouter router(primitives, scratch,
                      {.startStraightLength = 10,
                       .endStraightLength = 10,
                       .minRadius = 5,
                       .bendPenalty = 500});
  router.attachCorridor(&corridor);

  PathLoopScratch loopScratch;
  int routed = 0;
  for (int t = 0; t < TOTAL_ROUTES; ++t) {
    const Request request = requestFor(t);
    Path path = router.route(request.objective);
    ASSERT_FALSE(path.empty()) << "route " << t;
    ++routed;

    // The route runs from the source to the target, on their headings.
    EXPECT_EQ(path.front().x, request.objective.source.x) << t;
    EXPECT_EQ(path.front().y, request.objective.source.y) << t;
    EXPECT_EQ(path.front().heading, request.objective.source.heading) << t;
    EXPECT_EQ(path.back().x, request.objective.target.x) << t;
    EXPECT_EQ(path.back().y, request.objective.target.y) << t;
    EXPECT_EQ(path.back().heading, request.objective.target.heading) << t;

    // It stays on the grid and never crosses itself.
    for (const PathPoint& point : path) {
      ASSERT_LT(point.x, GRID_DIMENSION) << t;
      ASSERT_LT(point.y, GRID_DIMENSION) << t;
    }
    EXPECT_FALSE(pathSelfIntersects(path, GRID_DIMENSION, GRID_DIMENSION, loopScratch)) << t;

    // The length of the rendered curve agrees with the length the search
    // paid for. This is what the harness existed to check: the cost the
    // search adds up over its moves is the physical length of the wire, so
    // a stage that routes to a target length gets what it asked for.
    const SegmentedPath segmented = reconstructSegments(*primitives, path);
    std::vector<PathSegment> segments;
    Path copy = path;
    const std::vector<Point> samples = samplePath(*primitives, copy, copy.front(), segments);
    ASSERT_FALSE(samples.empty()) << t;
    const double sampled = polylineLength(samples);
    ASSERT_GT(segmented.nominalLength, 0.0) << t;
    EXPECT_NEAR(sampled / segmented.nominalLength, 1.0, 0.02) << t;
    // It is at least the straight line between the two ends.
    const double beeline = std::hypot(
        static_cast<double>(request.objective.target.x) - request.objective.source.x,
        static_cast<double>(request.objective.target.y) - request.objective.source.y);
    EXPECT_GE(sampled, beeline) << t;

    // The rendering ends on the target rather than drifting away from it.
    EXPECT_NEAR(samples.back().x(), static_cast<double>(request.objective.target.x), 1e-6) << t;
    EXPECT_NEAR(samples.back().y(), static_cast<double>(request.objective.target.y), 1e-6) << t;
  }
  EXPECT_EQ(routed, TOTAL_ROUTES);
  EXPECT_EQ(router.loopGuardRejections(), 0U);
}

TEST(RouteStress, EveryRouteIsTheSameOnASecondRun) {
  // The stage contracts promise a deterministic result, which is what makes
  // resume meaningful and a property test reproducible.
  auto primitives = std::make_shared<const MovePrimitives>(5);
  SearchScratch first(GRID_DIMENSION, GRID_DIMENSION);
  SearchScratch second(GRID_DIMENSION, GRID_DIMENSION);
  grid::BitGrid corridor(GRID_DIMENSION, GRID_DIMENSION);
  const SearchParams params{.startStraightLength = 10, .endStraightLength = 10,
                            .minRadius = 5, .bendPenalty = 500};
  DubinsRouter a(primitives, first, params);
  DubinsRouter b(primitives, second, params);
  a.attachCorridor(&corridor);
  b.attachCorridor(&corridor);

  for (int t = 0; t < 20; ++t) {
    const Request request = requestFor(t);
    EXPECT_EQ(a.route(request.objective), b.route(request.objective)) << t;
  }
}

TEST(RouteStress, ObstaclesInEveryCellStillLeaveAWayThrough) {
  auto primitives = std::make_shared<const MovePrimitives>(5);
  SearchScratch scratch(GRID_DIMENSION, GRID_DIMENSION);
  grid::BitGrid corridor(GRID_DIMENSION, GRID_DIMENSION);
  // A pillar in the middle of every cell of the lattice.
  for (uint32_t cellY = 0; cellY < GRID_DIMENSION; cellY += 100) {
    for (uint32_t cellX = 0; cellX < GRID_DIMENSION; cellX += 200) {
      for (uint32_t y = cellY + 35; y < cellY + 65; ++y) {
        for (uint32_t x = cellX + 90; x < cellX + 110; ++x) {
          corridor.setCell(x, y);
        }
      }
    }
  }
  DubinsRouter router(primitives, scratch,
                      {.startStraightLength = 10, .endStraightLength = 10,
                       .minRadius = 5, .bendPenalty = 500});
  router.attachCorridor(&corridor);

  PathLoopScratch loopScratch;
  for (int t = 0; t < TOTAL_ROUTES; ++t) {
    const Request request = requestFor(t);
    const Path path = router.route(request.objective);
    if (path.empty()) {
      continue;
    }
    for (const PathPoint& point : path) {
      EXPECT_FALSE(corridor.testCell(point.x, point.y)) << t;
    }
    EXPECT_FALSE(pathSelfIntersects(path, GRID_DIMENSION, GRID_DIMENSION, loopScratch)) << t;
  }
}

} // namespace
