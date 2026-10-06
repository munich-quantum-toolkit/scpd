/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// The routing stress test: a hundred routes over one grid of a thousand cells
// per side, two in each cell of a lattice of fifty cells, with the source and
// the target facing opposite ways. Every route must exist, end exactly on its
// target, not cross itself, render to the length the search paid for, and
// take a coupler that meets its target length.

#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/routing/CouplerInsertion.hpp"
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
#include <memory>
#include <utility>
#include <vector>

namespace {

using namespace mqt::scpd;
using namespace mqt::scpd::routing;

constexpr uint32_t GRID_DIMENSION = 1000;
constexpr int TOTAL_ROUTES = 100;
constexpr int LATTICE_CELLS = 50;

/// A move of a routed path: the heading it leaves and its primitive.
struct Move {
  Heading heading = 0;
  const Primitive* primitive = nullptr;
};

/// The moves of a routed path, read back from the tags of its points, from
/// the target to the source. The point before each state carries the move
/// that reached the state: the last cell a turn sweeps, or the state a
/// straight step leaves.
std::vector<Move> movesOf(const MovePrimitives& primitives, const Path& path) {
  std::vector<Move> moves;
  std::size_t at = path.size() - 1;
  int64_t x = path.back().x;
  int64_t y = path.back().y;
  Heading heading = path.back().heading;
  while (at > 0) {
    const PathPoint& tag = path[at - 1];
    const Primitive* move = primitives.find(tag.heading, tag.primitive);
    if (move == nullptr || move->exitHeading != heading) {
      return {};
    }
    moves.push_back({.heading = tag.heading, .primitive = move});
    x -= move->dx;
    y -= move->dy;
    heading = tag.heading;
    while (at > 0 && (std::cmp_not_equal(path[at - 1].x, x) ||
                      std::cmp_not_equal(path[at - 1].y, y))) {
      --at;
    }
    if (at == 0) {
      return {};
    }
    --at;
  }
  return moves;
}

/// The request with number @p t: two ends inside one cell of the lattice.
/// Requests @p t and @p t + LATTICE_CELLS share a cell and differ in their
/// headings.
RoutingObjective requestFor(const int t) {
  const int local = t % LATTICE_CELLS;
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
  return {
      .source = {.x = startX,
                 .y = startY,
                 .heading = startHeading,
                 .primitive = 0},
      .target = {.x = endX, .y = endY, .heading = endHeading, .primitive = 0}};
}

TEST(RouteStress, AHundredRoutesOverOneGrid) {
  auto primitives = std::make_shared<const MovePrimitives>(5);
  SearchScratch scratch(GRID_DIMENSION, GRID_DIMENSION);
  const grid::BitGrid outsideCorridor(GRID_DIMENSION, GRID_DIMENSION);
  DubinsRouter router(primitives, scratch,
                      {.startStraightLength = 10,
                       .endStraightLength = 10,
                       .minRadius = 5,
                       .bendPenalty = 500});
  router.attachCorridor(&outsideCorridor);

  // The largest amount by which the cost of a turn exceeds the length of its
  // curve. At a radius of five cells, it belongs to the eighth turns that
  // leave a diagonal heading (see Primitive::cost).
  double largestExcess = 0.0;
  for (Heading heading = 0; heading < NUM_HEADINGS; ++heading) {
    for (const Primitive& p : primitives->of(heading)) {
      if (p.exitHeading != heading) {
        largestExcess =
            std::max(largestExcess, p.cost - polylineLength(p.samples));
      }
    }
  }

  PathLoopScratch loopScratch;
  int routed = 0;
  for (int t = 0; t < TOTAL_ROUTES; ++t) {
    const RoutingObjective request = requestFor(t);
    Path path = router.route(request);
    ASSERT_FALSE(path.empty()) << "route " << t;
    ++routed;

    // The route runs from the source to the target, on their headings.
    EXPECT_EQ(path.front().x, request.source.x) << t;
    EXPECT_EQ(path.front().y, request.source.y) << t;
    EXPECT_EQ(path.front().heading, request.source.heading) << t;
    EXPECT_EQ(path.back().x, request.target.x) << t;
    EXPECT_EQ(path.back().y, request.target.y) << t;
    EXPECT_EQ(path.back().heading, request.target.heading) << t;

    // It stays on the grid and never crosses itself.
    for (const PathPoint& point : path) {
      ASSERT_LT(point.x, GRID_DIMENSION) << t;
      ASSERT_LT(point.y, GRID_DIMENSION) << t;
    }
    EXPECT_FALSE(
        pathSelfIntersects(path, GRID_DIMENSION, GRID_DIMENSION, loopScratch))
        << t;

    // The length of the rendered curve agrees with the length the search
    // paid for. The rendering draws every move once, along its own curve, so
    // its length is the sum of the curve lengths of the moves; rounding at
    // the ends of the curves leaves a tenth of a cell. A move costs at least
    // the length of its curve, and a turn at most the largest excess more.
    std::vector<PathSegment> segments;
    Path copy = path;
    const std::vector<Point> samples =
        samplePath(*primitives, copy, copy.front(), segments);
    ASSERT_FALSE(samples.empty()) << t;
    const double sampled = polylineLength(samples);
    const std::vector<Move> moves = movesOf(*primitives, path);
    ASSERT_FALSE(moves.empty()) << t;
    double curves = 0.0;
    double paid = 0.0;
    double turns = 0.0;
    for (const Move& move : moves) {
      curves += polylineLength(move.primitive->samples);
      paid += move.primitive->cost;
      turns += move.primitive->exitHeading != move.heading ? 1.0 : 0.0;
    }
    EXPECT_NEAR(sampled, curves, 0.15) << t;
    EXPECT_GE(paid, curves - 0.01) << t;
    EXPECT_LE(paid, curves + (turns * largestExcess) + 0.01) << t;
    // It is at least the straight line between the two ends.
    const double beeline =
        std::hypot(static_cast<double>(request.target.x) - request.source.x,
                   static_cast<double>(request.target.y) - request.source.y);
    EXPECT_GE(sampled, beeline) << t;

    // The rendering ends on the target rather than drifting away from it.
    EXPECT_NEAR(samples.back().x(), static_cast<double>(request.target.x), 1e-6)
        << t;
    EXPECT_NEAR(samples.back().y(), static_cast<double>(request.target.y), 1e-6)
        << t;

    // The coupler splice measures lengths as samplePath() renders them. Its
    // dogleg ends on the middle cell of the longest straight run, if the
    // target is the rendered length from that cell to the end plus the
    // length of the dogleg. The spliced path then renders to the target.
    const auto longest = std::ranges::max_element(
        segments, {}, [](const PathSegment& s) { return s.steps(); });
    ASSERT_TRUE(longest->straight()) << t;
    const double fromMiddle = sampled - longest->lengthAt[longest->steps() / 2];
    const Heading couplerHeading = longest->heading;
    const double target =
        fromMiddle +
        buildDogleg(*primitives, turned(couplerHeading, 2), -1, 14).cost;
    Path spliced = path;
    ASSERT_TRUE(spliceCouplerDogleg(*primitives, target, spliced,
                                    GRID_DIMENSION, GRID_DIMENSION,
                                    couplerHeading)
                    .has_value())
        << t;
    EXPECT_NEAR(renderedLength(*primitives, spliced), target, 1e-6) << t;
  }
  EXPECT_EQ(routed, TOTAL_ROUTES);
  EXPECT_EQ(router.loopGuardRejections(), 0U);
}

TEST(RouteStress, EarlierSearchesDoNotChangeARoute) {
  // A search keeps nothing from the searches before it: a router that has
  // routed other requests in its scratch returns the path that a new router
  // with a new scratch returns.
  auto primitives = std::make_shared<const MovePrimitives>(5);
  SearchScratch shared(GRID_DIMENSION, GRID_DIMENSION);
  const grid::BitGrid outsideCorridor(GRID_DIMENSION, GRID_DIMENSION);
  const SearchParams params{.startStraightLength = 10,
                            .endStraightLength = 10,
                            .minRadius = 5,
                            .bendPenalty = 500};
  DubinsRouter used(primitives, shared, params);
  used.attachCorridor(&outsideCorridor);

  constexpr int requests = 20;
  std::vector<Path> fresh;
  for (int t = 0; t < requests; ++t) {
    SearchScratch scratch(GRID_DIMENSION, GRID_DIMENSION);
    DubinsRouter router(primitives, scratch, params);
    router.attachCorridor(&outsideCorridor);
    fresh.push_back(router.route(requestFor(t)));
    ASSERT_FALSE(fresh.back().empty()) << t;
  }
  // Forward and then backward, so that every request also runs after the
  // searches that follow it in the forward order.
  for (int t = 0; t < requests; ++t) {
    EXPECT_EQ(used.route(requestFor(t)), fresh[static_cast<std::size_t>(t)])
        << t;
  }
  for (int t = requests - 1; t >= 0; --t) {
    EXPECT_EQ(used.route(requestFor(t)), fresh[static_cast<std::size_t>(t)])
        << t;
  }
}

TEST(RouteStress, ObstaclesInEveryCellStillLeaveAPathThrough) {
  auto primitives = std::make_shared<const MovePrimitives>(5);
  SearchScratch scratch(GRID_DIMENSION, GRID_DIMENSION);
  grid::BitGrid outsideCorridor(GRID_DIMENSION, GRID_DIMENSION);
  // A pillar in the middle of every cell of the lattice.
  for (uint32_t cellY = 0; cellY < GRID_DIMENSION; cellY += 100) {
    for (uint32_t cellX = 0; cellX < GRID_DIMENSION; cellX += 200) {
      for (uint32_t y = cellY + 35; y < cellY + 65; ++y) {
        for (uint32_t x = cellX + 90; x < cellX + 110; ++x) {
          outsideCorridor.setCell(x, y);
        }
      }
    }
  }
  DubinsRouter router(primitives, scratch,
                      {.startStraightLength = 10,
                       .endStraightLength = 10,
                       .minRadius = 5,
                       .bendPenalty = 500});
  router.attachCorridor(&outsideCorridor);

  PathLoopScratch loopScratch;
  for (int t = 0; t < TOTAL_ROUTES; ++t) {
    const Path path = router.route(requestFor(t));
    ASSERT_FALSE(path.empty()) << "route " << t;
    for (const PathPoint& point : path) {
      EXPECT_FALSE(outsideCorridor.testCell(point.x, point.y)) << t;
    }
    EXPECT_FALSE(
        pathSelfIntersects(path, GRID_DIMENSION, GRID_DIMENSION, loopScratch))
        << t;
  }
}

} // namespace
