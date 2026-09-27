/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// The analytic answer has one job: it must never exceed the turning of what
// the grid search comes back with. That is what makes it usable as the
// admissible bound of an exact method, and it is what these tests measure,
// over the same hundred requests the routing stress harness uses.

#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/routing/AnalyticDubins.hpp"
#include "mqt-scpd/routing/DubinsRouter.hpp"
#include "mqt-scpd/routing/Heading.hpp"
#include "mqt-scpd/routing/Path.hpp"
#include "mqt-scpd/routing/PathGeometry.hpp"
#include "mqt-scpd/routing/Primitives.hpp"
#include "mqt-scpd/routing/SearchScratch.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>

namespace {

using namespace mqt::scpd;
using namespace mqt::scpd::routing;

constexpr uint32_t GRID_DIMENSION = 1000;
constexpr int TOTAL_ROUTES = 100;
constexpr int ROUTES_PER_GRID = 50;

/// The stress harness's requests, as `test_route_stress.cpp` lays them out:
/// a lattice of cells, the two ends inside one cell, facing opposite ways.
RoutingObjective requestFor(const int t) {
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
  return {{.x = startX,
           .y = startY,
           .heading = static_cast<Heading>(t % 8),
           .primitive = 0},
          {.x = endX,
           .y = endY,
           .heading = static_cast<Heading>((t + 4) % 8),
           .primitive = 0}};
}

/// `Driver::angleCostOf` (`src/pipeline/FinalRouter.cpp:3383`), which is what
/// the coupler insertion prices a feedline edge by: the length of the walk on
/// the ring of headings the way makes, its arrival at the target counted.
uint32_t angleCostOf(const MovePrimitives& primitives, const Path& way,
                     const Heading target) {
  if (way.empty()) {
    return 0;
  }
  const auto segmented = reconstructSegments(primitives, way);
  uint32_t cost = 0;
  Heading last = way.front().heading;
  bool first = true;
  for (const auto& segment : segmented.segments) {
    if (!first) {
      cost += headingDistance(last, segment.heading);
    }
    first = false;
    last = segment.heading;
  }
  cost += headingDistance(last, target);
  return cost;
}

/// `Driver::turnBound` (`src/pipeline/FinalRouter.cpp:3428`), the bound the
/// trellis leans on today, so the two can be compared here.
uint32_t turnBound(const PathPoint& from, const PathPoint& to) {
  const auto dx = static_cast<int64_t>(to.x) - static_cast<int64_t>(from.x);
  const auto dy = static_cast<int64_t>(to.y) - static_cast<int64_t>(from.y);
  if (dx == 0 && dy == 0) {
    return headingDistance(from.heading, to.heading);
  }
  Heading beeline = 0;
  int64_t most = std::numeric_limits<int64_t>::min();
  for (Heading h = 0; h < NUM_HEADINGS; ++h) {
    const auto v = headingVector(h);
    const auto dot =
        (static_cast<int64_t>(v.dx) * dx) + (static_cast<int64_t>(v.dy) * dy);
    if (dot > most) {
      most = dot;
      beeline = h;
    }
  }
  auto best = NUM_HEADINGS;
  for (int eighth = -1; eighth <= 1; ++eighth) {
    const auto through = turned(beeline, eighth);
    const auto detour = headingDistance(from.heading, through) +
                        headingDistance(through, to.heading);
    best = detour < best ? detour : best;
  }
  return best;
}

/// The router the stress harness builds, with an optional pillar in the
/// middle of every cell of the lattice.
struct Fixture {
  std::shared_ptr<const MovePrimitives> primitives =
      std::make_shared<const MovePrimitives>(5);
  SearchScratch scratch{GRID_DIMENSION, GRID_DIMENSION};
  grid::BitGrid corridor{GRID_DIMENSION, GRID_DIMENSION};
  DubinsRouter router{primitives,
                      scratch,
                      {.startStraightLength = 10,
                       .endStraightLength = 10,
                       .minRadius = 5,
                       .bendPenalty = 500}};

  explicit Fixture(const bool pillars = false) {
    if (pillars) {
      for (uint32_t cellY = 0; cellY < GRID_DIMENSION; cellY += 100) {
        for (uint32_t cellX = 0; cellX < GRID_DIMENSION; cellX += 200) {
          for (uint32_t y = cellY + 35; y < cellY + 65; ++y) {
            for (uint32_t x = cellX + 90; x < cellX + 110; ++x) {
              corridor.setCell(x, y);
            }
          }
        }
      }
    }
    router.attachCorridor(&corridor);
  }
};

TEST(AnalyticDubins, NeverExceedsTheRouterOnAnEmptyGrid) {
  Fixture f;
  const AnalyticDubins analytic(*f.primitives);
  int compared = 0;
  for (int t = 0; t < TOTAL_ROUTES; ++t) {
    const RoutingObjective objective = requestFor(t);
    const Path way = f.router.route(objective);
    ASSERT_FALSE(way.empty()) << "route " << t;
    const auto real = angleCostOf(*f.primitives, way, objective.target.heading);
    EXPECT_LE(analytic.minTurns(objective.source, objective.target), real)
        << "route " << t << ": the bound is above what the router produced";
    ++compared;
  }
  EXPECT_EQ(compared, TOTAL_ROUTES);
}

TEST(AnalyticDubins, NeverExceedsTheRouterThroughObstacles) {
  // Obstacles can only make a way turn more, so the same bound must hold.
  Fixture f(true);
  const AnalyticDubins analytic(*f.primitives);
  for (int t = 0; t < TOTAL_ROUTES; ++t) {
    const RoutingObjective objective = requestFor(t);
    const Path way = f.router.route(objective);
    if (way.empty()) {
      continue;
    }
    const auto real = angleCostOf(*f.primitives, way, objective.target.heading);
    EXPECT_LE(analytic.minTurns(objective.source, objective.target), real)
        << "route " << t;
  }
}

TEST(AnalyticDubins, IsAtLeastAsSharpAsTheBoundItReplaces) {
  // The analytic answer is the exact least turning with nothing in the way,
  // so it cannot fall below any valid bound on the same thing. A pair where
  // it did would mean `turnBound` itself is not admissible.
  const MovePrimitives primitives(5);
  const AnalyticDubins analytic(primitives);
  int sharper = 0;
  int total = 0;
  for (int t = 0; t < TOTAL_ROUTES; ++t) {
    const RoutingObjective objective = requestFor(t);
    const auto bound = turnBound(objective.source, objective.target);
    const auto exact = analytic.minTurns(objective.source, objective.target);
    EXPECT_GE(exact, bound) << "route " << t;
    sharper += (exact > bound) ? 1 : 0;
    ++total;
  }
  std::cout << "[          ] sharper than turnBound on " << sharper << " of "
            << total << " requests\n";
}

TEST(AnalyticDubins, AnswersFarFasterThanASearch) {
  Fixture f;
  const AnalyticDubins analytic(*f.primitives);

  auto started = std::chrono::steady_clock::now();
  uint64_t sink = 0;
  for (int t = 0; t < TOTAL_ROUTES; ++t) {
    const RoutingObjective objective = requestFor(t);
    sink += analytic.minTurns(objective.source, objective.target);
  }
  const auto analytically =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now() - started)
          .count();

  started = std::chrono::steady_clock::now();
  for (int t = 0; t < TOTAL_ROUTES; ++t) {
    sink += f.router.route(requestFor(t)).size();
  }
  const auto searched = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now() - started)
                            .count();

  const double analyticUs =
      static_cast<double>(analytically) / 1000.0 / TOTAL_ROUTES;
  const double searchUs = static_cast<double>(searched) / 1000.0 / TOTAL_ROUTES;
  std::cout << "[          ] analytic " << analyticUs << " us/answer, search "
            << searchUs << " us/route, " << (searchUs / analyticUs) << "x\n";
  EXPECT_GT(searchUs, 10.0 * analyticUs);
  EXPECT_GT(sink, 0U);
}

TEST(AnalyticDubins, AStraightRunTurnsNothing) {
  const MovePrimitives primitives(5);
  const AnalyticDubins analytic(primitives);
  for (Heading h = 0; h < NUM_HEADINGS; ++h) {
    const auto v = headingVector(h);
    const PathPoint from{.x = 500, .y = 500, .heading = h, .primitive = 0};
    const PathPoint to{.x = static_cast<uint32_t>(500 + (v.dx * 100)),
                       .y = static_cast<uint32_t>(500 + (v.dy * 100)),
                       .heading = h,
                       .primitive = 0};
    EXPECT_EQ(analytic.minTurns(from, to), 0U) << static_cast<int>(h);
  }
}

TEST(AnalyticDubins, APoseBehindItselfCannotBeReachedWithoutTurning) {
  // The target sits back along the source's own heading, facing the same
  // way: no run of straights reaches it, so the way has to turn all the way
  // around and back.
  const MovePrimitives primitives(5);
  const AnalyticDubins analytic(primitives);
  const PathPoint from{.x = 500, .y = 500, .heading = 0, .primitive = 0};
  const PathPoint to{.x = 500, .y = 600, .heading = 0, .primitive = 0};
  EXPECT_GE(analytic.minTurns(from, to), 2U);
}

TEST(AnalyticDubins, TheCapIsStillALowerBound) {
  // Nothing was feasible within the cap, so the truth is above it.
  const MovePrimitives primitives(5);
  const AnalyticDubins tight(primitives, 0);
  const PathPoint from{.x = 500, .y = 500, .heading = 0, .primitive = 0};
  const PathPoint to{.x = 500, .y = 600, .heading = 0, .primitive = 0};
  EXPECT_EQ(tight.minTurns(from, to), 1U);
}

} // namespace
