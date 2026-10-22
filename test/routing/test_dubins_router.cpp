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
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <random>
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
  std::vector<uint16_t> wire =
      std::vector<uint16_t>(static_cast<std::size_t>(WIDTH) * HEIGHT, 0);
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

/// A router that packs one corridor after another finds, for the last of
/// them, the way a search that reads the corridor unpacked finds, whether
/// or not the static proximity changed between the two.
TEST(DubinsRouter, ACorridorPackedAgainRoutesAsAnUnpackedOne) {
  for (const bool newProximity : {false, true}) {
    Fixture reused;
    reused.block(100, 60, 110, HEIGHT - 1);
    ASSERT_FALSE(reused.router.route(ACROSS, true).empty());
    for (uint32_t y = 60; y < HEIGHT; ++y) {
      for (uint32_t x = 100; x <= 110; ++x) {
        reused.corridor.setCell(x, y, false);
        reused.obstacles.setCell(x, y, false);
      }
    }
    reused.block(180, 0, 190, 140);
    Fixture fresh;
    fresh.block(180, 0, 190, 140);
    if (newProximity) {
      reused.router.computeStaticProximity(4, 20);
      fresh.router.computeStaticProximity(4, 20);
    }
    fresh.router.attachCorridorUnpacked(&fresh.corridor);
    const Path again = reused.router.route(ACROSS, true);
    ASSERT_FALSE(again.empty()) << newProximity;
    EXPECT_EQ(again, fresh.router.route(ACROSS, true)) << newProximity;
  }
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
  const std::vector<uint16_t> small(100, 0);
  EXPECT_THROW(f.router.attachWireProximity(&small), std::invalid_argument);
  EXPECT_THROW(f.router.setStaticProximity(std::vector<uint8_t>(100, 0)),
               std::invalid_argument);
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

/// Whether every cell of a path passes the crossing rule **on the heading
/// the cell carries** — the reading the final router's count and the
/// design-rule check make of a committed path.
std::size_t cellsAgainstTheRule(const DubinsRouter& router, const Path& path) {
  std::size_t against = 0;
  for (const PathPoint& point : path) {
    against += router.crossingAllowedOrthogonal(point.x, point.y, point.heading) ? 0 : 1;
  }
  return against;
}

TEST(DubinsRouter, TheExitHeadingCheckMakesTheSearchAgreeWithTheCount) {
  // The same wire as above, and routes that have to cross it and turn
  // north soon after. The search tests the cells a move sweeps with the
  // heading the move entered on, while `reconstruct` records a move's end
  // cell with the heading it leaves on — so a route that crosses straight
  // and begins its turn on the last cell of the halo passes the search and
  // fails the count that reads the path (12 of the 22 crossing wires of the
  // 2026-10-04 baseline). With the exit heading tested as well, the search
  // refuses what the count refuses, and still finds a way.
  Path wire;
  for (uint32_t y = 20; y < 180; ++y) {
    wire.push_back({.x = 150, .y = y, .heading = 4, .primitive = 0});
  }
  std::size_t violationsWithout = 0;
  std::size_t routedWith = 0;
  std::size_t routedWithout = 0;
  for (const uint32_t targetX : {158U, 160U, 163U, 166U, 170U}) {
    for (const uint32_t targetY : {60U, 70U, 80U, 120U, 130U, 140U}) {
      for (const Heading heading : {Heading{4}, Heading{0}, Heading{6}}) {
        const RoutingObjective objective{
            {.x = 30, .y = 100, .heading = 6, .primitive = 0},
            {.x = targetX, .y = targetY, .heading = heading, .primitive = 0}};
        Fixture plain;
        plain.router.buildOrthogonalConstraints({wire}, {false}, 6);
        const Path without = plain.router.routeOrthogonal(objective);
        if (!without.empty()) {
          ++routedWithout;
          violationsWithout += cellsAgainstTheRule(plain.router, without) == 0 ? 0 : 1;
        }
        Fixture strict;
        strict.router.buildOrthogonalConstraints({wire}, {false}, 6);
        strict.router.setOrthogonalExitCheck(true);
        const Path with = strict.router.routeOrthogonal(objective);
        if (!with.empty()) {
          ++routedWith;
          // What the search returns under the check is what the count
          // accepts, cell by cell.
          EXPECT_EQ(cellsAgainstTheRule(strict.router, with), 0U)
              << "target " << targetX << "," << targetY << " heading " << heading;
        }
      }
    }
  }
  // The check costs no route: every objective the plain search joins, the
  // strict one joins too.
  EXPECT_EQ(routedWith, routedWithout);
  EXPECT_GT(routedWith, 0U);
  // And the defect is real: without the check, some of these routes break
  // the rule the count reads.
  EXPECT_GT(violationsWithout, 0U);
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

/// The targets of the multi-target tests: ports on every side of a wall,
/// arriving on several headings.
std::vector<PathPoint> multiTargets() {
  return {{.x = 260, .y = 100, .heading = 6, .primitive = 0},
          {.x = 250, .y = 40, .heading = 6, .primitive = 0},
          {.x = 220, .y = 170, .heading = 4, .primitive = 0},
          {.x = 200, .y = 30, .heading = 0, .primitive = 0},
          {.x = 270, .y = 160, .heading = 7, .primitive = 0},
          {.x = 180, .y = 120, .heading = 2, .primitive = 0}};
}

/// One search to every target costs, per target, what a search to that
/// target alone costs, and finds the same verdict, in the same corridor.
TEST(DubinsRouter, AMultiTargetSearchCostsWhatEachSingleSearchCosts) {
  for (const bool usePenalty : {false, true}) {
    for (const Heuristic heuristic :
         {Heuristic::DistanceField, Heuristic::Octile}) {
      Fixture f;
      f.block(120, 50, 130, 150);
      f.block(160, 0, 170, 70);
      f.router.setHeuristic(heuristic);
      const auto targets = multiTargets();
      std::vector<DubinsRouter::MultiTarget> asked;
      for (const PathPoint& target : targets) {
        asked.push_back({.target = target, .own = nullptr});
      }
      const auto multi = f.router.routeMulti(ACROSS.source, asked, usePenalty);
      ASSERT_EQ(multi.size(), targets.size());
      std::size_t found = 0;
      for (std::size_t k = 0; k < targets.size(); ++k) {
        const Path single = f.router.route(
            {.source = ACROSS.source, .target = targets[k]}, usePenalty);
        if (single.empty()) {
          EXPECT_NE(multi[k].outcome, DubinsRouter::MultiRoute::Outcome::Found)
              << k;
          continue;
        }
        ++found;
        ASSERT_EQ(multi[k].outcome, DubinsRouter::MultiRoute::Outcome::Found)
            << k;
        EXPECT_EQ(multi[k].cost, f.router.lastSearchCost()) << k;
        EXPECT_EQ(multi[k].way.front(), single.front()) << k;
        EXPECT_EQ(multi[k].way.back(), single.back()) << k;
        EXPECT_TRUE(avoids(multi[k].way, f.corridor)) << k;
      }
      EXPECT_GE(found, 5U);
    }
  }
}

/// A search to one target is the single search: the same way, cost and
/// expansions.
TEST(DubinsRouter, AMultiTargetSearchToOneTargetIsTheSingleSearch) {
  Fixture f;
  f.block(120, 50, 130, 150);
  const auto target = multiTargets()[2];
  const Path single =
      f.router.route({.source = ACROSS.source, .target = target}, true);
  const auto cost = f.router.lastSearchCost();
  const auto expansions = f.router.lastSearchExpansions();
  ASSERT_FALSE(single.empty());
  EXPECT_GT(expansions, 0U);
  const auto multi = f.router.routeMulti(
      ACROSS.source, {{.target = target, .own = nullptr}}, true);
  ASSERT_EQ(multi.size(), 1U);
  EXPECT_EQ(multi[0].outcome, DubinsRouter::MultiRoute::Outcome::Found);
  EXPECT_EQ(multi[0].way, single);
  EXPECT_EQ(multi[0].cost, cost);
  EXPECT_EQ(f.router.lastSearchExpansions(), expansions);
}

/// With the union of the targets' corridors attached, a target whose way
/// stays in its own corridor costs what a search in that corridor alone
/// costs, and any target costs no more than that.
TEST(DubinsRouter, AMultiTargetSearchInTheUnionCostsNoMoreThanEachOwnCorridor) {
  Fixture f;
  const auto targets = multiTargets();
  // Each target's corridor closes a block of its own on the straight way.
  std::vector<grid::BitGrid> own(targets.size(), grid::BitGrid{WIDTH, HEIGHT});
  for (std::size_t k = 0; k < targets.size(); ++k) {
    const auto x0 = static_cast<uint32_t>(100 + (20 * k));
    for (uint32_t y = 60; y <= 140; ++y) {
      for (uint32_t x = x0; x <= x0 + 8; ++x) {
        own[k].setCell(x, y);
      }
    }
  }
  // The union of the open space: a cell is closed only where every target's
  // corridor closes it, which here is nowhere.
  std::vector<DubinsRouter::MultiTarget> asked;
  for (std::size_t k = 0; k < targets.size(); ++k) {
    asked.push_back({.target = targets[k], .own = &own[k]});
  }
  const auto multi = f.router.routeMulti(ACROSS.source, asked, true);
  std::size_t kept = 0;
  for (std::size_t k = 0; k < targets.size(); ++k) {
    f.router.attachCorridor(&own[k]);
    const Path single =
        f.router.route({.source = ACROSS.source, .target = targets[k]}, true);
    const auto cost = f.router.lastSearchCost();
    f.router.attachCorridor(&f.corridor);
    ASSERT_FALSE(single.empty()) << k;
    const auto outcome = multi[k].outcome;
    ASSERT_TRUE(outcome == DubinsRouter::MultiRoute::Outcome::Found ||
                outcome == DubinsRouter::MultiRoute::Outcome::LeftOwnCorridor)
        << k;
    EXPECT_LE(multi[k].cost, cost) << k;
    if (outcome == DubinsRouter::MultiRoute::Outcome::Found) {
      ++kept;
      EXPECT_EQ(multi[k].cost, cost) << k;
      EXPECT_TRUE(avoids(multi[k].way, own[k])) << k;
    } else {
      EXPECT_FALSE(avoids(multi[k].way, own[k])) << k;
    }
  }
  // The first target's block lies across its straight way; the way through
  // the union runs through it.
  EXPECT_EQ(multi[0].outcome,
            DubinsRouter::MultiRoute::Outcome::LeftOwnCorridor);
  EXPECT_GE(kept, 1U);
}

/// A target no way reaches does not keep the search from the others, and a
/// turn bound none of a target's ways fits is said as such.
TEST(DubinsRouter, AMultiTargetSearchSettlesEveryTargetOnItsOwn) {
  Fixture f;
  // A box closed on every side around one target.
  f.block(200, 140, 260, 141);
  f.block(200, 190, 260, 191);
  f.block(200, 140, 201, 191);
  f.block(259, 140, 260, 191);
  std::vector<DubinsRouter::MultiTarget> asked{
      {.target = {.x = 240, .y = 165, .heading = 6, .primitive = 0},
       .own = nullptr},
      {.target = ACROSS.target, .own = nullptr},
      {.target = {.x = 150, .y = 30, .heading = 0, .primitive = 0},
       .own = nullptr}};
  const auto multi = f.router.routeMulti(ACROSS.source, asked);
  EXPECT_EQ(multi[0].outcome, DubinsRouter::MultiRoute::Outcome::NoWay);
  ASSERT_EQ(multi[1].outcome, DubinsRouter::MultiRoute::Outcome::Found);
  EXPECT_EQ(countBends(multi[1].way), 0U);
  EXPECT_EQ(multi[1].turns, 0U);
  ASSERT_EQ(multi[2].outcome, DubinsRouter::MultiRoute::Outcome::Found);
  EXPECT_GT(multi[2].turns, 0U);

  f.router.setMaxTurns(0);
  const auto straight = f.router.routeMulti(ACROSS.source, asked);
  f.router.setMaxTurns(-1);
  EXPECT_EQ(straight[1].outcome, DubinsRouter::MultiRoute::Outcome::Found);
  EXPECT_EQ(straight[2].outcome, DubinsRouter::MultiRoute::Outcome::CutOff);
}

/// A search resumed from target to target costs, for every target and in
/// every order of the targets, what a fresh search to that target alone
/// costs in the same corridor, under `order` and `heuristic`.
void resumedCostsAsFresh(const Heuristic heuristic,
                         const DubinsRouter::ResumeOrder order) {
  Fixture f;
  f.block(120, 50, 130, 150);
  f.block(160, 0, 170, 70);
  f.router.setHeuristic(heuristic);
  f.router.setResumeOrder(order);
  const auto all = multiTargets();
  const std::vector<PathPoint> targets{all[0], all[2], all[3], all[5]};
  std::vector<uint32_t> fresh;
  for (const PathPoint& target : targets) {
    ASSERT_FALSE(
        f.router.route({.source = ACROSS.source, .target = target}, true)
            .empty());
    fresh.push_back(f.router.lastSearchCost());
  }
  std::vector<std::size_t> sequence{0, 1, 2, 3};
  std::size_t sequences = 0;
  do {
    DubinsRouter::PausedSearch search;
    uint64_t resumed = 0;
    for (const std::size_t k : sequence) {
      const auto answer = f.router.routeResumable(
          search, ACROSS.source, targets[k], nullptr, true, targets);
      ASSERT_EQ(answer.outcome, DubinsRouter::MultiRoute::Outcome::Found) << k;
      EXPECT_EQ(answer.cost, fresh[k]) << k;
      EXPECT_TRUE(answer.way.front().samePlace(ACROSS.source)) << k;
      EXPECT_TRUE(answer.way.back().samePlace(targets[k])) << k;
      EXPECT_TRUE(avoids(answer.way, f.corridor)) << k;
      resumed += f.router.lastSearchExpansions();
    }
    EXPECT_TRUE(search.started());
    EXPECT_GT(search.states(), 0U);
    EXPECT_GT(resumed, 0U);
    ++sequences;
  } while (std::ranges::next_permutation(sequence).found);
  EXPECT_EQ(sequences, 24U);
}

TEST(DubinsRouter, AResumedSearchCostsWhatAFreshSearchCostsInAnyOrder) {
  for (const auto heuristic : {Heuristic::DistanceField, Heuristic::Octile}) {
    for (const auto order :
         {DubinsRouter::ResumeOrder::Target, DubinsRouter::ResumeOrder::Minimum,
          DubinsRouter::ResumeOrder::Offset}) {
      SCOPED_TRACE(static_cast<int>(order));
      SCOPED_TRACE(static_cast<int>(heuristic));
      resumedCostsAsFresh(heuristic, order);
    }
  }
}

/// A target whose goal state an earlier call closed on its way is answered
/// without a search, and a target no way reaches leaves the search able to
/// answer the next one.
TEST(DubinsRouter, AResumedSearchAnswersATargetPassedOnTheWay) {
  Fixture f;
  // The far target first: the straight way to it runs through the goal
  // state of the near one.
  const PathPoint near{.x = 150, .y = 100, .heading = 6, .primitive = 0};
  DubinsRouter::PausedSearch search;
  ASSERT_EQ(
      f.router.routeResumable(search, ACROSS.source, ACROSS.target, nullptr)
          .outcome,
      DubinsRouter::MultiRoute::Outcome::Found);
  const auto passed =
      f.router.routeResumable(search, ACROSS.source, near, nullptr);
  ASSERT_EQ(passed.outcome, DubinsRouter::MultiRoute::Outcome::Found);
  EXPECT_EQ(f.router.lastSearchExpansions(), 0U);
  EXPECT_EQ(passed.turns, 0U);
  ASSERT_FALSE(
      f.router.route({.source = ACROSS.source, .target = near}).empty());
  EXPECT_EQ(passed.cost, f.router.lastSearchCost());

  // A target shut in a box, then one in the open.
  Fixture g;
  g.block(200, 140, 260, 141);
  g.block(200, 190, 260, 191);
  g.block(200, 140, 201, 191);
  g.block(259, 140, 260, 191);
  DubinsRouter::PausedSearch again;
  EXPECT_EQ(g.router
                .routeResumable(
                    again, ACROSS.source,
                    {.x = 240, .y = 165, .heading = 6, .primitive = 0}, nullptr)
                .outcome,
            DubinsRouter::MultiRoute::Outcome::NoWay);
  const auto open =
      g.router.routeResumable(again, ACROSS.source, ACROSS.target, nullptr);
  ASSERT_EQ(open.outcome, DubinsRouter::MultiRoute::Outcome::Found);
  EXPECT_EQ(countBends(open.way), 0U);
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

/// A memo shared by many strips gives every strip the box the search
/// without it gives, on segments along either axis, in either direction,
/// and on the diagonal.
TEST(DubinsRouter, TheFreeStripWithAMemoIsTheFreeStripWithoutOne) {
  Fixture f;
  std::mt19937 random(7);
  std::uniform_int_distribution<uint32_t> anyX(0, WIDTH - 1);
  std::uniform_int_distribution<uint32_t> anyY(0, HEIGHT - 1);
  for (int obstacle = 0; obstacle < 400; ++obstacle) {
    f.obstacles.setCell(anyX(random), anyY(random));
  }
  f.router.attachObstacles(&f.obstacles);
  DubinsRouter::FreeStripMemo memo;
  std::uniform_int_distribution<uint32_t> length(0, 80);
  std::uniform_int_distribution<int> direction(0, 5);
  for (int segment = 0; segment < 600; ++segment) {
    const uint32_t x = anyX(random);
    const uint32_t y = anyY(random);
    const uint32_t run = length(random);
    uint32_t toX = x;
    uint32_t toY = y;
    switch (direction(random)) {
    case 0:
      toX = std::min(WIDTH - 1, x + run);
      break;
    case 1:
      toX = x > run ? x - run : 0;
      break;
    case 2:
      toY = std::min(HEIGHT - 1, y + run);
      break;
    case 3:
      toY = y > run ? y - run : 0;
      break;
    default:
      toX = std::min(WIDTH - 1, x + run);
      toY = std::min(HEIGHT - 1, y + run);
      break;
    }
    const PathPoint from{.x = x, .y = y, .heading = 0, .primitive = 0};
    const PathPoint to{.x = toX, .y = toY, .heading = 0, .primitive = 0};
    const CellBox plain = f.router.freeStripAlong(from, to);
    const CellBox remembered = f.router.freeStripAlong(from, to, memo);
    EXPECT_EQ(remembered.minX, plain.minX) << segment;
    EXPECT_EQ(remembered.maxX, plain.maxX) << segment;
    EXPECT_EQ(remembered.minY, plain.minY) << segment;
    EXPECT_EQ(remembered.maxY, plain.maxY) << segment;
  }
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

/// The eighth turns of a way: the heading distance summed over its points.
uint32_t eighthTurns(const Path& path) {
  uint32_t turns = 0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    turns += headingDistance(path[i - 1].heading, path[i].heading);
  }
  return turns;
}

/// One case of the turn bound tests: walls on the grid and a source and a
/// target on either side.
struct TurnCase {
  std::vector<std::array<uint32_t, 4>> walls;
  RoutingObjective objective;
};

/// Cases where the cheapest way turns, and where a tight turn bound leaves
/// a way, a cut-off or both: a wall with one gap, a winding passage, a
/// target that faces back toward the source, and random blocks.
std::vector<TurnCase> turnCases() {
  std::vector<TurnCase> cases;
  const RoutingObjective back{
      {.x = 30, .y = 100, .heading = 6, .primitive = 0},
      {.x = 200, .y = 60, .heading = 2, .primitive = 0}};
  const RoutingObjective up{{.x = 40, .y = 170, .heading = 6, .primitive = 0},
                            {.x = 250, .y = 40, .heading = 0, .primitive = 0}};
  cases.push_back({{{140, 0, 150, 80}, {140, 120, 150, HEIGHT - 1}}, ACROSS});
  cases.push_back(
      {{{80, 0, 90, 150}, {150, 50, 160, HEIGHT - 1}, {220, 0, 230, 150}},
       ACROSS});
  cases.push_back({{{120, 40, 130, 160}}, back});
  cases.push_back({{{100, 100, 200, 110}, {150, 0, 160, 60}}, up});
  std::mt19937 rng(7);
  std::uniform_int_distribution<uint32_t> x(60, 230);
  std::uniform_int_distribution<uint32_t> y(10, 180);
  std::uniform_int_distribution<uint32_t> side(4, 30);
  for (int layout = 0; layout < 4; ++layout) {
    TurnCase random{{}, layout % 2 == 0 ? ACROSS : up};
    for (int k = 0; k < 8; ++k) {
      const uint32_t x0 = x(rng);
      const uint32_t y0 = y(rng);
      random.walls.push_back({x0, y0, std::min(x0 + side(rng), 240U),
                              std::min(y0 + side(rng), HEIGHT - 1)});
    }
    cases.push_back(random);
  }
  return cases;
}

/// Each heading has one move that does not turn, the straight step along
/// it. The turn bound treats that move as a straight run of its model, so
/// the bound needs it to be one.
TEST(DubinsRouter, TheOnlyMoveThatDoesNotTurnIsTheStraightStep) {
  const MovePrimitives primitives(5);
  for (Heading heading = 0; heading < NUM_HEADINGS; ++heading) {
    std::size_t straight = 0;
    const HeadingVector v = headingVector(heading);
    for (const Primitive& p : primitives.of(heading)) {
      if (p.exitHeading != heading) {
        continue;
      }
      ++straight;
      EXPECT_EQ((v.dx * p.dy) - (v.dy * p.dx), 0) << int{heading};
      EXPECT_GT((v.dx * p.dx) + (v.dy * p.dy), 0) << int{heading};
    }
    EXPECT_EQ(straight, 1U) << int{heading};
  }
}

/// With the turn bound, a search finds a way wherever the search without it
/// does, at the same price, and never returns a way over the turn bound.
TEST(DubinsRouter, TheTurnBoundKeepsEveryWayAndItsPrice) {
  std::size_t found = 0;
  std::size_t cutOff = 0;
  for (const TurnCase& one : turnCases()) {
    for (const Heuristic heuristic :
         {Heuristic::DistanceField, Heuristic::Octile}) {
      for (const bool usePenalty : {false, true}) {
        Fixture f;
        for (const auto& wall : one.walls) {
          f.block(wall[0], wall[1], wall[2], wall[3]);
        }
        f.router.setHeuristic(heuristic);
        for (const int16_t bound : {-1, 0, 1, 2, 3, 4, 5, 6, 8}) {
          f.router.setMaxTurns(bound);
          f.router.setTurnLowerBound(false);
          const Path off = f.router.route(one.objective, usePenalty);
          const auto offCost = f.router.lastSearchCost();
          f.router.setTurnLowerBound(true);
          const Path on = f.router.route(one.objective, usePenalty);
          const auto onCost = f.router.lastSearchCost();
          const bool onCutOff = f.router.lastSearchCutOff();
          f.router.setTurnLowerBound(false);
          f.router.setMaxTurns(-1);
          if (!on.empty() && bound >= 0) {
            EXPECT_LE(eighthTurns(on), static_cast<uint32_t>(bound)) << bound;
          }
          if (off.empty()) {
            continue;
          }
          ++found;
          ASSERT_FALSE(on.empty()) << bound;
          EXPECT_EQ(onCost, offCost) << bound;
          EXPECT_TRUE(avoids(on, f.corridor)) << bound;
          cutOff += onCutOff ? 1U : 0U;
        }
      }
    }
  }
  EXPECT_GE(found, 100U);
  // Some searches that found a way still dropped moves for the bound.
  EXPECT_GT(cutOff, 0U);
}

/// A search with the turn bound that finds no way under a turn bound says
/// that it was cut off wherever a way exists without the bound.
TEST(DubinsRouter, TheTurnBoundCutsOffWhatTheTurnLimitCutsOff) {
  for (const TurnCase& one : turnCases()) {
    Fixture f;
    for (const auto& wall : one.walls) {
      f.block(wall[0], wall[1], wall[2], wall[3]);
    }
    const Path free = f.router.route(one.objective, true);
    f.router.setTurnLowerBound(true);
    for (const int16_t bound : {0, 1, 2, 3}) {
      f.router.setMaxTurns(bound);
      const Path bounded = f.router.route(one.objective, true);
      if (bounded.empty() && !free.empty()) {
        EXPECT_TRUE(f.router.lastSearchCutOff()) << bound;
      }
    }
  }
}

/// With the turn bound, a multi-target search costs what it costs without
/// it, target by target.
TEST(DubinsRouter, TheTurnBoundKeepsTheMultiTargetPrices) {
  for (const int16_t bound : {-1, 2, 4}) {
    Fixture f;
    f.block(120, 50, 130, 150);
    f.block(160, 0, 170, 70);
    std::vector<DubinsRouter::MultiTarget> asked;
    for (const PathPoint& target : multiTargets()) {
      asked.push_back({.target = target, .own = nullptr});
    }
    f.router.setMaxTurns(bound);
    const auto off = f.router.routeMulti(ACROSS.source, asked, true);
    f.router.setTurnLowerBound(true);
    const auto on = f.router.routeMulti(ACROSS.source, asked, true);
    ASSERT_EQ(on.size(), off.size());
    for (std::size_t k = 0; k < off.size(); ++k) {
      if (off[k].outcome != DubinsRouter::MultiRoute::Outcome::Found) {
        continue;
      }
      ASSERT_EQ(on[k].outcome, DubinsRouter::MultiRoute::Outcome::Found)
          << bound << " " << k;
      EXPECT_EQ(on[k].cost, off[k].cost) << bound << " " << k;
      if (bound >= 0) {
        EXPECT_LE(on[k].turns, static_cast<uint32_t>(bound)) << k;
      }
    }
  }
}

} // namespace
