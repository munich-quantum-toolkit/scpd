/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// How close route() comes to the cheapest path. A brute-force Dijkstra search
// over the same states, primitive tables and move rules finds the optimum on
// small random grids, and route() is compared with it.

#include "../SplitMix.hpp"
#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/routing/DubinsRouter.hpp"
#include "mqt-scpd/routing/Heading.hpp"
#include "mqt-scpd/routing/Path.hpp"
#include "mqt-scpd/routing/Primitives.hpp"
#include "mqt-scpd/routing/SearchScratch.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <utility>
#include <vector>

namespace {

using mqt::scpd::test::SplitMix;
using namespace mqt::scpd;
using namespace mqt::scpd::routing;

/// The cost of a move as the search charges it, in hundredths of a cell.
uint32_t moveCost(const Heading heading, const Primitive& move,
                  const uint16_t bendPenalty) {
  const auto length = static_cast<uint32_t>(
      static_cast<double>(static_cast<float>(move.cost)) * 100.0);
  return length + (headingDistance(heading, move.exitHeading) * bendPenalty);
}

/// Whether a cell lies on the grid and in the corridor.
bool freeCell(const grid::BitGrid& outsideCorridor, const int64_t x,
              const int64_t y) {
  return x >= 0 && y >= 0 && std::cmp_less(x, outsideCorridor.width()) &&
         std::cmp_less(y, outsideCorridor.height()) &&
         !outsideCorridor.testCell(static_cast<uint32_t>(x),
                                   static_cast<uint32_t>(y));
}

/// The cost of the cheapest path from one state to another under the move
/// rules of the search. A move needs every cell it sweeps and its end cell
/// free; its start cell is the state the path is already in. The start state
/// needs a free cell too.
std::optional<uint32_t> cheapestPath(const MovePrimitives& primitives,
                                     const grid::BitGrid& outsideCorridor,
                                     const PathPoint& from, const PathPoint& to,
                                     const uint16_t bendPenalty) {
  if (!freeCell(outsideCorridor, from.x, from.y)) {
    return std::nullopt;
  }
  const auto width = static_cast<std::size_t>(outsideCorridor.width());
  const auto index = [&](const uint32_t x, const uint32_t y,
                         const Heading heading) {
    return ((((static_cast<std::size_t>(y) * width) + x) * NUM_HEADINGS) +
            (heading & 7U));
  };
  std::vector<uint32_t> best(width * outsideCorridor.height() * NUM_HEADINGS,
                             std::numeric_limits<uint32_t>::max());
  using Item = std::pair<uint32_t, std::size_t>;
  std::priority_queue<Item, std::vector<Item>, std::greater<>> open;
  best[index(from.x, from.y, from.heading)] = 0;
  open.emplace(0, index(from.x, from.y, from.heading));
  const std::size_t goal = index(to.x, to.y, to.heading);
  while (!open.empty()) {
    const auto [cost, state] = open.top();
    open.pop();
    if (cost != best[state]) {
      continue;
    }
    if (state == goal) {
      return cost;
    }
    const auto heading = static_cast<Heading>(state % NUM_HEADINGS);
    const std::size_t cell = state / NUM_HEADINGS;
    const auto x = static_cast<int64_t>(cell % width);
    const auto y = static_cast<int64_t>(cell / width);
    for (const Primitive& move : primitives.of(heading)) {
      const bool clear =
          freeCell(outsideCorridor, x + move.dx, y + move.dy) &&
          std::ranges::all_of(move.swept, [&](const CellOffset& c) {
            return (c.dx == 0 && c.dy == 0) ||
                   freeCell(outsideCorridor, x + c.dx, y + c.dy);
          });
      if (!clear) {
        continue;
      }
      const std::size_t next =
          index(static_cast<uint32_t>(x + move.dx),
                static_cast<uint32_t>(y + move.dy), move.exitHeading);
      const uint32_t total = cost + moveCost(heading, move, bendPenalty);
      if (total < best[next]) {
        best[next] = total;
        open.emplace(total, next);
      }
    }
  }
  return std::nullopt;
}

/// The cost of a routed path, read back from the tags of its points.
///
/// The point before each state carries the move that reached the state. A
/// straight step starts on that point. A turn starts on the first point of
/// the run of points that carry its tag. The cell alone does not mark the
/// start, because some turns sweep their start cell twice. Where the search
/// begins with a turn, the turn starts on the stub cell before that run. At a
/// radius of one cell, an exact quarter turn that begins the search leaves no
/// point; its move is then the one from the first point to the second (see
/// Path).
std::optional<uint32_t> costOf(const MovePrimitives& primitives,
                               const Path& path, const uint16_t bendPenalty) {
  const auto isOn = [&](const std::size_t point, const int64_t x,
                        const int64_t y) {
    return std::cmp_equal(path[point].x, x) && std::cmp_equal(path[point].y, y);
  };
  uint32_t total = 0;
  std::size_t state = path.size() - 1;
  Heading heading = path.back().heading;
  while (state > 0) {
    const PathPoint& tag = path[state - 1];
    const Primitive* move = primitives.find(tag.heading, tag.primitive);
    if ((move == nullptr || move->exitHeading != heading) && state == 1) {
      move = nullptr;
      for (const Primitive& turn : primitives.of(tag.heading)) {
        if (turn.exitHeading == heading &&
            isOn(1, static_cast<int64_t>(path[0].x) + turn.dx,
                 static_cast<int64_t>(path[0].y) + turn.dy)) {
          move = &turn;
        }
      }
    }
    if (move == nullptr || move->exitHeading != heading) {
      return std::nullopt;
    }
    total += moveCost(tag.heading, *move, bendPenalty);
    const int64_t x = static_cast<int64_t>(path[state].x) - move->dx;
    const int64_t y = static_cast<int64_t>(path[state].y) - move->dy;
    std::size_t start = state - 1;
    if (move->exitHeading != tag.heading) {
      while (start > 0 && path[start - 1].heading == tag.heading &&
             path[start - 1].primitive == tag.primitive) {
        --start;
      }
      if (start > 0 && !isOn(start, x, y)) {
        --start;
      }
    }
    if (!isOn(start, x, y)) {
      return std::nullopt;
    }
    state = start;
    heading = tag.heading;
  }
  if (heading != path.front().heading) {
    return std::nullopt;
  }
  return total;
}

/// The number of random grids of the comparison at a bend radius of 5.
constexpr int GRIDS = 1500;

/// The outcome of the comparison over the random grids, for one setting of
/// the bend lower bound.
struct Comparison {
  /// The grids on which route() returned a path.
  int routed = 0;
  /// The grids on which route() rejected a path because it crossed itself.
  int rejectedLoops = 0;
  /// The grids on which route() returned a path dearer than the optimum.
  int worse = 0;
  /// The largest amount by which a path exceeded the optimum.
  uint32_t largestExcess = 0;
};

/// The comparison without the bend lower bound and with it, in this order.
using Comparisons = std::array<Comparison, 2>;

/// Compares route() with the optimum on random grids of 30 to 60 cells per
/// side. The comparison without the bend lower bound always runs; the one
/// with it runs when @p withBendLowerBound is set.
Comparisons compareWithTheOptimum(const uint32_t radius,
                                  const uint16_t bendPenalty, const int grids,
                                  const bool withBendLowerBound) {
  auto primitives = std::make_shared<const MovePrimitives>(radius);
  Comparisons result;
  SplitMix random(2026);
  const auto below = [&](const uint32_t n) {
    return static_cast<uint32_t>(random.next() % n);
  };
  for (int g = 0; g < grids; ++g) {
    const uint32_t width = 30 + below(31);
    const uint32_t height = 30 + below(31);
    SearchScratch scratch(width, height);
    grid::BitGrid outsideCorridor(width, height);
    const uint32_t boxes = below(9);
    for (uint32_t b = 0; b < boxes; ++b) {
      const uint32_t x0 = below(width);
      const uint32_t y0 = below(height);
      const uint32_t x1 = std::min(width, x0 + 1 + below(10));
      const uint32_t y1 = std::min(height, y0 + 1 + below(10));
      for (uint32_t y = y0; y < y1; ++y) {
        for (uint32_t x = x0; x < x1; ++x) {
          outsideCorridor.setCell(x, y);
        }
      }
    }
    DubinsRouter router(primitives, scratch,
                        {.startStraightLength = 0,
                         .endStraightLength = 0,
                         .minRadius = static_cast<uint8_t>(radius),
                         .bendPenalty = bendPenalty});
    router.attachCorridor(&outsideCorridor);
    const RoutingObjective objective{
        .source = {.x = below(width),
                   .y = below(height),
                   .heading = static_cast<Heading>(below(8)),
                   .primitive = 0},
        .target = {.x = below(width),
                   .y = below(height),
                   .heading = static_cast<Heading>(below(8)),
                   .primitive = 0}};
    const std::optional<uint32_t> optimum =
        cheapestPath(*primitives, outsideCorridor, objective.source,
                     objective.target, bendPenalty);
    const std::size_t settings = withBendLowerBound ? result.size() : 1;
    for (std::size_t setting = 0; setting < settings; ++setting) {
      Comparison& comparison = result[setting];
      router.setBendLowerBound(setting == 1);
      const uint64_t rejections = router.loopGuardRejections();
      const Path path = router.route(objective);
      if (!optimum.has_value()) {
        EXPECT_TRUE(path.empty()) << radius << ' ' << g;
        continue;
      }
      if (path.empty()) {
        // The search found a path that crossed itself and rejected it.
        EXPECT_EQ(router.loopGuardRejections(), rejections + 1)
            << radius << ' ' << g;
        ++comparison.rejectedLoops;
        continue;
      }
      const std::optional<uint32_t> cost =
          costOf(*primitives, path, bendPenalty);
      EXPECT_TRUE(cost.has_value()) << radius << ' ' << g;
      if (!cost.has_value()) {
        continue;
      }
      ++comparison.routed;
      EXPECT_GE(*cost, *optimum) << radius << ' ' << g;
      if (*cost > *optimum) {
        ++comparison.worse;
        comparison.largestExcess =
            std::max(comparison.largestExcess, *cost - *optimum);
      }
    }
  }
  return result;
}

/// The comparison at a bend radius of 5 and a bend penalty of 500, computed
/// once for both tests that read it.
const Comparisons& comparisons() {
  static const Comparisons RESULT = compareWithTheOptimum(5, 500, GRIDS, true);
  return RESULT;
}

TEST(RouteOptimality, WithoutTheBendLowerBoundEveryPathIsTheCheapest) {
  // At a bend radius of 5, the distance term drops along a turn by at most 20
  // more than the length of the turn, per eighth turn. A turn pays a bend
  // penalty of 500 per eighth turn on top of its length, so the estimate is
  // consistent.
  const Comparison& c = comparisons()[0];
  EXPECT_GT(c.routed, GRIDS / 2);
  EXPECT_LT(c.rejectedLoops, GRIDS / 20);
  EXPECT_EQ(c.worse, 0);
}

TEST(RouteOptimality,
     WithoutTheBendLowerBoundEveryPathIsTheCheapestAtAnyRadius) {
  // At every radius up to 23 cells, the distance term drops along a turn by
  // at most 96 more than the length of the turn, per eighth turn. The
  // default bend penalty of 100 covers that, so the estimate is consistent.
  // On grids of 30 to 60 cells, a turning circle of 24 cells forces many
  // paths at a radius of 12 to cross themselves, so the test does not bound
  // the rejections.
  constexpr int grids = 300;
  for (const uint32_t radius : {1U, 3U, 8U, 12U}) {
    const Comparison c = compareWithTheOptimum(radius, 100, grids, false)[0];
    EXPECT_GT(c.routed, grids / 10) << radius;
    EXPECT_EQ(c.worse, 0) << radius;
  }
}

TEST(RouteOptimality, WithTheBendLowerBoundFewPathsCostAtMostTwentyMore) {
  // With the bend term, the estimate already holds the bend penalty, so the
  // drop of the distance term beyond the length of a turn shows, and route()
  // may return a dearer path on a few grids. The bounds are what the
  // comparison measures at a bend radius of 5.
  const Comparison& c = comparisons()[1];
  EXPECT_GT(c.routed, GRIDS / 2);
  EXPECT_LT(c.rejectedLoops, GRIDS / 20);
  EXPECT_LE(c.worse, c.routed / 100);
  EXPECT_LE(c.largestExcess, 20U);
}

} // namespace
