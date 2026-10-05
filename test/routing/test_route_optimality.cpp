/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// How close route() comes to the cheapest way. A brute-force Dijkstra search
// over the same states, primitive tables and move rules finds the optimum on
// small random grids, and route() is compared with it.

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

using namespace mqt::scpd;
using namespace mqt::scpd::routing;

constexpr uint16_t BEND_PENALTY = 500;

/// The next number of a fixed sequence, so that the grids are the same on
/// every platform.
uint64_t splitmix(uint64_t& state) {
  uint64_t z = (state += 0x9E3779B97F4A7C15ULL);
  z = (z ^ (z >> 30U)) * 0xBF58476D1CE4E5B9ULL;
  z = (z ^ (z >> 27U)) * 0x94D049BB133111EBULL;
  return z ^ (z >> 31U);
}

/// The cost of a move as the search charges it, in hundredths of a cell.
uint32_t moveCost(const Heading heading, const Primitive& move) {
  const auto length = static_cast<uint32_t>(
      static_cast<double>(static_cast<float>(move.cost)) * 100.0);
  return length + (headingDistance(heading, move.exitHeading) * BEND_PENALTY);
}

/// Whether a cell lies on the grid and outside the blocked cells.
bool freeCell(const grid::BitGrid& corridor, const int64_t x, const int64_t y) {
  return x >= 0 && y >= 0 && std::cmp_less(x, corridor.width()) &&
         std::cmp_less(y, corridor.height()) &&
         !corridor.testCell(static_cast<uint32_t>(x), static_cast<uint32_t>(y));
}

/// The cost of the cheapest way from one state to another under the move
/// rules of the search. A move needs every cell it sweeps and its end cell
/// free; its start cell is the state the way is already in. The start state
/// needs a free cell too.
std::optional<uint32_t> cheapestWay(const MovePrimitives& primitives,
                                    const grid::BitGrid& corridor,
                                    const PathPoint& from,
                                    const PathPoint& to) {
  if (!freeCell(corridor, from.x, from.y)) {
    return std::nullopt;
  }
  const auto width = static_cast<std::size_t>(corridor.width());
  const auto index = [&](const uint32_t x, const uint32_t y,
                         const Heading heading) {
    return ((((static_cast<std::size_t>(y) * width) + x) * NUM_HEADINGS) +
            (heading & 7U));
  };
  std::vector<uint32_t> best(width * corridor.height() * NUM_HEADINGS,
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
          freeCell(corridor, x + move.dx, y + move.dy) &&
          std::ranges::all_of(move.swept, [&](const CellOffset& c) {
            return (c.dx == 0 && c.dy == 0) ||
                   freeCell(corridor, x + c.dx, y + c.dy);
          });
      if (!clear) {
        continue;
      }
      const std::size_t next =
          index(static_cast<uint32_t>(x + move.dx),
                static_cast<uint32_t>(y + move.dy), move.exitHeading);
      const uint32_t total = cost + moveCost(heading, move);
      if (total < best[next]) {
        best[next] = total;
        open.emplace(total, next);
      }
    }
  }
  return std::nullopt;
}

/// The cost of a routed way, read back from the tags of its points. The
/// point before each state carries the move that reached the state: the last
/// swept cell of a turn, or the state a straight step leaves.
std::optional<uint32_t> costOf(const MovePrimitives& primitives,
                               const Path& path) {
  uint32_t total = 0;
  std::size_t at = path.size() - 1;
  int64_t x = path.back().x;
  int64_t y = path.back().y;
  Heading heading = path.back().heading;
  while (at > 0) {
    const PathPoint& tag = path[at - 1];
    const Primitive* move = primitives.find(tag.heading, tag.primitive);
    if (move == nullptr || move->exitHeading != heading) {
      return std::nullopt;
    }
    total += moveCost(tag.heading, *move);
    x -= move->dx;
    y -= move->dy;
    heading = tag.heading;
    while (at > 0 && (std::cmp_not_equal(path[at - 1].x, x) ||
                      std::cmp_not_equal(path[at - 1].y, y))) {
      --at;
    }
    if (at == 0) {
      return std::nullopt;
    }
    --at;
  }
  if (heading != path.front().heading) {
    return std::nullopt;
  }
  return total;
}

/// The number of random grids of the comparison.
constexpr int GRIDS = 1500;

/// The outcome of the comparison over the random grids, for one setting of
/// the bend lower bound.
struct Comparison {
  /// The grids on which route() returned a way.
  int routed = 0;
  /// The grids on which route() rejected a way because it crossed itself.
  int rejectedLoops = 0;
  /// The grids on which route() returned a way dearer than the optimum.
  int worse = 0;
  /// The largest amount by which a way exceeded the optimum.
  uint32_t largestExcess = 0;
};

/// The comparison without the bend lower bound and with it, in this order.
using Comparisons = std::array<Comparison, 2>;

Comparisons compareWithTheOptimum() {
  auto primitives = std::make_shared<const MovePrimitives>(5);
  Comparisons result;
  uint64_t state = 2026;
  const auto below = [&](const uint32_t n) {
    return static_cast<uint32_t>(splitmix(state) % n);
  };
  for (int g = 0; g < GRIDS; ++g) {
    const uint32_t width = 30 + below(31);
    const uint32_t height = 30 + below(31);
    SearchScratch scratch(width, height);
    grid::BitGrid corridor(width, height);
    const uint32_t boxes = below(9);
    for (uint32_t b = 0; b < boxes; ++b) {
      const uint32_t x0 = below(width);
      const uint32_t y0 = below(height);
      const uint32_t x1 = std::min(width, x0 + 1 + below(10));
      const uint32_t y1 = std::min(height, y0 + 1 + below(10));
      for (uint32_t y = y0; y < y1; ++y) {
        for (uint32_t x = x0; x < x1; ++x) {
          corridor.setCell(x, y);
        }
      }
    }
    DubinsRouter router(primitives, scratch,
                        {.startStraightLength = 0,
                         .endStraightLength = 0,
                         .minRadius = 5,
                         .bendPenalty = BEND_PENALTY});
    router.attachCorridor(&corridor);
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
        cheapestWay(*primitives, corridor, objective.source, objective.target);
    for (std::size_t setting = 0; setting < result.size(); ++setting) {
      Comparison& comparison = result[setting];
      router.setBendLowerBound(setting == 1);
      const uint64_t rejections = router.loopGuardRejections();
      const Path path = router.route(objective);
      if (!optimum.has_value()) {
        EXPECT_TRUE(path.empty()) << g;
        continue;
      }
      if (path.empty()) {
        // The search found a way that crossed itself and rejected it.
        EXPECT_EQ(router.loopGuardRejections(), rejections + 1) << g;
        ++comparison.rejectedLoops;
        continue;
      }
      const std::optional<uint32_t> cost = costOf(*primitives, path);
      EXPECT_TRUE(cost.has_value()) << g;
      if (!cost.has_value()) {
        continue;
      }
      ++comparison.routed;
      EXPECT_GE(*cost, *optimum) << g;
      if (*cost > *optimum) {
        ++comparison.worse;
        comparison.largestExcess =
            std::max(comparison.largestExcess, *cost - *optimum);
      }
    }
  }
  return result;
}

/// The comparison, computed once for both tests.
const Comparisons& comparisons() {
  static const Comparisons RESULT = compareWithTheOptimum();
  return RESULT;
}

TEST(RouteOptimality, WithoutTheBendLowerBoundEveryWayIsTheCheapest) {
  // Each turn pays a bend penalty of 500 on top of its length, far above the
  // 20 by which the distance term can exceed the length of a cardinal eighth
  // turn. The estimate then never exceeds what a state still has to pay.
  const Comparison& c = comparisons()[0];
  EXPECT_GT(c.routed, GRIDS / 2);
  EXPECT_LT(c.rejectedLoops, GRIDS / 20);
  EXPECT_EQ(c.worse, 0);
}

TEST(RouteOptimality, WithTheBendLowerBoundAFewWaysCostSlightlyMore) {
  // With the bend term, the estimate already holds the bend penalty, so the
  // excess of the distance term over the length of a turn shows: route()
  // returns a dearer way on a few grids, by no more than the 20 of one
  // cardinal eighth turn.
  const Comparison& c = comparisons()[1];
  EXPECT_GT(c.routed, GRIDS / 2);
  EXPECT_LT(c.rejectedLoops, GRIDS / 20);
  EXPECT_GT(c.worse, 0);
  EXPECT_LE(c.worse, c.routed / 100);
  EXPECT_LE(c.largestExcess, 20U);
}

} // namespace
