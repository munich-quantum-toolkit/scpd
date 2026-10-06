/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// What a routing run costs in memory, and why.
//
// Two things drive the cost: the search scratch, which is one record per cell
// and heading, and the grids a router owns. The scratch is eight bytes per
// state and belongs to one thread; the obstacle mask and the corridor mask are
// bit grids attached by pointer and shared by every thread; the wire
// proximity is a view. The per-cell cost of a router is measured from what it
// allocates, and a peak figure is projected from it.

#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/routing/DubinsRouter.hpp"
#include "mqt-scpd/routing/Heading.hpp"
#include "mqt-scpd/routing/Path.hpp"
#include "mqt-scpd/routing/Primitives.hpp"
#include "mqt-scpd/routing/SearchScratch.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

namespace {

using namespace mqt::scpd;
using namespace mqt::scpd::routing;

/// The cells along each side of the router grid of the largest benchmark
/// chip: sixty-nine qubits at a cell size of 10 um.
constexpr uint32_t LARGEST_SIDE = 5400;
/// The cells of the router grid of the largest benchmark chip.
constexpr std::size_t LARGEST_CELLS =
    static_cast<std::size_t>(LARGEST_SIDE) * LARGEST_SIDE;
constexpr double MEGABYTE = 1024.0 * 1024.0;

TEST(MemoryModel, TheObstacleMaskIsOneBitPerCell) {
  const grid::BitGrid mask(LARGEST_SIDE, LARGEST_SIDE);
  EXPECT_EQ(mask.size(), LARGEST_CELLS);
  const std::size_t bytes = mask.words().size() * sizeof(uint64_t);
  EXPECT_EQ(bytes, ((LARGEST_CELLS + 63) / 64) * sizeof(uint64_t));
  // About three and a half megabytes, against twenty-eight as one byte per
  // cell.
  EXPECT_LT(static_cast<double>(bytes) / MEGABYTE, 3.5);
}

TEST(MemoryModel, TheSearchScratchIsEightBytesPerState) {
  EXPECT_EQ(sizeof(SearchNode), 8U);
  const SearchScratch scratch(1000, 1000);
  EXPECT_EQ(scratch.size(), 1000ULL * 1000ULL * 8ULL);
  const double perCell =
      static_cast<double>(scratch.size() * sizeof(SearchNode)) /
      (1000.0 * 1000.0);
  EXPECT_DOUBLE_EQ(perCell, 64.0);
  // The scratch of the largest benchmark, per thread: about 1.8 GB.
  const double largest =
      static_cast<double>(LARGEST_CELLS) * perCell / MEGABYTE;
  EXPECT_GT(largest, 1750.0);
  EXPECT_LT(largest, 1800.0);
}

TEST(MemoryModel, TheScratchNeedsNoClearingWhenItsIterationWrapsAround) {
  SearchScratch scratch(4, 4);
  scratch.beginSearch();
  const uint16_t first = scratch.iteration();
  const uint32_t start = scratch.index(1, 2, 3);
  scratch.setStart(start);
  ASSERT_EQ(scratch.at(start).iteration, first);

  // The sixteen-bit counter skips zero, so it runs through 65535 numbers and
  // then comes back to the number of the first search. The record of that
  // search must not pass for a record of the new one.
  for (uint32_t search = 0; search < 0xFFFFU; ++search) {
    scratch.beginSearch();
  }
  ASSERT_EQ(scratch.iteration(), first);
  EXPECT_EQ(scratch.at(start).g, SearchNode::UNSEEN);
  for (uint32_t i = 0; i < scratch.size(); ++i) {
    EXPECT_NE(scratch.at(i).iteration, scratch.iteration()) << i;
  }
}

TEST(MemoryModel, AScratchMovesButIsNeitherCopiedNorAssigned) {
  // A router keeps a pointer to its scratch and the size of its grid. A copy
  // would double the largest allocation, and an assignment would change the
  // size under the router.
  static_assert(!std::is_copy_constructible_v<SearchScratch>);
  static_assert(!std::is_copy_assignable_v<SearchScratch>);
  static_assert(!std::is_move_assignable_v<SearchScratch>);
  static_assert(std::is_nothrow_move_constructible_v<SearchScratch>);
  SearchScratch scratch(4, 3);
  const SearchScratch moved(std::move(scratch));
  EXPECT_EQ(moved.width(), 4U);
  EXPECT_EQ(moved.height(), 3U);
  EXPECT_EQ(moved.size(), 4U * 3U * NUM_HEADINGS);
  // The moved-from scratch is the scratch of an empty grid. The test reads
  // this documented state on purpose.
  // NOLINTBEGIN(bugprone-use-after-move,clang-analyzer-cplusplus.Move)
  EXPECT_EQ(scratch.size(), 0U);
  EXPECT_EQ(scratch.width(), 0U);
  EXPECT_EQ(scratch.height(), 0U);
  // NOLINTEND(bugprone-use-after-move,clang-analyzer-cplusplus.Move)
}

TEST(MemoryModel, ARouterCopiesNeitherTheObstaclesNorTheCorridor) {
  constexpr uint32_t side = 200;
  auto primitives = std::make_shared<const MovePrimitives>(5);
  SearchScratch scratch(side, side);
  grid::BitGrid obstacles(side, side);
  grid::BitGrid outsideCorridor(side, side);
  std::vector<uint8_t> wire(static_cast<std::size_t>(side) * side, 0);
  DubinsRouter router(primitives, scratch);
  router.attachObstacles(&obstacles);
  router.attachCorridorUnpacked(&outsideCorridor);
  router.attachWireProximity(&wire);

  // The router reads the caller's grids, so a change made after attaching
  // them is the one the router sees. Nothing was copied.
  EXPECT_FALSE(router.corridorBlocked(50, 50));
  outsideCorridor.setCell(50, 50);
  EXPECT_TRUE(router.corridorBlocked(50, 50));
  EXPECT_EQ(router.obstacles(), &obstacles);
  EXPECT_EQ(router.corridor(), &outsideCorridor);

  obstacles.setCell(100, 100);
  router.computeStaticProximity(4, 20);
  EXPECT_EQ(router.staticProximity()[(100ULL * side) + 100], 20);

  wire[(60ULL * side) + 60] = 7;
  EXPECT_EQ(router.wirePenalty((60ULL * side) + 60), 7);
}

TEST(MemoryModel, TheStaticProximityKeepsNoMemoryOfItsGrowth) {
  // The growth of the static proximity needs a byte per cell while it runs.
  // A router computes the proximity once and then routes for a long time, so
  // it must not keep that memory.
  constexpr uint32_t side = 400;
  auto primitives = std::make_shared<const MovePrimitives>(5);
  SearchScratch scratch(side, side);
  grid::BitGrid obstacles(side, side);
  for (uint32_t x = 0; x < side; ++x) {
    obstacles.setCell(x, 200);
  }
  DubinsRouter router(primitives, scratch);
  router.attachObstacles(&obstacles);
  const std::size_t held = router.heldBytes();
  router.computeStaticProximity(10, 40);
  EXPECT_EQ(router.staticPenalty((200ULL * side) + 7), 40);
  EXPECT_GT(router.staticPenalty((205ULL * side) + 7), 0);
  EXPECT_EQ(router.heldBytes(), held);
}

TEST(MemoryModel, ARouterAndItsScratchHoldAboutSeventySixBytesPerCell) {
  // A router that has used everything it owns: obstacles on a third of the
  // grid, the static proximity grown from them, crossing constraints, an
  // exemption, a single-crossing feedline, and a search of each kind. Each
  // search runs through a corridor across a tenth of the grid, as a wire
  // runs through its corridor.
  constexpr uint32_t side = 400;
  constexpr std::size_t cells = static_cast<std::size_t>(side) * side;
  auto primitives = std::make_shared<const MovePrimitives>(5);
  SearchScratch scratch(side, side);
  grid::BitGrid obstacles(side, side);
  for (uint32_t y = 0; y < side; ++y) {
    for (uint32_t x = 0; x < side; ++x) {
      // Blocks of 20 by 20 cells in a lattice of 40 by 30 cells.
      if ((x % 40) >= 10 && (x % 40) < 30 && (y % 30) >= 5 && (y % 30) < 25) {
        obstacles.setCell(x, y);
      }
    }
  }
  // A band of rows for the first search and a band of columns for the
  // second, each forty cells wide.
  grid::BitGrid rows = obstacles;
  grid::BitGrid columns = obstacles;
  for (uint32_t y = 0; y < side; ++y) {
    for (uint32_t x = 0; x < side; ++x) {
      if (y < 180 || y >= 220) {
        rows.setCell(x, y);
      }
      if (x < 180 || x >= 220) {
        columns.setCell(x, y);
      }
    }
  }
  DubinsRouter router(primitives, scratch,
                      {.startStraightLength = 10,
                       .endStraightLength = 10,
                       .minRadius = 5,
                       .bendPenalty = 500});
  router.attachObstacles(&obstacles);
  router.attachCorridor(&rows);
  router.computeStaticProximity(5, 40);
  const Path feedline = router.route(
      {.source = {.x = 2, .y = 182, .heading = 6, .primitive = 0},
       .target = {.x = 397, .y = 212, .heading = 6, .primitive = 0}});
  ASSERT_FALSE(feedline.empty());
  router.buildOrthogonalConstraints({feedline}, {}, 6);
  router.setSingleCrossingFeedline(&feedline, 10, 2);
  std::vector<uint32_t> exempt(1000);
  for (uint32_t i = 0; i < exempt.size(); ++i) {
    exempt[i] = i * 151;
  }
  router.setCrossingExemption(exempt);
  router.attachCorridor(&columns);
  EXPECT_FALSE(
      router
          .routeOrthogonal(
              {.source = {.x = 192, .y = 2, .heading = 4, .primitive = 0},
               .target = {.x = 207, .y = 397, .heading = 4, .primitive = 0}})
          .empty());

  const std::size_t scratchBytes = scratch.size() * sizeof(SearchNode);
  const double perCell =
      static_cast<double>(scratchBytes + router.heldBytes()) /
      static_cast<double>(cells);
  // Sixty-four bytes of scratch. Per cell, the router owns one byte each of
  // static proximity, packed working grid, crossing constraints and crossing
  // rules, and four of distance field: eight bytes. The crossing rules hold
  // the exemption and the single-crossing overlay. The open list adds three
  // bytes here, and the rest is small.
  EXPECT_GT(perCell, 74.0);
  EXPECT_LT(perCell, 78.0);

  // Projected to the router grid of the largest benchmark, one router takes
  // about 2.2 GB, so three routers in parallel need about 6.6 GB.
  const double largest = perCell * static_cast<double>(LARGEST_CELLS);
  EXPECT_GT(largest, 2.1e9);
  EXPECT_LT(largest, 2.3e9);
}

} // namespace
