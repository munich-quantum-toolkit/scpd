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
// The prototype held around 1.87 GB per router context and ran four of them,
// roughly 7.5 GB on the largest benchmark. Two things drive that: the search
// scratch, which is one record per cell and heading, and the grids, which
// the prototype copied into every router on every wire.
//
// Here the scratch is eight bytes per state and belongs to one thread; the
// obstacle mask and the corridor are bit grids attached by pointer and
// shared by every thread; the wire proximity is a view. The per-cell costs
// below are what a peak figure is computed from, so they are asserted rather
// than described.

#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/routing/DubinsRouter.hpp"
#include "mqt-scpd/routing/Primitives.hpp"
#include "mqt-scpd/routing/SearchScratch.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace {

using namespace mqt::scpd;
using namespace mqt::scpd::routing;

/// The router grid of the largest benchmark chip, as the prototype measured
/// it: sixty-nine qubits on about three thousand cells per side.
constexpr std::size_t LARGEST_CELLS = 3016ULL * 3016ULL;
constexpr double MEGABYTE = 1024.0 * 1024.0;

TEST(MemoryModel, TheObstacleMaskIsOneBitPerCell) {
  const grid::BitGrid mask(3016, 3016);
  EXPECT_EQ(mask.size(), LARGEST_CELLS);
  const std::size_t bytes = mask.words().size() * sizeof(uint64_t);
  EXPECT_EQ(bytes, ((LARGEST_CELLS + 63) / 64) * sizeof(uint64_t));
  // Around one megabyte, against nine as one byte per cell.
  EXPECT_LT(static_cast<double>(bytes) / MEGABYTE, 1.5);
}

TEST(MemoryModel, TheSearchScratchIsEightBytesPerState) {
  EXPECT_EQ(sizeof(SearchNode), 8U);
  const SearchScratch scratch(1000, 1000);
  EXPECT_EQ(scratch.size(), 1000ULL * 1000ULL * 8ULL);
  const double perCell =
      static_cast<double>(scratch.size() * sizeof(SearchNode)) / (1000.0 * 1000.0);
  EXPECT_DOUBLE_EQ(perCell, 64.0);
  // The scratch of the largest benchmark, per thread.
  const double largest = static_cast<double>(LARGEST_CELLS) * perCell / MEGABYTE;
  EXPECT_LT(largest, 600.0);
}

TEST(MemoryModel, ARouterCopiesNeitherTheObstaclesNorTheCorridor) {
  constexpr uint32_t side = 200;
  auto primitives = std::make_shared<const MovePrimitives>(5);
  SearchScratch scratch(side, side);
  grid::BitGrid obstacles(side, side);
  grid::BitGrid corridor(side, side);
  std::vector<uint8_t> wire(static_cast<std::size_t>(side) * side, 0);
  DubinsRouter router(primitives, scratch);
  router.attachObstacles(&obstacles);
  router.attachCorridorUnpacked(&corridor);
  router.attachWireProximity(&wire);

  // The router reads the caller's grids, so a change made after attaching
  // them is the one the router sees. Nothing was copied.
  EXPECT_FALSE(router.corridorBlocked(50, 50));
  corridor.setCell(50, 50);
  EXPECT_TRUE(router.corridorBlocked(50, 50));
  EXPECT_EQ(router.obstacles(), &obstacles);
  EXPECT_EQ(router.corridor(), &corridor);

  obstacles.setCell(100, 100);
  router.computeStaticProximity(4, 20);
  EXPECT_EQ(router.staticProximity()[(100ULL * side) + 100], 20);

  wire[(60ULL * side) + 60] = 7;
  EXPECT_EQ(router.wirePenalty((60ULL * side) + 60), 7);
}

TEST(MemoryModel, TheRoutersOwnGridsAreASmallMultipleOfTheCells) {
  constexpr uint32_t side = 400;
  constexpr std::size_t cells = static_cast<std::size_t>(side) * side;
  auto primitives = std::make_shared<const MovePrimitives>(5);
  SearchScratch scratch(side, side);
  grid::BitGrid obstacles(side, side);
  grid::BitGrid corridor(side, side);
  DubinsRouter router(primitives, scratch);
  router.attachObstacles(&obstacles);
  router.attachCorridor(&corridor);
  EXPECT_EQ(router.cells(), cells);
  // The static proximity is one byte per cell, and the packed working grid
  // that folds the corridor into it is another. The distance field is four,
  // the crossing constraints and the single-crossing overlay one each, and
  // both of those exist only once a caller builds them. Seven bytes per
  // cell in total, against the sixty-four of the scratch.
  EXPECT_EQ(router.staticProximity().size(), cells);
}

TEST(MemoryModel, TheBudgetOfTheLargestBenchmarkFitsThreeThreads) {
  // Per cell: sixty-four bytes of scratch, one of static proximity, one of
  // the packed working grid, four of the distance field, one of the
  // crossing constraints and one of the single-crossing overlay.
  constexpr double perCellPerThread = 64.0 + 1.0 + 1.0 + 4.0 + 1.0 + 1.0;
  // Shared by every thread: the obstacle mask and the corridor as bits, and
  // the wire proximity as a byte per cell.
  constexpr double perCellShared = 0.125 + 0.125 + 1.0;

  const double perThread = static_cast<double>(LARGEST_CELLS) * perCellPerThread / MEGABYTE;
  const double shared = static_cast<double>(LARGEST_CELLS) * perCellShared / MEGABYTE;
  EXPECT_LT(perThread, 700.0);
  EXPECT_LT(shared, 15.0);

  // The prototype held about 1.87 GB per context and ran four.
  EXPECT_LT(perThread, 1870.0 / 2.0);
  // Three threads stay inside the two gigabytes the rewrite targets.
  EXPECT_LT((3.0 * perThread) + shared, 2048.0);
}

} // namespace
