/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/routing/Heading.hpp"
#include "mqt-scpd/routing/Path.hpp"
#include "mqt-scpd/routing/Primitives.hpp"
#include "mqt-scpd/routing/SelfIntersection.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <utility>
#include <vector>

namespace {

using namespace mqt::scpd::routing;

/// A path through the given cells, all on one heading.
Path pathOf(const std::vector<std::pair<uint32_t, uint32_t>>& cells) {
  Path path;
  for (const auto& [x, y] : cells) {
    path.push_back({.x = x, .y = y, .heading = 0, .primitive = 0});
  }
  return path;
}

TEST(SelfIntersection, AnOpenPathIsClean) {
  PathLoopScratch scratch;
  Path path;
  for (uint32_t x = 10; x < 60; ++x) {
    path.push_back({.x = x, .y = 20, .heading = 6, .primitive = 0});
  }
  for (uint32_t y = 20; y < 60; ++y) {
    path.push_back({.x = 59, .y = y, .heading = 4, .primitive = 0});
  }
  EXPECT_FALSE(pathSelfIntersects(path, 100, 100, scratch));
  EXPECT_EQ(scratch.spurRevisitsIgnored, 0U);
}

TEST(SelfIntersection, ARevisitedCellIsFound) {
  // A square loop that closes on itself.
  std::vector<std::pair<uint32_t, uint32_t>> cells;
  for (uint32_t x = 10; x <= 30; ++x) {
    cells.emplace_back(x, 10);
  }
  for (uint32_t y = 11; y <= 30; ++y) {
    cells.emplace_back(30, y);
  }
  for (uint32_t x = 29; x >= 10; --x) {
    cells.emplace_back(x, 30);
  }
  for (uint32_t y = 29; y >= 10; --y) {
    cells.emplace_back(10, y);
  }
  PathLoopScratch scratch;
  std::vector<PathLoopHit> hits;
  EXPECT_EQ(findPathSelfIntersections(pathOf(cells), 100, 100, scratch, &hits),
            1U);
  ASSERT_EQ(hits.size(), 1U);
  EXPECT_EQ(hits[0].kind, PathLoopKind::Revisit);
  EXPECT_EQ(hits[0].x, 10U);
  EXPECT_EQ(hits[0].y, 10U);
}

TEST(SelfIntersection, ADiagonalCrossingIsFoundWithoutARepeatedCell) {
  // The two legs cross inside one block while all four cells stay distinct;
  // the geometry is the one measured on the 69-qubit chip.
  const Path path = pathOf({{3700, 4307},
                            {3701, 4306},
                            {3702, 4305},
                            {3703, 4305},
                            {3703, 4306},
                            {3702, 4306},
                            {3701, 4305},
                            {3700, 4304}});
  PathLoopScratch scratch;
  std::vector<PathLoopHit> hits;
  EXPECT_EQ(findPathSelfIntersections(path, 5000, 5000, scratch, &hits), 1U);
  ASSERT_EQ(hits.size(), 1U);
  EXPECT_EQ(hits[0].kind, PathLoopKind::DiagonalCross);
}

TEST(SelfIntersection, TheSpurWindowIgnoresTheSpurOfATurnButNotALoop) {
  // At a bend radius of five cells, an eighth turn from a cardinal heading
  // sweeps one cell past the end of its arc. A routed path therefore lists
  // the end, the cell past it and the end again.
  const MovePrimitives primitives(5);
  const Primitive* turn = nullptr;
  for (const Primitive& move : primitives.of(6)) {
    if (move.exitHeading == 5) {
      turn = &move;
    }
  }
  ASSERT_NE(turn, nullptr);
  Path path;
  for (uint32_t x = 10; x < 30; ++x) {
    path.push_back(
        {.x = x, .y = 20, .heading = 6, .primitive = primitives.straight(6)});
  }
  for (const CellOffset& offset : turn->swept) {
    const PathPoint cell{.x = static_cast<uint32_t>(30 + offset.dx),
                         .y = static_cast<uint32_t>(20 + offset.dy),
                         .heading = 6,
                         .primitive = turn->id};
    if (!path.back().samePlace(cell)) {
      path.push_back(cell);
    }
  }
  const HeadingVector exit = headingVector(5);
  for (int32_t k = 0; k < 20; ++k) {
    path.push_back({.x = static_cast<uint32_t>(30 + turn->dx + (k * exit.dx)),
                    .y = static_cast<uint32_t>(20 + turn->dy + (k * exit.dy)),
                    .heading = 5,
                    .primitive = primitives.straight(5)});
  }
  PathLoopScratch scratch;
  EXPECT_FALSE(pathSelfIntersects(path, 100, 100, scratch));
  EXPECT_EQ(scratch.spurRevisitsIgnored, 1U);
  EXPECT_EQ(scratch.maxSpurDistance, 2U);

  // A path that comes back to a cell five steps later has a loop.
  std::vector<PathLoopHit> hits;
  EXPECT_EQ(
      findPathSelfIntersections(
          pathOf({{10, 10}, {11, 10}, {12, 10}, {12, 11}, {11, 11}, {10, 10}}),
          100, 100, scratch, &hits),
      1U);
  ASSERT_EQ(hits.size(), 1U);
  EXPECT_EQ(hits[0].kind, PathLoopKind::Revisit);
  EXPECT_GT(hits[0].secondIndex - hits[0].firstIndex, PATH_LOOP_SPUR_WINDOW);
}

TEST(SelfIntersection, ALongExactRetraceIsALoopNotASpur) {
  // The trap a stack collapse falls into: it pops every short backtrack and
  // so unwinds this retrace one cell at a time, reporting nothing.
  std::vector<std::pair<uint32_t, uint32_t>> cells;
  for (uint32_t x = 10; x <= 40; ++x) {
    cells.emplace_back(x, 20);
  }
  for (uint32_t x = 39; x >= 10; --x) {
    cells.emplace_back(x, 20);
  }
  PathLoopScratch scratch;
  std::vector<PathLoopHit> hits;
  EXPECT_GT(findPathSelfIntersections(pathOf(cells), 100, 100, scratch, &hits),
            0U);
  ASSERT_FALSE(hits.empty());
  EXPECT_EQ(hits[0].kind, PathLoopKind::Revisit);
}

TEST(SelfIntersection, OneCrossingIsOneEventNotOnePerCell) {
  // After a hit the detector restarts, so a stretch that runs back along
  // itself counts as the crossings it is, not as one per cell.
  std::vector<std::pair<uint32_t, uint32_t>> cells;
  for (uint32_t x = 10; x <= 60; ++x) {
    cells.emplace_back(x, 20);
  }
  for (uint32_t x = 59; x >= 10; --x) {
    cells.emplace_back(x, 20);
  }
  PathLoopScratch scratch;
  std::vector<PathLoopHit> hits;
  const uint32_t events =
      findPathSelfIntersections(pathOf(cells), 100, 100, scratch, &hits);
  EXPECT_LT(events, 10U);
  EXPECT_GT(events, 0U);
}

TEST(SelfIntersection, RasterizingFillsTheGapsAndClipsToTheGrid) {
  const Path path = pathOf({{2, 2}, {8, 5}, {8, 5}, {200, 5}});
  PathLoopScratch scratch;
  rasterizePathCells(path, 20, 20, scratch.xs, scratch.ys);
  ASSERT_FALSE(scratch.xs.empty());
  EXPECT_EQ(scratch.xs.front(), 2);
  EXPECT_EQ(scratch.ys.front(), 2);
  // Consecutive cells differ by at most one along each axis, and no cell
  // outside the grid survives.
  for (std::size_t i = 1; i < scratch.xs.size(); ++i) {
    EXPECT_LE(std::abs(scratch.xs[i] - scratch.xs[i - 1]), 1);
    EXPECT_LE(std::abs(scratch.ys[i] - scratch.ys[i - 1]), 1);
    EXPECT_LT(scratch.xs[i], 20);
    EXPECT_LT(scratch.ys[i], 20);
  }
}

TEST(SelfIntersection, TheQuickCheckReportsTheFirstEventOfTheFullScan) {
  // Two separate loops: the full scan finds both, the quick check stops at
  // the first and reports that one.
  std::vector<std::pair<uint32_t, uint32_t>> cells;
  for (uint32_t x = 10; x <= 20; ++x) {
    cells.emplace_back(x, 10);
  }
  for (uint32_t y = 11; y <= 20; ++y) {
    cells.emplace_back(20, y);
  }
  for (uint32_t x = 19; x >= 15; --x) {
    cells.emplace_back(x, 20);
  }
  for (uint32_t y = 19; y >= 5; --y) {
    cells.emplace_back(15, y);
  }
  for (uint32_t x = 16; x <= 60; ++x) {
    cells.emplace_back(x, 5);
  }
  for (uint32_t y = 6; y <= 15; ++y) {
    cells.emplace_back(60, y);
  }
  for (uint32_t x = 59; x >= 50; --x) {
    cells.emplace_back(x, 15);
  }
  for (uint32_t y = 14; y >= 1; --y) {
    cells.emplace_back(50, y);
  }
  const Path path = pathOf(cells);
  PathLoopScratch scratch;
  std::vector<PathLoopHit> hits;
  ASSERT_EQ(findPathSelfIntersections(path, 100, 100, scratch, &hits), 2U);

  PathLoopHit first;
  EXPECT_TRUE(pathSelfIntersects(path, 100, 100, scratch, &first));
  EXPECT_EQ(first.kind, hits[0].kind);
  EXPECT_EQ(first.x, hits[0].x);
  EXPECT_EQ(first.y, hits[0].y);
  EXPECT_EQ(first.firstIndex, hits[0].firstIndex);
  EXPECT_EQ(first.secondIndex, hits[0].secondIndex);
  // A caller that does not ask for the event gets the same answer.
  EXPECT_TRUE(pathSelfIntersects(path, 100, 100, scratch));
}

TEST(SelfIntersection, ACleanPathLeavesTheCallersEventAlone) {
  Path path;
  for (uint32_t x = 10; x < 60; ++x) {
    path.push_back({.x = x, .y = 20, .heading = 6, .primitive = 0});
  }
  PathLoopScratch scratch;
  PathLoopHit first{.kind = PathLoopKind::DiagonalCross,
                    .x = 7,
                    .y = 8,
                    .firstIndex = 9,
                    .secondIndex = 10};
  EXPECT_FALSE(pathSelfIntersects(path, 100, 100, scratch, &first));
  EXPECT_EQ(first.kind, PathLoopKind::DiagonalCross);
  EXPECT_EQ(first.x, 7U);
  EXPECT_EQ(first.y, 8U);
  EXPECT_EQ(first.firstIndex, 9U);
  EXPECT_EQ(first.secondIndex, 10U);
}

TEST(SelfIntersection, TheScanAddsItsEventsAfterTheCallersEntries) {
  std::vector<std::pair<uint32_t, uint32_t>> cells;
  for (uint32_t x = 10; x <= 40; ++x) {
    cells.emplace_back(x, 20);
  }
  for (uint32_t y = 21; y <= 30; ++y) {
    cells.emplace_back(40, y);
  }
  for (uint32_t x = 39; x >= 30; --x) {
    cells.emplace_back(x, 30);
  }
  for (uint32_t y = 29; y >= 10; --y) {
    cells.emplace_back(30, y);
  }
  PathLoopScratch scratch;
  std::vector<PathLoopHit> hits(1);
  hits[0].x = 99;
  EXPECT_EQ(findPathSelfIntersections(pathOf(cells), 100, 100, scratch, &hits),
            1U);
  ASSERT_EQ(hits.size(), 2U);
  EXPECT_EQ(hits[0].x, 99U);
  EXPECT_EQ(hits[1].kind, PathLoopKind::Revisit);
  EXPECT_EQ(hits[1].x, 30U);
  EXPECT_EQ(hits[1].y, 20U);
}

TEST(SelfIntersection, AShortPathHasNothingToFind) {
  PathLoopScratch scratch;
  EXPECT_FALSE(pathSelfIntersects(Path{}, 100, 100, scratch));
  EXPECT_FALSE(pathSelfIntersects(pathOf({{5, 5}}), 100, 100, scratch));
  EXPECT_FALSE(pathSelfIntersects(pathOf({{5, 5}, {6, 5}}), 100, 100, scratch));
}

} // namespace
