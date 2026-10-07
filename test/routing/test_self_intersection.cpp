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

/// Appends one move from @p state to @p path in the format of Path, and moves
/// @p state to the end of the move.
void appendMove(Path& path, PathPoint& state, const Primitive& move) {
  PathPoint start = state;
  start.primitive = move.id;
  if (!path.empty() && path.back().samePlace(start)) {
    path.back() = start;
  } else {
    path.push_back(start);
  }
  for (const CellOffset& offset : move.swept) {
    const PathPoint cell{
        .x = static_cast<uint32_t>(static_cast<int32_t>(state.x) + offset.dx),
        .y = static_cast<uint32_t>(static_cast<int32_t>(state.y) + offset.dy),
        .heading = state.heading,
        .primitive = move.id};
    if (!path.back().samePlace(cell)) {
      path.push_back(cell);
    }
  }
  state.x = static_cast<uint32_t>(static_cast<int32_t>(state.x) + move.dx);
  state.y = static_cast<uint32_t>(static_cast<int32_t>(state.y) + move.dy);
  state.heading = move.exitHeading;
}

/// A routed path from (500, 500) on @p heading: three straight steps, the
/// given moves, three straight steps and the end.
Path routedPath(const MovePrimitives& primitives, const Heading heading,
                const std::vector<const Primitive*>& moves) {
  Path path;
  PathPoint state{.x = 500, .y = 500, .heading = heading, .primitive = 0};
  const auto straight = [&] {
    appendMove(
        path, state,
        *primitives.find(state.heading, primitives.straight(state.heading)));
  };
  for (int step = 0; step < 3; ++step) {
    straight();
  }
  for (const Primitive* move : moves) {
    appendMove(path, state, *move);
  }
  for (int step = 0; step < 3; ++step) {
    straight();
  }
  state.primitive = primitives.straight(state.heading);
  path.push_back(state);
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
  // A hairpin whose second leg crosses the first between two diagonal steps
  // through one 2 by 2 block. No cell repeats, so only the test of the block
  // finds the crossing.
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

TEST(SelfIntersection, ALoopOfFourStepsIsFound) {
  PathLoopScratch scratch;
  std::vector<PathLoopHit> hits;
  // A ring around a 2 by 2 block.
  EXPECT_EQ(
      findPathSelfIntersections(
          pathOf({{10, 10}, {11, 10}, {11, 11}, {10, 11}, {10, 10}, {9, 10}}),
          100, 100, scratch, &hits),
      1U);
  // A run of two cells out and back.
  EXPECT_EQ(
      findPathSelfIntersections(
          pathOf({{10, 10}, {11, 10}, {12, 10}, {11, 10}, {10, 10}, {10, 11}}),
          100, 100, scratch, &hits),
      1U);
  ASSERT_EQ(hits.size(), 2U);
  for (const PathLoopHit& hit : hits) {
    EXPECT_EQ(hit.kind, PathLoopKind::Revisit);
    EXPECT_EQ(hit.x, 10U);
    EXPECT_EQ(hit.y, 10U);
    EXPECT_EQ(hit.secondIndex - hit.firstIndex, 4U);
  }
}

TEST(SelfIntersection, NoTwoMovesOfTheRouterReadAsALoop) {
  // Two moves cannot close a loop, but the cells they list can repeat a cell
  // within the spur window: an eighth turn lists the cell past its end, and
  // at a radius of one cell a U-turn lists the cell between its arcs twice.
  // Every pair of moves at every radius the router accepts must pass.
  std::size_t spurs = 0;
  for (uint32_t radius = 1; radius <= 23; ++radius) {
    const MovePrimitives primitives(radius);
    PathLoopScratch scratch;
    for (Heading heading = 0; heading < NUM_HEADINGS; ++heading) {
      for (const Primitive& first : primitives.of(heading)) {
        for (const Primitive& second : primitives.of(first.exitHeading)) {
          const Path path = routedPath(primitives, heading, {&first, &second});
          EXPECT_FALSE(pathSelfIntersects(path, 1000, 1000, scratch))
              << "radius " << radius << ", heading " << int{heading}
              << ", moves " << first.id << " and " << second.id;
          for (std::size_t j = 1; j < scratch.xs.size(); ++j) {
            for (std::size_t i = j > 3 ? j - 3 : 0; i < j; ++i) {
              if (scratch.xs[i] == scratch.xs[j] &&
                  scratch.ys[i] == scratch.ys[j]) {
                ++spurs;
              }
            }
          }
        }
      }
    }
  }
  EXPECT_GT(spurs, 0U);
}

TEST(SelfIntersection, TheShortestLoopOfARoutedPathIsFound) {
  // At a radius of one cell, three exact quarter turns of a diagonal heading
  // and one quarter turn back close a loop after five steps.
  const MovePrimitives primitives(1);
  std::vector<const Primitive*> moves;
  Heading heading = 1;
  for (const uint32_t id : {900U, 900U, 900U, 901U}) {
    const Primitive* move = primitives.find(heading, id);
    ASSERT_NE(move, nullptr);
    moves.push_back(move);
    heading = move->exitHeading;
  }
  PathLoopScratch scratch;
  PathLoopHit hit;
  ASSERT_TRUE(pathSelfIntersects(routedPath(primitives, 1, moves), 1000, 1000,
                                 scratch, &hit));
  EXPECT_EQ(hit.kind, PathLoopKind::Revisit);
  EXPECT_GT(hit.secondIndex - hit.firstIndex, PATH_LOOP_SPUR_WINDOW);
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
  EXPECT_EQ(findPathSelfIntersections(pathOf(cells), 100, 100, scratch, &hits),
            1U);
  EXPECT_EQ(hits.size(), 1U);
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

TEST(SelfIntersection, AJumpFarOutsideTheGridKeepsTheCellsInside) {
  // Each path runs between a cell of the grid and a point far outside it, up
  // to the end of the range of a coordinate. The cells are those of the part
  // of the run inside the grid.
  constexpr uint32_t far = 4294967294U;
  constexpr uint32_t signedLimit = 2147483648U;
  PathLoopScratch scratch;
  struct Case {
    Path path;
    int32_t startX;
    int32_t startY;
    int32_t stepX;
    int32_t stepY;
  };
  for (const Case& c : {Case{.path = pathOf({{0, 5}, {far, 5}}),
                             .startX = 0,
                             .startY = 5,
                             .stepX = 1,
                             .stepY = 0},
                        Case{.path = pathOf({{signedLimit, 5}, {0, 5}}),
                             .startX = 99,
                             .startY = 5,
                             .stepX = -1,
                             .stepY = 0},
                        Case{.path = pathOf({{0, 0}, {far, far}}),
                             .startX = 0,
                             .startY = 0,
                             .stepX = 1,
                             .stepY = 1}}) {
    rasterizePathCells(c.path, 100, 100, scratch.xs, scratch.ys);
    ASSERT_EQ(scratch.xs.size(), 100U);
    ASSERT_EQ(scratch.ys.size(), 100U);
    for (std::size_t i = 0; i < 100; ++i) {
      const auto step = static_cast<int32_t>(i);
      EXPECT_EQ(scratch.xs[i], c.startX + (step * c.stepX)) << i;
      EXPECT_EQ(scratch.ys[i], c.startY + (step * c.stepY)) << i;
    }
  }
}

TEST(SelfIntersection, AStepAcrossAStretchOutsideTheGridCrossesNoBlock) {
  // The path leaves the grid through its right edge at (99, 10) and comes
  // back through its top edge at (50, 99). The two cells are no neighbors,
  // so the step between them crosses no 2 by 2 block. Later, a diagonal step
  // from (51, 11) to (50, 10) crosses the block at (50, 10).
  const Path path = pathOf({{90, 10},
                            {200, 10},
                            {200, 200},
                            {50, 200},
                            {50, 12},
                            {51, 11},
                            {50, 10}});
  PathLoopScratch scratch;
  std::vector<PathLoopHit> hits;
  EXPECT_EQ(findPathSelfIntersections(path, 100, 100, scratch, &hits), 0U);
  EXPECT_TRUE(hits.empty());
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

TEST(SelfIntersection, AKeptScratchKeepsItsMemory) {
  // A serpentine of diagonal steps that ends on its own first cell.
  std::vector<std::pair<uint32_t, uint32_t>> cells;
  for (uint32_t row = 0; row < 20; ++row) {
    for (uint32_t x = 0; x < 40; ++x) {
      const uint32_t across = row % 2 == 0 ? x : 39 - x;
      cells.emplace_back(10 + across, 10 + (3 * row) + (x % 2));
    }
  }
  cells.emplace_back(10, 10);
  const Path path = pathOf(cells);
  PathLoopScratch scratch;
  EXPECT_EQ(scratch.heldBytes(), 0U);
  ASSERT_TRUE(pathSelfIntersects(path, 100, 100, scratch));
  const std::size_t held = scratch.heldBytes();
  // The count covers more than the rasterized cells.
  EXPECT_GT(held,
            (scratch.xs.capacity() + scratch.ys.capacity()) * sizeof(int32_t));
  // Another scan of the path, and a scan of a shorter one, fit in the memory
  // the scratch already holds.
  EXPECT_GT(findPathSelfIntersections(path, 100, 100, scratch, nullptr), 0U);
  EXPECT_EQ(scratch.heldBytes(), held);
  EXPECT_FALSE(pathSelfIntersects(
      pathOf({{10, 10}, {11, 11}, {12, 12}, {13, 12}}), 100, 100, scratch));
  EXPECT_EQ(scratch.heldBytes(), held);
}

TEST(SelfIntersection, AShortPathHasNothingToFind) {
  PathLoopScratch scratch;
  EXPECT_FALSE(pathSelfIntersects(Path{}, 100, 100, scratch));
  EXPECT_FALSE(pathSelfIntersects(pathOf({{5, 5}}), 100, 100, scratch));
  EXPECT_FALSE(pathSelfIntersects(pathOf({{5, 5}, {6, 5}}), 100, 100, scratch));
}

} // namespace
