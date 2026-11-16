/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// The moves a bottleneck refuses, on gates whose crossings are known by hand.

#include "mqt-scpd/grid/Bottlenecks.hpp"
#include "mqt-scpd/grid/Chambers.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace mqt::scpd::grid {
namespace {

GridMetrics unitGrid() {
  return GridMetrics::fit(
      {.minX = 0.0, .minY = 0.0, .maxX = 19.0, .maxY = 19.0}, 20, 20);
}

/// The direction of a step, as STEP_DX and STEP_DY number it.
std::size_t directionOf(const int dx, const int dy) {
  for (std::size_t step = 0; step < STEP_DX.size(); ++step) {
    if (STEP_DX[step] == dx && STEP_DY[step] == dy) {
      return step;
    }
  }
  return STEP_DX.size();
}

bool refuses(const std::vector<BlockedMove>& moves, const std::size_t cell,
             const std::size_t direction) {
  return std::ranges::any_of(moves, [&](const BlockedMove& move) {
    return move.cell == cell && move.direction == direction;
  });
}

TEST(GateMoves, ACellOnTheLineOfAGateIsEnclosed) {
  const auto grid = unitGrid();
  const Bottleneck gate{.first = grid.index(10, 5),
                        .second = grid.index(10, 9)};

  const auto moves = bottleneckMoves({gate}, grid);

  ASSERT_EQ(moves.size(), 1U);
  for (std::size_t step = 0; step < STEP_DX.size(); ++step) {
    EXPECT_TRUE(refuses(moves[0], grid.index(10, 7), step)) << "step " << step;
  }
  // The way into it is refused too, from every side.
  EXPECT_TRUE(refuses(moves[0], grid.index(9, 7), directionOf(1, 0)));
  EXPECT_TRUE(refuses(moves[0], grid.index(11, 8), directionOf(-1, -1)));
}

TEST(GateMoves, ADiagonalStepCannotSlipThroughADiagonalGate) {
  const auto grid = unitGrid();
  const Bottleneck gate{.first = grid.index(5, 5), .second = grid.index(9, 9)};

  const auto moves = bottleneckMoves({gate}, grid);

  // From one side of the gate to the other between two of its cells.
  EXPECT_TRUE(refuses(moves[0], grid.index(6, 7), directionOf(1, -1)));
  EXPECT_TRUE(refuses(moves[0], grid.index(7, 6), directionOf(-1, 1)));
}

TEST(GateMoves, AStepThatCrossesNoGateIsOpen) {
  const auto grid = unitGrid();
  const Bottleneck gate{.first = grid.index(10, 5),
                        .second = grid.index(10, 9)};

  const auto moves = bottleneckMoves({gate}, grid);

  EXPECT_FALSE(refuses(moves[0], grid.index(2, 2), directionOf(1, 0)));
  // Beside the gate, along it, nothing is crossed.
  EXPECT_FALSE(refuses(moves[0], grid.index(8, 7), directionOf(0, 1)));
}

TEST(GateMoves, EveryRefusedStepIsRefusedBackAsWell) {
  const auto grid = unitGrid();
  const auto moves = bottleneckMoves(
      {{.first = grid.index(3, 14), .second = grid.index(8, 11)},
       {.first = grid.index(12, 2), .second = grid.index(17, 2)}},
      grid);

  ASSERT_EQ(moves.size(), 2U);
  for (const auto& gate : moves) {
    for (const auto& move : gate) {
      const auto x =
          static_cast<int>(move.cell % grid.width) + STEP_DX[move.direction];
      const auto y =
          static_cast<int>(move.cell / grid.width) + STEP_DY[move.direction];
      const auto back =
          directionOf(-STEP_DX[move.direction], -STEP_DY[move.direction]);
      EXPECT_TRUE(refuses(gate,
                          grid.index(static_cast<std::uint32_t>(x),
                                     static_cast<std::uint32_t>(y)),
                          back));
    }
  }
}

} // namespace
} // namespace mqt::scpd::grid
