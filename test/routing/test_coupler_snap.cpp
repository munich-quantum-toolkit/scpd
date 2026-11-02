/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/routing/CouplerInsertion.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

namespace {

using namespace mqt::scpd::routing;

// A pad along y: feedline ports (102, 63) and (102, 83), the resonator's stub
// at the second port's end, on the -x side (the coupler's orientation).
const CouplerSnapPad AXIS{.inX = 102.0,
                          .inY = 63.0,
                          .outX = 102.0,
                          .outY = 83.0,
                          .anchorX = 100.0,
                          .anchorY = 83.0};

std::vector<CouplerSnapMove> movesOf(const CouplerSnapPad& pad,
                                     const std::vector<std::int32_t>& fs,
                                     const std::vector<std::int32_t>& rs) {
  return couplerSnapMoves(pad, 5, fs, rs);
}

TEST(CouplerSnap, PutsTheMiddleOfTheFeedlinePortsOnTheNearestGridPoint) {
  // The middle (102, 73) goes to (100, 75).
  const auto moves = movesOf(AXIS, {0}, {0});
  ASSERT_EQ(moves.size(), 1U);
  EXPECT_EQ(moves[0].x, -2);
  EXPECT_EQ(moves[0].y, 2);
}

TEST(CouplerSnap, FMovesAlongTheFeedlineTowardThePortWithoutTheStub) {
  // The stub sits at the second port (y 83), so f runs toward y 63.
  const auto moves = movesOf(AXIS, {20}, {0});
  ASSERT_EQ(moves.size(), 1U);
  EXPECT_EQ(moves[0].x, -2);
  EXPECT_EQ(moves[0].y, 2 - 20);
  EXPECT_EQ(moves[0].f, 20);
  EXPECT_EQ(moves[0].r, 0);
}

TEST(CouplerSnap, FTurnsWithTheMirroredPort) {
  auto mirrored = AXIS;
  mirrored.anchorY = 63.0;  // the stub at the first port's end
  const auto moves = movesOf(mirrored, {20}, {0});
  ASSERT_EQ(moves.size(), 1U);
  EXPECT_EQ(moves[0].y, 2 + 20);
}

TEST(CouplerSnap, PositiveRMovesTowardTheResonator) {
  const auto moves = movesOf(AXIS, {0}, {20, -20});
  ASSERT_EQ(moves.size(), 2U);
  EXPECT_EQ(moves[0].x, -2 - 20);  // toward the stub's side, -x
  EXPECT_EQ(moves[0].r, 20);
  EXPECT_EQ(moves[1].x, -2 + 20);
  EXPECT_EQ(moves[1].r, -20);
  EXPECT_EQ(moves[1].y, 2);
}

TEST(CouplerSnap, ADiagonalPadMovesInWholeDiagonalGridSteps) {
  // Ports (100, 100) and (114, 114), middle (107, 107) -> (105, 105); the
  // stub at the second port, on the side of (+1, -1). Twenty cells along the
  // diagonal are three diagonal steps of (5, 5).
  const CouplerSnapPad diagonal{.inX = 100.0,
                                .inY = 100.0,
                                .outX = 114.0,
                                .outY = 114.0,
                                .anchorX = 116.0,
                                .anchorY = 112.0};
  const auto moves = movesOf(diagonal, {0, 20}, {20});
  ASSERT_EQ(moves.size(), 2U);
  // r = 20 only: three steps toward the resonator, (+15, -15).
  EXPECT_EQ(moves[0].x, -2 + 15);
  EXPECT_EQ(moves[0].y, -2 - 15);
  // f = 20 as well: three steps toward the first port, (-15, -15).
  EXPECT_EQ(moves[1].x, -2 + 15 - 15);
  EXPECT_EQ(moves[1].y, -2 - 15 - 15);
}

TEST(CouplerSnap, TakesEveryPairOfFAndRInOrderAndDropsRepeats) {
  const auto moves = movesOf(AXIS, {0, 20, 20}, {0, 20, -20});
  ASSERT_EQ(moves.size(), 6U);
  EXPECT_EQ(moves[0].f, 0);
  EXPECT_EQ(moves[0].r, 0);
  EXPECT_EQ(moves[2].r, -20);
  EXPECT_EQ(moves[3].f, 20);
  EXPECT_EQ(moves[3].r, 0);
}

}  // namespace
