/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/routing/CrossingConstraints.hpp"
#include "mqt-scpd/routing/Heading.hpp"
#include "mqt-scpd/routing/Path.hpp"
#include "mqt-scpd/routing/PathGeometry.hpp"
#include "mqt-scpd/routing/Primitives.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <vector>

namespace {

using namespace mqt::scpd::routing;

constexpr uint32_t WIDTH = 200;
constexpr uint32_t HEIGHT = 200;

/// A straight wire from (x, y0) to (x, y1 - 1), traveling toward positive y.
Path verticalWire(const uint32_t x, const uint32_t y0, const uint32_t y1) {
  Path wire;
  for (uint32_t y = y0; y < y1; ++y) {
    wire.push_back({.x = x, .y = y, .heading = 4, .primitive = 0});
  }
  return wire;
}

/// A straight wire from (x, y) on heading 5, toward positive x and positive y,
/// of @p length cells.
Path diagonalWire(const uint32_t x, const uint32_t y, const uint32_t length) {
  Path wire;
  for (uint32_t k = 0; k < length; ++k) {
    wire.push_back({.x = x + k, .y = y + k, .heading = 5, .primitive = 0});
  }
  return wire;
}

/// Appends one move from @p state to @p wire in the format of Path, and moves
/// @p state to the end of the move.
void appendMove(Path& wire, PathPoint& state, const Primitive& move) {
  PathPoint start = state;
  start.primitive = move.id;
  if (!wire.empty() && wire.back().samePlace(start)) {
    wire.back() = start;
  } else {
    wire.push_back(start);
  }
  for (const CellOffset& offset : move.swept) {
    const PathPoint cell{
        .x = static_cast<uint32_t>(static_cast<int32_t>(state.x) + offset.dx),
        .y = static_cast<uint32_t>(static_cast<int32_t>(state.y) + offset.dy),
        .heading = state.heading,
        .primitive = move.id};
    if (!wire.back().samePlace(cell)) {
      wire.push_back(cell);
    }
  }
  state.x = static_cast<uint32_t>(static_cast<int32_t>(state.x) + move.dx);
  state.y = static_cast<uint32_t>(static_cast<int32_t>(state.y) + move.dy);
  state.heading = move.exitHeading;
}

/// A feedline in the format of Path: a run east along row @p y that ends on
/// column @p x, the turn @p turn from there, and a run of forty steps on the
/// exit heading of the turn.
Path feedlineWithTurn(const MovePrimitives& primitives, const Primitive& turn,
                      const uint32_t x, const uint32_t y) {
  Path wire;
  PathPoint state{.x = x - 40, .y = y, .heading = 6, .primitive = 0};
  while (state.x < x) {
    appendMove(wire, state, *primitives.find(6, primitives.straight(6)));
  }
  appendMove(wire, state, turn);
  for (int step = 0; step < 40; ++step) {
    appendMove(
        wire, state,
        *primitives.find(state.heading, primitives.straight(state.heading)));
  }
  state.primitive = primitives.straight(state.heading);
  wire.push_back(state);
  return wire;
}

TEST(CrossingConstraints, WithoutFeedlinesEveryCellIsAllowed) {
  const CrossingConstraints constraints;
  EXPECT_TRUE(constraints.empty());
  EXPECT_TRUE(constraints.allowed(10, 10, 0));
  EXPECT_TRUE(constraints.turnAllowed(10, 10));
  EXPECT_EQ(constraints.maskAt(10, 10), 0U);
}

TEST(CrossingConstraints, AStraightRunMayOnlyBeCrossedAtARightAngle) {
  CrossingConstraints constraints;
  constraints.build(WIDTH, HEIGHT, {verticalWire(100, 20, 180)}, {false}, 5);
  EXPECT_FALSE(constraints.empty());

  // On the wire and within the radius, the mask holds the wire's heading.
  EXPECT_EQ(constraints.maskAt(100, 100), 1U << 4U);
  EXPECT_EQ(constraints.maskAt(105, 100), 1U << 4U);
  // Traveling east or west crosses it at a right angle.
  EXPECT_TRUE(constraints.allowed(100, 100, 6));
  EXPECT_TRUE(constraints.allowed(100, 100, 2));
  // Along it, or at forty-five degrees, is refused.
  EXPECT_FALSE(constraints.allowed(100, 100, 4));
  EXPECT_FALSE(constraints.allowed(100, 100, 0));
  EXPECT_FALSE(constraints.allowed(100, 100, 5));
  // Beyond the radius nothing is constrained.
  EXPECT_EQ(constraints.maskAt(106, 100), 0U);
  EXPECT_TRUE(constraints.allowed(106, 100, 5));
}

TEST(CrossingConstraints, ADiagonalRunMayOnlyBeCrossedAtARightAngle) {
  // A feedline from (30, 30) to (169, 169), through (100, 100).
  CrossingConstraints constraints;
  constraints.build(WIDTH, HEIGHT, {diagonalWire(30, 30, 140)}, {false}, 5);
  EXPECT_EQ(constraints.maskAt(100, 100), 1U << 5U);
  EXPECT_EQ(constraints.maskAt(104, 100), 1U << 5U);
  // The other two diagonal headings cross it at a right angle.
  EXPECT_TRUE(constraints.allowed(100, 100, 3));
  EXPECT_TRUE(constraints.allowed(104, 100, 7));
  // The cardinal headings cross it at forty-five degrees, and the headings
  // along it do not cross it at all.
  for (const Heading heading : std::array<Heading, 6>{0, 2, 4, 6, 1, 5}) {
    EXPECT_FALSE(constraints.allowed(100, 100, heading)) << int{heading};
    EXPECT_FALSE(constraints.allowed(104, 100, heading)) << int{heading};
  }
  EXPECT_FALSE(constraints.turnAllowed(100, 100));
}

TEST(CrossingConstraints, OnlyTheThreeLowBitsOfAHeadingCount) {
  CrossingConstraints constraints;
  constraints.build(WIDTH, HEIGHT, {verticalWire(100, 20, 180)}, {false}, 5);
  for (Heading heading = 0; heading < NUM_HEADINGS; ++heading) {
    for (const uint32_t turns : {1U, 2U, 31U}) {
      const auto same = static_cast<Heading>(heading + (NUM_HEADINGS * turns));
      EXPECT_EQ(constraints.allowed(100, 100, same),
                constraints.allowed(100, 100, heading))
          << int{same};
    }
  }
}

TEST(CrossingConstraints, ATurnMayNotTouchAConstrainedCell) {
  CrossingConstraints constraints;
  constraints.build(WIDTH, HEIGHT, {verticalWire(100, 20, 180)}, {false}, 5);
  // An arc changes its heading along its length, so no heading makes a turn
  // safe in the zone of a straight run, a bend or an end.
  EXPECT_FALSE(constraints.turnAllowed(100, 100));
  EXPECT_FALSE(constraints.turnAllowed(105, 100));
  EXPECT_FALSE(constraints.turnAllowed(100, 25));
  EXPECT_TRUE(constraints.turnAllowed(106, 100));
  EXPECT_FALSE(constraints.turnAllowed(WIDTH, 10));
  constraints.clear();
  EXPECT_TRUE(constraints.turnAllowed(100, 100));
}

TEST(CrossingConstraints, TheEndsOfAFeedlineCannotBeCrossed) {
  CrossingConstraints constraints;
  constraints.build(WIDTH, HEIGHT, {verticalWire(100, 20, 180)}, {false}, 5);
  // The first and the last ten cells, and their neighbors, are closed.
  EXPECT_EQ(constraints.maskAt(100, 25), CrossingConstraints::CURVE_ZONE);
  EXPECT_EQ(constraints.maskAt(101, 25), CrossingConstraints::CURVE_ZONE);
  EXPECT_EQ(constraints.maskAt(100, 175), CrossingConstraints::CURVE_ZONE);
  EXPECT_FALSE(constraints.allowed(100, 25, 6));
  EXPECT_FALSE(constraints.allowed(100, 175, 2));
}

TEST(CrossingConstraints, ATurnCannotBeCrossed) {
  // Toward positive x along y = 50, a quarter turn onto heading 4, then
  // toward positive y along x = 105.
  const MovePrimitives primitives(5);
  const Primitive* turn = nullptr;
  for (const Primitive& move : primitives.of(6)) {
    if (move.exitHeading == 4) {
      turn = &move;
    }
  }
  ASSERT_NE(turn, nullptr);
  const Path wire = feedlineWithTurn(primitives, *turn, 100, 50);
  CrossingConstraints constraints;
  constraints.build(WIDTH, HEIGHT, {wire}, {false}, 5);
  // Every cell of the turn, from the start of its arc to its end, is closed.
  for (const PathPoint& cell : wire) {
    if (cell.x >= 100 && cell.y <= 55 && cell.x <= 105) {
      EXPECT_EQ(constraints.maskAt(cell.x, cell.y),
                CrossingConstraints::CURVE_ZONE)
          << cell.x << "," << cell.y;
    }
  }
  EXPECT_FALSE(constraints.allowed(100, 50, 0));
  EXPECT_FALSE(constraints.allowed(105, 55, 6));
  // Away from the turn and the ends, both legs are straight runs.
  EXPECT_EQ(constraints.maskAt(80, 50), 1U << 6U);
  EXPECT_EQ(constraints.maskAt(105, 75), 1U << 4U);
}

TEST(CrossingConstraints, NoStraightWireCrossesTheBendingPartOfATurn) {
  // A turn first sweeps cells straight ahead, but its rendered curve bends
  // from its start. A north-south wire therefore finds no column from the
  // start of the arc to its end in which every cell admits it.
  struct Case {
    uint32_t radius;
    int expandRadius;
  };
  for (const Case c : {Case{.radius = 5, .expandRadius = 1},
                       Case{.radius = 23, .expandRadius = 10}}) {
    const MovePrimitives primitives(c.radius);
    for (const Primitive& turn : primitives.of(6)) {
      if (turn.exitHeading == 6) {
        continue;
      }
      const Path wire = feedlineWithTurn(primitives, turn, 100, 100);
      CrossingConstraints constraints;
      constraints.build(WIDTH, HEIGHT, {wire}, {false}, c.expandRadius);
      for (int dx = 0; dx <= std::abs(turn.dx); ++dx) {
        const auto x = static_cast<uint32_t>(100 + dx);
        bool closed = false;
        for (uint32_t y = 0; y < HEIGHT && !closed; ++y) {
          closed =
              !constraints.allowed(x, y, 0) || !constraints.allowed(x, y, 4);
        }
        EXPECT_TRUE(closed) << "radius " << c.radius << ", turn " << turn.id
                            << ", column +" << dx;
      }
    }
  }
}

TEST(CrossingConstraints, AnArcThatStartsOnTheSourceStubStartsTheTurnThere) {
  // The turn tag starts at the end of the source stub, so the cells beside
  // that cell are closed.
  const MovePrimitives primitives(5);
  const Primitive* turn = nullptr;
  for (const Primitive& move : primitives.of(6)) {
    if (move.exitHeading == 5) {
      turn = &move;
    }
  }
  ASSERT_NE(turn, nullptr);
  Path wire = feedlineWithTurn(primitives, *turn, 100, 100);
  std::size_t arcStart = 0;
  while (wire[arcStart].x != 100) {
    ++arcStart;
  }
  ASSERT_EQ(wire[arcStart].primitive, turn->id);
  CrossingConstraints constraints;
  constraints.build(WIDTH, HEIGHT, {wire}, {false}, 1);
  EXPECT_FALSE(constraints.turnAllowed(100, 100));
  EXPECT_EQ(constraints.maskAt(99, 99), CrossingConstraints::CURVE_ZONE);
  EXPECT_EQ(constraints.maskAt(99, 101), CrossingConstraints::CURVE_ZONE);
  EXPECT_FALSE(constraints.allowed(99, 101, 4));
}

TEST(CrossingConstraints, ASkippedFeedlineAddsNoConstraint) {
  CrossingConstraints constraints;
  constraints.build(WIDTH, HEIGHT,
                    {verticalWire(50, 20, 180), verticalWire(150, 20, 180)},
                    {true, false}, 5);
  EXPECT_EQ(constraints.maskAt(50, 100), 0U);
  EXPECT_EQ(constraints.maskAt(150, 100), 1U << 4U);
}

TEST(CrossingConstraints, AnEmptyFeedlineAddsNoConstraint) {
  CrossingConstraints constraints;
  constraints.build(WIDTH, HEIGHT, {Path{}, verticalWire(150, 20, 180)},
                    {false, false}, 5);
  EXPECT_EQ(constraints.maskAt(150, 100), 1U << 4U);
  EXPECT_EQ(constraints.maskAt(50, 100), 0U);
  EXPECT_TRUE(constraints.allowed(50, 100, 4));
}

TEST(CrossingConstraints,
     AFeedlineAlongTheBorderConstrainsTheCellsInsideTheGrid) {
  // The wire runs along the first column and starts in the corner, so both
  // the zone around its straight run and the zone around its end reach past
  // the edge of the grid.
  CrossingConstraints constraints;
  constraints.build(WIDTH, HEIGHT, {verticalWire(0, 0, 180)}, {false}, 5);
  EXPECT_EQ(constraints.maskAt(0, 100), 1U << 4U);
  EXPECT_EQ(constraints.maskAt(5, 100), 1U << 4U);
  EXPECT_EQ(constraints.maskAt(6, 100), 0U);
  EXPECT_EQ(constraints.maskAt(0, 0), CrossingConstraints::CURVE_ZONE);
  EXPECT_EQ(constraints.maskAt(1, 0), CrossingConstraints::CURVE_ZONE);
  EXPECT_TRUE(constraints.allowed(0, 100, 6));
  EXPECT_FALSE(constraints.allowed(0, 100, 4));
}

TEST(CrossingConstraints, ACellListedTwiceOnAStraightRunIsNoBend) {
  // The test for a straight run looks at the next cell somewhere else, so a
  // cell that appears twice in a row keeps its place on the run.
  Path wire = verticalWire(100, 20, 180);
  const PathPoint repeated = wire[80];
  wire.insert(wire.begin() + 80, repeated);
  CrossingConstraints constraints;
  constraints.build(WIDTH, HEIGHT, {wire}, {false}, 5);
  EXPECT_EQ(constraints.maskAt(100, 100), 1U << 4U);
  EXPECT_EQ(constraints.maskAt(101, 100), 1U << 4U);
  EXPECT_TRUE(constraints.allowed(100, 100, 6));
}

TEST(CrossingConstraints, ACellOutsideTheGridIsNotAllowed) {
  CrossingConstraints constraints;
  constraints.build(WIDTH, HEIGHT, {verticalWire(100, 20, 180)}, {false}, 5);
  EXPECT_FALSE(constraints.allowed(WIDTH, 10, 6));
  EXPECT_EQ(constraints.maskAt(WIDTH, 10), 0U);
}

TEST(CrossingConstraints, PrimitiveTablesIdentifyATerminalTurn) {
  const MovePrimitives table(23);
  const Primitive* turn = nullptr;
  for (const Primitive& move : table.of(0)) {
    if (move.exitHeading == 2) {
      turn = &move;
      break;
    }
  }
  ASSERT_NE(turn, nullptr);
  Path feedline = straightRun(table, {.x = 100, .y = 140, .heading = 0}, 40);
  PathPoint state = feedline.back();
  appendMove(feedline, state, *turn);
  ASSERT_GT(feedline.size(), 60U);
  // Both end zones are far from the turn's origin. Its first swept steps
  // look straight, and no later tag reveals the terminal turn.
  CrossingConstraints withoutTables;
  withoutTables.build(WIDTH, HEIGHT, {feedline}, {}, 0);
  EXPECT_EQ(withoutTables.maskAt(100, 100), 1U << 0U);
  EXPECT_TRUE(withoutTables.allowed(100, 100, 2));
  CrossingConstraints withTables;
  withTables.build(WIDTH, HEIGHT, {feedline}, {}, 0, &table);
  EXPECT_EQ(withTables.maskAt(100, 100), CrossingConstraints::CURVE_ZONE);
  EXPECT_FALSE(withTables.allowed(100, 100, 2));
  EXPECT_FALSE(withTables.turnAllowed(100, 100));
}

TEST(CrossingConstraints, ClearingForgetsEveryConstraint) {
  CrossingConstraints constraints;
  constraints.build(WIDTH, HEIGHT, {verticalWire(100, 20, 180)}, {false}, 5);
  constraints.clear();
  EXPECT_TRUE(constraints.empty());
  EXPECT_TRUE(constraints.allowed(100, 100, 5));
}

TEST(CrossingConstraints, ClearingReleasesTheMasks) {
  CrossingConstraints constraints;
  EXPECT_EQ(constraints.heldBytes(), 0U);
  constraints.build(WIDTH, HEIGHT, {verticalWire(100, 20, 180)}, {false}, 5);
  EXPECT_GE(constraints.heldBytes(), std::size_t{WIDTH} * HEIGHT);
  constraints.clear();
  EXPECT_EQ(constraints.heldBytes(), 0U);
}

} // namespace
