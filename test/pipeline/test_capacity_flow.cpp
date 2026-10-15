/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// The capacity check of a graph of chambers, on graphs small enough that the
// answer is known.

#include "mqt-scpd/milp/Model.hpp"
#include "mqt-scpd/pipeline/CapacityFlow.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <vector>

namespace mqt::scpd::pipeline {
namespace {

FlowDemand between(const std::uint32_t from, const std::uint32_t to) {
  return {.from = {from}, .to = {to}};
}

TEST(CapacityFlow, TwoWiresThroughAGateForOneAreShortByOne) {
  const std::vector<FlowEdge> edges = {{.chambers = {0, 1}, .capacity = 1}};
  const auto check =
      checkCapacity(2, edges, {between(0, 1), between(0, 1)}, {});

  EXPECT_EQ(check.status, milp::SolveStatus::Optimal);
  EXPECT_EQ(check.shortBy, 1U);
  EXPECT_EQ(check.load[0], 2U);
  EXPECT_EQ(check.overflow[0], 1U);
}

TEST(CapacityFlow, TwoWiresThroughAGateForTwoFit) {
  const std::vector<FlowEdge> edges = {{.chambers = {0, 1}, .capacity = 2}};
  const auto check =
      checkCapacity(2, edges, {between(0, 1), between(0, 1)}, {});

  EXPECT_EQ(check.shortBy, 0U);
  EXPECT_EQ(check.overflow[0], 0U);
  ASSERT_TRUE(check.ways[0].has_value());
  EXPECT_EQ(*check.ways[0], std::vector<std::uint32_t>{0});
}

TEST(CapacityFlow, SpreadsWiresOverTwoGatesThatTakeOneEach) {
  // Two ways from 0 to 2: straight through gate 0, or round through gates
  // 1 and 2. Fewest gates first would send both wires through gate 0.
  const std::vector<FlowEdge> edges = {{.chambers = {0, 2}, .capacity = 1},
                                       {.chambers = {0, 1}, .capacity = 1},
                                       {.chambers = {1, 2}, .capacity = 1}};
  const auto check =
      checkCapacity(3, edges, {between(0, 2), between(0, 2)}, {});

  EXPECT_EQ(check.shortBy, 0U);
  EXPECT_EQ(check.load[0], 1U);
  EXPECT_EQ(check.load[1], 1U);
  EXPECT_EQ(check.load[2], 1U);
}

TEST(CapacityFlow, AnEdgeOpenToOneWireIsNoWayForAnother) {
  const std::vector<FlowEdge> edges = {
      {.chambers = {0, 1},
       .capacity = 5,
       .users = std::vector<std::uint32_t>{1}}};
  const auto check =
      checkCapacity(2, edges, {between(0, 1), between(0, 1)}, {});

  EXPECT_FALSE(check.ways[0].has_value());
  ASSERT_TRUE(check.ways[1].has_value());
  EXPECT_EQ(*check.ways[1], std::vector<std::uint32_t>{0});
  EXPECT_EQ(check.load[0], 1U);
}

TEST(CapacityFlow, AGateBetweenThreeChambersJoinsAllOfThem) {
  const std::vector<FlowEdge> edges = {{.chambers = {0, 1, 2}, .capacity = 1}};
  const auto check = checkCapacity(3, edges, {between(0, 2)}, {});

  ASSERT_TRUE(check.ways[0].has_value());
  EXPECT_EQ(*check.ways[0], std::vector<std::uint32_t>{0});
  EXPECT_EQ(check.shortBy, 0U);
}

TEST(CapacityFlow, AWireThatStartsWhereItEndsPassesNothing) {
  const std::vector<FlowEdge> edges = {{.chambers = {0, 1}, .capacity = 0}};
  const auto check = checkCapacity(2, edges, {{.from = {0, 1}, .to = {1}}}, {});

  ASSERT_TRUE(check.ways[0].has_value());
  EXPECT_TRUE(check.ways[0]->empty());
  EXPECT_EQ(check.shortBy, 0U);
}

TEST(InTurn, TheSecondWireThroughAGateForOneOverfillsIt) {
  // Together they do not fit, and nothing leads round the gate.
  const std::vector<FlowEdge> edges = {{.chambers = {0, 1}, .capacity = 1}};
  const auto check = checkInTurn(2, edges, {between(0, 1), between(0, 1)});

  for (const auto& way : check.ways) {
    ASSERT_TRUE(way.has_value());
    EXPECT_EQ(*way, std::vector<std::uint32_t>{0});
  }
  EXPECT_EQ(check.load[0], 2U);
  EXPECT_EQ(check.overflow[0], 1U);
  EXPECT_EQ(check.shortBy, 1U);
}

TEST(InTurn, TheSecondWireTakesTheWayRoundAFullGate) {
  const std::vector<FlowEdge> edges = {{.chambers = {0, 2}, .capacity = 1},
                                       {.chambers = {0, 1}, .capacity = 1},
                                       {.chambers = {1, 2}, .capacity = 1}};
  const auto check = checkInTurn(3, edges, {between(0, 2), between(0, 2)});

  ASSERT_TRUE(check.ways[0].has_value());
  EXPECT_EQ(*check.ways[0], std::vector<std::uint32_t>{0});
  ASSERT_TRUE(check.ways[1].has_value());
  EXPECT_EQ(*check.ways[1], (std::vector<std::uint32_t>{1, 2}));
  EXPECT_EQ(check.shortBy, 0U);
}

/// A wire routed first takes the one gate a later wire has no way round;
/// the sweep after takes the first off that gate and round the other way,
/// as the feedline routing rips up and reroutes (user, 2026-10-14).
TEST(InTurn, TakesAnEarlierWireOffAFullGateForOneWithNoOtherWay) {
  const std::vector<FlowEdge> edges = {
      {.chambers = {0, 2}, .capacity = 1},
      {.chambers = {0, 1},
       .capacity = 1,
       .users = std::vector<std::uint32_t>{0}},
      {.chambers = {1, 2},
       .capacity = 1,
       .users = std::vector<std::uint32_t>{0}}};
  const auto check = checkInTurn(3, edges, {between(0, 2), between(0, 2)});

  ASSERT_TRUE(check.ways[0].has_value());
  EXPECT_EQ(*check.ways[0], (std::vector<std::uint32_t>{1, 2}));
  ASSERT_TRUE(check.ways[1].has_value());
  EXPECT_EQ(*check.ways[1], std::vector<std::uint32_t>{0});
  EXPECT_EQ(check.shortBy, 0U);
  EXPECT_GE(check.sweeps, 2U);
}

/// A wire's detour is measured against its shortest way over the edges
/// that take a wire at all (user, 2026-10-14): a gate that takes none is
/// no way to measure by, so the way round it, however long, is the way the
/// wire takes, and the gate stays empty.
TEST(InTurn, MeasuresTheDetourAgainstTheShortestWayOverGatesThatTakeAWire) {
  // Straight from 0 to 3 through a gate for no wire, or round through 1
  // and 2 at more than one and a half times the length.
  const std::vector<FlowEdge> edges = {
      {.chambers = {0, 3}, .capacity = 0, .from = {5, 0}, .to = {5, 1}},
      {.chambers = {0, 1}, .capacity = 1, .from = {0, 10}, .to = {1, 10}},
      {.chambers = {1, 2}, .capacity = 1, .from = {10, 20}, .to = {10, 21}},
      {.chambers = {2, 3}, .capacity = 1, .from = {20, 10}, .to = {21, 10}}};
  const FlowDemand wanted{
      .from = {0}, .to = {3}, .start = {0, 0}, .end = {10, 0}};
  const auto check = checkInTurn(4, edges, {wanted}, 1.5);

  ASSERT_TRUE(check.ways[0].has_value());
  EXPECT_EQ(*check.ways[0], (std::vector<std::uint32_t>{1, 2, 3}));
  EXPECT_EQ(check.shortBy, 0U);
}

/// The rules of an earlier round, asked for by `InTurnRules`: with one
/// sweep nothing is ripped up, so the first wire keeps the gate the second
/// has no way round, and the second overfills it.
TEST(InTurn, StopsAfterTheFirstSweepWhenAskedTo) {
  const std::vector<FlowEdge> edges = {
      {.chambers = {0, 2}, .capacity = 1},
      {.chambers = {0, 1},
       .capacity = 1,
       .users = std::vector<std::uint32_t>{0}},
      {.chambers = {1, 2},
       .capacity = 1,
       .users = std::vector<std::uint32_t>{0}}};
  const auto check = checkInTurn(3, edges, {between(0, 2), between(0, 2)}, 0.0,
                                 {.mostSweeps = 1, .detourOverTakers = true});

  EXPECT_EQ(check.sweeps, 1U);
  EXPECT_EQ(check.shortBy, 1U);
  ASSERT_TRUE(check.ways[0].has_value());
  EXPECT_EQ(*check.ways[0], std::vector<std::uint32_t>{0});
}

/// Measured against the shortest way over any gate, as before round 25,
/// the way round a gate for no wire is too long a detour, and the wire
/// takes the gate and overfills it.
TEST(InTurn, MeasuresTheDetourAgainstAnyGateWhenAskedTo) {
  const std::vector<FlowEdge> edges = {
      {.chambers = {0, 3}, .capacity = 0, .from = {5, 0}, .to = {5, 1}},
      {.chambers = {0, 1}, .capacity = 1, .from = {0, 10}, .to = {1, 10}},
      {.chambers = {1, 2}, .capacity = 1, .from = {10, 20}, .to = {10, 21}},
      {.chambers = {2, 3}, .capacity = 1, .from = {20, 10}, .to = {21, 10}}};
  const FlowDemand wanted{
      .from = {0}, .to = {3}, .start = {0, 0}, .end = {10, 0}};
  const auto check = checkInTurn(4, edges, {wanted}, 1.5,
                                 {.mostSweeps = 8, .detourOverTakers = false});

  ASSERT_TRUE(check.ways[0].has_value());
  EXPECT_EQ(*check.ways[0], std::vector<std::uint32_t>{0});
  EXPECT_EQ(check.shortBy, 1U);
}

TEST(InTurn, AGateThatTakesNoWireIsOverfilledWhenNothingLeadsRound) {
  const std::vector<FlowEdge> edges = {{.chambers = {0, 1}, .capacity = 1},
                                       {.chambers = {1, 2}, .capacity = 0}};
  const auto check = checkInTurn(3, edges, {between(0, 2)});

  ASSERT_TRUE(check.ways[0].has_value());
  EXPECT_EQ(*check.ways[0], (std::vector<std::uint32_t>{0, 1}));
  EXPECT_EQ(check.overflow[0], 0U);
  EXPECT_EQ(check.overflow[1], 1U);
  EXPECT_EQ(check.shortBy, 1U);
}

TEST(InTurn, AnEdgeOpenToOneWireIsNoWayForAnother) {
  const std::vector<FlowEdge> edges = {
      {.chambers = {0, 1},
       .capacity = 5,
       .users = std::vector<std::uint32_t>{1}}};
  const auto check = checkInTurn(2, edges, {between(0, 1), between(0, 1)});

  EXPECT_FALSE(check.ways[0].has_value());
  ASSERT_TRUE(check.ways[1].has_value());
  EXPECT_EQ(check.load[0], 1U);
}

TEST(InTurn, AWireThatStartsWhereItEndsPassesNothing) {
  const auto check = checkInTurn(2, {{.chambers = {0, 1}, .capacity = 0}},
                                 {{.from = {0, 1}, .to = {1}}});

  ASSERT_TRUE(check.ways[0].has_value());
  EXPECT_TRUE(check.ways[0]->empty());
  EXPECT_EQ(check.shortBy, 0U);
}

TEST(InTurn, AWireCrossesTheFeedlineItIsPrescribed) {
  // A gate and a stretch both join 0 and 1. A wire prescribed to cross the
  // feedline takes the stretch, any other wire the gate.
  const std::vector<FlowEdge> edges = {{.chambers = {0, 1}, .capacity = 5},
                                       {.chambers = {0, 1}, .capacity = 1}};
  const auto check = checkInTurn(
      2, edges, {{.from = {0}, .to = {1}, .crossings = {{1}}}, between(0, 1)});

  ASSERT_TRUE(check.ways[0].has_value());
  EXPECT_EQ(*check.ways[0], std::vector<std::uint32_t>{1});
  ASSERT_TRUE(check.ways[1].has_value());
  EXPECT_EQ(*check.ways[1], std::vector<std::uint32_t>{0});
}

TEST(InTurn, AWireCrossesItsFeedlineEvenWhereItStartsAtItsEnd) {
  // It starts in the chamber it ends in, and still has to cross once: over
  // the stretch and back through the gate.
  const std::vector<FlowEdge> edges = {{.chambers = {0, 1}, .capacity = 1},
                                       {.chambers = {0, 1}, .capacity = 1}};
  const auto check =
      checkInTurn(2, edges, {{.from = {0}, .to = {0}, .crossings = {{1}}}});

  ASSERT_TRUE(check.ways[0].has_value());
  EXPECT_EQ(check.ways[0]->size(), 2U);
  EXPECT_EQ(std::ranges::count(*check.ways[0], 1U), 1);
  EXPECT_EQ(check.shortBy, 0U);
}

TEST(InTurn, AWireCrossesAFeedlineWithOneChamberOnBothSides) {
  // A feedline open at one end has the same chamber on both sides. A wire
  // prescribed to cross it does so from that chamber back into it, and
  // takes a place on it.
  const std::vector<FlowEdge> edges = {{.chambers = {0, 1}, .capacity = 5},
                                       {.chambers = {0, 0}, .capacity = 1}};
  const auto check =
      checkInTurn(2, edges,
                  {{.from = {0}, .to = {0}, .crossings = {{1}}},
                   {.from = {0}, .to = {1}, .crossings = {{1}}},
                   {.from = {0}, .to = {1}, .crossings = {{1}}}});

  ASSERT_TRUE(check.ways[0].has_value());
  EXPECT_EQ(*check.ways[0], std::vector<std::uint32_t>{1});
  ASSERT_TRUE(check.ways[1].has_value());
  EXPECT_EQ(std::ranges::count(*check.ways[1], 1U), 1);
  EXPECT_EQ(check.load[1], 3U);
  EXPECT_EQ(check.shortBy, 2U);
}

TEST(InTurn, AWireOfFixedLengthCrossesAFeedlineWithOneChamberOnBothSides) {
  const std::vector<FlowEdge> edges = {
      {.chambers = {0, 0}, .capacity = 1, .from = {5, 0}, .to = {5, 10}}};
  const auto check = checkInTurn(1, edges,
                                 {{.from = {0},
                                   .to = {0},
                                   .crossings = {{0}},
                                   .longest = 100.0,
                                   .start = {0, 5},
                                   .end = {10, 5}}});

  ASSERT_TRUE(check.ways[0].has_value());
  EXPECT_EQ(*check.ways[0], std::vector<std::uint32_t>{0});
}

TEST(InTurn, AWireCrossesEveryFeedlineItIsPrescribedOnce) {
  // Two feedlines between 0 and 1, and between 1 and 2.
  const std::vector<FlowEdge> edges = {{.chambers = {0, 1}, .capacity = 1},
                                       {.chambers = {1, 2}, .capacity = 1},
                                       {.chambers = {0, 2}, .capacity = 5}};
  const auto check = checkInTurn(
      3, edges, {{.from = {0}, .to = {2}, .crossings = {{0}, {1}}}});

  ASSERT_TRUE(check.ways[0].has_value());
  EXPECT_EQ(*check.ways[0], (std::vector<std::uint32_t>{0, 1}));
}

TEST(InTurn, AFeedlineWithNowhereToCrossLeavesNoWay) {
  const std::vector<FlowEdge> edges = {{.chambers = {0, 1}, .capacity = 5}};
  const auto check =
      checkInTurn(2, edges, {{.from = {0}, .to = {1}, .crossings = {{}}}});

  EXPECT_FALSE(check.ways[0].has_value());
  EXPECT_EQ(check.load[0], 0U);
}

/// Chamber 0 to chamber 2 straight through edge 0, whose middle lies far
/// off, or round through edges 1 and 2 near the line from start to end.
std::vector<FlowEdge> nearAndFar(const std::uint32_t nearCapacity) {
  return {{.chambers = {0, 2},
           .capacity = 1,
           .users = std::nullopt,
           .from = {.x = 15.0, .y = 100.0},
           .to = {.x = 15.0, .y = 100.0}},
          {.chambers = {0, 1},
           .capacity = nearCapacity,
           .users = std::nullopt,
           .from = {.x = 10.0, .y = 0.0},
           .to = {.x = 10.0, .y = 0.0}},
          {.chambers = {1, 2},
           .capacity = nearCapacity,
           .users = std::nullopt,
           .from = {.x = 20.0, .y = 0.0},
           .to = {.x = 20.0, .y = 0.0}}};
}

FlowDemand ofLength(const double longest) {
  return {.from = {0},
          .to = {2},
          .crossings = {},
          .longest = longest,
          .start = {.x = 0.0, .y = 0.0},
          .end = {.x = 30.0, .y = 0.0}};
}

TEST(InTurn, AWireOfFixedLengthTakesItsShortestWay) {
  // The fewest edges would be edge 0, 2 x 101 cells long; the way through
  // edges 1 and 2 is 30.
  const auto check = checkInTurn(3, nearAndFar(1), {ofLength(1000.0)});

  ASSERT_TRUE(check.ways[0].has_value());
  EXPECT_EQ(*check.ways[0], (std::vector<std::uint32_t>{1, 2}));
  EXPECT_DOUBLE_EQ(check.lengths[0], 30.0);
  EXPECT_FALSE(check.tooLong[0]);
}

TEST(InTurn, AWayLongerThanTheWireMayRunIsNoWay) {
  const auto check = checkInTurn(3, nearAndFar(1), {ofLength(20.0)});

  EXPECT_FALSE(check.ways[0].has_value());
  EXPECT_TRUE(check.tooLong[0]);
  EXPECT_DOUBLE_EQ(check.lengths[0], 30.0);
  EXPECT_EQ(check.load[1], 0U);
}

TEST(InTurn, AWireOfFixedLengthOverfillsAShortWayRatherThanTakeALongOne) {
  // The short way takes no wire and the long one is too long: the wire
  // takes the short way, and its edges have overflow.
  const auto check = checkInTurn(3, nearAndFar(0), {ofLength(50.0)});

  ASSERT_TRUE(check.ways[0].has_value());
  EXPECT_EQ(*check.ways[0], (std::vector<std::uint32_t>{1, 2}));
  EXPECT_FALSE(check.tooLong[0]);
  EXPECT_EQ(check.shortBy, 2U);
}

TEST(InTurn, MeasuresAWayFromCentreToCentreOfTheEdgesItPasses) {
  // One bottleneck from (10,0) to (10,100): its line touches the straight
  // way from start to end, but its centre lies 50 off it.
  const std::vector<FlowEdge> edges = {{.chambers = {0, 1},
                                        .capacity = 1,
                                        .users = std::nullopt,
                                        .from = {.x = 10.0, .y = 0.0},
                                        .to = {.x = 10.0, .y = 100.0}}};
  const FlowDemand wanted{.from = {0},
                          .to = {1},
                          .crossings = {},
                          .longest = 1000.0,
                          .start = {.x = 0.0, .y = 0.0},
                          .end = {.x = 20.0, .y = 0.0}};
  const auto check = checkInTurn(2, edges, {wanted});

  ASSERT_TRUE(check.ways[0].has_value());
  EXPECT_DOUBLE_EQ(check.lengths[0], 2.0 * std::hypot(10.0, 50.0));
}

TEST(InTurn, AWireWhoseShortWayIsFullGoesNoFurtherThanItsDetour) {
  // The short way through edges 1 and 2 is 30 long and takes one wire,
  // which the first wire is; the way round through edge 0 is 2 x 101. With
  // a detour of 1.5 the second wire takes the short way and overfills it;
  // with one of 10 it goes round.
  FlowDemand any = ofLength(0.0);
  any.longest.reset();
  const auto near = checkInTurn(3, nearAndFar(1), {any, any}, 1.5);
  ASSERT_TRUE(near.ways[1].has_value());
  EXPECT_EQ(*near.ways[1], (std::vector<std::uint32_t>{1, 2}));
  EXPECT_EQ(near.shortBy, 2U);

  const auto far = checkInTurn(3, nearAndFar(1), {any, any}, 10.0);
  ASSERT_TRUE(far.ways[0].has_value());
  EXPECT_EQ(*far.ways[0], (std::vector<std::uint32_t>{1, 2}));
  ASSERT_TRUE(far.ways[1].has_value());
  EXPECT_EQ(*far.ways[1], std::vector<std::uint32_t>{0});
  EXPECT_EQ(far.shortBy, 0U);
}

TEST(InTurn, RefusesAChamberOutOfRange) {
  EXPECT_THROW(static_cast<void>(checkInTurn(
                   2, {{.chambers = {0, 2}, .capacity = 1}}, {between(0, 1)})),
               std::invalid_argument);
}

TEST(InTurn, RefusesAnEdgeToCrossThatIsNotThere) {
  EXPECT_THROW(static_cast<void>(
                   checkInTurn(2, {{.chambers = {0, 1}, .capacity = 1}},
                               {{.from = {0}, .to = {1}, .crossings = {{3}}}})),
               std::invalid_argument);
}

TEST(GapCapacity, AGapHoldsItsLengthOverTheClearance) {
  EXPECT_EQ(wiresThroughGap(20.0, 19.0), 1U);
  EXPECT_EQ(wiresThroughGap(38.0, 19.0), 2U);
  EXPECT_EQ(wiresThroughGap(56.9, 19.0), 2U);
}

TEST(GapCapacity, AGapNarrowerThanTheClearanceHoldsNothing) {
  EXPECT_EQ(wiresThroughGap(18.0, 19.0), 0U);
  EXPECT_EQ(wiresThroughGap(0.0, 19.0), 0U);
  EXPECT_EQ(wiresThroughGap(20.0, 0.0), 0U);
}

} // namespace
} // namespace mqt::scpd::pipeline
