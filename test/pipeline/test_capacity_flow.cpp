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

#include <cstdint>
#include <optional>
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
