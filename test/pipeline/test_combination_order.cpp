/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// The fixed order of the conflict-directed stage of the global SAT search.

#include "mqt-scpd/pipeline/CombinationOrder.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <set>
#include <string>
#include <vector>

namespace mqt::scpd::pipeline {
namespace {

TEST(CombinationOrder, FewerMovedChainsThenSmallerStatesThenChainOrder) {
  // Two moved chains come after one, whatever their states.
  EXPECT_TRUE(triedBefore({0, 7, 0}, {1, 1, 0}));
  // On the same number of moved chains, the smaller sum of the states.
  EXPECT_TRUE(triedBefore({2, 0, 1}, {0, 2, 2}));
  // On the same sum, the chain with the smaller index, then its state.
  EXPECT_TRUE(triedBefore({2, 0, 0}, {0, 2, 0}));
  EXPECT_TRUE(triedBefore({1, 2, 0}, {2, 1, 0}));
  // The order is strict: no combination comes before itself.
  EXPECT_FALSE(triedBefore({1, 2, 0}, {1, 2, 0}));
}

TEST(CombinationOrder, TheRoundStartsFromTheBestFailedCombination) {
  // Three chains of three states each. Check 2 found less against it than
  // check 1 and names chains 0 and 2; it moves chain 1 already.
  const std::vector<FailedCombination> failed{
      {.pick = {0, 2, 0}, .against = 3, .named = {0}, .check = 1},
      {.pick = {0, 1, 0}, .against = 1, .named = {2, 0, 1}, .check = 2}};
  std::vector<std::uint32_t> asked;
  const auto round = nextCombinationRound(failed,
                                          [&](const std::uint32_t chain) {
                                            asked.push_back(chain);
                                            return std::size_t{3};
                                          },
                                          {});
  ASSERT_TRUE(round.has_value());
  EXPECT_EQ(round->base.check, 2U);
  EXPECT_EQ(round->movable, (std::vector<std::uint32_t>{0, 2}));
  // Chain 1 stands on a state already and is not asked about.
  EXPECT_EQ(asked, (std::vector<std::uint32_t>{0, 2}));
  const std::vector<Combination> expected{
      {1, 1, 0}, {0, 1, 1}, {2, 1, 0}, {0, 1, 2}};
  EXPECT_EQ(round->picks, expected);
  EXPECT_TRUE(std::ranges::is_sorted(round->picks, triedBefore));
}

TEST(CombinationOrder, ACombinationTriedBeforeIsNotTriedAgain) {
  const std::vector<FailedCombination> failed{
      {.pick = {0, 0}, .against = 1, .named = {1}, .check = 0}};
  const std::set<Combination> tried{{0, 1}};
  const auto round = nextCombinationRound(
      failed, [](std::uint32_t) { return std::size_t{3}; }, tried);
  ASSERT_TRUE(round.has_value());
  EXPECT_EQ(round->picks, (std::vector<Combination>{{0, 2}}));
}

TEST(CombinationOrder, ABaseWithNothingToMoveGivesWayToTheNext) {
  // The best failed combination names only a chain with no other state and
  // a chain it moves already; the next one names a chain that can move.
  const std::vector<FailedCombination> failed{
      {.pick = {1, 0, 0}, .against = 1, .named = {0, 1}, .check = 4},
      {.pick = {2, 0, 0}, .against = 2, .named = {2}, .check = 5}};
  const auto round = nextCombinationRound(failed,
                                          [](const std::uint32_t chain) {
                                            return chain == 1 ? std::size_t{1}
                                                              : std::size_t{2};
                                          },
                                          {});
  ASSERT_TRUE(round.has_value());
  EXPECT_EQ(round->base.check, 5U);
  EXPECT_EQ(round->picks, (std::vector<Combination>{{2, 0, 1}}));
}

TEST(CombinationOrder, NoRoundWhenNoChainCanMove) {
  const std::vector<FailedCombination> failed{
      {.pick = {0, 0}, .against = 1, .named = {}, .check = 0},
      {.pick = {0, 1}, .against = 1, .named = {1}, .check = 1}};
  EXPECT_FALSE(nextCombinationRound(
                   failed, [](std::uint32_t) { return std::size_t{4}; }, {})
                   .has_value());
}

TEST(CombinationOrder, TheRoundIsTheSameWhateverTheOrderOfTheFailures) {
  std::vector<FailedCombination> failed{
      {.pick = {0, 0, 0, 0}, .against = 2, .named = {1, 3}, .check = 0},
      {.pick = {1, 0, 0, 0}, .against = 2, .named = {2, 3}, .check = 1},
      {.pick = {2, 0, 0, 0}, .against = 5, .named = {1}, .check = 2}};
  const auto states = [](const std::uint32_t chain) {
    return std::size_t{chain + 2U};
  };
  const auto first = nextCombinationRound(failed, states, {});
  std::ranges::reverse(failed);
  const auto second = nextCombinationRound(failed, states, {});
  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());
  // The commit and check 1 found as much; the earlier check is the base.
  EXPECT_EQ(first->base.check, 0U);
  EXPECT_EQ(second->base.check, 0U);
  EXPECT_EQ(first->picks, second->picks);
}

/// Three chains: chain 0 with the terminal edge f0 and the feedline edge
/// f1, chain 1 with f2, chain 2 with f3; one coupler each, chain 0's pad at
/// (10, 10), chain 1's at (100, 10), chain 2's at (10, 100).
ChipChains threeChains() {
  ChipChains chip;
  chip.chainOfFeedline = {{"f0", 0}, {"f1", 0}, {"f2", 1}, {"f3", 2}};
  chip.wiresOfChain = {{"5"}, {"6"}, {"7"}};
  chip.couplers = {{{10.0, 10.0}, 0}, {{100.0, 10.0}, 1}, {{10.0, 100.0}, 2}};
  return chip;
}

TEST(CombinationOrder, AGateAtATerminalEdgeNamesItsChain) {
  // 17q at end-edge weight 0.1: the gate over runs from wire 9's port to
  // "f0 at its coupler", and f0 is chain 0's terminal edge.
  const CheckFinding finding{.touching = {"f0"},
                             .against = {"g112"},
                             .overGates = {{60.0, 60.0}},
                             .named = {}};
  EXPECT_EQ(namedChains(finding, threeChains()), (std::set<std::uint32_t>{0}));
}

TEST(CombinationOrder, LostWiresAndOtherRulesNameTheirChains) {
  const CheckFinding finding{
      .touching = {"f2"}, .against = {"7"}, .overGates = {}, .named = {0}};
  EXPECT_EQ(namedChains(finding, threeChains()),
            (std::set<std::uint32_t>{0, 1, 2}));
}

TEST(CombinationOrder, AGateThatTouchesNoFeedlineNamesTheNearestCoupler) {
  // Nothing else names a chain: each gate over names the chain of the pad
  // nearest the middle of its line.
  const CheckFinding finding{.touching = {},
                             .against = {"g7", "g8"},
                             .overGates = {{90.0, 20.0}, {15.0, 90.0}},
                             .named = {}};
  EXPECT_EQ(namedChains(finding, threeChains()),
            (std::set<std::uint32_t>{1, 2}));
}

TEST(CombinationOrder, TheNearestCouplerIsUsedOnlyWhenNothingElseNames) {
  const CheckFinding finding{.touching = {"f3"},
                             .against = {"g7"},
                             .overGates = {{90.0, 20.0}},
                             .named = {}};
  EXPECT_EQ(namedChains(finding, threeChains()), (std::set<std::uint32_t>{2}));
}

TEST(CombinationOrder, ATieBetweenPadsNamesTheSmallerChain) {
  auto chip = threeChains();
  chip.couplers = {{{0.0, 10.0}, 2}, {{20.0, 10.0}, 1}};
  const CheckFinding finding{.touching = {},
                             .against = {"g1"},
                             .overGates = {{10.0, 10.0}},
                             .named = {}};
  EXPECT_EQ(namedChains(finding, chip), (std::set<std::uint32_t>{1}));
}

} // namespace
} // namespace mqt::scpd::pipeline
