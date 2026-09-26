/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/routing/ChainTrellis.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <random>
#include <set>
#include <tuple>
#include <vector>

namespace {

using namespace mqt::scpd::routing;

/// A trellis whose step prices are held in a table, so a test can say what
/// the answer must be and count what the search asked for.
struct Table {
  std::vector<uint32_t> width;
  /// price[layer][from][to]
  std::vector<std::vector<std::vector<uint64_t>>> price;
  /// What the bound answers. Defaults to the price itself, i.e. a bound that
  /// is exact everywhere.
  std::vector<std::vector<std::vector<uint64_t>>> hint;
  std::vector<std::vector<uint64_t>> node;

  mutable std::set<std::tuple<std::size_t, uint32_t, uint32_t>> asked;

  [[nodiscard]] TrellisProblem problem() const {
    TrellisProblem out;
    out.width = width;
    if (!node.empty()) {
      out.nodeCost = [this](const std::size_t layer, const uint32_t which) {
        return node[layer][which];
      };
    }
    out.bound = [this](const std::size_t layer, const uint32_t from,
                       const uint32_t to) { return hint[layer][from][to]; };
    out.evaluate = [this](const std::size_t layer, const uint32_t from,
                          const uint32_t to) {
      // Every step is priced at most once. The search promises it and the
      // caller pays a real A* per call, so hold it to the promise.
      const auto fresh = asked.emplace(layer, from, to).second;
      EXPECT_TRUE(fresh) << "step " << layer << " " << from << "->" << to
                         << " priced twice";
      return price[layer][from][to];
    };
    return out;
  }

  /// The optimum by exhaustion, for the search to be held against.
  [[nodiscard]] uint64_t exhaustive() const {
    const auto layers = width.size();
    std::vector<uint64_t> best(width[0], 0);
    if (!node.empty()) {
      for (uint32_t i = 0; i < width[0]; ++i) {
        best[i] = node[0][i];
      }
    }
    for (std::size_t layer = 0; layer + 1 < layers; ++layer) {
      std::vector<uint64_t> next(width[layer + 1], TRELLIS_UNREACHABLE);
      for (uint32_t j = 0; j < width[layer + 1]; ++j) {
        for (uint32_t i = 0; i < width[layer]; ++i) {
          const auto step = price[layer][i][j];
          if (best[i] == TRELLIS_UNREACHABLE || step == TRELLIS_UNREACHABLE) {
            continue;
          }
          const auto reach =
              best[i] + step + (node.empty() ? 0 : node[layer + 1][j]);
          next[j] = reach < next[j] ? reach : next[j];
        }
      }
      best = next;
    }
    uint64_t least = TRELLIS_UNREACHABLE;
    for (const auto value : best) {
      least = value < least ? value : least;
    }
    return least;
  }

  /// The price of one assignment, for a returned answer to be checked
  /// against — a cost that does not match its own path is the worst kind of
  /// wrong.
  [[nodiscard]] uint64_t priceOf(const std::vector<uint32_t>& chosen) const {
    uint64_t total = node.empty() ? 0 : node[0][chosen[0]];
    for (std::size_t layer = 0; layer + 1 < width.size(); ++layer) {
      const auto step = price[layer][chosen[layer]][chosen[layer + 1]];
      if (step == TRELLIS_UNREACHABLE) {
        return TRELLIS_UNREACHABLE;
      }
      total += step + (node.empty() ? 0 : node[layer + 1][chosen[layer + 1]]);
    }
    return total;
  }
};

/// A trellis of `width` nodes on each of `layers` layers, every step priced
/// by the rule given, and the bound exact.
Table tableOf(const std::size_t layers, const uint32_t width,
              const std::function<uint64_t(std::size_t, uint32_t, uint32_t)>&
                  rule) {
  Table out;
  out.width.assign(layers, width);
  for (std::size_t layer = 0; layer + 1 < layers; ++layer) {
    std::vector<std::vector<uint64_t>> block(width,
                                             std::vector<uint64_t>(width, 0));
    for (uint32_t i = 0; i < width; ++i) {
      for (uint32_t j = 0; j < width; ++j) {
        block[i][j] = rule(layer, i, j);
      }
    }
    out.price.push_back(block);
  }
  out.hint = out.price;
  return out;
}

TEST(ChainTrellis, FindsTheKnownOptimum) {
  // Node 0 of every layer is dear to leave, node 1 is cheap: the optimum runs
  // along node 1 and costs 10 per step.
  auto table = tableOf(5, 2, [](std::size_t, const uint32_t i, const uint32_t j) {
    return static_cast<uint64_t>(i == 1 && j == 1 ? 10 : 100);
  });
  const auto result = solveTrellis(table.problem());
  ASSERT_TRUE(result.solved);
  EXPECT_EQ(result.cost, table.exhaustive());
  EXPECT_EQ(result.cost, table.priceOf(result.chosen));
  EXPECT_EQ(result.chosen, (std::vector<uint32_t>{1, 1, 1, 1, 1}));
}

TEST(ChainTrellis, AnExactBoundPricesOnlyTheAnswer) {
  // With the bound equal to the price everywhere, the first winner the sweep
  // names is already optimal, so only its own steps are ever priced.
  auto table = tableOf(6, 4, [](std::size_t, const uint32_t i, const uint32_t j) {
    return static_cast<uint64_t>((i * 7) + (j * 3) + 1);
  });
  const auto result = solveTrellis(table.problem());
  ASSERT_TRUE(result.solved);
  EXPECT_EQ(result.cost, table.exhaustive());
  EXPECT_EQ(result.evaluations, table.width.size() - 1);
}

TEST(ChainTrellis, AWorthlessBoundStillAnswersExactly) {
  // A bound of zero says nothing at all. The answer must not change; only the
  // number of steps priced may.
  auto table = tableOf(5, 4, [](std::size_t, const uint32_t i, const uint32_t j) {
    return static_cast<uint64_t>(((i * 13) ^ (j * 5)) + 1);
  });
  const auto exact = solveTrellis(table.problem());

  Table blind = table;
  blind.asked.clear();
  for (auto& block : blind.hint) {
    for (auto& row : block) {
      for (auto& value : row) {
        value = 0;
      }
    }
  }
  const auto result = solveTrellis(blind.problem());
  ASSERT_TRUE(result.solved);
  EXPECT_EQ(result.cost, table.exhaustive());
  EXPECT_EQ(result.cost, exact.cost);
  EXPECT_GT(result.evaluations, exact.evaluations);
}

TEST(ChainTrellis, NodeCostsAreCounted) {
  auto table = tableOf(4, 3, [](std::size_t, uint32_t, uint32_t) {
    return static_cast<uint64_t>(10);
  });
  // Every step costs the same, so the node price alone decides: node 2 is
  // free, the others are not.
  table.node.assign(4, std::vector<uint64_t>{5, 5, 0});
  const auto result = solveTrellis(table.problem());
  ASSERT_TRUE(result.solved);
  EXPECT_EQ(result.chosen, (std::vector<uint32_t>{2, 2, 2, 2}));
  EXPECT_EQ(result.cost, table.exhaustive());
  EXPECT_EQ(result.cost, 30U);
}

TEST(ChainTrellis, ALayerNothingReachesShutsTheTrellis) {
  auto table = tableOf(4, 2, [](const std::size_t layer, uint32_t, uint32_t) {
    // Nothing gets out of layer 1.
    return layer == 1 ? TRELLIS_UNREACHABLE : static_cast<uint64_t>(1);
  });
  const auto result = solveTrellis(table.problem());
  EXPECT_FALSE(result.solved);
  EXPECT_EQ(result.cost, TRELLIS_UNREACHABLE);
  EXPECT_TRUE(result.chosen.empty());
  // The wall is the step out of layer 1: everything up to and including it
  // is reached going forward, nothing past it is.
  ASSERT_EQ(result.reachedFromStart.size(), 4U);
  EXPECT_TRUE(result.reachedFromStart[0]);
  EXPECT_TRUE(result.reachedFromStart[1]);
  EXPECT_FALSE(result.reachedFromStart[2]);
  EXPECT_FALSE(result.reachedFromStart[3]);
  // And the same wall read from the other end.
  ASSERT_EQ(result.reachedFromEnd.size(), 4U);
  EXPECT_FALSE(result.reachedFromEnd[0]);
  EXPECT_FALSE(result.reachedFromEnd[1]);
  EXPECT_TRUE(result.reachedFromEnd[2]);
  EXPECT_TRUE(result.reachedFromEnd[3]);
}

TEST(ChainTrellis, ASolvedTrellisReachesEveryLayerBothWays) {
  auto table = tableOf(5, 3, [](std::size_t, const uint32_t i, const uint32_t j) {
    return static_cast<uint64_t>((i * 3) + j + 1);
  });
  const auto result = solveTrellis(table.problem());
  ASSERT_TRUE(result.solved);
  for (std::size_t layer = 0; layer < table.width.size(); ++layer) {
    EXPECT_TRUE(result.reachedFromStart[layer]) << "layer " << layer;
    EXPECT_TRUE(result.reachedFromEnd[layer]) << "layer " << layer;
  }
}

TEST(ChainTrellis, AnImpossibleStepIsRoutedRound) {
  // The cheap way along node 0 is broken in the middle; the answer has to
  // step aside and come back.
  auto table = tableOf(4, 2, [](const std::size_t layer, const uint32_t i,
                                const uint32_t j) -> uint64_t {
    if (layer == 1 && i == 0 && j == 0) {
      return TRELLIS_UNREACHABLE;
    }
    return i == 0 && j == 0 ? 1 : 10;
  });
  const auto result = solveTrellis(table.problem());
  ASSERT_TRUE(result.solved);
  EXPECT_EQ(result.cost, table.exhaustive());
  EXPECT_EQ(result.cost, table.priceOf(result.chosen));
}

TEST(ChainTrellis, OneLayerHasNothingToPrice) {
  Table table;
  table.width = {3};
  table.node.assign(1, std::vector<uint64_t>{7, 2, 9});
  const auto result = solveTrellis(table.problem());
  ASSERT_TRUE(result.solved);
  EXPECT_EQ(result.chosen, (std::vector<uint32_t>{1}));
  EXPECT_EQ(result.cost, 2U);
  EXPECT_EQ(result.evaluations, 0U);
}

TEST(ChainTrellis, MatchesExhaustionOnRandomTrellises) {
  std::mt19937 rng(20260926);
  std::uniform_int_distribution<uint32_t> layerCount(2, 8);
  std::uniform_int_distribution<uint32_t> nodeCount(1, 12);
  std::uniform_int_distribution<uint64_t> weight(0, 40);
  std::uniform_int_distribution<uint32_t> broken(0, 9);
  std::uniform_int_distribution<uint64_t> slack(0, 40);

  uint64_t priced = 0;
  uint64_t total = 0;
  for (int trial = 0; trial < 400; ++trial) {
    Table table;
    const auto layers = layerCount(rng);
    for (uint32_t layer = 0; layer < layers; ++layer) {
      table.width.push_back(nodeCount(rng));
    }
    for (std::size_t layer = 0; layer + 1 < layers; ++layer) {
      std::vector<std::vector<uint64_t>> block(
          table.width[layer],
          std::vector<uint64_t>(table.width[layer + 1], 0));
      for (auto& row : block) {
        for (auto& value : row) {
          value = broken(rng) == 0 ? TRELLIS_UNREACHABLE : weight(rng);
        }
      }
      table.price.push_back(block);
      total += static_cast<uint64_t>(table.width[layer]) *
               table.width[layer + 1];
    }
    // A bound that is a genuine lower bound and usually not a tight one: the
    // price less a random slack, and anything at all where the step cannot be
    // built.
    table.hint = table.price;
    for (auto& block : table.hint) {
      for (auto& row : block) {
        for (auto& value : row) {
          if (value == TRELLIS_UNREACHABLE) {
            value = weight(rng);
          } else {
            const auto off = slack(rng);
            value = off > value ? 0 : value - off;
          }
        }
      }
    }
    for (uint32_t layer = 0; layer < layers; ++layer) {
      std::vector<uint64_t> row;
      for (uint32_t node = 0; node < table.width[layer]; ++node) {
        row.push_back(weight(rng));
      }
      table.node.push_back(row);
    }

    const auto result = solveTrellis(table.problem());
    const auto expected = table.exhaustive();
    ASSERT_EQ(result.solved, expected != TRELLIS_UNREACHABLE)
        << "trial " << trial;
    EXPECT_EQ(result.cost, expected) << "trial " << trial;
    if (result.solved) {
      EXPECT_EQ(table.priceOf(result.chosen), expected) << "trial " << trial;
    }
    priced += result.evaluations;
  }
  // Not a correctness claim, a record of what the laziness is worth on this
  // shape: with a bound this loose it still prices well under half.
  EXPECT_LT(priced, total / 2);
}

TEST(ChainTrellis, IsDeterministicUnderTies) {
  auto table = tableOf(5, 6, [](std::size_t, uint32_t, uint32_t) {
    return static_cast<uint64_t>(1);
  });
  const auto first = solveTrellis(table.problem());
  Table again = table;
  again.asked.clear();
  const auto second = solveTrellis(again.problem());
  ASSERT_TRUE(first.solved);
  EXPECT_EQ(first.chosen, second.chosen);
  // Every step costs the same, so the lowest index wins throughout.
  EXPECT_EQ(first.chosen, (std::vector<uint32_t>{0, 0, 0, 0, 0}));
}

} // namespace
