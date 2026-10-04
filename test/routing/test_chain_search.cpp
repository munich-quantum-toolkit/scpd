/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/routing/ChainSearch.hpp"
#include "mqt-scpd/routing/ChainTrellis.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <random>
#include <span>
#include <utility>
#include <vector>

namespace {

using namespace mqt::scpd::routing;

/// A chain whose step prices are held in a table, so a test can say what the
/// answer must be and count what the search asked for.
///
/// The table is the trellis's own shape — a price per layer and pair of
/// choices — plus one rule that the trellis has no way to express: a
/// `conflict` that reads the whole prefix. With no conflict the two searches
/// solve the same problem and must agree; with one they do not, which is the
/// reason `solveChainAStar` exists.
struct Chain {
  std::vector<uint32_t> width;
  /// price[layer][from][to]
  std::vector<std::vector<std::vector<uint64_t>>> price;
  /// What the bound answers. Defaults to `price`, a bound that is exact
  /// wherever no conflict applies.
  std::vector<std::vector<std::vector<uint64_t>>> hint;

  /// What laying `to` after `prefix` costs on top of the table price, or
  /// TRELLIS_UNREACHABLE when the prefix leaves no room for it at all.
  /// Empty for a chain whose steps do not block each other.
  std::function<uint64_t(std::span<const uint32_t> prefix, uint32_t to)>
      conflict;

  mutable uint32_t asked = 0;

  /// The real price of one step, prefix and all.
  [[nodiscard]] uint64_t stepOf(const std::span<const uint32_t> prefix,
                                const uint32_t to) const {
    const auto layer = prefix.size() - 1;
    const auto base = price[layer][prefix[layer]][to];
    if (!conflict) {
      return base;
    }
    const auto extra = conflict(prefix, to);
    if (base == TRELLIS_UNREACHABLE || extra == TRELLIS_UNREACHABLE) {
      return TRELLIS_UNREACHABLE;
    }
    return base + extra;
  }

  [[nodiscard]] ChainProblem problem() const {
    ChainProblem out;
    out.width = width;
    out.bound = [this](const std::size_t layer, const uint32_t from,
                       const uint32_t to) { return hint[layer][from][to]; };
    out.step = [this](const std::span<const uint32_t> prefix, const uint32_t to,
                      int64_t& redone) {
      ++asked;
      redone = 0;
      return stepOf(prefix, to);
    };
    return out;
  }

  /// The same chain as a trellis, which can only be told the table price —
  /// the approximation whose error this whole search is here to remove.
  [[nodiscard]] TrellisProblem asTrellis() const {
    TrellisProblem out;
    out.width = width;
    out.bound = [this](const std::size_t layer, const uint32_t from,
                       const uint32_t to) { return hint[layer][from][to]; };
    out.evaluate = [this](const std::size_t layer, const uint32_t from,
                          const uint32_t to) { return price[layer][from][to]; };
    return out;
  }

  /// What one whole run of choices really costs, the conflicts counted.
  [[nodiscard]] uint64_t priceOf(const std::vector<uint32_t>& chosen) const {
    uint64_t total = 0;
    for (std::size_t layer = 0; layer + 1 < width.size(); ++layer) {
      const auto step =
          stepOf(std::span(chosen).first(layer + 1), chosen[layer + 1]);
      if (step == TRELLIS_UNREACHABLE) {
        return TRELLIS_UNREACHABLE;
      }
      total += step;
    }
    return total;
  }

  /// The optimum by exhaustion over every run of choices. The chains a test
  /// builds are small enough for this to be the reference.
  [[nodiscard]] uint64_t exhaustive() const {
    std::vector<uint32_t> run(width.size(), 0);
    auto least = TRELLIS_UNREACHABLE;
    for (;;) {
      const auto total = priceOf(run);
      least = total < least ? total : least;
      std::size_t at = 0;
      for (; at < run.size(); ++at) {
        if (++run[at] < width[at]) {
          break;
        }
        run[at] = 0;
      }
      if (at == run.size()) {
        return least;
      }
    }
  }
};

/// A chain of `width` choices on each of `layers` layers, every step priced
/// by the rule given, and the bound exact.
Chain chainOf(
    const std::size_t layers, const uint32_t width,
    const std::function<uint64_t(std::size_t, uint32_t, uint32_t)>& rule) {
  Chain out;
  out.width.assign(layers, width);
  for (std::size_t layer = 0; layer + 1 < layers; ++layer) {
    std::vector<std::vector<uint64_t>> block(width,
                                             std::vector<uint64_t>(width, 0));
    for (uint32_t from = 0; from < width; ++from) {
      for (uint32_t to = 0; to < width; ++to) {
        block[from][to] = rule(layer, from, to);
      }
    }
    out.price.push_back(block);
  }
  out.hint = out.price;
  return out;
}

/// Flatten a bound to zero — it says nothing, and the answer must not move.
void blind(Chain& chain) {
  for (auto& block : chain.hint) {
    for (auto& row : block) {
      for (auto& value : row) {
        value = 0;
      }
    }
  }
}

TEST(ChainSearch, FindsTheKnownOptimum) {
  // Choice 0 of every layer is dear to leave, choice 1 is cheap: the optimum
  // runs along choice 1 and costs 10 a step.
  auto chain =
      chainOf(5, 2, [](std::size_t, const uint32_t from, const uint32_t to) {
        return static_cast<uint64_t>(from == 1 && to == 1 ? 10 : 100);
      });
  const auto result = solveChainAStar(chain.problem());
  ASSERT_TRUE(result.solved);
  EXPECT_EQ(result.cost, chain.exhaustive());
  EXPECT_EQ(result.cost, chain.priceOf(result.chosen));
  EXPECT_EQ(result.chosen, (std::vector<uint32_t>{1, 1, 1, 1, 1}));
}

TEST(ChainSearch, APrefixThatBlocksItselfCostsAnother) {
  // **The reason this search replaces the trellis**, written down as
  // something that runs.
  //
  // Every step is priced from the table, and on top of that one pair of
  // choices cannot stand together: once choice 1 is taken, no later step may
  // land on choice 2. A trellis cannot see that — the step into choice 2
  // carries one price whatever came before it — so it names a run of choices
  // that cannot be built, which is exactly how a feedline edge is lost at
  // the commit. The prefix search prices the blocked step at everything and
  // takes another combination.
  //
  // The table discounts exactly the three steps of the run 0, 1, 2, 2 and
  // charges ten for every other step, so that run costs 3 and no other costs
  // less than 21. The trellis therefore names it, and it is the one run the
  // conflict forbids.
  auto chain = chainOf(4, 3,
                       [](const std::size_t layer, const uint32_t from,
                          const uint32_t to) -> uint64_t {
                         const uint32_t along[] = {0, 1, 2, 2};
                         return (from == along[layer] && to == along[layer + 1])
                                    ? 1
                                    : 10;
                       });
  chain.conflict = [](const std::span<const uint32_t> prefix,
                      const uint32_t to) {
    const auto held = std::ranges::find(prefix, 1U) != prefix.end();
    return (held && to == 2) ? TRELLIS_UNREACHABLE : uint64_t{0};
  };

  const auto exact = solveChainAStar(chain.problem());
  ASSERT_TRUE(exact.solved);
  const auto optimum = chain.exhaustive();
  ASSERT_EQ(optimum, 21U);
  EXPECT_EQ(exact.cost, optimum);
  EXPECT_EQ(chain.priceOf(exact.chosen), optimum);

  // And the trellis, told the only thing it can be told, answers a run of
  // choices that cannot be built at all.
  const auto approximate = solveTrellis(chain.asTrellis());
  ASSERT_TRUE(approximate.solved);
  EXPECT_EQ(approximate.cost, 3U) << "the trellis under-prices";
  EXPECT_EQ(approximate.chosen, (std::vector<uint32_t>{0, 1, 2, 2}));
  EXPECT_EQ(chain.priceOf(approximate.chosen), TRELLIS_UNREACHABLE)
      << "the run the trellis named is one that blocks itself";
}

TEST(ChainSearch, BeatsTheTrellisOnRandomChainsThatBlockThemselves) {
  // The same thing over a batch rather than one hand-built case: small
  // chains, random table prices, and a random rule that makes a handful of
  // pairs of choices refuse to stand together.
  //
  // The prefix search must return the optimum of every one of them. The
  // trellis must fail some of them, which is the whole claim; where it
  // happens to agree that is no fault of its own, so only the batch is
  // asserted on.
  std::mt19937 rng(20260928);
  std::uniform_int_distribution<uint32_t> layerCount(3, 5);
  std::uniform_int_distribution<uint32_t> choiceCount(2, 4);
  std::uniform_int_distribution<uint64_t> weight(1, 40);
  std::uniform_int_distribution<uint64_t> slack(0, 20);
  std::uniform_int_distribution<uint32_t> pick(0, 3);

  uint32_t trellisWorse = 0;
  uint32_t solvedBoth = 0;
  for (int trial = 0; trial < 300; ++trial) {
    const auto layers = layerCount(rng);
    const auto choices = choiceCount(rng);
    auto chain = chainOf(layers, choices, [&](std::size_t, uint32_t, uint32_t) {
      return weight(rng);
    });
    // A bound that is a genuine lower bound and usually not a tight one.
    // With a conflict rule on top, the table price is itself a lower bound
    // on the real step, so undercutting it keeps the bound admissible.
    for (auto& block : chain.hint) {
      for (auto& row : block) {
        for (auto& value : row) {
          const auto off = slack(rng);
          value = off > value ? 0 : value - off;
        }
      }
    }
    // Two choices that cannot both stand. Held as a table so the rule is
    // the same however often it is asked.
    std::vector<std::vector<bool>> blocks(choices,
                                          std::vector<bool>(choices, false));
    for (uint32_t held = 0; held < choices; ++held) {
      for (uint32_t later = 0; later < choices; ++later) {
        blocks[held][later] = pick(rng) == 0;
      }
    }
    chain.conflict = [blocks](const std::span<const uint32_t> prefix,
                              const uint32_t to) {
      for (const auto held : prefix) {
        if (blocks[held][to]) {
          return TRELLIS_UNREACHABLE;
        }
      }
      return uint64_t{0};
    };

    const auto optimum = chain.exhaustive();
    const auto exact = solveChainAStar(chain.problem());
    ASSERT_EQ(exact.solved, optimum != TRELLIS_UNREACHABLE)
        << "trial " << trial;
    EXPECT_EQ(exact.cost, optimum) << "trial " << trial;
    if (exact.solved) {
      EXPECT_EQ(chain.priceOf(exact.chosen), optimum) << "trial " << trial;
    }

    const auto approximate = solveTrellis(chain.asTrellis());
    if (!approximate.solved) {
      continue;
    }
    ++solvedBoth;
    // What the trellis named, priced honestly.
    const auto real = chain.priceOf(approximate.chosen);
    EXPECT_GE(real, optimum) << "trial " << trial;
    trellisWorse += (real != optimum) ? 1U : 0U;
  }
  EXPECT_GT(solvedBoth, 0U);
  // The claim: on chains whose steps block each other the trellis is wrong,
  // and not rarely.
  EXPECT_GT(trellisWorse, solvedBoth / 10)
      << "the trellis answered " << trellisWorse << " of " << solvedBoth
      << " wrongly";
}

TEST(ChainSearch, AnExactBoundPricesOnlyTheAnswer) {
  // With the bound equal to the price everywhere and one clear optimum, the
  // search walks straight down it: `L - 1` steps priced and no others.
  auto chain =
      chainOf(6, 4, [](std::size_t, const uint32_t from, const uint32_t to) {
        return static_cast<uint64_t>((from * 7) + (to * 3) + 1);
      });
  const auto result = solveChainAStar(chain.problem());
  ASSERT_TRUE(result.solved);
  EXPECT_EQ(result.cost, chain.exhaustive());
  EXPECT_EQ(result.routed, chain.width.size() - 1);
  EXPECT_EQ(chain.asked, result.routed);
}

TEST(ChainSearch, AWorthlessBoundStillAnswersExactly) {
  // A bound of zero says nothing at all. The answer must not change; only
  // the number of steps priced may.
  auto chain =
      chainOf(5, 4, [](std::size_t, const uint32_t from, const uint32_t to) {
        return static_cast<uint64_t>(((from * 13) ^ (to * 5)) + 1);
      });
  const auto sharp = solveChainAStar(chain.problem());
  ASSERT_TRUE(sharp.solved);

  Chain flat = chain;
  flat.asked = 0;
  blind(flat);
  const auto result = solveChainAStar(flat.problem());
  ASSERT_TRUE(result.solved);
  EXPECT_EQ(result.cost, chain.exhaustive());
  EXPECT_EQ(result.cost, sharp.cost);
  EXPECT_GT(result.routed, sharp.routed);
}

TEST(ChainSearch, AnUnbuildableStepIsRoutedRound) {
  // The cheap way along choice 0 is broken in the middle; the answer has to
  // step aside and come back.
  auto chain = chainOf(4, 2,
                       [](const std::size_t layer, const uint32_t from,
                          const uint32_t to) -> uint64_t {
                         if (layer == 1 && from == 0 && to == 0) {
                           return TRELLIS_UNREACHABLE;
                         }
                         return from == 0 && to == 0 ? 1 : 10;
                       });
  const auto result = solveChainAStar(chain.problem());
  ASSERT_TRUE(result.solved);
  EXPECT_EQ(result.cost, chain.exhaustive());
  EXPECT_EQ(result.cost, chain.priceOf(result.chosen));
}

TEST(ChainSearch, ALayerNothingReachesNamesTheWall) {
  auto chain = chainOf(4, 2, [](const std::size_t layer, uint32_t, uint32_t) {
    // Nothing gets out of layer 1.
    return layer == 1 ? TRELLIS_UNREACHABLE : static_cast<uint64_t>(1);
  });
  // The bound has to be asked about a step it can answer for, so flatten it:
  // a bound that already says "unreachable" would shut the chain before a
  // single step was priced, and then the wall would be nothing but the
  // bound's own opinion.
  blind(chain);
  const auto result = solveChainAStar(chain.problem());
  EXPECT_FALSE(result.solved);
  EXPECT_EQ(result.cost, TRELLIS_UNREACHABLE);
  EXPECT_TRUE(result.chosen.empty());
  // Layer 1 is reached and nothing past it is, so the step out of layer 1 is
  // what shut the chain.
  EXPECT_EQ(result.reached, 1U);
}

TEST(ChainSearch, OneLayerHasNothingToPrice) {
  Chain chain;
  chain.width = {3};
  const auto result = solveChainAStar(chain.problem());
  ASSERT_TRUE(result.solved);
  EXPECT_EQ(result.chosen, (std::vector<uint32_t>{0}));
  EXPECT_EQ(result.cost, 0U);
  EXPECT_EQ(result.routed, 0U);
}

TEST(ChainSearch, AnEmptyLayerShutsTheChain) {
  Chain chain;
  chain.width = {3, 0, 2};
  const auto result = solveChainAStar(chain.problem());
  EXPECT_FALSE(result.solved);
  EXPECT_EQ(result.routed, 0U);
}

TEST(ChainSearch, MatchesTheTrellisWhereNothingBlocks) {
  // Without a conflict rule the two searches solve the same problem, so they
  // must agree — on the price, and on the run of choices, since both break
  // ties towards the lowest index.
  std::mt19937 rng(20260928);
  std::uniform_int_distribution<uint32_t> layerCount(2, 6);
  std::uniform_int_distribution<uint32_t> choiceCount(1, 5);
  std::uniform_int_distribution<uint64_t> weight(0, 40);
  std::uniform_int_distribution<uint32_t> broken(0, 9);
  std::uniform_int_distribution<uint64_t> slack(0, 40);

  for (int trial = 0; trial < 200; ++trial) {
    Chain chain;
    const auto layers = layerCount(rng);
    for (uint32_t layer = 0; layer < layers; ++layer) {
      chain.width.push_back(choiceCount(rng));
    }
    for (std::size_t layer = 0; layer + 1 < layers; ++layer) {
      std::vector<std::vector<uint64_t>> block(
          chain.width[layer], std::vector<uint64_t>(chain.width[layer + 1], 0));
      for (auto& row : block) {
        for (auto& value : row) {
          value = broken(rng) == 0 ? TRELLIS_UNREACHABLE : weight(rng);
        }
      }
      chain.price.push_back(block);
    }
    chain.hint = chain.price;
    for (auto& block : chain.hint) {
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

    const auto expected = chain.exhaustive();
    const auto result = solveChainAStar(chain.problem());
    ASSERT_EQ(result.solved, expected != TRELLIS_UNREACHABLE)
        << "trial " << trial;
    EXPECT_EQ(result.cost, expected) << "trial " << trial;
    const auto trellis = solveTrellis(chain.asTrellis());
    EXPECT_EQ(result.cost, trellis.cost) << "trial " << trial;
    if (result.solved) {
      EXPECT_EQ(chain.priceOf(result.chosen), expected) << "trial " << trial;
    }
  }
}

TEST(ChainSearch, ARedonePriceIsCountedAndCostsTheProof) {
  // A step answers with a `redone` price to say: laying me meant laying an
  // earlier step again, and the prefix costs this much more or less than it
  // did. The coupler insertion does that when the two ways at one end of a
  // chain will only both fit with the later one laid first.
  ChainProblem problem;
  problem.width = {2, 2, 2};
  problem.bound = [](std::size_t, uint32_t, uint32_t) { return uint64_t{0}; };
  problem.step = [](const std::span<const uint32_t> prefix, const uint32_t to,
                    int64_t& redone) {
    // Every step costs ten, and the second one into choice 1 lays the first
    // again, five cheaper than it was.
    redone = (prefix.size() == 2 && to == 1) ? -5 : 0;
    return uint64_t{10};
  };
  const auto result = solveChainAStar(problem);
  ASSERT_TRUE(result.solved);
  EXPECT_EQ(result.cost, 15U);
  ASSERT_EQ(result.chosen.size(), 3U);
  EXPECT_EQ(result.chosen.back(), 1U);
  EXPECT_TRUE(result.relaid);
  // A prefix cheaper than the prefix it extends is the one thing A* rests on
  // not happening, so the answer is the best it found and not a proved
  // optimum.
  EXPECT_FALSE(result.optimal);
}

TEST(ChainSearch, WithoutARedonePriceTheAnswerIsProved) {
  auto chain =
      chainOf(5, 3, [](std::size_t, const uint32_t from, const uint32_t to) {
        return static_cast<uint64_t>((from * 5) + to + 1);
      });
  const auto result = solveChainAStar(chain.problem());
  ASSERT_TRUE(result.solved);
  EXPECT_TRUE(result.optimal);
  EXPECT_FALSE(result.relaid);
}

TEST(ChainSearch, TheBudgetStopsTheSearchAndClaimsNothing) {
  auto chain =
      chainOf(6, 4, [](std::size_t, const uint32_t from, const uint32_t to) {
        return static_cast<uint64_t>((from * 7) + (to * 3) + 1);
      });
  auto problem = chain.problem();
  problem.budget = std::chrono::nanoseconds(1);
  const auto result = solveChainAStar(problem);
  // Whatever it managed, it may not call it an optimum, and what it hands
  // back must price as what it says it costs.
  EXPECT_FALSE(result.optimal);
  if (result.solved) {
    EXPECT_EQ(chain.priceOf(result.chosen), result.cost);
  } else {
    EXPECT_TRUE(result.chosen.empty());
  }
}

TEST(ChainSearch, NoBudgetMeansNoLimit) {
  auto chain =
      chainOf(5, 4, [](std::size_t, const uint32_t from, const uint32_t to) {
        return static_cast<uint64_t>(((from * 13) ^ (to * 5)) + 1);
      });
  auto problem = chain.problem();
  problem.budget = std::chrono::nanoseconds(0);
  const auto result = solveChainAStar(problem);
  ASSERT_TRUE(result.solved);
  EXPECT_TRUE(result.optimal);
  EXPECT_EQ(result.cost, chain.exhaustive());
}

TEST(ChainSearch, EveryCompleteRunIsReportedAndTheCheapestIsKept) {
  // What the budget falls back on, and what `-v 1` prints: every run of
  // choices whose every step is priced. Each one must be a whole run, must
  // cost what it is reported to cost, and none may undercut the answer.
  auto chain =
      chainOf(5, 3, [](std::size_t, const uint32_t from, const uint32_t to) {
        return static_cast<uint64_t>(((from * 11) ^ (to * 7)) + 1);
      });
  blind(chain);
  auto problem = chain.problem();
  std::vector<std::pair<std::vector<uint32_t>, uint64_t>> runs;
  problem.found = [&](const std::span<const uint32_t> run,
                      const uint64_t cost) {
    runs.emplace_back(std::vector<uint32_t>(run.begin(), run.end()), cost);
  };
  const auto result = solveChainAStar(problem);
  ASSERT_TRUE(result.solved);
  ASSERT_FALSE(runs.empty());
  for (const auto& [run, cost] : runs) {
    EXPECT_EQ(run.size(), chain.width.size());
    EXPECT_EQ(chain.priceOf(run), cost);
    EXPECT_GE(cost, result.cost);
  }
  // The answer is one of the runs that were reported.
  EXPECT_NE(std::ranges::find(runs, std::pair{result.chosen, result.cost}),
            runs.end());
}

/// Every run of choices of a chain, cheapest first by `priceOf`, ties in
/// lexicographic order of the run — the order a reader can check by hand.
std::vector<std::pair<uint64_t, std::vector<uint32_t>>>
everyRunOf(const Chain& chain) {
  std::vector<std::pair<uint64_t, std::vector<uint32_t>>> runs;
  std::vector<uint32_t> run(chain.width.size(), 0);
  for (;;) {
    runs.emplace_back(chain.priceOf(run), run);
    std::size_t at = run.size();
    while (at > 0) {
      --at;
      if (++run[at] < chain.width[at]) {
        break;
      }
      run[at] = 0;
      if (at == 0) {
        std::ranges::sort(runs);
        return runs;
      }
    }
  }
}

TEST(ChainSearch, ARefusedCheapestRunYieldsTheNextCheapest) {
  // Four layers of three choices with prices that make every run cost
  // something different. `accept` refuses the one run that is cheapest of
  // all; the answer must then be the second-cheapest run there is, priced
  // as it says, and the refusal counted once.
  auto chain =
      chainOf(4, 3, [](const std::size_t layer, const uint32_t from,
                       const uint32_t to) {
        return static_cast<uint64_t>((layer * 100) + (from * 7) + (to * 3) +
                                     ((from + to) % 2 == 0 ? 1 : 2));
      });
  const auto runs = everyRunOf(chain);
  ASSERT_LT(runs[0].first, runs[1].first)
      << "the fixture needs a unique cheapest run";
  ASSERT_LT(runs[1].first, runs[2].first)
      << "the fixture needs a unique second-cheapest run";
  const auto plain = solveChainAStar(chain.problem());
  ASSERT_TRUE(plain.solved);
  EXPECT_EQ(plain.chosen, runs[0].second);

  auto problem = chain.problem();
  std::vector<std::vector<uint32_t>> offered;
  problem.accept = [&](const std::span<const uint32_t> run,
                       const uint64_t cost) {
    offered.emplace_back(run.begin(), run.end());
    EXPECT_EQ(chain.priceOf(offered.back()), cost);
    return offered.back() != runs[0].second;
  };
  const auto result = solveChainAStar(problem);
  ASSERT_TRUE(result.solved);
  EXPECT_TRUE(result.optimal);
  EXPECT_EQ(result.chosen, runs[1].second);
  EXPECT_EQ(result.cost, runs[1].first);
  EXPECT_EQ(result.rejected, 1U);
  EXPECT_FALSE(result.outOfTrials);
  // Offered cheapest first: the refused optimum, then the answer.
  ASSERT_EQ(offered.size(), 2U);
  EXPECT_EQ(offered[0], runs[0].second);
  EXPECT_EQ(offered[1], runs[1].second);
}

TEST(ChainSearch, AcceptingEveryRunChangesNothing) {
  // An `accept` that refuses nothing is the search as it was, figure for
  // figure: the same run, the same price, the same work.
  auto chain =
      chainOf(5, 4, [](std::size_t, const uint32_t from, const uint32_t to) {
        return static_cast<uint64_t>(((from * 13) ^ (to * 5)) + 1);
      });
  const auto plain = solveChainAStar(chain.problem());
  const auto plainAsked = chain.asked;
  chain.asked = 0;
  auto problem = chain.problem();
  problem.accept = [](std::span<const uint32_t>, uint64_t) { return true; };
  const auto accepting = solveChainAStar(problem);
  ASSERT_TRUE(plain.solved);
  ASSERT_TRUE(accepting.solved);
  EXPECT_EQ(accepting.chosen, plain.chosen);
  EXPECT_EQ(accepting.cost, plain.cost);
  EXPECT_EQ(accepting.optimal, plain.optimal);
  EXPECT_EQ(accepting.routed, plain.routed);
  EXPECT_EQ(accepting.expansions, plain.expansions);
  EXPECT_EQ(accepting.reached, plain.reached);
  EXPECT_EQ(chain.asked, plainAsked);
  EXPECT_EQ(accepting.rejected, 0U);
}

TEST(ChainSearch, RefusingEveryRunExhaustsTheChain) {
  // Three layers of two choices: eight runs, every step buildable. An
  // `accept` that refuses them all leaves the search with every run
  // weighed and none taken — `optimal` without `solved`, eight refusals,
  // nothing in `chosen`, and the eight runs offered cheapest first.
  auto chain =
      chainOf(3, 2, [](const std::size_t layer, const uint32_t from,
                       const uint32_t to) {
        return static_cast<uint64_t>((layer * 10) + (from * 4) + (to * 2) + 1);
      });
  const auto runs = everyRunOf(chain);
  auto problem = chain.problem();
  std::vector<uint64_t> offered;
  std::vector<std::vector<uint32_t>> found;
  problem.found = [&](const std::span<const uint32_t> run, uint64_t) {
    found.emplace_back(run.begin(), run.end());
  };
  problem.accept = [&](std::span<const uint32_t>, const uint64_t cost) {
    offered.push_back(cost);
    return false;
  };
  const auto result = solveChainAStar(problem);
  EXPECT_FALSE(result.solved);
  EXPECT_TRUE(result.optimal);
  EXPECT_FALSE(result.outOfTime);
  EXPECT_FALSE(result.outOfTrials);
  EXPECT_TRUE(result.chosen.empty());
  EXPECT_EQ(result.cost, TRELLIS_UNREACHABLE);
  EXPECT_EQ(result.rejected, 8U);
  EXPECT_EQ(found.size(), 8U);
  ASSERT_EQ(offered.size(), 8U);
  EXPECT_TRUE(std::ranges::is_sorted(offered));
  for (std::size_t at = 0; at < runs.size(); ++at) {
    EXPECT_EQ(offered[at], runs[at].first);
  }
}

TEST(ChainSearch, TheTrialBoundStopsTheRefusals) {
  // The same chain, two trials allowed: two refusals, then the search stops
  // and says it was the trials and not the clock or the chain.
  auto chain =
      chainOf(3, 2, [](const std::size_t layer, const uint32_t from,
                       const uint32_t to) {
        return static_cast<uint64_t>((layer * 10) + (from * 4) + (to * 2) + 1);
      });
  auto problem = chain.problem();
  problem.maxAccepts = 2;
  problem.accept = [](std::span<const uint32_t>, uint64_t) { return false; };
  const auto result = solveChainAStar(problem);
  EXPECT_FALSE(result.solved);
  EXPECT_FALSE(result.optimal);
  EXPECT_FALSE(result.outOfTime);
  EXPECT_TRUE(result.outOfTrials);
  EXPECT_EQ(result.rejected, 2U);
  EXPECT_TRUE(result.chosen.empty());
  // The bound is a bound on refusals: a run accepted within it is the
  // answer, and the search does not say it ran out.
  auto again = chain.problem();
  again.maxAccepts = 2;
  uint32_t asked = 0;
  again.accept = [&](std::span<const uint32_t>, uint64_t) {
    return ++asked == 2;
  };
  const auto second = solveChainAStar(again);
  EXPECT_TRUE(second.solved);
  EXPECT_FALSE(second.outOfTrials);
  EXPECT_EQ(second.rejected, 1U);
  EXPECT_EQ(second.chosen, everyRunOf(chain)[1].second);
}

TEST(ChainSearch, TheClockIsReadAfterARefusal) {
  // A budget the first refusal alone outlasts: the search stops on the
  // clock right after that refusal, hands back no run — the one it reached
  // was refused — and says it was the time.
  auto chain =
      chainOf(4, 3, [](std::size_t, const uint32_t from, const uint32_t to) {
        return static_cast<uint64_t>((from * 7) + (to * 3) + 1);
      });
  auto problem = chain.problem();
  problem.budget = std::chrono::milliseconds(1);
  uint32_t asked = 0;
  problem.accept = [&](std::span<const uint32_t>, uint64_t) {
    ++asked;
    const auto until = std::chrono::steady_clock::now() +
                       std::chrono::milliseconds(5);
    while (std::chrono::steady_clock::now() < until) {
    }
    return false;
  };
  const auto result = solveChainAStar(problem);
  EXPECT_FALSE(result.solved);
  EXPECT_TRUE(result.outOfTime);
  EXPECT_FALSE(result.outOfTrials);
  EXPECT_TRUE(result.chosen.empty());
  EXPECT_EQ(asked, 1U);
  EXPECT_EQ(result.rejected, 1U);
}

TEST(ChainSearch, IsDeterministicUnderTies) {
  auto chain = chainOf(5, 6, [](std::size_t, uint32_t, uint32_t) {
    return static_cast<uint64_t>(1);
  });
  const auto first = solveChainAStar(chain.problem());
  Chain again = chain;
  again.asked = 0;
  const auto second = solveChainAStar(again.problem());
  ASSERT_TRUE(first.solved);
  EXPECT_EQ(first.chosen, second.chosen);
  EXPECT_EQ(first.routed, second.routed);
  // Every step costs the same, so the lowest index wins throughout.
  EXPECT_EQ(first.chosen, (std::vector<uint32_t>{0, 0, 0, 0, 0}));
}

} // namespace
