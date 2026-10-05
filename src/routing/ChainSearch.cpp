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

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <queue>
#include <vector>

namespace mqt::scpd::routing {

namespace {

/// Addition that stays at TRELLIS_UNREACHABLE instead of wrapping past it.
[[nodiscard]] uint64_t plus(const uint64_t a, const uint64_t b) {
  if (a == TRELLIS_UNREACHABLE || b == TRELLIS_UNREACHABLE) {
    return TRELLIS_UNREACHABLE;
  }
  return a > TRELLIS_UNREACHABLE - b ? TRELLIS_UNREACHABLE : a + b;
}

/// A prefix, held as its last choice and the prefix it extends. The whole
/// run of choices is read by walking `parent`, so no node copies one.
struct Node {
  uint32_t parent;
  uint32_t choice;
  uint32_t layer;
  /// The price of the prefix. Real once `real` is set; the bound for its own
  /// last step, and therefore too low, before that.
  uint64_t price;
  bool real;
};

/// One entry of the open list. `f` is the price of the prefix plus the
/// cheapest run of bounds from it to the end.
struct Open {
  uint64_t f;
  uint32_t layer;
  uint32_t node;
};

/// Which of two open entries the search takes first: the cheaper, then the
/// deeper, then the one made first. The last two only break ties, so they
/// cannot bear on the answer's price — they are here so that two runs of one
/// chain answer the same.
struct Worse {
  [[nodiscard]] bool operator()(const Open& a, const Open& b) const {
    if (a.f != b.f) {
      return a.f > b.f;
    }
    if (a.layer != b.layer) {
      return a.layer < b.layer;
    }
    return a.node > b.node;
  }
};

constexpr uint32_t NO_PARENT = std::numeric_limits<uint32_t>::max();

} // namespace

ChainSolution solveChainAStar(const ChainProblem& problem) {
  const auto layers = problem.width.size();
  ChainSolution out;
  if (layers == 0) {
    return out;
  }
  for (const auto width : problem.width) {
    if (width == 0) {
      // A layer with no choice at all shuts the chain; nothing to search.
      return out;
    }
  }

  // The heuristic: `rest[layer][choice]` is the cheapest run of bounds from
  // that choice to the end of the chain. One backward pass over the bound
  // matrices settles it, which costs `O(L n^2)` answers of a function that
  // answers for free.
  //
  // It is the exact optimum of the relaxation in which no step blocks any
  // other, so it is a lower bound on the rest of the chain **under every
  // prefix** — which is what makes the search exact. It is the same
  // relaxation the trellis uses as its price array, asked as a heuristic
  // instead.
  std::vector<std::vector<uint64_t>> rest(layers);
  rest[layers - 1].assign(problem.width[layers - 1], 0);
  for (std::size_t layer = layers - 1; layer > 0; --layer) {
    rest[layer - 1].assign(problem.width[layer - 1], TRELLIS_UNREACHABLE);
    for (uint32_t from = 0; from < problem.width[layer - 1]; ++from) {
      auto least = TRELLIS_UNREACHABLE;
      for (uint32_t to = 0; to < problem.width[layer]; ++to) {
        const auto reach =
            plus(problem.bound(layer - 1, from, to), rest[layer][to]);
        least = reach < least ? reach : least;
      }
      rest[layer - 1][from] = least;
    }
  }

  std::vector<Node> arena;
  std::priority_queue<Open, std::vector<Open>, Worse> open;
  const auto began = std::chrono::steady_clock::now();
  const auto spent = [&] { return std::chrono::steady_clock::now() - began; };

  // The run of choices a node stands for, read back through its parents.
  std::vector<uint32_t> prefix;
  const auto prefixOf = [&](const uint32_t node) {
    prefix.clear();
    for (auto at = node; at != NO_PARENT; at = arena[at].parent) {
      prefix.push_back(arena[at].choice);
    }
    std::ranges::reverse(prefix);
  };

  // A prefix of one layer has no step yet, so it is real from the start.
  for (uint32_t choice = 0; choice < problem.width[0]; ++choice) {
    if (rest[0][choice] == TRELLIS_UNREACHABLE) {
      continue;
    }
    arena.push_back({.parent = NO_PARENT,
                     .choice = choice,
                     .layer = 0,
                     .price = 0,
                     .real = true});
    open.push({.f = rest[0][choice],
               .layer = 0,
               .node = static_cast<uint32_t>(arena.size() - 1)});
  }

  while (!open.empty()) {
    if (problem.budget.count() > 0 && spent() > problem.budget) {
      // Out of time. `out` already holds the cheapest complete run of
      // choices the search reached, every step of it routed against the run
      // itself, and `optimal` stays false to say it was not proved cheapest.
      out.outOfTime = true;
      return out;
    }
    const auto top = open.top();
    open.pop();
    // Every node is pushed once as a placeholder and at most once more with
    // its step priced, and each push is popped exactly once, so nothing in
    // this queue is ever stale.
    const auto layer = arena[top.node].layer;
    const auto choice = arena[top.node].choice;

    if (!arena[top.node].real) {
      // A bound may have risen since this placeholder was pushed — the
      // caller's bound is allowed to learn — so ask it again before paying
      // for the step: a placeholder whose `f` is no longer the cheapest in
      // the queue goes back with the new one, unpriced.
      {
        const auto parent = arena[top.node].parent;
        const auto guess = plus(arena[parent].price,
                                problem.bound(layer - 1, arena[parent].choice,
                                              choice));
        const auto again = plus(guess, rest[layer][choice]);
        if (again > top.f) {
          arena[top.node].price = guess;
          open.push({.f = again, .layer = layer, .node = top.node});
          continue;
        }
      }
      // A prefix whose bound already reaches the cheapest complete run
      // priced so far cannot win; it is dropped unpriced. One that can is
      // told how much its step may cost and still win, so that the step can
      // stop early. Both are exact: the complete run is in the queue as a
      // real node and comes out when nothing cheaper is left.
      if (out.solved && top.f >= out.cost) {
        ++out.pruned;
        continue;
      }
      {
        const auto parent = arena[top.node].parent;
        const auto floor = plus(arena[parent].price, rest[layer][choice]);
        problem.stepBudget = out.solved && out.cost > floor
                                 ? out.cost - 1 - floor
                                 : TRELLIS_UNREACHABLE;
      }
      // The search has committed to looking at this prefix, so pay for its
      // last step: routed against the ways the prefix before it laid, which
      // is the whole reason this search exists.
      prefixOf(arena[top.node].parent);
      int64_t redone = 0;
      const auto priced = problem.step(prefix, choice, redone);
      problem.stepBudget = TRELLIS_UNREACHABLE;
      ++out.routed;
      if (priced == TRELLIS_UNREACHABLE) {
        // This prefix blocks itself. Nothing that extends it is a chain.
        continue;
      }
      auto price = plus(arena[arena[top.node].parent].price, priced);
      if (redone != 0 && price != TRELLIS_UNREACHABLE) {
        // Laying this step meant laying an earlier one again, so the prefix
        // no longer costs what its parent did plus this step.
        out.relaid = true;
        const auto by = static_cast<uint64_t>(redone < 0 ? -redone : redone);
        price = redone < 0 ? (by > price ? 0 : price - by) : plus(price, by);
      }
      const auto f = plus(price, rest[layer][choice]);
      if (f == TRELLIS_UNREACHABLE) {
        continue;
      }
      arena[top.node].price = price;
      arena[top.node].real = true;
      if (layer + 1 == layers) {
        // A complete run of choices, every step of it routed. Keep the
        // cheapest of them: if the budget runs out before the search proves
        // an optimum, that is the answer. They do not arrive cheapest first
        // — the end of the chain is reached in order of the *bound* on what
        // a run costs, and its last step is priced only once it is reached.
        //
        // Not with an `accept`: then `solved` means an accepted run exists,
        // and a run recorded here could be one the caller refuses when it
        // pops — the budget would hand back a run that failed the test.
        prefixOf(top.node);
        if (!problem.accept && (!out.solved || price < out.cost)) {
          out.solved = true;
          out.cost = price;
          out.chosen = prefix;
        }
        if (problem.found) {
          problem.found(prefix, price);
        }
      }
      open.push({.f = f, .layer = layer, .node = top.node});
      continue;
    }

    out.reached = std::max(out.reached, layer);
    if (layer + 1 == layers) {
      // Every step of this prefix is real, and every prefix still on the
      // queue carries a price that cannot be too high. Nothing can undercut
      // it.
      prefixOf(top.node);
      if (problem.accept &&
          !problem.accept(prefix, arena[top.node].price)) {
        // Refused. The next real pop of a last-layer node is the
        // next-cheapest complete run, so the search simply goes on — after
        // reading the clock again, because a test that routes takes
        // seconds and the loop head reads it only once per pop.
        ++out.rejected;
        if (problem.budget.count() > 0 && spent() > problem.budget) {
          out.outOfTime = true;
          return out;
        }
        if (problem.maxAccepts > 0 && out.rejected >= problem.maxAccepts) {
          out.outOfTrials = true;
          return out;
        }
        continue;
      }
      out.solved = true;
      out.optimal = !out.relaid;
      out.cost = arena[top.node].price;
      out.chosen = prefix;
      return out;
    }

    ++out.expansions;
    const auto price = arena[top.node].price;
    const auto next = layer + 1;
    for (uint32_t to = 0; to < problem.width[next]; ++to) {
      const auto guess = plus(price, problem.bound(layer, choice, to));
      const auto f = plus(guess, rest[next][to]);
      if (f == TRELLIS_UNREACHABLE) {
        continue;
      }
      arena.push_back({.parent = top.node,
                       .choice = to,
                       .layer = next,
                       .price = guess,
                       .real = false});
      open.push({.f = f,
                 .layer = next,
                 .node = static_cast<uint32_t>(arena.size() - 1)});
    }
  }
  // The queue ran dry: every run of choices there is has been weighed, so
  // the answer is settled whether or not a step was ever relaid — nothing is
  // left in the queue for an out-of-order price to have hidden. With an
  // `accept` that refused every run this is `optimal` and not `solved`: the
  // search saw every run there is and none passed.
  out.optimal = true;
  return out;
}

} // namespace mqt::scpd::routing
