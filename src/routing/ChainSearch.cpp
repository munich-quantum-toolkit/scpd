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
#include <cstddef>
#include <cstdint>
#include <limits>
#include <queue>
#include <set>
#include <unordered_map>
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
  /// Whether this is a placeholder still on the open list, its step not yet
  /// priced or dropped.
  bool pending = false;
};

/// One entry of the open list. `f` is the price of the prefix plus the
/// cheapest run of bounds from it to the end; `key` is what the list is
/// ordered by, `f` itself or, under weighted A\*, the price plus the
/// weighted run of bounds.
struct Open {
  uint64_t f;
  uint64_t key;
  uint32_t layer;
  uint32_t node;
};

/// Which of two open entries the search takes first: the smaller key, then
/// the deeper, then the one made first. The last two only break ties, so
/// they cannot bear on the answer's price — they are here so that two runs
/// of one chain answer the same.
struct Worse {
  [[nodiscard]] bool operator()(const Open& a, const Open& b) const {
    if (a.key != b.key) {
      return a.key > b.key;
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

  // The least rest of each layer, for `siblingBudget`.
  std::vector<uint64_t> leastRest(layers, TRELLIS_UNREACHABLE);
  for (std::size_t layer = 0; layer < layers; ++layer) {
    for (const auto one : rest[layer]) {
      leastRest[layer] = std::min(leastRest[layer], one);
    }
  }

  // The key of an entry: `g + w * h`, with `w * h` rounded to the nearest
  // whole number, halves up. At w = 1 it is `f` itself.
  const uint64_t weight = std::max<uint64_t>(problem.weightThousandths, 1000);
  const auto keyOf = [weight](const uint64_t g, const uint64_t h) {
    if (g == TRELLIS_UNREACHABLE || h == TRELLIS_UNREACHABLE) {
      return TRELLIS_UNREACHABLE;
    }
    if (weight == 1000) {
      return plus(g, h);
    }
    if (h > (TRELLIS_UNREACHABLE - 500) / weight) {
      return TRELLIS_UNREACHABLE;
    }
    return plus(g, ((h * weight) + 500) / 1000);
  };

  std::vector<Node> arena;
  std::priority_queue<Open, std::vector<Open>, Worse> open;
  // The placeholders pushed under each prefix, for `siblingKeys`.
  std::unordered_map<uint32_t, std::vector<uint32_t>> childrenOf;

  // The run of choices a node stands for, read back through its parents.
  std::vector<uint32_t> prefix;
  const auto prefixOf = [&](const uint32_t node) {
    prefix.clear();
    for (auto at = node; at != NO_PARENT; at = arena[at].parent) {
      prefix.push_back(arena[at].choice);
    }
    std::ranges::reverse(prefix);
  };

  // The classes of interchangeable choices (`ChainProblem::classOf`): the
  // next member of each choice's class by index, and whether a choice is
  // its class's representative.
  constexpr uint32_t NO_MEMBER = std::numeric_limits<uint32_t>::max();
  const bool classed = problem.classOf.size() == layers;
  std::vector<std::vector<uint32_t>> nextOf;
  if (classed) {
    nextOf.resize(layers);
    for (std::size_t layer = 0; layer < layers; ++layer) {
      nextOf[layer].assign(problem.width[layer], NO_MEMBER);
      std::vector<uint32_t> last(problem.width[layer], NO_MEMBER);
      for (uint32_t choice = 0; choice < problem.width[layer]; ++choice) {
        const auto owner = problem.classOf[layer][choice];
        if (last[owner] != NO_MEMBER) {
          nextOf[layer][last[owner]] = choice;
        }
        last[owner] = choice;
      }
    }
  }
  const auto offeredAlone = [&](const std::size_t layer,
                                const uint32_t choice) {
    return !classed || problem.classOf[layer][choice] == choice;
  };

  // A prefix of one layer has no step yet, so it is real from the start;
  // a longer one is pushed as a placeholder on the bound of its last step.
  const auto pushChild = [&](const uint32_t parent, const uint32_t layer,
                             const uint32_t to) {
    if (parent == NO_PARENT) {
      if (rest[0][to] == TRELLIS_UNREACHABLE) {
        return;
      }
      arena.push_back({.parent = NO_PARENT,
                       .choice = to,
                       .layer = 0,
                       .price = 0,
                       .real = true});
      open.push({.f = rest[0][to],
                 .key = keyOf(0, rest[0][to]),
                 .layer = 0,
                 .node = static_cast<uint32_t>(arena.size() - 1)});
      return;
    }
    const auto guess = plus(arena[parent].price,
                            problem.bound(layer - 1, arena[parent].choice, to));
    const auto f = plus(guess, rest[layer][to]);
    if (f == TRELLIS_UNREACHABLE) {
      return;
    }
    arena.push_back({.parent = parent,
                     .choice = to,
                     .layer = layer,
                     .price = guess,
                     .real = false,
                     .pending = true});
    childrenOf[parent].push_back(static_cast<uint32_t>(arena.size() - 1));
    open.push({.f = f,
               .key = keyOf(guess, rest[layer][to]),
               .layer = layer,
               .node = static_cast<uint32_t>(arena.size() - 1)});
  };
  // The next member after `choice` in its class, offered once per parent.
  std::set<std::pair<uint32_t, uint32_t>> offered;
  const auto offerNext = [&](const uint32_t parent, const uint32_t layer,
                             const uint32_t choice) {
    if (!classed) {
      return;
    }
    const auto member = nextOf[layer][choice];
    if (member == NO_MEMBER || !offered.insert({parent, member}).second) {
      return;
    }
    ++out.membersOffered;
    pushChild(parent, layer, member);
  };
  // A refused prefix offers the next member at every layer of it.
  const auto offerAlong = [&](const uint32_t node) {
    for (auto at = node; at != NO_PARENT; at = arena[at].parent) {
      offerNext(arena[at].parent, arena[at].layer, arena[at].choice);
    }
  };

  for (uint32_t choice = 0; choice < problem.width[0]; ++choice) {
    if (offeredAlone(0, choice)) {
      pushChild(NO_PARENT, 0, choice);
    }
  }

  while (!open.empty()) {
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
          open.push({.f = again,
                     .key = keyOf(guess, rest[layer][choice]),
                     .layer = layer,
                     .node = top.node});
          continue;
        }
      }
      // A prefix whose bound already reaches the cheapest complete run
      // priced so far cannot win; it is dropped unpriced. One that can is
      // told how much its step may cost and still win, so that the step can
      // stop early. Both are exact: the complete run is in the queue as a
      // real node and comes out when nothing cheaper is left.
      arena[top.node].pending = false;
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
        const auto least = plus(arena[parent].price, leastRest[layer]);
        problem.siblingBudget = out.solved && out.cost > least
                                    ? out.cost - 1 - least
                                    : TRELLIS_UNREACHABLE;
        // The keys of the steps under the same prefix that still wait.
        problem.stepKey = top.f;
        problem.siblingKeys.assign(problem.width[layer], TRELLIS_UNREACHABLE);
        for (const auto sibling : childrenOf[parent]) {
          const auto& node = arena[sibling];
          if (!node.pending) {
            continue;
          }
          const auto f = plus(node.price, rest[layer][node.choice]);
          if (!out.solved || f < out.cost) {
            problem.siblingKeys[node.choice] = f;
          }
        }
      }
      // The search has committed to looking at this prefix, so pay for its
      // last step: routed against the ways the prefix before it laid, which
      // is the whole reason this search exists.
      prefixOf(arena[top.node].parent);
      int64_t redone = 0;
      problem.askNext = false;
      const auto priced = problem.step(prefix, choice, redone);
      if (problem.askNext) {
        problem.askNext = false;
        offerNext(arena[top.node].parent, layer, choice);
      }
      problem.stepBudget = TRELLIS_UNREACHABLE;
      problem.siblingBudget = TRELLIS_UNREACHABLE;
      ++out.routed;
      if (problem.progress && problem.progressEvery > 0 &&
          out.routed % problem.progressEvery == 0) {
        problem.progress(out, open.size());
      }
      if (priced == TRELLIS_UNREACHABLE) {
        // This prefix blocks itself. Nothing that extends it is a chain;
        // a member of a class at any layer of it may carry instead.
        offerAlong(top.node);
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
        offerAlong(top.node);
        continue;
      }
      arena[top.node].price = price;
      arena[top.node].real = true;
      if (layer + 1 == layers) {
        // A complete run of choices, every step of it routed. Keep the
        // cheapest of them: a placeholder whose bound reaches it is dropped
        // unpriced, and `stepBudget` tells a step how much it may cost. They
        // do not arrive cheapest first — the end of the chain is reached in
        // order of the *bound* on what a run costs, and its last step is
        // priced only once it is reached.
        //
        // Not with an `accept`: then `solved` means an accepted run exists,
        // and a run recorded here could be one the caller refuses when it
        // pops.
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
      open.push({.f = f,
                 .key = keyOf(price, rest[layer][choice]),
                 .layer = layer,
                 .node = top.node});
      continue;
    }

    if (problem.admit && layer > 0) {
      // A test of the prefix put off until it is taken: refused, it goes as
      // though its last step had found no way. It is asked at most once a
      // prefix, since every real prefix comes off the list once.
      prefixOf(top.node);
      if (!problem.admit(prefix)) {
        offerAlong(top.node);
        continue;
      }
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
        // next-cheapest complete run, so the search simply goes on.
        ++out.rejected;
        if (problem.maxAccepts > 0 && out.rejected >= problem.maxAccepts) {
          out.outOfTrials = true;
          return out;
        }
        continue;
      }
      out.solved = true;
      out.optimal = !out.relaid && weight == 1000;
      out.cost = arena[top.node].price;
      out.chosen = prefix;
      if (weight != 1000) {
        // The least `g + h` left: no run through an entry still in the
        // list costs less, and the answer is a run.
        out.lowerBound = out.cost;
        for (; !open.empty(); open.pop()) {
          out.lowerBound = std::min(out.lowerBound, open.top().f);
        }
      }
      return out;
    }

    ++out.expansions;
    if (problem.expanded) {
      prefixOf(top.node);
      problem.expanded(prefix);
    }
    const auto price = arena[top.node].price;
    const auto next = layer + 1;
    for (uint32_t to = 0; to < problem.width[next]; ++to) {
      if (!offeredAlone(next, to)) {
        continue;
      }
      const auto guess = plus(price, problem.bound(layer, choice, to));
      const auto f = plus(guess, rest[next][to]);
      if (f == TRELLIS_UNREACHABLE) {
        continue;
      }
      arena.push_back({.parent = top.node,
                       .choice = to,
                       .layer = next,
                       .price = guess,
                       .real = false,
                       .pending = true});
      childrenOf[top.node].push_back(static_cast<uint32_t>(arena.size() - 1));
      open.push({.f = f,
                 .key = keyOf(guess, rest[next][to]),
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
