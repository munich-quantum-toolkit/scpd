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

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <queue>
#include <set>
#include <span>
#include <unordered_map>
#include <utility>
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
  // The least node index of a complete run priced at `out.cost`, for
  // `ChainProblem::winnerNode`.
  uint32_t winner = UINT32_MAX;

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
        const auto guess =
            plus(arena[parent].price,
                 problem.bound(layer - 1, arena[parent].choice, choice));
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
      problem.stepNode = top.node;
      problem.winnerNode = out.solved ? winner : UINT32_MAX;
      problem.stepTested = true;
      const auto priced = problem.step(prefix, choice, redone);
      problem.stepBudget = TRELLIS_UNREACHABLE;
      ++out.routed;
      if (problem.progress && problem.progressEvery > 0 &&
          out.routed % problem.progressEvery == 0) {
        problem.progress(out, open.size());
      }
      if (problem.stop) {
        // The caller ended the search. What it found so far stands,
        // unproved.
        out.stopped = true;
        return out;
      }
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
          winner = top.node;
        } else if (!problem.accept && price == out.cost) {
          winner = std::min(winner, top.node);
        }
        if (problem.found && problem.stepTested) {
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
      if (problem.accept && !problem.accept(prefix, arena[top.node].price)) {
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
      out.optimal = !out.relaid;
      out.cost = arena[top.node].price;
      out.chosen = prefix;
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

namespace {

/// Which of two open entries the search from the back takes first: the
/// cheaper, then the one nearer the first layer, then the one made first.
struct WorseBack {
  [[nodiscard]] bool operator()(const Open& a, const Open& b) const {
    if (a.f != b.f) {
      return a.f > b.f;
    }
    if (a.layer != b.layer) {
      return a.layer > b.layer;
    }
    return a.node > b.node;
  }
};

/// What became of a step of the search from the front that was priced.
enum class Priced : std::uint8_t { Requeued, Pruned, Dead, Real, Stopped };

} // namespace

ChainSolution solveChainBidirectional(const ChainProblem& problem) {
  const auto layers = problem.width.size();
  if (!problem.stepBack || problem.accept || layers < 3) {
    return solveChainAStar(problem);
  }
  ChainSolution out;
  for (const auto width : problem.width) {
    if (width == 0) {
      return out;
    }
  }

  // The two heuristics: `rest` from a choice to the end of the chain, for
  // the search from the front, as `solveChainAStar` builds it; `lead` from
  // the start of the chain to a choice, for the search from the back.
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
  std::vector<std::vector<uint64_t>> lead(layers);
  lead[0].assign(problem.width[0], 0);
  for (std::size_t layer = 1; layer < layers; ++layer) {
    lead[layer].assign(problem.width[layer], TRELLIS_UNREACHABLE);
    for (uint32_t to = 0; to < problem.width[layer]; ++to) {
      auto least = TRELLIS_UNREACHABLE;
      for (uint32_t from = 0; from < problem.width[layer - 1]; ++from) {
        const auto reach =
            plus(lead[layer - 1][from], problem.bound(layer - 1, from, to));
        least = reach < least ? reach : least;
      }
      lead[layer][to] = least;
    }
  }

  // The search from the front, as in `solveChainAStar`. A node can be made
  // by an expansion or by a run laid again from a meeting, so every child
  // is made once (`children`), and an entry of the queue whose node has
  // been priced since it was pushed is stale.
  std::vector<Node> arena;
  std::vector<bool> dead;
  std::vector<bool> expanded;
  std::priority_queue<Open, std::vector<Open>, Worse> open;
  std::unordered_map<uint64_t, uint32_t> children;
  const auto childKey = [](const uint32_t parent, const uint32_t choice) {
    return (static_cast<uint64_t>(parent) << 32U) | choice;
  };
  uint32_t winner = UINT32_MAX;
  std::vector<uint32_t> prefix;
  const auto prefixOf = [&](const uint32_t node) {
    prefix.clear();
    for (auto at = node; at != NO_PARENT; at = arena[at].parent) {
      prefix.push_back(arena[at].choice);
    }
    std::ranges::reverse(prefix);
  };
  const auto made = [&](const Node node) {
    arena.push_back(node);
    dead.push_back(false);
    expanded.push_back(false);
    return static_cast<uint32_t>(arena.size() - 1);
  };

  // The search from the back: a node is a suffix, its choice on `layer`
  // and its parent the suffix one layer further on.
  std::vector<Node> back;
  std::priority_queue<Open, std::vector<Open>, WorseBack> openBack;
  std::vector<uint32_t> suffix;
  const auto suffixOf = [&](const uint32_t node) {
    suffix.clear();
    for (auto at = node; at != NO_PARENT; at = back[at].parent) {
      suffix.push_back(back[at].choice);
    }
  };

  // The real nodes of either search, by layer and choice, where the two
  // meet; the pairs already laid again; and the pairs still to lay.
  std::vector<std::vector<std::vector<uint32_t>>> frontAt(layers);
  std::vector<std::vector<std::vector<uint32_t>>> backAt(layers);
  for (std::size_t layer = 0; layer < layers; ++layer) {
    frontAt[layer].resize(problem.width[layer]);
    backAt[layer].resize(problem.width[layer]);
  }
  std::set<std::pair<uint32_t, uint32_t>> met;
  std::deque<std::pair<uint32_t, uint32_t>> meetings;
  // A meeting is worth laying again while the two prices together are
  // below the cheapest run so far; the cheapest partner is taken.
  const auto meet = [&](const uint32_t front, const uint32_t suffixNode,
                        const bool fromFront) {
    const auto layer = fromFront ? arena[front].layer : back[suffixNode].layer;
    const auto choice =
        fromFront ? arena[front].choice : back[suffixNode].choice;
    const auto& others =
        fromFront ? backAt[layer][choice] : frontAt[layer][choice];
    uint32_t best = NO_PARENT;
    uint64_t least = TRELLIS_UNREACHABLE;
    for (const auto other : others) {
      const auto price = fromFront ? back[other].price : arena[other].price;
      if (price < least) {
        least = price;
        best = other;
      }
    }
    if (best == NO_PARENT) {
      return;
    }
    const auto pair =
        fromFront ? std::pair{front, best} : std::pair{best, suffixNode};
    const auto together =
        plus(least, fromFront ? arena[front].price : back[suffixNode].price);
    if ((out.solved && together >= out.cost) || met.contains(pair)) {
      return;
    }
    met.insert(pair);
    meetings.push_back(pair);
  };

  // Price the last step of a node of the search from the front, as the loop
  // of `solveChainAStar` does. `queued` is the `f` of the entry it was
  // popped with, or TRELLIS_UNREACHABLE for a node of a run laid again,
  // which is priced whatever its bound now says.
  const auto price = [&](const uint32_t node, const uint64_t queued) -> Priced {
    const auto layer = arena[node].layer;
    const auto choice = arena[node].choice;
    const auto parent = arena[node].parent;
    const auto guess =
        plus(arena[parent].price,
             problem.bound(layer - 1, arena[parent].choice, choice));
    const auto again = plus(guess, rest[layer][choice]);
    if (queued != TRELLIS_UNREACHABLE && again > queued) {
      arena[node].price = guess;
      open.push({.f = again, .layer = layer, .node = node});
      return Priced::Requeued;
    }
    if (out.solved && again >= out.cost) {
      ++out.pruned;
      return Priced::Pruned;
    }
    const auto floor = plus(arena[parent].price, rest[layer][choice]);
    problem.stepBudget = out.solved && out.cost > floor ? out.cost - 1 - floor
                                                        : TRELLIS_UNREACHABLE;
    prefixOf(parent);
    int64_t redone = 0;
    problem.stepNode = node;
    problem.winnerNode = out.solved ? winner : UINT32_MAX;
    problem.stepTested = true;
    const auto priced = problem.step(prefix, choice, redone);
    problem.stepBudget = TRELLIS_UNREACHABLE;
    ++out.routed;
    if (problem.progress && problem.progressEvery > 0 &&
        out.routed % problem.progressEvery == 0) {
      problem.progress(out, open.size());
    }
    if (problem.stop) {
      out.stopped = true;
      return Priced::Stopped;
    }
    if (priced == TRELLIS_UNREACHABLE) {
      dead[node] = true;
      return Priced::Dead;
    }
    auto total = plus(arena[parent].price, priced);
    if (redone != 0 && total != TRELLIS_UNREACHABLE) {
      out.relaid = true;
      const auto by = static_cast<uint64_t>(redone < 0 ? -redone : redone);
      total = redone < 0 ? (by > total ? 0 : total - by) : plus(total, by);
    }
    const auto f = plus(total, rest[layer][choice]);
    if (f == TRELLIS_UNREACHABLE) {
      dead[node] = true;
      return Priced::Dead;
    }
    arena[node].price = total;
    arena[node].real = true;
    if (layer + 1 == layers) {
      prefixOf(node);
      if (!out.solved || total < out.cost) {
        out.solved = true;
        out.cost = total;
        out.chosen = prefix;
        winner = node;
      } else if (total == out.cost) {
        winner = std::min(winner, node);
      }
      if (problem.found && problem.stepTested) {
        problem.found(prefix, total);
      }
    }
    open.push({.f = f, .layer = layer, .node = node});
    frontAt[layer][choice].push_back(node);
    meet(node, NO_PARENT, true);
    return Priced::Real;
  };

  // Lay the run of a meeting again from the front: the prefix, then the
  // suffix's choices one step at a time, each priced by `step` against the
  // prefix before it. A step already priced is not priced again.
  const auto layAgain = [&](const uint32_t front,
                            const uint32_t suffixNode) -> bool {
    ++out.meetings;
    suffixOf(suffixNode);
    const std::vector<uint32_t> choices = suffix;
    auto at = front;
    for (std::size_t k = 1; k < choices.size(); ++k) {
      const auto choice = choices[k];
      const auto key = childKey(at, choice);
      uint32_t node = 0;
      if (const auto found = children.find(key); found != children.end()) {
        node = found->second;
        if (dead[node]) {
          return true;
        }
        if (arena[node].real) {
          at = node;
          continue;
        }
      } else {
        if (expanded[at]) {
          // The expansion left the child out: its bound reaches nothing.
          return true;
        }
        const auto layer = arena[at].layer + 1;
        node = made(
            {.parent = at,
             .choice = choice,
             .layer = layer,
             .price = plus(arena[at].price,
                           problem.bound(layer - 1, arena[at].choice, choice)),
             .real = false});
        children.emplace(key, node);
      }
      const auto result = price(node, TRELLIS_UNREACHABLE);
      if (result == Priced::Stopped) {
        return false;
      }
      if (result != Priced::Real) {
        return true;
      }
      at = node;
    }
    if (arena[at].layer + 1 == layers) {
      ++out.meetingsJoined;
    }
    return true;
  };

  // One step of the search from the back: price a suffix's step or grow
  // children from a suffix. False when `stepBack` ended the search.
  const auto stepFromTheBack = [&]() -> bool {
    while (!openBack.empty()) {
      const auto top = openBack.top();
      openBack.pop();
      const auto layer = back[top.node].layer;
      const auto choice = back[top.node].choice;
      if (!back[top.node].real) {
        const auto parent = back[top.node].parent;
        const auto guess =
            plus(back[parent].price,
                 problem.bound(layer, choice, back[parent].choice));
        const auto again = plus(guess, lead[layer][choice]);
        if (again > top.f) {
          back[top.node].price = guess;
          openBack.push({.f = again, .layer = layer, .node = top.node});
          continue;
        }
        if (out.solved && top.f >= out.cost) {
          continue;
        }
        const auto floor = plus(back[parent].price, lead[layer][choice]);
        problem.stepBudget = out.solved && out.cost > floor
                                 ? out.cost - 1 - floor
                                 : TRELLIS_UNREACHABLE;
        problem.stepNode = UINT32_MAX;
        problem.winnerNode = UINT32_MAX;
        problem.stepTested = true;
        suffixOf(parent);
        const auto priced = problem.stepBack(suffix, choice);
        problem.stepBudget = TRELLIS_UNREACHABLE;
        ++out.routedBack;
        if (problem.stop) {
          out.stopped = true;
          return false;
        }
        if (priced == TRELLIS_UNREACHABLE) {
          return true;
        }
        const auto total = plus(back[parent].price, priced);
        const auto f = plus(total, lead[layer][choice]);
        if (f == TRELLIS_UNREACHABLE) {
          return true;
        }
        back[top.node].price = total;
        back[top.node].real = true;
        openBack.push({.f = f, .layer = layer, .node = top.node});
        backAt[layer][choice].push_back(top.node);
        meet(NO_PARENT, top.node, false);
        return true;
      }
      if (layer == 0) {
        continue;
      }
      ++out.expansionsBack;
      const auto total = back[top.node].price;
      for (uint32_t from = 0; from < problem.width[layer - 1]; ++from) {
        const auto guess = plus(total, problem.bound(layer - 1, from, choice));
        const auto f = plus(guess, lead[layer - 1][from]);
        if (f == TRELLIS_UNREACHABLE) {
          continue;
        }
        back.push_back({.parent = top.node,
                        .choice = from,
                        .layer = static_cast<uint32_t>(layer - 1),
                        .price = guess,
                        .real = false});
        openBack.push({.f = f,
                       .layer = static_cast<uint32_t>(layer - 1),
                       .node = static_cast<uint32_t>(back.size() - 1)});
      }
    }
    return true;
  };

  for (uint32_t choice = 0; choice < problem.width[0]; ++choice) {
    if (rest[0][choice] == TRELLIS_UNREACHABLE) {
      continue;
    }
    const auto node = made({.parent = NO_PARENT,
                            .choice = choice,
                            .layer = 0,
                            .price = 0,
                            .real = true});
    children.emplace(childKey(NO_PARENT, choice), node);
    frontAt[0][choice].push_back(node);
    open.push({.f = rest[0][choice], .layer = 0, .node = node});
  }
  // A suffix of one layer has no step yet, so it is real from the start.
  for (uint32_t choice = 0; choice < problem.width[layers - 1]; ++choice) {
    const auto f = lead[layers - 1][choice];
    if (f == TRELLIS_UNREACHABLE) {
      continue;
    }
    back.push_back({.parent = NO_PARENT,
                    .choice = choice,
                    .layer = static_cast<uint32_t>(layers - 1),
                    .price = 0,
                    .real = true});
    openBack.push({.f = f,
                   .layer = static_cast<uint32_t>(layers - 1),
                   .node = static_cast<uint32_t>(back.size() - 1)});
    backAt[layers - 1][choice].push_back(
        static_cast<uint32_t>(back.size() - 1));
  }

  bool backTurn = false;
  while (!open.empty()) {
    while (!meetings.empty()) {
      const auto [front, suffixNode] = meetings.front();
      meetings.pop_front();
      if (!layAgain(front, suffixNode)) {
        return out;
      }
    }
    if (backTurn && !openBack.empty()) {
      backTurn = false;
      if (!stepFromTheBack()) {
        return out;
      }
      continue;
    }
    if (open.empty()) {
      break;
    }
    const auto top = open.top();
    open.pop();
    const auto node = top.node;
    const auto layer = arena[node].layer;
    const auto choice = arena[node].choice;
    if (dead[node] || expanded[node]) {
      continue;
    }
    if (!arena[node].real) {
      const auto result = price(node, top.f);
      if (result == Priced::Stopped) {
        return out;
      }
      backTurn = backTurn || result == Priced::Real || result == Priced::Dead;
      continue;
    }
    if (top.f != plus(arena[node].price, rest[layer][choice])) {
      // Pushed as a placeholder and priced since, by a run laid again.
      continue;
    }

    out.reached = std::max(out.reached, layer);
    if (layer + 1 == layers) {
      prefixOf(node);
      out.solved = true;
      out.optimal = !out.relaid;
      out.cost = arena[node].price;
      out.chosen = prefix;
      return out;
    }

    ++out.expansions;
    expanded[node] = true;
    if (problem.expanded) {
      prefixOf(node);
      problem.expanded(prefix);
    }
    const auto total = arena[node].price;
    const auto next = layer + 1;
    for (uint32_t to = 0; to < problem.width[next]; ++to) {
      const auto key = childKey(node, to);
      if (children.contains(key)) {
        continue;
      }
      const auto guess = plus(total, problem.bound(layer, choice, to));
      const auto f = plus(guess, rest[next][to]);
      if (f == TRELLIS_UNREACHABLE) {
        continue;
      }
      const auto child = made({.parent = node,
                               .choice = to,
                               .layer = next,
                               .price = guess,
                               .real = false});
      children.emplace(key, child);
      open.push({.f = f, .layer = next, .node = child});
    }
  }
  out.optimal = true;
  return out;
}

} // namespace mqt::scpd::routing
