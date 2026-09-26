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

#include <cstddef>
#include <cstdint>
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

/// The prices of every step, layer by layer, in one flat block per layer.
/// `price[layer][(from * width[layer + 1]) + to]` is what the search currently
/// believes the step costs, and `priced` says whether that belief is the real
/// answer or still the bound.
struct Steps {
  std::vector<std::vector<uint64_t>> price;
  std::vector<std::vector<bool>> priced;
};

} // namespace

TrellisResult solveTrellis(const TrellisProblem& problem) {
  const auto layers = problem.width.size();
  TrellisResult result;
  result.reachedFromStart.assign(layers, false);
  result.reachedFromEnd.assign(layers, false);
  if (layers == 0) {
    return result;
  }
  for (const auto width : problem.width) {
    if (width == 0) {
      // A layer with no choice at all shuts the trellis; nothing to search.
      return result;
    }
  }

  const auto at = [&](const std::size_t layer, const uint32_t node) {
    return problem.nodeCost ? problem.nodeCost(layer, node) : 0U;
  };

  // Every step starts at its bound. Nothing is routed here.
  Steps steps;
  steps.price.resize(layers == 0 ? 0 : layers - 1);
  steps.priced.resize(steps.price.size());
  for (std::size_t layer = 0; layer + 1 < layers; ++layer) {
    const auto from = problem.width[layer];
    const auto to = problem.width[layer + 1];
    steps.price[layer].assign(static_cast<std::size_t>(from) * to, 0);
    steps.priced[layer].assign(static_cast<std::size_t>(from) * to, false);
    for (uint32_t i = 0; i < from; ++i) {
      for (uint32_t j = 0; j < to; ++j) {
        steps.price[layer][(static_cast<std::size_t>(i) * to) + j] =
            problem.bound(layer, i, j);
      }
    }
  }

  // One sweep of the layers: the cheapest way to each node of each layer
  // under the prices held now, and where it came from. Ties go to the lowest
  // node index, so two runs of the same trellis answer the same.
  std::vector<std::vector<uint64_t>> best(layers);
  std::vector<std::vector<uint32_t>> cameFrom(layers);
  const auto sweep = [&]() {
    for (std::size_t layer = 0; layer < layers; ++layer) {
      best[layer].assign(problem.width[layer], TRELLIS_UNREACHABLE);
      cameFrom[layer].assign(problem.width[layer], 0);
    }
    for (uint32_t node = 0; node < problem.width[0]; ++node) {
      best[0][node] = at(0, node);
    }
    for (std::size_t layer = 0; layer + 1 < layers; ++layer) {
      const auto to = problem.width[layer + 1];
      for (uint32_t j = 0; j < to; ++j) {
        auto least = TRELLIS_UNREACHABLE;
        uint32_t from = 0;
        for (uint32_t i = 0; i < problem.width[layer]; ++i) {
          const auto step =
              steps.price[layer][(static_cast<std::size_t>(i) * to) + j];
          const auto reach = plus(best[layer][i], step);
          if (reach < least) {
            least = reach;
            from = i;
          }
        }
        best[layer + 1][j] = plus(least, at(layer + 1, j));
        cameFrom[layer + 1][j] = from;
      }
    }
  };

  // The cheapest assignment under the prices held now, read back through the
  // trail the sweep left.
  const auto winner = [&](std::vector<uint32_t>& into) {
    auto least = TRELLIS_UNREACHABLE;
    uint32_t end = 0;
    for (uint32_t node = 0; node < problem.width[layers - 1]; ++node) {
      if (best[layers - 1][node] < least) {
        least = best[layers - 1][node];
        end = node;
      }
    }
    if (least == TRELLIS_UNREACHABLE) {
      into.clear();
      return TRELLIS_UNREACHABLE;
    }
    into.assign(layers, 0);
    into[layers - 1] = end;
    for (std::size_t layer = layers - 1; layer > 0; --layer) {
      into[layer - 1] = cameFrom[layer][into[layer]];
    }
    return least;
  };

  std::vector<uint32_t> path;
  for (;;) {
    sweep();
    const auto cost = winner(path);
    if (cost == TRELLIS_UNREACHABLE) {
      break;
    }
    // The first step of the winner that is still carrying its bound. Price
    // it: either it holds, and the step after it is asked next, or it rises
    // and the next sweep picks a different winner.
    bool settled = true;
    for (std::size_t layer = 0; layer + 1 < layers; ++layer) {
      const auto to = problem.width[layer + 1];
      const auto cell =
          (static_cast<std::size_t>(path[layer]) * to) + path[layer + 1];
      if (steps.priced[layer][cell]) {
        continue;
      }
      steps.price[layer][cell] =
          problem.evaluate(layer, path[layer], path[layer + 1]);
      steps.priced[layer][cell] = true;
      ++result.evaluations;
      settled = false;
      break;
    }
    if (settled) {
      // Every step of the winner is real, and every step it does not use is
      // priced at or below what it would really cost. Nothing can undercut
      // it.
      result.chosen = path;
      result.cost = cost;
      result.solved = true;
      break;
    }
  }

  // How far the trellis carries from each end under the prices it ended
  // with. On one that came back unsolved, the first layer nothing reaches is
  // the wall, and the step into it is what shut the chain.
  {
    sweep();
    std::vector<std::vector<uint64_t>> rest(layers);
    for (std::size_t layer = 0; layer < layers; ++layer) {
      rest[layer].assign(problem.width[layer], TRELLIS_UNREACHABLE);
    }
    for (uint32_t node = 0; node < problem.width[layers - 1]; ++node) {
      rest[layers - 1][node] = 0;
    }
    for (std::size_t layer = layers - 1; layer > 0; --layer) {
      const auto to = problem.width[layer];
      for (uint32_t i = 0; i < problem.width[layer - 1]; ++i) {
        auto least = TRELLIS_UNREACHABLE;
        for (uint32_t j = 0; j < to; ++j) {
          const auto step =
              steps.price[layer - 1][(static_cast<std::size_t>(i) * to) + j];
          const auto reach = plus(step, rest[layer][j]);
          least = reach < least ? reach : least;
        }
        rest[layer - 1][i] = least;
      }
    }
    for (std::size_t layer = 0; layer < layers; ++layer) {
      for (uint32_t node = 0; node < problem.width[layer]; ++node) {
        if (best[layer][node] != TRELLIS_UNREACHABLE) {
          result.reachedFromStart[layer] = true;
        }
        if (rest[layer][node] != TRELLIS_UNREACHABLE) {
          result.reachedFromEnd[layer] = true;
        }
      }
    }
  }
  return result;
}

} // namespace mqt::scpd::routing
