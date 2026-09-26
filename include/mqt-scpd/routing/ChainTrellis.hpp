/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <vector>

namespace mqt::scpd::routing {

/// The exact shortest path through a layered graph whose edges are dear to
/// price.
///
/// A trellis is a run of layers, each holding a handful of choices, where the
/// price of a step depends on the choice at both of its ends and on nothing
/// else. The feedline chain of the coupler insertion is one: a layer per
/// coupler, a node per coupler option, and the step between two of them is a
/// routed feedline edge priced by how much it turns. Pricing one step is a
/// full A* through the chip; pricing every pair of every layer is what makes
/// the exact answer expensive, and it is the only expensive thing here.
///
/// So the price of a step is asked for twice. `bound` must answer for free
/// and must never exceed what `evaluate` would answer — a lower bound, in the
/// sense an admissible heuristic is one. `evaluate` is the real thing, and
/// the search calls it only for the steps that decide the answer: it solves
/// the trellis over the bounds it holds, prices the first step of the winner
/// it has not yet priced, and solves again. When a winner comes back with
/// every step already priced, no cheaper assignment can exist, because every
/// unpriced step anywhere else is still carrying a price that cannot be too
/// high. That is the whole proof, and it is why the answer is exact however
/// weak the bound is — a weak bound costs evaluations, never correctness.
///
/// The same argument is A* with the relaxed graph as its heuristic. On a
/// trellis the relaxed graph's optimum *is* one sweep of the layers, so the
/// heuristic is recomputed exactly rather than maintained in an open list.

/// A price no assignment may have: a step that cannot be built at all.
inline constexpr uint64_t TRELLIS_UNREACHABLE =
    std::numeric_limits<uint64_t>::max();

/// One trellis to solve.
struct TrellisProblem {
  /// How many nodes each layer holds. A layer of one is a fixed waypoint.
  /// Fewer than two layers is not a problem — it has no steps to price.
  std::vector<uint32_t> width;

  /// What choosing node `node` at layer `layer` costs on its own, before any
  /// step is priced. Empty for a trellis whose whole price is in its steps.
  std::function<uint64_t(std::size_t layer, uint32_t node)> nodeCost;

  /// A lower bound on `evaluate(layer, from, to)`, answered without pricing
  /// the step. It may answer TRELLIS_UNREACHABLE for a step already known to
  /// be impossible. **It must never exceed what `evaluate` returns**: the
  /// answer is exact only as far as this holds.
  std::function<uint64_t(std::size_t layer, uint32_t from, uint32_t to)> bound;

  /// What the step from node `from` of layer `layer` to node `to` of the next
  /// layer really costs, TRELLIS_UNREACHABLE when it cannot be built. Called
  /// at most once per step.
  std::function<uint64_t(std::size_t layer, uint32_t from, uint32_t to)>
      evaluate;
};

/// What the search found.
struct TrellisResult {
  /// One node per layer. Empty when nothing was found.
  std::vector<uint32_t> chosen;

  /// The price of `chosen`, TRELLIS_UNREACHABLE when nothing was found.
  uint64_t cost = TRELLIS_UNREACHABLE;

  /// How many steps were priced — how much of the trellis the bound could
  /// not settle on its own. The figure to watch: it is the runtime.
  uint32_t evaluations = 0;

  /// Whether a complete assignment exists at all.
  bool solved = false;

  /// Per layer, whether any node of it can be got to from the first layer
  /// under the prices the search ended with, and whether any node of it can
  /// get to the last.
  ///
  /// On a trellis that came back unsolved these name the wall. The first
  /// layer `reachedFromStart` marks false is where the chain stopped going
  /// forward, so the step that shut it is the one into that layer; the last
  /// layer `reachedFromEnd` marks false is the same thing read backwards.
  /// Widening the choice anywhere else cannot help.
  ///
  /// A layer is on some complete path exactly where both are true, which on a
  /// solved trellis is every layer.
  std::vector<bool> reachedFromStart;
  std::vector<bool> reachedFromEnd;
};

/// Solve one trellis exactly, pricing as few steps as the bound allows.
[[nodiscard]] TrellisResult solveTrellis(const TrellisProblem& problem);

} // namespace mqt::scpd::routing
