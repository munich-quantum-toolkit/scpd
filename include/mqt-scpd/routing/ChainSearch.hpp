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

#include "mqt-scpd/routing/ChainTrellis.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <span>
#include <vector>

namespace mqt::scpd::routing {

/// The exact cheapest run of choices along a chain whose steps block each
/// other.
///
/// `solveTrellis` beside this one solves the same chain far more cheaply,
/// and it rests on an assumption this one drops: that the price of a step
/// depends on the choice at both of its ends and on nothing else. For the
/// feedline chain of the coupler insertion that assumption is false. Two
/// feedline edges that meet at a coupler can block each other, so what a
/// step costs depends on what was laid before it — the trellis prices a pair
/// that cannot both be built as though it could, and the commit is where an
/// edge is lost.
///
/// So a node here is a **prefix**: the choices made for layers `0..k`. The
/// search asks `step` for the real price of laying the next one against the
/// ways that prefix has already laid, and a prefix that blocks itself simply
/// costs everything and the search takes another combination. That is the
/// behaviour no width of trellis node can express.
///
/// It is A\* over those prefixes. `bound` supplies the heuristic: the
/// cheapest run of bounds from a layer to the end, which is the optimum of
/// the relaxation in which no step blocks any other. Because `bound` never
/// exceeds what `step` answers **under any prefix**, that heuristic never
/// overestimates and the answer is the true optimum.
///
/// A step is priced when the search reaches it, not when it is generated: a
/// child is pushed carrying `bound` for its step, and the real price is paid
/// only when it is popped, whereupon its cost is corrected and it goes back
/// on the queue. Only a node popped with its own step already real is
/// expanded. The placeholder is a lower bound, so this changes nothing about
/// the answer, and it means a step is routed exactly when the search commits
/// to looking at it. `solveTrellis` defers its own pricing the same way.
///
/// Nothing is merged. The node count is therefore exponential in the worst
/// case, and `budget` is the answer to that: when it runs out the search
/// hands back the cheapest complete run of choices it has found — every step
/// of it routed, and every one of them routed against the run itself — and
/// says that it did not prove it the cheapest.

/// One chain to solve.
struct ChainProblem {
  /// How many choices each layer offers. A layer of one is a fixed waypoint.
  std::vector<uint32_t> width;

  /// A lower bound on the price of the step from choice `from` of layer
  /// `layer` to choice `to` of the next, **under any prefix**. It must
  /// answer for free: the search asks it `n x n` per layer up front.
  std::function<uint64_t(std::size_t layer, uint32_t from, uint32_t to)> bound;

  /// The real price of the step into choice `to`, laid against the ways
  /// `prefix` has already laid. `prefix` holds the choices of layers
  /// `0..prefix.size() - 1`, so the step runs from layer
  /// `prefix.size() - 1` into layer `prefix.size()`.
  /// TRELLIS_UNREACHABLE when it cannot be built at all.
  ///
  /// `redone` comes back holding what this call changed about the price of
  /// the prefix itself, and is zero in the ordinary case. It is not zero
  /// when laying this step meant laying an earlier one again — which the
  /// coupler insertion does when the two ways at one end of a chain will
  /// only both fit if the later one is laid first. A prefix whose price was
  /// redone can be cheaper than the prefix it extends, so the search can no
  /// longer prove its answer the cheapest and says so through
  /// `ChainSolution::relaid`.
  std::function<uint64_t(std::span<const uint32_t> prefix, uint32_t to,
                         int64_t& redone)>
      step;

  /// How long the search may run before it settles for the best complete run
  /// of choices it has found. Zero is no limit, and then the answer is
  /// always the true optimum.
  std::chrono::nanoseconds budget{0};

  /// Called every time the search completes a run of choices whose every
  /// step is routed — the run and what it costs. They arrive in no
  /// particular order of price: a run is reached in order of the *bound* on
  /// what it costs, and the last step of it is priced only once it is
  /// reached. `ChainSolution::chosen` keeps the cheapest of them.
  /// Optional.
  std::function<void(std::span<const uint32_t> chosen, uint64_t cost)> found;
};

/// What the search found.
struct ChainSolution {
  /// One choice per layer. Empty when nothing was found.
  std::vector<uint32_t> chosen;

  /// The price of `chosen`, TRELLIS_UNREACHABLE when nothing was found.
  uint64_t cost = TRELLIS_UNREACHABLE;

  /// How many prefixes the search grew children from.
  uint32_t expansions = 0;

  /// How many steps were really priced — the figure to watch, because every
  /// one of them is a full search of the chip.
  uint32_t routed = 0;

  /// The deepest layer a prefix whose every step is real ever got to. On a
  /// chain that came back unsolved this names the wall: nothing reaches past
  /// that layer, so the step out of it is what shut the chain and widening
  /// the choice anywhere earlier cannot help.
  uint32_t reached = 0;

  /// Whether a complete run of choices exists at all.
  bool solved = false;

  /// Whether the search settled the question. True when it proved `chosen`
  /// the cheapest run there is, and true on an unsolved answer when it saw
  /// every run of choices there is and none joined the chain.
  bool optimal = false;

  /// Whether the budget stopped the search. This is what separates a chain
  /// that **has** no answer from one the search did not have time to find,
  /// and the two must never be reported by one name.
  bool outOfTime = false;

  /// Whether `step` ever answered with a `redone` price — whether the search
  /// ever had to lay an earlier step again to lay a later one. That is what
  /// costs it the proof: a prefix can then be cheaper than the prefix it
  /// extends, and A\* rests on the opposite.
  bool relaid = false;
};

/// Solve one chain exactly, pricing as few steps as the bound allows.
[[nodiscard]] ChainSolution solveChainAStar(const ChainProblem& problem);

} // namespace mqt::scpd::routing
