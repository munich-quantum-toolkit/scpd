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
/// case. The search has no time limit: it runs until it proves an optimum,
/// until it has seen every run of choices, or until `maxAccepts` stops it.
/// The answer is therefore the same on every machine and at every speed.
/// `progress` reports how far the search has come while it runs.

struct ChainSolution;

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

  /// Set by the search before every call of `step`: the most the step may
  /// cost and still leave the prefix able to beat the cheapest complete run
  /// priced so far, or TRELLIS_UNREACHABLE while no run has been priced. A
  /// `step` may stop early once it knows it will cost more — the answer it
  /// gives then is not read, because the prefix cannot win.
  mutable uint64_t stepBudget = TRELLIS_UNREACHABLE;

  /// Set with `stepBudget`: the largest step budget any choice of the same
  /// layer gets under the same prefix, or TRELLIS_UNREACHABLE while no run
  /// has been priced. The cheapest complete run only falls, so a later
  /// `stepBudget` for any of those steps is at most this. A `step` that
  /// prices the steps into every choice at once may stop at this budget
  /// and hold each step to its own budget when it is asked for it.
  mutable uint64_t siblingBudget = TRELLIS_UNREACHABLE;

  /// Set with `stepBudget`: the `f` of the step being priced — the price of
  /// its prefix plus its bound plus the run of bounds after it — and, for
  /// every choice of the same layer, the `f` of its step under the same
  /// prefix while that step waits on the open list and is below the
  /// cheapest complete run priced so far; TRELLIS_UNREACHABLE for a choice
  /// whose step is priced, dropped, never offered or at or above that run,
  /// since the search will never price it. Under plain A\* every key listed
  /// is at least `stepKey`. A `step` that prices several steps at once may
  /// pick the ones the search will reach soon by these keys.
  mutable uint64_t stepKey = TRELLIS_UNREACHABLE;
  mutable std::vector<uint64_t> siblingKeys;

  /// Called every time the search completes a run of choices whose every
  /// step is routed — the run and what it costs. They arrive in no
  /// particular order of price: a run is reached in order of the *bound* on
  /// what it costs, and the last step of it is priced only once it is
  /// reached.
  /// Optional.
  std::function<void(std::span<const uint32_t> chosen, uint64_t cost)> found;

  /// Whether the search may stop on a complete run of choices, asked at the
  /// moment it would otherwise hand that run back as the answer: the run
  /// and its real price. `false` refuses it, and the search runs on to the
  /// next-cheapest complete run. Every entry of the open list carries a
  /// lower bound on the true cost through it, so complete runs pop with
  /// their real price in non-decreasing order — unless a step was `redone`,
  /// which already costs the proof — and the next real pop after a refusal
  /// is the next-cheapest run there is. Optional; without it the first
  /// complete run to pop is the answer, as before.
  ///
  /// With it set, `ChainSolution::solved` means an **accepted** run exists.
  /// A refused run is never handed back, not even when `maxAccepts` stops
  /// the search: the caller asked for a run that passes its test and gets
  /// none rather than one that failed it. `found` keeps firing for every
  /// complete run as it is priced, accepted or not. The coupler insertion's
  /// targeted repair is what this is for (2026-10-04): its test of a run is
  /// a local rip-up and re-route that takes seconds, and `maxAccepts` bounds
  /// how often it is asked.
  std::function<bool(std::span<const uint32_t> chosen, uint64_t cost)> accept;

  /// How many complete runs `accept` may refuse before the search stops with
  /// nothing, zero for no limit. `ChainSolution::outOfTrials` says when it
  /// struck.
  uint32_t maxAccepts = 0;

  /// Called after every `progressEvery` priced steps with what the search
  /// has found so far and how many entries its open list holds. It reports
  /// and decides nothing. Optional; zero `progressEvery` calls it never.
  std::function<void(const ChainSolution& sofar, std::size_t open)> progress;
  uint32_t progressEvery = 0;

  /// Called when a prefix of two layers or more whose every step is priced
  /// comes off the open list, before it is expanded or taken as the
  /// answer. `false` drops it as though its last step had found no way.
  /// This is how a caller puts off a test of a step until the search takes
  /// the step: a test that only refuses, asked here instead of in `step`,
  /// leaves the prefixes the search takes and the answer as they are, since
  /// a refused prefix grows nothing either way. Optional.
  std::function<bool(std::span<const uint32_t> prefix)> admit;

  /// Called every time the search grows children from a prefix, with the
  /// prefix. It reports and decides nothing. Optional.
  std::function<void(std::span<const uint32_t> prefix)> expanded;

  /// The weight `w` of weighted A\*, in thousandths: 1000 (plain A\*) or
  /// more. The open list is ordered by `g + w * h`, where `g` is what a
  /// prefix's priced steps cost and `h` the run of bounds from its last
  /// layer to the end, `w * h` rounded to the nearest whole number with
  /// halves up. The first complete run to come out is the answer, and it
  /// costs at most `w` times the optimum: every entry still in the list
  /// holds a lower bound on what a run through it costs.
  /// `ChainSolution::lowerBound` says the least of those bounds at the
  /// stop. At 1000 the search is the plain one, step for step.
  uint32_t weightThousandths = 1000;

  /// Classes of interchangeable choices, empty for none:
  /// `classOf[layer][choice]` is the class's representative, its member
  /// with the smallest index. The caller vouches that two members of a
  /// class cost the same on every step from every choice of the layer
  /// before and to every choice of the layer after, so that swapping one
  /// for another changes no run's price. The search offers the
  /// representative alone. The next member of the class is offered in a
  /// prefix's place when the step into that prefix's choice finds no way or
  /// is refused, when `step` sets `askNext`, and when a step of any prefix
  /// that extends it does: every refusal offers, at every layer of the
  /// refused prefix, the next member after the choice there, at most once
  /// per parent.
  ///
  /// Complete: take any run R the search with no classes could answer.
  /// Replace each of its choices by its class's representative and call
  /// that run R0; it costs what R costs. Either the search reaches R0
  /// whole, or a step of a prefix of R0 finds no way or is refused; then
  /// the next member at every layer of that prefix is offered, and so on
  /// along the members in order, so the search goes on to every run that
  /// differs from R0 only in members until one carries or none is left. No
  /// run is lost that differs from a refused one in a member; the runs
  /// never offered are those whose every prefix carried with
  /// representatives, which an offered run of the same price stands for.
  std::vector<std::vector<uint32_t>> classOf;

  /// Set by `step` to have the next member of the class of the choice it
  /// prices offered as well, though the step carried: for a step whose
  /// price is not the one its class was built on.
  mutable bool askNext = false;
};

/// What the search found.
struct ChainSolution {
  /// One choice per layer. Empty when nothing was found.
  std::vector<uint32_t> chosen;

  /// The price of `chosen`, TRELLIS_UNREACHABLE when nothing was found.
  uint64_t cost = TRELLIS_UNREACHABLE;

  /// How many prefixes the search grew children from.
  uint32_t expansions = 0;
  /// Placeholders dropped unpriced because their bound already reached
  /// the cheapest complete run priced so far.
  uint32_t pruned = 0;

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
  /// every run of choices there is and none joined the chain. False on a
  /// solved answer of weighted A\* (`ChainProblem::weightThousandths`).
  bool optimal = false;

  /// Whether `step` ever answered with a `redone` price — whether the search
  /// ever had to lay an earlier step again to lay a later one. That is what
  /// costs it the proof: a prefix can then be cheaper than the prefix it
  /// extends, and A\* rests on the opposite.
  bool relaid = false;

  /// How many members of a class were offered in a representative's place
  /// (`ChainProblem::classOf`).
  uint32_t membersOffered = 0;

  /// Under `ChainProblem::weightThousandths` above 1000: a lower bound on
  /// the cheapest run there is, the least `g + h` left in the open list
  /// when the search stopped, or the answer's cost when that is less.
  /// TRELLIS_UNREACHABLE otherwise.
  uint64_t lowerBound = TRELLIS_UNREACHABLE;

  /// How many complete runs `accept` refused. Zero without an `accept`.
  uint32_t rejected = 0;

  /// Whether `maxAccepts` stopped the search: every run it was allowed to
  /// offer was refused and it was not allowed to offer another. A search
  /// that ran out of trials and one that saw every run there is must never
  /// be reported by one name.
  bool outOfTrials = false;
};

/// Solve one chain exactly, pricing as few steps as the bound allows.
[[nodiscard]] ChainSolution solveChainAStar(const ChainProblem& problem);

} // namespace mqt::scpd::routing
