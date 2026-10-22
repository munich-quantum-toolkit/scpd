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

#include "mqt-scpd/pipeline/mqt_scpd_pipeline_export.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace mqt::scpd::pipeline {

/// A whole-chip combination of chain states for the global SAT search of
/// the coupler insertion: per chain the number of the state it stands on,
/// 0 for the state the commit left.
using Combination = std::vector<std::size_t>;

/// A combination the whole-chip check found UNSAT.
struct FailedCombination {
  /// The combination.
  Combination pick;
  /// How many lost wires and edges over the check found against it; the
  /// largest value for a combination with an edge that has no way.
  std::size_t against = 0;
  /// The chains the check names: the conflicts of the combination.
  std::set<std::uint32_t> named;
  /// The number of the check, 0 for the commit.
  std::uint32_t check = 0;
};

/// One round of the conflict-directed stage of the global SAT search.
struct CombinationRound {
  /// The failed combination the round starts from.
  FailedCombination base;
  /// The chains the round moves, one per combination.
  std::vector<std::uint32_t> movable;
  /// The combinations of the round, in the order they are tried.
  std::vector<Combination> picks;
};

/// What the whole-chip check said against a combination, for
/// `namedChains`.
struct CheckFinding {
  /// The feedline edges (`f<k>`) an edge over lies on or ends at, a
  /// chain's terminal edges included.
  std::set<std::string> touching;
  /// The lost wires and the edges over, by name.
  std::vector<std::string> against;
  /// The middle of the line of every bottleneck over its capacity.
  std::vector<std::array<double, 2>> overGates;
  /// Chains named by another rule of the caller, such as the chains around
  /// a feedline edge with no way.
  std::set<std::uint32_t> named;
};

/// What the chip the check judged holds, for `namedChains`.
struct ChipChains {
  /// The chain of every feedline edge by name, terminal edges included.
  std::map<std::string, std::uint32_t> chainOfFeedline;
  /// Per chain, the wires its feedline holds.
  std::vector<std::set<std::string>> wiresOfChain;
  /// The centre of the pad every coupler stands on, with its chain.
  std::vector<std::pair<std::array<double, 2>, std::uint32_t>> couplers;
};

/// The chains a failed whole-chip check names, for the conflict-directed
/// stage of the global SAT search: the chains of the feedline edges an edge
/// over lies on or ends at, a terminal edge included; the chains whose
/// feedline holds a lost wire; and `CheckFinding::named`. When none of
/// these names a chain, every bottleneck over its capacity names the chain
/// of the coupler whose pad centre lies nearest the middle of its line, the
/// chain with the smaller index on a tie.
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT std::set<std::uint32_t>
namedChains(const CheckFinding& finding, const ChipChains& chip);

/// Whether combination `a` is tried before combination `b`: fewer moved
/// chains first, then the smaller sum of the state numbers, then the moved
/// chains and their states in chain order, the smaller first. The order is
/// total, so the trials never depend on the order the combinations were
/// made in.
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT bool triedBefore(const Combination& a,
                                                        const Combination& b);

/// The next round of the conflict-directed stage, or nothing when no
/// failed combination names a chain to move.
///
/// The base is the combination of `failed` with the fewest lost wires and
/// edges over, the earliest check on a tie, that names a chain it does not
/// move and that has a state to move to. The round holds every combination
/// that moves one such chain onto one of its states on top of the base and
/// that `tried` does not hold, in the order of `triedBefore`.
///
/// @param failed The failed combinations of the last round.
/// @param statesOf How many states a chain has, the state of the commit
/// included. It is asked for the named chains a base does not move, bases
/// in order and chains in chain order, so that a caller can collect the
/// states of a chain when they are first needed.
/// @param tried The combinations tried so far.
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT std::optional<CombinationRound>
nextCombinationRound(std::vector<FailedCombination> failed,
                     const std::function<std::size_t(std::uint32_t)>& statesOf,
                     const std::set<Combination>& tried);

} // namespace mqt::scpd::pipeline
