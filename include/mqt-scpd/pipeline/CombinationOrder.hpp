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

/// A bottleneck over its capacity as the whole-chip check names it: its
/// name, the names of the walls at its two ends and the middle of its line.
struct OverGate {
  std::string name;
  std::vector<std::string> ends;
  std::array<double, 2> middle{};
};

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
  /// The bottlenecks over its capacity the check found (the pair rounds).
  std::vector<OverGate> gates;
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

/// Which end of a chain (a feedline): its start, the first coupler, or its
/// end, the last coupler.
enum class ChainSide : std::uint8_t { Start, End };

/// One end of a chain.
struct ChainEnd {
  std::uint32_t chain = 0;
  ChainSide side = ChainSide::Start;
  auto operator<=>(const ChainEnd&) const = default;
};

/// Two chain ends that compete for a bottleneck over its capacity, the
/// lower chain first, and the bottleneck that named them first.
struct ConflictPair {
  ChainEnd first;
  ChainEnd second;
  std::string gate;
};

/// What `conflictPairs` knows of the chip: per feedline edge (`f<k>`) its
/// chain, its two waypoints and whether it is a terminal edge; per resonator
/// the chain and waypoint of its coupler; per chain its number of waypoints,
/// the waypoints of its start and end couplers and their pad centres.
struct FeedlineMap {
  struct Feedline {
    std::uint32_t chain = 0;
    std::size_t from = 0;
    std::size_t to = 0;
    bool terminal = false;
  };
  std::map<std::string, Feedline> feedlines;
  std::map<std::string, std::pair<std::uint32_t, std::size_t>> resonators;
  std::vector<std::size_t> size;
  std::vector<std::array<std::size_t, 2>> ends;
  std::vector<std::array<std::array<double, 2>, 2>> centres;
};

/// The chain ends that compete for the bottlenecks over, for the pair rounds
/// of the global SAT search. Each end wall of a bottleneck names a chain end:
/// a terminal edge (`fN`, `fN at its coupler`, `fN at its launcher`) the end
/// it belongs to; an inner edge the end nearer it by waypoint; a resonator's
/// lead, pad or port the end nearer its coupler's waypoint; any other wall
/// none. When the two walls name two chains, they are the pair; when they
/// name one chain only, its partner is the end coupler of another chain
/// whose pad centre lies nearest the middle of the bottleneck (the lower
/// chain, then its start, on a tie). A bottleneck that names no chain makes
/// no pair. Each pair comes once, in the order of the bottleneck that named
/// it first.
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT std::vector<ConflictPair>
conflictPairs(const std::vector<OverGate>& gates, const FeedlineMap& chip);

/// The combinations of one pair round of the global SAT search, on top of
/// `base`: chain `a` alone on each of `statesA` (state numbers, best first),
/// then chain `b` alone on each of `statesB`, then both, by the sum of the
/// two states' ranks in their lists, then by a's rank; every combination
/// `tried` holds left out.
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT std::vector<Combination>
pairRoundPicks(const Combination& base, std::uint32_t a,
               const std::vector<std::size_t>& statesA, std::uint32_t b,
               const std::vector<std::size_t>& statesB,
               const std::set<Combination>& tried);

} // namespace mqt::scpd::pipeline
