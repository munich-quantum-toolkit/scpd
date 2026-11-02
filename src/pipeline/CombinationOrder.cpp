/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/pipeline/CombinationOrder.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <optional>
#include <regex>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace mqt::scpd::pipeline {

namespace {

/// The key `triedBefore` compares: the number of moved chains, the sum of
/// their state numbers, and the moved chains with their states.
[[nodiscard]] auto rankOf(const Combination& pick) {
  std::size_t moved = 0;
  std::size_t sum = 0;
  std::vector<std::pair<std::size_t, std::size_t>> moves;
  for (std::size_t chain = 0; chain < pick.size(); ++chain) {
    if (pick[chain] != 0) {
      ++moved;
      sum += pick[chain];
      moves.emplace_back(chain, pick[chain]);
    }
  }
  return std::tuple{moved, sum, std::move(moves)};
}

} // namespace

std::set<std::uint32_t> namedChains(const CheckFinding& finding,
                                    const ChipChains& chip) {
  auto out = finding.named;
  for (const auto& name : finding.touching) {
    if (const auto found = chip.chainOfFeedline.find(name);
        found != chip.chainOfFeedline.end()) {
      out.insert(found->second);
    }
  }
  for (std::uint32_t chain = 0; chain < chip.wiresOfChain.size(); ++chain) {
    if (std::ranges::any_of(finding.against, [&](const std::string& name) {
          return chip.wiresOfChain[chain].contains(name);
        })) {
      out.insert(chain);
    }
  }
  if (!out.empty()) {
    return out;
  }
  for (const auto& [x, y] : finding.overGates) {
    auto best = std::numeric_limits<double>::infinity();
    auto chain = std::numeric_limits<std::uint32_t>::max();
    for (const auto& [centre, owner] : chip.couplers) {
      const auto distance = std::hypot(centre[0] - x, centre[1] - y);
      if (distance < best || (distance == best && owner < chain)) {
        best = distance;
        chain = owner;
      }
    }
    if (chain != std::numeric_limits<std::uint32_t>::max()) {
      out.insert(chain);
    }
  }
  return out;
}

bool triedBefore(const Combination& a, const Combination& b) {
  return rankOf(a) < rankOf(b);
}

std::optional<CombinationRound>
nextCombinationRound(std::vector<FailedCombination> failed,
                     const std::function<std::size_t(std::uint32_t)>& statesOf,
                     const std::set<Combination>& tried) {
  std::ranges::stable_sort(failed, [](const FailedCombination& a,
                                      const FailedCombination& b) {
    return a.against != b.against ? a.against < b.against : a.check < b.check;
  });
  for (auto& base : failed) {
    CombinationRound round;
    for (const auto chain : base.named) {
      if (chain >= base.pick.size() || base.pick[chain] != 0) {
        continue;
      }
      const auto states = statesOf(chain);
      if (states < 2) {
        continue;
      }
      round.movable.push_back(chain);
      for (std::size_t state = 1; state < states; ++state) {
        auto pick = base.pick;
        pick[chain] = state;
        if (!tried.contains(pick)) {
          round.picks.push_back(std::move(pick));
        }
      }
    }
    if (round.picks.empty()) {
      continue;
    }
    std::ranges::stable_sort(round.picks, triedBefore);
    round.base = std::move(base);
    return round;
  }
  return std::nullopt;
}

std::vector<ConflictPair> conflictPairs(const std::vector<OverGate>& gates,
                                        const FeedlineMap& chip) {
  static const std::regex feedline(R"(^(f\d+)(?: at its (?:coupler|launcher))?(?:,.*)?$)");
  static const std::regex lead(R"(^(?:lead|pad) of ([^,]+)(?:,.*)?$)");
  static const std::regex port(R"(^(\S+) at its (?:source|target)(?:,.*)?$)");
  // The end nearer a waypoint of a chain, the start on a tie.
  const auto nearer = [&](const std::uint32_t chain, const std::size_t at) {
    const auto [start, end] = chip.ends[chain];
    const auto toStart = at > start ? at - start : start - at;
    const auto toEnd = at > end ? at - end : end - at;
    return toEnd < toStart ? ChainSide::End : ChainSide::Start;
  };
  const auto endOfWall = [&](const std::string& name) -> std::optional<ChainEnd> {
    std::smatch match;
    if (std::regex_match(name, match, feedline)) {
      const auto found = chip.feedlines.find(match[1].str());
      if (found == chip.feedlines.end() || found->second.chain >= chip.ends.size()) {
        return std::nullopt;
      }
      const auto& edge = found->second;
      if (edge.terminal) {
        return ChainEnd{.chain = edge.chain,
                        .side = edge.from == 0 ? ChainSide::Start : ChainSide::End};
      }
      return ChainEnd{.chain = edge.chain,
                      .side = edge.from < chip.size[edge.chain] / 2 ? ChainSide::Start
                                                                    : ChainSide::End};
    }
    if (std::regex_match(name, match, lead) || std::regex_match(name, match, port)) {
      const auto found = chip.resonators.find(match[1].str());
      if (found == chip.resonators.end() || found->second.first >= chip.ends.size()) {
        return std::nullopt;
      }
      return ChainEnd{.chain = found->second.first,
                      .side = nearer(found->second.first, found->second.second)};
    }
    return std::nullopt;
  };
  // The end coupler of another chain than `chain` whose pad centre lies
  // nearest `middle`.
  const auto partnerOf = [&](const std::uint32_t chain,
                             const std::array<double, 2>& middle) -> std::optional<ChainEnd> {
    std::optional<ChainEnd> best;
    double distance = std::numeric_limits<double>::infinity();
    for (std::uint32_t other = 0; other < chip.centres.size(); ++other) {
      if (other == chain) {
        continue;
      }
      for (const auto side : {ChainSide::Start, ChainSide::End}) {
        const auto& centre = chip.centres[other][side == ChainSide::Start ? 0 : 1];
        const auto d = std::hypot(centre[0] - middle[0], centre[1] - middle[1]);
        if (d < distance) {
          distance = d;
          best = ChainEnd{.chain = other, .side = side};
        }
      }
    }
    return best;
  };
  std::vector<ConflictPair> pairs;
  std::set<std::pair<ChainEnd, ChainEnd>> seen;
  for (const auto& gate : gates) {
    std::vector<ChainEnd> named;
    for (const auto& name : gate.ends) {
      if (const auto end = endOfWall(name)) {
        named.push_back(*end);
      }
    }
    if (named.empty()) {
      continue;
    }
    auto one = named.front();
    std::optional<ChainEnd> two;
    for (const auto& end : named) {
      if (end.chain != one.chain) {
        two = end;
        break;
      }
    }
    if (!two.has_value()) {
      two = partnerOf(one.chain, gate.middle);
    }
    if (!two.has_value()) {
      continue;
    }
    if (two->chain < one.chain) {
      std::swap(one, *two);
    }
    if (seen.insert({one, *two}).second) {
      pairs.push_back({.first = one, .second = *two, .gate = gate.name});
    }
  }
  return pairs;
}

std::vector<Combination> pairRoundPicks(const Combination& base, const std::uint32_t a,
                                        const std::vector<std::size_t>& statesA,
                                        const std::uint32_t b,
                                        const std::vector<std::size_t>& statesB,
                                        const std::set<Combination>& tried) {
  std::vector<Combination> picks;
  const auto add = [&](Combination pick) {
    if (!tried.contains(pick) &&
        std::ranges::find(picks, pick) == picks.end()) {
      picks.push_back(std::move(pick));
    }
  };
  for (const auto state : statesA) {
    auto pick = base;
    pick[a] = state;
    add(std::move(pick));
  }
  for (const auto state : statesB) {
    auto pick = base;
    pick[b] = state;
    add(std::move(pick));
  }
  std::vector<std::pair<std::size_t, std::size_t>> ranks;
  for (std::size_t i = 0; i < statesA.size(); ++i) {
    for (std::size_t j = 0; j < statesB.size(); ++j) {
      ranks.emplace_back(i, j);
    }
  }
  std::ranges::stable_sort(ranks, [](const auto& x, const auto& y) {
    return x.first + x.second != y.first + y.second ? x.first + x.second < y.first + y.second
                                                    : x.first < y.first;
  });
  for (const auto& [i, j] : ranks) {
    auto pick = base;
    pick[a] = statesA[i];
    pick[b] = statesB[j];
    add(std::move(pick));
  }
  return picks;
}

} // namespace mqt::scpd::pipeline
