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

} // namespace mqt::scpd::pipeline
