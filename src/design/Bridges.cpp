/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/design/Bridges.hpp"

#include "mqt-scpd/design/Roles.hpp"
#include "mqt-scpd/flatbuffers/design.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <regex>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

namespace mqt::scpd::design {

namespace {

/// The two sides of one rule, compiled once for the whole chip.
struct CompiledRule {
  std::regex first;
  std::regex second;
};

/// One entry per configured rule, so that a match names the rule by its
/// position in the configuration. A rule that is missing or does not compile
/// has no entry and matches nothing; design::validate reports it.
std::vector<std::optional<CompiledRule>> compile(const BridgeRules& rules) {
  std::vector<std::optional<CompiledRule>> compiled;
  compiled.reserve(rules.size());
  for (const auto& rule : rules) {
    if (rule == nullptr) {
      compiled.emplace_back();
      continue;
    }
    try {
      compiled.emplace_back(CompiledRule{
          .first = std::regex(rule->first, std::regex::ECMAScript),
          .second = std::regex(rule->second, std::regex::ECMAScript)});
    } catch (const std::regex_error&) {
      compiled.emplace_back();
    }
  }
  return compiled;
}

} // namespace

std::vector<std::vector<BridgeMatch>>
bridgeMatchesOf(const flatbuffers::design::ChipT& chip,
                const BridgeRules& rules) {
  const auto compiled = compile(rules);
  std::vector<std::vector<BridgeMatch>> matches(chip.ports.size());
  for (std::size_t index = 0; index < chip.ports.size(); ++index) {
    if (chip.ports[index] == nullptr) {
      continue;
    }
    const auto& label = chip.ports[index]->label;
    for (std::size_t rule = 0; rule < compiled.size(); ++rule) {
      if (!compiled[rule].has_value()) {
        continue;
      }
      if (std::smatch capture;
          std::regex_match(label, capture, compiled[rule]->first)) {
        matches[index].push_back(
            {.rule = rule, .first = true, .key = capture[1].str()});
      }
      if (std::smatch capture;
          std::regex_match(label, capture, compiled[rule]->second)) {
        matches[index].push_back(
            {.rule = rule, .first = false, .key = capture[1].str()});
      }
    }
  }
  return matches;
}

std::vector<BridgePair>
bridgePairsOf(const flatbuffers::design::ChipT& chip,
              const std::vector<std::vector<BridgeMatch>>& matches) {
  struct Sides {
    std::optional<std::uint32_t> first;
    std::optional<std::uint32_t> second;
    bool ambiguous = false;
  };
  using Key = std::tuple<std::size_t, std::string_view, std::string_view>;
  std::map<Key, Sides> groups;
  for (std::uint32_t index = 0; index < chip.ports.size(); ++index) {
    const auto& port = chip.ports[index];
    if (port == nullptr || port->component.empty() || !isRoutable(port->role) ||
        matches.at(index).size() != 1) {
      continue;
    }
    const auto& match = matches[index].front();
    auto& group = groups[{match.rule, port->component, match.key}];
    auto& side = match.first ? group.first : group.second;
    group.ambiguous |= side.has_value();
    side = index;
  }

  // Preserve the order of the rules, then the order of first-side port indices.
  std::map<std::pair<std::size_t, std::uint32_t>, BridgePair> ordered;
  for (const auto& [key, group] : groups) {
    if (!group.ambiguous && group.first && group.second) {
      ordered.emplace(
          std::pair{std::get<0>(key), *group.first},
          BridgePair{.first = *group.first, .second = *group.second});
    }
  }

  std::vector<BridgePair> pairs;
  pairs.reserve(ordered.size());
  for (const auto& [key, pair] : ordered) {
    pairs.push_back(pair);
  }
  return pairs;
}

std::vector<ComponentPorts> componentsOf(const flatbuffers::design::ChipT& chip,
                                         const BridgeRules& rules) {
  // A map ordered by name gives a stable order independent of the input.
  std::map<std::string, ComponentPorts> byName;
  for (std::uint32_t index = 0; index < chip.ports.size(); ++index) {
    const auto& port = chip.ports[index];
    if (port == nullptr || port->component.empty() || !isRoutable(port->role)) {
      continue;
    }
    auto& component = byName[port->component];
    component.name = port->component;
    component.ports.push_back(index);
  }

  std::vector<bool> paired(chip.ports.size(), false);
  for (const auto& pair : bridgePairsOf(chip, bridgeMatchesOf(chip, rules))) {
    byName.at(chip.ports[pair.first]->component).bridges.push_back(pair);
    paired[pair.first] = true;
    paired[pair.second] = true;
  }

  std::vector<ComponentPorts> components;
  components.reserve(byName.size());
  for (auto& [name, component] : byName) {
    for (const auto port : component.ports) {
      if (!paired[port]) {
        component.ends.push_back(port);
      }
    }
    components.push_back(std::move(component));
  }
  return components;
}

std::vector<bool> bridgingPorts(const flatbuffers::design::ChipT& chip,
                                const BridgeRules& rules) {
  std::vector<bool> bridging(chip.ports.size(), false);
  for (const auto& pair : bridgePairsOf(chip, bridgeMatchesOf(chip, rules))) {
    bridging[pair.first] = true;
    bridging[pair.second] = true;
  }
  return bridging;
}

} // namespace mqt::scpd::design
