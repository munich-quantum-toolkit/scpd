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

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <map>
#include <regex>
#include <string>
#include <vector>

namespace mqt::scpd::design {

namespace {

/// The two sides of one rule, compiled once for the whole chip.
struct CompiledRule {
  std::regex first;
  std::regex second;
};

std::vector<CompiledRule> compile(const BridgeRules& rules) {
  std::vector<CompiledRule> compiled;
  compiled.reserve(rules.size());
  for (const auto& rule : rules) {
    if (rule == nullptr) {
      continue;
    }
    try {
      compiled.push_back(
          {.first = std::regex(rule->first, std::regex::ECMAScript),
           .second = std::regex(rule->second, std::regex::ECMAScript)});
    } catch (const std::regex_error&) {
      // A rule that does not compile pairs nothing. Which rule it was, and
      // why, is what design::validate reports.
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
      if (std::smatch capture;
          std::regex_match(label, capture, compiled[rule].first)) {
        matches[index].push_back(
            {.rule = rule, .first = true, .key = capture[1].str()});
      }
      if (std::smatch capture;
          std::regex_match(label, capture, compiled[rule].second)) {
        matches[index].push_back(
            {.rule = rule, .first = false, .key = capture[1].str()});
      }
    }
  }
  return matches;
}

std::vector<ComponentPorts> componentsOf(const flatbuffers::design::ChipT& chip,
                                         const BridgeRules& rules) {
  // A map ordered by name, so the components come out the same on every run
  // whatever order the ports were read in.
  std::map<std::string, ComponentPorts> byName;
  for (std::uint32_t index = 0; index < chip.ports.size(); ++index) {
    const auto& port = *chip.ports[index];
    if (port.component.empty() || !isRoutable(port.role)) {
      continue;
    }
    auto& component = byName[port.component];
    component.name = port.component;
    component.ports.push_back(index);
    if (port.role == flatbuffers::design::UnassignedRole::Resonator) {
      component.qubit = true;
    }
  }

  const auto matches = bridgeMatchesOf(chip, rules);

  std::vector<ComponentPorts> components;
  components.reserve(byName.size());
  for (auto& [name, component] : byName) {
    std::ranges::sort(component.ports);
    std::vector<bool> paired(component.ports.size(), false);

    // The rules are applied in the order they are declared, and each rule
    // over the component's ports in ascending order, so the pairing depends
    // on the configuration alone.
    for (std::size_t rule = 0; rule < rules.size(); ++rule) {
      for (std::size_t left = 0; left < component.ports.size(); ++left) {
        if (paired[left]) {
          continue;
        }
        const auto side = std::ranges::find_if(
            matches[component.ports[left]], [&](const BridgeMatch& match) {
              return match.rule == rule && match.first;
            });
        if (side == matches[component.ports[left]].end()) {
          continue;
        }
        for (std::size_t right = 0; right < component.ports.size(); ++right) {
          if (paired[right] || right == left) {
            continue;
          }
          const auto other = std::ranges::find_if(
              matches[component.ports[right]], [&](const BridgeMatch& match) {
                return match.rule == rule && !match.first &&
                       match.key == side->key;
              });
          if (other == matches[component.ports[right]].end()) {
            continue;
          }
          paired[left] = true;
          paired[right] = true;
          component.bridges.push_back({.first = component.ports[left],
                                       .second = component.ports[right]});
          break;
        }
      }
    }

    for (std::size_t position = 0; position < component.ports.size();
         ++position) {
      if (!paired[position]) {
        component.ends.push_back(component.ports[position]);
      }
    }
    components.push_back(std::move(component));
  }
  return components;
}

std::vector<bool> bridgingPorts(const flatbuffers::design::ChipT& chip,
                                const BridgeRules& rules) {
  std::vector<bool> bridging(chip.ports.size(), false);
  for (const auto& component : componentsOf(chip, rules)) {
    for (const auto& bridge : component.bridges) {
      bridging[bridge.first] = true;
      bridging[bridge.second] = true;
    }
  }
  return bridging;
}

} // namespace mqt::scpd::design
