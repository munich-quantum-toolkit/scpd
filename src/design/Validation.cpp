/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/design/Validation.hpp"

#include "mqt-scpd/design/Bridges.hpp"
#include "mqt-scpd/design/Roles.hpp"
#include "mqt-scpd/flatbuffers/config.hpp"
#include "mqt-scpd/flatbuffers/design.hpp"

#include <cstddef>
#include <regex>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace mqt::scpd::design {

namespace {

using flatbuffers::config::ConfigT;
using flatbuffers::config::PortConfigT;
using flatbuffers::config::PortPatternsT;
using flatbuffers::design::AssignedRole;
using flatbuffers::design::ChipT;
using flatbuffers::design::Rotation;
using flatbuffers::design::UnassignedRole;

void requirePositive(const double value, const std::string& what,
                     Problems& problems) {
  if (!(value > 0.0)) {
    problems.push_back(what + " must be positive");
  }
}

void requireRotation(const Rotation rotation, Problems& problems) {
  if (rotation == Rotation::Unset) {
    problems.emplace_back("rotation is unset");
  }
}

void append(Problems& into, const Problems& from, const std::string& prefix) {
  for (const auto& problem : from) {
    into.push_back(prefix + problem);
  }
}

/// A pattern that names one thing has to capture it, or it says which labels
/// have the thing without saying which one.
void requireOneCapture(const std::string& expression, const std::string& key,
                       Problems& problems) {
  try {
    const std::regex compiled(expression, std::regex::ECMAScript);
    if (compiled.mark_count() != 1) {
      problems.push_back(key +
                         " pattern must have exactly one capture group, not " +
                         std::to_string(compiled.mark_count()));
    }
  } catch (const std::regex_error&) {
    // The compile failure is reported by requirePattern.
  }
}

void requirePattern(const std::string& expression, const std::string& key,
                    Problems& problems) {
  if (expression.empty()) {
    problems.push_back(key + " pattern is empty");
    return;
  }
  try {
    static_cast<void>(std::regex(expression, std::regex::ECMAScript));
  } catch (const std::regex_error& error) {
    problems.push_back(key + " pattern does not compile: " + error.what());
  }
}

/// Problems of one configured sequence against the chip: every label must
/// be a routable port, and none may appear twice.
void checkSequence(
    const std::vector<std::string>& sequence, const std::string& name,
    const std::unordered_map<std::string_view, UnassignedRole>& roleOf,
    Problems& problems) {
  std::unordered_map<std::string_view, std::size_t> firstAt;
  for (std::size_t i = 0; i < sequence.size(); ++i) {
    const std::string prefix =
        name + "[" + std::to_string(i) + "]: '" + sequence[i] + "' ";
    const auto role = roleOf.find(sequence[i]);
    if (role == roleOf.end()) {
      problems.push_back(prefix + "is not a port of the chip");
    } else if (!isRoutable(role->second)) {
      problems.push_back(prefix + "is a " +
                         std::string(roleName(role->second)) +
                         " port, not a routable one");
    }
    const auto [first, inserted] = firstAt.try_emplace(sequence[i], i);
    if (!inserted) {
      problems.push_back(prefix + "appears twice; first at " +
                         std::to_string(first->second));
    }
  }
}

} // namespace

Problems validate(const flatbuffers::design::PortT& port) {
  Problems problems;
  if (port.label.empty()) {
    problems.emplace_back("label is empty");
  }
  if (port.role == UnassignedRole::Unset) {
    problems.emplace_back("role is unset");
  }
  return problems;
}

Problems validate(const flatbuffers::design::ChipT& chip) {
  Problems problems;
  for (std::size_t i = 0; i < chip.obstacles.size(); ++i) {
    const auto* const obstacle = chip.obstacles[i].get();
    if (obstacle == nullptr || obstacle->vertices.size() < 3) {
      problems.push_back("obstacle " + std::to_string(i) +
                         " has fewer than three vertices");
    }
  }
  for (std::size_t i = 0; i < chip.ports.size(); ++i) {
    const std::string prefix = "port " + std::to_string(i) + ": ";
    const auto* const port = chip.ports[i].get();
    if (port == nullptr) {
      problems.push_back(prefix + "missing");
      continue;
    }
    append(problems, validate(*port), prefix);
  }
  return problems;
}

Problems validate(const flatbuffers::design::ConnectionT& connection) {
  Problems problems;
  if (connection.source_role == AssignedRole::Unset) {
    problems.emplace_back("source role is unset");
  }
  if (connection.target_role == AssignedRole::Unset) {
    problems.emplace_back("target role is unset");
  }
  return problems;
}

Problems validate(const flatbuffers::design::DesignRulesT& rules) {
  Problems problems;
  requirePositive(rules.min_wire_spacing, "min_wire_spacing", problems);
  requirePositive(rules.min_obstacle_spacing, "min_obstacle_spacing", problems);
  requirePositive(rules.min_bend_radius, "min_bend_radius", problems);
  requirePositive(rules.min_straight_length, "min_straight_length", problems);
  requirePositive(rules.target_resonator_length, "target_resonator_length",
                  problems);
  requirePositive(rules.resonator_length_tolerance,
                  "resonator_length_tolerance", problems);
  if (rules.max_feedline_utilization == 0) {
    problems.emplace_back("max_feedline_utilization must be at least one");
  }
  return problems;
}

Problems validate(const flatbuffers::design::CpwCouplerT& coupler) {
  Problems problems;
  requireRotation(coupler.rotation, problems);
  requirePositive(coupler.length, "length", problems);
  requirePositive(coupler.height, "height", problems);
  if (coupler.port == nullptr) {
    problems.emplace_back("port is missing");
    return problems;
  }
  if (coupler.port->role != UnassignedRole::Coupler) {
    problems.emplace_back("port role must be Coupler");
  }
  append(problems, validate(*coupler.port), "port: ");
  return problems;
}

Problems validate(const flatbuffers::design::BridgeT& bridge) {
  Problems problems;
  requireRotation(bridge.rotation, problems);
  requirePositive(bridge.width, "width", problems);
  requirePositive(bridge.height, "height", problems);
  return problems;
}

Problems validate(const PortPatternsT& patterns) {
  Problems problems;
  requirePattern(patterns.launcher, "launcher", problems);
  requirePattern(patterns.resonator, "resonator", problems);
  requirePattern(patterns.conventional, "conventional", problems);
  // The bridge pattern is optional, like the component one: a chip whose
  // components carry no crossing declares none.
  if (!patterns.bridge_pair.empty()) {
    requirePattern(patterns.bridge_pair, "bridge_pair", problems);
  }
  // The component pattern is optional: a chip whose planning stages need no
  // component grouping does not have to declare one. Where it is given it
  // must capture the name, or it would say which labels have a component
  // without saying what it is called.
  if (!patterns.component.empty()) {
    requirePattern(patterns.component, "component", problems);
    requireOneCapture(patterns.component, "component", problems);
  }
  return problems;
}

Problems validate(const flatbuffers::config::BridgeRuleT& rule) {
  Problems problems;
  requirePattern(rule.first, "first", problems);
  requireOneCapture(rule.first, "first", problems);
  requirePattern(rule.second, "second", problems);
  requireOneCapture(rule.second, "second", problems);
  return problems;
}

Problems validate(const PortConfigT& ports) {
  Problems problems;
  if (ports.patterns == nullptr) {
    problems.emplace_back("patterns are missing");
  } else {
    append(problems, validate(*ports.patterns), "patterns: ");
  }
  if (ports.sequences == nullptr) {
    problems.emplace_back("[ports.sequences] is missing");
  }
  for (std::size_t i = 0; i < ports.bridge_pairs.size(); ++i) {
    const auto& rule = ports.bridge_pairs[i];
    const auto where = "bridge_pairs[" + std::to_string(i) + "]: ";
    if (rule == nullptr) {
      problems.push_back(where + "is missing");
      continue;
    }
    append(problems, validate(*rule), where);
  }
  // A rule may only pair ports the bridge_pair pattern selected, so a rule
  // without that pattern can pair nothing and says the opposite.
  if (!ports.bridge_pairs.empty() && ports.patterns != nullptr &&
      ports.patterns->bridge_pair.empty()) {
    problems.emplace_back(
        "bridge_pairs are declared without a bridge_pair pattern");
  }
  return problems;
}

Problems validate(const ConfigT& config) {
  Problems problems;
  if (config.chip_input.empty()) {
    problems.emplace_back("chip_input is empty");
  }
  if (config.ports == nullptr) {
    problems.emplace_back("ports section is missing");
  } else {
    append(problems, validate(*config.ports), "ports: ");
  }
  if (config.rules == nullptr) {
    problems.emplace_back("design rules are missing");
  } else {
    append(problems, validate(*config.rules), "rules: ");
  }
  if (config.grid != nullptr && config.grid->capacity_cells_x == 0) {
    problems.emplace_back("grid: capacity_cells_x must be at least one");
  }
  return problems;
}

Problems validate(const ConfigT& config, const ChipT& chip) {
  Problems problems;
  if (config.ports == nullptr) {
    return problems;
  }
  const auto& ports = *config.ports;

  std::unordered_map<std::string_view, UnassignedRole> roleOf;
  for (const auto& port : chip.ports) {
    if (port != nullptr) {
      roleOf.emplace(port->label, port->role);
    }
  }

  // The two declarations of a bridge have to agree. The pattern says which
  // ports are ends of a crossing and the rules say which two of them pair, so
  // a port either declaration knows and the other does not is a configuration
  // that has drifted apart, and the pairing it produces is not the one it
  // reads as.
  const auto matches = bridgeMatchesOf(chip, ports.bridge_pairs);
  for (std::size_t index = 0; index < chip.ports.size(); ++index) {
    const auto& port = chip.ports[index];
    if (port == nullptr) {
      continue;
    }
    const auto bridging = port->role == UnassignedRole::BridgePair;
    const auto claimed = matches[index].size();
    if (bridging && claimed == 0) {
      problems.push_back("port '" + port->label +
                         "' is a bridge_pair port that no bridge rule pairs");
    } else if (!bridging && claimed > 0) {
      problems.push_back("port '" + port->label +
                         "' is paired by a bridge rule but its role is " +
                         std::string(roleName(port->role)));
    } else if (claimed > 1) {
      problems.push_back("port '" + port->label + "' is paired by " +
                         std::to_string(claimed) + " bridge rule sides");
    }
  }

  if (ports.sequences != nullptr) {
    const auto& sequences = *ports.sequences;
    checkSequence(sequences.all_outer, "all_outer", roleOf, problems);
    checkSequence(sequences.fixed_outer, "fixed_outer", roleOf, problems);
    const std::unordered_set<std::string_view> allOuter(
        sequences.all_outer.begin(), sequences.all_outer.end());
    for (std::size_t i = 0; i < sequences.fixed_outer.size(); ++i) {
      if (!allOuter.contains(sequences.fixed_outer[i])) {
        problems.push_back("fixed_outer[" + std::to_string(i) + "]: '" +
                           sequences.fixed_outer[i] + "' is not in all_outer");
      }
    }
  }
  return problems;
}

} // namespace mqt::scpd::design
