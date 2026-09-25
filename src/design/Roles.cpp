/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/design/Roles.hpp"

#include "mqt-scpd/design/Validation.hpp"
#include "mqt-scpd/flatbuffers/config.hpp"
#include "mqt-scpd/flatbuffers/design.hpp"

#include <regex>
#include <string>
#include <string_view>
#include <vector>

namespace mqt::scpd::design {

namespace {

using flatbuffers::config::PortPatternsT;
using flatbuffers::design::ChipT;
using flatbuffers::design::UnassignedRole;

struct CompiledPattern {
  std::string_view key;
  UnassignedRole role;
  std::regex expression;
};

/// The patterns in the order of the configuration keys.
std::vector<CompiledPattern> compile(const PortPatternsT& patterns) {
  std::vector<CompiledPattern> compiled;
  const auto add = [&](const std::string_view key, const UnassignedRole role,
                       const std::string& expression) {
    compiled.push_back(
        {.key = key,
         .role = role,
         .expression = std::regex(expression, std::regex::ECMAScript)});
  };
  add("launcher", UnassignedRole::Launcher, patterns.launcher);
  add("resonator", UnassignedRole::Resonator, patterns.resonator);
  add("conventional", UnassignedRole::Conventional, patterns.conventional);
  return compiled;
}

} // namespace

std::string_view roleName(const UnassignedRole role) {
  switch (role) {
  case UnassignedRole::Launcher:
    return "launcher";
  case UnassignedRole::Resonator:
    return "resonator";
  case UnassignedRole::Conventional:
    return "conventional";
  case UnassignedRole::Coupler:
    return "coupler";
  case UnassignedRole::Unset:
    break;
  }
  return "unset";
}

bool isRoutable(const UnassignedRole role) {
  return role == UnassignedRole::Resonator ||
         role == UnassignedRole::Conventional;
}

Problems classifyPorts(ChipT& chip, const PortPatternsT& patterns) {
  Problems problems = validate(patterns);
  if (!problems.empty()) {
    return problems;
  }
  const auto compiled = compile(patterns);
  for (auto& port : chip.ports) {
    if (port == nullptr) {
      continue;
    }
    std::vector<std::string_view> matched;
    UnassignedRole role = UnassignedRole::Unset;
    for (const auto& pattern : compiled) {
      if (std::regex_match(port->label, pattern.expression)) {
        matched.push_back(pattern.key);
        role = pattern.role;
      }
    }
    if (matched.size() == 1) {
      port->role = role;
      continue;
    }
    port->role = UnassignedRole::Unset;
    if (matched.empty()) {
      problems.push_back("port '" + port->label + "' matches no role pattern");
      continue;
    }
    std::string names;
    for (const auto& key : matched) {
      names += names.empty() ? "" : ", ";
      names += key;
    }
    problems.push_back("port '" + port->label +
                       "' matches more than one role pattern: " + names);
  }
  return problems;
}

} // namespace mqt::scpd::design
