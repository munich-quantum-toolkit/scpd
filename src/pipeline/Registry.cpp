/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/pipeline/Registry.hpp"

#include "mqt-scpd/pipeline/CapacityPlanner.hpp"
#include "mqt-scpd/pipeline/Stages.hpp"

#include <algorithm>
#include <format>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace mqt::scpd::pipeline {
namespace {

/// The name a stage falls back to when the configuration selects none. Each
/// is the one implementation this release ships.
constexpr std::string_view DEFAULT_CAPACITY_PLANNER = "watershed";

/// A configured name, or the default when the configuration leaves it empty.
std::string_view orDefault(const std::string& configured,
                           const std::string_view fallback) {
  return configured.empty() ? fallback : std::string_view(configured);
}

/// The problem of a selected name that a registry does not carry, or nothing.
template <typename Interface>
std::optional<std::string> unknownName(const Registry<Interface>& registry,
                                       const std::string_view name,
                                       const std::string_view key) {
  if (registry.contains(name)) {
    return std::nullopt;
  }
  std::string known;
  for (const auto& registered : registry.names()) {
    known += known.empty() ? "" : ", ";
    known += registered;
  }
  return std::format("{} '{}' is not a known implementation; the registered "
                     "ones are {}",
                     key, name, known.empty() ? "none" : known);
}

} // namespace

const Registry<ICapacityPlanner>& capacityPlanners() {
  static const auto REGISTRY = [] {
    Registry<ICapacityPlanner> made;
    made.add(std::string(DEFAULT_CAPACITY_PLANNER), makeWatershedPlanner);
    return made;
  }();
  return REGISTRY;
}

std::string_view selectedCapacityPlanner(const ConfigT& config) {
  if (config.stages != nullptr && config.stages->capacity != nullptr) {
    return orDefault(config.stages->capacity->planner,
                     DEFAULT_CAPACITY_PLANNER);
  }
  return DEFAULT_CAPACITY_PLANNER;
}

std::vector<std::string> planProblems(const ChipT& chip, const ConfigT& config,
                                      const std::vector<std::string>& steps) {
  const auto takes = [&steps](const std::string_view step) {
    return std::ranges::find(steps, step) != steps.end();
  };
  std::vector<std::string> problems;
  const auto note = [&problems](std::optional<std::string> problem) {
    if (problem.has_value()) {
      problems.push_back(*std::move(problem));
    }
  };
  if (takes("capacity")) {
    note(unknownName(capacityPlanners(), selectedCapacityPlanner(config),
                     "[stages.capacity] planner"));
    // The capacity step builds the detail grid and stamps the port bands.
    note(detailGridProblem(chip, config));
    note(orientationProblem(chip));
  }
  return problems;
}

} // namespace mqt::scpd::pipeline
