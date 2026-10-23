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

#include "mqt-scpd/pipeline/Assigner.hpp"
#include "mqt-scpd/pipeline/CapacityPlanner.hpp"
#include "mqt-scpd/pipeline/CorridorRouter.hpp"
#include "mqt-scpd/pipeline/DetailRouter.hpp"
#include "mqt-scpd/pipeline/FinalRouter.hpp"
#include "mqt-scpd/pipeline/GlobalRouter.hpp"
#include "mqt-scpd/pipeline/Stages.hpp"

#include <algorithm>
#include <format>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace mqt::scpd::pipeline {
namespace {

/// The name a stage falls back to when the configuration selects none. Each
/// is the one implementation this release ships.
constexpr std::string_view DEFAULT_CAPACITY_PLANNER = "watershed";
constexpr std::string_view DEFAULT_GLOBAL_ROUTER = "hanan-milp";
constexpr std::string_view DEFAULT_ASSIGNER = "ordered-milp";
constexpr std::string_view DEFAULT_CORRIDOR_ROUTER = "partition-astar";
constexpr std::string_view DEFAULT_DETAIL_ROUTER = "pixel-astar";
constexpr std::string_view DEFAULT_FINAL_ROUTER = "dubins";

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

const Registry<IGlobalRouter>& globalRouters() {
  static const auto REGISTRY = [] {
    Registry<IGlobalRouter> made;
    made.add(std::string(DEFAULT_GLOBAL_ROUTER), makeHananMilpRouter);
    return made;
  }();
  return REGISTRY;
}

const Registry<IAssigner>& assigners() {
  static const auto REGISTRY = [] {
    Registry<IAssigner> made;
    made.add(std::string(DEFAULT_ASSIGNER), makeOrderedMilpAssigner);
    return made;
  }();
  return REGISTRY;
}

const Registry<ICorridorRouter>& corridorRouters() {
  static const auto REGISTRY = [] {
    Registry<ICorridorRouter> made;
    made.add(std::string(DEFAULT_CORRIDOR_ROUTER), makePartitionAStarRouter);
    return made;
  }();
  return REGISTRY;
}

const Registry<IDetailRouter>& detailRouters() {
  static const auto REGISTRY = [] {
    Registry<IDetailRouter> made;
    made.add(std::string(DEFAULT_DETAIL_ROUTER), makePixelAStarRouter);
    return made;
  }();
  return REGISTRY;
}

const Registry<IFinalRouter>& finalRouters() {
  static const auto REGISTRY = [] {
    Registry<IFinalRouter> made;
    made.add(std::string(DEFAULT_FINAL_ROUTER), makeDubinsFinalRouter);
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

std::string_view selectedGlobalRouter(const ConfigT& config) {
  if (config.stages != nullptr && config.stages->global != nullptr) {
    return orDefault(config.stages->global->router, DEFAULT_GLOBAL_ROUTER);
  }
  return DEFAULT_GLOBAL_ROUTER;
}

std::string_view selectedAssigner(const ConfigT& config) {
  if (config.stages != nullptr && config.stages->assignment != nullptr) {
    return orDefault(config.stages->assignment->assigner, DEFAULT_ASSIGNER);
  }
  return DEFAULT_ASSIGNER;
}

std::string_view selectedCorridorRouter(const ConfigT& config) {
  if (config.stages != nullptr && config.stages->corridor != nullptr) {
    return orDefault(config.stages->corridor->router, DEFAULT_CORRIDOR_ROUTER);
  }
  return DEFAULT_CORRIDOR_ROUTER;
}

std::string_view selectedDetailRouter(const ConfigT& config) {
  if (config.stages != nullptr && config.stages->detail != nullptr) {
    return orDefault(config.stages->detail->router, DEFAULT_DETAIL_ROUTER);
  }
  return DEFAULT_DETAIL_ROUTER;
}

std::string_view selectedFinalRouter(const ConfigT& config) {
  if (config.stages != nullptr && config.stages->final != nullptr) {
    return orDefault(config.stages->final->router, DEFAULT_FINAL_ROUTER);
  }
  return DEFAULT_FINAL_ROUTER;
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
  }
  // The capacity and corridor steps build the detail grid, and the assignment
  // places the launcher slots on it.
  if (takes("capacity") || takes("assign") || takes("corridor")) {
    note(detailGridProblem(chip, config));
  }
  if (takes("global")) {
    note(unknownName(globalRouters(), selectedGlobalRouter(config),
                     "[stages.global] router"));
    try {
      static_cast<void>(innerCircuitOf(chip, config));
    } catch (const std::invalid_argument& error) {
      problems.emplace_back(error.what());
    }
  }
  if (takes("corridor")) {
    note(unknownName(corridorRouters(), selectedCorridorRouter(config),
                     "[stages.corridor] router"));
  }
  if (takes("detail")) {
    note(unknownName(detailRouters(), selectedDetailRouter(config),
                     "[stages.detail] router"));
  }
  if (takes("final")) {
    note(unknownName(finalRouters(), selectedFinalRouter(config),
                     "[stages.final] router"));
  }
  if (!takes("assign")) {
    return problems;
  }
  note(unknownName(assigners(), selectedAssigner(config),
                   "[stages.assignment] assigner"));
  const auto target =
      config.stages != nullptr && config.stages->assignment != nullptr
          ? config.stages->assignment->launcher_target
          : 0U;
  if (target == 0) {
    problems.emplace_back(
        "[stages.assignment] launcher_target is missing; it is a per-chip "
        "figure with no default");
  } else if (config.rules != nullptr &&
             (target + config.rules->feedline_terminations) % 2 != 0) {
    problems.push_back(std::format(
        "[stages.assignment] launcher_target {} plus [design_rules] "
        "feedline_terminations {} is odd; every feedline chain has two ends, "
        "so the sum has to be even",
        target, config.rules->feedline_terminations));
  }
  note(launcherOrderProblem(chip, config));
  note(launcherPlacesProblem(chip, config));
  note(ringCountProblem(chip, config));
  return problems;
}

namespace {

/// Whether the global step solves a model: it does where the chip has an
/// inner circuit. A ring that the global step refuses counts as one it
/// solves, so that the list keeps the step.
bool globalSolves(const ChipT& chip, const ConfigT& config) {
  try {
    return !innerCircuitOf(chip, config).targets.empty();
  } catch (const std::invalid_argument&) {
    return true;
  }
}

} // namespace

std::vector<std::string> solvingSteps(const ChipT& chip, const ConfigT& config,
                                      const std::vector<std::string>& steps) {
  std::vector<std::string> solving;
  for (const auto& step : steps) {
    if (step == "assign" || (step == "global" && globalSolves(chip, config))) {
      solving.push_back(step);
    }
  }
  return solving;
}

} // namespace mqt::scpd::pipeline
