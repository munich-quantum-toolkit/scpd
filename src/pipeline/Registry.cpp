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
#include "mqt-scpd/pipeline/GlobalRouter.hpp"
#include "mqt-scpd/pipeline/Stages.hpp"

#include <string>
#include <string_view>

namespace mqt::scpd::pipeline {
namespace {

/// The name a stage falls back to when the configuration selects none. Each
/// is the one implementation this release ships.
constexpr std::string_view DEFAULT_CAPACITY_PLANNER = "watershed";
constexpr std::string_view DEFAULT_GLOBAL_ROUTER = "hanan-milp";
constexpr std::string_view DEFAULT_ASSIGNER = "ordered-milp";

/// A configured name, or the default when the configuration leaves it empty.
std::string_view orDefault(const std::string& configured, const std::string_view fallback) {
  return configured.empty() ? fallback : std::string_view(configured);
}

} // namespace

const Registry<ICapacityPlanner>& capacityPlanners() {
  static const auto registry = [] {
    Registry<ICapacityPlanner> made;
    made.add(std::string(DEFAULT_CAPACITY_PLANNER), makeWatershedPlanner);
    return made;
  }();
  return registry;
}

const Registry<IGlobalRouter>& globalRouters() {
  static const auto registry = [] {
    Registry<IGlobalRouter> made;
    made.add(std::string(DEFAULT_GLOBAL_ROUTER), makeHananMilpRouter);
    return made;
  }();
  return registry;
}

const Registry<IAssigner>& assigners() {
  static const auto registry = [] {
    Registry<IAssigner> made;
    made.add(std::string(DEFAULT_ASSIGNER), makeOrderedMilpAssigner);
    return made;
  }();
  return registry;
}

std::string_view selectedCapacityPlanner(const ConfigT& config) {
  if (config.stages != nullptr && config.stages->capacity != nullptr) {
    return orDefault(config.stages->capacity->planner, DEFAULT_CAPACITY_PLANNER);
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

} // namespace mqt::scpd::pipeline
