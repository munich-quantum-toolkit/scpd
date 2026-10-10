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

#include "mqt-scpd/pipeline/Stages.hpp"
#include "mqt-scpd/pipeline/mqt_scpd_pipeline_export.hpp"

#include <format>
#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace mqt::scpd::pipeline {

/**
 * @brief The implementations of one stage interface, by name.
 *
 * Swapping an algorithm is a stated requirement of a research tool, and this
 * is the whole mechanism: a name, a factory and a list. The first release
 * registers one implementation per stage; a second one becomes a new file
 * rather than a refactor.
 *
 * @tparam Interface The stage interface.
 */
template <typename Interface> class Registry {
public:
  /// What builds an implementation.
  using Factory = std::function<std::unique_ptr<Interface>()>;

  /**
   * @brief Registers a factory under a name.
   * @param name The name.
   * @param factory The factory.
   * @throws std::invalid_argument When the name is empty or taken.
   */
  void add(std::string name, Factory factory) {
    if (name.empty()) {
      throw std::invalid_argument("a registered implementation needs a name");
    }
    if (factoryByName.contains(name)) {
      throw std::invalid_argument(
          std::format("'{}' is already registered", name));
    }
    factoryByName.emplace(std::move(name), std::move(factory));
  }

  /**
   * @brief Builds the implementation a name stands for.
   * @param name The name.
   * @return The implementation.
   * @throws std::invalid_argument When the name is not registered. The message
   * lists the names that are.
   */
  [[nodiscard]] std::unique_ptr<Interface>
  make(const std::string_view name) const {
    const auto found = factoryByName.find(std::string(name));
    if (found == factoryByName.end()) {
      std::string known;
      for (const auto& [registered, factory] : factoryByName) {
        known += known.empty() ? "" : ", ";
        known += registered;
      }
      throw std::invalid_argument(std::format(
          "'{}' is no known implementation; the registered ones are {}", name,
          known.empty() ? "none" : known));
    }
    return found->second();
  }

  /**
   * @brief Lists the registered names.
   * @return The names, sorted, so that a listing is the same on every run.
   */
  [[nodiscard]] std::vector<std::string> names() const {
    std::vector<std::string> registered;
    registered.reserve(factoryByName.size());
    for (const auto& [name, factory] : factoryByName) {
      registered.push_back(name);
    }
    return registered;
  }

  /**
   * @brief Checks whether a name is registered.
   * @param name The name.
   * @return @c true when a factory is registered under it.
   */
  [[nodiscard]] bool contains(const std::string_view name) const {
    return factoryByName.contains(std::string(name));
  }

private:
  std::map<std::string, Factory> factoryByName;
};

/**
 * @brief Returns the capacity planners this build ships.
 * @return The registry.
 */
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT const Registry<ICapacityPlanner>&
capacityPlanners();

/**
 * @brief Returns the capacity planner a configuration selects.
 * @param config The run configuration.
 * @return The configured name, or the default when the configuration names
 * none.
 */
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT std::string_view
selectedCapacityPlanner(const ConfigT& config);

/**
 * @brief Returns the global routers this build ships.
 * @return The registry.
 */
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT const Registry<IGlobalRouter>&
globalRouters();

/**
 * @brief Returns the global router a configuration selects.
 * @param config The run configuration.
 * @return The configured name, or the default when the configuration names
 * none.
 */
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT std::string_view
selectedGlobalRouter(const ConfigT& config);

/**
 * @brief Returns the assigners this build ships.
 * @return The registry.
 */
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT const Registry<IAssigner>& assigners();

/**
 * @brief Returns the assigner a configuration selects.
 * @param config The run configuration.
 * @return The configured name, or the default when the configuration names
 * none.
 */
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT std::string_view
selectedAssigner(const ConfigT& config);

/**
 * @brief Returns the corridor routers this build ships.
 * @return The registry.
 */
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT const Registry<ICorridorRouter>&
corridorRouters();

/**
 * @brief Returns the corridor router a configuration selects.
 * @param config The run configuration.
 * @return The configured name, or the default when the configuration names
 * none.
 */
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT std::string_view
selectedCorridorRouter(const ConfigT& config);

} // namespace mqt::scpd::pipeline
