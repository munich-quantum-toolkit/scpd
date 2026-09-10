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

/// The implementations of one stage interface, by name.
///
/// Swapping an algorithm is a stated requirement of a research tool, and this
/// is the whole mechanism: a name, a factory, and a list. There is no plugin
/// loading and no configuration language. The first release registers one
/// implementation per stage, which is expected rather than a smell — the
/// second one becomes a new file rather than a refactor.
template <typename Interface> class Registry {
public:
  using Factory = std::function<std::unique_ptr<Interface>()>;

  /// Register a factory under a name.
  ///
  /// @throws std::invalid_argument when the name is empty or taken.
  void add(std::string name, Factory factory) {
    if (name.empty()) {
      throw std::invalid_argument("a registered implementation needs a name");
    }
    if (factories_.contains(name)) {
      throw std::invalid_argument(
          std::format("'{}' is already registered", name));
    }
    factories_.emplace(std::move(name), std::move(factory));
  }

  /// Build the implementation a name stands for.
  ///
  /// @throws std::invalid_argument when the name is not registered. The
  /// message lists what is.
  [[nodiscard]] std::unique_ptr<Interface>
  make(const std::string_view name) const {
    const auto found = factories_.find(std::string(name));
    if (found == factories_.end()) {
      std::string known;
      for (const auto& [registered, _] : factories_) {
        known += known.empty() ? "" : ", ";
        known += registered;
      }
      throw std::invalid_argument(std::format(
          "'{}' is no known implementation; the registered ones are {}", name,
          known.empty() ? "none" : known));
    }
    return found->second();
  }

  /// The registered names, in order.
  [[nodiscard]] std::vector<std::string> names() const {
    std::vector<std::string> registered;
    registered.reserve(factories_.size());
    for (const auto& [name, _] : factories_) {
      registered.push_back(name);
    }
    return registered;
  }

  [[nodiscard]] bool contains(const std::string_view name) const {
    return factories_.contains(std::string(name));
  }

private:
  // Ordered, so `names()` is the same on every run and every platform.
  std::map<std::string, Factory> factories_;
};

/// The implementations this build ships, one registry per stage.
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT const Registry<ICapacityPlanner>&
capacityPlanners();
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT const Registry<IGlobalRouter>&
globalRouters();
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT const Registry<IAssigner>& assigners();

/// The corridor routers this build ships.
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT const Registry<ICorridorRouter>&
corridorRouters();

/// The detail routers this build ships.
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT const Registry<IDetailRouter>&
detailRouters();

/// The name a configuration selects for a stage, or the stage's default when
/// the configuration names none.
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT std::string_view
selectedCapacityPlanner(const ConfigT& config);
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT std::string_view
selectedGlobalRouter(const ConfigT& config);
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT std::string_view
selectedAssigner(const ConfigT& config);

/// The corridor router a configuration selects, or the default.
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT std::string_view
selectedCorridorRouter(const ConfigT& config);

/// The detail router a configuration selects, or the default.
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT std::string_view
selectedDetailRouter(const ConfigT& config);

} // namespace mqt::scpd::pipeline
