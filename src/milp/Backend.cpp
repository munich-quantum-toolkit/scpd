/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/milp/Backend.hpp"

#include "mqt-scpd/milp/Model.hpp"
#include "mqt-scpd/milp/Mps.hpp"

#include <format>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace mqt::scpd::milp {
namespace {

/// The external backend of the process, and the lock that guards it. It is
/// process state because it is installed once, by the Python layer at import
/// time, and read by every stage afterwards.
std::mutex& externalMutex() {
  static std::mutex mutex;
  return mutex;
}

std::shared_ptr<ISolverBackend>& externalSlot() {
  static std::shared_ptr<ISolverBackend> backend;
  return backend;
}

/// A backend that hands the model out as MPS text and reads the answer back.
class MpsBackend final : public ISolverBackend {
public:
  MpsBackend(std::string name, MpsSolveFunction solve)
      : name_(std::move(name)), solve_(std::move(solve)) {}

  [[nodiscard]] std::string_view name() const override { return name_; }

  [[nodiscard]] Solution solve(const Model& model, const SolveOptions& options) const override {
    std::vector<std::string> names;
    names.reserve(model.variableCount());
    for (const auto& variable : model.variables()) {
      names.push_back(variable.name);
    }

    auto solution = solve_(toMps(model), names, options);
    if (solution.backend.empty()) {
      solution.backend = name_;
    }
    if (solution.hasValues() && solution.values.size() != model.variableCount()) {
      return {.status = SolveStatus::Error,
              .backend = name_,
              .message = std::format("the backend returned {} values for {} variables",
                                     solution.values.size(), model.variableCount())};
    }
    return solution;
  }

private:
  std::string name_;
  MpsSolveFunction solve_;
};

} // namespace

std::unique_ptr<ISolverBackend> makeMpsBackend(std::string name, MpsSolveFunction solve) {
  if (!solve) {
    throw std::invalid_argument("an MPS backend needs a solve function");
  }
  return std::make_unique<MpsBackend>(std::move(name), std::move(solve));
}

BackendChoice backendChoiceFromName(const std::string_view name) {
  if (name == "auto") {
    return BackendChoice::Auto;
  }
  if (name == "highs") {
    return BackendChoice::Highs;
  }
  // The external backend is named after the solver it reaches, because that
  // is the name a user knows it by.
  if (name == "gurobi") {
    return BackendChoice::External;
  }
  throw std::invalid_argument(
      std::format("'{}' is no solver backend; use 'auto', 'highs' or 'gurobi'", name));
}

void setExternalBackend(std::shared_ptr<ISolverBackend> backend) {
  const std::lock_guard lock(externalMutex());
  externalSlot() = std::move(backend);
}

std::shared_ptr<ISolverBackend> externalBackend() {
  const std::lock_guard lock(externalMutex());
  return externalSlot();
}

std::shared_ptr<ISolverBackend> resolveBackend(const BackendChoice choice) {
  if (choice != BackendChoice::Highs) {
    if (auto external = externalBackend()) {
      return external;
    }
    if (choice == BackendChoice::External) {
      throw std::runtime_error(
          "no external solver is registered; install gurobipy and a licence, or use 'highs'");
    }
  }
  // HiGHS is linked in, so this branch always has an answer. That is what
  // makes an installation without a commercial licence fully functional.
  return makeHighsBackend();
}

Solution solve(const Model& model, const BackendChoice choice, const SolveOptions& options) {
  return resolveBackend(choice)->solve(model, options);
}

} // namespace mqt::scpd::milp
