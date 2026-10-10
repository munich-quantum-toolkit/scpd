/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/pipeline/Solver.hpp"

#include "mqt-scpd/milp/Backend.hpp"
#include "mqt-scpd/milp/Model.hpp"
#include "mqt-scpd/pipeline/Report.hpp"
#include "mqt-scpd/pipeline/Stages.hpp"

#include <cmath>
#include <cstdlib>
#include <format>
#include <string>
#include <string_view>

namespace mqt::scpd::pipeline {
namespace {

/// How a solve ended, as one line: the backend, the status, and the figures
/// the backend reported.
std::string outcomeOf(const milp::Solution& solution) {
  auto text =
      std::format("{} {}", solution.backend, milp::statusName(solution.status));
  if (!std::isnan(solution.objective)) {
    text +=
        std::format("{}objective {:.2f}", FIGURE_SEPARATOR, solution.objective);
  }
  if (!std::isnan(solution.bound)) {
    text += std::format("{}bound {:.2f}", FIGURE_SEPARATOR, solution.bound);
  }
  if (!std::isnan(solution.gap)) {
    text +=
        std::format("{}gap {:.1f} %", FIGURE_SEPARATOR, solution.gap * 100.0);
  }
  if (solution.hasValues()) {
    text += std::string(FIGURE_SEPARATOR) + counted(solution.nodes, "node");
  }
  if (!solution.message.empty()) {
    text += std::string(FIGURE_SEPARATOR) + solution.message;
  }
  return text;
}

/// The backend a configuration asks for, and what a solve may spend.
struct Backend {
  milp::BackendChoice choice = milp::BackendChoice::Auto;
  milp::SolveOptions options;
};

/// The backend comes from the configuration, and the environment overrides
/// it, so a run can be repeated with the other backend without a change to
/// the file it is judged on.
Backend backendOf(const ConfigT& config) {
  std::string name;
  Backend backend;
  if (config.stages != nullptr && config.stages->solver != nullptr) {
    const auto& solver = *config.stages->solver;
    name = solver.backend;
    backend.options.timeLimit = solver.time_limit;
    backend.options.relativeGap = solver.relative_gap;
  }
  if (const auto* fromEnvironment = std::getenv("SCPD_SOLVER");
      fromEnvironment != nullptr && *fromEnvironment != '\0') {
    name = fromEnvironment;
  }
  backend.choice = milp::backendChoiceFromName(name.empty() ? "auto" : name);
  return backend;
}

} // namespace

milp::Solution solveWith(const milp::Model& model, const ConfigT& config,
                         const Report& report, const std::string_view task) {
  auto [choice, options] = backendOf(config);
  if (report.wants(Detail::Steps)) {
    report.line(Detail::Steps,
                std::format("model {}{}{}{}{}", model.name(), FIGURE_SEPARATOR,
                            counted(model.variableCount(), "variable"),
                            FIGURE_SEPARATOR,
                            counted(model.constraintCount(), "constraint")));
  }
  options.observer = report.solveObserver(task);
  const auto solution = milp::solve(model, choice, options);
  if (report.wants(Detail::Steps)) {
    report.line(Detail::Steps, outcomeOf(solution));
  }
  return solution;
}

std::string solverName(const ConfigT& config) {
  const auto choice = backendOf(config).choice;
  const auto external = milp::externalBackend();
  if (choice == milp::BackendChoice::Highs ||
      (choice == milp::BackendChoice::Auto && external == nullptr)) {
    return "HiGHS " + milp::highsVersion();
  }
  // Without a registered backend the solve itself fails and says why.
  return external != nullptr ? std::string(external->name()) : "gurobi";
}

} // namespace mqt::scpd::pipeline
