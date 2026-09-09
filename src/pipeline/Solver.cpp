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
#include "mqt-scpd/pipeline/Stages.hpp"

#include <cstdlib>
#include <string>

namespace mqt::scpd::pipeline {

milp::Solution solveWith(const milp::Model& model, const ConfigT& config) {
  std::string backend;
  milp::SolveOptions options;
  if (config.stages != nullptr && config.stages->solver != nullptr) {
    const auto& solver = *config.stages->solver;
    backend = solver.backend;
    options.timeLimit = solver.time_limit;
    options.relativeGap = solver.relative_gap;
  }
  // The environment wins over the configuration, so a run can be repeated
  // against the other backend without editing the file it is judged on.
  if (const auto* fromEnvironment = std::getenv("SCPD_SOLVER");
      fromEnvironment != nullptr && *fromEnvironment != '\0') {
    backend = fromEnvironment;
  }
  if (backend.empty()) {
    backend = "auto";
  }
  return milp::solve(model, milp::backendChoiceFromName(backend), options);
}

} // namespace mqt::scpd::pipeline
