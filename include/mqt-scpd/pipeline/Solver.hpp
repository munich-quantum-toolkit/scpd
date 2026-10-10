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

#include "mqt-scpd/milp/Model.hpp"
#include "mqt-scpd/pipeline/Report.hpp"
#include "mqt-scpd/pipeline/Stages.hpp"
#include "mqt-scpd/pipeline/mqt_scpd_pipeline_export.hpp"

#include <string>
#include <string_view>

namespace mqt::scpd::pipeline {

/**
 * @brief Solves the model of a stage with the backend the run asks for.
 *
 * The backend comes from `[stages.solver] backend`. The environment variable
 * `SCPD_SOLVER` overrides it for one run, so a run can be repeated with the
 * other backend without a change to the configuration it is judged on. The
 * time limit and the relative gap come from the same section.
 *
 * At the Steps level the report receives the size of the model before the
 * solve and how the solve ended after it. While the backend searches, its
 * progress reaches the report under @p task, and its own log the Solver
 * level.
 *
 * @param model The model.
 * @param config The run configuration.
 * @param report Where the solve reports.
 * @param task What the solve is for, such as "solving the inner circuit".
 * @return The solution.
 * @throws std::invalid_argument When the backend name is none of "auto",
 * "highs" and "gurobi".
 * @throws std::runtime_error When "gurobi" is asked for and no external
 * backend is registered.
 */
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT milp::Solution
solveWith(const milp::Model& model, const ConfigT& config, const Report& report,
          std::string_view task);

/**
 * @brief Names the backend that a solve with a configuration uses.
 *
 * The backend is chosen as solveWith() chooses it, from `[stages.solver]
 * backend` and the environment variable `SCPD_SOLVER`.
 *
 * @param config The run configuration.
 * @return "HiGHS" and its version, or the name of the external backend.
 * @throws std::invalid_argument When the backend name is none of "auto",
 * "highs" and "gurobi".
 */
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT std::string
solverName(const ConfigT& config);

} // namespace mqt::scpd::pipeline
