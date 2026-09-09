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
#include "mqt-scpd/pipeline/Stages.hpp"
#include "mqt-scpd/pipeline/mqt_scpd_pipeline_export.hpp"

namespace mqt::scpd::pipeline {

/// Solve a model with the backend the run asks for.
///
/// The backend comes from `[stages.solver] backend` in the configuration,
/// which the environment variable `SCPD_SOLVER` overrides for one run. It is
/// the only environment variable the tool reads: everything else that used
/// to be one in the prototype is a configuration key or is gone.
///
/// @throws std::invalid_argument when the name is none of "auto", "highs"
/// and "gurobi".
/// @throws std::runtime_error when "gurobi" is asked for and no external
/// backend is registered.
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT milp::Solution solveWith(const milp::Model& model,
                                                                const ConfigT& config);

} // namespace mqt::scpd::pipeline
