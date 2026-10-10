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
#include "mqt-scpd/milp/mqt_scpd_milp_export.hpp"

#include <string>

namespace mqt::scpd::milp {

/**
 * @brief Writes a model as free-form MPS text.
 *
 * Every mixed-integer solver reads MPS, and MPS carries what the model holds:
 * bounds, a variable type, linear rows and one linear objective. The text
 * names rows and columns as the model does, so a solution comes back by name.
 *
 * The objective is always written as a minimization, with the coefficients
 * negated for a maximization, because MPS has no sense field that every
 * reader honors. The objective offset is written as the right-hand side of
 * the objective row, negated, which is how solvers read a constant.
 *
 * @param model The model.
 * @return The MPS text.
 */
[[nodiscard]] MQT_SCPD_MILP_EXPORT std::string toMps(const Model& model);

} // namespace mqt::scpd::milp
