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

#include "mqt-scpd/design/Validation.hpp"
#include "mqt-scpd/drc/mqt_scpd_drc_export.hpp"
#include "mqt-scpd/flatbuffers/drc.hpp"

namespace mqt::scpd::drc {

using design::Problems;

/**
 * @brief Validates the design-rule reports that a run writes to drc.json.
 * @param reports The reports to check.
 * @return A problem for every report without a stage, for every finding
 * without a rule, a severity or a wire, for every limit that is not positive,
 * and for every clearance kind that is set for a rule other than wire
 * clearance or left unset for that rule, empty when the reports are valid.
 */
[[nodiscard]] MQT_SCPD_DRC_EXPORT Problems
validate(const flatbuffers::drc::DrcReportsT& reports);

} // namespace mqt::scpd::drc
