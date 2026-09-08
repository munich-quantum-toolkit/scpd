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

/// Problems of the reports in drc.json: a report without a stage, a finding
/// without a rule, a severity or a wire, a limit that is not positive, and a
/// clearance kind that is set for a rule other than wire clearance or unset
/// for that rule.
[[nodiscard]] MQT_SCPD_DRC_EXPORT Problems
validate(const flatbuffers::drc::DrcReportsT& reports);

} // namespace mqt::scpd::drc
