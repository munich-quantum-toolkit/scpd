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

#include "mqt-scpd/design/mqt_scpd_design_export.hpp"
#include "mqt-scpd/flatbuffers/design.hpp"

#include <string>
#include <vector>

namespace mqt::scpd::design {

/// The semantic problems of a value, empty when the value is valid.
///
/// The FlatBuffers verifier checks that a buffer is well formed and that its
/// required fields are present. It cannot see an enum left at Unset or a
/// dimension left at zero, because a scalar field always reads as a value.
/// The validate functions check what the verifier cannot. Every function
/// returns every problem it finds, so a caller can report them all at once.
using Problems = std::vector<std::string>;

/// Problems of a port: an empty label or an unset role.
[[nodiscard]] MQT_SCPD_DESIGN_EXPORT Problems
validate(const flatbuffers::design::PortT& port);

/// Problems of a chip: an obstacle with fewer than three vertices, and the
/// problems of its ports, each prefixed by the port's index.
[[nodiscard]] MQT_SCPD_DESIGN_EXPORT Problems
validate(const flatbuffers::design::ChipT& chip);

/// Problems of a connection: an unset role at either end.
[[nodiscard]] MQT_SCPD_DESIGN_EXPORT Problems
validate(const flatbuffers::design::ConnectionT& connection);

/// Problems of the design rules: a length that is not positive, or a feedline
/// utilization of zero.
[[nodiscard]] MQT_SCPD_DESIGN_EXPORT Problems
validate(const flatbuffers::design::DesignRulesT& rules);

/// Problems of a coupler: an unset rotation, a dimension that is not positive,
/// a missing port, a port whose role is not Coupler, and the port's own
/// problems.
[[nodiscard]] MQT_SCPD_DESIGN_EXPORT Problems
validate(const flatbuffers::design::CpwCouplerT& coupler);

/// Problems of a bridge: an unset rotation or a dimension that is not positive.
[[nodiscard]] MQT_SCPD_DESIGN_EXPORT Problems
validate(const flatbuffers::design::BridgeT& bridge);

} // namespace mqt::scpd::design
