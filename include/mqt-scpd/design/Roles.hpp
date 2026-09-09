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
#include "mqt-scpd/design/mqt_scpd_design_export.hpp"
#include "mqt-scpd/flatbuffers/config.hpp"
#include "mqt-scpd/flatbuffers/design.hpp"

#include <string_view>

namespace mqt::scpd::design {

/// The name of a role as the configuration keys and the doctor table spell
/// it: "launcher", "resonator", "conventional", "coupler", "mating", or
/// "unset".
[[nodiscard]] MQT_SCPD_DESIGN_EXPORT std::string_view
roleName(flatbuffers::design::UnassignedRole role);

/// Whether a wire may end at a port of this role: Resonator or Conventional.
[[nodiscard]] MQT_SCPD_DESIGN_EXPORT bool
isRoutable(flatbuffers::design::UnassignedRole role);

/// Set the role of every port from the patterns of the configuration.
///
/// A port must match exactly one pattern. A port that matches none, or more
/// than one, is a problem naming the label and the patterns involved, and
/// its role stays Unset. The problems of the patterns themselves are
/// returned first, and nothing is classified while they exist.
///
/// @returns Every problem found, empty when every port has a role.
[[nodiscard]] MQT_SCPD_DESIGN_EXPORT Problems
classifyPorts(flatbuffers::design::ChipT& chip,
              const flatbuffers::config::PortPatternsT& patterns);

} // namespace mqt::scpd::design
