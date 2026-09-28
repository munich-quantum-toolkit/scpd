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

/**
 * @brief Names a port role as the configuration keys spell it.
 * @param role The role to name.
 * @return One of "launcher", "resonator", "conventional", "coupler", or
 * "unset" for any other value. The result refers to a string literal and
 * stays valid for the life of the program.
 */
[[nodiscard]] MQT_SCPD_DESIGN_EXPORT std::string_view
roleName(flatbuffers::design::UnassignedRole role);

/**
 * @brief Reports whether a wire may end at a port of the given role.
 * @param role The role to test.
 * @return @c true for the roles @c Resonator and @c Conventional, @c false
 * for every other role.
 */
[[nodiscard]] MQT_SCPD_DESIGN_EXPORT bool
isRoutable(flatbuffers::design::UnassignedRole role);

/**
 * @brief Sets the role of every port of a chip from the configured patterns.
 *
 * Each label is matched against one regular expression per role. A port must
 * match exactly one pattern; a port that matches none, or more than one,
 * keeps the role @c Unset and is reported with its label and the patterns
 * involved. The patterns themselves are validated first, and no port is
 * classified while a pattern is empty or does not compile.
 *
 * @param chip The chip whose ports are classified. Its ports are modified in
 * place.
 * @param patterns One regular expression per role, from the configuration.
 * @return Every problem found, empty when every port carries a role.
 * @post Every port of @p chip carries a role when the result is empty.
 */
[[nodiscard]] MQT_SCPD_DESIGN_EXPORT Problems
classifyPorts(flatbuffers::design::ChipT& chip,
              const flatbuffers::config::PortPatternsT& patterns);

} // namespace mqt::scpd::design
