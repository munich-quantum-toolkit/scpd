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
#include "mqt-scpd/flatbuffers/config.hpp"
#include "mqt-scpd/flatbuffers/design.hpp"

#include <string>
#include <vector>

namespace mqt::scpd::design {

/**
 * @brief The semantic problems of a value, empty when the value is valid.
 *
 * The FlatBuffers verifier checks that a buffer is well formed and that its
 * required fields are present. It cannot see an enum left at @c Unset or a
 * dimension left at zero, because a scalar field always reads as a value.
 * The @c validate overloads check what the verifier cannot. Each of them
 * collects every problem it finds instead of stopping at the first, so that a
 * caller can report them all at once.
 */
using Problems = std::vector<std::string>;

/**
 * @brief Validates a port.
 * @param port The port to check.
 * @return A problem for an empty label and one for an unset role, empty when
 * the port is valid.
 */
[[nodiscard]] MQT_SCPD_DESIGN_EXPORT Problems
validate(const flatbuffers::design::PortT& port);

/**
 * @brief Validates a chip and the ports it carries.
 * @param chip The chip to check.
 * @return A problem for every obstacle with fewer than three vertices and for
 * every problem of every port, each prefixed by the port's index, empty when
 * the chip is valid.
 */
[[nodiscard]] MQT_SCPD_DESIGN_EXPORT Problems
validate(const flatbuffers::design::ChipT& chip);

/**
 * @brief Validates one endpoint pair of the solved design.
 * @param connection The connection to check.
 * @return A problem for an unset role at either end, empty when the
 * connection is valid.
 */
[[nodiscard]] MQT_SCPD_DESIGN_EXPORT Problems
validate(const flatbuffers::design::ConnectionT& connection);

/**
 * @brief Validates the design rules.
 * @param rules The rules to check.
 * @return A problem for every length that is not positive and one for a
 * feedline utilization of zero, empty when the rules are valid.
 */
[[nodiscard]] MQT_SCPD_DESIGN_EXPORT Problems
validate(const flatbuffers::design::DesignRulesT& rules);

/**
 * @brief Validates a coplanar-waveguide coupler.
 * @param coupler The coupler to check.
 * @return A problem for an unset rotation, for every dimension that is not
 * positive, for a missing port, for a port whose role is not @c Coupler, and
 * for every problem of that port, empty when the coupler is valid.
 */
[[nodiscard]] MQT_SCPD_DESIGN_EXPORT Problems
validate(const flatbuffers::design::CpwCouplerT& coupler);

/**
 * @brief Validates an air bridge.
 * @param bridge The bridge to check.
 * @return A problem for an unset rotation and for every dimension that is not
 * positive, empty when the bridge is valid.
 */
[[nodiscard]] MQT_SCPD_DESIGN_EXPORT Problems
validate(const flatbuffers::design::BridgeT& bridge);

/**
 * @brief Validates the role patterns of a configuration.
 * @param patterns The patterns to check.
 * @return A problem for every expression that is empty or does not compile,
 * named by its configuration key, empty when the patterns are valid.
 */
[[nodiscard]] MQT_SCPD_DESIGN_EXPORT Problems
validate(const flatbuffers::config::PortPatternsT& patterns);

/**
 * @brief Validates the port section of a configuration.
 * @param ports The section to check.
 * @return A problem for missing patterns, for every problem of the patterns,
 * and for missing sequences, empty when the section is valid.
 */
[[nodiscard]] MQT_SCPD_DESIGN_EXPORT Problems
validate(const flatbuffers::config::PortConfigT& ports);

/**
 * @brief Validates what a configuration can be checked for without its chip.
 * @param config The configuration to check.
 * @return A problem for an empty chip input, for a missing port section or
 * missing design rules, for every problem of both, and for a capacity grid
 * without columns, empty when the configuration is valid on its own.
 */
[[nodiscard]] MQT_SCPD_DESIGN_EXPORT Problems
validate(const flatbuffers::config::ConfigT& config);

/**
 * @brief Validates a configuration against the chip it names.
 * @param config The configuration to check. A configuration without a port
 * section has nothing to check here and yields no problem.
 * @param chip The chip the configuration names, with its ports classified.
 * @return A problem for every sequence label that is not a port of @p chip,
 * that is not routable, or that appears twice, and for every fixed port that
 * is not in @c all_outer, empty when the configuration fits the chip.
 */
[[nodiscard]] MQT_SCPD_DESIGN_EXPORT Problems
validate(const flatbuffers::config::ConfigT& config,
         const flatbuffers::design::ChipT& chip);

} // namespace mqt::scpd::design
