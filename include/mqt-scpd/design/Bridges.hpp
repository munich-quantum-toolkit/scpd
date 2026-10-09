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

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace mqt::scpd::design {

/// The rules that pair the ports of a component, as the configuration carries
/// them.
using BridgeRules =
    std::vector<std::unique_ptr<flatbuffers::config::BridgeRuleT>>;

/// Two ports of one component that a wire crosses between: it enters at one
/// and leaves at the other.
struct BridgePair {
  /// The port index of the end that matched the first side of the rule.
  std::uint32_t first = 0;
  /// The port index of the end that matched the second side of the rule.
  std::uint32_t second = 0;
};

/// Which side of which rule the label of a port matches, and what that side
/// captured. Two ports pair when they match the two sides of one rule and
/// their captures agree.
struct BridgeMatch {
  /// The rule, by its position in the configured list.
  std::size_t rule = 0;
  /// Whether the label matched the first side of the rule rather than its
  /// second.
  bool first = false;
  /// The capture of the one group of the side that matched.
  std::string key;
};

/// How the ports of one component take part in routing.
struct ComponentPorts {
  /// The component the ports name.
  std::string name;
  /// Whether the component carries a resonator port, which makes it a qubit.
  bool qubit = false;
  /// Its routable ports, by port index, ascending.
  std::vector<std::uint32_t> ports;
  /// The pairs a wire crosses, in the order the rules declare them.
  std::vector<BridgePair> bridges;
  /// The routable ports that no bridge claims.
  std::vector<std::uint32_t> ends;
};

/**
 * @brief Lists every rule side that the label of each port matches.
 *
 * A port matches at most one side of one rule in a sound configuration. The
 * function reports every match rather than the first, so that a label two
 * rules claim is a problem the configuration check can name, instead of a
 * pairing that depends on the order the rules were written in. A rule that is
 * missing or does not compile matches nothing and keeps its position.
 *
 * @param chip The chip whose port labels are matched.
 * @param rules The bridge rules of the configuration.
 * @return One list of matches per port, indexed like @p chip's ports.
 */
[[nodiscard]] MQT_SCPD_DESIGN_EXPORT std::vector<std::vector<BridgeMatch>>
bridgeMatchesOf(const flatbuffers::design::ChipT& chip,
                const BridgeRules& rules);

/**
 * @brief Groups the routable ports of a chip by component and pairs them.
 *
 * A component that carries a resonator port is a qubit. No rule names the
 * ports of a qubit, so each of them ends a wire. The ports of a coupler pair
 * off across its artwork, and a port left over ends a wire too.
 *
 * Which two ports pair is declared: a rule names both sides of the crossing,
 * each with one capture group, and two ports of one component pair when they
 * match the two sides of one rule with equal captures. The rules apply in the
 * order the configuration lists them, and each rule over the ports of a
 * component in ascending order, so the pairing depends on the configuration
 * alone.
 *
 * @param chip The classified chip. A port without a component, or with a role
 * no wire runs to, takes no part.
 * @param rules The bridge rules of the configuration.
 * @return The components, ordered by name, so that the result does not depend
 * on the order the ports were read in.
 */
[[nodiscard]] MQT_SCPD_DESIGN_EXPORT std::vector<ComponentPorts>
componentsOf(const flatbuffers::design::ChipT& chip, const BridgeRules& rules);

/**
 * @brief Marks the ports that are one end of a bridge.
 *
 * A bridge port needs twice the approach of a port that ends a wire: the wire
 * does not stop there but carries on across the component and out of the
 * other end, so the space in front of it has to clear the artwork of the
 * component rather than only leave the port.
 *
 * @param chip The classified chip.
 * @param rules The bridge rules of the configuration.
 * @return One flag per port, indexed like @p chip's ports.
 */
[[nodiscard]] MQT_SCPD_DESIGN_EXPORT std::vector<bool>
bridgingPorts(const flatbuffers::design::ChipT& chip, const BridgeRules& rules);

} // namespace mqt::scpd::design
