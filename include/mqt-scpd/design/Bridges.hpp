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

/// The rules that pair a component's ports, as the configuration carries them.
using BridgeRules =
    std::vector<std::unique_ptr<flatbuffers::config::BridgeRuleT>>;

/// A pair of ports of one component that a wire crosses: it enters at one and
/// leaves at the other.
struct BridgePair {
  std::uint32_t first = 0;
  std::uint32_t second = 0;
};

/// Which side of which rule a port's label matches, and what that side
/// captured. Two ports pair when they match the two sides of one rule and
/// their captures agree.
struct BridgeMatch {
  /// The rule, by position in the configured list.
  std::size_t rule = 0;
  /// Whether the label matched the rule's first side rather than its second.
  bool first = false;
  /// The one capture group of the side that matched.
  std::string key;
};

/// How one component's ports take part in routing.
struct ComponentPorts {
  /// The component the ports name.
  std::string name;
  /// Whether the component carries a resonator, which makes it a qubit.
  bool qubit = false;
  /// Its routable ports, by port index, ascending.
  std::vector<std::uint32_t> ports;
  /// The pairs a wire crosses, in the order the rules declare them.
  std::vector<BridgePair> bridges;
  /// The routable ports no bridge claims.
  std::vector<std::uint32_t> ends;
};

/// Every rule side each port's label matches, indexed by port.
///
/// A port matches at most one side of one rule in a sound configuration.
/// The function reports every match rather than the first, so that a label
/// two rules claim is a problem the configuration check can name instead of
/// a pairing that silently depends on the order the rules were written in.
[[nodiscard]] MQT_SCPD_DESIGN_EXPORT std::vector<std::vector<BridgeMatch>>
bridgeMatchesOf(const flatbuffers::design::ChipT& chip,
                const BridgeRules& rules);

/// The components of a chip and how their ports pair, ordered by name so that
/// the result does not depend on the order the ports were read in.
///
/// A component that carries a resonator port is a qubit. No rule names a
/// qubit's ports, so each of them ends a wire; a coupler's ports pair off
/// across its artwork and the leftover one ends a wire too.
///
/// Which two ports pair is **declared**: a rule names both sides of the
/// crossing, each with one capture group, and two ports of one component pair
/// when they match the two sides of one rule with equal captures. Geometry
/// picks the same two ports on the chips at hand — the ends of a crossing face
/// opposite ways and sit a coupler's width apart — but it says what the
/// artwork happens to be, not what the crossing is meant to be, and the two
/// were never checked against each other.
[[nodiscard]] MQT_SCPD_DESIGN_EXPORT std::vector<ComponentPorts>
componentsOf(const flatbuffers::design::ChipT& chip, const BridgeRules& rules);

/// Whether each port of the chip is one end of a bridge, indexed by port.
///
/// A bridge port needs twice the approach a port that merely ends a wire
/// needs: the wire does not stop there, it carries on across the component
/// and out of the other end, so the space in front of it has to clear the
/// component's own artwork rather than just leave the port.
[[nodiscard]] MQT_SCPD_DESIGN_EXPORT std::vector<bool>
bridgingPorts(const flatbuffers::design::ChipT& chip, const BridgeRules& rules);

} // namespace mqt::scpd::design
