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

#include "mqt-scpd/pipeline/Stages.hpp"
#include "mqt-scpd/pipeline/mqt_scpd_pipeline_export.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace mqt::scpd::pipeline {

/// The two ports of a component that a wire may pass between.
///
/// A coupler's artwork has routable ports on opposite sides, and an inner
/// wire that reaches one of them leaves through the other. Which two ports
/// those are is declared by the configuration's bridge rules, and which
/// component a port belongs to by its component pattern, so nothing reads a
/// label to find either. `design::componentsOf` is what works the pairs out.
struct PortBridge {
  std::uint32_t from = 0;
  std::uint32_t to = 0;
};

/// What the inner circuit is solved over, derived from the chip, the
/// configured ring and the component grouping.
struct MQT_SCPD_PIPELINE_EXPORT InnerCircuit {
  /// The routable ports the configured ring does not carry.
  std::vector<std::uint32_t> innerPorts;
  /// The inner ports that no bridge claims, which are the ports the inner
  /// circuit has to reach: every port of an inner qubit, and the one port of
  /// a coupler that is left over once its bridges are paired off.
  std::vector<std::uint32_t> targets;
  /// Every declared bridge, oriented so that `from` is the inner port.
  std::vector<PortBridge> bridges;
  /// The bridges whose `to` port the configured ring carries. The ring keeps
  /// such a port only where the solved circuit actually surfaces there. Every
  /// other bridge is internal, and the Global stage shuts it unless
  /// `[stages.global] internal_bridges` says otherwise.
  std::vector<PortBridge> outerBridges;
};

/// Derive the inner circuit of a chip.
///
/// @throws std::invalid_argument when the configuration carries no ring, a
/// ring label is not a port of the chip, or the ring carries both ends of one
/// bridge and so leaves that crossing no inside to come from.
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT InnerCircuit innerCircuitOf(const ChipT& chip,
                                                                   const ConfigT& config);

/// The global router of the first release: a binary flow model on the Hanan
/// lattice each capacity chain induces, solved as a mixed-integer program.
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT std::unique_ptr<IGlobalRouter> makeHananMilpRouter();

} // namespace mqt::scpd::pipeline
