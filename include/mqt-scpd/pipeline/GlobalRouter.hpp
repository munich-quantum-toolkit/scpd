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

#include <cstdint>
#include <memory>
#include <vector>

namespace mqt::scpd::pipeline {

/**
 * @brief Two ports of one component that a wire passes between.
 *
 * A coupler has routable ports on opposite sides, and an inner wire that
 * reaches one of them leaves through the other. The bridge rules of the
 * configuration declare which two ports those are, and the component pattern
 * which component a port belongs to, so no label is read to find either.
 */
struct PortBridge {
  /// The port index of the inner end.
  std::uint32_t from = 0;
  /// The port index of the other end.
  std::uint32_t to = 0;
};

/**
 * @brief What the inner circuit is solved over: the ports the configured ring
 * does not carry, and how the bridges join them to it.
 */
struct MQT_SCPD_PIPELINE_EXPORT InnerCircuit {
  /// The routable ports the configured ring does not carry, ascending.
  std::vector<std::uint32_t> innerPorts;
  /// The inner ports that no bridge claims, ascending. These are the ports the
  /// inner circuit has to reach: every port of an inner qubit, and the one
  /// port of a coupler that is left once its bridges are paired off.
  std::vector<std::uint32_t> targets;
  /// Every declared bridge, oriented so that `from` is the inner end.
  std::vector<PortBridge> bridges;
  /// The bridges whose `to` port the configured ring carries. The ring keeps
  /// such a port only where the solved circuit surfaces there. Every other
  /// bridge is internal, and the stage shuts it unless
  /// `[stages.global] internal_bridges` says otherwise.
  std::vector<PortBridge> outerBridges;
};

/**
 * @brief Derives the inner circuit of a chip.
 * @param chip The classified chip.
 * @param config The run configuration, which carries the ring and the bridge
 * rules.
 * @return The inner circuit.
 * @throws std::invalid_argument When the configuration carries no ring, when
 * a ring label is not a port of the chip, or when the ring carries both ends
 * of one bridge and so leaves that bridge no inner end.
 */
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT InnerCircuit
innerCircuitOf(const ChipT& chip, const ConfigT& config);

/**
 * @brief Creates the global router of the first release.
 *
 * The router solves a binary flow model on the Hanan lattice of each
 * capacity chain as one mixed-integer program.
 *
 * @return The router.
 */
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT std::unique_ptr<IGlobalRouter>
makeHananMilpRouter();

} // namespace mqt::scpd::pipeline
