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
#include <string>

namespace mqt::scpd::pipeline {

/// The Final stage held after its coupler insertion, to move couplers by
/// hand and look at the capacity graph again: a debugging aid.
///
/// The constructor runs the Final stage as `stop_after = "couplers"` does:
/// the inner and the outer routing, then the coupler insertion. The state
/// is kept. `setOption` then puts one coupler on another of its options
/// and draws its two feedline edges again, and `graph` runs the capacity
/// analysis of `SCPD_BOTTLENECKS` on the chip as it stands. The stage's own
/// switches apply as in a run of the stage.
class MQT_SCPD_PIPELINE_EXPORT CouplerSession {
public:
  /// Runs the Final stage up to and with the coupler insertion.
  ///
  /// @throws std::invalid_argument when the configuration carries no grid
  /// section or no design rules, or the detail routing no grid.
  CouplerSession(const ChipT& chip, const GlobalRoutingT& global,
                 const AssignmentT& assignment, const DetailRoutingT& detail,
                 const ConfigT& config, const Progress& progress = {},
                 std::uint32_t verbosity = 0);
  ~CouplerSession();
  CouplerSession(const CouplerSession&) = delete;
  CouplerSession& operator=(const CouplerSession&) = delete;
  CouplerSession(CouplerSession&&) = delete;
  CouplerSession& operator=(CouplerSession&&) = delete;

  /// The couplers as they stand and every option of each, as JSON.
  [[nodiscard]] std::string couplers() const;

  /// Puts coupler `coupler` on option `option` and draws its two feedline
  /// edges again. Returns JSON with what became of each edge.
  ///
  /// @throws std::invalid_argument when the coupler or the option does not
  /// exist.
  std::string setOption(std::uint32_t coupler, std::uint32_t option);

  /// The capacity graph of the chip as it stands, as the JSON of
  /// `final-capacity-graph.json`.
  [[nodiscard]] std::string graph();

private:
  struct State;
  std::unique_ptr<State> state_;
};

} // namespace mqt::scpd::pipeline
