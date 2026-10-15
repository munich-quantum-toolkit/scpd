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
/// and draws its two feedline edges again, `graph` runs the capacity
/// analysis of `SCPD_BOTTLENECKS` on the chip as it stands,
/// `insertionCheck` runs the capacity check of the coupler insertion, and
/// `routeFeedlines` runs the feedline routing. The stage's own
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

  /// The capacity check of the coupler insertion on the chip as it stands,
  /// with every chain on the options and ways it has now: what the check of
  /// `SCPD_CAPACITY_RULE` says once the last step of every chain stands.
  /// Returns JSON with `verdict` (`SAT` or `UNSAT`), `lost` (the outer wires
  /// the graph has no way for) and `graph`, the capacity graph the check
  /// saw, in the form of `final-capacity-graph.json`.
  [[nodiscard]] std::string insertionCheck();

  /// Runs the feedline routing of the Final stage on the chip as it stands:
  /// phase 4, as `stop_after = "feedlines"` runs it. Returns JSON with the
  /// fails of every wire as the stage counts them (`bad` is unrouted, open
  /// and crossing together; `unrouted`, `open`, `crossing`, `short`, `long`
  /// and `loops` name the wires), every wire's way (`wires`), and per round
  /// of the feedline sweep its direction, how many wires found a way in
  /// phase 1 and after relaxation, and which failed (`rounds`).
  ///
  /// The routing changes the state for good: a coupler moved afterwards
  /// does not move the wires the routing drew.
  [[nodiscard]] std::string routeFeedlines();

  /// Draw the stage's pictures from now on: every search the session makes
  /// afterwards, the feedline routing's included, is handed to `debug` as
  /// the stage hands it its pictures in a run with `-d`.
  void setDebug(Debug debug);

private:
  struct State;
  std::unique_ptr<State> state_;
};

} // namespace mqt::scpd::pipeline
