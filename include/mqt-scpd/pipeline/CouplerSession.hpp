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

/// The Final stage held after its coupler insertion, to move couplers by
/// hand and look at the capacity graph again: a debugging aid.
///
/// The constructor runs the Final stage as `stop_after_phase =
/// "feedline-insertion"` does: the inner and the outer routing, then the
/// coupler insertion. The state is kept. `setOption` then puts one coupler on
/// another of its options and draws its two feedline edges again, `graph` runs
/// the capacity analysis of `SCPD_BOTTLENECKS` on the chip as it stands,
/// `insertionCheck` runs the capacity check of the coupler insertion, and
/// `routeFeedlines` runs the feedline routing. The stage's own
/// switches apply as in a run of the stage.
class MQT_SCPD_PIPELINE_EXPORT CouplerSession {
public:
  /// Runs the Final stage up to and with the coupler insertion. `debug`
  /// receives the records of that run, the global SAT search included, as
  /// the stage hands them in a run with `-d`, and every record of the
  /// session's later calls.
  ///
  /// @throws std::invalid_argument when the configuration carries no grid
  /// section or no design rules, or the detail routing no grid.
  CouplerSession(const ChipT& chip, const GlobalRoutingT& global,
                 const AssignmentT& assignment, const DetailRoutingT& detail,
                 const ConfigT& config, const Report& report = {},
                 Debug debug = {});
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
  /// saw, in the form of `final-capacity-graph.json`. Under
  /// `SCPD_SLOT_CHECK` (on) the check by crossing slots, `slotCheck`,
  /// judges that graph: `verdict`, `lost`, `over` and `graph` are then its
  /// own, and `previousVerdict` and `previousLost` keep the answer of the
  /// check before.
  [[nodiscard]] std::string insertionCheck();

  /// The two local checks of the global SAT search on chain `chain` with
  /// its couplers on `options`, one per waypoint (0 at a fixed waypoint).
  /// The chain's couplers are put on `options` and their feedline edges
  /// drawn as `setOption` draws them; then the check with every other chain
  /// standing and the check of the chain alone run as the search runs them,
  /// and the couplers go back to where they stood. Returns JSON with
  /// `chain`, `options`, `undrawn` (the chain's edges with no way) and, for
  /// each check (`standing`, `alone`), its `verdict` (`SAT` or `UNSAT`), its
  /// `lost` wires and the `graph` it judged, in the form of
  /// `final-capacity-graph.json`.
  ///
  /// @throws std::invalid_argument when the chain does not exist, `options`
  /// does not hold one option per waypoint, or names an option the coupler
  /// of its waypoint does not have.
  [[nodiscard]] std::string localCheck(std::uint32_t chain,
                                       const std::vector<std::size_t>& options);

  /// The step check of the chain search at edge `step` of chain `chain`,
  /// rebuilt with the chain's couplers on `options`, one per waypoint (0 at
  /// a fixed waypoint): what a `step-check` record judged. The couplers of
  /// waypoints 0 to `step + 1` are put on `options` and their feedline edges
  /// drawn as `setOption` draws them, against the chip as it stands; the
  /// chain search drew them against the prefix it grew, so the ways may
  /// differ from the search's own. The check judges the chip the step check
  /// judges: no other chain standing, the chain's couplers up to the far end
  /// of edge `step` and its edges 0 to `step`, with the chain's own wires
  /// and the free wires beside them routed and judged. The prefix up to edge
  /// `step - 1` is judged the same way to give the wires the step closes.
  /// The couplers then go back to where they stood. Returns JSON with
  /// `chain`, `step`, `edge` (the edge's slot, `f<edge>`), `options` (the
  /// waypoints after `step + 1` on 0, as the record holds them), `verdict`
  /// (`SAT` or `UNSAT`), `lost`, `closed` (lost now but not by the prefix),
  /// `undrawn` (the edges 0 to `step` with no way) and `graph`, the graph
  /// judged, in the form of `final-capacity-graph.json`.
  ///
  /// @throws std::invalid_argument when the chain does not exist, has no
  /// edge `step`, `options` does not hold one option per waypoint, or names
  /// an option the coupler of its waypoint does not have.
  [[nodiscard]] std::string stepCheck(std::uint32_t chain, std::size_t step,
                                      const std::vector<std::size_t>& options);

  /// Every chain as it stands, as JSON: per chain its index (`chain`), the
  /// coupler of every waypoint (`couplers`, null for a fixed waypoint), the
  /// option every waypoint stands on (`options`, 0 for a fixed waypoint)
  /// and its feedline edges by name (`edges`).
  [[nodiscard]] std::string chains() const;

  /// Routes the feedlines of the chip as it stands, then compares the slot
  /// check of every feedline alone with a rip-up and re-route of that
  /// feedline's wires alone: every segment of `span` edges of every chain is
  /// searched as the targeted repair searches it, and up to `runs` runs of
  /// options it reaches are applied, judged and put back. Returns JSON with
  /// `segments`, `classes` (SAT and clean, SAT but failing, UNSAT but clean,
  /// UNSAT and failing, over every feedline judged), `unjudged`, the seconds
  /// spent, `begun` (what the chip as it began came to) and `runs`, one
  /// record per run with the chain, the segment, the moves of couplers onto
  /// options, the slot verdict and what each feedline came to. The feedline
  /// routing changes the state for good; the comparison leaves it as it
  /// found it.
  [[nodiscard]] std::string compareLocally(std::uint32_t runs,
                                           std::size_t span);

  /// Runs the feedline routing of the Final stage on the chip as it stands:
  /// phase 4, as `stop_after_phase = "feedline-constrained-routing"` runs it.
  /// Returns JSON with the fails of every wire as the stage counts them (`bad`
  /// is unrouted, open and crossing together; `unrouted`, `open`, `crossing`,
  /// `short`, `long` and `loops` name the wires), every wire's way (`wires`),
  /// and per round of the feedline sweep its direction, how many wires found a
  /// way in phase 1 and after relaxation, and which failed (`rounds`).
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
