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

#include "mqt-scpd/drc/mqt_scpd_drc_export.hpp"
#include "mqt-scpd/flatbuffers/artifacts.hpp"
#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/flatbuffers/drc.hpp"
#include "mqt-scpd/flatbuffers/geometry.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"

#include <array>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace mqt::scpd::drc {

/// One wire as a rule sees it.
///
/// The Final stage supplies its wires in router cells and the Finalize stage
/// will supply the same wires as analytic segments in layout units. A rule is
/// written once, against this.
struct CheckedWire {
  /// The connection it draws, which is what a finding names.
  std::uint32_t connection = 0;
  /// The cells it runs over, in the order it runs over them.
  std::span<const flatbuffers::geometry::RCoord> cells;
  /// The components its two ends sit on, empty where an end is not a port.
  ///
  /// No rule reads them any more. The clearance rule used to exempt two wires
  /// that end on one component near their ports, on the assumption that a
  /// component's ports sit closer together than the wire spacing; the two
  /// ports of a qubit can sit nine hundred units apart, and the exemption let
  /// a wire pass the other's approach. A junction is now geometry alone:
  /// two terminals within the clearance of each other.
  std::array<std::string_view, 2> components;
  /// Whether it is an edge of a feedline chain that other wires may cross.
  ///
  /// Every edge is one, at a launcher or between two couplers. Other wires
  /// are meant to cross them, which is what the air-bridges are for, so a
  /// clearance rule between an edge and a conventional or inner wire would
  /// measure something the design never asked for; what binds there is the
  /// crossing rule, which asks for a right angle. An edge does keep the
  /// clearance from every resonator, except within the reach of the coupler
  /// the two share, and from every other edge, except within the reach of a
  /// coupler they meet at.
  ///
  /// The first and last edge of a chain used to be excluded from this and so
  /// stood in every wire's way, which walled off the plane from the chip
  /// edge to the first coupler. The prototype excludes them everywhere
  /// instead (`FinalGrid.cpp:10677` and `:10788`, `is_first_last_feedline`),
  /// and the router now does the same.
  bool feedline = false;
  /// Whether it is an edge of a feedline chain at all, at a launcher or
  /// between two couplers. The search draws every edge free of the crossing
  /// rule and fences it by every other edge instead, so the rule does not
  /// bind on an edge: two edges beside each other are the clearance rule's
  /// business.
  bool edge = false;
  /// Whether it is a resonator.
  bool resonator = false;
  /// The ports its two ends stand on, by index, `NO_PORT` where an end is
  /// not a port. A coupler's port is the one it created, so a resonator and
  /// the edges of its own chain share it, and the rule does not bind between
  /// them near the coupler: that is the coupling.
  std::array<std::uint32_t, 2> ports{NO_PORT, NO_PORT};

  static constexpr std::uint32_t NO_PORT = 0xFFFFFFFFU;
};

/// A coupler as the rules see it: the port it created, where it sits, and
/// how far the meeting at it reaches, in cells.
struct CheckedCoupler {
  std::uint32_t port = CheckedWire::NO_PORT;
  double x = 0.0;
  double y = 0.0;
  double reach = 0.0;
};

/// What the rules see of the Final stage.
struct CellView {
  /// The grid the cells are counted on.
  grid::GridMetrics grid;
  std::vector<CheckedWire> wires;
  std::vector<CheckedCoupler> couplers;
  /// The chip. No rule reads it for now: the obstacle rule is out until its
  /// raster exempts the port approaches the router's mask exempts, because
  /// until then it reports every stub out of a port.
  const flatbuffers::design::ChipT* chip = nullptr;
};

/// What the checker is allowed to forgive.
struct Settings {
  /// Below this many cells two wires are one net, not a near miss.
  double shortThreshold = 2.0;
  /// How far past a junction two wires may still hug, as a multiple of the
  /// clearance the rule requires.
  double junctionRadiusNorm = 1.5;
};

/// What the rules see of a final routing: every wire as the cells it runs
/// over, the edges of the feedline chains with their kind, and the couplers.
/// Built here, once, so that the command line and the tests check the same
/// view.
[[nodiscard]] MQT_SCPD_DRC_EXPORT CellView
viewOfFinal(const flatbuffers::design::ChipT& chip,
            const flatbuffers::artifacts::GlobalRoutingT& global,
            const flatbuffers::artifacts::AssignmentT& assignment,
            const flatbuffers::artifacts::FinalRoutingT& routing,
            const flatbuffers::design::DesignRulesT& rules);

/// Run the rules that read a view in router cells: wire clearance, feedline
/// orthogonality and wire loop.
///
/// Obstacle clearance is out for now: the keepout is baked into the
/// router's mask with the port approaches exempted, and a check that
/// rasterizes the keepout without those exemptions reports every stub out
/// of a port.
[[nodiscard]] MQT_SCPD_DRC_EXPORT flatbuffers::drc::DrcReportT
checkCells(const CellView& view, const flatbuffers::design::DesignRulesT& rules,
           const Settings& settings = {});

/// The reports of a run as the text of `drc.json`.
///
/// The report is schema-defined even though it is written as JSON, because a
/// violation has to be machine-readable. It is rendered here rather than in
/// Python so that the one schema does not have to be generated twice.
[[nodiscard]] MQT_SCPD_DRC_EXPORT std::string
toJson(const flatbuffers::drc::DrcReportsT& reports);

} // namespace mqt::scpd::drc
