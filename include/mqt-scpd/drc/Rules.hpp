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
  /// Whether it is a feedline that runs from one coupler to another.
  ///
  /// Other wires are meant to cross those, which is what the air-bridges are
  /// for, so a clearance rule against them would measure something the design
  /// never asked for. The first and last edge of a chain are hard obstacles
  /// for every wire and are checked.
  bool feedline = false;
};

/// What the rules see of the Final stage.
struct CellView {
  /// The grid the cells are counted on.
  grid::GridMetrics grid;
  std::vector<CheckedWire> wires;
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

/// Run the rules that read a view in router cells: wire clearance and wire
/// loop.
///
/// Feedline orthogonality is not here. It needs feedlines, which the stage
/// does not produce yet. Obstacle clearance is out for now: the keepout is
/// baked into the router's mask with the port approaches exempted, and a
/// check that rasterizes the keepout without those exemptions reports every
/// stub out of a port.
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
