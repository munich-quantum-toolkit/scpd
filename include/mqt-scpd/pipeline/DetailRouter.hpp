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

#include "mqt-scpd/flatbuffers/geometry.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"
#include "mqt-scpd/pipeline/Stages.hpp"
#include "mqt-scpd/pipeline/mqt_scpd_pipeline_export.hpp"

#include <array>
#include <memory>
#include <optional>

namespace mqt::scpd::pipeline {

/**
 * @brief Returns the cells of the detail grid that a place of a corridor
 * names.
 *
 * The Corridor stage writes its places as layout points of the corner frame,
 * in which a cell is the unit square from its corner. A feed and a target sit
 * at the center of their cell, and a crossing sits on the edge between two
 * cells.
 *
 * @param detail The metrics of the detail grid.
 * @param place The layout point that the corridor routing holds.
 * @return For a crossing on a vertical edge, the cell to its left and the cell
 * to its right; on a horizontal edge, the cell above it and the cell below it.
 * For a place inside a cell, or a crossing on the first row or column, that
 * one cell twice. None for a place beyond the grid.
 */
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT
    std::optional<std::array<flatbuffers::geometry::DCoord, 2>>
    cellsOfPlace(const grid::GridMetrics& detail,
                 const flatbuffers::geometry::Point& place);

/**
 * @brief Creates the detail router of the first release.
 *
 * The router runs an eight-connected A* search on the detail grid. No two
 * wires share a cell, and no diagonal step passes between two cells of one
 * other wire, because in copper that step is a crossing.
 *
 * The router first draws a seed for every wire. It cuts the corridor of a
 * wire at its crossings into one piece per partition and searches each piece
 * inside its partition. It tries the pieces of one partition in up to
 * `[stages.detail] orderings` orders and keeps the order that draws the most
 * pieces. Where a piece finds no way, the router moves the wires in its way
 * and draws them again. It then joins the pieces of each wire. A wire with a
 * missing piece is searched again from end to end, and the wires of the inner
 * circuit are searched last, over the space the ring leaves.
 *
 * The seed ignores the clearance. The router then routes every wire again
 * across the borders, inside a band of `[stages.detail] corridor_spacings`
 * wire spacings around its seed, and checks the discrete clearance
 * derived from `[design_rules] min_wire_spacing` against every other wire. It
 * sweeps the wires for at most `[stages.detail] rounds` rounds, alternating
 * forward and backward. A wire that finds no way may release up to
 * `[stages.detail] max_relaxation` neighbors per direction in the wire list.
 * It releases each neighbor at most once, first along the sweep and then
 * against it. Each release starts a search in the same band with a cost
 * that favors the space between the wire's two ring neighbors.
 *
 * The clearance is the rule converted to whole cells: the rule spans
 * `ceil(min_wire_spacing / cell)` cells, and the router keeps one cell less
 * than that clear. A wire fails when it has no path or when one of its cells
 * lies within that clearance of another wire. A connection without a
 * corridor gets a wire without a path but is no fail of this stage, because
 * the Corridor stage counts it already.
 *
 * @return The router.
 */
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT std::unique_ptr<IDetailRouter>
makePixelAStarRouter();

} // namespace mqt::scpd::pipeline
