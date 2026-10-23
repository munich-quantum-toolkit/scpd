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

/// The two cells a place of the corridor plan lies between, or one cell twice.
///
/// The Corridor stage writes its places as layout points of the corner frame,
/// in which a cell is the unit square from its corner: a feed, a target and a
/// slot of a border sit at the centre of their cell, and a crossing on the
/// edge between two cells, so either of the two will do. The detail grid
/// names a cell by its centre, half a cell on, which is why the frame is
/// converted before the cell is read.
///
/// @param detail The metrics of the detail grid.
/// @param place The layout point the corridor wrote.
/// @return For a crossing, the cell on its left or above it and the cell on
/// its right or below it; for a place inside a cell, or a crossing on the
/// first row or column, that one cell twice; nothing for a place beyond the
/// grid.
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT
    std::optional<std::array<flatbuffers::geometry::DCoord, 2>>
    cellsOfPlace(const grid::GridMetrics& detail,
                 const flatbuffers::geometry::Point& place);

/// The detail router of the first release.
///
/// Eight-connected A* on the detail grid, in three passes.
///
/// A corridor cuts its wire into one piece per partition it names, and every
/// piece is searched for inside that partition alone; the wires of one
/// partition are tried in several orders and the order that draws the most of
/// them wins. The pieces are then joined in corridor order — a concatenation
/// and not a search, because the corridor already says which piece belongs to
/// which wire. Finally every wire is offered a way of its own inside a band
/// around the one it was given, which is what takes the kinks out of the
/// joints, and the inner circuit is drawn last against the finished ring.
///
/// Two wires never share a cell. Where the prototype lets a wire cross another
/// at a price and keeps only a set of pixels beside its crossing test, the
/// occupancy here is per wire and hard, and a diagonal step whose two corners
/// are both taken is refused, which is the only other way two eight-connected
/// paths can cross.
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT std::unique_ptr<IDetailRouter>
makePixelAStarRouter();

} // namespace mqt::scpd::pipeline
