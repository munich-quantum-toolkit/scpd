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

#include <memory>

namespace mqt::scpd::pipeline {

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
