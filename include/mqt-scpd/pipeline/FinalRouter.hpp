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

/// The final router of the first release.
///
/// Curvature-constrained A* over the Dubins primitives of one bend radius, on
/// a grid whose cell is about ten layout units. It runs five phases and each
/// one leaves a snapshot of what it drew:
///
/// 1. the wires of the inner circuit, inside their own unit cells;
/// 2. the wires of the ring, with a meander where a resonator is short;
/// 3. the CPW couplers, which carry the resonators' source ports;
/// 4. the launcher-to-launcher feedline chains that drive the couplers;
/// 5. the refinement that widens the room around each of them.
///
/// One driver runs every routing phase. It sweeps the wire list, alternating
/// direction, and offers each wire a way of its own inside a band around the
/// way it has; a wire that finds none lets go of its neighbours one at a time
/// and tries again, and rolls them back when nothing came of it. The
/// prototype writes that loop out four times, once per phase, with the phases
/// differing only in their parameters.
///
/// The clearance holds against **every** wire. The prototype keeps a wire
/// clear of the two beside it in the ring and of nothing else, which is what
/// its own pictures show; here the grid carries how many wires guard each
/// cell, a wire charges its own when it is put down and gives it up when it
/// is taken off, and a search refuses a guarded cell outright.
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT std::unique_ptr<IFinalRouter>
makeDubinsFinalRouter();

} // namespace mqt::scpd::pipeline
