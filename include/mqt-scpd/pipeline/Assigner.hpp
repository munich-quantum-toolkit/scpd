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
#include "mqt-scpd/pipeline/Stages.hpp"
#include "mqt-scpd/pipeline/mqt_scpd_pipeline_export.hpp"

#include <cstdint>
#include <memory>
#include <vector>

namespace mqt::scpd::pipeline {

/// Everything the assignment model reads about the physical design, worked
/// out before the model is built.
///
/// The prototype's model called back into the capacity grid *while it was
/// being built*, running a graph search per ring node to find that node's
/// nearest launcher, and mutated the grid after solving. That is what made
/// the model inseparable from the geometry engine and what made a solver
/// abstraction impossible. Everything the model needs is a plain value here,
/// so building the model touches no grid at all.
struct MQT_SCPD_PIPELINE_EXPORT AssignmentInputs {
  /// The ring the model consumes, in order: ports of the chip.
  std::vector<std::uint32_t> ring;
  /// Whether each ring node is a resonator, which is what needs a launcher.
  std::vector<bool> isResonator;
  /// The launcher ports, in the order the flow variables index them.
  std::vector<std::uint32_t> launchers;
  /// Where each of those launcher slots sits, parallel to `launchers`. A
  /// feed that starts between two launchers is worked out from these.
  std::vector<flatbuffers::geometry::Point> launcherPosition;
  /// The launcher each ring node would reach most cheaply, as an index into
  /// `launchers`. This is the lookup that used to run inside the model.
  std::vector<std::uint32_t> nearestLauncher;
  /// The ring distance between consecutive nodes, which is the weight of the
  /// ring edge that joins them.
  std::vector<double> edgeWeight;
};

/// Work out the assignment inputs from the solved capacity plan and the ring
/// the global stage produced.
///
/// @throws std::invalid_argument when the plan carries no launcher, or the
/// ring is empty.
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT AssignmentInputs
assignmentInputs(const ChipT& chip, const CapacityPlanT& capacity, const GlobalRoutingT& global);

/// The assigner of the first release: a minimum-overlap model on the ring of
/// outer ports, solved as a mixed-integer program.
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT std::unique_ptr<IAssigner> makeOrderedMilpAssigner();

} // namespace mqt::scpd::pipeline
