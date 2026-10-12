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
#include <optional>
#include <string>
#include <vector>

namespace mqt::scpd::pipeline {

/**
 * @brief Everything the assignment model reads about the chip, worked out
 * before the model is built.
 *
 * The values are plain data, so building the model reads no grid and runs no
 * search.
 */
struct MQT_SCPD_PIPELINE_EXPORT AssignmentInputs {
  /// The ring that the model consumes, in order, as port indices.
  std::vector<std::uint32_t> ring;
  /// Whether each ring node is a resonator. Only a resonator needs a
  /// launcher.
  std::vector<bool> isResonator;
  /// The launcher ports, in the order that the flow variables index them.
  std::vector<std::uint32_t> launchers;
  /// Where each launcher slot sits, parallel to `launchers`. A feed between
  /// two launchers is worked out from these.
  std::vector<flatbuffers::geometry::Point> launcherPosition;
  /// The launcher nearest to each ring node, as an index into `launchers`.
  std::vector<std::uint32_t> nearestLauncher;
};

/**
 * @brief Works out the assignment inputs from the capacity plan and the ring
 * of the Global stage.
 * @param chip The classified chip.
 * @param capacity The capacity plan, which carries the launcher slots.
 * @param global The global routing, which carries the ring and the
 * resonators.
 * @return The inputs.
 * @throws std::invalid_argument When the plan carries no launcher, when the
 * ring is empty, or when the launcher slots do not go once around the chip in
 * index order against the direction of the ring. The order is checked only
 * when there are at least three launchers, not all on one line, and the ring
 * encloses an area.
 */
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT AssignmentInputs
assignmentInputs(const ChipT& chip, const CapacityPlanT& capacity,
                 const GlobalRoutingT& global);

/**
 * @brief Checks the launcher numbering of a chip against its configured ring,
 * before any stage runs.
 *
 * The assignment walks the launchers in index order, so they have to go once
 * around the chip in the opposite direction to the port ring. The stage
 * refuses other launchers, but only after the capacity and global stages
 * have run. This check reads only the chip and the configuration, so a run
 * can refuse such a chip first. It reads the centers of the launchers that
 * get a slot and of the configured ring, and follows the same rules as
 * assignmentInputs().
 *
 * @param chip The classified chip.
 * @param config The configuration, which carries the ring.
 * @return What is wrong with the order, or nothing when it is right or has
 * no direction to check.
 */
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT std::optional<std::string>
launcherOrderProblem(const ChipT& chip, const ConfigT& config);

/**
 * @brief Creates the assigner of the first release.
 *
 * The assigner solves a model of the ring of outer ports that keeps the
 * feedlines from crossing, as one mixed-integer program.
 *
 * @return The assigner.
 */
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT std::unique_ptr<IAssigner>
makeOrderedMilpAssigner();

} // namespace mqt::scpd::pipeline
