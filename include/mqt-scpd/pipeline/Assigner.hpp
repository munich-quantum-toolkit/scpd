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
 * @throws std::invalid_argument When the plan carries no launcher or the ring
 * is empty.
 */
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT AssignmentInputs
assignmentInputs(const ChipT& chip, const CapacityPlanT& capacity,
                 const GlobalRoutingT& global);

/**
 * @brief Checks the order in which the chip input lists its launchers against
 * the configured ring.
 *
 * The assignment walks the launchers in the order of the chip input, so they
 * have to go once around the chip in the opposite direction to the port ring.
 * The check reads only the chip and the configuration: the centers of the
 * launchers that get a slot, and of the configured ring. The Assignment stage
 * runs this check and refuses what it reports, and planProblems() runs it
 * before any stage. The order is checked only when there are at least three
 * launchers, not all on one line, and the ring encloses an area.
 *
 * @param chip The classified chip.
 * @param config The configuration, which carries the ring.
 * @return What is wrong with the order, or nothing when it is right or has
 * no direction to check.
 */
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT std::optional<std::string>
launcherOrderProblem(const ChipT& chip, const ConfigT& config);

/**
 * @brief Checks that the launchers of a chip with resonators can feed them.
 *
 * The assignment feeds a resonator on the segment between two neighbouring
 * launchers. With one launcher, that segment has no length. With two
 * launchers, or with launchers that all lie on one line, two segments run
 * along one line, and two resonators can share one feed point. A chip whose
 * ports include a resonator therefore needs at least three launchers that do
 * not all lie on one line.
 *
 * The check reads the resonator ports of the chip and the launcher slots at
 * the places the Capacity stage gives them, so that it judges the points the
 * Assignment stage feeds between. planProblems() runs it before any stage.
 * The Assignment stage refuses the same launchers only where the ring it
 * assigns carries a resonator.
 *
 * @param chip The classified chip.
 * @param config The run configuration, whose grid places the slots.
 * @return What is wrong with the launchers, or nothing when the chip has no
 * resonator or launchers enough.
 */
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT std::optional<std::string>
launcherPlacesProblem(const ChipT& chip, const ConfigT& config);

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
