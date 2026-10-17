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

#include "mqt-scpd/design/Validation.hpp"
#include "mqt-scpd/flatbuffers/artifacts.hpp"
#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/io/mqt_scpd_io_export.hpp"

#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace mqt::scpd::io {

using design::Problems;

/**
 * @brief Validates a stage artifact.
 *
 * Besides the components that the output carries, the check covers what the
 * later stages rely on: a capacity chain is a tree whose links stay in the
 * plan, a target lies on one chain only, and a gate has the capacity of its
 * bottleneck; the lattice of a Global routing names only its own points and
 * edges; an Assignment has one connection, launcher and feed per ring node
 * and chains of ring nodes; a corridor names one more partition than it has
 * crossings.
 *
 * @param artifact The artifact to check.
 * @return A problem for an empty producer, one for a missing output, and every
 * other problem found, empty when the artifact is valid.
 */
[[nodiscard]] MQT_SCPD_IO_EXPORT Problems
validate(const flatbuffers::artifacts::ArtifactT& artifact);

/**
 * @brief Checks that a stage artifact fits a chip.
 *
 * An artifact names ports by their index into the chip. An artifact that was
 * made from another chip can name a port this chip does not have, and a
 * stage that read it would read past the end of the port list.
 *
 * @param artifact The artifact to check.
 * @param chip The chip a stage reads the artifact with.
 * @return A problem for every port the artifact names that @p chip does not
 * have, empty when the artifact fits.
 */
[[nodiscard]] MQT_SCPD_IO_EXPORT Problems
validate(const flatbuffers::artifacts::ArtifactT& artifact,
         const flatbuffers::design::ChipT& chip);

/**
 * @brief Serializes a stage artifact as a run directory stores it.
 * @param artifact The artifact to serialize.
 * @pre @p artifact is valid, so validate() reports no problem for it.
 * @return The bytes of the artifact, beginning with the file identifier that
 * records the schema version.
 * @throws std::invalid_argument If @p artifact has a problem. The message
 * names every problem.
 */
[[nodiscard]] MQT_SCPD_IO_EXPORT std::vector<std::uint8_t>
writeArtifact(const flatbuffers::artifacts::ArtifactT& artifact);

/**
 * @brief Reads back the bytes that a run directory stores.
 *
 * The bytes are verified structurally, by the FlatBuffers verifier, and then
 * semantically, by validate().
 *
 * @param bytes The stored artifact.
 * @return The artifact.
 * @throws std::invalid_argument If @p bytes is not a well-formed artifact with
 * the identifier of this schema version, or if the artifact it holds has a
 * problem.
 */
[[nodiscard]] MQT_SCPD_IO_EXPORT flatbuffers::artifacts::ArtifactT
readArtifact(std::span<const std::uint8_t> bytes);

/**
 * @brief Renders a stored artifact as JSON.
 *
 * The field names, the enum names and the union tags come from the type tables
 * that the schema compiler emits, so the JSON follows the schema without a
 * second description of the model. Names are the ones the schema spells, which
 * are snake_case.
 *
 * @param bytes The stored artifact.
 * @return The artifact as indented JSON, ending in a newline.
 * @throws std::invalid_argument If @p bytes is not a well-formed artifact with
 * the identifier of this schema version, or if the artifact it holds has a
 * problem.
 */
[[nodiscard]] MQT_SCPD_IO_EXPORT std::string
artifactToJson(std::span<const std::uint8_t> bytes);

} // namespace mqt::scpd::io
