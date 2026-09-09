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

#include "mqt-scpd/flatbuffers/config.hpp"
#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/io/mqt_scpd_io_export.hpp"

#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

namespace mqt::scpd::io {

/**
 * @brief Reads the chip input into the in-memory model.
 *
 * Of the four top-level keys of `routing_config.json`, the loader reads
 * @c obstacles and @c ports. It accepts and ignores @c sampleSpacing, and it
 * requires @c nets to be empty. Every other key, a polygon with fewer than
 * three vertices, a vertex that is not a pair of numbers, and a port without a
 * center are problems. The ports keep the order of the file, which fixes the
 * index each of them is referred to by.
 *
 * @param text The JSON text of the chip input.
 * @return The chip, with every port left at the role @c Unset. Classifying the
 * ports is the job of loadChip().
 * @throws std::invalid_argument If @p text is not a JSON object, or if it has
 * a problem. The message names every problem, up to a limit.
 */
[[nodiscard]] MQT_SCPD_IO_EXPORT flatbuffers::design::ChipT
readChipJson(std::string_view text);

/**
 * @brief Reads the chip input and fits it to a configuration.
 *
 * The function validates the configuration, reads @p text as readChipJson()
 * does, classifies every port from the configuration's role patterns, and
 * checks the configured port sequences against the resulting chip.
 *
 * @param text The JSON text of the chip input.
 * @param config The configuration that names the patterns and the sequences.
 * @return The chip, with a role on every port.
 * @throws std::invalid_argument If the configuration has a problem, if
 * @p text has one, if a port matches no role pattern or more than one, or if a
 * configured sequence does not fit the chip. The message names every problem,
 * up to a limit.
 */
[[nodiscard]] MQT_SCPD_IO_EXPORT flatbuffers::design::ChipT
loadChip(std::string_view text, const flatbuffers::config::ConfigT& config);

/**
 * @brief Serializes a classified chip for the boundary between core and
 * Python.
 * @param chip The chip to serialize.
 * @pre @p chip is valid, so every port carries a label and a role.
 * @return The bytes of the chip in the layout of the design schema.
 * @throws std::invalid_argument If @p chip has a problem, such as a port whose
 * role is unset. The message names every problem.
 */
[[nodiscard]] MQT_SCPD_IO_EXPORT std::vector<std::uint8_t>
writeChip(const flatbuffers::design::ChipT& chip);

/**
 * @brief Reads back the bytes that writeChip() produced.
 *
 * The bytes are verified structurally, by the FlatBuffers verifier, and then
 * semantically, by the design module's validation.
 *
 * @param bytes The serialized chip.
 * @return The chip.
 * @throws std::invalid_argument If @p bytes is not a well-formed chip buffer,
 * or if the chip it holds has a problem.
 */
[[nodiscard]] MQT_SCPD_IO_EXPORT flatbuffers::design::ChipT
readChip(std::span<const std::uint8_t> bytes);

} // namespace mqt::scpd::io
