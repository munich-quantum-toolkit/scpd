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

/// Read the text of the prototype's routing_config.json into a chip whose
/// ports carry no role yet.
///
/// Of the four top-level keys the loader reads `obstacles` and `ports`;
/// `sampleSpacing` is accepted and ignored, and `nets` must be empty. Every
/// other key, a polygon with fewer than three vertices, a vertex that is not
/// a pair of numbers, and a port without a center are problems. The ports
/// keep the order of the file, which fixes their PortRef indices.
///
/// @throws std::invalid_argument when the text is not JSON or has a problem.
/// The message names every problem, up to a limit.
[[nodiscard]] MQT_SCPD_IO_EXPORT flatbuffers::design::ChipT
readChipJson(std::string_view text);

/// Read the chip input, classify its ports from the configuration's
/// patterns, and check the configuration against the chip.
///
/// @throws std::invalid_argument when the configuration has a problem, when
/// the text has one, when a port matches no pattern or several, or when a
/// configured sequence or start component does not fit the chip.
[[nodiscard]] MQT_SCPD_IO_EXPORT flatbuffers::design::ChipT
loadChip(std::string_view text, const flatbuffers::config::ConfigT& config);

/// Serialize a classified chip for handing it between the core and Python.
///
/// @throws std::invalid_argument when the chip has a problem, such as a port
/// whose role is unset.
[[nodiscard]] MQT_SCPD_IO_EXPORT std::vector<std::uint8_t>
writeChip(const flatbuffers::design::ChipT& chip);

/// Verify serialized chip bytes, structurally and then semantically, and
/// unpack them.
///
/// @throws std::invalid_argument when the bytes are not a well-formed chip
/// buffer or when the chip has a problem.
[[nodiscard]] MQT_SCPD_IO_EXPORT flatbuffers::design::ChipT
readChip(std::span<const std::uint8_t> bytes);

} // namespace mqt::scpd::io
