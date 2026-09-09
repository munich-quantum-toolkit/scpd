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
#include "mqt-scpd/io/mqt_scpd_io_export.hpp"

#include <cstdint>
#include <span>

namespace mqt::scpd::io {

/// Verify serialized configuration bytes, as the Python loader hands them
/// to the core, and unpack them.
///
/// The configuration is checked for shape only. Its meaning is checked by
/// the design module's validate, which a caller runs to report every
/// problem at once.
///
/// @throws std::invalid_argument when the bytes are not a well-formed
/// configuration buffer.
[[nodiscard]] MQT_SCPD_IO_EXPORT flatbuffers::config::ConfigT
readConfig(std::span<const std::uint8_t> bytes);

} // namespace mqt::scpd::io
