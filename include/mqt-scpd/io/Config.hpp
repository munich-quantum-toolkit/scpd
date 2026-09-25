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

/**
 * @brief Reads a configuration as the Python loader hands it to the core.
 *
 * The bytes are verified for shape only. What the configuration means is
 * checked by the @c validate overloads of the design module, which a caller
 * runs to report every problem at once.
 *
 * @param bytes The serialized configuration.
 * @return The configuration, with the schema's defaults on every absent field.
 * @throws std::invalid_argument If @p bytes is not a well-formed configuration
 * buffer.
 */
[[nodiscard]] MQT_SCPD_IO_EXPORT flatbuffers::config::ConfigT
readConfig(std::span<const std::uint8_t> bytes);

} // namespace mqt::scpd::io
