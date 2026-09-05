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
#include "mqt-scpd/io/mqt_scpd_io_export.hpp"

#include <cstdint>
#include <span>
#include <vector>

namespace mqt::scpd::io {

using design::Problems;

/// Problems of an artifact: an empty producer, a missing output, and the
/// problems of the components its output carries.
[[nodiscard]] MQT_SCPD_IO_EXPORT Problems
validate(const flatbuffers::artifacts::ArtifactT& artifact);

/// Serialize an artifact as a run directory stores it.
///
/// @throws std::invalid_argument when the artifact has a problem. The message
/// lists every problem.
[[nodiscard]] MQT_SCPD_IO_EXPORT std::vector<std::uint8_t>
writeArtifact(const flatbuffers::artifacts::ArtifactT& artifact);

/// Verify stored bytes, structurally and then semantically, and unpack them.
///
/// @throws std::invalid_argument when the bytes are not a well-formed artifact
/// with the identifier of this schema, or when the artifact has a problem.
[[nodiscard]] MQT_SCPD_IO_EXPORT flatbuffers::artifacts::ArtifactT
readArtifact(std::span<const std::uint8_t> bytes);

} // namespace mqt::scpd::io
