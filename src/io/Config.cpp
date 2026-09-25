/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/io/Config.hpp"

#include "mqt-scpd/flatbuffers/config.hpp"

#include <flatbuffers/buffer.h>
#include <flatbuffers/verifier.h>

#include <cstdint>
#include <span>
#include <stdexcept>

namespace mqt::scpd::io {

flatbuffers::config::ConfigT
readConfig(const std::span<const std::uint8_t> bytes) {
  ::flatbuffers::Verifier verifier(bytes.data(), bytes.size());
  if (!verifier.VerifyBuffer<flatbuffers::config::Config>(nullptr)) {
    throw std::invalid_argument("bytes are not a configuration buffer");
  }
  flatbuffers::config::ConfigT config;
  ::flatbuffers::GetRoot<flatbuffers::config::Config>(bytes.data())
      ->UnPackTo(&config);
  return config;
}

} // namespace mqt::scpd::io
