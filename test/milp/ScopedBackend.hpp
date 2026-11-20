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

#include "mqt-scpd/milp/Backend.hpp"

#include <memory>
#include <utility>

namespace mqt::scpd::milp::test {

/// Registers an external backend while it lives, and clears the registration
/// again, also when the test fails on the way.
///
/// The registration is one per process. Without the guard, a test that fails
/// before it clears the registration leaves its backend to every later test
/// of the same binary.
class ScopedBackend {
public:
  explicit ScopedBackend(std::shared_ptr<ISolverBackend> backend) {
    setExternalBackend(std::move(backend));
  }
  ScopedBackend(const ScopedBackend&) = delete;
  ScopedBackend& operator=(const ScopedBackend&) = delete;
  ScopedBackend(ScopedBackend&&) = delete;
  ScopedBackend& operator=(ScopedBackend&&) = delete;
  ~ScopedBackend() { setExternalBackend(nullptr); }
};

} // namespace mqt::scpd::milp::test
