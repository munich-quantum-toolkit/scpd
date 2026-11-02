/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// The stage tests solve with the backend that their configuration names. The
// environment variable SCPD_SOLVER overrides that backend, so a value in the
// shell that runs the tests would change what they test. This file clears the
// variable once, before the first test of the binary.

// <cstdlib> declares unsetenv and _putenv_s, but include-cleaner knows no
// header for these functions outside the C++ standard.
#include <cstdlib> // NOLINT(misc-include-cleaner)

namespace mqt::scpd::pipeline {
namespace {

/// Clears SCPD_SOLVER when the test binary starts, before the first test.
/// Neither call throws, so the object may live in static storage.
struct DefaultSolver {
  DefaultSolver() noexcept {
#ifdef _WIN32
    _putenv_s("SCPD_SOLVER", "");
#else
    unsetenv("SCPD_SOLVER"); // NOLINT(misc-include-cleaner)
#endif
  }
};

[[maybe_unused]] const DefaultSolver DEFAULT_SOLVER;

} // namespace
} // namespace mqt::scpd::pipeline
