/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "Add.hpp"

#include <nanobind/nanobind.h>

namespace nb = nanobind;
using namespace nb::literals;

// NOLINTNEXTLINE(performance-unnecessary-value-param)
NB_MODULE(MQT_SCPD_MODULE_NAME, m) { m.def("add", &add, "Add two integers."); }
