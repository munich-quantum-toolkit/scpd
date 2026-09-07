/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include <nanobind/nanobind.h>

// The bindings expose what the command-line interface needs and nothing else.
// NOLINTNEXTLINE(performance-unnecessary-value-param)
NB_MODULE(MQT_SCPD_MODULE_NAME, m) {
  m.doc() = "Internal bindings of the MQT SCPD core. The command-line "
            "interface is the supported product.";
}
