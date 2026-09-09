# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

# HiGHS is a private dependency: it is what the module solves with, and nothing above the module
# sees a HiGHS type.
add_mqt_scpd_library(milp PRIVATE_LINK_LIBRARIES mqt-scpd-highs)
