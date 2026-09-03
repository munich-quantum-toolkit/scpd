# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

add_mqt_scpd_library(design GENERATED_HEADERS design_generated.hpp config_generated.hpp
                     LINK_LIBRARIES MQT::ScpdGeometry)
