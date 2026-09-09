# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

# Boost.Polygon is a private dependency: the one Voronoi construction of the capacity stage lives
# here, and nothing above this module sees a Boost type.
add_mqt_scpd_library(grid LINK_LIBRARIES MQT::ScpdGeometry MQT::ScpdDesign PRIVATE_LINK_LIBRARIES
                     mqt-scpd-polygon)
