# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

add_mqt_scpd_library(
  drc
  GENERATED_HEADERS
  drc.hpp
  LINK_LIBRARIES
  MQT::ScpdGrid
  MQT::ScpdDesign
  MQT::ScpdGeometry)
