# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

# The stage artifacts belong to MQT::ScpdIO, which owns artifacts.fbs; the pipeline reaches them
# through that module rather than claiming the generated header a second time.
add_mqt_scpd_library(
  pipeline
  LINK_LIBRARIES
  MQT::ScpdDesign
  MQT::ScpdGeometry
  MQT::ScpdGrid
  MQT::ScpdIO
  MQT::ScpdMilp
  MQT::ScpdRouting)
