# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

if(NOT TARGET MQT::SCPD)
  # collect headers and source files
  file(GLOB_RECURSE SCPD_HEADERS ${MQT_SCPD_INCLUDE_BUILD_DIR}/*.hpp)
  file(GLOB_RECURSE SCPD_SOURCES **.cpp)

  # add SCPD Package library
  add_library(${PROJECT_NAME} ${SCPD_HEADERS} ${SCPD_SOURCES})

  # set include directories
  target_include_directories(${PROJECT_NAME}
                             PUBLIC $<BUILD_INTERFACE:${MQT_SCPD_INCLUDE_BUILD_DIR}>)

  # add MQT alias
  add_library(MQT::SCPD ALIAS ${PROJECT_NAME})
endif()
