# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

if(NOT TARGET MQT::ProjectWarnings)
  add_library(project_warnings INTERFACE)

  include(${PROJECT_SOURCE_DIR}/cmake/CompilerWarnings.cmake)
  set_project_warnings(project_warnings)

  add_library(MQT::ProjectWarnings ALIAS project_warnings)
  set_target_properties(project_warnings PROPERTIES EXPORT_NAME ProjectWarnings)
endif()

if(NOT TARGET MQT::ProjectOptions)
  add_library(project_options INTERFACE)

  include(${PROJECT_SOURCE_DIR}/cmake/CompilerOptions.cmake)
  enable_project_options(project_options)

  add_library(MQT::ProjectOptions ALIAS project_options)
  set_target_properties(project_options PROPERTIES EXPORT_NAME ProjectOptions)
endif()

# Disable contraction to preserve exact ties in geometry calculations. Keep this option private and
# independent of any MQT::ProjectOptions a parent project defines.
if(NOT TARGET mqt-scpd-fp-contract-off)
  add_library(mqt-scpd-fp-contract-off INTERFACE)
  if(NOT MSVC)
    target_compile_options(mqt-scpd-fp-contract-off INTERFACE -ffp-contract=off)
  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(mqt-scpd-fp-contract-off INTERFACE /clang:-ffp-contract=off)
  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC" AND MSVC_VERSION LESS 1930)
    # Avoid /fp:strict because it also changes floating-point exception handling.
    message(
      WARNING
        "MSVC ${CMAKE_CXX_COMPILER_VERSION} is older than Visual Studio 2022 (MSVC 19.30), and "
        "its /fp:precise can contract a * b + c into a fused multiply-add. The results of MQT SCPD "
        "can then differ in the last bit from those of other platforms. "
        "Use Visual Studio 2022 or newer.")
  endif()
endif()

# Add modules in dependency order.
add_subdirectory(geometry)
add_subdirectory(design)
add_subdirectory(grid)
add_subdirectory(routing)
add_subdirectory(milp)
add_subdirectory(drc)
add_subdirectory(io)
add_subdirectory(pipeline)

if(NOT TARGET MQT::SCPD)
  add_library(${MQT_SCPD_TARGET_NAME} INTERFACE)
  target_link_libraries(
    ${MQT_SCPD_TARGET_NAME}
    INTERFACE MQT::ScpdGeometry
              MQT::ScpdDesign
              MQT::ScpdGrid
              MQT::ScpdRouting
              MQT::ScpdMilp
              MQT::ScpdDrc
              MQT::ScpdIO
              MQT::ScpdPipeline)
  add_library(MQT::SCPD ALIAS ${MQT_SCPD_TARGET_NAME})
  set_target_properties(${MQT_SCPD_TARGET_NAME} PROPERTIES EXPORT_NAME SCPD)
endif()
