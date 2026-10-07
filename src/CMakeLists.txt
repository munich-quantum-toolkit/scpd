# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

if(NOT TARGET MQT::ProjectWarnings)
  # use the warnings specified in CompilerWarnings.cmake
  add_library(project_warnings INTERFACE)

  # standard compiler warnings
  include(${PROJECT_SOURCE_DIR}/cmake/CompilerWarnings.cmake)
  set_project_warnings(project_warnings)

  # add MQT alias
  add_library(MQT::ProjectWarnings ALIAS project_warnings)
  set_target_properties(project_warnings PROPERTIES EXPORT_NAME ProjectWarnings)
endif()

if(NOT TARGET MQT::ProjectOptions)
  # use the options specified in CompilerOptions.cmake
  add_library(project_options INTERFACE)

  # standard compiler options
  include(${PROJECT_SOURCE_DIR}/cmake/CompilerOptions.cmake)
  enable_project_options(project_options)

  # add MQT alias
  add_library(MQT::ProjectOptions ALIAS project_options)
  set_target_properties(project_options PROPERTIES EXPORT_NAME ProjectOptions)
endif()

# Floating-point contraction stays off in every target that MQT SCPD compiles. A parent project may
# define MQT::ProjectOptions first, so the option comes from the target mqt-scpd-fp-contract-off,
# which does not depend on the guard above. The module libraries, the tests and the Python bindings
# link mqt-scpd-fp-contract-off PRIVATE, so the option does not reach their users.
#
# Contraction turns a * b + c into one fused multiply-add, which rounds once instead of twice.
# Without contraction, +, -, *, / and std::sqrt round the same on every x86-64 and arm64 target, so
# exact ties such as those in the obstacle mask do not depend on the compiler or the target. The
# option does not cover the math library: std::hypot, std::atan2, std::sin, std::cos and the like
# may differ in the last bit between C runtimes.
if(NOT TARGET mqt-scpd-fp-contract-off)
  add_library(mqt-scpd-fp-contract-off INTERFACE)
  if(NOT MSVC)
    # GCC, Clang and every other compiler that takes GCC options
    target_compile_options(mqt-scpd-fp-contract-off INTERFACE -ffp-contract=off)
  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    # clang-cl contracts by default, as clang does, and takes the clang option through /clang:.
    target_compile_options(mqt-scpd-fp-contract-off INTERFACE /clang:-ffp-contract=off)
  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC" AND MSVC_VERSION LESS 1930)
    # cl.exe turns contraction off with /fp:strict or with `#pragma fp_contract(off)`. Since Visual
    # Studio 2022 (MSVC 19.30), its default /fp:precise does not contract on any target, arm64
    # included, so the build needs neither. An older cl.exe can contract under /fp:precise. The
    # build does not add /fp:strict, which also changes the handling of floating-point exceptions
    # and costs speed.
    message(
      WARNING
        "MSVC ${CMAKE_CXX_COMPILER_VERSION} is older than Visual Studio 2022 (MSVC 19.30), and "
        "its /fp:precise can contract a * b + c into a fused multiply-add. The results of MQT SCPD "
        "can then differ in the last bit from those of other platforms. "
        "Use Visual Studio 2022 or newer.")
  endif()
endif()

# The eight module libraries, in dependency order. Nothing depends on pipeline, which is the top.
add_subdirectory(geometry)
add_subdirectory(design)
add_subdirectory(grid)
add_subdirectory(routing)
add_subdirectory(milp)
add_subdirectory(drc)
add_subdirectory(io)
add_subdirectory(pipeline)

# The whole core as one target, for the bindings and for a project that embeds MQT SCPD without
# picking modules.
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
