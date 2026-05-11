# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

# set required cmake version
cmake_minimum_required(VERSION 3.24...4.3)

project(
  mqt-scpd
  LANGUAGES C CXX
  DESCRIPTION "MQT SCPD: Super Conducting Physical Design")

option(BUILD_MQT_SCPD_BINDINGS "Build the MQT SCPD Python bindings" OFF)
if(BUILD_MQT_SCPD_BINDINGS)
  # ensure that the BINDINGS option is set
  set(BINDINGS
      ON
      CACHE INTERNAL "Enable settings related to Python bindings")
  # Some common settings for finding Python
  set(Python_FIND_VIRTUALENV
      FIRST
      CACHE STRING "Give precedence to virtualenvs when searching for Python")
  set(Python_FIND_FRAMEWORK
      LAST
      CACHE STRING "Prefer Brew/Conda to Apple framework Python")
  set(Python_ARTIFACTS_INTERACTIVE
      ON
      CACHE BOOL "Prevent multiple searches for Python and instead cache the results.")

  if(DISABLE_GIL)
    message(STATUS "Disabling Python GIL")
    add_compile_definitions(Py_GIL_DISABLED)
  endif()

  # top-level call to find Python
  find_package(Python 3.10 REQUIRED COMPONENTS Interpreter Development.Module
                                               ${SKBUILD_SABI_COMPONENT})
endif()

option(BUILD_MQT_SCPD_TESTS "Also build tests for the MQT SCPD project" ${PROJECT_IS_TOP_LEVEL})

# set the include directory for the build tree
set(MQT_SCPD_INCLUDE_BUILD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")

# set prefix for all MQT SCPD targets
set(MQT_SCPD_TARGET_NAME mqt-scpd)

include(cmake/AddMQTPythonBinding.cmake)
include(cmake/ExternalDependencies.cmake)
include(cmake/PackageAddTest.cmake)
include(cmake/StandardProjectSettings.cmake)

# add main library code
add_subdirectory(src)

# add bindings code if enabled
if(BUILD_MQT_SCPD_BINDINGS)
  add_subdirectory(bindings)
endif()

# add test code
if(BUILD_MQT_SCPD_TESTS)
  enable_testing()
  include(GoogleTest)
  add_subdirectory(test)
endif()
