# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

include(CMakeDependentOption)
include(FetchContent)
set(FETCH_PACKAGES "")

if(BUILD_MQT_SCPD_BINDINGS)
  execute_process(
    COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
    OUTPUT_STRIP_TRAILING_WHITESPACE
    OUTPUT_VARIABLE nanobind_ROOT)
  find_package(nanobind CONFIG REQUIRED)
endif()

# Match the version in the generated headers. Only schema regeneration builds `flatc`.
option(MQT_SCPD_BUILD_FLATC "Build the FlatBuffers compiler for schema regeneration" OFF)
set(FLATBUFFERS_VERSION
    25.12.19
    CACHE STRING "FlatBuffers version")
set(FLATBUFFERS_URL
    https://github.com/google/flatbuffers/archive/refs/tags/v${FLATBUFFERS_VERSION}.tar.gz)
set(FLATBUFFERS_BUILD_FLATC
    ${MQT_SCPD_BUILD_FLATC}
    CACHE BOOL "" FORCE)
set(FLATBUFFERS_BUILD_FLATLIB
    OFF
    CACHE BOOL "" FORCE)
set(FLATBUFFERS_BUILD_TESTS
    OFF
    CACHE BOOL "" FORCE)
set(FLATBUFFERS_INSTALL
    OFF
    CACHE BOOL "" FORCE)
FetchContent_Declare(flatbuffers URL ${FLATBUFFERS_URL})
list(APPEND FETCH_PACKAGES flatbuffers)

set(NLOHMANN_JSON_VERSION
    3.12.0
    CACHE STRING "nlohmann/json version")
set(NLOHMANN_JSON_URL
    https://github.com/nlohmann/json/releases/download/v${NLOHMANN_JSON_VERSION}/json.tar.xz)
set(JSON_BuildTests
    OFF
    CACHE INTERNAL "")
set(JSON_Install
    OFF
    CACHE INTERNAL "")
FetchContent_Declare(nlohmann_json URL ${NLOHMANN_JSON_URL})
list(APPEND FETCH_PACKAGES nlohmann_json)

# HiGHS versions can choose different layouts with the same objective.
set(HIGHS_VERSION
    1.16.0
    CACHE STRING "HiGHS version")
set(HIGHS_URL https://github.com/ERGO-Code/HiGHS/archive/refs/tags/v${HIGHS_VERSION}.tar.gz)
FetchContent_Declare(highs URL ${HIGHS_URL} EXCLUDE_FROM_ALL)
list(APPEND FETCH_PACKAGES highs)

# SOURCE_SUBDIR skips configuration of the header-only Boost dependencies.
set(BOOST_VERSION
    1.89.0
    CACHE STRING "Boost version")
foreach(library IN ITEMS polygon config)
  FetchContent_Declare(
    boost_${library}
    URL https://github.com/boostorg/${library}/archive/refs/tags/boost-${BOOST_VERSION}.tar.gz
        SOURCE_SUBDIR include EXCLUDE_FROM_ALL)
  list(APPEND FETCH_PACKAGES boost_${library})
endforeach()

if(BUILD_MQT_SCPD_TESTS)
  set(gtest_force_shared_crt
      ON
      CACHE BOOL "" FORCE)
  set(GTEST_VERSION
      1.17.0
      CACHE STRING "Google Test version")
  set(GTEST_URL https://github.com/google/googletest/archive/refs/tags/v${GTEST_VERSION}.tar.gz)
  FetchContent_Declare(googletest URL ${GTEST_URL} FIND_PACKAGE_ARGS ${GTEST_VERSION} NAMES GTest)
  list(APPEND FETCH_PACKAGES googletest)
endif()

# Scope the options to dependency configuration so parent-project settings are preserved.
block(
  SCOPE_FOR
  VARIABLES
  PROPAGATE
  flatbuffers_SOURCE_DIR
  nlohmann_json_SOURCE_DIR
  highs_SOURCE_DIR
  boost_polygon_SOURCE_DIR
  boost_config_SOURCE_DIR)
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
set(CMAKE_POLICY_DEFAULT_CMP0126 NEW)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION OFF)
set(BUILD_TESTING OFF)
set(BUILD_EXAMPLES OFF)
set(BUILD_CXX_EXAMPLE OFF)
set(BUILD_CXX_EXE OFF)
set(HIGHSINT64 OFF)
set(ZLIB OFF)
set(BUILD_SHARED_LIBS OFF)
FetchContent_MakeAvailable(${FETCH_PACKAGES})
endblock()

# HiGHS assertions can stop Debug solves on harmless bound roundoff.
if(TARGET highs)
  target_compile_definitions(highs PRIVATE NDEBUG)
endif()

# System includes keep project warnings out of third-party headers.
if(NOT TARGET mqt-scpd-flatbuffers)
  add_library(mqt-scpd-flatbuffers INTERFACE)
  target_include_directories(mqt-scpd-flatbuffers SYSTEM
                             INTERFACE $<BUILD_INTERFACE:${flatbuffers_SOURCE_DIR}/include>)
endif()

if(NOT TARGET mqt-scpd-json)
  add_library(mqt-scpd-json INTERFACE)
  target_include_directories(mqt-scpd-json SYSTEM
                             INTERFACE $<BUILD_INTERFACE:${nlohmann_json_SOURCE_DIR}/include>)
endif()

if(NOT TARGET mqt-scpd-highs)
  add_library(mqt-scpd-highs INTERFACE)
  target_link_libraries(mqt-scpd-highs INTERFACE highs::highs)
  get_target_property(MQT_SCPD_HIGHS_INCLUDES highs::highs INTERFACE_INCLUDE_DIRECTORIES)
  if(MQT_SCPD_HIGHS_INCLUDES)
    target_include_directories(mqt-scpd-highs SYSTEM INTERFACE ${MQT_SCPD_HIGHS_INCLUDES})
  endif()
endif()

if(NOT TARGET mqt-scpd-polygon)
  add_library(mqt-scpd-polygon INTERFACE)
  target_include_directories(
    mqt-scpd-polygon SYSTEM INTERFACE $<BUILD_INTERFACE:${boost_polygon_SOURCE_DIR}/include>
                                      $<BUILD_INTERFACE:${boost_config_SOURCE_DIR}/include>)
endif()
