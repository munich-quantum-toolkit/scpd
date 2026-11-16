# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

# Declare all external dependencies and make sure that they are available.

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

# FlatBuffers provides the schema-generated data model. The core links only the header-only runtime.
# The compiler `flatc` is built on request by `uvx nox -s schemas`, which regenerates the committed
# code under include/mqt-scpd/flatbuffers/ and python/mqt/scpd/flatbuffers/. It is never part of a
# regular or a wheel build. The version must match the one that generated the committed code, which
# the generated headers assert at compile time.
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

# nlohmann/json reads the chip input and, later, writes the metrics and the log. Only the
# header-only library is used.
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

# Boost.Polygon supplies the Voronoi diagram of the capacity stage. It is header-only and needs only
# Boost.Config, so the build fetches these two libraries of one Boost release, about 5 MB, and not
# the whole distribution of about 100 MB. Neither is configured: SOURCE_SUBDIR names a directory
# without a CMakeLists.txt, so FetchContent only unpacks them. A Boost installed on the system is
# never used.
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

# Make all declared dependencies available.
FetchContent_MakeAvailable(${FETCH_PACKAGES})

# The header-only FlatBuffers runtime, as a system include so that the project's warnings do not
# apply to it.
if(NOT TARGET mqt-scpd-flatbuffers)
  add_library(mqt-scpd-flatbuffers INTERFACE)
  target_include_directories(mqt-scpd-flatbuffers SYSTEM
                             INTERFACE $<BUILD_INTERFACE:${flatbuffers_SOURCE_DIR}/include>)
endif()

# The header-only nlohmann/json library, likewise as a system include.
if(NOT TARGET mqt-scpd-json)
  add_library(mqt-scpd-json INTERFACE)
  target_include_directories(mqt-scpd-json SYSTEM
                             INTERFACE $<BUILD_INTERFACE:${nlohmann_json_SOURCE_DIR}/include>)
endif()

# Boost.Polygon and Boost.Config, likewise as system includes.
if(NOT TARGET mqt-scpd-polygon)
  add_library(mqt-scpd-polygon INTERFACE)
  target_include_directories(
    mqt-scpd-polygon SYSTEM INTERFACE $<BUILD_INTERFACE:${boost_polygon_SOURCE_DIR}/include>
                                      $<BUILD_INTERFACE:${boost_config_SOURCE_DIR}/include>)
endif()
