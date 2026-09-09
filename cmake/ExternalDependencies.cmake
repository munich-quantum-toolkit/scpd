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

# HiGHS solves the mixed-integer programs of the planning stages. It is linked in, so an
# installation without a commercial solver licence is fully functional. Gurobi is reached at run
# time through gurobipy over an MPS round trip and is never a build dependency.
set(HIGHS_VERSION
    1.11.0
    CACHE STRING "HiGHS version")
set(HIGHS_URL https://github.com/ERGO-Code/HiGHS/archive/refs/tags/v${HIGHS_VERSION}.tar.gz)
set(BUILD_EXAMPLES
    OFF
    CACHE BOOL "" FORCE)
set(HIGHSINT64
    OFF
    CACHE BOOL "" FORCE)
set(ZLIB
    OFF
    CACHE BOOL "" FORCE)
set(BUILD_CXX_EXAMPLE
    OFF
    CACHE BOOL "" FORCE)
FetchContent_Declare(highs URL ${HIGHS_URL} DOWNLOAD_EXTRACT_TIMESTAMP ON)
list(APPEND FETCH_PACKAGES highs)

# Boost.Polygon supplies the one Voronoi construction of the capacity stage. Only that library is
# configured, and a user-installed Boost is never required. Boost's own CMake declares
# BUILD_SHARED_LIBS as a cache option, which would otherwise turn every target of this project and
# of HiGHS into a shared library and leave the wheel with a runtime dependency to ship.
set(BUILD_SHARED_LIBS
    OFF
    CACHE BOOL "" FORCE)
set(BOOST_VERSION
    1.89.0
    CACHE STRING "Boost version")
set(BOOST_URL
    https://github.com/boostorg/boost/releases/download/boost-${BOOST_VERSION}/boost-${BOOST_VERSION}-cmake.tar.xz
)
set(BOOST_INCLUDE_LIBRARIES
    polygon
    CACHE STRING "" FORCE)
set(BOOST_ENABLE_CMAKE
    ON
    CACHE BOOL "" FORCE)
FetchContent_Declare(Boost URL ${BOOST_URL} DOWNLOAD_EXTRACT_TIMESTAMP ON)
list(APPEND FETCH_PACKAGES Boost)

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

# HiGHS ships its headers across several directories and builds with warnings the project treats as
# errors, so it is reached through a system include as well.
if(NOT TARGET mqt-scpd-highs)
  add_library(mqt-scpd-highs INTERFACE)
  target_link_libraries(mqt-scpd-highs INTERFACE highs::highs)
  get_target_property(MQT_SCPD_HIGHS_INCLUDES highs::highs INTERFACE_INCLUDE_DIRECTORIES)
  if(MQT_SCPD_HIGHS_INCLUDES)
    target_include_directories(mqt-scpd-highs SYSTEM INTERFACE ${MQT_SCPD_HIGHS_INCLUDES})
  endif()
endif()

# Boost.Polygon, likewise as a system include.
if(NOT TARGET mqt-scpd-polygon)
  add_library(mqt-scpd-polygon INTERFACE)
  target_include_directories(mqt-scpd-polygon SYSTEM
                             INTERFACE $<BUILD_INTERFACE:${boost_polygon_SOURCE_DIR}/include>)
  target_link_libraries(mqt-scpd-polygon INTERFACE Boost::polygon)
endif()
