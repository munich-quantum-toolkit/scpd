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
# installation without a commercial solver license works in full. Gurobi is reached at run time
# through gurobipy and an MPS round trip, and is never a build dependency. A different HiGHS version
# can return a different optimum among equal ones, so the version is pinned. Only the library is
# built: EXCLUDE_FROM_ALL leaves out the solver executable and the examples. HiGHS compiles with
# -mpopcnt on x86-64 Linux, so the Linux x86-64 wheels need a processor with the POPCNT instruction,
# which every x86-64 processor since about 2008 has.
set(HIGHS_VERSION
    1.11.0
    CACHE STRING "HiGHS version")
set(HIGHS_URL https://github.com/ERGO-Code/HiGHS/archive/refs/tags/v${HIGHS_VERSION}.tar.gz)
FetchContent_Declare(highs URL ${HIGHS_URL} EXCLUDE_FROM_ALL)
list(APPEND FETCH_PACKAGES highs)

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

# Make all declared dependencies available. The options of HiGHS are normal variables that exist
# only inside the block, so these options neither write the cache nor change a value that the user
# or a parent project set. CMP0077 makes option() honor a normal variable, and CMP0126 keeps
# set(CACHE) from removing one. HiGHS still includes CPack, which writes CPACK_* entries to the
# cache and CPackConfig.cmake to the build tree. The project builds no CPack package, so nothing
# reads them.
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
# The tests of the dependencies: HiGHS would otherwise add its test tree and the CTest dashboard
# targets. The project's own tests do not depend on this variable.
set(BUILD_TESTING OFF)
# HiGHS: the library only, with 32-bit indices and without zlib.
set(BUILD_EXAMPLES OFF)
set(BUILD_CXX_EXAMPLE OFF)
set(BUILD_CXX_EXE OFF)
set(HIGHSINT64 OFF)
set(ZLIB OFF)
# HiGHS defaults BUILD_SHARED_LIBS to ON, and a shared library would be one more file for the wheel
# to ship, so it is built static.
set(BUILD_SHARED_LIBS OFF)
FetchContent_MakeAvailable(${FETCH_PACKAGES})
endblock()

# HiGHS is always built without link-time optimization. The project switches it on for Release
# builds only after the dependencies are made available, so a first configure builds HiGHS without
# it and a later configure, which finds the setting in the cache, builds HiGHS with it. The two
# builds round some floating-point results differently, so a solve could depend on how often the
# build directory was configured.
#
# HiGHS is also always built without its assertions, as its own build does for Release. In a Debug
# build an assertion of its bookkeeping, which compares two bounds to 1e-12, stops a solve on a
# rounding difference that it leaves without effect otherwise. No HiGHS header changes the layout of
# a type with NDEBUG, so the project's own code may still be built with assertions.
if(TARGET highs)
  set_target_properties(highs PROPERTIES INTERPROCEDURAL_OPTIMIZATION OFF)
  target_compile_definitions(highs PRIVATE NDEBUG)
endif()

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

# HiGHS keeps its headers in several directories and builds with warnings that the project treats as
# errors, so it is reached through a system include as well.
if(NOT TARGET mqt-scpd-highs)
  add_library(mqt-scpd-highs INTERFACE)
  target_link_libraries(mqt-scpd-highs INTERFACE highs::highs)
  get_target_property(MQT_SCPD_HIGHS_INCLUDES highs::highs INTERFACE_INCLUDE_DIRECTORIES)
  if(MQT_SCPD_HIGHS_INCLUDES)
    target_include_directories(mqt-scpd-highs SYSTEM INTERFACE ${MQT_SCPD_HIGHS_INCLUDES})
  endif()
endif()

# Boost.Polygon and Boost.Config, likewise as system includes.
if(NOT TARGET mqt-scpd-polygon)
  add_library(mqt-scpd-polygon INTERFACE)
  target_include_directories(
    mqt-scpd-polygon SYSTEM INTERFACE $<BUILD_INTERFACE:${boost_polygon_SOURCE_DIR}/include>
                                      $<BUILD_INTERFACE:${boost_config_SOURCE_DIR}/include>)
endif()
