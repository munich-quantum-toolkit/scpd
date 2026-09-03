# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

# Declare one MQT SCPD module library. Adapted from MQT Core's AddMQTCoreLibrary.cmake.
#
# add_mqt_scpd_library(<module> [ALIAS_NAME <name>] [GENERATED_HEADERS <file>...] [LINK_LIBRARIES
# <target>...])
#
# The function creates the target `mqt-scpd-<module>` with the alias `MQT::Scpd<Module>`. It
# collects the public headers from `include/mqt-scpd/<module>/` and the sources from
# `src/<module>/`. The headers form a `FILE_SET HEADERS` whose base directory is `include/`, so a
# header is included as `mqt-scpd/<module>/<Header>.hpp`.
#
# GENERATED_HEADERS names schema-generated headers under `include/mqt-scpd/flatbuffers/` that the
# module owns. They join the header set, and the module links the FlatBuffers runtime.
#
# LINK_LIBRARIES names the modules this module depends on. The dependency direction between the
# modules is documented in ARCHITECTURE.md.
#
# A module without sources becomes an INTERFACE library. The first source file in `src/<module>/`
# turns it into a regular library with an export header; nothing else changes for its users.

function(kebab_to_camel output input)
  string(REPLACE "-" ";" parts "${input}")
  set(result "")
  foreach(part ${parts})
    string(SUBSTRING ${part} 0 1 first)
    string(SUBSTRING ${part} 1 -1 rest)
    string(TOUPPER ${first} first)
    string(APPEND result "${first}${rest}")
  endforeach()
  set(${output}
      "${result}"
      PARENT_SCOPE)
endfunction()

function(add_mqt_scpd_library module)
  cmake_parse_arguments(ARG "" "ALIAS_NAME" "GENERATED_HEADERS;LINK_LIBRARIES" ${ARGN})

  set(name ${MQT_SCPD_TARGET_NAME}-${module})
  if(TARGET ${name})
    return()
  endif()

  if(NOT ARG_ALIAS_NAME)
    kebab_to_camel(ARG_ALIAS_NAME ${module})
  endif()
  set(alias MQT::Scpd${ARG_ALIAS_NAME})

  # collect headers and source files
  file(GLOB_RECURSE headers ${MQT_SCPD_INCLUDE_BUILD_DIR}/mqt-scpd/${module}/*.hpp)
  file(GLOB_RECURSE sources ${PROJECT_SOURCE_DIR}/src/${module}/*.cpp)
  foreach(generated ${ARG_GENERATED_HEADERS})
    set(path ${MQT_SCPD_INCLUDE_BUILD_DIR}/mqt-scpd/flatbuffers/${generated})
    if(NOT EXISTS ${path})
      if(MQT_SCPD_BUILD_FLATC)
        # The schema session configures the project before it generates the headers.
        message(STATUS "${name}: ${path} will be generated")
        continue()
      endif()
      message(FATAL_ERROR "${name}: generated header ${path} does not exist. "
                          "Run `uvx nox -s schemas` to regenerate it.")
    endif()
    list(APPEND headers ${path})
  endforeach()

  if(sources)
    set(scope PUBLIC)
    add_library(${name})
    target_sources(${name} PRIVATE ${sources})
  else()
    set(scope INTERFACE)
    add_library(${name} INTERFACE)
  endif()
  add_library(${alias} ALIAS ${name})

  # add headers using file sets
  target_sources(
    ${name}
    ${scope}
    FILE_SET
    HEADERS
    BASE_DIRS
    ${MQT_SCPD_INCLUDE_BUILD_DIR}
    FILES
    ${headers})

  # Set C++ standard
  target_compile_features(${name} ${scope} cxx_std_20)

  if(ARG_GENERATED_HEADERS)
    target_link_libraries(${name} ${scope} $<BUILD_INTERFACE:mqt-scpd-flatbuffers>)
  endif()

  if(ARG_LINK_LIBRARIES)
    target_link_libraries(${name} ${scope} ${ARG_LINK_LIBRARIES})
  endif()

  set_target_properties(${name} PROPERTIES EXPORT_NAME Scpd${ARG_ALIAS_NAME})

  if(scope STREQUAL "INTERFACE")
    return()
  endif()

  # Add link libraries for warnings and options
  target_link_libraries(${name} PRIVATE MQT::ProjectWarnings MQT::ProjectOptions)

  # Always compile with position-independent code to enable usage in shared libraries
  set_target_properties(${name} PROPERTIES POSITION_INDEPENDENT_CODE ON)

  # generate export header
  string(REPLACE "-" "_" base_name ${name})
  string(TOUPPER ${base_name} static_define)
  include(GenerateExportHeader)
  generate_export_header(
    ${name} BASE_NAME ${base_name} EXPORT_FILE_NAME
    ${CMAKE_CURRENT_BINARY_DIR}/include/mqt-scpd/${module}/${base_name}_export.hpp)
  target_sources(
    ${name} PUBLIC FILE_SET HEADERS BASE_DIRS ${CMAKE_CURRENT_BINARY_DIR}/include FILES
                   ${CMAKE_CURRENT_BINARY_DIR}/include/mqt-scpd/${module}/${base_name}_export.hpp)
  if(NOT BUILD_SHARED_LIBS)
    target_compile_definitions(${name} PUBLIC ${static_define}_STATIC_DEFINE)
  endif()
endfunction()
