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

# The eight module libraries, in dependency order. The dependency graph is documented in
# ARCHITECTURE.md; nothing depends on pipeline, which is the top.
add_subdirectory(geometry)
add_subdirectory(design)
add_subdirectory(grid)
add_subdirectory(routing)
add_subdirectory(milp)
add_subdirectory(drc)
add_subdirectory(io)
add_subdirectory(pipeline)
