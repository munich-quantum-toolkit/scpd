# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

# Patches a checkout of artem-ogre/CDT for the CDT world: the set of
# constraint edges (`CDT::EdgeUSet`) becomes the flat hash set of
# src/grid/cdt/FlatEdgeSet.h, which a state copies as one array rather than
# as one node per edge. Run as
#   cmake -DCDT_DIR=<checkout> -DSOURCE_DIR=<repository> -P PatchCdt.cmake
# (FetchContent runs it as the PATCH_COMMAND of the download). Patching twice
# is harmless.

if(NOT DEFINED CDT_DIR OR NOT DEFINED SOURCE_DIR)
  message(FATAL_ERROR "PatchCdt.cmake needs CDT_DIR and SOURCE_DIR")
endif()

set(utils "${CDT_DIR}/CDT/include/CDTUtils.h")
if(NOT EXISTS "${utils}")
  message(FATAL_ERROR "PatchCdt.cmake: no ${utils}")
endif()
configure_file("${SOURCE_DIR}/src/grid/cdt/FlatEdgeSet.h"
               "${CDT_DIR}/CDT/include/mqt_scpd_flat_edge_set.h" COPYONLY)

file(READ "${utils}" text)
set(original "typedef unordered_set<Edge> EdgeUSet;")
set(patched "typedef mqt_scpd_cdt::FlatEdgeSet<Edge, VertInd, noVertex> EdgeUSet;")
string(FIND "${text}" "${patched}" already)
if(already EQUAL -1)
  string(FIND "${text}" "${original}" found)
  if(found EQUAL -1)
    message(FATAL_ERROR "PatchCdt.cmake: CDTUtils.h has no '${original}'")
  endif()
  string(REPLACE "${original}" "${patched}" text "${text}")
  # The include, after the header's own includes: before the CDT namespace.
  string(FIND "${text}" "namespace CDT" at)
  if(at EQUAL -1)
    message(FATAL_ERROR "PatchCdt.cmake: CDTUtils.h has no CDT namespace")
  endif()
  string(SUBSTRING "${text}" 0 ${at} head)
  string(SUBSTRING "${text}" ${at} -1 tail)
  set(text "${head}#include \"mqt_scpd_flat_edge_set.h\"\n\n${tail}")
  file(WRITE "${utils}" "${text}")
  message(STATUS "PatchCdt.cmake: patched ${utils}")
else()
  message(STATUS "PatchCdt.cmake: ${utils} already patched")
endif()
