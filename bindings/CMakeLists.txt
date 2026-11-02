# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

if(APPLE)
  set(BASEPOINT @loader_path)
else()
  set(BASEPOINT $ORIGIN)
endif()
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
list(
  APPEND
  CMAKE_INSTALL_RPATH
  ${BASEPOINT}
  ${BASEPOINT}/${CMAKE_INSTALL_LIBDIR}
  ${BASEPOINT}/../core/${CMAKE_INSTALL_LIBDIR}
  ${BASEPOINT}/../core/lib
  ${BASEPOINT}/../core/lib64
  ${BASEPOINT}/../../core/${CMAKE_INSTALL_LIBDIR}
  ${BASEPOINT}/../../core/lib
  ${BASEPOINT}/../../core/lib64)

add_mqt_python_binding(
  SCPD
  ${MQT_SCPD_TARGET_NAME}-bindings
  bindings.cpp
  MODULE_NAME
  pyscpd
  INSTALL_DIR
  .
  LINK_LIBS
  MQT::SCPD)

# The extension links HiGHS, FlatBuffers and nlohmann/json in, and the license of each asks that its
# notice goes with every copy of the binary. HiGHS compiles in its LP file reader, which carries an
# MIT notice of its own. The wheel carries them in its .dist-info/licenses directory, taken from the
# sources that the build fetched. The Boost Software License and the zlib license of pdqsort ask no
# notice for object code.
if(SKBUILD)
  install(
    FILES ${highs_SOURCE_DIR}/LICENSE.txt
    DESTINATION ${SKBUILD_METADATA_DIR}/licenses
    RENAME HiGHS-LICENSE.txt
    COMPONENT ${MQT_SCPD_TARGET_NAME}_Python)
  install(
    FILES ${highs_SOURCE_DIR}/extern/filereaderlp/LICENSE
    DESTINATION ${SKBUILD_METADATA_DIR}/licenses
    RENAME HiGHS-filereaderlp-LICENSE.txt
    COMPONENT ${MQT_SCPD_TARGET_NAME}_Python)
  install(
    FILES ${flatbuffers_SOURCE_DIR}/LICENSE
    DESTINATION ${SKBUILD_METADATA_DIR}/licenses
    RENAME FlatBuffers-LICENSE.txt
    COMPONENT ${MQT_SCPD_TARGET_NAME}_Python)
  install(
    FILES ${nlohmann_json_SOURCE_DIR}/LICENSE.MIT
    DESTINATION ${SKBUILD_METADATA_DIR}/licenses
    RENAME nlohmann_json-LICENSE.txt
    COMPONENT ${MQT_SCPD_TARGET_NAME}_Python)
endif()

# install the Python stub files in editable mode for better IDE support
if(SKBUILD_STATE STREQUAL "editable")
  install(
    FILES ${PROJECT_SOURCE_DIR}/python/mqt/scpd/pyscpd.pyi
    DESTINATION .
    COMPONENT ${MQT_SCPD_TARGET_NAME}_Python)
endif()
