# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

if(TARGET MQT::ScpdGrid)
  file(GLOB_RECURSE GRID_TEST_SOURCES CONFIGURE_DEPENDS *.cpp)
  if(GRID_TEST_SOURCES)
    package_add_test(mqt-scpd-grid-test MQT::ScpdGrid ${GRID_TEST_SOURCES})
  endif()
endif()
