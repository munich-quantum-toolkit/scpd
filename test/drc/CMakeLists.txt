# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

if(TARGET MQT::ScpdDrc)
  file(GLOB_RECURSE DRC_TEST_SOURCES CONFIGURE_DEPENDS *.cpp)
  if(DRC_TEST_SOURCES)
    package_add_test(mqt-scpd-drc-test MQT::ScpdDrc ${DRC_TEST_SOURCES})
  endif()
endif()
