# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

if(TARGET MQT::ScpdRouting)
  file(GLOB_RECURSE ROUTING_TEST_SOURCES CONFIGURE_DEPENDS *.cpp)
  if(ROUTING_TEST_SOURCES)
    package_add_test(mqt-scpd-routing-test MQT::ScpdRouting ${ROUTING_TEST_SOURCES})
  endif()
endif()
