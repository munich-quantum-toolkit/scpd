# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

if(TARGET MQT::ScpdRouting)
  file(GLOB_RECURSE ROUTING_TEST_SOURCES CONFIGURE_DEPENDS *.cpp)
  # The allocation failure tests replace the global operator new, so they build into a binary of
  # their own.
  list(FILTER ROUTING_TEST_SOURCES EXCLUDE REGEX "/test_allocation_failure\\.cpp$")
  if(ROUTING_TEST_SOURCES)
    package_add_test(mqt-scpd-routing-test MQT::ScpdRouting ${ROUTING_TEST_SOURCES})
  endif()
  package_add_test(mqt-scpd-routing-allocation-test MQT::ScpdRouting test_allocation_failure.cpp)
endif()
