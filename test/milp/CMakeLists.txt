# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

if(TARGET MQT::ScpdMilp)
  file(GLOB_RECURSE MILP_TEST_SOURCES *.cpp)
  if(MILP_TEST_SOURCES)
    package_add_test(mqt-scpd-milp-test MQT::ScpdMilp ${MILP_TEST_SOURCES})
  endif()
endif()
