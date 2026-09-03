# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

if(TARGET MQT::ScpdDesign)
  file(GLOB_RECURSE DESIGN_TEST_SOURCES *.cpp)
  if(DESIGN_TEST_SOURCES)
    package_add_test(mqt-scpd-design-test MQT::ScpdDesign ${DESIGN_TEST_SOURCES})
  endif()
endif()
