# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

if(TARGET MQT::ScpdIO)
  file(GLOB_RECURSE IO_TEST_SOURCES CONFIGURE_DEPENDS *.cpp)
  if(IO_TEST_SOURCES)
    package_add_test(mqt-scpd-io-test MQT::ScpdIO ${IO_TEST_SOURCES})
    # The fixture tests read the two benchmark inputs the repository carries.
    target_compile_definitions(mqt-scpd-io-test
                               PRIVATE MQT_SCPD_BENCHMARK_DIR="${PROJECT_SOURCE_DIR}/benchmarks")
  endif()
endif()
