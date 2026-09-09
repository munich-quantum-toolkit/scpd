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
    package_add_test(mqt-scpd-grid-test "MQT::ScpdGrid;MQT::ScpdIO" ${GRID_TEST_SOURCES})
    # The benchmark test rasterizes the largest chip input when it is present.
    target_compile_definitions(mqt-scpd-grid-test
                               PRIVATE MQT_SCPD_BENCHMARK_DIR="${PROJECT_SOURCE_DIR}/benchmarks")
  endif()
endif()
