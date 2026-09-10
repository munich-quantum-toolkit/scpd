# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

if(TARGET MQT::ScpdPipeline)
  file(GLOB_RECURSE PIPELINE_TEST_SOURCES CONFIGURE_DEPENDS *.cpp)
  if(PIPELINE_TEST_SOURCES)
    package_add_test(mqt-scpd-pipeline-test "MQT::ScpdPipeline;MQT::ScpdIO;MQT::ScpdDrc"
                     ${PIPELINE_TEST_SOURCES})
    # The stage tests run on the benchmark inputs the repository carries.
    target_compile_definitions(mqt-scpd-pipeline-test
                               PRIVATE MQT_SCPD_BENCHMARK_DIR="${PROJECT_SOURCE_DIR}/benchmarks")
  endif()
endif()
