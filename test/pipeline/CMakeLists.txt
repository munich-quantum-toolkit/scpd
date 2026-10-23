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
    # Some stage tests read the MPS text of their model back with HiGHS, so they reach HiGHS
    # directly.
    package_add_test(mqt-scpd-pipeline-test
                     "MQT::ScpdPipeline;MQT::ScpdIO;MQT::ScpdDrc;mqt-scpd-highs"
                     ${PIPELINE_TEST_SOURCES})
    # The capacity graph tests read the data the Final stage records for its page.
    target_link_libraries(mqt-scpd-pipeline-test PRIVATE mqt-scpd-json)
    # The stage tests read the two-qubit fixture of the repository.
    # The routing stage tests run on the benchmark inputs the repository carries, and skip
    # without them.
    target_compile_definitions(
      mqt-scpd-pipeline-test
      PRIVATE MQT_SCPD_FIXTURE_DIR="${PROJECT_SOURCE_DIR}/test/fixtures"
              MQT_SCPD_BENCHMARK_DIR="${PROJECT_SOURCE_DIR}/benchmarks")
  endif()
endif()
