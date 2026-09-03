# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

if(TARGET MQT::ScpdPipeline)
  file(GLOB_RECURSE PIPELINE_TEST_SOURCES *.cpp)
  if(PIPELINE_TEST_SOURCES)
    package_add_test(mqt-scpd-pipeline-test MQT::ScpdPipeline ${PIPELINE_TEST_SOURCES})
  endif()
endif()
