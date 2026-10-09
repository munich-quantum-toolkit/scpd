/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#pragma once

// The two-qubit fixture of the repository, as the stages take it. The
// configuration is built here rather than parsed, because the TOML loader is
// Python's; it carries the values of test/fixtures/mini/config.toml.

#include "mqt-scpd/flatbuffers/config.hpp"
#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/io/Chip.hpp"

#include <gtest/gtest.h>

#include <fstream>
#include <iterator>
#include <memory>
#include <string>

namespace mqt::scpd::pipeline::test {

/// The configuration of test/fixtures/mini/config.toml.
inline flatbuffers::config::ConfigT miniConfig() {
  using namespace flatbuffers::config;
  ConfigT config;
  config.chip_input = "routing_config.json";
  config.ports = std::make_unique<PortConfigT>();
  config.ports->patterns = std::make_unique<PortPatternsT>();
  config.ports->patterns->launcher = R"(^Chip\.port\d+$)";
  config.ports->patterns->resonator = R"(^Q\d+\.port0$)";
  config.ports->patterns->conventional = R"(^(Q\d+\.port1|C\d+\.port0)$)";
  config.ports->sequences = std::make_unique<PortSequencesT>();
  config.ports->sequences->all_outer = {"Q1.port0", "Q1.port1", "C12.port0",
                                        "Q2.port1", "Q2.port0"};
  config.ports->sequences->fixed_outer = {"Q1.port0", "Q1.port1", "C12.port0"};
  config.rules = std::make_unique<flatbuffers::design::DesignRulesT>();
  config.rules->min_wire_spacing = 185.0;
  config.rules->min_obstacle_spacing = 25.0;
  config.rules->min_bend_radius = 50.0;
  config.rules->min_straight_length = 100.0;
  config.rules->target_resonator_length = 2500.0;
  config.rules->resonator_length_tolerance = 100.0;
  config.rules->max_feedline_utilization = 4;
  config.rules->feedline_terminations = 1;
  config.grid = std::make_unique<GridParamsT>();
  config.grid->capacity_cells_x = 12;
  config.grid->launcher_offset_x = 0;
  config.grid->launcher_offset_y = 0;
  config.stages = std::make_unique<StageParamsT>();
  config.stages->capacity = std::make_unique<CapacityParamsT>();
  config.stages->global = std::make_unique<GlobalParamsT>();
  config.stages->assignment = std::make_unique<AssignmentParamsT>();
  config.stages->assignment->launcher_target = 1;
  config.stages->solver = std::make_unique<SolverParamsT>();
  config.stages->corridor = std::make_unique<CorridorParamsT>();
  return config;
}

/// The chip input of the fixture, as JSON text.
inline std::string miniChipText() {
  const std::string path =
      std::string(MQT_SCPD_FIXTURE_DIR) + "/mini/routing_config.json";
  std::ifstream file(path);
  EXPECT_TRUE(file.is_open()) << path;
  return {std::istreambuf_iterator<char>(file),
          std::istreambuf_iterator<char>()};
}

/// The chip of the fixture, classified with a configuration.
inline flatbuffers::design::ChipT
miniChip(const flatbuffers::config::ConfigT& config) {
  return io::loadChip(miniChipText(), config);
}

/// The chip of the fixture, classified with its own configuration.
inline flatbuffers::design::ChipT miniChip() { return miniChip(miniConfig()); }

} // namespace mqt::scpd::pipeline::test
