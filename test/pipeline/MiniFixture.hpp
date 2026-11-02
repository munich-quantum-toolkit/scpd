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
#include <utility>

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

/// The fixture with Q2.port0 off the ring, and two more ports on the coupler
/// bar that a bridge rule pairs: C12.port1 on top, near the inner port, and
/// C12.port2 below it, on the ring.
inline flatbuffers::config::ConfigT bridgedConfig() {
  auto config = miniConfig();
  config.ports->patterns->component = R"(^([^.]+)\.port\d+$)";
  config.ports->patterns->bridge_pair = R"(^C\d+\.port[12]$)";
  auto rule = std::make_unique<flatbuffers::config::BridgeRuleT>();
  rule->first = R"(^(C\d+)\.port1$)";
  rule->second = R"(^(C\d+)\.port2$)";
  config.ports->bridge_pairs.push_back(std::move(rule));
  config.ports->sequences->all_outer = {"Q1.port0", "Q1.port1", "C12.port2",
                                        "C12.port0", "Q2.port1"};
  return config;
}

/// The chip of the bridged fixture, classified with a configuration.
inline flatbuffers::design::ChipT
bridgedChip(const flatbuffers::config::ConfigT& config = bridgedConfig()) {
  auto text = miniChipText();
  const std::string anchor = R"("ports": {)";
  const auto at = text.find(anchor);
  EXPECT_NE(at, std::string::npos);
  text.insert(at + anchor.size(), R"(
    "C12.port1": {"center": [900.0, 270.0], "orientation": 90.0},
    "C12.port2": {"center": [900.0, 130.0], "orientation": 270.0},)");
  return io::loadChip(text, config);
}

} // namespace mqt::scpd::pipeline::test
