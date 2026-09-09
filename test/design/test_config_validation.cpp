/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/design/Validation.hpp"
#include "mqt-scpd/flatbuffers/config.hpp"
#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/flatbuffers/geometry.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace {

using namespace mqt::scpd::flatbuffers::config;
using mqt::scpd::design::Problems;
using mqt::scpd::design::validate;
using mqt::scpd::flatbuffers::design::ChipT;
using mqt::scpd::flatbuffers::design::DesignRulesT;
using mqt::scpd::flatbuffers::design::PortT;
using mqt::scpd::flatbuffers::design::UnassignedRole;
using mqt::scpd::flatbuffers::geometry::Point;

std::unique_ptr<PortPatternsT> benchmarkPatterns() {
  auto patterns = std::make_unique<PortPatternsT>();
  patterns->launcher = R"(^Chip\.port\d+$)";
  patterns->resonator = R"(^Qb\d+\.port0$)";
  patterns->conventional = R"(^(Qb\d+\.port1|Coupler\d+_\d+\.port[0-4])$)";
  return patterns;
}

std::unique_ptr<DesignRulesT> benchmarkRules() {
  auto rules = std::make_unique<DesignRulesT>();
  rules->min_wire_spacing = 185.0;
  rules->min_obstacle_spacing = 25.0;
  rules->min_bend_radius = 50.0;
  rules->min_straight_length = 100.0;
  rules->target_resonator_length = 2500.0;
  rules->resonator_length_tolerance = 100.0;
  rules->max_feedline_utilization = 6;
  rules->feedline_terminations = 0;
  return rules;
}

ConfigT manualConfig() {
  ConfigT config;
  config.chip_input = "routing_config.json";
  config.ports = std::make_unique<PortConfigT>();
  config.ports->patterns = benchmarkPatterns();
  config.ports->sequences = std::make_unique<PortSequencesT>();
  config.ports->sequences->all_outer = {"Qb1.port0", "Qb1.port1",
                                        "Coupler1_2.port3"};
  config.ports->sequences->fixed_outer = {"Qb1.port0"};
  config.rules = benchmarkRules();
  return config;
}

ChipT smallChip() {
  ChipT chip;
  const auto add = [&](const std::string& label, const UnassignedRole role) {
    auto port = std::make_unique<PortT>();
    port->label = label;
    port->center = Point(0.0, 0.0);
    port->role = role;
    chip.ports.push_back(std::move(port));
  };
  add("Chip.port0", UnassignedRole::Launcher);
  add("Qb1.port0", UnassignedRole::Resonator);
  add("Qb1.port1", UnassignedRole::Conventional);
  add("Coupler1_2.port3", UnassignedRole::Conventional);
  return chip;
}

TEST(ConfigValidation, PatternsMustCompile) {
  EXPECT_TRUE(validate(*benchmarkPatterns()).empty());

  PortPatternsT broken;
  broken.launcher = "(";
  broken.resonator = R"(^Qb\d+\.port0$)";
  broken.mating = "[";
  const auto problems = validate(broken);
  ASSERT_EQ(problems.size(), 3U);
  EXPECT_EQ(problems[0].substr(0, 33), "launcher pattern does not compile");
  EXPECT_EQ(problems[1], "conventional pattern is empty");
  EXPECT_EQ(problems[2].substr(0, 31), "mating pattern does not compile");
}

TEST(ConfigValidation, ThePortSectionNeedsPatternsAndSequences) {
  ConfigT config = manualConfig();
  EXPECT_TRUE(validate(*config.ports).empty());

  config.ports->sequences.reset();
  EXPECT_EQ(validate(*config.ports),
            (Problems{"[ports.sequences] is missing"}));

  config.ports->patterns.reset();
  EXPECT_EQ(validate(*config.ports),
            (Problems{"patterns are missing", "[ports.sequences] is missing"}));
}

TEST(ConfigValidation, AConfigurationNeedsEverySection) {
  EXPECT_TRUE(validate(manualConfig()).empty());

  ConfigT config;
  config.grid = std::make_unique<GridParamsT>();
  config.grid->capacity_cells_x = 0;
  EXPECT_EQ(validate(config),
            (Problems{"chip_input is empty", "ports section is missing",
                      "design rules are missing",
                      "grid: capacity_cells_x must be at least one"}));

  ConfigT nested = manualConfig();
  nested.rules->min_bend_radius = 0.0;
  nested.ports->patterns->launcher.clear();
  EXPECT_EQ(validate(nested),
            (Problems{"ports: patterns: launcher pattern is empty",
                      "rules: min_bend_radius must be positive"}));
}

TEST(ConfigValidation, SequencesMustNameRoutablePortsOfTheChip) {
  const ChipT chip = smallChip();
  EXPECT_TRUE(validate(manualConfig(), chip).empty());

  ConfigT config = manualConfig();
  config.ports->sequences->all_outer = {"Qb1.port0", "Qb9.port0", "Chip.port0",
                                        "Qb1.port0"};
  config.ports->sequences->fixed_outer = {"Qb1.port1"};
  EXPECT_EQ(validate(config, chip),
            (Problems{"all_outer[1]: 'Qb9.port0' is not a port of the chip",
                      "all_outer[2]: 'Chip.port0' is a launcher port, not a "
                      "routable one",
                      "all_outer[3]: 'Qb1.port0' appears twice; first at 0",
                      "fixed_outer[0]: 'Qb1.port1' is not in all_outer"}));
}

} // namespace
