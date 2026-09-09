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

/// A rule that pairs a coupler's port1 with its port2.
std::unique_ptr<BridgeRuleT> couplerRule() {
  auto rule = std::make_unique<BridgeRuleT>();
  rule->first = R"(^(Coupler\d+_\d+)\.port1$)";
  rule->second = R"(^(Coupler\d+_\d+)\.port2$)";
  return rule;
}

TEST(ConfigValidation, PatternsMustCompile) {
  EXPECT_TRUE(validate(*benchmarkPatterns()).empty());

  PortPatternsT broken;
  broken.launcher = "(";
  broken.resonator = R"(^Qb\d+\.port0$)";
  const auto problems = validate(broken);
  ASSERT_EQ(problems.size(), 2U);
  EXPECT_EQ(problems[0].substr(0, 33), "launcher pattern does not compile");
  EXPECT_EQ(problems[1], "conventional pattern is empty");
}

TEST(ConfigValidation, ABridgeRuleMustCaptureTheComponentItPairsOn) {
  BridgeRuleT rule;
  rule.first = R"(^(Coupler\d+_\d+)\.port1$)";
  rule.second = R"(^(Coupler\d+_\d+)\.port2$)";
  EXPECT_TRUE(validate(rule).empty());

  BridgeRuleT loose;
  loose.first = R"(^Coupler\d+_\d+\.port1$)";
  loose.second = "(";
  const auto problems = validate(loose);
  ASSERT_EQ(problems.size(), 2U);
  EXPECT_EQ(problems[0],
            "first pattern must have exactly one capture group, not 0");
  EXPECT_EQ(problems[1].substr(0, 31), "second pattern does not compile");
}

TEST(ConfigValidation, ABridgeRuleNeedsTheBridgePatternThatSelectsItsPorts) {
  ConfigT config = manualConfig();
  config.ports->bridge_pairs.push_back(std::make_unique<BridgeRuleT>());
  config.ports->bridge_pairs.back()->first = R"(^(Coupler\d+_\d+)\.port1$)";
  config.ports->bridge_pairs.back()->second = R"(^(Coupler\d+_\d+)\.port2$)";

  EXPECT_EQ(
      validate(*config.ports),
      (Problems{"bridge_pairs are declared without a bridge_pair pattern"}));

  config.ports->patterns->bridge_pair = R"(^Coupler\d+_\d+\.port[1-4]$)";
  EXPECT_TRUE(validate(*config.ports).empty());
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

TEST(ConfigValidation, TheBridgePatternAndTheBridgeRulesMustAgree) {
  // The pattern says which ports are ends of a crossing and the rules say
  // which two of them pair. A port one of them knows and the other does not
  // is a configuration that has drifted apart.
  ChipT chip = smallChip();
  const auto add = [&](const std::string& label, const UnassignedRole role) {
    auto port = std::make_unique<PortT>();
    port->label = label;
    port->center = Point(0.0, 0.0);
    port->role = role;
    chip.ports.push_back(std::move(port));
  };
  add("Coupler1_2.port1", UnassignedRole::BridgePair);
  add("Coupler1_2.port2", UnassignedRole::BridgePair);

  ConfigT config = manualConfig();
  config.ports->patterns->bridge_pair = R"(^Coupler\d+_\d+\.port[1-2]$)";
  config.ports->bridge_pairs.push_back(couplerRule());
  EXPECT_TRUE(validate(config, chip).empty());

  // A rule that reaches a port of another role.
  config.ports->bridge_pairs.back()->second = R"(^(Coupler\d+_\d+)\.port3$)";
  const auto crossed = validate(config, chip);
  ASSERT_EQ(crossed.size(), 2U);
  EXPECT_EQ(crossed[0], "port 'Coupler1_2.port3' is paired by a bridge rule "
                        "but its role is conventional");
  EXPECT_EQ(crossed[1], "port 'Coupler1_2.port2' is a bridge_pair port that no "
                        "bridge rule pairs");

  // No rule at all, and the pattern still naming two ports.
  config.ports->bridge_pairs.clear();
  const auto orphaned = validate(config, chip);
  ASSERT_EQ(orphaned.size(), 2U);
  EXPECT_EQ(orphaned[0], "port 'Coupler1_2.port1' is a bridge_pair port that "
                         "no bridge rule pairs");
}

TEST(ConfigValidation, SequencesMustNameRoutablePortsOfTheChip) {
  const ChipT chip = smallChip();
  EXPECT_TRUE(validate(manualConfig(), chip).empty());
  EXPECT_TRUE(validate(ConfigT(), chip).empty());

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
