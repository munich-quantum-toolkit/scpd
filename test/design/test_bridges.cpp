/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/design/Bridges.hpp"

#include "mqt-scpd/design/Roles.hpp"
#include "mqt-scpd/flatbuffers/config.hpp"
#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/flatbuffers/geometry.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace {

using namespace mqt::scpd::flatbuffers::design;
using mqt::scpd::design::BridgeRules;
using mqt::scpd::design::bridgingPorts;
using mqt::scpd::design::classifyPorts;
using mqt::scpd::design::componentsOf;
using mqt::scpd::flatbuffers::config::BridgeRuleT;
using mqt::scpd::flatbuffers::config::PortPatternsT;
using mqt::scpd::flatbuffers::geometry::Point;

PortPatternsT patterns() {
  PortPatternsT compiled;
  compiled.launcher = R"(^Chip\.port\d+$)";
  compiled.resonator = R"(^Qb\d+\.port0$)";
  compiled.conventional = R"(^(Qb\d+\.port1|Coupler\d+_\d+\.port0)$)";
  compiled.bridge_pair = R"(^Coupler\d+_\d+\.port[1-4]$)";
  compiled.component = R"(^([^.]+)\.port\d+$)";
  return compiled;
}

/// The rules the benchmark chips declare: port1 pairs with port2, port3 with
/// port4, both within one coupler.
BridgeRules couplerRules() {
  BridgeRules rules;
  for (const auto& [first, second] :
       {std::pair{R"(^(Coupler\d+_\d+)\.port1$)", R"(^(Coupler\d+_\d+)\.port2$)"},
        std::pair{R"(^(Coupler\d+_\d+)\.port3$)", R"(^(Coupler\d+_\d+)\.port4$)"}}) {
    auto rule = std::make_unique<BridgeRuleT>();
    rule->first = first;
    rule->second = second;
    rules.push_back(std::move(rule));
  }
  return rules;
}

/// A chip of the given labels, classified, with every port at the origin.
///
/// The positions are all the same on purpose: the pairing is declared, so no
/// orientation and no distance takes part in it.
ChipT chipWith(const std::vector<std::string>& labels) {
  ChipT chip;
  chip.ports.reserve(labels.size());
  for (const auto& label : labels) {
    auto port = std::make_unique<PortT>();
    port->label = label;
    port->center = Point(0.0, 0.0);
    chip.ports.push_back(std::move(port));
  }
  const auto compiled = patterns();
  EXPECT_TRUE(classifyPorts(chip, compiled).empty());
  return chip;
}

/// The bridges of one component, as label pairs.
std::vector<std::pair<std::string, std::string>> bridgesOf(const ChipT& chip,
                                                           const std::string& name,
                                                           const BridgeRules& rules) {
  std::vector<std::pair<std::string, std::string>> pairs;
  for (const auto& component : componentsOf(chip, rules)) {
    if (component.name != name) {
      continue;
    }
    for (const auto& bridge : component.bridges) {
      pairs.emplace_back(chip.ports[bridge.first]->label, chip.ports[bridge.second]->label);
    }
  }
  return pairs;
}

/// The ends of one component, as labels.
std::vector<std::string> endsOf(const ChipT& chip, const std::string& name,
                                const BridgeRules& rules) {
  std::vector<std::string> labels;
  for (const auto& component : componentsOf(chip, rules)) {
    if (component.name != name) {
      continue;
    }
    for (const auto port : component.ends) {
      labels.push_back(chip.ports[port]->label);
    }
  }
  return labels;
}

TEST(Bridges, ARulePairsThePortsItsCapturesAgreeOn) {
  const ChipT chip = chipWith({"Coupler1_2.port0", "Coupler1_2.port1", "Coupler1_2.port2",
                               "Coupler1_2.port3", "Coupler1_2.port4"});
  const auto rules = couplerRules();

  EXPECT_EQ(bridgesOf(chip, "Coupler1_2", rules),
            (std::vector{std::pair<std::string, std::string>{"Coupler1_2.port1",
                                                             "Coupler1_2.port2"},
                         std::pair<std::string, std::string>{"Coupler1_2.port3",
                                                             "Coupler1_2.port4"}}));
  // The port no rule claims ends a wire. It is the one the inner circuit has
  // to reach.
  EXPECT_EQ(endsOf(chip, "Coupler1_2", rules), (std::vector<std::string>{"Coupler1_2.port0"}));
}

TEST(Bridges, ARuleNeverPairsAcrossComponents) {
  // Both sides capture the component, so two ports of different couplers do
  // not pair however well their labels otherwise match.
  const ChipT chip = chipWith({"Coupler1_2.port1", "Coupler3_4.port2"});
  const auto rules = couplerRules();

  EXPECT_TRUE(bridgesOf(chip, "Coupler1_2", rules).empty());
  EXPECT_TRUE(bridgesOf(chip, "Coupler3_4", rules).empty());
  EXPECT_EQ(endsOf(chip, "Coupler1_2", rules), (std::vector<std::string>{"Coupler1_2.port1"}));
}

TEST(Bridges, APortWhoseOtherSideIsAbsentEndsAWire) {
  // The 4-qubit chip's couplers carry three ports, so only one rule of a
  // two-rule configuration finds both of its sides.
  const ChipT chip =
      chipWith({"Coupler1_2.port0", "Coupler1_2.port1", "Coupler1_2.port2", "Coupler1_2.port3"});
  const auto rules = couplerRules();

  EXPECT_EQ(bridgesOf(chip, "Coupler1_2", rules).size(), 1U);
  EXPECT_EQ(endsOf(chip, "Coupler1_2", rules),
            (std::vector<std::string>{"Coupler1_2.port0", "Coupler1_2.port3"}));
}

TEST(Bridges, AQubitHasNoBridge) {
  // No rule names a qubit's ports, so each of them ends a wire.
  const ChipT chip = chipWith({"Qb1.port0", "Qb1.port1"});
  const auto rules = couplerRules();

  EXPECT_TRUE(bridgesOf(chip, "Qb1", rules).empty());
  EXPECT_EQ(endsOf(chip, "Qb1", rules), (std::vector<std::string>{"Qb1.port0", "Qb1.port1"}));
}

TEST(Bridges, ThePairingDoesNotDependOnTheOrderThePortsWereRead) {
  const ChipT forward = chipWith({"Coupler1_2.port1", "Coupler1_2.port2", "Coupler1_2.port3",
                                  "Coupler1_2.port4"});
  const ChipT backward = chipWith({"Coupler1_2.port4", "Coupler1_2.port3", "Coupler1_2.port2",
                                   "Coupler1_2.port1"});
  const auto rules = couplerRules();

  EXPECT_EQ(bridgesOf(forward, "Coupler1_2", rules), bridgesOf(backward, "Coupler1_2", rules));
}

TEST(Bridges, WithoutRulesNothingBridges) {
  const ChipT chip = chipWith({"Coupler1_2.port1", "Coupler1_2.port2"});
  const BridgeRules none;

  EXPECT_TRUE(bridgesOf(chip, "Coupler1_2", none).empty());
  const auto bridging = bridgingPorts(chip, none);
  EXPECT_TRUE(std::ranges::none_of(bridging, [](const bool value) { return value; }));
}

TEST(Bridges, EveryBridgePortIsMarkedAsOne) {
  const ChipT chip = chipWith({"Coupler1_2.port0", "Coupler1_2.port1", "Coupler1_2.port2"});
  const auto bridging = bridgingPorts(chip, couplerRules());

  EXPECT_FALSE(bridging[0]);
  EXPECT_TRUE(bridging[1]);
  EXPECT_TRUE(bridging[2]);
}

} // namespace
