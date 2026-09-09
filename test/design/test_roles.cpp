/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/design/Roles.hpp"
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

using namespace mqt::scpd::flatbuffers::design;
using mqt::scpd::design::classifyPorts;
using mqt::scpd::design::isRoutable;
using mqt::scpd::design::Problems;
using mqt::scpd::design::roleName;
using mqt::scpd::flatbuffers::config::PortPatternsT;
using mqt::scpd::flatbuffers::geometry::Point;

PortPatternsT benchmarkPatterns() {
  PortPatternsT patterns;
  patterns.launcher = R"(^Chip\.port\d+$)";
  patterns.resonator = R"(^Qb\d+\.port0$)";
  patterns.conventional = R"(^(Qb\d+\.port1|Coupler\d+_\d+\.port[0-4])$)";
  patterns.mating = R"(^(Qb\d+\.port[2-5]|Coupler\d+_\d+\.port[5-6])$)";
  return patterns;
}

ChipT chipWith(const std::vector<std::string>& labels) {
  ChipT chip;
  chip.ports.reserve(labels.size());
  for (const auto& label : labels) {
    auto port = std::make_unique<PortT>();
    port->label = label;
    port->center = Point(0.0, 0.0);
    chip.ports.push_back(std::move(port));
  }
  return chip;
}

std::vector<UnassignedRole> rolesOf(const ChipT& chip) {
  std::vector<UnassignedRole> roles;
  roles.reserve(chip.ports.size());
  for (const auto& port : chip.ports) {
    roles.push_back(port->role);
  }
  return roles;
}

TEST(Roles, NamesFollowTheConfigurationKeys) {
  EXPECT_EQ(roleName(UnassignedRole::Launcher), "launcher");
  EXPECT_EQ(roleName(UnassignedRole::Resonator), "resonator");
  EXPECT_EQ(roleName(UnassignedRole::Conventional), "conventional");
  EXPECT_EQ(roleName(UnassignedRole::Coupler), "coupler");
  EXPECT_EQ(roleName(UnassignedRole::Mating), "mating");
  EXPECT_EQ(roleName(UnassignedRole::Unset), "unset");

  EXPECT_TRUE(isRoutable(UnassignedRole::Resonator));
  EXPECT_TRUE(isRoutable(UnassignedRole::Conventional));
  EXPECT_FALSE(isRoutable(UnassignedRole::Launcher));
  EXPECT_FALSE(isRoutable(UnassignedRole::Mating));
  EXPECT_FALSE(isRoutable(UnassignedRole::Unset));
}

TEST(Roles, EveryPortTakesTheRoleOfItsOnePattern) {
  ChipT chip = chipWith({"Chip.port0", "Qb1.port0", "Qb1.port1",
                         "Coupler1_2.port3", "Qb1.port2", "Coupler1_2.port6"});

  EXPECT_TRUE(classifyPorts(chip, benchmarkPatterns()).empty());
  EXPECT_EQ(rolesOf(chip),
            (std::vector{UnassignedRole::Launcher, UnassignedRole::Resonator,
                         UnassignedRole::Conventional,
                         UnassignedRole::Conventional, UnassignedRole::Mating,
                         UnassignedRole::Mating}));
}

TEST(Roles, AnUnmatchedPortIsAProblemAndStaysUnset) {
  ChipT chip = chipWith({"Qb1.port0", "Qb1.port2", "Feedline.port0"});
  PortPatternsT patterns = benchmarkPatterns();
  patterns.mating.clear();

  EXPECT_EQ(classifyPorts(chip, patterns),
            (Problems{"port 'Qb1.port2' matches no role pattern",
                      "port 'Feedline.port0' matches no role pattern"}));
  EXPECT_EQ(rolesOf(chip),
            (std::vector{UnassignedRole::Resonator, UnassignedRole::Unset,
                         UnassignedRole::Unset}));
}

TEST(Roles, APortMatchingSeveralPatternsIsAProblemNamingThem) {
  ChipT chip = chipWith({"Qb1.port0"});
  PortPatternsT patterns = benchmarkPatterns();
  patterns.conventional = R"(^Qb\d+\.port\d$)";

  EXPECT_EQ(classifyPorts(chip, patterns),
            (Problems{"port 'Qb1.port0' matches more than one role pattern: "
                      "resonator, conventional"}));
  EXPECT_EQ(chip.ports[0]->role, UnassignedRole::Unset);
}

TEST(Roles, BrokenPatternsAreReportedBeforeAnythingIsClassified) {
  ChipT chip = chipWith({"Chip.port0"});
  PortPatternsT patterns = benchmarkPatterns();
  patterns.resonator = "(";
  patterns.mating.clear();
  patterns.conventional.clear();

  const auto problems = classifyPorts(chip, patterns);
  ASSERT_EQ(problems.size(), 2U);
  EXPECT_EQ(problems[0].substr(0, 34), "resonator pattern does not compile");
  EXPECT_EQ(problems[1], "conventional pattern is empty");
  EXPECT_EQ(chip.ports[0]->role, UnassignedRole::Unset);
}

} // namespace
