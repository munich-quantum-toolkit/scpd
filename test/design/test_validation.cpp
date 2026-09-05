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
#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/flatbuffers/geometry.hpp"

#include <flatbuffers/buffer.h>
#include <flatbuffers/flatbuffer_builder.h>
#include <flatbuffers/verifier.h>
#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

namespace {

using namespace mqt::scpd::flatbuffers::design;
using mqt::scpd::design::Problems;
using mqt::scpd::design::validate;
using mqt::scpd::flatbuffers::geometry::Point;
using mqt::scpd::flatbuffers::geometry::PolygonT;

DesignRulesT benchmarkRules() {
  DesignRulesT rules;
  rules.min_wire_spacing = 185.0;
  rules.min_obstacle_spacing = 25.0;
  rules.min_bend_radius = 50.0;
  rules.min_straight_length = 100.0;
  rules.target_resonator_length = 2500.0;
  rules.resonator_length_tolerance = 100.0;
  rules.max_feedline_utilization = 6;
  rules.feedline_terminations = 0;
  return rules;
}

std::unique_ptr<PortT> makePort(const std::string& label,
                                const UnassignedRole role) {
  auto port = std::make_unique<PortT>();
  port->label = label;
  port->center = Point(0.0, 0.0);
  port->role = role;
  return port;
}

TEST(DesignValidation, AnAbsentRoleReadsAsUnsetAndFailsValidation) {
  // The verifier accepts a connection without role fields; the roles then
  // read as Unset, which validation reports instead of treating them as the
  // first real member.
  flatbuffers::FlatBufferBuilder builder;
  const PortRef target(3);
  const auto table = builder.StartTable();
  builder.AddStruct(Connection::VT_TARGET, &target);
  builder.Finish(flatbuffers::Offset<Connection>(builder.EndTable(table)));

  flatbuffers::Verifier verifier(builder.GetBufferPointer(), builder.GetSize());
  ASSERT_TRUE(verifier.VerifyBuffer<Connection>(nullptr));

  ConnectionT connection;
  flatbuffers::GetRoot<Connection>(builder.GetBufferPointer())
      ->UnPackTo(&connection);
  EXPECT_EQ(connection.source_role, AssignedRole::Unset);
  EXPECT_EQ(connection.target_role, AssignedRole::Unset);
  EXPECT_EQ(validate(connection),
            (Problems{"source role is unset", "target role is unset"}));

  connection.source_role = AssignedRole::ResonatorSource;
  connection.target_role = AssignedRole::ResonatorTarget;
  EXPECT_TRUE(validate(connection).empty());
}

TEST(DesignValidation, DesignRulesMustBePositiveLengths) {
  EXPECT_TRUE(validate(benchmarkRules()).empty());

  const DesignRulesT absent;
  EXPECT_EQ(validate(absent),
            (Problems{"min_wire_spacing must be positive",
                      "min_obstacle_spacing must be positive",
                      "min_bend_radius must be positive",
                      "min_straight_length must be positive",
                      "target_resonator_length must be positive",
                      "resonator_length_tolerance must be positive",
                      "max_feedline_utilization must be at least one"}));

  DesignRulesT noTerminations = benchmarkRules();
  noTerminations.feedline_terminations = 0;
  EXPECT_TRUE(validate(noTerminations).empty());
}

TEST(DesignValidation, ComponentsNeedRotationDimensionsAndACouplerPort) {
  CpwCouplerT coupler;
  EXPECT_EQ(validate(coupler),
            (Problems{"rotation is unset", "length must be positive",
                      "height must be positive", "port is missing"}));

  coupler.rotation = Rotation::R0;
  coupler.length = 200.0;
  coupler.height = 26.0;
  coupler.port = makePort("Qb1.port0", UnassignedRole::Resonator);
  EXPECT_EQ(validate(coupler), (Problems{"port role must be Coupler"}));

  coupler.port->role = UnassignedRole::Coupler;
  EXPECT_TRUE(validate(coupler).empty());

  BridgeT bridge;
  bridge.rotation = Rotation::R45;
  bridge.width = 60.0;
  EXPECT_EQ(validate(bridge), (Problems{"height must be positive"}));
}

TEST(DesignValidation, ChipProblemsNameTheOffendingPortAndObstacle) {
  ChipT chip;
  chip.obstacles.push_back(std::make_unique<PolygonT>());
  chip.obstacles.back()->vertices = {Point(0.0, 0.0), Point(1.0, 0.0)};
  chip.ports.push_back(makePort("Chip.port0", UnassignedRole::Launcher));
  chip.ports.push_back(makePort("", UnassignedRole::Unset));

  EXPECT_EQ(validate(chip),
            (Problems{"obstacle 0 has fewer than three vertices",
                      "port 1: label is empty", "port 1: role is unset"}));
}

} // namespace
