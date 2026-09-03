/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/generated/design_generated.hpp"
#include "mqt-scpd/generated/geometry_generated.hpp"

#include <flatbuffers/buffer.h>
#include <flatbuffers/flatbuffer_builder.h>
#include <flatbuffers/verifier.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <string>
#include <utility>

namespace {

using namespace mqt::scpd::generated;

template <typename NativeT> NativeT roundTrip(const NativeT& object) {
  flatbuffers::FlatBufferBuilder builder;
  builder.Finish(NativeT::TableType::Pack(builder, &object));

  flatbuffers::Verifier verifier(builder.GetBufferPointer(), builder.GetSize());
  EXPECT_TRUE(verifier.VerifyBuffer<typename NativeT::TableType>(nullptr));

  NativeT result;
  flatbuffers::GetRoot<typename NativeT::TableType>(builder.GetBufferPointer())
      ->UnPackTo(&result);
  return result;
}

std::unique_ptr<PortT> makePort(std::string label, const Point centre,
                                const UnassignedRole role) {
  auto port = std::make_unique<PortT>();
  port->label = std::move(label);
  port->centre = centre;
  port->orientation = 90.0;
  port->role = role;
  return port;
}

// The numeric values of the enums are the on-disk format. FlatBuffers rules
// allow appending members, never renumbering.
TEST(DesignSchema, UnassignedRoleValuesAreTheWireFormat) {
  EXPECT_EQ(static_cast<std::uint8_t>(UnassignedRole::Launcher), 0U);
  EXPECT_EQ(static_cast<std::uint8_t>(UnassignedRole::Resonator), 1U);
  EXPECT_EQ(static_cast<std::uint8_t>(UnassignedRole::Conventional), 2U);
  EXPECT_STREQ(EnumNameUnassignedRole(UnassignedRole::Resonator), "Resonator");
}

TEST(DesignSchema, AssignedRoleValuesAreTheWireFormat) {
  EXPECT_EQ(static_cast<std::uint8_t>(AssignedRole::FeedlineSource), 0U);
  EXPECT_EQ(static_cast<std::uint8_t>(AssignedRole::FeedlineTarget), 1U);
  EXPECT_EQ(static_cast<std::uint8_t>(AssignedRole::ResonatorSource), 2U);
  EXPECT_EQ(static_cast<std::uint8_t>(AssignedRole::ResonatorTarget), 3U);
  EXPECT_EQ(static_cast<std::uint8_t>(AssignedRole::ConventionalSource), 4U);
  EXPECT_EQ(static_cast<std::uint8_t>(AssignedRole::ConventionalTarget), 5U);
  EXPECT_STREQ(EnumNameAssignedRole(AssignedRole::ResonatorSource),
               "ResonatorSource");
}

TEST(DesignSchema, RotationIsEightWay) {
  EXPECT_EQ(static_cast<std::uint8_t>(Rotation::R0), 0U);
  EXPECT_EQ(static_cast<std::uint8_t>(Rotation::R315), 7U);
  EXPECT_EQ(static_cast<std::uint8_t>(Rotation::MAX), 7U);
}

TEST(DesignSchema, ChipRoundTripsPortsWithRoles) {
  ChipT chip;
  auto outline = std::make_unique<PolygonT>();
  outline->vertices = {Point(0.0, 0.0), Point(1000.0, 0.0),
                       Point(1000.0, 1000.0), Point(0.0, 1000.0)};
  chip.obstacles.push_back(std::move(outline));
  chip.ports.push_back(
      makePort("Chip.port0", Point(0.0, 500.0), UnassignedRole::Launcher));
  chip.ports.push_back(
      makePort("Qb1.port0", Point(500.0, 500.0), UnassignedRole::Resonator));
  chip.ports.push_back(makePort("Coupler1_2.port3", Point(600.0, 500.0),
                                UnassignedRole::Conventional));

  const ChipT back = roundTrip(chip);
  EXPECT_EQ(back, chip);
  ASSERT_EQ(back.ports.size(), 3U);
  EXPECT_EQ(back.ports[1]->label, "Qb1.port0");
  EXPECT_EQ(back.ports[1]->role, UnassignedRole::Resonator);
  EXPECT_EQ(back.ports[1]->centre, Point(500.0, 500.0));
  EXPECT_DOUBLE_EQ(back.ports[1]->orientation, 90.0);
}

TEST(DesignSchema, ConnectionSourceIsAbsentUntilMaterialized) {
  // A resonator's source port is created by coupler insertion in the Final
  // stage, so the Assignment stage records the connection without one.
  ConnectionT fed;
  fed.target = PortRef(1);
  fed.source_role = AssignedRole::ResonatorSource;
  fed.target_role = AssignedRole::ResonatorTarget;

  const ConnectionT backFed = roundTrip(fed);
  EXPECT_EQ(backFed, fed);
  EXPECT_EQ(backFed.source, nullptr);
  EXPECT_EQ(backFed.target.index(), 1U);

  ConnectionT feedline;
  feedline.source = std::make_unique<PortRef>(0);
  feedline.target = PortRef(7);
  feedline.source_role = AssignedRole::FeedlineSource;
  feedline.target_role = AssignedRole::FeedlineTarget;

  const ConnectionT backFeedline = roundTrip(feedline);
  EXPECT_EQ(backFeedline, feedline);
  ASSERT_NE(backFeedline.source, nullptr);
  EXPECT_EQ(backFeedline.source->index(), 0U);
}

TEST(DesignSchema, DesignRulesRoundTripInLayoutUnits) {
  DesignRulesT rules;
  rules.min_wire_spacing = 185.0;
  rules.min_obstacle_spacing = 25.0;
  rules.min_bend_radius = 50.0;
  rules.min_straight_length = 100.0;
  rules.target_resonator_length = 2500.0;
  rules.resonator_length_tolerance = 100.0;
  rules.max_feedline_utilization = 6;
  rules.feedline_terminations = 0;

  const DesignRulesT back = roundTrip(rules);
  EXPECT_EQ(back, rules);
  EXPECT_DOUBLE_EQ(back.min_wire_spacing, 185.0);
  EXPECT_EQ(back.max_feedline_utilization, 6U);
}

TEST(DesignSchema, ComponentsCarryExactDimensions) {
  CpwCouplerT coupler;
  coupler.port = PortRef(42);
  coupler.centre = Point(1200.0, 800.0);
  coupler.rotation = Rotation::R90;
  coupler.length = 200.0;
  coupler.height = 26.0;

  const CpwCouplerT backCoupler = roundTrip(coupler);
  EXPECT_EQ(backCoupler, coupler);
  EXPECT_EQ(backCoupler.port.index(), 42U);
  EXPECT_EQ(backCoupler.rotation, Rotation::R90);

  BridgeT bridge;
  bridge.centre = Point(300.0, 300.0);
  bridge.rotation = Rotation::R45;
  bridge.width = 60.0;
  bridge.height = 60.0;

  const BridgeT backBridge = roundTrip(bridge);
  EXPECT_EQ(backBridge, bridge);
  EXPECT_DOUBLE_EQ(backBridge.width, 60.0);
}

} // namespace
