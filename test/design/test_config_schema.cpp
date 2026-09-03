/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/generated/config_generated.hpp"
#include "mqt-scpd/generated/design_generated.hpp"

#include <flatbuffers/buffer.h>
#include <flatbuffers/flatbuffer_builder.h>
#include <flatbuffers/verifier.h>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>

namespace {

using namespace mqt::scpd::generated;

template <typename NativeT> NativeT roundTrip(const NativeT& object) {
  flatbuffers::FlatBufferBuilder builder;
  builder.Finish(NativeT::TableType::Pack(builder, &object));

  flatbuffers::Verifier verifier(builder.GetBufferPointer(),
                                 builder.GetSize());
  EXPECT_TRUE(verifier.VerifyBuffer<typename NativeT::TableType>(nullptr));

  NativeT result;
  flatbuffers::GetRoot<typename NativeT::TableType>(builder.GetBufferPointer())
      ->UnPackTo(&result);
  return result;
}

std::unique_ptr<PortPatternsT> benchmarkPatterns() {
  auto patterns = std::make_unique<PortPatternsT>();
  patterns->launcher = R"(^Chip\.port\d+$)";
  patterns->resonator = R"(^Qb?\d+\.port0$)";
  patterns->conventional = R"(^(Qb?\d+\.port1|Coupler\d+_\d+\.port[0-4])$)";
  return patterns;
}

// The schema defaults are the values the loader applies to keys a
// configuration leaves out. They are the documented defaults of config.toml.
TEST(ConfigSchema, PortDetectionDefaultsToManual) {
  const PortConfigT ports;
  EXPECT_EQ(ports.detection, PortDetection::Manual);
  EXPECT_TRUE(ports.start_component.empty());
  EXPECT_EQ(ports.sequences, nullptr);
}

TEST(ConfigSchema, GridDefaults) {
  const GridParamsT grid;
  EXPECT_EQ(grid.capacity_cells_x, 50U);
  EXPECT_EQ(grid.capacity_cells_y, 0U);
  EXPECT_EQ(grid.launcher_offset_x, 15U);
  EXPECT_EQ(grid.launcher_offset_y, 15U);
}

TEST(ConfigSchema, FinalStageDefaults) {
  const FinalParamsT final;
  EXPECT_DOUBLE_EQ(final.meander_length, 600.0);
  EXPECT_EQ(final.expansion, 200U);
  EXPECT_EQ(final.rounds, 10U);
  EXPECT_EQ(final.refinement_rounds, 15U);
}

TEST(ConfigSchema, ComponentDefaultsAreThePhysicalDimensions) {
  const ComponentParamsT components;
  EXPECT_DOUBLE_EQ(components.coupler_length, 200.0);
  EXPECT_DOUBLE_EQ(components.coupler_height, 26.0);
  EXPECT_DOUBLE_EQ(components.bridge_width, 60.0);
  EXPECT_DOUBLE_EQ(components.bridge_height, 60.0);
}

TEST(ConfigSchema, DrcDefaultsSkipTheAdvisoryRules) {
  const DrcParamsT drc;
  EXPECT_FALSE(drc.all);
  EXPECT_FALSE(drc.component_overlap);
  EXPECT_FALSE(drc.min_straight_length);
  EXPECT_FALSE(drc.min_bend_radius);
  EXPECT_DOUBLE_EQ(drc.short_threshold, 2.0);
  EXPECT_DOUBLE_EQ(drc.junction_radius_norm, 1.5);
}

TEST(ConfigSchema, ManualConfigRoundTripsSequences) {
  ConfigT config;
  config.chip_input = "routing_config.json";
  config.ports = std::make_unique<PortConfigT>();
  config.ports->patterns = benchmarkPatterns();
  config.ports->detection = PortDetection::Manual;
  config.ports->sequences = std::make_unique<PortSequencesT>();
  config.ports->sequences->all_outer = {"Qb1.port0", "Qb1.port1",
                                        "Coupler1_2.port3"};
  config.ports->sequences->fixed_outer = {};
  config.rules = std::make_unique<DesignRulesT>();
  config.rules->min_wire_spacing = 185.0;
  config.stages = std::make_unique<StageParamsT>();
  config.stages->assignment = std::make_unique<AssignmentParamsT>();
  config.stages->assignment->launcher_target = 24;

  const ConfigT back = roundTrip(config);
  EXPECT_EQ(back, config);
  ASSERT_NE(back.ports->sequences, nullptr);
  EXPECT_EQ(back.ports->sequences->all_outer,
            (std::vector<std::string>{"Qb1.port0", "Qb1.port1",
                                      "Coupler1_2.port3"}));
  EXPECT_TRUE(back.ports->sequences->fixed_outer.empty());
  EXPECT_EQ(back.stages->assignment->launcher_target, 24U);
  EXPECT_EQ(back.grid, nullptr);
}

TEST(ConfigSchema, AutoConfigRoundTripsStartComponent) {
  ConfigT config;
  config.chip_input = "routing_config.json";
  config.ports = std::make_unique<PortConfigT>();
  config.ports->patterns = benchmarkPatterns();
  config.ports->detection = PortDetection::Auto;
  config.ports->start_component = "Qb15";
  config.rules = std::make_unique<DesignRulesT>();

  const ConfigT back = roundTrip(config);
  EXPECT_EQ(back, config);
  EXPECT_EQ(back.ports->detection, PortDetection::Auto);
  EXPECT_EQ(back.ports->start_component, "Qb15");
  EXPECT_EQ(back.ports->sequences, nullptr);
}

} // namespace
