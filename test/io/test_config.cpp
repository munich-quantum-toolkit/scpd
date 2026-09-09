/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/flatbuffers/config.hpp"
#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/io/Config.hpp"

#include <flatbuffers/flatbuffer_builder.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using namespace mqt::scpd::flatbuffers::config;
using mqt::scpd::flatbuffers::design::DesignRulesT;
using mqt::scpd::io::readConfig;

TEST(ConfigBuffer, UnpacksWhatTheBuilderPacked) {
  ConfigT config;
  config.chip_input = "routing_config.json";
  config.ports = std::make_unique<PortConfigT>();
  config.ports->patterns = std::make_unique<PortPatternsT>();
  config.ports->patterns->launcher = R"(^Chip\.port\d+$)";
  config.ports->patterns->resonator = R"(^Qb\d+\.port0$)";
  config.ports->patterns->conventional = R"(^Qb\d+\.port1$)";
  config.ports->patterns->mating = R"(^Qb\d+\.port[2-5]$)";
  config.ports->sequences = std::make_unique<PortSequencesT>();
  config.ports->sequences->all_outer = {"Qb15.port1", "Qb15.port0"};
  config.ports->sequences->fixed_outer = {"Qb15.port0"};
  config.rules = std::make_unique<DesignRulesT>();
  config.rules->min_wire_spacing = 185.0;
  config.grid = std::make_unique<GridParamsT>();
  config.grid->capacity_cells_x = 25;
  config.grid->capacity_cells_y = 25;
  config.grid->launcher_offset_x = 20;

  flatbuffers::FlatBufferBuilder builder;
  builder.Finish(Config::Pack(builder, &config));
  const std::vector<std::uint8_t> bytes(
      builder.GetBufferPointer(),
      std::next(builder.GetBufferPointer(), builder.GetSize()));

  const ConfigT back = readConfig(bytes);
  EXPECT_EQ(back, config);
  EXPECT_EQ(back.ports->patterns->mating, R"(^Qb\d+\.port[2-5]$)");
  EXPECT_EQ(back.ports->sequences->fixed_outer,
            (std::vector<std::string>{"Qb15.port0"}));
  EXPECT_EQ(back.grid->launcher_offset_y, 15U);
}

TEST(ConfigBuffer, RefusesBytesThatAreNotAConfiguration) {
  const std::vector<std::uint8_t> garbage = {9, 9, 9, 9, 9, 9, 9, 9};
  EXPECT_THROW(static_cast<void>(readConfig(garbage)), std::invalid_argument);
}

} // namespace
