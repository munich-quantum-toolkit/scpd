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
#include "mqt-scpd/flatbuffers/geometry.hpp"
#include "mqt-scpd/io/Chip.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

using namespace mqt::scpd::flatbuffers::config;
using mqt::scpd::flatbuffers::design::ChipT;
using mqt::scpd::flatbuffers::design::DesignRulesT;
using mqt::scpd::flatbuffers::design::UnassignedRole;
using mqt::scpd::flatbuffers::geometry::Point;
using mqt::scpd::io::loadChip;
using mqt::scpd::io::readChip;
using mqt::scpd::io::readChipJson;
using mqt::scpd::io::writeChip;

constexpr std::string_view CHIP_TEXT = R"({
  "sampleSpacing": 1.0,
  "nets": [],
  "obstacles": [
    {"polygon": [[0.0, 0.0], [100.0, 0.0], [100.0, 50.0]]},
    {"polygon": [[200, 200], [300, 200], [300, 300], [200, 300]]}
  ],
  "ports": {
    "Qb1.port0": {"center": [10.5, 20.5], "orientation": 495.0},
    "Chip.port0": {"center": [-800.0, 850.0], "orientation": 90.0},
    "Qb1.port1": {"center": [30.0, 40.0]}
  }
})";

ConfigT manualConfig() {
  ConfigT config;
  config.chip_input = "routing_config.json";
  config.ports = std::make_unique<PortConfigT>();
  config.ports->patterns = std::make_unique<PortPatternsT>();
  config.ports->patterns->launcher = R"(^Chip\.port\d+$)";
  config.ports->patterns->resonator = R"(^Qb\d+\.port0$)";
  config.ports->patterns->conventional = R"(^Qb\d+\.port1$)";
  config.ports->sequences = std::make_unique<PortSequencesT>();
  config.ports->sequences->all_outer = {"Qb1.port0", "Qb1.port1"};
  config.ports->sequences->fixed_outer = {"Qb1.port0"};
  config.rules = std::make_unique<DesignRulesT>();
  config.rules->min_wire_spacing = 185.0;
  config.rules->min_obstacle_spacing = 25.0;
  config.rules->min_bend_radius = 50.0;
  config.rules->min_straight_length = 100.0;
  config.rules->target_resonator_length = 2500.0;
  config.rules->resonator_length_tolerance = 100.0;
  config.rules->max_feedline_utilization = 6;
  return config;
}

void expectThrowMentioning(const std::string_view text,
                           const std::string_view fragment) {
  try {
    static_cast<void>(readChipJson(text));
    FAIL() << "no exception for " << fragment;
  } catch (const std::invalid_argument& error) {
    EXPECT_NE(std::string_view(error.what()).find(fragment),
              std::string_view::npos)
        << error.what();
  }
}

TEST(ChipJson, ReadsObstaclesAndPortsInFileOrder) {
  const ChipT chip = readChipJson(CHIP_TEXT);

  ASSERT_EQ(chip.obstacles.size(), 2U);
  EXPECT_EQ(
      chip.obstacles[0]->vertices,
      (std::vector{Point(0.0, 0.0), Point(100.0, 0.0), Point(100.0, 50.0)}));
  ASSERT_EQ(chip.ports.size(), 3U);
  EXPECT_EQ(chip.ports[0]->label, "Qb1.port0");
  EXPECT_EQ(chip.ports[0]->center, Point(10.5, 20.5));
  EXPECT_DOUBLE_EQ(chip.ports[0]->orientation, 495.0);
  EXPECT_EQ(chip.ports[0]->role, UnassignedRole::Unset);
  EXPECT_EQ(chip.ports[1]->label, "Chip.port0");
  EXPECT_EQ(chip.ports[2]->label, "Qb1.port1");
  EXPECT_DOUBLE_EQ(chip.ports[2]->orientation, 0.0);
}

TEST(ChipJson, RejectsWhatTheLoaderDoesNotRead) {
  expectThrowMentioning("not json", "not JSON");
  expectThrowMentioning("[1, 2]", "not a JSON object");
  expectThrowMentioning(R"({"obstacles": [], "ports": {}, "nets": [{"a": 1}]})",
                        "nets is not empty (1 entries)");
  expectThrowMentioning(R"({"obstacles": [], "ports": {}, "layers": 2})",
                        "unknown key 'layers'");
  expectThrowMentioning(R"({"ports": {}})", "obstacles are missing");
  expectThrowMentioning(R"({"obstacles": []})", "ports are missing");
}

TEST(ChipJson, NamesTheOffendingObstacleOrPort) {
  expectThrowMentioning(
      R"({"obstacles": [{"polygon": [[0, 0], [1, 1]]}], "ports": {}})",
      "obstacle 0 has fewer than three vertices");
  expectThrowMentioning(
      R"({"obstacles": [{"polygon": [[0, 0], [1], [2, 2]]}], "ports": {}})",
      "obstacle 0 vertex 1 is not a pair of numbers");
  expectThrowMentioning(
      R"({"obstacles": [{"polygon": [[0, 0], [1, 0], [1, 1]], "layer": 1}], "ports": {}})",
      "obstacle 0 has the unknown key 'layer'");
  expectThrowMentioning(
      R"({"obstacles": [], "ports": {"Qb1.port0": {"orientation": 90.0}}})",
      "port 'Qb1.port0' has no center that is a pair of numbers");
  expectThrowMentioning(
      R"({"obstacles": [], "ports": {"Qb1.port0": {"center": [0, 0], "orientation": "north"}}})",
      "port 'Qb1.port0' has an orientation that is not a number");
  expectThrowMentioning(
      R"({"obstacles": [], "ports": {"Qb1.port0": {"center": [0, 0], "width": 1}}})",
      "port 'Qb1.port0' has the unknown key 'width'");
}

TEST(ChipJson, ReportsManyProblemsUpToALimit) {
  std::string text = R"({"obstacles": [], "ports": {)";
  for (int i = 0; i < 25; ++i) {
    text += (i == 0 ? "" : ", ");
    text += "\"P" + std::to_string(i) + "\": {}";
  }
  text += "}}";
  try {
    static_cast<void>(readChipJson(text));
    FAIL();
  } catch (const std::invalid_argument& error) {
    const std::string_view message = error.what();
    EXPECT_NE(message.find("port 'P19'"), std::string_view::npos);
    EXPECT_EQ(message.find("port 'P20'"), std::string_view::npos);
    EXPECT_NE(message.find("and 5 more"), std::string_view::npos);
  }
}

TEST(ChipLoad, ClassifiesThePortsFromTheConfiguration) {
  const ChipT chip = loadChip(CHIP_TEXT, manualConfig());

  EXPECT_EQ(chip.ports[0]->role, UnassignedRole::Resonator);
  EXPECT_EQ(chip.ports[1]->role, UnassignedRole::Launcher);
  EXPECT_EQ(chip.ports[2]->role, UnassignedRole::Conventional);
}

TEST(ChipLoad, RefusesAPortWithoutARoleOrAConfigurationThatDoesNotFit) {
  ConfigT config = manualConfig();
  config.ports->patterns->conventional = R"(^Coupler\d+\.port1$)";
  EXPECT_THROW(static_cast<void>(loadChip(CHIP_TEXT, config)),
               std::invalid_argument);

  config = manualConfig();
  config.ports->sequences->all_outer.emplace_back("Qb7.port0");
  try {
    static_cast<void>(loadChip(CHIP_TEXT, config));
    FAIL();
  } catch (const std::invalid_argument& error) {
    EXPECT_NE(std::string_view(error.what()).find("'Qb7.port0' is not a port"),
              std::string_view::npos);
  }

  config = manualConfig();
  config.rules.reset();
  try {
    static_cast<void>(loadChip(CHIP_TEXT, config));
    FAIL();
  } catch (const std::invalid_argument& error) {
    EXPECT_NE(std::string_view(error.what()).find("design rules are missing"),
              std::string_view::npos);
  }
}

TEST(ChipBuffer, RoundTripsAClassifiedChipAndRefusesTheRest) {
  const ChipT chip = loadChip(CHIP_TEXT, manualConfig());
  const std::vector<std::uint8_t> bytes = writeChip(chip);
  EXPECT_EQ(readChip(bytes), chip);

  const ChipT unclassified = readChipJson(CHIP_TEXT);
  EXPECT_THROW(static_cast<void>(writeChip(unclassified)),
               std::invalid_argument);

  const std::vector<std::uint8_t> garbage = {1, 2, 3, 4, 5, 6, 7, 8};
  EXPECT_THROW(static_cast<void>(readChip(garbage)), std::invalid_argument);
}

} // namespace
