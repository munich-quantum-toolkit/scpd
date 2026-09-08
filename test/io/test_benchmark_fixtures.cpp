/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// The two benchmark inputs the repository carries, loaded with the patterns
// and the port sequences of their shipped configurations. The sequences are
// the hand-written literals of the prototype's drivers; loading checks every
// label against the chip.

#include "mqt-scpd/flatbuffers/config.hpp"
#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/io/Chip.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <fstream>
#include <iterator>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

using namespace mqt::scpd::flatbuffers::config;
using mqt::scpd::flatbuffers::design::ChipT;
using mqt::scpd::flatbuffers::design::DesignRulesT;
using mqt::scpd::flatbuffers::design::UnassignedRole;
using mqt::scpd::io::loadChip;

std::string readFile(const std::string& path) {
  std::ifstream file(path);
  EXPECT_TRUE(file.is_open()) << path;
  return {std::istreambuf_iterator<char>(file),
          std::istreambuf_iterator<char>()};
}

ConfigT configFor(const std::string_view resonator,
                  const std::string_view conventional,
                  std::vector<std::string> allOuter,
                  std::vector<std::string> fixedOuter) {
  ConfigT config;
  config.chip_input = "routing_config.json";
  config.ports = std::make_unique<PortConfigT>();
  config.ports->patterns = std::make_unique<PortPatternsT>();
  config.ports->patterns->launcher = R"(^Chip\.port\d+$)";
  config.ports->patterns->resonator = std::string(resonator);
  config.ports->patterns->conventional = std::string(conventional);
  config.ports->sequences = std::make_unique<PortSequencesT>();
  config.ports->sequences->all_outer = std::move(allOuter);
  config.ports->sequences->fixed_outer = std::move(fixedOuter);
  config.rules = std::make_unique<DesignRulesT>();
  config.rules->min_wire_spacing = 185.0;
  config.rules->min_obstacle_spacing = 25.0;
  config.rules->min_bend_radius = 50.0;
  config.rules->min_straight_length = 100.0;
  config.rules->target_resonator_length = 2500.0;
  config.rules->resonator_length_tolerance = 100.0;
  config.rules->max_feedline_utilization = 4;
  return config;
}

std::map<UnassignedRole, std::size_t> roleCounts(const ChipT& chip) {
  std::map<UnassignedRole, std::size_t> counts;
  for (const auto& port : chip.ports) {
    ++counts[port->role];
  }
  return counts;
}

const std::string BENCHMARKS = MQT_SCPD_BENCHMARK_DIR;

// The driver pins every outer port, so the fixed sequence is the whole ring.
const std::vector<std::string> FOUR_QUBIT_RING = {
    "Q1.port0", "Q1.port1", "C12.port0", "Q2.port1", "Q2.port0", "C23.port0",
    "Q3.port0", "Q3.port1", "C34.port0", "Q4.port0", "Q4.port1", "C14.port0",
};

const std::vector<std::string> NINE_QUBIT_ALL_OUTER = {
    "Qb1.port1",        "Qb1.port0",        "Coupler1_2.port1",
    "Coupler1_2.port0", "Coupler1_2.port3", "Qb2.port1",
    "Qb2.port0",        "Coupler2_3.port3", "Coupler2_3.port0",
    "Coupler2_3.port1", "Qb3.port0",        "Qb3.port1",
    "Coupler3_6.port1", "Coupler3_6.port0", "Coupler3_6.port3",
    "Qb6.port1",        "Qb6.port0",        "Coupler6_9.port1",
    "Coupler6_9.port0", "Coupler6_9.port3", "Qb9.port0",
    "Qb9.port1",        "Coupler8_9.port1", "Coupler8_9.port0",
    "Coupler8_9.port3", "Qb8.port0",        "Qb8.port1",
    "Coupler7_8.port3", "Coupler7_8.port0", "Coupler7_8.port1",
    "Qb7.port1",        "Qb7.port0",        "Coupler4_7.port3",
    "Coupler4_7.port0", "Coupler4_7.port1", "Qb4.port0",
    "Qb4.port1",        "Coupler1_4.port3", "Coupler1_4.port0",
    "Coupler1_4.port1",
};

// The driver's fixed sequence stops three entries short of the ring: it omits
// Qb4 and Coupler1_4, which its own all_outer still lists.
const std::vector<std::string> NINE_QUBIT_FIXED_OUTER = {
    "Qb1.port1", "Qb1.port0", "Coupler1_2.port0",
    "Qb2.port1", "Qb2.port0", "Coupler2_3.port0",
    "Qb3.port0", "Qb3.port1", "Coupler3_6.port0",
    "Qb6.port1", "Qb6.port0", "Coupler6_9.port0",
    "Qb9.port0", "Qb9.port1", "Coupler8_9.port0",
    "Qb8.port0", "Qb8.port1", "Coupler7_8.port0",
    "Qb7.port1", "Qb7.port0", "Coupler4_7.port0",
};

TEST(BenchmarkFixtures, FourQubitChipLoadsWithItsSequences) {
  const ChipT chip = loadChip(readFile(BENCHMARKS + "/4q/routing_config.json"),
                              configFor(R"(^Q\d+\.port0$)",
                                        R"(^(Q\d+\.port1|C\d+\.port[0-2])$)",
                                        FOUR_QUBIT_RING, FOUR_QUBIT_RING));

  EXPECT_EQ(chip.obstacles.size(), 28U);
  EXPECT_EQ(roleCounts(chip), (std::map<UnassignedRole, std::size_t>{
                                  {UnassignedRole::Launcher, 16},
                                  {UnassignedRole::Resonator, 4},
                                  {UnassignedRole::Conventional, 16}}));
}

TEST(BenchmarkFixtures, NineQubitChipLoadsWithItsSequences) {
  const ChipT chip =
      loadChip(readFile(BENCHMARKS + "/9q/routing_config.json"),
               configFor(R"(^Qb\d+\.port0$)",
                         R"(^(Qb\d+\.port1|Coupler\d+_\d+\.port[0-4])$)",
                         NINE_QUBIT_ALL_OUTER, NINE_QUBIT_FIXED_OUTER));

  EXPECT_EQ(roleCounts(chip), (std::map<UnassignedRole, std::size_t>{
                                  {UnassignedRole::Launcher, 24},
                                  {UnassignedRole::Resonator, 9},
                                  {UnassignedRole::Conventional, 69}}));
}

TEST(BenchmarkFixtures, ASequenceLabelTheChipLacksIsRefused) {
  std::vector<std::string> ring = FOUR_QUBIT_RING;
  ring.emplace_back("Q9.port0");
  EXPECT_THROW(
      static_cast<void>(loadChip(
          readFile(BENCHMARKS + "/4q/routing_config.json"),
          configFor(R"(^Q\d+\.port0$)", R"(^(Q\d+\.port1|C\d+\.port[0-2])$)",
                    ring, FOUR_QUBIT_RING))),
      std::invalid_argument);
}

} // namespace
