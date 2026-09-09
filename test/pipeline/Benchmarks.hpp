/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#pragma once

// The benchmark chips the repository ships, as the stages take them.
//
// Every stage test loads its input through here, so the eight chips are
// described once. The configuration is built rather than parsed, because the
// TOML loader is Python's; the values that are per chip are read out of the
// shipped `config.toml` so that no figure is copied by hand.

#include "mqt-scpd/flatbuffers/config.hpp"
#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/io/Chip.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace mqt::scpd::pipeline {

using flatbuffers::config::ConfigT;
using flatbuffers::design::ChipT;

inline const std::string BENCHMARKS = MQT_SCPD_BENCHMARK_DIR;

inline std::string readFile(const std::string& path) {
  std::ifstream file(path);
  EXPECT_TRUE(file.is_open()) << path;
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

/// A chip and its configuration, as the shipped files describe them.
///
/// The configuration is built here rather than parsed, because the TOML
/// loader is Python's; what this needs is the same values.
struct Benchmark {
  ChipT chip;
  ConfigT config;
};

/// Read the port sequences out of a shipped configuration.
///
/// The file is TOML and this is C++, so the two arrays are read by hand.
/// That is deliberate: a test that built its own ring would not be testing
/// the ring the chip actually ships with.
inline std::vector<std::string> readArray(const std::string& text, const std::string& key) {
  std::vector<std::string> values;
  const auto start = text.find(key + " = [");
  if (start == std::string::npos) {
    return values;
  }
  const auto end = text.find(']', start);
  for (auto quote = text.find('"', start); quote < end && quote != std::string::npos;) {
    const auto close = text.find('"', quote + 1);
    values.push_back(text.substr(quote + 1, close - quote - 1));
    quote = text.find('"', close + 1);
  }
  return values;
}

/// Read one whole-number key out of a shipped configuration.
///
/// A key the file leaves out is one the chip takes at its default, so the
/// default is what the fixture uses. Reading the scalars rather than copying
/// them keeps the fixture on the figures the chip ships with, which is the
/// reason the port sequences are read as well.
inline std::uint32_t readScalar(const std::string& text, const std::string& key,
                         const std::uint32_t fallback) {
  const auto start = text.find(key + " = ");
  if (start == std::string::npos) {
    return fallback;
  }
  return static_cast<std::uint32_t>(std::stoul(text.substr(start + key.size() + 3)));
}

/// One declared bridge rule, as the two expressions the configuration pairs on.
using RulePair = std::pair<std::string, std::string>;

inline Benchmark load(const std::string& chip, const std::string& resonator,
               const std::string& conventional, const std::string& bridgePair,
               const std::vector<RulePair>& bridgeRules, const bool internalBridges = false) {
  namespace fbc = flatbuffers::config;
  Benchmark benchmark;
  const auto configText = readFile(BENCHMARKS + "/" + chip + "/config.toml");

  auto& config = benchmark.config;
  config.chip_input = "routing_config.json";
  config.ports = std::make_unique<fbc::PortConfigT>();
  config.ports->patterns = std::make_unique<fbc::PortPatternsT>();
  config.ports->patterns->launcher = R"(^Chip\.port\d+$)";
  config.ports->patterns->resonator = resonator;
  config.ports->patterns->conventional = conventional;
  config.ports->patterns->bridge_pair = bridgePair;
  config.ports->patterns->component = R"(^([^.]+)\.port\d+$)";
  for (const auto& [first, second] : bridgeRules) {
    auto rule = std::make_unique<fbc::BridgeRuleT>();
    rule->first = first;
    rule->second = second;
    config.ports->bridge_pairs.push_back(std::move(rule));
  }
  config.ports->sequences = std::make_unique<fbc::PortSequencesT>();
  config.ports->sequences->all_outer = readArray(configText, "all_outer");
  config.ports->sequences->fixed_outer = readArray(configText, "fixed_outer");

  config.rules = std::make_unique<flatbuffers::design::DesignRulesT>();
  config.rules->min_wire_spacing = 185.0;
  config.rules->min_obstacle_spacing = 25.0;
  config.rules->min_bend_radius = 50.0;
  config.rules->min_straight_length = 100.0;
  config.rules->target_resonator_length = 2500.0;
  config.rules->resonator_length_tolerance = 100.0;
  config.rules->max_feedline_utilization = readScalar(configText, "max_feedline_utilization", 0);
  config.rules->feedline_terminations = readScalar(configText, "feedline_terminations", 0);

  config.grid = std::make_unique<fbc::GridParamsT>();
  config.grid->capacity_cells_x = readScalar(configText, "capacity_cells_x", 50);
  config.grid->capacity_cells_y = readScalar(configText, "capacity_cells_y", 0);
  config.grid->launcher_offset_x = readScalar(configText, "launcher_offset_x", 15);
  config.grid->launcher_offset_y = readScalar(configText, "launcher_offset_y", 15);
  config.grid->detail_factor = 30;

  config.stages = std::make_unique<fbc::StageParamsT>();
  config.stages->capacity = std::make_unique<fbc::CapacityParamsT>();
  config.stages->assignment = std::make_unique<fbc::AssignmentParamsT>();
  config.stages->assignment->launcher_target = readScalar(configText, "launcher_target", 0);
  config.stages->global = std::make_unique<fbc::GlobalParamsT>();
  config.stages->global->internal_bridges = internalBridges;

  benchmark.chip =
      io::loadChip(readFile(BENCHMARKS + "/" + chip + "/routing_config.json"), config);
  return benchmark;
}

/// The seven chips whose qubits are `Qb<n>` and whose couplers are
/// `Coupler<a>_<b>` with five ports each.
inline Benchmark qubitAndCouplerChip(const std::string& chip, const bool internalBridges = false) {
  return load(chip, R"(^Qb\d+\.port0$)", R"(^(Qb\d+\.port1|Coupler\d+_\d+\.port0)$)",
              R"(^Coupler\d+_\d+\.port[1-4]$)",
              {{R"(^(Coupler\d+_\d+)\.port1$)", R"(^(Coupler\d+_\d+)\.port2$)"},
               {R"(^(Coupler\d+_\d+)\.port3$)", R"(^(Coupler\d+_\d+)\.port4$)"}},
              internalBridges);
}

/// The 4-qubit chip, whose qubits are `Q<n>` and whose couplers are `C<a><b>`
/// with three ports each. That naming difference is what the role patterns are
/// configuration for, so it stays in the fixture.
inline Benchmark fourQubit() {
  return load("4q", R"(^Q\d+\.port0$)", R"(^(Q\d+\.port1|C\d+\.port0)$)",
              R"(^C\d+\.port[12]$)", {{R"(^(C\d+)\.port1$)", R"(^(C\d+)\.port2$)"}});
}

inline Benchmark seventeenQubit() { return qubitAndCouplerChip("17q"); }

inline Benchmark nineQubit(const bool internalBridges = false) {
  return qubitAndCouplerChip("9q", internalBridges);
}

/// The benchmark chip a directory name stands for.
inline Benchmark benchmarkOf(const std::string& chip) {
  return chip == "4q" ? fourQubit() : qubitAndCouplerChip(chip);
}

} // namespace mqt::scpd::pipeline
