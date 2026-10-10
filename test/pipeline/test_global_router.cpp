/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// The Global stage: which ports form the inner circuit, how the circuit
// reaches them, the ring it leaves for the assignment, and what it reports.

#include "MiniFixture.hpp"
#include "mqt-scpd/design/Roles.hpp"
#include "mqt-scpd/flatbuffers/artifacts.hpp"
#include "mqt-scpd/flatbuffers/config.hpp"
#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/flatbuffers/geometry.hpp"
#include "mqt-scpd/geometry/Geometry.hpp"
#include "mqt-scpd/io/Artifacts.hpp"
#include "mqt-scpd/io/Chip.hpp"
#include "mqt-scpd/pipeline/CapacityPlanner.hpp"
#include "mqt-scpd/pipeline/GlobalRouter.hpp"
#include "mqt-scpd/pipeline/Report.hpp"
#include "mqt-scpd/pipeline/Stages.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace mqt::scpd::pipeline {
namespace {

using flatbuffers::artifacts::ArtifactT;
using flatbuffers::artifacts::CapacityElement;
using flatbuffers::artifacts::CapacityNodeT;
using flatbuffers::config::BridgeRuleT;
using flatbuffers::config::PortPatternsT;
using flatbuffers::design::PortT;
using flatbuffers::geometry::Point;

/// The index of the port a label names.
std::uint32_t indexOf(const ChipT& chip, const std::string_view label) {
  for (std::uint32_t index = 0; index < chip.ports.size(); ++index) {
    if (chip.ports[index]->label == label) {
      return index;
    }
  }
  ADD_FAILURE() << "the chip carries no port " << label;
  return 0;
}

/// The indices of the ports a list of labels names, in its order.
std::vector<std::uint32_t> indicesOf(const ChipT& chip,
                                     const std::vector<std::string>& labels) {
  std::vector<std::uint32_t> indices;
  indices.reserve(labels.size());
  for (const auto& label : labels) {
    indices.push_back(indexOf(chip, label));
  }
  return indices;
}

// innerCircuitOf, on a chip of labels alone. Qb1 is on the ring and Qb2 is
// inside it; the coupler between them has a conventional port on the ring and
// two bridges, one of which surfaces on the ring.

std::vector<std::string> coupledLabels() {
  return {"Chip.port0",       "Qb1.port0",        "Qb1.port1",
          "Qb2.port0",        "Qb2.port1",        "Coupler1_2.port0",
          "Coupler1_2.port1", "Coupler1_2.port2", "Coupler1_2.port3",
          "Coupler1_2.port4"};
}

ChipT coupledChip() {
  PortPatternsT patterns;
  patterns.launcher = R"(^Chip\.port\d+$)";
  patterns.resonator = R"(^Qb\d+\.port0$)";
  patterns.conventional = R"(^(Qb\d+\.port1|Coupler\d+_\d+\.port0)$)";
  patterns.bridge_pair = R"(^Coupler\d+_\d+\.port[1-4]$)";
  patterns.component = R"(^([^.]+)\.port\d+$)";

  ChipT chip;
  for (const auto& label : coupledLabels()) {
    auto port = std::make_unique<PortT>();
    port->label = label;
    port->center = Point(0.0, 0.0);
    chip.ports.push_back(std::move(port));
  }
  EXPECT_TRUE(design::classifyPorts(chip, patterns).empty());
  return chip;
}

ConfigT coupledConfig(const std::vector<std::string>& ring) {
  ConfigT config;
  config.ports = std::make_unique<flatbuffers::config::PortConfigT>();
  config.ports->sequences =
      std::make_unique<flatbuffers::config::PortSequencesT>();
  config.ports->sequences->all_outer = ring;
  for (const auto& [first, second] :
       {std::pair{R"(^(Coupler\d+_\d+)\.port1$)",
                  R"(^(Coupler\d+_\d+)\.port2$)"},
        std::pair{R"(^(Coupler\d+_\d+)\.port3$)",
                  R"(^(Coupler\d+_\d+)\.port4$)"}}) {
    auto rule = std::make_unique<BridgeRuleT>();
    rule->first = first;
    rule->second = second;
    config.ports->bridge_pairs.push_back(std::move(rule));
  }
  return config;
}

TEST(InnerCircuit, HoldsThePortsTheRingDoesNotCarry) {
  const auto chip = coupledChip();
  const auto circuit = innerCircuitOf(
      chip, coupledConfig({"Qb1.port0", "Qb1.port1", "Coupler1_2.port0",
                           "Coupler1_2.port2"}));

  EXPECT_EQ(circuit.innerPorts,
            indicesOf(chip, {"Qb2.port0", "Qb2.port1", "Coupler1_2.port1",
                             "Coupler1_2.port3", "Coupler1_2.port4"}));
  // The bridged ports are claimed, so the inner qubit's ports are left.
  EXPECT_EQ(circuit.targets, indicesOf(chip, {"Qb2.port0", "Qb2.port1"}));
}

TEST(InnerCircuit, OrientsABridgeFromItsInnerEndToTheRing) {
  const auto chip = coupledChip();
  const auto circuit = innerCircuitOf(
      chip, coupledConfig({"Qb1.port0", "Qb1.port1", "Coupler1_2.port0",
                           "Coupler1_2.port2"}));

  ASSERT_EQ(circuit.bridges.size(), 2U);
  ASSERT_EQ(circuit.outerBridges.size(), 1U);
  EXPECT_EQ(circuit.outerBridges[0].from, indexOf(chip, "Coupler1_2.port1"));
  EXPECT_EQ(circuit.outerBridges[0].to, indexOf(chip, "Coupler1_2.port2"));
  // The other bridge has both ends inside, in whichever orientation.
  const auto internal =
      std::ranges::find_if(circuit.bridges, [&](const PortBridge& bridge) {
        return bridge.from != circuit.outerBridges[0].from;
      });
  ASSERT_NE(internal, circuit.bridges.end());
  EXPECT_EQ(std::minmax(internal->from, internal->to),
            std::minmax(indexOf(chip, "Coupler1_2.port3"),
                        indexOf(chip, "Coupler1_2.port4")));
}

TEST(InnerCircuit, RefusesAConfigurationWithoutARing) {
  EXPECT_THROW(static_cast<void>(innerCircuitOf(coupledChip(), ConfigT{})),
               std::invalid_argument);
}

TEST(InnerCircuit, RefusesARingLabelTheChipDoesNotCarry) {
  try {
    static_cast<void>(innerCircuitOf(
        coupledChip(), coupledConfig({"Qb1.port0", "Qb9.port0"})));
    FAIL() << "an unknown ring label has to be refused";
  } catch (const std::invalid_argument& error) {
    EXPECT_NE(std::string(error.what()).find("Qb9.port0"), std::string::npos);
  }
}

TEST(InnerCircuit, RefusesARingThatCarriesBothEndsOfABridge) {
  EXPECT_THROW(
      static_cast<void>(innerCircuitOf(
          coupledChip(), coupledConfig({"Qb1.port0", "Coupler1_2.port1",
                                        "Coupler1_2.port2"}))),
      std::invalid_argument);
}

// The stage, on the two-qubit fixture and on variants of it. The Global stage
// reads only the chains of a capacity plan, so most tests write the plan out
// by hand: that decides which ports share a lattice.

/// One chain of a plan written by hand: target ports, then a launcher.
struct HandChain {
  /// The target ports, in the order the chain links them.
  std::vector<std::uint32_t> targets;
  /// The capacity of a gate between the last target and the launcher, or
  /// none for a chain without a gate.
  std::optional<std::uint32_t> gate;
};

CapacityPlanT planOf(const std::vector<HandChain>& chains) {
  CapacityPlanT plan;
  const auto link = [&plan](const CapacityElement kind, const std::uint32_t id,
                            const std::uint32_t capacity, const bool last) {
    auto node = std::make_unique<CapacityNodeT>();
    node->kind = kind;
    node->id = id;
    node->capacity = capacity;
    if (!last) {
      node->next = {static_cast<std::uint32_t>(plan.nodes.size() + 1)};
    }
    plan.nodes.push_back(std::move(node));
  };
  for (const auto& chain : chains) {
    plan.chains.push_back(static_cast<std::uint32_t>(plan.nodes.size()));
    for (const auto port : chain.targets) {
      link(CapacityElement::Target, port, 0, false);
    }
    if (chain.gate.has_value()) {
      link(CapacityElement::Bottleneck, 0, *chain.gate, false);
    }
    link(CapacityElement::Launcher, 0, 0, true);
  }
  return plan;
}

/// The fixture with Q2.port1 off the ring, so that it is an inner port the
/// circuit has to reach. The component pattern groups the ports by the name
/// in front of the dot.
ConfigT innerConfig() {
  auto config = test::miniConfig();
  config.ports->patterns->component = R"(^([^.]+)\.port\d+$)";
  config.ports->sequences->all_outer = {"Q1.port0", "Q1.port1", "C12.port0",
                                        "Q2.port0"};
  return config;
}

/// The fixture with Q2.port0 off the ring, and two more ports on the coupler
/// bar that a bridge rule pairs: C12.port1 on top, near the inner port, and
/// C12.port2 below it, on the ring.
ConfigT bridgedConfig() {
  auto config = test::miniConfig();
  config.ports->patterns->component = R"(^([^.]+)\.port\d+$)";
  config.ports->patterns->bridge_pair = R"(^C\d+\.port[12]$)";
  auto rule = std::make_unique<BridgeRuleT>();
  rule->first = R"(^(C\d+)\.port1$)";
  rule->second = R"(^(C\d+)\.port2$)";
  config.ports->bridge_pairs.push_back(std::move(rule));
  config.ports->sequences->all_outer = {"Q1.port0", "Q1.port1", "C12.port2",
                                        "C12.port0", "Q2.port1"};
  return config;
}

ChipT bridgedChip(const ConfigT& config = bridgedConfig()) {
  auto text = test::miniChipText();
  const std::string anchor = R"("ports": {)";
  const auto at = text.find(anchor);
  EXPECT_NE(at, std::string::npos);
  text.insert(at + anchor.size(), R"(
    "C12.port1": {"center": [900.0, 270.0], "orientation": 90.0},
    "C12.port2": {"center": [900.0, 130.0], "orientation": 270.0},)");
  return io::loadChip(text, config);
}

/// The bridged fixture with both ends of the bridge off the ring, so that the
/// bridge is an internal crossing.
ConfigT internalBridgeConfig(const bool allowed) {
  auto config = bridgedConfig();
  config.ports->sequences->all_outer = {"Q1.port0", "Q1.port1", "C12.port0",
                                        "Q2.port1"};
  config.stages->global->internal_bridges = allowed;
  return config;
}

/// The chains of the bridged fixture: one through the inner port, the top of
/// the bridge and Q2.port1, and one from the ring end of the bridge to a
/// launcher, through a gate of the given capacity.
CapacityPlanT bridgedPlan(const ChipT& chip,
                          const std::optional<std::uint32_t> gate) {
  return planOf(
      {{.targets = indicesOf(chip, {"Q2.port0", "C12.port1", "Q2.port1"}),
        .gate = std::nullopt},
       {.targets = indicesOf(chip, {"C12.port2"}), .gate = gate}});
}

std::vector<std::uint32_t> ringOf(const GlobalRoutingT& routing) {
  std::vector<std::uint32_t> ring;
  ring.reserve(routing.outer_ring.size());
  for (const auto& port : routing.outer_ring) {
    ring.push_back(port.index());
  }
  return ring;
}

/// The resonators of a routing, which come ascending by port index.
std::vector<std::uint32_t> resonatorsOf(const GlobalRoutingT& routing) {
  std::vector<std::uint32_t> resonators;
  resonators.reserve(routing.resonators.size());
  for (const auto& port : routing.resonators) {
    resonators.push_back(port.index());
  }
  return resonators;
}

/// The length of the lattice edges the circuit selected.
double selectedLength(const GlobalRoutingT& routing) {
  double length = 0.0;
  for (const auto& lattice : routing.lattices) {
    for (const auto edge : lattice->selected) {
      const auto first = static_cast<std::size_t>(edge) * 2;
      length += geometry::distance(lattice->points[lattice->edges[first]],
                                   lattice->points[lattice->edges[first + 1]]);
    }
  }
  return length;
}

/// Everything the stage reports, as a renderer would receive it.
struct Recorder {
  std::vector<std::pair<Detail, std::string>> lines;
  std::vector<std::vector<Figure>> results;
  std::vector<std::optional<std::uint64_t>> fails;

  [[nodiscard]] Report report(const Detail level) {
    return Report(
        level, {.line =
                    [this](const Detail detail, const std::string_view text) {
                      lines.emplace_back(detail, std::string(text));
                    },
                .result =
                    [this](const std::vector<Figure>& figures,
                           const std::optional<std::uint64_t> count) {
                      results.push_back(figures);
                      fails.push_back(count);
                    }});
  }

  [[nodiscard]] bool said(const Detail detail,
                          const std::string_view start) const {
    return std::ranges::any_of(lines, [&](const auto& line) {
      return line.first == detail && line.second.starts_with(start);
    });
  }
};

TEST(GlobalRouter, KeepsTheConfiguredRingWithoutAnInnerCircuit) {
  const auto config = test::miniConfig();
  const auto chip = test::miniChip();
  const auto capacity = makeWatershedPlanner()->run(chip, config, {});
  Recorder recorder;

  const auto routing = makeHananMilpRouter()->run(
      chip, capacity, config, recorder.report(Detail::Summary));

  EXPECT_EQ(ringOf(routing),
            indicesOf(chip, config.ports->sequences->all_outer));
  EXPECT_TRUE(routing.connections.empty());
  EXPECT_TRUE(routing.lattices.empty());
  EXPECT_DOUBLE_EQ(routing.objective, 0.0);
  EXPECT_EQ(resonatorsOf(routing), indicesOf(chip, {"Q1.port0", "Q2.port0"}));

  ASSERT_EQ(recorder.results.size(), 1U);
  EXPECT_EQ(recorder.results[0][0].text, "no inner circuit");
  EXPECT_EQ(recorder.fails[0], 0U);
}

TEST(GlobalRouter, ReachesAnInnerPortAroundTheArtwork) {
  const auto config = innerConfig();
  const auto chip = test::miniChip(config);
  const auto plan = planOf(
      {{.targets = indicesOf(chip, {"Q2.port1", "Q2.port0"}), .gate = {}}});

  const auto routing = makeHananMilpRouter()->run(chip, plan, config, {});

  ASSERT_EQ(routing.connections.size(), 1U);
  ASSERT_NE(routing.connections[0]->source, nullptr);
  EXPECT_EQ(routing.connections[0]->source->index(), indexOf(chip, "Q2.port0"));
  EXPECT_EQ(routing.connections[0]->target.index(), indexOf(chip, "Q2.port1"));
  // The lattice corner inside the pad of Q2 is gone, so the wire runs around
  // the pad: 320 to the right and 320 down, from one anchor to the other.
  EXPECT_NEAR(routing.objective, 640.0, 1.0e-9);
  EXPECT_NEAR(routing.objective, selectedLength(routing), 1.0e-9);
  // No bridge leads anywhere, so the ring is the configured one.
  EXPECT_EQ(ringOf(routing),
            indicesOf(chip, config.ports->sequences->all_outer));
}

TEST(GlobalRouter, CountsATargetThatNoLatticeCarriesAsAFail) {
  // A chain of one port is a single lattice node, which is no lattice.
  const auto config = innerConfig();
  const auto chip = test::miniChip(config);
  const auto plan =
      planOf({{.targets = indicesOf(chip, {"Q2.port1"}), .gate = {}}});
  Recorder recorder;

  const auto routing = makeHananMilpRouter()->run(
      chip, plan, config, recorder.report(Detail::Summary));

  EXPECT_TRUE(routing.connections.empty());
  ASSERT_EQ(recorder.fails.size(), 1U);
  EXPECT_EQ(recorder.fails[0], 1U);
}

TEST(GlobalRouter, SurfacesThroughABridgeWhereTheRingCanCarryTheWire) {
  const auto config = bridgedConfig();
  const auto chip = bridgedChip();

  const auto routing = makeHananMilpRouter()->run(
      chip, bridgedPlan(chip, std::nullopt), config, {});

  // Through the bridge the wire is 450 long, from Q2.port1 it would be 640.
  ASSERT_EQ(routing.connections.size(), 1U);
  EXPECT_EQ(routing.connections[0]->source->index(),
            indexOf(chip, "C12.port1"));
  EXPECT_EQ(routing.connections[0]->target.index(), indexOf(chip, "Q2.port0"));
  EXPECT_NEAR(routing.objective, 450.0, 1.0e-9);
  // The ring keeps the port the wire surfaces at, and since the wire feeds a
  // resonator, the assignment sees that port as a resonator too.
  EXPECT_EQ(ringOf(routing),
            indicesOf(chip, config.ports->sequences->all_outer));
  auto resonators = indicesOf(chip, {"Q1.port0", "C12.port2", "Q2.port0"});
  std::ranges::sort(resonators);
  EXPECT_EQ(resonatorsOf(routing), resonators);
}

TEST(GlobalRouter, DropsTheRingPortOfABridgeThatCannotCarryTheWire) {
  // A gate without room cuts the ring end of the bridge off from every
  // launcher, so the wire has to come from Q2.port1 instead.
  const auto config = bridgedConfig();
  const auto chip = bridgedChip();

  const auto routing =
      makeHananMilpRouter()->run(chip, bridgedPlan(chip, 0U), config, {});

  ASSERT_EQ(routing.connections.size(), 1U);
  EXPECT_EQ(routing.connections[0]->source->index(), indexOf(chip, "Q2.port1"));
  EXPECT_EQ(ringOf(routing),
            indicesOf(chip, {"Q1.port0", "Q1.port1", "C12.port0", "Q2.port1"}));
  EXPECT_EQ(resonatorsOf(routing), indicesOf(chip, {"Q1.port0", "Q2.port0"}));
}

TEST(GlobalRouter, ReachesAnInnerPortPastAnInternalBridge) {
  // The bridge would not shorten the wire. Shut, its two ports carry no flow;
  // open, they could. Either way the wire runs from Q2.port1 around the pad.
  for (const bool allowed : {false, true}) {
    SCOPED_TRACE(allowed ? "internal bridges allowed"
                         : "internal bridges shut");
    const auto config = internalBridgeConfig(allowed);
    const auto chip = bridgedChip(config);
    const auto plan = planOf(
        {{.targets = indicesOf(chip, {"Q2.port0", "C12.port1", "Q2.port1"}),
          .gate = std::nullopt},
         {.targets = indicesOf(chip, {"C12.port2", "Q2.port1"}),
          .gate = std::nullopt}});
    Recorder recorder;

    const auto routing = makeHananMilpRouter()->run(
        chip, plan, config, recorder.report(Detail::Summary));

    ASSERT_EQ(routing.connections.size(), 1U);
    EXPECT_EQ(routing.connections[0]->source->index(),
              indexOf(chip, "Q2.port1"));
    EXPECT_EQ(routing.connections[0]->target.index(),
              indexOf(chip, "Q2.port0"));
    EXPECT_NEAR(routing.objective, 640.0, 1.0e-9);
    ASSERT_EQ(recorder.fails.size(), 1U);
    EXPECT_EQ(recorder.fails[0], 0U);
  }
}

TEST(GlobalRouter, GivesTheSameAnswerTwice) {
  const auto bytes = [] {
    const auto chip = bridgedChip();
    ArtifactT artifact;
    artifact.producer = "test";
    artifact.output.Set(makeHananMilpRouter()->run(
        chip, bridgedPlan(chip, std::nullopt), bridgedConfig(), {}));
    return io::writeArtifact(artifact);
  };
  EXPECT_EQ(bytes(), bytes());
}

TEST(GlobalRouter, ReportsTheCircuitAndHowItWasSolved) {
  const auto config = bridgedConfig();
  const auto chip = bridgedChip();
  Recorder recorder;

  const auto routing =
      makeHananMilpRouter()->run(chip, bridgedPlan(chip, std::nullopt), config,
                                 recorder.report(Detail::Items));

  ASSERT_EQ(recorder.results.size(), 1U);
  const auto& figures = recorder.results[0];
  ASSERT_EQ(figures.size(), 3U);
  EXPECT_EQ(figures[0].text, "1 inner wire");
  EXPECT_EQ(figures[1].text, counted(routing.lattices.size(), "lattice"));
  EXPECT_EQ(figures[2].text, "length 450.00");
  EXPECT_EQ(recorder.fails[0], 0U);

  EXPECT_TRUE(recorder.said(Detail::Steps, "model inner-circuit"));
  EXPECT_TRUE(recorder.said(Detail::Steps, "highs optimal"));
  EXPECT_TRUE(recorder.said(Detail::Items, std::string("C12.port1 ") +
                                               std::string(ARROW) +
                                               " Q2.port0 · length 450.00"));
}

} // namespace
} // namespace mqt::scpd::pipeline
