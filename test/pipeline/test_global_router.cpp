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
#include "mqt-scpd/milp/Backend.hpp"
#include "mqt-scpd/milp/Model.hpp"
#include "mqt-scpd/pipeline/CapacityPlanner.hpp"
#include "mqt-scpd/pipeline/GlobalRouter.hpp"
#include "mqt-scpd/pipeline/Report.hpp"
#include "mqt-scpd/pipeline/Stages.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <format>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

namespace mqt::scpd::pipeline {
namespace {

using flatbuffers::artifacts::ArtifactT;
using flatbuffers::artifacts::CapacityElement;
using flatbuffers::artifacts::CapacityNodeT;
using flatbuffers::config::BridgeRuleT;
using flatbuffers::config::PortPatternsT;
using flatbuffers::design::DesignRulesT;
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

/// The chip of coupledLabels. Without components, no pattern groups the ports
/// and every coupler port is a conventional port.
ChipT coupledChip(const bool components = true) {
  PortPatternsT patterns;
  patterns.launcher = R"(^Chip\.port\d+$)";
  patterns.resonator = R"(^Qb\d+\.port0$)";
  if (components) {
    patterns.conventional = R"(^(Qb\d+\.port1|Coupler\d+_\d+\.port0)$)";
    patterns.bridge_pair = R"(^Coupler\d+_\d+\.port[1-4]$)";
    patterns.component = R"(^([^.]+)\.port\d+$)";
  } else {
    patterns.conventional = R"(^(Qb\d+\.port1|Coupler\d+_\d+\.port\d+)$)";
  }

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

TEST(InnerCircuit, TargetsEveryInnerPortOfNoComponent) {
  const auto chip = coupledChip(false);
  const auto circuit = innerCircuitOf(
      chip, coupledConfig({"Qb1.port0", "Qb1.port1", "Coupler1_2.port0",
                           "Coupler1_2.port2"}));

  // No component groups the ports, so no bridge claims one and each inner
  // port ends a wire.
  EXPECT_TRUE(circuit.bridges.empty());
  EXPECT_EQ(circuit.targets, circuit.innerPorts);
  EXPECT_EQ(circuit.innerPorts,
            indicesOf(chip, {"Qb2.port0", "Qb2.port1", "Coupler1_2.port1",
                             "Coupler1_2.port3", "Coupler1_2.port4"}));
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

using test::bridgedChip;
using test::bridgedConfig;

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

/// One port of a chip placed by hand.
struct HandPort {
  std::string label;
  Point center;
  /// The direction the port faces, in degrees.
  double orientation = 0.0;
};

/// A chip of hand-placed ports and rectangles of artwork.
///
/// The ports are classified by @p patterns. Each rectangle is given by its
/// lower left and upper right corner.
ChipT handChip(const std::vector<HandPort>& ports,
               const PortPatternsT& patterns,
               const std::vector<std::pair<Point, Point>>& rectangles = {}) {
  ChipT chip;
  for (const auto& [label, center, orientation] : ports) {
    auto port = std::make_unique<PortT>();
    port->label = label;
    port->center = center;
    port->orientation = orientation;
    chip.ports.push_back(std::move(port));
  }
  for (const auto& [low, high] : rectangles) {
    auto polygon = std::make_unique<flatbuffers::geometry::PolygonT>();
    polygon->vertices = {low, Point(high.x(), low.y()), high,
                         Point(low.x(), high.y())};
    chip.obstacles.push_back(std::move(polygon));
  }
  EXPECT_TRUE(design::classifyPorts(chip, patterns).empty());
  return chip;
}

/// The patterns of a hand chip. `R.port0`, `R1.port0` and so on are
/// resonators. `S.port0` and `S.port1` are the two ends of a bridge, and
/// every other port named `X.port0` is a conventional port of its own
/// component.
PortPatternsT handPatterns() {
  PortPatternsT patterns;
  patterns.launcher = R"(^Chip\.port\d+$)";
  patterns.resonator = R"(^R\d*\.port0$)";
  patterns.conventional = R"(^[A-QT-Z]\.port0$)";
  patterns.bridge_pair = R"(^S\.port[01]$)";
  patterns.component = R"(^([^.]+)\.port\d+$)";
  return patterns;
}

/// The ring end of the bridge of a hand chip. S.port0 is the inner end, and
/// every inner wire of a hand chip leaves the inner circuit there. The ring
/// end lies on no chain, so no chain capacity limits the bridge.
HandPort ringEnd() {
  return {.label = "S.port1",
          .center = Point(-1000.0, -1000.0),
          .orientation = 180.0};
}

/// The configuration of a hand chip. Without design rules, the lattice lines
/// of a port run through its center. A rule pairs S.port0 with S.port1.
ConfigT handConfig(const std::vector<std::string>& ring) {
  ConfigT config;
  config.ports = std::make_unique<flatbuffers::config::PortConfigT>();
  config.ports->sequences =
      std::make_unique<flatbuffers::config::PortSequencesT>();
  config.ports->sequences->all_outer = ring;
  auto rule = std::make_unique<BridgeRuleT>();
  rule->first = R"(^(S)\.port0$)";
  rule->second = R"(^(S)\.port1$)";
  config.ports->bridge_pairs.push_back(std::move(rule));
  config.stages = std::make_unique<flatbuffers::config::StageParamsT>();
  config.stages->global =
      std::make_unique<flatbuffers::config::GlobalParamsT>();
  config.stages->solver =
      std::make_unique<flatbuffers::config::SolverParamsT>();
  return config;
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

TEST(GlobalRouter, RunsAroundALatticeNodeOnArtwork) {
  // The lattice of S and T is a square. Its corner at (100, 0) lies on
  // artwork, so the node goes, and the wire runs over the other corner.
  const auto chip =
      handChip({ringEnd(),
                {.label = "S.port0", .center = Point(0.0, 0.0)},
                {.label = "T.port0",
                 .center = Point(100.0, 100.0),
                 .orientation = 180.0}},
               handPatterns(), {{Point(90.0, -10.0), Point(110.0, 10.0)}});
  const auto config = handConfig({"S.port1"});
  const auto plan = planOf(
      {{.targets = indicesOf(chip, {"T.port0", "S.port0"}), .gate = {}}});

  const auto routing = makeHananMilpRouter()->run(chip, plan, config, {});

  ASSERT_EQ(routing.connections.size(), 1U);
  ASSERT_NE(routing.connections[0]->source, nullptr);
  EXPECT_EQ(routing.connections[0]->source->index(), indexOf(chip, "S.port0"));
  EXPECT_EQ(routing.connections[0]->target.index(), indexOf(chip, "T.port0"));
  EXPECT_NEAR(routing.objective, 200.0, 1.0e-9);
  EXPECT_NEAR(routing.objective, selectedLength(routing), 1.0e-9);
  ASSERT_EQ(routing.lattices.size(), 1U);
  const auto& lattice = *routing.lattices[0];
  EXPECT_TRUE(std::ranges::any_of(lattice.selected, [&](const auto edge) {
    const auto first = static_cast<std::size_t>(edge) * 2;
    return std::ranges::any_of(
        std::array{lattice.edges[first], lattice.edges[first + 1]},
        [&](const auto node) {
          return geometry::distance(lattice.points[node], Point(0.0, 100.0)) <
                 1.0e-9;
        });
  }));
}

TEST(GlobalRouter, RefusesAPlanThatDoesNotFitTheChip) {
  const auto config = innerConfig();
  const auto chip = test::miniChip(config);
  const auto refusal = [&](const CapacityPlanT& plan) {
    try {
      static_cast<void>(makeHananMilpRouter()->run(chip, plan, config, {}));
    } catch (const std::invalid_argument& error) {
      return std::string(error.what());
    }
    return std::string();
  };
  const auto plan = [&chip] {
    return planOf(
        {{.targets = indicesOf(chip, {"Q2.port1", "Q2.port0"}), .gate = {}}});
  };
  // The plan holds the nodes 0, 1 and 2, in a line.
  const auto ports = static_cast<std::uint32_t>(chip.ports.size());

  auto unknownChain = plan();
  unknownChain.chains.push_back(3);
  EXPECT_EQ(refusal(unknownChain), "the capacity plan does not fit the chip: "
                                   "a chain starts at node 3 of 3");

  auto unknownLink = plan();
  unknownLink.nodes[2]->next = {7};
  EXPECT_EQ(refusal(unknownLink), "the capacity plan does not fit the chip: "
                                  "node 2 leads to node 7 of 3");

  auto unknownPort = plan();
  unknownPort.nodes[0]->id = ports;
  EXPECT_EQ(refusal(unknownPort),
            std::format("the capacity plan does not fit the chip: node 0 "
                        "names port {} of {}",
                        ports, ports));

  auto cycle = plan();
  cycle.nodes[2]->next = {0};
  EXPECT_EQ(refusal(cycle), "the capacity plan does not fit the chip: "
                            "node 2 leads back to node 0");

  // Two chains may share a link, and that is no cycle.
  auto shared = plan();
  shared.chains.push_back(1);
  EXPECT_EQ(refusal(shared), "");
}

TEST(GlobalRouter, RunsNoLatticeEdgeAcrossArtwork) {
  // The lattice of S and T is a square, and a bar of artwork crosses it from
  // the bottom side to the top side. No node of the square lies on the bar,
  // so only the edges themselves can say that no wire fits.
  const auto chip =
      handChip({ringEnd(),
                {.label = "S.port0", .center = Point(0.0, 0.0)},
                {.label = "T.port0",
                 .center = Point(100.0, 100.0),
                 .orientation = 180.0}},
               handPatterns(), {{Point(40.0, -10.0), Point(60.0, 110.0)}});
  const auto config = handConfig({"S.port1"});
  const auto plan = planOf(
      {{.targets = indicesOf(chip, {"T.port0", "S.port0"}), .gate = {}}});
  Recorder recorder;

  const auto routing = makeHananMilpRouter()->run(
      chip, plan, config, recorder.report(Detail::Summary));

  EXPECT_TRUE(routing.connections.empty());
  EXPECT_DOUBLE_EQ(selectedLength(routing), 0.0);
  ASSERT_EQ(recorder.fails.size(), 1U);
  EXPECT_EQ(recorder.fails[0], 1U);
}

TEST(GlobalRouter, LeavesAPortWhoseAnchorLiesOnItsOwnArtwork) {
  // The anchor of S lies inside the square around it. The edge to T leaves
  // the square and crosses nothing else, so the wire runs.
  const auto chip = handChip(
      {ringEnd(),
       {.label = "S.port0", .center = Point(0.0, 0.0)},
       {.label = "T.port0", .center = Point(100.0, 0.0), .orientation = 180.0}},
      handPatterns(), {{Point(-10.0, -10.0), Point(10.0, 10.0)}});
  const auto config = handConfig({"S.port1"});
  const auto plan = planOf(
      {{.targets = indicesOf(chip, {"T.port0", "S.port0"}), .gate = {}}});

  const auto routing = makeHananMilpRouter()->run(chip, plan, config, {});

  ASSERT_EQ(routing.connections.size(), 1U);
  EXPECT_EQ(routing.connections[0]->source->index(), indexOf(chip, "S.port0"));
  EXPECT_EQ(routing.connections[0]->target.index(), indexOf(chip, "T.port0"));
  EXPECT_NEAR(routing.objective, 100.0, 1.0e-9);
}

TEST(GlobalRouter, LeavesAPortWhoseAnchorLiesOnItsOwnArtworkAtTheTarget) {
  // The same exit at the other end of the edge: the anchor of T lies inside
  // the square around it.
  const auto chip = handChip(
      {ringEnd(),
       {.label = "S.port0", .center = Point(0.0, 0.0)},
       {.label = "T.port0", .center = Point(100.0, 0.0), .orientation = 180.0}},
      handPatterns(), {{Point(90.0, -10.0), Point(110.0, 10.0)}});
  const auto config = handConfig({"S.port1"});
  const auto plan = planOf(
      {{.targets = indicesOf(chip, {"T.port0", "S.port0"}), .gate = {}}});

  const auto routing = makeHananMilpRouter()->run(chip, plan, config, {});

  ASSERT_EQ(routing.connections.size(), 1U);
  EXPECT_NEAR(routing.objective, 100.0, 1.0e-9);
}

/// A chip of the ports S and T and one rectangle of artwork between
/// @p low and @p high, and whether the global stage runs a wire from S to T.
bool runsAcross(const Point& source, const Point& target, const Point& low,
                const Point& high) {
  const auto chip =
      handChip({ringEnd(),
                {.label = "S.port0", .center = source},
                {.label = "T.port0", .center = target, .orientation = 180.0}},
               handPatterns(), {{low, high}});
  const auto config = handConfig({"S.port1"});
  const auto plan = planOf(
      {{.targets = indicesOf(chip, {"T.port0", "S.port0"}), .gate = {}}});
  const auto routing = makeHananMilpRouter()->run(chip, plan, config, {});
  return !routing.connections.empty();
}

TEST(GlobalRouter, RunsNoWireFromAPortOnTheBorderAcrossTheArtwork) {
  // S lies on the left side of the square, and the edge crosses the square
  // to its right side. S does not leave the square, so the edge is blocked.
  EXPECT_FALSE(runsAcross(Point(-10.0, 0.0), Point(100.0, 0.0),
                          Point(-10.0, -10.0), Point(10.0, 10.0)));
}

TEST(GlobalRouter, RunsNoWireBetweenPortsOnOppositeSidesOfTheArtwork) {
  // S and T lie on opposite sides of the rectangle, and the edge runs
  // through it from one side to the other.
  EXPECT_FALSE(runsAcross(Point(0.0, 0.0), Point(100.0, 0.0), Point(0.0, -10.0),
                          Point(100.0, 10.0)));
}

TEST(GlobalRouter, RunsNoWireThatStaysInsideTheArtwork) {
  // S and T both lie inside the rectangle, and the edge never leaves it.
  EXPECT_FALSE(runsAcross(Point(1.0, 0.0), Point(101.0, 0.0),
                          Point(-10.0, -10.0), Point(110.0, 10.0)));
}

/// Whether the global stage runs a wire from S to T when every anchor lies
/// @p reach in front of its port. S faces along +x, T along -x, and the one
/// rectangle of artwork lies between @p low and @p high.
bool runsWithReach(const Point& source, const Point& target, const Point& low,
                   const Point& high, const double reach) {
  const auto chip =
      handChip({ringEnd(),
                {.label = "S.port0", .center = source},
                {.label = "T.port0", .center = target, .orientation = 180.0}},
               handPatterns(), {{low, high}});
  auto config = handConfig({"S.port1"});
  config.rules = std::make_unique<DesignRulesT>();
  config.rules->min_straight_length = reach;
  const auto plan = planOf(
      {{.targets = indicesOf(chip, {"T.port0", "S.port0"}), .gate = {}}});
  const auto routing = makeHananMilpRouter()->run(chip, plan, config, {});
  return !routing.connections.empty();
}

TEST(GlobalRouter, LeavesTheArtworkOfThePortInFrontOfIt) {
  // S lies inside its square, and its anchor 10 in front of it as well. The
  // edge to T leaves the square the way S faces.
  EXPECT_TRUE(runsWithReach(Point(0.0, 0.0), Point(100.0, 0.0),
                            Point(-20.0, -20.0), Point(20.0, 20.0), 10.0));
}

TEST(GlobalRouter, RunsNoWireFromAnAnchorOnTheArtworkOfAnotherComponent) {
  // The anchor of S lies 10 in front of S, inside a square that S does not
  // touch. The square is artwork of another component, and the edge to T
  // crosses it.
  EXPECT_FALSE(runsWithReach(Point(0.0, 0.0), Point(100.0, 0.0),
                             Point(5.0, -5.0), Point(15.0, 5.0), 10.0));
}

TEST(GlobalRouter, RunsNoWireSidewaysOutOfTheArtworkOfThePort) {
  // S and T face the same way, and their anchors lie 100 in front of them.
  // The edge between the anchors runs at a right angle to the way S faces,
  // through the square that S lies in. It does not leave the square the way
  // S faces. In each case, the cosine of the angle between the edge and the
  // way S faces is a positive rounding error instead of zero.
  struct Case {
    double orientation;
    Point target;
  };
  for (const auto& [orientation, target] :
       {Case{.orientation = 90.0, .target = Point(5000.0, 0.0)},
        Case{.orientation = 180.0, .target = Point(0.0, 5000.0)},
        Case{.orientation = 270.0, .target = Point(-5000.0, 0.0)}}) {
    const auto chip = handChip(
        {ringEnd(),
         {.label = "S.port0",
          .center = Point(0.0, 0.0),
          .orientation = orientation},
         {.label = "T.port0", .center = target, .orientation = orientation}},
        handPatterns(), {{Point(-500.0, -500.0), Point(500.0, 500.0)}});
    auto config = handConfig({"S.port1"});
    config.rules = std::make_unique<DesignRulesT>();
    config.rules->min_straight_length = 100.0;
    const auto plan = planOf(
        {{.targets = indicesOf(chip, {"T.port0", "S.port0"}), .gate = {}}});

    const auto routing = makeHananMilpRouter()->run(chip, plan, config, {});

    EXPECT_TRUE(routing.connections.empty()) << "S faces " << orientation;
  }
}

TEST(GlobalRouter, RunsNoWireBackThroughTheArtworkOfThePort) {
  // S faces right, and T lies to its left. The edge from the anchor of S to
  // T runs back through the square that S lies on.
  EXPECT_FALSE(runsWithReach(Point(0.0, 0.0), Point(-100.0, 0.0),
                             Point(-20.0, -10.0), Point(10.0, 10.0), 5.0));
}

TEST(GlobalRouter, RunsOneWireFromAPortOnSeveralLattices) {
  // S borders two chains, so it is a node of two lattices. A port takes one
  // wire, so only one of A and B is reached.
  const auto chip =
      handChip({ringEnd(),
                {.label = "S.port0", .center = Point(0.0, 0.0)},
                {.label = "A.port0", .center = Point(100.0, 0.0)},
                {.label = "B.port0", .center = Point(0.0, 100.0)}},
               handPatterns());
  const auto config = handConfig({"S.port1"});
  const auto plan = planOf(
      {{.targets = indicesOf(chip, {"S.port0", "A.port0"}), .gate = {}},
       {.targets = indicesOf(chip, {"S.port0", "B.port0"}), .gate = {}}});
  Recorder recorder;

  const auto routing = makeHananMilpRouter()->run(
      chip, plan, config, recorder.report(Detail::Summary));

  ASSERT_EQ(routing.connections.size(), 1U);
  EXPECT_EQ(routing.connections[0]->source->index(), indexOf(chip, "S.port0"));
  ASSERT_EQ(recorder.fails.size(), 1U);
  EXPECT_EQ(recorder.fails[0], 1U);
}

TEST(GlobalRouter, DrawsOneLatticeLineThroughAnchorsThatDifferInTheLastBits) {
  // Each anchor lies min_straight_length in front of its port. S faces up
  // and A faces down, so both anchors lie on x = 0, but the cosine of 90 and
  // 270 degrees leaves them about 1e-14 apart. They share one column of the
  // lattice, not two columns that let two wires run on one line.
  const auto chip = handChip(
      {ringEnd(),
       {.label = "S.port0", .center = Point(0.0, 0.0), .orientation = 90.0},
       {.label = "A.port0", .center = Point(0.0, 300.0), .orientation = 270.0}},
      handPatterns());
  auto config = handConfig({"S.port1"});
  config.rules = std::make_unique<DesignRulesT>();
  config.rules->min_straight_length = 50.0;
  const auto plan = planOf(
      {{.targets = indicesOf(chip, {"A.port0", "S.port0"}), .gate = {}}});

  const auto routing = makeHananMilpRouter()->run(chip, plan, config, {});

  ASSERT_EQ(routing.lattices.size(), 1U);
  EXPECT_EQ(routing.lattices[0]->points.size(), 2U);
  ASSERT_EQ(routing.connections.size(), 1U);
  EXPECT_NEAR(selectedLength(routing), 200.0, 1.0e-9);
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

TEST(GlobalRouter, CountsAResonatorAsAFailWhereNoRingBridgeCanCarryItsWire) {
  // A gate without room cuts the ring end of the bridge off from every
  // launcher. Q2.port1 lies on the same lattice as the resonator Q2.port0,
  // but it is a ring port and supplies no inner wire, so the resonator stays
  // unreached.
  const auto config = bridgedConfig();
  const auto chip = bridgedChip();
  Recorder recorder;

  const auto routing = makeHananMilpRouter()->run(
      chip, bridgedPlan(chip, 0U), config, recorder.report(Detail::Summary));

  EXPECT_TRUE(routing.connections.empty());
  EXPECT_EQ(ringOf(routing),
            indicesOf(chip, {"Q1.port0", "Q1.port1", "C12.port0", "Q2.port1"}));
  EXPECT_EQ(resonatorsOf(routing), indicesOf(chip, {"Q1.port0", "Q2.port0"}));
  ASSERT_EQ(recorder.fails.size(), 1U);
  EXPECT_EQ(recorder.fails[0], 1U);
}

TEST(GlobalRouter, SurfacesThroughABridgeThatAnotherChainCutsOff) {
  // The ring end of the bridge borders a second chain, whose gate has no
  // room. The first chain still carries the wire to a launcher, so the wire
  // surfaces there as it does without the second chain.
  const auto config = bridgedConfig();
  const auto chip = bridgedChip();
  const auto plan = planOf(
      {{.targets = indicesOf(chip, {"Q2.port0", "C12.port1", "Q2.port1"}),
        .gate = std::nullopt},
       {.targets = indicesOf(chip, {"C12.port2"}), .gate = std::nullopt},
       {.targets = indicesOf(chip, {"C12.port2"}), .gate = 0U}});
  Recorder recorder;

  const auto routing = makeHananMilpRouter()->run(
      chip, plan, config, recorder.report(Detail::Summary));

  ASSERT_EQ(routing.connections.size(), 1U);
  EXPECT_EQ(routing.connections[0]->source->index(),
            indexOf(chip, "C12.port1"));
  EXPECT_NEAR(routing.objective, 450.0, 1.0e-9);
  ASSERT_EQ(recorder.fails.size(), 1U);
  EXPECT_EQ(recorder.fails[0], 0U);
}

TEST(GlobalRouter, CountsARingPortAsAFailWhereItsChainHasNoRoom) {
  // Q1.port0 and Q1.port1 lie behind a gate that passes one wire. The inner
  // circuit still solves as it does without them, and the ring port that the
  // chain has no room for counts as a fail.
  const auto config = bridgedConfig();
  const auto chip = bridgedChip();
  auto plan = planOf(
      {{.targets = indicesOf(chip, {"Q2.port0", "C12.port1", "Q2.port1"}),
        .gate = std::nullopt},
       {.targets = indicesOf(chip, {"C12.port2"}), .gate = std::nullopt},
       {.targets = indicesOf(chip, {"Q1.port0", "Q1.port1"}), .gate = 1U}});
  Recorder recorder;

  const auto routing = makeHananMilpRouter()->run(
      chip, plan, config, recorder.report(Detail::Items));

  ASSERT_EQ(routing.connections.size(), 1U);
  EXPECT_NEAR(routing.objective, 450.0, 1.0e-9);
  ASSERT_EQ(recorder.fails.size(), 1U);
  EXPECT_EQ(recorder.fails[0], 1U);
  EXPECT_TRUE(recorder.said(Detail::Steps, "1 ring port without room"));
  EXPECT_TRUE(recorder.said(Detail::Items, "Q1.port0 has no room") ||
              recorder.said(Detail::Items, "Q1.port1 has no room"));
}

TEST(GlobalRouter, CountsAResonatorAsAFailWithoutARingBridge) {
  // Both ends of the bridge lie off the ring, so no wire can surface on the
  // ring. Shut, the bridge carries no flow; open, it leads only to the other
  // inner end. Q2.port1 is a ring port and supplies no inner wire. Either way
  // the resonator Q2.port0 stays unreached.
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

    EXPECT_TRUE(routing.connections.empty());
    EXPECT_EQ(resonatorsOf(routing), indicesOf(chip, {"Q1.port0", "Q2.port0"}));
    ASSERT_EQ(recorder.fails.size(), 1U);
    EXPECT_EQ(recorder.fails[0], 1U);
  }
}

/// The ports of a hand chip whose inner target @p target lies 100 from the
/// ring port @p ring and 1000 from S.port0, the inner end of the ring bridge.
std::vector<HandPort> decoyedPorts(const std::string& target,
                                   const std::string& ring) {
  std::vector<HandPort> ports{
      {.label = target, .center = Point(0.0, 0.0)},
      {.label = "S.port0", .center = Point(0.0, 1000.0), .orientation = 270.0},
      {.label = ring, .center = Point(100.0, 0.0), .orientation = 180.0}};
  if (ring != "S.port1") {
    ports.push_back(ringEnd());
  }
  return ports;
}

TEST(GlobalRouter, TakesEveryInnerWireFromTheInnerEndOfARingBridge) {
  // A ring port has a wire of the assignment: to a launcher, to a feedline,
  // or, at the ring end of a bridge, the wire that surfaces there. So no ring
  // port supplies an inner wire, however close it lies. The wire of each
  // target comes from S.port0 and surfaces at S.port1. For a resonator,
  // S.port1 is then a resonator of the assignment.
  for (const std::string target : {"R.port0", "T.port0"}) {
    for (const std::string ring : {"A.port0", "R1.port0", "S.port1"}) {
      SCOPED_TRACE(target + " beside the ring port " + ring);
      const auto chip = handChip(decoyedPorts(target, ring), handPatterns());
      auto labels = std::vector<std::string>{ring};
      if (ring != "S.port1") {
        labels.emplace_back("S.port1");
      }
      const auto config = handConfig(labels);
      const auto plan =
          planOf({{.targets = indicesOf(chip, {target, ring, "S.port0"}),
                   .gate = {}}});
      Recorder recorder;

      const auto routing = makeHananMilpRouter()->run(
          chip, plan, config, recorder.report(Detail::Summary));

      ASSERT_EQ(routing.connections.size(), 1U);
      ASSERT_NE(routing.connections[0]->source, nullptr);
      EXPECT_EQ(routing.connections[0]->source->index(),
                indexOf(chip, "S.port0"));
      EXPECT_EQ(routing.connections[0]->target.index(), indexOf(chip, target));
      EXPECT_NEAR(routing.objective, 1000.0, 1.0e-9);
      EXPECT_EQ(ringOf(routing), indicesOf(chip, labels));
      std::vector<std::string> resonators;
      if (target == "R.port0") {
        resonators = {target, "S.port1"};
      }
      if (ring == "R1.port0") {
        resonators.push_back(ring);
      }
      auto expected = indicesOf(chip, resonators);
      std::ranges::sort(expected);
      EXPECT_EQ(resonatorsOf(routing), expected);
      ASSERT_EQ(recorder.fails.size(), 1U);
      EXPECT_EQ(recorder.fails[0], 0U);
    }
  }
}

TEST(GlobalRouter, CountsAnInnerTargetAsAFailWhereOnlyARingPortReachesIt) {
  // The lattice carries the target and a ring port, but no inner end of a
  // bridge. The target stays unreached, and the ring end of the bridge
  // leaves the ring. A resonator is not fed, and the routing says so.
  for (const std::string target : {"R.port0", "T.port0"}) {
    for (const std::string ring : {"A.port0", "R1.port0"}) {
      SCOPED_TRACE(target + " beside the ring port " + ring);
      const auto chip = handChip(decoyedPorts(target, ring), handPatterns());
      const auto config = handConfig({ring, "S.port1"});
      const auto plan =
          planOf({{.targets = indicesOf(chip, {target, ring}), .gate = {}}});
      Recorder recorder;

      const auto routing = makeHananMilpRouter()->run(
          chip, plan, config, recorder.report(Detail::Summary));

      EXPECT_TRUE(routing.connections.empty());
      EXPECT_EQ(ringOf(routing), indicesOf(chip, {ring}));
      std::vector<std::string> resonators;
      if (target == "R.port0") {
        resonators.push_back(target);
      }
      if (ring == "R1.port0") {
        resonators.push_back(ring);
      }
      auto expected = indicesOf(chip, resonators);
      std::ranges::sort(expected);
      EXPECT_EQ(resonatorsOf(routing), expected);
      ASSERT_EQ(recorder.fails.size(), 1U);
      EXPECT_EQ(recorder.fails[0], 1U);
    }
  }
}

TEST(GlobalRouter, MarksTheRingPortOfAResonatorWirePastAnInternalBridge) {
  // The resonator T is fed through two bridges. The wire runs from the inner
  // end X.port1 of the ring bridge to I.port1, crosses the internal bridge,
  // and runs on from I.port2 to T. It surfaces at X.port2, so the assignment
  // has to see X.port2 as a resonator. The chip and its mirror image list
  // the lattice nodes of each wire in opposite orders.
  PortPatternsT patterns;
  patterns.launcher = R"(^Chip\.port\d+$)";
  patterns.resonator = R"(^[RT]\.port0$)";
  patterns.conventional = R"(^C\.port0$)";
  patterns.bridge_pair = R"(^[XI]\.port[12]$)";
  patterns.component = R"(^([^.]+)\.port\d+$)";
  for (const double side : {1.0, -1.0}) {
    SCOPED_TRACE(side > 0.0 ? "the chip" : "its mirror image");
    const auto facing = side > 0.0 ? 180.0 : 0.0;
    const auto chip =
        handChip({{.label = "X.port1", .center = Point(0.0, 0.0)},
                  {.label = "X.port2",
                   .center = Point(0.0, -50.0),
                   .orientation = 270.0},
                  {.label = "I.port1",
                   .center = Point(side * 10.0, 0.0),
                   .orientation = facing},
                  {.label = "I.port2", .center = Point(side * 90.0, 0.0)},
                  {.label = "T.port0",
                   .center = Point(side * 100.0, 0.0),
                   .orientation = facing},
                  {.label = "R.port0", .center = Point(side * 500.0, 500.0)}},
                 patterns);
    auto config = handConfig({"X.port2", "R.port0"});
    config.stages->global->internal_bridges = true;
    auto rule = std::make_unique<BridgeRuleT>();
    rule->first = R"(^([XI])\.port1$)";
    rule->second = R"(^([XI])\.port2$)";
    config.ports->bridge_pairs.push_back(std::move(rule));
    const auto plan = planOf(
        {{.targets = indicesOf(chip, {"X.port1", "I.port1", "T.port0"}),
          .gate = std::nullopt},
         {.targets = indicesOf(chip, {"I.port2", "T.port0"}),
          .gate = std::nullopt},
         {.targets = indicesOf(chip, {"X.port2"}), .gate = std::nullopt}});

    const auto routing = makeHananMilpRouter()->run(chip, plan, config, {});

    ASSERT_EQ(routing.connections.size(), 2U);
    // Each connection runs the way the wire does, from X.port1 towards T.
    std::set<std::pair<std::uint32_t, std::uint32_t>> connections;
    for (const auto& connection : routing.connections) {
      ASSERT_NE(connection->source, nullptr);
      connections.emplace(connection->source->index(),
                          connection->target.index());
    }
    EXPECT_EQ(connections,
              (std::set<std::pair<std::uint32_t, std::uint32_t>>{
                  {indexOf(chip, "X.port1"), indexOf(chip, "I.port1")},
                  {indexOf(chip, "I.port2"), indexOf(chip, "T.port0")}}));
    EXPECT_NEAR(routing.objective, 20.0, 1.0e-9);
    EXPECT_EQ(ringOf(routing), indicesOf(chip, {"X.port2", "R.port0"}));
    auto resonators = indicesOf(chip, {"X.port2", "T.port0", "R.port0"});
    std::ranges::sort(resonators);
    EXPECT_EQ(resonatorsOf(routing), resonators);
  }
}

/// A backend that answers with the longest circuit instead of the shortest.
///
/// A solve that stops at a gap or a time limit returns a feasible solution
/// that is not the shortest one. This backend returns such a solution on
/// purpose: it rewards every edge that the stage charges for. The largest
/// coefficient is the penalty of an unreached target, and it keeps its sign,
/// so the circuit still reaches every target it can.
class LongestCircuitBackend final : public milp::ISolverBackend {
public:
  [[nodiscard]] std::string_view name() const override { return "longest"; }

  [[nodiscard]] milp::Solution
  solve(const milp::Model& model,
        const milp::SolveOptions& options) const override {
    double penalty = 0.0;
    for (const auto& variable : model.variables()) {
      penalty = std::max(penalty, variable.objective);
    }
    milp::Model longest(model.name());
    for (const auto& variable : model.variables()) {
      const auto objective = variable.objective == penalty
                                 ? variable.objective
                                 : -variable.objective;
      std::ignore =
          longest.addVariable(variable.name, variable.lower, variable.upper,
                              variable.type, objective);
    }
    for (const auto& constraint : model.constraints()) {
      milp::LinearExpr expression;
      for (const auto& [index, coefficient] : constraint.coefficients) {
        expression.add(milp::Var{.index = index}, coefficient);
      }
      longest.addRange(constraint.name, expression, constraint.lower,
                       constraint.upper);
    }
    longest.setSense(model.sense());
    return milp::makeHighsBackend()->solve(longest, options);
  }
};

TEST(GlobalRouter, NamesEveryReachedTargetInAFeasibleSolution) {
  // One lattice carries the resonator, the inner end of the bridge and three
  // ring ports. The longest circuit runs wires through every node it may. A
  // ring port carries no inner wire, and a port that a wire ran on through
  // would join two wires into one component with three port ends, whose
  // target would get no connection.
  auto config = bridgedConfig();
  config.stages->solver->backend = "gurobi";
  const auto chip = bridgedChip(config);
  const auto plan =
      planOf({{.targets = indicesOf(chip, {"Q2.port0", "C12.port1", "Q2.port1",
                                           "C12.port0", "Q1.port1"}),
               .gate = {}},
              {.targets = indicesOf(chip, {"C12.port2"}), .gate = {}}});
  milp::setExternalBackend(std::make_shared<LongestCircuitBackend>());
  Recorder recorder;

  const auto routing = makeHananMilpRouter()->run(
      chip, plan, config, recorder.report(Detail::Summary));
  milp::setExternalBackend(nullptr);

  ASSERT_EQ(recorder.fails.size(), 1U);
  EXPECT_EQ(recorder.fails[0], 0U);
  ASSERT_EQ(routing.connections.size(), 1U);
  ASSERT_NE(routing.connections[0]->source, nullptr);
  EXPECT_EQ(routing.connections[0]->source->index(),
            indexOf(chip, "C12.port1"));
  EXPECT_EQ(routing.connections[0]->target.index(), indexOf(chip, "Q2.port0"));
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
