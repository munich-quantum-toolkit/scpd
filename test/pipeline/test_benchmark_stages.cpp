/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// The three planning stages on the benchmark inputs the repository carries.
// These are the tests that say whether the stages run at all, on real
// geometry rather than on a fixture built to make them run.

#include "mqt-scpd/flatbuffers/artifacts.hpp"
#include "mqt-scpd/flatbuffers/config.hpp"
#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/geometry/Geometry.hpp"
#include "mqt-scpd/io/Chip.hpp"
#include "mqt-scpd/pipeline/Assigner.hpp"
#include "mqt-scpd/pipeline/CapacityPlanner.hpp"
#include "mqt-scpd/pipeline/GlobalRouter.hpp"
#include "mqt-scpd/grid/Bottlenecks.hpp"
#include "mqt-scpd/grid/DistanceTransform.hpp"
#include "mqt-scpd/grid/Partitions.hpp"
#include "mqt-scpd/grid/Voronoi.hpp"
#include "mqt-scpd/pipeline/Registry.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <fstream>
#include <functional>
#include <iterator>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace mqt::scpd::pipeline {
namespace {

using flatbuffers::config::ConfigT;
using flatbuffers::design::ChipT;

const std::string BENCHMARKS = MQT_SCPD_BENCHMARK_DIR;

std::string readFile(const std::string& path) {
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
std::vector<std::string> readArray(const std::string& text, const std::string& key) {
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
std::uint32_t readScalar(const std::string& text, const std::string& key,
                         const std::uint32_t fallback) {
  const auto start = text.find(key + " = ");
  if (start == std::string::npos) {
    return fallback;
  }
  return static_cast<std::uint32_t>(std::stoul(text.substr(start + key.size() + 3)));
}

/// One declared bridge rule, as the two expressions the configuration pairs on.
using RulePair = std::pair<std::string, std::string>;

Benchmark load(const std::string& chip, const std::string& resonator,
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
Benchmark qubitAndCouplerChip(const std::string& chip, const bool internalBridges = false) {
  return load(chip, R"(^Qb\d+\.port0$)", R"(^(Qb\d+\.port1|Coupler\d+_\d+\.port0)$)",
              R"(^Coupler\d+_\d+\.port[1-4]$)",
              {{R"(^(Coupler\d+_\d+)\.port1$)", R"(^(Coupler\d+_\d+)\.port2$)"},
               {R"(^(Coupler\d+_\d+)\.port3$)", R"(^(Coupler\d+_\d+)\.port4$)"}},
              internalBridges);
}

/// The 4-qubit chip, whose qubits are `Q<n>` and whose couplers are `C<a><b>`
/// with three ports each. That naming difference is what the role patterns are
/// configuration for, so it stays in the fixture.
Benchmark fourQubit() {
  return load("4q", R"(^Q\d+\.port0$)", R"(^(Q\d+\.port1|C\d+\.port0)$)",
              R"(^C\d+\.port[12]$)", {{R"(^(C\d+)\.port1$)", R"(^(C\d+)\.port2$)"}});
}

Benchmark seventeenQubit() { return qubitAndCouplerChip("17q"); }

Benchmark nineQubit(const bool internalBridges = false) {
  return qubitAndCouplerChip("9q", internalBridges);
}

/// The benchmark chip a directory name stands for.
Benchmark benchmarkOf(const std::string& chip) {
  return chip == "4q" ? fourQubit() : qubitAndCouplerChip(chip);
}

/// Rule 1: every ring node is reached. The assignment carries one connection
/// per port of the ring, and no port twice.
void expectEveryRingNodeIsReached(const Benchmark& benchmark, const AssignmentT& assignment) {
  ASSERT_EQ(assignment.connections.size(), assignment.ring.size());
  std::set<std::uint32_t> reached;
  for (const auto& connection : assignment.connections) {
    const auto port = connection->target.index();
    EXPECT_TRUE(reached.insert(port).second)
        << benchmark.chip.ports[port]->label << " is reached twice";
  }
  for (const auto& node : assignment.ring) {
    EXPECT_TRUE(reached.contains(node.index()))
        << benchmark.chip.ports[node.index()]->label << " carries no connection";
  }
}

/// Rule 2: a launcher feeds conventional ports, at most one each. What stands
/// on a launcher slot is a conventional port and nothing else, because every
/// resonator moved onto the segment between two slots.
void expectALauncherFeedsOneConventionalPort(const Benchmark& benchmark,
                                             const CapacityPlanT& plan,
                                             const GlobalRoutingT& global,
                                             const AssignmentT& assignment) {
  ASSERT_EQ(assignment.feeds.size(), assignment.ring.size());
  std::set<std::uint32_t> resonators;
  for (const auto& port : global.resonators) {
    resonators.insert(port.index());
  }

  std::vector<std::size_t> fed(plan.launchers.size(), 0);
  for (std::size_t index = 0; index < assignment.ring.size(); ++index) {
    const auto& feed = assignment.feeds[index];
    const auto slot = std::ranges::find_if(plan.launchers, [&](const auto& candidate) {
      return geometry::distance(feed, candidate->position) < 1e-6;
    });
    const auto port = assignment.ring[index].index();
    if (slot == plan.launchers.end()) {
      EXPECT_TRUE(resonators.contains(port))
          << benchmark.chip.ports[port]->label << " is a conventional port fed from no launcher";
      continue;
    }
    EXPECT_FALSE(resonators.contains(port))
        << benchmark.chip.ports[port]->label << " is a resonator left on a launcher";
    ++fed[static_cast<std::size_t>(std::distance(plan.launchers.begin(), slot))];
  }
  for (std::size_t slot = 0; slot < fed.size(); ++slot) {
    EXPECT_LE(fed[slot], 1U) << benchmark.chip.ports[plan.launchers[slot]->port.index()]->label
                             << " feeds " << fed[slot] << " ports";
  }
}

/// Rule 3: the ring keeps its cyclic order. The launcher slots along the ring
/// fall, and come back up once where the walk closes its turn of the launcher
/// ring. A second rise means the walk turned twice, and then two ring nodes
/// share a launcher.
void expectTheRingKeepsItsCyclicOrder(const CapacityPlanT& plan, const AssignmentT& assignment) {
  ASSERT_FALSE(assignment.launchers.empty());
  std::map<std::uint32_t, std::size_t> slotOf;
  for (std::size_t index = 0; index < plan.launchers.size(); ++index) {
    slotOf.emplace(plan.launchers[index]->port.index(), index);
  }

  std::vector<std::size_t> walk;
  walk.reserve(assignment.launchers.size());
  for (const auto& given : assignment.launchers) {
    const auto slot = slotOf.find(given.index());
    ASSERT_NE(slot, slotOf.end());
    walk.push_back(slot->second);
  }
  std::size_t rises = 0;
  for (std::size_t index = 0; index < walk.size(); ++index) {
    if (walk[(index + 1) % walk.size()] > walk[index]) {
      ++rises;
    }
  }
  EXPECT_EQ(rises, 1U) << "the walk turns the launcher ring " << rises << " times, not once";
}

TEST(BenchmarkStages, PartitionsTheFourQubitChip) {
  const auto benchmark = fourQubit();
  const auto plan = capacityPlanners().make("watershed")->run(benchmark.chip, benchmark.config);

  ASSERT_NE(plan.capacity_grid, nullptr);
  EXPECT_EQ(plan.capacity_grid->width, 12U);
  EXPECT_EQ(plan.detail_grid->width, 12U * 30U);
  // Every launcher of the chip gets a slot the wires enter through.
  EXPECT_FALSE(plan.launchers.empty());
  // The free space is divided, and the divisions meet somewhere.
  EXPECT_FALSE(plan.partitions.empty());
  EXPECT_FALSE(plan.borders.empty());
  // Every chain is rooted at a target and every node index it names exists.
  EXPECT_FALSE(plan.chains.empty());
  for (const auto root : plan.chains) {
    ASSERT_LT(root, plan.nodes.size());
    EXPECT_EQ(plan.nodes[root]->kind, flatbuffers::artifacts::CapacityElement::Target);
  }
  for (const auto& node : plan.nodes) {
    for (const auto next : node->next) {
      EXPECT_LT(next, plan.nodes.size());
    }
  }
}

TEST(BenchmarkStages, KeepsOnlyTheBottlenecksItsChainsCross) {
  // A candidate the saddle search found and the chains then walked past
  // constrains nothing. The artifact carries the gates that bind and no
  // others, so every bottleneck it holds is named by some chain.
  const auto benchmark = nineQubit();
  const auto plan = capacityPlanners().make("watershed")->run(benchmark.chip, benchmark.config);

  ASSERT_FALSE(plan.bottlenecks.empty());
  std::set<std::uint32_t> named;
  for (const auto& node : plan.nodes) {
    if (node->kind == flatbuffers::artifacts::CapacityElement::Bottleneck) {
      named.insert(node->id);
      EXPECT_LT(node->id, plan.bottlenecks.size());
    }
  }
  EXPECT_EQ(named.size(), plan.bottlenecks.size());

  // The pruning is what makes the difference: the raw candidate set is much
  // larger than what survives on the chains. It is not as large as it once
  // was, because the search now reports one line per narrowing rather than
  // one per cell of a plateau.
  const auto scene = buildScene(benchmark.chip, benchmark.config);
  const auto axis =
      grid::rasterizeMedialAxis(scene.blocked, scene.detail, grid::medialAxis(scene.blocked));
  const auto candidates =
      grid::findBottlenecks(scene.blocked, axis, grid::squaredDistanceTransform(scene.blocked),
                            scene.detail, {.targets = scene.targetCell});
  EXPECT_GT(candidates.size(), plan.bottlenecks.size() * 3);
}

TEST(BenchmarkStages, LeavesNoGateWithoutSomethingBeyondIt) {
  // A gate with nothing past it is a place the walk could not get through, so
  // no wire ever passes it and it is dropped. Two gates in a line are one
  // gate, so no gate has a lone gate as its only child either.
  const auto benchmark = nineQubit();
  const auto plan = capacityPlanners().make("watershed")->run(benchmark.chip, benchmark.config);

  for (const auto& node : plan.nodes) {
    if (node->kind != flatbuffers::artifacts::CapacityElement::Bottleneck) {
      continue;
    }
    ASSERT_FALSE(node->next.empty());
    if (node->next.size() == 1) {
      EXPECT_NE(plan.nodes[node->next.front()]->kind,
                flatbuffers::artifacts::CapacityElement::Bottleneck);
    }
  }
}

TEST(BenchmarkStages, PartitionsMoreThanTheClearCapacityCellsAlone) {
  // The partitioning is what the gates carve out: every chamber a chain walks
  // through is a partition, and the watershed over the clear capacity cells
  // only fills what no chamber claimed. A run that produced the seeds alone
  // would have at most one partition per clear cell.
  const auto benchmark = nineQubit();
  const auto plan = capacityPlanners().make("watershed")->run(benchmark.chip, benchmark.config);
  const auto scene = buildScene(benchmark.chip, benchmark.config);
  const auto seeds = grid::freeCellSeeds(scene.blocked, scene.detail, scene.capacity);

  EXPECT_GT(plan.partitions.size(), seeds.size());
}

TEST(BenchmarkStages, PartitionsTheNineQubitChip) {
  const auto benchmark = nineQubit();
  const auto plan = capacityPlanners().make("watershed")->run(benchmark.chip, benchmark.config);
  EXPECT_FALSE(plan.partitions.empty());
  EXPECT_FALSE(plan.chains.empty());
  EXPECT_FALSE(plan.launchers.empty());
}

TEST(BenchmarkStages, TheFourQubitChipHasNoInnerCircuit) {
  // Its outer ring is its entire port ring, which makes the global stage a
  // no-op. That is a valid pipeline state and the artifact is written empty.
  const auto benchmark = fourQubit();
  const auto circuit = innerCircuitOf(benchmark.chip, benchmark.config);
  EXPECT_TRUE(circuit.targets.empty());

  const auto plan = capacityPlanners().make("watershed")->run(benchmark.chip, benchmark.config);
  const auto global =
      globalRouters().make("hanan-milp")->run(benchmark.chip, plan, benchmark.config);
  EXPECT_TRUE(global.lattices.empty());
  EXPECT_TRUE(global.connections.empty());
  EXPECT_FALSE(global.outer_ring.empty());
  EXPECT_FALSE(global.resonators.empty());
}

TEST(BenchmarkStages, TheNineQubitChipHasAnInnerCircuitToSolve) {
  const auto benchmark = nineQubit();
  const auto circuit = innerCircuitOf(benchmark.chip, benchmark.config);
  EXPECT_FALSE(circuit.innerPorts.empty());
  EXPECT_FALSE(circuit.targets.empty());
  // Its inner couplers bridge, so a wire can cross their artwork.
  EXPECT_FALSE(circuit.bridges.empty());
}

TEST(BenchmarkStages, ShutsTheInternalBridgesUnlessTheConfigurationOpensThem) {
  // A bridge whose far end the ring does not carry lets a wire cross a
  // component the assignment never sees, so nothing downstream could say
  // where that wire runs. Both of its ports are stilled by default.
  const auto benchmark = nineQubit();
  const auto circuit = innerCircuitOf(benchmark.chip, benchmark.config);

  std::set<std::uint32_t> surfacing;
  for (const auto& bridge : circuit.outerBridges) {
    surfacing.insert(bridge.from);
    surfacing.insert(bridge.to);
  }
  std::set<std::uint32_t> internal;
  for (const auto& bridge : circuit.bridges) {
    if (!surfacing.contains(bridge.from) && !surfacing.contains(bridge.to)) {
      internal.insert(bridge.from);
      internal.insert(bridge.to);
    }
  }
  ASSERT_FALSE(internal.empty());

  const auto plan = capacityPlanners().make("watershed")->run(benchmark.chip, benchmark.config);
  const auto shut = globalRouters().make("hanan-milp")->run(benchmark.chip, plan, benchmark.config);
  for (const auto& connection : shut.connections) {
    EXPECT_FALSE(internal.contains(connection->source->index()));
    EXPECT_FALSE(internal.contains(connection->target.index()));
  }

  // Opening them changes nothing on this chip: its circuit never wanted a
  // crossing, so the switch costs a solution only where one was being used.
  const auto opened = nineQubit(true);
  const auto crossing =
      globalRouters().make("hanan-milp")->run(opened.chip, plan, opened.config);
  EXPECT_EQ(crossing.connections.size(), shut.connections.size());
  EXPECT_DOUBLE_EQ(crossing.objective, shut.objective);
}

/// Run one check over every benchmark chip a fixture covers.
void forEachBenchmark(const std::function<void(const Benchmark&)>& check) {
  check(fourQubit());
  check(nineQubit());
  check(seventeenQubit());
}

TEST(BenchmarkStages, CarriesEveryTargetOnSomeCapacityChain) {
  // The chains are what every later stage routes along, so a target no chain
  // names is a port the plan cannot reach at all. Every port the scene placed
  // a target cell for is on one, whether it roots a chain of its own or sits
  // in the chamber of another.
  forEachBenchmark([](const Benchmark& benchmark) {
    const auto scene = buildScene(benchmark.chip, benchmark.config);
    const auto plan = capacityPlanners().make("watershed")->run(benchmark.chip, benchmark.config);
    ASSERT_FALSE(scene.targetPort.empty());

    std::set<std::uint32_t> named;
    for (const auto& node : plan.nodes) {
      if (node->kind == flatbuffers::artifacts::CapacityElement::Target) {
        named.insert(node->id);
      }
    }
    for (const auto port : scene.targetPort) {
      EXPECT_TRUE(named.contains(port))
          << benchmark.chip.ports[port]->label << " is on no capacity chain";
    }
  });
}

TEST(BenchmarkStages, ServesEveryInnerTarget) {
  // A target the inner circuit does not reach is reported rather than routed,
  // which is what keeps one unreachable port from losing the whole stage. On
  // these chips there is nothing to report: every inner target gets its wire.
  forEachBenchmark([](const Benchmark& benchmark) {
    const auto circuit = innerCircuitOf(benchmark.chip, benchmark.config);
    const auto plan = capacityPlanners().make("watershed")->run(benchmark.chip, benchmark.config);
    const auto global =
        globalRouters().make("hanan-milp")->run(benchmark.chip, plan, benchmark.config);

    std::set<std::uint32_t> served;
    for (const auto& connection : global.connections) {
      served.insert(connection->target.index());
    }
    for (const auto target : circuit.targets) {
      EXPECT_TRUE(served.contains(target))
          << benchmark.chip.ports[target]->label << " is left unserved";
    }
    // And no port takes two wires, which is what a demand per lattice gave.
    EXPECT_EQ(served.size(), global.connections.size());
  });
}

TEST(BenchmarkStages, KeepsTheGatesOfAChamberSeveralCorridorsMeet) {
  // A gate is hidden when the chamber cannot see it at all, so one clear line
  // of sight keeps it. Asking that every cell beside it see it clearly is a
  // stricter question and drops every gate of a chamber that several
  // corridors meet, because each of them has a neighbour across the sight
  // line from one of its own ends.
  //
  // On the 17-qubit chip that closed a chamber four gates border: the chain
  // of Coupler16_17.port3 ended at its own target, the pruning then took the
  // gate that led into it, the bridge that surfaces there was shut for want
  // of a launcher, and Coupler12_13.port0 was left with no supply. Every
  // chain of that chip now leads somewhere.
  const auto benchmark = seventeenQubit();
  const auto plan = capacityPlanners().make("watershed")->run(benchmark.chip, benchmark.config);
  ASSERT_FALSE(plan.chains.empty());
  for (const auto root : plan.chains) {
    EXPECT_FALSE(plan.nodes[root]->next.empty())
        << "the chain of " << benchmark.chip.ports[plan.nodes[root]->id]->label
        << " leads nowhere";
  }
}

TEST(BenchmarkStages, ReportsWhatThePortsOwnApproachesKeepClear) {
  // The bands and the launcher sweeps are obstacles the chip input does not
  // carry, so the picture has no way to show them unless the plan does.
  const auto benchmark = nineQubit();
  const auto scene = buildScene(benchmark.chip, benchmark.config);

  // Every cell a band reserved is part of the keepout. The reverse does not
  // hold: a launcher sweep is keepout and was never reserved, and a target
  // dug back out of a band left the reserved set and not the keepout.
  std::size_t reserved = 0;
  std::size_t keepout = 0;
  for (std::size_t cell = 0; cell < scene.reserved.size(); ++cell) {
    if (scene.reserved.test(cell)) {
      ++reserved;
      EXPECT_TRUE(scene.keepout.test(cell)) << "cell " << cell;
    }
    keepout += scene.keepout.test(cell) ? 1 : 0;
  }
  EXPECT_GT(reserved, 0U);
  EXPECT_GT(keepout, reserved);

  const auto plan = capacityPlanners().make("watershed")->run(benchmark.chip, benchmark.config);
  ASSERT_FALSE(plan.port_keepout.empty());
  for (const auto& ring : plan.port_keepout) {
    EXPECT_GE(ring->vertices.size(), 4U);
    EXPECT_EQ(ring->vertices.front(), ring->vertices.back());
  }
}

TEST(BenchmarkStages, GivesATargetOneWireHoweverManyLatticesCarryIt) {
  // A chamber border runs between two capacity chains, so a target beside one
  // is a node of both lattices. Its demand is on the port: asking each
  // lattice for a wire of its own would put two wires on one port and take
  // the supply from a target that then has none. The 17-qubit chip has four
  // such targets.
  const auto benchmark = seventeenQubit();
  const auto plan = capacityPlanners().make("watershed")->run(benchmark.chip, benchmark.config);
  const auto global =
      globalRouters().make("hanan-milp")->run(benchmark.chip, plan, benchmark.config);
  ASSERT_FALSE(global.connections.empty());

  std::set<std::uint32_t> served;
  for (const auto& connection : global.connections) {
    EXPECT_TRUE(served.insert(connection->target.index()).second)
        << "two wires end at " << benchmark.chip.ports[connection->target.index()]->label;
  }
}

TEST(BenchmarkStages, KeepsTheInnerCircuitOffTheArtwork) {
  // The lattice a Hanan grid spans runs straight over the couplers and the
  // qubits between its ports. A node that lands on artwork is removed, and so
  // is every edge that reached it, or the inner circuit reports an objective
  // over paths that could never be built.
  const auto benchmark = nineQubit();
  const auto plan = capacityPlanners().make("watershed")->run(benchmark.chip, benchmark.config);
  const auto global =
      globalRouters().make("hanan-milp")->run(benchmark.chip, plan, benchmark.config);
  ASSERT_FALSE(global.lattices.empty());

  /// Whether a point is inside a polygon, by ray casting. The first polygon
  /// is the chip outline and is not artwork.
  const auto onArtwork = [&](const flatbuffers::geometry::Point& point) {
    for (std::size_t index = 1; index < benchmark.chip.obstacles.size(); ++index) {
      const auto& vertices = benchmark.chip.obstacles[index]->vertices;
      bool inside = false;
      for (std::size_t at = 0, before = vertices.size() - 1; at < vertices.size();
           before = at++) {
        const auto& a = vertices[at];
        const auto& b = vertices[before];
        if ((a.y() > point.y()) != (b.y() > point.y()) &&
            point.x() < ((b.x() - a.x()) * (point.y() - a.y()) / (b.y() - a.y())) + a.x()) {
          inside = !inside;
        }
      }
      if (inside) {
        return true;
      }
    }
    return false;
  };

  std::size_t edges = 0;
  for (const auto& lattice : global.lattices) {
    for (std::size_t at = 0; at + 1 < lattice->edges.size(); at += 2) {
      ++edges;
      // A port's own node is kept whatever it sits on, because that is where
      // a wire has to start; every other end of an edge is off the artwork.
      for (const auto node : {lattice->edges[at], lattice->edges[at + 1]}) {
        const auto isPort = std::ranges::any_of(global.connections, [&](const auto& connection) {
          return connection->target.index() == node;
        });
        if (!isPort) {
          EXPECT_FALSE(onArtwork(lattice->points[node]));
        }
      }
    }
  }
  EXPECT_GT(edges, 0U);
}

TEST(BenchmarkStages, AssignsTheFourQubitChip) {
  // The chip has no inner circuit, so nothing is added to the ring and the ring
  // the assignment consumes is the configured sequence itself.
  const auto benchmark = fourQubit();
  const auto plan = capacityPlanners().make("watershed")->run(benchmark.chip, benchmark.config);
  const auto global =
      globalRouters().make("hanan-milp")->run(benchmark.chip, plan, benchmark.config);
  const auto assignment =
      assigners().make("ordered-milp")->run(benchmark.chip, plan, global, benchmark.config);

  EXPECT_EQ(assignment.ring.size(), benchmark.config.ports->sequences->all_outer.size());
  EXPECT_EQ(assignment.launchers.size(), assignment.ring.size());
  EXPECT_GT(assignment.objective, 0.0);
  expectEveryRingNodeIsReached(benchmark, assignment);
}

/// The benchmark chips, by the directory each ships in.
class BenchmarkAssignment : public testing::TestWithParam<std::string> {};

TEST_P(BenchmarkAssignment, FollowsTheRulesOfTheRing) {
  const auto benchmark = benchmarkOf(GetParam());
  const auto plan = capacityPlanners().make("watershed")->run(benchmark.chip, benchmark.config);
  const auto global =
      globalRouters().make("hanan-milp")->run(benchmark.chip, plan, benchmark.config);
  const auto assignment =
      assigners().make("ordered-milp")->run(benchmark.chip, plan, global, benchmark.config);

  expectEveryRingNodeIsReached(benchmark, assignment);
  expectALauncherFeedsOneConventionalPort(benchmark, plan, global, assignment);
  expectTheRingKeepsItsCyclicOrder(plan, assignment);
}

INSTANTIATE_TEST_SUITE_P(EveryChip, BenchmarkAssignment,
                         testing::Values("4q", "9q", "17q", "21q", "33q", "45q", "57q", "69q"),
                         [](const testing::TestParamInfo<std::string>& chip) {
                           return chip.param;
                         });

} // namespace
} // namespace mqt::scpd::pipeline
