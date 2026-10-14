/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// The Assignment stage: the inputs it works out before the model exists, what
// its assignment holds on the two-qubit fixture, and what it reports.

#include "MiniFixture.hpp"
#include "mqt-scpd/flatbuffers/artifacts.hpp"
#include "mqt-scpd/flatbuffers/config.hpp"
#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/io/Artifacts.hpp"
#include "mqt-scpd/pipeline/Assigner.hpp"
#include "mqt-scpd/pipeline/CapacityPlanner.hpp"
#include "mqt-scpd/pipeline/GlobalRouter.hpp"
#include "mqt-scpd/pipeline/Report.hpp"
#include "mqt-scpd/pipeline/Stages.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace mqt::scpd::pipeline {
namespace {

namespace fba = flatbuffers::artifacts;
namespace fbd = flatbuffers::design;
using flatbuffers::artifacts::ArtifactT;

// The inputs, on a chip whose ports sit on a line, so that the nearest
// launcher is plain to see.

ChipT lineOfPorts() {
  ChipT chip;
  for (int index = 0; index < 4; ++index) {
    auto port = std::make_unique<fbd::PortT>();
    port->label = "p" + std::to_string(index);
    port->center = {static_cast<double>(index) * 100.0, 0.0};
    port->role = index == 0 ? fbd::UnassignedRole::Launcher
                            : fbd::UnassignedRole::Resonator;
    chip.ports.push_back(std::move(port));
  }
  return chip;
}

CapacityPlanT planWithLaunchersAt(const std::vector<double>& positions) {
  CapacityPlanT plan;
  for (std::size_t index = 0; index < positions.size(); ++index) {
    auto slot = std::make_unique<fba::LauncherSlotT>();
    slot->port = fbd::PortRef(static_cast<std::uint32_t>(index));
    slot->position = {positions[index], 0.0};
    plan.launchers.push_back(std::move(slot));
  }
  return plan;
}

GlobalRoutingT routingWithRing(const std::vector<std::uint32_t>& ring,
                               const std::vector<std::uint32_t>& resonators) {
  GlobalRoutingT routing;
  for (const auto port : ring) {
    routing.outer_ring.emplace_back(port);
  }
  for (const auto port : resonators) {
    routing.resonators.emplace_back(port);
  }
  return routing;
}

TEST(AssignmentInputs, TakeTheRingOfTheGlobalStageInItsOrder) {
  const auto inputs =
      assignmentInputs(lineOfPorts(), planWithLaunchersAt({0.0}),
                       routingWithRing({3, 1, 2}, {1, 3}));

  // The inner circuit decides which outer ports carry a wire, so the ring is
  // the one the Global stage wrote and not the configured one.
  EXPECT_EQ(inputs.ring, (std::vector<std::uint32_t>{3, 1, 2}));
  EXPECT_EQ(inputs.isResonator, (std::vector<bool>{true, true, false}));
}

TEST(AssignmentInputs, FindTheNearestLauncherOfEveryRingNode) {
  // Two launchers, one beyond each end of the line of ports.
  const auto inputs =
      assignmentInputs(lineOfPorts(), planWithLaunchersAt({-50.0, 350.0}),
                       routingWithRing({1, 2, 3}, {1}));

  // The port at 200 is 250 from the first launcher and 150 from the second.
  EXPECT_EQ(inputs.nearestLauncher, (std::vector<std::uint32_t>{0, 1, 1}));
  EXPECT_EQ(inputs.launchers, (std::vector<std::uint32_t>{0, 1}));
}

TEST(AssignmentInputs, RefuseAPlanWithoutALauncherOrARingWithoutAPort) {
  const auto chip = lineOfPorts();
  EXPECT_THROW(static_cast<void>(assignmentInputs(chip, CapacityPlanT{},
                                                  routingWithRing({1}, {1}))),
               std::invalid_argument);
  EXPECT_THROW(static_cast<void>(assignmentInputs(
                   chip, planWithLaunchersAt({0.0}), routingWithRing({}, {}))),
               std::invalid_argument);
}

// The launcher order, on a chip whose four ring ports sit clockwise on the
// corners of a square and whose four launchers sit on the middle of its sides.

ChipT squareChip(const std::vector<std::array<double, 2>>& launchers) {
  ChipT chip;
  const std::vector<std::array<double, 2>> corners = {
      {0.0, 100.0}, {100.0, 100.0}, {100.0, 0.0}, {0.0, 0.0}};
  for (std::size_t index = 0; index < corners.size(); ++index) {
    auto port = std::make_unique<fbd::PortT>();
    port->label = "r" + std::to_string(index);
    port->center = {corners[index][0], corners[index][1]};
    port->role = fbd::UnassignedRole::Resonator;
    chip.ports.push_back(std::move(port));
  }
  for (std::size_t index = 0; index < launchers.size(); ++index) {
    auto port = std::make_unique<fbd::PortT>();
    port->label = "l" + std::to_string(index);
    port->center = {launchers[index][0], launchers[index][1]};
    port->role = fbd::UnassignedRole::Launcher;
    chip.ports.push_back(std::move(port));
  }
  return chip;
}

CapacityPlanT planWithLaunchersOf(const ChipT& chip) {
  CapacityPlanT plan;
  for (std::uint32_t index = 0; index < chip.ports.size(); ++index) {
    if (chip.ports[index]->role != fbd::UnassignedRole::Launcher) {
      continue;
    }
    auto slot = std::make_unique<fba::LauncherSlotT>();
    slot->port = fbd::PortRef(index);
    slot->position = chip.ports[index]->center;
    plan.launchers.push_back(std::move(slot));
  }
  return plan;
}

/// The configuration of squareChip: the ring runs clockwise around the four
/// resonators.
ConfigT squareConfig() {
  ConfigT config;
  config.ports = std::make_unique<flatbuffers::config::PortConfigT>();
  config.ports->sequences =
      std::make_unique<flatbuffers::config::PortSequencesT>();
  config.ports->sequences->all_outer = {"r0", "r1", "r2", "r3"};
  return config;
}

std::string
orderProblemOf(const std::vector<std::array<double, 2>>& launchers) {
  return launcherOrderProblem(squareChip(launchers), squareConfig())
      .value_or("");
}

TEST(LauncherOrder, AcceptsLaunchersListedAgainstTheRing) {
  // The ring runs clockwise, so the launchers run counterclockwise: left,
  // bottom, right, top. Any of them may come first.
  EXPECT_EQ(orderProblemOf(
                {{-10.0, 50.0}, {50.0, -10.0}, {110.0, 50.0}, {50.0, 110.0}}),
            "");
  EXPECT_EQ(orderProblemOf(
                {{110.0, 50.0}, {50.0, 110.0}, {-10.0, 50.0}, {50.0, -10.0}}),
            "");
}

TEST(LauncherOrder, RefusesLaunchersListedWithTheRing) {
  EXPECT_NE(orderProblemOf(
                {{-10.0, 50.0}, {50.0, 110.0}, {110.0, 50.0}, {50.0, -10.0}})
                .find("in the direction of the port ring"),
            std::string::npos);
}

TEST(LauncherOrder, RefusesLaunchersListedOutOfOrder) {
  // Left, right, bottom, top: the walk crosses the chip twice.
  EXPECT_NE(orderProblemOf(
                {{-10.0, 50.0}, {110.0, 50.0}, {50.0, -10.0}, {50.0, 110.0}})
                .find("does not list the launcher ports in order"),
            std::string::npos);
}

TEST(LauncherOrder, IsWhatTheStageRefuses) {
  const std::vector<std::array<double, 2>> launchers = {
      {-10.0, 50.0}, {50.0, 110.0}, {110.0, 50.0}, {50.0, -10.0}};
  const auto chip = squareChip(launchers);
  try {
    static_cast<void>(makeOrderedMilpAssigner()->run(
        chip, planWithLaunchersOf(chip), routingWithRing({0, 1, 2, 3}, {0}),
        squareConfig(), {}));
    FAIL() << "the stage has to refuse launchers listed with the ring";
  } catch (const std::invalid_argument& error) {
    EXPECT_EQ(error.what(), orderProblemOf(launchers));
  }
}

TEST(Assigner, GivesEachOfTwoResonatorsAChainOfItsOwn) {
  // Two launcher ends and two terminations on two resonators leave no chord
  // to use: each resonator is a chain of one, from a launcher to a
  // termination.
  const auto chip = squareChip({{-10.0, 50.0}, {110.0, 50.0}});
  auto config = squareConfig();
  config.rules = std::make_unique<fbd::DesignRulesT>();
  config.rules->max_feedline_utilization = 4;
  config.rules->feedline_terminations = 2;
  config.stages = std::make_unique<flatbuffers::config::StageParamsT>();
  config.stages->assignment =
      std::make_unique<flatbuffers::config::AssignmentParamsT>();
  config.stages->assignment->launcher_target = 2;

  const auto assignment = makeOrderedMilpAssigner()->run(
      chip, planWithLaunchersOf(chip), routingWithRing({0, 1}, {0, 1}), config,
      {});

  ASSERT_EQ(assignment.chains.size(), 2U);
  std::set<std::uint32_t> chained;
  for (const auto& chain : assignment.chains) {
    ASSERT_EQ(chain->nodes.size(), 1U);
    chained.insert(chain->nodes.front());
    // The launcher is the start of the chain; the termination is its end.
    ASSERT_NE(chain->start, nullptr);
    EXPECT_EQ(chain->start->index(),
              assignment.launchers[chain->nodes.front()].index());
    EXPECT_EQ(chain->end, nullptr);
  }
  EXPECT_EQ(chained, (std::set<std::uint32_t>{0, 1}));
}

// The stage on the two-qubit fixture.

/// The inputs of the stage on the fixture, and its assignment.
struct Assigned {
  ChipT chip;
  GlobalRoutingT global;
  AssignmentT assignment;
};

Assigned assignMini(const ConfigT& config, const Report& report = {}) {
  auto chip = test::miniChip(config);
  const auto capacity = makeWatershedPlanner()->run(chip, config, {});
  auto global = makeHananMilpRouter()->run(chip, capacity, config, {});
  auto assignment =
      makeOrderedMilpAssigner()->run(chip, capacity, global, config, report);
  return {.chip = std::move(chip),
          .global = std::move(global),
          .assignment = std::move(assignment)};
}

TEST(Assigner, ConnectsEveryRingPortInRingOrder) {
  const auto [chip, global, assignment] = assignMini(test::miniConfig());

  ASSERT_EQ(assignment.ring.size(), global.outer_ring.size());
  ASSERT_EQ(assignment.connections.size(), assignment.ring.size());
  ASSERT_EQ(assignment.launchers.size(), assignment.ring.size());
  ASSERT_EQ(assignment.feeds.size(), assignment.ring.size());
  std::set<std::uint32_t> conventionalSlots;
  for (std::size_t index = 0; index < assignment.ring.size(); ++index) {
    const auto port = assignment.ring[index].index();
    EXPECT_EQ(port, global.outer_ring[index].index());
    const auto& connection = *assignment.connections[index];
    EXPECT_EQ(connection.target.index(), port);
    if (chip.ports[port]->role == fbd::UnassignedRole::Resonator) {
      // The coupler that feeds a resonator comes later, so there is no source
      // port yet.
      EXPECT_EQ(connection.source, nullptr);
      EXPECT_EQ(connection.source_role, fbd::AssignedRole::ResonatorSource);
      continue;
    }
    // A conventional port runs from the launcher it was given, and no two
    // share one.
    ASSERT_NE(connection.source, nullptr);
    EXPECT_EQ(connection.source->index(), assignment.launchers[index].index());
    EXPECT_EQ(connection.source_role, fbd::AssignedRole::FeedlineSource);
    EXPECT_TRUE(conventionalSlots.insert(connection.source->index()).second);
  }
}

TEST(Assigner, GivesNoTwoRingNodesOneFeedPoint) {
  const auto [chip, global, assignment] = assignMini(test::miniConfig());

  std::set<std::pair<double, double>> feeds;
  for (const auto& feed : assignment.feeds) {
    EXPECT_TRUE(feeds.emplace(feed.x(), feed.y()).second)
        << "two ring nodes are fed at (" << feed.x() << ", " << feed.y() << ")";
  }
}

TEST(Assigner, DrivesEveryResonatorFromOneFeedlineChain) {
  const auto config = test::miniConfig();
  const auto [chip, global, assignment] = assignMini(config);

  std::multiset<std::uint32_t> chained;
  std::uint32_t launcherEnds = 0;
  std::uint32_t terminationEnds = 0;
  for (const auto& chain : assignment.chains) {
    for (const auto node : chain->nodes) {
      ASSERT_LT(node, assignment.ring.size());
      EXPECT_EQ(chip.ports[assignment.ring[node].index()]->role,
                fbd::UnassignedRole::Resonator);
      chained.insert(node);
    }
    // An end without a launcher ends at a termination.
    for (const auto* end : {chain->start.get(), chain->end.get()}) {
      ++(end != nullptr ? launcherEnds : terminationEnds);
    }
  }
  std::multiset<std::uint32_t> resonators;
  for (std::uint32_t node = 0; node < assignment.ring.size(); ++node) {
    if (chip.ports[assignment.ring[node].index()]->role ==
        fbd::UnassignedRole::Resonator) {
      resonators.insert(node);
    }
  }
  EXPECT_EQ(chained, resonators);
  // The configuration says how many chain ends are launchers and how many
  // are terminations.
  EXPECT_EQ(launcherEnds, config.stages->assignment->launcher_target);
  EXPECT_EQ(terminationEnds, config.rules->feedline_terminations);
}

TEST(Assigner, RefusesAConfigurationWithoutALauncherTarget) {
  auto config = test::miniConfig();
  config.stages->assignment->launcher_target = 0;
  try {
    static_cast<void>(assignMini(config));
    FAIL() << "a missing launcher target has to be refused";
  } catch (const std::invalid_argument& error) {
    EXPECT_NE(std::string(error.what()).find("launcher_target"),
              std::string::npos);
  }
}

TEST(Assigner, SaysWhyAnAssignmentHasNoSolution) {
  // Without a termination, the three conventional ports and both ends of the
  // feedline chain need five of the four launchers.
  auto config = test::miniConfig();
  config.rules->feedline_terminations = 0;
  try {
    static_cast<void>(assignMini(config));
    FAIL() << "an assignment without a solution has to be refused";
  } catch (const std::runtime_error& error) {
    EXPECT_NE(std::string(error.what()).find("infeasible"), std::string::npos)
        << error.what();
  }
}

TEST(Assigner, GivesTheSameAnswerTwice) {
  const auto bytes = [] {
    ArtifactT artifact;
    artifact.producer = "test";
    artifact.output.Set(assignMini(test::miniConfig()).assignment);
    return io::writeArtifact(artifact);
  };
  EXPECT_EQ(bytes(), bytes());
}

TEST(Assigner, ReportsTheAssignmentAndHowItWasSolved) {
  std::vector<std::vector<Figure>> results;
  std::vector<std::optional<std::uint64_t>> fails;
  std::vector<std::string> steps;
  std::vector<std::string> entries;
  const Report report(
      Detail::Items,
      {.line =
           [&steps](const Detail detail, const std::string_view text) {
             if (detail == Detail::Steps) {
               steps.emplace_back(text);
             }
           },
       .entry =
           [&entries](const Detail /*detail*/, const std::string_view label,
                      const std::vector<Figure>& /*figures*/) {
             entries.emplace_back(label);
           },
       .result =
           [&](const std::vector<Figure>& figures,
               const std::optional<std::uint64_t> count) {
             results.push_back(figures);
             fails.push_back(count);
           }});

  const auto [chip, global, assignment] =
      assignMini(test::miniConfig(), report);

  ASSERT_EQ(results.size(), 1U);
  ASSERT_EQ(results[0].size(), 4U);
  EXPECT_EQ(results[0][0].text, "5 ports");
  EXPECT_EQ(results[0][1].text, "1 launcher end");
  EXPECT_TRUE(results[0][2].text.starts_with("objective "))
      << results[0][2].text;
  EXPECT_EQ(results[0][3].text, "optimal");
  EXPECT_EQ(fails[0], 0U);
  EXPECT_TRUE(std::ranges::any_of(steps, [](const std::string& line) {
    return line.starts_with("model assignment");
  }));
  EXPECT_EQ(entries.size(), assignment.chains.size());
}

// The stage on chips whose ports sit on the sides of a square: the ring runs
// clockwise and the launchers counterclockwise around it.

/// A chip of eight ring ports, clockwise around the square from its top left
/// corner, and six launchers outside it, counterclockwise.
ChipT octagonChip() {
  ChipT chip;
  const std::vector<std::array<double, 2>> ring = {
      {0.0, 300.0}, {150.0, 300.0}, {300.0, 300.0}, {300.0, 150.0},
      {300.0, 0.0}, {150.0, 0.0},   {0.0, 0.0},     {0.0, 150.0}};
  const std::vector<std::array<double, 2>> launchers = {
      {-50.0, 225.0}, {-50.0, 75.0},  {150.0, -50.0},
      {350.0, 75.0},  {350.0, 225.0}, {150.0, 350.0}};
  for (std::size_t index = 0; index < ring.size(); ++index) {
    auto port = std::make_unique<fbd::PortT>();
    port->label = "r" + std::to_string(index);
    port->center = {ring[index][0], ring[index][1]};
    chip.ports.push_back(std::move(port));
  }
  for (std::size_t index = 0; index < launchers.size(); ++index) {
    auto port = std::make_unique<fbd::PortT>();
    port->label = "l" + std::to_string(index);
    port->center = {launchers[index][0], launchers[index][1]};
    port->role = fbd::UnassignedRole::Launcher;
    chip.ports.push_back(std::move(port));
  }
  return chip;
}

/// The configuration of octagonChip with a ring that opens at @p first.
ConfigT octagonConfig(const std::size_t first, const std::uint32_t target,
                      const std::uint32_t terminations) {
  ConfigT config;
  config.ports = std::make_unique<flatbuffers::config::PortConfigT>();
  config.ports->sequences =
      std::make_unique<flatbuffers::config::PortSequencesT>();
  for (std::size_t step = 0; step < 8; ++step) {
    config.ports->sequences->all_outer.push_back(
        "r" + std::to_string((first + step) % 8));
  }
  config.rules = std::make_unique<fbd::DesignRulesT>();
  config.rules->max_feedline_utilization = 8;
  config.rules->feedline_terminations = terminations;
  config.stages = std::make_unique<flatbuffers::config::StageParamsT>();
  config.stages->assignment =
      std::make_unique<flatbuffers::config::AssignmentParamsT>();
  config.stages->assignment->launcher_target = target;
  return config;
}

/// The assignment of octagonChip, with the ring opened at @p first and the
/// ports @p resonators as resonators.
AssignmentT assignOctagon(const std::size_t first,
                          const std::vector<std::uint32_t>& resonators,
                          const std::uint32_t target,
                          const std::uint32_t terminations,
                          const Report& report = {}) {
  const auto chip = octagonChip();
  std::vector<std::uint32_t> ring;
  ring.reserve(8);
  for (std::uint32_t step = 0; step < 8; ++step) {
    ring.push_back(static_cast<std::uint32_t>((first + step) % 8));
  }
  return makeOrderedMilpAssigner()->run(
      chip, planWithLaunchersOf(chip), routingWithRing(ring, resonators),
      octagonConfig(first, target, terminations), report);
}

TEST(Assigner, FindsTheSameOptimumWhereverTheRingOpens) {
  // Four conventional ports and two launcher ends need all six launchers.
  // The ring may open on a resonator that takes no launcher, and that
  // resonator must not take a launcher place of its own.
  const std::vector<std::uint32_t> resonators = {0, 3, 4, 7};
  const auto objective = assignOctagon(1, resonators, 2, 2).objective;
  for (std::size_t first = 0; first < 8; ++first) {
    EXPECT_NEAR(assignOctagon(first, resonators, 2, 2).objective, objective,
                1.0e-6)
        << "the ring opens at r" << first;
  }
}

/// The assignment of octagonChip on a ring of only the ports @p ring.
AssignmentT assignPartOfOctagon(const std::vector<std::uint32_t>& ring,
                                const std::vector<std::uint32_t>& resonators,
                                const std::uint32_t target,
                                const std::uint32_t terminations,
                                const Report& report = {}) {
  const auto chip = octagonChip();
  auto config = octagonConfig(0, target, terminations);
  config.ports->sequences->all_outer.clear();
  for (const auto port : ring) {
    config.ports->sequences->all_outer.push_back("r" + std::to_string(port));
  }
  return makeOrderedMilpAssigner()->run(chip, planWithLaunchersOf(chip),
                                        routingWithRing(ring, resonators),
                                        config, report);
}

TEST(Assigner, GivesALoneResonatorAChainOfItsOwn) {
  // One resonator is a chain from a launcher to a termination, and every
  // conventional port is still given a launcher.
  std::vector<std::optional<std::uint64_t>> fails;
  const Report report(Detail::Summary,
                      {.result = [&](const std::vector<Figure>& /*figures*/,
                                     const std::optional<std::uint64_t> count) {
                        fails.push_back(count);
                      }});

  const auto assignment =
      assignPartOfOctagon({0, 1, 2, 4, 5, 6}, {2}, 1, 1, report);

  ASSERT_EQ(assignment.chains.size(), 1U);
  const auto& chain = *assignment.chains.front();
  ASSERT_EQ(chain.nodes.size(), 1U);
  EXPECT_EQ(chain.nodes[0], 2U);
  ASSERT_NE(chain.start, nullptr);
  EXPECT_EQ(chain.start->index(), assignment.launchers[chain.nodes[0]].index());
  EXPECT_EQ(chain.end, nullptr);
  std::set<std::uint32_t> launchers;
  for (const auto& launcher : assignment.launchers) {
    launchers.insert(launcher.index());
  }
  EXPECT_EQ(launchers.size(), 6U);
  ASSERT_EQ(fails.size(), 1U);
  EXPECT_EQ(fails[0], 0U);
}

TEST(Assigner, GivesEveryPortOfARingWithoutResonatorsALauncher) {
  // Six conventional ports and no resonator: no chain, and six launchers.
  const auto assignment = assignPartOfOctagon({0, 1, 2, 4, 5, 6}, {}, 2, 0);

  EXPECT_TRUE(assignment.chains.empty());
  std::set<std::uint32_t> launchers;
  for (const auto& connection : assignment.connections) {
    ASSERT_NE(connection->source, nullptr);
    launchers.insert(connection->source->index());
  }
  EXPECT_EQ(launchers.size(), 6U);
}

TEST(Assigner, RefusesAConfigurationWithoutDesignRules) {
  auto config = test::miniConfig();
  const auto chip = test::miniChip(config);
  const auto capacity = makeWatershedPlanner()->run(chip, config, {});
  const auto global = makeHananMilpRouter()->run(chip, capacity, config, {});
  config.rules.reset();
  EXPECT_THROW(static_cast<void>(makeOrderedMilpAssigner()->run(
                   chip, capacity, global, config, {})),
               std::invalid_argument);
}

} // namespace
} // namespace mqt::scpd::pipeline
