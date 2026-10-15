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
#include "mqt-scpd/geometry/Geometry.hpp"
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

TEST(AssignmentInputs, RefuseAPortThatTheChipDoesNotCarry) {
  // The chip has four ports, so index 4 names none.
  const auto chip = lineOfPorts();
  const auto refusal = [&chip](const CapacityPlanT& plan,
                               const GlobalRoutingT& routing) {
    try {
      static_cast<void>(assignmentInputs(chip, plan, routing));
    } catch (const std::invalid_argument& error) {
      return std::string(error.what());
    }
    return std::string();
  };
  const auto plan = planWithLaunchersAt({0.0});
  EXPECT_EQ(refusal(plan, routingWithRing({1, 4}, {1})),
            "the global routing names port 4, but the chip has 4 ports");
  EXPECT_EQ(refusal(plan, routingWithRing({1}, {4})),
            "the global routing names port 4, but the chip has 4 ports");
  EXPECT_EQ(refusal(planWithLaunchersAt({0.0, 1.0, 2.0, 3.0, 4.0}),
                    routingWithRing({1}, {1})),
            "the capacity plan names port 4, but the chip has 4 ports");
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

std::string
placesProblemOf(const std::vector<std::array<double, 2>>& launchers) {
  return launcherPlacesProblem(squareChip(launchers), squareConfig())
      .value_or("");
}

/// The reason the stage gives for a chip whose launchers cannot feed its
/// resonators.
constexpr std::string_view PLACES_REASON =
    "feeding resonators needs at least three launchers that do not all lie "
    "on one line; with fewer, two resonators can share one feed point";

TEST(LauncherPlaces, AcceptThreeLaunchersAroundTheChip) {
  EXPECT_EQ(placesProblemOf({{-10.0, 50.0}, {50.0, -10.0}, {110.0, 50.0}}), "");
}

TEST(LauncherPlaces, RefuseFewerThanThreeLaunchers) {
  EXPECT_EQ(placesProblemOf({{-10.0, 50.0}, {110.0, 50.0}}),
            "the chip has 2 launcher ports; " + std::string(PLACES_REASON));
  EXPECT_EQ(placesProblemOf({{-10.0, 50.0}}),
            "the chip has 1 launcher port; " + std::string(PLACES_REASON));
}

TEST(LauncherPlaces, RefuseLaunchersOnOneLine) {
  EXPECT_EQ(placesProblemOf({{-10.0, -10.0}, {50.0, -10.0}, {110.0, -10.0}}),
            "the launcher ports of the chip lie on one line; " +
                std::string(PLACES_REASON));
}

TEST(LauncherPlaces, AcceptLaunchersAroundTheChipWhenTheFirstTwoCoincide) {
  // The first two launchers share a center, so they give no line a
  // direction; the other two lie off every line through it.
  EXPECT_EQ(placesProblemOf(
                {{-10.0, 50.0}, {-10.0, 50.0}, {50.0, -10.0}, {110.0, 50.0}}),
            "");
}

TEST(LauncherPlaces, JudgeTheSlotsAndNotTheCenters) {
  // The third launcher lies a hundredth of a unit off the line of the other
  // two, far less than a detail cell, so its slot lies on that line.
  EXPECT_EQ(placesProblemOf({{-10.0, -10.0}, {50.0, -10.0}, {110.0, -9.99}}),
            "the launcher ports of the chip lie on one line; " +
                std::string(PLACES_REASON));
}

TEST(LauncherPlaces, AcceptTwoLaunchersOnAChipWithoutResonators) {
  auto chip = squareChip({{-10.0, 50.0}, {110.0, 50.0}});
  for (auto& port : chip.ports) {
    if (port->role == fbd::UnassignedRole::Resonator) {
      port->role = fbd::UnassignedRole::Conventional;
    }
  }
  EXPECT_EQ(launcherPlacesProblem(chip, squareConfig()), std::nullopt);
}

TEST(LauncherPlaces, AreWhatTheStageRefuses) {
  // Two launchers and a ring with a resonator: the feeds of the resonators
  // on the two segments between the launchers lie on one line.
  const std::vector<std::array<double, 2>> launchers = {{-10.0, 50.0},
                                                        {110.0, 50.0}};
  const auto chip = squareChip(launchers);
  auto config = squareConfig();
  config.rules = std::make_unique<fbd::DesignRulesT>();
  try {
    static_cast<void>(makeOrderedMilpAssigner()->run(
        chip, planWithLaunchersOf(chip), routingWithRing({0, 1, 2, 3}, {0}),
        config, {}));
    FAIL() << "the stage has to refuse two launchers for a resonator";
  } catch (const std::invalid_argument& error) {
    EXPECT_EQ(error.what(), placesProblemOf(launchers));
  }
}

TEST(Assigner, GivesEveryPortOfARingWithoutResonatorsOneOfTwoLaunchers) {
  // Without a resonator there is no feed to place, so two launchers serve
  // two conventional ports.
  const auto chip = squareChip({{-10.0, 50.0}, {110.0, 50.0}});
  auto config = squareConfig();
  config.rules = std::make_unique<fbd::DesignRulesT>();
  config.rules->max_feedline_utilization = 4;
  config.stages = std::make_unique<flatbuffers::config::StageParamsT>();
  config.stages->assignment =
      std::make_unique<flatbuffers::config::AssignmentParamsT>();
  config.stages->assignment->launcher_target = 2;

  const auto assignment = makeOrderedMilpAssigner()->run(
      chip, planWithLaunchersOf(chip), routingWithRing({0, 1}, {}), config, {});

  EXPECT_TRUE(assignment.chains.empty());
  std::set<std::uint32_t> launchers;
  for (const auto& connection : assignment.connections) {
    ASSERT_NE(connection->source, nullptr);
    launchers.insert(connection->source->index());
  }
  EXPECT_EQ(launchers.size(), 2U);
}

TEST(Assigner, GivesEachOfTwoResonatorsAChainOfItsOwn) {
  // Two launcher ends and two terminations on two resonators leave no chord
  // to use: each resonator is a chain of one, from a launcher to a
  // termination.
  const auto chip =
      squareChip({{-10.0, 50.0}, {50.0, -10.0}, {110.0, 50.0}, {50.0, 110.0}});
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

TEST(Assigner, RefusesFeedsCloserThanTheWireSpacing) {
  // Neighbouring launchers lie about 85 units apart. A resonator fed between
  // two of them lies half of that from each, below a wire spacing of 50.
  const auto chip =
      squareChip({{-10.0, 50.0}, {50.0, -10.0}, {110.0, 50.0}, {50.0, 110.0}});
  auto config = squareConfig();
  config.rules = std::make_unique<fbd::DesignRulesT>();
  config.rules->max_feedline_utilization = 4;
  config.rules->feedline_terminations = 2;
  config.rules->min_wire_spacing = 50.0;
  config.stages = std::make_unique<flatbuffers::config::StageParamsT>();
  config.stages->assignment =
      std::make_unique<flatbuffers::config::AssignmentParamsT>();
  config.stages->assignment->launcher_target = 2;

  try {
    static_cast<void>(makeOrderedMilpAssigner()->run(
        chip, planWithLaunchersOf(chip), routingWithRing({0, 1}, {0, 1}),
        config, {}));
    FAIL() << "the stage has to refuse feeds closer than the wire spacing";
  } catch (const std::invalid_argument& error) {
    EXPECT_NE(std::string(error.what()).find("too close to feed 1 resonator"),
              std::string::npos)
        << error.what();
  }

  // At a spacing of 40 every part is long enough.
  config.rules->min_wire_spacing = 40.0;
  EXPECT_NO_THROW(static_cast<void>(makeOrderedMilpAssigner()->run(
      chip, planWithLaunchersOf(chip), routingWithRing({0, 1}, {0, 1}), config,
      {})));
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

/// The message with which the stage refuses the fixture under @p config, or
/// an empty string when it does not refuse it.
std::string refusalOfMini(const ConfigT& config) {
  try {
    static_cast<void>(assignMini(config));
  } catch (const std::invalid_argument& error) {
    return error.what();
  }
  return {};
}

TEST(Assigner, SaysWhenTheRingNeedsMoreLaunchersThanTheChipHas) {
  // The three conventional ports and two launcher ends need five of the four
  // launchers.
  auto config = test::miniConfig();
  config.rules->feedline_terminations = 0;
  config.stages->assignment->launcher_target = 2;
  const auto refusal = refusalOfMini(config);
  EXPECT_NE(refusal.find("need 5 launchers, but the chip has 4"),
            std::string::npos)
      << refusal;
}

TEST(Assigner, SaysWhenTheChainsHaveAnOddNumberOfEnds) {
  // One launcher end and no termination leave a chain with one end.
  auto config = test::miniConfig();
  config.rules->feedline_terminations = 0;
  const auto refusal = refusalOfMini(config);
  EXPECT_NE(refusal.find("is odd"), std::string::npos) << refusal;
}

TEST(Assigner, SaysWhenTheChainsCannotCarryEveryResonator) {
  // One chain of at most one resonator cannot carry the two of the fixture.
  auto config = test::miniConfig();
  config.rules->max_feedline_utilization = 1;
  const auto refusal = refusalOfMini(config);
  EXPECT_NE(refusal.find("the ring carries 2 resonators, but [design_rules] "
                         "max_feedline_utilization 1 lets 1 feedline chain "
                         "carry at most 1"),
            std::string::npos)
      << refusal;
}

TEST(Assigner, SaysWhenTheRingHasFewerResonatorsThanChainEnds) {
  // Four launcher ends need four resonators, and the ring carries two.
  const auto chip =
      squareChip({{-10.0, 50.0}, {50.0, -10.0}, {110.0, 50.0}, {50.0, 110.0}});
  auto config = squareConfig();
  config.rules = std::make_unique<fbd::DesignRulesT>();
  config.rules->max_feedline_utilization = 4;
  config.stages = std::make_unique<flatbuffers::config::StageParamsT>();
  config.stages->assignment =
      std::make_unique<flatbuffers::config::AssignmentParamsT>();
  config.stages->assignment->launcher_target = 4;
  try {
    static_cast<void>(makeOrderedMilpAssigner()->run(
        chip, planWithLaunchersOf(chip), routingWithRing({0, 1}, {0, 1}),
        config, {}));
    FAIL() << "more launcher ends than resonators have to be refused";
  } catch (const std::invalid_argument& error) {
    EXPECT_NE(std::string(error.what())
                  .find("the ring carries 2 resonators, too few for "
                        "[stages.assignment] launcher_target 4"),
              std::string::npos)
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

/// The side of the line through @p from and @p to that @p point lies on: 1 to
/// the left, -1 to the right, and 0 on the line.
int sideOf(const geometry::Point& from, const geometry::Point& to,
           const geometry::Point& point) {
  const auto cross = geometry::cross(geometry::subtract(to, from),
                                     geometry::subtract(point, from));
  constexpr auto tolerance = 1.0e-9;
  if (cross > tolerance) {
    return 1;
  }
  return cross < -tolerance ? -1 : 0;
}

/// Whether the segments @p a0 to @p a1 and @p b0 to @p b1 share a point.
bool touch(const geometry::Point& a0, const geometry::Point& a1,
           const geometry::Point& b0, const geometry::Point& b1) {
  const auto a0Side = sideOf(b0, b1, a0);
  const auto a1Side = sideOf(b0, b1, a1);
  const auto b0Side = sideOf(a0, a1, b0);
  const auto b1Side = sideOf(a0, a1, b1);
  if (a0Side * a1Side > 0 || b0Side * b1Side > 0) {
    return false;
  }
  if (a0Side != 0 || a1Side != 0) {
    return true;
  }
  // All four points lie on one line: the segments share a point where their
  // extents overlap on both axes.
  return std::max(std::min(a0.x(), a1.x()), std::min(b0.x(), b1.x())) <=
             std::min(std::max(a0.x(), a1.x()), std::max(b0.x(), b1.x())) &&
         std::max(std::min(a0.y(), a1.y()), std::min(b0.y(), b1.y())) <=
             std::min(std::max(a0.y(), a1.y()), std::max(b0.y(), b1.y()));
}

/// Whether the line of a feedline chain, from its start launcher through the
/// feeds of its nodes to its end launcher, meets itself. Two neighboring
/// pieces meet where the second turns back along the first, and two other
/// pieces where they share a point.
bool meetsItself(const ChipT& chip, const AssignmentT& assignment,
                 const fba::FeedlineChainT& chain) {
  std::vector<geometry::Point> line;
  if (chain.start != nullptr) {
    line.push_back(chip.ports[chain.start->index()]->center);
  }
  for (const auto node : chain.nodes) {
    line.push_back(assignment.feeds[node]);
  }
  if (chain.end != nullptr) {
    line.push_back(chip.ports[chain.end->index()]->center);
  }
  for (std::size_t first = 0; first + 1 < line.size(); ++first) {
    for (std::size_t second = first + 1; second + 1 < line.size(); ++second) {
      const auto& a0 = line[first];
      const auto& a1 = line[first + 1];
      const auto& b0 = line[second];
      const auto& b1 = line[second + 1];
      if (second == first + 1) {
        const auto ahead = ((a1.x() - a0.x()) * (b1.x() - b0.x())) +
                           ((a1.y() - a0.y()) * (b1.y() - b0.y()));
        if (sideOf(a0, a1, b1) == 0 && ahead <= 0.0) {
          return true;
        }
      } else if (touch(a0, a1, b0, b1)) {
        return true;
      }
    }
  }
  return false;
}

/// A set of resonators on octagonChip with its count of terminations.
struct OctagonCase {
  std::vector<std::uint32_t> resonators;
  std::uint32_t terminations = 0;
};

/// The resonator sets on octagonChip whose ring may open on resonators that
/// take no launcher. On {0, 3, 4, 7}, four conventional ports and two
/// launcher ends need all six launchers. On {0, 1, 6, 7}, the ring may open
/// on two such resonators in a row.
std::vector<OctagonCase> openingCases() {
  return {OctagonCase{.resonators = {0, 3, 4, 7}, .terminations = 2},
          OctagonCase{.resonators = {0, 1, 6, 7}, .terminations = 0}};
}

TEST(Assigner, FindsTheSameOptimumWhereverTheRingOpens) {
  // A resonator that the ring opens on and that takes no launcher must not
  // take a launcher place of its own.
  for (const auto& [resonators, terminations] : openingCases()) {
    const auto objective =
        assignOctagon(1, resonators, 2, terminations).objective;
    for (std::size_t first = 0; first < 8; ++first) {
      EXPECT_NEAR(assignOctagon(first, resonators, 2, terminations).objective,
                  objective, 1.0e-6)
          << "the ring opens at r" << first << " with " << terminations
          << " terminations";
    }
  }
}

TEST(Assigner, RunsEveryChainOneWayWhereverTheRingOpens) {
  // Every chain runs one way along its launchers, the chain that crosses the
  // opening of the ring as well, so the line of a chain never meets itself.
  const auto chip = octagonChip();
  for (const auto& [resonators, terminations] : openingCases()) {
    for (std::size_t first = 0; first < 8; ++first) {
      const auto assignment = assignOctagon(first, resonators, 2, terminations);
      for (const auto& chain : assignment.chains) {
        EXPECT_FALSE(meetsItself(chip, assignment, *chain))
            << "the ring opens at r" << first << " with " << terminations
            << " terminations";
      }
    }
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
