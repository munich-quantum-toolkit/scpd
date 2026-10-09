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
#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/io/Artifacts.hpp"
#include "mqt-scpd/pipeline/Assigner.hpp"
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
  ASSERT_EQ(results[0].size(), 3U);
  EXPECT_EQ(results[0][0].text, "5 ports on 1 launcher");
  EXPECT_TRUE(results[0][1].text.starts_with("objective "))
      << results[0][1].text;
  EXPECT_EQ(results[0][2].text, "optimal");
  EXPECT_EQ(fails[0], 0U);
  EXPECT_TRUE(std::ranges::any_of(steps, [](const std::string& line) {
    return line.starts_with("model assignment");
  }));
  EXPECT_EQ(entries.size(), assignment.chains.size());
}

TEST(Assigner, SaysSoWhenThereIsNothingToAssign) {
  // With one resonator on the ring there is no chain to build. Every port of
  // the ring is left without a launcher, and each one counts as a fail.
  auto config = test::miniConfig();
  config.ports->patterns->resonator = R"(^Q1\.port0$)";
  config.ports->patterns->conventional =
      R"(^(Q\d+\.port1|C\d+\.port0|Q2\.port0)$)";
  std::vector<std::vector<Figure>> results;
  std::vector<std::optional<std::uint64_t>> fails;
  const Report report(Detail::Summary,
                      {.result = [&](const std::vector<Figure>& figures,
                                     const std::optional<std::uint64_t> count) {
                        results.push_back(figures);
                        fails.push_back(count);
                      }});

  const auto [chip, global, assignment] = assignMini(config, report);

  EXPECT_TRUE(assignment.connections.empty());
  ASSERT_EQ(results.size(), 1U);
  EXPECT_EQ(results[0].back().text, "nothing to assign");
  EXPECT_EQ(fails[0], assignment.ring.size());
}

} // namespace
} // namespace mqt::scpd::pipeline
