/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// The Capacity stage on the two-qubit fixture: what its plan holds, that it is
// the same on every run, and what it reports.

#include "MiniFixture.hpp"
#include "mqt-scpd/flatbuffers/artifacts.hpp"
#include "mqt-scpd/flatbuffers/config.hpp"
#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/grid/Watershed.hpp"
#include "mqt-scpd/io/Artifacts.hpp"
#include "mqt-scpd/io/Chip.hpp"
#include "mqt-scpd/pipeline/CapacityPlanner.hpp"
#include "mqt-scpd/pipeline/Report.hpp"
#include "mqt-scpd/pipeline/Stages.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace mqt::scpd::pipeline {
namespace {

using flatbuffers::artifacts::ArtifactT;
using flatbuffers::artifacts::CapacityElement;

CapacityPlanT planMini(const Report& report = {}) {
  return makeWatershedPlanner()->run(test::miniChip(), test::miniConfig(),
                                     report);
}

TEST(CapacityPlanner, PlansTheMiniChip) {
  const auto plan = planMini();

  ASSERT_NE(plan.capacity_grid, nullptr);
  ASSERT_NE(plan.detail_grid, nullptr);
  EXPECT_EQ(plan.capacity_grid->width, 12U);
  // The detail grid refines every capacity cell into the default 30 by 30.
  EXPECT_EQ(plan.detail_grid->width, 12U * 30U);
  EXPECT_FALSE(plan.partitions.empty());
  EXPECT_FALSE(plan.borders.empty());
  EXPECT_FALSE(plan.chains.empty());
  // Every launcher of the chip feeds it from a slot of its own.
  EXPECT_EQ(plan.launchers.size(), 4U);
  for (const auto& border : plan.borders) {
    EXPECT_GE(border->budget, 1U);
    EXPECT_FALSE(border->samples.empty());
  }
}

TEST(CapacityPlanner, KeepsOnlyTheBottlenecksItsChainsName) {
  const auto plan = planMini();

  std::set<std::uint32_t> named;
  for (const auto& node : plan.nodes) {
    for (const auto child : node->next) {
      EXPECT_LT(child, plan.nodes.size());
    }
    if (node->kind == CapacityElement::Bottleneck) {
      ASSERT_LT(node->id, plan.bottlenecks.size());
      named.insert(node->id);
    }
  }
  EXPECT_EQ(named.size(), plan.bottlenecks.size());
  for (const auto root : plan.chains) {
    ASSERT_LT(root, plan.nodes.size());
    EXPECT_EQ(plan.nodes[root]->kind, CapacityElement::Target);
  }
}

TEST(CapacityPlanner, GivesTheSameAnswerTwice) {
  // The stage contract is deterministic, which is what makes a resumed run
  // equal to an uninterrupted one.
  const auto bytes = [] {
    ArtifactT artifact;
    artifact.producer = "test";
    artifact.output.Set(planMini());
    return io::writeArtifact(artifact);
  };
  EXPECT_EQ(bytes(), bytes());
}

TEST(CapacityPlanner, ReportsItsResultAndHowFarItHasGot) {
  std::vector<std::vector<Figure>> results;
  std::vector<std::string> steps;
  std::set<std::string> tasks;
  const Report report(
      Detail::Steps,
      {.line =
           [&steps](const Detail detail, const std::string_view text) {
             if (detail == Detail::Steps) {
               steps.emplace_back(text);
             }
           },
       .result =
           [&results](const std::vector<Figure>& figures,
                      const std::optional<std::uint64_t> /*fails*/) {
             results.push_back(figures);
           },
       .progress =
           [&tasks](const std::string_view task,
                    const std::string_view /*detail*/,
                    const std::uint64_t /*done*/,
                    const std::uint64_t /*total*/) { tasks.emplace(task); }});

  const auto plan = planMini(report);

  ASSERT_EQ(results.size(), 1U);
  ASSERT_EQ(results[0].size(), 3U);
  EXPECT_EQ(results[0][0].text, counted(plan.partitions.size(), "partition"));
  EXPECT_EQ(results[0][1].text, counted(plan.bottlenecks.size(), "bottleneck"));
  EXPECT_EQ(results[0][2].text, counted(plan.chains.size(), "chain"));
  ASSERT_FALSE(steps.empty());
  EXPECT_EQ(steps[0], "capacity grid 12×10 · detail grid 360×300");
  EXPECT_FALSE(tasks.empty());
}

TEST(CapacityScene, TakesTheDefaultGridWhenTheConfigurationLeavesItOut) {
  // A configuration may leave out [grid] when every value in it is a
  // default, and plans as if it carried the section with its defaults.
  auto bare = test::miniConfig();
  bare.grid.reset();
  auto defaults = test::miniConfig();
  defaults.grid = std::make_unique<flatbuffers::config::GridParamsT>();
  const auto chip = test::miniChip();

  const auto withoutGrid = buildScene(chip, bare);
  const auto withDefaults = buildScene(chip, defaults);

  EXPECT_EQ(withoutGrid.capacity.width, withDefaults.capacity.width);
  EXPECT_EQ(withoutGrid.detail.height, withDefaults.detail.height);
  EXPECT_EQ(withoutGrid.blocked, withDefaults.blocked);
  EXPECT_EQ(withoutGrid.targetCell, withDefaults.targetCell);
}

TEST(CapacityScene, TakesTheCellsAlongYFromTheConfigurationWhenGiven) {
  auto config = test::miniConfig();
  config.grid->capacity_cells_y = 7;

  const auto scene = buildScene(test::miniChip(), config);

  EXPECT_EQ(scene.capacity.width, 12U);
  EXPECT_EQ(scene.capacity.height, 7U);
}

TEST(CapacityScene, TakesTheDefaultCrossingPitchWithoutACapacitySection) {
  auto config = test::miniConfig();
  config.stages->capacity.reset();
  EXPECT_DOUBLE_EQ(crossingPitch(config),
                   flatbuffers::config::CapacityParamsT{}.crossing_pitch);
  config.stages.reset();
  EXPECT_DOUBLE_EQ(crossingPitch(config),
                   flatbuffers::config::CapacityParamsT{}.crossing_pitch);
}

TEST(CapacityScene, RefusesAConfigurationItCannotBuildAGridFrom) {
  const auto chip = test::miniChip();
  auto noRules = test::miniConfig();
  noRules.rules.reset();
  EXPECT_THROW(static_cast<void>(buildScene(chip, noRules)),
               std::invalid_argument);

  auto noDetail = test::miniConfig();
  noDetail.grid->detail_factor = 0;
  EXPECT_THROW(static_cast<void>(buildScene(chip, noDetail)),
               std::invalid_argument);
}

TEST(CapacityScene, RefusesAChipWithoutALauncher) {
  auto chip = test::miniChip();
  for (auto& port : chip.ports) {
    if (port->role == flatbuffers::design::UnassignedRole::Launcher) {
      port->role = flatbuffers::design::UnassignedRole::Unset;
    }
  }
  EXPECT_THROW(static_cast<void>(buildScene(chip, test::miniConfig())),
               std::invalid_argument);
}

/// The fixture with a block of artwork in front of Q2.port1, inside the reach
/// of its band. The band runs into the artwork, so the stage takes it back and
/// places the target beyond the artwork.
ChipT blockedApproachChip() {
  auto text = test::miniChipText();
  const std::string anchor = R"("obstacles": [)";
  const auto at = text.find(anchor);
  EXPECT_NE(at, std::string::npos);
  text.insert(at + anchor.size(), R"(
    {"polygon": [[1500.0, 150.0], [1560.0, 150.0], [1560.0, 250.0],
                 [1500.0, 250.0]]},)");
  return io::loadChip(text, test::miniConfig());
}

TEST(CapacityPlanner, GivesEveryPartitionALabelOfItsOwn) {
  // The cells of the band that was taken back stay reserved for the approach
  // of their port, and a reserved cell is part of no partition.
  const auto plan = makeWatershedPlanner()->run(blockedApproachChip(),
                                                test::miniConfig(), {});

  ASSERT_FALSE(plan.partitions.empty());
  for (const auto& partition : plan.partitions) {
    EXPECT_GE(partition->label, grid::FIRST_PARTITION_LABEL);
  }
  for (const auto& border : plan.borders) {
    EXPECT_GE(border->first, grid::FIRST_PARTITION_LABEL);
  }
}

TEST(CapacityScene, FreesTheCellOfEveryLauncher) {
  const auto scene = buildScene(test::miniChip(), test::miniConfig());

  ASSERT_EQ(scene.launcherCell.size(), 4U);
  for (const auto cell : scene.launcherCell) {
    EXPECT_FALSE(scene.blocked.test(cell));
  }
  // Every routable port of the ring has a target on free space.
  EXPECT_EQ(scene.targetCell.size(), 5U);
  for (const auto cell : scene.targetCell) {
    EXPECT_FALSE(scene.blocked.test(cell));
  }
}

} // namespace
} // namespace mqt::scpd::pipeline
