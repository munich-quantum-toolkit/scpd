/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// The Capacity stage on the two-qubit fixture and on small chips built here:
// what its plan holds, that it is the same on every run, and what it reports.

#include "MiniFixture.hpp"
#include "mqt-scpd/flatbuffers/artifacts.hpp"
#include "mqt-scpd/flatbuffers/config.hpp"
#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/grid/Partitions.hpp"
#include "mqt-scpd/grid/Watershed.hpp"
#include "mqt-scpd/io/Artifacts.hpp"
#include "mqt-scpd/io/Chip.hpp"
#include "mqt-scpd/pipeline/CapacityPlanner.hpp"
#include "mqt-scpd/pipeline/Report.hpp"
#include "mqt-scpd/pipeline/Stages.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
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

/// The label of the partition whose outlines hold a point, by the even-odd
/// rule over all of its rings, or none.
std::optional<std::uint32_t> partitionAt(const CapacityPlanT& plan,
                                         const double x, const double y) {
  for (const auto& partition : plan.partitions) {
    bool inside = false;
    for (const auto& outline : partition->outlines) {
      const auto& ring = outline->vertices;
      for (std::size_t index = 0, previous = ring.size() - 1;
           index < ring.size(); previous = index++) {
        const auto& a = ring[index];
        const auto& b = ring[previous];
        if ((a.y() > y) != (b.y() > y) &&
            x < a.x() + ((y - a.y()) * (b.x() - a.x()) / (b.y() - a.y()))) {
          inside = !inside;
        }
      }
    }
    if (inside) {
      return partition->label;
    }
  }
  return std::nullopt;
}

/// A corridor from Q1 on the left to the launcher on the right, pinched twice.
/// Q2 sits between the pinches, so the chain of Q1 keeps both gates: the near
/// one has a target and the far one beyond it.
ChipT pinchedCorridorChip(const ConfigT& config) {
  const auto rect = [](const double x0, const double y0, const double x1,
                       const double y1) {
    return std::format(
        R"({{"polygon": [[{}, {}], [{}, {}], [{}, {}], [{}, {}]]}})", x0, y0,
        x1, y0, x1, y1, x0, y1);
  };
  const auto pinch = [&rect](const double x, const double gap) {
    return rect(x, 0.0, x + 10.0, 55.0 - (gap / 2.0)) + ", " +
           rect(x, 55.0 + (gap / 2.0), x + 10.0, 110.0);
  };
  const auto text = std::format(
      R"({{"sampleSpacing": 1.0, "nets": [],
  "obstacles": [{}, {}, {}, {}],
  "ports": {{
    "Chip.port0": {{"center": [310.0, 55.0], "orientation": 180.0}},
    "Q1.port0": {{"center": [40.0, 55.0], "orientation": 0.0}},
    "Q2.port0": {{"center": [150.0, 20.0], "orientation": 90.0}}}}}})",
      rect(0.0, 45.0, 40.0, 65.0), rect(145.0, 0.0, 155.0, 20.0),
      pinch(100.0, 20.0), pinch(200.0, 30.0));
  return io::loadChip(text, config);
}

/// The configuration of pinchedCorridorChip: one detail cell per capacity
/// cell, five layout units wide.
ConfigT pinchedCorridorConfig() {
  auto config = test::miniConfig();
  config.ports->patterns->conventional = "^$";
  config.ports->sequences->all_outer = {"Q1.port0", "Q2.port0"};
  config.ports->sequences->fixed_outer = {"Q1.port0"};
  config.rules->min_wire_spacing = 10.0;
  config.rules->min_obstacle_spacing = 5.0;
  config.rules->min_bend_radius = 5.0;
  config.rules->min_straight_length = 10.0;
  config.grid->capacity_cells_x = 62;
  config.grid->detail_factor = 1;
  return config;
}

TEST(CapacityPlanner, PutsATargetInThePartitionAroundIt) {
  // Every chamber beyond a gate is a partition of its own, and the chamber
  // of the target holds the target as well.
  const auto config = pinchedCorridorConfig();
  const auto chip = pinchedCorridorChip(config);
  const auto scene = buildScene(chip, config);
  const auto plan = makeWatershedPlanner()->run(chip, config, {});
  ASSERT_EQ(plan.bottlenecks.size(), 2U);

  ASSERT_EQ(scene.targetCell.size(), 2U);
  for (const auto cell : scene.targetCell) {
    const auto at = scene.detail.cell(cell);
    const auto centre = scene.detail.toLayout(at.x(), at.y());
    const auto label = partitionAt(plan, centre.x(), centre.y());
    ASSERT_TRUE(label.has_value());
    std::size_t neighbours = 0;
    for (const auto& [dx, dy] :
         {std::pair{-1.0, 0.0}, {1.0, 0.0}, {0.0, -1.0}, {0.0, 1.0}}) {
      const auto there = scene.detail.toLayout(at.x() + dx, at.y() + dy);
      if (const auto beside = partitionAt(plan, there.x(), there.y())) {
        EXPECT_EQ(*beside, *label)
            << "target cell (" << at.x() << ", " << at.y() << ")";
        ++neighbours;
      }
    }
    EXPECT_GT(neighbours, 0U);
  }
}

/// A border along the middle row of the detail grid, from one edge of the
/// grid to the other.
grid::PartitionBorder middleRow(const CapacityScene& scene) {
  grid::PartitionBorder border{.first = 2, .second = 3, .samples = {}};
  const auto row = std::floor(static_cast<double>(scene.detail.height) / 2.0);
  for (std::uint32_t x = 0; x < scene.detail.width; ++x) {
    border.samples.emplace_back(static_cast<double>(x) + 0.5, row);
  }
  return border;
}

TEST(CapacityScene, OffersOnlyTheCrossingSlotsInsideTheLauncherRing) {
  const auto config = test::miniConfig();
  const auto scene = buildScene(test::miniChip(), config);
  double minX = 0.0;
  double minY = 0.0;
  double maxX = 0.0;
  double maxY = 0.0;
  for (std::size_t index = 0; index < scene.launcherCell.size(); ++index) {
    const auto cell = scene.detail.cell(scene.launcherCell[index]);
    const auto x = static_cast<double>(cell.x()) + 0.5;
    const auto y = static_cast<double>(cell.y()) + 0.5;
    minX = index == 0 ? x : std::min(minX, x);
    minY = index == 0 ? y : std::min(minY, y);
    maxX = index == 0 ? x : std::max(maxX, x);
    maxY = index == 0 ? y : std::max(maxY, y);
  }
  // A border along the middle row from one edge of the grid to the other
  // runs past the launchers on both sides.
  const auto border = middleRow(scene);
  const auto pitch = crossingPitch(config);

  const auto all = grid::borderSlots(border, scene.detail, pitch);
  const auto offered = crossingSlots(border, scene, pitch);

  EXPECT_LT(offered.size(), all.size());
  EXPECT_FALSE(offered.empty());
  for (const auto& slot : offered) {
    EXPECT_GT(slot.x(), minX);
    EXPECT_LT(slot.x(), maxX);
    EXPECT_GT(slot.y(), minY);
    EXPECT_LT(slot.y(), maxY);
  }
  for (const auto& slot : all) {
    const auto inside = slot.x() > minX && slot.x() < maxX && slot.y() > minY &&
                        slot.y() < maxY;
    EXPECT_EQ(inside, std::ranges::any_of(offered, [&](const auto& kept) {
                return kept.x() == slot.x() && kept.y() == slot.y();
              }));
  }
}

/// The fixture with its four launchers on one line, at the left edge of the
/// chip where Chip.port0 already sits.
ChipT launcherLineChip() {
  auto text = test::miniChipText();
  for (const auto& [from, to] : {std::pair<std::string_view, std::string_view>{
                                     "[300.0, 1600.0]", "[-600.0, 1000.0]"},
                                 {"[2000.0, 200.0]", "[-600.0, 600.0]"},
                                 {"[700.0, -600.0]", "[-600.0, -200.0]"}}) {
    const auto at = text.find(from);
    EXPECT_NE(at, std::string::npos) << from;
    text.replace(at, from.size(), to);
  }
  return io::loadChip(text, test::miniConfig());
}

TEST(CapacityScene, OffersTheCrossingSlotsAlongALineOfLaunchers) {
  // The launchers span no interval along x, so the grid bounds the slots
  // there. Along y the launcher cells still bound them.
  const auto config = test::miniConfig();
  const auto scene = buildScene(launcherLineChip(), config);
  ASSERT_EQ(scene.launcherCell.size(), 4U);
  double minY = 0.0;
  double maxY = 0.0;
  for (std::size_t index = 0; index < scene.launcherCell.size(); ++index) {
    const auto y =
        static_cast<double>(scene.detail.cell(scene.launcherCell[index]).y()) +
        0.5;
    minY = index == 0 ? y : std::min(minY, y);
    maxY = index == 0 ? y : std::max(maxY, y);
  }
  const auto border = middleRow(scene);
  ASSERT_GT(border.samples.front().y(), minY);
  ASSERT_LT(border.samples.front().y(), maxY);
  const auto pitch = crossingPitch(config);

  const auto all = grid::borderSlots(border, scene.detail, pitch);
  const auto offered = crossingSlots(border, scene, pitch);

  EXPECT_FALSE(offered.empty());
  EXPECT_EQ(offered.size(), all.size());
}

TEST(CapacityScene, OffersTheCrossingSlotsAroundASingleLauncher) {
  // One launcher spans no interval on either axis, so the grid bounds the
  // slots on both.
  auto chip = test::miniChip();
  for (auto& port : chip.ports) {
    if (port->role == flatbuffers::design::UnassignedRole::Launcher &&
        port->label != "Chip.port0") {
      port->role = flatbuffers::design::UnassignedRole::Unset;
    }
  }
  const auto config = test::miniConfig();
  const auto scene = buildScene(chip, config);
  ASSERT_EQ(scene.launcherCell.size(), 1U);
  const auto border = middleRow(scene);
  const auto pitch = crossingPitch(config);

  const auto all = grid::borderSlots(border, scene.detail, pitch);
  const auto offered = crossingSlots(border, scene, pitch);

  EXPECT_FALSE(offered.empty());
  EXPECT_EQ(offered.size(), all.size());
}

} // namespace
} // namespace mqt::scpd::pipeline
