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
#include <limits>
#include <map>
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

TEST(CapacityPlanner, DrawsThePartitionsAroundTheirCells) {
  // A cell of the detail grid covers half a cell step on each side of its
  // layout point. The partitions cover the free cells, so their outlines
  // reach half a cell step beyond the grid box on every side and no further.
  const auto config = test::miniConfig();
  const auto chip = test::miniChip(config);
  const auto plan = makeWatershedPlanner()->run(chip, config, {});
  const auto detail = buildScene(chip, config).detail;
  const auto box = detail.box();
  const auto halfX = detail.cellWidth / 2.0;
  const auto halfY = detail.cellHeight / 2.0;
  constexpr auto tolerance = 1.0e-9;

  auto lowX = std::numeric_limits<double>::max();
  auto lowY = std::numeric_limits<double>::max();
  auto highX = std::numeric_limits<double>::lowest();
  auto highY = std::numeric_limits<double>::lowest();
  for (const auto& partition : plan.partitions) {
    for (const auto& ring : partition->outlines) {
      for (const auto& vertex : ring->vertices) {
        lowX = std::min(lowX, vertex.x());
        lowY = std::min(lowY, vertex.y());
        highX = std::max(highX, vertex.x());
        highY = std::max(highY, vertex.y());
      }
    }
  }
  EXPECT_NEAR(lowX, box.minX - halfX, tolerance);
  EXPECT_NEAR(lowY, box.minY - halfY, tolerance);
  EXPECT_NEAR(highX, box.maxX + halfX, tolerance);
  EXPECT_NEAR(highY, box.maxY + halfY, tolerance);
}

TEST(CapacityPlanner, SaysWhichKeyToLowerWhenThePartitionLabelsRunOut) {
  // Every free capacity cell seeds a partition, and a plan holds fewer than
  // 65536 of them.
  auto config = test::miniConfig();
  config.grid->capacity_cells_x = 400;
  config.grid->detail_factor = 1;
  try {
    static_cast<void>(
        makeWatershedPlanner()->run(test::miniChip(config), config, {}));
    FAIL() << "a grid with more free cells than labels has to be refused";
  } catch (const std::length_error& error) {
    EXPECT_NE(std::string(error.what()).find("lower [grid] capacity_cells_x"),
              std::string::npos)
        << error.what();
  }
}

TEST(CapacityPlanner, TreatsEveryClearanceBeyondTheWidestPlaceAlike) {
  // Both clearances are wider than any free place of the fixture, so every
  // place counts as narrow under either. The larger one squares to more than
  // the cell counter holds.
  auto wide = test::miniConfig();
  wide.stages->capacity->bottleneck_clearance = 1.0e4;
  auto wider = test::miniConfig();
  wider.stages->capacity->bottleneck_clearance = 1.0e9;

  const auto first = makeWatershedPlanner()->run(test::miniChip(), wide, {});
  const auto second = makeWatershedPlanner()->run(test::miniChip(), wider, {});
  EXPECT_EQ(first, second);
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

  auto tooFine = test::miniConfig();
  tooFine.grid->detail_factor = 1000;
  EXPECT_THROW(static_cast<void>(buildScene(chip, tooFine)),
               std::invalid_argument);
  EXPECT_THROW(static_cast<void>(launcherSlotPositions(chip, tooFine)),
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

TEST(CapacityPlanner, GivesASlotToEveryLauncherWithADirectionAndNoOtherPort) {
  auto chip = test::miniChip();
  const auto launcher = std::ranges::find_if(chip.ports, [](const auto& port) {
    return port->role == flatbuffers::design::UnassignedRole::Launcher;
  });
  ASSERT_NE(launcher, chip.ports.end());
  EXPECT_TRUE(hasLauncherSlot(**launcher));
  // An orientation that is not a number gives no direction.
  (*launcher)->orientation = std::nan("");
  EXPECT_FALSE(hasLauncherSlot(**launcher));

  std::vector<std::uint32_t> slotted;
  for (std::uint32_t index = 0; index < chip.ports.size(); ++index) {
    if (hasLauncherSlot(*chip.ports[index])) {
      slotted.push_back(index);
    }
  }
  const auto plan =
      makeWatershedPlanner()->run(chip, test::miniConfig(), Report{});
  std::vector<std::uint32_t> planned;
  planned.reserve(plan.launchers.size());
  for (const auto& slot : plan.launchers) {
    planned.push_back(slot->port.index());
  }
  EXPECT_EQ(slotted.size(), 3U);
  EXPECT_EQ(planned, slotted);
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
/// one has a target and the far one beyond it. A block of artwork in front of
/// Q2 is optional.
ChipT pinchedCorridorChip(const ConfigT& config,
                          const bool blockInFrontOfQ2 = false) {
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
  const auto block =
      blockInFrontOfQ2 ? ", " + rect(140.0, 30.0, 160.0, 45.0) : std::string{};
  const auto text = std::format(
      R"({{"sampleSpacing": 1.0, "nets": [],
  "obstacles": [{}, {}, {}, {}{}],
  "ports": {{
    "Chip.port0": {{"center": [310.0, 55.0], "orientation": 180.0}},
    "Q1.port0": {{"center": [40.0, 55.0], "orientation": 0.0}},
    "Q2.port0": {{"center": [150.0, 20.0], "orientation": 90.0}}}}}})",
      rect(0.0, 45.0, 40.0, 65.0), rect(145.0, 0.0, 155.0, 20.0),
      pinch(100.0, 20.0), pinch(200.0, 30.0), block);
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

/// Whether every partition of a plan is one four-connected piece on the
/// detail grid of its scene.
bool everyPartitionIsOnePiece(const CapacityPlanT& plan,
                              const CapacityScene& scene) {
  std::vector<grid::PartitionOutline> outlines;
  auto highest = grid::FIRST_PARTITION_LABEL;
  for (const auto& partition : plan.partitions) {
    const auto label = static_cast<grid::PartitionLabel>(partition->label);
    highest = std::max(highest, label);
    for (const auto& ring : partition->outlines) {
      grid::PartitionOutline outline{.label = label, .ring = {}};
      for (const auto& vertex : ring->vertices) {
        outline.ring.push_back(grid::layoutToCorner(scene.detail, vertex));
      }
      outlines.push_back(std::move(outline));
    }
  }
  auto labels = grid::rasterizePartitions(outlines, scene.detail);
  const auto next = grid::nextPartitionLabel(highest);
  return grid::separatePartitionPieces(scene.blocked, labels, scene.detail,
                                       next) == next;
}

TEST(CapacityPlanner, KeepsEveryPartitionInOnePiece) {
  // A wire crosses a partition from any of its borders to any other, which
  // it can only do when the cells of the partition touch along their edges.
  const auto mini = test::miniChip();
  EXPECT_TRUE(everyPartitionIsOnePiece(planMini(),
                                       buildScene(mini, test::miniConfig())));

  const auto blocked = blockedApproachChip();
  EXPECT_TRUE(everyPartitionIsOnePiece(
      makeWatershedPlanner()->run(blocked, test::miniConfig(), {}),
      buildScene(blocked, test::miniConfig())));

  const auto config = pinchedCorridorConfig();
  const auto pinched = pinchedCorridorChip(config);
  EXPECT_TRUE(
      everyPartitionIsOnePiece(makeWatershedPlanner()->run(pinched, config, {}),
                               buildScene(pinched, config)));
}

TEST(CapacityPlanner, JoinsThePartitionsOnBothSidesOfEveryGate) {
  // With several detail cells per capacity cell, a gate line runs through
  // cells of its own. Those cells belong to the chamber beyond the gate, so
  // the partitions on the two sides of the gate share a border.
  auto config = pinchedCorridorConfig();
  config.grid->capacity_cells_x = 21;
  config.grid->detail_factor = 3;
  const auto plan =
      makeWatershedPlanner()->run(pinchedCorridorChip(config), config, {});
  ASSERT_EQ(plan.bottlenecks.size(), 2U);

  std::set<std::pair<std::uint32_t, std::uint32_t>> borders;
  for (const auto& border : plan.borders) {
    borders.emplace(border->first, border->second);
  }
  for (const auto& gate : plan.bottlenecks) {
    const auto& from = gate->from;
    const auto& to = gate->to;
    const auto length = std::hypot(to.x() - from.x(), to.y() - from.y());
    ASSERT_GT(length, 0.0);
    const auto normalX = -(to.y() - from.y()) / length;
    const auto normalY = (to.x() - from.x()) / length;
    const auto middleX = (from.x() + to.x()) / 2.0;
    const auto middleY = (from.y() + to.y()) / 2.0;
    // The first partition met when walking away from the gate on one side,
    // from the centre of the detail cell next to the cells that the gate line
    // runs through. Those cells belong to the chamber beyond, which may be
    // no more than two cells wide.
    const auto beyond = static_cast<int>(std::ceil(
        std::max(plan.detail_grid->cell_width, plan.detail_grid->cell_height)));
    const auto sideOf = [&](const double sign) -> std::optional<std::uint32_t> {
      for (int units = beyond; units <= beyond + 20; ++units) {
        const auto step = sign * static_cast<double>(units);
        if (const auto label = partitionAt(plan, middleX + (step * normalX),
                                           middleY + (step * normalY))) {
          return label;
        }
      }
      return std::nullopt;
    };
    const auto before = sideOf(-1.0);
    const auto after = sideOf(1.0);
    ASSERT_TRUE(before.has_value());
    ASSERT_TRUE(after.has_value());
    ASSERT_NE(*before, *after);
    EXPECT_TRUE(borders.contains(std::minmax(*before, *after)))
        << "no border between partitions " << *before << " and " << *after;
  }
}

TEST(CapacityPlanner, LeavesTheBandItTookBackOutOfEveryPartition) {
  // The band in front of Q2 runs into the block, so the stage takes it back
  // and leaves its cells free. They stay reserved for the approach of the
  // port, so no partition holds them, although the chain of Q1 walks past.
  const auto config = pinchedCorridorConfig();
  const auto chip = pinchedCorridorChip(config, true);
  const auto scene = buildScene(chip, config);
  const auto plan = makeWatershedPlanner()->run(chip, config, {});

  std::size_t freeReserved = 0;
  for (std::size_t cell = 0; cell < scene.detail.cells(); ++cell) {
    if (!scene.reserved.test(cell) || scene.blocked.test(cell)) {
      continue;
    }
    ++freeReserved;
    const auto at = scene.detail.cell(cell);
    const auto centre = scene.detail.toLayout(at.x(), at.y());
    EXPECT_FALSE(partitionAt(plan, centre.x(), centre.y()).has_value())
        << "reserved cell (" << at.x() << ", " << at.y() << ")";
  }
  EXPECT_GT(freeReserved, 0U);
}

/// What lies in the alcove of twoPinchChip.
enum class Alcove : std::uint8_t { None, Empty, WithTarget };

/// A corridor from Q1 on the left to the launcher on the right with two
/// pinches in a row and nothing between them. An alcove can open off the
/// corridor between the pinches through a narrow mouth, empty or with Q2 in
/// it.
ChipT twoPinchChip(const ConfigT& config, const double nearGap,
                   const double farGap, const Alcove alcove = Alcove::None) {
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
  // The alcove is a room beside the corridor, from 130 to 180 along it and
  // from 130 to 170 across, entered through a mouth 20 wide and 20 deep at
  // its middle.
  const auto room = alcove == Alcove::None
                        ? std::string{}
                        : ", " + rect(0.0, 110.0, 130.0, 170.0) + ", " +
                              rect(180.0, 110.0, 310.0, 170.0) + ", " +
                              rect(130.0, 110.0, 145.0, 130.0) + ", " +
                              rect(165.0, 110.0, 180.0, 130.0) + ", " +
                              rect(0.0, 170.0, 310.0, 180.0);
  const auto q2 = alcove == Alcove::WithTarget ? std::string{R"(,
    "Q2.port0": {"center": [155.0, 160.0], "orientation": 270.0})"}
                                               : std::string{};
  const auto text = std::format(
      R"({{"sampleSpacing": 1.0, "nets": [],
  "obstacles": [{}, {}, {}{}],
  "ports": {{
    "Chip.port0": {{"center": [310.0, 55.0], "orientation": 180.0}},
    "Q1.port0": {{"center": [40.0, 55.0], "orientation": 0.0}}{}}}}})",
      rect(0.0, 45.0, 40.0, 65.0), pinch(100.0, nearGap), pinch(200.0, farGap),
      room, q2);
  return io::loadChip(text, config);
}

/// The configuration of twoPinchChip, with Q2 in the ring when it is there.
ConfigT twoPinchConfig(const bool withQ2 = false) {
  auto config = pinchedCorridorConfig();
  config.ports->sequences->all_outer = {"Q1.port0"};
  if (withQ2) {
    config.ports->sequences->all_outer.emplace_back("Q2.port0");
  }
  return config;
}

/// The x coordinate of the middle of each bottleneck of a plan.
std::vector<double> bottleneckColumns(const CapacityPlanT& plan) {
  std::vector<double> columns;
  columns.reserve(plan.bottlenecks.size());
  for (const auto& gate : plan.bottlenecks) {
    columns.push_back((gate->from.x() + gate->to.x()) / 2.0);
  }
  return columns;
}

/// Whether a plan keeps a bottleneck at the mouth of the alcove.
bool keepsTheMouth(const CapacityPlanT& plan) {
  return std::ranges::any_of(plan.bottlenecks, [](const auto& gate) {
    return std::max(gate->from.y(), gate->to.y()) > 105.0;
  });
}

TEST(CapacityPlanner, KeepsOnlyTheTighterOfTwoGatesInARow) {
  // The wires that pass one pinch are the wires that reach the other, so the
  // chain keeps the pinch that carries fewer wires, near or far.
  const auto config = twoPinchConfig();
  for (const auto& [nearGap, farGap, column] :
       {std::tuple{20.0, 40.0, 105.0}, std::tuple{40.0, 20.0, 205.0}}) {
    SCOPED_TRACE(std::format("gaps {} and {}", nearGap, farGap));
    const auto plan = makeWatershedPlanner()->run(
        twoPinchChip(config, nearGap, farGap), config, {});
    ASSERT_EQ(plan.chains.size(), 1U);
    const auto columns = bottleneckColumns(plan);
    ASSERT_EQ(columns.size(), 1U);
    EXPECT_NEAR(columns.front(), column, 10.0);
  }
}

TEST(CapacityPlanner, DropsAGateThatLeadsNowhere) {
  // With Q2 in the alcove, wires pass its mouth and a chain keeps it. With
  // the alcove empty, nothing beyond the mouth is a target or a launcher, so
  // no chain keeps it.
  const auto ring = twoPinchConfig(true);
  const auto served = makeWatershedPlanner()->run(
      twoPinchChip(ring, 20.0, 40.0, Alcove::WithTarget), ring, {});
  EXPECT_TRUE(keepsTheMouth(served));

  const auto config = twoPinchConfig();
  const auto empty = makeWatershedPlanner()->run(
      twoPinchChip(config, 20.0, 40.0, Alcove::Empty), config, {});
  EXPECT_FALSE(keepsTheMouth(empty));
  EXPECT_EQ(bottleneckColumns(empty).size(), 1U);
}

/// How many chains reach each target port of a plan, counting a target twice
/// on one chain twice.
std::map<std::uint32_t, std::size_t>
chainsPerTarget(const CapacityPlanT& plan) {
  std::map<std::uint32_t, std::size_t> chains;
  for (const auto root : plan.chains) {
    std::vector<std::uint32_t> pending{root};
    while (!pending.empty()) {
      const auto& node = *plan.nodes[pending.back()];
      pending.pop_back();
      if (node.kind == CapacityElement::Target) {
        ++chains[node.id];
      }
      pending.insert(pending.end(), node.next.begin(), node.next.end());
    }
  }
  return chains;
}

TEST(CapacityPlanner, PutsEveryTargetOnOneChain) {
  // The Global stage asks one wire of each target over its chain, so a
  // target on two chains would ask for its wire twice.
  const auto pinched = pinchedCorridorConfig();
  const auto alcove = twoPinchConfig(true);
  for (const auto& [name, plan] :
       std::vector<std::pair<std::string, CapacityPlanT>>{
           {"mini", planMini()},
           {"blocked approach",
            makeWatershedPlanner()->run(blockedApproachChip(),
                                        test::miniConfig(), {})},
           {"pinched corridor", makeWatershedPlanner()->run(
                                    pinchedCorridorChip(pinched), pinched, {})},
           {"alcove with a target",
            makeWatershedPlanner()->run(
                twoPinchChip(alcove, 20.0, 40.0, Alcove::WithTarget), alcove,
                {})}}) {
    SCOPED_TRACE(name);
    const auto chains = chainsPerTarget(plan);
    EXPECT_FALSE(chains.empty());
    for (const auto& [port, count] : chains) {
      EXPECT_EQ(count, 1U) << "port " << port;
    }
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

TEST(CapacityScene, PicksTheCrossingSlotsFromTheSamplesInsideTheLauncherRing) {
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

  const auto offered = crossingSlots(border, scene, pitch);

  // The slots are picked from the samples inside the ring alone. A slot
  // picked outside the ring and dropped afterwards would keep the samples
  // within one pitch of it from becoming slots, so the walk over the inside
  // samples starts at the first of them and takes it.
  auto inside = border;
  std::erase_if(inside.samples, [&](const grid::Point& sample) {
    return sample.x() <= minX || sample.x() >= maxX || sample.y() <= minY ||
           sample.y() >= maxY;
  });
  ASSERT_FALSE(inside.samples.empty());
  ASSERT_LT(inside.samples.size(), border.samples.size());
  const auto expected = grid::borderSlots(inside, scene.detail, pitch);
  ASSERT_EQ(offered.size(), expected.size());
  for (std::size_t index = 0; index < offered.size(); ++index) {
    EXPECT_EQ(offered[index].x(), expected[index].x());
    EXPECT_EQ(offered[index].y(), expected[index].y());
  }
  EXPECT_EQ(offered.front().x(), inside.samples.front().x());
  for (const auto& slot : offered) {
    EXPECT_GT(slot.x(), minX);
    EXPECT_LT(slot.x(), maxX);
    EXPECT_GT(slot.y(), minY);
    EXPECT_LT(slot.y(), maxY);
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
  // there. No launcher sits on the top or the bottom edge, so the grid bounds
  // them along y as well.
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

TEST(CapacityScene, OffersTheCrossingSlotsBeyondLaunchersOnTwoOppositeEdges) {
  // Chip.port0 sits on the left edge and Chip.port2 on the right edge, lower
  // down. No launcher sits on the top edge, so a border above both of them
  // still offers every slot.
  auto text = test::miniChipText();
  const std::string_view from = "[2000.0, 200.0]";
  const auto at = text.find(from);
  ASSERT_NE(at, std::string::npos);
  text.replace(at, from.size(), "[2000.0, -300.0]");
  auto chip = io::loadChip(text, test::miniConfig());
  for (auto& port : chip.ports) {
    if (port->label == "Chip.port1" || port->label == "Chip.port3") {
      port->role = flatbuffers::design::UnassignedRole::Unset;
    }
  }
  const auto config = test::miniConfig();
  const auto scene = buildScene(chip, config);
  ASSERT_EQ(scene.launcherCell.size(), 2U);
  auto border = middleRow(scene);
  const auto row = std::floor(static_cast<double>(scene.detail.height) * 0.8);
  for (auto& sample : border.samples) {
    sample = grid::Point(sample.x(), row);
  }
  for (const auto cell : scene.launcherCell) {
    ASSERT_GT(row, static_cast<double>(scene.detail.cell(cell).y()) + 0.5);
  }
  const auto pitch = crossingPitch(config);

  const auto offered = crossingSlots(border, scene, pitch);

  EXPECT_FALSE(offered.empty());
  for (const auto& slot : offered) {
    EXPECT_EQ(slot.y(), row);
  }
}

} // namespace
} // namespace mqt::scpd::pipeline
