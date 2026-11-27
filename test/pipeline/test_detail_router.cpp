/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// The Detail stage on the two-qubit fixture: the paths it promises the later
// stages, which is that every wire runs over free cells from its feed to its
// target, no wire meets itself or another wire, and no two wires come within
// the clearance; and what it reports.

#include "MiniFixture.hpp"
#include "mqt-scpd/flatbuffers/artifacts.hpp"
#include "mqt-scpd/flatbuffers/config.hpp"
#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/flatbuffers/geometry.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"
#include "mqt-scpd/grid/Partitions.hpp"
#include "mqt-scpd/io/Artifacts.hpp"
#include "mqt-scpd/pipeline/Assigner.hpp"
#include "mqt-scpd/pipeline/CapacityPlanner.hpp"
#include "mqt-scpd/pipeline/CorridorRouter.hpp"
#include "mqt-scpd/pipeline/DetailRouter.hpp"
#include "mqt-scpd/pipeline/GlobalRouter.hpp"
#include "mqt-scpd/pipeline/Report.hpp"
#include "mqt-scpd/pipeline/Stages.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <initializer_list>
#include <limits>
#include <map>
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
using flatbuffers::artifacts::DetailWireT;
using flatbuffers::geometry::DCoord;

/// What the stages before the detail router make of the fixture.
struct Planned {
  ChipT chip;
  CapacityPlanT capacity;
  GlobalRoutingT global;
  AssignmentT assignment;
  CorridorRoutingT corridor;
};

Planned planMini(const ConfigT& config = test::miniConfig()) {
  Planned planned{.chip = test::miniChip(config),
                  .capacity = {},
                  .global = {},
                  .assignment = {},
                  .corridor = {}};
  planned.capacity = makeWatershedPlanner()->run(planned.chip, config, {});
  planned.global =
      makeHananMilpRouter()->run(planned.chip, planned.capacity, config, {});
  planned.assignment = makeOrderedMilpAssigner()->run(
      planned.chip, planned.capacity, planned.global, config, {});
  planned.corridor = makePartitionAStarRouter()->run(
      planned.chip, planned.capacity, planned.assignment, config, {});
  return planned;
}

DetailRoutingT drawMini(const Planned& planned, const Report& report = {},
                        const ConfigT& config = test::miniConfig()) {
  return makePixelAStarRouter()->run(planned.chip, planned.capacity,
                                     planned.global, planned.assignment,
                                     planned.corridor, config, report);
}

/// The message the stage refuses its inputs with, or an empty one when it
/// draws them.
std::string refusalOf(const Planned& planned,
                      const ConfigT& config = test::miniConfig()) {
  try {
    static_cast<void>(drawMini(planned, {}, config));
  } catch (const std::invalid_argument& error) {
    return error.what();
  }
  return {};
}

/// Every wire of a routing, the ring first and then the inner circuit, with
/// its name for a failure message.
std::vector<std::pair<std::string, const DetailWireT*>>
wiresOf(const DetailRoutingT& routing) {
  std::vector<std::pair<std::string, const DetailWireT*>> wires;
  wires.reserve(routing.wires.size() + routing.inner.size());
  for (std::size_t index = 0; index < routing.wires.size(); ++index) {
    wires.emplace_back(std::format("wire {}", index),
                       routing.wires[index].get());
  }
  for (std::size_t index = 0; index < routing.inner.size(); ++index) {
    wires.emplace_back(std::format("inner wire {}", index),
                       routing.inner[index].get());
  }
  return wires;
}

/// How many wires of a routing have no path.
std::size_t undrawnOf(const DetailRoutingT& routing) {
  std::size_t undrawn = 0;
  for (const auto& [name, wire] : wiresOf(routing)) {
    undrawn += wire->path.empty() ? 1U : 0U;
  }
  return undrawn;
}

/// The clearance of the stage in cells: the rule spans
/// `ceil(min_wire_spacing / cell)` cells, and the stage keeps one cell less
/// than that clear.
std::uint32_t clearanceOf(const ConfigT& config,
                          const grid::GridMetrics& detail) {
  const auto spans = grid::cellsFor(config.rules->min_wire_spacing, detail);
  return spans == 0 ? 0 : spans - 1;
}

/// Checks that every wire is a run of eight-connected, free cells. The
/// blocked cells are no artifact, so the check rebuilds them from the chip
/// and the configuration, as the stage does.
void expectFreeConnectedPaths(const DetailRoutingT& routing,
                              const CapacityScene& scene) {
  for (const auto& [name, wire] : wiresOf(routing)) {
    const auto& path = wire->path;
    for (std::size_t step = 0; step < path.size(); ++step) {
      ASSERT_TRUE(scene.detail.contains(path[step].x(), path[step].y()))
          << name << " leaves the grid";
      const auto cell =
          (static_cast<std::size_t>(path[step].y()) * scene.detail.width) +
          path[step].x();
      EXPECT_FALSE(scene.blocked.test(cell))
          << name << " runs over a blocked cell at (" << path[step].x() << ", "
          << path[step].y() << ")";
      if (step == 0) {
        continue;
      }
      const auto dx = std::max(path[step].x(), path[step - 1].x()) -
                      std::min(path[step].x(), path[step - 1].x());
      const auto dy = std::max(path[step].y(), path[step - 1].y()) -
                      std::min(path[step].y(), path[step - 1].y());
      EXPECT_TRUE(dx <= 1 && dy <= 1 && dx + dy > 0)
          << name << " steps from (" << path[step - 1].x() << ", "
          << path[step - 1].y() << ") to (" << path[step].x() << ", "
          << path[step].y() << ")";
    }
  }
}

/// Checks that no two cells of the routing are the same and that no two
/// diagonal steps cross inside one two-by-two block, within one wire or
/// between two. Either is a short in copper.
void expectNoWireMeetsAnother(const DetailRoutingT& routing,
                              const std::uint32_t width) {
  std::map<std::size_t, std::string> owner;
  std::map<std::size_t, std::pair<bool, std::string>> diagonals;
  for (const auto& [name, wire] : wiresOf(routing)) {
    const auto& path = wire->path;
    for (std::size_t step = 0; step < path.size(); ++step) {
      const auto cell =
          (static_cast<std::size_t>(path[step].y()) * width) + path[step].x();
      const auto [entry, added] = owner.emplace(cell, name);
      EXPECT_TRUE(added) << entry->second << " and " << name
                         << " both run over (" << path[step].x() << ", "
                         << path[step].y() << ")";
      if (step == 0 || path[step].x() == path[step - 1].x() ||
          path[step].y() == path[step - 1].y()) {
        continue;
      }
      const auto block = (static_cast<std::size_t>(
                              std::min(path[step].y(), path[step - 1].y())) *
                          width) +
                         std::min(path[step].x(), path[step - 1].x());
      const auto rising = (path[step].x() > path[step - 1].x()) ==
                          (path[step].y() > path[step - 1].y());
      const auto [crossing, first] =
          diagonals.emplace(block, std::pair{rising, name});
      EXPECT_TRUE(first || crossing->second.first == rising)
          << crossing->second.second << " and " << name
          << " cross between the cells (" << path[step - 1].x() << ", "
          << path[step - 1].y() << ") and (" << path[step].x() << ", "
          << path[step].y() << ")";
    }
  }
}

/// Checks that no cell of one wire lies within the clearance of a cell of
/// another wire.
void expectTheClearance(const DetailRoutingT& routing,
                        const grid::GridMetrics& detail,
                        const std::uint32_t clearance) {
  ASSERT_GT(clearance, 0U);
  std::map<std::pair<std::int64_t, std::int64_t>, std::string> owner;
  for (const auto& [name, wire] : wiresOf(routing)) {
    for (const auto& cell : wire->path) {
      owner.emplace(std::pair{cell.x(), cell.y()}, name);
    }
  }
  const auto reach = static_cast<std::int64_t>(clearance);
  for (const auto& [cell, name] : owner) {
    for (auto dy = -reach; dy <= reach; ++dy) {
      for (auto dx = -reach; dx <= reach; ++dx) {
        if ((dx * dx) + (dy * dy) > reach * reach) {
          continue;
        }
        const auto near = owner.find({cell.first + dx, cell.second + dy});
        ASSERT_TRUE(near == owner.end() || near->second == name)
            << name << " at (" << cell.first << ", " << cell.second
            << ") lies within " << clearance << " cells of " << near->second
            << " on a grid of " << detail.width << TIMES << detail.height;
      }
    }
  }
}

TEST(DetailRouter, DrawsEveryWireOfTheFixture) {
  const auto planned = planMini();
  const auto routing = drawMini(planned);

  ASSERT_EQ(routing.wires.size(), planned.assignment.connections.size());
  ASSERT_EQ(routing.inner.size(), planned.global.connections.size());
  ASSERT_FALSE(routing.wires.empty());
  EXPECT_EQ(undrawnOf(routing), 0U);
}

TEST(DetailRouter, DescribesTheDetailGridOfTheScene) {
  const auto planned = planMini();
  const auto routing = drawMini(planned);
  const auto detail = buildScene(planned.chip, test::miniConfig()).detail;

  ASSERT_NE(routing.grid, nullptr);
  EXPECT_EQ(routing.grid->width, detail.width);
  EXPECT_EQ(routing.grid->height, detail.height);
  EXPECT_EQ(routing.grid->origin, detail.origin);
  EXPECT_EQ(routing.grid->cell_width, detail.cellWidth);
  EXPECT_EQ(routing.grid->cell_height, detail.cellHeight);
}

TEST(DetailRouter, DrawsEveryWireOverFreeConnectedCells) {
  const auto planned = planMini();
  const auto routing = drawMini(planned);

  expectFreeConnectedPaths(routing,
                           buildScene(planned.chip, test::miniConfig()));
}

TEST(DetailRouter, KeepsWiresApart) {
  const auto planned = planMini();
  const auto routing = drawMini(planned);
  const auto config = test::miniConfig();
  const auto detail = buildScene(planned.chip, config).detail;

  expectNoWireMeetsAnother(routing, detail.width);
  expectTheClearance(routing, detail, clearanceOf(config, detail));
}

TEST(DetailRouter, RunsEveryWireFromItsFeedToItsTarget) {
  const auto planned = planMini();
  const auto routing = drawMini(planned);
  const auto detail = buildScene(planned.chip, test::miniConfig()).detail;

  // The corridor writes a cell as the layout point of its center in the
  // corner frame, so the check compares layout points.
  const auto centerOf = [&detail](const DCoord& cell) {
    return grid::cornerToLayout(detail, cell.x() + 0.5, cell.y() + 0.5);
  };
  const auto same = [](const auto& a, const auto& b) {
    return std::abs(a.x() - b.x()) < 1.0e-6 && std::abs(a.y() - b.y()) < 1.0e-6;
  };
  for (std::size_t index = 0; index < routing.wires.size(); ++index) {
    const auto& path = routing.wires[index]->path;
    ASSERT_FALSE(path.empty()) << "wire " << index;
    const auto& corridor = *planned.corridor.corridors[index];
    ASSERT_NE(corridor.source, nullptr);
    ASSERT_NE(corridor.target, nullptr);
    EXPECT_TRUE(same(centerOf(path.front()), *corridor.source))
        << "wire " << index << " starts at (" << path.front().x() << ", "
        << path.front().y() << ")";
    EXPECT_TRUE(same(centerOf(path.back()), *corridor.target))
        << "wire " << index << " ends at (" << path.back().x() << ", "
        << path.back().y() << ")";
  }
}

TEST(DetailRouter, WritesAnArtifactThatPassesValidation) {
  const auto planned = planMini();
  ArtifactT artifact;
  artifact.producer = "test";
  artifact.output.Set(drawMini(planned));

  EXPECT_TRUE(io::validate(artifact).empty());
}

TEST(DetailRouter, LeavesAConnectionWithoutACorridorUndrawnAndUncounted) {
  // The Corridor stage counts the connection among its own fails, and the
  // closing line of a run adds the fails of every stage, so the Detail stage
  // does not count the connection again.
  auto planned = planMini();
  ASSERT_FALSE(planned.corridor.corridors.empty());
  planned.corridor.corridors.front()->partitions.clear();
  planned.corridor.corridors.front()->crossings.clear();
  std::optional<std::uint64_t> fails;
  const Report report(
      Detail::Summary,
      {.result = [&fails](const std::vector<Figure>&,
                          const std::optional<std::uint64_t> count) {
        fails = count;
      }});
  const auto routing = drawMini(planned, report);

  ASSERT_EQ(routing.wires.size(), planned.assignment.connections.size());
  EXPECT_TRUE(routing.wires.front()->path.empty());
  EXPECT_EQ(undrawnOf(routing), 1U);
  ASSERT_TRUE(fails.has_value());
  EXPECT_EQ(*fails, 0U);
}

TEST(DetailRouter, GivesTheSameAnswerTwice) {
  const auto planned = planMini();
  const auto first = drawMini(planned);
  const auto second = drawMini(planned);

  EXPECT_EQ(first, second);
}

/// What a report receives: the entries and their levels, the results, and
/// how often the progress moved.
struct Spoken {
  std::vector<std::pair<Detail, std::string>> entries;
  std::vector<std::vector<Figure>> results;
  std::vector<std::optional<std::uint64_t>> fails;
  std::uint64_t progressed = 0;

  [[nodiscard]] Report report() {
    return {
        Detail::Items,
        {.entry =
             [this](const Detail level, const std::string_view label,
                    const std::vector<Figure>& /*figures*/) {
               entries.emplace_back(level, std::string(label));
             },
         .result =
             [this](const std::vector<Figure>& figures,
                    const std::optional<std::uint64_t> failed) {
               results.push_back(figures);
               fails.push_back(failed);
             },
         .progress = [this](const std::string_view /*task*/,
                            const std::string_view /*detail*/,
                            const std::uint64_t /*done*/,
                            const std::uint64_t /*total*/) { ++progressed; }}};
  }
};

/// Builds one open square partition for the given wire endpoints.
Planned
planSquare(const ConfigT& config,
           const std::initializer_list<std::pair<DCoord, DCoord>> endpoints) {
  using flatbuffers::artifacts::CorridorT;
  using flatbuffers::artifacts::GridExtentT;
  using flatbuffers::artifacts::PartitionT;
  using flatbuffers::design::ConnectionT;
  using flatbuffers::design::PortT;
  using flatbuffers::design::UnassignedRole;
  using flatbuffers::geometry::Point;
  using flatbuffers::geometry::PolygonT;

  Planned planned;
  auto launcher = std::make_unique<PortT>();
  launcher->center = Point(0.0, 0.0);
  launcher->role = UnassignedRole::Launcher;
  planned.chip.ports.push_back(std::move(launcher));
  auto opposite = std::make_unique<PortT>();
  opposite->center = Point(1200.0, 1200.0);
  opposite->role = UnassignedRole::Launcher;
  planned.chip.ports.push_back(std::move(opposite));

  const auto scene = buildScene(planned.chip, config);
  const auto& grid = scene.detail;
  auto extent = std::make_unique<GridExtentT>();
  extent->width = grid.width;
  extent->height = grid.height;
  extent->origin = grid.origin;
  extent->cell_width = grid.cellWidth;
  extent->cell_height = grid.cellHeight;
  planned.capacity.detail_grid = std::move(extent);
  planned.capacity.blocked_words = scene.blocked.words();
  auto partition = std::make_unique<PartitionT>();
  partition->label = 1;
  auto outline = std::make_unique<PolygonT>();
  outline->vertices = {grid::cornerToLayout(grid, 0, 0),
                       grid::cornerToLayout(grid, grid.width, 0),
                       grid::cornerToLayout(grid, grid.width, grid.height),
                       grid::cornerToLayout(grid, 0, grid.height)};
  partition->outlines.push_back(std::move(outline));
  planned.capacity.partitions.push_back(std::move(partition));

  for (const auto& [from, to] : endpoints) {
    planned.assignment.connections.push_back(std::make_unique<ConnectionT>());
    auto corridor = std::make_unique<CorridorT>();
    corridor->partitions = {1};
    corridor->source =
        std::make_unique<Point>(grid.toLayout(from.x(), from.y()));
    corridor->target = std::make_unique<Point>(grid.toLayout(to.x(), to.y()));
    planned.corridor.corridors.push_back(std::move(corridor));
  }
  return planned;
}

TEST(DetailRouter, DrawsAStraightWireInEachGridDirection) {
  auto config = test::miniConfig();
  config.grid->capacity_cells_x = 13;
  config.grid->detail_factor = 1;
  const DCoord start(6, 6);
  for (const int dx : {-1, 0, 1}) {
    for (const int dy : {-1, 0, 1}) {
      if (dx == 0 && dy == 0) {
        continue;
      }
      const DCoord middle(static_cast<std::uint32_t>(6 + dx),
                          static_cast<std::uint32_t>(6 + dy));
      const DCoord goal(static_cast<std::uint32_t>(6 + 2 * dx),
                        static_cast<std::uint32_t>(6 + 2 * dy));
      const auto planned = planSquare(config, {{start, goal}});
      const auto routing = drawMini(planned, {}, config);
      ASSERT_EQ(routing.wires.size(), 1U);
      EXPECT_EQ(routing.wires.front()->path,
                (std::vector<DCoord>{start, middle, goal}));
    }
  }
}

TEST(DetailRouter, ReportsAConnectionThatWouldHaveToCrossAnotherWire) {
  auto config = test::miniConfig();
  config.grid->capacity_cells_x = 13;
  config.grid->detail_factor = 1;
  const auto planned = planSquare(
      config, {{DCoord(0, 0), DCoord(12, 12)}, {DCoord(0, 12), DCoord(12, 0)}});
  Spoken spoken;

  const auto routing = drawMini(planned, spoken.report(), config);

  EXPECT_EQ(undrawnOf(routing), 1U);
  ASSERT_EQ(spoken.fails.size(), 1U);
  EXPECT_EQ(spoken.fails.front(), 1U);
  expectFreeConnectedPaths(routing, buildScene(planned.chip, config));
  expectNoWireMeetsAnother(routing, routing.grid->width);
}

TEST(DetailRouter, CountsClearanceOnlyBetweenDrawnWires) {
  auto config = test::miniConfig();
  config.grid->capacity_cells_x = 13;
  config.grid->detail_factor = 1;
  const auto planned = planSquare(
      config, {{DCoord(0, 0), DCoord(12, 12)}, {DCoord(0, 1), DCoord(12, 0)}});
  Spoken spoken;
  const auto routing = drawMini(planned, spoken.report(), config);
  EXPECT_EQ(undrawnOf(routing), 1U);
  ASSERT_EQ(spoken.fails.size(), 1U);
  EXPECT_EQ(spoken.fails.front(), 1U);
}

TEST(DetailRouter, CountsTheWiresWithConflictingFixedEndpoints) {
  auto config = test::miniConfig();
  config.grid->capacity_cells_x = 13;
  config.grid->detail_factor = 1;
  config.stages->detail =
      std::make_unique<flatbuffers::config::DetailParamsT>();
  config.stages->detail->rounds = 2;
  const auto planned = planSquare(config, {{DCoord(0, 5), DCoord(12, 5)},
                                           {DCoord(0, 6), DCoord(12, 6)},
                                           {DCoord(0, 10), DCoord(12, 10)}});
  Spoken spoken;

  const auto routing = drawMini(planned, spoken.report(), config);

  EXPECT_EQ(undrawnOf(routing), 0U);
  ASSERT_EQ(spoken.fails.size(), 1U);
  EXPECT_EQ(spoken.fails.front(), 2U);
  expectNoWireMeetsAnother(routing, routing.grid->width);
}

TEST(DetailRouter, CountsTwoConnectionsWithTheSameTerminalAsOneUnrouted) {
  auto config = test::miniConfig();
  config.grid->capacity_cells_x = 13;
  config.grid->detail_factor = 1;
  config.rules->min_wire_spacing = 0;
  const auto planned = planSquare(
      config, {{DCoord(0, 1), DCoord(12, 1)}, {DCoord(0, 9), DCoord(12, 1)}});
  Spoken spoken;
  const auto routing = drawMini(planned, spoken.report(), config);
  ASSERT_EQ(routing.wires.size(), 2U);
  ASSERT_FALSE(routing.wires.front()->path.empty());
  EXPECT_EQ(routing.wires.front()->path.back(), DCoord(12, 1));
  EXPECT_TRUE(routing.wires.back()->path.empty());
  ASSERT_EQ(spoken.fails.size(), 1U);
  EXPECT_EQ(spoken.fails.front(), 1U);
}

TEST(DetailRouter, AcceptsAnObstacleReachLargerThanTheGrid) {
  auto config = test::miniConfig();
  const auto planned = planMini(config);
  const auto detail = buildScene(planned.chip, config).detail;
  config.stages->detail =
      std::make_unique<flatbuffers::config::DetailParamsT>();
  config.stages->detail->obstacle_penalty_reach =
      std::numeric_limits<std::uint32_t>::max() *
      std::min(detail.cellWidth, detail.cellHeight);
  config.stages->detail->corridor_spacings =
      std::numeric_limits<std::uint32_t>::max();
  const auto routing = drawMini(planned, {}, config);
  expectFreeConnectedPaths(routing, buildScene(planned.chip, config));
  EXPECT_EQ(undrawnOf(routing), 0U);
}

TEST(DetailRouter, KeepsDiagonalEndpointsOutsideTheClearanceDisc) {
  auto config = test::miniConfig();
  config.grid->capacity_cells_x = 13;
  config.grid->detail_factor = 1;
  config.rules->min_wire_spacing = 250.0;
  const auto planned = planSquare(
      config, {{DCoord(0, 0), DCoord(0, 1)}, {DCoord(2, 3), DCoord(2, 4)}});
  Spoken spoken;

  const auto routing = drawMini(planned, spoken.report(), config);

  EXPECT_EQ(undrawnOf(routing), 0U);
  ASSERT_EQ(spoken.fails.size(), 1U);
  EXPECT_EQ(spoken.fails.front(), 0U);
  const auto grid = buildScene(planned.chip, config).detail;
  EXPECT_EQ(clearanceOf(config, grid), 2U);
  expectTheClearance(routing, grid, 2U);
}

TEST(DetailRouter, CountsConflictsWhenTheClearanceSpansTheGrid) {
  auto config = test::miniConfig();
  config.grid->capacity_cells_x = 13;
  config.grid->detail_factor = 1;
  config.rules->min_wire_spacing = 2500.0;
  const auto planned = planSquare(
      config, {{DCoord(0, 0), DCoord(0, 1)}, {DCoord(12, 11), DCoord(12, 12)}});
  Spoken spoken;

  const auto routing = drawMini(planned, spoken.report(), config);

  EXPECT_EQ(undrawnOf(routing), 0U);
  ASSERT_EQ(spoken.fails.size(), 1U);
  EXPECT_EQ(spoken.fails.front(), 2U);
}

TEST(DetailRouter, AcceptsALimitLargerThanTheNumberOfNeighbors) {
  auto config = test::miniConfig();
  config.grid->capacity_cells_x = 13;
  config.grid->detail_factor = 1;
  config.stages->detail =
      std::make_unique<flatbuffers::config::DetailParamsT>();
  config.stages->detail->rounds = 2;
  const auto planned = planSquare(config, {{DCoord(0, 0), DCoord(12, 12)},
                                           {DCoord(0, 12), DCoord(12, 0)},
                                           {DCoord(3, 0), DCoord(4, 0)},
                                           {DCoord(8, 12), DCoord(9, 12)}});
  config.stages->detail->max_relaxation = 3;
  Spoken bounded;
  const auto reference = drawMini(planned, bounded.report(), config);
  EXPECT_EQ(undrawnOf(reference), 1U);
  ASSERT_EQ(bounded.fails.size(), 1U);
  EXPECT_EQ(bounded.fails.front(), 1U);

  for (const auto limit : {2U, std::numeric_limits<std::uint32_t>::max()}) {
    config.stages->detail->max_relaxation = limit;
    Spoken spoken;
    const auto routing = drawMini(planned, spoken.report(), config);
    EXPECT_EQ(routing, reference);
    EXPECT_EQ(spoken.fails, bounded.fails);
  }
}

TEST(DetailRouter, CountsACorridorWhoseFeedIsOutsideTheGrid) {
  auto planned = planMini();
  ASSERT_FALSE(planned.corridor.corridors.empty());
  auto& corridor = *planned.corridor.corridors.front();
  ASSERT_FALSE(corridor.partitions.empty());
  const auto detail = buildScene(planned.chip, test::miniConfig()).detail;
  corridor.source = std::make_unique<flatbuffers::geometry::Point>(
      grid::cornerToLayout(detail, -1.0, 0.5));
  Spoken spoken;

  const auto routing = drawMini(planned, spoken.report());

  EXPECT_TRUE(routing.wires.front()->path.empty());
  EXPECT_EQ(undrawnOf(routing), 1U);
  ASSERT_EQ(spoken.fails.size(), 1U);
  EXPECT_EQ(spoken.fails.front(), 1U);
}

TEST(DetailRouter, CountsAnInnerConnectionWhoseEndpointsAreOwned) {
  auto planned = planMini();
  ASSERT_GE(planned.assignment.connections.size(), 2U);
  auto inner = std::make_unique<flatbuffers::design::ConnectionT>();
  inner->source = std::make_unique<flatbuffers::design::PortRef>(
      planned.assignment.connections[0]->target);
  inner->target = planned.assignment.connections[1]->target;
  planned.global.connections.push_back(std::move(inner));
  Spoken spoken;

  const auto routing = drawMini(planned, spoken.report());

  EXPECT_TRUE(routing.inner.back()->path.empty());
  EXPECT_EQ(undrawnOf(routing), 1U);
  ASSERT_EQ(spoken.fails.size(), 1U);
  EXPECT_EQ(spoken.fails.front(), 1U);
}

TEST(DetailRouter, ReportsItsRoundsAndItsResultOnce) {
  const auto planned = planMini();
  Spoken spoken;
  const auto routing = drawMini(planned, spoken.report());

  ASSERT_FALSE(spoken.entries.empty());
  EXPECT_EQ(spoken.entries.front(),
            (std::pair{Detail::Steps, std::string("round 1 forward")}));
  EXPECT_GT(spoken.progressed, 0U);
  ASSERT_EQ(spoken.results.size(), 1U);
  ASSERT_TRUE(spoken.fails.front().has_value());
  EXPECT_EQ(*spoken.fails.front(), 0U);
  const auto drawn =
      std::format("drawn {}/{}", routing.wires.size() + routing.inner.size(),
                  routing.wires.size() + routing.inner.size());
  EXPECT_TRUE(std::ranges::any_of(
      spoken.results.front(),
      [&drawn](const Figure& figure) { return figure.text == drawn; }));
}

TEST(DetailRouter, ReadsThePlacesOfTheCorridorInTheCornerFrame) {
  const auto detail = buildScene(test::miniChip(), test::miniConfig()).detail;
  const auto cells = [&detail](const double x, const double y) {
    return cellsOfPlace(detail, grid::cornerToLayout(detail, x, y));
  };
  // A feed at the center of a cell names that cell twice.
  EXPECT_EQ(cells(123.5, 12.5), (std::array{DCoord(123, 12), DCoord(123, 12)}));
  // A crossing on a vertical edge names the cell on its left and the cell on
  // its right; on a horizontal edge, the cell above and the cell below.
  EXPECT_EQ(cells(151.0, 43.5), (std::array{DCoord(150, 43), DCoord(151, 43)}));
  EXPECT_EQ(cells(150.5, 44.0), (std::array{DCoord(150, 43), DCoord(150, 44)}));
  // On the first column no cell lies to the left: the one cell twice.
  EXPECT_EQ(cells(0.0, 7.5), (std::array{DCoord(0, 7), DCoord(0, 7)}));
  // A place beyond the grid names no cell.
  EXPECT_FALSE(cells(-3.0, 5.5).has_value());
  EXPECT_FALSE(cells(0.5, 300.5).has_value());
  EXPECT_FALSE(cells(1e30, 0.5).has_value());
  EXPECT_FALSE(
      cells(std::numeric_limits<double>::quiet_NaN(), 0.5).has_value());
  EXPECT_FALSE(cells(0.5, std::numeric_limits<double>::infinity()).has_value());
}

TEST(DetailRouter, RefusesAPlanMadeOnAnotherDetailGrid) {
  const auto planned = planMini();
  auto config = test::miniConfig();
  config.grid->capacity_cells_x += 1;

  EXPECT_NE(refusalOf(planned, config).find("detail grid"), std::string::npos);
}

TEST(DetailRouter, RefusesABlockedMaskThatDoesNotMatchTheChip) {
  auto planned = planMini();
  ASSERT_FALSE(planned.capacity.blocked_words.empty());
  planned.capacity.blocked_words.front() ^= 1U;

  EXPECT_NE(refusalOf(planned).find("blocked cells"), std::string::npos);
}

TEST(DetailRouter, RefusesACorridorRoutingOfAnotherAssignment) {
  auto planned = planMini();
  ASSERT_FALSE(planned.corridor.corridors.empty());
  planned.corridor.corridors.pop_back();

  EXPECT_NE(refusalOf(planned).find("corridor routing holds"),
            std::string::npos);
}

} // namespace
} // namespace mqt::scpd::pipeline
