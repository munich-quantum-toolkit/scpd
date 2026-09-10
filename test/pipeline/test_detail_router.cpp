/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// The Detail stage on the benchmark inputs. What is checked is the copper it
// produces: every wire is a run of eight-connected free cells from the point
// it is fed at to the cell of its target port, it does not meet itself, and no
// two wires meet at all — neither on a cell nor between two of them.

#include "Benchmarks.hpp"
#include "mqt-scpd/flatbuffers/artifacts.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"
#include "mqt-scpd/pipeline/CapacityPlanner.hpp"
#include "mqt-scpd/pipeline/Registry.hpp"
#include "mqt-scpd/routing/SelfIntersection.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace mqt::scpd::pipeline {
namespace {

using flatbuffers::artifacts::AssignmentT;
using flatbuffers::artifacts::CapacityPlanT;
using flatbuffers::artifacts::CorridorRoutingT;
using flatbuffers::artifacts::DetailRoutingT;
using flatbuffers::artifacts::DetailWireT;
using flatbuffers::artifacts::GlobalRoutingT;

/// What the four stages before the detail router produce.
struct Planned {
  CapacityPlanT capacity;
  GlobalRoutingT global;
  AssignmentT assignment;
  CorridorRoutingT corridor;
};

Planned plan(const Benchmark& benchmark) {
  Planned planned;
  planned.capacity = capacityPlanners()
                         .make("watershed")
                         ->run(benchmark.chip, benchmark.config);
  planned.global =
      globalRouters()
          .make("hanan-milp")
          ->run(benchmark.chip, planned.capacity, benchmark.config);
  planned.assignment = assigners()
                           .make("ordered-milp")
                           ->run(benchmark.chip, planned.capacity,
                                 planned.global, benchmark.config);
  planned.corridor = corridorRouters()
                         .make("partition-astar")
                         ->run(benchmark.chip, planned.capacity,
                               planned.assignment, benchmark.config);
  return planned;
}

DetailRoutingT routeDetail(const Benchmark& benchmark, const Planned& planned) {
  return detailRouters()
      .make("pixel-astar")
      ->run(benchmark.chip, planned.capacity, planned.global,
            planned.assignment, planned.corridor, benchmark.config);
}

/// The cells of a wire as flat indices on the detail grid.
std::vector<std::size_t> cellsOf(const DetailWireT& wire,
                                 const std::uint32_t width) {
  std::vector<std::size_t> cells;
  cells.reserve(wire.path.size());
  for (const auto& cell : wire.path) {
    cells.push_back((static_cast<std::size_t>(cell.y()) * width) + cell.x());
  }
  return cells;
}

/// Every wire is a run of eight-connected cells, and none of them is blocked.
///
/// The obstacle mask is not an artifact: the stage rebuilds it from the chip
/// and the configuration, and so does this, which is what makes the check one
/// of the copper rather than of the stage's own bookkeeping.
void expectPathsAreConnectedAndFree(const DetailRoutingT& routing,
                                    const CapacityScene& scene) {
  const auto check = [&scene](const auto& wires, const std::string& what) {
    for (std::size_t index = 0; index < wires.size(); ++index) {
      const auto& path = wires[index]->path;
      for (std::size_t step = 0; step < path.size(); ++step) {
        const auto cell =
            (static_cast<std::size_t>(path[step].y()) * scene.detail.width) +
            path[step].x();
        ASSERT_LT(cell, scene.blocked.size())
            << what << " " << index << " leaves the grid";
        EXPECT_FALSE(scene.blocked.test(cell))
            << what << " " << index << " runs over an obstacle at ("
            << path[step].x() << ", " << path[step].y() << ")";
        if (step == 0) {
          continue;
        }
        const auto dx = std::max(path[step].x(), path[step - 1].x()) -
                        std::min(path[step].x(), path[step - 1].x());
        const auto dy = std::max(path[step].y(), path[step - 1].y()) -
                        std::min(path[step].y(), path[step - 1].y());
        EXPECT_TRUE((dx <= 1 && dy <= 1) && (dx + dy) > 0)
            << what << " " << index << " steps from (" << path[step - 1].x()
            << ", " << path[step - 1].y() << ") to (" << path[step].x() << ", "
            << path[step].y() << ")";
      }
    }
  };
  check(routing.wires, "wire");
  check(routing.inner, "inner wire");
}

/// No wire meets itself.
///
/// Two shapes count: the same cell twice, and two diagonal steps that cross
/// inside one two-by-two block. The detector is the one the design-rule check
/// uses, so a wire this stage accepts and one the check accepts cannot drift
/// apart.
void expectNoWireMeetsItself(const DetailRoutingT& routing) {
  routing::PathLoopScratch scratch;
  const auto check = [&scratch](const auto& wires, const std::string& what) {
    for (std::size_t index = 0; index < wires.size(); ++index) {
      const auto& path = wires[index]->path;
      scratch.xs.clear();
      scratch.ys.clear();
      std::set<std::pair<std::uint32_t, std::uint32_t>> seen;
      for (const auto& cell : path) {
        scratch.xs.push_back(static_cast<std::int32_t>(cell.x()));
        scratch.ys.push_back(static_cast<std::int32_t>(cell.y()));
        EXPECT_TRUE(seen.emplace(cell.x(), cell.y()).second)
            << what << " " << index << " comes back to (" << cell.x() << ", "
            << cell.y() << ")";
      }
      EXPECT_EQ(routing::scanCellsForSelfIntersection(scratch, nullptr, true),
                0U)
          << what << " " << index << " crosses itself";
    }
  };
  check(routing.wires, "wire");
  check(routing.inner, "inner wire");
}

/// No two wires share a cell, and no two cross between cells.
///
/// Sharing a cell is a short. So is the one crossing that shares none: two
/// diagonal steps through one two-by-two block, which is what the prototype's
/// occupancy of pixels cannot see, because it never records the lines between
/// them.
void expectNoTwoWiresMeet(const DetailRoutingT& routing,
                          const std::uint32_t width) {
  std::map<std::size_t, std::size_t> owner;
  std::map<std::pair<std::size_t, bool>, std::size_t> diagonals;
  const auto check = [&](const auto& wires, const std::size_t offset) {
    for (std::size_t index = 0; index < wires.size(); ++index) {
      const auto cells = cellsOf(*wires[index], width);
      for (std::size_t step = 0; step < cells.size(); ++step) {
        const auto [entry, added] = owner.emplace(cells[step], index + offset);
        EXPECT_TRUE(added || entry->second == index + offset)
            << "wires " << entry->second << " and " << (index + offset)
            << " share the cell (" << (cells[step] % width) << ", "
            << (cells[step] / width) << ")";
        if (step == 0) {
          continue;
        }
        const auto before = cells[step - 1];
        const auto x = static_cast<std::int64_t>(cells[step] % width);
        const auto y = static_cast<std::int64_t>(cells[step] / width);
        const auto px = static_cast<std::int64_t>(before % width);
        const auto py = static_cast<std::int64_t>(before / width);
        if (x == px || y == py) {
          continue;
        }
        // The block a diagonal step runs through, and which of its two
        // diagonals it is. Both diagonals of one block always meet.
        const auto block = (static_cast<std::size_t>(std::min(y, py)) * width) +
                           static_cast<std::size_t>(std::min(x, px));
        const auto rising = ((x - px) * (y - py)) > 0;
        for (const auto other : {true, false}) {
          const auto found = diagonals.find({block, other});
          if (found != diagonals.end() && found->second != index + offset &&
              (other != rising)) {
            ADD_FAILURE() << "wires " << found->second << " and "
                          << (index + offset) << " cross between the cells ("
                          << px << ", " << py << ") and (" << x << ", " << y
                          << ")";
          }
        }
        diagonals.emplace(std::pair{block, rising}, index + offset);
      }
    }
  };
  check(routing.wires, 0);
  check(routing.inner, routing.wires.size());
}

/// The design rule the stage has to hold, on the grid it drew on.
///
/// `min_wire_spacing` is a length and the router works in cells, so the rule
/// is converted, and the conversion is the stage's own: the rule spans
/// `ceil(spacing / cell)` cells and what is kept clear is one less. The check
/// is made against that number and not against the length, because a grid
/// whose cell is 40 layout units cannot express a distance of 185 — asking it
/// to would be a check of the grid rather than of the copper. On the eight
/// benchmarks it comes to 4 to 9 cells, which is 153 to 182 layout units.
[[nodiscard]] std::uint32_t rulePixels(const Benchmark& benchmark,
                                       const grid::GridMetrics& detail) {
  const auto spacing = benchmark.config.rules->min_wire_spacing;
  const auto spans = grid::cellsFor(spacing, detail);
  return spans == 0 ? 0 : spans - 1;
}

/// No two wires come within the design rule of each other.
///
/// This is the check the stage exists to pass, and the one the prototype does
/// not: it keeps the clearance between a wire and the two beside it in the
/// ring, and against every other wire the only rule is that no cell is shared.
/// Here every pair is checked, the inner circuit included, and a wire that
/// comes too close to another counts exactly as a wire that was never drawn.
///
/// The cells of every wire go into a map first, so the check costs one
/// neighbourhood per cell rather than one comparison per pair of cells: the
/// 69-qubit chip draws 111 thousand cells, and the pairs of those are twelve
/// billion.
void expectWiresKeepTheirDistance(const DetailRoutingT& routing,
                                  const grid::GridMetrics& detail,
                                  const std::uint32_t pixels) {
  ASSERT_GT(pixels, 0U) << "the design rule spans no cell of this grid";

  std::map<std::size_t, std::size_t> owner;
  const auto record = [&](const auto& wires, const std::size_t offset) {
    for (std::size_t index = 0; index < wires.size(); ++index) {
      for (const auto cell : cellsOf(*wires[index], detail.width)) {
        owner.emplace(cell, index + offset);
      }
    }
  };
  record(routing.wires, 0);
  record(routing.inner, routing.wires.size());

  // Every offset the rule reaches, once, so the sweep below is a lookup per
  // cell of it rather than a square root.
  std::vector<std::pair<std::int64_t, std::int64_t>> around;
  const auto reach = static_cast<std::int64_t>(pixels);
  for (auto dy = -reach; dy <= reach; ++dy) {
    for (auto dx = -reach; dx <= reach; ++dx) {
      if ((dx != 0 || dy != 0) && ((dx * dx) + (dy * dy)) <= (reach * reach)) {
        around.emplace_back(dx, dy);
      }
    }
  }

  const auto width = static_cast<std::int64_t>(detail.width);
  const auto height = static_cast<std::int64_t>(detail.height);
  std::size_t breaches = 0;
  for (const auto& [cell, wire] : owner) {
    const auto x = static_cast<std::int64_t>(cell) % width;
    const auto y = static_cast<std::int64_t>(cell) / width;
    for (const auto& [dx, dy] : around) {
      const auto nx = x + dx;
      const auto ny = y + dy;
      if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
        continue;
      }
      const auto found =
          owner.find(static_cast<std::size_t>((ny * width) + nx));
      if (found == owner.end() || found->second == wire) {
        continue;
      }
      ++breaches;
      // One report is enough to find the place; the count says how much of
      // the chip is affected.
      if (breaches == 1) {
        ADD_FAILURE() << "wires " << wire << " and " << found->second << " are "
                      << dx << ", " << dy << " cells apart at (" << x << ", "
                      << y << "), and the rule is " << pixels << " cells";
      }
    }
  }
  EXPECT_EQ(breaches, 0U) << breaches << " cells lie within " << pixels
                          << " cells of another wire";
}

/// A wire begins where the assignment feeds it and ends at the cell of its
/// target port.
///
/// This is the place the prototype is off by one at both ends: it recovers
/// which fragment belongs to which wire by matching endpoints and loses a cell
/// doing it. The identity survives this stage, so the ends are exact.
void expectWiresRunFromFeedToTarget(const DetailRoutingT& routing,
                                    const CorridorRoutingT& corridor,
                                    const grid::GridMetrics& detail) {
  for (std::size_t index = 0; index < routing.wires.size(); ++index) {
    const auto& path = routing.wires[index]->path;
    if (path.empty()) {
      continue;
    }
    const auto& route = *corridor.corridors[index];
    ASSERT_NE(route.source, nullptr);
    ASSERT_NE(route.target, nullptr);
    // The corridor stage writes a cell as the layout point of its centre in
    // the frame the partition geometry uses, where a whole number is a cell
    // corner. The check is made in layout units so that it states what the two
    // artifacts agree on rather than repeating either one's conversion.
    const auto centreOf = [&detail](const auto& cell) {
      return detail.toLayout(cell.x() + 0.5, cell.y() + 0.5);
    };
    const auto same = [](const auto& a, const auto& b) {
      return std::abs(a.x() - b.x()) < 1.0e-6 &&
             std::abs(a.y() - b.y()) < 1.0e-6;
    };
    EXPECT_TRUE(same(centreOf(path.front()), *route.source))
        << "wire " << index << " starts at (" << path.front().x() << ", "
        << path.front().y() << ") and is fed at (" << route.source->x() << ", "
        << route.source->y() << ")";
    EXPECT_TRUE(same(centreOf(path.back()), *route.target))
        << "wire " << index << " ends at (" << path.back().x() << ", "
        << path.back().y() << ") and its target port is at ("
        << route.target->x() << ", " << route.target->y() << ")";
  }
}

class BenchmarkDetail : public testing::TestWithParam<std::string> {};

TEST_P(BenchmarkDetail, DrawsCopperTheRulesAllow) {
  const auto benchmark = benchmarkOf(GetParam());
  const auto planned = plan(benchmark);
  const auto routing = routeDetail(benchmark, planned);
  const auto scene = buildScene(benchmark.chip, benchmark.config);

  // One wire per connection of the assignment, in its order, and one per
  // connection of the inner circuit, so a reader needs no key to line them up.
  ASSERT_EQ(routing.wires.size(), planned.assignment.connections.size());
  ASSERT_EQ(routing.inner.size(), planned.global.connections.size());
  ASSERT_NE(routing.grid, nullptr);
  EXPECT_EQ(routing.grid->width, scene.detail.width);
  EXPECT_EQ(routing.grid->height, scene.detail.height);

  expectPathsAreConnectedAndFree(routing, scene);
  expectNoWireMeetsItself(routing);
  expectNoTwoWiresMeet(routing, scene.detail.width);
  expectWiresRunFromFeedToTarget(routing, planned.corridor, scene.detail);
  expectWiresKeepTheirDistance(routing, scene.detail,
                               rulePixels(benchmark, scene.detail));
}

INSTANTIATE_TEST_SUITE_P(EveryChip, BenchmarkDetail,
                         testing::Values("4q", "9q", "17q", "21q", "33q", "45q",
                                         "57q", "69q"),
                         [](const testing::TestParamInfo<std::string>& chip) {
                           return chip.param;
                         });

/// Every connection of every benchmark chip is drawn.
///
/// This is the bar the stage is judged against, and it is the prototype's own:
/// its benchmark table reports no detail-routing failure on any of the eight.
class CompleteDetail : public testing::TestWithParam<std::string> {};

TEST_P(CompleteDetail, DrawEveryConnection) {
  const auto benchmark = benchmarkOf(GetParam());
  const auto planned = plan(benchmark);
  const auto routing = routeDetail(benchmark, planned);

  std::size_t undrawn = 0;
  for (const auto& wire : routing.wires) {
    undrawn += wire->path.empty() ? 1U : 0U;
  }
  EXPECT_EQ(undrawn, 0U) << undrawn << " of " << routing.wires.size()
                         << " connections were not drawn";

  std::size_t inner = 0;
  for (const auto& wire : routing.inner) {
    inner += wire->path.empty() ? 1U : 0U;
  }
  EXPECT_EQ(inner, 0U) << inner << " of " << routing.inner.size()
                       << " inner connections were not drawn";
}

INSTANTIATE_TEST_SUITE_P(EveryChip, CompleteDetail,
                         testing::Values("4q", "9q", "17q", "21q", "33q", "45q",
                                         "57q", "69q"),
                         [](const testing::TestParamInfo<std::string>& chip) {
                           return chip.param;
                         });

/// Every wire of every benchmark chip keeps the design rule to every other.
///
/// The bar of this stage, and a bar the prototype does not clear: a wire that
/// comes closer to another than the rule allows is a failure, exactly like a
/// wire that was not drawn.
class SpacedDetail : public testing::TestWithParam<std::string> {};

TEST_P(SpacedDetail, KeepTheWireSpacing) {
  const auto benchmark = benchmarkOf(GetParam());
  const auto planned = plan(benchmark);
  const auto routing = routeDetail(benchmark, planned);
  const auto scene = buildScene(benchmark.chip, benchmark.config);

  expectWiresKeepTheirDistance(routing, scene.detail,
                               rulePixels(benchmark, scene.detail));
}

INSTANTIATE_TEST_SUITE_P(EveryChip, SpacedDetail,
                         testing::Values("4q", "9q", "17q", "21q", "33q", "45q",
                                         "57q", "69q"),
                         [](const testing::TestParamInfo<std::string>& chip) {
                           return chip.param;
                         });

/// Two runs of the same input give the same answer, which is what makes a
/// resumed run equal to an uninterrupted one.
TEST(DetailRouter, IsDeterministic) {
  const auto benchmark = nineQubit();
  const auto planned = plan(benchmark);
  const auto first = routeDetail(benchmark, planned);
  const auto second = routeDetail(benchmark, planned);

  ASSERT_EQ(first.wires.size(), second.wires.size());
  for (std::size_t index = 0; index < first.wires.size(); ++index) {
    EXPECT_EQ(first.wires[index]->path, second.wires[index]->path)
        << "wire " << index << " came out differently the second time";
  }
  ASSERT_EQ(first.inner.size(), second.inner.size());
  for (std::size_t index = 0; index < first.inner.size(); ++index) {
    EXPECT_EQ(first.inner[index]->path, second.inner[index]->path)
        << "inner wire " << index << " came out differently the second time";
  }
}

} // namespace
} // namespace mqt::scpd::pipeline
