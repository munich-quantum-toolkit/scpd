/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// The Final stage on the benchmark inputs. What is checked is the copper it
// produces: every connection is drawn, no wire meets itself,
// every wire runs from the point the assignment feeds it to the cell of its
// target port, and no two wires come within the design rule of each other.
//
// The last three are the design-rule check itself, called here rather than
// written a second time, so that what the stage is judged by and what
// `mqt-scpd drc` reports cannot drift apart.

#include "Benchmarks.hpp"
#include "mqt-scpd/drc/Rules.hpp"
#include "mqt-scpd/flatbuffers/artifacts.hpp"
#include "mqt-scpd/flatbuffers/drc.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"
#include "mqt-scpd/pipeline/CapacityPlanner.hpp"
#include "mqt-scpd/pipeline/Registry.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace mqt::scpd::pipeline {
namespace {

using flatbuffers::artifacts::AssignmentT;
using flatbuffers::artifacts::CapacityPlanT;
using flatbuffers::artifacts::CorridorRoutingT;
using flatbuffers::artifacts::DetailRoutingT;
using flatbuffers::artifacts::FinalRoutingT;
using flatbuffers::artifacts::GlobalRoutingT;
using flatbuffers::drc::DrcRule;

/// What the five stages before the final router produce.
struct Planned {
  CapacityPlanT capacity;
  GlobalRoutingT global;
  AssignmentT assignment;
  CorridorRoutingT corridor;
  DetailRoutingT detail;
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
  planned.detail =
      detailRouters()
          .make("pixel-astar")
          ->run(benchmark.chip, planned.capacity, planned.global,
                planned.assignment, planned.corridor, benchmark.config);
  return planned;
}

FinalRoutingT routeFinal(const Benchmark& benchmark, const Planned& planned) {
  return finalRouters().make("dubins")->run(benchmark.chip, planned.capacity,
                                            planned.global, planned.assignment,
                                            planned.detail, benchmark.config);
}

grid::GridMetrics gridOf(const FinalRoutingT& routing) {
  return {.width = routing.grid->width,
          .height = routing.grid->height,
          .origin = routing.grid->origin,
          .cellWidth = routing.grid->cell_width,
          .cellHeight = routing.grid->cell_height};
}

/// The component of a port, or nothing where the port has none.
std::string_view componentOf(const Benchmark& benchmark,
                             const std::uint32_t port) {
  return port < benchmark.chip.ports.size()
             ? benchmark.chip.ports[port]->component
             : std::string_view{};
}

/// What the design-rule check sees of this stage: every wire as the cells it
/// runs over, with the components its two ends sit on.
drc::CellView viewOf(const Benchmark& benchmark, const Planned& planned,
                     const FinalRoutingT& routing) {
  drc::CellView view;
  view.grid = gridOf(routing);
  view.chip = &benchmark.chip;
  for (std::size_t index = 0; index < routing.wires.size(); ++index) {
    if (routing.wires[index]->path.empty()) {
      continue;
    }
    const auto& connection = *planned.assignment.connections[index];
    view.wires.push_back(
        {.connection = static_cast<std::uint32_t>(index),
         .cells = routing.wires[index]->path,
         .components = {std::string_view{},
                        componentOf(benchmark, connection.target.index())},
         .feedline = false});
  }
  for (std::size_t index = 0; index < routing.inner.size(); ++index) {
    if (routing.inner[index]->path.empty()) {
      continue;
    }
    const auto& connection = *planned.global.connections[index];
    view.wires.push_back(
        {.connection = static_cast<std::uint32_t>(routing.wires.size() + index),
         .cells = routing.inner[index]->path,
         .components = {connection.source == nullptr
                            ? std::string_view{}
                            : componentOf(benchmark,
                                          connection.source->index()),
                        componentOf(benchmark, connection.target.index())},
         .feedline = false});
  }
  return view;
}

std::size_t countOf(const flatbuffers::drc::DrcReportT& report,
                    const DrcRule rule) {
  std::size_t found = 0;
  for (const auto& finding : report.findings) {
    if (finding->rule == rule) {
      found += 1;
    }
  }
  return found;
}

void reportFirst(const flatbuffers::drc::DrcReportT& report,
                 const DrcRule rule) {
  for (const auto& finding : report.findings) {
    if (finding->rule == rule) {
      ADD_FAILURE() << finding->message;
      return;
    }
  }
}

class Final : public testing::TestWithParam<std::string> {};
class SpacedFinal : public testing::TestWithParam<std::string> {};

/// Every connection of the plan is drawn, the inner circuit included.
///
/// A wire the stage could not draw carries no cells, so this counts the
/// failures rather than trusting a list beside the paths.
TEST_P(Final, EveryConnectionIsDrawn) {
  const auto benchmark = benchmarkOf(GetParam());
  const auto planned = plan(benchmark);
  const auto routing = routeFinal(benchmark, planned);

  ASSERT_EQ(routing.wires.size(), planned.assignment.connections.size());
  ASSERT_EQ(routing.inner.size(), planned.global.connections.size());
  for (std::size_t index = 0; index < routing.wires.size(); ++index) {
    EXPECT_FALSE(routing.wires[index]->path.empty())
        << "connection " << index << " was not drawn";
  }
  for (std::size_t index = 0; index < routing.inner.size(); ++index) {
    EXPECT_FALSE(routing.inner[index]->path.empty())
        << "inner connection " << index << " was not drawn";
  }
  EXPECT_TRUE(routing.unresolved.empty())
      << routing.unresolved.size() << " connections were left unresolved";
}

/// A wire begins where the assignment feeds it and ends at the cell of its
/// target port, and it does not meet itself on the way.
TEST_P(Final, WiresRunFromFeedToTargetWithoutMeetingThemselves) {
  const auto benchmark = benchmarkOf(GetParam());
  const auto planned = plan(benchmark);
  const auto routing = routeFinal(benchmark, planned);
  const auto grid = gridOf(routing);
  const auto view = viewOf(benchmark, planned, routing);
  const auto report = drc::checkCells(view, *benchmark.config.rules);

  EXPECT_EQ(countOf(report, DrcRule::WireLoop), 0U);
  reportFirst(report, DrcRule::WireLoop);

  for (std::size_t index = 0; index < routing.wires.size(); ++index) {
    const auto& path = routing.wires[index]->path;
    if (path.empty()) {
      continue;
    }
    const auto feed = grid.clampToCell(planned.assignment.feeds[index]);
    EXPECT_EQ(path.front().x(), feed.x()) << "wire " << index;
    EXPECT_EQ(path.front().y(), feed.y()) << "wire " << index;
  }
}

INSTANTIATE_TEST_SUITE_P(EveryChip, Final,
                         testing::Values("4q", "9q", "17q", "21q", "33q", "45q",
                                         "57q", "69q"),
                         [](const testing::TestParamInfo<std::string>& chip) {
                           return chip.param;
                         });

/// No two wires come within the design rule of each other.
///
/// This is the check the stage exists to pass. The prototype keeps the
/// clearance between a wire and the two beside it in the ring and against
/// nothing else, which is why its own pictures show wires touching; here every
/// pair is checked, the inner circuit included.
///
/// One encounter is not a violation, and it is one a working design makes on
/// purpose: two wires that meet at a junction, their ends within the rule of
/// each other, as at two ports of one coupler. The router's fence makes the
/// same exemption, so the copper and the verdict answer the same question.
/// That two wires end on one component exempts nothing by itself.
TEST_P(SpacedFinal, KeepTheWireSpacing) {
  const auto benchmark = benchmarkOf(GetParam());
  const auto planned = plan(benchmark);
  const auto routing = routeFinal(benchmark, planned);
  const auto view = viewOf(benchmark, planned, routing);
  const auto report = drc::checkCells(view, *benchmark.config.rules);

  EXPECT_EQ(countOf(report, DrcRule::WireClearance), 0U);
  reportFirst(report, DrcRule::WireClearance);
}

INSTANTIATE_TEST_SUITE_P(EveryChip, SpacedFinal,
                         testing::Values("4q", "9q", "17q", "21q", "33q", "45q",
                                         "57q", "69q"),
                         [](const testing::TestParamInfo<std::string>& chip) {
                           return chip.param;
                         });

/// Two runs of the same input give the same answer, which is what makes a
/// resumed run equal to an uninterrupted one.
TEST(FinalRouter, IsDeterministic) {
  const auto benchmark = nineQubit();
  const auto planned = plan(benchmark);
  const auto first = routeFinal(benchmark, planned);
  const auto second = routeFinal(benchmark, planned);

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

/// Every phase leaves a snapshot, so a picture of one can be drawn from the
/// artifact rather than by running the stage again.
TEST(FinalRouter, EveryPhaseLeavesASnapshot) {
  const auto benchmark = fourQubit();
  const auto planned = plan(benchmark);
  const auto routing = routeFinal(benchmark, planned);

  ASSERT_EQ(routing.phases.size(), 5U);
  const std::vector<std::string> names{"inner", "outer", "couplers",
                                       "feedlines", "refined"};
  for (std::size_t index = 0; index < names.size(); ++index) {
    EXPECT_EQ(routing.phases[index]->name, names[index]);
    EXPECT_EQ(routing.phases[index]->wires.size(),
              planned.assignment.connections.size());
    EXPECT_EQ(routing.phases[index]->inner.size(),
              planned.global.connections.size());
  }
}

/// Every pass starts every wire on the way the Detail stage drew and ends on
/// a line that says how many wires are unrouted or open, and the stage ends
/// on one over every wire. "Fails: 0" there promises what the artifact and
/// the design-rule check find: every connection drawn and no two wires within
/// the rule.
TEST(FinalRouter, StartsOnTheDetailWaysAndReportsItsFails) {
  const auto benchmark = nineQubit();
  const auto planned = plan(benchmark);
  std::vector<std::string> lines;
  const auto routing = finalRouters().make("dubins")->run(
      benchmark.chip, planned.capacity, planned.global, planned.assignment,
      planned.detail, benchmark.config,
      [&lines](const std::string_view line) { lines.emplace_back(line); });

  const auto lineWith = [&lines](const std::string_view first,
                                 const std::string_view second) {
    const auto found =
        std::ranges::find_if(lines, [&](const std::string& line) {
          return line.find(first) != std::string::npos &&
                 line.find(second) != std::string::npos;
        });
    return found == lines.end() ? std::string{} : *found;
  };
  const std::string_view seeded = "start on the way the Detail stage drew";
  EXPECT_FALSE(lineWith("inner routing:", seeded).empty());
  EXPECT_FALSE(lineWith("outer routing:", seeded).empty());
  EXPECT_FALSE(lineWith("inner routing:", "Fails:").empty());
  EXPECT_FALSE(lineWith("outer routing:", "Fails:").empty());
  const auto total = lineWith("final routing:", "Fails:");
  ASSERT_FALSE(total.empty());
  const auto fails = std::stoul(total.substr(total.find("Fails:") + 6));

  const auto view = viewOf(benchmark, planned, routing);
  const auto report = drc::checkCells(view, *benchmark.config.rules);
  const bool everyWireDrawn =
      routing.unresolved.empty() &&
      std::ranges::none_of(routing.inner,
                           [](const auto& wire) { return wire->path.empty(); });
  const bool holds =
      everyWireDrawn && countOf(report, DrcRule::WireClearance) == 0;
  EXPECT_EQ(fails == 0, holds) << total;
}

/// With a debug sink the stage hands out a picture of its grid and then one
/// of every search, each an SVG document named after the pass, the round,
/// the wire and the kind of search.
TEST(FinalRouter, DrawsItsGridAndEverySearchWhenAsked) {
  const auto benchmark = fourQubit();
  const auto planned = plan(benchmark);
  std::vector<std::pair<std::string, std::string>> pictures;
  const auto routing = finalRouters().make("dubins")->run(
      benchmark.chip, planned.capacity, planned.global, planned.assignment,
      planned.detail, benchmark.config, {},
      [&pictures](const std::string_view name, const std::string_view content) {
        pictures.emplace_back(name, content);
        return std::string(name);
      });

  ASSERT_FALSE(pictures.empty());
  EXPECT_EQ(pictures.front().first, "final-grid.svg");
  std::size_t searches = 0;
  for (const auto& [name, content] : pictures) {
    EXPECT_TRUE(content.starts_with("<svg")) << name;
    EXPECT_TRUE(content.ends_with("</svg>\n")) << name;
    searches += name.find("-normal.svg") != std::string::npos ? 1 : 0;
  }
  EXPECT_GE(searches, routing.wires.size());
}

} // namespace
} // namespace mqt::scpd::pipeline
