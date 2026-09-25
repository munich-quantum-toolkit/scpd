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
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <set>
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
using flatbuffers::design::AssignedRole;
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

/// One chip planned and routed, shared by every test on it.
///
/// The Final stage takes minutes on the largest chip, and the stage is
/// deterministic, so every test of a chip reads the one run rather than
/// making its own.
struct Routed {
  Benchmark benchmark;
  Planned planned;
  FinalRoutingT routing;
};

const Routed& routedOf(const std::string& chip) {
  static std::map<std::string, std::unique_ptr<Routed>> cache;
  auto& slot = cache[chip];
  if (slot == nullptr) {
    slot = std::make_unique<Routed>();
    slot->benchmark = benchmarkOf(chip);
    slot->planned = plan(slot->benchmark);
    slot->routing = routeFinal(slot->benchmark, slot->planned);
  }
  return *slot;
}

grid::GridMetrics gridOf(const FinalRoutingT& routing) {
  return {.width = routing.grid->width,
          .height = routing.grid->height,
          .origin = routing.grid->origin,
          .cellWidth = routing.grid->cell_width,
          .cellHeight = routing.grid->cell_height};
}

/// What the design-rule check sees of this stage, as `mqt-scpd drc` sees it.
drc::CellView viewOf(const Benchmark& benchmark, const Planned& planned,
                     const FinalRoutingT& routing) {
  return drc::viewOfFinal(benchmark.chip, planned.global, planned.assignment,
                          routing, *benchmark.config.rules);
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

/// How far a resonator's length lies from `target_resonator_length`, at
/// worst, in layout units: the length the artifact carries plus the run from
/// the last cell of the way to the port itself, which the wire covers without
/// cells of its own. The stage cuts every resonator back to its coupler and
/// makes what is left the target length, within the rule's tolerance.
double worstResonatorDeviation(const Benchmark& benchmark,
                               const Planned& planned,
                               const FinalRoutingT& routing) {
  const auto grid = gridOf(routing);
  const double target = benchmark.config.rules->target_resonator_length;
  double worst = 0.0;
  for (std::size_t index = 0; index < routing.wires.size(); ++index) {
    const auto& wire = *routing.wires[index];
    const auto& connection = *planned.assignment.connections[index];
    if (wire.path.empty() ||
        connection.target_role != AssignedRole::ResonatorTarget) {
      continue;
    }
    const auto& port = *benchmark.chip.ports[connection.target.index()];
    const auto& last = wire.path.back();
    const auto end = grid.toLayout(last.x(), last.y());
    const double gap =
        std::hypot(end.x() - port.center.x(), end.y() - port.center.y());
    worst = std::max(worst, std::abs((wire.length + gap) - target));
  }
  return worst;
}

/// Whether the stage's own promise holds on a routing: every connection and
/// every feedline edge drawn, the design-rule check without an active
/// finding, and every resonator the target length within tolerance.
bool holdsEverywhere(const Benchmark& benchmark, const Planned& planned,
                     const FinalRoutingT& routing) {
  const auto view =
      drc::viewOfFinal(benchmark.chip, planned.global, planned.assignment,
                       routing, *benchmark.config.rules);
  const auto report = drc::checkCells(view, *benchmark.config.rules);
  const bool everyWireDrawn =
      routing.unresolved.empty() &&
      std::ranges::none_of(
          routing.inner, [](const auto& wire) { return wire->path.empty(); }) &&
      std::ranges::none_of(routing.feedlines,
                           [](const auto& wire) { return wire->path.empty(); });
  return everyWireDrawn && report.findings.empty() &&
         worstResonatorDeviation(benchmark, planned, routing) <=
             benchmark.config.rules->resonator_length_tolerance;
}

class Final : public testing::TestWithParam<std::string> {};
class SpacedFinal : public testing::TestWithParam<std::string> {};

/// Every connection of the plan is drawn, the inner circuit included.
///
/// A wire the stage could not draw carries no cells, so this counts the
/// failures rather than trusting a list beside the paths.
TEST_P(Final, EveryConnectionIsDrawn) {
  const auto& [benchmark, planned, routing] = routedOf(GetParam());

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

/// A wire begins where the assignment feeds it — a resonator at its coupler
/// — and ends at the cell of its target port, and it does not meet itself
/// on the way.
TEST_P(Final, WiresRunFromFeedToTargetWithoutMeetingThemselves) {
  const auto& [benchmark, planned, routing] = routedOf(GetParam());
  const auto grid = gridOf(routing);
  const auto view = viewOf(benchmark, planned, routing);
  const auto report = drc::checkCells(view, *benchmark.config.rules);

  EXPECT_EQ(countOf(report, DrcRule::WireLoop), 0U);
  reportFirst(report, DrcRule::WireLoop);

  // A resonator begins at its coupler, whose port is at the anchor; every
  // other wire begins where the assignment feeds it.
  std::vector<const flatbuffers::design::CpwCouplerT*> couplerOf(routing.wires.size(), nullptr);
  for (const auto& coupler : routing.couplers) {
    if (coupler->connection.index() < couplerOf.size()) {
      couplerOf[coupler->connection.index()] = coupler.get();
    }
  }
  for (std::size_t index = 0; index < routing.wires.size(); ++index) {
    const auto& path = routing.wires[index]->path;
    if (path.empty()) {
      continue;
    }
    const auto start = couplerOf[index] != nullptr
                           ? grid.clampToCell(couplerOf[index]->port->center)
                           : grid.clampToCell(planned.assignment.feeds[index]);
    EXPECT_EQ(path.front().x(), start.x()) << "wire " << index;
    EXPECT_EQ(path.front().y(), start.y()) << "wire " << index;
  }
}

/// Every resonator is the target length within the rule's tolerance, the
/// run to the port counted, and every drawn wire carries its length. The
/// length is the sampled curve of the way and not the cells it sweeps, which
/// is what the stage measured when it lengthened the way.
TEST_P(Final, ResonatorsReachTheTargetLength) {
  const auto& [benchmark, planned, routing] = routedOf(GetParam());
  ASSERT_GT(benchmark.config.rules->target_resonator_length, 0.0);

  std::size_t resonators = 0;
  for (std::size_t index = 0; index < routing.wires.size(); ++index) {
    const auto& wire = *routing.wires[index];
    if (wire.path.empty()) {
      continue;
    }
    EXPECT_GT(wire.length, 0.0) << "wire " << index << " carries no length";
    resonators += planned.assignment.connections[index]->target_role ==
                          AssignedRole::ResonatorTarget
                      ? 1
                      : 0;
  }
  EXPECT_GT(resonators, 0U);
  EXPECT_LE(worstResonatorDeviation(benchmark, planned, routing),
            benchmark.config.rules->resonator_length_tolerance);
}

/// Every resonator carries a coupler, and the coupler carries the port the
/// assignment left absent: the ports of the chip followed by the couplers'
/// ports in coupler order, which is what every feedline edge names.
TEST_P(Final, EveryResonatorGetsACoupler) {
  const auto& [benchmark, planned, routing] = routedOf(GetParam());

  std::size_t resonators = 0;
  for (const auto& connection : planned.assignment.connections) {
    resonators +=
        connection->target_role == AssignedRole::ResonatorTarget ? 1 : 0;
  }
  EXPECT_EQ(routing.couplers.size(), resonators);
  std::vector<bool> completed(planned.assignment.connections.size(), false);
  for (const auto& coupler : routing.couplers) {
    ASSERT_LT(coupler->connection.index(), completed.size());
    EXPECT_FALSE(completed[coupler->connection.index()]);
    completed[coupler->connection.index()] = true;
    EXPECT_EQ(planned.assignment.connections[coupler->connection.index()]
                  ->target_role,
              AssignedRole::ResonatorTarget);
    ASSERT_NE(coupler->port, nullptr);
    EXPECT_FALSE(coupler->port->label.empty());
    EXPECT_EQ(coupler->port->role,
              flatbuffers::design::UnassignedRole::Coupler);
    EXPECT_GT(coupler->length, 0.0);
    EXPECT_GT(coupler->height, 0.0);
  }
  const auto ports = benchmark.chip.ports.size() + routing.couplers.size();
  ASSERT_EQ(routing.feedline_edges.size(), routing.feedlines.size());
  EXPECT_EQ(routing.feedlines.size(), [&] {
    std::size_t edges = 0;
    for (const auto& chain : planned.assignment.chains) {
      edges += chain->nodes.size() - 1 + (chain->start != nullptr ? 1 : 0) +
               (chain->end != nullptr ? 1 : 0);
    }
    return edges;
  }());
  for (const auto& edge : routing.feedline_edges) {
    EXPECT_LT(edge->from.index(), ports);
    EXPECT_LT(edge->to.index(), ports);
  }
}

/// The edge into a coupler and the edge out of it meet at the coupler's
/// port and nowhere else: two consecutive edges of a chain share exactly one
/// cell, and edges of different chains share none.
TEST_P(Final, TheEdgesOfAChainMeetOnlyAtTheCoupler) {
  const auto& [benchmark, planned, routing] = routedOf(GetParam());
  ASSERT_EQ(routing.feedline_edges.size(), routing.feedlines.size());
  const auto cellsOf = [](const auto& wire) {
    std::set<std::pair<std::uint32_t, std::uint32_t>> cells;
    for (const auto& cell : wire->path) {
      cells.emplace(cell.x(), cell.y());
    }
    return cells;
  };
  for (std::size_t a = 0; a < routing.feedlines.size(); ++a) {
    const auto first = cellsOf(routing.feedlines[a]);
    for (std::size_t b = a + 1; b < routing.feedlines.size(); ++b) {
      const auto second = cellsOf(routing.feedlines[b]);
      std::size_t shared = 0;
      for (const auto& cell : first) {
        shared += second.contains(cell) ? 1 : 0;
      }
      const auto& one = *routing.feedline_edges[a];
      const auto& two = *routing.feedline_edges[b];
      const bool consecutive = one.chain == two.chain && one.to.index() == two.from.index();
      EXPECT_EQ(shared, consecutive ? 1U : 0U)
          << "feedline edges " << a << " and " << b << " share " << shared << " cells";
    }
  }
}

/// Every edge of every feedline chain is drawn, and no wire crosses a
/// feedline other than at a right angle: the design-rule check's second
/// rule, which runs the router's own test.
TEST_P(Final, FeedlinesAreDrawnAndCrossedAtRightAngles) {
  const auto& [benchmark, planned, routing] = routedOf(GetParam());
  const auto view = viewOf(benchmark, planned, routing);
  const auto report = drc::checkCells(view, *benchmark.config.rules);

  EXPECT_FALSE(routing.feedlines.empty());
  for (std::size_t index = 0; index < routing.feedlines.size(); ++index) {
    EXPECT_FALSE(routing.feedlines[index]->path.empty())
        << "feedline edge " << index << " was not drawn";
  }
  EXPECT_EQ(countOf(report, DrcRule::FeedlineOrthogonality), 0U);
  reportFirst(report, DrcRule::FeedlineOrthogonality);
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
  const auto& [benchmark, planned, routing] = routedOf(GetParam());
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
    // The couplers and the feedlines exist from the third phase on.
    const bool placed = index >= 2;
    EXPECT_EQ(routing.phases[index]->couplers.size(),
              placed ? routing.couplers.size() : 0U);
    EXPECT_EQ(routing.phases[index]->feedlines.size(),
              placed ? routing.feedlines.size() : 0U);
  }
}

/// Every pass starts every wire on the way the Detail stage drew and ends on
/// a line that says how many wires are unrouted, open, short, long, crossing
/// a feedline or meeting themselves, and the stage ends on one over every
/// wire. "Fails: 0"
/// there promises what the artifact and the design-rule check find: every
/// connection and every feedline edge drawn, no active finding, and every
/// resonator the target length within tolerance.
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

  // Every resonator's length is said, way plus the run to the port against
  // `meander_length`, and a line sums them up.
  std::size_t resonators = 0;
  std::size_t said = 0;
  for (std::size_t index = 0; index < planned.assignment.connections.size();
       ++index) {
    resonators += planned.assignment.connections[index]->target_role ==
                          AssignedRole::ResonatorTarget
                      ? 1
                      : 0;
  }
  for (const auto& line : lines) {
    said += line.find("resonator ") != std::string::npos &&
                    line.find("to the port") != std::string::npos
                ? 1
                : 0;
  }
  EXPECT_EQ(said, resonators);
  EXPECT_FALSE(lineWith("resonators:", "in all").empty());

  EXPECT_EQ(fails == 0, holdsEverywhere(benchmark, planned, routing)) << total;
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
