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
#include "mqt-scpd/pipeline/CouplerSession.hpp"
#include "mqt-scpd/pipeline/Registry.hpp"

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <map>
#include <memory>
#include <regex>
#include <set>
#include <stdexcept>
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

  // The end state carries the verdict against every wire, and the count of
  // the wires with one is the count the last line said. An undrawn wire is
  // `Unrouted` and nothing else; a drawn one is not. The phase snapshots are
  // not judged and carry none.
  std::size_t held = 0;
  for (const auto* list : {&routing.wires, &routing.inner, &routing.feedlines}) {
    for (const auto& wire : *list) {
      const auto verdict = static_cast<std::uint8_t>(wire->verdict);
      held += verdict != 0 ? 1 : 0;
      const bool unrouted =
          (verdict & static_cast<std::uint8_t>(
                         mqt::scpd::flatbuffers::artifacts::FinalVerdict::Unrouted)) != 0;
      EXPECT_EQ(unrouted, wire->path.empty());
    }
  }
  EXPECT_EQ(held, fails) << total;
  for (const auto& phase : routing.phases) {
    for (const auto* list : {&phase->wires, &phase->inner, &phase->feedlines}) {
      for (const auto& wire : *list) {
        EXPECT_EQ(static_cast<std::uint8_t>(wire->verdict), 0) << phase->name;
      }
    }
  }
}

/// A pass flagged `onlyUnsettled` redraws the flagged wire and no other.
/// The probe behind `SCPD_PROBE_ONLY_UNSETTLED` flags one ring wire of the
/// four-qubit chip after the feedline pass, sweeps the whole ring once with
/// the flag, says how many wires were tried and whose ways moved, and puts
/// everything back; the stage's own result is unchanged by it.
TEST(FinalRouter, OnlyUnsettledRedrawsOneWire) {
  const auto benchmark = fourQubit();
  const auto planned = plan(benchmark);
  const auto plain = routeFinal(benchmark, planned);
  setenv("SCPD_PROBE_ONLY_UNSETTLED", "3", 1);
  std::vector<std::string> lines;
  const auto probed = finalRouters().make("dubins")->run(
      benchmark.chip, planned.capacity, planned.global, planned.assignment,
      planned.detail, benchmark.config,
      [&lines](const std::string_view line) { lines.emplace_back(line); });
  unsetenv("SCPD_PROBE_ONLY_UNSETTLED");

  const auto found = std::ranges::find_if(lines, [](const std::string& line) {
    return line.find("probe onlyUnsettled:") != std::string::npos;
  });
  ASSERT_NE(found, lines.end());
  EXPECT_NE(found->find("wire 3 flagged (1 member)"), std::string::npos)
      << *found;
  EXPECT_NE(found->find("1 wire tried"), std::string::npos) << *found;
  // The one way that may move is the flagged wire's own.
  const bool none = found->find("0 ways moved") != std::string::npos;
  const bool own = found->find("1 way moved: 3") != std::string::npos;
  EXPECT_TRUE(none || own) << *found;
  // And the probe leaves the stage on what it ends on without it.
  ASSERT_EQ(probed.wires.size(), plain.wires.size());
  for (std::size_t index = 0; index < plain.wires.size(); ++index) {
    EXPECT_EQ(probed.wires[index]->path, plain.wires[index]->path)
        << "wire " << index << " differs under the probe";
  }
}

/// Under `SCPD_CAPACITY_CHAIN` every settled chain is checked against the
/// capacity graph of the chains before it. A chain that closes no wire's
/// way is not searched again, so where none does, the stage ends where it
/// ends without the check.
/// `SCPD_CAPACITY_STEP` checks the capacity graph after every step of the
/// chain search and says, for every feedline edge a settled chain lays,
/// whether the graph still carries every wire. `SCPD_CAPACITY_RULE` is off
/// by default, so the check reports and changes nothing.
TEST(FinalRouter, TheStepCheckSaysForEveryFeedlineEdgeWhetherTheGraphCarries) {
  const auto benchmark = nineQubit();
  const auto planned = plan(benchmark);
  const auto plain = routeFinal(benchmark, planned);
  setenv("SCPD_CAPACITY_STEP", "1", 1);
  std::vector<std::string> lines;
  const auto checked = finalRouters().make("dubins")->run(
      benchmark.chip, planned.capacity, planned.global, planned.assignment,
      planned.detail, benchmark.config,
      [&lines](const std::string_view line) { lines.emplace_back(line); });
  unsetenv("SCPD_CAPACITY_STEP");

  // The edges of the settled chains, from their layers.
  std::size_t edges = 0;
  bool clocked = false;
  const std::regex settledLine(
      R"(\[Coupler Insertion\] chain \d+: (optimum|best).* over (\d+) layers)");
  for (const auto& line : lines) {
    std::smatch match;
    if (std::regex_search(line, match, settledLine)) {
      edges += std::stoul(match[2].str()) - 1;
    }
    clocked = clocked || line.find("in the time given") != std::string::npos ||
              line.find("out of time") != std::string::npos;
  }
  ASSERT_GT(edges, 0U);

  std::size_t verdicts = 0;
  std::size_t summaries = 0;
  const std::regex edgeLine(R"(\[Capacity Step\] chain \d+ f\d+: )");
  for (const auto& line : lines) {
    if (std::regex_search(line, edgeLine)) {
      ++verdicts;
      EXPECT_TRUE(line.find(": SAT, closes ") != std::string::npos ||
                  line.find(": UNSAT — ") != std::string::npos)
          << line;
    }
    if (line.find(" steps checked in ") != std::string::npos) {
      ++summaries;
      EXPECT_EQ(line.find(": 0 steps checked"), std::string::npos) << line;
    }
  }
  EXPECT_EQ(verdicts, edges);
  EXPECT_GT(summaries, 0U);

  // Report only: unless a chain ran into its clock, which the checks spend,
  // every wire is routed as without them.
  if (!clocked) {
    ASSERT_EQ(checked.wires.size(), plain.wires.size());
    for (std::size_t index = 0; index < plain.wires.size(); ++index) {
      EXPECT_EQ(checked.wires[index]->path, plain.wires[index]->path)
          << "wire " << index << " differs under the step check";
    }
  }
}

/// `SCPD_CAPACITY_RULE=1` refuses every step of the chain search whose
/// edge closes a wire: every feedline edge of a chain settled under it
/// closes none. A chain searched again without the rule says so first.
TEST(FinalRouter, TheCapacityRuleLetsNoSettledEdgeCloseAWire) {
  const auto benchmark = nineQubit();
  const auto planned = plan(benchmark);
  setenv("SCPD_CAPACITY_RULE", "1", 1);
  std::vector<std::string> lines;
  static_cast<void>(finalRouters().make("dubins")->run(
      benchmark.chip, planned.capacity, planned.global, planned.assignment,
      planned.detail, benchmark.config,
      [&lines](const std::string_view line) { lines.emplace_back(line); }));
  unsetenv("SCPD_CAPACITY_RULE");

  std::set<std::string> withoutTheRule;
  const std::regex againLine(
      R"(\[Coupler Insertion\] chain (\d+): .* searched again without)");
  const std::regex edgeLine(R"(\[Capacity Step\] chain (\d+) f\d+: )");
  std::size_t verdicts = 0;
  for (const auto& line : lines) {
    std::smatch match;
    if (std::regex_search(line, match, againLine)) {
      withoutTheRule.insert(match[1].str());
    }
  }
  for (const auto& line : lines) {
    std::smatch match;
    if (!std::regex_search(line, match, edgeLine)) {
      continue;
    }
    ++verdicts;
    if (!withoutTheRule.contains(match[1].str())) {
      EXPECT_NE(line.find(", closes no wire"), std::string::npos) << line;
    }
  }
  EXPECT_GT(verdicts, 0U);
}

TEST(FinalRouter, TheCapacityCheckLeavesAChainThatClosesNoWayAsItIs) {
  const auto benchmark = nineQubit();
  const auto planned = plan(benchmark);
  const auto plain = routeFinal(benchmark, planned);
  setenv("SCPD_CAPACITY_CHAIN", "1", 1);
  std::vector<std::string> lines;
  const auto checked = finalRouters().make("dubins")->run(
      benchmark.chip, planned.capacity, planned.global, planned.assignment,
      planned.detail, benchmark.config,
      [&lines](const std::string_view line) { lines.emplace_back(line); });
  unsetenv("SCPD_CAPACITY_CHAIN");

  std::vector<std::string> checks;
  bool searchedAgain = false;
  for (const auto& line : lines) {
    if (line.find("[Capacity Check] chain ") == std::string::npos) {
      continue;
    }
    if (line.find("searched again") != std::string::npos) {
      searchedAgain = true;
    } else {
      checks.push_back(line);
    }
  }
  const auto settled = std::ranges::count_if(lines, [](const auto& line) {
    return line.find("[Coupler Insertion] chain ") != std::string::npos &&
           (line.find(": optimum ") != std::string::npos ||
            line.find(": best ") != std::string::npos);
  });
  ASSERT_GT(settled, 0);
  EXPECT_EQ(std::ssize(checks), settled);
  for (const auto& line : checks) {
    EXPECT_NE(line.find(" wire"), std::string::npos) << line;
  }
  if (!searchedAgain) {
    for (const auto& line : checks) {
      EXPECT_NE(line.find(", 0 more with it"), std::string::npos) << line;
    }
    ASSERT_EQ(checked.wires.size(), plain.wires.size());
    for (std::size_t index = 0; index < plain.wires.size(); ++index) {
      EXPECT_EQ(checked.wires[index]->path, plain.wires[index]->path)
          << "wire " << index << " differs under the check";
    }
  }
}

/// A coupler session holds the stage after its coupler insertion. A coupler
/// moved to another option stands on it, its two feedline edges are drawn
/// again, and the capacity graph is analysed on the chip as it stands.
TEST(CouplerSession, MovesACouplerAndDrawsItsEdgesAgain) {
  const auto benchmark = fourQubit();
  const auto planned = plan(benchmark);
  CouplerSession session(benchmark.chip, planned.global, planned.assignment,
                         planned.detail, benchmark.config);

  const auto listed = nlohmann::json::parse(session.couplers()).at("couplers");
  const nlohmann::json* picked = nullptr;
  for (const auto& coupler : listed) {
    if (coupler.at("options").size() > 1) {
      picked = &coupler;
      break;
    }
  }
  ASSERT_NE(picked, nullptr);
  const auto index = picked->at("index").get<std::uint32_t>();
  const auto option = picked->at("chosen").get<std::uint32_t>() == 0 ? 1U : 0U;
  std::set<std::string> expected;
  for (const auto* const side : {"edgeIn", "edgeOut"}) {
    if (!picked->at(side).is_null()) {
      expected.insert(picked->at(side).get<std::string>());
    }
  }

  const auto moved = nlohmann::json::parse(session.setOption(index, option));
  EXPECT_EQ(moved.at("coupler").get<std::uint32_t>(), index);
  std::set<std::string> redrawn;
  for (const auto& edge : moved.at("edges")) {
    redrawn.insert(edge.at("edge").get<std::string>());
  }
  EXPECT_EQ(redrawn, expected);
  const auto after = nlohmann::json::parse(session.couplers());
  for (const auto& coupler : after.at("couplers")) {
    if (coupler.at("index").get<std::uint32_t>() == index) {
      EXPECT_EQ(coupler.at("chosen").get<std::uint32_t>(), option);
    }
  }
  const auto graph = nlohmann::json::parse(session.graph());
  for (const auto* const key : {"raster", "nodes", "edges", "wires"}) {
    EXPECT_TRUE(graph.contains(key)) << key;
  }
  EXPECT_THROW(
      static_cast<void>(session.setOption(
          index, static_cast<std::uint32_t>(picked->at("options").size()))),
      std::invalid_argument);
}

/// `SCPD_COUPLER_SHIFT` (20) moves an option that leaves the coupler box
/// into it, by at most that many cells along each axis. On 4q some options
/// are moved, and none further than the limit.
TEST(CouplerSession, MovesAnOptionIntoTheBoxByAtMostTheLimit) {
  const auto benchmark = fourQubit();
  const auto planned = plan(benchmark);
  CouplerSession session(benchmark.chip, planned.global, planned.assignment,
                         planned.detail, benchmark.config);
  std::vector<std::pair<long, long>> moved;
  const auto listed = nlohmann::json::parse(session.couplers());
  for (const auto& coupler : listed.at("couplers")) {
    for (const auto& option : coupler.at("options")) {
      const auto& move = option.at("moved");
      moved.emplace_back(move.at(0).get<long>(), move.at(1).get<long>());
    }
  }

  ASSERT_FALSE(moved.empty());
  EXPECT_TRUE(std::ranges::any_of(moved, [](const auto& move) {
    return move.first != 0 || move.second != 0;
  }));
  for (const auto& [x, y] : moved) {
    EXPECT_LE(std::abs(x), 20);
    EXPECT_LE(std::abs(y), 20);
  }
}

/// On 4q the ways of Q1 and Q4 run along the box edge near their target
/// length, and their couplers reach offset 0 only when moved into the box.
/// With the move every feedline edge is drawn and the chain makes four
/// quarter turns, an angle cost of 8.
TEST(FinalRouter, TheFourQubitChainMakesFourQuarterTurns) {
  const auto benchmark = fourQubit();
  const auto planned = plan(benchmark);
  long cost = -1;
  long undrawn = -1;
  const std::regex said(
      R"(==> coupler insertion: (\d+) feedline edges NOT drawn, feedline angle cost (\d+))");
  static_cast<void>(finalRouters().make("dubins")->run(
      benchmark.chip, planned.capacity, planned.global, planned.assignment,
      planned.detail, benchmark.config, [&](const std::string_view line) {
        std::match_results<std::string_view::const_iterator> match;
        if (std::regex_search(line.begin(), line.end(), match, said)) {
          undrawn = std::stol(match[1].str());
          cost = std::stol(match[2].str());
        }
      }));

  EXPECT_EQ(undrawn, 0);
  EXPECT_EQ(cost, 8);
}

/// No two edges of one chain cross. The stage says so after the coupler
/// insertion and after the feedline pass (`CHECK chain crossings`), and on
/// 4q both checks are green.
TEST(FinalRouter, NoEdgeCrossesAnEdgeOfItsOwnChain) {
  const auto benchmark = fourQubit();
  const auto planned = plan(benchmark);
  std::vector<std::string> checks;
  static_cast<void>(finalRouters().make("dubins")->run(
      benchmark.chip, planned.capacity, planned.global, planned.assignment,
      planned.detail, benchmark.config, [&](const std::string_view line) {
        if (line.find("CHECK chain crossings") != std::string_view::npos) {
          checks.emplace_back(line);
        }
      }));

  ASSERT_GE(checks.size(), 2U);
  for (const auto& line : checks) {
    EXPECT_NE(line.find("0 pairs of edges of one chain cross"),
              std::string::npos)
        << line;
  }
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

/// An edge between two couplers keeps inside the box the launcher stubs
/// leave open, the one every coupler stands in; the starting and ending
/// edges of a chain come from the border and may leave it.
TEST(FinalRouter, KeepsTheEdgesBetweenCouplersInsideTheCouplerBox) {
  const auto benchmark = benchmarkOf("9q");
  const auto planned = plan(benchmark);
  std::vector<std::string> lines;
  const auto routing = finalRouters().make("dubins")->run(
      benchmark.chip, planned.capacity, planned.global, planned.assignment,
      planned.detail, benchmark.config,
      [&lines](const std::string_view line) { lines.emplace_back(line); });

  const auto said = std::ranges::find_if(lines, [](const std::string& line) {
    return line.find("couplers sit inside x ") != std::string::npos;
  });
  ASSERT_NE(said, lines.end());
  long minX = 0;
  long maxX = 0;
  long minY = 0;
  long maxY = 0;
  ASSERT_EQ(std::sscanf(said->c_str() + said->find("couplers sit inside x "),
                        "couplers sit inside x %ld..%ld y %ld..%ld", &minX,
                        &maxX, &minY, &maxY),
            4);

  ASSERT_EQ(routing.feedlines.size(), routing.feedline_edges.size());
  std::size_t between = 0;
  for (std::size_t index = 0; index < routing.feedlines.size(); ++index) {
    if (routing.feedline_edges[index]->terminal) {
      continue;
    }
    ++between;
    for (const auto& cell : routing.feedlines[index]->path) {
      const auto x = static_cast<long>(cell.x());
      const auto y = static_cast<long>(cell.y());
      EXPECT_TRUE(x >= minX && x <= maxX && y >= minY && y <= maxY)
          << "edge " << index << " leaves the box at (" << x << "," << y << ")";
    }
  }
  EXPECT_GT(between, 0U);
}

/// `SCPD_EDGE_LAUNCHER_MARGIN` lengthens the run a feedline edge keeps
/// closed in front of every launcher but its own: with a margin of 30 cells
/// every edge the insertion draws keeps the clearance from that longer run,
/// where without it an edge of the four-qubit chip runs through it.
TEST(FinalRouter, AnEdgeKeepsTheLauncherMarginClear) {
  const auto benchmark = fourQubit();
  const auto planned = plan(benchmark);
  // The edges as the insertion leaves them and the launchers of the plain
  // wires, from the data of the capacity graph; and the run the stage says
  // it keeps closed.
  struct Seen {
    nlohmann::json graph;
    long run = -1;
  };
  const auto runWith = [&](const char* margin) {
    Seen seen;
    setenv("SCPD_BOTTLENECKS", "1", 1);
    setenv("SCPD_EDGE_LAUNCHER_MARGIN", margin, 1);
    static_cast<void>(finalRouters().make("dubins")->run(
        benchmark.chip, planned.capacity, planned.global, planned.assignment,
        planned.detail, benchmark.config,
        [&seen](const std::string_view line) {
          const auto at = line.find("a feedline edge keeps ");
          if (at != std::string_view::npos) {
            seen.run = std::stol(std::string(line.substr(at + 22)));
          }
        },
        [&seen](const std::string_view name, const std::string_view content) {
          if (name == "final-capacity-graph.json") {
            seen.graph = nlohmann::json::parse(content);
          }
          return std::string(name);
        }));
    unsetenv("SCPD_EDGE_LAUNCHER_MARGIN");
    unsetenv("SCPD_BOTTLENECKS");
    return seen;
  };
  // The least distance from an edge to the run of `run` cells off a launcher
  // that is not at one of the edge's ends.
  const auto closest = [](const nlohmann::json& graph, const long run) {
    double least = std::numeric_limits<double>::max();
    for (const auto& wire : graph.at("wires")) {
      const auto& source = wire.at("source");
      if (!source.at("label").get<std::string>().starts_with("Chip.port")) {
        continue;
      }
      const auto px = std::lround(source.at("port")[0].get<double>());
      const auto py = std::lround(source.at("port")[1].get<double>());
      const auto hx = source.at("heading")[0].get<long>();
      const auto hy = source.at("heading")[1].get<long>();
      for (const auto& stroke : graph.at("strokes")) {
        if (stroke.at("kind") != "feedline") {
          continue;
        }
        const auto& points = stroke.at("points");
        const auto near = [&](const nlohmann::json& point) {
          return std::hypot(static_cast<double>(point[0].get<long>() - px),
                            static_cast<double>(point[1].get<long>() - py)) <
                 30.0;
        };
        if (near(points.front()) || near(points.back())) {
          continue;
        }
        for (const auto& point : points) {
          for (long k = 0; k <= run; ++k) {
            least = std::min(
                least, std::hypot(static_cast<double>(point[0].get<long>() -
                                                      (px + (k * hx))),
                                  static_cast<double>(point[1].get<long>() -
                                                      (py + (k * hy)))));
          }
        }
      }
    }
    return least;
  };

  const auto plain = runWith("0");
  const auto kept = runWith("30");
  EXPECT_EQ(plain.run, -1) << "no margin, no line";
  ASSERT_GT(kept.run, 30);
  ASSERT_FALSE(kept.graph.is_null());
  ASSERT_FALSE(plain.graph.is_null());
  const auto clearance = kept.graph.at("clearance").get<double>();
  EXPECT_GE(closest(kept.graph, kept.run), clearance);
  EXPECT_LT(closest(plain.graph, kept.run), clearance);
}

/// `SCPD_COUPLER_BOX_MARGIN` keeps every coupler further in from the sides of
/// the coupler box: with a margin of 20 cells no pad cell lies closer to a
/// side than that, where without it a pad of the four-qubit chip does.
TEST(FinalRouter, ACouplerKeepsTheBoxMarginClear) {
  const auto benchmark = fourQubit();
  const auto planned = plan(benchmark);
  struct Seen {
    nlohmann::json graph;
    std::string box;
    bool said = false;
  };
  const auto runWith = [&](const char* margin) {
    Seen seen;
    setenv("SCPD_BOTTLENECKS", "1", 1);
    setenv("SCPD_COUPLER_BOX_MARGIN", margin, 1);
    static_cast<void>(finalRouters().make("dubins")->run(
        benchmark.chip, planned.capacity, planned.global, planned.assignment,
        planned.detail, benchmark.config,
        [&seen](const std::string_view line) {
          if (line.find("couplers sit inside x ") != std::string_view::npos) {
            seen.box = std::string(line.substr(line.find("couplers sit")));
          }
          seen.said = seen.said || line.find("SCPD_COUPLER_BOX_MARGIN") !=
                                       std::string_view::npos;
        },
        [&seen](const std::string_view name, const std::string_view content) {
          if (name == "final-capacity-graph.json") {
            seen.graph = nlohmann::json::parse(content);
          }
          return std::string(name);
        }));
    unsetenv("SCPD_COUPLER_BOX_MARGIN");
    unsetenv("SCPD_BOTTLENECKS");
    return seen;
  };
  // The least distance of a cell of a coupler pad to a side of the box.
  const auto closest = [](const Seen& seen) {
    long minX = 0;
    long maxX = 0;
    long minY = 0;
    long maxY = 0;
    EXPECT_EQ(std::sscanf(seen.box.c_str(),
                          "couplers sit inside x %ld..%ld y %ld..%ld", &minX,
                          &maxX, &minY, &maxY),
              4);
    const auto& kinds = seen.graph.at("wallKinds");
    const auto pad = static_cast<long>(std::ranges::find(kinds, "coupler pad") -
                                       kinds.begin() + 1);
    const auto width = seen.graph.at("width").get<long>();
    const auto& raster = seen.graph.at("raster");
    long least = std::numeric_limits<long>::max();
    long cell = 0;
    for (std::size_t at = 0; at + 1 < raster.size(); at += 2) {
      const auto code = raster[at].get<long>();
      const auto run = raster[at + 1].get<long>();
      if (code == pad) {
        for (auto k = cell; k < cell + run; ++k) {
          const auto x = k % width;
          const auto y = k / width;
          least = std::min({least, x - minX, maxX - x, y - minY, maxY - y});
        }
      }
      cell += run;
    }
    return least;
  };

  const auto plain = runWith("0");
  const auto kept = runWith("20");
  ASSERT_FALSE(plain.graph.is_null());
  ASSERT_FALSE(kept.graph.is_null());
  EXPECT_FALSE(plain.said);
  EXPECT_TRUE(kept.said);
  EXPECT_GE(closest(kept), 20);
  EXPECT_LT(closest(plain), 20);
}

/// With a debug sink the stage draws the bottlenecks of the chip the coupler
/// insertion leaves: once, after the picture of the coupler options, with a
/// line for each bottleneck whose tooltip names the obstacles at its two
/// ends.
TEST(FinalRouter, DrawsTheBottlenecksAfterTheCouplerInsertion) {
  const auto benchmark = fourQubit();
  const auto planned = plan(benchmark);
  std::vector<std::pair<std::string, std::string>> pictures;
  // The analysis is off unless asked for.
  setenv("SCPD_BOTTLENECKS", "1", 1);
  static_cast<void>(finalRouters().make("dubins")->run(
      benchmark.chip, planned.capacity, planned.global, planned.assignment,
      planned.detail, benchmark.config, {},
      [&pictures](const std::string_view name, const std::string_view content) {
        pictures.emplace_back(name, content);
        return std::string(name);
      }));
  unsetenv("SCPD_BOTTLENECKS");

  const auto named = [&pictures](const std::string_view name) {
    return std::ranges::find_if(pictures, [name](const auto& picture) {
      return picture.first == name;
    });
  };
  const auto options = named("final-coupler-options.svg");
  const auto bottlenecks = named("final-bottlenecks.svg");
  ASSERT_NE(bottlenecks, pictures.end());
  ASSERT_NE(options, pictures.end());
  EXPECT_LT(options - pictures.begin(), bottlenecks - pictures.begin());
  EXPECT_EQ(std::ranges::count_if(pictures,
                                  [](const auto& picture) {
                                    return picture.first ==
                                           "final-bottlenecks.svg";
                                  }),
            1);
  const auto& content = bottlenecks->second;
  EXPECT_NE(content.find("<line x1="), std::string::npos);
  EXPECT_NE(content.find(" · "), std::string::npos);
}

/// With a debug sink the stage draws the capacity graph of the chambers the
/// bottlenecks cut, once and after the bottlenecks, with a node per chamber
/// and the ports of the wires.
TEST(FinalRouter, DrawsTheCapacityGraphAfterTheBottlenecks) {
  const auto benchmark = fourQubit();
  const auto planned = plan(benchmark);
  std::vector<std::pair<std::string, std::string>> pictures;
  // The analysis is off unless asked for.
  setenv("SCPD_BOTTLENECKS", "1", 1);
  static_cast<void>(finalRouters().make("dubins")->run(
      benchmark.chip, planned.capacity, planned.global, planned.assignment,
      planned.detail, benchmark.config, {},
      [&pictures](const std::string_view name, const std::string_view content) {
        pictures.emplace_back(name, content);
        return std::string(name);
      }));
  unsetenv("SCPD_BOTTLENECKS");

  const auto named = [&pictures](const std::string_view name) {
    return std::ranges::find_if(pictures, [name](const auto& picture) {
      return picture.first == name;
    });
  };
  const auto bottlenecks = named("final-bottlenecks.svg");
  const auto graph = named("final-capacity-graph.svg");
  ASSERT_NE(graph, pictures.end());
  ASSERT_NE(bottlenecks, pictures.end());
  EXPECT_LT(bottlenecks - pictures.begin(), graph - pictures.begin());
  EXPECT_EQ(std::ranges::count_if(pictures,
                                  [](const auto& picture) {
                                    return picture.first ==
                                           "final-capacity-graph.svg";
                                  }),
            1);
  const auto& content = graph->second;
  EXPECT_NE(content.find("class=\"node\""), std::string::npos);
  EXPECT_NE(content.find("port of "), std::string::npos);

  // The same graph as data, for the page `plan --debug` builds from it.
  const auto written = named("final-capacity-graph.json");
  ASSERT_NE(written, pictures.end());
  const auto data = nlohmann::json::parse(written->second);
  const auto width = data.at("width").get<std::size_t>();
  const auto height = data.at("height").get<std::size_t>();
  // The raster covers the grid, run by run, and names only chambers there are.
  const auto& raster = data.at("raster");
  ASSERT_EQ(raster.size() % 2, 0U);
  std::size_t cells = 0;
  const auto chambers = data.at("nodes").size();
  for (std::size_t at = 0; at < raster.size(); at += 2) {
    const auto code = raster[at].get<std::size_t>();
    EXPECT_TRUE(code < 6 || code - 6 < chambers) << code;
    cells += raster[at + 1].get<std::size_t>();
  }
  EXPECT_EQ(cells, width * height);
  // Every edge carries the wires whose way takes it.
  const auto& edges = data.at("edges");
  std::vector<std::size_t> taken(edges.size(), 0);
  const auto& wires = data.at("wires");
  ASSERT_FALSE(wires.empty());
  for (const auto& wire : wires) {
    // A target port is named and lies in a chamber.
    EXPECT_FALSE(wire.at("target").at("label").get<std::string>().empty());
    EXPECT_FALSE(wire.at("target").at("chambers").empty());
    EXPECT_EQ(wire.at("target").at("heading").size(), 2U);
    if (!wire.at("way").is_null()) {
      for (const auto& index : wire.at("way")) {
        ++taken.at(index.get<std::size_t>());
      }
    }
  }
  for (std::size_t index = 0; index < edges.size(); ++index) {
    EXPECT_EQ(edges[index].at("load").get<std::size_t>(), taken[index]);
    EXPECT_EQ(edges[index].at("through").size(), taken[index]);
  }
  // A way crosses exactly the feedline edges its wire is prescribed, each
  // once, and no other.
  for (const auto& wire : wires) {
    if (wire.at("way").is_null()) {
      continue;
    }
    std::vector<std::string> crossed;
    for (const auto& index : wire.at("way")) {
      const auto& edge = edges.at(index.get<std::size_t>());
      if (edge.at("kind") == "stretch") {
        crossed.push_back(edge.at("feedline").get<std::string>());
      }
    }
    auto prescribed = wire.at("crosses").get<std::vector<std::string>>();
    std::ranges::sort(crossed);
    std::ranges::sort(prescribed);
    EXPECT_EQ(crossed, prescribed) << wire.at("name");
  }
  // A bottleneck is an edge only where the insertion has a hand in it: an
  // end on a feedline edge, a coupler pad or a resonator's lead.
  const auto ofTheInsertion = [](const std::string& end) {
    return end.starts_with("pad of ") || end.starts_with("lead of ") ||
           (end.size() > 1 && end[0] == 'f' &&
            std::isdigit(static_cast<unsigned char>(end[1])) != 0);
  };
  for (const auto& edge : edges) {
    if (edge.at("kind") != "bottleneck") {
      continue;
    }
    const auto& ends = edge.at("ends");
    EXPECT_TRUE(ofTheInsertion(ends[0].get<std::string>()) ||
                ofTheInsertion(ends[1].get<std::string>()))
        << ends;
  }
  // A port run is the rule's straight length from the port, once: a target
  // run starts at the target port and not at the router's target cell,
  // which already lies that far out.
  const auto straight = data.at("straightLength").get<double>();
  ASSERT_GT(straight, 1.0);
  std::size_t runs = 0;
  for (const auto& stroke : data.at("strokes")) {
    const auto name = stroke.at("name").get<std::string>();
    if (!name.ends_with(" at its target, left") &&
        !name.ends_with(" at its source, left")) {
      continue;
    }
    ++runs;
    const auto& points = stroke.at("points");
    ASSERT_EQ(points.size(), 2U) << name;
    const auto length =
        std::hypot(points[1][0].get<double>() - points[0][0].get<double>(),
                   points[1][1].get<double>() - points[0][1].get<double>());
    // The ends are rounded to whole cells.
    EXPECT_NEAR(length, straight, 1.5) << name;
  }
  EXPECT_GE(runs, wires.size());
  // Every terminal lies in the slot of its port run, a free cell between the
  // run's two walls.
  std::vector<std::size_t> codes;
  codes.reserve(width * height);
  for (std::size_t at = 0; at < raster.size(); at += 2) {
    codes.insert(codes.end(), raster[at + 1].get<std::size_t>(),
                 raster[at].get<std::size_t>());
  }
  ASSERT_EQ(codes.size(), width * height);
  const auto& kinds = data.at("wallKinds");
  const auto stub = static_cast<std::size_t>(std::ranges::find(kinds, "stub") -
                                             kinds.begin() + 1);
  ASSERT_LE(stub, kinds.size());
  std::set<std::pair<std::int64_t, std::int64_t>> slots;
  for (const auto& cell : data.at("slots")) {
    const auto x = cell[0].get<std::int64_t>();
    const auto y = cell[1].get<std::int64_t>();
    slots.emplace(x, y);
    const auto code = codes.at((static_cast<std::size_t>(y) * width) +
                               static_cast<std::size_t>(x));
    EXPECT_TRUE(code == 0 || code >= 6) << x << "," << y << " is a wall";
  }
  for (const auto& wire : wires) {
    for (const auto* const end : {"source", "target"}) {
      const auto& terminal = wire.at(end).at("cell");
      EXPECT_TRUE(slots.contains(
          {terminal[0].get<std::int64_t>(), terminal[1].get<std::int64_t>()}))
          << wire.at("name") << " " << end;
    }
  }
  // A port run is two walls one cell thick: no square of four cells is all
  // port run.
  const auto isStub = [&](const std::size_t x, const std::size_t y) {
    return codes[(y * width) + x] == stub;
  };
  std::size_t stubCells = 0;
  for (std::size_t y = 0; y + 1 < height; ++y) {
    for (std::size_t x = 0; x + 1 < width; ++x) {
      stubCells += isStub(x, y) ? 1 : 0;
      EXPECT_FALSE(isStub(x, y) && isStub(x + 1, y) && isStub(x, y + 1) &&
                   isStub(x + 1, y + 1))
          << x << "," << y;
    }
  }
  EXPECT_GT(stubCells, 0U);
  // No bottleneck joins the two walls of one port run: its line would cross
  // the slot between them.
  const auto runOf = [](const std::string& end) {
    for (const std::string_view side : {", left", ", right"}) {
      if (end.ends_with(side)) {
        return end.substr(0, end.size() - side.size());
      }
    }
    return std::string();
  };
  std::vector<nlohmann::json> cuts;
  for (const auto& edge : edges) {
    if (edge.at("kind") == "bottleneck") {
      cuts.push_back(edge);
    }
  }
  for (const auto& cut : data.at("aside")) {
    cuts.push_back(cut);
  }
  for (const auto& cut : cuts) {
    const auto& ends = cut.at("ends");
    const auto run = runOf(ends[0].get<std::string>());
    EXPECT_TRUE(run.empty() || run != runOf(ends[1].get<std::string>()))
        << ends;
  }
  // The end of a port run at its port is closed: no way leads from the slot
  // round behind the run, so a cut that ends on a port run parts two
  // chambers.
  for (const auto& cut : data.at("aside")) {
    const auto& ends = cut.at("ends");
    EXPECT_TRUE(runOf(ends[0].get<std::string>()).empty() &&
                runOf(ends[1].get<std::string>()).empty())
        << ends;
  }
}

} // namespace
} // namespace mqt::scpd::pipeline
