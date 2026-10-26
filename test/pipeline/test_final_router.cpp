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
#include "MiniFixture.hpp"
#include "mqt-scpd/drc/Rules.hpp"
#include "mqt-scpd/flatbuffers/artifacts.hpp"
#include "mqt-scpd/flatbuffers/drc.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"
#include "mqt-scpd/grid/Rasterize.hpp"
#include "mqt-scpd/pipeline/CapacityPlanner.hpp"
#include "mqt-scpd/pipeline/CouplerSession.hpp"
#include "mqt-scpd/pipeline/Registry.hpp"
#include "mqt-scpd/pipeline/Report.hpp"

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <format>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <numbers>
#include <optional>
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
using flatbuffers::config::FinalParamsT;
using flatbuffers::config::StageParamsT;
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
                         ->run(benchmark.chip, benchmark.config, {});
  planned.global =
      globalRouters()
          .make("hanan-milp")
          ->run(benchmark.chip, planned.capacity, benchmark.config, {});
  planned.assignment = assigners()
                           .make("ordered-milp")
                           ->run(benchmark.chip, planned.capacity,
                                 planned.global, benchmark.config, {});
  planned.corridor = corridorRouters()
                         .make("partition-astar")
                         ->run(benchmark.chip, planned.capacity,
                               planned.assignment, benchmark.config, {});
  planned.detail =
      detailRouters()
          .make("pixel-astar")
          ->run(benchmark.chip, planned.capacity, planned.global,
                planned.assignment, planned.corridor, benchmark.config);
  return planned;
}

/// The `[stages.final]` parameters of a benchmark's configuration, for a
/// test that sets one of them.
FinalParamsT& finalParamsOf(Benchmark& benchmark) {
  if (benchmark.config.stages == nullptr) {
    benchmark.config.stages = std::make_unique<StageParamsT>();
  }
  if (benchmark.config.stages->final == nullptr) {
    benchmark.config.stages->final = std::make_unique<FinalParamsT>();
  }
  return *benchmark.config.stages->final;
}

/// The coupler insertion's prices before the defaults of 2026-10-22: every
/// end edge at its full price (`chain_end_edge_weight = 1`) and every
/// offset (`coupler_offsets = "0,1,2,3,4,5,6,7"`). For the tests written
/// for a coupler on a named option or a chip as those prices leave it.
Benchmark withOldPrices(Benchmark benchmark) {
  auto& final = finalParamsOf(benchmark);
  final.chain_end_edge_weight = 1.0;
  final.coupler_offsets = "0,1,2,3,4,5,6,7";
  return benchmark;
}

/// A report that keeps every line a stage says down to `level`: the steps
/// of the stage, as a run without `-v` shows them, unless asked for more.
Report linesInto(std::vector<std::string>& lines,
                 const Detail level = Detail::Steps) {
  return {
      level,
      {.line = [&lines](const Detail /*level*/, const std::string_view text) {
        lines.emplace_back(text);
      }}};
}

/// A report that hands every line a stage says to `listen`.
Report hearing(std::function<void(std::string_view)> listen) {
  return {Detail::Solver,
          {.line = [listen = std::move(listen)](const Detail /*level*/,
                                                const std::string_view text) {
            listen(text);
          }}};
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

/// A drawn wire that carries a resonator's length, and how far that length
/// lies from the target.
struct Deviation {
  const flatbuffers::artifacts::FinalWireT* wire = nullptr;
  double deviation = 0.0;
};

/// The resonators the stage drew, each with how far its length lies from the
/// target: the length the artifact carries plus the run from the last cell
/// of the way to the port itself, which the wire covers without cells of its
/// own. A resonator that reaches its qubit through a bridge carries its
/// length on the inner wire beyond the bridge, and the wire that feeds the
/// bridge carries none.
std::vector<Deviation> resonatorDeviations(const Benchmark& benchmark,
                                           const Planned& planned,
                                           const FinalRoutingT& routing) {
  using flatbuffers::design::UnassignedRole;
  const auto grid = gridOf(routing);
  const double target = benchmark.config.rules->target_resonator_length;
  const auto roleOf = [&](const std::uint32_t port) {
    return benchmark.chip.ports[port]->role;
  };
  std::vector<Deviation> deviations;
  const auto measure = [&](const flatbuffers::artifacts::FinalWireT& wire,
                           const std::uint32_t port) {
    const auto& center = benchmark.chip.ports[port]->center;
    const auto& last = wire.path.back();
    const auto end = grid.toLayout(last.x(), last.y());
    const double gap = std::hypot(end.x() - center.x(), end.y() - center.y());
    deviations.push_back(
        {.wire = &wire, .deviation = (wire.length + gap) - target});
  };
  for (std::size_t index = 0; index < routing.wires.size(); ++index) {
    const auto& wire = *routing.wires[index];
    const auto& connection = *planned.assignment.connections[index];
    if (wire.path.empty() ||
        connection.target_role != AssignedRole::ResonatorTarget ||
        roleOf(connection.target.index()) == UnassignedRole::BridgePair) {
      continue;
    }
    measure(wire, connection.target.index());
  }
  for (std::size_t index = 0; index < routing.inner.size(); ++index) {
    const auto& wire = *routing.inner[index];
    const auto& connection = *planned.global.connections[index];
    if (wire.path.empty() || connection.source == nullptr ||
        roleOf(connection.source->index()) != UnassignedRole::BridgePair ||
        roleOf(connection.target.index()) != UnassignedRole::Resonator) {
      continue;
    }
    measure(wire, connection.target.index());
  }
  return deviations;
}

/// How far the resonator farthest from the target length lies from it.
double worstResonatorDeviation(const Benchmark& benchmark,
                               const Planned& planned,
                               const FinalRoutingT& routing) {
  double worst = 0.0;
  for (const auto& [wire, deviation] :
       resonatorDeviations(benchmark, planned, routing)) {
    worst = std::max(worst, std::abs(deviation));
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
  std::vector<const flatbuffers::design::CpwCouplerT*> couplerOf(
      routing.wires.size(), nullptr);
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

/// Every drawn wire carries its length, and the stage's verdict on a
/// resonator's length is the truth of the copper: a resonator it calls
/// neither short nor long is the target length within the rule's tolerance,
/// a resonator it calls short is shorter than that, and one it calls long is
/// longer. The stage does not promise that every resonator reaches the
/// length: where the chip has no room for a meander, it says so instead.
TEST_P(Final, ResonatorsReachTheTargetLength) {
  const auto& [benchmark, planned, routing] = routedOf(GetParam());
  ASSERT_GT(benchmark.config.rules->target_resonator_length, 0.0);
  const double tolerance = benchmark.config.rules->resonator_length_tolerance;
  using mqt::scpd::flatbuffers::artifacts::FinalVerdict;

  std::size_t resonators = 0;
  for (const auto& [wire, deviation] :
       resonatorDeviations(benchmark, planned, routing)) {
    EXPECT_GT(wire->length, 0.0) << "resonator " << resonators;
    const auto verdict = static_cast<std::uint8_t>(wire->verdict);
    const bool tooShort =
        (verdict & static_cast<std::uint8_t>(FinalVerdict::Short)) != 0;
    const bool tooLong =
        (verdict & static_cast<std::uint8_t>(FinalVerdict::Long)) != 0;
    if (tooShort) {
      EXPECT_LT(deviation, -tolerance) << "resonator " << resonators;
    } else if (tooLong) {
      EXPECT_GT(deviation, tolerance) << "resonator " << resonators;
    } else {
      EXPECT_LE(std::abs(deviation), tolerance) << "resonator " << resonators;
    }
    ++resonators;
  }
  EXPECT_GT(resonators, 0U);
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

/// The edge into a coupler ends at one port of the coupler and the edge out
/// of it starts at the other, so no two feedline edges share a cell: not two
/// consecutive edges of a chain, whose ends the coupler's length keeps
/// apart, and not edges of different chains.
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
      EXPECT_EQ(shared, 0U) << "feedline edges " << a << " and " << b
                            << " share " << shared << " cells";
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
  const std::vector<std::string> names{
      "pre-feedline-routing", "pre-feedline-refinement", "feedline-insertion",
      "feedline-constrained-routing", "feedline-constrained-refinement"};
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

/// What a report of the stage receives: its entries, its result and how
/// often the progress moved.
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

  /// The labels of the entries of one level, in order.
  [[nodiscard]] std::vector<std::string> labels(const Detail level) const {
    std::vector<std::string> found;
    for (const auto& [at, label] : entries) {
      if (at == level) {
        found.push_back(label);
      }
    }
    return found;
  }
};

/// The value of the figure `name <value>`, or nothing.
std::optional<std::uint64_t> valueOf(const std::vector<Figure>& figures,
                                     const std::string_view name) {
  for (const auto& figure : figures) {
    if (figure.text.starts_with(name) && figure.text.size() > name.size() + 1 &&
        figure.text[name.size()] == ' ') {
      return std::stoull(figure.text.substr(name.size() + 1));
    }
  }
  return std::nullopt;
}

/// The mini chip, as the stages take it.
Benchmark miniBenchmark() {
  Benchmark benchmark;
  benchmark.config = test::miniConfig();
  benchmark.chip = test::miniChip(benchmark.config);
  return benchmark;
}

/// Every pass starts every wire on the way the Detail stage drew and ends on
/// a line that says how many wires are unrouted, open, short, long, crossing
/// a feedline or meeting themselves, and the stage ends on one over every
/// wire. "Fails: 0 | Bad: 0" there promises what the artifact and the
/// design-rule check find: every connection and every feedline edge drawn,
/// no active finding, and every resonator the target length within
/// tolerance. `Fails` counts the resonators of the wrong length, `Bad` the
/// wires with no usable way.
TEST(FinalRouter, StartsOnTheDetailWaysAndReportsItsFails) {
  const auto benchmark = nineQubit();
  const auto planned = plan(benchmark);
  std::vector<std::string> lines;
  std::vector<std::string> passes;
  std::vector<std::vector<Figure>> results;
  std::vector<std::optional<std::uint64_t>> failed;
  const Report report{
      Detail::Steps,
      {.line =
           [&lines](const Detail /*level*/, const std::string_view text) {
             lines.emplace_back(text);
           },
       .entry =
           [&passes](const Detail /*level*/, const std::string_view label,
                     const std::vector<Figure>& /*figures*/) {
             passes.emplace_back(label);
           },
       .result =
           [&](const std::vector<Figure>& figures,
               const std::optional<std::uint64_t> fails) {
             results.push_back(figures);
             failed.push_back(fails);
           }}};
  const auto routing = finalRouters().make("dubins")->run(
      benchmark.chip, planned.capacity, planned.global, planned.assignment,
      planned.detail, benchmark.config, report);

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
  // Every pass ends on an entry of its own.
  EXPECT_NE(std::ranges::find(passes, "inner routing"), passes.end());
  EXPECT_NE(std::ranges::find(passes, "outer routing"), passes.end());
  // The result comes once, with the wires without a usable way as its fails
  // and the resonators of the wrong length beside them.
  ASSERT_EQ(results.size(), 1U);
  ASSERT_TRUE(failed.front().has_value());
  const auto& figures = results.front();
  const auto tooShort = valueOf(figures, "short");
  const auto tooLong = valueOf(figures, "long");
  const auto bad = valueOf(figures, "bad");
  ASSERT_TRUE(tooShort.has_value() && tooLong.has_value() && bad.has_value());
  const auto lengths = *tooShort + *tooLong;
  EXPECT_EQ(*failed.front(), *bad);

  // Every resonator's length is said, way plus the run to the port against
  // the target length, and a line sums them up.
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

  EXPECT_EQ(lengths + *bad == 0, holdsEverywhere(benchmark, planned, routing));

  // The end state carries the verdict against every wire, and the count of
  // the wires with one is the count the result said. An undrawn wire is
  // `Unrouted` and nothing else; a drawn one is not. The phase snapshots are
  // not judged and carry none.
  std::size_t held = 0;
  for (const auto* list :
       {&routing.wires, &routing.inner, &routing.feedlines}) {
    for (const auto& wire : *list) {
      const auto verdict = static_cast<std::uint8_t>(wire->verdict);
      held += verdict != 0 ? 1 : 0;
      const bool unrouted =
          (verdict &
           static_cast<std::uint8_t>(
               mqt::scpd::flatbuffers::artifacts::FinalVerdict::Unrouted)) != 0;
      EXPECT_EQ(unrouted, wire->path.empty());
    }
  }
  // A wire can be both of the wrong length and without a usable way, so it
  // counts in both figures but holds one verdict.
  EXPECT_LE(std::max(lengths, *bad), held);
  EXPECT_LE(held, lengths + *bad);
  for (const auto& phase : routing.phases) {
    for (const auto* list : {&phase->wires, &phase->inner, &phase->feedlines}) {
      for (const auto& wire : *list) {
        EXPECT_EQ(static_cast<std::uint8_t>(wire->verdict), 0) << phase->name;
      }
    }
  }
}

/// The stage reports each phase as one entry of the summary, in the order
/// the phases run, moves the progress while it searches, and reports its
/// result once: the wires without a usable way are its fails, and the
/// resonators of the wrong length are figures beside them.
TEST(FinalRouter, ReportsOneEntryPerPhaseAndItsResultOnce) {
  const auto benchmark = miniBenchmark();
  const auto planned = plan(benchmark);
  Spoken spoken;
  static_cast<void>(finalRouters().make("dubins")->run(
      benchmark.chip, planned.capacity, planned.global, planned.assignment,
      planned.detail, benchmark.config, spoken.report()));

  EXPECT_EQ(spoken.labels(Detail::Summary),
            (std::vector<std::string>{
                "pre-feedline-routing", "pre-feedline-refinement",
                "feedline-insertion", "feedline-constrained-routing",
                "feedline-constrained-refinement"}));
  EXPECT_GT(spoken.progressed, 0U);
  ASSERT_EQ(spoken.results.size(), 1U);
  const auto& figures = spoken.results.front();
  ASSERT_TRUE(spoken.fails.front().has_value());
  const auto tooShort = valueOf(figures, "short");
  const auto tooLong = valueOf(figures, "long");
  const auto bad = valueOf(figures, "bad");
  ASSERT_TRUE(tooShort.has_value() && tooLong.has_value() && bad.has_value());
  EXPECT_EQ(*spoken.fails.front(), *bad);
  EXPECT_TRUE(valueOf(figures, "angle cost").has_value());
  EXPECT_TRUE(std::ranges::any_of(figures, [](const Figure& figure) {
    return figure.text.starts_with("drawn ");
  }));
}

/// `[run] stop_after` ends the entries with the phase it names, and the
/// result then carries no figure of the insertion.
TEST(FinalRouter, StopsItsEntriesAtThePhaseTheRunStopsAfter) {
  auto benchmark = miniBenchmark();
  benchmark.config.run = std::make_unique<flatbuffers::config::RunParamsT>();
  benchmark.config.run->stop_after = "pre-feedline-refinement";
  const auto planned = plan(benchmark);
  Spoken spoken;
  static_cast<void>(finalRouters().make("dubins")->run(
      benchmark.chip, planned.capacity, planned.global, planned.assignment,
      planned.detail, benchmark.config, spoken.report()));

  EXPECT_EQ(spoken.labels(Detail::Summary),
            (std::vector<std::string>{"pre-feedline-routing",
                                      "pre-feedline-refinement"}));
  ASSERT_EQ(spoken.results.size(), 1U);
  EXPECT_FALSE(valueOf(spoken.results.front(), "angle cost").has_value());
}

/// The global SAT search starts from the slot check of the whole chip as the
/// commit leaves it. The nine-qubit chip is SAT there, so the search says
/// so and searches nothing.
TEST(FinalRouter, TheGlobalSatSearchFindsTheNineQubitChipSatAfterTheCommit) {
  const auto benchmark = nineQubit();
  const auto planned = plan(benchmark);
  std::vector<std::string> lines;
  static_cast<void>(finalRouters().make("dubins")->run(
      benchmark.chip, planned.capacity, planned.global, planned.assignment,
      planned.detail, benchmark.config, linesInto(lines)));

  const auto said = std::ranges::count_if(lines, [](const std::string& line) {
    return line.find("global SAT: the chip is SAT after the commit; nothing "
                     "to search") != std::string::npos;
  });
  const auto searched =
      std::ranges::count_if(lines, [](const std::string& line) {
        return line.find("global SAT: the chip is UNSAT") != std::string::npos;
      });
  EXPECT_EQ(said, 1);
  EXPECT_EQ(searched, 0);
}

/// What the global SAT search of one coupler insertion came to, with the
/// switches in `env` set while it runs.
struct GlobalSatOutcome {
  /// The couplers as the search left them (`CouplerSession::couplers`).
  std::string couplers;
  /// The verdict of the whole-chip check after the search.
  std::string verdict;
  /// The search's lines that name its combinations and its answer, in
  /// order, without the clock and the seconds.
  std::vector<std::string> combinations;
  /// The search's line per collected chain, sorted, without the seconds.
  std::vector<std::string> chains;
  /// Every line of the search.
  std::vector<std::string> lines;
  /// Every line of the insertion.
  std::vector<std::string> every;
};

GlobalSatOutcome globalSatOf(const Benchmark& benchmark,
                             const Planned& planned) {
  std::vector<std::string> lines;
  // Down to the items, so that every combination says what it lost.
  CouplerSession session(benchmark.chip, planned.global, planned.assignment,
                         planned.detail, benchmark.config,
                         linesInto(lines, Detail::Items));
  GlobalSatOutcome out;
  out.every = lines;
  out.couplers = session.couplers();
  out.verdict = nlohmann::json::parse(session.insertionCheck())
                    .at("verdict")
                    .get<std::string>();
  const std::regex seconds(R"(; [0-9.]+s(, [0-9.]+s of it in slot checks)?$)");
  for (const auto& line : lines) {
    const auto at = line.find("global SAT:");
    if (at == std::string::npos) {
      continue;
    }
    out.lines.push_back(line);
    const auto text = std::regex_replace(line.substr(at), seconds, "");
    if (text.starts_with("global SAT: combination ") ||
        text.starts_with("global SAT: SAT after ") ||
        text.starts_with("global SAT: no SAT combination ")) {
      out.combinations.push_back(text);
    } else if (text.find(" runs of ") != std::string::npos) {
      out.chains.push_back(text);
    }
  }
  std::ranges::sort(out.chains);
  return out;
}

/// `coupler_offsets` names the offsets the option generator turns a pad
/// by, 0,1,2,5,6,7 by default. Those offsets named are the default: the
/// couplers come out with the same options, and none on offset 3 or 4
/// unless the coupler says that none of the allowed offsets fits it. With
/// every offset named but the smallest the chip offers then, no coupler
/// has an option on it, and the settings line names the offsets allowed.
TEST(CouplerSession, OffersOnlyTheAllowedOffsets) {
  const auto benchmark = nineQubit();
  const auto planned = plan(benchmark);
  const auto withOffsets = [&](const char* offsets) {
    auto chip = benchmark;
    finalParamsOf(chip).coupler_offsets = offsets;
    return globalSatOf(chip, planned);
  };
  const auto plain = globalSatOf(benchmark, planned);
  const auto listed = withOffsets("0,1,2,5,6,7");
  EXPECT_EQ(listed.couplers, plain.couplers);

  const auto offsetsOf = [](const GlobalSatOutcome& outcome) {
    std::set<int> seen;
    const auto parsed = nlohmann::json::parse(outcome.couplers);
    for (const auto& coupler : parsed.at("couplers")) {
      for (const auto& option : coupler.at("options")) {
        seen.insert(option.at("offset").get<int>());
      }
    }
    return seen;
  };
  const auto fellBack = [](const GlobalSatOutcome& outcome) {
    return std::ranges::any_of(outcome.every, [](const std::string& line) {
      return line.find("every offset is allowed for it") != std::string::npos;
    });
  };
  ASSERT_FALSE(fellBack(plain));
  const auto byDefault = offsetsOf(plain);
  EXPECT_FALSE(byDefault.contains(3));
  EXPECT_FALSE(byDefault.contains(4));

  const auto every = withOffsets("0,1,2,3,4,5,6,7");
  const auto offered = offsetsOf(every);
  ASSERT_GT(offered.size(), 1U);
  const auto excluded = *offered.begin();
  std::string allowed;
  for (int offset = 0; offset < 8; ++offset) {
    if (offset != excluded) {
      allowed += std::format("{}{}", allowed.empty() ? "" : ",", offset);
    }
  }
  const auto reduced = withOffsets(allowed.c_str());
  ASSERT_FALSE(fellBack(reduced));
  const auto seen = offsetsOf(reduced);
  EXPECT_FALSE(seen.contains(excluded));
  EXPECT_FALSE(seen.empty());
  const auto named = std::format("coupler offsets {}", allowed);
  EXPECT_TRUE(std::ranges::any_of(reduced.every, [&](const std::string& line) {
    return line.find(named) != std::string::npos;
  }));
}

/// `chain_end_edge_weight` weighs the price of the edges between a
/// launcher and its coupler in the chain searches, 0.1 by default. At 0.1
/// the stage routes as without the key. At 0.1 every edge is still
/// drawn, and the first chain, which both searches settle on the same chip,
/// costs at weight 1 no less than its optimum at weight 1 when both settle
/// it on an optimum: the weight changes which run is cheapest, not what a
/// run costs. A later chain is checked against the chains settled before
/// it, which the weight may have moved.
TEST(FinalRouter, TheEndEdgeWeightChangesOnlyThePrice) {
  const auto benchmark = nineQubit();
  const auto planned = plan(benchmark);
  struct Outcome {
    FinalRoutingT routing;
    std::vector<std::string> lines;
  };
  const auto run = [&](const std::optional<double> weight) {
    auto chip = benchmark;
    if (weight) {
      finalParamsOf(chip).chain_end_edge_weight = *weight;
    }
    Outcome out;
    out.routing = finalRouters().make("dubins")->run(
        chip.chip, planned.capacity, planned.global, planned.assignment,
        planned.detail, chip.config, linesInto(out.lines));
    return out;
  };
  const auto plain = run(std::nullopt);
  const auto light = run(0.1);
  ASSERT_EQ(light.routing.wires.size(), plain.routing.wires.size());
  for (std::size_t index = 0; index < plain.routing.wires.size(); ++index) {
    EXPECT_EQ(light.routing.wires[index]->path,
              plain.routing.wires[index]->path)
        << "wire " << index << " differs at weight 0.1";
  }
  ASSERT_EQ(light.routing.feedlines.size(), plain.routing.feedlines.size());
  for (std::size_t index = 0; index < plain.routing.feedlines.size(); ++index) {
    EXPECT_EQ(light.routing.feedlines[index]->path,
              plain.routing.feedlines[index]->path)
        << "feedline " << index << " differs at weight 0.1";
  }

  const auto full = run(1.0);
  const std::regex optimum(
      R"(coupler insertion: chain (\d+): optimum (\d+) over \d+ layers)");
  const std::regex unweighted(R"(, (\d+) at weight 1$)");
  const std::regex drawn(R"(coupler insertion: .* (\d+) of (\d+) edges drawn)");
  std::map<std::string, std::uint64_t> best;
  for (const auto& line : full.lines) {
    std::smatch match;
    if (std::regex_search(line, match, optimum)) {
      best[match[1].str()] = std::stoull(match[2].str());
    }
  }
  ASSERT_FALSE(best.empty());
  std::size_t compared = 0;
  for (const auto& line : light.lines) {
    std::smatch match;
    if (std::regex_search(line, match, drawn)) {
      EXPECT_EQ(match[1].str(), match[2].str()) << line;
    }
    if (!std::regex_search(line, match, optimum)) {
      continue;
    }
    const auto chain = match[1].str();
    ASSERT_TRUE(std::regex_search(line, match, unweighted)) << line;
    if (const auto found = best.find(chain);
        chain == "0" && found != best.end()) {
      ++compared;
      EXPECT_GE(std::stoull(match[1].str()), found->second) << line;
    }
  }
  EXPECT_EQ(compared, 1U);
}

/// The capacity rule refuses every step of the chain search whose edge
/// closes a wire: every feedline edge of a chain settled under it closes
/// none. A chain searched again without the rule says so first.
TEST(FinalRouter, TheCapacityRuleLetsNoSettledEdgeCloseAWire) {
  const auto benchmark = nineQubit();
  const auto planned = plan(benchmark);
  std::vector<std::string> lines;
  static_cast<void>(finalRouters().make("dubins")->run(
      benchmark.chip, planned.capacity, planned.global, planned.assignment,
      planned.detail, benchmark.config, linesInto(lines)));

  std::set<std::string> withoutTheRule;
  const std::regex againLine(
      R"(coupler insertion: chain (\d+): .* searched again without)");
  const std::regex edgeLine(R"(capacity step: chain (\d+) f\d+: )");
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

/// `chain_rule_refusals` ends the search of a chain under the capacity
/// rule once the rule has refused that many steps before a run of options
/// joined the chain, and the chain is searched again without the rule. At
/// one refusal, a chain whose first refused step comes before its first run
/// says so and then gets its answer from the search without the rule. The
/// 17-qubit chip has such chains; the 9-qubit chip has none.
TEST(FinalRouter, TheRefusalLimitSearchesAChainAgainWithoutTheRule) {
  auto benchmark = seventeenQubit();
  finalParamsOf(benchmark).chain_rule_refusals = 1;
  const auto planned = plan(benchmark);
  std::vector<std::string> lines;
  static_cast<void>(finalRouters().make("dubins")->run(
      benchmark.chip, planned.capacity, planned.global, planned.assignment,
      planned.detail, benchmark.config, linesInto(lines)));

  const std::regex limit(
      R"(coupler insertion: chain (\d+): the capacity rule refused 1 step )"
      R"(before a run of options joined the chain \(chain_rule_refusals, )"
      R"(\d+ steps routed\) — searched again without the capacity rule)");
  const std::regex answer(
      R"(coupler insertion: chain (\d+): (optimum|best found|no run of )"
      R"(options joins the chain — leaving it as it stands))");
  std::set<std::string> limited;
  std::set<std::string> answered;
  for (const auto& line : lines) {
    std::smatch match;
    if (std::regex_search(line, match, limit)) {
      EXPECT_FALSE(answered.contains(match[1].str())) << line;
      limited.insert(match[1].str());
    } else if (std::regex_search(line, match, answer)) {
      answered.insert(match[1].str());
    }
  }
  EXPECT_FALSE(limited.empty());
  for (const auto& chain : limited) {
    EXPECT_TRUE(answered.contains(chain)) << "chain " << chain;
  }
}

/// The benchmark with the debug pages `turnOn` names on; every other page
/// stays off, as a configuration without a `[debug]` section leaves them.
Benchmark withDebugPages(
    Benchmark benchmark,
    const std::function<void(flatbuffers::config::DebugParamsT&)>& turnOn) {
  benchmark.config.debug =
      std::make_unique<flatbuffers::config::DebugParamsT>();
  turnOn(*benchmark.config.debug);
  return benchmark;
}

/// By default the chain search applies the capacity rule and the insertion
/// reports the capacity graph it leaves behind.
TEST(FinalRouter, TheCapacityRuleAndTheBottleneckReportAreOnByDefault) {
  const auto benchmark = withDebugPages(
      nineQubit(), [](auto& debug) { debug.feedline_insertion = true; });
  const auto planned = plan(benchmark);
  std::vector<std::string> lines;
  bool graphWritten = false;
  static_cast<void>(finalRouters().make("dubins")->run(
      benchmark.chip, planned.capacity, planned.global, planned.assignment,
      planned.detail, benchmark.config, linesInto(lines),
      [&graphWritten](const std::string_view kind, const std::string_view) {
        graphWritten = graphWritten || kind == "capacity-graph";
      }));

  const std::regex edgeLine(R"(capacity step: chain \d+ f\d+: )");
  EXPECT_TRUE(std::ranges::any_of(lines, [&](const std::string& line) {
    return std::regex_search(line, edgeLine);
  }));
  EXPECT_TRUE(graphWritten);
}

/// The layered search prices its steps by the stub bound: the analytic
/// bound with the forced end runs folded in, and says so.
TEST(FinalRouter, TheChainBoundDefaultsToTheStubBound) {
  const auto benchmark = fourQubit();
  const auto planned = plan(benchmark);
  std::vector<std::string> lines;
  static_cast<void>(finalRouters().make("dubins")->run(
      benchmark.chip, planned.capacity, planned.global, planned.assignment,
      planned.detail, benchmark.config, linesInto(lines)));

  EXPECT_TRUE(std::ranges::any_of(lines, [](const std::string& line) {
    return line.find("bound analytic with the end runs") != std::string::npos;
  }));
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

/// A coupler session runs the capacity check of the coupler insertion on
/// the chip as it stands, and then the feedline routing, which counts its
/// fails over wires the routing returns.
TEST(CouplerSession, ChecksTheChipAndRoutesTheFeedlines) {
  const auto benchmark = fourQubit();
  const auto planned = plan(benchmark);
  CouplerSession session(benchmark.chip, planned.global, planned.assignment,
                         planned.detail, benchmark.config);

  const auto check = nlohmann::json::parse(session.insertionCheck());
  const auto verdict = check.at("verdict").get<std::string>();
  EXPECT_TRUE(verdict == "SAT" || verdict == "UNSAT") << verdict;
  EXPECT_EQ(verdict == "SAT", check.at("lost").empty());
  for (const auto* const key : {"raster", "nodes", "edges", "wires"}) {
    EXPECT_TRUE(check.at("graph").contains(key)) << key;
  }

  const auto routed = nlohmann::json::parse(session.routeFeedlines());
  std::set<std::string> names;
  bool feedline = false;
  for (const auto& wire : routed.at("wires")) {
    names.insert(wire.at("name").get<std::string>());
    feedline = feedline || wire.at("kind") == "feedline";
    EXPECT_EQ(wire.at("drawn").get<bool>(), !wire.at("path").empty());
  }
  EXPECT_TRUE(feedline);
  std::size_t bad = 0;
  for (const auto* const kind :
       {"unrouted", "open", "crossing", "short", "long", "loops"}) {
    for (const auto& name : routed.at(kind)) {
      EXPECT_TRUE(names.contains(name.get<std::string>())) << kind;
    }
  }
  for (const auto* const kind : {"unrouted", "open", "crossing"}) {
    bad += routed.at(kind).size();
  }
  EXPECT_EQ(routed.at("bad").get<std::size_t>(), bad);
  // Every round of the feedline sweep, with the wires that failed in it.
  ASSERT_FALSE(routed.at("rounds").empty());
  for (const auto& round : routed.at("rounds")) {
    for (const auto& name : round.at("failed")) {
      EXPECT_TRUE(names.contains(name.get<std::string>()));
    }
  }
}

/// The insertion lays no edge over the run into a target port: the same
/// moves as `LosesAWireWhosePortAFeedlineCovers` leave every port of 4q in
/// a chamber.
TEST(CouplerSession, KeepsTheEdgesOffThePortStubs) {
  // Written for the options and the chip of the old prices.
  const auto benchmark = withOldPrices(fourQubit());
  const auto planned = plan(benchmark);
  CouplerSession session(benchmark.chip, planned.global, planned.assignment,
                         planned.detail, benchmark.config);
  for (const auto& [coupler, option] :
       {std::pair{0U, 16U}, std::pair{1U, 17U}, std::pair{2U, 1U}}) {
    static_cast<void>(session.setOption(coupler, option));
  }

  const auto check = nlohmann::json::parse(session.insertionCheck());
  for (const auto& line : check.at("graph").at("summary")) {
    EXPECT_FALSE(line.get<std::string>().starts_with("a port in no chamber"))
        << line;
  }
}

/// No wire crosses a terminal feedline edge in the capacity graph: such an
/// edge has no stretch.
TEST(CouplerSession, GivesATerminalEdgeNoStretch) {
  const auto benchmark = fourQubit();
  const auto planned = plan(benchmark);
  CouplerSession session(benchmark.chip, planned.global, planned.assignment,
                         planned.detail, benchmark.config);

  const auto graph =
      nlohmann::json::parse(session.insertionCheck()).at("graph");
  std::size_t stretches = 0;
  for (const auto& edge : graph.at("edges")) {
    if (edge.at("kind") == "stretch") {
      ++stretches;
      EXPECT_FALSE(edge.value("terminal", false)) << edge.at("name");
    }
  }
  EXPECT_GT(stretches, 0U);
}

/// Every cell a way from `CouplerSession::routeFeedlines` stands on, and
/// every unit square it steps across diagonally with the way the step
/// leans, from the corners the JSON keeps.
struct Covered {
  std::set<std::pair<std::int64_t, std::int64_t>> cells;
  std::set<std::pair<std::pair<std::int64_t, std::int64_t>, bool>> diagonals;

  /// Whether two ways cross: they share a cell, or two diagonal steps pass
  /// through one unit square the other way round.
  [[nodiscard]] bool crosses(const Covered& other) const {
    return std::ranges::any_of(
               cells,
               [&](const auto& cell) { return other.cells.contains(cell); }) ||
           std::ranges::any_of(diagonals, [&](const auto& step) {
             return other.diagonals.contains({step.first, !step.second});
           });
  }
};

Covered coverOf(const nlohmann::json& path) {
  Covered covered;
  for (std::size_t k = 0; k + 1 < path.size(); ++k) {
    auto x = path[k][0].get<std::int64_t>();
    auto y = path[k][1].get<std::int64_t>();
    const auto tx = path[k + 1][0].get<std::int64_t>();
    const auto ty = path[k + 1][1].get<std::int64_t>();
    const auto dx = (tx > x) - (tx < x);
    const auto dy = (ty > y) - (ty < y);
    while (x != tx || y != ty) {
      covered.cells.insert({x, y});
      if (dx != 0 && dy != 0) {
        covered.diagonals.insert(
            {{std::min(x, x + dx), std::min(y, y + dy)}, dx == dy});
      }
      x += dx;
      y += dy;
    }
    covered.cells.insert({tx, ty});
  }
  return covered;
}

/// The moves of a random walk over the coupler options of 4q, replayed on
/// a fresh session.
void replay(CouplerSession& session,
            const std::vector<std::pair<std::uint32_t, std::uint32_t>>& moves) {
  for (const auto& [coupler, option] : moves) {
    static_cast<void>(session.setOption(coupler, option));
  }
}

/// Whether the feedline routing counts a wire as bad: unrouted, open or
/// crossing.
bool isBad(const nlohmann::json& routed, const std::string& name) {
  return std::ranges::any_of(
      std::array{"unrouted", "open", "crossing"}, [&](const char* kind) {
        return std::ranges::any_of(routed.at(kind), [&](const auto& listed) {
          return listed.template get<std::string>() == name;
        });
      });
}

/// A wire whose port a feedline edge leaves room for only by being crossed
/// on the port run has no legal way: the capacity check loses it, and the
/// feedline routing counts it as bad. On 4q this walk lays f1 across the
/// run in front of the target of wire 5.
TEST(CouplerSession, KeepsTheFeedlinesOffAWiresStubs) {
  // Written for the options and the chip of the old prices.
  const auto benchmark = withOldPrices(fourQubit());
  const auto planned = plan(benchmark);
  CouplerSession session(benchmark.chip, planned.global, planned.assignment,
                         planned.detail, benchmark.config);
  replay(session, {{1, 12},
                   {1, 19},
                   {1, 17},
                   {3, 16},
                   {3, 6},
                   {3, 3},
                   {3, 13},
                   {3, 1},
                   {1, 15}});

  const auto check = nlohmann::json::parse(session.insertionCheck());
  const auto routed = nlohmann::json::parse(session.routeFeedlines());
  const bool lost = std::ranges::any_of(check.at("lost"), [](const auto& wire) {
    return wire.template get<std::string>() == "5";
  });
  EXPECT_EQ(lost, isBad(routed, "5"));
}

/// A resonator crosses no feedline edge between couplers after the feedline
/// routing, the edges of its own coupler included, or the routing counts it
/// as bad. On 4q this walk puts an edge of resonator 4's own coupler in
/// front of its target.
TEST(CouplerSession, KeepsAResonatorOffTheEdgesBetweenCouplers) {
  // Written for the options and the chip of the old prices.
  const auto benchmark = withOldPrices(fourQubit());
  const auto planned = plan(benchmark);
  CouplerSession session(benchmark.chip, planned.global, planned.assignment,
                         planned.detail, benchmark.config);
  replay(session,
         {{1, 23}, {0, 2}, {2, 20}, {2, 14}, {2, 11}, {2, 2}, {2, 9}, {2, 1}});
  // An edge between couplers is the edge out of one and into the next, so
  // it is named twice; a terminal edge once.
  std::map<std::string, std::uint32_t> named;
  const auto listed = nlohmann::json::parse(session.couplers());
  for (const auto& coupler : listed.at("couplers")) {
    for (const auto* const side : {"edgeIn", "edgeOut"}) {
      if (!coupler.at(side).is_null()) {
        ++named[coupler.at(side).get<std::string>()];
      }
    }
  }

  const auto routed = nlohmann::json::parse(session.routeFeedlines());
  std::map<std::string, Covered> between;
  for (const auto& wire : routed.at("wires")) {
    const auto name = wire.at("name").get<std::string>();
    if (wire.at("kind") == "feedline" && named[name] == 2) {
      between[name] = coverOf(wire.at("path"));
    }
  }
  ASSERT_FALSE(between.empty());
  for (const auto& wire : routed.at("wires")) {
    const auto name = wire.at("name").get<std::string>();
    if (wire.at("kind") != "resonator" || isBad(routed, name)) {
      continue;
    }
    const auto way = coverOf(wire.at("path"));
    for (const auto& [edge, covered] : between) {
      EXPECT_FALSE(way.crosses(covered)) << name << " crosses " << edge;
    }
  }
}

/// The gaps between a resonator's lead and an edge of its own coupler are
/// bottlenecks of the capacity graph, also where the edge is a terminal
/// one, and they take what they measure. On 4q, coupler 1 on option 19
/// leaves resonator 4 no way out between its lead and the terminal edge f0.
TEST(CouplerSession, KeepsTheGapsBetweenALeadAndItsEdges) {
  // Written for the options and the chip of the old prices.
  const auto benchmark = withOldPrices(fourQubit());
  const auto planned = plan(benchmark);
  CouplerSession session(benchmark.chip, planned.global, planned.assignment,
                         planned.detail, benchmark.config);
  replay(session, {{1, 12}, {1, 19}});

  // The terminal edges are wires of the check, not walls, so the gap is
  // between the lead and the edge on from the coupler.
  const auto check = nlohmann::json::parse(session.insertionCheck());
  bool found = false;
  for (const auto& edge : check.at("graph").at("edges")) {
    const auto ends = edge.value("ends", nlohmann::json::array());
    const bool lead = std::ranges::any_of(ends, [](const auto& end) {
      return end.template get<std::string>().starts_with("lead of 4");
    });
    const bool ownEdge = std::ranges::any_of(ends, [](const auto& end) {
      const auto name = end.template get<std::string>();
      return name == "f0" || name == "f1";
    });
    if (lead && ownEdge) {
      found = true;
      EXPECT_LT(edge.at("capacity").get<std::size_t>(), 2U) << edge.at("name");
    }
  }
  EXPECT_TRUE(found);
}

/// The crossings of a feedline edge are points a lane spacing apart, the
/// wire clearance times √2: two wires that cross a clearance apart keep it
/// only while both run straight, and past the band they turn, and after a
/// common 45° turn their lanes lie a clearance over √2 apart. So a stretch
/// takes at most one crossing per lane spacing of its length, plus one.
TEST(CouplerSession, PacksTheCrossingsOfAnEdgeALaneSpacingApart) {
  const auto benchmark = fourQubit();
  const auto planned = plan(benchmark);
  CouplerSession session(benchmark.chip, planned.global, planned.assignment,
                         planned.detail, benchmark.config);

  const auto check = nlohmann::json::parse(session.insertionCheck());
  const auto clearance = check.at("graph").at("clearance").get<double>();
  const auto spacing = clearance * std::numbers::sqrt2;
  bool tighter = false;
  for (const auto& edge : check.at("graph").at("edges")) {
    if (edge.at("kind") != "stretch" || edge.at("terminal").get<bool>()) {
      continue;
    }
    const auto length = edge.at("length").get<double>();
    const auto most =
        static_cast<std::size_t>(std::floor(length / spacing)) + 1;
    EXPECT_LE(edge.at("capacity").get<std::size_t>(), most) << edge.at("name");
    tighter =
        tighter ||
        static_cast<std::size_t>(std::floor(length / clearance)) + 1 > most;
  }
  // The chip has a stretch that the clearance alone would pack tighter.
  EXPECT_TRUE(tighter);
}

/// The insertion check judges every chain on its own first (user,
/// 2026-10-14): `local` holds one verdict per chain, from the capacity graph
/// of the chip with only that chain's edges standing, and names the edges
/// it stood.
TEST(CouplerSession, ChecksEveryChainOnItsOwnFirst) {
  const auto benchmark = nineQubit();
  const auto planned = plan(benchmark);
  CouplerSession session(benchmark.chip, planned.global, planned.assignment,
                         planned.detail, benchmark.config);
  const auto listed = nlohmann::json::parse(session.couplers()).at("couplers");
  std::map<std::uint32_t, std::set<std::string>> edgesOf;
  for (const auto& coupler : listed) {
    for (const auto* const side : {"edgeIn", "edgeOut"}) {
      if (!coupler.at(side).is_null()) {
        edgesOf[coupler.at("chain").get<std::uint32_t>()].insert(
            coupler.at(side).get<std::string>());
      }
    }
  }
  ASSERT_GT(edgesOf.size(), 1U);

  const auto check = nlohmann::json::parse(session.insertionCheck());
  const auto& local = check.at("local");
  ASSERT_EQ(local.size(), edgesOf.size());
  for (const auto& entry : local) {
    const auto chain = entry.at("chain").get<std::uint32_t>();
    const auto verdict = entry.at("verdict").get<std::string>();
    EXPECT_TRUE(verdict == "SAT" || verdict == "UNSAT") << verdict;
    EXPECT_TRUE(entry.at("lost").is_array());
    EXPECT_TRUE(entry.at("shortBy").is_number());
    std::set<std::string> stood;
    for (const auto& name : entry.at("edges")) {
      stood.insert(name.get<std::string>());
    }
    EXPECT_EQ(stood, edgesOf.at(chain)) << "chain " << chain;
  }
}

/// In the check of the whole chip the terminal edges, from a launcher to
/// the first coupler of a chain and from its last to a launcher, are wires
/// that have to find their way through the graph, not walls (user,
/// 2026-10-14): the insertion lays each chain on its own, so two terminal
/// edges may even cross, and as walls they would judge a chip that is not
/// the one the feedline routing draws.
TEST(CouplerSession, CarriesTheTerminalEdgesAsWiresInTheCheckOfTheChip) {
  const auto benchmark = fourQubit();
  const auto planned = plan(benchmark);
  CouplerSession session(benchmark.chip, planned.global, planned.assignment,
                         planned.detail, benchmark.config);

  const auto check = nlohmann::json::parse(session.insertionCheck());
  const auto& graph = check.at("graph");
  std::set<std::string> terminals;
  for (const auto& wire : graph.at("wires")) {
    if (wire.value("terminal", false)) {
      terminals.insert(wire.at("name").get<std::string>());
      EXPECT_TRUE(wire.at("crosses").empty()) << wire.at("name");
      EXPECT_FALSE(wire.at("status").get<std::string>().empty());
      EXPECT_FALSE(wire.at("source").at("chambers").empty()) << wire.at("name");
      EXPECT_FALSE(wire.at("target").at("chambers").empty()) << wire.at("name");
    }
  }
  // The chain of 4q runs from one launcher to another.
  EXPECT_EQ(terminals.size(), 2U);
  // Each keeps a port run at both ends: at the coupler the coupler's own
  // run, longer than the rule's straight length the plain wires keep, and
  // at the launcher the straight start (user, 2026-10-14).
  const auto straight = graph.at("straightLength").get<double>();
  std::size_t runs = 0;
  for (const auto& stroke : graph.at("strokes")) {
    const auto name = stroke.at("name").get<std::string>();
    if (stroke.at("kind") == "feedline") {
      EXPECT_FALSE(terminals.contains(name)) << name;
      continue;
    }
    const auto at = name.find(" at its ");
    if (at == std::string::npos || !terminals.contains(name.substr(0, at))) {
      continue;
    }
    ++runs;
    const auto& points = stroke.at("points");
    const auto length = std::hypot(
        points.back()[0].get<double>() - points.front()[0].get<double>(),
        points.back()[1].get<double>() - points.front()[1].get<double>());
    if (name.find("at its coupler") != std::string::npos) {
      // The pad's run of 21 steps, a diagonal step √2 cells long: 21 cells
      // axial, 29.7 diagonal.
      EXPECT_GE(length, 20.0) << name;
    } else {
      EXPECT_NEAR(length, straight, 1.5) << name;
    }
  }
  EXPECT_EQ(runs, 2 * terminals.size());
  for (const auto& edge : graph.at("edges")) {
    if (edge.at("kind") == "stretch") {
      EXPECT_FALSE(terminals.contains(edge.at("feedline").get<std::string>()))
          << edge.at("name");
    }
  }
}

/// With the terminal edges as wires the chains stand open at their ends,
/// so the check closes every mouth between two chains (user, 2026-10-14):
/// two lines, each from a flank of the run the ending terminal edge of one
/// chain leaves its coupler on to the flank on the same side of the run the
/// starting terminal edge of the next chain reaches its coupler on. Where a
/// line crosses a wall, a resonator's lead or a pad, the wall closes the
/// mouth there and the line is cut into the gates between the walls; every
/// gate holds its length over the clearance, under the capacity margin from
/// `capacity_margin_from_wires` wires on. The two ends of one chain enclose
/// its own pocket and close no mouth, so 4q with its one chain has none; 9q
/// has one, from the end of its second chain to the start of its first.
TEST(CouplerSession, ClosesTheMouthBetweenTwoChainsWithTwoLines) {
  {
    const auto benchmark = fourQubit();
    const auto planned = plan(benchmark);
    CouplerSession session(benchmark.chip, planned.global, planned.assignment,
                           planned.detail, benchmark.config);
    const auto check = nlohmann::json::parse(session.insertionCheck());
    for (const auto& edge : check.at("graph").at("edges")) {
      EXPECT_FALSE(edge.value("mouth", false)) << edge.at("name");
    }
  }
  const auto benchmark = nineQubit();
  const auto planned = plan(benchmark);
  CouplerSession session(benchmark.chip, planned.global, planned.assignment,
                         planned.detail, benchmark.config);

  const auto check = nlohmann::json::parse(session.insertionCheck());
  const auto& graph = check.at("graph");
  const auto clearance = graph.at("clearance").get<double>();
  const auto margin = graph.at("capacityMargin").get<double>();
  const auto marginFrom =
      graph.at("capacityMarginFromWires").get<std::size_t>();
  std::size_t mouths = 0;
  std::size_t fromFlanks = 0;
  for (const auto& edge : graph.at("edges")) {
    if (!edge.value("mouth", false)) {
      continue;
    }
    ++mouths;
    EXPECT_EQ(edge.at("kind"), "bottleneck");
    for (const auto& end : edge.at("ends")) {
      const auto name = end.get<std::string>();
      fromFlanks += name.starts_with("f") &&
                            name.find(" at its coupler, ") != std::string::npos
                        ? 1
                        : 0;
    }
    // Its length over the clearance, under the margin from `marginFrom`
    // wires on and under the share of it the nearest of the eight headings
    // can use, which is cos(22.5 degrees) at the least; one more where a
    // gap too narrow for any wire is the own-run passage of the terminal
    // edge at its flank.
    const auto length = edge.at("length").get<double>();
    const auto plain = static_cast<std::size_t>(std::floor(length / clearance));
    const auto taken = plain >= marginFrom ? clearance * margin : clearance;
    const auto least = static_cast<std::size_t>(
        std::floor(length * std::cos(std::numbers::pi / 8.0) / taken));
    EXPECT_GE(edge.at("capacity").get<std::size_t>(), least) << edge.at("name");
    EXPECT_LE(edge.at("capacity").get<std::size_t>(), plain + 1)
        << edge.at("name");
    EXPECT_GE(edge.at("chambers").size(), 2U) << edge.at("name");
  }
  // One mouth of two lines, each from a flank of the one chain's last
  // coupler's run to a flank of the other chain's first coupler's run, in
  // as many gates as walls it crosses.
  EXPECT_GE(mouths, 2U);
  EXPECT_GE(fromFlanks, 4U);
  // A gate of a mouth line at the flank of a terminal edge's own run lets
  // that edge into its slot however narrow it is, so no terminal edge is
  // lost for want of a way into its slot.
  for (const auto& name : check.at("lost")) {
    EXPECT_FALSE(name.get<std::string>().starts_with("f")) << name;
  }
}

/// The crossings of a feedline edge are points, and none lies within the
/// wire clearance of the run a terminal edge keeps at its coupler or its
/// launcher (user, 2026-10-14), as none lies within it of a resonator's
/// lead: a wire crossing there would run into the terminal edge's straight.
TEST(CouplerSession, KeepsTheCrossingPointsClearOfTheTerminalEdgesRuns) {
  const auto benchmark = seventeenQubit();
  const auto planned = plan(benchmark);
  CouplerSession session(benchmark.chip, planned.global, planned.assignment,
                         planned.detail, benchmark.config);

  const auto check = nlohmann::json::parse(session.insertionCheck());
  const auto& graph = check.at("graph");
  const auto clearance = graph.at("clearance").get<double>();
  std::vector<std::array<double, 4>> runs;
  for (const auto& stroke : graph.at("strokes")) {
    const auto name = stroke.at("name").get<std::string>();
    if (!name.starts_with("f") || name.find(" at its ") == std::string::npos) {
      continue;
    }
    const auto& points = stroke.at("points");
    runs.push_back(
        {points.front()[0].get<double>(), points.front()[1].get<double>(),
         points.back()[0].get<double>(), points.back()[1].get<double>()});
  }
  ASSERT_FALSE(runs.empty());
  const auto apart = [](const std::array<double, 4>& run, const double x,
                        const double y) {
    const auto dx = run[2] - run[0];
    const auto dy = run[3] - run[1];
    const auto along = std::clamp(((x - run[0]) * dx + (y - run[1]) * dy) /
                                      ((dx * dx) + (dy * dy)),
                                  0.0, 1.0);
    return std::hypot(x - run[0] - (along * dx), y - run[1] - (along * dy));
  };
  std::size_t points = 0;
  for (const auto& edge : graph.at("edges")) {
    if (edge.at("kind") != "stretch" || edge.at("terminal").get<bool>()) {
      continue;
    }
    EXPECT_EQ(edge.at("points").size(), edge.at("capacity").get<std::size_t>())
        << edge.at("name");
    for (const auto& point : edge.at("points")) {
      ++points;
      const auto x = point[0].get<double>();
      const auto y = point[1].get<double>();
      for (const auto& run : runs) {
        EXPECT_GE(apart(run, x, y), clearance - 1e-9) << edge.at("name");
      }
    }
  }
  EXPECT_GT(points, 0U);
}

/// The capacity graph of the coupler insertion as `CouplerSession::graph`
/// gives it, before the check by crossing slots judges it: the graph of
/// gates and stretches, for the tests of that graph.
nlohmann::json stretchCheck(CouplerSession& session) {
  return nlohmann::json{{"graph", nlohmann::json::parse(session.graph())}};
}

/// A crossing of a feedline edge takes a rectangle (user, 2026-10-14): the
/// band's reach across the edge to either side and half a clearance along
/// it either way, and it touches no other wall. So around every crossing
/// point, past the edge's own inflated line, the cells of that rectangle
/// are free or chamber cells.
TEST(CouplerSession, KeepsARectangleClearAroundEveryCrossing) {
  const auto benchmark = seventeenQubit();
  const auto planned = plan(benchmark);
  CouplerSession session(benchmark.chip, planned.global, planned.assignment,
                         planned.detail, benchmark.config);

  const auto check = stretchCheck(session);
  const auto& graph = check.at("graph");
  const auto width = graph.at("width").get<std::int64_t>();
  const auto height = graph.at("height").get<std::int64_t>();
  const auto clearance = graph.at("clearance").get<double>();
  const auto reach = graph.at("crossingReach").get<std::int64_t>();
  const auto inflation = graph.at("inflation").get<double>();
  // The raster, run-length coded: a code below 6 is a wall.
  std::vector<std::uint8_t> code;
  code.reserve(static_cast<std::size_t>(width * height));
  const auto& raster = graph.at("raster");
  for (std::size_t at = 0; at + 1 < raster.size(); at += 2) {
    const auto value = static_cast<std::uint8_t>(
        std::min<std::size_t>(raster[at].get<std::size_t>(), 255));
    code.insert(code.end(), raster[at + 1].get<std::size_t>(), value);
  }
  ASSERT_EQ(code.size(), static_cast<std::size_t>(width * height));
  const auto free = [&](const std::int64_t x, const std::int64_t y) {
    if (x < 0 || y < 0 || x >= width || y >= height) {
      return false;
    }
    const auto c = code[static_cast<std::size_t>((y * width) + x)];
    return c == 0 || c >= 6;
  };
  std::size_t points = 0;
  for (const auto& edge : graph.at("edges")) {
    if (edge.at("kind") != "stretch" || edge.at("terminal").get<bool>() ||
        edge.at("points").empty()) {
      continue;
    }
    const auto& line = edge.at("line");
    const auto dx = line.back()[0].get<std::int64_t>() -
                    line.front()[0].get<std::int64_t>();
    const auto dy = line.back()[1].get<std::int64_t>() -
                    line.front()[1].get<std::int64_t>();
    if (dx == 0 && dy == 0) {
      continue;
    }
    const std::int64_t ax = (dx > 0) - (dx < 0);
    const std::int64_t ay = (dy > 0) - (dy < 0);
    const auto norm =
        std::hypot(static_cast<double>(ax), static_cast<double>(ay));
    const auto half =
        static_cast<std::int64_t>(std::ceil(clearance / 2.0 / norm));
    const auto beyond = static_cast<std::int64_t>(std::ceil(inflation / norm));
    for (const auto& point : edge.at("points")) {
      ++points;
      const auto px = point[0].get<std::int64_t>();
      const auto py = point[1].get<std::int64_t>();
      for (std::int64_t along = -half; along <= half; ++along) {
        for (std::int64_t across = -reach; across <= reach; ++across) {
          if (std::abs(across) <= beyond) {
            continue;
          }
          // Across the edge: the step turned by a right angle.
          EXPECT_TRUE(free(px + (along * ax) - (across * ay),
                           py + (along * ay) + (across * ax)))
              << edge.at("name") << " at " << px << "," << py << " along "
              << along << " across " << across;
        }
      }
    }
  }
  EXPECT_GT(points, 0U);
}

/// The slot of a terminal edge's run is closed at its mouth, at both ends
/// of the edge (user, 2026-10-14): a gate from the tip of one flank to the
/// tip of the other, for one wire, the edge's own. A wire that passed in
/// front of the slot would cross the edge where it leaves the slot.
TEST(CouplerSession, ClosesTheSlotOfEveryTerminalEdgesRunForTheEdgeAlone) {
  const auto benchmark = nineQubit();
  const auto planned = plan(benchmark);
  CouplerSession session(benchmark.chip, planned.global, planned.assignment,
                         planned.detail, benchmark.config);

  const auto check = nlohmann::json::parse(session.insertionCheck());
  const auto& graph = check.at("graph");
  std::set<std::string> terminals;
  for (const auto& wire : graph.at("wires")) {
    if (wire.value("terminal", false)) {
      terminals.insert(wire.at("name").get<std::string>());
    }
  }
  ASSERT_FALSE(terminals.empty());
  std::set<std::string> closed;
  for (const auto& edge : graph.at("edges")) {
    if (!edge.value("slot", false)) {
      continue;
    }
    const auto left = edge.at("ends")[0].get<std::string>();
    const auto right = edge.at("ends")[1].get<std::string>();
    const auto run = left.substr(0, left.rfind(", "));
    EXPECT_EQ(run, right.substr(0, right.rfind(", "))) << edge.at("ends");
    const auto name = run.substr(0, run.find(" at its "));
    EXPECT_TRUE(terminals.contains(name)) << run;
    EXPECT_EQ(edge.at("capacity").get<std::size_t>(), 1U) << run;
    ASSERT_TRUE(edge.at("users").is_array()) << run;
    EXPECT_EQ(edge.at("users").size(), 1U) << run;
    EXPECT_EQ(edge.at("users")[0].get<std::string>(), name) << run;
    closed.insert(run);
  }
  // Every terminal edge: its run at the launcher and at the coupler.
  EXPECT_EQ(closed.size(), 2 * terminals.size());
}

/// A gate between the run of a terminal edge and the run of a plain wire
/// is kept (user, 2026-10-14): the terminal edge's run moves with the
/// coupler's option, as a pad or a lead does, so the gate is the
/// insertion's to answer for. Every terminal edge leaves its launcher
/// beside the launchers of plain wires, so 17q has such gates.
TEST(CouplerSession, KeepsTheGatesAtTheRunsOfTheTerminalEdges) {
  const auto benchmark = seventeenQubit();
  const auto planned = plan(benchmark);
  CouplerSession session(benchmark.chip, planned.global, planned.assignment,
                         planned.detail, benchmark.config);

  const auto check = nlohmann::json::parse(session.insertionCheck());
  std::size_t kept = 0;
  for (const auto& edge : check.at("graph").at("edges")) {
    if (edge.at("kind") != "bottleneck" || edge.value("mouth", false)) {
      continue;
    }
    const auto a = edge.at("ends")[0].get<std::string>();
    const auto b = edge.at("ends")[1].get<std::string>();
    const auto terminalRun = [](const std::string& name) {
      return name.starts_with("f") &&
             (name.find(" at its launcher") != std::string::npos ||
              name.find(" at its coupler") != std::string::npos);
    };
    const auto plainRun = [](const std::string& name) {
      return !name.starts_with("f") && !name.starts_with("lead of") &&
             name.find(" at its ") != std::string::npos;
    };
    kept += (terminalRun(a) && plainRun(b)) || (terminalRun(b) && plainRun(a))
                ? 1
                : 0;
  }
  EXPECT_GT(kept, 0U);
}

/// A gate's line runs free between its two ends (user, 2026-10-14): a cut
/// that crossed a wall — the run of a terminal edge, say — is cut at that
/// wall into the gates of its free stretches, so no gate joins chambers
/// that a wall between them keeps apart.
TEST(CouplerSession, CutsEveryGateAtTheWallsItCrosses) {
  const auto benchmark = seventeenQubit();
  const auto planned = plan(benchmark);
  CouplerSession session(benchmark.chip, planned.global, planned.assignment,
                         planned.detail, benchmark.config);

  const auto check = stretchCheck(session);
  const auto& graph = check.at("graph");
  const auto width = graph.at("width").get<std::int64_t>();
  const auto height = graph.at("height").get<std::int64_t>();
  std::vector<std::uint8_t> code;
  const auto& raster = graph.at("raster");
  for (std::size_t at = 0; at + 1 < raster.size(); at += 2) {
    code.insert(code.end(), raster[at + 1].get<std::size_t>(),
                static_cast<std::uint8_t>(
                    std::min<std::size_t>(raster[at].get<std::size_t>(), 255)));
  }
  ASSERT_EQ(code.size(), static_cast<std::size_t>(width * height));
  std::size_t gates = 0;
  for (const auto& edge : graph.at("edges")) {
    if (edge.at("kind") != "bottleneck") {
      continue;
    }
    ++gates;
    const auto& line = edge.at("line");
    // Every cell strictly between the ends, on the line as the grid walks
    // it, is free or a chamber cell.
    const auto cells = grid::lineCells(
        line[0][0].get<std::int64_t>(), line[0][1].get<std::int64_t>(),
        line[1][0].get<std::int64_t>(), line[1][1].get<std::int64_t>(),
        static_cast<std::uint32_t>(width), static_cast<std::uint32_t>(height));
    for (std::size_t k = 1; k + 1 < cells.size(); ++k) {
      const auto c = code[cells[k]];
      EXPECT_TRUE(c == 0 || c >= 6)
          << edge.at("name") << " " << edge.at("ends") << " " << edge.at("line")
          << " crosses a wall at " << cells[k] % width << ","
          << cells[k] / width << " code " << static_cast<int>(c) << " cell "
          << k << " of " << cells.size();
    }
  }
  EXPECT_GT(gates, 0U);
}

/// A line of the capacity graph, in cells.
struct GateLine {
  double ax = 0.0;
  double ay = 0.0;
  double bx = 0.0;
  double by = 0.0;
};

GateLine gateLineOf(const nlohmann::json& edge) {
  const auto& line = edge.at("line");
  return {.ax = line[0][0].get<double>(),
          .ay = line[0][1].get<double>(),
          .bx = line[1][0].get<double>(),
          .by = line[1][1].get<double>()};
}

double lengthOf(const GateLine& line) {
  return std::hypot(line.bx - line.ax, line.by - line.ay);
}

/// How far `x, y` lies from the line, and where along it its foot is: 0 at
/// the line's first end, 1 at its second.
std::pair<double, double> awayFrom(const GateLine& line, const double x,
                                   const double y) {
  const double dx = line.bx - line.ax;
  const double dy = line.by - line.ay;
  const double squared = (dx * dx) + (dy * dy);
  const double at =
      squared > 0.0
          ? std::clamp((((x - line.ax) * dx) + ((y - line.ay) * dy)) / squared,
                       0.0, 1.0)
          : 0.0;
  return {std::hypot(x - (line.ax + (at * dx)), y - (line.ay + (at * dy))), at};
}

/// Whether the two lines cross more than `near` cells from the ends of
/// both, or one ends within `near` cells of the other's middle.
bool meetInTheMiddle(const GateLine& a, const GateLine& b,
                     const double near = 1.5) {
  const auto lengthA = lengthOf(a);
  const auto lengthB = lengthOf(b);
  if (lengthA < 1.0 || lengthB < 1.0) {
    return false;
  }
  const double rx = a.bx - a.ax;
  const double ry = a.by - a.ay;
  const double sx = b.bx - b.ax;
  const double sy = b.by - b.ay;
  const double den = (rx * sy) - (ry * sx);
  if (std::abs(den) > 1e-9) {
    const double t = (((b.ax - a.ax) * sy) - ((b.ay - a.ay) * sx)) / den;
    const double u = (((b.ax - a.ax) * ry) - ((b.ay - a.ay) * rx)) / den;
    if (t > near / lengthA && t < 1.0 - (near / lengthA) &&
        u > near / lengthB && u < 1.0 - (near / lengthB)) {
      return true;
    }
  }
  const auto endsOn = [near](const GateLine& line, const double x,
                             const double y, const double length) {
    const auto [away, at] = awayFrom(line, x, y);
    return away < near && at > near / length && at < 1.0 - (near / length);
  };
  return endsOn(b, a.ax, a.ay, lengthB) || endsOn(b, a.bx, a.by, lengthB) ||
         endsOn(a, b.ax, b.ay, lengthA) || endsOn(a, b.bx, b.by, lengthA);
}

/// No gate of the capacity graph crosses another in the middle of either,
/// and none ends on the middle of another (user, 2026-10-14): a gate beside
/// the chambers on both sides of a line it crosses lets a wire over that
/// line unpaid, and two that run a cell apart cut slivers that are chambers
/// to the flow.
TEST(CouplerSession, LetsNoGateMeetAnotherInTheMiddle) {
  const auto checkChip = [](const Benchmark& benchmark) {
    const auto planned = plan(benchmark);
    CouplerSession session(benchmark.chip, planned.global, planned.assignment,
                           planned.detail, benchmark.config);
    const auto check = stretchCheck(session);
    std::vector<std::pair<std::string, GateLine>> gates;
    for (const auto& edge : check.at("graph").at("edges")) {
      if (edge.at("kind") == "bottleneck") {
        gates.emplace_back(edge.at("name").get<std::string>() + " " +
                               edge.at("ends").dump(),
                           gateLineOf(edge));
      }
    }
    EXPECT_GT(gates.size(), 0U);
    for (std::size_t i = 0; i < gates.size(); ++i) {
      for (std::size_t j = i + 1; j < gates.size(); ++j) {
        EXPECT_FALSE(meetInTheMiddle(gates[i].second, gates[j].second))
            << gates[i].first << " x " << gates[j].first;
      }
    }
  };
  checkChip(fourQubit());
  checkChip(nineQubit());
  checkChip(seventeenQubit());
}

/// Where the tips of two runs at their couplers lie nearly on one line,
/// the two lines of the mouth between them would run over the slots and
/// over each other; one line between the two nearest tips closes that
/// mouth (user, 2026-10-14). So no mouth gate runs within half a clearance
/// of a tip of its own runs other than its ends, and no tip has more than
/// two mouth gates ending on it: the two outer lines of a mouth may leave
/// one tip when the other tip of that run lies between them, and a line
/// cut at the walls it crosses leaves one gate on its tip.
TEST(CouplerSession, ClosesAMouthWhoseLinesWouldRunOverTheSlotsWithOneLine) {
  const auto checkChip = [](const Benchmark& benchmark) {
    const auto planned = plan(benchmark);
    CouplerSession session(benchmark.chip, planned.global, planned.assignment,
                           planned.detail, benchmark.config);
    const auto check = nlohmann::json::parse(session.insertionCheck());
    const auto& graph = check.at("graph");
    const auto clearance = graph.at("clearance").get<double>();
    std::map<std::string, std::pair<double, double>> tips;
    for (const auto& edge : graph.at("edges")) {
      if (edge.value("slot", false)) {
        const auto line = gateLineOf(edge);
        tips[edge.at("ends")[0].get<std::string>()] = {line.ax, line.ay};
        tips[edge.at("ends")[1].get<std::string>()] = {line.bx, line.by};
      }
    }
    EXPECT_GT(tips.size(), 0U);
    std::map<std::string, std::vector<std::string>> endingOn;
    std::size_t mouths = 0;
    for (const auto& edge : graph.at("edges")) {
      if (!edge.value("mouth", false)) {
        continue;
      }
      ++mouths;
      const auto line = gateLineOf(edge);
      for (const auto& [name, tip] : tips) {
        if (std::hypot(tip.first - line.ax, tip.second - line.ay) <= 1.5 ||
            std::hypot(tip.first - line.bx, tip.second - line.by) <= 1.5) {
          endingOn[name].push_back(edge.at("name").get<std::string>() + " " +
                                   edge.at("ends").dump());
        }
      }
      for (const auto& end : edge.at("ends")) {
        const auto name = end.get<std::string>();
        const auto pos = name.find(" at its coupler, ");
        if (pos == std::string::npos) {
          continue;
        }
        const auto run = name.substr(0, pos + 17);
        const auto side = name.substr(pos + 17);
        const auto other = tips.find(run + (side == "left" ? "right" : "left"));
        if (other == tips.end()) {
          continue;
        }
        const auto [x, y] = other->second;
        if (std::hypot(x - line.ax, y - line.ay) <= 1.5 ||
            std::hypot(x - line.bx, y - line.by) <= 1.5) {
          continue;
        }
        EXPECT_GE(awayFrom(line, x, y).first, clearance / 2.0)
            << edge.at("name") << " " << edge.at("ends") << " runs by "
            << other->first;
      }
    }
    EXPECT_GT(mouths, 0U);
    for (const auto& [name, gates] : endingOn) {
      std::string listed;
      for (const auto& gate : gates) {
        listed += gate + "; ";
      }
      EXPECT_LE(gates.size(), 2U) << name << ": " << listed;
    }
  };
  checkChip(nineQubit());
  checkChip(seventeenQubit());
}

/// Every mouth between two chains closes, and the check's summary says so by
/// saying nothing: a mouth left open joins the room inside the chains to the
/// room outside, and every edge there loses its places to cross. On 33q the
/// lead of the next coupler's resonator stands between the two runs, and the
/// mouth line from the one run's tip meets it before the other run's tip
/// (user, 2026-10-15): the line closes the mouth on to that lead, which the
/// pad and the run's flanks carry on to the slot.
TEST(CouplerSession, ClosesEveryMouthThoughALeadStandsBeforeTheTip) {
  for (const auto& chip : {"17q", "33q"}) {
    const auto benchmark = benchmarkOf(chip);
    const auto planned = plan(benchmark);
    CouplerSession session(benchmark.chip, planned.global, planned.assignment,
                           planned.detail, benchmark.config);
    const auto check = nlohmann::json::parse(session.insertionCheck());
    std::size_t mouths = 0;
    for (const auto& edge : check.at("graph").at("edges")) {
      mouths += edge.value("mouth", false) ? 1 : 0;
    }
    EXPECT_GT(mouths, 0U) << chip;
    for (const auto& line : check.at("graph").at("summary")) {
      EXPECT_FALSE(line.get<std::string>().starts_with("no mouth"))
          << chip << ": " << line;
    }
  }
}

/// The entry of the local SAT benchmark: the feedlines routed, every segment
/// of every chain searched as the repair searches it, and every run it
/// reaches judged twice, by the slot check of each feedline alone and by a
/// rip-up and re-route of that feedline's wires alone, with the four classes
/// counted over every feedline judged. The chip ends as the comparison
/// began.
TEST(CouplerSession, ComparesTheSlotCheckWithTheRipUpOfEveryFeedline) {
  const auto benchmark = fourQubit();
  const auto planned = plan(benchmark);
  CouplerSession session(benchmark.chip, planned.global, planned.assignment,
                         planned.detail, benchmark.config);
  const auto before = nlohmann::json::parse(session.couplers());

  const auto compared = nlohmann::json::parse(session.compareLocally(2, 3));

  EXPECT_GT(compared.at("segments").get<int>(), 0);
  const auto& classes = compared.at("classes");
  const auto judged = classes.at("satClean").get<int>() +
                      classes.at("satFailing").get<int>() +
                      classes.at("unsatClean").get<int>() +
                      classes.at("unsatFailing").get<int>();
  EXPECT_GT(judged, 0);
  ASSERT_FALSE(compared.at("runs").empty());
  int feedlines = 0;
  for (const auto& run : compared.at("runs")) {
    for (const auto key :
         {"chain", "lo", "hi", "options", "moves", "slot", "feedlines"}) {
      EXPECT_TRUE(run.contains(key)) << key;
    }
    EXPECT_TRUE(run.at("slot").contains("verdict"));
    for (const auto& move : run.at("moves")) {
      EXPECT_TRUE(move.contains("coupler") && move.contains("option"));
    }
    for (const auto& feedline : run.at("feedlines")) {
      EXPECT_TRUE(feedline.contains("sat") && feedline.contains("clean"));
      ++feedlines;
    }
  }
  EXPECT_EQ(judged, feedlines);
  EXPECT_EQ(nlohmann::json::parse(session.couplers()).at("couplers"),
            before.at("couplers"));
}

/// The chain search checks its steps against the chain alone, not against
/// the chains settled before it (user, 2026-10-14), and says so.
TEST(CouplerSession, SearchesEveryChainAgainstItselfAlone) {
  const auto benchmark = fourQubit();
  const auto planned = plan(benchmark);
  std::vector<std::string> lines;
  CouplerSession session(benchmark.chip, planned.global, planned.assignment,
                         planned.detail, benchmark.config, linesInto(lines));
  EXPECT_TRUE(std::ranges::any_of(lines, [](const std::string& line) {
    return line.find("each checked against itself alone") != std::string::npos;
  }));
}

/// Two feedline edges of one chain never cross after the feedline routing,
/// at the coupler they share either: not on a cell, and not as two diagonal
/// steps through one unit square.
TEST(CouplerSession, KeepsTheEdgesOfAChainApartInTheFeedlineRouting) {
  const auto benchmark = fourQubit();
  const auto planned = plan(benchmark);
  CouplerSession session(benchmark.chip, planned.global, planned.assignment,
                         planned.detail, benchmark.config);
  const auto listed = nlohmann::json::parse(session.couplers()).at("couplers");
  std::map<std::string, std::uint32_t> chainOf;
  for (const auto& coupler : listed) {
    for (const auto* const side : {"edgeIn", "edgeOut"}) {
      if (!coupler.at(side).is_null()) {
        chainOf[coupler.at(side).get<std::string>()] =
            coupler.at("chain").get<std::uint32_t>();
      }
    }
  }

  const auto routed = nlohmann::json::parse(session.routeFeedlines());
  std::map<std::string, Covered> edges;
  for (const auto& wire : routed.at("wires")) {
    const auto name = wire.at("name").get<std::string>();
    if (wire.at("kind") == "feedline" && chainOf.contains(name)) {
      edges[name] = coverOf(wire.at("path"));
    }
  }
  ASSERT_FALSE(edges.empty());
  for (const auto& [one, a] : edges) {
    for (const auto& [two, b] : edges) {
      if (one >= two || chainOf[one] != chainOf[two]) {
        continue;
      }
      EXPECT_FALSE(a.crosses(b)) << one << " and " << two;
    }
  }
}

/// An edge that meets a terminal edge of its own chain at a coupler keeps the
/// wire clearance from the run the terminal edge makes at that coupler: the
/// run follows from the coupler's option, so the edge is drawn round it even
/// before the terminal edge is. On 9q every
/// open option of every coupler next to a launcher is drawn, and the run is
/// measured against the other edge outside that edge's own slot.
TEST(CouplerSession, KeepsAnEdgeClearOfTheRunOfItsTerminalNeighbour) {
  const auto benchmark = nineQubit();
  const auto planned = plan(benchmark);
  CouplerSession session(benchmark.chip, planned.global, planned.assignment,
                         planned.detail, benchmark.config);
  const auto clearance =
      nlohmann::json::parse(session.graph()).at("clearance").get<double>();
  const auto listed = nlohmann::json::parse(session.couplers()).at("couplers");
  using Cell = std::array<double, 2>;
  const auto apart = [](const Cell& a, const Cell& b) {
    return std::hypot(a[0] - b[0], a[1] - b[1]);
  };
  std::map<std::uint32_t, std::uint32_t> lastAt;
  for (const auto& coupler : listed) {
    auto& last = lastAt[coupler.at("chain").get<std::uint32_t>()];
    last = std::max(last, coupler.at("at").get<std::uint32_t>());
  }
  // The edge's own slot at the coupler opens again after the guard: the
  // run of `terminal_slot_cells` cells, or the pad's run and a bend.
  constexpr std::size_t SLOT = 30;
  std::size_t measured = 0;
  for (const auto& coupler : listed) {
    if (coupler.at("edgeIn").is_null() || coupler.at("edgeOut").is_null()) {
      continue;
    }
    const auto at = coupler.at("at").get<std::uint32_t>();
    const bool terminalIn = at == 1;
    const bool terminalOut =
        at == lastAt[coupler.at("chain").get<std::uint32_t>()];
    if (terminalIn == terminalOut) {
      continue;
    }
    const auto terminal = coupler.at(terminalIn ? "edgeIn" : "edgeOut");
    const auto index = coupler.at("index").get<std::uint32_t>();
    for (const auto& option : coupler.at("options")) {
      if (!option.at("open").get<bool>()) {
        continue;
      }
      const auto moved = nlohmann::json::parse(
          session.setOption(index, option.at("index").get<std::uint32_t>()));
      std::vector<Cell> run;
      std::vector<Cell> other;
      for (const auto& edge : moved.at("edges")) {
        if (!edge.at("drawn").get<bool>()) {
          continue;
        }
        std::vector<Cell> way;
        for (const auto& cell : edge.at("way")) {
          way.push_back({cell.at(0).get<double>(), cell.at(1).get<double>()});
        }
        // Both ways from the coupler on.
        const bool endsHere =
            edge.at("edge") == terminal ? terminalIn : !terminalIn;
        if (endsHere) {
          std::ranges::reverse(way);
        }
        if (edge.at("edge") == terminal) {
          const auto port = way.front();
          const auto length = apart({option.at("in").at(0).get<double>(),
                                     option.at("in").at(1).get<double>()},
                                    {option.at("out").at(0).get<double>(),
                                     option.at("out").at(1).get<double>()});
          for (const auto& cell : way) {
            if (apart(cell, port) > length) {
              break;
            }
            run.push_back(cell);
          }
        } else if (way.size() > SLOT) {
          other.assign(way.begin() + SLOT, way.end());
        }
      }
      if (run.empty() || other.empty()) {
        continue;
      }
      ++measured;
      for (const auto& a : run) {
        for (const auto& b : other) {
          ASSERT_GE(apart(a, b), clearance - 1.0)
              << "coupler " << index << " option " << option.at("index")
              << ": (" << b[0] << ", " << b[1] << ") of the edge beside "
              << terminal << " is " << apart(a, b) << " from (" << a[0] << ", "
              << a[1] << ") of its run";
        }
      }
    }
  }
  EXPECT_GT(measured, 0U);

  // The case that showed it, from the comparison of the local checks:
  // coupler '1' on option 0 and coupler '2' on option 31. f8 ran past the
  // port of f9, and f9 then found no way out of its run.
  std::map<std::string, std::uint32_t> indexOf;
  for (const auto& coupler : listed) {
    indexOf[coupler.at("resonator").get<std::string>()] =
        coupler.at("index").get<std::uint32_t>();
  }
  static_cast<void>(session.setOption(indexOf.at("1"), 0));
  const auto moved =
      nlohmann::json::parse(session.setOption(indexOf.at("2"), 31));
  for (const auto& edge : moved.at("edges")) {
    EXPECT_TRUE(edge.at("drawn").get<bool>()) << edge.at("edge");
  }
}

/// Two feedline edges that meet at a coupler keep the wire clearance from
/// each other outside the meeting the feedline routing grants them: two
/// cells closer than the clearance have their middle within half the
/// coupling length and a clearance of the coupler's anchor. The feedline
/// routing does not draw an edge between two couplers again, so an edge the
/// insertion lets closer stays open. On 4q every open option of every
/// coupler is drawn, and the two edges at that coupler are measured.
/// the slots.
TEST(CouplerSession, KeepsTheEdgesAtACouplerApartOutsideTheirMeeting) {
  const auto benchmark = fourQubit();
  const auto planned = plan(benchmark);
  CouplerSession session(benchmark.chip, planned.global, planned.assignment,
                         planned.detail, benchmark.config);
  const auto clearance =
      nlohmann::json::parse(session.graph()).at("clearance").get<double>();
  const auto listed = nlohmann::json::parse(session.couplers()).at("couplers");
  using Cell = std::array<double, 2>;
  const auto cellOf = [](const nlohmann::json& at) {
    return Cell{at.at(0).get<double>(), at.at(1).get<double>()};
  };
  const auto apart = [](const Cell& a, const Cell& b) {
    return std::hypot(a[0] - b[0], a[1] - b[1]);
  };
  std::size_t measured = 0;
  for (const auto& coupler : listed) {
    if (coupler.at("edgeIn").is_null() || coupler.at("edgeOut").is_null()) {
      continue;
    }
    const auto index = coupler.at("index").get<std::uint32_t>();
    for (const auto& option : coupler.at("options")) {
      if (!option.at("open").get<bool>()) {
        continue;
      }
      const auto moved = nlohmann::json::parse(
          session.setOption(index, option.at("index").get<std::uint32_t>()));
      std::vector<std::vector<Cell>> ways;
      for (const auto& edge : moved.at("edges")) {
        if (edge.at("drawn").get<bool>()) {
          std::vector<Cell> way;
          for (const auto& at : edge.at("way")) {
            way.push_back(cellOf(at));
          }
          ways.push_back(std::move(way));
        }
      }
      if (ways.size() != 2) {
        continue;
      }
      ++measured;
      const auto anchor = cellOf(option.at("anchor"));
      const auto in = cellOf(option.at("in"));
      const auto out = cellOf(option.at("out"));
      // One cell of slack on both figures: the rule is a little under the
      // clearance in cells, and the pad's ports stand about the coupling
      // length apart.
      const auto reach = (0.5 * apart(in, out)) + clearance + 1.0;
      const bool portsMeet = apart(in, out) <= clearance;
      for (const auto& a : ways[0]) {
        for (const auto& b : ways[1]) {
          if (apart(a, b) >= clearance - 1.0) {
            continue;
          }
          const Cell middle{0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1])};
          const bool meets =
              apart(middle, anchor) <= reach ||
              (portsMeet && (apart(middle, in) <= (1.5 * clearance) + 1.0 ||
                             apart(middle, out) <= (1.5 * clearance) + 1.0));
          EXPECT_TRUE(meets)
              << "coupler " << index << " option " << option.at("index")
              << ": (" << a[0] << ", " << a[1] << ") and (" << b[0] << ", "
              << b[1] << ") are " << apart(a, b) << " apart";
          if (!meets) {
            return;
          }
        }
      }
    }
  }
  EXPECT_GT(measured, 0U);
}

/// The slot check judges the capacity graph in memory (`slotGraphOf`); its
/// JSON, which the page shows, must say the same. On 4q every coupler is
/// moved to each of its first open options and the two are compared.
TEST(CouplerSession, JudgesTheGraphInMemoryAsItsJson) {
  const auto benchmark = fourQubit();
  const auto planned = plan(benchmark);
  CouplerSession session(benchmark.chip, planned.global, planned.assignment,
                         planned.detail, benchmark.config);
  const auto listed = nlohmann::json::parse(session.couplers()).at("couplers");
  std::size_t compared = 0;
  for (const auto& coupler : listed) {
    std::size_t tried = 0;
    for (const auto& option : coupler.at("options")) {
      if (!option.at("open").get<bool>() || ++tried > 3) {
        continue;
      }
      static_cast<void>(
          session.setOption(coupler.at("index").get<std::uint32_t>(),
                            option.at("index").get<std::uint32_t>()));
      const auto check = nlohmann::json::parse(session.insertionCheck());
      const auto& page = check.at("graph");
      EXPECT_EQ(check.at("verdict"), page.at("verdict"));
      ++compared;
    }
  }
  EXPECT_GT(compared, 0U);
}

/// `coupler_shift_cells` (20) moves an option that leaves the coupler box
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
/// With the move every feedline edge is drawn, and the edges between two
/// couplers turn four eighths in all. The terminal edges are left out:
/// how much an edge from a launcher turns depends on where the coupler sits
/// against the launcher, and the couplers sit where the resonators' meanders
/// keep their length.
TEST(FinalRouter, TheFourQubitChainTurnsLittleBetweenItsCouplers) {
  const auto benchmark = fourQubit();
  const auto planned = plan(benchmark);
  long undrawn = -1;
  std::map<long, long> inner;
  const std::regex said(
      R"(==> coupler insertion: (\d+) feedline edges NOT drawn)");
  const std::regex edge(
      R"(coupler insertion: edge f(\d+) chain \d+ \(\d+->\d+(, terminal)?\): \d+ cells, angle (\d+))");
  static_cast<void>(finalRouters().make("dubins")->run(
      benchmark.chip, planned.capacity, planned.global, planned.assignment,
      planned.detail, benchmark.config,
      hearing([&](const std::string_view line) {
        std::match_results<std::string_view::const_iterator> match;
        if (std::regex_search(line.begin(), line.end(), match, said)) {
          undrawn = std::stol(match[1].str());
        }
        if (std::regex_search(line.begin(), line.end(), match, edge) &&
            !match[2].matched) {
          inner[std::stol(match[1].str())] = std::stol(match[3].str());
        }
      })));

  EXPECT_EQ(undrawn, 0);
  ASSERT_FALSE(inner.empty()) << "no edge between two couplers was drawn";
  long turns = 0;
  for (const auto& [slot, angle] : inner) {
    turns += angle;
  }
  EXPECT_EQ(turns, 4);
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
      planned.detail, benchmark.config,
      hearing([&](const std::string_view line) {
        if (line.find("CHECK chain crossings") != std::string_view::npos) {
          checks.emplace_back(line);
        }
      })));

  ASSERT_GE(checks.size(), 2U);
  for (const auto& line : checks) {
    EXPECT_NE(line.find("0 pairs of edges of one chain cross"),
              std::string::npos)
        << line;
  }
}

/// The records a run hands its debug sink, each as its kind and its data.
using Records = std::vector<std::pair<std::string, nlohmann::json>>;

/// Runs the Final stage of `benchmark` with a debug sink and returns every
/// record it got, in order.
Records recordsOf(const Benchmark& benchmark) {
  const auto planned = plan(benchmark);
  Records records;
  static_cast<void>(finalRouters().make("dubins")->run(
      benchmark.chip, planned.capacity, planned.global, planned.assignment,
      planned.detail, benchmark.config, {},
      [&records](const std::string_view kind, const std::string_view record) {
        records.emplace_back(kind, nlohmann::json::parse(record));
      }));
  return records;
}

/// The records of one kind, in order.
std::vector<nlohmann::json> ofKind(const Records& records,
                                   const std::string_view kind) {
  std::vector<nlohmann::json> found;
  for (const auto& [name, data] : records) {
    if (name == kind) {
      found.push_back(data);
    }
  }
  return found;
}

/// A dashboard asks for the grid once, first, and then for one record per
/// search of its phase: the pass, the round, the wire, the kind of search,
/// what it found and the way as its bends.
TEST(FinalRouter, RecordsItsGridAndEverySearchOfAPhaseForItsDashboard) {
  const auto benchmark = withDebugPages(fourQubit(), [](auto& debug) {
    debug.pre_feedline_routing_dashboard = true;
  });
  const auto records = recordsOf(benchmark);

  ASSERT_FALSE(records.empty());
  EXPECT_EQ(records.front().first, "grid");
  const auto& grid = records.front().second;
  EXPECT_GT(grid.at("width").get<std::size_t>(), 0U);
  EXPECT_GT(grid.at("height").get<std::size_t>(), 0U);
  EXPECT_FALSE(grid.at("blocked").empty());
  EXPECT_EQ(ofKind(records, "grid").size(), 1U);

  const auto searches = ofKind(records, "search");
  // Every ring wire of the 4-qubit chip is searched at least once before
  // the feedlines; the inner circuit too.
  EXPECT_GE(searches.size(), 12U);
  for (const auto& search : searches) {
    EXPECT_EQ(search.at("phase"), "pre-feedline-routing");
    EXPECT_TRUE(search.contains("pass"));
    EXPECT_TRUE(search.contains("round"));
    EXPECT_TRUE(search.contains("wire"));
    EXPECT_TRUE(search.contains("kind"));
    EXPECT_TRUE(search.at("found").is_boolean());
    EXPECT_TRUE(search.at("way").is_array());
  }
  // Only the phase asked for is recorded: no search of the feedline
  // routing, and nothing of the other pages.
  for (const auto& [kind, data] : records) {
    EXPECT_TRUE(kind == "grid" || kind == "search") << kind;
  }
}

/// A sink that no page asks for gets nothing: the `[debug]` section decides
/// what is recorded, not the presence of the sink.
TEST(FinalRouter, RecordsNothingWhenNoDebugPageAsksForIt) {
  EXPECT_TRUE(recordsOf(fourQubit()).empty());
  EXPECT_TRUE(
      recordsOf(withDebugPages(fourQubit(), [](auto& /*debug*/) {})).empty());
}

/// The insertion page asks for the coupler options once, before the chain
/// search, and for the capacity graph of the whole-chip check after the
/// commit: the same graph as data the coupler session answers.
TEST(FinalRouter,
     RecordsTheCouplerOptionsAndTheCapacityGraphForTheInsertionPage) {
  const auto records = recordsOf(withDebugPages(
      fourQubit(), [](auto& debug) { debug.feedline_insertion = true; }));

  const auto options = ofKind(records, "coupler-options");
  ASSERT_EQ(options.size(), 1U);
  const auto& couplers = options.front().at("couplers");
  EXPECT_EQ(couplers.size(), 4U);
  EXPECT_TRUE(couplers.front().at("options").front().contains("pad"));

  const auto graphs = ofKind(records, "capacity-graph");
  ASSERT_FALSE(graphs.empty());
  const auto first = std::ranges::find_if(records, [](const auto& record) {
    return record.first == "capacity-graph";
  });
  const auto before = std::ranges::find_if(records, [](const auto& record) {
    return record.first == "coupler-options";
  });
  EXPECT_LT(before - records.begin(), first - records.begin());
  const auto& data = graphs.back();
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
  EXPECT_TRUE(data.contains("verdict"));
}

/// The local SAT page asks for every step check of the chain search: the
/// chain, the step, the options the step stands on, the verdict, the wires
/// without a way and the ones this edge closed.
TEST(FinalRouter, RecordsEveryStepCheckForTheLocalSatPage) {
  const auto records = recordsOf(
      withDebugPages(fourQubit(), [](auto& debug) { debug.local_sat = true; }));

  const auto steps = ofKind(records, "step-check");
  ASSERT_FALSE(steps.empty());
  for (const auto& step : steps) {
    EXPECT_TRUE(step.at("chain").is_number_unsigned());
    EXPECT_TRUE(step.at("step").is_number_unsigned());
    EXPECT_TRUE(step.at("options").is_array());
    const auto verdict = step.at("verdict").get<std::string>();
    EXPECT_TRUE(verdict == "SAT" || verdict == "UNSAT") << verdict;
    EXPECT_EQ(verdict == "SAT", step.at("lost").empty());
    EXPECT_TRUE(step.at("closed").is_array());
    EXPECT_TRUE(step.at("refused").is_boolean());
    // The coupler of every entry of `options`; a fixed waypoint has none
    // and stays on option 0.
    const auto& couplers = step.at("couplers");
    ASSERT_EQ(couplers.size(), step.at("options").size());
    for (std::size_t at = 0; at < couplers.size(); ++at) {
      if (couplers[at].is_null()) {
        EXPECT_EQ(step.at("options")[at].get<std::size_t>(), 0U);
      } else {
        EXPECT_TRUE(couplers[at].is_number_unsigned());
      }
    }
  }
  for (const auto& [kind, data] : records) {
    EXPECT_EQ(kind, "step-check");
  }
}

/// The global SAT page asks for the search's course: what it set out from,
/// the states it collected per chain, every combination with its verdict,
/// and the result. A chip that is SAT after the commit records the result
/// alone, and says why nothing was searched.
TEST(FinalRouter, RecordsTheGlobalSatSearchForItsPage) {
  const auto records = recordsOf(withDebugPages(
      fourQubit(), [](auto& debug) { debug.global_sat = true; }));

  const auto events = ofKind(records, "global-sat");
  ASSERT_FALSE(events.empty());
  EXPECT_EQ(events.back().at("event"), "result");
  EXPECT_FALSE(events.back().at("searched").get<bool>());
  EXPECT_FALSE(events.back().at("why").get<std::string>().empty());
  for (const auto& [kind, data] : records) {
    EXPECT_EQ(kind, "global-sat");
  }
}

/// The records a coupler session hands its debug sink while it runs the
/// Final stage up to and with the coupler insertion.
Records sessionRecordsOf(const Benchmark& benchmark, const Planned& planned,
                         std::unique_ptr<CouplerSession>& session) {
  Records records;
  session = std::make_unique<CouplerSession>(
      benchmark.chip, planned.global, planned.assignment, planned.detail,
      benchmark.config, Report{},
      [&records](const std::string_view kind, const std::string_view record) {
        records.emplace_back(kind, nlohmann::json::parse(record));
      });
  return records;
}

/// The global SAT page asks for every chain state the search checks
/// locally: the committed state of each chain it searches, first, and every
/// state a segment's runs reach. A record names the chain, the segment, the
/// options per waypoint, the coupler per waypoint, the couplers it moves off
/// the committed state, the check that found it SAT and the chain's own
/// lost wires. The 17-qubit chip is UNSAT after the commit, so its search
/// runs; the 4- and 9-qubit chips are SAT there. A local check of the
/// coupler session on the committed options judges as the search did, and
/// puts the chip back as it stood.
TEST(CouplerSession, RecordsEveryChainStateTheGlobalSatSearchChecks) {
  const auto benchmark = withDebugPages(
      seventeenQubit(), [](auto& debug) { debug.global_sat = true; });
  const auto planned = plan(benchmark);
  std::unique_ptr<CouplerSession> session;
  const auto records = sessionRecordsOf(benchmark, planned, session);

  const auto states = ofKind(records, "chain-state");
  ASSERT_FALSE(states.empty());
  const auto events = ofKind(records, "global-sat");
  ASSERT_FALSE(events.empty());
  EXPECT_TRUE(events.back().at("searched").get<bool>());
  std::map<std::size_t, std::size_t> kept;
  for (const auto& event : events) {
    if (event.at("event") == "states") {
      EXPECT_EQ(event.at("options").size(),
                event.at("states").get<std::size_t>());
      kept[event.at("chain").get<std::size_t>()] =
          event.at("states").get<std::size_t>();
    }
  }
  // Every state a combination picks has its options on record, in stage 2
  // as in stage 1.
  for (const auto& event : events) {
    if (event.at("event") != "combination") {
      continue;
    }
    const auto picks = event.at("states").get<std::vector<std::size_t>>();
    for (std::size_t chain = 0; chain < picks.size(); ++chain) {
      if (picks[chain] != 0) {
        ASSERT_TRUE(kept.contains(chain)) << "chain " << chain;
        EXPECT_LT(picks[chain], kept[chain]) << "chain " << chain;
      }
    }
  }
  std::map<std::uint32_t, nlohmann::json> committed;
  for (const auto& state : states) {
    const auto chain = state.at("chain").get<std::uint32_t>();
    const auto& options = state.at("options");
    const auto& couplers = state.at("couplers");
    ASSERT_EQ(couplers.size(), options.size());
    const auto check = state.at("check").get<std::string>();
    EXPECT_TRUE(check == "standing" || check == "alone" || check == "none")
        << check;
    EXPECT_EQ(state.at("verdict") == "SAT", check != "none");
    EXPECT_EQ(check == "standing", state.at("lost").empty());
    EXPECT_GE(state.at("seconds").get<double>(), 0.0);
    if (state.at("committed").get<bool>()) {
      // The committed state comes first and moves nothing.
      EXPECT_FALSE(committed.contains(chain));
      EXPECT_TRUE(state.at("segment").is_null());
      EXPECT_TRUE(state.at("moved").empty());
      committed[chain] = state;
      continue;
    }
    ASSERT_TRUE(committed.contains(chain));
    EXPECT_TRUE(state.at("segment").is_number_unsigned());
    // `moved` names exactly the couplers whose option differs from the
    // committed state.
    const auto& from = committed[chain].at("options");
    std::set<std::uint32_t> differ;
    for (std::size_t at = 0; at < options.size(); ++at) {
      if (options[at] != from[at]) {
        ASSERT_FALSE(couplers[at].is_null());
        differ.insert(couplers[at].get<std::uint32_t>());
      }
    }
    std::set<std::uint32_t> named;
    for (const auto& move : state.at("moved")) {
      named.insert(move.at("coupler").get<std::uint32_t>());
      EXPECT_NE(move.at("from"), move.at("to"));
    }
    EXPECT_EQ(named, differ);
  }
  ASSERT_FALSE(committed.empty());

  const auto before = session->chains();
  for (const auto& [chain, state] : committed) {
    const auto options = state.at("options").get<std::vector<std::size_t>>();
    const auto local =
        nlohmann::json::parse(session->localCheck(chain, options));
    EXPECT_EQ(local.at("chain").get<std::uint32_t>(), chain);
    const auto check = state.at("check").get<std::string>();
    EXPECT_EQ(local.at("standing").at("verdict") == "SAT", check == "standing")
        << "chain " << chain;
    EXPECT_EQ(local.at("standing").at("lost"), state.at("lost"));
    if (check == "alone") {
      EXPECT_EQ(local.at("alone").at("verdict"), "SAT") << "chain " << chain;
    }
    for (const auto* const which : {"standing", "alone"}) {
      for (const auto* const key : {"raster", "nodes", "edges", "wires"}) {
        EXPECT_TRUE(local.at(which).at("graph").contains(key)) << key;
      }
    }
  }
  EXPECT_EQ(session->chains(), before);
}

/// The search records no chain state when the global SAT page is off,
/// though it runs.
TEST(CouplerSession, RecordsNoChainStateWithoutTheGlobalSatPage) {
  const auto benchmark = withDebugPages(
      seventeenQubit(), [](auto& debug) { debug.feedline_insertion = true; });
  const auto planned = plan(benchmark);
  std::unique_ptr<CouplerSession> session;
  const auto records = sessionRecordsOf(benchmark, planned, session);

  EXPECT_FALSE(ofKind(records, "coupler-options").empty());
  EXPECT_TRUE(ofKind(records, "chain-state").empty());
  EXPECT_TRUE(ofKind(records, "global-sat").empty());
}

/// A coupler session lists every chain: the coupler of every waypoint, the
/// option it stands on and the chain's feedline edges. A local check puts
/// the chain's couplers on other options for the checks only, and refuses a
/// chain, an option count or an option that does not exist.
TEST(CouplerSession, ListsTheChainsAndChecksOneLocally) {
  const auto benchmark = fourQubit();
  const auto planned = plan(benchmark);
  CouplerSession session(benchmark.chip, planned.global, planned.assignment,
                         planned.detail, benchmark.config);

  const auto chains = nlohmann::json::parse(session.chains()).at("chains");
  ASSERT_EQ(chains.size(), 1U);
  const auto& chain = chains.front();
  EXPECT_EQ(chain.at("chain").get<std::uint32_t>(), 0U);
  const auto& couplers = chain.at("couplers");
  const auto options = chain.at("options").get<std::vector<std::size_t>>();
  ASSERT_EQ(couplers.size(), options.size());
  EXPECT_EQ(chain.at("edges").size(), options.size() - 1);
  const auto listed = nlohmann::json::parse(session.couplers()).at("couplers");
  std::size_t placed = 0;
  for (std::size_t at = 0; at < options.size(); ++at) {
    if (couplers[at].is_null()) {
      EXPECT_EQ(options[at], 0U);
      continue;
    }
    ++placed;
    for (const auto& coupler : listed) {
      if (coupler.at("index") == couplers[at]) {
        EXPECT_EQ(coupler.at("chosen").get<std::size_t>(), options[at]);
        EXPECT_EQ(coupler.at("at").get<std::size_t>(), at);
      }
    }
  }
  EXPECT_EQ(placed, listed.size());

  // Another option for the first coupler that has one.
  auto moved = options;
  for (const auto& coupler : listed) {
    if (coupler.at("options").size() > 1) {
      const auto at = coupler.at("at").get<std::size_t>();
      moved[at] = options[at] == 0 ? 1 : 0;
      break;
    }
  }
  ASSERT_NE(moved, options);
  const auto before = session.chains();
  const auto local = nlohmann::json::parse(session.localCheck(0, moved));
  EXPECT_EQ(local.at("options").get<std::vector<std::size_t>>(), moved);
  EXPECT_TRUE(local.at("undrawn").is_array());
  for (const auto* const which : {"standing", "alone"}) {
    const auto verdict = local.at(which).at("verdict").get<std::string>();
    EXPECT_TRUE(verdict == "SAT" || verdict == "UNSAT") << verdict;
    EXPECT_TRUE(local.at(which).at("lost").is_array());
    EXPECT_TRUE(local.at(which).at("graph").contains("wires")) << which;
  }
  EXPECT_EQ(local.at("alone").at("verdict") == "SAT",
            local.at("alone").at("lost").empty());
  EXPECT_EQ(session.chains(), before);

  EXPECT_THROW(static_cast<void>(session.localCheck(1, options)),
               std::invalid_argument);
  auto shorter = options;
  shorter.pop_back();
  EXPECT_THROW(static_cast<void>(session.localCheck(0, shorter)),
               std::invalid_argument);
  auto beyond = options;
  for (std::size_t at = 0; at < options.size(); ++at) {
    if (!couplers[at].is_null()) {
      beyond[at] = 1000;
      break;
    }
  }
  EXPECT_THROW(static_cast<void>(session.localCheck(0, beyond)),
               std::invalid_argument);
}

/// A step check of the coupler session rebuilds a step check of the chain
/// search: on the options of every recorded step it judges the chip the
/// step judged, the chain alone up to the far end of the step's edge. The
/// session draws the edges again against the chip as it stands, so its
/// ways may differ from the search's own; on the steps of the committed
/// options of the 9-qubit chip they give the recorded verdict. The step
/// check refuses a chain, a step or options that do not exist, and puts the
/// chip back as it stood.
TEST(CouplerSession, RebuildsTheStepChecksOfTheChainSearch) {
  const auto benchmark =
      withDebugPages(nineQubit(), [](auto& debug) { debug.local_sat = true; });
  const auto planned = plan(benchmark);
  std::unique_ptr<CouplerSession> session;
  const auto records = sessionRecordsOf(benchmark, planned, session);
  const auto steps = ofKind(records, "step-check");
  ASSERT_FALSE(steps.empty());

  const auto before = session->chains();
  const auto chains = nlohmann::json::parse(before).at("chains");
  std::size_t onCommit = 0;
  std::size_t agreeing = 0;
  for (const auto& recorded : steps) {
    const auto chain = recorded.at("chain").get<std::uint32_t>();
    const auto step = recorded.at("step").get<std::size_t>();
    const auto options = recorded.at("options").get<std::vector<std::size_t>>();
    const auto rebuilt =
        nlohmann::json::parse(session->stepCheck(chain, step, options));
    EXPECT_EQ(rebuilt.at("chain"), recorded.at("chain"));
    EXPECT_EQ(rebuilt.at("step"), recorded.at("step"));
    EXPECT_EQ(rebuilt.at("edge"), recorded.at("edge"));
    EXPECT_EQ(rebuilt.at("options"), recorded.at("options"));
    EXPECT_EQ(rebuilt.at("verdict") == "SAT", rebuilt.at("lost").empty());
    EXPECT_TRUE(rebuilt.at("closed").is_array());
    EXPECT_TRUE(rebuilt.at("undrawn").is_array());
    EXPECT_TRUE(rebuilt.at("graph").contains("wires"));
    agreeing += rebuilt.at("verdict") == recorded.at("verdict") ? 1U : 0U;
    // A step on the committed options: every waypoint up to the far end of
    // its edge stands where the chain settled.
    const auto committed =
        chains.at(chain).at("options").get<std::vector<std::size_t>>();
    if (std::equal(options.begin(),
                   options.begin() + static_cast<std::ptrdiff_t>(step + 2),
                   committed.begin())) {
      ++onCommit;
      EXPECT_EQ(rebuilt.at("verdict"), recorded.at("verdict"))
          << "chain " << chain << " step " << step;
      EXPECT_EQ(rebuilt.at("lost"), recorded.at("lost"))
          << "chain " << chain << " step " << step;
    }
  }
  EXPECT_GT(onCommit, 0U);
  // Most steps off the committed options agree as well.
  EXPECT_GE(agreeing * 10, steps.size() * 8)
      << agreeing << " of " << steps.size() << " step checks agree";
  std::printf("%zu of %zu step checks agree with the chain search\n", agreeing,
              steps.size());
  EXPECT_EQ(session->chains(), before);

  const auto& first = steps.front();
  const auto options = first.at("options").get<std::vector<std::size_t>>();
  const auto chain = first.at("chain").get<std::uint32_t>();
  const auto edges = chains.at(chain).at("edges").size();
  EXPECT_THROW(static_cast<void>(session->stepCheck(
                   static_cast<std::uint32_t>(chains.size()), 0, options)),
               std::invalid_argument);
  EXPECT_THROW(static_cast<void>(session->stepCheck(chain, edges, options)),
               std::invalid_argument);
  auto shorter = options;
  shorter.pop_back();
  EXPECT_THROW(static_cast<void>(session->stepCheck(chain, 0, shorter)),
               std::invalid_argument);
  auto beyond = options;
  const auto& couplers = chains.at(chain).at("couplers");
  for (std::size_t at = 0; at < couplers.size(); ++at) {
    if (!couplers[at].is_null()) {
      beyond[at] = 1000;
      break;
    }
  }
  EXPECT_THROW(static_cast<void>(session->stepCheck(chain, 0, beyond)),
               std::invalid_argument);
  EXPECT_EQ(session->chains(), before);
}

/// The index asks for what every phase came to: one record per phase, in
/// the order the phases run, with the wires and the fails counted.
TEST(FinalRouter, RecordsWhatEveryPhaseCameToForTheIndex) {
  const auto records = recordsOf(
      withDebugPages(fourQubit(), [](auto& debug) { debug.html = true; }));

  const auto phases = ofKind(records, "phase");
  ASSERT_EQ(phases.size(), 5U);
  const std::array<std::string, 5> names{
      "pre-feedline-routing", "pre-feedline-refinement", "feedline-insertion",
      "feedline-constrained-routing", "feedline-constrained-refinement"};
  for (std::size_t at = 0; at < names.size(); ++at) {
    EXPECT_EQ(phases[at].at("phase"), names[at]);
    EXPECT_GT(phases[at].at("wires").get<std::size_t>(), 0U);
    EXPECT_TRUE(phases[at].at("drawn").is_number_unsigned());
    EXPECT_TRUE(phases[at].at("unrouted").is_number_unsigned());
  }
  for (const auto& [kind, data] : records) {
    EXPECT_EQ(kind, "phase");
  }
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
      planned.detail, benchmark.config, linesInto(lines));

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

/// `edge_launcher_margin_cells` lengthens the run a feedline edge keeps
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
  const auto runWith = [&](const std::uint32_t margin) {
    Seen seen;
    auto chip = withDebugPages(
        benchmark, [](auto& debug) { debug.feedline_insertion = true; });
    finalParamsOf(chip).edge_launcher_margin_cells = margin;
    static_cast<void>(finalRouters().make("dubins")->run(
        chip.chip, planned.capacity, planned.global, planned.assignment,
        planned.detail, chip.config,
        hearing([&seen](const std::string_view line) {
          const auto at = line.find("a feedline edge keeps ");
          if (at != std::string_view::npos) {
            seen.run = std::stol(std::string(line.substr(at + 22)));
          }
        }),
        [&seen](const std::string_view kind, const std::string_view record) {
          if (kind == "capacity-graph") {
            seen.graph = nlohmann::json::parse(record);
          }
        }));
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

  const auto plain = runWith(0);
  const auto kept = runWith(30);
  EXPECT_EQ(plain.run, -1) << "no margin, no line";
  ASSERT_GT(kept.run, 30);
  ASSERT_FALSE(kept.graph.is_null());
  ASSERT_FALSE(plain.graph.is_null());
  const auto clearance = kept.graph.at("clearance").get<double>();
  EXPECT_GE(closest(kept.graph, kept.run), clearance);
  EXPECT_LT(closest(plain.graph, kept.run), clearance);
}

/// `coupler_box_margin_cells` keeps every coupler further in from the sides of
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
  const auto runWith = [&](const std::uint32_t margin) {
    Seen seen;
    auto chip = withDebugPages(
        benchmark, [](auto& debug) { debug.feedline_insertion = true; });
    finalParamsOf(chip).coupler_box_margin_cells = margin;
    static_cast<void>(finalRouters().make("dubins")->run(
        chip.chip, planned.capacity, planned.global, planned.assignment,
        planned.detail, chip.config,
        hearing([&seen](const std::string_view line) {
          if (line.find("couplers sit inside x ") != std::string_view::npos) {
            seen.box = std::string(line.substr(line.find("couplers sit")));
          }
          seen.said = seen.said || line.find("coupler_box_margin_cells") !=
                                       std::string_view::npos;
        }),
        [&seen](const std::string_view kind, const std::string_view record) {
          if (kind == "capacity-graph") {
            seen.graph = nlohmann::json::parse(record);
          }
        }));
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

  const auto plain = runWith(0);
  const auto kept = runWith(20);
  ASSERT_FALSE(plain.graph.is_null());
  ASSERT_FALSE(kept.graph.is_null());
  EXPECT_FALSE(plain.said);
  EXPECT_TRUE(kept.said);
  EXPECT_GE(closest(kept), 20);
  EXPECT_LT(closest(plain), 20);
}

} // namespace
} // namespace mqt::scpd::pipeline
