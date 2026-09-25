/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// The rules that read a view in router cells, on wires built by hand, so that
// what each rule finds and what it forgives is stated rather than inferred
// from a routed chip.

#include "mqt-scpd/drc/Rules.hpp"
#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/flatbuffers/drc.hpp"
#include "mqt-scpd/flatbuffers/geometry.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace mqt::scpd::drc {
namespace {

using flatbuffers::design::DesignRulesT;
using flatbuffers::drc::DrcRule;
using flatbuffers::geometry::RCoord;

/// A grid of ten layout units per cell, as the router grid of every benchmark
/// chip is. The wire spacing of 185 then spans nineteen cells.
grid::GridMetrics tenUnitGrid() {
  return {.width = 400,
          .height = 400,
          .origin = {0.0, 0.0},
          .cellWidth = 10.0,
          .cellHeight = 10.0};
}

DesignRulesT rules() {
  DesignRulesT written;
  written.min_wire_spacing = 185.0;
  written.min_obstacle_spacing = 25.0;
  return written;
}

/// A straight run of cells along x at a fixed y.
std::vector<RCoord> along(const std::uint32_t y, const std::uint32_t from,
                          const std::uint32_t to) {
  std::vector<RCoord> cells;
  for (std::uint32_t x = from; x <= to; ++x) {
    cells.emplace_back(x, y, 6);
  }
  return cells;
}

std::size_t countOf(const flatbuffers::drc::DrcReportT& report,
                    const DrcRule rule) {
  std::size_t found = 0;
  for (const auto& finding : report.findings) {
    found += finding->rule == rule ? 1 : 0;
  }
  return found;
}

TEST(WireClearance, TwoRunsFurtherApartThanTheRuleAreClean) {
  const auto first = along(100, 10, 200);
  const auto second = along(120, 10, 200);
  CellView view;
  view.grid = tenUnitGrid();
  view.wires = {
      {.connection = 0, .cells = first, .components = {}, .feedline = false},
      {.connection = 1, .cells = second, .components = {}, .feedline = false}};

  EXPECT_EQ(countOf(checkCells(view, rules()), DrcRule::WireClearance), 0U);
}

TEST(WireClearance, TwoRunsInsideTheRuleAreOneFinding) {
  const auto first = along(100, 10, 200);
  const auto second = along(110, 10, 200);
  CellView view;
  view.grid = tenUnitGrid();
  view.wires = {
      {.connection = 0, .cells = first, .components = {}, .feedline = false},
      {.connection = 1, .cells = second, .components = {}, .feedline = false}};

  const auto report = checkCells(view, rules());
  // One finding for the pair, at its closest approach, however many cells of
  // the two lie within the rule.
  ASSERT_EQ(countOf(report, DrcRule::WireClearance), 1U);
  EXPECT_EQ(report.findings.front()->measured, 100.0);
  EXPECT_EQ(report.findings.front()->limit, 185.0);
  EXPECT_EQ(report.findings.front()->clearance_kind,
            flatbuffers::drc::ClearanceKind::NearMiss);
}

/// A straight run of cells along y at a fixed x, heading up the grid.
std::vector<RCoord> upward(const std::uint32_t x, const std::uint32_t from,
                           const std::uint32_t to) {
  std::vector<RCoord> cells;
  for (std::uint32_t y = from; y <= to; ++y) {
    cells.emplace_back(x, y, 0);
  }
  return cells;
}

TEST(FeedlineOrthogonality, ARightAngleCrossingOfAFeedlineIsAllowed) {
  // A feedline between two couplers along x, and a wire straight across it
  // along y: the crossing is at a right angle and the rule is content.
  const auto feedline = along(200, 10, 390);
  const auto wire = upward(200, 100, 300);
  CellView view;
  view.grid = tenUnitGrid();
  view.wires = {
      {.connection = 0, .cells = wire, .components = {}, .feedline = false},
      {.connection = 1, .cells = feedline, .components = {}, .feedline = true}};

  const auto report = checkCells(view, rules());
  EXPECT_EQ(countOf(report, DrcRule::FeedlineOrthogonality), 0U);
  // The feedline is left out of the clearance rule altogether.
  EXPECT_EQ(countOf(report, DrcRule::WireClearance), 0U);
  EXPECT_EQ(report.feedlines_skipped, 1U);
}

TEST(FeedlineOrthogonality, ARunBesideAFeedlineIsAFinding) {
  // The same feedline, and a wire that runs along it five cells away:
  // inside the band the crossing rule keeps around a straight run, on a
  // heading that is not across it.
  const auto feedline = along(200, 10, 390);
  const auto wire = along(205, 100, 300);
  CellView view;
  view.grid = tenUnitGrid();
  view.wires = {
      {.connection = 0, .cells = wire, .components = {}, .feedline = false},
      {.connection = 1, .cells = feedline, .components = {}, .feedline = true}};

  const auto report = checkCells(view, rules());
  EXPECT_EQ(countOf(report, DrcRule::FeedlineOrthogonality), 1U);
}

TEST(WireClearance, AFeedlineKeepsTheRuleFromAResonatorButNotFromOtherWires) {
  // An edge between two couplers ten cells from a conventional wire is
  // nothing — the wire crosses such edges on purpose — but ten cells from a
  // resonator it is a finding: a feedline keeps the clearance from every
  // resonator but its own coupler's.
  const auto feedline = along(200, 10, 200);
  const auto conventional = along(210, 10, 200);
  const auto resonator = along(190, 10, 200);
  CellView view;
  view.grid = tenUnitGrid();
  view.wires = {
      {.connection = 0, .cells = feedline, .components = {}, .feedline = true},
      {.connection = 1, .cells = conventional, .components = {}, .feedline = false},
      {.connection = 2, .cells = resonator, .components = {}, .feedline = false, .resonator = true}};

  const auto report = checkCells(view, rules());
  ASSERT_EQ(countOf(report, DrcRule::WireClearance), 1U);
  EXPECT_EQ(report.findings.front()->wires[0].index(), 0U);
  EXPECT_EQ(report.findings.front()->wires[1].index(), 2U);
}

TEST(WireClearance, TwoEdgesOfTheChainsKeepTheRuleFromEachOther) {
  // Two edges ten cells apart that meet at no coupler is a finding: a
  // feedline may be crossed by the wires that pass it, never by another
  // feedline, so the two keep the clearance like any other pair.
  const auto first = along(200, 10, 200);
  const auto second = along(210, 10, 200);
  CellView view;
  view.grid = tenUnitGrid();
  view.wires = {{.connection = 0,
                 .cells = first,
                 .components = {},
                 .feedline = true,
                 .edge = true,
                 .ports = {1, 2}},
                {.connection = 1,
                 .cells = second,
                 .components = {},
                 .feedline = true,
                 .edge = true,
                 .ports = {3, 4}}};

  const auto report = checkCells(view, rules());
  ASSERT_EQ(countOf(report, DrcRule::WireClearance), 1U);
  EXPECT_EQ(report.findings.front()->wires[0].index(), 0U);
  EXPECT_EQ(report.findings.front()->wires[1].index(), 1U);
}

TEST(WireClearance, TwoEdgesThatMeetAtACouplerAreFreeWithinItsReach) {
  // The two edges of one coupler run into it from either side and lie
  // beside each other there on purpose; within the coupler's reach the rule
  // does not bind between them.
  const auto first = along(200, 150, 250);
  const auto second = along(203, 150, 250);
  CellView view;
  view.grid = tenUnitGrid();
  view.couplers = {{.port = 7, .x = 200.0, .y = 200.0, .reach = 60.0}};
  view.wires = {{.connection = 0,
                 .cells = first,
                 .components = {},
                 .feedline = true,
                 .edge = true,
                 .ports = {1, 7}},
                {.connection = 1,
                 .cells = second,
                 .components = {},
                 .feedline = true,
                 .edge = true,
                 .ports = {7, 4}}};

  const auto report = checkCells(view, rules());
  EXPECT_EQ(countOf(report, DrcRule::WireClearance), 0U);
}

TEST(WireClearance, AnEdgeAtALauncherDoesNotWallOffAConventionalWire) {
  // A terminal edge used to be checked against every wire, which made the
  // run from the chip edge to the first coupler a wall. It is an edge like
  // any other now: a conventional wire beside it is nothing.
  const auto terminal = along(200, 10, 200);
  const auto conventional = along(210, 10, 200);
  CellView view;
  view.grid = tenUnitGrid();
  view.wires = {{.connection = 0,
                 .cells = terminal,
                 .components = {},
                 .feedline = true,
                 .edge = true,
                 .ports = {1, 2}},
                {.connection = 1, .cells = conventional, .components = {}, .feedline = false}};

  const auto report = checkCells(view, rules());
  EXPECT_EQ(countOf(report, DrcRule::WireClearance), 0U);
}

TEST(FeedlineOrthogonality, AResonatorIsFreeAroundItsOwnCoupler) {
  // The wire runs beside the feedline, but both share the coupler's port
  // and the run lies within the coupler's reach: that is the coupling, and
  // neither the crossing rule nor the clearance rule binds there.
  const auto feedline = along(200, 150, 250);
  const auto wire = along(203, 160, 240);
  CellView view;
  view.grid = tenUnitGrid();
  view.couplers = {{.port = 7, .x = 200.0, .y = 200.0, .reach = 60.0}};
  view.wires = {{.connection = 0,
                 .cells = wire,
                 .components = {},
                 .feedline = false,
                 .resonator = true,
                 .ports = {7, CheckedWire::NO_PORT}},
                {.connection = 1,
                 .cells = feedline,
                 .components = {},
                 .feedline = true,
                 .ports = {3, 7}}};

  const auto report = checkCells(view, rules());
  EXPECT_EQ(countOf(report, DrcRule::FeedlineOrthogonality), 0U);
  EXPECT_EQ(countOf(report, DrcRule::WireClearance), 0U);
}

TEST(WireClearance, APairInsideTheShortThresholdIsAShort) {
  const auto first = along(100, 10, 200);
  const auto second = along(101, 10, 200);
  CellView view;
  view.grid = tenUnitGrid();
  view.wires = {
      {.connection = 0, .cells = first, .components = {}, .feedline = false},
      {.connection = 1, .cells = second, .components = {}, .feedline = false}};

  const auto report = checkCells(view, rules());
  ASSERT_EQ(countOf(report, DrcRule::WireClearance), 1U);
  EXPECT_EQ(report.findings.front()->clearance_kind,
            flatbuffers::drc::ClearanceKind::Short);
}

TEST(WireClearance, TwoWiresWhoseEndsMeetAreExemptThere) {
  // Two wires whose ends sit within the clearance of each other, as the two
  // approaches to two ports of one coupler do: they converge at their ends,
  // and the ends have nowhere else to go. No component is named; the
  // geometry alone decides.
  const auto first = along(100, 10, 200);
  const auto second = along(110, 10, 200);
  CellView view;
  view.grid = tenUnitGrid();
  view.wires = {
      {.connection = 0, .cells = first, .components = {}, .feedline = false},
      {.connection = 1, .cells = second, .components = {}, .feedline = false}};

  // The two run parallel for the whole grid, so the exemption reaches only as
  // far as one and a half rules from either end and the middle still counts.
  EXPECT_EQ(countOf(checkCells(view, rules()), DrcRule::WireClearance), 1U);

  // The same two wires, only as long as the junction reaches.
  const auto shortFirst = along(100, 180, 200);
  const auto shortSecond = along(110, 180, 200);
  view.wires = {{.connection = 0,
                 .cells = shortFirst,
                 .components = {},
                 .feedline = false},
                {.connection = 1,
                 .cells = shortSecond,
                 .components = {},
                 .feedline = false}};
  EXPECT_EQ(countOf(checkCells(view, rules()), DrcRule::WireClearance), 0U);
}

TEST(WireClearance, OneComponentIsNoExemptionWhenTheEndsLieApart) {
  // Two wires that end on one component whose ports lie far apart — the
  // two ports of a qubit can sit nine hundred units apart — and that run
  // beside each other in between. Nothing meets there, so it counts.
  const auto first = along(100, 10, 200);
  const auto second = along(110, 60, 150);
  CellView view;
  view.grid = tenUnitGrid();
  view.wires = {{.connection = 0,
                 .cells = first,
                 .components = {"", "Qb2"},
                 .feedline = false},
                {.connection = 1,
                 .cells = second,
                 .components = {"", "Qb2"},
                 .feedline = false}};
  EXPECT_EQ(countOf(checkCells(view, rules()), DrcRule::WireClearance), 1U);
}

TEST(WireClearance, AFeedlineBetweenTwoCouplersIsNotChecked) {
  const auto first = along(100, 10, 200);
  const auto second = along(101, 10, 200);
  CellView view;
  view.grid = tenUnitGrid();
  view.wires = {
      {.connection = 0, .cells = first, .components = {}, .feedline = false},
      {.connection = 1, .cells = second, .components = {}, .feedline = true}};

  const auto report = checkCells(view, rules());
  EXPECT_EQ(countOf(report, DrcRule::WireClearance), 0U);
  EXPECT_EQ(report.feedlines_skipped, 1U);
}

TEST(WireLoop, AWireThatComesBackToACellIsReported) {
  std::vector<RCoord> square;
  for (std::uint32_t x = 10; x <= 30; ++x) {
    square.emplace_back(x, 10, 6);
  }
  for (std::uint32_t y = 11; y <= 30; ++y) {
    square.emplace_back(30, y, 4);
  }
  for (std::uint32_t x = 29; x >= 10; --x) {
    square.emplace_back(x, 30, 2);
  }
  for (std::uint32_t y = 29; y >= 10; --y) {
    square.emplace_back(10, y, 0);
  }
  CellView view;
  view.grid = tenUnitGrid();
  view.wires = {
      {.connection = 0, .cells = square, .components = {}, .feedline = false}};

  EXPECT_GT(countOf(checkCells(view, rules()), DrcRule::WireLoop), 0U);
}

TEST(WireLoop, AWireThatOnlyBacktracksOverTwoStepsIsNot) {
  // The router emits a state per heading change, so the cell it turns on is
  // emitted twice. There are hundreds of those per layout and none of them is
  // a loop: the bend radius is five cells, so no real loop is that short.
  std::vector<RCoord> path = along(10, 10, 40);
  path.emplace_back(39, 10, 7);
  path.emplace_back(40, 9, 7);
  CellView view;
  view.grid = tenUnitGrid();
  view.wires = {
      {.connection = 0, .cells = path, .components = {}, .feedline = false}};

  EXPECT_EQ(countOf(checkCells(view, rules()), DrcRule::WireLoop), 0U);
}

} // namespace
} // namespace mqt::scpd::drc
