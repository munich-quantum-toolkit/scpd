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

TEST(WireClearance, TwoWiresThatEndOnOneComponentAreExemptWhereTheyMeet) {
  // Two wires running to two ports of one component, converging at their
  // ends. The ports of a component sit closer together than the wire spacing
  // and each of the two has to be reached, so the approaches have nowhere
  // else to go.
  const auto first = along(100, 10, 200);
  const auto second = along(110, 10, 200);
  CellView view;
  view.grid = tenUnitGrid();
  view.wires = {{.connection = 0,
                 .cells = first,
                 .components = {"", "Qb1"},
                 .feedline = false},
                {.connection = 1,
                 .cells = second,
                 .components = {"", "Qb1"},
                 .feedline = false}};

  // The two run parallel for the whole grid, so the exemption reaches only as
  // far as one and a half rules from either end and the middle still counts.
  EXPECT_EQ(countOf(checkCells(view, rules()), DrcRule::WireClearance), 1U);

  // The same two wires, only as long as the junction reaches.
  const auto shortFirst = along(100, 180, 200);
  const auto shortSecond = along(110, 180, 200);
  view.wires = {{.connection = 0,
                 .cells = shortFirst,
                 .components = {"", "Qb1"},
                 .feedline = false},
                {.connection = 1,
                 .cells = shortSecond,
                 .components = {"", "Qb1"},
                 .feedline = false}};
  EXPECT_EQ(countOf(checkCells(view, rules()), DrcRule::WireClearance), 0U);
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

TEST(ObstacleClearance, NothingIsCheckedWithoutAChip) {
  const auto first = along(100, 10, 200);
  CellView view;
  view.grid = tenUnitGrid();
  view.wires = {
      {.connection = 0, .cells = first, .components = {}, .feedline = false}};

  EXPECT_EQ(countOf(checkCells(view, rules()), DrcRule::ObstacleClearance), 0U);
}

} // namespace
} // namespace mqt::scpd::drc
