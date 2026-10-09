/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// The geometry the watershed labels induce: outlines, the borders between
// partitions, the crossing slots and the seeds.

#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"
#include "mqt-scpd/grid/Partitions.hpp"
#include "mqt-scpd/grid/Watershed.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

namespace mqt::scpd::grid {
namespace {

GridMetrics unitGrid(const std::uint32_t width, const std::uint32_t height) {
  return GridMetrics::fit({.minX = 0.0,
                           .minY = 0.0,
                           .maxX = static_cast<double>(width - 1),
                           .maxY = static_cast<double>(height - 1)},
                          width, height);
}

/// Two labels side by side across a free grid, meeting on one column.
std::vector<PartitionLabel> splitLeftAndRight(const GridMetrics& grid,
                                              const std::uint32_t at) {
  std::vector<PartitionLabel> labels(grid.cells(), LABEL_NONE);
  for (std::uint32_t y = 0; y < grid.height; ++y) {
    for (std::uint32_t x = 0; x < grid.width; ++x) {
      labels[grid.index(x, y)] =
          x < at ? FIRST_PARTITION_LABEL
                 : static_cast<PartitionLabel>(FIRST_PARTITION_LABEL + 1);
    }
  }
  return labels;
}

TEST(Partitions, FindsTheBorderBetweenTwoLabelsAndItsLength) {
  const auto grid = unitGrid(10, 6);
  const BitGrid free(10, 6);
  const auto partitions =
      extractPartitions(free, splitLeftAndRight(grid, 4), grid);

  ASSERT_EQ(partitions.borders.size(), 1U);
  const auto& border = partitions.borders.front();
  EXPECT_EQ(border.first, FIRST_PARTITION_LABEL);
  EXPECT_EQ(border.second, FIRST_PARTITION_LABEL + 1);
  // The two labels meet along a full column of six cells.
  EXPECT_EQ(border.samples.size(), 6U);
  EXPECT_DOUBLE_EQ(border.center().x(), 4.0);
  EXPECT_DOUBLE_EQ(border.center().y(), 3.0);
}

TEST(Partitions, TracesOneClosedOutlinePerLabel) {
  const auto grid = unitGrid(10, 6);
  const BitGrid free(10, 6);
  const auto partitions =
      extractPartitions(free, splitLeftAndRight(grid, 4), grid);

  ASSERT_EQ(partitions.outlines.size(), 2U);
  for (const auto& outline : partitions.outlines) {
    ASSERT_GE(outline.ring.size(), 4U);
    // A ring closes on the corner it started from.
    EXPECT_DOUBLE_EQ(outline.ring.front().x(), outline.ring.back().x());
    EXPECT_DOUBLE_EQ(outline.ring.front().y(), outline.ring.back().y());
  }
}

TEST(Partitions, TreatsAnObstacleAsNoLabelAtAll) {
  const auto grid = unitGrid(10, 6);
  BitGrid mask(10, 6);
  // A wall down the meeting column: the two labels no longer touch, so
  // there is no border to budget.
  for (std::uint32_t y = 0; y < 6; ++y) {
    mask.setCell(4, y);
  }

  const auto partitions =
      extractPartitions(mask, splitLeftAndRight(grid, 4), grid);
  EXPECT_TRUE(partitions.borders.empty());
}

TEST(Partitions, NeverTracesAReservedCellAsAPartition) {
  // A reserved cell belongs to the approach of a port. Even where it is free,
  // it is no partition and borders none.
  const auto grid = unitGrid(10, 6);
  const BitGrid free(10, 6);
  auto labels = splitLeftAndRight(grid, 4);
  for (std::uint32_t y = 0; y < 6; ++y) {
    labels[grid.index(0, y)] = LABEL_RESERVED;
  }

  const auto partitions = extractPartitions(free, labels, grid);

  for (const auto& outline : partitions.outlines) {
    EXPECT_GE(outline.label, FIRST_PARTITION_LABEL);
  }
  for (const auto& border : partitions.borders) {
    EXPECT_GE(border.first, FIRST_PARTITION_LABEL);
  }
  EXPECT_EQ(partitions.borders.size(), 1U);
}

TEST(Partitions, RefusesLabelsThatDoNotFitTheGrid) {
  const auto grid = unitGrid(10, 6);
  const BitGrid free(10, 6);
  const std::vector<PartitionLabel> tooFew(4, LABEL_NONE);
  EXPECT_THROW(static_cast<void>(extractPartitions(free, tooFew, grid)),
               std::invalid_argument);
}

TEST(RasterizePartitions, GivesBackTheLabelsTheOutlinesWereTracedFrom) {
  const auto grid = unitGrid(10, 6);
  const BitGrid free(10, 6);
  const auto labels = splitLeftAndRight(grid, 4);

  const auto partitions = extractPartitions(free, labels, grid);

  EXPECT_EQ(rasterizePartitions(partitions.outlines, grid), labels);
}

TEST(RasterizePartitions, CancelsAHoleAgainstTheRingAroundIt) {
  const auto grid = unitGrid(7, 7);
  BitGrid blocked(7, 7);
  blocked.setCell(3, 3);
  std::vector<PartitionLabel> labels(grid.cells(), FIRST_PARTITION_LABEL);
  labels[grid.index(3, 3)] = LABEL_NONE;

  const auto partitions = extractPartitions(blocked, labels, grid);
  // The partition surrounds the obstacle, so it is traced as two rings.
  ASSERT_EQ(partitions.outlines.size(), 2U);

  const auto filled = rasterizePartitions(partitions.outlines, grid);
  EXPECT_EQ(filled, labels);
  EXPECT_EQ(filled[grid.index(3, 3)], LABEL_NONE);
}

TEST(RasterizePartitions, KeepsAPartitionOfOneCell) {
  const auto grid = unitGrid(5, 5);
  BitGrid blocked(5, 5, true);
  blocked.setCell(2, 2, false);
  std::vector<PartitionLabel> labels(grid.cells(), LABEL_NONE);
  labels[grid.index(2, 2)] = FIRST_PARTITION_LABEL;

  const auto partitions = extractPartitions(blocked, labels, grid);

  EXPECT_EQ(rasterizePartitions(partitions.outlines, grid), labels);
}

TEST(BorderSlots, PutsTheSlotsOneWireSpacingApart) {
  // Ten cells across ten layout units: one cell step is 10/9 units.
  const auto grid = GridMetrics::fit(
      {.minX = 0.0, .minY = 0.0, .maxX = 10.0, .maxY = 10.0}, 10, 10);
  PartitionBorder border{
      .first = FIRST_PARTITION_LABEL,
      .second = static_cast<PartitionLabel>(FIRST_PARTITION_LABEL + 1),
      .samples = {}};
  for (std::uint32_t y = 0; y < 10; ++y) {
    border.samples.emplace_back(4.5, static_cast<double>(y));
  }

  const auto slots = borderSlots(border, grid, 3.0);

  ASSERT_FALSE(slots.empty());
  for (std::size_t i = 0; i < slots.size(); ++i) {
    for (std::size_t j = i + 1; j < slots.size(); ++j) {
      const auto a = grid.toLayout(slots[i].x(), slots[i].y());
      const auto b = grid.toLayout(slots[j].x(), slots[j].y());
      EXPECT_GE(std::hypot(a.x() - b.x(), a.y() - b.y()), 3.0);
    }
  }
}

TEST(BorderSlots, GivesABorderShorterThanOneSpacingASingleSlot) {
  const auto grid = GridMetrics::fit(
      {.minX = 0.0, .minY = 0.0, .maxX = 10.0, .maxY = 10.0}, 10, 10);
  const PartitionBorder border{
      .first = FIRST_PARTITION_LABEL,
      .second = static_cast<PartitionLabel>(FIRST_PARTITION_LABEL + 1),
      .samples = {{4.5, 0.0}, {4.5, 1.0}}};

  EXPECT_EQ(borderSlots(border, grid, 100.0).size(), 1U);
}

TEST(BorderSlots, RefusesASpacingThatIsNotPositive) {
  const auto grid = unitGrid(4, 4);
  const PartitionBorder border{
      .first = FIRST_PARTITION_LABEL,
      .second = static_cast<PartitionLabel>(FIRST_PARTITION_LABEL + 1),
      .samples = {{1.5, 0.0}}};

  EXPECT_THROW(std::ignore = borderSlots(border, grid, 0.0),
               std::invalid_argument);
}

TEST(FreeCellSeeds, SeedsOnlyTheCapacityCellsWithoutAnObstacle) {
  const auto coarse = unitGrid(4, 4);
  const auto detail = coarse.refined(5);
  BitGrid mask(detail.width, detail.height);
  // One obstacle cell inside the coarse cell (1, 1) disqualifies it.
  mask.setCell(7, 7);

  const auto seeds = freeCellSeeds(mask, detail, coarse);
  EXPECT_EQ(seeds.size(), 15U);
  // The seeds arrive in row-major order over the coarse grid, which is what
  // makes the watershed labels reproducible.
  EXPECT_TRUE(std::ranges::is_sorted(seeds));
  EXPECT_EQ(std::ranges::find(seeds, detail.index(7, 7)), seeds.end());
}

TEST(FreeCellSeeds, RefusesADetailGridThatIsNoWholeRefinement) {
  const auto coarse = unitGrid(4, 4);
  const auto detail = unitGrid(18, 20);
  const BitGrid mask(18, 20);
  EXPECT_THROW(static_cast<void>(freeCellSeeds(mask, detail, coarse)),
               std::invalid_argument);
}

} // namespace
} // namespace mqt::scpd::grid
