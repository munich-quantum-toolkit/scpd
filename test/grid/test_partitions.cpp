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
#include <limits>
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

TEST(Partitions, ConvertCornerCoordinatesHalfACellFromTheCellCenters) {
  // Ten by six cells over a box with steps of 10 and 20 layout units. The
  // center of cell (0, 0) is the origin of the grid, at (100, -50).
  const auto grid = GridMetrics::fit(
      {.minX = 100.0, .minY = -50.0, .maxX = 190.0, .maxY = 50.0}, 10, 6);

  // The corners of the grid lie half a cell step outside the box.
  EXPECT_EQ(cornerToLayout(grid, 0.0, 0.0), Point(95.0, -60.0));
  EXPECT_EQ(cornerToLayout(grid, 10.0, 6.0), Point(195.0, 60.0));
  // The center of a cell in corner coordinates is the layout point of the
  // cell.
  EXPECT_EQ(cornerToLayout(grid, 3.5, 2.5), grid.toLayout(3.0, 2.0));
  EXPECT_EQ(layoutToCorner(grid, grid.toLayout(3.0, 2.0)), Point(3.5, 2.5));
  EXPECT_EQ(layoutToCorner(grid, Point(95.0, -60.0)), Point(0.0, 0.0));
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

TEST(Partitions, StartsEveryRingAtItsFirstCornerInRowOrder) {
  // A partition with a hole, and two cells of another label that touch at a
  // corner, so that some corners carry two rings.
  const auto grid = unitGrid(7, 7);
  const BitGrid free(7, 7);
  std::vector<PartitionLabel> labels(grid.cells(), FIRST_PARTITION_LABEL);
  const auto hole = static_cast<PartitionLabel>(FIRST_PARTITION_LABEL + 1);
  const auto touching = static_cast<PartitionLabel>(FIRST_PARTITION_LABEL + 2);
  for (const auto& [x, y] : {std::pair{2U, 2U}, {3U, 2U}, {2U, 3U}, {3U, 3U}}) {
    labels[grid.index(x, y)] = hole;
  }
  labels[grid.index(5, 4)] = touching;
  labels[grid.index(4, 5)] = touching;

  const auto partitions = extractPartitions(free, labels, grid);
  ASSERT_FALSE(partitions.outlines.empty());
  for (const auto& outline : partitions.outlines) {
    ASSERT_FALSE(outline.ring.empty());
    const auto& start = outline.ring.front();
    for (const auto& corner : outline.ring) {
      EXPECT_TRUE(start.y() < corner.y() ||
                  (start.y() == corner.y() && start.x() <= corner.x()))
          << "label " << outline.label << " starts at (" << start.x() << ", "
          << start.y() << ") before (" << corner.x() << ", " << corner.y()
          << ")";
    }
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
  // it is not a partition and borders none.
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

TEST(SeparatePartitionPieces, GivesAPieceCutOffByAnObstacleItsOwnLabel) {
  // One label on both sides of a wall: the larger piece on the left keeps
  // it, and the smaller one on the right takes the next free label.
  const auto grid = unitGrid(7, 3);
  BitGrid blocked(7, 3);
  for (std::uint32_t y = 0; y < 3; ++y) {
    blocked.set(grid.index(4, y), true);
  }
  std::vector<PartitionLabel> labels(grid.cells(), FIRST_PARTITION_LABEL);
  const auto next = static_cast<PartitionLabel>(FIRST_PARTITION_LABEL + 5);

  EXPECT_EQ(separatePartitionPieces(blocked, labels, grid, next),
            static_cast<PartitionLabel>(next + 1));

  for (std::uint32_t y = 0; y < 3; ++y) {
    for (std::uint32_t x = 0; x < 7; ++x) {
      const auto expected = x <= 4 ? FIRST_PARTITION_LABEL : next;
      EXPECT_EQ(labels[grid.index(x, y)], expected) << x << "," << y;
    }
  }
}

TEST(SeparatePartitionPieces, SplitsCellsThatMeetOnlyAtACorner) {
  // A wire crosses from cell to cell over an edge, never over a corner.
  const auto grid = unitGrid(2, 2);
  const BitGrid free(2, 2);
  std::vector<PartitionLabel> labels = {FIRST_PARTITION_LABEL, LABEL_RESERVED,
                                        LABEL_RESERVED, FIRST_PARTITION_LABEL};
  const auto next = static_cast<PartitionLabel>(FIRST_PARTITION_LABEL + 1);

  EXPECT_EQ(separatePartitionPieces(free, labels, grid, next),
            static_cast<PartitionLabel>(next + 1));

  // Of two pieces of one cell each, the first in row order keeps the label.
  EXPECT_EQ(labels,
            (std::vector<PartitionLabel>{FIRST_PARTITION_LABEL, LABEL_RESERVED,
                                         LABEL_RESERVED, next}));
}

TEST(SeparatePartitionPieces, LeavesPartitionsInOnePieceAsTheyAre) {
  const auto grid = unitGrid(10, 6);
  const BitGrid free(10, 6);
  auto labels = splitLeftAndRight(grid, 4);
  const auto before = labels;
  const auto next = static_cast<PartitionLabel>(FIRST_PARTITION_LABEL + 2);

  EXPECT_EQ(separatePartitionPieces(free, labels, grid, next), next);
  EXPECT_EQ(labels, before);
}

TEST(SeparatePartitionPieces, ChangesNothingWhenTheLabelsRunOut) {
  const auto grid = unitGrid(3, 2);
  const BitGrid free(3, 2);
  std::vector<PartitionLabel> labels = {FIRST_PARTITION_LABEL,
                                        LABEL_NONE,
                                        FIRST_PARTITION_LABEL,
                                        LABEL_NONE,
                                        LABEL_NONE,
                                        LABEL_NONE};
  const auto before = labels;

  EXPECT_THROW(
      static_cast<void>(separatePartitionPieces(
          free, labels, grid, std::numeric_limits<PartitionLabel>::max())),
      std::length_error);
  EXPECT_EQ(labels, before);
}

TEST(SeparatePartitionPieces, RefusesLabelsThatDoNotFitTheGrid) {
  const auto grid = unitGrid(3, 3);
  const BitGrid free(3, 3);
  std::vector<PartitionLabel> labels(4, FIRST_PARTITION_LABEL);
  EXPECT_THROW(static_cast<void>(separatePartitionPieces(
                   free, labels, grid, FIRST_PARTITION_LABEL + 1)),
               std::invalid_argument);
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

TEST(BorderSlots, KeepsTheSpacingOnABorderThatBends) {
  // An arch: up the left leg at x = 2.5, across the top at y = 8.5 and down
  // the right leg at x = 6.5. The samples start in the middle of the top, so
  // the order they come in does not start the walk at an end.
  PartitionBorder border{
      .first = FIRST_PARTITION_LABEL,
      .second = static_cast<PartitionLabel>(FIRST_PARTITION_LABEL + 1),
      .samples = {}};
  for (std::uint32_t x = 4; x <= 6; ++x) {
    border.samples.emplace_back(static_cast<double>(x), 8.5);
  }
  border.samples.emplace_back(3.0, 8.5);
  for (std::uint32_t y = 2; y <= 8; ++y) {
    border.samples.emplace_back(2.5, static_cast<double>(y));
    border.samples.emplace_back(6.5, static_cast<double>(y));
  }
  const auto grid = unitGrid(10, 10);

  const auto slots = borderSlots(border, grid, 3.0);

  // The arch is one piece about 17 units long: six slots fit at a spacing of
  // three, and no two of them lie closer than that.
  EXPECT_EQ(slots.size(), 6U);
  for (std::size_t i = 0; i < slots.size(); ++i) {
    for (std::size_t j = i + 1; j < slots.size(); ++j) {
      EXPECT_GE(
          std::hypot(slots[i].x() - slots[j].x(), slots[i].y() - slots[j].y()),
          3.0)
          << i << " and " << j;
    }
  }
}

TEST(BorderSlots, KeepsTheSpacingOnAStaircase) {
  // A border that falls to the left at 45 degrees from a short top row, in
  // the row order of the scan. On the staircase the sample after the next
  // one lies within the reach of a step, so neither end of the staircase has
  // a single neighbor, and the walk does not start at an end.
  PartitionBorder border{
      .first = FIRST_PARTITION_LABEL,
      .second = static_cast<PartitionLabel>(FIRST_PARTITION_LABEL + 1),
      .samples = {{7.5, 0.0},
                  {8.5, 0.0},
                  {7.0, 0.5},
                  {9.0, 0.5},
                  {6.5, 1.0},
                  {9.5, 1.0}}};
  for (std::uint32_t step = 0; step < 6; ++step) {
    const auto offset = static_cast<double>(step);
    border.samples.emplace_back(6.0 - offset, 1.5 + offset);
    border.samples.emplace_back(5.5 - offset, 2.0 + offset);
  }
  const auto grid = unitGrid(12, 12);

  const auto slots = borderSlots(border, grid, 4.0);

  ASSERT_FALSE(slots.empty());
  for (std::size_t i = 0; i < slots.size(); ++i) {
    for (std::size_t j = i + 1; j < slots.size(); ++j) {
      EXPECT_GE(
          std::hypot(slots[i].x() - slots[j].x(), slots[i].y() - slots[j].y()),
          4.0)
          << i << " and " << j;
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

TEST(Partitions, FindsNothingOnAnEmptyGrid) {
  const GridMetrics empty;
  const BitGrid mask(0, 0);

  const auto partitions = extractPartitions(mask, {}, empty);

  EXPECT_TRUE(partitions.outlines.empty());
  EXPECT_TRUE(partitions.borders.empty());
  EXPECT_TRUE(rasterizePartitions({}, empty).empty());
}

TEST(Partitions, PutsTheCenterOfABorderWithoutSamplesAtTheOrigin) {
  const PartitionBorder border;
  EXPECT_EQ(border.center(), Point());
}

TEST(BorderSlots, GivesABorderWithoutSamplesNoSlot) {
  const PartitionBorder border{
      .first = FIRST_PARTITION_LABEL,
      .second = static_cast<PartitionLabel>(FIRST_PARTITION_LABEL + 1),
      .samples = {}};
  EXPECT_TRUE(borderSlots(border, unitGrid(4, 4), 1.0).empty());
}

TEST(FreeCellSeeds, RefusesAMaskOfTheWrongSize) {
  const auto coarse = unitGrid(4, 4);
  const auto detail = coarse.refined(5);
  const BitGrid mask(detail.width + 1, detail.height);
  EXPECT_THROW(static_cast<void>(freeCellSeeds(mask, detail, coarse)),
               std::invalid_argument);
}

} // namespace
} // namespace mqt::scpd::grid
