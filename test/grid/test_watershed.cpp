/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/grid/Watershed.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <vector>

namespace {

using namespace mqt::scpd::grid;

TEST(Watershed, TwoSeedsSplitAFreeGridAtTheMidline) {
  const BitGrid blocked(20, 10);
  std::vector<PartitionLabel> labels(blocked.size(), LABEL_NONE);
  const std::vector<std::size_t> seeds = {(blocked.width() * 5) + 2,
                                          (blocked.width() * 5) + 17};

  const PartitionLabel next =
      runWatershed(blocked, seeds, labels, FIRST_PARTITION_LABEL);
  EXPECT_EQ(next, FIRST_PARTITION_LABEL + 2);
  for (uint32_t y = 0; y < 10; ++y) {
    for (uint32_t x = 0; x < 20; ++x) {
      const PartitionLabel label = labels[(y * 20) + x];
      if (x < 9) {
        EXPECT_EQ(label, FIRST_PARTITION_LABEL) << x << "," << y;
      } else if (x > 10) {
        EXPECT_EQ(label, FIRST_PARTITION_LABEL + 1) << x << "," << y;
      } else {
        EXPECT_NE(label, LABEL_NONE) << x << "," << y;
      }
    }
  }
}

TEST(Watershed, TiesGoToTheLowerSeedAndRunsAreDeterministic) {
  const BitGrid blocked(21, 21);
  std::vector<PartitionLabel> first(blocked.size(), LABEL_NONE);
  const std::vector<std::size_t> seeds = {(21 * 10) + 5, (21 * 10) + 15};
  static_cast<void>(runWatershed(blocked, seeds, first, FIRST_PARTITION_LABEL));
  // The midline is equidistant from both seeds: the first seed wins it.
  EXPECT_EQ(first[(21 * 10) + 10], FIRST_PARTITION_LABEL);
  EXPECT_EQ(first[(21 * 0) + 10], FIRST_PARTITION_LABEL);

  std::vector<PartitionLabel> second(blocked.size(), LABEL_NONE);
  static_cast<void>(
      runWatershed(blocked, seeds, second, FIRST_PARTITION_LABEL));
  EXPECT_EQ(first, second);
}

TEST(Watershed, BarriersAndReservedCellsAreNotEntered) {
  BitGrid blocked(10, 5);
  for (uint32_t y = 0; y < 5; ++y) {
    blocked.setCell(5, y);
  }
  std::vector<PartitionLabel> labels(blocked.size(), LABEL_NONE);
  labels[(2 * 10) + 8] = LABEL_RESERVED;
  const std::vector<std::size_t> seeds = {(2 * 10) + 1, (2 * 10) + 8,
                                          (2 * 10) + 7, (2 * 10) + 7};

  const PartitionLabel next =
      runWatershed(blocked, seeds, labels, FIRST_PARTITION_LABEL);
  // The reserved seed and the duplicate seed are skipped.
  EXPECT_EQ(next, FIRST_PARTITION_LABEL + 2);
  EXPECT_EQ(labels[(2 * 10) + 0], FIRST_PARTITION_LABEL);
  EXPECT_EQ(labels[(2 * 10) + 4], FIRST_PARTITION_LABEL);
  EXPECT_EQ(labels[(2 * 10) + 5], LABEL_NONE);
  EXPECT_EQ(labels[(2 * 10) + 6], FIRST_PARTITION_LABEL + 1);
  EXPECT_EQ(labels[(2 * 10) + 9], FIRST_PARTITION_LABEL + 1);
  EXPECT_EQ(labels[(2 * 10) + 8], LABEL_RESERVED);

  // A second run does not enter the partitions of the first.
  std::vector<PartitionLabel> again = labels;
  const std::vector<std::size_t> late = {(0 * 10) + 9};
  EXPECT_EQ(runWatershed(blocked, late, again, next), next);
  EXPECT_EQ(again, labels);
}

TEST(Watershed, TheMajorityFilterSmoothsAJaggedBorder) {
  const BitGrid blocked(10, 6);
  std::vector<PartitionLabel> labels(blocked.size(), LABEL_NONE);
  for (uint32_t y = 0; y < 6; ++y) {
    for (uint32_t x = 0; x < 10; ++x) {
      labels[(y * 10) + x] = x < 5 ? 2 : 3;
    }
  }
  // One cell of the right partition sticks into the left one.
  labels[(3 * 10) + 4] = 3;
  smoothPartitionBorders(blocked, labels, 2, 1, 3);
  EXPECT_EQ(labels[(3 * 10) + 4], 2);
  EXPECT_EQ(labels[(3 * 10) + 5], 3);
  EXPECT_EQ(labels[(0 * 10) + 4], 2);
}

TEST(Watershed, SkippedSeedsTakeNoLabelAndUnreachedCellsStayFree) {
  // A wall at x = 5 cuts off the two columns to its right.
  BitGrid blocked(8, 5);
  for (uint32_t y = 0; y < 5; ++y) {
    blocked.setCell(5, y);
  }
  std::vector<PartitionLabel> labels(blocked.size(), LABEL_NONE);
  const std::vector<std::size_t> seeds = {blocked.size() + 3, (2 * 8) + 5,
                                          (1 * 8) + 1, (3 * 8) + 3};

  const PartitionLabel next =
      runWatershed(blocked, seeds, labels, FIRST_PARTITION_LABEL);
  // The seed off the grid and the seed on the wall are skipped, so the two
  // accepted seeds get the first two labels.
  EXPECT_EQ(next, FIRST_PARTITION_LABEL + 2);
  EXPECT_EQ(labels[(1 * 8) + 1], FIRST_PARTITION_LABEL);
  EXPECT_EQ(labels[(3 * 8) + 3], FIRST_PARTITION_LABEL + 1);
  for (uint32_t y = 0; y < 5; ++y) {
    for (uint32_t x = 0; x < 8; ++x) {
      const PartitionLabel label = labels[(y * 8) + x];
      if (x < 5) {
        EXPECT_NE(label, LABEL_NONE) << x << "," << y;
      } else {
        EXPECT_EQ(label, LABEL_NONE) << x << "," << y;
      }
    }
  }
}

TEST(Watershed, AGridWithoutCellsAcceptsNoSeed) {
  const BitGrid blocked;
  std::vector<PartitionLabel> labels;
  const std::vector<std::size_t> seeds = {0};
  EXPECT_EQ(runWatershed(blocked, seeds, labels, FIRST_PARTITION_LABEL),
            FIRST_PARTITION_LABEL);
  EXPECT_TRUE(labels.empty());
  smoothPartitionBorders(blocked, labels, FIRST_PARTITION_LABEL, 1, 3);
  EXPECT_TRUE(labels.empty());
}

TEST(Watershed, LabelsShorterThanTheGridAreLeftAlone) {
  const BitGrid blocked(10, 6);
  std::vector<PartitionLabel> labels(blocked.size() - 1, LABEL_NONE);
  const std::vector<std::size_t> seeds = {0, 9};
  EXPECT_EQ(runWatershed(blocked, seeds, labels, FIRST_PARTITION_LABEL),
            FIRST_PARTITION_LABEL);
  EXPECT_EQ(labels,
            std::vector<PartitionLabel>(blocked.size() - 1, LABEL_NONE));

  // A jagged border that the majority filter would smooth on a full grid.
  for (std::size_t i = 0; i < labels.size(); ++i) {
    labels[i] = (i % 10) < 5 ? 2 : 3;
  }
  labels[(3 * 10) + 4] = 3;
  const std::vector<PartitionLabel> jagged = labels;
  smoothPartitionBorders(blocked, labels, 2, 1, 3);
  EXPECT_EQ(labels, jagged);
}

TEST(Watershed, MoreSeedsThanLabelsAreRefusedWithoutAChange) {
  // Every one of the 65792 cells is a seed. The labels from
  // FIRST_PARTITION_LABEL up to one below the largest label cover 65533 of
  // them, because the label after the last one must fit as well.
  constexpr PartitionLabel largest = std::numeric_limits<PartitionLabel>::max();
  const BitGrid blocked(256, 257);
  std::vector<std::size_t> seeds(blocked.size());
  std::iota(seeds.begin(), seeds.end(), std::size_t{0});
  std::vector<PartitionLabel> labels(blocked.size(), LABEL_NONE);
  EXPECT_THROW(static_cast<void>(
                   runWatershed(blocked, seeds, labels, FIRST_PARTITION_LABEL)),
               std::length_error);
  EXPECT_EQ(labels, std::vector<PartitionLabel>(blocked.size(), LABEL_NONE));

  // As many seeds as labels fit: the run returns the largest label.
  const std::size_t fitting = largest - FIRST_PARTITION_LABEL;
  const std::vector<std::size_t> first(
      seeds.begin(), seeds.begin() + static_cast<std::ptrdiff_t>(fitting));
  EXPECT_EQ(runWatershed(blocked, first, labels, FIRST_PARTITION_LABEL),
            largest);
  EXPECT_EQ(labels[0], FIRST_PARTITION_LABEL);
  EXPECT_EQ(labels[fitting - 1], largest - 1);

  // A run that starts at the largest label cannot accept a seed. A run that
  // accepts no seed returns its first label.
  const BitGrid small(3, 3);
  std::vector<PartitionLabel> fresh(small.size(), LABEL_NONE);
  const std::vector<std::size_t> center = {4};
  EXPECT_THROW(static_cast<void>(runWatershed(small, center, fresh, largest)),
               std::length_error);
  EXPECT_EQ(fresh, std::vector<PartitionLabel>(small.size(), LABEL_NONE));
  EXPECT_EQ(runWatershed(small, {}, fresh, largest), largest);
  EXPECT_EQ(runWatershed(small, center, fresh, largest - 1), largest);
  EXPECT_EQ(fresh, std::vector<PartitionLabel>(small.size(), largest - 1));
}

TEST(Watershed, BlockedCellsNeitherVoteNorChangeInTheMajorityFilter) {
  // The center cell holds label 3, and so do the blocked cells at x = 0 and
  // x = 2 of rows 1 and 2. Four of the five free cells in the window of the
  // center hold label 2, a clear majority. Counted, the blocked cells would
  // give label 3 the majority instead.
  BitGrid blocked(3, 3);
  blocked.setCell(0, 1);
  blocked.setCell(2, 1);
  blocked.setCell(0, 2);
  blocked.setCell(2, 2);
  std::vector<PartitionLabel> labels = {2, 2, 2, 3, 3, 3, 3, 2, 3};
  smoothPartitionBorders(blocked, labels, 2, 1, 3);
  EXPECT_EQ(labels, (std::vector<PartitionLabel>{2, 2, 2, 3, 2, 3, 3, 2, 3}));
}

TEST(Watershed, EarlierRunsNeitherVoteNorChangeInTheMajorityFilter) {
  // The run starts at label 3, and the cells of label 2 belong to an earlier
  // run. Four of the five cells of the run in the window of the center hold
  // label 3, a clear majority. Counted, the four cells of label 2 would leave
  // no label a clear majority.
  const BitGrid blocked(3, 3);
  std::vector<PartitionLabel> labels = {3, 3, 3, 2, 4, 2, 2, 3, 2};
  smoothPartitionBorders(blocked, labels, 3, 1, 3);
  EXPECT_EQ(labels, (std::vector<PartitionLabel>{3, 3, 3, 2, 3, 2, 2, 3, 2}));
}

} // namespace
