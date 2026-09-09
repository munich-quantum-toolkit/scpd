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
#include <vector>

namespace {

using namespace mqt::scpd::grid;

TEST(Watershed, TwoSeedsSplitAFreeGridAtTheMidline) {
  const BitGrid blocked(20, 10);
  std::vector<PartitionLabel> labels(blocked.size(), LABEL_NONE);
  const std::vector<std::size_t> seeds = {blocked.width() * 5 + 2, blocked.width() * 5 + 17};

  const PartitionLabel next = runWatershed(blocked, seeds, labels, FIRST_PARTITION_LABEL);
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
  const std::vector<std::size_t> seeds = {21 * 10 + 5, 21 * 10 + 15};
  static_cast<void>(runWatershed(blocked, seeds, first, FIRST_PARTITION_LABEL));
  // The midline is equidistant from both seeds: the first seed wins it.
  EXPECT_EQ(first[21 * 10 + 10], FIRST_PARTITION_LABEL);
  EXPECT_EQ(first[21 * 0 + 10], FIRST_PARTITION_LABEL);

  std::vector<PartitionLabel> second(blocked.size(), LABEL_NONE);
  static_cast<void>(runWatershed(blocked, seeds, second, FIRST_PARTITION_LABEL));
  EXPECT_EQ(first, second);
}

TEST(Watershed, BarriersAndReservedCellsAreNotEntered) {
  BitGrid blocked(10, 5);
  for (uint32_t y = 0; y < 5; ++y) {
    blocked.setCell(5, y);
  }
  std::vector<PartitionLabel> labels(blocked.size(), LABEL_NONE);
  labels[2 * 10 + 8] = LABEL_RESERVED;
  const std::vector<std::size_t> seeds = {2 * 10 + 1, 2 * 10 + 8, 2 * 10 + 7, 2 * 10 + 7};

  const PartitionLabel next = runWatershed(blocked, seeds, labels, FIRST_PARTITION_LABEL);
  // The reserved seed and the duplicate seed are skipped.
  EXPECT_EQ(next, FIRST_PARTITION_LABEL + 2);
  EXPECT_EQ(labels[2 * 10 + 0], FIRST_PARTITION_LABEL);
  EXPECT_EQ(labels[2 * 10 + 4], FIRST_PARTITION_LABEL);
  EXPECT_EQ(labels[2 * 10 + 5], LABEL_NONE);
  EXPECT_EQ(labels[2 * 10 + 6], FIRST_PARTITION_LABEL + 1);
  EXPECT_EQ(labels[2 * 10 + 9], FIRST_PARTITION_LABEL + 1);
  EXPECT_EQ(labels[2 * 10 + 8], LABEL_RESERVED);

  // A second run does not enter the partitions of the first.
  std::vector<PartitionLabel> again = labels;
  const std::vector<std::size_t> late = {0 * 10 + 9};
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

} // namespace
