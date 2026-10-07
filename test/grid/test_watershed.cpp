/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "../SplitMix.hpp"
#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/grid/Watershed.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <numeric>
#include <span>
#include <stdexcept>
#include <vector>

namespace {

using namespace mqt::scpd::grid;
using mqt::scpd::test::SplitMix;

/// Whether the majority filter turned @p before into a valid labeling
/// @p after: blocked cells and cells outside the run keep their labels, every
/// other cell holds a label of the run, and every seed keeps its label.
testing::AssertionResult isValidSmoothing(
    const BitGrid& blocked, const std::vector<PartitionLabel>& before,
    const std::vector<PartitionLabel>& after,
    const std::span<const std::size_t> seeds, const PartitionLabel firstLabel,
    const PartitionLabel nextLabel) {
  for (std::size_t cell = 0; cell < blocked.size(); ++cell) {
    if (blocked.test(cell) || before[cell] < firstLabel) {
      if (after[cell] != before[cell]) {
        return testing::AssertionFailure()
               << "cell " << cell << " outside the run changed from "
               << before[cell] << " to " << after[cell];
      }
    } else if (after[cell] < firstLabel || after[cell] >= nextLabel) {
      return testing::AssertionFailure()
             << "cell " << cell << " holds " << after[cell]
             << ", which is no label of the run";
    }
  }
  for (const std::size_t seed : seeds) {
    if (seed < blocked.size() && after[seed] != before[seed]) {
      return testing::AssertionFailure()
             << "seed " << seed << " changed from " << before[seed] << " to "
             << after[seed];
    }
  }
  return testing::AssertionSuccess();
}

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

TEST(Watershed, TheLowerSeedWinsTheMidlineInEveryLayout) {
  // Two seeds mirrored across the midline of a free grid, swapped left and
  // right and top and bottom. Every cell of the midline is equidistant from
  // both seeds, so the first seed wins all of them in every layout.
  constexpr uint32_t side = 21;
  const BitGrid blocked(side, side);
  const auto at = [](const uint32_t x, const uint32_t y) {
    return (static_cast<std::size_t>(y) * side) + x;
  };
  const auto midlineOfFirstSeed = [&](const std::vector<std::size_t>& seeds,
                                      const bool column) {
    std::vector<PartitionLabel> labels(blocked.size(), LABEL_NONE);
    static_cast<void>(
        runWatershed(blocked, seeds, labels, FIRST_PARTITION_LABEL));
    uint32_t count = 0;
    for (uint32_t i = 0; i < side; ++i) {
      const std::size_t cell = column ? at(10, i) : at(i, 10);
      if (labels[cell] == FIRST_PARTITION_LABEL) {
        ++count;
      }
    }
    return count;
  };
  EXPECT_EQ(midlineOfFirstSeed({at(5, 10), at(15, 10)}, true), side);
  EXPECT_EQ(midlineOfFirstSeed({at(15, 10), at(5, 10)}, true), side);
  EXPECT_EQ(midlineOfFirstSeed({at(10, 5), at(10, 15)}, false), side);
  EXPECT_EQ(midlineOfFirstSeed({at(10, 15), at(10, 5)}, false), side);
}

TEST(Watershed, TheLowerSeedWinsTheDiagonalInEveryLayout) {
  // Two seeds mirrored across the diagonal x = y, in both orders. Every cell
  // of the diagonal is equidistant from both seeds, but one front reaches it
  // along x and the other along y. So the first seed wins every diagonal cell
  // that a front reaches, and the labels of the two seed orders mirror each
  // other. The first layout is a free grid; the others carry random obstacles
  // mirrored across the diagonal.
  SplitMix random(29);
  for (int round = 0; round < 100; ++round) {
    const auto side =
        static_cast<uint32_t>(round == 0 ? 21 : random.between(9, 31));
    const auto at = [&](const uint32_t x, const uint32_t y) {
      return (static_cast<std::size_t>(y) * side) + x;
    };
    BitGrid blocked(side, side);
    const double density = round == 0 ? 0.0 : 0.05 + (0.3 * random.unit());
    for (uint32_t y = 0; y < side; ++y) {
      for (uint32_t x = 0; x <= y; ++x) {
        if (random.unit() < density) {
          blocked.setCell(x, y);
          blocked.setCell(y, x);
        }
      }
    }
    uint32_t seedX = 5;
    uint32_t seedY = 10;
    while (round > 0 && (seedX == seedY || blocked.testCell(seedX, seedY))) {
      seedX = static_cast<uint32_t>(random.between(0, side - 1));
      seedY = static_cast<uint32_t>(random.between(0, side - 1));
    }
    const std::vector<std::size_t> seeds = {at(seedX, seedY), at(seedY, seedX)};
    const std::vector<std::size_t> swapped = {seeds[1], seeds[0]};
    std::vector<PartitionLabel> labels(blocked.size(), LABEL_NONE);
    static_cast<void>(
        runWatershed(blocked, seeds, labels, FIRST_PARTITION_LABEL));
    std::vector<PartitionLabel> mirrored(blocked.size(), LABEL_NONE);
    static_cast<void>(
        runWatershed(blocked, swapped, mirrored, FIRST_PARTITION_LABEL));
    uint32_t reached = 0;
    for (uint32_t y = 0; y < side; ++y) {
      for (uint32_t x = 0; x < side; ++x) {
        ASSERT_EQ(labels[at(x, y)], mirrored[at(y, x)])
            << "round " << round << ", cell " << x << "," << y;
      }
      const PartitionLabel diagonal = labels[at(y, y)];
      ASSERT_NE(diagonal, FIRST_PARTITION_LABEL + 1)
          << "round " << round << ", cell " << y << "," << y;
      reached += diagonal == FIRST_PARTITION_LABEL ? 1 : 0;
    }
    if (round == 0) {
      EXPECT_EQ(reached, side);
    }
  }
}

TEST(Watershed, TheLowerSeedWinsWhereTheArrivalTimesDifferByRoundingOnly) {
  // Two square rooms of 3 to 5 cells on a side, joined by a corridor one cell
  // wide along their top rows. All other cells are blocked.
  // The first seed sits left of the bottom row of the left room. Its front
  // crosses that room and then m + k corridor cells to the meeting cell. The
  // second seed sits m + 1 cells right of the right room, on a corridor along
  // its bottom row. Its front crosses m corridor cells, that room and k
  // corridor cells to the meeting cell. Both fronts take the same steps in
  // another order, so their arrival times at the meeting cell differ by
  // rounding only. The first seed wins the meeting cell in both seed orders.
  for (uint32_t room = 3; room <= 5; ++room) {
    for (uint32_t m = 1; m <= 4; ++m) {
      for (uint32_t k = 0; k <= 2; ++k) {
        const uint32_t top = room - 1;
        const uint32_t meetingX = room + 1 + m + k;
        const uint32_t rightRoomX = meetingX + k + 1;
        const uint32_t secondX = rightRoomX + room + m;
        const uint32_t width = secondX + 1;
        BitGrid blocked(width, room, true);
        blocked.setCell(0, 0, false);
        for (uint32_t y = 0; y < room; ++y) {
          for (uint32_t x = 1; x <= room; ++x) {
            blocked.setCell(x, y, false);
            blocked.setCell(rightRoomX + x - 1, y, false);
          }
        }
        for (uint32_t x = room + 1; x < rightRoomX; ++x) {
          blocked.setCell(x, top, false);
        }
        for (uint32_t x = rightRoomX + room; x <= secondX; ++x) {
          blocked.setCell(x, 0, false);
        }
        const std::size_t meeting =
            (static_cast<std::size_t>(top) * width) + meetingX;
        for (const std::vector<std::size_t>& seeds :
             {std::vector<std::size_t>{0, secondX},
              std::vector<std::size_t>{secondX, 0}}) {
          std::vector<PartitionLabel> labels(blocked.size(), LABEL_NONE);
          static_cast<void>(
              runWatershed(blocked, seeds, labels, FIRST_PARTITION_LABEL));
          EXPECT_EQ(labels[meeting], FIRST_PARTITION_LABEL)
              << "room " << room << ", m " << m << ", k " << k
              << ", first seed at " << seeds[0];
        }
      }
    }
  }
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
  smoothPartitionBorders(blocked, {}, labels, 2, 1, 3);
  EXPECT_EQ(labels[(3 * 10) + 4], 2);
  EXPECT_EQ(labels[(3 * 10) + 5], 3);
  EXPECT_EQ(labels[(0 * 10) + 4], 2);
}

TEST(Watershed, TheIterationCapEndsAMajorityFilterThatNeverSettles) {
  // Passes that read the labels from before the pass flip every free cell of
  // these labels on every pass and never settle. The cells (3, 0) and (0, 1)
  // are blocked. The call returns after any number of passes, and the labels
  // stay a valid labeling of the run.
  BitGrid blocked(4, 2);
  blocked.setCell(3, 0);
  blocked.setCell(0, 1);
  const std::vector<PartitionLabel> input = {2, 3, 2, 0, 0, 3, 2, 3};
  for (const int iterations : {0, 1, 2, 3, 10001}) {
    std::vector<PartitionLabel> labels = input;
    smoothPartitionBorders(blocked, {}, labels, 2, 1, iterations);
    EXPECT_TRUE(isValidSmoothing(blocked, input, labels, {}, 2, 4))
        << iterations << " passes";
  }
}

TEST(Watershed, TheMajorityFilterKeepsEveryPartitionWithItsSeed) {
  // The center seed of a free grid grows a partition of 61 cells, and the two
  // corner seeds grow partitions of 10 cells each. In a window of radius 2 or
  // 3, the center partition holds the majority at every border cell. Only
  // their seeds keep the corner partitions from vanishing.
  const BitGrid blocked(9, 9);
  const std::vector<std::size_t> seeds = {(9 * 4) + 4, 0, 8};
  for (const int radius : {2, 3}) {
    std::vector<PartitionLabel> labels(blocked.size(), LABEL_NONE);
    const PartitionLabel next =
        runWatershed(blocked, seeds, labels, FIRST_PARTITION_LABEL);
    ASSERT_EQ(next, FIRST_PARTITION_LABEL + 3);
    const std::vector<PartitionLabel> before = labels;
    smoothPartitionBorders(blocked, seeds, labels, FIRST_PARTITION_LABEL,
                           radius, 5);
    EXPECT_TRUE(isValidSmoothing(blocked, before, labels, seeds,
                                 FIRST_PARTITION_LABEL, next))
        << "radius " << radius;
    for (std::size_t i = 0; i < seeds.size(); ++i) {
      EXPECT_EQ(labels[seeds[i]],
                static_cast<PartitionLabel>(FIRST_PARTITION_LABEL + i))
          << "radius " << radius << ", seed " << i;
    }
  }
}

TEST(Watershed, TheMajorityFilterLeavesAValidLabelingOnRandomGrids) {
  // Random grids with obstacles, reserved cells, cells of an earlier run and
  // one watershed run. After any number of passes, the labels are a valid
  // labeling of the run, and the mirror image of the input gives the mirror
  // image of the result.
  SplitMix random(43);
  constexpr PartitionLabel earlier = FIRST_PARTITION_LABEL;
  constexpr PartitionLabel firstLabel = FIRST_PARTITION_LABEL + 1;
  for (int round = 0; round < 300; ++round) {
    const auto width = static_cast<uint32_t>(random.between(1, 24));
    const auto height = static_cast<uint32_t>(random.between(1, 24));
    const auto mirror = [&](const std::size_t cell) {
      return ((cell / width) * width) + (width - 1 - (cell % width));
    };
    BitGrid blocked(width, height);
    std::vector<PartitionLabel> labels(blocked.size(), LABEL_NONE);
    const double density = 0.3 * random.unit();
    for (std::size_t cell = 0; cell < blocked.size(); ++cell) {
      const double draw = random.unit();
      if (draw < density) {
        blocked.set(cell, true);
      } else if (draw < density + 0.02) {
        labels[cell] = LABEL_RESERVED;
      } else if (draw < density + 0.06) {
        labels[cell] = earlier;
      }
    }
    // Some seeds fall off the grid, on a blocked cell or on a labeled cell.
    std::vector<std::size_t> seeds(
        static_cast<std::size_t>(random.between(1, 12)));
    for (std::size_t& seed : seeds) {
      seed = static_cast<std::size_t>(
          random.between(0, static_cast<int64_t>(blocked.size()) + 1));
    }
    const PartitionLabel next =
        runWatershed(blocked, seeds, labels, firstLabel);
    const int radius = static_cast<int>(random.between(0, 4));
    const int iterations = static_cast<int>(random.between(0, 6));

    std::vector<PartitionLabel> smoothed = labels;
    smoothPartitionBorders(blocked, seeds, smoothed, firstLabel, radius,
                           iterations);
    ASSERT_TRUE(
        isValidSmoothing(blocked, labels, smoothed, seeds, firstLabel, next))
        << "round " << round;

    BitGrid mirroredBlocked(width, height);
    std::vector<PartitionLabel> mirrored(blocked.size(), LABEL_NONE);
    for (std::size_t cell = 0; cell < blocked.size(); ++cell) {
      mirroredBlocked.set(mirror(cell), blocked.test(cell));
      mirrored[mirror(cell)] = labels[cell];
    }
    std::vector<std::size_t> mirroredSeeds;
    for (const std::size_t seed : seeds) {
      if (seed < blocked.size()) {
        mirroredSeeds.push_back(mirror(seed));
      }
    }
    smoothPartitionBorders(mirroredBlocked, mirroredSeeds, mirrored, firstLabel,
                           radius, iterations);
    for (std::size_t cell = 0; cell < blocked.size(); ++cell) {
      ASSERT_EQ(mirrored[mirror(cell)], smoothed[cell])
          << "round " << round << ", cell " << cell;
    }
  }
}

TEST(Watershed, AWindowLargerThanTheGridCountsTheWholeGrid) {
  // The window ends at the edge of the grid. The largest radius therefore
  // gives the labels of the radius that just covers the grid from every cell,
  // at the same cost.
  const BitGrid blocked(10, 6);
  std::vector<PartitionLabel> input(blocked.size(), LABEL_NONE);
  for (uint32_t y = 0; y < 6; ++y) {
    for (uint32_t x = 0; x < 10; ++x) {
      input[(y * 10) + x] = x < 5 ? 2 : 3;
    }
  }
  input[(3 * 10) + 4] = 3;
  std::vector<PartitionLabel> covering = input;
  smoothPartitionBorders(blocked, {}, covering, 2, 9, 3);
  std::vector<PartitionLabel> largest = input;
  smoothPartitionBorders(blocked, {}, largest, 2,
                         std::numeric_limits<int>::max(), 3);
  EXPECT_EQ(largest, covering);
}

TEST(Watershed, ANegativeRadiusOrPassCountIsRefused) {
  const BitGrid blocked(10, 6);
  std::vector<PartitionLabel> labels(blocked.size(), 2);
  labels[(3 * 10) + 4] = 3;
  const std::vector<PartitionLabel> input = labels;
  EXPECT_THROW(smoothPartitionBorders(blocked, {}, labels, 2, -1, 3),
               std::invalid_argument);
  EXPECT_THROW(smoothPartitionBorders(blocked, {}, labels, 2,
                                      std::numeric_limits<int>::min(), 3),
               std::invalid_argument);
  EXPECT_THROW(smoothPartitionBorders(blocked, {}, labels, 2, 1, -1),
               std::invalid_argument);
  EXPECT_EQ(labels, input);
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
  smoothPartitionBorders(blocked, {}, labels, FIRST_PARTITION_LABEL, 1, 3);
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
  smoothPartitionBorders(blocked, {}, labels, 2, 1, 3);
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
  smoothPartitionBorders(blocked, {}, labels, 2, 1, 3);
  EXPECT_EQ(labels, (std::vector<PartitionLabel>{2, 2, 2, 3, 2, 3, 3, 2, 3}));
}

TEST(Watershed, EarlierRunsNeitherVoteNorChangeInTheMajorityFilter) {
  // The run starts at label 3, and the cells of label 2 belong to an earlier
  // run. Four of the five cells of the run in the window of the center hold
  // label 3, a clear majority. Counted, the four cells of label 2 would leave
  // no label a clear majority.
  const BitGrid blocked(3, 3);
  std::vector<PartitionLabel> labels = {3, 3, 3, 2, 4, 2, 2, 3, 2};
  smoothPartitionBorders(blocked, {}, labels, 3, 1, 3);
  EXPECT_EQ(labels, (std::vector<PartitionLabel>{3, 3, 3, 2, 3, 2, 2, 3, 2}));
}

} // namespace
