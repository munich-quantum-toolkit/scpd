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
#include "mqt-scpd/grid/Bottlenecks.hpp"
#include "mqt-scpd/grid/DistanceTransform.hpp"
#include "mqt-scpd/grid/ProfileCuts.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <utility>
#include <vector>

namespace {

using namespace mqt::scpd::grid;

/// A small chip: walls painted as rectangles, each static or dynamic.
struct Chip {
  std::uint32_t width = 120;
  std::uint32_t height = 80;
  std::vector<std::uint32_t> wallOf =
      std::vector<std::uint32_t>(static_cast<std::size_t>(120) * 80, NO_WALL);
  std::vector<ProfileRole> roles;
  std::vector<std::uint32_t> runs;
  BitGrid slots = BitGrid(120, 80);
  std::vector<ProfileLine> lines;
  std::vector<ProfileMouth> mouths;
  std::vector<NearestBlocked> field;
  std::vector<std::size_t> sideOf;

  std::uint32_t wall(const ProfileRole role,
                     const std::uint32_t run = NO_WALL) {
    roles.push_back(role);
    runs.push_back(run);
    return static_cast<std::uint32_t>(roles.size() - 1);
  }
  void paint(const std::uint32_t wall, const int x0, const int y0, const int x1,
             const int y1) {
    for (int y = y0; y <= y1; ++y) {
      for (int x = x0; x <= x1; ++x) {
        wallOf[cell(x, y)] = wall;
      }
    }
  }
  [[nodiscard]] std::size_t cell(const int x, const int y) const {
    return (static_cast<std::size_t>(y) * width) + static_cast<std::size_t>(x);
  }
  std::vector<Bottleneck> cuts(const ProfileOptions& options = {
                                   .widest = 60.0,
                                   .minimumRise = 2.0,
                                   .sameNarrowing = 10.0,
                                   .narrowest = 2.0}) {
    // One field per side: every static wall lies in the field `sideOf`
    // names, the first by default.
    std::size_t sides = 1;
    for (const auto side : sideOf) {
      sides = std::max(sides, side + 1);
    }
    std::vector<BitGrid> statics(sides, BitGrid(width, height));
    for (std::size_t c = 0; c < wallOf.size(); ++c) {
      const auto wall = wallOf[c];
      if (wall != NO_WALL && roles[wall] == ProfileRole::Static) {
        statics[wall < sideOf.size() ? sideOf[wall] : 0].set(c);
      }
    }
    field.clear();
    for (const auto& mask : statics) {
      field.push_back(nearestBlocked(mask));
    }
    std::vector<const NearestBlocked*> fields;
    for (const auto& one : field) {
      fields.push_back(&one);
    }
    return profileCuts({.width = width,
                        .height = height,
                        .wallOf = wallOf,
                        .roles = roles,
                        .runs = runs,
                        .fields = fields,
                        .slots = &slots,
                        .lines = lines,
                        .mouths = mouths},
                       options);
  }
  [[nodiscard]] int x(const std::size_t c) const {
    return static_cast<int>(c % width);
  }
  [[nodiscard]] int y(const std::size_t c) const {
    return static_cast<int>(c / width);
  }
};

/// The cuts between two walls.
std::vector<Bottleneck> between(const Chip& chip,
                                const std::vector<Bottleneck>& cuts,
                                const std::uint32_t a, const std::uint32_t b) {
  std::vector<Bottleneck> found;
  for (const auto& cut : cuts) {
    const auto one = chip.wallOf[cut.first];
    const auto two = chip.wallOf[cut.second];
    if ((one == a && two == b) || (one == b && two == a)) {
      found.push_back(cut);
    }
  }
  return found;
}

/// A bar, a static floor with a peak: one cut, at the peak.
Chip barOverPeak() {
  Chip chip;
  const auto floor = chip.wall(ProfileRole::Static);
  const auto bar = chip.wall(ProfileRole::Dynamic);
  chip.paint(floor, 0, 60, 119, 79);
  for (int k = 0; k <= 20; ++k) {
    chip.paint(floor, 60 - k, 40 + k, 60 + k, 40 + k);
  }
  chip.paint(bar, 10, 20, 110, 24);
  return chip;
}

TEST(ProfileCuts, CutsAtTheNarrowestPlaceOfAGap) {
  auto chip = barOverPeak();
  const auto cuts = between(chip, chip.cuts(), 0, 1);
  ASSERT_EQ(cuts.size(), 1U);
  const auto onBar =
      chip.wallOf[cuts[0].first] == 1 ? cuts[0].first : cuts[0].second;
  const auto onFloor = onBar == cuts[0].first ? cuts[0].second : cuts[0].first;
  EXPECT_NEAR(chip.x(onBar), 60, 2);
  EXPECT_EQ(chip.y(onBar), 24);
  EXPECT_EQ(chip.x(onFloor), 60);
  EXPECT_EQ(chip.y(onFloor), 40);
}

TEST(ProfileCuts, CutsAStretchOfOneWidthOnceInItsMiddle) {
  Chip chip;
  const auto floor = chip.wall(ProfileRole::Static);
  const auto bar = chip.wall(ProfileRole::Dynamic);
  chip.paint(floor, 0, 40, 119, 79);
  chip.paint(bar, 20, 20, 100, 24);
  const auto cuts = between(chip, chip.cuts(), floor, bar);
  ASSERT_EQ(cuts.size(), 1U);
  const auto onBar =
      chip.wallOf[cuts[0].first] == bar ? cuts[0].first : cuts[0].second;
  EXPECT_NEAR(chip.x(onBar), 60, 3);
}

TEST(ProfileCuts, LeavesAGapWiderThanTheWidestAlone) {
  auto chip = barOverPeak();
  EXPECT_TRUE(between(chip,
                      chip.cuts({.widest = 10.0,
                                 .minimumRise = 2.0,
                                 .sameNarrowing = 10.0,
                                 .narrowest = 2.0}),
                      0, 1)
                  .empty());
}

TEST(ProfileCuts, CutsNoLineThroughAnotherWall) {
  auto chip = barOverPeak();
  const auto block = chip.wall(ProfileRole::Dynamic);
  chip.paint(block, 50, 30, 70, 33);
  const auto cuts = chip.cuts();
  for (const auto& cut : between(chip, cuts, 0, 1)) {
    const auto onBar = chip.wallOf[cut.first] == 1 ? cut.first : cut.second;
    EXPECT_FALSE(std::abs(chip.x(onBar) - 60) <= 12)
        << "a cut runs through the block";
  }
  EXPECT_FALSE(between(chip, cuts, block, 0).empty());
}

TEST(ProfileCuts, MeasuresTwoInflatedLinesBetweenTheirLines) {
  Chip chip;
  const auto upper = chip.wall(ProfileRole::Dynamic);
  const auto lower = chip.wall(ProfileRole::Dynamic);
  // The upper line runs straight at y = 20; the lower comes up to y = 36 at
  // x = 60 and falls away to both sides. Both are 2 cells to either side.
  ProfileLine top{
      .wall = upper, .points = {{10, 20}, {110, 20}}, .radius = 2.0};
  ProfileLine bottom{
      .wall = lower, .points = {{10, 56}, {60, 36}, {110, 56}}, .radius = 2.0};
  const auto paintLine = [&chip](const ProfileLine& line) {
    for (std::size_t k = 0; k + 1 < line.points.size(); ++k) {
      const auto [ax, ay] = line.points[k];
      const auto [bx, by] = line.points[k + 1];
      for (int s = 0; s <= 200; ++s) {
        const auto t = s / 200.0;
        const auto cx = static_cast<int>(std::lround(ax + (t * (bx - ax))));
        const auto cy = static_cast<int>(std::lround(ay + (t * (by - ay))));
        chip.paint(line.wall, cx - 2, cy - 2, cx + 2, cy + 2);
      }
    }
  };
  paintLine(top);
  paintLine(bottom);
  chip.lines = {top, bottom};
  const auto cuts = between(chip, chip.cuts(), upper, lower);
  ASSERT_EQ(cuts.size(), 1U);
  const auto onTop =
      chip.wallOf[cuts[0].first] == upper ? cuts[0].first : cuts[0].second;
  const auto onBottom = onTop == cuts[0].first ? cuts[0].second : cuts[0].first;
  EXPECT_NEAR(chip.x(onTop), 60, 2);
  EXPECT_EQ(chip.y(onTop), 22);
  EXPECT_EQ(chip.y(onBottom), 34);
}

TEST(ProfileCuts, ClosesBothSidesOfASlotMouth) {
  Chip chip;
  const auto bar = chip.wall(ProfileRole::Dynamic);
  constexpr std::uint32_t RUN = 7;
  const auto left = chip.wall(ProfileRole::Static, RUN);
  const auto right = chip.wall(ProfileRole::Static, RUN);
  const auto floor = chip.wall(ProfileRole::Static);
  chip.paint(bar, 10, 10, 110, 14);
  chip.paint(floor, 0, 70, 119, 79);
  // A run from the floor up to y = 50: two flanks with the slot between.
  chip.paint(left, 54, 50, 54, 69);
  chip.paint(right, 66, 50, 66, 69);
  for (int y = 50; y <= 69; ++y) {
    for (int x = 55; x <= 65; ++x) {
      chip.slots.set(chip.cell(x, y));
    }
  }
  chip.mouths = {{.flank = left,
                  .tip = chip.cell(54, 50),
                  .towardX = 0.0,
                  .towardY = -1.0,
                  .staticClearance = 100.0},
                 {.flank = right,
                  .tip = chip.cell(66, 50),
                  .towardX = 0.0,
                  .towardY = -1.0,
                  .staticClearance = 100.0}};
  const auto cuts = chip.cuts();
  EXPECT_EQ(between(chip, cuts, left, bar).size(), 1U);
  EXPECT_EQ(between(chip, cuts, right, bar).size(), 1U);
  for (const auto& cut : cuts) {
    EXPECT_FALSE(chip.slots.test(cut.saddle));
  }
}

TEST(ProfileCuts, CutsAThinWallToTheWallOnEachSide) {
  Chip chip;
  const auto ceiling = chip.wall(ProfileRole::Static);
  const auto floor = chip.wall(ProfileRole::Static);
  const auto line = chip.wall(ProfileRole::Dynamic);
  chip.sideOf = {0, 1, 0};
  // A ceiling that dips to y = 15 at x = 60, a floor that rises to y = 44
  // there, and a line one cell thick at y = 30 between them.
  chip.paint(ceiling, 0, 0, 119, 5);
  chip.paint(floor, 0, 70, 119, 79);
  for (int k = 0; k <= 10; ++k) {
    chip.paint(ceiling, 60 - k, 5, 60 + k, 15 - k);
    chip.paint(floor, 60 - (2 * k), 44 + k, 60 + (2 * k), 70);
  }
  chip.paint(line, 15, 30, 105, 30);
  const auto cuts = chip.cuts();
  const auto up = between(chip, cuts, line, ceiling);
  const auto down = between(chip, cuts, line, floor);
  ASSERT_EQ(up.size(), 1U);
  ASSERT_EQ(down.size(), 1U);
  const auto onLine = [&](const Bottleneck& cut) {
    return chip.wallOf[cut.first] == line ? cut.first : cut.second;
  };
  EXPECT_NEAR(chip.x(onLine(up[0])), 60, 2);
  EXPECT_NEAR(chip.x(onLine(down[0])), 60, 2);
}

TEST(ProfileCuts, CutsNothingInTheWedgeWhereTwoWallsTouch) {
  Chip chip;
  const auto one = chip.wall(ProfileRole::Dynamic);
  const auto two = chip.wall(ProfileRole::Dynamic);
  // Two arms of a V that meet at the bottom.
  for (int k = 0; k <= 40; ++k) {
    chip.paint(one, 60 - k - 1, 70 - k, 60 - k, 70 - k);
    chip.paint(two, 61 + k, 70 - k, 62 + k, 70 - k);
  }
  EXPECT_TRUE(between(chip, chip.cuts(), one, two).empty());
}

} // namespace
