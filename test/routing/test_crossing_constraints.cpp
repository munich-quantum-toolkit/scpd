/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/routing/CrossingConstraints.hpp"
#include "mqt-scpd/routing/Heading.hpp"
#include "mqt-scpd/routing/Path.hpp"
#include "mqt-scpd/routing/RoomRules.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace {

using namespace mqt::scpd::routing;

/// A way of cells that runs `steps` cells on each heading of `headings` in
/// turn from (x0, y0); every cell carries the heading of the step into it,
/// the first cell that of the step out of it.
Path wayOf(const uint32_t x0, const uint32_t y0,
           const std::vector<Heading>& headings, const uint32_t steps) {
  Path way;
  way.push_back({.x = x0, .y = y0, .heading = headings.front()});
  for (const auto heading : headings) {
    const auto v = headingVector(heading);
    for (uint32_t step = 0; step < steps; ++step) {
      const auto& last = way.back();
      way.push_back(
          {.x = static_cast<uint32_t>(static_cast<int64_t>(last.x) + v.dx),
           .y = static_cast<uint32_t>(static_cast<int64_t>(last.y) + v.dy),
           .heading = heading});
    }
  }
  return way;
}

/// The masks as the rule states them, cell by cell: a cell within the
/// radius (a disc) of a straight cell carries the bit of that cell's
/// heading; a cell within one cell of a bend or of the first and last
/// `pinCells` cells of a way may not be entered.
std::vector<uint8_t> masksByRule(const uint32_t width, const uint32_t height,
                                 const std::vector<Path>& ways,
                                 const int radius, const std::size_t pinCells) {
  std::vector<uint8_t> masks(static_cast<std::size_t>(width) * height, 0);
  std::vector<bool> blocked(masks.size(), false);
  const auto inside = [&](const int64_t x, const int64_t y) {
    return x >= 0 && y >= 0 && x < width && y < height;
  };
  const auto block = [&](const PathPoint& cell) {
    for (int64_t dy = -1; dy <= 1; ++dy) {
      for (int64_t dx = -1; dx <= 1; ++dx) {
        const auto x = static_cast<int64_t>(cell.x) + dx;
        const auto y = static_cast<int64_t>(cell.y) + dy;
        if (inside(x, y)) {
          blocked[(static_cast<std::size_t>(y) * width) +
                  static_cast<std::size_t>(x)] = true;
        }
      }
    }
  };
  for (const auto& way : ways) {
    const auto straight = straightCells(way);
    for (std::size_t at = 0; at < way.size(); ++at) {
      const auto& cell = way[at];
      if (!straight[at]) {
        block(cell);
        continue;
      }
      for (int64_t dy = -radius; dy <= radius; ++dy) {
        for (int64_t dx = -radius; dx <= radius; ++dx) {
          const auto x = static_cast<int64_t>(cell.x) + dx;
          const auto y = static_cast<int64_t>(cell.y) + dy;
          if ((dx * dx) + (dy * dy) <= static_cast<int64_t>(radius) * radius &&
              inside(x, y)) {
            masks[(static_cast<std::size_t>(y) * width) +
                  static_cast<std::size_t>(x)] |=
                static_cast<uint8_t>(1U << (cell.heading & 7U));
          }
        }
      }
    }
    for (std::size_t at = 0; at < way.size(); ++at) {
      if (at < pinCells || at + pinCells >= way.size()) {
        block(way[at]);
      }
    }
  }
  for (std::size_t cell = 0; cell < masks.size(); ++cell) {
    if (blocked[cell]) {
      masks[cell] = CrossingConstraints::CURVE_ZONE;
    }
  }
  return masks;
}

} // namespace

// Straight runs on all eight headings, bends of an eighth and a quarter
// turn, two ways that overlap and ways that reach past the grid's edges:
// every cell carries the mask the rule states.
TEST(CrossingConstraints, MasksFollowTheRuleCellByCell) {
  constexpr uint32_t WIDTH = 90;
  constexpr uint32_t HEIGHT = 80;
  constexpr int RADIUS = 6;
  constexpr std::size_t PIN_CELLS = 3;
  const std::vector<Path> ways{
      wayOf(40, 30, {6, 5, 4, 3, 2, 1, 0, 7}, 7),
      wayOf(2, 3, {6, 5, 4, 6}, 9),
      wayOf(60, 70, {1, 2, 0, 7}, 8),
      wayOf(88, 5, {4, 3, 2}, 12),
  };
  CrossingConstraints rule;
  rule.build(WIDTH, HEIGHT, ways, {}, RADIUS, PIN_CELLS);
  const auto expected = masksByRule(WIDTH, HEIGHT, ways, RADIUS, PIN_CELLS);
  for (uint32_t y = 0; y < HEIGHT; ++y) {
    for (uint32_t x = 0; x < WIDTH; ++x) {
      ASSERT_EQ(rule.maskAt(x, y),
                expected[(static_cast<std::size_t>(y) * WIDTH) + x])
          << "cell (" << x << ", " << y << ")";
    }
  }
}

// A way left out by `skip` marks nothing.
TEST(CrossingConstraints, SkippedWayMarksNothing) {
  CrossingConstraints rule;
  rule.build(40, 40, {wayOf(5, 20, {2}, 30)}, {true}, 5, 2);
  for (uint32_t y = 0; y < 40; ++y) {
    for (uint32_t x = 0; x < 40; ++x) {
      EXPECT_EQ(rule.maskAt(x, y), 0U);
    }
  }
}
