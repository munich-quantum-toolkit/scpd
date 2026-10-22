/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// Chambers: the free space cut by bottlenecks, on masks whose rooms the test
// knows.

#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/grid/Bottlenecks.hpp"
#include "mqt-scpd/grid/Chambers.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <queue>
#include <random>
#include <utility>
#include <vector>

namespace mqt::scpd::grid {
namespace {

constexpr std::uint32_t WIDTH = 61;
constexpr std::uint32_t HEIGHT = 41;
constexpr std::uint32_t MIDDLE = HEIGHT / 2;

GridMetrics unitGrid() {
  return GridMetrics::fit({.minX = 0.0,
                           .minY = 0.0,
                           .maxX = static_cast<double>(WIDTH - 1),
                           .maxY = static_cast<double>(HEIGHT - 1)},
                          WIDTH, HEIGHT);
}

std::size_t at(const std::uint32_t x, const std::uint32_t y) {
  return (static_cast<std::size_t>(y) * WIDTH) + x;
}

/// A corridor along x, free on the rows MIDDLE - 5 to MIDDLE + 5.
BitGrid corridor() {
  BitGrid mask(WIDTH, HEIGHT);
  for (std::uint32_t y = 0; y < HEIGHT; ++y) {
    for (std::uint32_t x = 0; x < WIDTH; ++x) {
      if (y + 5 < MIDDLE || y > MIDDLE + 5) {
        mask.setCell(x, y);
      }
    }
  }
  return mask;
}

TEST(Chambers, AGateAcrossACorridorCutsItInTwo) {
  const auto mask = corridor();
  const std::vector<Bottleneck> gates = {{.first = at(30, MIDDLE - 6),
                                          .second = at(30, MIDDLE + 6),
                                          .saddle = at(30, MIDDLE)}};
  const auto chambers = chambersOf(mask, gates, unitGrid());

  EXPECT_EQ(chambers.count, 2U);
  const auto left = chambers.of[at(10, MIDDLE)];
  const auto right = chambers.of[at(50, MIDDLE)];
  EXPECT_NE(left, NO_CHAMBER);
  EXPECT_NE(right, NO_CHAMBER);
  EXPECT_NE(left, right);
  ASSERT_EQ(chambers.beside.size(), 1U);
  EXPECT_EQ(chambers.beside[0],
            (std::vector<std::uint32_t>{std::min(left, right),
                                        std::max(left, right)}));
}

TEST(Chambers, TwoGatesWhoseLinesTouchEachOpenOntoBothSides) {
  // Two gates across the corridor in the columns 30 and 31: no cell lies
  // between their lines, and each sees the other's line on one side. Each
  // opens all the same onto the chamber behind that line.
  const auto mask = corridor();
  const std::vector<Bottleneck> gates = {{.first = at(30, MIDDLE - 6),
                                          .second = at(30, MIDDLE + 6),
                                          .saddle = at(30, MIDDLE)},
                                         {.first = at(31, MIDDLE - 6),
                                          .second = at(31, MIDDLE + 6),
                                          .saddle = at(31, MIDDLE)}};
  const auto chambers = chambersOf(mask, gates, unitGrid());

  const auto left = chambers.of[at(10, MIDDLE)];
  const auto right = chambers.of[at(50, MIDDLE)];
  ASSERT_NE(left, NO_CHAMBER);
  ASSERT_NE(right, NO_CHAMBER);
  ASSERT_NE(left, right);
  const std::vector<std::uint32_t> both{std::min(left, right),
                                        std::max(left, right)};
  ASSERT_EQ(chambers.beside.size(), 2U);
  EXPECT_EQ(chambers.beside[0], both);
  EXPECT_EQ(chambers.beside[1], both);
}

TEST(Chambers, AGateWithAWayAroundItCutsNothing) {
  // An island in the middle of the corridor, and a gate from it to the
  // upper wall: the free space still runs around the island below.
  auto mask = corridor();
  for (std::uint32_t x = 28; x <= 32; ++x) {
    for (std::uint32_t y = MIDDLE - 1; y <= MIDDLE + 1; ++y) {
      mask.setCell(x, y);
    }
  }
  const std::vector<Bottleneck> gates = {{.first = at(30, MIDDLE + 1),
                                          .second = at(30, MIDDLE + 6),
                                          .saddle = at(30, MIDDLE + 3)}};
  const auto chambers = chambersOf(mask, gates, unitGrid());

  EXPECT_EQ(chambers.count, 1U);
  ASSERT_EQ(chambers.beside.size(), 1U);
  EXPECT_EQ(chambers.beside[0].size(), 1U);
}

TEST(Chambers, WithoutGatesEveryPieceOfFreeSpaceIsAChamber) {
  // A wall across the corridor splits it without any gate.
  auto mask = corridor();
  for (std::uint32_t y = 0; y < HEIGHT; ++y) {
    mask.setCell(30, y);
  }
  const auto chambers = chambersOf(mask, {}, unitGrid());

  EXPECT_EQ(chambers.count, 2U);
  EXPECT_EQ(chambers.of[at(30, MIDDLE)], NO_CHAMBER);
  EXPECT_NE(chambers.of[at(10, MIDDLE)], chambers.of[at(50, MIDDLE)]);
  EXPECT_TRUE(chambers.beside.empty());
}

TEST(Chambers, ADiagonalStepBetweenTwoWallCornersIsNoPassage) {
  // Two walls that touch only at a corner close the corridor.
  auto mask = corridor();
  for (std::uint32_t y = 0; y <= MIDDLE; ++y) {
    mask.setCell(30, y);
  }
  for (std::uint32_t y = MIDDLE + 1; y < HEIGHT; ++y) {
    mask.setCell(31, y);
  }
  const auto chambers = chambersOf(mask, {}, unitGrid());

  EXPECT_EQ(chambers.count, 2U);
}

/// The chambers by a flood from every free cell in turn, in row order: the
/// definition the labelling has to match, cell for cell.
std::vector<std::uint32_t> floodedChambers(const BitGrid& blocked,
                                           const std::vector<Bottleneck>& gates,
                                           const GridMetrics& grid) {
  const auto width = static_cast<std::int64_t>(grid.width);
  const auto height = static_cast<std::int64_t>(grid.height);
  std::vector<std::uint8_t> refused(grid.cells(), 0);
  for (const auto& gate : bottleneckMoves(gates, grid)) {
    for (const auto& move : gate) {
      refused[move.cell] |= static_cast<std::uint8_t>(1U << move.direction);
    }
  }
  const auto onALine = [&](const std::int64_t x, const std::int64_t y) {
    const auto cell = static_cast<std::size_t>((y * width) + x);
    for (std::size_t step = 0; step < STEP_DX.size(); ++step) {
      const auto nx = x + STEP_DX[step];
      const auto ny = y + STEP_DY[step];
      if (nx >= 0 && ny >= 0 && nx < width && ny < height &&
          (refused[cell] & (1U << step)) == 0U) {
        return false;
      }
    }
    return true;
  };
  std::vector<std::uint32_t> of(grid.cells(), NO_CHAMBER);
  std::uint32_t count = 0;
  for (std::size_t start = 0; start < of.size(); ++start) {
    if (blocked.test(start) || of[start] != NO_CHAMBER ||
        onALine(static_cast<std::int64_t>(start) % width,
                static_cast<std::int64_t>(start) / width)) {
      continue;
    }
    const auto label = count++;
    of[start] = label;
    std::queue<std::size_t> pending;
    pending.push(start);
    while (!pending.empty()) {
      const auto current = pending.front();
      pending.pop();
      const auto cx = static_cast<std::int64_t>(current) % width;
      const auto cy = static_cast<std::int64_t>(current) / width;
      for (std::size_t step = 0; step < STEP_DX.size(); ++step) {
        const auto nx = cx + STEP_DX[step];
        const auto ny = cy + STEP_DY[step];
        if (nx < 0 || ny < 0 || nx >= width || ny >= height ||
            (refused[current] & (1U << step)) != 0U) {
          continue;
        }
        const auto next = static_cast<std::size_t>((ny * width) + nx);
        if (blocked.test(next) || of[next] != NO_CHAMBER) {
          continue;
        }
        if (step >= 4 &&
            blocked.test(static_cast<std::size_t>((cy * width) + nx)) &&
            blocked.test(static_cast<std::size_t>((ny * width) + cx))) {
          continue;
        }
        of[next] = label;
        pending.push(next);
      }
    }
  }
  return of;
}

TEST(Chambers, ABottleneckRefusesEveryStepAcrossItsLine) {
  std::mt19937 rng(11);
  std::uniform_int_distribution<std::uint32_t> column(0, WIDTH - 1);
  std::uniform_int_distribution<std::uint32_t> row(0, HEIGHT - 1);
  const auto grid = unitGrid();
  for (int k = 0; k < 200; ++k) {
    const Bottleneck gate{.first = at(column(rng), row(rng)),
                          .second = at(column(rng), row(rng)),
                          .saddle = 0};
    const auto x0 = static_cast<double>(gate.first % WIDTH) + 0.5;
    const auto y0 = static_cast<double>(gate.first / WIDTH) + 0.5;
    const auto x1 = static_cast<double>(gate.second % WIDTH) + 0.5;
    const auto y1 = static_cast<double>(gate.second / WIDTH) + 0.5;
    const auto side = [&](const double px, const double py) {
      return ((x1 - x0) * (py - y0)) - ((y1 - y0) * (px - x0));
    };
    std::vector<std::pair<std::size_t, std::size_t>> refused;
    const auto moves = bottleneckMoves({gate}, grid);
    for (const auto& move : moves[0]) {
      refused.emplace_back(move.cell, move.direction);
    }
    std::ranges::sort(refused);
    // Every step whose two cell centres lie strictly on either side of the
    // line, and crosses it between its ends, is refused both ways.
    for (std::uint32_t y = 0; y < HEIGHT; ++y) {
      for (std::uint32_t x = 0; x < WIDTH; ++x) {
        for (std::size_t step = 0; step < STEP_DX.size(); ++step) {
          const auto nx = static_cast<std::int64_t>(x) + STEP_DX[step];
          const auto ny = static_cast<std::int64_t>(y) + STEP_DY[step];
          if (nx < 0 || ny < 0 || nx >= WIDTH || ny >= HEIGHT) {
            continue;
          }
          const auto ax = x + 0.5;
          const auto ay = y + 0.5;
          const auto bx = static_cast<double>(nx) + 0.5;
          const auto by = static_cast<double>(ny) + 0.5;
          const auto s1 = side(ax, ay);
          const auto s2 = side(bx, by);
          const auto t1 = ((bx - ax) * (y0 - ay)) - ((by - ay) * (x0 - ax));
          const auto t2 = ((bx - ax) * (y1 - ay)) - ((by - ay) * (x1 - ax));
          if (((s1 > 0 && s2 < 0) || (s1 < 0 && s2 > 0)) &&
              ((t1 > 0 && t2 < 0) || (t1 < 0 && t2 > 0))) {
            EXPECT_TRUE(
                std::ranges::binary_search(refused, std::pair{at(x, y), step}))
                << "gate " << k << " cell " << x << "," << y << " step "
                << step;
          }
        }
      }
    }
  }
}

TEST(Chambers, LabelEveryCellAsAFloodFromTheLowestCellWould) {
  for (std::uint32_t seed = 1; seed <= 20; ++seed) {
    std::mt19937 rng(seed);
    std::bernoulli_distribution wall(seed % 3 == 0   ? 0.35
                                     : seed % 3 == 1 ? 0.12
                                                     : 0.03);
    std::uniform_int_distribution<std::uint32_t> column(0, WIDTH - 1);
    std::uniform_int_distribution<std::uint32_t> row(0, HEIGHT - 1);
    BitGrid mask(WIDTH, HEIGHT);
    for (std::size_t cell = 0; cell < mask.size(); ++cell) {
      if (wall(rng)) {
        mask.set(cell);
      }
    }
    std::vector<Bottleneck> gates;
    for (int k = 0; k < 30; ++k) {
      const auto first = at(column(rng), row(rng));
      const auto second = at(column(rng), row(rng));
      mask.set(first);
      mask.set(second);
      gates.push_back({.first = first, .second = second, .saddle = first});
    }
    const auto expected = floodedChambers(mask, gates, unitGrid());
    const auto chambers = chambersOf(mask, gates, unitGrid());
    EXPECT_EQ(chambers.of, expected) << "seed " << seed;
    std::uint32_t count = 0;
    for (const auto label : expected) {
      if (label != NO_CHAMBER) {
        count = std::max(count, label + 1);
      }
    }
    EXPECT_EQ(chambers.count, count) << "seed " << seed;
  }
}

/// The same on a grid whose rows span several words of the masks and start
/// inside a word: the labelling scans the rows 64 cells at a time.
TEST(Chambers, LabelEveryCellOfAWideGridAsAFloodWould) {
  constexpr std::uint32_t WIDE = 200;
  constexpr std::uint32_t ROWS = 37;
  const auto grid = GridMetrics::fit({.minX = 0.0,
                                      .minY = 0.0,
                                      .maxX = static_cast<double>(WIDE - 1),
                                      .maxY = static_cast<double>(ROWS - 1)},
                                     WIDE, ROWS);
  for (std::uint32_t seed = 1; seed <= 20; ++seed) {
    std::mt19937 rng(seed);
    std::bernoulli_distribution wall(seed % 3 == 0   ? 0.35
                                     : seed % 3 == 1 ? 0.12
                                                     : 0.03);
    std::uniform_int_distribution<std::uint32_t> column(0, WIDE - 1);
    std::uniform_int_distribution<std::uint32_t> row(0, ROWS - 1);
    BitGrid mask(WIDE, ROWS);
    for (std::size_t cell = 0; cell < mask.size(); ++cell) {
      if (wall(rng)) {
        mask.set(cell);
      }
    }
    std::vector<Bottleneck> gates;
    for (int k = 0; k < 60; ++k) {
      const auto y0 = static_cast<std::size_t>(row(rng));
      const auto first = (y0 * WIDE) + column(rng);
      const auto y1 = static_cast<std::size_t>(row(rng));
      const auto second = (y1 * WIDE) + column(rng);
      mask.set(first);
      mask.set(second);
      gates.push_back({.first = first, .second = second, .saddle = first});
    }
    const auto expected = floodedChambers(mask, gates, grid);
    const auto chambers = chambersOf(mask, gates, grid);
    EXPECT_EQ(chambers.of, expected) << "seed " << seed;
  }
}

} // namespace
} // namespace mqt::scpd::grid
