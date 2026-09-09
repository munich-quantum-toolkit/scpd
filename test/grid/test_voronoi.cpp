/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// The medial axis, on masks whose axis is known by construction.

#include "mqt-scpd/geometry/Geometry.hpp"
#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"
#include "mqt-scpd/grid/Voronoi.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace mqt::scpd::grid {
namespace {

/// A grid of the given size over a box one layout unit per cell.
GridMetrics squareGrid(const std::uint32_t width, const std::uint32_t height) {
  return GridMetrics::fit({.minX = 0.0,
                           .minY = 0.0,
                           .maxX = static_cast<double>(width - 1),
                           .maxY = static_cast<double>(height - 1)},
                          width, height);
}

/// A horizontal corridor: two walls with free space between them.
BitGrid corridor(const std::uint32_t width, const std::uint32_t height,
                 const std::uint32_t wallThickness) {
  BitGrid mask(width, height);
  for (std::uint32_t y = 0; y < wallThickness; ++y) {
    for (std::uint32_t x = 0; x < width; ++x) {
      mask.setCell(x, y);
      mask.setCell(x, height - 1 - y);
    }
  }
  return mask;
}

TEST(MedialAxis, RunsAlongTheMiddleOfAStraightCorridor) {
  const auto mask = corridor(40, 21, 3);
  const auto edges = medialAxis(mask);
  ASSERT_FALSE(edges.empty());

  // Every surviving edge has to sit near the centre line of the corridor,
  // which is the row halfway between the two walls.
  constexpr double CENTER = 10.0;
  for (const auto& edge : edges) {
    EXPECT_NEAR(edge.y0, CENTER, 1.5);
    EXPECT_NEAR(edge.y1, CENTER, 1.5);
  }
}

TEST(MedialAxis, DropsTheEdgesOfOneWallAgainstItself) {
  // A single wall across an otherwise empty grid has no free space on two
  // sides of anything, so the same-wall filter must leave nothing between
  // its own boundary cells near it.
  BitGrid mask(30, 30);
  for (std::uint32_t x = 0; x < 30; ++x) {
    mask.setCell(x, 15);
  }

  const auto edges = medialAxis(mask);
  // Whatever survives runs between the wall and the border of the grid, and
  // therefore away from the wall itself rather than along it.
  for (const auto& edge : edges) {
    const auto onWall = std::abs(edge.y0 - 15.0) < 0.5 && std::abs(edge.y1 - 15.0) < 0.5;
    EXPECT_FALSE(onWall);
  }
}

TEST(MedialAxis, ReturnsNothingForAMaskWithoutEnoughWall) {
  EXPECT_TRUE(medialAxis(BitGrid{}).empty());
  EXPECT_TRUE(medialAxis(BitGrid(8, 8)).empty());
}

TEST(MedialAxis, NeverMarksABlockedCell) {
  const auto mask = corridor(40, 21, 3);
  const auto grid = squareGrid(40, 21);
  const auto axis = rasterizeMedialAxis(mask, grid, medialAxis(mask));

  ASSERT_EQ(axis.cells.size(), mask.size());
  for (std::size_t cell = 0; cell < mask.size(); ++cell) {
    if (mask.test(cell)) {
      EXPECT_FALSE(axis.onAxis(cell)) << "cell " << cell;
    }
  }
}

TEST(MedialAxis, JoinsTheCellsOfOneEdgeIntoAPath) {
  const auto mask = corridor(40, 21, 3);
  const auto grid = squareGrid(40, 21);
  const auto axis = rasterizeMedialAxis(mask, grid, medialAxis(mask));

  std::size_t onAxis = 0;
  for (std::size_t cell = 0; cell < mask.size(); ++cell) {
    if (axis.onAxis(cell)) {
      ++onAxis;
      // Every cell of the axis is reachable from another one; an isolated
      // cell would be a hole in the corridor's skeleton.
      EXPECT_FALSE(axis.neighbors.at(cell).empty()) << "cell " << cell;
    }
  }
  EXPECT_GT(onAxis, 10U);

  // Adjacency is symmetric, so a walk along the axis can go either way.
  for (const auto& [cell, neighbors] : axis.neighbors) {
    for (const auto neighbor : neighbors) {
      const auto& back = axis.neighbors.at(neighbor);
      EXPECT_NE(std::ranges::find(back, cell), back.end());
    }
  }
}

TEST(MedialAxis, MarksAJunctionWhereThreeCorridorsMeet) {
  // A T of free space in a solid block: the three arms meet in one place.
  BitGrid mask(41, 41, true);
  for (std::uint32_t x = 5; x < 36; ++x) {
    for (std::uint32_t y = 18; y < 23; ++y) {
      mask.setCell(x, y, false);
    }
  }
  for (std::uint32_t y = 23; y < 36; ++y) {
    for (std::uint32_t x = 18; x < 23; ++x) {
      mask.setCell(x, y, false);
    }
  }

  const auto grid = squareGrid(41, 41);
  const auto axis = rasterizeMedialAxis(mask, grid, medialAxis(mask));

  std::size_t junctions = 0;
  for (const auto state : axis.cells) {
    junctions += state == AxisCell::Junction ? 1U : 0U;
  }
  EXPECT_GE(junctions, 1U);
}

TEST(MedialAxis, RefusesAMaskOfTheWrongSize) {
  const BitGrid mask(10, 10);
  EXPECT_THROW(static_cast<void>(rasterizeMedialAxis(mask, squareGrid(12, 10), {})),
               std::invalid_argument);
}

} // namespace
} // namespace mqt::scpd::grid
