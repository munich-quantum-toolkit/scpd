/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/flatbuffers/geometry.hpp"
#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"
#include "mqt-scpd/grid/Rasterize.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

namespace {

using namespace mqt::scpd::grid;
using mqt::scpd::flatbuffers::design::ChipT;
using mqt::scpd::flatbuffers::geometry::PolygonT;

/// A chip with the given obstacles and nothing else.
ChipT chipWith(const std::vector<std::vector<Point>>& obstacles) {
  ChipT chip;
  for (const auto& vertices : obstacles) {
    auto polygon = std::make_unique<PolygonT>();
    polygon->vertices = vertices;
    chip.obstacles.push_back(std::move(polygon));
  }
  return chip;
}

/// One cell per layout unit over 0..100 in both axes.
const GridMetrics& unitGrid() {
  static const GridMetrics GRID = GridMetrics::fit(
      BoundingBox{.minX = 0.0, .minY = 0.0, .maxX = 100.0, .maxY = 100.0}, 101,
      101);
  return GRID;
}

/// A square obstacle from 20 to 40 in both axes.
const std::vector<Point>& square() {
  static const std::vector<Point> SQUARE = {
      Point(20.0, 20.0), Point(40.0, 20.0), Point(40.0, 40.0),
      Point(20.0, 40.0)};
  return SQUARE;
}

TEST(Rasterize, FillsEveryPolygonTheFirstIncluded) {
  const RasterizedObstacles raster =
      rasterizeObstacles(chipWith({square()}), unitGrid());
  const BitGrid& blocked = raster.blocked;
  EXPECT_TRUE(blocked.testCell(30, 30));
  EXPECT_TRUE(blocked.testCell(20, 20));
  EXPECT_TRUE(blocked.testCell(40, 40));
  EXPECT_FALSE(blocked.testCell(41, 30));
  EXPECT_FALSE(blocked.testCell(19, 30));
  EXPECT_FALSE(blocked.testCell(0, 0));
  EXPECT_FALSE(blocked.testCell(100, 100));
  EXPECT_EQ(blocked.count(), 21U * 21U);
  EXPECT_EQ(raster.keepoutCells, 0U);
}

TEST(Rasterize, ThinPolygonsKeepTheirEdges) {
  // A diagonal sliver narrower than a cell still blocks a connected line.
  const std::vector<Point> sliver = {Point(10.0, 10.0), Point(60.0, 60.1),
                                     Point(60.0, 59.9)};
  const BitGrid blocked =
      rasterizeObstacles(chipWith({sliver}), unitGrid()).blocked;
  // The two end cells have one blocked neighbor each and count as islands.
  for (uint32_t i = 11; i <= 59; ++i) {
    EXPECT_TRUE(blocked.testCell(i, i)) << i;
  }
  EXPECT_FALSE(blocked.testCell(10, 10));
}

TEST(Rasterize, IslandsAndBordersAreHandled) {
  BitGrid mask(10, 10);
  mask.setCell(5, 5);
  mask.setCell(2, 2);
  mask.setCell(3, 2);
  mask.setCell(2, 3);
  mask.setCell(3, 3);
  mask.setCell(7, 7);
  mask.setCell(8, 8);
  removeIslands(mask);
  // A lone cell and a pair are islands; a block of four stays.
  EXPECT_FALSE(mask.testCell(5, 5));
  EXPECT_FALSE(mask.testCell(7, 7));
  EXPECT_FALSE(mask.testCell(8, 8));
  EXPECT_TRUE(mask.testCell(2, 2));
  EXPECT_TRUE(mask.testCell(3, 3));

  blockBorder(mask, 2, 2);
  EXPECT_TRUE(mask.testCell(0, 5));
  EXPECT_TRUE(mask.testCell(1, 5));
  EXPECT_FALSE(mask.testCell(2, 5));
  EXPECT_TRUE(mask.testCell(5, 9));
  EXPECT_TRUE(mask.testCell(5, 8));
  EXPECT_FALSE(mask.testCell(5, 7));
}

TEST(Rasterize, TheKeepoutIsAnExactDistanceAroundTheEdges) {
  RasterOptions options;
  options.keepout = 5.0;
  const RasterizedObstacles raster =
      rasterizeObstacles(chipWith({square()}), unitGrid(), options);
  const BitGrid& blocked = raster.blocked;
  EXPECT_TRUE(blocked.testCell(15, 30));
  EXPECT_FALSE(blocked.testCell(14, 30));
  EXPECT_TRUE(blocked.testCell(45, 30));
  EXPECT_TRUE(blocked.testCell(30, 45));
  EXPECT_FALSE(blocked.testCell(30, 46));
  // A corner: the distance is to the vertex, not to the box.
  EXPECT_FALSE(blocked.testCell(16, 16));
  EXPECT_TRUE(blocked.testCell(17, 17));
  EXPECT_GT(raster.keepoutCells, 0U);
  EXPECT_EQ(raster.exemptedCells, 0U);

  // The border of the grid is blocked with the keepout, in one pass.
  options.borderX = 3;
  options.borderY = 3;
  const BitGrid bordered =
      rasterizeObstacles(chipWith({square()}), unitGrid(), options).blocked;
  EXPECT_TRUE(bordered.testCell(0, 50));
  EXPECT_TRUE(bordered.testCell(2, 50));
  EXPECT_FALSE(bordered.testCell(3, 50));
  EXPECT_TRUE(bordered.testCell(50, 100));
  EXPECT_TRUE(bordered.testCell(50, 98));
  EXPECT_FALSE(bordered.testCell(50, 97));
}

TEST(Rasterize, ACorridorReleasesKeepoutCellsButNeverPolygonCells) {
  RasterOptions options;
  options.keepout = 5.0;
  options.keepoutExemptions.push_back(
      {.from = Point(30.0, 40.0), .to = Point(30.0, 60.0), .halfWidth = 2.0});
  const RasterizedObstacles raster =
      rasterizeObstacles(chipWith({square()}), unitGrid(), options);
  const BitGrid& blocked = raster.blocked;
  EXPECT_FALSE(blocked.testCell(30, 43));
  EXPECT_FALSE(blocked.testCell(32, 43));
  EXPECT_TRUE(blocked.testCell(33, 43));
  EXPECT_TRUE(blocked.testCell(30, 40));
  EXPECT_TRUE(blocked.testCell(30, 39));
  EXPECT_GT(raster.exemptedCells, 0U);
}

TEST(Rasterize, LineCellsAreConnectedAndClipped) {
  const std::vector<std::size_t> cells = lineCells(0, 0, 4, 2, 10, 10);
  EXPECT_EQ(cells.size(), 5U);
  EXPECT_EQ(cells.front(), 0U);
  EXPECT_EQ(cells.back(), (2U * 10U) + 4U);

  const std::vector<std::size_t> clipped = lineCells(-2, 0, 2, 0, 10, 10);
  EXPECT_EQ(clipped.size(), 3U);

  EXPECT_DOUBLE_EQ(
      distanceToSegment(Point(0.0, 3.0), Point(-5.0, 0.0), Point(5.0, 0.0)),
      3.0);
  EXPECT_DOUBLE_EQ(
      distanceToSegment(Point(8.0, 4.0), Point(-5.0, 0.0), Point(5.0, 0.0)),
      5.0);
  EXPECT_DOUBLE_EQ(
      distanceToSegment(Point(3.0, 4.0), Point(0.0, 0.0), Point(0.0, 0.0)),
      5.0);
}

TEST(Rasterize, MissingAndEmptyObstaclesAddNoCells) {
  RasterOptions options;
  options.keepout = 5.0;
  const RasterizedObstacles alone =
      rasterizeObstacles(chipWith({square()}), unitGrid(), options);

  ChipT chip = chipWith({square(), {}});
  chip.obstacles.push_back(nullptr);
  const RasterizedObstacles raster =
      rasterizeObstacles(chip, unitGrid(), options);
  EXPECT_EQ(raster.blocked, alone.blocked);
  EXPECT_EQ(raster.keepoutCells, alone.keepoutCells);
  EXPECT_EQ(raster.exemptedCells, alone.exemptedCells);
}

TEST(Rasterize, APolygonOfTwoVerticesBlocksOnlyItsEdges) {
  const GridMetrics& grid = unitGrid();
  PolygonT segment;
  segment.vertices = {Point(30.0, 30.0), Point(40.0, 35.0)};
  BitGrid mask(grid.width, grid.height);
  fillPolygon(mask, grid, segment);

  // The edge to the second vertex and the closing edge back to the first.
  BitGrid expected(grid.width, grid.height);
  for (const std::size_t index :
       lineCells(30, 30, 40, 35, grid.width, grid.height)) {
    expected.set(index);
  }
  for (const std::size_t index :
       lineCells(40, 35, 30, 30, grid.width, grid.height)) {
    expected.set(index);
  }
  EXPECT_EQ(mask, expected);
}

TEST(Rasterize, APolygonWithoutVerticesKeepsTheMask) {
  const GridMetrics& grid = unitGrid();
  BitGrid mask(grid.width, grid.height);
  mask.setCell(50, 50);
  const BitGrid before = mask;
  fillPolygon(mask, grid, PolygonT{});
  EXPECT_EQ(mask, before);
}

TEST(Rasterize, IslandRemovalLeavesTheEdgeOfTheMaskAlone) {
  const auto keepsItsCells = [](BitGrid mask) {
    const BitGrid before = mask;
    removeIslands(mask);
    return mask == before;
  };
  // Every cell of a mask two cells across lies on its edge.
  BitGrid narrow(2, 5);
  narrow.setCell(1, 2);
  EXPECT_TRUE(keepsItsCells(narrow));
  BitGrid flat(5, 2);
  flat.setCell(2, 0);
  EXPECT_TRUE(keepsItsCells(flat));
  // A lone cell on the edge of a wider mask stays as well.
  BitGrid wide(10, 10);
  wide.setCell(0, 5);
  wide.setCell(5, 9);
  EXPECT_TRUE(keepsItsCells(wide));
}

} // namespace
