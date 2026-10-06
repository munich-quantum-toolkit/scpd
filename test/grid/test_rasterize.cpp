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
#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/flatbuffers/geometry.hpp"
#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"
#include "mqt-scpd/grid/Rasterize.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <utility>
#include <vector>

namespace {

using namespace mqt::scpd::grid;
using mqt::scpd::flatbuffers::design::ChipT;
using mqt::scpd::flatbuffers::geometry::PolygonT;
using mqt::scpd::test::SplitMix;

/// The Bresenham walk over the whole line, one cell after the other, which
/// keeps the cells on the grid.
std::vector<std::size_t> fullWalk(int64_t x0, int64_t y0, const int64_t x1,
                                  const int64_t y1, const uint32_t width,
                                  const uint32_t height) {
  std::vector<std::size_t> cells;
  const int64_t dx = std::abs(x1 - x0);
  const int64_t dy = -std::abs(y1 - y0);
  const int64_t sx = (x0 < x1) ? 1 : -1;
  const int64_t sy = (y0 < y1) ? 1 : -1;
  int64_t err = dx + dy;
  while (true) {
    if (x0 >= 0 && y0 >= 0 && std::cmp_less(x0, width) &&
        std::cmp_less(y0, height)) {
      cells.push_back((static_cast<std::size_t>(y0) * width) +
                      static_cast<std::size_t>(x0));
    }
    if (x0 == x1 && y0 == y1) {
      break;
    }
    const int64_t e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x0 += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y0 += sy;
    }
  }
  return cells;
}

/// The cells fillPolygon() blocks, found cell by cell: every cell of the
/// polygon's box against every edge, and every edge walked in full.
BitGrid perCellFill(const GridMetrics& grid, const PolygonT& polygon) {
  BitGrid mask(grid.width, grid.height);
  std::vector<Point> nodes;
  int64_t minX = grid.width;
  int64_t maxX = 0;
  int64_t minY = grid.height;
  int64_t maxY = 0;
  for (const auto& vertex : polygon.vertices) {
    const Point node = grid.toCell(vertex);
    nodes.push_back(node);
    minX = std::min(minX, static_cast<int64_t>(std::floor(node.x())));
    maxX = std::max(maxX, static_cast<int64_t>(std::ceil(node.x())));
    minY = std::min(minY, static_cast<int64_t>(std::floor(node.y())));
    maxY = std::max(maxY, static_cast<int64_t>(std::ceil(node.y())));
  }
  minX = std::max<int64_t>(0, minX);
  minY = std::max<int64_t>(0, minY);
  maxX = std::min<int64_t>(static_cast<int64_t>(grid.width) - 1, maxX);
  maxY = std::min<int64_t>(static_cast<int64_t>(grid.height) - 1, maxY);
  if (nodes.size() >= 3) {
    for (int64_t y = minY; y <= maxY; ++y) {
      const auto py = static_cast<double>(y);
      for (int64_t x = minX; x <= maxX; ++x) {
        const auto px = static_cast<double>(x);
        bool inside = false;
        for (std::size_t i = 0, j = nodes.size() - 1; i < nodes.size();
             j = i++) {
          const double xi = nodes[i].x();
          const double yi = nodes[i].y();
          const double xj = nodes[j].x();
          const double yj = nodes[j].y();
          if (((yi > py) != (yj > py)) &&
              (px < ((xj - xi) * (py - yi) / (yj - yi)) + xi)) {
            inside = !inside;
          }
        }
        if (inside) {
          mask.setCell(static_cast<uint32_t>(x), static_cast<uint32_t>(y));
        }
      }
    }
  }
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    const Point a = nodes[i];
    const Point b = nodes[(i + 1) % nodes.size()];
    for (const std::size_t index :
         fullWalk(std::llround(a.x()), std::llround(a.y()), std::llround(b.x()),
                  std::llround(b.y()), grid.width, grid.height)) {
      mask.set(index);
    }
  }
  return mask;
}

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

TEST(Rasterize, TheKeepoutMatchesACellByCellReferenceOnRandomChips) {
  // Overlapping random polygons and corridors. The reference measures every
  // cell center that the polygons leave free against every obstacle edge and
  // against every corridor.
  SplitMix random(17);
  const GridMetrics& grid = unitGrid();
  const auto anywhere = [&] {
    return Point(-10.0 + (random.unit() * 120.0),
                 -10.0 + (random.unit() * 120.0));
  };
  for (int round = 0; round < 40; ++round) {
    std::vector<std::vector<Point>> obstacles(
        static_cast<std::size_t>(random.between(1, 6)));
    for (auto& vertices : obstacles) {
      const int64_t count = random.between(2, 8);
      for (int64_t i = 0; i < count; ++i) {
        vertices.push_back(anywhere());
      }
    }
    RasterOptions options;
    options.keepout = 0.5 + (random.unit() * 4.0);
    const int64_t corridors = random.between(0, 6);
    for (int64_t i = 0; i < corridors; ++i) {
      options.keepoutExemptions.push_back({.from = anywhere(),
                                           .to = anywhere(),
                                           .halfWidth = random.unit() * 5.0});
    }
    const ChipT chip = chipWith(obstacles);
    const RasterizedObstacles raster = rasterizeObstacles(chip, grid, options);
    const BitGrid polygons = rasterizeObstacles(chip, grid).blocked;

    BitGrid expected = polygons;
    std::size_t keepoutCells = 0;
    std::size_t exemptedCells = 0;
    for (uint32_t y = 0; y < grid.height; ++y) {
      for (uint32_t x = 0; x < grid.width; ++x) {
        if (polygons.testCell(x, y)) {
          continue;
        }
        const Point center = grid.toLayout(x, y);
        const bool near = std::ranges::any_of(obstacles, [&](const auto& ring) {
          for (std::size_t i = 0; i < ring.size(); ++i) {
            if (distanceToSegment(center, ring[i],
                                  ring[(i + 1) % ring.size()]) <=
                options.keepout) {
              return true;
            }
          }
          return false;
        });
        if (!near) {
          continue;
        }
        ++keepoutCells;
        if (std::ranges::any_of(
                options.keepoutExemptions, [&](const Corridor& corridor) {
                  return distanceToSegment(center, corridor.from,
                                           corridor.to) <= corridor.halfWidth;
                })) {
          ++exemptedCells;
        } else {
          expected.setCell(x, y);
        }
      }
    }
    ASSERT_EQ(raster.blocked, expected) << "round " << round;
    ASSERT_EQ(raster.keepoutCells, keepoutCells) << "round " << round;
    ASSERT_EQ(raster.exemptedCells, exemptedCells) << "round " << round;
  }
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

  // The squared length of these segments does not fit into a double.
  EXPECT_DOUBLE_EQ(
      distanceToSegment(Point(5.0, 4.0), Point(1e300, 3.0), Point(0.0, 3.0)),
      1.0);
  EXPECT_DOUBLE_EQ(
      distanceToSegment(Point(5.0, 4.0), Point(0.0, 3.0), Point(1e300, 3.0)),
      1.0);
  EXPECT_DOUBLE_EQ(
      distanceToSegment(Point(-3.0, 7.0), Point(0.0, 3.0), Point(1e300, 3.0)),
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

TEST(Rasterize, FillMatchesACellByCellReferenceOnRandomPolygons) {
  SplitMix random(11);
  for (int round = 0; round < 400; ++round) {
    const auto width = static_cast<uint32_t>(random.between(2, 70));
    const auto height = static_cast<uint32_t>(random.between(2, 70));
    const BoundingBox box{.minX = -3.0,
                          .minY = 7.0,
                          .maxX = -3.0 + (0.7 * (width - 1)),
                          .maxY = 7.0 + (1.3 * (height - 1))};
    const GridMetrics grid = GridMetrics::fit(box, width, height);
    PolygonT polygon;
    const int64_t vertices = random.between(1, 14);
    const int kind = round % 4;
    for (int64_t i = 0; i < vertices; ++i) {
      double x = 0.0;
      double y = 0.0;
      if (kind == 0) {
        // Anywhere in and around the box.
        x = box.minX - 5.0 + (random.unit() * (box.width() + 10.0));
        y = box.minY - 5.0 + (random.unit() * (box.height() + 10.0));
      } else if (kind == 1) {
        // On cell centers and halfway between them, so that vertices and
        // edges lie exactly on rows and columns.
        const int64_t halfX =
            random.between(-4, (2 * static_cast<int64_t>(width)) + 4);
        const int64_t halfY =
            random.between(-4, (2 * static_cast<int64_t>(height)) + 4);
        x = box.minX + (grid.cellWidth * static_cast<double>(halfX) / 2.0);
        y = box.minY + (grid.cellHeight * static_cast<double>(halfY) / 2.0);
      } else if (kind == 2) {
        // A star around the center of the box.
        const double angle = 2.0 * std::numbers::pi * static_cast<double>(i) /
                             static_cast<double>(vertices);
        const double radius = (0.2 + random.unit()) * box.width();
        x = box.minX + (box.width() / 2.0) + (radius * std::cos(angle));
        y = box.minY + (box.height() / 2.0) + (radius * std::sin(angle));
      } else {
        // Far off the grid.
        x = box.minX + ((random.unit() - 0.5) * 2000.0);
        y = box.minY + ((random.unit() - 0.5) * 2000.0);
      }
      polygon.vertices.emplace_back(x, y);
    }
    BitGrid mask(width, height);
    fillPolygon(mask, grid, polygon);
    ASSERT_EQ(mask, perCellFill(grid, polygon))
        << "round " << round << ", " << vertices << " vertices";
  }
}

TEST(Rasterize, LineCellsMatchTheFullWalkOnRandomLines) {
  SplitMix random(13);
  for (int round = 0; round < 4000; ++round) {
    const auto width = static_cast<uint32_t>(random.between(1, 50));
    const auto height = static_cast<uint32_t>(random.between(1, 50));
    // Most ends lie on the grid or near it, some far off it.
    int64_t reach = 20;
    if (round % 40 == 0) {
      reach = 100000;
    } else if (round % 2 == 0) {
      reach = 200;
    }
    const int64_t x0 = random.between(-reach, width + reach);
    const int64_t y0 = random.between(-reach, height + reach);
    const int64_t x1 = random.between(-reach, width + reach);
    const int64_t y1 = random.between(-reach, height + reach);
    ASSERT_EQ(lineCells(x0, y0, x1, y1, width, height),
              fullWalk(x0, y0, x1, y1, width, height))
        << "(" << x0 << ", " << y0 << ") to (" << x1 << ", " << y1 << ") on "
        << width << "x" << height;
  }
}

TEST(Rasterize, LineCellsAtAFarScaleRepeatTheNearScale) {
  // The cells of a line with a slope of 3 / 7 through a cell repeat every
  // seven steps, whatever the length of the line. So a line whose ends lie
  // 2^40 cells away has the cells of the same line with ends a few hundred
  // cells away, which the full walk can check.
  constexpr int64_t farScale = int64_t{1} << 40U;
  constexpr int64_t nearScale = 50;
  constexpr uint32_t width = 60;
  constexpr uint32_t height = 40;
  struct Shape {
    int64_t stepX = 0;
    int64_t stepY = 0;
    int64_t throughX = 0;
    int64_t throughY = 0;
  };
  // Along x and along y, in both directions, and one line that enters the
  // grid through its bottom edge.
  const std::vector<Shape> shapes = {
      {.stepX = 7, .stepY = 3},
      {.stepX = -7, .stepY = -3},
      {.stepX = 3, .stepY = 7},
      {.stepX = -3, .stepY = -7},
      {.stepX = 7, .stepY = 3, .throughX = 20, .throughY = -4}};
  const auto cellsAt = [](const Shape& shape, const int64_t scale) {
    return lineCells(shape.throughX - (shape.stepX * scale),
                     shape.throughY - (shape.stepY * scale),
                     shape.throughX + (shape.stepX * scale),
                     shape.throughY + (shape.stepY * scale), width, height);
  };
  for (const Shape& shape : shapes) {
    const std::vector<std::size_t> nearCells = cellsAt(shape, nearScale);
    EXPECT_FALSE(nearCells.empty());
    EXPECT_EQ(nearCells, fullWalk(shape.throughX - (shape.stepX * nearScale),
                                  shape.throughY - (shape.stepY * nearScale),
                                  shape.throughX + (shape.stepX * nearScale),
                                  shape.throughY + (shape.stepY * nearScale),
                                  width, height));
    EXPECT_EQ(cellsAt(shape, farScale), nearCells)
        << shape.stepX << ", " << shape.stepY;
  }
}

TEST(Rasterize, LineCellsRefuseEndsTooFarApart) {
  constexpr int64_t limit = int64_t{1} << 61U;
  EXPECT_THROW(static_cast<void>(lineCells(0, 0, limit, 0, 10, 10)),
               std::invalid_argument);
  EXPECT_THROW(static_cast<void>(lineCells(3, -limit, 3, 0, 10, 10)),
               std::invalid_argument);
  EXPECT_THROW(static_cast<void>(
                   lineCells(std::numeric_limits<int64_t>::min(), 0,
                             std::numeric_limits<int64_t>::max(), 0, 10, 10)),
               std::invalid_argument);
  // Just below the limit, the line runs along the bottom row of the grid.
  EXPECT_EQ(lineCells(0, 0, limit - 1, 5, 10, 10).size(), 10U);
}

TEST(Rasterize, AVertexFarOffTheGridKeepsTheDirectionOfItsEdges) {
  // Two edges of slope 0.4 run from (10, 10) and from (10, 90) to a vertex
  // 1e30 units off the grid. The rasterization finishes, and the lines of
  // both edges keep their slope across the grid.
  const std::vector<Point> band = {Point(10.0, 10.0), Point(1e30, 4e29),
                                   Point(10.0, 90.0)};
  RasterOptions options;
  options.keepout = 2.0;
  const RasterizedObstacles raster =
      rasterizeObstacles(chipWith({band}), unitGrid(), options);
  const BitGrid& blocked = raster.blocked;
  EXPECT_TRUE(blocked.testCell(35, 20));
  EXPECT_TRUE(blocked.testCell(60, 30));
  EXPECT_TRUE(blocked.testCell(85, 40));
  EXPECT_TRUE(blocked.testCell(100, 46));
  EXPECT_TRUE(blocked.testCell(20, 94));
  EXPECT_TRUE(blocked.testCell(35, 100));
  // The keepout reaches the cell 1.86 units below the lower edge, but not the
  // cell 4.64 units below it.
  EXPECT_TRUE(blocked.testCell(60, 28));
  EXPECT_FALSE(blocked.testCell(60, 25));
  EXPECT_FALSE(blocked.testCell(5, 50));
}

TEST(Rasterize, AnEdgeToAVertexFarOffTheGridBlocksOnlyItsOwnCells) {
  // One cell per layout unit over 0..9 in both axes.
  const GridMetrics grid = GridMetrics::fit(
      BoundingBox{.minX = 0.0, .minY = 0.0, .maxX = 9.0, .maxY = 9.0}, 10, 10);
  const auto fill = [&](const std::vector<Point>& vertices) {
    BitGrid mask(grid.width, grid.height);
    PolygonT polygon;
    polygon.vertices = vertices;
    fillPolygon(mask, grid, polygon);
    return mask;
  };
  const auto cellsOf = [&](const uint32_t minX, const uint32_t maxX,
                           const uint32_t minY, const uint32_t maxY) {
    BitGrid mask(grid.width, grid.height);
    for (uint32_t y = minY; y <= maxY; ++y) {
      for (uint32_t x = minX; x <= maxX; ++x) {
        mask.setCell(x, y);
      }
    }
    return mask;
  };

  // A triangle wholly right of the grid blocks no cell.
  EXPECT_EQ(
      fill({Point(1e300, 3.0), Point(6e17, 3.0), Point(6e17, 7.0)}).count(),
      0U);
  // The edge from (5, 3) to the far vertex runs right, not left.
  EXPECT_EQ(fill({Point(5.0, 3.0), Point(1e300, 3.0)}), cellsOf(5, 9, 3, 3));
  // Two far vertices: every edge stays short enough for its line.
  BitGrid band(grid.width, grid.height);
  EXPECT_NO_THROW(band =
                      fill({Point(2.0, 2.0), Point(7.0, 2.0), Point(1e300, 5.0),
                            Point(1e20, 6.0), Point(2.0, 8.0)}));
  EXPECT_EQ(band, cellsOf(2, 9, 2, 8));

  // The edges of slope 0.4 from (10, 10) and from (10, 90) to a vertex 1e300
  // units off the grid keep their slope across the grid.
  BitGrid wedge(unitGrid().width, unitGrid().height);
  PolygonT polygon;
  polygon.vertices = {Point(10.0, 10.0), Point(1e300, 4e299),
                      Point(10.0, 90.0)};
  fillPolygon(wedge, unitGrid(), polygon);
  EXPECT_TRUE(wedge.testCell(35, 20));
  EXPECT_TRUE(wedge.testCell(85, 40));
  EXPECT_TRUE(wedge.testCell(100, 46));
  EXPECT_TRUE(wedge.testCell(20, 94));
  EXPECT_TRUE(wedge.testCell(35, 100));
  EXPECT_FALSE(wedge.testCell(60, 29));
  EXPECT_FALSE(wedge.testCell(5, 90));
}

TEST(Rasterize, ANonFiniteVertexIsRefusedAndKeepsTheMask) {
  const GridMetrics& grid = unitGrid();
  BitGrid mask(grid.width, grid.height);
  mask.setCell(50, 50);
  const BitGrid before = mask;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  PolygonT polygon;
  polygon.vertices = {Point(10.0, 10.0), Point(60.0, 10.0), Point(nan, 40.0)};
  EXPECT_THROW(fillPolygon(mask, grid, polygon), std::invalid_argument);
  EXPECT_EQ(mask, before);
  polygon.vertices[2] = Point(30.0, infinity);
  EXPECT_THROW(fillPolygon(mask, grid, polygon), std::invalid_argument);
  EXPECT_EQ(mask, before);
  EXPECT_THROW(
      static_cast<void>(rasterizeObstacles(
          chipWith({square(), {Point(nan, nan), Point(1.0, 1.0)}}), grid)),
      std::invalid_argument);

  // A vertex that is finite in layout units but not in cells.
  const GridMetrics tiny = GridMetrics::fit(
      BoundingBox{.minX = 0.0, .minY = 0.0, .maxX = 1e-297, .maxY = 1e-297}, 11,
      11);
  BitGrid small(tiny.width, tiny.height);
  PolygonT far;
  far.vertices = {Point(0.0, 0.0), Point(1e20, 0.0), Point(0.0, 1e-297)};
  EXPECT_THROW(fillPolygon(small, tiny, far), std::invalid_argument);
}

TEST(Rasterize, ACorridorFreesACellThatTheIslandRuleFreed) {
  // A line one cell wide from x = 20 to x = 30 along row 70. The island rule
  // frees its two end cells, so they are no polygon cells any more.
  const std::vector<Point> line = {Point(20.0, 70.0), Point(30.0, 70.0),
                                   Point(30.0, 70.2), Point(20.0, 70.2)};
  const BitGrid plain =
      rasterizeObstacles(chipWith({line}), unitGrid()).blocked;
  EXPECT_FALSE(plain.testCell(20, 70));
  EXPECT_TRUE(plain.testCell(21, 70));
  EXPECT_TRUE(plain.testCell(29, 70));
  EXPECT_FALSE(plain.testCell(30, 70));

  // The keepout is measured from the edges and blocks the end cells again.
  RasterOptions options;
  options.keepout = 2.0;
  const BitGrid guarded =
      rasterizeObstacles(chipWith({line}), unitGrid(), options).blocked;
  EXPECT_TRUE(guarded.testCell(20, 70));

  // There they are keepout cells, so a corridor that reaches them frees them.
  // The polygon cell next to them stays blocked.
  options.keepoutExemptions.push_back(
      {.from = Point(14.0, 70.0), .to = Point(20.0, 70.0), .halfWidth = 0.5});
  const RasterizedObstacles exempted =
      rasterizeObstacles(chipWith({line}), unitGrid(), options);
  EXPECT_FALSE(exempted.blocked.testCell(20, 70));
  EXPECT_TRUE(exempted.blocked.testCell(21, 70));
  EXPECT_GT(exempted.exemptedCells, 0U);
}

} // namespace
