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

/// The number of lattice steps per cell of the vertices of perCellFill().
constexpr int64_t LATTICE = 64;

/// A vertex in lattice steps of 1 / LATTICE cell from cell (0, 0).
struct LatticePoint {
  int64_t x = 0;
  int64_t y = 0;
};

/// The cell nearest to a coordinate in lattice steps. A coordinate halfway
/// between two cells takes the one farther from cell zero.
int64_t nearestCell(const int64_t steps) {
  constexpr int64_t half = LATTICE / 2;
  return steps >= 0 ? (steps + half) / LATTICE : -((half - steps) / LATTICE);
}

/// The cells of the fill rule of fillPolygon(), found cell by cell in whole
/// numbers: every cell center against every edge by the even-odd rule, and
/// every edge walked in full between the nearest cells of its ends.
BitGrid perCellFill(const uint32_t width, const uint32_t height,
                    const std::vector<LatticePoint>& nodes) {
  BitGrid mask(width, height);
  if (nodes.size() >= 3) {
    for (uint32_t y = 0; y < height; ++y) {
      const int64_t py = static_cast<int64_t>(y) * LATTICE;
      for (uint32_t x = 0; x < width; ++x) {
        const int64_t px = static_cast<int64_t>(x) * LATTICE;
        bool inside = false;
        for (std::size_t i = 0, j = nodes.size() - 1; i < nodes.size();
             j = i++) {
          const LatticePoint& from = nodes[i];
          const LatticePoint& to = nodes[j];
          if ((from.y > py) == (to.y > py)) {
            continue;
          }
          // The edge crosses the row right of the center when the center
          // lies left of the edge, seen from its lower end.
          const int64_t side = ((to.x - from.x) * (py - from.y)) -
                               ((px - from.x) * (to.y - from.y));
          if (to.y > from.y ? side > 0 : side < 0) {
            inside = !inside;
          }
        }
        if (inside) {
          mask.setCell(x, y);
        }
      }
    }
  }
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    const LatticePoint& a = nodes[i];
    const LatticePoint& b = nodes[(i + 1) % nodes.size()];
    for (const std::size_t index :
         fullWalk(nearestCell(a.x), nearestCell(a.y), nearestCell(b.x),
                  nearestCell(b.y), width, height)) {
      mask.set(index);
    }
  }
  return mask;
}

/// The distance from a point to a segment, from the closed formula: the foot
/// of the perpendicular, clamped onto the segment.
double referenceDistance(const Point point, const Point from, const Point to) {
  const double dx = to.x() - from.x();
  const double dy = to.y() - from.y();
  const double length2 = (dx * dx) + (dy * dy);
  const double along =
      ((point.x() - from.x()) * dx) + ((point.y() - from.y()) * dy);
  const double t = length2 > 0.0 ? std::clamp(along / length2, 0.0, 1.0) : 0.0;
  const double ex = point.x() - (from.x() + (t * dx));
  const double ey = point.y() - (from.y() + (t * dy));
  return std::sqrt((ex * ex) + (ey * ey));
}

/// The smallest distance from a point to an edge of the polygons.
double
referenceDistanceToEdges(const Point point,
                         const std::vector<std::vector<Point>>& obstacles) {
  double nearest = std::numeric_limits<double>::infinity();
  for (const auto& ring : obstacles) {
    for (std::size_t i = 0; i < ring.size(); ++i) {
      nearest =
          std::min(nearest, referenceDistance(point, ring[i],
                                              ring[(i + 1) % ring.size()]));
    }
  }
  return nearest;
}

/// The smallest distance from a point to a port corridor, less its half
/// width.
double
referenceDepthInPortCorridors(const Point point,
                              const std::vector<PortCorridor>& portCorridors) {
  double nearest = std::numeric_limits<double>::infinity();
  for (const PortCorridor& portCorridor : portCorridors) {
    nearest = std::min(
        nearest, referenceDistance(point, portCorridor.from, portCorridor.to) -
                     portCorridor.halfWidth);
  }
  return nearest;
}

/// The gap between a distance and its limit below which rounding could
/// decide a cell, in layout units. The rounding errors of both distances lie
/// far below it.
constexpr double TIE_MARGIN = 1e-9;

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

TEST(Rasterize, APolygonOfTwoCellsDisappears) {
  // A strip one unit long and a fifth of a unit high blocks the cells
  // (50, 50) and (51, 50). Each of them has seven free neighbors.
  const std::vector<Point> strip = {Point(50.0, 50.0), Point(51.0, 50.0),
                                    Point(51.0, 50.2), Point(50.0, 50.2)};
  BitGrid filled(unitGrid().width, unitGrid().height);
  PolygonT polygon;
  polygon.vertices = strip;
  fillPolygon(filled, unitGrid(), polygon);
  EXPECT_EQ(filled.count(), 2U);
  EXPECT_TRUE(filled.testCell(50, 50));
  EXPECT_TRUE(filled.testCell(51, 50));

  EXPECT_EQ(rasterizeObstacles(chipWith({strip}), unitGrid()).blocked.count(),
            0U);
}

TEST(Rasterize, ABorderWiderThanTheMaskBlocksAllOfIt) {
  BitGrid columns(5, 4);
  blockBorder(columns, 7, 0);
  EXPECT_EQ(columns.count(), columns.size());
  BitGrid rows(5, 4);
  blockBorder(rows, 0, 9);
  EXPECT_EQ(rows.count(), rows.size());
}

TEST(Rasterize, TheBorderReachesTheLastColumnOfTheWidestMask) {
  // The widest mask holds 2^32 - 1 columns in 512 MiB. Its last column lies
  // 2^32 - 2 columns from the left edge, and that index plus the border of two
  // columns exceeds the range of uint32_t.
  constexpr uint32_t width = std::numeric_limits<uint32_t>::max();
  BitGrid mask(width, 1);
  blockBorder(mask, 2, 0);
  EXPECT_TRUE(mask.testCell(0, 0));
  EXPECT_TRUE(mask.testCell(1, 0));
  EXPECT_FALSE(mask.testCell(2, 0));
  EXPECT_FALSE(mask.testCell(width - 3, 0));
  EXPECT_TRUE(mask.testCell(width - 2, 0));
  EXPECT_TRUE(mask.testCell(width - 1, 0));
  EXPECT_EQ(mask.count(), 4U);
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

TEST(Rasterize, TheKeepoutBlocksACenterAtExactlyTheKeepoutDistance) {
  // Cell steps of exactly 10 units from -1234.5, so that every cell center
  // and every distance below is exact, whatever the order of the operations.
  // Column c lies at 10 c - 1234.5 and row r at 10 r - 1234.5.
  const GridMetrics grid = GridMetrics::fit(
      BoundingBox{
          .minX = -1234.5, .minY = -1234.5, .maxX = 2755.5, .maxY = 2755.5},
      400, 400);
  ASSERT_EQ(grid.cellWidth, 10.0);
  ASSERT_EQ(grid.cellHeight, 10.0);
  // Each edge lies an odd multiple of 5 units from the centers along its
  // normal: 25 units right of column 100, 25 units left of column 155,
  // 25 units above row 221 and 25 units below row 326. No center lies
  // exactly 25 units from a corner.
  const double left = -209.5;
  const double right = 290.5;
  const double bottom = 1000.5;
  const double top = 2000.5;
  RasterOptions options;
  options.keepout = 25.0;
  const ChipT chip = chipWith({{Point(left, bottom), Point(right, bottom),
                                Point(right, top), Point(left, top)}});
  const RasterizedObstacles raster = rasterizeObstacles(chip, grid, options);
  const BitGrid& blocked = raster.blocked;

  // A center exactly 25 units from an edge is blocked, a center 35 units
  // from it is free.
  EXPECT_TRUE(blocked.testCell(100, 274));
  EXPECT_FALSE(blocked.testCell(99, 274));
  EXPECT_TRUE(blocked.testCell(155, 274));
  EXPECT_FALSE(blocked.testCell(156, 274));
  EXPECT_TRUE(blocked.testCell(128, 221));
  EXPECT_FALSE(blocked.testCell(128, 220));
  EXPECT_TRUE(blocked.testCell(128, 326));
  EXPECT_FALSE(blocked.testCell(128, 327));

  // The keepout cells are the free cells whose center lies 25 units or less
  // from the rectangle. Every value here is a multiple of a half, so the
  // squared distances are exact.
  const BitGrid polygons = rasterizeObstacles(chip, grid).blocked;
  std::size_t keepoutCells = 0;
  for (uint32_t y = 0; y < grid.height; ++y) {
    for (uint32_t x = 0; x < grid.width; ++x) {
      const double px = -1234.5 + (10.0 * x);
      const double py = -1234.5 + (10.0 * y);
      const double dx = std::max({left - px, 0.0, px - right});
      const double dy = std::max({bottom - py, 0.0, py - top});
      if (!polygons.testCell(x, y) && (dx * dx) + (dy * dy) <= 625.0) {
        ++keepoutCells;
      }
    }
  }
  EXPECT_EQ(raster.keepoutCells, keepoutCells);
  EXPECT_EQ(blocked.count(), polygons.count() + keepoutCells);
}

TEST(Rasterize, AKeepoutAsLongAsAPythagoreanDistanceReachesTheCell) {
  // Whole legs give an exact sum of squares, so each distance is exactly a
  // whole number. std::hypot need not round correctly: the formula
  // 220 * sqrt(1 + (21 / 220)^2) gives the next double above 221.
  EXPECT_EQ(distanceToSegment(Point(17.0, 16.0), Point(20.0, 20.0),
                              Point(40.0, 20.0)),
            5.0);
  EXPECT_EQ(distanceToSegment(Point(61.0, 260.0), Point(40.0, 20.0),
                              Point(40.0, 40.0)),
            221.0);
  EXPECT_EQ(distanceToSegment(Point(-4059.0, 4060.0), Point(0.0, 0.0),
                              Point(1.0e6, 0.0)),
            5741.0);

  // The keepout blocks a cell whose center lies at exactly the keepout from a
  // corner of the square, and no cell beyond.
  RasterOptions options;
  options.keepout = 5.0;
  const BitGrid near =
      rasterizeObstacles(chipWith({square()}), unitGrid(), options).blocked;
  EXPECT_TRUE(near.testCell(17, 16));
  EXPECT_TRUE(near.testCell(16, 17));
  EXPECT_FALSE(near.testCell(16, 16));

  options.keepout = 221.0;
  const GridMetrics wide = GridMetrics::fit(
      BoundingBox{.minX = 0.0, .minY = 0.0, .maxX = 300.0, .maxY = 300.0}, 301,
      301);
  const BitGrid far =
      rasterizeObstacles(chipWith({square()}), wide, options).blocked;
  EXPECT_TRUE(far.testCell(61, 260));
  EXPECT_TRUE(far.testCell(260, 61));
  EXPECT_FALSE(far.testCell(62, 260));
  EXPECT_FALSE(far.testCell(61, 261));
}

TEST(Rasterize, TheKeepoutComparesTheComputedDistance) {
  // The cell center (50, 50) lies 12.829492330551147 units right of and
  // 24.0552981197834 units above the corner v of a triangle. The squares of
  // these legs sum exactly to the square of the double 27.26267120242119, but
  // the squares round, so the computed distance need not equal that value.
  // The keepout compares the computed distance: a keepout equal to it blocks
  // the cell, and the next double below it leaves the cell free.
  const Point v(37.17050766944885, 25.9447018802166);
  const std::vector<Point> triangle = {v, Point(v.x() - 10.0, v.y()),
                                       Point(v.x(), v.y() - 10.0)};
  const double computed =
      distanceToSegment(Point(50.0, 50.0), v, Point(v.x() - 10.0, v.y()));
  RasterOptions options;
  options.keepout = computed;
  EXPECT_TRUE(rasterizeObstacles(chipWith({triangle}), unitGrid(), options)
                  .blocked.testCell(50, 50));
  options.keepout = std::nextafter(computed, 0.0);
  EXPECT_FALSE(rasterizeObstacles(chipWith({triangle}), unitGrid(), options)
                   .blocked.testCell(50, 50));
}

TEST(Rasterize, TheDistanceHoldsWhereItsSquareLeavesTheRangeOfADouble) {
  // The square of the first distance overflows, and that of the second
  // falls below the normal numbers.
  EXPECT_DOUBLE_EQ(
      distanceToSegment(Point(1e200, 1e200), Point(0.0, 0.0), Point(0.0, 0.0)),
      std::numbers::sqrt2 * 1e200);
  EXPECT_DOUBLE_EQ(distanceToSegment(Point(3e-160, 4e-160), Point(0.0, 0.0),
                                     Point(0.0, 0.0)),
                   5e-160);
}

TEST(Rasterize, APortCorridorReleasesKeepoutCellsButNeverPolygonCells) {
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

TEST(Rasterize, APortCorridorWithANegativeHalfWidthFreesNoCell) {
  // The strip holds no point. Its window along x is empty, and its window
  // along y is not.
  RasterOptions options;
  options.keepout = 5.0;
  const RasterizedObstacles plain =
      rasterizeObstacles(chipWith({square()}), unitGrid(), options);
  options.keepoutExemptions.push_back(
      {.from = Point(17.5, 10.0), .to = Point(17.5, 50.0), .halfWidth = -3.5});
  const RasterizedObstacles exempted =
      rasterizeObstacles(chipWith({square()}), unitGrid(), options);
  EXPECT_EQ(exempted.blocked, plain.blocked);
  EXPECT_EQ(exempted.exemptedCells, 0U);
}

TEST(Rasterize, TheKeepoutMatchesACellByCellReferenceOnRandomChips) {
  // Overlapping random polygons and port corridors. The reference measures
  // every cell center that the polygons leave free against every obstacle
  // edge and against every port corridor, with its own distance. No center
  // lies within rounding of the keepout or of the half width of a
  // port corridor, so that the rounding of neither distance decides a cell.
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
    const int64_t portCorridors = random.between(0, 6);
    for (int64_t i = 0; i < portCorridors; ++i) {
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
        const double beyond =
            referenceDistanceToEdges(center, obstacles) - options.keepout;
        ASSERT_GT(std::fabs(beyond), TIE_MARGIN) << "round " << round;
        if (beyond > 0.0) {
          continue;
        }
        ++keepoutCells;
        const double depth =
            referenceDepthInPortCorridors(center, options.keepoutExemptions);
        ASSERT_GT(std::fabs(depth), TIE_MARGIN) << "round " << round;
        if (depth <= 0.0) {
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

TEST(Rasterize, TheKeepoutOfLongDiagonalEdgesMatchesAReferenceOnALargeGrid) {
  // Thin triangles whose long edges cross a grid of 3000 by 3000 cells at
  // several slopes, a keepout of 185 units and two port corridors across
  // them. The reference measures every cell of every 53rd row against every
  // edge and every port corridor. No center lies within rounding of a limit.
  const GridMetrics grid = GridMetrics::fit(
      BoundingBox{.minX = 0.0, .minY = 0.0, .maxX = 30000.0, .maxY = 30000.0},
      3000, 3000);
  const std::vector<std::vector<Point>> obstacles = {
      {Point(100.0, 50.0), Point(29900.0, 29950.0), Point(29870.0, 29950.0)},
      {Point(200.0, 29900.0), Point(29800.0, 120.0), Point(29760.0, 120.0)},
      {Point(5000.0, 0.0), Point(9000.0, 30000.0), Point(8950.0, 30000.0)},
      {Point(0.0, 21000.0), Point(30000.0, 15000.0), Point(30000.0, 15040.0)}};
  RasterOptions options;
  options.keepout = 185.0;
  options.keepoutExemptions = {{.from = Point(3000.0, 27000.0),
                                .to = Point(27000.0, 3000.0),
                                .halfWidth = 92.5},
                               {.from = Point(15000.0, 0.0),
                                .to = Point(16000.0, 30000.0),
                                .halfWidth = 92.5}};
  const ChipT chip = chipWith(obstacles);
  const RasterizedObstacles raster = rasterizeObstacles(chip, grid, options);
  const BitGrid polygons = rasterizeObstacles(chip, grid).blocked;

  std::size_t keepoutCells = 0;
  std::size_t exemptedCells = 0;
  for (uint32_t y = 7; y < grid.height; y += 53) {
    for (uint32_t x = 0; x < grid.width; ++x) {
      bool expected = polygons.testCell(x, y);
      if (!expected) {
        const Point center = grid.toLayout(x, y);
        const double beyond =
            referenceDistanceToEdges(center, obstacles) - options.keepout;
        ASSERT_GT(std::fabs(beyond), TIE_MARGIN) << x << ", " << y;
        if (beyond <= 0.0) {
          ++keepoutCells;
          const double depth =
              referenceDepthInPortCorridors(center, options.keepoutExemptions);
          ASSERT_GT(std::fabs(depth), TIE_MARGIN) << x << ", " << y;
          expected = depth > 0.0;
          exemptedCells += expected ? 0 : 1;
        }
      }
      ASSERT_EQ(raster.blocked.testCell(x, y), expected) << x << ", " << y;
    }
  }
  // The sampled rows hold keepout cells, and the port corridors free some of
  // them.
  EXPECT_GT(keepoutCells, exemptedCells);
  EXPECT_GT(exemptedCells, 0U);
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

  // The squared length of these segments fits into a double. One end lies
  // 1e17 units away, and the nearest point lies next to the other end.
  EXPECT_DOUBLE_EQ(
      distanceToSegment(Point(5.0, 4.0), Point(1e17, 3.0), Point(0.0, 3.0)),
      1.0);
  EXPECT_DOUBLE_EQ(
      distanceToSegment(Point(5.0, 4.0), Point(0.0, 3.0), Point(1e17, 3.0)),
      1.0);
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
  // The vertices lie on a lattice of 1/64 cell, and the cell steps are sums
  // of powers of two. So every vertex converts into cells exactly, and the
  // reference decides every tie by the rule: a vertex on a row, a center on
  // an edge, a vertex halfway between two cells.
  SplitMix random(11);
  for (int round = 0; round < 400; ++round) {
    const auto width = static_cast<uint32_t>(random.between(2, 70));
    const auto height = static_cast<uint32_t>(random.between(2, 70));
    const BoundingBox box{.minX = -3.0,
                          .minY = 7.0,
                          .maxX = -3.0 + (0.75 * (width - 1)),
                          .maxY = 7.0 + (1.25 * (height - 1))};
    const GridMetrics grid = GridMetrics::fit(box, width, height);
    ASSERT_EQ(grid.cellWidth, 0.75);
    ASSERT_EQ(grid.cellHeight, 1.25);
    const auto columns = static_cast<int64_t>(width);
    const auto rows = static_cast<int64_t>(height);
    std::vector<LatticePoint> nodes;
    const int64_t vertices = random.between(1, 14);
    const int kind = round % 4;
    for (int64_t i = 0; i < vertices; ++i) {
      LatticePoint node;
      if (kind == 0) {
        // Anywhere in and around the grid.
        node = {.x = random.between(-5 * LATTICE, (columns + 5) * LATTICE),
                .y = random.between(-5 * LATTICE, (rows + 5) * LATTICE)};
      } else if (kind == 1) {
        // On cell centers and halfway between them, so that vertices and
        // edges lie exactly on rows and columns.
        constexpr int64_t half = LATTICE / 2;
        node = {.x = half * random.between(-4, (2 * columns) + 4),
                .y = half * random.between(-4, (2 * rows) + 4)};
      } else if (kind == 2) {
        // A star around the middle of the grid.
        const double angle = 2.0 * std::numbers::pi * static_cast<double>(i) /
                             static_cast<double>(vertices);
        const double radius =
            (0.2 + random.unit()) * static_cast<double>(columns * LATTICE);
        node = {.x = (columns * LATTICE / 2) +
                     std::llround(radius * std::cos(angle)),
                .y = (rows * LATTICE / 2) +
                     std::llround(radius * std::sin(angle))};
      } else {
        // Far off the grid.
        node = {.x = random.between(-2000 * LATTICE, 2000 * LATTICE),
                .y = random.between(-2000 * LATTICE, 2000 * LATTICE)};
      }
      nodes.push_back(node);
    }
    PolygonT polygon;
    for (const LatticePoint& node : nodes) {
      polygon.vertices.emplace_back(
          box.minX + (grid.cellWidth * static_cast<double>(node.x) /
                      static_cast<double>(LATTICE)),
          box.minY + (grid.cellHeight * static_cast<double>(node.y) /
                      static_cast<double>(LATTICE)));
    }
    BitGrid mask(width, height);
    fillPolygon(mask, grid, polygon);
    ASSERT_EQ(mask, perCellFill(width, height, nodes))
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
  // 1e17 or 1e30 units off the grid, in both vertex orders. The rasterization
  // finishes, the lines of both edges keep their slope across the grid, and
  // the band between them is filled.
  for (const double far : {1e17, 1e30}) {
    for (const bool reversed : {false, true}) {
      SCOPED_TRACE(testing::Message()
                   << "vertex at " << far << (reversed ? ", reversed" : ""));
      std::vector<Point> band = {Point(10.0, 10.0), Point(far, 0.4 * far),
                                 Point(10.0, 90.0)};
      if (reversed) {
        std::ranges::reverse(band);
      }
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
      EXPECT_TRUE(blocked.testCell(60, 50));
      EXPECT_TRUE(blocked.testCell(30, 60));
      // The keepout reaches the cells 1.86 units below the lower edge and
      // above the upper edge, but not the cells 4.64 units below the lower
      // edge and 2.79 units above the upper edge.
      EXPECT_TRUE(blocked.testCell(60, 28));
      EXPECT_TRUE(blocked.testCell(20, 96));
      EXPECT_FALSE(blocked.testCell(60, 25));
      EXPECT_FALSE(blocked.testCell(20, 97));
      EXPECT_FALSE(blocked.testCell(5, 50));
    }
  }
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

TEST(Rasterize, APortCorridorFreesACellThatTheIslandRuleFreed) {
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

  // There they are keepout cells, so a port corridor that reaches them frees
  // them. The polygon cell next to them stays blocked.
  options.keepoutExemptions.push_back(
      {.from = Point(14.0, 70.0), .to = Point(20.0, 70.0), .halfWidth = 0.5});
  const RasterizedObstacles exempted =
      rasterizeObstacles(chipWith({line}), unitGrid(), options);
  EXPECT_FALSE(exempted.blocked.testCell(20, 70));
  EXPECT_TRUE(exempted.blocked.testCell(21, 70));
  EXPECT_GT(exempted.exemptedCells, 0U);
}

} // namespace
