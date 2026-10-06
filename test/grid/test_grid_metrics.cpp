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
#include "mqt-scpd/grid/GridMetrics.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

namespace {

using namespace mqt::scpd::grid;

const BoundingBox BOX{
    .minX = 100.0, .minY = -50.0, .maxX = 1100.0, .maxY = 450.0};

TEST(GridMetrics, CellsCoverTheBoxCornerToCorner) {
  const GridMetrics grid = GridMetrics::fit(BOX, 101, 51);
  EXPECT_DOUBLE_EQ(grid.cellWidth, 10.0);
  EXPECT_DOUBLE_EQ(grid.cellHeight, 10.0);
  EXPECT_EQ(grid.cells(), 101U * 51U);
  EXPECT_EQ(grid.box(), BOX);

  EXPECT_EQ(grid.toCell(Point(100.0, -50.0)), Point(0.0, 0.0));
  EXPECT_EQ(grid.toCell(Point(1100.0, 450.0)), Point(100.0, 50.0));
  EXPECT_EQ(grid.toLayout(3.0, 4.0), Point(130.0, -10.0));
  EXPECT_EQ(grid.roundToCell(Point(134.9, -14.9)), DCoord(3, 4));
  // Half a cell past the box still rounds onto the edge cell.
  EXPECT_EQ(grid.roundToCell(Point(96.0, 0.0)), DCoord(0, 5));
  EXPECT_FALSE(grid.roundToCell(Point(94.0, 0.0)).has_value());
  EXPECT_FALSE(grid.roundToCell(Point(500.0, 460.0)).has_value());
  EXPECT_EQ(grid.clampToCell(Point(5000.0, -500.0)), DCoord(100, 0));
  EXPECT_EQ(grid.index(3, 4), (4U * 101U) + 3U);
  EXPECT_EQ(grid.cell((4U * 101U) + 3U), DCoord(3, 4));
  EXPECT_TRUE(grid.contains(100, 50));
  EXPECT_FALSE(grid.contains(101, 0));
  EXPECT_FALSE(grid.contains(-1, 0));
}

TEST(GridMetrics, PointsFarOffTheGridOrNotFiniteRoundAndClampSafely) {
  const GridMetrics grid = GridMetrics::fit(BOX, 101, 51);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();

  // No cell is nearest to a coordinate that is not finite or far off the grid.
  EXPECT_FALSE(grid.roundToCell(Point(nan, 0.0)).has_value());
  EXPECT_FALSE(grid.roundToCell(Point(500.0, nan)).has_value());
  EXPECT_FALSE(grid.roundToCell(Point(infinity, 0.0)).has_value());
  EXPECT_FALSE(grid.roundToCell(Point(500.0, -infinity)).has_value());
  EXPECT_FALSE(grid.roundToCell(Point(1e300, 0.0)).has_value());
  EXPECT_FALSE(grid.roundToCell(Point(500.0, -1e300)).has_value());

  // Each coordinate clamps onto the first or the last cell of its axis.
  EXPECT_EQ(grid.clampToCell(Point(infinity, 0.0)), DCoord(100, 5));
  EXPECT_EQ(grid.clampToCell(Point(-infinity, 0.0)), DCoord(0, 5));
  EXPECT_EQ(grid.clampToCell(Point(1e300, -1e300)), DCoord(100, 0));
  EXPECT_EQ(grid.clampToCell(Point(-1e300, 1e300)), DCoord(0, 50));
  EXPECT_EQ(grid.clampToCell(Point(1e300, 1e19)), DCoord(100, 50));

  // A coordinate that is not a number has no side to clamp to.
  EXPECT_THROW(static_cast<void>(grid.clampToCell(Point(nan, 0.0))),
               std::invalid_argument);
  EXPECT_THROW(static_cast<void>(grid.clampToCell(Point(500.0, nan))),
               std::invalid_argument);
}

TEST(GridMetrics, AGridWithoutCellsHasNoCellToClampTo) {
  const GridMetrics empty;
  EXPECT_FALSE(empty.roundToCell(Point(3.0, 4.0)).has_value());
  EXPECT_THROW(static_cast<void>(empty.clampToCell(Point(3.0, 4.0))),
               std::invalid_argument);
  GridMetrics noColumns = GridMetrics::fit(BOX, 101, 51);
  noColumns.width = 0;
  EXPECT_THROW(static_cast<void>(noColumns.clampToCell(Point(500.0, 0.0))),
               std::invalid_argument);
}

TEST(GridMetrics, RefinementAndAspectRatioFollowTheBox) {
  const GridMetrics coarse = GridMetrics::fit(BOX, 11, 6);
  const GridMetrics fine = coarse.refined(10);
  EXPECT_EQ(fine.width, 110U);
  EXPECT_EQ(fine.height, 60U);
  const BoundingBox fineBox = fine.box();
  EXPECT_NEAR(fineBox.maxX, BOX.maxX, 1e-9);
  EXPECT_NEAR(fineBox.maxY, BOX.maxY, 1e-9);

  const GridMetrics aspect = GridMetrics::fitWidth(BOX, 50);
  EXPECT_EQ(aspect.width, 50U);
  EXPECT_EQ(aspect.height, 25U);
}

TEST(GridMetrics, TheRouterGridDividesEveryCapacityCell) {
  // A 30000 unit chip on 50 capacity cells of 600 units each: 60 router
  // cells of 10 units per capacity cell.
  const BoundingBox chip{
      .minX = 0.0, .minY = 0.0, .maxX = 30000.0, .maxY = 15000.0};
  const GridMetrics capacity = GridMetrics::fit(chip, 50, 25);
  const GridMetrics router = routerGrid(capacity, 10.0);
  EXPECT_EQ(router.width, 3000U);
  EXPECT_EQ(router.height, 1500U);
  EXPECT_NEAR(router.cellWidth, 10.0, 0.01);

  // A capacity cell of 595 units is not a multiple of the division. It rounds
  // up to 60 router cells, so this router cell is narrower than the division.
  const BoundingBox odd{
      .minX = 0.0, .minY = 0.0, .maxX = 29750.0, .maxY = 29750.0};
  const GridMetrics oddRouter = routerGrid(GridMetrics::fit(odd, 50, 50), 10.0);
  EXPECT_EQ(oddRouter.width, 3000U);
  EXPECT_LT(oddRouter.cellWidth, 10.0);
}

TEST(GridMetrics, CellsForReproducesTheClearanceLiterals) {
  const BoundingBox chip{
      .minX = 0.0, .minY = 0.0, .maxX = 29910.0, .maxY = 29910.0};
  const GridMetrics router = routerGrid(GridMetrics::fit(chip, 50, 50), 10.0);
  EXPECT_NEAR(router.cellWidth, 9.97, 0.01);
  // The wire clearance of 185 units is 19 cells on every benchmark. The cell
  // step lies below 10 units, so the coupler footprint of 200 by 26 units
  // takes 21 by 3 cells and the bridge of 60 by 60 units takes 7 by 7.
  EXPECT_EQ(cellsFor(185.0, router), 19U);
  EXPECT_EQ(cellsFor(200.0, router), 21U);
  EXPECT_EQ(cellsFor(26.0, router), 3U);
  EXPECT_EQ(cellsFor(60.0, router), 7U);
  EXPECT_EQ(cellsFor(0.0, router), 0U);

  // At a cell step of 10 units, the footprint takes 20 by 3 cells and the
  // bridge 6 by 6.
  const GridMetrics exact = GridMetrics::fit(chip, 2992, 2992);
  EXPECT_NEAR(exact.cellWidth, 10.0, 1e-9);
  EXPECT_EQ(cellsFor(200.0, exact), 20U);
  EXPECT_EQ(cellsFor(26.0, exact), 3U);
  EXPECT_EQ(cellsFor(60.0, exact), 6U);
}

TEST(GridMetrics, CellsForCountsAWholeNumberOfStepsExactly) {
  // The cell step of this grid is 0.09999999999999999, a rounding error below
  // 0.1.
  const GridMetrics small = GridMetrics::fit(
      BoundingBox{.minX = 0.0, .minY = 0.0, .maxX = 0.3, .maxY = 0.3}, 4, 4);
  EXPECT_LT(small.cellWidth, 0.1);
  EXPECT_EQ(cellsFor(0.1, small), 1U);
  EXPECT_EQ(cellsFor(0.3, small), 3U);
  // A length clearly above a whole number of steps still takes one more cell.
  EXPECT_EQ(cellsFor(0.1 * (1.0 + 1e-6), small), 2U);

  // Every whole multiple of the cell step, on grids of many sizes.
  for (uint32_t cells = 2; cells <= 200; ++cells) {
    for (const double extent : {0.3, 1.0, 7.7, 4000.0, 29910.0, 30000.0}) {
      const GridMetrics grid = GridMetrics::fit(
          BoundingBox{.minX = 0.0, .minY = 0.0, .maxX = extent, .maxY = extent},
          cells, cells);
      for (uint32_t steps = 1; steps <= 25; ++steps) {
        ASSERT_EQ(cellsFor(steps * grid.cellWidth, grid), steps)
            << cells << " cells over " << extent;
      }
    }
  }
  // Rules in tenths on grids whose cell step is a tenth.
  for (uint32_t cells = 2; cells <= 400; ++cells) {
    const double extent = (cells - 1) * 0.1;
    const GridMetrics grid = GridMetrics::fit(
        BoundingBox{.minX = 0.0, .minY = 0.0, .maxX = extent, .maxY = extent},
        cells, cells);
    for (uint32_t tenths = 1; tenths <= 20; ++tenths) {
      ASSERT_EQ(cellsFor(tenths * 0.1, grid), tenths) << cells << " cells";
    }
  }
}

TEST(GridMetrics, TheRouterGridKeepsAWholeNumberOfStepsPerCapacityCell) {
  // Capacity cells of an exact multiple of the division keep that multiple,
  // wherever the box lies.
  for (const uint32_t capacityCells : {20U, 30U, 40U, 50U, 60U, 64U, 100U}) {
    for (uint32_t steps = 1; steps <= 400; ++steps) {
      for (const double minX : {0.0, -800.0, 1234.5, -27000.0}) {
        const double extent = capacityCells * 10.0 * steps;
        const GridMetrics capacity =
            GridMetrics::fit(BoundingBox{.minX = minX,
                                         .minY = minX,
                                         .maxX = minX + extent,
                                         .maxY = minX + extent},
                             capacityCells, capacityCells);
        const GridMetrics router = routerGrid(capacity, 10.0);
        ASSERT_EQ(router.width, capacityCells * steps)
            << capacityCells << " capacity cells of " << steps << " steps from "
            << minX;
        ASSERT_EQ(router.height, capacityCells * steps);
      }
    }
  }
}

TEST(GridMetrics, RefusesMoreCellsPerAxisThanAnIndexHolds) {
  constexpr uint32_t largest = 4294967295U;
  const GridMetrics capacity = GridMetrics::fit(
      BoundingBox{.minX = 0.0, .minY = 0.0, .maxX = 30000.0, .maxY = 30000.0},
      50, 50);
  // 6e8 router cells per capacity cell, 3e10 along each axis.
  EXPECT_THROW(static_cast<void>(routerGrid(capacity, 1e-6)),
               std::length_error);
  // 600 router cells per capacity cell, 30000 along each axis.
  EXPECT_EQ(routerGrid(capacity, 1.0).width, 30000U);

  // Two cells refined by 2^31 - 1 give 2^32 - 2 cells, by 2^31 give 2^32.
  const GridMetrics pair = GridMetrics::fit(BOX, 2, 2);
  EXPECT_EQ(pair.refined(2147483647U).width, largest - 1);
  EXPECT_THROW(static_cast<void>(pair.refined(2147483648U)), std::length_error);
  EXPECT_THROW(static_cast<void>(capacity.refined(100000000U)),
               std::length_error);

  // A box 1e10 times higher than wide.
  const BoundingBox tower{.minX = 0.0, .minY = 0.0, .maxX = 1.0, .maxY = 1e10};
  EXPECT_THROW(static_cast<void>(GridMetrics::fitWidth(tower, 10)),
               std::length_error);
  const BoundingBox tall{.minX = 0.0, .minY = 0.0, .maxX = 1.0, .maxY = 1e8};
  EXPECT_EQ(GridMetrics::fitWidth(tall, 10).height, 1000000000U);

  // A rule of 2^32 cells of one unit, and one of 2^32 - 1.
  const GridMetrics unit = GridMetrics::fit(
      BoundingBox{.minX = 0.0, .minY = 0.0, .maxX = 1.0, .maxY = 1.0}, 2, 2);
  ASSERT_EQ(unit.cellWidth, 1.0);
  EXPECT_EQ(cellsFor(4294967295.0, unit), largest);
  EXPECT_THROW(static_cast<void>(cellsFor(4294967296.0, unit)),
               std::length_error);
}

TEST(GridMetrics, RefusesAGridWithoutACellStep) {
  EXPECT_THROW(static_cast<void>(GridMetrics::fit(BOX, 1, 10)),
               std::invalid_argument);
  const BoundingBox flat{.minX = 0.0, .minY = 0.0, .maxX = 10.0, .maxY = 0.0};
  EXPECT_THROW(static_cast<void>(GridMetrics::fit(flat, 10, 10)),
               std::invalid_argument);
  EXPECT_THROW(static_cast<void>(routerGrid(GridMetrics::fit(BOX, 11, 6), 0.0)),
               std::invalid_argument);
}

TEST(GridMetrics, ChipBoundsSpanObstaclesAndPorts) {
  mqt::scpd::flatbuffers::design::ChipT chip;
  auto polygon = std::make_unique<mqt::scpd::flatbuffers::geometry::PolygonT>();
  polygon->vertices = {Point(0.0, 0.0), Point(100.0, 0.0), Point(100.0, 50.0)};
  chip.obstacles.push_back(std::move(polygon));
  auto port = std::make_unique<mqt::scpd::flatbuffers::design::PortT>();
  port->label = "Chip.port0";
  port->center = Point(-20.0, 80.0);
  chip.ports.push_back(std::move(port));

  EXPECT_EQ(
      chipBounds(chip),
      (BoundingBox{.minX = -20.0, .minY = 0.0, .maxX = 100.0, .maxY = 80.0}));

  const mqt::scpd::flatbuffers::design::ChipT empty;
  EXPECT_THROW(static_cast<void>(chipBounds(empty)), std::invalid_argument);
}

TEST(GridMetrics, FitWidthKeepsAtLeastTwoRows) {
  // A box a thousand times wider than high rounds to no row at all.
  const BoundingBox strip{
      .minX = 0.0, .minY = 0.0, .maxX = 1000.0, .maxY = 1.0};
  const GridMetrics grid = GridMetrics::fitWidth(strip, 10);
  EXPECT_EQ(grid.width, 10U);
  EXPECT_EQ(grid.height, 2U);
  EXPECT_DOUBLE_EQ(grid.cellHeight, 1.0);
}

TEST(GridMetrics, FitWidthRefusesAGridWithoutACellStep) {
  const BoundingBox noWidth{
      .minX = 5.0, .minY = 0.0, .maxX = 5.0, .maxY = 10.0};
  EXPECT_THROW(static_cast<void>(GridMetrics::fitWidth(noWidth, 10)),
               std::invalid_argument);
  const BoundingBox noHeight{
      .minX = 0.0, .minY = 5.0, .maxX = 10.0, .maxY = 5.0};
  EXPECT_THROW(static_cast<void>(GridMetrics::fitWidth(noHeight, 10)),
               std::invalid_argument);
  EXPECT_THROW(static_cast<void>(GridMetrics::fitWidth(BOX, 1)),
               std::invalid_argument);
}

TEST(GridMetrics, RefinementRefusesAZeroFactorAndAGridWithoutACellStep) {
  const GridMetrics grid = GridMetrics::fit(BOX, 11, 6);
  EXPECT_THROW(static_cast<void>(grid.refined(0)), std::invalid_argument);

  const GridMetrics empty;
  EXPECT_THROW(static_cast<void>(empty.refined(2)), std::invalid_argument);
  GridMetrics flat = grid;
  flat.cellHeight = 0.0;
  EXPECT_THROW(static_cast<void>(flat.refined(2)), std::invalid_argument);
}

TEST(GridMetrics, ChipBoundsSkipMissingObstaclesAndPorts) {
  mqt::scpd::flatbuffers::design::ChipT chip;
  chip.obstacles.push_back(nullptr);
  chip.obstacles.push_back(
      std::make_unique<mqt::scpd::flatbuffers::geometry::PolygonT>());
  chip.ports.push_back(nullptr);
  // Neither a missing entry nor a polygon without vertices gives a point.
  EXPECT_THROW(static_cast<void>(chipBounds(chip)), std::invalid_argument);

  auto port = std::make_unique<mqt::scpd::flatbuffers::design::PortT>();
  port->center = Point(5.0, 7.0);
  chip.ports.push_back(std::move(port));
  EXPECT_EQ(chipBounds(chip),
            (BoundingBox{.minX = 5.0, .minY = 7.0, .maxX = 5.0, .maxY = 7.0}));
}

} // namespace
