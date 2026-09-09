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
#include "mqt-scpd/geometry/Geometry.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>

namespace {

using namespace mqt::scpd::grid;

const BoundingBox BOX{.minX = 100.0, .minY = -50.0, .maxX = 1100.0, .maxY = 450.0};

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
  EXPECT_EQ(grid.index(3, 4), 4U * 101U + 3U);
  EXPECT_EQ(grid.cell(4U * 101U + 3U), DCoord(3, 4));
  EXPECT_TRUE(grid.contains(100, 50));
  EXPECT_FALSE(grid.contains(101, 0));
  EXPECT_FALSE(grid.contains(-1, 0));
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
  const BoundingBox chip{.minX = 0.0, .minY = 0.0, .maxX = 30000.0, .maxY = 15000.0};
  const GridMetrics capacity = GridMetrics::fit(chip, 50, 25);
  const GridMetrics router = routerGrid(capacity, 10.0);
  EXPECT_EQ(router.width, 3000U);
  EXPECT_EQ(router.height, 1500U);
  EXPECT_NEAR(router.cellWidth, 10.0, 0.01);

  // A capacity cell that is not a multiple of the division rounds up, so a
  // router cell is never wider than the division.
  const BoundingBox odd{.minX = 0.0, .minY = 0.0, .maxX = 29750.0, .maxY = 29750.0};
  const GridMetrics oddRouter = routerGrid(GridMetrics::fit(odd, 50, 50), 10.0);
  EXPECT_EQ(oddRouter.width, 3000U);
  EXPECT_LT(oddRouter.cellWidth, 10.0);
}

TEST(GridMetrics, CellsForReproducesTheClearanceLiterals) {
  const BoundingBox chip{.minX = 0.0, .minY = 0.0, .maxX = 29910.0, .maxY = 29910.0};
  const GridMetrics router = routerGrid(GridMetrics::fit(chip, 50, 50), 10.0);
  EXPECT_NEAR(router.cellWidth, 9.97, 0.01);
  // The wire clearance of 185 units is 19 cells on every benchmark, the
  // coupler footprint 20 by 3 cells, the bridge 6 by 6.
  EXPECT_EQ(cellsFor(185.0, router), 19U);
  EXPECT_EQ(cellsFor(200.0, router), 21U);
  EXPECT_EQ(cellsFor(26.0, router), 3U);
  EXPECT_EQ(cellsFor(60.0, router), 7U);
  EXPECT_EQ(cellsFor(0.0, router), 0U);

  const GridMetrics exact = GridMetrics::fit(chip, 2992, 2992);
  EXPECT_NEAR(exact.cellWidth, 10.0, 1e-9);
  EXPECT_EQ(cellsFor(200.0, exact), 20U);
  EXPECT_EQ(cellsFor(60.0, exact), 6U);
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

  EXPECT_EQ(chipBounds(chip),
            (BoundingBox{.minX = -20.0, .minY = 0.0, .maxX = 100.0, .maxY = 80.0}));

  const mqt::scpd::flatbuffers::design::ChipT empty;
  EXPECT_THROW(static_cast<void>(chipBounds(empty)), std::invalid_argument);
}

} // namespace
