/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// The grid over a benchmark chip the repository carries, so that the
// rasterization, the keepout and the distance transform are exercised on
// real geometry rather than on hand-built polygons only.

#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/geometry/Geometry.hpp"
#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/grid/DistanceTransform.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"
#include "mqt-scpd/grid/PortBands.hpp"
#include "mqt-scpd/grid/Rasterize.hpp"
#include "mqt-scpd/io/Chip.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

namespace {

using namespace mqt::scpd;
using namespace mqt::scpd::grid;
using mqt::scpd::flatbuffers::design::ChipT;

const std::string BENCHMARKS = MQT_SCPD_BENCHMARK_DIR;

/// The nine-qubit chip, whose input the repository carries.
ChipT nineQubitChip() {
  std::ifstream file(BENCHMARKS + "/9q/routing_config.json");
  EXPECT_TRUE(file.is_open());
  const std::string text{std::istreambuf_iterator<char>(file),
                         std::istreambuf_iterator<char>()};
  return io::readChipJson(text);
}

TEST(BenchmarkRaster, TheCapacityGridCoversTheChipAndLeavesRoomToRoute) {
  const ChipT chip = nineQubitChip();
  const BoundingBox box = chipBounds(chip);
  EXPECT_GT(box.width(), 0.0);
  EXPECT_GT(box.height(), 0.0);

  // The nine-qubit configuration asks for fifty cells across, and the
  // height follows the aspect ratio of the chip.
  const GridMetrics capacity = GridMetrics::fitWidth(box, 50);
  EXPECT_EQ(capacity.width, 50U);
  EXPECT_GT(capacity.height, 10U);

  // The detail grid of the capacity stage, thirty cells per capacity cell.
  const GridMetrics detail = capacity.refined(30);
  EXPECT_EQ(detail.width, 1500U);

  const RasterizedObstacles raster = rasterizeObstacles(chip, detail);
  const std::size_t blocked = raster.blocked.count();
  EXPECT_GT(blocked, 0U);
  // The artwork covers part of the chip, never all of it: what stays free
  // is what the later stages route through.
  EXPECT_LT(blocked, detail.cells() / 2);
}

TEST(BenchmarkRaster, TheKeepoutWidensEveryObstacleAndStaysBounded) {
  const ChipT chip = nineQubitChip();
  const GridMetrics grid = GridMetrics::fitWidth(chipBounds(chip), 50).refined(20);

  const RasterizedObstacles plain = rasterizeObstacles(chip, grid);
  RasterOptions options;
  options.keepout = 25.0;
  const RasterizedObstacles guarded = rasterizeObstacles(chip, grid, options);

  EXPECT_GT(guarded.blocked.count(), plain.blocked.count());
  EXPECT_GT(guarded.keepoutCells, 0U);
  EXPECT_EQ(guarded.exemptedCells, 0U);
  // Every cell the plain raster blocked is still blocked, and the keepout
  // does not swallow the chip.
  for (std::size_t i = 0; i < plain.blocked.size(); ++i) {
    if (plain.blocked.test(i)) {
      ASSERT_TRUE(guarded.blocked.test(i)) << i;
    }
  }
  EXPECT_LT(guarded.blocked.count(), grid.cells() * 3 / 4);
}

TEST(BenchmarkRaster, APortCorridorReleasesTheKeepoutAroundItsPort) {
  const ChipT chip = nineQubitChip();
  const BoundingBox box = chipBounds(chip);
  const GridMetrics grid = GridMetrics::fitWidth(box, 50).refined(20);

  RasterOptions options;
  options.keepout = 25.0;
  // Every port carries small polygons at its foot, whose keepout would
  // otherwise wall the port in. The corridor is the strip the port's own
  // band will occupy.
  for (const auto& port : chip.ports) {
    const Step step = orientationStep(port->orientation);
    if (step.x == 0 && step.y == 0) {
      continue;
    }
    const double length = 100.0 + 50.0;
    options.keepoutExemptions.push_back(
        {.from = port->center,
         .to = Point(port->center.x() + (step.x * length), port->center.y() + (step.y * length)),
         .halfWidth = 185.0 / 2.0});
  }
  const RasterizedObstacles guarded = rasterizeObstacles(chip, grid, options);
  EXPECT_GT(guarded.exemptedCells, 0U);
  EXPECT_LT(guarded.exemptedCells, guarded.keepoutCells);
}

TEST(BenchmarkRaster, TheDistanceTransformFindsTheOpenSpace) {
  const ChipT chip = nineQubitChip();
  const GridMetrics grid = GridMetrics::fitWidth(chipBounds(chip), 50).refined(10);
  const RasterizedObstacles raster = rasterizeObstacles(chip, grid);
  const std::vector<uint32_t> distance = squaredDistanceTransform(raster.blocked);

  ASSERT_EQ(distance.size(), grid.cells());
  uint32_t widest = 0;
  for (std::size_t i = 0; i < distance.size(); ++i) {
    if (raster.blocked.test(i)) {
      EXPECT_EQ(distance[i], 0U) << i;
    } else {
      EXPECT_GT(distance[i], 0U) << i;
      if (distance[i] != DISTANCE_UNBOUNDED) {
        widest = std::max(widest, distance[i]);
      }
    }
  }
  // The widest free space of the chip is many cells across, which is where
  // the partitioning of the capacity stage puts its seeds.
  EXPECT_GT(widest, 100U);
}

} // namespace
