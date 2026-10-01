/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// The grid over real chip inputs, so that the rasterization, the keepout and
// the distance transform are exercised on real geometry rather than on
// hand-built polygons only. The two-qubit fixture of the repository always
// runs; the benchmark chips run when MQT_SCPD_BENCHMARKS names a clone of
// planar-superconducting-pd.

#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/grid/DistanceTransform.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"
#include "mqt-scpd/grid/Rasterize.hpp"
#include "mqt-scpd/io/Chip.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <numbers>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace {

using namespace mqt::scpd;
using namespace mqt::scpd::grid;
using mqt::scpd::flatbuffers::design::ChipT;

/// The chip input at @p path, or no chip when the file cannot be opened.
std::optional<ChipT> chipAt(const std::string& path) {
  std::ifstream file(path);
  if (!file.is_open()) {
    return std::nullopt;
  }
  const std::string text{std::istreambuf_iterator<char>(file),
                         std::istreambuf_iterator<char>()};
  return io::readChipJson(text);
}

/// The two-qubit fixture of the repository.
ChipT miniChip() {
  auto chip =
      chipAt(std::string(MQT_SCPD_FIXTURE_DIR) + "/mini/routing_config.json");
  EXPECT_TRUE(chip.has_value());
  return chip.value_or(ChipT{});
}

/// The nine-qubit benchmark chip, or no chip when MQT_SCPD_BENCHMARKS is
/// unset.
std::optional<ChipT> nineQubitChip() {
  const char* root = std::getenv("MQT_SCPD_BENCHMARKS");
  if (root == nullptr || *root == '\0') {
    return std::nullopt;
  }
  auto chip = chipAt(std::string(root) + "/inputs/9q/routing_config.json");
  EXPECT_TRUE(chip.has_value()) << "MQT_SCPD_BENCHMARKS=" << root;
  return chip;
}

/// The unit step along an orientation in degrees: the signs of its cosine
/// and sine, where a component within a few hundredths of zero counts as
/// zero.
Point stepAlong(const double orientationDegrees) {
  const double radians = orientationDegrees * (std::numbers::pi / 180.0);
  constexpr double threshold = 0.05;
  const auto sign = [](const double value) {
    if (value > threshold) {
      return 1.0;
    }
    return value < -threshold ? -1.0 : 0.0;
  };
  return {sign(std::cos(radians)), sign(std::sin(radians))};
}

TEST(ChipRaster, TheFixtureRastersIntoItsObstaclesAndLeavesTheRestFree) {
  const ChipT chip = miniChip();
  const GridMetrics grid =
      GridMetrics::fitWidth(chipBounds(chip), 50).refined(10);

  const RasterizedObstacles raster = rasterizeObstacles(chip, grid);
  EXPECT_GT(raster.blocked.count(), 0U);
  EXPECT_LT(raster.blocked.count(), grid.cells());

  // Every obstacle vertex lies on or next to a blocked cell.
  for (const auto& polygon : chip.obstacles) {
    for (const Point& vertex : polygon->vertices) {
      const auto cell = grid.clampToCell(vertex);
      bool near = false;
      for (int dy = -1; dy <= 1 && !near; ++dy) {
        for (int dx = -1; dx <= 1 && !near; ++dx) {
          const auto x = static_cast<int64_t>(cell.x()) + dx;
          const auto y = static_cast<int64_t>(cell.y()) + dy;
          if (x < 0 || y < 0 || std::cmp_greater_equal(x, grid.width) ||
              std::cmp_greater_equal(y, grid.height)) {
            continue;
          }
          near = raster.blocked.testCell(static_cast<uint32_t>(x),
                                         static_cast<uint32_t>(y));
        }
      }
      EXPECT_TRUE(near) << vertex.x() << ", " << vertex.y();
    }
  }
}

TEST(BenchmarkRaster, TheGridCoversTheChipAndLeavesRoomToRoute) {
  const auto chip = nineQubitChip();
  if (!chip) {
    GTEST_SKIP() << "MQT_SCPD_BENCHMARKS is not set";
  }
  const BoundingBox box = chipBounds(*chip);
  EXPECT_GT(box.width(), 0.0);
  EXPECT_GT(box.height(), 0.0);

  // Fifty cells across, as the nine-qubit configuration asks for; the height
  // follows the aspect ratio of the chip.
  const GridMetrics coarse = GridMetrics::fitWidth(box, 50);
  EXPECT_EQ(coarse.width, 50U);
  EXPECT_GT(coarse.height, 10U);

  const GridMetrics detail = coarse.refined(30);
  EXPECT_EQ(detail.width, 1500U);

  const RasterizedObstacles raster = rasterizeObstacles(*chip, detail);
  const std::size_t blocked = raster.blocked.count();
  EXPECT_GT(blocked, 0U);
  // The artwork covers part of the chip, never all of it: what stays free is
  // what a wire can route through.
  EXPECT_LT(blocked, detail.cells() / 2);
}

TEST(BenchmarkRaster, TheKeepoutWidensEveryObstacleAndStaysBounded) {
  const auto chip = nineQubitChip();
  if (!chip) {
    GTEST_SKIP() << "MQT_SCPD_BENCHMARKS is not set";
  }
  const GridMetrics grid =
      GridMetrics::fitWidth(chipBounds(*chip), 50).refined(20);

  const RasterizedObstacles plain = rasterizeObstacles(*chip, grid);
  RasterOptions options;
  options.keepout = 25.0;
  const RasterizedObstacles guarded = rasterizeObstacles(*chip, grid, options);

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
  const auto chip = nineQubitChip();
  if (!chip) {
    GTEST_SKIP() << "MQT_SCPD_BENCHMARKS is not set";
  }
  const GridMetrics grid =
      GridMetrics::fitWidth(chipBounds(*chip), 50).refined(20);

  RasterOptions options;
  options.keepout = 25.0;
  // Every port carries small polygons at its foot, whose keepout would
  // otherwise wall the port in. The corridor is the strip a wire runs along
  // out of the port.
  for (const auto& port : chip->ports) {
    const Point step = stepAlong(port->orientation);
    if (step.x() == 0.0 && step.y() == 0.0) {
      continue;
    }
    const double length = 100.0 + 50.0;
    options.keepoutExemptions.push_back(
        {.from = port->center,
         .to = Point(port->center.x() + (step.x() * length),
                     port->center.y() + (step.y() * length)),
         .halfWidth = 185.0 / 2.0});
  }
  const RasterizedObstacles guarded = rasterizeObstacles(*chip, grid, options);
  EXPECT_GT(guarded.exemptedCells, 0U);
  EXPECT_LT(guarded.exemptedCells, guarded.keepoutCells);
}

TEST(BenchmarkRaster, TheDistanceTransformFindsTheOpenSpace) {
  const auto chip = nineQubitChip();
  if (!chip) {
    GTEST_SKIP() << "MQT_SCPD_BENCHMARKS is not set";
  }
  const GridMetrics grid =
      GridMetrics::fitWidth(chipBounds(*chip), 50).refined(10);
  const RasterizedObstacles raster = rasterizeObstacles(*chip, grid);
  const std::vector<uint32_t> distance =
      squaredDistanceTransform(raster.blocked);

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
  // The widest free space of the chip is many cells across.
  EXPECT_GT(widest, 100U);
}

} // namespace
