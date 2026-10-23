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
// runs. The benchmark chips run when MQT_SCPD_BENCHMARKS is set: it must name
// a clone of planar-superconducting-pd, or the tests fail.

#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/flatbuffers/geometry.hpp"
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
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace {

using namespace mqt::scpd;
using namespace mqt::scpd::grid;
using mqt::scpd::flatbuffers::design::ChipT;
using mqt::scpd::flatbuffers::geometry::PolygonT;

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

/// The area and the perimeter of a polygon, in layout units.
std::pair<double, double> areaAndPerimeter(const std::vector<Point>& vertices) {
  double twiceArea = 0.0;
  double perimeter = 0.0;
  for (std::size_t i = 0; i < vertices.size(); ++i) {
    const Point& a = vertices[i];
    const Point& b = vertices[(i + 1) % vertices.size()];
    twiceArea += (a.x() * b.y()) - (b.x() * a.y());
    perimeter += std::hypot(b.x() - a.x(), b.y() - a.y());
  }
  return {std::fabs(twiceArea) / 2.0, perimeter};
}

/// The distance from @p point to the segment from @p from to @p to, from the
/// projection onto the line through the segment, clamped to the segment.
double referenceDistance(const Point point, const Point from, const Point to) {
  const double dx = to.x() - from.x();
  const double dy = to.y() - from.y();
  const double px = point.x() - from.x();
  const double py = point.y() - from.y();
  const double length2 = (dx * dx) + (dy * dy);
  const double t = length2 > 0.0
                       ? std::clamp(((px * dx) + (py * dy)) / length2, 0.0, 1.0)
                       : 0.0;
  const double ex = px - (t * dx);
  const double ey = py - (t * dy);
  return std::sqrt((ex * ex) + (ey * ey));
}

/// The margin, in layout units, by which the reference distance may miss the
/// limit on the wrong side. The reference rounds differently from the raster,
/// so a center within rounding of the limit may fall on either side.
constexpr double SLACK = 1e-9;

/// A segment from its first point to its second, in layout units.
using Segment = std::pair<Point, Point>;

/// The distance from every cell center of @p grid to the nearest of
/// @p segments, row-major, from referenceDistance(). The function measures
/// only the cells near a segment. A cell farther than @p reach from every
/// segment holds a value above @p reach, or infinity.
std::vector<double> nearestSegment(const GridMetrics& grid,
                                   const std::vector<Segment>& segments,
                                   const double reach) {
  std::vector<double> nearest(grid.cells(),
                              std::numeric_limits<double>::infinity());
  for (const auto& [from, to] : segments) {
    const auto low =
        grid.clampToCell(Point(std::min(from.x(), to.x()) - reach,
                               std::min(from.y(), to.y()) - reach));
    const auto high =
        grid.clampToCell(Point(std::max(from.x(), to.x()) + reach,
                               std::max(from.y(), to.y()) + reach));
    for (uint32_t y = low.y(); y <= high.y(); ++y) {
      for (uint32_t x = low.x(); x <= high.x(); ++x) {
        const std::size_t i = grid.index(x, y);
        nearest[i] = std::min(
            nearest[i], referenceDistance(grid.toLayout(static_cast<double>(x),
                                                        static_cast<double>(y)),
                                          from, to));
      }
    }
  }
  return nearest;
}

/// The edges of every obstacle polygon of @p chip.
std::vector<Segment> obstacleEdges(const ChipT& chip) {
  std::vector<Segment> edges;
  for (const auto& polygon : chip.obstacles) {
    const auto& vertices = polygon->vertices;
    for (std::size_t i = 0; i < vertices.size(); ++i) {
      edges.emplace_back(vertices[i], vertices[(i + 1) % vertices.size()]);
    }
  }
  return edges;
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

/// The nine-qubit benchmark chip. The tests are skipped when
/// MQT_SCPD_BENCHMARKS is unset, and they fail when it is empty or names no
/// clone of planar-superconducting-pd.
class BenchmarkRaster : public ::testing::Test {
protected:
  void SetUp() override {
    const char* root = std::getenv("MQT_SCPD_BENCHMARKS");
    if (root == nullptr) {
      GTEST_SKIP() << "MQT_SCPD_BENCHMARKS is not set";
    }
    ASSERT_NE(*root, '\0') << "MQT_SCPD_BENCHMARKS is set but empty";
    const std::string path =
        std::string(root) + "/inputs/9q/routing_config.json";
    auto loaded = chipAt(path);
    ASSERT_TRUE(loaded.has_value())
        << "MQT_SCPD_BENCHMARKS names no clone of planar-superconducting-pd: "
        << path << " cannot be opened";
    chip = std::move(*loaded);
  }

  ChipT chip;
};

TEST_F(BenchmarkRaster, TheGridCoversTheChipAndEveryPolygonBlocksItsArea) {
  const BoundingBox box = chipBounds(chip);
  ASSERT_GT(box.width(), 0.0);
  ASSERT_GT(box.height(), 0.0);

  // Fifty cells across. The height follows the aspect ratio of the chip.
  const GridMetrics coarse = GridMetrics::fitWidth(box, 50);
  EXPECT_EQ(coarse.width, 50U);
  EXPECT_EQ(coarse.height, static_cast<uint32_t>(
                               std::lround(50.0 * box.height() / box.width())));

  const GridMetrics detail = coarse.refined(30);
  EXPECT_EQ(detail.width, 1500U);
  EXPECT_EQ(detail.height, coarse.height * 30U);

  const RasterizedObstacles raster = rasterizeObstacles(chip, detail);
  ASSERT_FALSE(chip.obstacles.empty());
  const double cellArea = detail.cellWidth * detail.cellHeight;
  const double cellLength = std::min(detail.cellWidth, detail.cellHeight);
  for (std::size_t p = 0; p < chip.obstacles.size(); ++p) {
    const PolygonT& polygon = *chip.obstacles[p];
    ChipT alone;
    alone.obstacles.push_back(std::make_unique<PolygonT>(polygon));
    const BitGrid own = rasterizeObstacles(alone, detail).blocked;

    // The fill blocks every cell whose center lies inside the polygon, and the
    // cells of its outline. Only the cells along an edge can disagree with the
    // area, about one cell per cell length of edge. So the blocked cells match
    // the area in cells to within the perimeter in cells.
    const auto [area, perimeter] = areaAndPerimeter(polygon.vertices);
    EXPECT_NEAR(static_cast<double>(own.count()), area / cellArea,
                perimeter / cellLength)
        << "polygon " << p;

    // The chip blocks every cell that the polygon blocks on its own.
    const auto& ownWords = own.words();
    const auto& chipWords = raster.blocked.words();
    for (std::size_t w = 0; w < ownWords.size(); ++w) {
      ASSERT_EQ(ownWords[w] & ~chipWords[w], 0U) << "polygon " << p;
    }
  }
}

TEST_F(BenchmarkRaster, TheKeepoutWidensEveryObstacleByExactlyItsDistance) {
  const GridMetrics grid =
      GridMetrics::fitWidth(chipBounds(chip), 50).refined(20);

  const RasterizedObstacles plain = rasterizeObstacles(chip, grid);
  RasterOptions options;
  // The obstacle spacing of the nine-qubit design rules.
  options.keepout = 25.0;
  const RasterizedObstacles guarded = rasterizeObstacles(chip, grid, options);

  EXPECT_GT(guarded.blocked.count(), plain.blocked.count());
  EXPECT_EQ(guarded.blocked.count(),
            plain.blocked.count() + guarded.keepoutCells);
  EXPECT_EQ(guarded.exemptedCells, 0U);
  // Every cell the plain raster blocked is still blocked.
  for (std::size_t i = 0; i < plain.blocked.size(); ++i) {
    if (plain.blocked.test(i)) {
      ASSERT_TRUE(guarded.blocked.test(i)) << i;
    }
  }

  // The keepout blocks a cell exactly when its center lies within the keepout
  // of an obstacle edge.
  const std::vector<double> nearest =
      nearestSegment(grid, obstacleEdges(chip), options.keepout);
  for (std::size_t i = 0; i < grid.cells(); ++i) {
    if (plain.blocked.test(i)) {
      continue;
    }
    if (guarded.blocked.test(i)) {
      ASSERT_LE(nearest[i], options.keepout + SLACK) << i;
    } else {
      ASSERT_GT(nearest[i], options.keepout - SLACK) << i;
    }
  }
}

TEST_F(BenchmarkRaster, APortCorridorReleasesTheKeepoutAroundItsPort) {
  const GridMetrics grid =
      GridMetrics::fitWidth(chipBounds(chip), 50).refined(20);

  const RasterizedObstacles plain = rasterizeObstacles(chip, grid);
  RasterOptions options;
  // The obstacle spacing of the nine-qubit design rules.
  options.keepout = 25.0;
  const RasterizedObstacles walled = rasterizeObstacles(chip, grid, options);
  // Every port carries small polygons at its foot, whose keepout would
  // otherwise wall the port in. The port corridor is the strip a wire runs
  // along out of the port: as long as the shortest straight part plus the bend
  // radius, and as wide as the wire spacing.
  constexpr double length = 100.0 + 50.0;
  for (const auto& port : chip.ports) {
    const Point step = stepAlong(port->orientation);
    if (step.x() == 0.0 && step.y() == 0.0) {
      continue;
    }
    options.keepoutExemptions.push_back(
        {.from = port->center,
         .to = Point(port->center.x() + (step.x() * length),
                     port->center.y() + (step.y() * length)),
         .halfWidth = 185.0 / 2.0});
  }
  ASSERT_FALSE(options.keepoutExemptions.empty());
  const RasterizedObstacles guarded = rasterizeObstacles(chip, grid, options);
  EXPECT_GT(guarded.exemptedCells, 0U);

  // Walking out of each port along its orientation, the first cell that its
  // polygons leave free lies in the keepout, and the port corridor frees it.
  const double halfCell = grid.cellWidth / 2.0;
  for (const auto& port : chip.ports) {
    const Point step = stepAlong(port->orientation);
    if (step.x() == 0.0 && step.y() == 0.0) {
      continue;
    }
    const double unit = 1.0 / std::hypot(step.x(), step.y());
    std::optional<std::size_t> first;
    for (int k = 0; !first && k * halfCell <= length; ++k) {
      const double walked = k * halfCell * unit;
      const auto cell =
          grid.clampToCell(Point(port->center.x() + (step.x() * walked),
                                 port->center.y() + (step.y() * walked)));
      const std::size_t i = grid.index(cell.x(), cell.y());
      if (!plain.blocked.test(i)) {
        first = i;
      }
    }
    ASSERT_TRUE(first.has_value()) << port->label;
    EXPECT_TRUE(walled.blocked.test(*first)) << port->label;
    EXPECT_FALSE(guarded.blocked.test(*first)) << port->label;
  }

  // A port corridor frees a keepout cell exactly when the cell center lies
  // within the half width of the port corridor.
  std::vector<Segment> strips;
  strips.reserve(options.keepoutExemptions.size());
  for (const PortCorridor& portCorridor : options.keepoutExemptions) {
    strips.emplace_back(portCorridor.from, portCorridor.to);
  }
  const double halfWidth = options.keepoutExemptions.front().halfWidth;
  const std::vector<double> nearest = nearestSegment(grid, strips, halfWidth);
  for (std::size_t i = 0; i < grid.cells(); ++i) {
    if (plain.blocked.test(i) || !walled.blocked.test(i)) {
      continue;
    }
    if (guarded.blocked.test(i)) {
      ASSERT_GT(nearest[i], halfWidth - SLACK) << i;
    } else {
      ASSERT_LE(nearest[i], halfWidth + SLACK) << i;
    }
  }
}

TEST_F(BenchmarkRaster, TheDistanceTransformFindsTheOpenSpace) {
  const GridMetrics grid =
      GridMetrics::fitWidth(chipBounds(chip), 50).refined(10);
  const RasterizedObstacles raster = rasterizeObstacles(chip, grid);
  const std::vector<uint32_t> distance =
      squaredDistanceTransform(raster.blocked);

  ASSERT_EQ(distance.size(), grid.cells());
  std::vector<std::pair<int64_t, int64_t>> blockedCells;
  std::size_t widest = 0;
  for (std::size_t i = 0; i < distance.size(); ++i) {
    if (raster.blocked.test(i)) {
      EXPECT_EQ(distance[i], 0U) << i;
      blockedCells.emplace_back(static_cast<int64_t>(i % grid.width),
                                static_cast<int64_t>(i / grid.width));
    } else {
      EXPECT_GT(distance[i], 0U) << i;
      if (distance[i] > distance[widest]) {
        widest = i;
      }
    }
  }
  ASSERT_FALSE(blockedCells.empty());

  // The squared distance of a free cell is that to the nearest blocked cell,
  // counted cell by cell: at the widest free space and at cells spread over
  // the grid.
  std::vector<std::size_t> samples = {widest};
  for (std::size_t i = 0; i < distance.size(); i += 499) {
    samples.push_back(i);
  }
  for (const std::size_t i : samples) {
    const auto x = static_cast<int64_t>(i % grid.width);
    const auto y = static_cast<int64_t>(i / grid.width);
    int64_t reference = DISTANCE_UNBOUNDED;
    for (const auto& [bx, by] : blockedCells) {
      reference =
          std::min(reference, ((bx - x) * (bx - x)) + ((by - y) * (by - y)));
    }
    EXPECT_EQ(static_cast<int64_t>(distance[i]), reference) << i;
  }
}

} // namespace
