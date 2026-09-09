/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/grid/Rasterize.hpp"

#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/flatbuffers/geometry.hpp"
#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace mqt::scpd::grid {

namespace {

using flatbuffers::design::ChipT;
using flatbuffers::geometry::PolygonT;

/// The keepout marks cells with 2 before the corridors release some of them,
/// so that a corridor can never free a cell of the polygons themselves.
enum : uint8_t { FREE = 0, POLYGON = 1, KEEPOUT = 2 };

std::vector<uint8_t> keepoutMask(const ChipT& chip, const GridMetrics& grid,
                                 const RasterOptions& options,
                                 const BitGrid& blocked, std::size_t& keepoutCells,
                                 std::size_t& exemptedCells) {
  std::vector<uint8_t> cells(grid.cells(), FREE);
  for (std::size_t i = 0; i < cells.size(); ++i) {
    cells[i] = blocked.test(i) ? POLYGON : FREE;
  }

  // Visit the cells of a window around an edge, given in layout units.
  const auto forEachCellNear = [&](const Point a, const Point b,
                                   const double halfWidth, auto&& visit) {
    const Point ca = grid.toCell(a);
    const Point cb = grid.toCell(b);
    const auto extraX =
        static_cast<int64_t>(std::ceil(halfWidth / grid.cellWidth)) + 1;
    const auto extraY =
        static_cast<int64_t>(std::ceil(halfWidth / grid.cellHeight)) + 1;
    const int64_t x0 = std::max<int64_t>(
        0, static_cast<int64_t>(std::floor(std::min(ca.x(), cb.x()))) - extraX);
    const int64_t x1 = std::min<int64_t>(
        grid.width - 1,
        static_cast<int64_t>(std::ceil(std::max(ca.x(), cb.x()))) + extraX);
    const int64_t y0 = std::max<int64_t>(
        0, static_cast<int64_t>(std::floor(std::min(ca.y(), cb.y()))) - extraY);
    const int64_t y1 = std::min<int64_t>(
        grid.height - 1,
        static_cast<int64_t>(std::ceil(std::max(ca.y(), cb.y()))) + extraY);
    for (int64_t y = y0; y <= y1; ++y) {
      for (int64_t x = x0; x <= x1; ++x) {
        const Point center =
            grid.toLayout(static_cast<double>(x), static_cast<double>(y));
        if (distanceToSegment(center, a, b) <= halfWidth) {
          visit(grid.index(static_cast<uint32_t>(x), static_cast<uint32_t>(y)));
        }
      }
    }
  };

  for (std::size_t p = 1; p < chip.obstacles.size(); ++p) {
    const auto* const polygon = chip.obstacles[p].get();
    if (polygon == nullptr || polygon->vertices.size() < 2) {
      continue;
    }
    const auto& vertices = polygon->vertices;
    for (std::size_t i = 0; i < vertices.size(); ++i) {
      const Point a = vertices[i];
      const Point b = vertices[(i + 1) % vertices.size()];
      forEachCellNear(a, b, options.keepout, [&](const std::size_t index) {
        if (cells[index] == FREE) {
          cells[index] = KEEPOUT;
          ++keepoutCells;
        }
      });
    }
  }
  for (const auto& corridor : options.keepoutExemptions) {
    forEachCellNear(corridor.from, corridor.to, corridor.halfWidth,
                    [&](const std::size_t index) {
                      if (cells[index] == KEEPOUT) {
                        cells[index] = FREE;
                        ++exemptedCells;
                      }
                    });
  }
  return cells;
}

} // namespace

double distanceToSegment(const Point point, const Point from, const Point to) {
  const double dx = to.x() - from.x();
  const double dy = to.y() - from.y();
  const double length2 = (dx * dx) + (dy * dy);
  double t = 0.0;
  if (length2 > 1e-18) {
    t = (((point.x() - from.x()) * dx) + ((point.y() - from.y()) * dy)) / length2;
    t = std::clamp(t, 0.0, 1.0);
  }
  const double cx = from.x() + (t * dx);
  const double cy = from.y() + (t * dy);
  return std::hypot(point.x() - cx, point.y() - cy);
}

std::vector<std::size_t> lineCells(int64_t x0, int64_t y0, const int64_t x1,
                                   const int64_t y1, const uint32_t width,
                                   const uint32_t height) {
  std::vector<std::size_t> cells;
  const int64_t dx = std::abs(x1 - x0);
  const int64_t dy = -std::abs(y1 - y0);
  const int64_t sx = (x0 < x1) ? 1 : -1;
  const int64_t sy = (y0 < y1) ? 1 : -1;
  int64_t err = dx + dy;
  while (true) {
    if (x0 >= 0 && y0 >= 0 && x0 < static_cast<int64_t>(width) &&
        y0 < static_cast<int64_t>(height)) {
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

void fillPolygon(BitGrid& mask, const GridMetrics& grid,
                 const PolygonT& polygon) {
  const auto& vertices = polygon.vertices;
  if (vertices.empty()) {
    return;
  }
  std::vector<Point> nodes;
  nodes.reserve(vertices.size());
  int64_t minX = grid.width;
  int64_t maxX = 0;
  int64_t minY = grid.height;
  int64_t maxY = 0;
  for (const auto& vertex : vertices) {
    const Point node = grid.toCell(vertex);
    nodes.push_back(node);
    minX = std::min(minX, static_cast<int64_t>(std::floor(node.x())));
    maxX = std::max(maxX, static_cast<int64_t>(std::ceil(node.x())));
    minY = std::min(minY, static_cast<int64_t>(std::floor(node.y())));
    maxY = std::max(maxY, static_cast<int64_t>(std::ceil(node.y())));
  }
  minX = std::max<int64_t>(0, minX);
  minY = std::max<int64_t>(0, minY);
  maxX = std::min<int64_t>(grid.width - 1, maxX);
  maxY = std::min<int64_t>(grid.height - 1, maxY);

  // The area: every cell whose center lies inside, by ray casting. The
  // window is the polygon's own box, so a polygon cannot flood the grid.
  if (nodes.size() >= 3) {
    for (int64_t y = minY; y <= maxY; ++y) {
      const auto py = static_cast<double>(y);
      for (int64_t x = minX; x <= maxX; ++x) {
        const auto px = static_cast<double>(x);
        bool inside = false;
        for (std::size_t i = 0, j = nodes.size() - 1; i < nodes.size(); j = i++) {
          const double xi = nodes[i].x();
          const double yi = nodes[i].y();
          const double xj = nodes[j].x();
          const double yj = nodes[j].y();
          const bool crosses = ((yi > py) != (yj > py)) &&
                               (px < ((xj - xi) * (py - yi) / (yj - yi)) + xi);
          if (crosses) {
            inside = !inside;
          }
        }
        if (inside) {
          mask.setCell(static_cast<uint32_t>(x), static_cast<uint32_t>(y));
        }
      }
    }
  }

  // The edges: a line of cells between the rounded vertex cells, so that the
  // outline has no gap where the area test misses a thin polygon.
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    const Point a = nodes[i];
    const Point b = nodes[(i + 1) % nodes.size()];
    for (const std::size_t index :
         lineCells(std::llround(a.x()), std::llround(a.y()), std::llround(b.x()),
                   std::llround(b.y()), grid.width, grid.height)) {
      mask.set(index);
    }
  }
}

void removeIslands(BitGrid& mask) {
  const uint32_t width = mask.width();
  const uint32_t height = mask.height();
  if (width < 3 || height < 3) {
    return;
  }
  const BitGrid before = mask;
  for (uint32_t y = 1; y + 1 < height; ++y) {
    for (uint32_t x = 1; x + 1 < width; ++x) {
      if (!before.testCell(x, y)) {
        continue;
      }
      int freeNeighbors = 0;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (!before.testCell(static_cast<uint32_t>(static_cast<int64_t>(x) + dx),
                           static_cast<uint32_t>(static_cast<int64_t>(y) + dy))) {
            ++freeNeighbors;
          }
        }
      }
      if (freeNeighbors >= 7) {
        mask.setCell(x, y, false);
      }
    }
  }
}

void blockBorder(BitGrid& mask, const uint32_t cells) {
  if (cells == 0) {
    return;
  }
  const uint32_t width = mask.width();
  const uint32_t height = mask.height();
  for (uint32_t y = 0; y < height; ++y) {
    for (uint32_t x = 0; x < width; ++x) {
      if (x < cells || x + cells >= width || y < cells || y + cells >= height) {
        mask.setCell(x, y);
      }
    }
  }
}

RasterizedObstacles rasterizeObstacles(const ChipT& chip,
                                       const GridMetrics& grid,
                                       const RasterOptions& options) {
  RasterizedObstacles result;
  result.blocked = BitGrid(grid.width, grid.height);
  BitGrid& blocked = result.blocked;

  // The first polygon is the chip outline.
  for (std::size_t p = 1; p < chip.obstacles.size(); ++p) {
    if (chip.obstacles[p] != nullptr) {
      fillPolygon(blocked, grid, *chip.obstacles[p]);
    }
  }
  removeIslands(blocked);

  if (options.keepout > 0.0) {
    const std::vector<uint8_t> cells =
        keepoutMask(chip, grid, options, blocked, result.keepoutCells,
                    result.exemptedCells);
    for (std::size_t i = 0; i < cells.size(); ++i) {
      if (cells[i] != FREE) {
        blocked.set(i);
      }
    }
  }
  blockBorder(blocked, options.border);
  return result;
}

} // namespace mqt::scpd::grid
