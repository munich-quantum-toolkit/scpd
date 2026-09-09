/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/grid/GridMetrics.hpp"

#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/flatbuffers/geometry.hpp"
#include "mqt-scpd/geometry/Geometry.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>

namespace mqt::scpd::grid {

GridMetrics GridMetrics::fit(const BoundingBox& box, const uint32_t width,
                             const uint32_t height) {
  if (width < 2 || height < 2) {
    throw std::invalid_argument("a grid needs at least two cells per axis");
  }
  if (!(box.width() > 0.0) || !(box.height() > 0.0)) {
    throw std::invalid_argument("a grid needs a box with a positive extent");
  }
  return {.width = width,
          .height = height,
          .origin = Point(box.minX, box.minY),
          .cellWidth = box.width() / (width - 1),
          .cellHeight = box.height() / (height - 1)};
}

GridMetrics GridMetrics::fitWidth(const BoundingBox& box, const uint32_t width) {
  if (!(box.width() > 0.0)) {
    throw std::invalid_argument("a grid needs a box with a positive extent");
  }
  const auto height = static_cast<uint32_t>(
      std::llround(width * (box.height() / box.width())));
  return fit(box, width, std::max<uint32_t>(height, 2));
}

GridMetrics GridMetrics::refined(const uint32_t factor) const {
  if (factor == 0) {
    throw std::invalid_argument("a refinement factor must be at least one");
  }
  return fit(box(), width * factor, height * factor);
}

BoundingBox GridMetrics::box() const {
  return {.minX = origin.x(),
          .minY = origin.y(),
          .maxX = origin.x() + (cellWidth * (width - 1)),
          .maxY = origin.y() + (cellHeight * (height - 1))};
}

Point GridMetrics::toCell(const Point point) const {
  return {(point.x() - origin.x()) / cellWidth,
          (point.y() - origin.y()) / cellHeight};
}

Point GridMetrics::toLayout(const double x, const double y) const {
  return {origin.x() + (x * cellWidth), origin.y() + (y * cellHeight)};
}

std::optional<DCoord> GridMetrics::roundToCell(const Point point) const {
  const Point cell = toCell(point);
  const auto x = std::llround(cell.x());
  const auto y = std::llround(cell.y());
  if (!contains(x, y)) {
    return std::nullopt;
  }
  return DCoord(static_cast<uint32_t>(x), static_cast<uint32_t>(y));
}

DCoord GridMetrics::clampToCell(const Point point) const {
  const Point cell = toCell(point);
  const auto x = std::clamp<int64_t>(std::llround(cell.x()), 0, width - 1);
  const auto y = std::clamp<int64_t>(std::llround(cell.y()), 0, height - 1);
  return {static_cast<uint32_t>(x), static_cast<uint32_t>(y)};
}

GridMetrics routerGrid(const GridMetrics& capacity, const double unitDivision) {
  if (!(unitDivision > 0.0)) {
    throw std::invalid_argument("the router cell size must be positive");
  }
  const BoundingBox box = capacity.box();
  const double capacityCellWidth = box.width() / capacity.width;
  const double capacityCellHeight = box.height() / capacity.height;
  const auto perCellX =
      static_cast<uint32_t>(std::ceil(capacityCellWidth / unitDivision));
  const auto perCellY =
      static_cast<uint32_t>(std::ceil(capacityCellHeight / unitDivision));
  return GridMetrics::fit(box, capacity.width * std::max<uint32_t>(perCellX, 1),
                          capacity.height * std::max<uint32_t>(perCellY, 1));
}

uint32_t cellsFor(const double distance, const GridMetrics& grid) {
  if (!(distance > 0.0)) {
    return 0;
  }
  const double step = std::min(grid.cellWidth, grid.cellHeight);
  return static_cast<uint32_t>(std::ceil(distance / step));
}

BoundingBox chipBounds(const flatbuffers::design::ChipT& chip) {
  std::optional<BoundingBox> box;
  const auto extend = [&](const Point point) {
    if (box) {
      box->extend(point);
    } else {
      box = BoundingBox{.minX = point.x(),
                        .minY = point.y(),
                        .maxX = point.x(),
                        .maxY = point.y()};
    }
  };
  for (const auto& obstacle : chip.obstacles) {
    if (obstacle == nullptr) {
      continue;
    }
    for (const auto& vertex : obstacle->vertices) {
      extend(vertex);
    }
  }
  for (const auto& port : chip.ports) {
    if (port != nullptr) {
      extend(port->center);
    }
  }
  if (!box) {
    throw std::invalid_argument(
        "a chip without vertices and ports has no bounds");
  }
  return *box;
}

} // namespace mqt::scpd::grid
