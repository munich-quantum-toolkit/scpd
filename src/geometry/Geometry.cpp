/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/geometry/Geometry.hpp"

#include "mqt-scpd/flatbuffers/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <span>
#include <stdexcept>

namespace mqt::scpd::geometry {

Point subtract(const Point to, const Point from) {
  return {to.x() - from.x(), to.y() - from.y()};
}

Point add(const Point a, const Point b) { return {a.x() + b.x(), a.y() + b.y()}; }

Point midpoint(const Point a, const Point b) {
  return {0.5 * (a.x() + b.x()), 0.5 * (a.y() + b.y())};
}

double dot(const Point a, const Point b) {
  return (a.x() * b.x()) + (a.y() * b.y());
}

double cross(const Point a, const Point b) {
  return (a.x() * b.y()) - (a.y() * b.x());
}

double norm(const Point vector) { return std::hypot(vector.x(), vector.y()); }

double distance(const Point a, const Point b) { return norm(subtract(a, b)); }

double angleOf(const Point vector) { return std::atan2(vector.y(), vector.x()); }

double normalizeAngle(const double radians) {
  constexpr double tau = 2.0 * std::numbers::pi;
  const double folded = std::fmod(radians, tau);
  return folded < 0.0 ? folded + tau : folded;
}

Point BoundingBox::center() const {
  return {0.5 * (minX + maxX), 0.5 * (minY + maxY)};
}

void BoundingBox::extend(const Point point) {
  minX = std::min(minX, point.x());
  minY = std::min(minY, point.y());
  maxX = std::max(maxX, point.x());
  maxY = std::max(maxY, point.y());
}

void BoundingBox::extend(const BoundingBox& other) {
  minX = std::min(minX, other.minX);
  minY = std::min(minY, other.minY);
  maxX = std::max(maxX, other.maxX);
  maxY = std::max(maxY, other.maxY);
}

BoundingBox boundingBox(const std::span<const Point> points) {
  if (points.empty()) {
    throw std::invalid_argument("a bounding box needs at least one point");
  }
  BoundingBox box{.minX = points.front().x(),
                  .minY = points.front().y(),
                  .maxX = points.front().x(),
                  .maxY = points.front().y()};
  for (const auto& point : points.subspan(1)) {
    box.extend(point);
  }
  return box;
}

BoundingBox boundingBox(const flatbuffers::geometry::PolygonT& polygon) {
  return boundingBox(std::span<const Point>(polygon.vertices));
}

} // namespace mqt::scpd::geometry
