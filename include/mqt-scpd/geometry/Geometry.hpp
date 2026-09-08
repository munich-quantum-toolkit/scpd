/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#pragma once

#include "mqt-scpd/flatbuffers/geometry.hpp"
#include "mqt-scpd/geometry/mqt_scpd_geometry_export.hpp"

#include <span>

namespace mqt::scpd::geometry {

/// The layout-space point of the schema: micrometers, as double.
using Point = flatbuffers::geometry::Point;

/// The vector from `from` to `to`.
[[nodiscard]] MQT_SCPD_GEOMETRY_EXPORT Point subtract(Point to, Point from);

/// The sum of two vectors.
[[nodiscard]] MQT_SCPD_GEOMETRY_EXPORT Point add(Point a, Point b);

/// The point halfway between two points.
[[nodiscard]] MQT_SCPD_GEOMETRY_EXPORT Point midpoint(Point a, Point b);

/// The dot product of two vectors.
[[nodiscard]] MQT_SCPD_GEOMETRY_EXPORT double dot(Point a, Point b);

/// The z component of the cross product. Positive when `b` lies to the left
/// of `a`.
[[nodiscard]] MQT_SCPD_GEOMETRY_EXPORT double cross(Point a, Point b);

/// The length of a vector.
[[nodiscard]] MQT_SCPD_GEOMETRY_EXPORT double norm(Point vector);

/// The distance between two points.
[[nodiscard]] MQT_SCPD_GEOMETRY_EXPORT double distance(Point a, Point b);

/// The direction of a vector in radians, in (-pi, pi], as `std::atan2` gives
/// it.
[[nodiscard]] MQT_SCPD_GEOMETRY_EXPORT double angleOf(Point vector);

/// The angle folded into [0, 2 pi).
[[nodiscard]] MQT_SCPD_GEOMETRY_EXPORT double normalizeAngle(double radians);

/// An axis-aligned box in layout space.
struct MQT_SCPD_GEOMETRY_EXPORT BoundingBox {
  double minX = 0.0;
  double minY = 0.0;
  double maxX = 0.0;
  double maxY = 0.0;

  [[nodiscard]] double width() const { return maxX - minX; }
  [[nodiscard]] double height() const { return maxY - minY; }
  [[nodiscard]] Point center() const;

  /// Grow the box to contain a point.
  void extend(Point point);

  /// Grow the box to contain another box.
  void extend(const BoundingBox& other);

  [[nodiscard]] bool operator==(const BoundingBox&) const = default;
};

/// The smallest box containing every point.
///
/// @throws std::invalid_argument when there is no point, because an empty
/// box has no meaningful bounds.
[[nodiscard]] MQT_SCPD_GEOMETRY_EXPORT BoundingBox
boundingBox(std::span<const Point> points);

/// The smallest box containing every vertex of the polygon.
[[nodiscard]] MQT_SCPD_GEOMETRY_EXPORT BoundingBox
boundingBox(const flatbuffers::geometry::PolygonT& polygon);

} // namespace mqt::scpd::geometry
