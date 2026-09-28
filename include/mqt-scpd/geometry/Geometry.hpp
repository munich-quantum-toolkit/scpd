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

/**
 * @brief A point in layout space, in micrometers.
 *
 * The alias names the schema's point type, so that the geometry helpers and
 * the serialized model share one representation.
 */
using Point = flatbuffers::geometry::Point;

/**
 * @brief Computes the vector from one point to another.
 * @param to The head of the vector.
 * @param from The tail of the vector.
 * @return The difference @p to minus @p from, component by component.
 */
[[nodiscard]] MQT_SCPD_GEOMETRY_EXPORT Point subtract(Point to, Point from);

/**
 * @brief Adds two vectors.
 * @param a The first vector.
 * @param b The second vector.
 * @return The component-wise sum.
 */
[[nodiscard]] MQT_SCPD_GEOMETRY_EXPORT Point add(Point a, Point b);

/**
 * @brief Computes the point halfway between two points.
 * @param a The first point.
 * @param b The second point.
 * @return The point whose distance to @p a and to @p b is equal.
 */
[[nodiscard]] MQT_SCPD_GEOMETRY_EXPORT Point midpoint(Point a, Point b);

/**
 * @brief Computes the dot product of two vectors.
 * @param a The first vector.
 * @param b The second vector.
 * @return The sum of the component-wise products. It is positive when the
 * two vectors point in the same half-plane, and zero when they are
 * perpendicular.
 */
[[nodiscard]] MQT_SCPD_GEOMETRY_EXPORT double dot(Point a, Point b);

/**
 * @brief Computes the z component of the cross product of two vectors.
 * @param a The first vector.
 * @param b The second vector.
 * @return The signed area of the parallelogram the two vectors span. It is
 * positive when @p b lies to the left of @p a, negative when it lies to the
 * right, and zero when the two are parallel.
 */
[[nodiscard]] MQT_SCPD_GEOMETRY_EXPORT double cross(Point a, Point b);

/**
 * @brief Computes the length of a vector.
 * @param vector The vector to measure.
 * @return The Euclidean length, which is never negative.
 */
[[nodiscard]] MQT_SCPD_GEOMETRY_EXPORT double norm(Point vector);

/**
 * @brief Computes the distance between two points.
 * @param a The first point.
 * @param b The second point.
 * @return The Euclidean distance, which is never negative.
 */
[[nodiscard]] MQT_SCPD_GEOMETRY_EXPORT double distance(Point a, Point b);

/**
 * @brief Computes the direction a vector points in.
 * @param vector The vector to measure. The zero vector yields zero.
 * @return The angle in radians in the range (-pi, pi], measured
 * counter-clockwise from the positive x axis, as `std::atan2` defines it.
 */
[[nodiscard]] MQT_SCPD_GEOMETRY_EXPORT double angleOf(Point vector);

/**
 * @brief Folds an angle into one full turn.
 * @param radians The angle to fold. Any finite value is accepted.
 * @return The equivalent angle in the range [0, 2 pi).
 */
[[nodiscard]] MQT_SCPD_GEOMETRY_EXPORT double normalizeAngle(double radians);

/**
 * @brief An axis-aligned box in layout space.
 *
 * The box is valid when @c minX is at most @c maxX and @c minY at most
 * @c maxY. The default box is the degenerate box at the origin.
 */
struct MQT_SCPD_GEOMETRY_EXPORT BoundingBox {
  /// The smallest x coordinate the box contains.
  double minX = 0.0;
  /// The smallest y coordinate the box contains.
  double minY = 0.0;
  /// The largest x coordinate the box contains.
  double maxX = 0.0;
  /// The largest y coordinate the box contains.
  double maxY = 0.0;

  /**
   * @brief Computes the extent of the box along the x axis.
   * @return The width, which is never negative for a valid box.
   */
  [[nodiscard]] double width() const { return maxX - minX; }

  /**
   * @brief Computes the extent of the box along the y axis.
   * @return The height, which is never negative for a valid box.
   */
  [[nodiscard]] double height() const { return maxY - minY; }

  /**
   * @brief Computes the center of the box.
   * @return The point halfway between the two corners.
   */
  [[nodiscard]] Point center() const;

  /**
   * @brief Grows the box until it contains a point.
   * @param point The point the box must contain.
   * @post The box contains @p point, and everything it contained before.
   */
  void extend(Point point);

  /**
   * @brief Grows the box until it contains another box.
   * @param other The box to absorb.
   * @post The box contains @p other, and everything it contained before.
   */
  void extend(const BoundingBox& other);

  /**
   * @brief Compares two boxes corner by corner.
   * @return @c true when all four coordinates are equal.
   */
  [[nodiscard]] bool operator==(const BoundingBox&) const = default;
};

/**
 * @brief Computes the smallest box that contains every given point.
 * @param points The points to cover.
 * @pre @p points is not empty.
 * @return The box, which contains every point of @p points.
 * @throws std::invalid_argument If @p points is empty, because an empty box
 * has no meaningful bounds.
 */
[[nodiscard]] MQT_SCPD_GEOMETRY_EXPORT BoundingBox
boundingBox(std::span<const Point> points);

/**
 * @brief Computes the smallest box that contains a polygon.
 * @param polygon The polygon to cover.
 * @pre @p polygon has at least one vertex.
 * @return The box, which contains every vertex of @p polygon.
 * @throws std::invalid_argument If @p polygon has no vertex.
 */
[[nodiscard]] MQT_SCPD_GEOMETRY_EXPORT BoundingBox
boundingBox(const flatbuffers::geometry::PolygonT& polygon);

} // namespace mqt::scpd::geometry
