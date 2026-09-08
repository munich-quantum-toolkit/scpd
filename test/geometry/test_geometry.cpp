/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/flatbuffers/geometry.hpp"
#include "mqt-scpd/geometry/Geometry.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <span>
#include <stdexcept>
#include <vector>

namespace {

using namespace mqt::scpd::geometry;

TEST(Geometry, VectorArithmetic) {
  const Point a(3.0, 4.0);
  const Point b(1.0, -2.0);

  EXPECT_EQ(subtract(a, b), Point(2.0, 6.0));
  EXPECT_EQ(add(a, b), Point(4.0, 2.0));
  EXPECT_EQ(midpoint(a, b), Point(2.0, 1.0));
  EXPECT_DOUBLE_EQ(dot(a, b), -5.0);
  EXPECT_DOUBLE_EQ(cross(a, b), -10.0);
  EXPECT_DOUBLE_EQ(norm(a), 5.0);
  EXPECT_DOUBLE_EQ(distance(a, b), std::hypot(2.0, 6.0));
}

TEST(Geometry, CrossIsPositiveForALeftTurn) {
  // Facing along +x, +y is on the left.
  EXPECT_GT(cross(Point(1.0, 0.0), Point(0.0, 1.0)), 0.0);
  EXPECT_LT(cross(Point(1.0, 0.0), Point(0.0, -1.0)), 0.0);
  EXPECT_DOUBLE_EQ(cross(Point(1.0, 0.0), Point(2.0, 0.0)), 0.0);
}

TEST(Geometry, AnglesFoldIntoOneTurn) {
  constexpr double pi = std::numbers::pi;
  EXPECT_DOUBLE_EQ(angleOf(Point(0.0, 1.0)), pi / 2.0);
  EXPECT_DOUBLE_EQ(angleOf(Point(-1.0, 0.0)), pi);
  EXPECT_DOUBLE_EQ(normalizeAngle(-pi / 2.0), 1.5 * pi);
  EXPECT_DOUBLE_EQ(normalizeAngle(2.0 * pi), 0.0);
  EXPECT_DOUBLE_EQ(normalizeAngle(5.0 * pi), pi);
  EXPECT_DOUBLE_EQ(normalizeAngle(0.25), 0.25);
}

TEST(Geometry, BoundingBoxCoversEveryPoint) {
  const std::vector<Point> points = {Point(2.0, -1.0), Point(-3.0, 4.0),
                                     Point(0.5, 0.5)};
  const BoundingBox box = boundingBox(points);
  EXPECT_EQ(box, (BoundingBox{.minX = -3.0, .minY = -1.0, .maxX = 2.0, .maxY = 4.0}));
  EXPECT_DOUBLE_EQ(box.width(), 5.0);
  EXPECT_DOUBLE_EQ(box.height(), 5.0);
  EXPECT_EQ(box.center(), Point(-0.5, 1.5));

  BoundingBox grown = box;
  grown.extend(Point(10.0, 0.0));
  EXPECT_EQ(grown, (BoundingBox{.minX = -3.0, .minY = -1.0, .maxX = 10.0, .maxY = 4.0}));
  grown.extend(BoundingBox{.minX = -5.0, .minY = -5.0, .maxX = -4.0, .maxY = -4.0});
  EXPECT_EQ(grown, (BoundingBox{.minX = -5.0, .minY = -5.0, .maxX = 10.0, .maxY = 4.0}));
}

TEST(Geometry, BoundingBoxOfAPolygonAndOfNothing) {
  mqt::scpd::flatbuffers::geometry::PolygonT polygon;
  polygon.vertices = {Point(0.0, 0.0), Point(100.0, 0.0), Point(100.0, 50.0)};
  EXPECT_EQ(boundingBox(polygon), (BoundingBox{.minX = 0.0, .minY = 0.0, .maxX = 100.0, .maxY = 50.0}));

  const std::vector<Point> none;
  EXPECT_THROW(static_cast<void>(boundingBox(std::span<const Point>(none))),
               std::invalid_argument);
}

} // namespace
