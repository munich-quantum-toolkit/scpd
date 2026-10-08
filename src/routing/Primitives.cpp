/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/routing/Primitives.hpp"

#include "mqt-scpd/routing/Heading.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <utility>

namespace mqt::scpd::routing {
namespace {
using Vector = std::array<double, 2>;

/// Coordinates are at most 128 cells. This covers roundoff in grid-end solves
/// and half-cell ties without moving points that lie away from a boundary.
constexpr double ROUNDING_TOLERANCE = 1e-10;

/// The unit vector of a heading.
Vector direction(const Heading heading) {
  const HeadingVector v = headingVector(heading);
  const double scale = isDiagonal(heading) ? std::numbers::sqrt2 / 2.0 : 1.0;
  return {v.dx * scale, v.dy * scale};
}

/// The cross product of two planar vectors.
double cross(const Vector& a, const Vector& b) {
  return a[0] * b[1] - a[1] * b[0];
}

/// Resolves half-cell ties independently of trigonometric roundoff.
int16_t roundCell(const double value) {
  const double half = std::floor(value) + 0.5;
  return static_cast<int16_t>(
      std::lround(std::abs(value - half) <= ROUNDING_TOLERANCE ? half : value));
}

/// Builds a clockwise turn with straight leads that meet a grid cell.
Primitive turn(const Heading entry, const uint32_t radius, const int eighths) {
  const Vector u = direction(entry);
  const Vector v = direction(turned(entry, eighths));
  const Vector normal{u[1], -u[0]};
  const double sweep = eighths * std::numbers::pi / 4.0;
  const double sine = eighths == 2 ? 1.0 : std::numbers::sqrt2 / 2.0;
  const double cosine = eighths == 2 ? 0.0 : std::numbers::sqrt2 / 2.0;
  const Vector arcEnd{radius * (sine * u[0] + (1.0 - cosine) * normal[0]),
                      radius * (sine * u[1] + (1.0 - cosine) * normal[1])};
  Primitive p;
  p.id = static_cast<uint16_t>(eighths == 2 ? 900 : 902);
  p.exitHeading = turned(entry, eighths);
  double leadIn = 0.0;
  double leadOut = 0.0;
  double best = std::numeric_limits<double>::infinity();
  // The nearest feasible grid end lies within two cells of the bare arc.
  // Select the shortest pair of nonnegative tangent leads, with grid order
  // breaking ties. Rotations and reflections preserve this choice.
  const auto centerX = static_cast<int>(std::lround(arcEnd[0]));
  const auto centerY = static_cast<int>(std::lround(arcEnd[1]));
  for (int x = centerX - 2; x <= centerX + 2; ++x) {
    for (int y = centerY - 2; y <= centerY + 2; ++y) {
      const Vector difference{x - arcEnd[0], y - arcEnd[1]};
      double a = cross(difference, v) / cross(u, v);
      double b = cross(u, difference) / cross(u, v);
      if (a < -ROUNDING_TOLERANCE || b < -ROUNDING_TOLERANCE) {
        continue;
      }
      a = std::max(0.0, a);
      b = std::max(0.0, b);
      if (a + b < best - ROUNDING_TOLERANCE) {
        best = a + b;
        leadIn = a;
        leadOut = b;
        p.dx = static_cast<int16_t>(x);
        p.dy = static_cast<int16_t>(y);
      }
    }
  }
  if (!std::isfinite(best)) {
    throw std::logic_error("a turn has no grid end with forward tangent leads");
  }
  p.cost = leadIn + radius * sweep + leadOut;
  p.samples.emplace_back(0.0, 0.0);
  const auto addStraight = [&p](const Vector& start, const Vector& unit,
                                const double length) {
    if (length <= ROUNDING_TOLERANCE) {
      return;
    }
    const auto count =
        static_cast<int>(std::ceil(length / MovePrimitives::SAMPLE_SPACING));
    for (int i = 1; i <= count; ++i) {
      const double distance = length * static_cast<double>(i) / count;
      p.samples.emplace_back(start[0] + distance * unit[0],
                             start[1] + distance * unit[1]);
    }
  };
  addStraight({0.0, 0.0}, u, leadIn);
  const Vector arcStart{leadIn * u[0], leadIn * u[1]};
  const auto count = static_cast<int>(
      std::ceil(radius * sweep / MovePrimitives::SAMPLE_SPACING));
  for (int i = 1; i <= count; ++i) {
    const double angle = sweep * static_cast<double>(i) / count;
    p.samples.emplace_back(
        arcStart[0] + radius * (std::sin(angle) * u[0] +
                                (1.0 - std::cos(angle)) * normal[0]),
        arcStart[1] + radius * (std::sin(angle) * u[1] +
                                (1.0 - std::cos(angle)) * normal[1]));
  }
  addStraight({arcStart[0] + arcEnd[0], arcStart[1] + arcEnd[1]}, v, leadOut);
  // The analytic endpoint is integral; remove only floating-point roundoff.
  p.samples.back() = flatbuffers::geometry::Point(p.dx, p.dy);
  for (const auto& point : p.samples) {
    const CellOffset cell{.dx = roundCell(point.x()),
                          .dy = roundCell(point.y())};
    if (p.swept.empty() || p.swept.back() != cell) {
      p.swept.push_back(cell);
    }
  }
  return p;
}

/// Reflects a clockwise turn about its entry direction.
Primitive reflected(Primitive p, const Heading entry) {
  const auto vector = headingVector(entry);
  const auto reflect = [vector](const double x, const double y) -> Vector {
    if (vector.dx == 0) {
      return {-x, y};
    }
    if (vector.dy == 0) {
      return {x, -y};
    }
    const double sign = vector.dx * vector.dy;
    return {sign * y, sign * x};
  };
  const auto end = reflect(p.dx, p.dy);
  p.dx = static_cast<int16_t>(end[0]);
  p.dy = static_cast<int16_t>(end[1]);
  p.exitHeading =
      turned(entry, -static_cast<int>(headingDistance(entry, p.exitHeading)));
  ++p.id;
  for (auto& cell : p.swept) {
    const auto xy = reflect(cell.dx, cell.dy);
    cell.dx = static_cast<int16_t>(xy[0]);
    cell.dy = static_cast<int16_t>(xy[1]);
  }
  for (auto& sample : p.samples) {
    const auto xy = reflect(sample.x(), sample.y());
    sample = flatbuffers::geometry::Point(xy[0], xy[1]);
  }
  return p;
}

/// Rotates a move clockwise by quarter turns without rounding.
Primitive rotated(Primitive p, const int quarters) {
  p.exitHeading = turned(p.exitHeading, 2 * quarters);
  for (int i = 0; i < quarters; ++i) {
    const auto x = p.dx;
    p.dx = p.dy;
    p.dy = static_cast<int16_t>(-x);
    for (auto& cell : p.swept) {
      const auto cx = cell.dx;
      cell.dx = cell.dy;
      cell.dy = static_cast<int16_t>(-cx);
    }
    for (auto& sample : p.samples) {
      sample = flatbuffers::geometry::Point(sample.y(), -sample.x());
    }
  }
  return p;
}
} // namespace

MovePrimitives::MovePrimitives(const uint32_t minRadius)
    : bendRadius(minRadius) {
  if (minRadius == 0 || minRadius > MAX_BEND_RADIUS) {
    throw std::invalid_argument(
        "the bend radius must be between one and 90 cells");
  }
  for (Heading canonical = 0; canonical < 2; ++canonical) {
    const Primitive quarter = turn(canonical, minRadius, 2);
    const Primitive eighth = turn(canonical, minRadius, 1);
    const std::array<Primitive, 4> turns{quarter, reflected(quarter, canonical),
                                         eighth, reflected(eighth, canonical)};
    for (int rotation = 0; rotation < 4; ++rotation) {
      const Heading heading = turned(canonical, 2 * rotation);
      indexOf[heading].fill(-1);
      auto& moves = byHeading[heading];
      for (const auto& move : turns) {
        moves.push_back(rotated(move, rotation));
      }
      const auto step = headingVector(heading);
      Primitive straight;
      straight.id = 904;
      straight.exitHeading = heading;
      straight.dx = step.dx;
      straight.dy = step.dy;
      straight.cost = isDiagonal(heading) ? std::numbers::sqrt2 : 1.0;
      straight.swept.push_back({.dx = straight.dx, .dy = straight.dy});
      const auto count =
          static_cast<int>(std::ceil(straight.cost / SAMPLE_SPACING));
      for (int i = 0; i <= count; ++i) {
        const double t = static_cast<double>(i) / count;
        straight.samples.emplace_back(t * step.dx, t * step.dy);
      }
      straightIds[heading] = straight.id;
      moves.push_back(std::move(straight));
      for (std::size_t i = 0; i < moves.size(); ++i) {
        indexOf[heading][moves[i].id] = static_cast<int16_t>(i);
      }
    }
  }
}

const Primitive* MovePrimitives::find(const Heading heading,
                                      const uint32_t id) const {
  if (id >= MAX_PRIMITIVE_ID) {
    return nullptr;
  }
  const int16_t index = indexOf[heading & 7U][id];
  if (index < 0) {
    return nullptr;
  }
  return &byHeading[heading & 7U][static_cast<std::size_t>(index)];
}

double MovePrimitives::cost(const Heading heading, const uint32_t id) const {
  const Primitive* primitive = find(heading, id);
  return primitive != nullptr ? primitive->cost : 0.0;
}

bool MovePrimitives::isStraight(const Heading heading,
                                const uint32_t id) const {
  const Primitive* primitive = find(heading, id);
  return primitive != nullptr && primitive->exitHeading == (heading & 7U);
}

} // namespace mqt::scpd::routing
