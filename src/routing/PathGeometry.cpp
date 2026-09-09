/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/routing/PathGeometry.hpp"

#include "mqt-scpd/flatbuffers/geometry.hpp"
#include "mqt-scpd/routing/Path.hpp"
#include "mqt-scpd/routing/Primitives.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>
#include <utility>
#include <vector>

namespace mqt::scpd::routing {

SegmentedPath reconstructSegments(const MovePrimitives& primitives,
                                  const Path& path) {
  SegmentedPath result;
  auto& segments = result.segments;
  bool started = false;
  Heading heading = 0;
  uint16_t primitive = 0;
  uint32_t x = 0;
  uint32_t y = 0;
  double length = 0.0;
  for (const PathPoint& point : path) {
    if (!started || heading != point.heading || primitive != point.primitive) {
      started = true;
      heading = point.heading;
      primitive = point.primitive;
      x = point.x;
      y = point.y;
      length += primitives.cost(point.heading, point.primitive);
      PathSegment segment;
      segment.heading = heading;
      segment.primitive = primitive;
      segment.straight = false;
      segment.cells = {point};
      segment.lengthAt = {length};
      segments.push_back(std::move(segment));
    } else if (!segments.empty() && primitives.isStraight(heading, primitive) &&
               !(x == point.x && y == point.y)) {
      length += primitives.cost(point.heading, point.primitive);
      PathSegment& segment = segments.back();
      segment.lengthAt.push_back(length);
      segment.straight = true;
      segment.cells.push_back(point);
      x = point.x;
      y = point.y;
    } else if (!segments.empty() && !primitives.isStraight(heading, primitive) &&
               !(x == point.x && y == point.y)) {
      // A turn primitive lists every cell it sweeps under one tag. The
      // segment stays one step; its recorded cell follows to the last one,
      // which is the true end of the arc.
      segments.back().cells[0] = point;
      x = point.x;
      y = point.y;
    }
  }
  result.nominalLength = length;
  return result;
}

namespace {

/// The rendering shared by both samplers.
std::vector<Point> render(const MovePrimitives& primitives, const Path& path,
                          const PathPoint start,
                          std::vector<PathSegment>& segments,
                          std::vector<std::pair<std::size_t, bool>>* bounds) {
  segments = reconstructSegments(primitives, path).segments;
  std::vector<Point> points;
  if (bounds != nullptr) {
    bounds->clear();
  }
  if (segments.empty()) {
    return points;
  }
  double xCurrent = start.x;
  double yCurrent = start.y;
  points.emplace_back(xCurrent, yCurrent);
  double accumulated = 0.0;
  // A residual larger than this is a broken path, not rounding; then the
  // rendering jumps to the true cell instead of stretching a huge curve.
  constexpr double maxTaper = 20.0;
  constexpr double zeroDisplacement = 1e-6;

  for (std::size_t j = 0; j < segments.size(); ++j) {
    PathSegment& segment = segments[j];
    if (bounds != nullptr) {
      bounds->emplace_back(points.empty() ? 0 : points.size() - 1, segment.straight);
    }
    for (uint32_t i = 0; i < segment.steps(); ++i) {
      const PathPoint& cell = segment.cells[i];
      const auto trueX = static_cast<double>(cell.x);
      const auto trueY = static_cast<double>(cell.y);
      if (std::abs(trueX - xCurrent) < zeroDisplacement &&
          std::abs(trueY - yCurrent) < zeroDisplacement) {
        segment.lengthAt[i] = accumulated;
        continue;
      }
      const Primitive* primitive = primitives.find(segment.heading, segment.primitive);
      const std::size_t unitStart = points.size();
      if (primitive != nullptr) {
        // The first sample is the origin and is skipped.
        for (std::size_t k = 1; k < primitive->samples.size(); ++k) {
          const Point& s = primitive->samples[k];
          points.emplace_back(xCurrent + s.x(), yCurrent + s.y());
        }
      }
      if (points.size() > unitStart) {
        const double errX = trueX - points.back().x();
        const double errY = trueY - points.back().y();
        if (std::sqrt((errX * errX) + (errY * errY)) > maxTaper) {
          points.resize(unitStart);
          points.emplace_back(trueX, trueY);
        } else {
          const std::size_t added = points.size() - unitStart;
          for (std::size_t k = 0; k < added; ++k) {
            const double t = static_cast<double>(k + 1) / static_cast<double>(added);
            Point& p = points[unitStart + k];
            p = Point(p.x() + (t * errX), p.y() + (t * errY));
          }
        }
      } else {
        points.emplace_back(trueX, trueY);
      }
      xCurrent = trueX;
      yCurrent = trueY;
      double increment = 0.0;
      for (std::size_t p = unitStart; p < points.size(); ++p) {
        increment += std::hypot(points[p].x() - points[p - 1].x(),
                                points[p].y() - points[p - 1].y());
      }
      accumulated += increment;
      segment.lengthAt[i] = accumulated;
    }
  }
  return points;
}

} // namespace

std::vector<Point> samplePath(const MovePrimitives& primitives, Path& path,
                              const PathPoint start,
                              std::vector<PathSegment>& segments) {
  path.erase(std::unique(path.begin(), path.end(),
                         [](const PathPoint& a, const PathPoint& b) {
                           return a.samePlace(b);
                         }),
             path.end());
  return render(primitives, path, start, segments, nullptr);
}

std::vector<Point> samplePathFromSecond(
    const MovePrimitives& primitives, Path path, const PathPoint start,
    std::vector<PathSegment>& segments,
    std::vector<std::pair<std::size_t, bool>>* segmentBounds) {
  if (!path.empty()) {
    path.erase(path.begin());
  }
  return render(primitives, path, start, segments, segmentBounds);
}

double polylineLength(const std::span<const Point> points) {
  double total = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    total += std::hypot(points[i].x() - points[i - 1].x(),
                        points[i].y() - points[i - 1].y());
  }
  return total;
}

uint32_t countBends(const Path& path) {
  if (path.size() < 3) {
    return 0;
  }
  uint32_t bends = 0;
  int64_t dxPrev = static_cast<int64_t>(path[1].x) - path[0].x;
  int64_t dyPrev = static_cast<int64_t>(path[1].y) - path[0].y;
  for (std::size_t i = 2; i < path.size(); ++i) {
    const int64_t dx = static_cast<int64_t>(path[i].x) - path[i - 1].x;
    const int64_t dy = static_cast<int64_t>(path[i].y) - path[i - 1].y;
    if (dx != dxPrev || dy != dyPrev) {
      ++bends;
    }
    dxPrev = dx;
    dyPrev = dy;
  }
  return bends;
}

} // namespace mqt::scpd::routing
