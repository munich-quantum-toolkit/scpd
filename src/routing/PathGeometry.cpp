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

#include "mqt-scpd/routing/Heading.hpp"
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

namespace {

/**
 * @brief Moves the recorded cell of a turn segment onto the end of its arc.
 * @param primitives The primitive tables.
 * @param turn The turn segment. Its cell is the first point of the turn.
 * @param next The first point after the turn, or @c nullptr at the end of the
 * path.
 * @param last The last point of the turn.
 */
void endTurn(const MovePrimitives& primitives, PathSegment& turn,
             const PathPoint* next, const PathPoint& last) {
  PathPoint& cell = turn.cells[0];
  const Primitive* primitive = primitives.find(turn.heading, turn.primitive);
  if (primitive == nullptr) {
    const PathPoint& end = next != nullptr ? *next : last;
    cell.x = end.x;
    cell.y = end.y;
    return;
  }
  const int64_t endX = static_cast<int64_t>(cell.x) + primitive->dx;
  const int64_t endY = static_cast<int64_t>(cell.y) + primitive->dy;
  const HeadingVector exit = headingVector(primitive->exitHeading);
  const bool stepBeyond = next != nullptr &&
                          static_cast<int64_t>(next->x) == endX + exit.dx &&
                          static_cast<int64_t>(next->y) == endY + exit.dy;
  if (next != nullptr && !stepBeyond) {
    cell.x = next->x;
    cell.y = next->y;
    return;
  }
  cell.x = static_cast<uint32_t>(endX);
  cell.y = static_cast<uint32_t>(endY);
}

/**
 * @brief Reports whether the arc of a turn starts on the straight point
 * before the turn.
 *
 * A routed path has this form where its search begins with a turn (see
 * Path). The point after the turn then lies at the end offset of the
 * primitive from the point before the turn. In the regular form, it lies at
 * that offset from the first point of the turn, or one step further along the
 * exit heading. For the primitives of radii 1 to 23, the two forms meet only
 * where the two points share a cell.
 *
 * @param primitives The primitive tables.
 * @param before The point before the first point of the turn.
 * @param turn The first point of the turn.
 * @param next The first point after the turn, or @c nullptr at the end of the
 * path.
 * @return @c true when the arc starts on @p before.
 */
bool arcStartsBefore(const MovePrimitives& primitives, const PathPoint& before,
                     const PathPoint& turn, const PathPoint* next) {
  if (next == nullptr || before.heading != turn.heading ||
      !primitives.isStraight(before.heading, before.primitive)) {
    return false;
  }
  const Primitive* primitive = primitives.find(turn.heading, turn.primitive);
  return primitive != nullptr &&
         static_cast<int64_t>(next->x) ==
             static_cast<int64_t>(before.x) + primitive->dx &&
         static_cast<int64_t>(next->y) ==
             static_cast<int64_t>(before.y) + primitive->dy;
}

} // namespace

Path straightRun(const MovePrimitives& primitives, const PathPoint start,
                 const uint32_t steps) {
  const HeadingVector v = headingVector(start.heading);
  const uint16_t straight = primitives.straight(start.heading);
  Path run;
  run.reserve(static_cast<std::size_t>(steps) + 1);
  for (uint64_t i = 0; i <= steps; ++i) {
    const auto step = static_cast<int64_t>(i);
    run.push_back({.x = static_cast<uint32_t>(static_cast<int64_t>(start.x) +
                                              (v.dx * step)),
                   .y = static_cast<uint32_t>(static_cast<int64_t>(start.y) +
                                              (v.dy * step)),
                   .heading = start.heading,
                   .primitive = straight});
  }
  return run;
}

SegmentedPath reconstructSegments(const MovePrimitives& primitives,
                                  const Path& path) {
  SegmentedPath result;
  auto& segments = result.segments;
  Heading heading = 0;
  uint16_t primitive = 0;
  uint32_t x = 0;
  uint32_t y = 0;
  double length = 0.0;
  // The index of the first point of the last segment.
  std::size_t segmentStart = 0;
  const auto closeTurn = [&](const PathPoint* next, const PathPoint& last) {
    endTurn(primitives, segments.back(), next, last);
    if (segmentStart == 0 ||
        !arcStartsBefore(primitives, path[segmentStart - 1], path[segmentStart],
                         next)) {
      return;
    }
    // The point before the turn counted a straight step, but its move is the
    // turn, which counts on its own.
    const PathPoint& before = path[segmentStart - 1];
    const double step = primitives.cost(before.heading, before.primitive);
    segments[segments.size() - 2].lengthAt.back() -= step;
    segments.back().lengthAt.back() -= step;
    length -= step;
  };
  for (std::size_t i = 0; i < path.size(); ++i) {
    const PathPoint& point = path[i];
    if (segments.empty() || heading != point.heading ||
        primitive != point.primitive) {
      if (!segments.empty() && !primitives.isStraight(heading, primitive)) {
        closeTurn(&point, path[i - 1]);
      }
      heading = point.heading;
      primitive = point.primitive;
      x = point.x;
      y = point.y;
      segmentStart = i;
      length += primitives.cost(point.heading, point.primitive);
      PathSegment segment;
      segment.heading = heading;
      segment.primitive = primitive;
      segment.cells = {point};
      segment.lengthAt = {length};
      segments.push_back(std::move(segment));
    } else if (primitives.isStraight(heading, primitive) &&
               (x != point.x || y != point.y)) {
      length += primitives.cost(point.heading, point.primitive);
      PathSegment& segment = segments.back();
      segment.lengthAt.push_back(length);
      segment.cells.push_back(point);
      x = point.x;
      y = point.y;
    }
  }
  if (!segments.empty() && !primitives.isStraight(heading, primitive)) {
    closeTurn(nullptr, path.back());
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

  for (PathSegment& segment : segments) {
    if (bounds != nullptr) {
      // The polyline holds the start before the first segment.
      bounds->emplace_back(points.size() - 1, segment.straight());
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
      const Primitive* primitive =
          primitives.find(segment.heading, segment.primitive);
      const std::size_t stepStart = points.size();
      if (primitive != nullptr &&
          primitive->exitHeading != (segment.heading & 7U)) {
        // The straight run before a turn ends one step before the start of
        // the arc. That step is drawn first.
        const HeadingVector v = headingVector(segment.heading);
        const double fromX = trueX - primitive->dx;
        const double fromY = trueY - primitive->dy;
        if (std::abs(fromX - xCurrent - v.dx) < zeroDisplacement &&
            std::abs(fromY - yCurrent - v.dy) < zeroDisplacement) {
          points.emplace_back(fromX, fromY);
          xCurrent = fromX;
          yCurrent = fromY;
        }
      }
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
            const double t =
                static_cast<double>(k + 1) / static_cast<double>(added);
            Point& p = points[unitStart + k];
            p = Point(p.x() + (t * errX), p.y() + (t * errY));
          }
        }
      } else {
        points.emplace_back(trueX, trueY);
      }
      xCurrent = trueX;
      yCurrent = trueY;
      // The polyline holds the start before any step, so the step's first
      // point has a point before it.
      accumulated +=
          polylineLength(std::span<const Point>(points).subspan(stepStart - 1));
      segment.lengthAt[i] = accumulated;
    }
  }
  return points;
}

} // namespace

std::vector<Point> samplePath(const MovePrimitives& primitives, Path& path,
                              const PathPoint start,
                              std::vector<PathSegment>& segments) {
  path.erase(std::ranges::unique(path,
                                 [](const PathPoint& a, const PathPoint& b) {
                                   return a.samePlace(b);
                                 })
                 .begin(),
             path.end());
  return render(primitives, path, start, segments, nullptr);
}

std::vector<Point>
samplePathFromSecond(const MovePrimitives& primitives, Path path,
                     const PathPoint start, std::vector<PathSegment>& segments,
                     std::vector<std::pair<std::size_t, bool>>* segmentBounds) {
  if (!path.empty() &&
      primitives.isStraight(path.front().heading, path.front().primitive)) {
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

double renderedLength(const MovePrimitives& primitives, const Path& path) {
  if (path.empty()) {
    return 0.0;
  }
  Path copy = path;
  std::vector<PathSegment> segments;
  return polylineLength(samplePath(primitives, copy, copy.front(), segments));
}

uint32_t countBends(const Path& path) {
  uint32_t bends = 0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    if (path[i].heading != path[i - 1].heading) {
      ++bends;
    }
  }
  return bends;
}

} // namespace mqt::scpd::routing
