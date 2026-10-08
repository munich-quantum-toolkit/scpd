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

std::vector<PathRun> pathRuns(const Path& path) {
  std::vector<PathRun> runs;
  for (std::size_t begin = 0; begin < path.size();) {
    std::size_t end = begin + 1;
    while (end < path.size() && path[end].heading == path[begin].heading &&
           path[end].primitive == path[begin].primitive) {
      ++end;
    }
    runs.push_back({begin, end});
    begin = end;
  }
  return runs;
}

std::vector<PathMove> decodePath(const MovePrimitives& primitives,
                                 const Path& path) {
  std::vector<PathMove> moves;
  for (const PathRun& run : pathRuns(path)) {
    const PathPoint origin = path[run.begin];
    PathPoint end = path[run.end - 1];
    const Primitive* primitive =
        primitives.find(origin.heading, origin.primitive);
    if (primitive != nullptr &&
        !primitives.isStraight(origin.heading, origin.primitive)) {
      end = {.x = static_cast<uint32_t>(static_cast<int64_t>(origin.x) +
                                        primitive->dx),
             .y = static_cast<uint32_t>(static_cast<int64_t>(origin.y) +
                                        primitive->dy),
             .heading = primitive->exitHeading,
             .primitive = primitives.straight(primitive->exitHeading)};
    } else if (primitive == nullptr && run.end < path.size()) {
      end = path[run.end];
    }
    moves.push_back({origin, end, run.begin, run.end});
  }
  return moves;
}

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
  double length = 0.0;
  for (const PathMove& move : decodePath(primitives, path)) {
    PathSegment segment;
    segment.heading = move.origin.heading;
    segment.primitive = move.origin.primitive;
    const bool straight =
        primitives.isStraight(segment.heading, segment.primitive);
    if (straight) {
      for (std::size_t i = move.begin; i < move.endIndex; ++i) {
        if (!segment.cells.empty() && segment.cells.back().samePlace(path[i])) {
          continue;
        }
        length += primitives.cost(segment.heading, segment.primitive);
        segment.cells.push_back(path[i]);
        segment.lengthAt.push_back(length);
      }
    } else {
      length += primitives.cost(segment.heading, segment.primitive);
      PathPoint endpoint = move.end;
      endpoint.heading = segment.heading;
      endpoint.primitive = segment.primitive;
      segment.cells.push_back(endpoint);
      segment.lengthAt.push_back(length);
    }
    result.segments.push_back(std::move(segment));
  }
  result.nominalLength = length;
  return result;
}

namespace {

/// The rendering shared by both samplers, using the same decoded move contract.
std::vector<Point> render(const MovePrimitives& primitives, const Path& path,
                          const PathPoint start,
                          std::vector<PathSegment>& segments,
                          std::vector<std::pair<std::size_t, bool>>* bounds) {
  segments = reconstructSegments(primitives, path).segments;
  if (bounds != nullptr) {
    bounds->clear();
  }
  if (segments.empty()) {
    return {};
  }
  std::vector<Point> points{
      {static_cast<double>(start.x), static_cast<double>(start.y)}};
  double accumulated = 0.0;
  const auto append = [&](const double x, const double y) {
    const Point& previous = points.back();
    const double distance = std::hypot(x - previous.x(), y - previous.y());
    if (distance > 1e-9) {
      accumulated += distance;
      points.emplace_back(x, y);
    }
  };
  const auto moves = decodePath(primitives, path);
  for (std::size_t i = 0; i < moves.size(); ++i) {
    const PathMove& move = moves[i];
    PathSegment& segment = segments[i];
    const bool straight =
        primitives.isStraight(segment.heading, segment.primitive);
    if (bounds != nullptr) {
      bounds->emplace_back(points.size() - 1, straight);
    }
    if (straight) {
      for (std::size_t k = 0; k < segment.cells.size(); ++k) {
        const PathPoint& cell = segment.cells[k];
        append(cell.x, cell.y);
        segment.lengthAt[k] = accumulated;
      }
      continue;
    }
    const Primitive* primitive =
        primitives.find(segment.heading, segment.primitive);
    if (primitive != nullptr) {
      // The previous straight run ends one step before a turn's tagged origin.
      append(move.origin.x, move.origin.y);
      for (std::size_t k = 1; k < primitive->samples.size(); ++k) {
        const Point& sample = primitive->samples[k];
        append(static_cast<double>(move.origin.x) + sample.x(),
               static_cast<double>(move.origin.y) + sample.y());
      }
    } else {
      append(move.end.x, move.end.y);
    }
    segment.lengthAt[0] = accumulated;
  }
  return points;
}

} // namespace

std::vector<Point> samplePath(const MovePrimitives& primitives, Path& path,
                              const PathPoint start,
                              std::vector<PathSegment>& segments) {
  path.erase(std::ranges::unique(path).begin(), path.end());
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
