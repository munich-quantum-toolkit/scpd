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
#include "mqt-scpd/routing/Path.hpp"
#include "mqt-scpd/routing/Primitives.hpp"
#include "mqt-scpd/routing/mqt_scpd_routing_export.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <utility>
#include <vector>

namespace mqt::scpd::routing {

/**
 * @brief The point type of a rendered path.
 *
 * In a rendered path, the coordinates of a point are in router grid cells.
 */
using flatbuffers::geometry::Point;

/**
 * @brief A path cut into its primitive-tagged runs.
 */
struct SegmentedPath {
  /// The runs, in the order of the path.
  std::vector<PathSegment> segments;
  /// The sum of the primitive costs, in cells. Arcs count with their rounded
  /// cost, so this is the length the search charges for the moves, not the
  /// exact length of the curve.
  double nominalLength = 0.0;
};

/**
 * @brief Cuts a path into segments wherever its heading or its primitive
 * changes.
 *
 * A straight run gathers one cell per step. A turn primitive is one step, and
 * its recorded cell is the last cell the turn emitted, which is the end of the
 * arc. Within a run, a point on the cell of the point before it adds no step.
 * The lengths of the segments are nominal: each step adds the cost of its
 * primitive. samplePath() replaces them with the rendered lengths.
 *
 * @param primitives The primitive tables that @p path refers to.
 * @param path The path to cut.
 * @return The segments and the nominal length of the path. An empty path
 * gives no segment.
 */
[[nodiscard]] MQT_SCPD_ROUTING_EXPORT SegmentedPath
reconstructSegments(const MovePrimitives& primitives, const Path& path);

/**
 * @brief Renders a path as the dense polyline of its exact curves.
 *
 * The function first removes consecutive repeats of a cell from @p path, in
 * place. Each step then renders the samples of its primitive from the current
 * position. The rendered end is pulled onto the grid cell of the step, and the
 * residual is spread linearly over the new points, so that the rendering never
 * drifts from the rasterized path. A residual above 20 cells marks a broken
 * path, and the rendering then jumps to the cell of the step. A step whose
 * cell equals the current position renders nothing.
 *
 * @param primitives The primitive tables that @p path refers to.
 * @param path The path to render. Consecutive repeats of a cell are removed
 * from it.
 * @param start The point the polyline starts at.
 * @param segments Receives the segments of @p path, with the rendered length
 * up to every step in @c lengthAt. Its previous content is replaced.
 * @return The polyline, which starts at @p start. It is empty when @p path is
 * empty.
 */
[[nodiscard]] MQT_SCPD_ROUTING_EXPORT std::vector<Point>
samplePath(const MovePrimitives& primitives, Path& path, PathPoint start,
           std::vector<PathSegment>& segments);

/**
 * @brief Renders a path as samplePath() does, but without its first point.
 *
 * The function works on a copy of @p path and drops the first point of the
 * copy, because that point is the start. Unlike samplePath(), it keeps
 * consecutive repeats of a cell. It also reports where each segment begins in
 * the polyline and whether the segment is straight.
 *
 * @param primitives The primitive tables that @p path refers to.
 * @param path The path to render. The function takes a copy.
 * @param start The point the polyline starts at.
 * @param segments Receives the segments of the copy, with the rendered length
 * up to every step in @c lengthAt. Its previous content is replaced.
 * @param segmentBounds Receives, when it is not @c nullptr, one pair per
 * segment: the index of the polyline point where the segment begins, and the
 * @c straight flag of the segment. Its previous content is discarded.
 * @return The polyline, which starts at @p start. It is empty when the copy
 * has no point left.
 */
[[nodiscard]] MQT_SCPD_ROUTING_EXPORT std::vector<Point> samplePathFromSecond(
    const MovePrimitives& primitives, Path path, PathPoint start,
    std::vector<PathSegment>& segments,
    std::vector<std::pair<std::size_t, bool>>* segmentBounds = nullptr);

/**
 * @brief Computes the length of a polyline.
 * @param points The points of the polyline, in order.
 * @return The sum of the distances between consecutive points, zero for fewer
 * than two points.
 */
[[nodiscard]] MQT_SCPD_ROUTING_EXPORT double
polylineLength(std::span<const Point> points);

/**
 * @brief Computes the length of a path as samplePath() renders it.
 *
 * The length follows the exact curves of the bends, not the cells they sweep.
 * The polyline starts at the first point of the path. The function renders a
 * copy, so @p path stays unchanged.
 *
 * @param primitives The primitive tables that @p path refers to.
 * @param path The path to measure.
 * @return The rendered length in cells, zero for an empty path.
 */
[[nodiscard]] MQT_SCPD_ROUTING_EXPORT double
renderedLength(const MovePrimitives& primitives, const Path& path);

/**
 * @brief Counts the direction changes between consecutive steps of a path.
 *
 * A step is the move from one point of the path to the next. A direction
 * change is a pair of consecutive steps with different cell offsets.
 *
 * @param path The path to examine.
 * @return The number of direction changes, zero for fewer than three points.
 */
[[nodiscard]] MQT_SCPD_ROUTING_EXPORT uint32_t countBends(const Path& path);

} // namespace mqt::scpd::routing
