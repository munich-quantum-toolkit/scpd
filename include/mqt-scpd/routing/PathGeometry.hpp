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
  /// The sum of the costs of the moves that the points carry, in cells. A
  /// straight point counts one straight step, unless it repeats the cell and
  /// the tag of the point before it, or its cell is the start of the arc of
  /// the turn after it (see Path); that turn counts its move. A turn counts
  /// its cost once. In a routed path, every move counts once, and the last
  /// point adds the straight step that leaves the path. The cost of a turn
  /// can exceed the length of its curve (see Primitive::cost), so this is the
  /// length the search charges; renderedLength() measures the curve.
  double nominalLength = 0.0;
};

/**
 * @brief Builds a straight run in the format of Path.
 *
 * The run starts at @p start and steps along the heading of @p start, one cell
 * per step. Every point carries that heading and its straight primitive. The
 * coordinates wrap around as the unsigned fields of PathPoint do, so a run of
 * cells relative to an origin may step below zero.
 *
 * @param primitives The primitive tables.
 * @param start The first point of the run. Its primitive is not read.
 * @param steps The number of steps.
 * @return The @p steps + 1 points of the run.
 */
[[nodiscard]] MQT_SCPD_ROUTING_EXPORT Path
straightRun(const MovePrimitives& primitives, PathPoint start, uint32_t steps);

/**
 * @brief Cuts a path into segments wherever its heading or its primitive
 * changes.
 *
 * A straight run gathers one cell per step. Within it, a point on the cell of
 * the point before it adds no step. A turn is one step, and its recorded cell
 * is the end of its arc. In a routed path, the point after a turn is that end,
 * even where the first point of the turn is not the start of its arc (see
 * Path). A path can also go on with a straight step from the end of an arc,
 * as a dogleg does. So the recorded cell is the point after the turn, unless
 * that point lies one step beyond the first point of the turn plus the end
 * offset of the primitive; then, and at the end of the path, it is that sum.
 * For a primitive the tables lack, it is the point after the turn, or the
 * last point of the path. The lengths of the segments are nominal and count
 * as SegmentedPath::nominalLength describes. samplePath() replaces them with
 * the rendered lengths.
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
 * position. A turn whose arc starts one straight step ahead first renders
 * that step, because that is where the straight run before a turn ends (see
 * Path). The rendered end is pulled onto the grid cell of the
 * step, and the residual is spread linearly over the new points, so that the
 * rendering never drifts from the rasterized path. A residual above 20 cells
 * marks a broken path, and the rendering then jumps to the cell of the step.
 * A step whose cell equals the current position renders nothing.
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
 * @brief Renders a path as samplePath() does, but without a straight first
 * point.
 *
 * The function works on a copy of @p path. A straight first point adds no
 * step to the polyline from @p start, which is normally that point, so the
 * function drops it from the copy. A first point that carries a turn is the
 * start of the arc, which the rendering needs to find the end of the arc, so
 * the copy keeps it. Either way the polyline is the one samplePath() draws
 * from the same start. Unlike samplePath(), the function keeps consecutive
 * repeats of a cell. It also reports where each segment begins in the
 * polyline and whether the segment is straight.
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
 * @brief Counts the heading changes between consecutive points of a path.
 *
 * Every point carries the heading of the move that leaves it, and the cells
 * that a turn sweeps carry the heading the turn starts on. Each turn therefore
 * counts once, whether it turns by an eighth or by a quarter. A repeated cell
 * on the same heading counts nothing.
 *
 * @param path The path to examine.
 * @return The number of pairs of consecutive points with different headings,
 * zero for fewer than two points.
 */
[[nodiscard]] MQT_SCPD_ROUTING_EXPORT uint32_t countBends(const Path& path);

} // namespace mqt::scpd::routing
