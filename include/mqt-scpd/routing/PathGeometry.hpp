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
 * @brief A half-open span of points carrying one primitive tag.
 */
struct PathRun {
  /// The first point of the run.
  std::size_t begin = 0;
  /// The first point after the run.
  std::size_t end = 0;
};

/**
 * @brief A decoded move, with geometry separate from swept occupancy.
 *
 * A straight run groups several steps. A turn is one primitive whose origin
 * is its first tagged cell and whose endpoint follows its exact offset.
 */
struct PathMove {
  /// The move's origin, carrying its heading and primitive.
  PathPoint origin;
  /// The geometric endpoint; swept cells need not end here.
  PathPoint end;
  /// The first swept point in the input path.
  std::size_t begin = 0;
  /// The first input point after this move's occupancy span.
  std::size_t endIndex = 0;
};

/**
 * @brief Partitions a path at changes of heading or primitive.
 * @param path The input path.
 * @return Half-open runs covering every input point once.
 */
[[nodiscard]] MQT_SCPD_ROUTING_EXPORT std::vector<PathRun>
pathRuns(const Path& path);

/**
 * @brief Decodes the geometry of each tagged run.
 * @param primitives The tables referenced by the path.
 * @param path The input path in the normalized Path format.
 * @return Moves with origins, endpoints, and input occupancy spans.
 */
[[nodiscard]] MQT_SCPD_ROUTING_EXPORT std::vector<PathMove>
decodePath(const MovePrimitives& primitives, const Path& path);

/**
 * @brief A path cut into its primitive-tagged runs.
 */
struct SegmentedPath {
  /// The runs, in the order of the path.
  std::vector<PathSegment> segments;
  /// The sum of the costs of tagged moves. Each distinct straight cell
  /// counts one step, including the terminal point's outgoing step. A turn
  /// counts once. renderedLength() measures the curve between path endpoints.
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
 * Straight runs retain their distinct cells. Each turn records its decoded
 * endpoint as one step. Nominal lengths charge the primitive costs;
 * samplePath() replaces them with rendered lengths.
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
 * The function removes consecutive repeats under the same tag from @p path.
 * The shared decoder supplies each turn's origin and endpoint. Rendering
 * translates its exact samples without stretching them. A straight step
 * between tagged runs connects the previous endpoint to the next origin.
 * Unknown primitives are rendered as a line to their recorded endpoint.
 *
 * @param primitives The primitive tables that @p path refers to.
 * @param path The path to render. Consecutive identical points are removed
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
 * point that starts no turn.
 *
 * The function works on a copy of @p path. A first point that starts a turn
 * is the start of the arc, which the rendering needs to find the turn and the
 * end of its arc, so the copy keeps it. That point carries the turn. Any
 * other first point is straight and adds no step to the polyline from
 * @p start, which is normally that point, so the function drops it from the
 * copy. Either way the polyline is the one
 * samplePath() draws from the same start. Unlike samplePath(), the function
 * keeps consecutive repeats of a cell. It also reports where each segment
 * begins in the polyline and whether the segment is straight.
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
