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

using flatbuffers::geometry::Point;

/// A path cut into its primitive-tagged runs.
struct SegmentedPath {
  std::vector<PathSegment> segments;
  /// The sum of the primitive costs, in cells. Arcs count with their rounded
  /// cost, so this is the length the search paid for, not the exact one.
  double nominalLength = 0.0;
};

/// Cut a path into segments wherever its heading or primitive changes. A
/// straight run gathers one cell per step; a turn primitive is one step
/// whose recorded cell is the last cell the turn emitted. The lengths of the
/// segments are nominal until the sampler fills in the rendered ones.
[[nodiscard]] MQT_SCPD_ROUTING_EXPORT SegmentedPath
reconstructSegments(const MovePrimitives& primitives, const Path& path);

/// Render a path as the dense polyline of its exact curves.
///
/// Consecutive repeats of a cell are dropped from the path in place. Each
/// step renders the samples of its primitive from the current position; the
/// rendered end is then pulled onto the step's grid cell, with the residual
/// spread linearly over the new points, so that the rendering never drifts
/// from the rasterized path. A step whose declared cell has not moved renders
/// nothing. The segments receive the rendered length up to every step.
///
/// @returns The polyline, starting at the start point.
[[nodiscard]] MQT_SCPD_ROUTING_EXPORT std::vector<Point>
samplePath(const MovePrimitives& primitives, Path& path, PathPoint start,
           std::vector<PathSegment>& segments);

/// Render a path as samplePath does, but on a copy whose first point is
/// dropped because it is the start, and report where each segment begins
/// in the polyline and whether it is straight. The form the coupler insertion
/// is calibrated on.
[[nodiscard]] MQT_SCPD_ROUTING_EXPORT std::vector<Point> samplePathFromSecond(
    const MovePrimitives& primitives, Path path, PathPoint start,
    std::vector<PathSegment>& segments,
    std::vector<std::pair<std::size_t, bool>>* segmentBounds = nullptr);

/// The length of a polyline.
[[nodiscard]] MQT_SCPD_ROUTING_EXPORT double
polylineLength(std::span<const Point> points);

/// The number of direction changes between consecutive steps of a path.
[[nodiscard]] MQT_SCPD_ROUTING_EXPORT uint32_t countBends(const Path& path);

} // namespace mqt::scpd::routing
