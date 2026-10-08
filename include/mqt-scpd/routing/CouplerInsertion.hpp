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

#include "mqt-scpd/routing/Heading.hpp"
#include "mqt-scpd/routing/Path.hpp"
#include "mqt-scpd/routing/Primitives.hpp"
#include "mqt-scpd/routing/mqt_scpd_routing_export.hpp"

#include <cstdint>
#include <functional>
#include <optional>

namespace mqt::scpd::routing {

/**
 * @brief A quarter turn followed by a straight run, in a frame whose origin is
 * the start of the turn.
 *
 * The coordinates are offsets from the origin. A negative offset wraps around
 * in the unsigned fields. The sum of a cell and an offset in unsigned
 * arithmetic is therefore the right cell when that cell has no negative
 * coordinate.
 */
struct DoglegGeometry {
  /// The swept cells of the turn and its end, then one cell per straight step
  /// from the end of the turn to the tip. The cells of the turn carry its tag,
  /// the cells of the run the straight step of the exit heading.
  Path path;
  /// The end of the run, on the exit heading and tagged with its straight
  /// step. Without a straight step, the tip is the end of the turn.
  PathPoint tip;
  /// The length of the dogleg as samplePath() renders it, in cells: the curve
  /// of the turn and the length of every straight step, the square root of
  /// two on a diagonal heading.
  double cost = 0.0;
};

/**
 * @brief Builds the quarter turn of the primitives that leaves a heading in
 * one turn direction, followed by a straight run.
 *
 * Every turn ends on a grid cell with the tangent of its exit heading.
 * Consecutive points of the dogleg occupy neighboring cells.
 *
 * @param primitives The move primitives.
 * @param entry The heading the turn starts on.
 * @param turnSign @c 1 for a clockwise turn and @c -1 for a counterclockwise
 * turn, with y up and in the sense of turned().
 * @param straightLength The number of straight steps after the turn.
 * @return The dogleg. Its tip lies @p straightLength steps past the end of the
 * turn, on the heading two eighth turns from @p entry in the direction of
 * @p turnSign.
 * @throws std::invalid_argument If @p turnSign is neither @c 1 nor @c -1.
 * @throws std::logic_error If the primitives hold no such quarter turn.
 */
[[nodiscard]] MQT_SCPD_ROUTING_EXPORT DoglegGeometry
buildDogleg(const MovePrimitives& primitives, Heading entry, int turnSign,
            uint32_t straightLength);

/** @brief The shape of the dogleg that a coupler adds to a resonator. */
struct CouplerDoglegOptions {
  /// Straight steps from the anchor to the first quarter turn, on the heading
  /// two eighth turns clockwise, with y up, from the coupler heading: the run
  /// the resonator couples along. The first turn starts this many cells from
  /// the anchor. Zero starts it at the anchor.
  uint32_t leadStraight = 0;
  /// Straight cells after the first quarter turn.
  uint32_t straightLength = 14;
  /// Straight cells after a second quarter turn; zero for a single dogleg.
  uint32_t secondStraightLength = 0;
  /// Whether the second turn runs against the first, an S-jog, rather than
  /// continuing it into a hook.
  bool secondTurnReverse = false;
  /// Whether the dogleg is mirrored across the coupler's axis.
  bool mirrored = false;
};

/** @brief Where a coupler's dogleg was spliced onto a resonator. */
struct CouplerSplice {
  /// The coupler's anchor: the new first point of the path, with its heading
  /// and primitive.
  PathPoint anchor;
  /// Whether the anchor satisfies the allowed-area filter. When no candidate
  /// does, the best collision-free candidate is spliced anyway and this is
  /// false, so that the miss stays visible.
  bool inAllowedArea = true;
};

/**
 * @brief Splices the dogleg a coupler needs onto a routed resonator, at the
 * point of the path that leaves the remaining path closest to the target
 * length.
 *
 * Each candidate joins the dogleg to a straight run through at most two
 * primitives. Lengths come from the rendered curves. A candidate must lie
 * inside the grid, avoid cells and diagonal steps of the remaining path,
 * and pass the self-intersection check for the new prefix.
 * Use @p candidateAllowed to check obstacles and geometric clearance on the
 * complete proposed path. This filter is a hard constraint.
 *
 * Among feasible candidates, anchors passing @p anchorAllowed are preferred.
 * If none passes that preference, the best feasible candidate still wins
 * with CouplerSplice::inAllowedArea set to false. An undershoot is charged
 * its absolute mismatch. An overshoot also pays the smallest achievable
 * mismatch. Equal charges prefer the earlier join along the path, then the
 * lexicographic primitive order. Mismatches are rounded to billionths of a
 * cell before comparison.
 *
 * The selected prefix replaces the path before its joining point. Allocation
 * failure or a throwing filter leaves the input path unchanged. Exceptions
 * from either filter propagate to the caller.
 *
 * @param primitives The move primitives the path was routed with.
 * @param targetLength The length the path should have after the splice, as
 * samplePath() renders it from the anchor, in cells. It is finite and not
 * negative.
 * @param path The routed resonator. On success, the path starts at the
 * coupler's anchor.
 * @param width The number of cells of the router grid along x.
 * @param height The number of cells of the router grid along y.
 * @param couplerHeading The heading the coupler's readout port faces.
 * @param options The shape of the dogleg.
 * @param anchorAllowed An optional filter on the anchor cell, called with its
 * x and y. An empty filter allows every cell.
 * @param candidateAllowed An optional hard filter on the complete proposed
 * path, including the remaining resonator. Use it to check obstacles and
 * clearance on the rendered geometry. It must return the same result for the
 * same candidate and must not modify the input path. Rejected candidates never
 * enter the anchor fallback. An empty filter allows every candidate.
 * @return The splice, or @c std::nullopt when @p path is empty or no candidate
 * meets the collision checks and @p candidateAllowed. In that case @p path
 * is unchanged.
 * @throws std::invalid_argument If @p couplerHeading is not a heading, or if
 * @p targetLength is negative or not finite.
 * @throws std::logic_error If the primitives hold no quarter turn that the
 * dogleg needs.
 * @throws std::bad_alloc If the memory runs out. @p path then stays as it was.
 */
[[nodiscard]] MQT_SCPD_ROUTING_EXPORT std::optional<CouplerSplice>
spliceCouplerDogleg(
    const MovePrimitives& primitives, double targetLength, Path& path,
    uint32_t width, uint32_t height, Heading couplerHeading,
    const CouplerDoglegOptions& options = {},
    const std::function<bool(uint32_t, uint32_t)>& anchorAllowed = {},
    const std::function<bool(const Path&)>& candidateAllowed = {});

} // namespace mqt::scpd::routing
