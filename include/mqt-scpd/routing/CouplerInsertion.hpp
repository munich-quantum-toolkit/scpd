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
  /// The swept cells of the turn, then one cell per straight step from the end
  /// of the turn to the tip.
  Path path;
  /// The end of the run; its heading is the exit heading.
  PathPoint tip;
  /// The length of the turn and the run, in cells.
  double cost = 0.0;
};

/**
 * @brief Builds the quarter turn of the primitives that leaves a heading in
 * one turn direction, followed by a straight run.
 *
 * Of several quarter turns in the same direction, the function takes the one
 * with the lowest identifier.
 *
 * @param primitives The move primitives.
 * @param entry The heading the turn starts on.
 * @param turnSign @c 1 for a clockwise turn and @c -1 for a counterclockwise
 * turn, in the sense of turned().
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
  /// across the coupler's orientation: the run the resonator couples along.
  /// The first turn starts this many cells from the anchor. Zero starts it at
  /// the anchor.
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
  /// The new first point of the path: the coupler's anchor, with the heading
  /// and primitive of the splice point.
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
 * A candidate pairs a cell of a straight run of the path with a connection of
 * at most two primitives from the dogleg onto the heading of the run. Each
 * primitive starts at the end of the one before it, and the connection ends on
 * the candidate's cell. A primitive ends at the offset Primitive::dx,
 * Primitive::dy from its start, which need not be its last swept cell. The
 * remaining path of a candidate runs from its cell
 * to the end. The mismatch of a candidate is the length of the remaining path,
 * plus the dogleg and the connection, minus the target length.
 *
 * A candidate is collision-free when every cell of its dogleg and connection
 * lies inside the grid, no such cell is a cell of the remaining path except
 * the candidate's cell where the last primitive sweeps it, and no diagonal
 * step between two consecutive such cells, or onto the candidate's cell,
 * crosses a diagonal step of the remaining path inside a 2 by 2 block. The
 * function does not test the dogleg against itself. A candidate can
 * win when it is collision-free and its anchor passes @p anchorAllowed. When
 * no candidate passes the filter, every collision-free candidate can win. An
 * undershoot is charged the size of its mismatch. An overshoot is charged its
 * mismatch plus the smallest mismatch size among the candidates that can win,
 * which favors an undershoot. Of the candidates that can win, the one with the
 * lowest charge wins. The function cuts the path before its cell and puts the
 * dogleg in front.
 *
 * @param primitives The move primitives the path was routed with.
 * @param targetLength The length the path should have after the splice, in
 * cells.
 * @param path The routed resonator. On success, the path starts at the
 * coupler's anchor.
 * @param width The number of cells of the router grid along x.
 * @param height The number of cells of the router grid along y.
 * @param orientation The heading the coupler's readout port faces.
 * @param options The shape of the dogleg.
 * @param anchorAllowed An optional filter on the anchor cell, called with its
 * x and y. An empty filter allows every cell.
 * @return The splice, or @c std::nullopt when @p path is empty or no candidate
 * is collision-free. In that case @p path is unchanged.
 * @throws std::invalid_argument If @p orientation is not a heading.
 * @throws std::logic_error If the primitives hold no quarter turn that the
 * dogleg needs.
 */
[[nodiscard]] MQT_SCPD_ROUTING_EXPORT std::optional<CouplerSplice>
spliceCouplerDogleg(
    const MovePrimitives& primitives, double targetLength, Path& path,
    uint32_t width, uint32_t height, Heading orientation,
    const CouplerDoglegOptions& options = {},
    const std::function<bool(uint32_t, uint32_t)>& anchorAllowed = {});

} // namespace mqt::scpd::routing
