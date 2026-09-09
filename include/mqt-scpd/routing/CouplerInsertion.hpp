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

/// A quarter turn followed by a straight run, in a frame whose origin is the
/// start of the turn.
struct DoglegGeometry {
  /// The cells of the turn and the run, from the first swept cell to the tip.
  Path path;
  /// The end of the run; its heading is the exit heading.
  PathPoint tip;
  /// The length of the turn and the run, in cells.
  double cost = 0.0;
};

/// The quarter turn of the primitives that leaves a heading in one turn
/// direction, with straightLength forced straight steps after it.
///
/// @param turnSign -1 for a turn against the clock, 1 for a turn with it.
/// @throws std::logic_error when the primitives hold no such turn.
[[nodiscard]] MQT_SCPD_ROUTING_EXPORT DoglegGeometry
buildDogleg(const MovePrimitives& primitives, Heading entry, int turnSign,
            uint32_t straightLength);

/// How the coupler's dogleg is built onto a resonator.
struct CouplerDoglegOptions {
  /// Straight cells after the first quarter turn.
  uint32_t straightLength = 14;
  /// Straight cells after a second quarter turn; zero for a single dogleg.
  uint32_t secondStraightLength = 0;
  /// Whether the second turn runs against the first, an S-jog, rather than
  /// continuing it into a hook.
  bool secondTurnReverse = false;
  /// Mirror the dogleg across the coupler's axis.
  bool mirrored = false;
};

/// Where a coupler's dogleg was spliced onto a resonator.
struct CouplerSplice {
  /// The new first point of the path: the coupler's anchor, with the
  /// heading and primitive of the splice point.
  PathPoint anchor;
  /// Whether the anchor satisfied the allowed-area predicate. When no
  /// candidate did, the best collision-free candidate is spliced anyway and
  /// this is false, so that the caller can report the miss.
  bool inAllowedArea = true;
};

/// Splice the dogleg a coupler needs onto a routed resonator, at the point
/// of the path that leaves the remaining path closest to the target length.
///
/// The candidate splice points are the cells of the straight runs of the
/// path. For each, the length of the path from there to its end plus the
/// dogleg and its one- or two-primitive connection is compared with the
/// target length. Candidates are tried in order of increasing mismatch, with
/// an undershoot preferred over an overshoot of the same size, and the first
/// whose dogleg does not collide with the remaining path wins. The path is
/// cut before that point and the dogleg put in front.
///
/// @param orientation The heading the coupler's readout port faces.
/// @param anchorAllowed An optional filter on the anchor cell.
/// @returns The splice, or nothing when no candidate is collision-free; then
/// the path is unchanged.
[[nodiscard]] MQT_SCPD_ROUTING_EXPORT std::optional<CouplerSplice>
spliceCouplerDogleg(const MovePrimitives& primitives, double targetLength,
                    Path& path, Heading orientation,
                    const CouplerDoglegOptions& options = {},
                    const std::function<bool(uint32_t, uint32_t)>& anchorAllowed = {});

} // namespace mqt::scpd::routing
