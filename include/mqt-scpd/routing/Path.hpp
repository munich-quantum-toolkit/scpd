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
#include "mqt-scpd/routing/Heading.hpp"

#include <cstdint>
#include <vector>

namespace mqt::scpd::routing {

/// A point of a routed path on the router grid: the state of the search that
/// produced it, plus the primitive that led to the next point. The primitive
/// is what lets the path's bend structure be rebuilt exactly.
struct PathPoint {
  uint32_t x = 0;
  uint32_t y = 0;
  Heading heading = 0;
  uint16_t primitive = 0;

  [[nodiscard]] flatbuffers::geometry::RCoord state() const {
    return {x, y, heading};
  }
  [[nodiscard]] bool samePlace(const PathPoint& other) const {
    return x == other.x && y == other.y;
  }
  [[nodiscard]] bool operator==(const PathPoint&) const = default;
};

/// A routed path, from its source to its target.
using Path = std::vector<PathPoint>;

/// The two ends of one routing request. The heading of the source is the
/// heading the wire leaves with; the heading of the target is the heading it
/// arrives with. The primitive fields are ignored.
struct RoutingObjective {
  PathPoint source;
  PathPoint target;
};

/// One primitive-tagged run of a path: a straight run of several steps, or a
/// single turn primitive. Rebuilt from the path by the segment reconstruction
/// of the path geometry; the sampler fills in the lengths.
struct PathSegment {
  Heading heading = 0;
  uint16_t primitive = 0;
  bool straight = false;
  /// The grid cells of the run, one per step.
  std::vector<PathPoint> cells;
  /// The path length up to and including each step, in cells.
  std::vector<double> lengthAt;
  /// The number of steps.
  [[nodiscard]] uint32_t steps() const {
    return static_cast<uint32_t>(cells.size());
  }
};

/// The tuning of one search.
struct SearchParams {
  /// Cells a wire runs straight out of its source before the search starts.
  uint32_t startStraightLength = 30;
  /// Cells a wire runs straight into its target after the search ends.
  uint32_t endStraightLength = 30;
  /// The bend radius, in cells. It is the radius the primitives were built
  /// with; a router refuses a pair that disagrees.
  uint8_t minRadius = 5;
  /// The cost of one eighth turn, in units of one hundredth of a cell.
  uint16_t bendPenalty = 100;
};

} // namespace mqt::scpd::routing
