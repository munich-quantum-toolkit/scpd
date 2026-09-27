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

#include <array>
#include <cstdint>
#include <vector>

namespace mqt::scpd::routing {

/// The fewest eighth turns a way of one bend radius can make from one pose to
/// another when nothing is in its way — a lower bound on the turning of
/// whatever the grid search comes back with, answered without routing
/// anything.
///
/// A way this router builds is a sequence of move primitives, and there are
/// only five of them per heading: the straight step, the eighth turns to
/// either side and the quarter turns to either side. The straight step does
/// not turn, so in this metric it is free and may be repeated at will. What
/// is left is small enough to answer in closed form:
///
///   minimise the turning of an arc sequence that walks the headings from the
///   source's to the target's, subject to the displacement the arcs do not
///   themselves cover lying in the cone of the headings the way runs straight
///   on.
///
/// The cone condition is just "the straight runs between the arcs can make up
/// the rest", with every run at least zero cells long. Two relaxations are
/// deliberate and both only widen the feasible set, which is what keeps the
/// answer a lower bound: the run lengths are real rather than whole cells,
/// and the minimum separation the router keeps between two arcs is dropped.
///
/// **It is a bound on the turning, not on the length**, and it knows nothing
/// of obstacles, of the corridor or of the edge of the chip. Every way the
/// search can return is also a way with nothing in its path, so this can
/// never exceed what the search produces — which is the whole reason it may
/// stand in for a real search in an exact method.
class MQT_SCPD_ROUTING_EXPORT AnalyticDubins {
public:
  /// How much turning the answer will look for before it gives up. Returning
  /// the cap plus one for a pose pair nothing reaches within it stays a lower
  /// bound: nothing was feasible at the cap, so the truth is above it.
  static constexpr std::uint32_t DEFAULT_CAP = 8;

  /// Reads the arcs straight off the primitives, so the model cannot drift
  /// from the move set the router actually expands.
  explicit AnalyticDubins(const MovePrimitives& primitives,
                          std::uint32_t cap = DEFAULT_CAP);

  /// The fewest eighth turns from one pose to the other, or `cap() + 1` when
  /// nothing reaches within the cap. Depends on the two headings and the
  /// displacement alone, so it is a pure function and a natural memo key.
  [[nodiscard]] std::uint32_t minTurns(const PathPoint& from,
                                       const PathPoint& to) const;

  [[nodiscard]] std::uint32_t cap() const { return cap_; }

private:
  /// One arc: where it comes out, how far it moves and what it turns.
  struct Arc {
    Heading exit = 0;
    std::int32_t dx = 0;
    std::int32_t dy = 0;
    std::uint32_t turn = 0;
  };

  /// Whether a run of arcs can be joined by straight runs into a way that
  /// covers what is left of the displacement.
  [[nodiscard]] static bool reaches(std::int64_t dx, std::int64_t dy,
                                    std::uint32_t headings);

  /// Depth-first over the arcs, within a budget of turning.
  [[nodiscard]] bool within(Heading heading, std::int64_t dx, std::int64_t dy,
                            Heading target, std::uint32_t budget,
                            std::uint32_t headings) const;

  std::array<std::vector<Arc>, NUM_HEADINGS> arcs_;
  std::uint32_t cap_;
};

} // namespace mqt::scpd::routing
