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
#include <functional>
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

  /// A cell the ways may not sweep: `true` for one that is closed.
  using Blocked = std::function<bool(std::int64_t x, std::int64_t y)>;

  /// The box every way has to stay in, inclusive on all four sides: what the
  /// edge corridor closes everything outside of. It is also what bounds the
  /// straight runs the enumeration below has to try.
  struct Box {
    std::int64_t minX = 0;
    std::int64_t minY = 0;
    std::int64_t maxX = 0;
    std::int64_t maxY = 0;
  };

  /// What `minTurnsAround` did, summed by the caller.
  struct AroundStats {
    /// Concrete ways tried against the obstacles.
    std::uint64_t paths = 0;
    /// Eighth turns the answer rose above `minTurns` by.
    std::uint64_t raised = 0;
    /// Pairs on which the path budget ran out, or a family had too many
    /// runs to enumerate, before it was settled.
    std::uint64_t budgetOut = 0;
  };

  /// The fewest eighth turns from one pose to the other **with the obstacles
  /// in the way**, as far as the families of ways can be exhausted. A way of
  /// `k` eighth turns is a run of arcs turning `k` in all with a straight run
  /// before, between and after them, and on the grid every such run is a
  /// whole number of cells: the family of a given arc sequence is the set of
  /// non-negative integer run lengths that add up to the displacement, which
  /// is finite once the box bounds every run. When every way of every
  /// sequence of `k` turns sweeps a blocked cell or leaves the box, no way
  /// of `k` turns exists and the answer is at least `k + 1`. The straight
  /// run out of the source is at least `minFirstRun` cells and the run into
  /// the target at least `minLastRun`, as the router forces them.
  ///
  /// A lower bound on what the grid search returns, exactly as `minTurns`
  /// is, as long as `blocked` and `box` close nothing the search leaves
  /// open: the search's moves are the same arcs and straight cells, so a way
  /// it finds is one of the ways tried here. Where the enumeration would
  /// exceed `pathBudget` concrete ways, the family is left unsettled and the
  /// answer stays at the turns proved so far.
  /// `maxRaise` bounds how far above `minTurns` the answer is looked for:
  /// each family is dearer to exhaust than the one below it, and a family
  /// with more than two runs to enumerate (three arcs or more between the
  /// ends) is left unsettled rather than walked, which ends the raising
  /// there. Default 2: the gaps the audit found.
  [[nodiscard]] std::uint32_t
  minTurnsAround(const PathPoint& from, const PathPoint& to,
                 const Blocked& blocked, const Box& box,
                 std::uint32_t minFirstRun, std::uint32_t minLastRun,
                 std::uint64_t pathBudget, AroundStats* stats = nullptr,
                 std::uint32_t maxRaise = 2) const;

private:
  /// One arc: where it comes out, how far it moves and what it turns, and
  /// the cells it sweeps from its start.
  struct Arc {
    Heading exit = 0;
    std::int32_t dx = 0;
    std::int32_t dy = 0;
    std::uint32_t turn = 0;
    std::vector<CellOffset> swept;
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
