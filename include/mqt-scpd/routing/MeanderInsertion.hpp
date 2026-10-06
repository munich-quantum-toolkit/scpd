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

#include "mqt-scpd/routing/Path.hpp"
#include "mqt-scpd/routing/Primitives.hpp"
#include "mqt-scpd/routing/mqt_scpd_routing_export.hpp"

#include <cstdint>
#include <functional>
#include <utility>
#include <vector>

namespace mqt::scpd::routing {

/// Whether a meander may run over a cell.
using CellPredicate = std::function<bool(uint32_t x, uint32_t y)>;

/// What running over a cell costs, for the priced insertion.
using CellPrice = std::function<uint32_t(uint32_t x, uint32_t y)>;

/// Where a meander may go and what it needs.
///
/// Every figure is in cells of the grid the path runs on, and none of them is
/// a design rule: they shape the meander, they do not judge it.
struct MeanderOptions {
  /// The grid, so that a spliced path can be tested for meeting itself.
  uint32_t width = 0;
  uint32_t height = 0;
  /// The box a meander stays in, both bounds included.
  CellBox box;
  /// How far short of the box a meander stays on the axis it reaches out
  /// along.
  uint32_t safety = 2;
  /// The shortest run between the two legs of a meander, which is what keeps
  /// the legs apart. The prototype calls it `min_straight_length` as well; it
  /// is not the design rule of that name, and decision 0019 keeps the two
  /// apart.
  uint32_t minStraightLength = 25;
  /// Straight cells at the start of the path a meander leaves alone.
  uint32_t startMargin = 4;
  /// A deficit under this is lengthened by this much more, so that the
  /// meander has a shape at all. The prototype's own margin.
  double smallDeficitMargin = 50.0;
  /// What one direction change costs against the cell price, for the priced
  /// insertion.
  double bendPrice = 10.0;
  /// Aim for the required length exactly rather than at least: the loop is
  /// deepened or flattened until the rendered length lies within
  /// `tolerance` of it, a path already within the tolerance is left alone,
  /// and a path longer than the required length and the tolerance cannot
  /// be helped, which the result says. The prototype's strict insertion,
  /// which the feedline phase uses to make a resonator its target length.
  bool exact = false;
  /// How far the rendered length may lie from the required one, in cells,
  /// in the exact mode.
  double tolerance = 0.0;
  /// The box a meander between two cells of the path may use, when given:
  /// the widest free strip along the line between the two, in the
  /// prototype's strict insertion. It is cut down to `box`.
  std::function<CellBox(const PathPoint&, const PathPoint&)> boxFor;
};

/// What an insertion came to.
struct MeanderResult {
  /// Whether the path is at least the required length now, with or without
  /// a meander.
  bool reached = false;
  /// Whether a meander was spliced in.
  bool inserted = false;
  /// Whether the path was already longer than the required length allows,
  /// in the exact mode, so that no meander could make it right.
  bool tooLong = false;
  /// The length the path had to reach, in cells.
  double required = 0.0;
  /// The rendered length of the path before and after, in cells.
  double lengthBefore = 0.0;
  double lengthAfter = 0.0;
  /// How many placements were tried, and how many of them fitted.
  uint32_t candidates = 0;
  uint32_t fits = 0;
  /// The budget a pair is drawn from: how many cells of the path lie on a
  /// straight run, how many of them the start margin puts out of reach, and
  /// the widest span in cells any pair of the rest spans. A loop needs
  /// `minStraightLength + 2 · minRadius` of span, so a widest span below
  /// that figure is an insertion that could not have placed anything.
  uint32_t straightCells = 0;
  uint32_t reserved = 0;
  uint32_t widestSpan = 0;

  /// Why the placements that were tried did not fit. A resonator that stays
  /// short says what stood in its way rather than only how many placements
  /// were counted, which is the one thing a count of 21420 does not tell:
  /// a loop refused for want of room is a different problem from one refused
  /// because no pair of the way lies far enough apart to carry it.
  struct Refusals {
    /// The two turned ends do not face each other.
    uint32_t headingsApart = 0;
    /// The pair lies closer than `minStraightLength` plus the two radii, so
    /// the loop's two legs would be nearer each other than the shape allows.
    uint32_t tooClose = 0;
    /// The loop would have to be shallower than nothing: the stretch it
    /// replaces is already longer than the length wanted.
    uint32_t noDepth = 0;
    /// The loop's far edge lies outside the box it may use — the widest free
    /// strip along the pair, cut down to the grid.
    uint32_t outsideBox = 0;
    /// The loop does not come back to the far end of the pair.
    uint32_t noJoin = 0;
    /// A cell of the head, the loop or the tail may not be entered: the
    /// corridor of the search, which is the band less the fence.
    uint32_t blocked = 0;
    /// The spliced path meets itself.
    uint32_t selfCrossing = 0;
    /// The rendering never landed within the tolerance of the required
    /// length, however the loop was deepened or flattened.
    uint32_t offLength = 0;

    /// Every refusal that happened, by name and count, the commonest first.
    [[nodiscard]] MQT_SCPD_ROUTING_EXPORT
        std::vector<std::pair<const char*, uint32_t>>
        said() const;
  };
  Refusals refusals;
};

/// Lengthen a path to a required length by splicing one meander into it.
///
/// This is the prototype's `meander_insertion`. A path that is long enough
/// is left as it is. Otherwise the cells of its straight runs are walked in
/// steps, from the end of the path, and for every pair of them a meander is
/// tried between the two: each end is turned onto the axis across the pair
/// by one or two moves of the primitives, and one rectangular loop is built
/// between the turned ends, as deep as the missing length makes it and as
/// wide as the pair is apart. The loop and its two turns replace the cells
/// between the pair. A placement fits when every cell of it may be entered
/// and the spliced path does not meet itself. The pairs nearest the end of
/// the path come first, so the loop goes as near the target as it fits: the
/// source end of a resonator is cut back to the target length when the
/// coupler is spliced in, and the feedline runs there.
///
/// Without a price the first placement that fits is taken, as in phase 1 of
/// the prototype's final routing. With one, every placement that fits is
/// scored by the price of its cells plus `bendPrice` per direction change,
/// and the cheapest is taken, as in the prototype's relaxation.
///
/// The length is measured as the sampler renders it, so what the result says
/// is what a reader of the path will measure.
///
/// @param required The length the path has to reach, in cells.
/// @param enterable Whether a cell may carry the meander.
/// @param price What a cell costs; empty for the first fit.
/// @returns What came of it. The path carries the meander when one was
/// inserted, and is unchanged otherwise.
[[nodiscard]] MQT_SCPD_ROUTING_EXPORT MeanderResult
insertMeander(const MovePrimitives& primitives, Path& path, double required,
              const CellPredicate& enterable, const MeanderOptions& options,
              const CellPrice& price = {});

} // namespace mqt::scpd::routing
