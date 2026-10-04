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
#include "mqt-scpd/routing/mqt_scpd_routing_export.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

namespace mqt::scpd::routing {

/// The geometry the coupler insertion's room rules measure, held apart from
/// the router so that it can be tested on paths drawn by hand.
///
/// The insertion routes every feedline edge against the artwork, the pads and
/// the leads, but never against the ring wires that will have to cross it or
/// run beside it. These helpers answer the questions the rules ask of a
/// routed way: how many wires its straight runs can carry across it, how
/// close two edges come to each other behind a pad, and on which side of the
/// pad each arm lies. Nothing here knows a wire, a coupler or a field; the
/// caller reads those and hands in cells.

/// Which cells of a path lie on a straight run.
///
/// This is the crossing rule's own definition (`CrossingConstraints::build`
/// calls it): a cell is straight when it steps to the next **distinct** cell
/// along its own heading. The router lists a cell again where its heading
/// changes on it, so the step tested is the one to the next cell somewhere
/// else, and a cell listed twice is no bend of its own. The last cell has no
/// step and is never straight.
[[nodiscard]] MQT_SCPD_ROUTING_EXPORT std::vector<bool>
straightCells(const Path& path);

/// How many parallel wires a run of straight cells can carry across it.
///
/// `(run - 1 - 2 * margin)` is the length left once a dead stretch of
/// `margin` cells at both ends is dropped; a run shorter than that carries
/// nothing, and otherwise one wire plus one more per `pitch`. At a margin of
/// 10 and a pitch of 20 a run of 21 carries one, 41 two, 61 three.
[[nodiscard]] MQT_SCPD_ROUTING_EXPORT uint32_t lanesOf(uint32_t run,
                                                       uint32_t pitch,
                                                       uint32_t margin);

/// What the straight runs of one way can carry across it.
struct CrossingCapacity {
  /// The sum of `lanesOf` over the runs.
  uint32_t lanes = 0;
  /// The length of every run of straight cells counted, in cells, in path
  /// order. Runs the skips cut short are counted at the length they keep.
  std::vector<uint32_t> runs;
};

/// The crossing capacity of a way, less its first `skipHead` and last
/// `skipTail` cells — the forced stubs at its two ends, where nothing may
/// cross. Straightness is judged on the whole path, so a run that the skip
/// cuts into still knows which way its cells step.
[[nodiscard]] MQT_SCPD_ROUTING_EXPORT CrossingCapacity
crossingCapacity(const Path& path, std::size_t skipHead, std::size_t skipTail,
                 uint32_t pitch, uint32_t margin);

/// The narrowest place between two arms of a channel.
struct Channel {
  /// Whether both arms had any cell to measure.
  bool measured = false;
  /// The least distance between a cell of the first arm and a cell of the
  /// second, in cells.
  double gap = 0.0;
  /// The distance between the first cell of each arm — where the two leave
  /// their skipped runs. A channel that never comes closer than this does
  /// not narrow: the arms run on or apart.
  double startGap = 0.0;
  /// The index into the first path and into the second where it occurs.
  std::size_t at = 0;
  std::size_t bt = 0;
  /// How many cells past its skip each arm's cell lies: `at` counted
  /// backwards from the end of the first path, `bt` forwards from the start
  /// of the second.
  std::size_t aDepth = 0;
  std::size_t bDepth = 0;
};

/// The channel between an edge that arrives at a pad and the edge that leaves
/// it.
///
/// The first arm is read **backwards** from the end of `a`: its last
/// `aSkipTail` cells are the forced run onto the pad and are dropped, and the
/// `aTake` cells before them are the arm. The second arm is read forwards
/// from the start of `b`, its first `bSkipHead` cells dropped and the `bTake`
/// after them taken. The answer is the least Euclidean distance between a
/// cell of one arm and a cell of the other.
[[nodiscard]] MQT_SCPD_ROUTING_EXPORT Channel
channelBetween(const Path& a, std::size_t aSkipTail, std::size_t aTake,
               const Path& b, std::size_t bSkipHead, std::size_t bTake);

/// Every cell the segment from one point to another passes through, both
/// ends included. Where the segment crosses a corner exactly, both cells
/// beside the corner are visited, so a wire lying on either is found.
MQT_SCPD_ROUTING_EXPORT void
supercoverLine(int64_t x0, int64_t y0, int64_t x1, int64_t y1,
               const std::function<void(int64_t x, int64_t y)>& visit);

/// Which side of a line through `centre` across `across` a point lies on:
/// 1 along `across`, -1 against it, 0 on the line.
[[nodiscard]] MQT_SCPD_ROUTING_EXPORT int
sideOf(const PathPoint& centre, HeadingVector across, const PathPoint& point);

/// Whether two points lie on the same side of that line. A point on the line
/// is on neither side and the answer is false.
[[nodiscard]] MQT_SCPD_ROUTING_EXPORT bool sameSide(const PathPoint& centre,
                                                    HeadingVector across,
                                                    const PathPoint& a,
                                                    const PathPoint& b);

} // namespace mqt::scpd::routing
