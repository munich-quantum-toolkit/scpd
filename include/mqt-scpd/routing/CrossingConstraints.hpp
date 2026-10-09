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
#include <vector>

namespace mqt::scpd::routing {

/// Where a route may cross the feedlines, and on which heading.
///
/// One byte per cell. A cell within `expandRadius` of a straight run of a
/// feedline remembers the run's heading, so a route may enter it at a right
/// angle to that heading only; a cell around a curve, or around the first
/// and last ten cells of a feedline, may not be entered at all. This is the
/// prototype's `build_orthogonal_path_constrains` and its
/// `can_cross_orthogonal_path_constraint`, held apart from the router so
/// that the design-rule check asks exactly the question the search asked
/// and the two cannot drift apart.
///
/// A straight run is read off the cells, not the moves: a cell that steps
/// to the next cell along its own heading is on a straight run, any other
/// cell is on a bend. The prototype read the moves, and a move that runs
/// straight before it bends counted as a bend along its whole length; the
/// artifact carries no moves, so the check could not have read them, and
/// the straight lead of such a move is a straight run in copper.
class MQT_SCPD_ROUTING_EXPORT CrossingConstraints {
public:
  /// A cell no route may enter, whatever its heading.
  static constexpr uint8_t CURVE_ZONE = 0xFF;

  CrossingConstraints() = default;

  /// Build the constraints of a grid from routed feedlines. Feedlines flagged
  /// in `skip` are left out.
  void build(uint32_t width, uint32_t height,
             const std::vector<Path>& feedlines, const std::vector<bool>& skip,
             int expandRadius = 10);

  /// Forget every constraint. `allowed` then permits everything.
  void clear();

  [[nodiscard]] bool empty() const { return masks_.empty(); }

  /// Whether a route may enter a cell under a heading.
  [[nodiscard]] bool allowed(uint32_t x, uint32_t y, Heading heading) const;

  /// Whether a cell of a way keeps the rule: on its own heading, or on
  /// `arrivedWith`, the heading of the cell before it.
  ///
  /// The search tests every cell a move enters on the heading the move
  /// starts on, and records the cell a move ends on with the heading the
  /// next move leaves on. A way that runs straight across a feedline and
  /// turns on the last cell within `expandRadius` of it is therefore one
  /// the search allows, and that cell carries the turned heading. Judged on
  /// its own heading alone it would break the rule the search kept; judged
  /// on the heading it arrived with as well, a way breaks the rule only
  /// where it runs off the right angle inside the band (user, 2026-10-09).
  [[nodiscard]] bool allowedArriving(uint32_t x, uint32_t y, Heading heading,
                                     Heading arrivedWith) const;

  /// The mask of a cell: 0 free, `CURVE_ZONE`, or the bits of the headings
  /// of the straight runs present there.
  [[nodiscard]] uint8_t maskAt(uint32_t x, uint32_t y) const;

private:
  uint32_t width_ = 0;
  uint32_t height_ = 0;
  std::vector<uint8_t> masks_;
};

} // namespace mqt::scpd::routing
