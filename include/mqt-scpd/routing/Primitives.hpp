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
#include "mqt-scpd/routing/mqt_scpd_routing_export.hpp"

#include <array>
#include <cstdint>
#include <span>
#include <vector>

namespace mqt::scpd::routing {

/// A cell relative to the start of a primitive.
struct CellOffset {
  int16_t dx = 0;
  int16_t dy = 0;
  [[nodiscard]] bool operator==(const CellOffset&) const = default;
};

/// One move of the search: a straight step, or a circular arc of the bend
/// radius that ends on another heading.
struct Primitive {
  /// The identifier the search stores per state; below MAX_PRIMITIVE_ID.
  uint16_t id = 0;
  /// The heading at the end of the move.
  Heading exitHeading = 0;
  /// The end of the move, relative to its start.
  int16_t dx = 0;
  int16_t dy = 0;
  /// The length of the move in cells; an arc is rounded up.
  double cost = 0.0;
  /// The cells the move sweeps, relative to its start, which the search
  /// tests for obstacles.
  std::vector<CellOffset> swept;
  /// Dense points along the exact curve of the move, relative to its start,
  /// starting at the origin: what the sampler renders.
  std::vector<flatbuffers::geometry::Point> samples;
};

/// The move primitives of the eight-way router: for every heading, the
/// straight step and the arcs of one bend radius that leave it, with their
/// swept cells and their exact curves.
///
/// Cardinal headings hold arcs of a quarter turn and an eighth turn to
/// either side; diagonal headings hold the eighth turns and, added on top,
/// the quarter turns. The tables are generated once per radius and shared,
/// read-only, by every router that uses them.
class MQT_SCPD_ROUTING_EXPORT MovePrimitives {
public:
  /// Identifiers are packed into ten bits of the search state.
  static constexpr uint32_t MAX_PRIMITIVE_ID = 1024;
  /// The spacing of the curve samples, in cells.
  static constexpr double SAMPLE_SPACING = 0.1;

  /// The primitives of one bend radius, in cells.
  ///
  /// @throws std::invalid_argument when the radius is zero.
  explicit MovePrimitives(uint32_t minRadius);

  [[nodiscard]] uint32_t minRadius() const { return minRadius_; }

  /// The primitives that leave a heading, in ascending identifier order.
  [[nodiscard]] std::span<const Primitive> of(const Heading heading) const {
    return byHeading_[heading & 7U];
  }

  /// The primitive of a heading with an identifier, or nothing.
  [[nodiscard]] const Primitive* find(Heading heading, uint32_t id) const;

  /// The identifier of the straight step of a heading.
  [[nodiscard]] uint16_t straight(const Heading heading) const {
    return straight_[heading & 7U];
  }

  /// The cost of a primitive, or zero for an identifier the heading lacks.
  [[nodiscard]] double cost(Heading heading, uint32_t id) const;

  /// Whether a primitive keeps its heading.
  [[nodiscard]] bool isStraight(Heading heading, uint32_t id) const;

private:
  uint32_t minRadius_;
  std::array<std::vector<Primitive>, NUM_HEADINGS> byHeading_;
  std::array<std::array<int16_t, MAX_PRIMITIVE_ID>, NUM_HEADINGS> indexOf_{};
  std::array<uint16_t, NUM_HEADINGS> straight_{};
};

} // namespace mqt::scpd::routing
