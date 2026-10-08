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

/**
 * @brief A cell relative to the start of a primitive.
 */
struct CellOffset {
  /// The offset along x, in cells.
  int16_t dx = 0;
  /// The offset along y, in cells.
  int16_t dy = 0;
  /**
   * @brief Compares two offsets component by component.
   * @return @c true when both components are equal.
   */
  [[nodiscard]] bool operator==(const CellOffset&) const = default;
};

/**
 * @brief One move of the search.
 *
 * A move is a straight step, or a circular arc of the bend radius with tangent
 * straight leads that ends on another heading.
 */
struct Primitive {
  /// The identifier the search stores per state, below
  /// MovePrimitives::MAX_PRIMITIVE_ID.
  uint16_t id = 0;
  /// The heading at the end of the move.
  Heading exitHeading = 0;
  /// The x offset of the end of the move from its start, in cells.
  int16_t dx = 0;
  /// The y offset of the end of the move from its start, in cells.
  int16_t dy = 0;
  /// The length of the move, in cells, including its straight leads.
  double cost = 0.0;
  /// The cells nearest the curve samples, in travel order. Consecutive
  /// duplicates are removed. A turn sweeps its start and end; a straight
  /// step sweeps only its end. The cells form a chain of neighbors.
  std::vector<CellOffset> swept;
  /// Points along the move, from the origin to (@c dx, @c dy). Chords are
  /// at most SAMPLE_SPACING long. Turns use an exact circular arc with
  /// tangent straight leads, so the curve meets both grid headings without
  /// reducing the bend radius. Samples include the joins of each piece.
  std::vector<flatbuffers::geometry::Point> samples;
};

/**
 * @brief The move primitives of the eight-way router.
 *
 * Every heading has one straight step and one eighth and quarter turn to
 * each side. A turn joins a circular arc of the bend radius to nonnegative
 * straight leads along its entry and exit headings. The leads reach a whole
 * grid cell without changing the radius or either tangent.
 *
 * The constructor selects the shortest leads among grid ends within two
 * cells of the bare arc end. It rotates and reflects two canonical headings
 * to preserve symmetry. Identifiers 900 and 901 denote clockwise and
 * counterclockwise quarter turns; 902 and 903 denote the eighth turns;
 * 904 denotes the straight step. The tables are read-only after construction.
 */
class MQT_SCPD_ROUTING_EXPORT MovePrimitives {
public:
  /**
   * @brief The bound on primitive identifiers.
   *
   * The search state packs an identifier into ten bits, so every identifier
   * lies below this value.
   */
  static constexpr uint32_t MAX_PRIMITIVE_ID = 1024;
  /**
   * @brief The step of the curve samples, in cells.
   *
   * Samples include piece joins and are at most this distance apart.
   */
  static constexpr double SAMPLE_SPACING = 0.1;
  /**
   * @brief The largest bend radius the tables can hold, in cells.
   *
   * A DubinsRouter accepts only the radii up to DubinsRouter::MAX_BEND_RADIUS,
   * because the moves of a larger radius do not fit its search tables. The
   * functions that read the primitives without a router, such as
   * buildDogleg(), take every radius up to this bound.
   */
  static constexpr uint32_t MAX_BEND_RADIUS = 90;

  /**
   * @brief Builds the primitives of one bend radius.
   *
   * @param minRadius The bend radius, in cells.
   * @throws std::invalid_argument If @p minRadius is zero or larger than
   * MAX_BEND_RADIUS. The constructor checks this before it builds a table.
   */
  explicit MovePrimitives(uint32_t minRadius);

  /**
   * @brief Returns the bend radius the primitives were built with.
   * @return The bend radius, in cells.
   */
  [[nodiscard]] uint32_t minRadius() const { return bendRadius; }

  /**
   * @brief Lists the primitives that leave a heading.
   * @param heading The heading. Only its three low bits are read.
   * @return The primitives, in ascending identifier order. The span stays
   * valid for the life of the table.
   */
  [[nodiscard]] std::span<const Primitive> of(const Heading heading) const {
    return byHeading[heading & 7U];
  }

  /**
   * @brief Looks up a primitive by its heading and its identifier.
   * @param heading The heading the primitive leaves. Only its three low bits
   * are read.
   * @param id The identifier of the primitive.
   * @return The primitive, or @c nullptr when @p heading has no primitive with
   * the identifier @p id.
   */
  [[nodiscard]] const Primitive* find(Heading heading, uint32_t id) const;

  /**
   * @brief Returns the identifier of the straight step of a heading.
   * @param heading The heading. Only its three low bits are read.
   * @return The identifier of the straight step.
   */
  [[nodiscard]] uint16_t straight(const Heading heading) const {
    return straightIds[heading & 7U];
  }

  /**
   * @brief Returns the cost of a primitive.
   * @param heading The heading the primitive leaves.
   * @param id The identifier of the primitive.
   * @return The cost in cells, or zero when @p heading has no primitive with
   * the identifier @p id.
   */
  [[nodiscard]] double cost(Heading heading, uint32_t id) const;

  /**
   * @brief Reports whether a primitive keeps its heading.
   * @param heading The heading the primitive leaves.
   * @param id The identifier of the primitive.
   * @return @c true when the primitive exists and ends on @p heading.
   */
  [[nodiscard]] bool isStraight(Heading heading, uint32_t id) const;

private:
  /// The bend radius, in cells.
  uint32_t bendRadius;
  /// The primitives of each heading, in ascending identifier order.
  std::array<std::vector<Primitive>, NUM_HEADINGS> byHeading;
  /// The position of each identifier in @c byHeading, or -1 when the heading
  /// has no primitive with that identifier.
  std::array<std::array<int16_t, MAX_PRIMITIVE_ID>, NUM_HEADINGS> indexOf{};
  /// The identifier of the straight step of each heading.
  std::array<uint16_t, NUM_HEADINGS> straightIds{};
};

} // namespace mqt::scpd::routing
