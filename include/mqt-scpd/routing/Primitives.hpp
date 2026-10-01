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
 * A move is a straight step, or a circular arc of the bend radius that ends on
 * another heading.
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
  /// The length of the move in cells. The length of an arc is rounded up.
  double cost = 0.0;
  /// The cells the move sweeps, relative to its start. The search tests these
  /// cells for obstacles.
  std::vector<CellOffset> swept;
  /// Dense points along the exact curve of the move, relative to its start.
  /// The first point is the origin. A rendered path is built from these
  /// points.
  std::vector<flatbuffers::geometry::Point> samples;
};

/**
 * @brief The move primitives of the eight-way router.
 *
 * For every heading, the table holds the straight step and the arcs of one
 * bend radius that leave the heading, each with its swept cells and its exact
 * curve. Cardinal headings hold arcs of a quarter turn and of an eighth turn
 * to either side. Diagonal headings hold the eighth turns and, in addition,
 * the quarter turns.
 *
 * The constructor builds the tables once per radius. The tables are read-only
 * after that, so any number of routers can share one instance. The generation
 * reproduces the tables of the research prototype exactly, including its
 * rounding, and isStraight() answers for an unknown identifier as the
 * prototype does.
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
   * @brief The spacing of the curve samples, in cells.
   */
  static constexpr double SAMPLE_SPACING = 0.1;

  /**
   * @brief Builds the primitives of one bend radius.
   *
   * The constructor leaves out a move whose identifier is MAX_PRIMITIVE_ID or
   * more.
   *
   * @param minRadius The bend radius, in cells.
   * @throws std::invalid_argument If @p minRadius is zero, or if @p minRadius
   * is so large that a heading has no straight step with an identifier below
   * MAX_PRIMITIVE_ID.
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
   * @return @c true when the primitive ends on @p heading. An identifier that
   * @p heading lacks reads as exit heading 0, so the result is then @c true
   * for heading 0 only.
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
