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
#include "mqt-scpd/routing/mqt_scpd_routing_export.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace mqt::scpd::routing {

/**
 * @brief The record of one search state, which is a cell and a heading.
 *
 * A record takes eight bytes, so that it never straddles two cache lines. The
 * record does not store the predecessor. The search reached the state through
 * the primitive (@c parentHeading, @c primitive), and the primitive table
 * holds the end offset of that primitive. The parent state is therefore the
 * cell minus that offset, at @c parentHeading.
 */
struct SearchNode {
  /**
   * @brief The cost of a state that the search has not reached.
   */
  static constexpr uint32_t UNSEEN = 0xFFFFFFFFU;
  /// The cost of the cheapest known path from the start state to this state,
  /// in hundredths of a cell.
  uint32_t g = UNSEEN;
  /// The search that last wrote the record. A record of another search reads
  /// as unseen, so the array needs no clearing between searches.
  uint16_t iteration = 0;
  /// The identifier of the primitive that reached the state.
  uint16_t primitive : 10 = 0;
  /// The heading the primitive that reached the state leaves from.
  uint16_t parentHeading : 3 = 0;
  /// 1 when the search has closed the state, 0 while the state is open.
  uint16_t closed : 1 = 0;
  /// Padding that fills the 16-bit word.
  uint16_t unused : 2 = 0;
};
static_assert(sizeof(SearchNode) == 8, "a search node is eight bytes");

/**
 * @brief The per-thread scratch of the search: one record per state of a grid.
 *
 * The scratch is the one large allocation of a router context. It takes eight
 * bytes for each of the eight headings of a cell, so 64 bytes per cell. On the
 * 5400 by 5400 cell router grid of the largest benchmark chip, that is about
 * 1.87 GB. A scratch belongs to the thread that searches with it. Nothing else
 * is copied per thread, because the grids a search reads are shared.
 */
class MQT_SCPD_ROUTING_EXPORT SearchScratch {
public:
  /**
   * @brief Allocates one record per state of a grid.
   * @param width The number of cells of the grid along x.
   * @param height The number of cells of the grid along y.
   * @throws std::invalid_argument If the grid has more than 2^32 - 1 states,
   * so that a 32-bit state index cannot address them all.
   * @throws std::bad_alloc If the records do not fit in memory.
   */
  SearchScratch(uint32_t width, uint32_t height);

  /**
   * @brief Returns the width of the grid.
   * @return The number of cells along x.
   */
  [[nodiscard]] uint32_t width() const { return gridWidth; }
  /**
   * @brief Returns the height of the grid.
   * @return The number of cells along y.
   */
  [[nodiscard]] uint32_t height() const { return gridHeight; }

  /**
   * @brief Computes the index of a state.
   *
   * The eight headings of a cell are adjacent, and the cells follow each
   * other row by row.
   *
   * @param x The x coordinate of the cell.
   * @param y The y coordinate of the cell.
   * @param heading The heading. Only its three low bits are read.
   * @pre @p x is below width(), and @p y is below height().
   * @return The index of the record of the state, below size().
   */
  [[nodiscard]] uint32_t index(const uint32_t x, const uint32_t y,
                               const Heading heading) const {
    return (((y * gridWidth) + x) * NUM_HEADINGS) + (heading & 7U);
  }

  /**
   * @brief Accesses the record of a state.
   *
   * Unlike `std::vector::at`, the function does not check @p index.
   *
   * @param index The index of the state, as index() computes it.
   * @pre @p index is below size().
   * @return The record, which the caller may change.
   */
  [[nodiscard]] SearchNode& at(const uint32_t index) { return nodes[index]; }
  /**
   * @brief Reads the record of a state.
   *
   * Unlike `std::vector::at`, the function does not check @p index.
   *
   * @param index The index of the state, as index() computes it.
   * @pre @p index is below size().
   * @return The record.
   */
  [[nodiscard]] const SearchNode& at(const uint32_t index) const {
    return nodes[index];
  }
  /**
   * @brief Counts the records.
   * @return The number of states of the grid, which is width() times height()
   * times NUM_HEADINGS.
   */
  [[nodiscard]] std::size_t size() const { return nodes.size(); }
  /**
   * @brief Gives read access to all records at once.
   * @return A pointer to the first of size() records, in the order of index().
   */
  [[nodiscard]] const SearchNode* data() const { return nodes.data(); }

  /**
   * @brief Returns the iteration of the search in progress.
   * @return The iteration. A record belongs to the search in progress only
   * when its @c iteration field equals this value.
   */
  [[nodiscard]] uint16_t iteration() const { return currentIteration; }

  /**
   * @brief Starts a new search.
   *
   * The function advances the iteration counter, so every record of an
   * earlier search reads as unseen. When the counter wraps around, the
   * function resets every record and restarts the counter at 1.
   *
   * @post No record carries the iteration of the new search.
   */
  void beginSearch();

  /**
   * @brief Makes a state the start state of the search in progress.
   *
   * The record gets the cost zero, the iteration of the search in progress,
   * the primitive identifier 0 and the open status. Its parent heading is its
   * own heading.
   *
   * @param index The index of the start state, as index() computes it.
   * @pre @p index is below size().
   */
  void setStart(uint32_t index);

private:
  /// The number of cells along x.
  uint32_t gridWidth;
  /// The number of cells along y.
  uint32_t gridHeight;
  /// One record per state, in the order of index().
  std::vector<SearchNode> nodes;
  /// The iteration of the search in progress.
  uint16_t currentIteration = 0;
};

} // namespace mqt::scpd::routing
