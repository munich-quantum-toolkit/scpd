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

/// The record of one search state: a cell and a heading.
///
/// Eight bytes, so that a record never straddles two cache lines. The
/// predecessor is not stored: the state was reached through the primitive
/// (parentHeading, primitive), whose end offset the primitive table holds, so
/// the parent is the cell minus that offset at parentHeading.
struct SearchNode {
  static constexpr uint32_t UNSEEN = 0xFFFFFFFFU;
  uint32_t g = UNSEEN;
  /// The search that last wrote the record; a record of another search reads
  /// as unseen without the array being cleared between searches.
  uint16_t iteration = 0;
  uint16_t primitive : 10;
  uint16_t parentHeading : 3;
  uint16_t closed : 1;
  uint16_t unused : 2;
};
static_assert(sizeof(SearchNode) == 8, "a search node is eight bytes");

/// The per-thread scratch of the search: one record per state of a grid.
///
/// This is the one large allocation of a router context, eight bytes times
/// eight headings per cell, 582 MB on the router grid of the largest
/// benchmark. It belongs to the thread that searches with it and nothing else
/// is copied per thread; the grids a search reads are shared.
class MQT_SCPD_ROUTING_EXPORT SearchScratch {
public:
  SearchScratch(uint32_t width, uint32_t height);

  [[nodiscard]] uint32_t width() const { return width_; }
  [[nodiscard]] uint32_t height() const { return height_; }

  /// The index of a state.
  [[nodiscard]] uint32_t index(const uint32_t x, const uint32_t y,
                               const Heading heading) const {
    return ((y * width_) + x) * NUM_HEADINGS + (heading & 7U);
  }

  [[nodiscard]] SearchNode& at(const uint32_t index) { return nodes_[index]; }
  [[nodiscard]] const SearchNode& at(const uint32_t index) const {
    return nodes_[index];
  }
  [[nodiscard]] std::size_t size() const { return nodes_.size(); }
  [[nodiscard]] const SearchNode* data() const { return nodes_.data(); }

  /// The iteration of the search in progress.
  [[nodiscard]] uint16_t iteration() const { return iteration_; }

  /// Start a new search: every record of the previous searches reads as
  /// unseen. When the iteration counter wraps, the array is cleared.
  void beginSearch();

  /// Set the start state of a search.
  void setStart(uint32_t index);

private:
  uint32_t width_;
  uint32_t height_;
  std::vector<SearchNode> nodes_;
  uint16_t iteration_ = 0;
};

} // namespace mqt::scpd::routing
