/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/routing/SearchScratch.hpp"

#include "mqt-scpd/routing/Heading.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <utility>

namespace mqt::scpd::routing {

SearchScratch::SearchScratch(const uint32_t width, const uint32_t height)
    : gridWidth(width), gridHeight(height) {
  // The product of two 32-bit numbers fits in 64 bits; times the headings it
  // may not, so the test divides instead.
  const uint64_t cells = static_cast<uint64_t>(width) * height;
  if (cells > 0xFFFFFFFFULL / NUM_HEADINGS) {
    throw std::invalid_argument("the grid has more states than a search index");
  }
  nodes.resize(static_cast<std::size_t>(cells) * NUM_HEADINGS);
}

SearchScratch::SearchScratch(SearchScratch&& other) noexcept
    : gridWidth(std::exchange(other.gridWidth, 0)),
      gridHeight(std::exchange(other.gridHeight, 0)),
      nodes(std::move(other.nodes)),
      currentIteration(std::exchange(other.currentIteration, 0)) {}

void SearchScratch::beginSearch() {
  ++currentIteration;
  if (currentIteration == 0) {
    std::ranges::fill(nodes, SearchNode{});
    currentIteration = 1;
  }
}

void SearchScratch::setStart(const uint32_t index) {
  SearchNode& node = nodes[index];
  node.g = 0;
  node.iteration = currentIteration;
  node.primitive = 0;
  node.parentHeading = index & 7U;
  node.closed = 0;
}

} // namespace mqt::scpd::routing
