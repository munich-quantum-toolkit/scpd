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

namespace mqt::scpd::routing {

SearchScratch::SearchScratch(const uint32_t width, const uint32_t height)
    : width_(width), height_(height) {
  const std::size_t states =
      static_cast<std::size_t>(width) * height * NUM_HEADINGS;
  if (states > 0xFFFFFFFFULL) {
    throw std::invalid_argument("the grid has more states than a search index");
  }
  nodes_.resize(states);
}

void SearchScratch::beginSearch() {
  ++iteration_;
  if (iteration_ == 0) {
    std::fill(nodes_.begin(), nodes_.end(), SearchNode{});
    iteration_ = 1;
  }
}

void SearchScratch::setStart(const uint32_t index) {
  SearchNode& node = nodes_[index];
  node.g = 0;
  node.iteration = iteration_;
  node.primitive = 0;
  node.parentHeading = index & 7U;
  node.closed = 0;
}

} // namespace mqt::scpd::routing
