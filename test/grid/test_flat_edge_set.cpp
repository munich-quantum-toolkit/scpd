/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// The flat set of constraint edges that `cmake/PatchCdt.cmake` gives CDT
// (`CDT::EdgeUSet`): it must behave as the hash set it replaces.

#include <CDT.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <random>
#include <set>
#include <utility>
#include <vector>

namespace mqt::scpd::grid {

namespace {
std::pair<std::uint32_t, std::uint32_t> keyOf(const CDT::Edge& e) {
  return {e.v1(), e.v2()};
}
} // namespace

TEST(FlatEdgeSet, BehavesLikeASet) {
  CDT::EdgeUSet set;
  std::set<std::pair<std::uint32_t, std::uint32_t>> reference;
  std::mt19937 rng(7);
  std::uniform_int_distribution<std::uint32_t> vertex(0, 400);
  EXPECT_TRUE(set.empty());
  EXPECT_EQ(set.size(), 0U);
  std::vector<CDT::Edge> edges;
  for (int k = 0; k < 20000; ++k) {
    const auto a = vertex(rng);
    auto b = vertex(rng);
    if (a == b) {
      b = (b + 1) % 401;
    }
    const CDT::Edge edge(a, b);
    edges.push_back(edge);
    const bool fresh = reference.insert(keyOf(edge)).second;
    EXPECT_EQ(set.insert(edge).second, fresh);
    EXPECT_EQ(set.count(edge), 1U);
    EXPECT_NE(set.find(edge), set.end());
  }
  EXPECT_EQ(set.size(), reference.size());
  // Erase every other edge inserted (some twice: the second erase finds nothing).
  for (std::size_t k = 0; k < edges.size(); k += 2) {
    const auto erased = reference.erase(keyOf(edges[k]));
    EXPECT_EQ(set.erase(edges[k]), erased);
    EXPECT_EQ(set.count(edges[k]), 0U);
  }
  EXPECT_EQ(set.size(), reference.size());
  for (const auto& edge : edges) {
    EXPECT_EQ(set.count(edge), reference.count(keyOf(edge)));
  }
  // Iteration visits every edge once.
  std::set<std::pair<std::uint32_t, std::uint32_t>> seen;
  for (const auto& edge : set) {
    EXPECT_TRUE(seen.insert(keyOf(edge)).second);
  }
  EXPECT_EQ(seen, reference);
  // A copy is independent.
  CDT::EdgeUSet copy = set;
  copy.erase(edges[1]);
  EXPECT_EQ(set.count(edges[1]), reference.count(keyOf(edges[1])));
  EXPECT_EQ(copy.size() + 1, set.size());
  set.clear();
  EXPECT_TRUE(set.empty());
  EXPECT_EQ(set.count(edges[1]), 0U);
}

} // namespace mqt::scpd::grid
