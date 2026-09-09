/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/routing/BucketQueue.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <random>
#include <vector>

namespace {

using namespace mqt::scpd::routing;

struct Entry {
  uint32_t f = 0;
  uint32_t payload = 0;
};

TEST(BucketQueue, PopsInAscendingOrder) {
  BucketQueue<Entry> queue;
  std::mt19937 rng(11);
  std::uniform_int_distribution<uint32_t> priorities(0, 500'000);
  std::vector<uint32_t> pushed;
  for (int i = 0; i < 20'000; ++i) {
    const uint32_t f = priorities(rng);
    queue.push({.f = f, .payload = static_cast<uint32_t>(i)}, f);
    pushed.push_back(f);
  }
  EXPECT_EQ(queue.size(), pushed.size());

  std::vector<uint32_t> popped;
  uint32_t previous = 0;
  while (!queue.empty()) {
    const Entry entry = queue.pop();
    // The pops never go backwards: this is what makes the search find the
    // cheapest path rather than a nearby one.
    EXPECT_GE(entry.f, previous);
    previous = entry.f;
    popped.push_back(entry.f);
  }
  std::sort(pushed.begin(), pushed.end());
  EXPECT_EQ(popped, pushed);
}

TEST(BucketQueue, TwoPrioritiesOneFineBlockApartNeverShareABucket) {
  // The defect a fine level indexed modulo its size has: a later, larger
  // priority lands in the bucket of an earlier, smaller one and is popped
  // first.
  BucketQueue<Entry, 1024> queue;
  queue.push({.f = 5, .payload = 1}, 5);
  queue.push({.f = 1029, .payload = 2}, 1029);
  queue.push({.f = 2053, .payload = 3}, 2053);
  EXPECT_EQ(queue.pop().payload, 1U);
  EXPECT_EQ(queue.pop().payload, 2U);
  EXPECT_EQ(queue.pop().payload, 3U);
  EXPECT_TRUE(queue.empty());
}

TEST(BucketQueue, APriorityBelowTheScanPositionIsClamped) {
  BucketQueue<Entry> queue;
  queue.push({.f = 100, .payload = 1}, 100);
  EXPECT_EQ(queue.pop().payload, 1U);
  // A slightly inconsistent estimate can produce this; the entry must not
  // be lost behind the scan position.
  queue.push({.f = 50, .payload = 2}, 50);
  EXPECT_EQ(queue.size(), 1U);
  EXPECT_EQ(queue.pop().payload, 2U);
}

TEST(BucketQueue, ClearingKeepsTheQueueUsable) {
  BucketQueue<Entry> queue;
  for (uint32_t f = 0; f < 5000; f += 7) {
    queue.push({.f = f, .payload = f}, f);
  }
  queue.clear();
  EXPECT_TRUE(queue.empty());
  EXPECT_EQ(queue.size(), 0U);
  EXPECT_EQ(queue.peek(), nullptr);

  // The scan position went back to zero with the entries.
  queue.push({.f = 0, .payload = 42}, 0);
  ASSERT_NE(queue.peek(), nullptr);
  EXPECT_EQ(queue.peek()->payload, 42U);
  EXPECT_EQ(queue.pop().payload, 42U);
  EXPECT_TRUE(queue.empty());
}

TEST(BucketQueue, PeekLooksAheadOnceTheScanPositionIsOnTheBucket) {
  BucketQueue<Entry> queue;
  queue.push({.f = 300, .payload = 3}, 300);
  queue.push({.f = 100, .payload = 1}, 100);
  queue.push({.f = 100, .payload = 9}, 100);
  // The scan position is still at zero, so there is nothing to look at yet:
  // the look-ahead is a hint for the prefetch, never the queue's answer.
  EXPECT_EQ(queue.peek(), nullptr);
  EXPECT_EQ(queue.pop().payload, 9U);
  // Now the position is on the bucket of the two entries at 100.
  ASSERT_NE(queue.peek(), nullptr);
  EXPECT_EQ(queue.peek()->payload, 1U);
  EXPECT_EQ(queue.pop().payload, 1U);
  EXPECT_EQ(queue.pop().payload, 3U);
}

} // namespace
