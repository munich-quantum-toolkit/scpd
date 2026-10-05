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
#include <cstddef>
#include <cstdint>
#include <functional>
#include <queue>
#include <random>
#include <utility>
#include <vector>

namespace {

using namespace mqt::scpd::routing;

struct Entry {
  uint32_t f = 0;
  uint32_t payload = 0;
};

/// The next number of a fixed sequence, so that the test is the same on
/// every platform.
uint64_t splitmix(uint64_t& state) {
  uint64_t z = (state += 0x9E3779B97F4A7C15ULL);
  z = (z ^ (z >> 30U)) * 0xBF58476D1CE4E5B9ULL;
  z = (z ^ (z >> 27U)) * 0x94D049BB133111EBULL;
  return z ^ (z >> 31U);
}

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
  std::ranges::sort(pushed);
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

TEST(BucketQueue, PrioritiesPastOneTurnOfTheCoarseLevelStillPopInOrder) {
  // The coarse level holds one bucket per block, modulo its size, so one
  // turn of it covers FINE_SIZE times COARSE_SIZE priorities. A search whose
  // costs grow past that wraps the bucket index around.
  constexpr uint32_t turn = 1024U * 1024U;
  BucketQueue<Entry> queue;
  queue.push({.f = turn - 2000, .payload = 1}, turn - 2000);
  EXPECT_EQ(queue.pop().payload, 1U);
  // Both entries lie past the end of the turn, in blocks whose buckets come
  // before the bucket of the scan position.
  queue.push({.f = turn + 5000, .payload = 3}, turn + 5000);
  queue.push({.f = turn + 3000, .payload = 2}, turn + 3000);
  EXPECT_EQ(queue.pop().f, turn + 3000);
  EXPECT_EQ(queue.pop().f, turn + 5000);
  EXPECT_TRUE(queue.empty());

  // A long run of rising priorities with a small spread at any one time, as
  // a search produces them, crosses the turn three times without losing or
  // reordering an entry. The run starts from a scan position of zero.
  queue.clear();
  uint32_t next = 17;
  queue.push({.f = next, .payload = 0}, next);
  for (uint32_t i = 1; i <= 1100; ++i) {
    const uint32_t lowest = next;
    next += 2953;
    queue.push({.f = next, .payload = i}, next);
    const Entry entry = queue.pop();
    EXPECT_EQ(entry.f, lowest);
    EXPECT_EQ(entry.payload, i - 1);
  }
  EXPECT_GT(next, 3 * turn);
  EXPECT_EQ(queue.size(), 1U);
  EXPECT_EQ(queue.pop().f, next);
}

TEST(BucketQueue, APopFromAnEmptyQueueGivesADefaultEntry) {
  BucketQueue<Entry> queue;
  const Entry fresh = queue.pop();
  EXPECT_EQ(fresh.f, 0U);
  EXPECT_EQ(fresh.payload, 0U);
  EXPECT_TRUE(queue.empty());
  EXPECT_EQ(queue.size(), 0U);

  // The same holds once the queue has been used and drained.
  queue.push({.f = 4000, .payload = 7}, 4000);
  EXPECT_EQ(queue.pop().payload, 7U);
  const Entry drained = queue.pop();
  EXPECT_EQ(drained.f, 0U);
  EXPECT_EQ(drained.payload, 0U);
  EXPECT_TRUE(queue.empty());
  EXPECT_EQ(queue.size(), 0U);
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

TEST(BucketQueue, ClearingKeepsTheMemoryOfASmallSearch) {
  BucketQueue<Entry> queue;
  for (uint32_t f = 0; f < 5000; ++f) {
    queue.push({.f = f, .payload = f}, f);
  }
  const std::size_t held = queue.heldEntries();
  ASSERT_GE(held, 5000U);
  queue.clear();
  // The next search reuses what this one allocated.
  EXPECT_EQ(queue.heldEntries(), held);
}

TEST(BucketQueue, TheMemoryIsTheLargestSearchNotTheSumOfAllSearches) {
  using Queue = BucketQueue<Entry>;
  Queue queue;
  // Each search fills other buckets than the ones before it. A queue that
  // kept every bucket's memory would end up with the sum of all searches.
  constexpr uint32_t searches = 50;
  constexpr uint32_t entries = 20'000;
  std::size_t largest = 0;
  for (uint32_t search = 0; search < searches; ++search) {
    queue.clear();
    for (uint32_t i = 0; i < entries; ++i) {
      const uint32_t f = (search * 20'000U) + ((i * 7919U) % 20'000U);
      queue.push({.f = f, .payload = i}, f);
    }
    largest = std::max(largest, queue.heldEntries());
    while (!queue.empty()) {
      (void)queue.pop();
    }
  }
  EXPECT_EQ(queue.heldEntries(), largest);
  // Every non-empty bucket holds at most one partly filled chunk.
  EXPECT_LE(largest, entries + (std::size_t{2048} * Queue::CHUNK_ENTRIES));
}

TEST(BucketQueue, PrioritiesBeyondTheCoarseLevelPopInOrder) {
  // The coarse level reaches 1024 blocks of 1024 priorities past the current
  // block. An entry beyond that waits in the overflow store.
  BucketQueue<Entry> queue;
  queue.push({.f = 0, .payload = 0}, 0);
  queue.push({.f = 1'100'000, .payload = 2}, 1'100'000);
  queue.push({.f = 60'000, .payload = 1}, 60'000);
  EXPECT_EQ(queue.size(), 3U);
  EXPECT_EQ(queue.pop().payload, 0U);
  EXPECT_EQ(queue.pop().payload, 1U);
  EXPECT_EQ(queue.size(), 1U);
  // A jump of three million from the scan position at 60 000.
  queue.push({.f = 3'060'000, .payload = 4}, 3'060'000);
  queue.push({.f = 1'200'000, .payload = 3}, 1'200'000);
  EXPECT_EQ(queue.size(), 3U);
  EXPECT_EQ(queue.pop().payload, 2U);
  EXPECT_EQ(queue.pop().payload, 3U);
  EXPECT_EQ(queue.pop().payload, 4U);
  EXPECT_TRUE(queue.empty());
  EXPECT_EQ(queue.size(), 0U);
  EXPECT_EQ(queue.pop().payload, 0U);
}

TEST(BucketQueue, AWaitingEntryKeepsItsPlaceAmongEqualPriorities) {
  // Among equal priorities the pop takes the entry pushed last, whether the
  // entries waited in the overflow store or not.
  BucketQueue<Entry> queue;
  constexpr uint32_t far = 5'000'000;
  queue.push({.f = far, .payload = 1}, far);
  queue.push({.f = far, .payload = 2}, far);
  queue.push({.f = far - 2'000'000, .payload = 0}, far - 2'000'000);
  EXPECT_EQ(queue.pop().payload, 0U);
  // The scan position now reaches the far block directly.
  queue.push({.f = far, .payload = 3}, far);
  EXPECT_EQ(queue.pop().payload, 3U);
  EXPECT_EQ(queue.pop().payload, 2U);
  EXPECT_EQ(queue.pop().payload, 1U);
  EXPECT_TRUE(queue.empty());
}

TEST(BucketQueue, LargeJumpsPopAsAReferenceQueueDoes) {
  // Pushes and pops interleave. Most priorities lie close above the last
  // pop, as in a search; some jump up to five million. The pops must come
  // out in the order of a reference priority queue, and the size must agree
  // after every step.
  using Item = std::pair<uint32_t, uint32_t>;
  BucketQueue<Entry> queue;
  std::priority_queue<Item, std::vector<Item>, std::greater<>> reference;
  uint64_t state = 7;
  uint32_t floor = 0;
  uint32_t payload = 0;
  std::vector<uint32_t> poppedPayloads;
  std::vector<uint32_t> pushedPayloads;
  for (int step = 0; step < 200'000; ++step) {
    const uint64_t roll = splitmix(state) % 100;
    if (roll < 55 || reference.empty()) {
      uint32_t f = floor + static_cast<uint32_t>(splitmix(state) % 5000);
      if (roll < 3) {
        f = floor + static_cast<uint32_t>(splitmix(state) % 5'000'000);
      }
      queue.push({.f = f, .payload = payload}, f);
      reference.emplace(f, payload);
      pushedPayloads.push_back(payload);
      ++payload;
    } else {
      const Entry entry = queue.pop();
      ASSERT_EQ(entry.f, reference.top().first) << step;
      reference.pop();
      floor = entry.f;
      poppedPayloads.push_back(entry.payload);
    }
    ASSERT_EQ(queue.size(), reference.size()) << step;
  }
  while (!reference.empty()) {
    const Entry entry = queue.pop();
    ASSERT_EQ(entry.f, reference.top().first);
    reference.pop();
    poppedPayloads.push_back(entry.payload);
  }
  EXPECT_TRUE(queue.empty());
  // Every entry came out once, and the scan position went past four turns
  // of the coarse level on the way.
  std::ranges::sort(poppedPayloads);
  EXPECT_EQ(poppedPayloads, pushedPayloads);
  EXPECT_GT(floor, 4U * 1024U * 1024U);
}

} // namespace
