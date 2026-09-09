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

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace mqt::scpd::routing {

/// A priority queue over integer priorities with constant-time push and
/// pop, for the open list of the search.
///
/// Two levels of buckets: the fine level holds one aligned block of
/// FINE_SIZE consecutive priorities, one bucket per priority, and the coarse
/// level holds every block above it, one bucket per block. A pop takes the
/// last entry of the lowest non-empty fine bucket; when the fine block is
/// exhausted, the next non-empty coarse block is moved into it. Every level
/// keeps a bitmask of its non-empty buckets, so the next bucket is found by
/// counting trailing zeros rather than by scanning.
///
/// Each fine bucket holds exactly one priority, so the pops come out in
/// ascending order. The span of the priorities held at one time must stay
/// below FINE_SIZE times COARSE_SIZE. The entry type carries its own priority
/// in a member named f.
template <typename Entry, uint32_t FINE_SIZE = 1024, uint32_t COARSE_SIZE = 1024>
class BucketQueue {
  static_assert((FINE_SIZE & (FINE_SIZE - 1)) == 0, "the fine size is a power of two");
  static_assert((COARSE_SIZE & (COARSE_SIZE - 1)) == 0,
                "the coarse size is a power of two");
  static_assert(FINE_SIZE >= 64 && COARSE_SIZE >= 64, "at least one mask word");

  static constexpr uint32_t FINE_MASK = FINE_SIZE - 1;
  static constexpr uint32_t COARSE_MASK = COARSE_SIZE - 1;
  static constexpr uint32_t FINE_WORDS = FINE_SIZE / 64;
  static constexpr uint32_t COARSE_WORDS = COARSE_SIZE / 64;

public:
  BucketQueue() : fine_(FINE_SIZE), coarse_(COARSE_SIZE) {}

  /// Remove every entry. The buckets keep their capacity.
  void clear() {
    clearLevel(fine_, fineMask_);
    clearLevel(coarse_, coarseMask_);
    size_ = 0;
    currentMin_ = 0;
  }

  void push(Entry entry, uint32_t priority) {
    // A slightly inconsistent heuristic can produce a priority just below
    // the current minimum. Clamp it, so that it is not lost behind the scan
    // position.
    if (priority < currentMin_) {
      priority = currentMin_;
    }
    if ((priority / FINE_SIZE) == (currentMin_ / FINE_SIZE)) {
      const uint32_t index = priority & FINE_MASK;
      fine_[index].push_back(std::move(entry));
      fineMask_[index >> 6U] |= (uint64_t{1} << (index & 63U));
    } else {
      const uint32_t index = (priority / FINE_SIZE) & COARSE_MASK;
      coarse_[index].push_back(std::move(entry));
      coarseMask_[index >> 6U] |= (uint64_t{1} << (index & 63U));
    }
    ++size_;
  }

  /// The entry with the lowest priority. The queue must not be empty.
  Entry pop() {
    while (true) {
      const uint32_t index = currentMin_ & FINE_MASK;
      if (!fine_[index].empty()) {
        Entry entry = std::move(fine_[index].back());
        fine_[index].pop_back();
        if (fine_[index].empty()) {
          fineMask_[index >> 6U] &= ~(uint64_t{1} << (index & 63U));
        }
        --size_;
        if (entry.f > currentMin_) {
          currentMin_ = entry.f;
        }
        return entry;
      }
      const uint32_t next = nextSet(fineMask_.data(), FINE_WORDS, index);
      if (next != NONE) {
        currentMin_ = (currentMin_ - index) + next;
        continue;
      }
      if (!refillFine()) {
        // Unreachable when size_ is kept exactly; never loop forever.
        if (size_ > 0) {
          --size_;
        }
        return Entry{};
      }
    }
  }

  /// The entry the next pop returns, when it lies in the fine bucket at the
  /// scan position; otherwise nothing. Lets a search prefetch its node data.
  [[nodiscard]] const Entry* peek() const {
    const auto& bucket = fine_[currentMin_ & FINE_MASK];
    return bucket.empty() ? nullptr : &bucket.back();
  }

  [[nodiscard]] bool empty() const { return size_ == 0; }
  [[nodiscard]] uint32_t size() const { return size_; }

private:
  static constexpr uint32_t NONE = std::numeric_limits<uint32_t>::max();

  /// The first set bit at or after from, or NONE.
  static uint32_t nextSet(const uint64_t* mask, const uint32_t words,
                          const uint32_t from) {
    uint32_t word = from >> 6U;
    uint64_t current = mask[word] & (~uint64_t{0} << (from & 63U));
    while (true) {
      if (current != 0) {
        return (word << 6U) | static_cast<uint32_t>(std::countr_zero(current));
      }
      if (++word >= words) {
        return NONE;
      }
      current = mask[word];
    }
  }

  template <typename Mask>
  static void clearLevel(std::vector<std::vector<Entry>>& level, Mask& mask) {
    for (std::size_t w = 0; w < mask.size(); ++w) {
      uint64_t bits = mask[w];
      while (bits != 0) {
        level[(w << 6U) | static_cast<uint32_t>(std::countr_zero(bits))].clear();
        bits &= bits - 1;
      }
      mask[w] = 0;
    }
  }

  /// Move the next non-empty coarse block into the fine level and set the
  /// scan position to its start. False when nothing is left.
  bool refillFine() {
    const uint32_t currentBlock = currentMin_ / FINE_SIZE;
    const uint32_t start = (currentBlock + 1) & COARSE_MASK;
    uint32_t index = nextSet(coarseMask_.data(), COARSE_WORDS, start);
    uint32_t block = 0;
    if (index != NONE) {
      block = currentBlock + 1 + (index - start);
    } else {
      index = nextSet(coarseMask_.data(), COARSE_WORDS, 0);
      if (index == NONE || index >= start) {
        return false;
      }
      block = currentBlock + 1 + (COARSE_SIZE - start) + index;
    }
    for (auto& entry : coarse_[index]) {
      const uint32_t bucket = entry.f & FINE_MASK;
      fine_[bucket].push_back(std::move(entry));
      fineMask_[bucket >> 6U] |= (uint64_t{1} << (bucket & 63U));
    }
    coarse_[index].clear();
    coarseMask_[index >> 6U] &= ~(uint64_t{1} << (index & 63U));
    currentMin_ = block * FINE_SIZE;
    return true;
  }

  std::vector<std::vector<Entry>> fine_;
  std::vector<std::vector<Entry>> coarse_;
  std::array<uint64_t, FINE_WORDS> fineMask_{};
  std::array<uint64_t, COARSE_WORDS> coarseMask_{};
  uint32_t currentMin_ = 0;
  uint32_t size_ = 0;
};

} // namespace mqt::scpd::routing
