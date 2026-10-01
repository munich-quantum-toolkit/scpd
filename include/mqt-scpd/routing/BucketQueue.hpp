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

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace mqt::scpd::routing {

/**
 * @brief A priority queue over integer priorities with constant-time push and
 * pop.
 *
 * The queue serves as the open list of the search. It has two levels of
 * buckets. The fine level holds one aligned block of @p FINE_SIZE consecutive
 * priorities, one bucket per priority. The coarse level holds every block
 * above it, one bucket per block. A pop takes the last entry of the lowest
 * non-empty fine bucket. When the fine block is exhausted, the next non-empty
 * coarse block moves into the fine level. Each level keeps a bitmask of its
 * non-empty buckets, so the next bucket is found by counting trailing zeros
 * instead of by a scan.
 *
 * A bucket is a stack of chunks of @c CHUNK_ENTRIES entries each, drawn from
 * one pool that the queue owns. A chunk that a pop or a clear() empties goes
 * back to the pool, so the memory of the queue is the most chunks it held at
 * any one time, whichever buckets the entries fell into. Entries never move
 * once they are pushed, and the queue never asks for memory it already had.
 *
 * Each fine bucket holds exactly one priority, so the pops come out in
 * ascending order of priority. The queue keeps a scan position: the priority
 * the next pop starts from. The scan position never decreases until clear()
 * resets it.
 *
 * @tparam Entry The entry type. It is default-constructible and carries its
 * own priority in an unsigned member named @c f.
 * @tparam FINE_SIZE The number of priorities in one block, a power of two and
 * at least 64.
 * @tparam COARSE_SIZE The number of buckets of the coarse level, a power of
 * two and at least 64.
 * @pre The span of the priorities held at one time stays below @p FINE_SIZE
 * times @p COARSE_SIZE.
 */
template <typename Entry, uint32_t FINE_SIZE = 1024,
          uint32_t COARSE_SIZE = 1024>
class BucketQueue {
  static_assert((FINE_SIZE & (FINE_SIZE - 1)) == 0,
                "the fine size is a power of two");
  static_assert((COARSE_SIZE & (COARSE_SIZE - 1)) == 0,
                "the coarse size is a power of two");
  static_assert(FINE_SIZE >= 64 && COARSE_SIZE >= 64, "at least one mask word");

  /**
   * @brief The mask that maps a priority to its bucket in the fine level.
   */
  static constexpr uint32_t FINE_MASK = FINE_SIZE - 1;
  /**
   * @brief The mask that maps a block to its bucket in the coarse level.
   */
  static constexpr uint32_t COARSE_MASK = COARSE_SIZE - 1;
  /**
   * @brief The number of 64-bit words of the fine bitmask.
   */
  static constexpr uint32_t FINE_WORDS = FINE_SIZE / 64;
  /**
   * @brief The number of 64-bit words of the coarse bitmask.
   */
  static constexpr uint32_t COARSE_WORDS = COARSE_SIZE / 64;

public:
  /**
   * @brief The number of entries of one chunk.
   */
  static constexpr uint32_t CHUNK_ENTRIES = 256;

  /**
   * @brief Creates an empty queue that holds no chunk yet.
   */
  BucketQueue() = default;

  /**
   * @brief Removes every entry.
   * @post The queue is empty and the scan position is zero. Every chunk is back
   * in the pool, so heldEntries() does not change.
   */
  void clear() {
    clearLevel(fine, fineMask);
    clearLevel(coarse, coarseMask);
    entryCount = 0;
    currentMin = 0;
  }

  /**
   * @brief Counts the entries the queue has memory for.
   * @return The entries of every chunk the queue owns, in use or in the pool.
   */
  [[nodiscard]] std::size_t heldEntries() const {
    return chunks.size() * std::size_t{CHUNK_ENTRIES};
  }

  /**
   * @brief Adds an entry.
   *
   * A priority below the scan position is raised to the scan position, so
   * that the entry is not lost behind it.
   *
   * @param entry The entry to add.
   * @param priority The priority of @p entry, equal to its member @c f.
   */
  void push(Entry entry, uint32_t priority) {
    // A slightly inconsistent heuristic can produce a priority just below
    // the current minimum. Clamp it, so that it is not lost behind the scan
    // position.
    priority = std::max(priority, currentMin);
    if ((priority / FINE_SIZE) == (currentMin / FINE_SIZE)) {
      const uint32_t index = priority & FINE_MASK;
      pushOnto(fine[index], std::move(entry));
      fineMask[index >> 6U] |= (uint64_t{1} << (index & 63U));
    } else {
      const uint32_t index = (priority / FINE_SIZE) & COARSE_MASK;
      pushOnto(coarse[index], std::move(entry));
      coarseMask[index >> 6U] |= (uint64_t{1} << (index & 63U));
    }
    ++entryCount;
  }

  /**
   * @brief Removes the entry with the lowest priority.
   *
   * Among the entries of the lowest priority, the pop takes the one added
   * last to their bucket.
   *
   * @pre The queue is not empty.
   * @return The removed entry. A pop from an empty queue returns a
   * default-constructed entry.
   */
  Entry pop() {
    while (true) {
      const uint32_t index = currentMin & FINE_MASK;
      if (fine[index].top != nullptr) {
        Entry entry = popFrom(fine[index]);
        if (fine[index].top == nullptr) {
          fineMask[index >> 6U] &= ~(uint64_t{1} << (index & 63U));
        }
        --entryCount;
        if (entry.f > currentMin) {
          currentMin = entry.f;
        }
        return entry;
      }
      const uint32_t next = nextSet(fineMask.data(), FINE_WORDS, index);
      if (next != NONE) {
        currentMin = (currentMin - index) + next;
        continue;
      }
      if (!refillFine()) {
        // Unreachable when entryCount is kept exactly; never loop forever.
        if (entryCount > 0) {
          --entryCount;
        }
        return Entry{};
      }
    }
  }

  /**
   * @brief Looks at the entry the next pop returns, without removing it.
   *
   * The entry is visible only when it lies in the fine bucket at the scan
   * position. A search can use the result to prefetch the data of the next
   * node.
   *
   * @return The entry, or @c nullptr when the fine bucket at the scan position
   * is empty. The pointer is valid until the queue changes.
   */
  [[nodiscard]] const Entry* peek() const {
    const Bucket& bucket = fine[currentMin & FINE_MASK];
    return bucket.top == nullptr ? nullptr
                                 : &bucket.top->entries[bucket.count - 1];
  }

  /**
   * @brief Reports whether the queue holds no entry.
   * @return @c true when the queue is empty.
   */
  [[nodiscard]] bool empty() const { return entryCount == 0; }
  /**
   * @brief Counts the entries in the queue.
   * @return The number of entries.
   */
  [[nodiscard]] uint32_t size() const { return entryCount; }

private:
  /**
   * @brief The result of nextSet() when no bit is set.
   */
  static constexpr uint32_t NONE = std::numeric_limits<uint32_t>::max();

  /**
   * @brief Finds the first set bit of a bitmask at or after a position.
   * @param mask The words of the bitmask.
   * @param words The number of words of @p mask.
   * @param from The position the search starts at.
   * @pre @p from is below 64 times @p words.
   * @return The position of the first set bit at or after @p from, or @c NONE
   * when there is none.
   */
  static uint32_t nextSet(const uint64_t* mask, const uint32_t words,
                          const uint32_t from) {
    uint32_t word = from >> 6U;
    // The scan reads the words of the bitmask through a raw pointer.
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
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
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  }

  /**
   * @brief A block of entries of one bucket.
   */
  struct Chunk {
    /// The entries, the oldest first.
    std::array<Entry, CHUNK_ENTRIES> entries{};
    /// The chunk below this one in its bucket, or the next chunk of the pool.
    Chunk* below = nullptr;
  };

  /**
   * @brief A bucket: a stack of chunks.
   */
  struct Bucket {
    /// The chunk the next entry goes into, or @c nullptr when the bucket is
    /// empty.
    Chunk* top = nullptr;
    /// The number of entries in @c top. Every chunk below it is full.
    uint32_t count = 0;
  };

  /**
   * @brief Takes a chunk from the pool, or makes one when the pool is empty.
   * @return A chunk that no bucket uses.
   */
  Chunk* takeChunk() {
    if (pool != nullptr) {
      Chunk* chunk = pool;
      pool = chunk->below;
      return chunk;
    }
    chunks.push_back(std::make_unique<Chunk>());
    return chunks.back().get();
  }

  /**
   * @brief Returns a chunk to the pool.
   * @param chunk The chunk, which no bucket uses any more.
   */
  void giveBack(Chunk* chunk) {
    chunk->below = pool;
    pool = chunk;
  }

  /**
   * @brief Pushes an entry onto a bucket.
   * @param bucket The bucket.
   * @param entry The entry.
   */
  void pushOnto(Bucket& bucket, Entry&& entry) {
    if (bucket.top == nullptr || bucket.count == CHUNK_ENTRIES) {
      Chunk* chunk = takeChunk();
      chunk->below = bucket.top;
      bucket.top = chunk;
      bucket.count = 0;
    }
    bucket.top->entries[bucket.count++] = std::move(entry);
  }

  /**
   * @brief Pops the entry pushed last onto a bucket.
   * @param bucket The bucket.
   * @pre @p bucket is not empty.
   * @return The entry.
   */
  Entry popFrom(Bucket& bucket) {
    Entry entry = std::move(bucket.top->entries[--bucket.count]);
    if (bucket.count == 0) {
      Chunk* emptied = bucket.top;
      bucket.top = emptied->below;
      bucket.count = bucket.top == nullptr ? 0 : CHUNK_ENTRIES;
      giveBack(emptied);
    }
    return entry;
  }

  /**
   * @brief Returns every chunk of a bucket to the pool.
   * @param bucket The bucket.
   * @post @p bucket is empty.
   */
  void release(Bucket& bucket) {
    while (bucket.top != nullptr) {
      Chunk* chunk = bucket.top;
      bucket.top = chunk->below;
      giveBack(chunk);
    }
    bucket.count = 0;
  }

  /**
   * @brief Empties the non-empty buckets of one level and clears its bitmask.
   *
   * The function visits only the buckets that the bitmask marks.
   *
   * @tparam Level The array type of the level.
   * @tparam Mask The array type of the bitmask.
   * @param level The buckets of the level.
   * @param mask The bitmask of the non-empty buckets of @p level.
   * @pre @p mask marks every non-empty bucket of @p level.
   * @post Every bucket of @p level is empty, its chunks are back in the pool,
   * and every word of @p mask is zero.
   */
  template <typename Level, typename Mask>
  void clearLevel(Level& level, Mask& mask) {
    for (std::size_t w = 0; w < mask.size(); ++w) {
      uint64_t bits = mask[w];
      while (bits != 0) {
        release(
            level[(w << 6U) | static_cast<uint32_t>(std::countr_zero(bits))]);
        bits &= bits - 1;
      }
      mask[w] = 0;
    }
  }

  /**
   * @brief Moves the next non-empty coarse block into the fine level.
   *
   * The search for the block starts after the current block and wraps around
   * the coarse level once.
   *
   * @return @c true when a block moved, @c false when the coarse level is
   * empty.
   * @post After a move, the scan position is the first priority of the moved
   * block.
   */
  bool refillFine() {
    const uint32_t currentBlock = currentMin / FINE_SIZE;
    const uint32_t start = (currentBlock + 1) & COARSE_MASK;
    uint32_t index = nextSet(coarseMask.data(), COARSE_WORDS, start);
    uint32_t block = 0;
    if (index != NONE) {
      block = currentBlock + 1 + (index - start);
    } else {
      index = nextSet(coarseMask.data(), COARSE_WORDS, 0);
      if (index == NONE || index >= start) {
        return false;
      }
      block = currentBlock + 1 + (COARSE_SIZE - start) + index;
    }
    // The entries move in the order they were pushed, so each fine bucket
    // stacks them as if they had been pushed onto it directly.
    Bucket& source = coarse[index];
    stackScratch.clear();
    for (const Chunk* chunk = source.top; chunk != nullptr;
         chunk = chunk->below) {
      stackScratch.push_back(chunk);
    }
    for (auto chunk = stackScratch.rbegin(); chunk != stackScratch.rend();
         ++chunk) {
      const uint32_t filled =
          (*chunk == source.top) ? source.count : CHUNK_ENTRIES;
      for (uint32_t i = 0; i < filled; ++i) {
        const Entry& entry = (*chunk)->entries[i];
        const uint32_t bucket = entry.f & FINE_MASK;
        pushOnto(fine[bucket], Entry{entry});
        fineMask[bucket >> 6U] |= (uint64_t{1} << (bucket & 63U));
      }
    }
    release(source);
    coarseMask[index >> 6U] &= ~(uint64_t{1} << (index & 63U));
    currentMin = block * FINE_SIZE;
    return true;
  }

  /// The fine level: one bucket per priority of the current block.
  std::array<Bucket, FINE_SIZE> fine{};
  /// The coarse level: one bucket per block, indexed by the block number
  /// modulo @p COARSE_SIZE.
  std::array<Bucket, COARSE_SIZE> coarse{};
  /// Every chunk the queue owns.
  std::vector<std::unique_ptr<Chunk>> chunks;
  /// The chunks no bucket uses, linked through @c Chunk::below.
  Chunk* pool = nullptr;
  /// The chunks of one coarse bucket, top first, while it moves.
  std::vector<const Chunk*> stackScratch;
  /// One bit per fine bucket, set when the bucket is not empty.
  std::array<uint64_t, FINE_WORDS> fineMask{};
  /// One bit per coarse bucket, set when the bucket is not empty.
  std::array<uint64_t, COARSE_WORDS> coarseMask{};
  /// The scan position.
  uint32_t currentMin = 0;
  /// The number of entries in the queue.
  uint32_t entryCount = 0;
};

} // namespace mqt::scpd::routing
