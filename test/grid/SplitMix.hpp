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

#include <cstdint>

namespace mqt::scpd::grid::test {

/**
 * @brief A small random generator (SplitMix64) whose numbers are the same on
 * every platform.
 *
 * The distributions of the standard library differ between implementations,
 * so the random tests draw their numbers from this generator.
 */
class SplitMix {
public:
  /**
   * @brief Creates a generator.
   * @param seed The first state.
   */
  explicit SplitMix(const uint64_t seed) : state(seed) {}

  /**
   * @brief Draws the next number.
   * @return A number from the full range of uint64_t.
   */
  uint64_t next() {
    state += 0x9E3779B97F4A7C15ULL;
    uint64_t z = state;
    z = (z ^ (z >> 30U)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27U)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31U);
  }

  /**
   * @brief Draws a whole number from a closed range.
   * @param low The smallest number.
   * @param high The largest number.
   * @pre @p low is at most @p high, and the range holds fewer than 2^63
   * numbers.
   * @return A number in [@p low, @p high].
   */
  int64_t between(const int64_t low, const int64_t high) {
    return low +
           static_cast<int64_t>(next() % static_cast<uint64_t>(high - low + 1));
  }

  /**
   * @brief Draws a number from [0, 1).
   * @return A number with 53 random bits.
   */
  double unit() { return static_cast<double>(next() >> 11U) * 0x1p-53; }

private:
  uint64_t state;
};

} // namespace mqt::scpd::grid::test
