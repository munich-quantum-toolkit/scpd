/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// The set of constraint edges of a CDT triangulation as an open-addressing
// hash table in one flat array, in place of `std::unordered_set<Edge>`
// (`CDT::EdgeUSet`): a triangulation copied for every state of the CDT world
// then copies one array, not one node per edge. `cmake/PatchCdt.cmake` puts
// this header beside CDT's and points the typedef at it.
//
// What CDT needs of the set: insert (with whether the edge was new), erase
// by edge, count/find/contains, size/empty, and iteration over the edges.

#ifndef MQT_SCPD_FLAT_EDGE_SET_H
#define MQT_SCPD_FLAT_EDGE_SET_H

#include <cstddef>
#include <cstdint>
#include <iterator>
#include <utility>
#include <vector>

namespace mqt_scpd_cdt {

template <class Edge, class VertInd, VertInd noVertex> class FlatEdgeSet {
public:
  using key_type = Edge;
  using value_type = Edge;
  using size_type = std::size_t;

  class const_iterator {
  public:
    using iterator_category = std::forward_iterator_tag;
    using value_type = Edge;
    using difference_type = std::ptrdiff_t;
    using pointer = const Edge*;
    using reference = const Edge&;
    const_iterator() = default;
    const_iterator(const Edge* at, const Edge* end) : at_(at), end_(end) { skip(); }
    reference operator*() const { return *at_; }
    pointer operator->() const { return at_; }
    const_iterator& operator++() {
      ++at_;
      skip();
      return *this;
    }
    const_iterator operator++(int) {
      auto copy = *this;
      ++*this;
      return copy;
    }
    bool operator==(const const_iterator& other) const { return at_ == other.at_; }
    bool operator!=(const const_iterator& other) const { return at_ != other.at_; }

  private:
    void skip() {
      while (at_ != end_ && *at_ == FlatEdgeSet::sentinel()) {
        ++at_;
      }
    }
    const Edge* at_ = nullptr;
    const Edge* end_ = nullptr;
  };
  using iterator = const_iterator;

  FlatEdgeSet() = default;

  [[nodiscard]] size_type size() const { return count_; }
  [[nodiscard]] bool empty() const { return count_ == 0; }

  void clear() {
    slots_.clear();
    count_ = 0;
  }

  void reserve(const size_type n) {
    if (n * 2 > slots_.size()) {
      rehash(n * 2);
    }
  }

  [[nodiscard]] const_iterator begin() const {
    return const_iterator(slots_.data(), slots_.data() + slots_.size());
  }
  [[nodiscard]] const_iterator end() const {
    return const_iterator(slots_.data() + slots_.size(), slots_.data() + slots_.size());
  }

  [[nodiscard]] const_iterator find(const Edge& edge) const {
    const auto at = slotOf(edge);
    if (at == npos) {
      return end();
    }
    return const_iterator(slots_.data() + at, slots_.data() + slots_.size());
  }
  [[nodiscard]] size_type count(const Edge& edge) const { return slotOf(edge) == npos ? 0 : 1; }
  [[nodiscard]] bool contains(const Edge& edge) const { return slotOf(edge) != npos; }

  std::pair<const_iterator, bool> insert(const Edge& edge) {
    if ((count_ + 1) * 2 > slots_.size()) {
      rehash(slots_.empty() ? 64 : slots_.size() * 2);
    }
    const auto mask = slots_.size() - 1;
    auto at = hashOf(edge) & mask;
    while (!(slots_[at] == sentinel())) {
      if (slots_[at] == edge) {
        return {const_iterator(slots_.data() + at, slots_.data() + slots_.size()), false};
      }
      at = (at + 1) & mask;
    }
    slots_[at] = edge;
    ++count_;
    return {const_iterator(slots_.data() + at, slots_.data() + slots_.size()), true};
  }

  size_type erase(const Edge& edge) {
    auto at = slotOf(edge);
    if (at == npos) {
      return 0;
    }
    // Backward shift: the entries after the hole that hashed before it move
    // up, so no tombstone is needed.
    const auto mask = slots_.size() - 1;
    auto hole = at;
    auto next = (hole + 1) & mask;
    while (!(slots_[next] == sentinel())) {
      const auto home = hashOf(slots_[next]) & mask;
      // Does `next` lie "after" its home while the hole lies between?
      const bool wraps = home <= next ? (hole >= home && hole < next)
                                      : (hole >= home || hole < next);
      if (wraps) {
        slots_[hole] = slots_[next];
        hole = next;
      }
      next = (next + 1) & mask;
    }
    slots_[hole] = sentinel();
    --count_;
    return 1;
  }

  /// The edge that marks an unused slot.
  static Edge sentinel() { return Edge(noVertex, noVertex); }

private:
  static constexpr size_type npos = static_cast<size_type>(-1);

  static std::uint64_t hashOf(const Edge& edge) {
    std::uint64_t key = (static_cast<std::uint64_t>(edge.v1()) << 32U) |
                        static_cast<std::uint64_t>(edge.v2());
    key ^= key >> 30U;
    key *= 0xbf58476d1ce4e5b9ULL;
    key ^= key >> 27U;
    key *= 0x94d049bb133111ebULL;
    key ^= key >> 31U;
    return key;
  }

  [[nodiscard]] size_type slotOf(const Edge& edge) const {
    if (slots_.empty()) {
      return npos;
    }
    const auto mask = slots_.size() - 1;
    auto at = hashOf(edge) & mask;
    while (!(slots_[at] == sentinel())) {
      if (slots_[at] == edge) {
        return at;
      }
      at = (at + 1) & mask;
    }
    return npos;
  }

  void rehash(const size_type capacity) {
    std::vector<Edge> old = std::move(slots_);
    slots_.assign(capacity, sentinel());
    count_ = 0;
    for (const auto& edge : old) {
      if (!(edge == sentinel())) {
        insert(edge);
      }
    }
  }

  std::vector<Edge> slots_;
  size_type count_ = 0;
};

} // namespace mqt_scpd_cdt

#endif
