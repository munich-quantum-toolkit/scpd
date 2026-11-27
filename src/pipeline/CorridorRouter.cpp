/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/pipeline/CorridorRouter.hpp"

#include "mqt-scpd/flatbuffers/artifacts.hpp"
#include "mqt-scpd/flatbuffers/config.hpp"
#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/flatbuffers/geometry.hpp"
#include "mqt-scpd/grid/GridMetrics.hpp"
#include "mqt-scpd/grid/Partitions.hpp"
#include "mqt-scpd/grid/Watershed.hpp"
#include "mqt-scpd/pipeline/CapacityPlanner.hpp"
#include "mqt-scpd/pipeline/Report.hpp"
#include "mqt-scpd/pipeline/Stages.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <queue>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace mqt::scpd::pipeline {
namespace {

namespace fba = flatbuffers::artifacts;
namespace fbg = flatbuffers::geometry;

using grid::LABEL_NONE;
using grid::PartitionLabel;

constexpr std::size_t NO_STATE = std::numeric_limits<std::size_t>::max();
constexpr double UNREACHED = std::numeric_limits<double>::infinity();

/// A point of the partition graph, in the cell coordinates of the partition
/// geometry: whole numbers on cell corners, so that a slot on a cell edge and
/// the center of a cell both land on a half.
struct Place {
  double x = 0.0;
  double y = 0.0;
};

/// One crossing slot: where it is, and the two partitions it joins.
struct Slot {
  Place place;
  PartitionLabel first = LABEL_NONE;
  PartitionLabel second = LABEL_NONE;
  /// The border the slot is on, as an index into the borders of the plan.
  std::uint32_t border = 0;

  /// The partition on the far side of the one a wire arrives from.
  [[nodiscard]] PartitionLabel beyond(const PartitionLabel from) const {
    return from == first ? second : first;
  }
};

/// One wire to route.
struct Request {
  std::size_t connection = 0;
  Place feed;
  Place target;
  PartitionLabel feedPartition = LABEL_NONE;
  PartitionLabel targetPartition = LABEL_NONE;

  /// Whether both endpoints belong to partitions.
  [[nodiscard]] bool hasEndpoints() const {
    return feedPartition != LABEL_NONE && targetPartition != LABEL_NONE;
  }
};

/// The slots a wire crosses and the partitions it runs through. There is
/// always one more partition than there are slots.
struct Route {
  std::vector<std::size_t> slots;
  std::vector<PartitionLabel> partitions;
};

/// A chord already drawn across a partition, and the wire that drew it.
struct Chord {
  Place from;
  Place to;
  std::size_t owner = 0;
};

/// The current sweep step, repeated during a long search to check for stops.
struct SearchProgress {
  std::reference_wrapper<const Report> report;
  std::string_view task;
  std::size_t done;
  std::size_t total;

  void update() const { report.get().progress(task, "", done, total); }
};

/// A cell coordinate back on the grid it came from.
///
/// The plan carries its border samples in layout units, and the way back
/// costs a little: a sample that was exactly half a cell comes back a
/// fraction of an ulp away from it. Every place the search works with is a
/// whole number or a half, and the determinant below is exact only while that
/// holds. A slot a hair off its half turns a point that lies on a chord into
/// one that lies beside it, and two wires then meet without the search
/// seeing it.
[[nodiscard]] double onGrid(const double value) {
  return std::round(value * 2.0) / 2.0;
}

/// Checks a capacity label before the router uses it.
[[nodiscard]] PartitionLabel checkedLabel(const std::uint32_t label) {
  if (label < grid::FIRST_PARTITION_LABEL ||
      label >= std::numeric_limits<PartitionLabel>::max()) {
    throw std::invalid_argument(
        std::format("partition label {} of the capacity plan is outside the "
                    "supported range",
                    label));
  }
  return static_cast<PartitionLabel>(label);
}

/// Converts a point from a capacity artifact before it can reach a grid index.
[[nodiscard]] grid::Point checkedCorner(const grid::GridMetrics& grid,
                                        const fbg::Point& point,
                                        const std::string_view kind) {
  const auto corner = grid::layoutToCorner(grid, point);
  // Leave room for round-off at the grid edges.
  if (corner.x() > -0.5 && corner.x() < grid.width + 0.5 && corner.y() > -0.5 &&
      corner.y() < grid.height + 0.5) {
    return corner;
  }
  throw std::invalid_argument(
      std::format("the capacity plan has a {} outside the detail grid", kind));
}

/// Which side of the line through `a` and `b` the point `c` lies on.
///
/// Every coordinate is a whole number or a half, so the determinant is exact
/// in double for any grid this tool builds.
[[nodiscard]] int sideOf(const Place& a, const Place& b, const Place& c) {
  const auto cross = ((b.x - a.x) * (c.y - a.y)) - ((b.y - a.y) * (c.x - a.x));
  if (cross > 0.0) {
    return 1;
  }
  return cross < 0.0 ? -1 : 0;
}

/// Whether the point `c` lies on the chord from `a` to `b`, between its ends.
///
/// The three points are known to lie on one line, so what is left is whether
/// `c` falls between the two along it.
[[nodiscard]] bool between(const Place& a, const Place& b, const Place& c) {
  return std::min(a.x, b.x) <= c.x && c.x <= std::max(a.x, b.x) &&
         std::min(a.y, b.y) <= c.y && c.y <= std::max(a.y, b.y);
}

/// Whether two chords intersect, including at an endpoint.
[[nodiscard]] bool meets(const Chord& a, const Chord& b) {
  if (std::max(a.from.x, a.to.x) < std::min(b.from.x, b.to.x) ||
      std::max(b.from.x, b.to.x) < std::min(a.from.x, a.to.x) ||
      std::max(a.from.y, a.to.y) < std::min(b.from.y, b.to.y) ||
      std::max(b.from.y, b.to.y) < std::min(a.from.y, a.to.y)) {
    return false;
  }
  const auto aFrom = sideOf(a.from, a.to, b.from);
  const auto aTo = sideOf(a.from, a.to, b.to);
  const auto bFrom = sideOf(b.from, b.to, a.from);
  const auto bTo = sideOf(b.from, b.to, a.to);
  return (aFrom * aTo < 0 && bFrom * bTo < 0) ||
         (aFrom == 0 && between(a.from, a.to, b.from)) ||
         (aTo == 0 && between(a.from, a.to, b.to)) ||
         (bFrom == 0 && between(b.from, b.to, a.from)) ||
         (bTo == 0 && between(b.from, b.to, a.to));
}

/// Whether a sample lies on the shared edge of cells with the border labels.
[[nodiscard]] bool separates(const grid::GridMetrics& detail,
                             const std::span<const PartitionLabel> labels,
                             const grid::PartitionBorder& border,
                             const grid::Point& sample) {
  const auto x = static_cast<std::int64_t>(std::floor(sample.x()));
  const auto y = static_cast<std::int64_t>(std::floor(sample.y()));
  const auto vertical = sample.x() == static_cast<double>(x);
  const auto horizontal = sample.y() == static_cast<double>(y);
  const auto otherX = x - (vertical ? 1 : 0);
  const auto otherY = y - (horizontal ? 1 : 0);
  if (horizontal == vertical || !detail.contains(x, y) ||
      !detail.contains(otherX, otherY)) {
    return false;
  }
  const auto labelAt = [&](const std::int64_t cellX, const std::int64_t cellY) {
    return labels[detail.index(static_cast<std::uint32_t>(cellX),
                               static_cast<std::uint32_t>(cellY))];
  };
  const auto first = labelAt(x, y);
  const auto second = labelAt(otherX, otherY);
  return (first == border.first && second == border.second) ||
         (first == border.second && second == border.first);
}

/// Rebuilds crossing slots from the borders recorded by Capacity.
///
/// @throws std::invalid_argument If a border does not separate its two
/// partitions or offers a different number of slots than its budget.
[[nodiscard]] std::vector<Slot>
slotsOf(const CapacityPlanT& plan, const CapacityScene& scene,
        const std::span<const PartitionLabel> labels, const double pitch,
        const Report& report) {
  std::vector<Slot> slots;
  const auto& detail = scene.detail;
  std::size_t samples = 0;
  for (std::uint32_t index = 0; index < plan.borders.size(); ++index) {
    const auto* entry = plan.borders[index].get();
    if (entry == nullptr) {
      throw std::invalid_argument(
          std::format("the capacity plan has no border {}", index));
    }
    const auto& border = *entry;
    grid::PartitionBorder geometry{.first = checkedLabel(border.first),
                                   .second = checkedLabel(border.second),
                                   .samples = {}};
    const auto borderError = [&] {
      return std::invalid_argument(
          std::format("border {} of the capacity plan does not separate "
                      "partitions {} and {}",
                      index, border.first, border.second));
    };
    if (geometry.first == geometry.second) {
      throw borderError();
    }
    geometry.samples.reserve(border.samples.size());
    for (const auto& sample : border.samples) {
      if (samples++ % 1024 == 0) {
        report.progress("building crossing slots", "", index,
                        plan.borders.size());
      }
      const auto corner = checkedCorner(detail, sample, "border sample");
      auto& point =
          geometry.samples.emplace_back(onGrid(corner.x()), onGrid(corner.y()));
      if (!separates(detail, labels, geometry, point)) {
        throw borderError();
      }
    }
    const auto positions = crossingSlots(geometry, scene, pitch);
    if (positions.size() != border.budget) {
      throw std::invalid_argument(std::format(
          "border {} between partitions {} and {} offers {} crossing slots "
          "and the plan gives it a budget of {}; the plan was made with "
          "another crossing pitch or by another version of mqt-scpd, so run "
          "the capacity step again",
          index, border.first, border.second, positions.size(), border.budget));
    }
    for (const auto& position : positions) {
      slots.push_back({.place = {.x = position.x(), .y = position.y()},
                       .first = geometry.first,
                       .second = geometry.second,
                       .border = index});
    }
  }
  return slots;
}

/// The partition graph of one run, with the wires committed to it.
class Corridors {
public:
  Corridors(const grid::GridMetrics& detail, std::vector<Slot> slots,
            const double pitch, const Report& report)
      : detailGrid(detail), slotList(std::move(slots)) {
    for (std::size_t index = 0; index < slotList.size(); ++index) {
      slotsByPartition[slotList[index].first].push_back(index);
      slotsByPartition[slotList[index].second].push_back(index);
    }
    findNearby(pitch, report);
    slotOwner.assign(slotList.size(), NO_OWNER);
    slotHistory.assign(slotList.size(), 0.0);
  }

  [[nodiscard]] const std::vector<Slot>& slots() const { return slotList; }

  /// What one blocked step costs the search: the diagonal of the detail grid,
  /// which is longer than most routes, so length rarely buys a displacement.
  [[nodiscard]] double displacementPrice() const {
    const auto box = detailGrid.box();
    return std::hypot(box.maxX - box.minX, box.maxY - box.minY);
  }

  /// Searches the way of one wire from its feed to its target, and searches
  /// again when the cheapest way is one the wire may not take.
  ///
  /// With no `displacement` the search uses only what is free. Given one, it
  /// may take a slot that another wire holds or cross a chord that another
  /// wire drew, and pays that price once per step that does so, however many
  /// wires the step displaces. The path cost adds this penalty to distance
  /// and slot history. A sweep asks for a displacing route only for a wire
  /// that the free search did not place, so the price never trades a free
  /// way for a shorter blocked one.
  ///
  /// A route that crosses itself or comes back to a slot it already used is
  /// a short. That is a property of the whole route and not of one step, so
  /// the search cannot rule it out as it goes. The slots of such a way are
  /// set aside and the search runs again, at most UNSOUND_RETRIES times. This
  /// often finds a second-best way, but not always: a slot that every way of
  /// the wire needs is set aside too.
  [[nodiscard]] std::optional<Route>
  route(const Request& request, const SearchProgress& progress,
        const double displacement = 0.0) const {
    if (!request.hasEndpoints()) {
      return std::nullopt;
    }
    // A clear direct route has no slot cost and is already the shortest.
    if (request.feedPartition == request.targetPartition &&
        clear(Chord{.from = request.feed,
                    .to = request.target,
                    .owner = request.connection},
              request.feedPartition)) {
      return Route{.slots = {}, .partitions = {request.feedPartition}};
    }

    std::vector<bool> aside(slotList.size(), false);
    for (std::uint32_t attempt = 0; attempt <= UNSOUND_RETRIES; ++attempt) {
      auto found = search(request, displacement, aside, progress);
      if (!found.has_value()) {
        return std::nullopt;
      }
      if (sound(request, *found)) {
        return found;
      }
      for (const auto slot : found->slots) {
        aside[slot] = true;
      }
    }
    return std::nullopt;
  }

  /// Lays a route down: its slots are taken and its chords drawn.
  void commit(const Request& request, const Route& route) {
    for (const auto slot : route.slots) {
      slotOwner[slot] = request.connection;
    }
    walk(request, route,
         [&](const Chord& chord, const PartitionLabel partition) {
           chordsByPartition[partition].push_back(chord);
         });
  }

  /// Takes a route back out.
  void withdraw(const Request& request, const Route& route) {
    for (const auto slot : route.slots) {
      slotOwner[slot] = NO_OWNER;
    }
    for (const auto partition : route.partitions) {
      auto& chords = chordsByPartition[partition];
      std::erase_if(chords, [&](const Chord& chord) {
        return chord.owner == request.connection;
      });
    }
  }

  /// The wires in the way of a route that may displace others: the ones that
  /// hold a slot it wants or a slot within one pitch of it, and the ones
  /// whose chords it would have to cross.
  [[nodiscard]] std::vector<std::size_t> blockersOf(const Request& request,
                                                    const Route& wanted) const {
    std::vector<std::size_t> owners;
    for (const auto slot : wanted.slots) {
      forEachHolder(slot, request.connection, [&](const std::size_t holder) {
        owners.push_back(slotOwner[holder]);
      });
    }
    walk(request, wanted,
         [&](const Chord& mine, const PartitionLabel partition) {
           const auto found = chordsByPartition.find(partition);
           if (found == chordsByPartition.end()) {
             return;
           }
           for (const auto& chord : found->second) {
             if (meets(mine, chord)) {
               owners.push_back(chord.owner);
             }
           }
         });
    std::ranges::sort(owners);
    const auto duplicates = std::ranges::unique(owners);
    owners.erase(duplicates.begin(), duplicates.end());
    std::erase(owners, request.connection);
    return owners;
  }

  /// Makes the slots that kept a wire from its route dearer for whoever
  /// stands on them: the slots it wanted and the slots within one pitch of
  /// them that another wire holds.
  ///
  /// Without this two wires trade one slot back and forth: each displaces
  /// the other, each round undoes the last, and neither settles. The price
  /// only rises, so the wire in the way often finds its own way cheaper
  /// elsewhere and gives the slot up. The charge is a fraction of one
  /// displacement, so a few rounds of wanting a slot add up to more than the
  /// detour around it. Where the slots are too few for every wire, no price
  /// settles the trade, and the stage keeps the best round instead.
  void chargeFor(const Request& request, const Route& route) {
    const auto amount = displacementPrice() / HISTORY_STEPS;
    std::vector<std::size_t> held;
    for (const auto slot : route.slots) {
      forEachHolder(slot, request.connection,
                    [&](const std::size_t holder) { held.push_back(holder); });
    }
    std::ranges::sort(held);
    const auto duplicates = std::ranges::unique(held);
    held.erase(duplicates.begin(), duplicates.end());
    for (const auto slot : held) {
      slotHistory[slot] += amount;
    }
  }

  /// The layout position of a slot, for the artifact.
  [[nodiscard]] fbg::Point layoutOf(const std::size_t slot) const {
    return grid::cornerToLayout(detailGrid, slotList[slot].place.x,
                                slotList[slot].place.y);
  }

private:
  /// How often a wire may be handed a second-best way when the cheapest one
  /// is a short.
  static constexpr std::uint32_t UNSOUND_RETRIES = 4;

  /// How many rounds of wanting one slot add up to the price of displacing a
  /// wire from it.
  static constexpr double HISTORY_STEPS = 8.0;

  static constexpr std::size_t NO_OWNER =
      std::numeric_limits<std::size_t>::max();

  /// Finds, for each slot, the other slots that lie closer than one crossing
  /// pitch.
  ///
  /// The slots of one border keep the pitch apart, but the slots of two
  /// borders do not: where partitions meet, the borders end next to one
  /// another, and each has a slot near the corner. Two wires that crossed
  /// there would run closer than the pitch. One wire may cross at both, as it
  /// does when it passes the corner from one partition into the next.
  void findNearby(const double pitch, const Report& report) {
    nearbySlots.assign(slotList.size(), {});
    // Keep the bucket indices bounded even when the pitch is tiny.
    const auto width = std::max(pitch, detailGrid.cellWidth);
    const auto height = std::max(pitch, detailGrid.cellHeight);
    const auto bucketOf = [&](const fbg::Point& position) {
      return std::pair{static_cast<std::int64_t>(std::floor(
                           (position.x() - detailGrid.origin.x()) / width)),
                       static_cast<std::int64_t>(std::floor(
                           (position.y() - detailGrid.origin.y()) / height))};
    };
    std::vector<fbg::Point> layout;
    layout.reserve(slotList.size());
    std::map<std::pair<std::int64_t, std::int64_t>, std::vector<std::size_t>>
        buckets;
    for (std::size_t slot = 0; slot < slotList.size(); ++slot) {
      layout.push_back(layoutOf(slot));
      buckets[bucketOf(layout.back())].push_back(slot);
    }
    for (std::size_t slot = 0; slot < slotList.size(); ++slot) {
      if (slot % 128 == 0) {
        report.progress("checking crossing pitch", "", slot, slotList.size());
      }
      const auto& here = layout[slot];
      const auto [column, row] = bucketOf(here);
      for (std::int64_t dy = -1; dy <= 1; ++dy) {
        for (std::int64_t dx = -1; dx <= 1; ++dx) {
          const auto found = buckets.find({column + dx, row + dy});
          if (found == buckets.end()) {
            continue;
          }
          for (const auto other : found->second) {
            const auto& there = layout[other];
            if (other != slot && std::hypot(here.x() - there.x(),
                                            here.y() - there.y()) < pitch) {
              nearbySlots[slot].push_back(other);
            }
          }
        }
      }
    }
  }

  /// Hands `visit` every slot that keeps a wire other than `connection` from
  /// taking `slot`: the slot itself or a slot within one pitch of it, when
  /// another wire holds it.
  template <typename Visitor>
  void forEachHolder(const std::size_t slot, const std::size_t connection,
                     const Visitor& visit) const {
    const auto check = [&](const std::size_t candidate) {
      if (slotOwner[candidate] != NO_OWNER &&
          slotOwner[candidate] != connection) {
        visit(candidate);
      }
    };
    check(slot);
    for (const auto other : nearbySlots[slot]) {
      check(other);
    }
  }

  /// Whether a wire other than `connection` holds `slot` or a slot within
  /// one pitch of it.
  [[nodiscard]] bool heldByAnother(const std::size_t slot,
                                   const std::size_t connection) const {
    auto held = false;
    forEachHolder(slot, connection, [&](std::size_t) { held = true; });
    return held;
  }

  [[nodiscard]] std::optional<Route>
  search(const Request& request, const double displacement,
         const std::vector<bool>& aside, const SearchProgress& progress) const {
    // A state is a slot together with the partition the wire carries on
    // into, because a slot joins two partitions and the one a wire came from
    // decides where it may go next.
    std::vector<double> best(slotList.size() * 2, UNREACHED);
    std::vector<std::size_t> from(slotList.size() * 2, NO_STATE);
    std::vector<bool> closed(slotList.size() * 2, false);

    using Entry = std::pair<double, std::size_t>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<>> queue;

    std::size_t work = 0;
    const auto checkpoint = [&] {
      constexpr std::size_t interval = 1024;
      if (work++ % interval == 0) {
        progress.update();
      }
    };
    const auto expand = [&](const Place& place, const PartitionLabel partition,
                            const double cost, const std::size_t parent) {
      // The partition the wire just left. Turning straight back into it lets
      // a wire snake across one border and back again and again: the detour
      // is short, so it costs the search almost nothing and costs the chip
      // two slots each time.
      const auto behind = parent == NO_STATE
                              ? LABEL_NONE
                              : slotList[parent / 2].beyond(partition);
      for (const auto slot : slotsOf(partition)) {
        checkpoint();
        const auto next = slotList[slot].beyond(partition);
        if (aside[slot] || next == behind) {
          continue;
        }
        const auto state = stateOf(slot, next);
        if (closed[state]) {
          continue;
        }
        const auto blocked = heldByAnother(slot, request.connection) ||
                             !clear(Chord{.from = place,
                                          .to = slotList[slot].place,
                                          .owner = request.connection},
                                    partition);
        if (blocked && displacement <= 0.0) {
          continue;
        }
        const auto step = cost + distance(place, slotList[slot].place) +
                          slotHistory[slot] + (blocked ? displacement : 0.0);
        if (step < best[state]) {
          best[state] = step;
          from[state] = parent;
          queue.emplace(step + distance(slotList[slot].place, request.target),
                        state);
        }
      }
    };

    // A blocked direct route still competes when displacement is allowed.
    auto shortest =
        request.feedPartition == request.targetPartition && displacement > 0.0
            ? distance(request.feed, request.target) + displacement
            : UNREACHED;
    auto arrival = NO_STATE;

    expand(request.feed, request.feedPartition, 0.0, NO_STATE);
    while (!queue.empty()) {
      checkpoint();
      const auto [estimate, state] = queue.top();
      queue.pop();
      if (estimate >= shortest) {
        break;
      }
      if (closed[state]) {
        continue;
      }
      closed[state] = true;

      const auto& slot = slotList[state / 2];
      const auto partition = partitionOf(state);
      if (partition == request.targetPartition) {
        const auto reaches = clear(Chord{.from = slot.place,
                                         .to = request.target,
                                         .owner = request.connection},
                                   partition);
        const auto total = best[state] + distance(slot.place, request.target) +
                           (reaches ? 0.0 : displacement);
        if ((reaches || displacement > 0.0) && total < shortest) {
          shortest = total;
          arrival = state;
        }
      }
      expand(slot.place, partition, best[state], state);
    }

    if (shortest == UNREACHED) {
      return std::nullopt;
    }

    Route route;
    for (auto state = arrival; state != NO_STATE; state = from[state]) {
      route.slots.push_back(state / 2);
      route.partitions.push_back(partitionOf(state));
    }
    std::ranges::reverse(route.slots);
    std::ranges::reverse(route.partitions);
    route.partitions.insert(route.partitions.begin(), request.feedPartition);
    return route;
  }

  [[nodiscard]] std::size_t stateOf(const std::size_t slot,
                                    const PartitionLabel partition) const {
    return (slot * 2) + (partition == slotList[slot].second ? 1U : 0U);
  }

  [[nodiscard]] PartitionLabel partitionOf(const std::size_t state) const {
    const auto& slot = slotList[state / 2];
    return (state % 2) == 1 ? slot.second : slot.first;
  }

  [[nodiscard]] std::span<const std::size_t>
  slotsOf(const PartitionLabel partition) const {
    const auto found = slotsByPartition.find(partition);
    return found == slotsByPartition.end() ? std::span<const std::size_t>{}
                                           : std::span{found->second};
  }

  [[nodiscard]] double distance(const Place& a, const Place& b) const {
    const auto from = grid::cornerToLayout(detailGrid, a.x, a.y);
    const auto to = grid::cornerToLayout(detailGrid, b.x, b.y);
    return std::hypot(from.x() - to.x(), from.y() - to.y());
  }

  /// Whether a chord can be drawn across a partition without meeting one
  /// that is there already.
  [[nodiscard]] bool clear(const Chord& mine,
                           const PartitionLabel partition) const {
    const auto found = chordsByPartition.find(partition);
    if (found == chordsByPartition.end()) {
      return true;
    }
    return std::ranges::none_of(
        found->second, [&](const Chord& chord) { return meets(mine, chord); });
  }

  /// Whether a route is one a wire may take.
  ///
  /// The search sees only what other wires committed, so a wire that enters a
  /// partition it already crossed can lay a chord over its own, or come back
  /// to a slot it already used. Either is a short, and the route is thrown
  /// away.
  [[nodiscard]] bool sound(const Request& request, const Route& route) const {
    auto slots = route.slots;
    std::ranges::sort(slots);
    if (std::ranges::adjacent_find(slots) != slots.end()) {
      return false;
    }
    return !selfCrossing(request, route);
  }

  /// Whether a wire lays a chord over one of its own.
  ///
  /// Only chords in one partition are compared. Two chords that follow one
  /// another share the slot between them, but they lie in the two partitions
  /// on either side of its border, so any meeting of two chords in one
  /// partition is a short.
  [[nodiscard]] bool selfCrossing(const Request& request,
                                  const Route& route) const {
    std::unordered_map<PartitionLabel, std::vector<Chord>> own;
    auto found = false;
    walk(
        request, route, [&](const Chord& mine, const PartitionLabel partition) {
          auto& chords = own[partition];
          found = found || std::ranges::any_of(chords, [&](const Chord& drawn) {
                    return meets(mine, drawn);
                  });
          chords.push_back(mine);
        });
    return found;
  }

  /// Hands every chord of a route to `visit`, in order. A route of k slots
  /// has k + 1 chords, one per partition it runs through.
  template <typename Visitor>
  void walk(const Request& request, const Route& route,
            const Visitor& visit) const {
    auto place = request.feed;
    for (std::size_t index = 0; index < route.slots.size(); ++index) {
      const auto& next = slotList[route.slots[index]].place;
      visit(Chord{.from = place, .to = next, .owner = request.connection},
            route.partitions[index]);
      place = next;
    }
    visit(
        Chord{.from = place, .to = request.target, .owner = request.connection},
        route.partitions.back());
  }

  grid::GridMetrics detailGrid;
  std::vector<Slot> slotList;
  std::unordered_map<PartitionLabel, std::vector<std::size_t>> slotsByPartition;
  std::unordered_map<PartitionLabel, std::vector<Chord>> chordsByPartition;
  /// The wire that holds each slot, or NO_OWNER.
  std::vector<std::size_t> slotOwner;
  /// What wanting each slot has added to its price so far.
  std::vector<double> slotHistory;
  /// For each slot, the other slots that lie closer than one crossing pitch.
  std::vector<std::vector<std::size_t>> nearbySlots;
};

/// The tuning the stage reads.
struct Sweeps {
  std::uint32_t rounds;
  /// The relaxation that the configuration asks for.
  std::uint32_t askedRelaxation;
  /// The relaxation that the sweeps use: the asked one, but fewer than half
  /// the connections, and at least one where two or more connections compete.
  std::uint32_t maxRelaxation;
};

/// Reads the tuning and limits the relaxation to fewer than half of the
/// `connections`, but to at least one when there are two or more.
///
/// A wire without a way rips up neighbors until it fits. When it may rip up
/// all the others, it takes their room, they find no way back, and every
/// round repeats this. Fewer than half leaves at least as many wires in place
/// as the sweep takes out. Two connections still get one: otherwise neither
/// could ever move the other aside, and a wire that takes the slot of the
/// other would keep it in every round.
[[nodiscard]] Sweeps sweepsOf(const ConfigT& config,
                              const std::size_t connections) {
  const flatbuffers::config::CorridorParamsT defaults;
  const auto& params =
      config.stages != nullptr && config.stages->corridor != nullptr
          ? *config.stages->corridor
          : defaults;
  if (params.rounds == 0) {
    throw std::invalid_argument("stages.corridor: rounds must be at least one");
  }
  const auto limit =
      connections < 2 ? 0 : std::max<std::size_t>(1, (connections - 1) / 2);
  return {.rounds = params.rounds,
          .askedRelaxation = params.max_relaxation,
          .maxRelaxation = static_cast<std::uint32_t>(
              std::min<std::size_t>(params.max_relaxation, limit))};
}

/// Checks that every connection of an assignment names a port of the chip.
///
/// @throws std::invalid_argument When a connection is missing or names a port
/// the chip does not carry.
void requirePortsOfChip(const AssignmentT& assignment, const ChipT& chip) {
  for (std::size_t index = 0; index < assignment.connections.size(); ++index) {
    const auto* connection = assignment.connections[index].get();
    if (connection == nullptr) {
      throw std::invalid_argument(
          std::format("the assignment has no connection {}", index));
    }
    if (connection->target.index() >= chip.ports.size()) {
      throw std::invalid_argument(std::format(
          "the assignment does not fit the chip: connection {} names port {} "
          "of {}",
          index, connection->target.index(), chip.ports.size()));
    }
  }
}

/// Checks that a plan was made on the detail grid that the stage rebuilds from
/// the chip and the configuration.
///
/// The plan carries its geometry in layout units, so a plan from another grid
/// still reads, but its outlines and borders no longer fall on cell edges.
///
/// @throws std::invalid_argument When the two grids differ.
void requireSameGrid(const CapacityPlanT& plan,
                     const grid::GridMetrics& detail) {
  const auto* extent = plan.detail_grid.get();
  if (extent == nullptr) {
    throw std::invalid_argument("the capacity plan has no detail grid");
  }
  if (extent->width != detail.width || extent->height != detail.height ||
      extent->origin != detail.origin ||
      extent->cell_width != detail.cellWidth ||
      extent->cell_height != detail.cellHeight) {
    throw std::invalid_argument(std::format(
        "the capacity plan was made on a detail grid of {}x{} cells of "
        "{:.6g}x{:.6g} and the configuration asks for {}x{} cells of "
        "{:.6g}x{:.6g}",
        extent->width, extent->height, extent->cell_width, extent->cell_height,
        detail.width, detail.height, detail.cellWidth, detail.cellHeight));
  }
}

/// Checks that each partition is four-connected and avoids blocked and
/// reserved cells.
void requireConnectedLabels(const std::span<const PartitionLabel> labels,
                            const CapacityScene& scene, const Report& report) {
  const auto& detail = scene.detail;
  std::vector<bool> seen(labels.size(), false);
  std::vector<bool> seenLabels(std::numeric_limits<PartitionLabel>::max(),
                               false);
  std::vector<std::size_t> pending;
  std::size_t visited = 0;
  const auto checkProgress = [&] {
    if (visited++ % 1024 == 0) {
      report.progress("checking partition connectivity", "", 0, 0);
    }
  };

  for (std::size_t start = 0; start < labels.size(); ++start) {
    checkProgress();
    const auto label = labels[start];
    if (label == LABEL_NONE || seen[start]) {
      continue;
    }
    if (seenLabels[label]) {
      throw std::invalid_argument(
          std::format("partition {} of the capacity plan falls apart into "
                      "pieces that do not touch",
                      label));
    }
    seenLabels[label] = true;
    const auto add = [&](const std::size_t cell) {
      if (!seen[cell] && labels[cell] == label) {
        seen[cell] = true;
        pending.push_back(cell);
      }
    };
    add(start);
    while (!pending.empty()) {
      checkProgress();
      const auto cell = pending.back();
      pending.pop_back();
      if (scene.blocked.test(cell)) {
        throw std::invalid_argument(std::format(
            "partition {} of the capacity plan covers a blocked cell", label));
      }
      if (scene.reserved.test(cell)) {
        throw std::invalid_argument(std::format(
            "partition {} of the capacity plan covers a reserved port approach",
            label));
      }
      const auto x = cell % detail.width;
      if (x > 0) {
        add(cell - 1);
      }
      if (x + 1 < detail.width) {
        add(cell + 1);
      }
      if (cell >= detail.width) {
        add(cell - detail.width);
      }
      if (cell + detail.width < labels.size()) {
        add(cell + detail.width);
      }
    }
  }
}

/// Rebuilds and validates the Capacity partition labels on the detail grid.
[[nodiscard]] std::vector<PartitionLabel> labelsOf(const CapacityPlanT& plan,
                                                   const CapacityScene& scene,
                                                   const Report& report) {
  const auto& detail = scene.detail;
  const auto progress = [&] {
    report.progress("rasterizing the partitions", "", 0, 0);
  };
  std::vector<grid::PartitionOutline> outlines;
  for (const auto& partition : plan.partitions) {
    if (partition == nullptr) {
      throw std::invalid_argument("the capacity plan has a missing partition");
    }
    const auto label = checkedLabel(partition->label);
    for (const auto& ring : partition->outlines) {
      if (ring == nullptr) {
        throw std::invalid_argument(
            std::format("partition {} has a missing outline", label));
      }
      grid::PartitionOutline outline{.label = label, .ring = {}};
      outline.ring.reserve(ring->vertices.size());
      for (const auto& vertex : ring->vertices) {
        if (outline.ring.size() % 1024 == 0) {
          progress();
        }
        outline.ring.push_back(
            checkedCorner(detail, vertex, "partition outline"));
      }
      outlines.push_back(std::move(outline));
    }
  }
  auto labels = grid::rasterizePartitions(outlines, detail, progress);
  requireConnectedLabels(labels, scene, report);
  return labels;
}

/// The partition a cell lies in, or the nearest one when the cell itself
/// carries no label.
///
/// A feed sits between two launchers, and a target was placed beyond the
/// band of its port, so either can land a cell short of the free space the
/// partitions were traced from. The search then starts from the nearest cell
/// that lies in a partition, in a square that grows until it finds one.
[[nodiscard]] PartitionLabel
partitionAt(const std::vector<PartitionLabel>& labels,
            const grid::GridMetrics& grid, const std::size_t cell,
            const std::uint32_t reach) {
  if (labels[cell] != LABEL_NONE) {
    return labels[cell];
  }
  const auto center = grid.cell(cell);
  for (std::uint32_t radius = 1; radius <= reach; ++radius) {
    const auto span = static_cast<std::int64_t>(radius);
    for (std::int64_t dy = -span; dy <= span; ++dy) {
      for (std::int64_t dx = -span; dx <= span; ++dx) {
        if (std::max(std::abs(dx), std::abs(dy)) != span) {
          continue;
        }
        const auto x = static_cast<std::int64_t>(center.x()) + dx;
        const auto y = static_cast<std::int64_t>(center.y()) + dy;
        if (!grid.contains(x, y)) {
          continue;
        }
        const auto index = grid.index(static_cast<std::uint32_t>(x),
                                      static_cast<std::uint32_t>(y));
        if (labels[index] != LABEL_NONE) {
          return labels[index];
        }
      }
    }
  }
  return LABEL_NONE;
}

/// The center of a cell, in the corner coordinates that the partition
/// geometry and the crossing slots share.
[[nodiscard]] Place placeOf(const grid::GridMetrics& grid,
                            const std::size_t cell) {
  const auto coordinate = grid.cell(cell);
  return {.x = static_cast<double>(coordinate.x()) + 0.5,
          .y = static_cast<double>(coordinate.y()) + 0.5};
}

/// How many wires are still without a way.
[[nodiscard]] std::size_t
unrouted(const std::vector<std::optional<Route>>& routes) {
  return static_cast<std::size_t>(std::ranges::count_if(
      routes, [](const auto& route) { return !route.has_value(); }));
}

/// The figure of how many wires have a way, such as "routed 56/58".
[[nodiscard]] Figure routedFigure(const std::size_t left,
                                  const std::size_t total) {
  return {.text = std::format("routed {}/{}", total - left, total)};
}

/// The corridor router of the first release.
class PartitionAStarRouter final : public ICorridorRouter {
public:
  [[nodiscard]] CorridorRoutingT run(const ChipT& chip,
                                     const CapacityPlanT& capacity,
                                     const AssignmentT& assignment,
                                     const ConfigT& config,
                                     const Report& report) const override {
    const auto sweeps = sweepsOf(config, assignment.connections.size());
    report.progress("rasterizing the chip", "", 0, 0);
    const auto scene = buildScene(chip, config);
    report.progress("rasterizing the partitions", "", 0, 0);
    requireSameGrid(capacity, scene.detail);
    if (capacity.blocked_words != scene.blocked.words()) {
      throw std::invalid_argument(
          "the blocked cells of the capacity plan do not match the chip and "
          "configuration; run the capacity step again");
    }
    requirePortsOfChip(assignment, chip);
    const auto labels = labelsOf(capacity, scene, report);

    const auto requests = requestsOf(scene, assignment, labels);
    const auto missingEndpoints = static_cast<std::size_t>(
        std::ranges::count_if(requests, [](const Request& request) {
          return !request.hasEndpoints();
        }));
    const auto pitch = crossingPitch(config);
    Corridors corridors(scene.detail,
                        slotsOf(capacity, scene, labels, pitch, report), pitch,
                        report);

    if (report.wants(Detail::Steps)) {
      report.line(
          Detail::Steps,
          std::format("{}{}{}{}{}", counted(requests.size(), "connection"),
                      FIGURE_SEPARATOR,
                      counted(capacity.partitions.size(), "partition"),
                      FIGURE_SEPARATOR,
                      counted(corridors.slots().size(), "crossing slot")));
      report.line(
          Detail::Steps,
          std::format("rounds {}{}max relaxation {}{}", sweeps.rounds,
                      FIGURE_SEPARATOR, sweeps.maxRelaxation,
                      sweeps.maxRelaxation < sweeps.askedRelaxation
                          ? std::format(" of {}", sweeps.askedRelaxation)
                          : ""));
    }

    std::vector<std::optional<Route>> routes(requests.size());
    // A round can rip up more wires than it places again, so the fails of
    // the rounds go up and down, and on a crowded chip two rounds can undo
    // each other until the last. The plan of the round with the fewest fails
    // is kept, the latest of them on a tie. Its fails include the wires that
    // the round ripped after its sweep had passed them, and so did not place
    // again.
    auto best = routes;
    auto bestLeft = requests.size();
    std::uint32_t bestRound = 0;
    std::uint32_t rounds = 0;
    for (std::uint32_t round = 0; round < sweeps.rounds; ++round) {
      const auto forward = (round % 2) == 0;
      const auto name = std::format("round {} {}", round + 1,
                                    forward ? "forward" : "backward");
      for (std::size_t step = 0; step < requests.size(); ++step) {
        const SearchProgress progress{.report = std::cref(report),
                                      .task = name,
                                      .done = step,
                                      .total = requests.size()};
        progress.update();
        const auto index = forward ? step : requests.size() - 1 - step;
        reroute(corridors, requests, routes, index, forward, sweeps, progress);
      }
      report.progress(name, "", requests.size(), requests.size());
      ++rounds;
      const auto left = unrouted(routes);
      report.entry(
          Detail::Summary, name,
          {routedFigure(left, requests.size()), verdict("fails", left)});
      if (left <= bestLeft) {
        best = routes;
        bestLeft = left;
        bestRound = rounds;
      }
      // Requests without both endpoint partitions cannot improve in later
      // sweeps. Keep their failures in the result.
      if (left == missingEndpoints) {
        break;
      }
    }
    if (unrouted(routes) > bestLeft) {
      restore(corridors, requests, routes, best);
      report.entry(Detail::Summary, std::format("back to round {}", bestRound),
                   {routedFigure(bestLeft, requests.size()),
                    verdict("fails", bestLeft)});
    }

    // A wire that a sweep ripped up after it had passed the wire can find
    // free slots in the plan that the stage keeps, for example where the wire
    // that took its way moved on later in the round. One last pass offers
    // every wire without a way the free slots only. It rips up nothing, so it
    // can only lower the fails.
    std::size_t placed = 0;
    for (std::size_t index = 0; index < requests.size(); ++index) {
      const SearchProgress progress{.report = std::cref(report),
                                    .task = "free slots",
                                    .done = index,
                                    .total = requests.size()};
      progress.update();
      if (routes[index].has_value()) {
        continue;
      }
      if (auto found = corridors.route(requests[index], progress);
          found.has_value()) {
        corridors.commit(requests[index], *found);
        routes[index] = std::move(found);
        ++placed;
      }
    }
    report.progress("free slots", "", requests.size(), requests.size());
    if (placed > 0) {
      const auto after = unrouted(routes);
      report.entry(
          Detail::Summary, "free slots",
          {routedFigure(after, requests.size()), verdict("fails", after)});
    }

    // A corridor has no clearance to break, so the fails of the stage are the
    // connections without a way.
    const auto left = unrouted(routes);

    if (report.wants(Detail::Items)) {
      for (std::size_t index = 0; index < requests.size(); ++index) {
        const auto& label =
            chip.ports[assignment.connections[index]->target.index()]->label;
        if (!routes[index].has_value()) {
          report.entry(Detail::Items, label,
                       {{.text = "unrouted", .tone = Tone::Bad}});
          continue;
        }
        report.entry(
            Detail::Items, label,
            {{.text = counted(routes[index]->partitions.size(), "partition")},
             {.text = counted(routes[index]->slots.size(), "crossing")}});
      }
    }
    report.result({routedFigure(left, requests.size()),
                   {.text = counted(rounds, "round")}},
                  left);
    return artifactOf(corridors, capacity, requests, routes, scene.detail);
  }

private:
  /// How far from its own cell a feed or a target may look for a partition.
  static constexpr std::uint32_t LABEL_REACH = 8;

  /// Puts the plan back exactly as it was.
  static void restore(Corridors& corridors,
                      const std::vector<Request>& requests,
                      std::vector<std::optional<Route>>& routes,
                      const std::vector<std::optional<Route>>& before) {
    for (std::size_t index = 0; index < routes.size(); ++index) {
      if (routes[index].has_value()) {
        corridors.withdraw(requests[index], *routes[index]);
        routes[index].reset();
      }
    }
    for (std::size_t index = 0; index < before.size(); ++index) {
      if (before[index].has_value()) {
        corridors.commit(requests[index], *before[index]);
        routes[index] = before[index];
      }
    }
  }

  /// One request per connection of the assignment, in its order.
  [[nodiscard]] static std::vector<Request>
  requestsOf(const CapacityScene& scene, const AssignmentT& assignment,
             const std::vector<PartitionLabel>& labels) {
    std::unordered_map<std::uint32_t, std::size_t> cellOfPort;
    for (std::size_t index = 0; index < scene.targetPort.size(); ++index) {
      cellOfPort.emplace(scene.targetPort[index], scene.targetCell[index]);
    }

    std::vector<Request> requests;
    requests.reserve(assignment.connections.size());
    for (std::size_t index = 0; index < assignment.connections.size();
         ++index) {
      Request request;
      request.connection = index;
      const auto target =
          cellOfPort.find(assignment.connections[index]->target.index());
      if (target != cellOfPort.end()) {
        request.target = placeOf(scene.detail, target->second);
        request.targetPartition =
            partitionAt(labels, scene.detail, target->second, LABEL_REACH);
      }
      if (index < assignment.feeds.size()) {
        if (const auto feed = scene.detail.roundToCell(assignment.feeds[index]);
            feed.has_value()) {
          const auto cell = scene.detail.index(feed->x(), feed->y());
          request.feed = placeOf(scene.detail, cell);
          request.feedPartition =
              partitionAt(labels, scene.detail, cell, LABEL_REACH);
        }
      }
      requests.push_back(request);
    }
    return requests;
  }

  /// Takes one wire out, routes it again, and rips up its neighbors when
  /// that is what it takes.
  ///
  /// A wire that fails puts every neighbor it ripped back as it was, so a
  /// failed attempt costs the plan nothing, and it keeps the way it had.
  static void reroute(Corridors& corridors,
                      const std::vector<Request>& requests,
                      std::vector<std::optional<Route>>& routes,
                      const std::size_t index, const bool forward,
                      const Sweeps& sweeps, const SearchProgress& progress) {
    const auto& request = requests[index];
    if (!request.hasEndpoints()) {
      return;
    }
    const auto before = routes[index];
    if (routes[index].has_value()) {
      corridors.withdraw(request, *routes[index]);
      routes[index].reset();
    }
    if (auto found = corridors.route(request, progress); found.has_value()) {
      corridors.commit(request, *found);
      routes[index] = std::move(found);
      return;
    }

    // Nothing free reaches the target. The wires ahead of this one in the
    // sweep are taken out one after another until it fits. The sweep has not
    // settled them yet, so it gives each of them a new way before the round
    // is over. Near the end of the sweep the count runs on past the last wire
    // to the first ones, which this round has passed; those wait for the
    // next round or the free-slot pass. The relaxation stays below the number
    // of connections, so the count never comes back to this wire.
    std::vector<std::pair<std::size_t, Route>> ripped;
    const auto total = static_cast<std::int64_t>(requests.size());
    for (std::uint32_t relax = 1; relax <= sweeps.maxRelaxation; ++relax) {
      const auto offset = static_cast<std::int64_t>(relax) * (forward ? 1 : -1);
      const auto neighbor = static_cast<std::size_t>(
          (((static_cast<std::int64_t>(index) + offset) % total) + total) %
          total);
      if (!routes[neighbor].has_value()) {
        continue;
      }
      corridors.withdraw(requests[neighbor], *routes[neighbor]);
      ripped.emplace_back(neighbor, std::move(*routes[neighbor]));
      routes[neighbor].reset();
      if (auto found = corridors.route(request, progress); found.has_value()) {
        corridors.commit(request, *found);
        routes[index] = std::move(found);
        return;
      }
    }

    for (const auto& [other, route] : std::ranges::reverse_view(ripped)) {
      corridors.commit(requests[other], route);
      routes[other] = route;
    }

    // The sweep takes out whoever stands beside this wire on the ring, which
    // is rarely who stands in its way. The lowest-cost route with
    // displacement allowed identifies blockers by their slots and chords.
    // The blockers come out one at a time. The sweep gives a
    // way of its own to each of them that it has not passed yet; one that it
    // has passed waits for the next round or the free-slot pass.
    std::vector<std::pair<std::size_t, Route>> displaced;
    std::optional<Route> wanted;
    for (std::uint32_t attempt = 0; attempt < sweeps.maxRelaxation; ++attempt) {
      // The way is asked again after every wire that comes out, because what
      // stands in the way changes as they go.
      wanted =
          corridors.route(request, progress, corridors.displacementPrice());
      if (!wanted.has_value()) {
        break;
      }
      std::optional<std::size_t> blocker;
      for (const auto other : corridors.blockersOf(request, *wanted)) {
        if (routes[other].has_value()) {
          blocker = other;
          break;
        }
      }
      // A way that no routed wire blocks is free, so the free search above
      // finds it too unless it gave up on unsound ways first. The wire then
      // keeps the way it had, and a later round or the free-slot pass tries
      // again.
      if (!blocker.has_value()) {
        break;
      }
      corridors.withdraw(requests[*blocker], *routes[*blocker]);
      displaced.emplace_back(*blocker, *routes[*blocker]);
      routes[*blocker].reset();
      if (auto found = corridors.route(request, progress); found.has_value()) {
        corridors.commit(request, *found);
        routes[index] = std::move(found);
        return;
      }
    }
    for (const auto& [other, route] : std::ranges::reverse_view(displaced)) {
      corridors.commit(requests[other], route);
      routes[other] = route;
    }
    if (wanted.has_value()) {
      corridors.chargeFor(request, *wanted);
    }

    // Nothing worked, so the wire gets back the way it had. Every wire that
    // came out above went back to its own way, so the plan is the plan at
    // the top of the attempt without this wire, and the old way fits again.
    if (before.has_value()) {
      corridors.commit(request, *before);
      routes[index] = before;
    }
  }

  [[nodiscard]] static CorridorRoutingT
  artifactOf(const Corridors& corridors, const CapacityPlanT& capacity,
             const std::vector<Request>& requests,
             const std::vector<std::optional<Route>>& routes,
             const grid::GridMetrics& detail) {
    const auto layoutOf = [&detail](const Place& place) {
      return std::make_unique<fbg::Point>(
          grid::cornerToLayout(detail, place.x, place.y));
    };

    CorridorRoutingT routing;
    routing.corridors.reserve(routes.size());
    for (std::size_t index = 0; index < routes.size(); ++index) {
      auto corridor = std::make_unique<fba::CorridorT>();
      if (requests[index].feedPartition != LABEL_NONE) {
        corridor->source = layoutOf(requests[index].feed);
      }
      if (requests[index].targetPartition != LABEL_NONE) {
        corridor->target = layoutOf(requests[index].target);
      }
      if (routes[index].has_value()) {
        for (const auto partition : routes[index]->partitions) {
          corridor->partitions.push_back(partition);
        }
        for (const auto slot : routes[index]->slots) {
          corridor->crossings.push_back(corridors.layoutOf(slot));
        }
      }
      routing.corridors.push_back(std::move(corridor));
    }

    std::vector<std::unique_ptr<fba::BorderSlotsT>> perBorder(
        capacity.borders.size());
    for (std::size_t id = 0; id < corridors.slots().size(); ++id) {
      const auto border = corridors.slots()[id].border;
      if (perBorder[border] == nullptr) {
        perBorder[border] = std::make_unique<fba::BorderSlotsT>();
        perBorder[border]->border = border;
      }
      perBorder[border]->positions.push_back(corridors.layoutOf(id));
    }
    for (auto& slots : perBorder) {
      if (slots != nullptr) {
        routing.slots.push_back(std::move(slots));
      }
    }
    return routing;
  }
};

} // namespace

std::unique_ptr<ICorridorRouter> makePartitionAStarRouter() {
  return std::make_unique<PartitionAStarRouter>();
}

} // namespace mqt::scpd::pipeline
