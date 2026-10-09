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
#include "mqt-scpd/grid/PortBands.hpp"
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
/// the centre of a cell both land on a half.
struct Place {
  double x = 0.0;
  double y = 0.0;
};

/// The border of a slot that lies on the approach of a port and on no border
/// of the plan.
constexpr std::uint32_t NO_BORDER = std::numeric_limits<std::uint32_t>::max();

/// One crossing slot: where it is, and the two partitions it joins.
struct Slot {
  Place place;
  PartitionLabel first = LABEL_NONE;
  PartitionLabel second = LABEL_NONE;
  /// The border the slot is on, as an index into the borders of the plan, or
  /// NO_BORDER for the way out of the pocket of a port.
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
  /// Whether the assignment gave this wire a point to be fed at. A wire
  /// without one is not routed, and its feed says nothing about where the
  /// ring runs.
  bool fed = false;
};

/// The rectangle the ports feed from.
///
/// Every wire starts at a point on the launcher ring, so the box that those
/// points span is the outside of the chip as far as this stage is concerned.
struct Bounds {
  double minX = 0.0;
  double minY = 0.0;
  double maxX = 0.0;
  double maxY = 0.0;

  /// Whether a place lies inside the ring, and not on it.
  ///
  /// A place on the rectangle sits beside the feed point of some port, and a
  /// wire that crosses there has slipped behind that port.
  [[nodiscard]] bool hold(const Place& place) const {
    return place.x > minX && place.x < maxX && place.y > minY && place.y < maxY;
  }
};

/// The slots a wire crosses and the partitions it runs through. There is
/// always one more partition than there are slots.
struct Route {
  std::vector<std::size_t> slots;
  std::vector<PartitionLabel> partitions;
};

/// A chord already drawn across a partition, and the wire that drew it.
///
/// `fromPin` and `toPin` say whether that end is a place the wire is pinned
/// to, which is the point it is fed at or the cell of its target port, and
/// not a crossing slot that the search may move it off.
struct Chord {
  Place from;
  Place to;
  std::size_t owner = 0;
  bool fromPin = false;
  bool toPin = false;
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

/// Whether two chords cross each other properly.
[[nodiscard]] bool crosses(const Place& a, const Place& b, const Place& c,
                           const Place& d) {
  return (sideOf(a, b, c) * sideOf(a, b, d)) < 0 &&
         (sideOf(c, d, a) * sideOf(c, d, b)) < 0;
}

/// Whether two chords lie along each other over a length rather than meeting
/// at a point.
///
/// Two wires that share a stretch of one corridor are one piece of copper:
/// the later stage has no room for the second one.
[[nodiscard]] bool overlaps(const Place& a, const Place& b, const Place& c,
                            const Place& d) {
  if (sideOf(a, b, c) != 0 || sideOf(a, b, d) != 0) {
    return false;
  }
  // The two run along one line, so they share a stretch when their spans
  // overlap in more than an end point. The wider axis decides: a vertical
  // pair is compared on y and every other pair on x.
  const auto vertical = std::abs(b.x - a.x) < std::abs(b.y - a.y);
  const auto first = vertical
                         ? std::pair{std::min(a.y, b.y), std::max(a.y, b.y)}
                         : std::pair{std::min(a.x, b.x), std::max(a.x, b.x)};
  const auto second = vertical
                          ? std::pair{std::min(c.y, d.y), std::max(c.y, d.y)}
                          : std::pair{std::min(c.x, d.x), std::max(c.x, d.x)};
  return std::min(first.second, second.second) >
         std::max(first.first, second.first);
}

/// Whether two places are one place.
[[nodiscard]] bool same(const Place& a, const Place& b) {
  return a.x == b.x && a.y == b.y;
}

/// Whether the point `c` lies on the chord from `a` to `b`, between its ends.
///
/// The three points are known to lie on one line, so what is left is whether
/// `c` falls between the two along it.
[[nodiscard]] bool between(const Place& a, const Place& b, const Place& c) {
  return std::min(a.x, b.x) <= c.x && c.x <= std::max(a.x, b.x) &&
         std::min(a.y, b.y) <= c.y && c.y <= std::max(a.y, b.y);
}

/// Whether a wire is pinned to a point that another wire runs over.
///
/// A wire is pinned at two places: the point it is fed at and the cell of its
/// target port. Neither can move, so a second wire drawn over one is two
/// wires on one point of the chip, however much room the corridor around them
/// has. The crossing test does not see it: a chord that stops on another one
/// never gets to its far side, so the two never cross properly, and a wire
/// could run along a whole line of feed points without one crossing.
///
/// A crossing slot is a different matter. It is where the plan says a wire
/// leaves one partition for the next, and the later stage may cross a little
/// to either side of it, so a chord that grazes one still leaves room.
[[nodiscard]] bool touches(const Chord& a, const Chord& b) {
  const auto over = [](const Chord& chord, const Place& pin) {
    return sideOf(chord.from, chord.to, pin) == 0 &&
           between(chord.from, chord.to, pin) && !same(chord.from, pin) &&
           !same(chord.to, pin);
  };
  const auto pinned = [&over](const Chord& chord, const Chord& other) {
    return (other.fromPin && over(chord, other.from)) ||
           (other.toPin && over(chord, other.to));
  };
  return pinned(a, b) || pinned(b, a);
}

/// Whether two chords may not both be drawn.
///
/// A proper crossing is what the plan has to rule out, so that the later
/// stage can draw it. Two chords that run along each other are ruled out as
/// well, because they put two wires in one place, and so is a chord drawn
/// over a point another wire is pinned to.
[[nodiscard]] bool meets(const Chord& a, const Chord& b) {
  return crosses(a.from, a.to, b.from, b.to) ||
         overlaps(a.from, a.to, b.from, b.to) || touches(a, b);
}

/// The partition graph of one run, with the wires committed to it.
class Corridors {
public:
  Corridors(const CapacityPlanT& plan, const grid::GridMetrics& detail,
            const double wireSpacing, const Bounds& ring)
      : detailGrid(detail) {
    for (std::uint32_t index = 0; index < plan.borders.size(); ++index) {
      const auto& border = *plan.borders[index];
      grid::PartitionBorder geometry{
          .first = static_cast<PartitionLabel>(border.first),
          .second = static_cast<PartitionLabel>(border.second),
          .samples = {}};
      geometry.samples.reserve(border.samples.size());
      for (const auto& sample : border.samples) {
        const auto cell = detail.toCell(sample);
        geometry.samples.emplace_back(onGrid(cell.x()), onGrid(cell.y()));
      }
      for (const auto& slot :
           grid::borderSlots(geometry, detail, wireSpacing)) {
        // Outside the ring the ports feed from there is nothing to route: a
        // wire that crosses there leaves through the ring, runs along the
        // back of it and comes in again behind another port.
        if (!ring.hold({.x = slot.x(), .y = slot.y()})) {
          continue;
        }
        slotsByPartition[geometry.first].push_back(slotList.size());
        slotsByPartition[geometry.second].push_back(slotList.size());
        slotList.push_back({.place = {.x = slot.x(), .y = slot.y()},
                            .first = geometry.first,
                            .second = geometry.second,
                            .border = index});
      }
    }
    // Where three partitions meet, one point is a sample of more than one
    // border and so a slot of each. They are one place on the chip, so one
    // wire crosses there and no more; the slots of a place share what records
    // that.
    std::map<std::pair<double, double>, std::size_t> places;
    placeOfSlot.reserve(slotList.size());
    for (const auto& slot : slotList) {
      const auto [entry, added] =
          places.try_emplace({slot.place.x, slot.place.y}, places.size());
      placeOfSlot.push_back(entry->second);
    }
    placeTaken.assign(places.size(), false);
    placeOwner.assign(places.size(), NO_OWNER);
    placeHistory.assign(places.size(), 0.0);
  }

  [[nodiscard]] const std::vector<Slot>& slots() const { return slotList; }

  /// The slots of a partition, empty for a pocket that no border reaches.
  [[nodiscard]] std::span<const std::size_t>
  slotsOfPartition(const PartitionLabel partition) const {
    return slotsOf(partition);
  }

  /// Adds the way out of the pocket of a port.
  ///
  /// The capacity grid places a target on the first cell beyond the band of
  /// its port that was free before the bands were stamped. It does not open
  /// the band itself, because nothing searches on that grid. Where the band
  /// and the artwork close around the port, the free space left is a pocket
  /// that no border reaches, and a wire could never leave it.
  ///
  /// The band is not artwork. It is the strip that the wire of the port
  /// leaves along, so it is a way out, and it carries one wire: the wire of
  /// that port. This adds it as a slot of its own, on no border of the plan.
  void addChannel(const Place& place, const PartitionLabel from,
                  const PartitionLabel to) {
    slotsByPartition[from].push_back(slotList.size());
    slotsByPartition[to].push_back(slotList.size());
    slotList.push_back(
        {.place = place, .first = from, .second = to, .border = NO_BORDER});
    placeOfSlot.push_back(placeTaken.size());
    placeTaken.push_back(false);
    placeOwner.push_back(NO_OWNER);
    placeHistory.push_back(0.0);
  }

  /// What displacing one wire costs the search: more than the longest route
  /// the chip can hold, so length never buys a displacement.
  [[nodiscard]] double displacementPrice() const {
    const auto box = detailGrid.box();
    return std::hypot(box.maxX - box.minX, box.maxY - box.minY);
  }

  /// Searches the way of one wire from its feed to its target, and searches
  /// again when the cheapest way is one the wire may not take.
  ///
  /// With no `displacement` the search uses only what is free. Given one, it
  /// may take a slot that another wire holds or cross a chord that another
  /// wire drew, and pays that price per wire it displaces. Set well above the
  /// length of any route, the price makes the search take a free way wherever
  /// one exists and displace as few wires as it can where none does.
  ///
  /// A route that crosses itself or comes back to a place it already used is
  /// a short. That is a property of the whole route and not of one step, so
  /// the search cannot rule it out as it goes. The slots of such a way are
  /// set aside and the search runs again, so that a second-best way is found
  /// where one exists.
  [[nodiscard]] std::optional<Route>
  route(const Request& request, const double displacement = 0.0) const {
    std::vector<bool> aside(slotList.size(), false);
    for (std::uint32_t attempt = 0; attempt <= UNSOUND_RETRIES; ++attempt) {
      auto found = search(request, displacement, aside);
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
      placeTaken[placeOfSlot[slot]] = true;
      placeOwner[placeOfSlot[slot]] = request.connection;
    }
    walk(request, route,
         [&](const Chord& chord, const PartitionLabel partition) {
           chordsByPartition[partition].push_back(chord);
         });
  }

  /// Takes a route back out.
  void withdraw(const Request& request, const Route& route) {
    for (const auto slot : route.slots) {
      placeTaken[placeOfSlot[slot]] = false;
      placeOwner[placeOfSlot[slot]] = NO_OWNER;
    }
    for (const auto partition : route.partitions) {
      auto& chords = chordsByPartition[partition];
      std::erase_if(chords, [&](const Chord& chord) {
        return chord.owner == request.connection;
      });
    }
  }

  /// The wires in the way of a route drawn as though the chip were empty:
  /// the ones that hold a slot it wants, and the ones whose chords it would
  /// have to cross.
  [[nodiscard]] std::vector<std::size_t> blockersOf(const Request& request,
                                                    const Route& wanted) const {
    std::vector<std::size_t> owners;
    for (const auto slot : wanted.slots) {
      if (placeOwner[placeOfSlot[slot]] != NO_OWNER) {
        owners.push_back(placeOwner[placeOfSlot[slot]]);
      }
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

  /// Makes the slots a wire wanted, and could not have, dearer for whoever
  /// stands on them.
  ///
  /// Without this two wires trade one slot back and forth: each displaces
  /// the other, each round undoes the last, and neither settles. The price
  /// only rises, so the wire in the way finds its own way cheaper elsewhere
  /// in the end and gives the slot up. The charge is a fraction of one
  /// displacement, so a few rounds of wanting a slot add up to more than the
  /// detour around it.
  void chargeFor(const Route& route) {
    const auto amount = displacementPrice() / HISTORY_STEPS;
    for (const auto slot : route.slots) {
      if (placeTaken[placeOfSlot[slot]]) {
        placeHistory[placeOfSlot[slot]] += amount;
      }
    }
  }

  /// Whether a route can be laid down as it stands: every slot still free,
  /// and every chord clear of the ones already drawn.
  [[nodiscard]] bool fits(const Request& request, const Route& route) const {
    if (std::ranges::any_of(route.slots, [&](const std::size_t slot) {
          return placeTaken[placeOfSlot[slot]];
        })) {
      return false;
    }
    auto clearAll = true;
    walk(request, route,
         [&](const Chord& chord, const PartitionLabel partition) {
           clearAll = clearAll && clear(chord, partition);
         });
    return clearAll;
  }

  /// The layout position of a slot, for the artifact.
  [[nodiscard]] fbg::Point layoutOf(const std::size_t slot) const {
    return detailGrid.toLayout(slotList[slot].place.x, slotList[slot].place.y);
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

  [[nodiscard]] std::optional<Route>
  search(const Request& request, const double displacement,
         const std::vector<bool>& aside) const {
    if (request.feedPartition == LABEL_NONE ||
        request.targetPartition == LABEL_NONE) {
      return std::nullopt;
    }

    // A state is a slot together with the partition the wire carries on
    // into, because a slot joins two partitions and the one a wire came from
    // decides where it may go next.
    std::vector<double> best(slotList.size() * 2, UNREACHED);
    std::vector<std::size_t> from(slotList.size() * 2, NO_STATE);
    std::vector<bool> closed(slotList.size() * 2, false);

    using Entry = std::pair<double, std::size_t>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<>> queue;

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
        if (aside[slot] || slotList[slot].beyond(partition) == behind) {
          continue;
        }
        const auto blocked = placeTaken[placeOfSlot[slot]] ||
                             !clear(Chord{.from = place,
                                          .to = slotList[slot].place,
                                          .owner = request.connection,
                                          .fromPin = parent == NO_STATE,
                                          .toPin = false},
                                    partition);
        if (blocked && displacement <= 0.0) {
          continue;
        }
        const auto state = stateOf(slot, slotList[slot].beyond(partition));
        const auto step = cost + distance(place, slotList[slot].place) +
                          placeHistory[placeOfSlot[slot]] +
                          (blocked ? displacement : 0.0);
        if (step < best[state]) {
          best[state] = step;
          from[state] = parent;
          queue.emplace(step + distance(slotList[slot].place, request.target),
                        state);
        }
      }
    };

    // Reaching the target inside the partition the wire is fed in is a whole
    // route, and it needs no slot.
    auto shortest = UNREACHED;
    auto arrival = NO_STATE;
    if (request.feedPartition == request.targetPartition) {
      const auto direct = clear(Chord{.from = request.feed,
                                      .to = request.target,
                                      .owner = request.connection,
                                      .fromPin = true,
                                      .toPin = true},
                                request.feedPartition);
      if (direct || displacement > 0.0) {
        shortest = distance(request.feed, request.target) +
                   (direct ? 0.0 : displacement);
      }
    }

    expand(request.feed, request.feedPartition, 0.0, NO_STATE);
    while (!queue.empty()) {
      const auto [estimate, state] = queue.top();
      queue.pop();
      if (closed[state] || estimate >= shortest) {
        continue;
      }
      closed[state] = true;

      const auto& slot = slotList[state / 2];
      const auto partition = partitionOf(state);
      if (partition == request.targetPartition) {
        const auto reaches = clear(Chord{.from = slot.place,
                                         .to = request.target,
                                         .owner = request.connection,
                                         .fromPin = false,
                                         .toPin = true},
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
    const auto from = detailGrid.toLayout(a.x, a.y);
    const auto to = detailGrid.toLayout(b.x, b.y);
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
  /// to a place it already used. Either is a short, and the route is thrown
  /// away.
  [[nodiscard]] bool sound(const Request& request, const Route& route) const {
    std::vector<std::size_t> places;
    places.reserve(route.slots.size());
    for (const auto slot : route.slots) {
      places.push_back(placeOfSlot[slot]);
    }
    std::ranges::sort(places);
    if (std::ranges::adjacent_find(places) != places.end()) {
      return false;
    }
    return !selfCrossing(request, route);
  }

  /// Whether a wire lays a chord over one of its own.
  ///
  /// Two chords that follow one another share the slot between them, which
  /// is a corner and not a short; any other meeting is one.
  [[nodiscard]] bool selfCrossing(const Request& request,
                                  const Route& route) const {
    std::unordered_map<PartitionLabel,
                       std::vector<std::pair<std::size_t, Chord>>>
        own;
    std::size_t step = 0;
    auto found = false;
    walk(
        request, route, [&](const Chord& mine, const PartitionLabel partition) {
          auto& chords = own[partition];
          found = found || std::ranges::any_of(chords, [&](const auto& drawn) {
                    return drawn.first + 1 != step && meets(mine, drawn.second);
                  });
          chords.emplace_back(step, mine);
          ++step;
        });
    return found;
  }

  /// Hands every chord of a route to `visit`, in order. A route of k slots
  /// has k + 1 chords, one per partition it runs through.
  template <typename Visitor>
  void walk(const Request& request, const Route& route,
            const Visitor& visit) const {
    auto place = request.feed;
    auto pin = true;
    for (std::size_t index = 0; index < route.slots.size(); ++index) {
      const auto& next = slotList[route.slots[index]].place;
      visit(Chord{.from = place,
                  .to = next,
                  .owner = request.connection,
                  .fromPin = pin,
                  .toPin = false},
            route.partitions[index]);
      place = next;
      pin = false;
    }
    visit(Chord{.from = place,
                .to = request.target,
                .owner = request.connection,
                .fromPin = pin,
                .toPin = true},
          route.partitions.back());
  }

  grid::GridMetrics detailGrid;
  std::vector<Slot> slotList;
  std::unordered_map<PartitionLabel, std::vector<std::size_t>> slotsByPartition;
  std::unordered_map<PartitionLabel, std::vector<Chord>> chordsByPartition;
  /// The place each slot sits at, as an index shared by every slot on it.
  std::vector<std::size_t> placeOfSlot;
  std::vector<bool> placeTaken;
  std::vector<std::size_t> placeOwner;
  std::vector<double> placeHistory;
};

/// The tuning the stage reads.
struct Sweeps {
  std::uint32_t rounds = 12;
  std::uint32_t maxRelaxation = 30;
};

[[nodiscard]] Sweeps sweepsOf(const ConfigT& config) {
  if (config.stages != nullptr && config.stages->corridor != nullptr) {
    const auto& params = *config.stages->corridor;
    return {.rounds = params.rounds, .maxRelaxation = params.max_relaxation};
  }
  return {};
}

/// The labels of the partitions of the plan, back on the detail grid.
[[nodiscard]] std::vector<PartitionLabel>
labelsOf(const CapacityPlanT& plan, const grid::GridMetrics& detail) {
  std::vector<grid::PartitionOutline> outlines;
  for (const auto& partition : plan.partitions) {
    for (const auto& ring : partition->outlines) {
      grid::PartitionOutline outline{
          .label = static_cast<PartitionLabel>(partition->label), .ring = {}};
      outline.ring.reserve(ring->vertices.size());
      for (const auto& vertex : ring->vertices) {
        outline.ring.push_back(detail.toCell(vertex));
      }
      outlines.push_back(std::move(outline));
    }
  }
  return grid::rasterizePartitions(outlines, detail);
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
  const auto centre = grid.cell(cell);
  for (std::uint32_t radius = 1; radius <= reach; ++radius) {
    const auto span = static_cast<std::int64_t>(radius);
    for (std::int64_t dy = -span; dy <= span; ++dy) {
      for (std::int64_t dx = -span; dx <= span; ++dx) {
        if (std::max(std::abs(dx), std::abs(dy)) != span) {
          continue;
        }
        const auto x = static_cast<std::int64_t>(centre.x()) + dx;
        const auto y = static_cast<std::int64_t>(centre.y()) + dy;
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

/// The centre of a cell, in the corner coordinates that the partition
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
    report.progress("rasterizing the chip", "", 0, 0);
    const auto scene = buildScene(chip, config);
    report.progress("rasterizing the partitions", "", 0, 0);
    const auto labels = labelsOf(capacity, scene.detail);

    const auto requests = requestsOf(scene, assignment, labels);

    Corridors corridors(capacity, scene.detail, crossingPitch(config),
                        ringOf(requests));
    openPockets(corridors, chip, scene, labels);

    const auto sweeps = sweepsOf(config);
    if (report.wants(Detail::Steps)) {
      report.line(
          Detail::Steps,
          std::format("{}{}{}{}{}", counted(requests.size(), "connection"),
                      FIGURE_SEPARATOR,
                      counted(capacity.partitions.size(), "partition"),
                      FIGURE_SEPARATOR,
                      counted(corridors.slots().size(), "crossing slot")));
      report.line(Detail::Steps,
                  std::format("rounds {}{}max relaxation {}", sweeps.rounds,
                              FIGURE_SEPARATOR, sweeps.maxRelaxation));
    }

    std::vector<std::optional<Route>> routes(requests.size());
    std::uint32_t rounds = 0;
    for (std::uint32_t round = 0; round < sweeps.rounds; ++round) {
      const auto forward = (round % 2) == 0;
      const auto name = std::format("round {} {}", round + 1,
                                    forward ? "forward" : "backward");
      for (std::size_t step = 0; step < requests.size(); ++step) {
        report.progress(name, "", step, requests.size());
        const auto index = forward ? step : requests.size() - 1 - step;
        reroute(corridors, requests, routes, index, forward, sweeps);
      }
      ++rounds;
      const auto left = unrouted(routes);
      report.entry(
          Detail::Summary, name,
          {routedFigure(left, requests.size()), verdict("fails", left)});
      // Only a round that routes everything ends the sweeps. A round in which
      // nothing moved is not the end: a wire that found no way charged the
      // slots it wanted, and it takes several such rounds before the charge
      // is high enough to move the wire that stands on one.
      if (left == 0) {
        break;
      }
    }

    // A wire that the sweep ripped on its way past is left for the sweep to
    // place again, and the last round has no sweep after it. Every wire still
    // without a way is therefore offered the room the others left, and only
    // that: this pass takes nothing from anyone, so it can be repeated until
    // it places nothing more.
    report.progress("placing what is left", "", 0, 0);
    const auto beforeRescue = unrouted(routes);
    for (auto placed = true; placed;) {
      placed = false;
      for (std::size_t index = 0; index < requests.size(); ++index) {
        if (routes[index].has_value()) {
          continue;
        }
        if (auto found = corridors.route(requests[index]); found.has_value()) {
          corridors.commit(requests[index], *found);
          routes[index] = std::move(found);
          placed = true;
        }
      }
    }
    const auto afterRescue = unrouted(routes);
    if (afterRescue != beforeRescue) {
      report.entry(
          Detail::Summary, "rescue",
          {{.text = std::format("placed {}", beforeRescue - afterRescue)},
           verdict("fails", afterRescue)});
    }

    report.progress("exchanging wires", "", 0, 0);
    exchange(corridors, requests, routes, sweeps);
    // A corridor has no clearance to break, so the fails of the stage are the
    // connections without a way.
    const auto left = unrouted(routes);
    if (left != afterRescue) {
      report.entry(Detail::Summary, "exchange",
                   {{.text = std::format("placed {}", afterRescue - left)},
                    verdict("fails", left)});
    }

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

  /// How far along its own approach a target may look for a way out of the
  /// pocket that the band of its port closed around it, in cells of the
  /// detail grid.
  static constexpr std::uint32_t POCKET_REACH = 64;

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

  /// Offers every wire still without a way a swap with the wires in its way,
  /// and keeps the swap only when fewer wires are left without one.
  static void exchange(Corridors& corridors,
                       const std::vector<Request>& requests,
                       std::vector<std::optional<Route>>& routes,
                       const Sweeps& sweeps) {
    for (std::size_t index = 0; index < requests.size(); ++index) {
      if (routes[index].has_value()) {
        continue;
      }
      const auto wanted =
          corridors.route(requests[index], corridors.displacementPrice());
      if (!wanted.has_value()) {
        continue;
      }
      const auto before = routes;
      for (const auto blocker :
           corridors.blockersOf(requests[index], *wanted)) {
        if (routes[blocker].has_value()) {
          corridors.withdraw(requests[blocker], *routes[blocker]);
          routes[blocker].reset();
        }
      }
      if (auto found = corridors.route(requests[index]); found.has_value()) {
        corridors.commit(requests[index], *found);
        routes[index] = std::move(found);
      }
      for (auto left = unrouted(routes) + 1; unrouted(routes) < left;) {
        left = unrouted(routes);
        for (std::size_t other = 0; other < requests.size(); ++other) {
          if (!routes[other].has_value()) {
            reroute(corridors, requests, routes, other, true, sweeps);
          }
        }
      }
      if (unrouted(routes) >= unrouted(before)) {
        restore(corridors, requests, routes, before);
      }
    }
  }

  /// Gives every target that no border reaches a way out along the approach
  /// of its own port.
  ///
  /// The walk goes forward from the target the way the port faces, over the
  /// cells the band took, until it reaches a cell that another partition
  /// holds. That cell is the far side, and the last cell of the band before
  /// it is where the wire crosses. A target that the walk does not get out of
  /// stays unreachable, which is then a matter of geometry.
  static void openPockets(Corridors& corridors, const ChipT& chip,
                          const CapacityScene& scene,
                          const std::vector<PartitionLabel>& labels) {
    for (std::size_t index = 0; index < scene.targetCell.size(); ++index) {
      const auto inside = labels[scene.targetCell[index]];
      if (inside == LABEL_NONE || !corridors.slotsOfPartition(inside).empty()) {
        continue;
      }
      const auto& port = *chip.ports[scene.targetPort[index]];
      const auto step = grid::orientationStep(port.orientation);
      const auto cell = scene.detail.cell(scene.targetCell[index]);
      auto x = static_cast<std::int64_t>(cell.x());
      auto y = static_cast<std::int64_t>(cell.y());
      for (std::uint32_t reach = 0; reach < POCKET_REACH; ++reach) {
        x += step.x;
        y += step.y;
        if (!scene.detail.contains(x, y)) {
          break;
        }
        const auto at = scene.detail.index(static_cast<std::uint32_t>(x),
                                           static_cast<std::uint32_t>(y));
        if (const auto beyond = labels[at];
            beyond != LABEL_NONE && beyond != inside) {
          // The wire crosses on the last cell of the band, one step back.
          corridors.addChannel({.x = static_cast<double>(x - step.x) + 0.5,
                                .y = static_cast<double>(y - step.y) + 0.5},
                               inside, beyond);
          break;
        }
      }
    }
  }

  /// The rectangle the ports feed from, in cells of the detail grid.
  ///
  /// A wire without a feed point says nothing about where the ring runs, and
  /// a run in which no wire has one has nothing to route; the ring is then
  /// the whole plane, which changes nothing.
  [[nodiscard]] static Bounds ringOf(const std::vector<Request>& requests) {
    auto found = false;
    Bounds ring;
    for (const auto& request : requests) {
      if (!request.fed) {
        continue;
      }
      if (!found) {
        ring = {.minX = request.feed.x,
                .minY = request.feed.y,
                .maxX = request.feed.x,
                .maxY = request.feed.y};
        found = true;
        continue;
      }
      ring.minX = std::min(ring.minX, request.feed.x);
      ring.minY = std::min(ring.minY, request.feed.y);
      ring.maxX = std::max(ring.maxX, request.feed.x);
      ring.maxY = std::max(ring.maxY, request.feed.y);
    }
    if (!found) {
      const auto limit = std::numeric_limits<double>::max();
      return {.minX = -limit, .minY = -limit, .maxX = limit, .maxY = limit};
    }
    return ring;
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
      if (target == cellOfPort.end()) {
        requests.push_back(request);
        continue;
      }
      request.target = placeOf(scene.detail, target->second);
      request.targetPartition =
          partitionAt(labels, scene.detail, target->second, LABEL_REACH);

      if (index < assignment.feeds.size()) {
        if (const auto feed = scene.detail.roundToCell(assignment.feeds[index]);
            feed.has_value()) {
          const auto cell = scene.detail.index(feed->x(), feed->y());
          request.feed = placeOf(scene.detail, cell);
          request.feedPartition =
              partitionAt(labels, scene.detail, cell, LABEL_REACH);
          request.fed = true;
        }
      }
      requests.push_back(request);
    }
    return requests;
  }

  /// Takes one wire out, routes it again, and rips up its neighbours when
  /// that is what it takes.
  ///
  /// A wire that fails puts every neighbour it ripped back as it was, so a
  /// failed attempt costs the plan nothing, and it keeps the way it had when
  /// that still fits.
  static void reroute(Corridors& corridors,
                      const std::vector<Request>& requests,
                      std::vector<std::optional<Route>>& routes,
                      const std::size_t index, const bool forward,
                      const Sweeps& sweeps) {
    const auto& request = requests[index];
    const auto before = routes[index];
    if (routes[index].has_value()) {
      corridors.withdraw(request, *routes[index]);
      routes[index].reset();
    }
    if (auto found = corridors.route(request); found.has_value()) {
      corridors.commit(request, *found);
      routes[index] = std::move(found);
      return;
    }

    // Nothing free reaches the target. The wires ahead of this one in the
    // sweep are taken out one after another until it fits. The sweep has not
    // settled them yet, so it gives each of them a new way before the round
    // is over.
    std::vector<std::pair<std::size_t, Route>> ripped;
    const auto total = static_cast<std::int64_t>(requests.size());
    for (std::uint32_t relax = 1; relax <= sweeps.maxRelaxation; ++relax) {
      const auto offset = static_cast<std::int64_t>(relax) * (forward ? 1 : -1);
      const auto neighbour = static_cast<std::size_t>(
          (((static_cast<std::int64_t>(index) + offset) % total) + total) %
          total);
      if (neighbour == index) {
        break;
      }
      if (routes[neighbour].has_value()) {
        corridors.withdraw(requests[neighbour], *routes[neighbour]);
        ripped.emplace_back(neighbour, *routes[neighbour]);
        routes[neighbour].reset();
      }
      if (auto found = corridors.route(request); found.has_value()) {
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
    // is rarely who stands in its way. The route the wire would take on an
    // empty chip names the wires that hold a slot it wants and the ones whose
    // chords it would cross. Those come out one at a time, and the sweep
    // gives each of them a way of its own.
    std::vector<std::pair<std::size_t, Route>> displaced;
    std::optional<Route> wanted;
    for (std::uint32_t attempt = 0; attempt < sweeps.maxRelaxation; ++attempt) {
      // The way is asked again after every wire that comes out, because what
      // stands in the way changes as they go.
      wanted = corridors.route(request, corridors.displacementPrice());
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
      if (!blocker.has_value()) {
        break;
      }
      corridors.withdraw(requests[*blocker], *routes[*blocker]);
      displaced.emplace_back(*blocker, *routes[*blocker]);
      routes[*blocker].reset();
      if (auto found = corridors.route(request); found.has_value()) {
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
      corridors.chargeFor(*wanted);
    }

    // Nothing worked, so the wire keeps the way it had if that still fits. It
    // was taken out at the top of the attempt, and a wire earlier in the
    // sweep may have taken part of it since; only then is it left out.
    if (before.has_value() && corridors.fits(request, *before)) {
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
      return std::make_unique<fbg::Point>(detail.toLayout(place.x, place.y));
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
    // The ways out of the pockets are on no border of the plan, so they are
    // listed together and marked, and not filed under a border they do not
    // belong to.
    auto pockets = std::make_unique<fba::BorderSlotsT>();
    pockets->pocket = true;
    for (std::size_t id = 0; id < corridors.slots().size(); ++id) {
      const auto border = corridors.slots()[id].border;
      if (border == NO_BORDER) {
        pockets->positions.push_back(corridors.layoutOf(id));
        continue;
      }
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
    if (!pockets->positions.empty()) {
      routing.slots.push_back(std::move(pockets));
    }
    return routing;
  }
};

} // namespace

std::unique_ptr<ICorridorRouter> makePartitionAStarRouter() {
  return std::make_unique<PartitionAStarRouter>();
}

} // namespace mqt::scpd::pipeline
