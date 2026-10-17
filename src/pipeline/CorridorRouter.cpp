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

/// Whether the point `c` lies on the chord from `a` to `b`, between its ends.
///
/// The three points are known to lie on one line, so what is left is whether
/// `c` falls between the two along it.
[[nodiscard]] bool between(const Place& a, const Place& b, const Place& c) {
  return std::min(a.x, b.x) <= c.x && c.x <= std::max(a.x, b.x) &&
         std::min(a.y, b.y) <= c.y && c.y <= std::max(a.y, b.y);
}

/// Whether one chord ends on the other, or the two share an end.
///
/// The crossing test does not see it: a chord that stops on another one never
/// gets to its far side, so the two never cross properly. Every end of a chord
/// is a fixed point that one wire passes: the point the wire is fed at, the
/// cell of its target port, or a crossing slot. A second wire drawn over that
/// point, or ending on it as well, puts two wires on one point of the chip. A
/// wire could otherwise run along a whole line of feed points without one
/// crossing.
///
/// The rule is stricter than the drawing needs. In a partition that is not
/// convex, a straight chord can run over a slot at an inward corner of the
/// border, and the wire through that slot need not cross the other wire.
[[nodiscard]] bool touches(const Chord& a, const Chord& b) {
  const auto over = [](const Chord& chord, const Place& end) {
    return sideOf(chord.from, chord.to, end) == 0 &&
           between(chord.from, chord.to, end);
  };
  const auto endsOn = [&over](const Chord& chord, const Chord& other) {
    return over(chord, other.from) || over(chord, other.to);
  };
  return endsOn(a, b) || endsOn(b, a);
}

/// Whether two chords may not both be drawn.
///
/// A proper crossing is what the plan has to rule out, so that the later
/// stage can draw it. Two chords that run along each other are ruled out as
/// well, because they put two wires in one place, and so is a chord drawn
/// over a point where another chord ends.
///
/// The chords are straight lines between where a wire enters and leaves a
/// partition. They stand in for the wires, and they decide exactly only in a
/// convex partition, where every chord stays inside.
[[nodiscard]] bool meets(const Chord& a, const Chord& b) {
  return crosses(a.from, a.to, b.from, b.to) ||
         overlaps(a.from, a.to, b.from, b.to) || touches(a, b);
}

/// The partition graph of one run, with the wires committed to it.
class Corridors {
public:
  /// Builds the graph of a plan.
  ///
  /// @throws std::invalid_argument When a border of the plan offers another
  /// number of slots than its budget, which a plan made with another
  /// crossing pitch, or by a version that picks the slots another way, does.
  Corridors(const CapacityPlanT& plan, const CapacityScene& scene,
            const double pitch)
      : detailGrid(scene.detail) {
    const auto& detail = scene.detail;
    for (std::uint32_t index = 0; index < plan.borders.size(); ++index) {
      const auto& border = *plan.borders[index];
      grid::PartitionBorder geometry{
          .first = static_cast<PartitionLabel>(border.first),
          .second = static_cast<PartitionLabel>(border.second),
          .samples = {}};
      geometry.samples.reserve(border.samples.size());
      for (const auto& sample : border.samples) {
        const auto corner = grid::layoutToCorner(detail, sample);
        geometry.samples.emplace_back(onGrid(corner.x()), onGrid(corner.y()));
      }
      const auto slots = crossingSlots(geometry, scene, pitch);
      if (slots.size() != border.budget) {
        throw std::invalid_argument(std::format(
            "border {} between partitions {} and {} offers {} crossing slots "
            "and the plan gives it a budget of {}; the plan was made with "
            "another crossing pitch or by another version of mqt-scpd, so run "
            "the capacity step again",
            index, border.first, border.second, slots.size(), border.budget));
      }
      for (const auto& slot : slots) {
        slotsByPartition[geometry.first].push_back(slotList.size());
        slotsByPartition[geometry.second].push_back(slotList.size());
        slotList.push_back({.place = {.x = slot.x(), .y = slot.y()},
                            .first = geometry.first,
                            .second = geometry.second,
                            .border = index});
      }
    }
    findNearby(pitch);
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
  /// wires the step displaces. The search therefore takes as few blocked
  /// steps as it can. A sweep asks for a displacing route only for a wire
  /// that the free search did not place, so the price never trades a free
  /// way for a shorter blocked one.
  ///
  /// A route that crosses itself or comes back to a slot it already used is
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

  /// The wires in the way of a route drawn as though the chip were empty:
  /// the ones that hold a slot it wants or a slot within one pitch of it, and
  /// the ones whose chords it would have to cross.
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

  /// Whether a route can be laid down as it stands: every slot still free,
  /// and every chord clear of the ones already drawn.
  [[nodiscard]] bool fits(const Request& request, const Route& route) const {
    if (std::ranges::any_of(route.slots, [&](const std::size_t slot) {
          return heldByAnother(slot, request.connection);
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
  void findNearby(const double pitch) {
    nearbySlots.assign(slotList.size(), {});
    // The slots, in layout units, binned into squares one pitch wide, so that
    // a slot is compared only with the slots of its own square and the eight
    // around it.
    const auto squareOf = [pitch](const double coordinate) {
      return static_cast<std::int64_t>(std::floor(coordinate / pitch));
    };
    std::vector<fbg::Point> layout;
    layout.reserve(slotList.size());
    std::map<std::pair<std::int64_t, std::int64_t>, std::vector<std::size_t>>
        squares;
    for (std::size_t slot = 0; slot < slotList.size(); ++slot) {
      layout.push_back(layoutOf(slot));
      squares[{squareOf(layout.back().x()), squareOf(layout.back().y())}]
          .push_back(slot);
    }
    for (std::size_t slot = 0; slot < slotList.size(); ++slot) {
      const auto& here = layout[slot];
      const auto column = squareOf(here.x());
      const auto row = squareOf(here.y());
      for (std::int64_t dy = -1; dy <= 1; ++dy) {
        for (std::int64_t dx = -1; dx <= 1; ++dx) {
          const auto found = squares.find({column + dx, row + dy});
          if (found == squares.end()) {
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
        const auto blocked = heldByAnother(slot, request.connection) ||
                             !clear(Chord{.from = place,
                                          .to = slotList[slot].place,
                                          .owner = request.connection},
                                    partition);
        if (blocked && displacement <= 0.0) {
          continue;
        }
        const auto state = stateOf(slot, slotList[slot].beyond(partition));
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

    // Reaching the target inside the partition the wire is fed in is a whole
    // route, and it needs no slot.
    auto shortest = UNREACHED;
    auto arrival = NO_STATE;
    if (request.feedPartition == request.targetPartition) {
      const auto direct = clear(Chord{.from = request.feed,
                                      .to = request.target,
                                      .owner = request.connection},
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
  std::uint32_t rounds = 12;
  /// The relaxation that the configuration asks for.
  std::uint32_t askedRelaxation = 4;
  /// The relaxation that the sweeps use: the asked one, but fewer than half
  /// the connections, and at least one where two or more connections compete.
  std::uint32_t maxRelaxation = 4;
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
  Sweeps sweeps;
  if (config.stages != nullptr && config.stages->corridor != nullptr) {
    const auto& params = *config.stages->corridor;
    sweeps.rounds = params.rounds;
    sweeps.askedRelaxation = params.max_relaxation;
  }
  const auto limit =
      connections < 2 ? 0 : std::max<std::size_t>(1, (connections - 1) / 2);
  sweeps.maxRelaxation = static_cast<std::uint32_t>(
      std::min<std::size_t>(sweeps.askedRelaxation, limit));
  return sweeps;
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

/// The labels of the partitions of the plan, back on the detail grid.
///
/// @throws std::invalid_argument When a partition falls apart into pieces
/// that do not touch. The search treats a partition as one region, and would
/// otherwise draw a chord from one piece to the other.
[[nodiscard]] std::vector<PartitionLabel> labelsOf(const CapacityPlanT& plan,
                                                   const CapacityScene& scene) {
  const auto& detail = scene.detail;
  std::vector<grid::PartitionOutline> outlines;
  for (const auto& partition : plan.partitions) {
    for (const auto& ring : partition->outlines) {
      grid::PartitionOutline outline{
          .label = static_cast<PartitionLabel>(partition->label), .ring = {}};
      outline.ring.reserve(ring->vertices.size());
      for (const auto& vertex : ring->vertices) {
        outline.ring.push_back(grid::layoutToCorner(detail, vertex));
      }
      outlines.push_back(std::move(outline));
    }
  }
  auto labels = grid::rasterizePartitions(outlines, detail);

  auto highest = grid::FIRST_PARTITION_LABEL;
  for (const auto label : labels) {
    highest = std::max(highest, label);
  }
  auto pieces = labels;
  if (grid::separatePartitionPieces(scene.blocked, pieces, detail,
                                    grid::nextPartitionLabel(highest)) !=
      grid::nextPartitionLabel(highest)) {
    const auto cell = std::ranges::mismatch(labels, pieces).in1;
    throw std::invalid_argument(
        std::format("partition {} of the capacity plan falls apart into "
                    "pieces that do not touch",
                    *cell));
  }
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
    report.progress("rasterizing the chip", "", 0, 0);
    const auto scene = buildScene(chip, config);
    report.progress("rasterizing the partitions", "", 0, 0);
    requireSameGrid(capacity, scene.detail);
    requirePortsOfChip(assignment, chip);
    const auto labels = labelsOf(capacity, scene);

    const auto requests = requestsOf(scene, assignment, labels);

    Corridors corridors(capacity, scene, crossingPitch(config));

    const auto sweeps = sweepsOf(config, requests.size());
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
    // the round ripped near its end and had no sweep left to place again.
    auto best = routes;
    auto bestLeft = requests.size();
    std::uint32_t bestRound = 0;
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
      // Only a round that routes everything ends the sweeps. A round in which
      // nothing moved is not the end: a wire that found no way charged the
      // slots it wanted, and it takes several such rounds before the charge
      // is high enough to move the wire that stands on one.
      if (left == 0) {
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
      report.progress("free slots", "", index, requests.size());
      if (routes[index].has_value()) {
        continue;
      }
      if (auto found = corridors.route(requests[index]); found.has_value()) {
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
    // is over. Near the end of the sweep the count runs on past the last wire
    // to the first ones, which this round has passed; those wait for the
    // next round. The relaxation stays below the number of connections, so
    // the count never comes back to this wire.
    std::vector<std::pair<std::size_t, Route>> ripped;
    const auto total = static_cast<std::int64_t>(requests.size());
    for (std::uint32_t relax = 1; relax <= sweeps.maxRelaxation; ++relax) {
      const auto offset = static_cast<std::int64_t>(relax) * (forward ? 1 : -1);
      const auto neighbor = static_cast<std::size_t>(
          (((static_cast<std::int64_t>(index) + offset) % total) + total) %
          total);
      if (routes[neighbor].has_value()) {
        corridors.withdraw(requests[neighbor], *routes[neighbor]);
        ripped.emplace_back(neighbor, *routes[neighbor]);
        routes[neighbor].reset();
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
      corridors.chargeFor(request, *wanted);
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
