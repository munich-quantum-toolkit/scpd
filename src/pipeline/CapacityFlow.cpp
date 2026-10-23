/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/pipeline/CapacityFlow.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <functional>
#include <limits>
#include <optional>
#include <queue>
#include <stdexcept>
#include <utility>
#include <vector>

namespace mqt::scpd::pipeline {
namespace {

/// Whether a demand may use an edge.
bool opens(const FlowEdge& edge, const std::uint32_t demand) {
  return !edge.users.has_value() ||
         std::ranges::find(*edge.users, demand) != edge.users->end();
}

/// Throws when an edge or a demand names a chamber out of range, or an edge
/// a demand that does not exist.
void checkNames(const std::uint32_t chambers,
                const std::vector<FlowEdge>& edges,
                const std::vector<FlowDemand>& demands) {
  for (std::size_t index = 0; index < edges.size(); ++index) {
    for (const auto chamber : edges[index].chambers) {
      if (chamber >= chambers) {
        throw std::invalid_argument(std::format(
            "edge {} names chamber {} of {}", index, chamber, chambers));
      }
    }
    if (edges[index].users.has_value()) {
      for (const auto user : *edges[index].users) {
        if (user >= demands.size()) {
          throw std::invalid_argument(
              std::format("edge {} is open to demand {} of {}", index, user,
                          demands.size()));
        }
      }
    }
  }
  for (std::size_t index = 0; index < demands.size(); ++index) {
    if (demands[index].crossings.size() > MAX_CROSSINGS) {
      throw std::invalid_argument(
          std::format("demand {} has {} groups to cross, more than {}", index,
                      demands[index].crossings.size(), MAX_CROSSINGS));
    }
    for (const auto& group : demands[index].crossings) {
      for (const auto edge : group) {
        if (edge >= edges.size()) {
          throw std::invalid_argument(
              std::format("demand {} has to cross edge {} of {}", index, edge,
                          edges.size()));
        }
      }
    }
    for (const auto& side : {demands[index].from, demands[index].to}) {
      for (const auto chamber : side) {
        if (chamber >= chambers) {
          throw std::invalid_argument(std::format(
              "demand {} names chamber {} of {}", index, chamber, chambers));
        }
      }
    }
  }
}

/// The edges at every chamber.
std::vector<std::vector<std::uint32_t>>
edgesAround(const std::uint32_t chambers, const std::vector<FlowEdge>& edges) {
  std::vector<std::vector<std::uint32_t>> around(chambers);
  for (std::uint32_t index = 0; index < edges.size(); ++index) {
    for (const auto chamber : edges[index].chambers) {
      around[chamber].push_back(index);
    }
  }
  return around;
}

/// The group of edges to cross each edge is in for one demand, or
/// `NO_GROUP`.
constexpr std::uint32_t NO_GROUP = std::numeric_limits<std::uint32_t>::max();

std::vector<std::uint32_t> groupsOf(const std::size_t edges,
                                    const FlowDemand& wanted) {
  std::vector<std::uint32_t> group(edges, NO_GROUP);
  for (std::uint32_t at = 0; at < wanted.crossings.size(); ++at) {
    for (const auto edge : wanted.crossings[at]) {
      group[edge] = at;
    }
  }
  return group;
}

/// The way of one demand with the fewest edges over the edges open to it
/// that `usable` lets it take; nothing when there is none.
///
/// A state of the search is a chamber and the groups of edges to cross the
/// way has crossed so far. An edge of a group is taken only while that
/// group is not crossed, and the way ends in an end chamber with every
/// group crossed.
template <typename Usable>
std::optional<std::vector<std::uint32_t>>
fewestEdges(const std::uint32_t chambers, const std::vector<FlowEdge>& edges,
            const std::vector<std::vector<std::uint32_t>>& around,
            const FlowDemand& wanted, const std::uint32_t demand,
            const Usable& usable) {
  const auto group = groupsOf(edges.size(), wanted);
  const auto masks = std::size_t{1} << wanted.crossings.size();
  const auto all = masks - 1;
  const auto stateOf = [masks](const std::uint32_t chamber,
                               const std::size_t mask) {
    return (static_cast<std::size_t>(chamber) * masks) + mask;
  };
  // The edge each state was first reached over, and the state before.
  constexpr auto START = std::numeric_limits<std::uint32_t>::max();
  constexpr auto UNSEEN = START - 1;
  std::vector<std::uint32_t> over(static_cast<std::size_t>(chambers) * masks,
                                  UNSEEN);
  std::vector<std::size_t> before(over.size(), 0);
  std::queue<std::size_t> pending;
  for (const auto chamber : wanted.from) {
    const auto state = stateOf(chamber, 0);
    if (over[state] == UNSEEN) {
      over[state] = START;
      pending.push(state);
    }
  }
  while (!pending.empty()) {
    const auto state = pending.front();
    pending.pop();
    const auto chamber = static_cast<std::uint32_t>(state / masks);
    const auto mask = state % masks;
    if (mask == all &&
        std::ranges::find(wanted.to, chamber) != wanted.to.end()) {
      std::vector<std::uint32_t> way;
      for (auto at = state; over[at] != START; at = before[at]) {
        way.push_back(over[at]);
      }
      std::ranges::reverse(way);
      return way;
    }
    for (const auto index : around[chamber]) {
      if (!opens(edges[index], demand) || !usable(index)) {
        continue;
      }
      auto onward = mask;
      if (group[index] != NO_GROUP) {
        const auto bit = std::size_t{1} << group[index];
        if ((mask & bit) != 0) {
          continue;
        }
        onward |= bit;
      }
      // An edge leads into its other chambers: back into the one it was
      // entered from is no step, and crosses nothing — unless that one is
      // all it has, as a stretch of a feedline open at one end has.
      const auto& sides = edges[index].chambers;
      const bool loop = std::ranges::all_of(
          sides, [chamber](const std::uint32_t c) { return c == chamber; });
      for (const auto next : sides) {
        if (next == chamber && !loop) {
          continue;
        }
        const auto reached = stateOf(next, onward);
        if (over[reached] == UNSEEN) {
          over[reached] = index;
          before[reached] = state;
          pending.push(reached);
        }
        if (loop) {
          break;
        }
      }
    }
  }
  return std::nullopt;
}

/// The centre of an edge's line: the middle of a bottleneck, or of the
/// stretch of a feedline a wire crosses.
FlowPoint centreOf(const FlowEdge& edge) {
  return {.x = 0.5 * (edge.from.x + edge.to.x),
          .y = 0.5 * (edge.from.y + edge.to.y)};
}

/// The distance between two points.
double apart(const FlowPoint a, const FlowPoint b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

/// The way of one demand whose length is shortest — from its start to the
/// centre of the first edge it passes, from centre to centre, and from the
/// last centre to its end (user, 2026-10-13) — over the edges open to it
/// that `usable` lets it take, and that length; nothing when there is none.
///
/// A state of the search is an edge, the chamber the way left it into, and
/// the groups of edges to cross so far, as in `fewestEdges`: where a way
/// stands depends on the edge it came through, so a chamber alone is not a
/// state.
template <typename Usable>
std::optional<std::pair<std::vector<std::uint32_t>, double>>
shortestWay(const std::vector<FlowEdge>& edges,
            const std::vector<std::vector<std::uint32_t>>& around,
            const FlowDemand& wanted, const std::uint32_t demand,
            const Usable& usable) {
  const auto group = groupsOf(edges.size(), wanted);
  const auto masks = std::size_t{1} << wanted.crossings.size();
  const auto all = masks - 1;
  const auto length = [](const FlowPoint a, const FlowPoint b) {
    return std::hypot(a.x - b.x, a.y - b.y);
  };
  const auto ends = [&](const std::uint32_t chamber) {
    return std::ranges::find(wanted.to, chamber) != wanted.to.end();
  };
  if (wanted.crossings.empty() && std::ranges::any_of(wanted.from, ends)) {
    return std::pair{std::vector<std::uint32_t>{},
                     length(wanted.start, wanted.end)};
  }
  // Every state an index: the edge, which of its chambers, the mask.
  std::vector<std::size_t> first(edges.size() + 1, 0);
  for (std::size_t index = 0; index < edges.size(); ++index) {
    first[index + 1] = first[index] + edges[index].chambers.size();
  }
  const auto states = first.back() * masks;
  const auto FINISH = states;
  std::vector<double> best(states + 1, std::numeric_limits<double>::max());
  std::vector<std::size_t> before(states + 1, FINISH);
  struct Entry {
    double cost;
    std::size_t state;
    bool operator>(const Entry& other) const { return cost > other.cost; }
  };
  std::priority_queue<Entry, std::vector<Entry>, std::greater<>> pending;
  // Into edge `index` from `chamber`, at `cost` so far and `mask` crossed:
  // a state for every other chamber of it.
  const auto enter = [&](const std::uint32_t index, const std::uint32_t chamber,
                         const std::size_t mask, const double cost,
                         const std::size_t from) {
    if (!opens(edges[index], demand) || !usable(index)) {
      return;
    }
    auto onward = mask;
    if (group[index] != NO_GROUP) {
      const auto bit = std::size_t{1} << group[index];
      if ((mask & bit) != 0) {
        return;
      }
      onward |= bit;
    }
    const auto& sides = edges[index].chambers;
    const bool loop = std::ranges::all_of(
        sides, [chamber](const std::uint32_t c) { return c == chamber; });
    for (std::size_t k = 0; k < sides.size(); ++k) {
      if (sides[k] == chamber && !loop) {
        continue;
      }
      const auto state = ((first[index] + k) * masks) + onward;
      if (cost < best[state]) {
        best[state] = cost;
        before[state] = from;
        pending.push({cost, state});
      }
      if (loop) {
        break;
      }
    }
  };
  for (const auto chamber : wanted.from) {
    for (const auto index : around[chamber]) {
      enter(index, chamber, 0, apart(wanted.start, centreOf(edges[index])),
            FINISH);
    }
  }
  while (!pending.empty()) {
    const auto [cost, state] = pending.top();
    pending.pop();
    if (cost > best[state]) {
      continue;
    }
    if (state == FINISH) {
      std::vector<std::uint32_t> way;
      for (auto at = before[FINISH]; at != FINISH; at = before[at]) {
        const auto slot = at / masks;
        const auto index = static_cast<std::uint32_t>(
            std::ranges::upper_bound(first, slot) - first.begin() - 1);
        way.push_back(index);
      }
      std::ranges::reverse(way);
      return std::pair{std::move(way), cost};
    }
    const auto slot = state / masks;
    const auto mask = state % masks;
    const auto index = static_cast<std::uint32_t>(
        std::ranges::upper_bound(first, slot) - first.begin() - 1);
    const auto chamber = edges[index].chambers[slot - first[index]];
    if (mask == all && ends(chamber)) {
      const auto done = cost + apart(wanted.end, centreOf(edges[index]));
      if (done < best[FINISH]) {
        best[FINISH] = done;
        before[FINISH] = state;
        pending.push({done, FINISH});
      }
    }
    for (const auto next : around[chamber]) {
      if (next != index) {
        enter(next, chamber, mask,
              cost + apart(centreOf(edges[index]), centreOf(edges[next])),
              state);
      }
    }
  }
  return std::nullopt;
}

} // namespace

std::uint32_t wiresThroughGap(const double cells, const double clearance) {
  if (!(clearance > 0.0) || cells < clearance) {
    return 0;
  }
  return static_cast<std::uint32_t>(std::floor(cells / clearance));
}

FlowCheck checkInTurn(const std::uint32_t chambers,
                      const std::vector<FlowEdge>& edges,
                      const std::vector<FlowDemand>& demands,
                      const double detour, const InTurnRules rules) {
  checkNames(chambers, edges, demands);
  const auto around = edgesAround(chambers, edges);
  FlowCheck check;
  check.ways.assign(demands.size(), std::nullopt);
  check.load.assign(edges.size(), 0);
  check.overflow.assign(edges.size(), 0);
  check.lengths.assign(demands.size(), 0.0);
  check.tooLong.assign(demands.size(), false);
  // An edge that still takes a wire; `barred` is one closed besides, while
  // a demand is routed off it.
  constexpr auto NONE = std::numeric_limits<std::uint32_t>::max();
  std::uint32_t barred = NONE;
  const auto free = [&](const std::uint32_t index) {
    return index != barred && check.load[index] < edges[index].capacity;
  };
  const auto any = [](const std::uint32_t /*index*/) { return true; };
  // An edge that takes a wire at all, whatever its load.
  const auto takes = [&](const std::uint32_t index) {
    return edges[index].capacity > 0;
  };
  using Way = std::optional<std::vector<std::uint32_t>>;
  // The way a demand takes by its rule over the edges as they are loaded;
  // `within` says whether it keeps to the edges that still take a wire.
  const auto routeOne = [&](const std::uint32_t demand, bool& within) -> Way {
    const auto& wanted = demands[demand];
    within = false;
    if (wanted.longest.has_value()) {
      // The shortest way within the capacities, else the shortest at all;
      // either only when it is short enough.
      for (const bool withinCapacity : {true, false}) {
        const auto found =
            withinCapacity ? shortestWay(edges, around, wanted, demand, free)
                           : shortestWay(edges, around, wanted, demand, any);
        if (!found.has_value()) {
          continue;
        }
        check.lengths[demand] = found->second;
        if (found->second <= *wanted.longest) {
          within = withinCapacity;
          return found->first;
        }
        check.tooLong[demand] = !withinCapacity;
      }
      return std::nullopt;
    }
    if (detour > 0.0) {
      // The shortest way within the capacities when it is no longer than
      // `detour` times the shortest over the edges that take a wire at all
      // (user, 2026-10-14: a gate that takes none is no way to measure by),
      // else that shortest; and when every way passes such a gate, the
      // shortest at all.
      auto shortest = rules.detourOverTakers
                          ? shortestWay(edges, around, wanted, demand, takes)
                          : shortestWay(edges, around, wanted, demand, any);
      if (!shortest.has_value()) {
        shortest = shortestWay(edges, around, wanted, demand, any);
      }
      if (!shortest.has_value()) {
        return std::nullopt;
      }
      const auto near = shortestWay(edges, around, wanted, demand, free);
      if (near.has_value() &&
          near->second <= (detour * shortest->second) + 1e-9) {
        within = true;
        check.lengths[demand] = near->second;
        return near->first;
      }
      check.lengths[demand] = shortest->second;
      return shortest->first;
    }
    auto way = fewestEdges(chambers, edges, around, wanted, demand, free);
    if (way.has_value()) {
      within = true;
      return way;
    }
    return fewestEdges(chambers, edges, around, wanted, demand, any);
  };
  const auto put = [&](const std::uint32_t demand, Way way) {
    if (way.has_value()) {
      for (const auto index : *way) {
        ++check.load[index];
      }
    }
    check.ways[demand] = std::move(way);
  };
  const auto take = [&](const std::uint32_t demand) {
    Way way = std::move(check.ways[demand]);
    check.ways[demand].reset();
    if (way.has_value()) {
      for (const auto index : *way) {
        --check.load[index];
      }
    }
    return way;
  };
  const auto overfills = [&](const std::uint32_t demand) {
    const auto& way = check.ways[demand];
    return way.has_value() &&
           std::ranges::any_of(*way, [&](const std::uint32_t index) {
             return check.load[index] > edges[index].capacity;
           });
  };

  // The first sweep, in order.
  for (std::uint32_t demand = 0; demand < demands.size(); ++demand) {
    bool within = false;
    put(demand, routeOne(demand, within));
  }
  // Rip up and reroute (user, 2026-10-14): a sweep backward, then forward,
  // and so on, over the demands that overfill an edge. Each is taken off
  // its way and routed again. When it keeps to no way within the
  // capacities, every demand on an edge its way would overfill is taken
  // off in turn and routed again within the capacities off that edge;
  // then the demand once more.
  while (check.sweeps < rules.mostSweeps) {
    bool lost = false;
    for (std::uint32_t demand = 0; demand < demands.size() && !lost; ++demand) {
      lost = overfills(demand);
    }
    if (!lost) {
      break;
    }
    ++check.sweeps;
    const bool backward = (check.sweeps % 2) == 0;
    bool moved = false;
    for (std::uint32_t k = 0; k < demands.size(); ++k) {
      const auto demand =
          backward ? static_cast<std::uint32_t>(demands.size() - 1 - k) : k;
      if (!overfills(demand)) {
        continue;
      }
      take(demand);
      bool within = false;
      auto way = routeOne(demand, within);
      if (!within && way.has_value()) {
        for (const auto index : *way) {
          for (std::uint32_t other = 0;
               other < demands.size() &&
               check.load[index] >= edges[index].capacity;
               ++other) {
            if (other == demand || !check.ways[other].has_value() ||
                std::ranges::find(*check.ways[other], index) ==
                    check.ways[other]->end()) {
              continue;
            }
            auto kept = take(other);
            barred = index;
            bool otherWithin = false;
            auto elsewhere = routeOne(other, otherWithin);
            barred = NONE;
            if (otherWithin && elsewhere.has_value()) {
              put(other, std::move(elsewhere));
              moved = true;
            } else {
              put(other, std::move(kept));
            }
          }
        }
        way = routeOne(demand, within);
      }
      moved = moved || within;
      put(demand, std::move(way));
    }
    if (!moved) {
      break;
    }
  }
  for (std::uint32_t index = 0; index < edges.size(); ++index) {
    check.overflow[index] = check.load[index] > edges[index].capacity
                                ? check.load[index] - edges[index].capacity
                                : 0U;
    check.shortBy += check.overflow[index];
  }
  return check;
}

} // namespace mqt::scpd::pipeline
