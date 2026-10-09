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

#include "mqt-scpd/milp/Backend.hpp"
#include "mqt-scpd/milp/Model.hpp"

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

/// The arcs one demand may use through one edge: into the edge from each of
/// its chambers, and out of it to each.
struct EdgeArcs {
  std::uint32_t edge = 0;
  std::vector<milp::Var> in;
  std::vector<milp::Var> out;
};

/// The variables of one demand.
struct DemandArcs {
  std::vector<EdgeArcs> edges;
  /// From the source to each start chamber, and from each end chamber to
  /// the sink.
  std::vector<milp::Var> start;
  std::vector<milp::Var> end;
};

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
      // entered from is no step, and crosses nothing.
      for (const auto next : edges[index].chambers) {
        if (next == chamber) {
          continue;
        }
        const auto reached = stateOf(next, onward);
        if (over[reached] == UNSEEN) {
          over[reached] = index;
          before[reached] = state;
          pending.push(reached);
        }
      }
    }
  }
  return std::nullopt;
}

/// The distance from a point to the line of an edge.
double toLine(const FlowPoint p, const FlowEdge& edge) {
  const auto dx = edge.to.x - edge.from.x;
  const auto dy = edge.to.y - edge.from.y;
  const auto squared = (dx * dx) + (dy * dy);
  const auto t = squared > 0.0 ? std::clamp((((p.x - edge.from.x) * dx) +
                                             ((p.y - edge.from.y) * dy)) /
                                                squared,
                                            0.0, 1.0)
                               : 0.0;
  return std::hypot(p.x - (edge.from.x + (t * dx)),
                    p.y - (edge.from.y + (t * dy)));
}

/// The distance between the lines of two edges: none where they cross.
double betweenLines(const FlowEdge& a, const FlowEdge& b) {
  const auto side = [](const FlowPoint p, const FlowPoint q,
                       const FlowPoint r) {
    return ((q.x - p.x) * (r.y - p.y)) - ((q.y - p.y) * (r.x - p.x));
  };
  const auto d1 = side(a.from, a.to, b.from);
  const auto d2 = side(a.from, a.to, b.to);
  const auto d3 = side(b.from, b.to, a.from);
  const auto d4 = side(b.from, b.to, a.to);
  if (((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0)) &&
      ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))) {
    return 0.0;
  }
  return std::min(
      {toLine(a.from, b), toLine(a.to, b), toLine(b.from, a), toLine(b.to, a)});
}

/// The way of one demand whose least length is shortest — from its start to
/// the line of the first edge it passes, from line to line, and from the last
/// line to its end — over the edges open to it that `usable` lets it take,
/// and that length; nothing when there is none.
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
    for (std::size_t k = 0; k < sides.size(); ++k) {
      if (sides[k] == chamber) {
        continue;
      }
      const auto state = ((first[index] + k) * masks) + onward;
      if (cost < best[state]) {
        best[state] = cost;
        before[state] = from;
        pending.push({cost, state});
      }
    }
  };
  for (const auto chamber : wanted.from) {
    for (const auto index : around[chamber]) {
      enter(index, chamber, 0, toLine(wanted.start, edges[index]), FINISH);
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
      const auto done = cost + toLine(wanted.end, edges[index]);
      if (done < best[FINISH]) {
        best[FINISH] = done;
        before[FINISH] = state;
        pending.push({done, FINISH});
      }
    }
    for (const auto next : around[chamber]) {
      if (next != index) {
        enter(next, chamber, mask,
              cost + betweenLines(edges[index], edges[next]), state);
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

FlowCheck checkCapacity(const std::uint32_t chambers,
                        const std::vector<FlowEdge>& edges,
                        const std::vector<FlowDemand>& demands,
                        const milp::SolveOptions& options) {
  checkNames(chambers, edges, demands);

  const auto around = edgesAround(chambers, edges);

  FlowCheck check;
  check.ways.assign(demands.size(), std::nullopt);
  check.load.assign(edges.size(), 0);
  check.overflow.assign(edges.size(), 0);

  // The chambers a demand reaches over the edges open to it, whatever the
  // capacities. A demand that reaches none of its end chambers has no way;
  // one that starts in an end chamber needs no edge.
  std::vector<std::vector<bool>> reach(demands.size());
  std::vector<std::uint32_t> modelled;
  for (std::uint32_t demand = 0; demand < demands.size(); ++demand) {
    const auto& wanted = demands[demand];
    if (std::ranges::any_of(wanted.from, [&](const std::uint32_t chamber) {
          return std::ranges::find(wanted.to, chamber) != wanted.to.end();
        })) {
      check.ways[demand] = std::vector<std::uint32_t>{};
      continue;
    }
    auto& seen = reach[demand];
    seen.assign(chambers, false);
    std::queue<std::uint32_t> pending;
    for (const auto chamber : wanted.from) {
      if (!seen[chamber]) {
        seen[chamber] = true;
        pending.push(chamber);
      }
    }
    while (!pending.empty()) {
      const auto chamber = pending.front();
      pending.pop();
      for (const auto index : around[chamber]) {
        if (!opens(edges[index], demand)) {
          continue;
        }
        for (const auto next : edges[index].chambers) {
          if (!seen[next]) {
            seen[next] = true;
            pending.push(next);
          }
        }
      }
    }
    if (std::ranges::any_of(wanted.to, [&](const std::uint32_t chamber) {
          return seen[chamber];
        })) {
      modelled.push_back(demand);
    }
  }
  if (modelled.empty()) {
    check.status = milp::SolveStatus::Optimal;
    return check;
  }

  // The network of every modelled demand, over the chambers it reaches.
  milp::Model model("capacity-check");
  std::vector<DemandArcs> arcs(demands.size());
  std::size_t arcCount = 0;
  for (const auto demand : modelled) {
    const auto& seen = reach[demand];
    auto& own = arcs[demand];
    for (std::uint32_t index = 0; index < edges.size(); ++index) {
      const auto& edge = edges[index];
      if (!opens(edge, demand) || edge.chambers.empty() ||
          !seen[edge.chambers.front()]) {
        continue;
      }
      EdgeArcs through{.edge = index, .in = {}, .out = {}};
      for (const auto chamber : edge.chambers) {
        through.in.push_back(model.addBinary(
            std::format("in_{}_{}_{}", demand, index, chamber)));
        through.out.push_back(model.addBinary(
            std::format("out_{}_{}_{}", demand, index, chamber)));
        arcCount += 2;
      }
      own.edges.push_back(std::move(through));
    }
    for (const auto chamber : demands[demand].from) {
      own.start.push_back(
          model.addBinary(std::format("start_{}_{}", demand, chamber)));
    }
    for (const auto chamber : demands[demand].to) {
      own.end.push_back(
          model.addBinary(std::format("end_{}_{}", demand, chamber)));
    }
  }

  // Every arc into an edge costs one, so a way takes no detour and closes no
  // loop; one unit of overflow costs more than every arc there is together.
  const auto overflowPrice = static_cast<double>(arcCount + 1);
  std::vector<milp::Var> overflow;
  overflow.reserve(edges.size());
  for (std::uint32_t index = 0; index < edges.size(); ++index) {
    overflow.push_back(model.addInteger(std::format("overflow_{}", index), 0.0,
                                        static_cast<double>(demands.size()),
                                        overflowPrice));
  }

  std::vector<milp::LinearExpr> load(edges.size());
  for (const auto demand : modelled) {
    const auto& own = arcs[demand];
    std::vector<milp::LinearExpr> balance(chambers);
    for (const auto& through : own.edges) {
      const auto& edge = edges[through.edge];
      milp::LinearExpr kept;
      for (std::size_t side = 0; side < edge.chambers.size(); ++side) {
        const auto chamber = edge.chambers[side];
        // Out of the chamber into the edge, and back out of the edge into
        // the chamber.
        balance[chamber].add(through.in[side], 1.0);
        balance[chamber].add(through.out[side], -1.0);
        kept.add(through.in[side], 1.0);
        kept.add(through.out[side], -1.0);
        load[through.edge].add(through.in[side], 1.0);
        model.addObjective(milp::LinearExpr(through.in[side]));
      }
      model.addEqual(std::format("keep_{}_{}", demand, through.edge), kept,
                     0.0);
    }
    milp::LinearExpr leaves;
    for (std::size_t at = 0; at < own.start.size(); ++at) {
      balance[demands[demand].from[at]].add(own.start[at], -1.0);
      leaves.add(own.start[at], 1.0);
    }
    milp::LinearExpr arrives;
    for (std::size_t at = 0; at < own.end.size(); ++at) {
      balance[demands[demand].to[at]].add(own.end[at], 1.0);
      arrives.add(own.end[at], 1.0);
    }
    model.addEqual(std::format("leave_{}", demand), leaves, 1.0);
    model.addEqual(std::format("arrive_{}", demand), arrives, 1.0);
    for (std::uint32_t chamber = 0; chamber < chambers; ++chamber) {
      if (!balance[chamber].terms().empty()) {
        model.addEqual(std::format("balance_{}_{}", demand, chamber),
                       balance[chamber], 0.0);
      }
    }
  }
  for (std::uint32_t index = 0; index < edges.size(); ++index) {
    if (load[index].terms().empty()) {
      continue;
    }
    auto row = load[index];
    row.add(overflow[index], -1.0);
    model.addLessOrEqual(std::format("capacity_{}", index), row,
                         static_cast<double>(edges[index].capacity));
  }

  const auto solution = milp::makeHighsBackend()->solve(model, options);
  check.status = solution.status;
  if (!solution.hasValues()) {
    return check;
  }

  // Each way read back from where it leaves: into an edge from the chamber
  // it stands in, out of that edge into the next, until it arrives.
  for (const auto demand : modelled) {
    const auto& own = arcs[demand];
    std::optional<std::uint32_t> at;
    for (std::size_t index = 0; index < own.start.size(); ++index) {
      if (solution.isSet(own.start[index])) {
        at = demands[demand].from[index];
      }
    }
    const auto arrived = [&](const std::uint32_t chamber) {
      for (std::size_t index = 0; index < own.end.size(); ++index) {
        if (demands[demand].to[index] == chamber &&
            solution.isSet(own.end[index])) {
          return true;
        }
      }
      return false;
    };
    std::vector<std::uint32_t> way;
    std::vector<bool> used(own.edges.size(), false);
    while (at.has_value() && !arrived(*at) && way.size() <= own.edges.size()) {
      std::optional<std::uint32_t> next;
      for (std::size_t index = 0; index < own.edges.size() && !next.has_value();
           ++index) {
        if (used[index]) {
          continue;
        }
        const auto& through = own.edges[index];
        const auto& edge = edges[through.edge];
        for (std::size_t side = 0; side < edge.chambers.size(); ++side) {
          if (edge.chambers[side] != *at || !solution.isSet(through.in[side])) {
            continue;
          }
          for (std::size_t exit = 0; exit < edge.chambers.size(); ++exit) {
            if (solution.isSet(through.out[exit])) {
              used[index] = true;
              way.push_back(through.edge);
              next = edge.chambers[exit];
              break;
            }
          }
          break;
        }
      }
      at = next;
    }
    check.ways[demand] = std::move(way);
  }
  for (std::uint32_t index = 0; index < edges.size(); ++index) {
    for (const auto demand : modelled) {
      for (const auto& through : arcs[demand].edges) {
        if (through.edge != index) {
          continue;
        }
        for (const auto in : through.in) {
          check.load[index] += solution.isSet(in) ? 1U : 0U;
        }
      }
    }
    check.overflow[index] = static_cast<std::uint32_t>(std::max<std::int64_t>(
        0, static_cast<std::int64_t>(check.load[index]) -
               static_cast<std::int64_t>(edges[index].capacity)));
    check.shortBy += check.overflow[index];
  }
  return check;
}

FlowCheck checkInTurn(const std::uint32_t chambers,
                      const std::vector<FlowEdge>& edges,
                      const std::vector<FlowDemand>& demands) {
  checkNames(chambers, edges, demands);
  const auto around = edgesAround(chambers, edges);
  FlowCheck check;
  check.status = milp::SolveStatus::Optimal;
  check.ways.reserve(demands.size());
  check.load.assign(edges.size(), 0);
  check.overflow.assign(edges.size(), 0);
  check.lengths.assign(demands.size(), 0.0);
  check.tooLong.assign(demands.size(), false);
  const auto free = [&](const std::uint32_t index) {
    return check.load[index] < edges[index].capacity;
  };
  const auto any = [](const std::uint32_t /*index*/) { return true; };
  for (std::uint32_t demand = 0; demand < demands.size(); ++demand) {
    const auto& wanted = demands[demand];
    std::optional<std::vector<std::uint32_t>> way;
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
          way = found->first;
          break;
        }
        check.tooLong[demand] = !withinCapacity;
      }
    } else {
      way = fewestEdges(chambers, edges, around, wanted, demand, free);
      if (!way.has_value()) {
        way = fewestEdges(chambers, edges, around, wanted, demand, any);
      }
    }
    if (way.has_value()) {
      for (const auto index : *way) {
        ++check.load[index];
      }
    }
    check.ways.push_back(std::move(way));
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
