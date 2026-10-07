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
#include <optional>
#include <queue>
#include <stdexcept>
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
    for (const auto& side : {demands[index].from, demands[index].to}) {
      for (const auto chamber : side) {
        if (chamber >= chambers) {
          throw std::invalid_argument(std::format(
              "demand {} names chamber {} of {}", index, chamber, chambers));
        }
      }
    }
  }

  std::vector<std::vector<std::uint32_t>> around(chambers);
  for (std::uint32_t index = 0; index < edges.size(); ++index) {
    for (const auto chamber : edges[index].chambers) {
      around[chamber].push_back(index);
    }
  }

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

} // namespace mqt::scpd::pipeline
