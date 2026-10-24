/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/io/Artifacts.hpp"

#include "mqt-scpd/design/Validation.hpp"
#include "mqt-scpd/flatbuffers/artifacts.hpp"
#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/flatbuffers/geometry.hpp"

#include <flatbuffers/flatbuffer_builder.h>
#include <flatbuffers/minireflect.h>
#include <flatbuffers/verifier.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <format>
#include <iterator>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace mqt::scpd::io {

namespace {

/// A count and its noun, singular only for a count of one: "1 crossing",
/// "2 crossings".
std::string counted(const std::size_t count, const std::string_view noun) {
  return std::format("{} {}{}", count, noun, count == 1 ? "" : "s");
}

using flatbuffers::artifacts::ArtifactT;
using flatbuffers::artifacts::StageOutput;
using flatbuffers::geometry::SegmentShape;

void append(Problems& into, const Problems& from, const std::string& prefix) {
  for (const auto& problem : from) {
    into.push_back(prefix + problem);
  }
}

template <typename ComponentT>
void validateEach(const std::vector<std::unique_ptr<ComponentT>>& components,
                  const std::string& what, Problems& problems) {
  for (std::size_t i = 0; i < components.size(); ++i) {
    const std::string prefix = what + " " + std::to_string(i) + ": ";
    if (components[i] == nullptr) {
      problems.push_back(prefix + "missing");
      continue;
    }
    append(problems, design::validate(*components[i]), prefix);
  }
}

/// Writing an artifact packs every entry of a vector of tables, so a missing
/// entry is a problem even where no other check reads the vector.
template <typename TableT>
void requireEach(const std::vector<std::unique_ptr<TableT>>& entries,
                 const std::string_view what, Problems& problems) {
  for (std::size_t i = 0; i < entries.size(); ++i) {
    if (entries[i] == nullptr) {
      problems.push_back(std::format("{} {}: missing", what, i));
    }
  }
}

void validateWires(
    const std::vector<std::unique_ptr<flatbuffers::artifacts::WireT>>& wires,
    Problems& problems) {
  for (std::size_t i = 0; i < wires.size(); ++i) {
    const std::string prefix = "wire " + std::to_string(i) + ": ";
    if (wires[i] == nullptr || wires[i]->path == nullptr) {
      problems.push_back(prefix + "path is missing");
      continue;
    }
    const auto& segments = wires[i]->path->segments;
    if (segments.empty()) {
      problems.push_back(prefix + "path has no segments");
    }
    for (std::size_t j = 0; j < segments.size(); ++j) {
      const std::string segment = prefix + "segment " + std::to_string(j) + " ";
      if (segments[j] == nullptr ||
          segments[j]->shape.type == SegmentShape::NONE) {
        problems.push_back(segment + "has no shape");
        continue;
      }
      if (const auto* const arc = segments[j]->shape.AsArc()) {
        if (!(arc->radius > 0.0)) {
          problems.push_back(segment + "radius must be positive");
        }
        if (arc->sweep == 0.0) {
          problems.push_back(segment + "sweep must not be zero");
        }
      }
    }
  }
}

/// The capacity chains are trees stored flat, and the later stages walk them
/// by index. Every index has to name a node, and no walk may come back to a
/// node it left, or the walk would not end. A target lies on one chain only,
/// because the Global stage asks one wire of each target over its chain.
void validateCapacityPlan(const flatbuffers::artifacts::CapacityPlanT& plan,
                          Problems& problems) {
  if (plan.capacity_grid == nullptr) {
    problems.emplace_back("capacity grid: missing");
  }
  if (plan.detail_grid == nullptr) {
    problems.emplace_back("detail grid: missing");
  }
  requireEach(plan.partitions, "partition", problems);
  for (std::size_t index = 0; index < plan.partitions.size(); ++index) {
    if (plan.partitions[index] != nullptr) {
      requireEach(plan.partitions[index]->outlines,
                  std::format("partition {} outline", index), problems);
    }
  }
  requireEach(plan.borders, "border", problems);
  requireEach(plan.bottlenecks, "bottleneck", problems);
  requireEach(plan.launchers, "launcher slot", problems);
  requireEach(plan.port_keepout, "port keepout", problems);
  const auto count = plan.nodes.size();
  bool linked = true;
  for (std::size_t index = 0; index < count; ++index) {
    if (plan.nodes[index] == nullptr) {
      problems.push_back(std::format("capacity node {}: missing", index));
      linked = false;
      continue;
    }
    const auto& node = *plan.nodes[index];
    for (const auto next : node.next) {
      if (next >= count) {
        problems.push_back(std::format(
            "capacity node {} links to node {} of {}", index, next, count));
        linked = false;
      }
    }
    if (node.kind == flatbuffers::artifacts::CapacityElement::Bottleneck) {
      // The Global stage reads the capacity of a gate from its node, and a
      // gate on several chains has a node on each, so every node of a
      // bottleneck has to say what the bottleneck says.
      if (node.id >= plan.bottlenecks.size()) {
        problems.push_back(
            std::format("capacity node {} names bottleneck {} of {}", index,
                        node.id, plan.bottlenecks.size()));
      } else if (plan.bottlenecks[node.id] != nullptr &&
                 node.capacity != plan.bottlenecks[node.id]->capacity) {
        problems.push_back(std::format(
            "capacity node {} lets {} through bottleneck {}, which lets {} "
            "through",
            index, counted(node.capacity, "wire"), node.id,
            counted(plan.bottlenecks[node.id]->capacity, "wire")));
      }
    }
  }
  for (std::size_t index = 0; index < plan.chains.size(); ++index) {
    if (plan.chains[index] >= count) {
      problems.push_back(
          std::format("capacity chain {} starts at node {} of {}", index,
                      plan.chains[index], count));
      linked = false;
    }
  }
  if (!linked) {
    return;
  }
  // Kahn's order: the nodes that no link reaches go first, and a node follows
  // once every link to it is taken. Nodes left over lie on a cycle.
  std::vector<std::size_t> incoming(count, 0);
  for (const auto& node : plan.nodes) {
    for (const auto next : node->next) {
      ++incoming[next];
    }
  }
  std::vector<std::size_t> ready;
  for (std::size_t index = 0; index < count; ++index) {
    if (incoming[index] == 0) {
      ready.push_back(index);
    }
  }
  std::size_t ordered = 0;
  while (!ready.empty()) {
    const auto index = ready.back();
    ready.pop_back();
    ++ordered;
    for (const auto next : plan.nodes[index]->next) {
      if (--incoming[next] == 0) {
        ready.push_back(next);
      }
    }
  }
  if (ordered != count) {
    problems.push_back(std::format(
        "capacity nodes link in a cycle: {} of {} {} on or behind one",
        count - ordered, counted(count, "node"),
        count - ordered == 1 ? "lies" : "lie"));
    return;
  }
  // A chain is a tree. Two chains may share a subtree without a target, but
  // one chain that reaches a node over two paths would count the subtree
  // behind it twice.
  std::unordered_map<std::uint32_t, std::size_t> chainOfTarget;
  for (std::size_t chain = 0; chain < plan.chains.size(); ++chain) {
    std::vector<bool> reached(count, false);
    std::vector<std::size_t> pending{plan.chains[chain]};
    reached[plan.chains[chain]] = true;
    while (!pending.empty()) {
      const auto index = pending.back();
      pending.pop_back();
      const auto& node = *plan.nodes[index];
      if (node.kind == flatbuffers::artifacts::CapacityElement::Target) {
        const auto [first, fresh] = chainOfTarget.emplace(node.id, chain);
        if (!fresh) {
          problems.push_back(
              first->second == chain
                  ? std::format("capacity chain {} reaches port {} twice",
                                chain, node.id)
                  : std::format("capacity chains {} and {} both reach port {}",
                                first->second, chain, node.id));
          return;
        }
      }
      for (const auto next : node.next) {
        if (reached[next]) {
          problems.push_back(std::format(
              "capacity chain {} reaches node {} twice", chain, next));
          return;
        }
        reached[next] = true;
        pending.push_back(next);
      }
    }
  }
}

void validateDetailWires(
    const std::vector<std::unique_ptr<flatbuffers::artifacts::DetailWireT>>&
        wires,
    const std::string& what, Problems& problems) {
  for (std::size_t index = 0; index < wires.size(); ++index) {
    if (wires[index] == nullptr) {
      problems.push_back(std::format("{} {}: missing", what, index));
      continue;
    }
    const auto& path = wires[index]->path;
    for (std::size_t step = 1; step < path.size(); ++step) {
      const auto& before = path[step - 1];
      const auto& after = path[step];
      const auto dx =
          std::max(before.x(), after.x()) - std::min(before.x(), after.x());
      const auto dy =
          std::max(before.y(), after.y()) - std::min(before.y(), after.y());
      if (dx > 1 || dy > 1) {
        problems.push_back(std::format("{} {} jumps from ({}, {}) to ({}, {})",
                                       what, index, before.x(), before.y(),
                                       after.x(), after.y()));
      }
    }
  }
}

void validateFinalWires(
    const std::vector<std::unique_ptr<flatbuffers::artifacts::FinalWireT>>&
        wires,
    const std::string& what, Problems& problems) {
  for (std::size_t index = 0; index < wires.size(); ++index) {
    if (wires[index] == nullptr) {
      problems.push_back(std::format("{} {}: missing", what, index));
      continue;
    }
    const auto& path = wires[index]->path;
    for (std::size_t step = 0; step < path.size(); ++step) {
      if (path[step].heading() >= 8) {
        problems.push_back(std::format("{} {} holds the heading {} at step {}",
                                       what, index, path[step].heading(),
                                       step));
      }
      if (step == 0) {
        continue;
      }
      const auto& before = path[step - 1];
      const auto& after = path[step];
      const auto dx =
          std::max(before.x(), after.x()) - std::min(before.x(), after.x());
      const auto dy =
          std::max(before.y(), after.y()) - std::min(before.y(), after.y());
      if (dx > 1 || dy > 1) {
        problems.push_back(std::format("{} {} jumps from ({}, {}) to ({}, {})",
                                       what, index, before.x(), before.y(),
                                       after.x(), after.y()));
      }
    }
  }
}

std::string join(const Problems& problems) {
  std::string joined;
  for (const auto& problem : problems) {
    joined += joined.empty() ? "" : "; ";
    joined += problem;
  }
  return joined;
}

} // namespace

Problems validate(const ArtifactT& artifact) {
  Problems problems;
  if (artifact.producer.empty()) {
    problems.emplace_back("producer is empty");
  }
  // The verifier lets an output type pass that a newer schema appended, and
  // the object model then holds no output to check.
  if (artifact.output.type > StageOutput::MAX) {
    problems.push_back(
        std::format("output type {} is unknown to this version of mqt-scpd",
                    static_cast<unsigned>(artifact.output.type)));
    return problems;
  }
  switch (artifact.output.type) {
  case StageOutput::NONE:
    problems.emplace_back("output is missing");
    break;
  case StageOutput::Assignment: {
    const auto& assignment = *artifact.output.AsAssignment();
    validateEach(assignment.connections, "connection", problems);
    // The connections, the launchers and the feeds are each one per ring
    // node, in the order of the ring.
    const auto parallel = [&](const std::size_t size,
                              const std::string_view what) {
      if (size != assignment.ring.size()) {
        problems.push_back(
            std::format("{} for {}", counted(size, what),
                        counted(assignment.ring.size(), "ring node")));
      }
    };
    parallel(assignment.connections.size(), "connection");
    parallel(assignment.launchers.size(), "launcher");
    parallel(assignment.feeds.size(), "feed");
    for (std::size_t index = 0; index < std::min(assignment.connections.size(),
                                                 assignment.ring.size());
         ++index) {
      const auto& connection = assignment.connections[index];
      if (connection != nullptr &&
          connection->target.index() != assignment.ring[index].index()) {
        problems.push_back(std::format(
            "connection {} targets port {}, not port {} of ring node {}", index,
            connection->target.index(), assignment.ring[index].index(), index));
      }
    }
    // A chain runs through ring nodes, each of them a node of the ring, and
    // one feedline drives each node of a chain.
    std::vector<std::optional<std::size_t>> chainOf(assignment.ring.size());
    for (std::size_t index = 0; index < assignment.chains.size(); ++index) {
      if (assignment.chains[index] == nullptr) {
        problems.push_back(std::format("chain {}: missing", index));
        continue;
      }
      if (assignment.chains[index]->nodes.empty()) {
        problems.push_back(
            std::format("chain {} runs through no ring node", index));
      }
      for (const auto node : assignment.chains[index]->nodes) {
        if (node >= assignment.ring.size()) {
          problems.push_back(std::format("chain {} names ring node {} of {}",
                                         index, node, assignment.ring.size()));
          continue;
        }
        if (chainOf[node].has_value()) {
          problems.push_back(
              *chainOf[node] == index
                  ? std::format("chain {} names ring node {} twice", index,
                                node)
                  : std::format("chains {} and {} both name ring node {}",
                                *chainOf[node], index, node));
          continue;
        }
        chainOf[node] = index;
      }
    }
    break;
  }
  case StageOutput::FinalRouting: {
    // A wire of the router grid is the cells it runs over with the heading it
    // held in each, so two consecutive cells differ by at most one along each
    // axis and every heading names one of the eight directions.
    const auto& routing = *artifact.output.AsFinalRouting();
    if (routing.grid == nullptr) {
      problems.emplace_back("grid is missing");
    }
    validateEach(routing.couplers, "coupler", problems);
    validateEach(routing.bridges, "bridge", problems);
    validateFinalWires(routing.wires, "wire", problems);
    validateFinalWires(routing.inner, "inner wire", problems);
    validateFinalWires(routing.feedlines, "feedline", problems);
    if (routing.feedline_edges.size() != routing.feedlines.size()) {
      problems.push_back(std::format("{} feedline edges describe {} feedlines",
                                     routing.feedline_edges.size(),
                                     routing.feedlines.size()));
    }
    for (std::size_t index = 0; index < routing.phases.size(); ++index) {
      if (routing.phases[index] == nullptr) {
        problems.push_back(std::format("phase {}: missing", index));
        continue;
      }
      const auto& phase = *routing.phases[index];
      if (phase.name.empty()) {
        problems.push_back(std::format("phase {} has no name", index));
      }
      validateFinalWires(phase.wires, phase.name + " wire", problems);
      validateFinalWires(phase.inner, phase.name + " inner wire", problems);
      validateFinalWires(phase.feedlines, phase.name + " feedline", problems);
    }
    break;
  }
  case StageOutput::CorridorRouting: {
    // A crossing carries the wire from one partition into the next, so a
    // corridor names one more partition than it has crossings. A corridor
    // without partitions is a connection the stage could not route.
    const auto& routing = *artifact.output.AsCorridorRouting();
    requireEach(routing.slots, "border slots", problems);
    for (std::size_t index = 0; index < routing.corridors.size(); ++index) {
      if (routing.corridors[index] == nullptr) {
        problems.push_back(std::format("corridor {}: missing", index));
        continue;
      }
      const auto& corridor = *routing.corridors[index];
      if ((!corridor.partitions.empty() || !corridor.crossings.empty()) &&
          corridor.crossings.size() + 1 != corridor.partitions.size()) {
        problems.push_back(
            std::format("corridor {} names {} and {}", index,
                        counted(corridor.partitions.size(), "partition"),
                        counted(corridor.crossings.size(), "crossing")));
      }
      // A way runs from the feed to the target, so a routed corridor has
      // both.
      if (!corridor.partitions.empty() && corridor.source == nullptr) {
        problems.push_back(
            std::format("corridor {} has a way but no source", index));
      }
      if (!corridor.partitions.empty() && corridor.target == nullptr) {
        problems.push_back(
            std::format("corridor {} has a way but no target", index));
      }
    }
    break;
  }
  case StageOutput::CapacityPlan:
    validateCapacityPlan(*artifact.output.AsCapacityPlan(), problems);
    break;
  case StageOutput::GlobalRouting: {
    const auto& routing = *artifact.output.AsGlobalRouting();
    validateEach(routing.connections, "connection", problems);
    // A picture draws a lattice by index, so every edge has two points of its
    // lattice and every selected edge is one of its edges.
    for (std::size_t index = 0; index < routing.lattices.size(); ++index) {
      if (routing.lattices[index] == nullptr) {
        problems.push_back(std::format("lattice {}: missing", index));
        continue;
      }
      const auto& lattice = *routing.lattices[index];
      if (lattice.edges.size() % 2 != 0) {
        problems.push_back(
            std::format("lattice {} holds {} edge indices, not two per edge",
                        index, lattice.edges.size()));
      }
      for (const auto point : lattice.edges) {
        if (point >= lattice.points.size()) {
          problems.push_back(std::format("lattice {} names point {} of {}",
                                         index, point, lattice.points.size()));
          break;
        }
      }
      const auto edges = lattice.edges.size() / 2;
      for (const auto edge : lattice.selected) {
        if (edge >= edges) {
          problems.push_back(std::format("lattice {} selects edge {} of {}",
                                         index, edge, edges));
          break;
        }
      }
    }
    break;
  }
  case StageOutput::DetailRouting: {
    // A wire is drawn as the cells it runs over, so two consecutive cells
    // differ by at most one along each axis. A path that steps further is not
    // a way a wire could take, and nothing downstream can tell which of the
    // two ends the gap belongs to.
    const auto& routing = *artifact.output.AsDetailRouting();
    if (routing.grid == nullptr) {
      problems.emplace_back("grid is missing");
    }
    validateDetailWires(routing.wires, "wire", problems);
    validateDetailWires(routing.inner, "inner wire", problems);
    break;
  }
  }
  return problems;
}

Problems validate(const ArtifactT& artifact,
                  const flatbuffers::design::ChipT& chip) {
  using flatbuffers::design::ConnectionT;
  using flatbuffers::design::PortRef;
  const auto count = chip.ports.size();
  Problems problems;
  const auto check = [&](const std::uint32_t port, const std::string& where) {
    if (port >= count) {
      problems.push_back(
          std::format("{} names port {} of {}", where, port, count));
    }
  };
  const auto checkAll = [&](const std::vector<PortRef>& ports,
                            const std::string_view what) {
    for (std::size_t index = 0; index < ports.size(); ++index) {
      check(ports[index].index(), std::format("{} {}", what, index));
    }
  };
  const auto checkConnections =
      [&](const std::vector<std::unique_ptr<ConnectionT>>& connections) {
        for (std::size_t index = 0; index < connections.size(); ++index) {
          if (connections[index] == nullptr) {
            continue;
          }
          const auto& connection = *connections[index];
          if (connection.source != nullptr) {
            check(connection.source->index(),
                  std::format("connection {} source", index));
          }
          check(connection.target.index(),
                std::format("connection {} target", index));
        }
      };

  switch (artifact.output.type) {
  case StageOutput::CapacityPlan: {
    const auto& plan = *artifact.output.AsCapacityPlan();
    for (std::size_t index = 0; index < plan.launchers.size(); ++index) {
      if (plan.launchers[index] != nullptr) {
        check(plan.launchers[index]->port.index(),
              std::format("launcher slot {}", index));
      }
    }
    for (std::size_t index = 0; index < plan.nodes.size(); ++index) {
      if (plan.nodes[index] != nullptr &&
          plan.nodes[index]->kind ==
              flatbuffers::artifacts::CapacityElement::Target) {
        check(plan.nodes[index]->id, std::format("capacity node {}", index));
      }
    }
    break;
  }
  case StageOutput::GlobalRouting: {
    const auto& routing = *artifact.output.AsGlobalRouting();
    checkConnections(routing.connections);
    checkAll(routing.outer_ring, "ring node");
    checkAll(routing.resonators, "resonator");
    break;
  }
  case StageOutput::Assignment: {
    const auto& assignment = *artifact.output.AsAssignment();
    checkConnections(assignment.connections);
    checkAll(assignment.ring, "ring node");
    checkAll(assignment.launchers, "launcher");
    for (std::size_t index = 0; index < assignment.chains.size(); ++index) {
      if (assignment.chains[index] == nullptr) {
        continue;
      }
      const auto& chain = *assignment.chains[index];
      for (const auto* end : {chain.start.get(), chain.end.get()}) {
        if (end != nullptr) {
          check(end->index(), std::format("chain {}", index));
        }
      }
    }
    break;
  }
  case StageOutput::FinalRouting:
  case StageOutput::Geometry:
  case StageOutput::CorridorRouting:
  case StageOutput::DetailRouting:
  case StageOutput::NONE:
    break;
  }
  return problems;
}

std::vector<std::uint8_t> writeArtifact(const ArtifactT& artifact) {
  if (const auto problems = validate(artifact); !problems.empty()) {
    throw std::invalid_argument("artifact is not valid: " + join(problems));
  }
  ::flatbuffers::FlatBufferBuilder builder;
  FinishArtifactBuffer(
      builder, flatbuffers::artifacts::Artifact::Pack(builder, &artifact));
  const auto* const begin = builder.GetBufferPointer();
  return {begin, std::next(begin, builder.GetSize())};
}

ArtifactT readArtifact(const std::span<const std::uint8_t> bytes) {
  ::flatbuffers::Verifier verifier(bytes.data(), bytes.size());
  if (!flatbuffers::artifacts::VerifyArtifactBuffer(verifier)) {
    throw std::invalid_argument(
        "bytes are not an artifact of this schema: verification failed or the "
        "identifier is not " +
        std::string(flatbuffers::artifacts::ArtifactIdentifier()));
  }
  ArtifactT artifact;
  flatbuffers::artifacts::GetArtifact(bytes.data())->UnPackTo(&artifact);
  if (const auto problems = validate(artifact); !problems.empty()) {
    throw std::invalid_argument("artifact is not valid: " + join(problems));
  }
  return artifact;
}

std::string artifactToJson(const std::span<const std::uint8_t> bytes) {
  // Reading the artifact first is what checks it; the rendering below walks
  // the buffer through the schema's type tables and cannot report a problem.
  static_cast<void>(readArtifact(bytes));
  return ::flatbuffers::FlatBufferToString(
             bytes.data(), flatbuffers::artifacts::ArtifactTypeTable(),
             /*multi_line=*/true, /*vector_delimited=*/true, /*indent=*/"  ",
             /*quotes=*/true) +
         "\n";
}

} // namespace mqt::scpd::io
