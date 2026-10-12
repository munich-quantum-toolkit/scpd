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

#include <cstddef>
#include <cstdint>
#include <format>
#include <iterator>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace mqt::scpd::io {

namespace {

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
/// node it left, or the walk would not end.
void validateCapacityPlan(const flatbuffers::artifacts::CapacityPlanT& plan,
                          Problems& problems) {
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
    if (node.kind == flatbuffers::artifacts::CapacityElement::Bottleneck &&
        node.id >= plan.bottlenecks.size()) {
      problems.push_back(
          std::format("capacity node {} names bottleneck {} of {}", index,
                      node.id, plan.bottlenecks.size()));
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
        "capacity nodes link in a cycle: {} of {} nodes lie on or behind one",
        count - ordered, count));
    return;
  }
  // A chain is a tree. Two chains may share a subtree, but one chain that
  // reaches a node over two paths would count the subtree behind it twice.
  for (std::size_t chain = 0; chain < plan.chains.size(); ++chain) {
    std::vector<bool> reached(count, false);
    std::vector<std::size_t> pending{plan.chains[chain]};
    reached[plan.chains[chain]] = true;
    while (!pending.empty()) {
      const auto index = pending.back();
      pending.pop_back();
      for (const auto next : plan.nodes[index]->next) {
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
  switch (artifact.output.type) {
  case StageOutput::NONE:
    problems.emplace_back("output is missing");
    break;
  case StageOutput::Assignment: {
    const auto& assignment = *artifact.output.AsAssignment();
    validateEach(assignment.connections, "connection", problems);
    // A chain names ring nodes, and each of them has to be a node of the ring.
    for (std::size_t index = 0; index < assignment.chains.size(); ++index) {
      if (assignment.chains[index] == nullptr) {
        problems.push_back(std::format("chain {}: missing", index));
        continue;
      }
      for (const auto node : assignment.chains[index]->nodes) {
        if (node >= assignment.ring.size()) {
          problems.push_back(std::format("chain {} names ring node {} of {}",
                                         index, node, assignment.ring.size()));
        }
      }
    }
    break;
  }
  case StageOutput::FinalRouting: {
    const auto& routing = *artifact.output.AsFinalRouting();
    validateEach(routing.couplers, "coupler", problems);
    validateEach(routing.bridges, "bridge", problems);
    break;
  }
  case StageOutput::Geometry: {
    const auto& geometry = *artifact.output.AsGeometry();
    validateWires(geometry.wires, problems);
    validateEach(geometry.couplers, "coupler", problems);
    validateEach(geometry.bridges, "bridge", problems);
    break;
  }
  case StageOutput::CorridorRouting: {
    // A crossing carries the wire from one partition into the next, so a
    // corridor names one more partition than it has crossings. A corridor
    // without partitions is a connection the stage could not route.
    const auto& routing = *artifact.output.AsCorridorRouting();
    for (std::size_t index = 0; index < routing.corridors.size(); ++index) {
      if (routing.corridors[index] == nullptr) {
        problems.push_back(std::format("corridor {}: missing", index));
        continue;
      }
      const auto& corridor = *routing.corridors[index];
      if (!corridor.partitions.empty() &&
          corridor.crossings.size() + 1 != corridor.partitions.size()) {
        problems.push_back(std::format(
            "corridor {} names {} partitions and {} crossings", index,
            corridor.partitions.size(), corridor.crossings.size()));
      }
    }
    break;
  }
  case StageOutput::CapacityPlan:
    validateCapacityPlan(*artifact.output.AsCapacityPlan(), problems);
    break;
  case StageOutput::GlobalRouting:
  case StageOutput::DetailRouting:
    break;
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
