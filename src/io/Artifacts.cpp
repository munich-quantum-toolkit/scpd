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
#include "mqt-scpd/flatbuffers/geometry.hpp"

#include <flatbuffers/flatbuffer_builder.h>
#include <flatbuffers/verifier.h>

#include <cstddef>
#include <cstdint>
#include <iterator>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
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
  case StageOutput::Assignment:
    validateEach(artifact.output.AsAssignment()->connections, "connection",
                 problems);
    break;
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
  case StageOutput::CapacityPlan:
  case StageOutput::GlobalRouting:
  case StageOutput::DetailRouting:
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

} // namespace mqt::scpd::io
