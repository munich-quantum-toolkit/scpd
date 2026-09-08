/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/io/Chip.hpp"

#include "mqt-scpd/design/Roles.hpp"
#include "mqt-scpd/design/Validation.hpp"
#include "mqt-scpd/flatbuffers/config.hpp"
#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/flatbuffers/geometry.hpp"

#include <flatbuffers/buffer.h>
#include <flatbuffers/flatbuffer_builder.h>
#include <flatbuffers/verifier.h>
#include <nlohmann/json.hpp>
#include <nlohmann/json_fwd.hpp>

#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <iterator>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace mqt::scpd::io {

namespace {

using design::Problems;
using flatbuffers::config::ConfigT;
using flatbuffers::design::Chip;
using flatbuffers::design::ChipT;
using flatbuffers::design::PortT;
using flatbuffers::design::UnassignedRole;
using flatbuffers::geometry::Point;
using flatbuffers::geometry::PolygonT;
// The prototype's JSON writer keeps the ports in insertion order. Keeping
// that order fixes the PortRef of every port to its position in the file.
using json = nlohmann::ordered_json;

/// A file with thousands of bad vertices should not produce a message with
/// thousands of lines.
constexpr std::size_t MAX_REPORTED_PROBLEMS = 20;

std::string join(const Problems& problems) {
  std::string joined;
  for (std::size_t i = 0; i < problems.size() && i < MAX_REPORTED_PROBLEMS;
       ++i) {
    joined += joined.empty() ? "" : "; ";
    joined += problems[i];
  }
  if (problems.size() > MAX_REPORTED_PROBLEMS) {
    joined += "; and " +
              std::to_string(problems.size() - MAX_REPORTED_PROBLEMS) + " more";
  }
  return joined;
}

void append(Problems& into, const Problems& from) {
  into.insert(into.end(), from.begin(), from.end());
}

void requireKeys(const json& object,
                 const std::initializer_list<std::string_view> allowed,
                 const std::string& where, Problems& problems) {
  for (const auto& [key, value] : object.items()) {
    bool known = false;
    for (const auto& name : allowed) {
      known = known || key == name;
    }
    if (!known) {
      std::string problem = where;
      problem += " has the unknown key '";
      problem += key;
      problem += "'";
      problems.push_back(std::move(problem));
    }
  }
}

bool readPoint(const json& value, Point& point) {
  if (!value.is_array() || value.size() != 2 || !value[0].is_number() ||
      !value[1].is_number()) {
    return false;
  }
  point = Point(value[0].get<double>(), value[1].get<double>());
  return true;
}

void readObstacles(const json& document, ChipT& chip, Problems& problems) {
  if (!document.contains("obstacles")) {
    problems.emplace_back("obstacles are missing");
    return;
  }
  const auto& obstacles = document["obstacles"];
  if (!obstacles.is_array()) {
    problems.emplace_back("obstacles is not an array");
    return;
  }
  chip.obstacles.reserve(obstacles.size());
  for (std::size_t i = 0; i < obstacles.size(); ++i) {
    const std::string where = "obstacle " + std::to_string(i);
    const auto& obstacle = obstacles[i];
    if (!obstacle.is_object()) {
      problems.push_back(where + " is not an object");
      continue;
    }
    requireKeys(obstacle, {"polygon"}, where, problems);
    if (!obstacle.contains("polygon") || !obstacle["polygon"].is_array()) {
      problems.push_back(where + " has no polygon array");
      continue;
    }
    auto polygon = std::make_unique<PolygonT>();
    const auto& vertices = obstacle["polygon"];
    polygon->vertices.reserve(vertices.size());
    for (std::size_t j = 0; j < vertices.size(); ++j) {
      Point point;
      if (!readPoint(vertices[j], point)) {
        problems.push_back(where + " vertex " + std::to_string(j) +
                           " is not a pair of numbers");
        continue;
      }
      polygon->vertices.push_back(point);
    }
    if (polygon->vertices.size() < 3) {
      problems.push_back(where + " has fewer than three vertices");
    }
    chip.obstacles.push_back(std::move(polygon));
  }
}

void readPorts(const json& document, ChipT& chip, Problems& problems) {
  if (!document.contains("ports")) {
    problems.emplace_back("ports are missing");
    return;
  }
  const auto& ports = document["ports"];
  if (!ports.is_object()) {
    problems.emplace_back("ports is not an object");
    return;
  }
  chip.ports.reserve(ports.size());
  for (const auto& [label, value] : ports.items()) {
    const std::string where = "port '" + label + "'";
    if (label.empty()) {
      problems.emplace_back("a port has an empty label");
      continue;
    }
    if (!value.is_object()) {
      problems.push_back(where + " is not an object");
      continue;
    }
    requireKeys(value, {"center", "orientation"}, where, problems);
    auto port = std::make_unique<PortT>();
    port->label = label;
    port->role = UnassignedRole::Unset;
    if (!value.contains("center") ||
        !readPoint(value["center"], port->center)) {
      problems.push_back(where + " has no center that is a pair of numbers");
      continue;
    }
    if (value.contains("orientation")) {
      if (!value["orientation"].is_number()) {
        problems.push_back(where + " has an orientation that is not a number");
        continue;
      }
      port->orientation = value["orientation"].get<double>();
    }
    chip.ports.push_back(std::move(port));
  }
}

} // namespace

ChipT readChipJson(const std::string_view text) {
  json document;
  try {
    document = json::parse(text);
  } catch (const json::parse_error& error) {
    throw std::invalid_argument(std::string("chip input is not JSON: ") +
                                error.what());
  }
  if (!document.is_object()) {
    throw std::invalid_argument("chip input is not a JSON object");
  }

  Problems problems;
  requireKeys(document, {"sampleSpacing", "nets", "obstacles", "ports"},
              "chip input", problems);
  if (document.contains("sampleSpacing") &&
      !document["sampleSpacing"].is_number()) {
    problems.emplace_back("sampleSpacing is not a number");
  }
  if (document.contains("nets")) {
    const auto& nets = document["nets"];
    if (!nets.is_array()) {
      problems.emplace_back("nets is not an array");
    } else if (!nets.empty()) {
      problems.push_back("nets is not empty (" + std::to_string(nets.size()) +
                         " entries); the loader reads no netlist");
    }
  }

  ChipT chip;
  readObstacles(document, chip, problems);
  readPorts(document, chip, problems);
  if (!problems.empty()) {
    throw std::invalid_argument("chip input is not valid: " + join(problems));
  }
  return chip;
}

ChipT loadChip(const std::string_view text, const ConfigT& config) {
  if (const auto problems = design::validate(config); !problems.empty()) {
    throw std::invalid_argument("configuration is not valid: " +
                                join(problems));
  }
  ChipT chip = readChipJson(text);
  Problems problems = design::classifyPorts(chip, *config.ports->patterns);
  if (problems.empty()) {
    append(problems, design::validate(config, chip));
  }
  if (!problems.empty()) {
    throw std::invalid_argument("chip does not fit the configuration: " +
                                join(problems));
  }
  return chip;
}

std::vector<std::uint8_t> writeChip(const ChipT& chip) {
  if (const auto problems = design::validate(chip); !problems.empty()) {
    throw std::invalid_argument("chip is not valid: " + join(problems));
  }
  ::flatbuffers::FlatBufferBuilder builder;
  builder.Finish(Chip::Pack(builder, &chip));
  const auto* const begin = builder.GetBufferPointer();
  return {begin, std::next(begin, builder.GetSize())};
}

ChipT readChip(const std::span<const std::uint8_t> bytes) {
  ::flatbuffers::Verifier verifier(bytes.data(), bytes.size());
  if (!verifier.VerifyBuffer<Chip>(nullptr)) {
    throw std::invalid_argument("bytes are not a chip buffer");
  }
  ChipT chip;
  ::flatbuffers::GetRoot<Chip>(bytes.data())->UnPackTo(&chip);
  if (const auto problems = design::validate(chip); !problems.empty()) {
    throw std::invalid_argument("chip is not valid: " + join(problems));
  }
  return chip;
}

} // namespace mqt::scpd::io
