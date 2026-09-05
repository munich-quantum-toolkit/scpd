/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/design/Validation.hpp"

#include "mqt-scpd/flatbuffers/design.hpp"

#include <cstddef>
#include <string>

namespace mqt::scpd::design {

namespace {

using flatbuffers::design::AssignedRole;
using flatbuffers::design::Rotation;
using flatbuffers::design::UnassignedRole;

void requirePositive(const double value, const std::string& what,
                     Problems& problems) {
  if (!(value > 0.0)) {
    problems.push_back(what + " must be positive");
  }
}

void requireRotation(const Rotation rotation, Problems& problems) {
  if (rotation == Rotation::Unset) {
    problems.emplace_back("rotation is unset");
  }
}

void append(Problems& into, const Problems& from, const std::string& prefix) {
  for (const auto& problem : from) {
    into.push_back(prefix + problem);
  }
}

} // namespace

Problems validate(const flatbuffers::design::PortT& port) {
  Problems problems;
  if (port.label.empty()) {
    problems.emplace_back("label is empty");
  }
  if (port.role == UnassignedRole::Unset) {
    problems.emplace_back("role is unset");
  }
  return problems;
}

Problems validate(const flatbuffers::design::ChipT& chip) {
  Problems problems;
  for (std::size_t i = 0; i < chip.obstacles.size(); ++i) {
    const auto* const obstacle = chip.obstacles[i].get();
    if (obstacle == nullptr || obstacle->vertices.size() < 3) {
      problems.push_back("obstacle " + std::to_string(i) +
                         " has fewer than three vertices");
    }
  }
  for (std::size_t i = 0; i < chip.ports.size(); ++i) {
    const std::string prefix = "port " + std::to_string(i) + ": ";
    const auto* const port = chip.ports[i].get();
    if (port == nullptr) {
      problems.push_back(prefix + "missing");
      continue;
    }
    append(problems, validate(*port), prefix);
  }
  return problems;
}

Problems validate(const flatbuffers::design::ConnectionT& connection) {
  Problems problems;
  if (connection.source_role == AssignedRole::Unset) {
    problems.emplace_back("source role is unset");
  }
  if (connection.target_role == AssignedRole::Unset) {
    problems.emplace_back("target role is unset");
  }
  return problems;
}

Problems validate(const flatbuffers::design::DesignRulesT& rules) {
  Problems problems;
  requirePositive(rules.min_wire_spacing, "min_wire_spacing", problems);
  requirePositive(rules.min_obstacle_spacing, "min_obstacle_spacing", problems);
  requirePositive(rules.min_bend_radius, "min_bend_radius", problems);
  requirePositive(rules.min_straight_length, "min_straight_length", problems);
  requirePositive(rules.target_resonator_length, "target_resonator_length",
                  problems);
  requirePositive(rules.resonator_length_tolerance,
                  "resonator_length_tolerance", problems);
  if (rules.max_feedline_utilization == 0) {
    problems.emplace_back("max_feedline_utilization must be at least one");
  }
  return problems;
}

Problems validate(const flatbuffers::design::CpwCouplerT& coupler) {
  Problems problems;
  requireRotation(coupler.rotation, problems);
  requirePositive(coupler.length, "length", problems);
  requirePositive(coupler.height, "height", problems);
  if (coupler.port == nullptr) {
    problems.emplace_back("port is missing");
    return problems;
  }
  if (coupler.port->role != UnassignedRole::Coupler) {
    problems.emplace_back("port role must be Coupler");
  }
  append(problems, validate(*coupler.port), "port: ");
  return problems;
}

Problems validate(const flatbuffers::design::BridgeT& bridge) {
  Problems problems;
  requireRotation(bridge.rotation, problems);
  requirePositive(bridge.width, "width", problems);
  requirePositive(bridge.height, "height", problems);
  return problems;
}

} // namespace mqt::scpd::design
