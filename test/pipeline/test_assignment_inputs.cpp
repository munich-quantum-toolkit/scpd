/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

// The precomputed assignment inputs. In the prototype these lookups ran
// inside the model builder, against a live capacity grid that the solve then
// mutated; that is what made the model inseparable from the geometry engine.
// Here they are a plain value, and these tests are what says so.

#include "mqt-scpd/flatbuffers/artifacts.hpp"
#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/pipeline/Assigner.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace mqt::scpd::pipeline {
namespace {

namespace fba = flatbuffers::artifacts;
namespace fbd = flatbuffers::design;

/// A chip whose ports sit on a line, so the nearest launcher is obvious.
ChipT lineOfPorts() {
  ChipT chip;
  for (int index = 0; index < 4; ++index) {
    auto port = std::make_unique<fbd::PortT>();
    port->label = "p" + std::to_string(index);
    port->center = {static_cast<double>(index) * 100.0, 0.0};
    port->role = index == 0 ? fbd::UnassignedRole::Launcher : fbd::UnassignedRole::Resonator;
    chip.ports.push_back(std::move(port));
  }
  return chip;
}

CapacityPlanT planWithLaunchersAt(const std::vector<double>& positions) {
  CapacityPlanT plan;
  for (std::size_t index = 0; index < positions.size(); ++index) {
    auto slot = std::make_unique<fba::LauncherSlotT>();
    slot->port = fbd::PortRef(static_cast<std::uint32_t>(index));
    slot->position = {positions[index], 0.0};
    plan.launchers.push_back(std::move(slot));
  }
  return plan;
}

GlobalRoutingT ringOf(const std::vector<std::uint32_t>& ring,
                      const std::vector<std::uint32_t>& resonators) {
  GlobalRoutingT routing;
  for (const auto port : ring) {
    routing.outer_ring.emplace_back(port);
  }
  for (const auto port : resonators) {
    routing.resonators.emplace_back(port);
  }
  return routing;
}

TEST(AssignmentInputs, TakeTheRingFromTheGlobalStageInItsOrder) {
  const auto chip = lineOfPorts();
  const auto inputs =
      assignmentInputs(chip, planWithLaunchersAt({0.0}), ringOf({3, 1, 2}, {1, 3}));

  EXPECT_EQ(inputs.ring, (std::vector<std::uint32_t>{3, 1, 2}));
  // The ring the assignment consumes is the one the inner circuit produced,
  // not the configured one, because the inner circuit decides which outer
  // ports carry a wire at all.
  EXPECT_EQ(inputs.isResonator, (std::vector<bool>{true, true, false}));
}

TEST(AssignmentInputs, PrecomputeTheNearestLauncherOfEveryRingNode) {
  const auto chip = lineOfPorts();
  // Two launchers, one at each end of the line of ports.
  const auto inputs =
      assignmentInputs(chip, planWithLaunchersAt({-50.0, 350.0}), ringOf({1, 2, 3}, {1}));

  ASSERT_EQ(inputs.nearestLauncher.size(), 3U);
  EXPECT_EQ(inputs.nearestLauncher[0], 0U);
  EXPECT_EQ(inputs.nearestLauncher[2], 1U);
}

TEST(AssignmentInputs, RefuseAPlanWithoutALauncherAndARingWithoutAPort) {
  const auto chip = lineOfPorts();
  EXPECT_THROW(static_cast<void>(assignmentInputs(chip, {}, ringOf({1}, {1}))),
               std::invalid_argument);
  EXPECT_THROW(
      static_cast<void>(assignmentInputs(chip, planWithLaunchersAt({0.0}), ringOf({}, {}))),
      std::invalid_argument);
}

} // namespace
} // namespace mqt::scpd::pipeline
