/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#pragma once

#include "mqt-scpd/flatbuffers/artifacts.hpp"
#include "mqt-scpd/flatbuffers/config.hpp"
#include "mqt-scpd/flatbuffers/design.hpp"
#include "mqt-scpd/pipeline/mqt_scpd_pipeline_export.hpp"

namespace mqt::scpd::pipeline {

using flatbuffers::artifacts::AssignmentT;
using flatbuffers::artifacts::CapacityPlanT;
using flatbuffers::artifacts::CorridorRoutingT;
using flatbuffers::artifacts::GlobalRoutingT;
using flatbuffers::config::ConfigT;
using flatbuffers::design::ChipT;

/// Every stage has the same shape, and the shape is contractual.
///
/// `run` is `const` and takes its inputs by `const&`, so a stage can neither
/// change its own state between runs nor reach into another stage's. It
/// returns a new value rather than writing into a shared design database.
/// Given the same inputs it produces the same output, which is what makes a
/// resumed run equal to an uninterrupted one and what the later threading
/// work needs.
///
/// The prototype held none of the three: it passed the chip by value into
/// three stages, had the assignment write into the capacity stage's request
/// queue while its own solver was running, and depended on hash iteration
/// order in several places.

/// Partitions the free space and budgets how many wires may cross between
/// the partitions.
class MQT_SCPD_PIPELINE_EXPORT ICapacityPlanner {
public:
  ICapacityPlanner() = default;
  ICapacityPlanner(const ICapacityPlanner&) = delete;
  ICapacityPlanner& operator=(const ICapacityPlanner&) = delete;
  ICapacityPlanner(ICapacityPlanner&&) = delete;
  ICapacityPlanner& operator=(ICapacityPlanner&&) = delete;
  virtual ~ICapacityPlanner() = default;

  [[nodiscard]] virtual CapacityPlanT run(const ChipT& chip,
                                          const ConfigT& config) const = 0;
};

/// Solves the inner circuit on the lattice each capacity chain induces, and
/// reports the outer port ring the assignment then consumes.
///
/// This runs **before** the assignment, not after it. The inner circuit
/// decides which outer ports the inner wires surface at, and the ring the
/// assignment works on is the configured one extended by exactly those. The
/// prototype's own drivers run the two stages in this order for that reason.
class MQT_SCPD_PIPELINE_EXPORT IGlobalRouter {
public:
  IGlobalRouter() = default;
  IGlobalRouter(const IGlobalRouter&) = delete;
  IGlobalRouter& operator=(const IGlobalRouter&) = delete;
  IGlobalRouter(IGlobalRouter&&) = delete;
  IGlobalRouter& operator=(IGlobalRouter&&) = delete;
  virtual ~IGlobalRouter() = default;

  [[nodiscard]] virtual GlobalRoutingT run(const ChipT& chip,
                                           const CapacityPlanT& capacity,
                                           const ConfigT& config) const = 0;
};

/// Assigns resonators to launchers as a minimum-overlap problem on the ring
/// of outer ports.
class MQT_SCPD_PIPELINE_EXPORT IAssigner {
public:
  IAssigner() = default;
  IAssigner(const IAssigner&) = delete;
  IAssigner& operator=(const IAssigner&) = delete;
  IAssigner(IAssigner&&) = delete;
  IAssigner& operator=(IAssigner&&) = delete;
  virtual ~IAssigner() = default;

  [[nodiscard]] virtual AssignmentT run(const ChipT& chip,
                                        const CapacityPlanT& capacity,
                                        const GlobalRoutingT& global,
                                        const ConfigT& config) const = 0;
};

/// Routes every assigned connection through the partitions, coarsely.
///
/// This is where a wire is told which corridors it runs through and where it
/// crosses from one into the next, and it is the stage that makes the wire
/// budgets binding: a border is crossed at one of a fixed set of slots, one
/// wire spacing apart, and no two wires take the same one. Inside a
/// partition two wires may not cross each other either, so what comes out is
/// a plan the detail router can follow without unpicking it again.
///
/// Only the connections of the assignment are routed here. The inner circuit
/// paid for the free space it crosses while it was solved, so it goes to the
/// detail router directly.
class MQT_SCPD_PIPELINE_EXPORT ICorridorRouter {
public:
  ICorridorRouter() = default;
  ICorridorRouter(const ICorridorRouter&) = delete;
  ICorridorRouter& operator=(const ICorridorRouter&) = delete;
  ICorridorRouter(ICorridorRouter&&) = delete;
  ICorridorRouter& operator=(ICorridorRouter&&) = delete;
  virtual ~ICorridorRouter() = default;

  [[nodiscard]] virtual CorridorRoutingT run(const ChipT& chip,
                                             const CapacityPlanT& capacity,
                                             const AssignmentT& assignment,
                                             const ConfigT& config) const = 0;
};

} // namespace mqt::scpd::pipeline
