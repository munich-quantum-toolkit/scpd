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
#include "mqt-scpd/pipeline/Report.hpp"
#include "mqt-scpd/pipeline/mqt_scpd_pipeline_export.hpp"

namespace mqt::scpd::pipeline {

using flatbuffers::artifacts::AssignmentT;
using flatbuffers::artifacts::CapacityPlanT;
using flatbuffers::artifacts::CorridorRoutingT;
using flatbuffers::artifacts::GlobalRoutingT;
using flatbuffers::config::ConfigT;
using flatbuffers::design::ChipT;

// Every stage has the same shape, and the shape is part of the contract.
// `run` is const and takes its inputs by const reference, so a stage can
// neither change its own state between runs nor reach into the state of
// another. It returns a new value rather than writing into a shared design
// database. Given the same inputs it produces the same output, which is what
// makes a resumed run equal to an uninterrupted one. A stage reports what it
// does through the Report it is given and never prints.

/**
 * @brief Partitions the free space of the chip and budgets how many wires may
 * cross between the partitions.
 */
class MQT_SCPD_PIPELINE_EXPORT ICapacityPlanner {
public:
  /**
   * @brief Creates a planner.
   */
  ICapacityPlanner() = default;
  ICapacityPlanner(const ICapacityPlanner&) = delete;
  ICapacityPlanner& operator=(const ICapacityPlanner&) = delete;
  ICapacityPlanner(ICapacityPlanner&&) = delete;
  ICapacityPlanner& operator=(ICapacityPlanner&&) = delete;
  virtual ~ICapacityPlanner() = default;

  /**
   * @brief Plans the capacity of a chip.
   * @param chip The classified chip.
   * @param config The run configuration.
   * @param report Where the stage reports what it does.
   * @return The capacity plan.
   */
  [[nodiscard]] virtual CapacityPlanT
  run(const ChipT& chip, const ConfigT& config, const Report& report) const = 0;
};

/**
 * @brief Solves the inner circuit on the lattice each capacity chain induces,
 * and reports the outer port ring the assignment consumes.
 *
 * The stage runs before the assignment. The inner circuit decides which
 * outer ports the inner wires surface at, and the ring the assignment works
 * on is the configured one extended by exactly those.
 */
class MQT_SCPD_PIPELINE_EXPORT IGlobalRouter {
public:
  /**
   * @brief Creates a router.
   */
  IGlobalRouter() = default;
  IGlobalRouter(const IGlobalRouter&) = delete;
  IGlobalRouter& operator=(const IGlobalRouter&) = delete;
  IGlobalRouter(IGlobalRouter&&) = delete;
  IGlobalRouter& operator=(IGlobalRouter&&) = delete;
  virtual ~IGlobalRouter() = default;

  /**
   * @brief Solves the inner circuit of a chip.
   * @param chip The classified chip.
   * @param capacity The capacity plan of the run.
   * @param config The run configuration.
   * @param report Where the stage reports what it does.
   * @return The global routing.
   */
  [[nodiscard]] virtual GlobalRoutingT run(const ChipT& chip,
                                           const CapacityPlanT& capacity,
                                           const ConfigT& config,
                                           const Report& report) const = 0;
};

/**
 * @brief Assigns resonators and conventional ports to launchers, on the ring
 * of outer ports.
 */
class MQT_SCPD_PIPELINE_EXPORT IAssigner {
public:
  /**
   * @brief Creates an assigner.
   */
  IAssigner() = default;
  IAssigner(const IAssigner&) = delete;
  IAssigner& operator=(const IAssigner&) = delete;
  IAssigner(IAssigner&&) = delete;
  IAssigner& operator=(IAssigner&&) = delete;
  virtual ~IAssigner() = default;

  /**
   * @brief Assigns the ports of the ring to launchers.
   * @param chip The classified chip.
   * @param capacity The capacity plan of the run.
   * @param global The global routing of the run, which carries the ring.
   * @param config The run configuration.
   * @param report Where the stage reports what it does.
   * @return The assignment.
   */
  [[nodiscard]] virtual AssignmentT run(const ChipT& chip,
                                        const CapacityPlanT& capacity,
                                        const GlobalRoutingT& global,
                                        const ConfigT& config,
                                        const Report& report) const = 0;
};

/**
 * @brief Routes every assigned connection through the partitions, coarsely.
 *
 * A wire is told which partitions it runs through and where it crosses from
 * one into the next. A border is crossed at one of a fixed set of slots, and
 * no two wires take the same one. Inside a partition two wires may not cross
 * each other either. Only the connections of the assignment are routed here;
 * the inner circuit paid for the free space it crosses when it was solved.
 */
class MQT_SCPD_PIPELINE_EXPORT ICorridorRouter {
public:
  /**
   * @brief Creates a router.
   */
  ICorridorRouter() = default;
  ICorridorRouter(const ICorridorRouter&) = delete;
  ICorridorRouter& operator=(const ICorridorRouter&) = delete;
  ICorridorRouter(ICorridorRouter&&) = delete;
  ICorridorRouter& operator=(ICorridorRouter&&) = delete;
  virtual ~ICorridorRouter() = default;

  /**
   * @brief Routes the connections of an assignment through the partitions.
   * @param chip The classified chip.
   * @param capacity The capacity plan of the run.
   * @param assignment The assignment of the run.
   * @param config The run configuration.
   * @param report Where the stage reports what it does.
   * @return One corridor per connection.
   */
  [[nodiscard]] virtual CorridorRoutingT run(const ChipT& chip,
                                             const CapacityPlanT& capacity,
                                             const AssignmentT& assignment,
                                             const ConfigT& config,
                                             const Report& report) const = 0;
};

} // namespace mqt::scpd::pipeline
