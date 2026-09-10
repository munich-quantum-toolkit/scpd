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

#include <functional>
#include <string_view>

namespace mqt::scpd::pipeline {

using flatbuffers::artifacts::AssignmentT;
using flatbuffers::artifacts::CapacityPlanT;
using flatbuffers::artifacts::CorridorRoutingT;
using flatbuffers::artifacts::DetailRoutingT;
using flatbuffers::artifacts::FinalRoutingT;
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

/// Where a stage reports what it is doing while it runs.
///
/// The Final stage takes minutes on the largest chip, and what it does in that
/// time — which wire found a way, which had to let a neighbour go, which is
/// still open — is only useful while it runs. So it is a callback and not a
/// report: the stage hands out one line at a time and the caller decides what
/// to do with it. Nothing is printed from the core, which is what keeps every
/// human-readable form of it in one place.
using Progress = std::function<void(std::string_view)>;

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

  [[nodiscard]] virtual CorridorRoutingT
  run(const ChipT& chip, const CapacityPlanT& capacity,
      const AssignmentT& assignment, const ConfigT& config,
      const Progress& progress = {}) const = 0;
};

/// Draws every wire on the detail grid, cell by cell.
///
/// The corridor stage said which partitions a wire runs through and where it
/// crosses from one into the next; this is where the copper goes. A wire is
/// cut at its crossings into one piece per partition, each piece is searched
/// for inside that partition alone, the pieces are joined in corridor order,
/// and the joined wire is then offered a way of its own through the same
/// partitions.
///
/// It takes the global routing as well as the corridor routing because the
/// inner circuit never entered the corridor stage: those connections paid for
/// the free space they cross while the inner circuit was solved, so they come
/// here unrouted and are drawn last, against the wires of the ring.
class MQT_SCPD_PIPELINE_EXPORT IDetailRouter {
public:
  IDetailRouter() = default;
  IDetailRouter(const IDetailRouter&) = delete;
  IDetailRouter& operator=(const IDetailRouter&) = delete;
  IDetailRouter(IDetailRouter&&) = delete;
  IDetailRouter& operator=(IDetailRouter&&) = delete;
  virtual ~IDetailRouter() = default;

  [[nodiscard]] virtual DetailRoutingT
  run(const ChipT& chip, const CapacityPlanT& capacity,
      const GlobalRoutingT& global, const AssignmentT& assignment,
      const CorridorRoutingT& corridor, const ConfigT& config,
      const Progress& progress = {}) const = 0;
};

/// Draws every wire as curvature-constrained copper on the router grid, and
/// completes the design the earlier stages planned.
///
/// The Detail stage's cell paths are a centre line on a grid and not geometry.
/// This stage draws each of them again over Dubins primitives of one bend
/// radius, on a grid whose cell is about ten layout units, and then finishes
/// what the plan left open: it inserts the CPW coupler that carries a
/// resonator's `ResonatorSource` port, routes the launcher-to-launcher
/// feedline chains that drive the couplers, and refines them.
///
/// **Coupler placement and feedline routing are one fixpoint, not two steps.**
/// The feedline repair loop turns an already-placed coupler around when that
/// is what lets a chain route, so the stage cannot be cut into "place the
/// couplers, then route the feedlines" without losing the mechanism that
/// reaches zero failures. That is why one call does both.
///
/// The obstacle clearance is not checked here. It is baked into the raster
/// mask before any search runs, so every cell a search may enter already keeps
/// it; the design-rule check verifies the committed result separately, because
/// the committed result is not always what the search produced.
class MQT_SCPD_PIPELINE_EXPORT IFinalRouter {
public:
  IFinalRouter() = default;
  IFinalRouter(const IFinalRouter&) = delete;
  IFinalRouter& operator=(const IFinalRouter&) = delete;
  IFinalRouter(IFinalRouter&&) = delete;
  IFinalRouter& operator=(IFinalRouter&&) = delete;
  virtual ~IFinalRouter() = default;

  [[nodiscard]] virtual FinalRoutingT
  run(const ChipT& chip, const CapacityPlanT& capacity,
      const GlobalRoutingT& global, const AssignmentT& assignment,
      const DetailRoutingT& detail, const ConfigT& config,
      const Progress& progress = {}) const = 0;
};

} // namespace mqt::scpd::pipeline
