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

#include "mqt-scpd/pipeline/Stages.hpp"
#include "mqt-scpd/pipeline/mqt_scpd_pipeline_export.hpp"

#include <memory>

namespace mqt::scpd::pipeline {

/// The corridor router of the first release.
///
/// It searches the partition graph rather than the pixels: the nodes are the
/// crossing slots of the partition borders, an edge runs between two slots of
/// one partition, and its cost is the distance between them. A wire is
/// therefore told which corridors it uses and where it changes from one into
/// the next, and the detail router fills in the pixels inside each.
///
/// Two rules make the plan one the detail router can follow. No two wires
/// take the same slot, so a border carries no more wires than it has room
/// for; and two wires may not cross inside one partition, so the order they
/// arrive at a border in is the order they leave it in.
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT std::unique_ptr<ICorridorRouter> makePartitionAStarRouter();

} // namespace mqt::scpd::pipeline
