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

/**
 * @brief Creates the corridor router of the first release.
 *
 * The router searches the partition graph and not the cells. The nodes are
 * the crossing slots of the partition borders, an edge joins two slots of one
 * partition, and its cost is the distance between them. A wire is therefore
 * told which partitions it runs through and where it crosses from one into
 * the next, and a later stage draws it inside each partition.
 *
 * Two rules make the plan one that the later stage can follow. No two wires
 * take the same slot, so a border carries no more wires than it has room for,
 * and the crossings of two wires lie at least one crossing pitch apart, also
 * where two borders meet and their slots lie closer than that. The pitch
 * applies to crossings only: a wire may cross a border closer than one pitch
 * to the target cell or the feed of another wire. Inside one partition no two
 * wires cross, run along each other or meet at a point, so the wires leave a
 * border in the order they arrive at it. The rule is checked on the straight
 * chord between where a wire enters and leaves a partition, which is exact
 * only in a convex partition.
 *
 * Where wires compete, the router rips some up and routes them again, for
 * `[stages.corridor] rounds` rounds. It keeps the round with the fewest wires
 * without a way. A last pass then offers each such wire the free slots of
 * that plan and rips up nothing.
 *
 * @return The router.
 */
[[nodiscard]] MQT_SCPD_PIPELINE_EXPORT std::unique_ptr<ICorridorRouter>
makePartitionAStarRouter();

} // namespace mqt::scpd::pipeline
