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

#include "mqt-scpd/grid/BitGrid.hpp"
#include "mqt-scpd/grid/Bottlenecks.hpp"
#include "mqt-scpd/grid/DistanceTransform.hpp"
#include "mqt-scpd/grid/RoutingBottlenecks.hpp"
#include "mqt-scpd/grid/mqt_scpd_grid_export.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <utility>
#include <vector>

namespace mqt::scpd::grid {

/// A cell that belongs to no wall.
inline constexpr std::uint32_t NO_WALL = static_cast<std::uint32_t>(-1);

/// What a wall is to `profileCuts`.
enum class ProfileRole : std::uint8_t {
  /// A wall that does not change while the dynamic walls do. It is found
  /// through a static field and never walked along.
  Static,
  /// A wall that changes: the search walks along it and cuts from it.
  Dynamic,
};

/// A dynamic wall drawn as a line inflated by a radius, as a feedline edge
/// is. The gap between two such walls is the distance of their lines less
/// both radii.
struct ProfileLine {
  /// The wall the line is the middle of.
  std::uint32_t wall = NO_WALL;
  /// The line through the cell centres, in cells.
  std::vector<std::pair<double, double>> points;
  /// How far the wall reaches to either side of the line, in cells.
  double radius = 0.0;
};

/// The open end of one flank of a port run: the side of a terminal's slot
/// that a cut has to close.
struct ProfileMouth {
  /// The flank.
  std::uint32_t flank = NO_WALL;
  /// The cell of the flank at the open end.
  std::size_t tip = 0;
  /// The unit direction out of the slot.
  double towardX = 0.0;
  double towardY = 0.0;
  /// How far ahead the nearest static wall of another run lies, in cells.
  /// A cut to a dynamic wall farther than this is not this mouth's: the
  /// static wall closes it, and a cut between two static walls is none.
  double staticClearance = 0.0;
};

/// The chip as `profileCuts` reads it.
struct ProfileScene {
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  /// The wall of every cell, or `NO_WALL`.
  std::span<const std::uint32_t> wallOf;
  /// The role of every wall.
  std::span<const ProfileRole> roles;
  /// The run every wall belongs to, or `NO_WALL`: the two flanks of a port
  /// run share it, so that no mouth is cut to its own run.
  std::span<const std::uint32_t> runs;
  /// The static fields: each the nearest cell of a set of static walls.
  /// Two fields on either side of the dynamic walls (the artwork inside,
  /// the border outside) let a cell see a static wall on each side.
  std::span<const NearestBlocked* const> fields;
  /// The free cells no cut may cross: the slots terminals lie in.
  const BitGrid* slots = nullptr;
  /// When given, a mask that holds every cell of every dynamic wall, and
  /// may hold cells of other walls: the dynamic walls are then gathered from
  /// its set bits alone, not from every cell of `wallOf`.
  const BitGrid* walls = nullptr;
  /// The dynamic walls drawn as inflated lines. Every other dynamic wall
  /// is drawn by its own outline.
  std::span<const ProfileLine> lines;
  /// The mouths of the slots.
  std::span<const ProfileMouth> mouths;
};

/// How `profileCuts` judges a gap, all in cells.
struct ProfileOptions {
  /// No gap wider than this is a bottleneck.
  double widest = 190.0;
  /// How much a gap must widen on both sides of its narrowest place, along
  /// the wall, before it narrows again: the persistence of the minimum.
  double minimumRise = 4.0;
  /// Cuts between the same two walls whose ends lie this close on both
  /// walls are one narrowing; the shortest stays.
  double sameNarrowing = 20.0;
  /// A gap narrower than this is a contact, not a gap.
  double narrowest = 3.0;
  /// The wire clearance, when a narrowing is to be cut at its two ends:
  /// a narrowing then runs along the wall as far as the gap holds as few
  /// wires as at its narrowest place (its width over the clearance), and a
  /// cut closes each end of it. 0 cuts a narrowing once, at the middle of
  /// its narrowest stretch.
  double clearance = 0.0;
  /// Whether of two cuts whose lines cross only the shorter is kept.
  bool dropCrossing = false;
  /// Whether a cut between two walls given as lines stays where the wedge
  /// rule would drop it.
  bool keepEdgeWedges = false;
};

/// The bottlenecks a dynamic wall takes part in, without a medial axis.
///
/// Every dynamic wall is walked along, and the width of the gap to every
/// wall within `widest` is a profile along it. A gap to a static wall is
/// read off the static fields; a gap to another dynamic wall is the
/// distance between the two walls' lines. A bottleneck is a local minimum
/// of a profile that the gap rises from by `minimumRise` on both sides,
/// whose line crosses only free cells. The mouth of every slot is closed by
/// a cut from each flank's open end to the nearest dynamic wall ahead.
/// Where two walls touch, the wedge between them is no bottleneck.
///
/// @returns The bottlenecks, each from a cell of one wall to a cell of the
/// other. The saddle is the middle cell of the line.
[[nodiscard]] MQT_SCPD_GRID_EXPORT std::vector<Bottleneck>
profileCuts(const ProfileScene& scene, const ProfileOptions& options = {});

/// What `profileCuts` keeps from one call to the next, by content: the
/// outline traces of a dynamic wall's cells, the simplified line of a
/// wall's dense line, and the narrowings of the gap from one wall's line to
/// another's. A wall with the same cells as a wall of an earlier call has
/// the same traces; a pair of walls with the same lines has the same
/// narrowings. Every entry is found by a hash and checked against the full
/// content it was made from, and the cuts themselves are drawn on every
/// call, so a call gives the same bottlenecks with the memo as without it.
/// A memo serves one thread at a time. It forgets everything when the grid
/// or the options change.
class MQT_SCPD_GRID_EXPORT ProfileMemo {
public:
  ProfileMemo();
  ~ProfileMemo();
  ProfileMemo(const ProfileMemo&) = delete;
  ProfileMemo& operator=(const ProfileMemo&) = delete;
  ProfileMemo(ProfileMemo&& other) noexcept;
  ProfileMemo& operator=(ProfileMemo&& other) noexcept;

  /// Forget every entry.
  void clear();

  /// How often the calls took an entry from the memo (`kept`) and how often
  /// they worked it out (`made`).
  struct Counts {
    std::uint64_t tracesKept = 0;
    std::uint64_t tracesMade = 0;
    std::uint64_t linesKept = 0;
    std::uint64_t linesMade = 0;
    std::uint64_t pairsKept = 0;
    std::uint64_t pairsMade = 0;
  };
  [[nodiscard]] Counts counts() const;

  /// The entries; defined where `profileCuts` reads them.
  struct State;
  [[nodiscard]] State& state() { return *state_; }

private:
  std::unique_ptr<State> state_;
};

/// `profileCuts` with the work on the walls kept in `memo` from call to
/// call. The result is the one `profileCuts` gives without it.
[[nodiscard]] MQT_SCPD_GRID_EXPORT std::vector<Bottleneck>
profileCuts(const ProfileScene& scene, const ProfileOptions& options,
            ProfileMemo& memo);

} // namespace mqt::scpd::grid
