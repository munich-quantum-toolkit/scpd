/*
 * Copyright (c) 2026 Chair for Design Automation, TUM
 * Copyright (c) 2026 Munich Quantum Software Company GmbH
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Licensed under the MIT License
 */

#include "mqt-scpd/grid/Stencil.hpp"

#include <cstddef>
#include <cstdint>

namespace mqt::scpd::grid {

Stencil stencilOf(const std::uint32_t reach, const double radius) {
  Stencil stencil;
  const auto side = static_cast<std::int32_t>(reach);
  const auto limit = radius * radius;
  const auto inside = [limit, side](const std::int32_t dx,
                                    const std::int32_t dy) {
    return dx >= -side && dx <= side && dy >= -side && dy <= side &&
           static_cast<double>((dx * dx) + (dy * dy)) <= limit;
  };
  for (std::int32_t dy = -side; dy <= side; ++dy) {
    for (std::int32_t dx = -side; dx <= side; ++dx) {
      if (inside(dx, dy)) {
        stencil.full.emplace_back(dx, dy);
      }
    }
  }
  for (std::int32_t sy = -1; sy <= 1; ++sy) {
    for (std::int32_t sx = -1; sx <= 1; ++sx) {
      if (sx == 0 && sy == 0) {
        continue;
      }
      auto& edge = stencil.edge[static_cast<std::size_t>(sy + 1)]
                               [static_cast<std::size_t>(sx + 1)];
      for (const auto& [dx, dy] : stencil.full) {
        if (!inside(dx + sx, dy + sy)) {
          edge.emplace_back(dx, dy);
        }
      }
    }
  }
  return stencil;
}

} // namespace mqt::scpd::grid
