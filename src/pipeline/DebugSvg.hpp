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

#include <cstddef>
#include <cstdint>
#include <format>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace mqt::scpd::pipeline::debug {

/// A rectangle of cells, both corners inclusive.
struct View {
  std::int64_t minX = 0;
  std::int64_t minY = 0;
  std::int64_t maxX = -1;
  std::int64_t maxY = -1;
  [[nodiscard]] std::int64_t width() const { return maxX - minX + 1; }
  [[nodiscard]] std::int64_t height() const { return maxY - minY + 1; }
};

/// A cell, as a pair of coordinates.
using Cell = std::pair<std::int64_t, std::int64_t>;

/// A picture of a part of the router grid, one SVG unit per cell.
///
/// The grid's y grows with the layout's y and an SVG's grows downward, so
/// every y is flipped through the grid's height and the picture has the
/// orientation of `mqt-scpd plot`. Fields over the cells are painted as runs
/// of equal value, merged across rows where the run repeats, because a
/// picture of a grid of three thousand cells a side made of one rectangle per
/// cell cannot be opened.
class Svg {
public:
  /// `headroom` is how many cells of room the picture keeps above the view,
  /// for a legend that covers no cell.
  Svg(const View& view, const std::uint32_t gridHeight,
      const double headroom = 0.0)
      : view_(view), height_(gridHeight), headroom_(headroom) {
    out_ =
        std::format("<svg xmlns=\"http://www.w3.org/2000/svg\" "
                    "viewBox=\"{} {:.1f} {} {:.1f}\" width=\"{}\" "
                    "height=\"{:.1f}\">\n",
                    view.minX, top(view.maxY) - headroom, view.width(),
                    static_cast<double>(view.height()) + headroom, view.width(),
                    static_cast<double>(view.height()) + headroom);
  }

  /// The SVG y of the top edge of the picture, headroom included.
  [[nodiscard]] double ceiling() const { return top(view_.maxY) - headroom_; }

  /// The SVG x of the left edge of a column, and the SVG y of the top edge
  /// of a row.
  [[nodiscard]] static double left(const std::int64_t x) {
    return static_cast<double>(x);
  }
  [[nodiscard]] double top(const std::int64_t y) const {
    return static_cast<double>(height_) - 1.0 - static_cast<double>(y);
  }
  /// The SVG point at the centre of a cell.
  [[nodiscard]] double centreX(const std::int64_t x) const {
    return left(x) + 0.5;
  }
  [[nodiscard]] double centreY(const std::int64_t y) const {
    return top(y) + 0.5;
  }
  [[nodiscard]] const View& view() const { return view_; }

  void style(const std::string_view css) {
    out_ += "<style>";
    out_ += css;
    out_ += "</style>\n";
    // The ground, so that a viewer with no page colour of its own does not
    // show the free cells as black.
    box(left(view_.minX), ceiling(), static_cast<double>(view_.width()),
        static_cast<double>(view_.height()) + headroom_, "bg");
  }

  /// Paint a field over the view: `valueOf(x, y)` says what a cell holds,
  /// zero for nothing, and `classOf(value)` names the class a value is drawn
  /// with. Runs of one value along a row are one rectangle, and a run that
  /// repeats exactly on the rows below grows down over them.
  template <typename ValueOf, typename ClassOf>
  void runs(ValueOf&& valueOf, ClassOf&& classOf) {
    struct Run {
      std::int64_t x0 = 0;
      std::int64_t x1 = 0;
      int value = 0;
      std::int64_t firstY = 0;
      std::int64_t rows = 0;
    };
    std::vector<Run> open;
    std::vector<Run> next;
    const auto flush = [&](const Run& run) {
      out_ += std::format(
          "<rect x=\"{}\" y=\"{}\" width=\"{}\" height=\"{}\" class=\"{}\"/>\n",
          run.x0, top(run.firstY), run.x1 - run.x0 + 1, run.rows,
          classOf(run.value));
    };
    // Top row first, so that a run grows downward in the picture.
    for (std::int64_t y = view_.maxY; y >= view_.minY; --y) {
      next.clear();
      std::int64_t x = view_.minX;
      while (x <= view_.maxX) {
        const int value = valueOf(x, y);
        std::int64_t end = x;
        while (end + 1 <= view_.maxX && valueOf(end + 1, y) == value) {
          ++end;
        }
        if (value != 0) {
          bool grown = false;
          for (auto& run : open) {
            if (run.rows != 0 && run.x0 == x && run.x1 == end &&
                run.value == value) {
              run.rows += 1;
              next.push_back(run);
              run.rows = 0; // taken over
              grown = true;
              break;
            }
          }
          if (!grown) {
            next.push_back(
                {.x0 = x, .x1 = end, .value = value, .firstY = y, .rows = 1});
          }
        }
        x = end + 1;
      }
      for (const auto& run : open) {
        if (run.rows != 0) {
          flush(run);
        }
      }
      open.swap(next);
    }
    for (const auto& run : open) {
      if (run.rows != 0) {
        flush(run);
      }
    }
  }

  /// A line through the centres of cells.
  void polyline(const std::vector<Cell>& cells, const std::string_view cls,
                const std::string_view extra = {}) {
    if (cells.size() < 2) {
      return;
    }
    out_ += std::format("<polyline class=\"{}\" {} points=\"", cls, extra);
    for (const auto& [x, y] : cells) {
      out_ += std::format("{:.1f},{:.1f} ", centreX(x), centreY(y));
    }
    out_ += "\"/>\n";
  }

  /// A closed outline through the centres of cells.
  void polygon(const std::vector<Cell>& cells, const std::string_view cls) {
    if (cells.size() < 3) {
      return;
    }
    out_ += std::format("<polygon class=\"{}\" points=\"", cls);
    for (const auto& [x, y] : cells) {
      out_ += std::format("{:.1f},{:.1f} ", centreX(x), centreY(y));
    }
    out_ += "\"/>\n";
  }

  /// The SVG point of a position in fractional cell coordinates, where the
  /// integer `i` is the centre of cell `i`.
  [[nodiscard]] static double atX(const double fx) { return fx + 0.5; }
  [[nodiscard]] double atY(const double fy) const {
    return static_cast<double>(height_) - 0.5 - fy;
  }

  /// A dot at an SVG point.
  void dot(const double sx, const double sy, const double radius,
           const std::string_view cls) {
    out_ += std::format(
        "<circle cx=\"{:.2f}\" cy=\"{:.2f}\" r=\"{:.1f}\" class=\"{}\"/>\n", sx,
        sy, radius, cls);
  }

  void circle(const std::int64_t x, const std::int64_t y, const double radius,
              const std::string_view cls) {
    out_ += std::format(
        "<circle cx=\"{:.1f}\" cy=\"{:.1f}\" r=\"{:.1f}\" class=\"{}\"/>\n",
        centreX(x), centreY(y), radius, cls);
  }

  /// A straight line between two SVG points.
  void line(const double x0, const double y0, const double x1, const double y1,
            const std::string_view cls) {
    out_ += std::format("<line x1=\"{:.1f}\" y1=\"{:.1f}\" x2=\"{:.1f}\" "
                        "y2=\"{:.1f}\" class=\"{}\"/>\n",
                        x0, y0, x1, y1, cls);
  }

  /// A rectangle in SVG coordinates.
  void box(const double x, const double y, const double width,
           const double height, const std::string_view cls) {
    out_ += std::format("<rect x=\"{:.1f}\" y=\"{:.1f}\" width=\"{:.1f}\" "
                        "height=\"{:.1f}\" class=\"{}\"/>\n",
                        x, y, width, height, cls);
  }

  /// A hollow square of `half` cells to each side of a cell's centre, with a
  /// tooltip a viewer shows when the pointer rests on it.
  void square(const std::int64_t x, const std::int64_t y, const double half,
              const std::string_view cls, const std::string_view tooltip) {
    out_ += std::format("<rect x=\"{:.1f}\" y=\"{:.1f}\" width=\"{:.1f}\" "
                        "height=\"{:.1f}\" class=\"{}\"><title>",
                        centreX(x) - half, centreY(y) - half, 2.0 * half,
                        2.0 * half, cls);
    escape(tooltip);
    out_ += "</title></rect>\n";
  }

  /// Text at an SVG point, in a font of `size` cells.
  void text(const double x, const double y, const std::string_view what,
            const std::string_view cls, const double size) {
    out_ += std::format("<text x=\"{:.1f}\" y=\"{:.1f}\" class=\"{}\" "
                        "font-size=\"{:.1f}\">",
                        x, y, cls, size);
    escape(what);
    out_ += "</text>\n";
  }

  [[nodiscard]] std::string finish() {
    out_ += "</svg>\n";
    return std::move(out_);
  }

private:
  /// Append text with the three characters XML cannot carry as they are.
  void escape(const std::string_view what) {
    for (const char c : what) {
      switch (c) {
      case '<':
        out_ += "&lt;";
        break;
      case '>':
        out_ += "&gt;";
        break;
      case '&':
        out_ += "&amp;";
        break;
      default:
        out_ += c;
      }
    }
  }

  View view_;
  std::uint32_t height_;
  double headroom_;
  std::string out_;
};

} // namespace mqt::scpd::pipeline::debug
