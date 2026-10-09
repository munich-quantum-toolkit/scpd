# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""The terminal output of a run: one block per stage, the levels of ``-v`` and the live progress.

This module is the only place that writes to the terminal while a run goes on. The core reports
lines, entries, results and progress through callbacks, and this module lays them out by the rules
of ``docs/terminal_output.md``. A block starts with the emoji and the name of its stage; the lines
of the block are indented, and every line that happened at a time carries the time since the
stage began, right-aligned. On a terminal a live line shows a spinner, the step the stage is at and
how far it has got; in a file or a pipe only the lines that last are written.
"""

from __future__ import annotations

import time
from contextlib import contextmanager
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, TextIO

from rich.cells import cell_len
from rich.console import Console as RichConsole
from rich.live import Live
from rich.spinner import Spinner
from rich.text import Text

if TYPE_CHECKING:
    from collections.abc import Callable, Generator, Sequence

__all__ = ["STAGE_STYLES", "Console", "Figure", "StageBlock", "StageStyle", "duration", "size"]


@dataclass(frozen=True)
class StageStyle:
    """How a stage is marked: an emoji, its plain form and a colour."""

    emoji: str
    ascii: str
    color: str


#: The mark of every step. An emoji has Emoji_Presentation and no variation selector, so it is two
#: cells wide on every terminal. A step without a style cannot report; a test checks every step.
STAGE_STYLES: dict[str, StageStyle] = {
    "capacity": StageStyle("🧩", "[#]", "cyan"),
    "global": StageStyle("🔳", "[+]", "blue"),
    "assign": StageStyle("📌", "[>]", "magenta"),
    "corridor": StageStyle("🧭", "[~]", "yellow"),
}

#: Symbols with a plain form for an output that cannot encode them.
_PLAIN: dict[str, str] = {"✓": "ok", "✗": "FAIL", "⚠": "!", "·": "-", "→": "->", "\N{MULTIPLICATION SIGN}": "x"}

#: The tones of a figure and the colours they take.
_TONES: dict[str, str] = {"plain": "", "good": "green", "bad": "red"}

#: The indent of a line of a block, and of a line that only -v shows.
_INDENT = 3
_DEEP_INDENT = 5

#: The width of the label column of an entry.
_LABEL_WIDTH = 18

#: The width the time column ends at in a file, and the widest it gets on a terminal.
_FILE_WIDTH = 72
_TERMINAL_WIDTH = 100

#: The levels of a line, as -v counts them.
_STEPS, _SOLVER = 1, 3


@dataclass(frozen=True)
class Figure:
    """One figure of a result or an entry, such as ``363 partitions`` or ``fails 2``."""

    text: str
    tone: str = "plain"


def duration(seconds: float) -> str:
    """A duration as a person reads it at a glance.

    Returns:
        Seconds with two decimals below a minute, minutes and seconds below an hour, and hours and
        minutes above.
    """
    if seconds < 60.0:
        return f"{seconds:.2f}s"
    whole = round(seconds)
    if whole < 3600:
        return f"{whole // 60}m {whole % 60:02d}s"
    return f"{whole // 3600}h {(whole % 3600) // 60:02d}m"


def size(count: int) -> str:
    """A file size in decimal units with one decimal.

    Returns:
        The size, such as ``840 B``, ``2.8 kB`` or ``1.3 MB``.
    """
    if count < 1000:
        return f"{count} B"
    if count < 1_000_000:
        return f"{count / 1000:.1f} kB"
    return f"{count / 1_000_000:.1f} MB"


@dataclass
class _Run:
    """What the closing line sums up."""

    stages: int = 0
    fails: int | None = None
    failed: bool = False
    began: float | None = None


@dataclass
class StageBlock:
    """The block of one stage while it runs."""

    console: Console
    name: str
    began: float
    header_written: bool = False
    fails: int | None = None
    task: str = ""
    task_detail: str = ""
    done: int = 0
    total: int = 0
    _spinner: Spinner = field(default_factory=lambda: Spinner("dots"))

    @property
    def elapsed(self) -> float:
        """The seconds since the stage began."""
        return self.console.clock() - self.began

    def line(self, level: int, text: str) -> None:
        """Write a line of free text, if the verbosity shows its level."""
        if level > self.console.verbosity:
            return
        if level >= _SOLVER:
            self._write(Text(" " * _DEEP_INDENT + self.console.plain(text), style="dim"))
            return
        indent = _INDENT if level == 0 else _DEEP_INDENT
        self._write(self.console.timed(Text(" " * indent + self.console.plain(text)), self.elapsed, dim=level > 0))

    def entry(self, level: int, label: str, figures: Sequence[Figure]) -> None:
        """Write an entry, such as one round, with its label in a column of its own."""
        if level > self.console.verbosity:
            return
        indent = _INDENT if level == 0 else _DEEP_INDENT
        text = Text(" " * indent + self.console.plain(label).ljust(_LABEL_WIDTH) + " ")
        text.append_text(self.console.figures(figures))
        self._write(self.console.timed(text, self.elapsed, dim=level > 0))

    def result(self, figures: Sequence[Figure], fails: int | None = None) -> None:
        """Write the result of the stage and keep its fails for the artifact line and the closing line."""
        self.fails = fails
        if fails is not None:
            run = self.console.run
            run.fails = (run.fails or 0) + fails
        if figures:
            text = Text(" " * _INDENT)
            text.append_text(self.console.figures(figures))
            self._write(self.console.timed(text, self.elapsed))

    def artifact(self, name: str, count: int) -> None:
        """Write the artifact line: the file the stage wrote, its size and the fails of the stage."""
        text = Text(" " * _INDENT)
        text.append(self.console.plain("✓"), style="green")
        text.append(f" {name}  {size(count)}")
        if self.fails is not None:
            text.append(self.console.plain(" · "))
            tone = "good" if self.fails == 0 else "bad"
            text.append(f"fails {self.fails}", style=_TONES[tone])
        self._write(text)

    def warning(self, text: str) -> None:
        """Write a warning, at every verbosity."""
        line = Text(" " * _INDENT)
        line.append(self.console.plain("⚠ " + text), style="yellow")
        self._write(line)

    def failure(self, text: str) -> None:
        """Write a failure of the stage, at every verbosity."""
        self.console.run.failed = True
        line = Text(" " * _INDENT)
        line.append(self.console.plain("✗ " + text), style="red")
        self._write(line)

    def progress(self, task: str, detail: str, done: int, total: int) -> None:
        """Show how far the stage has got. Only the live line on a terminal shows it."""
        self.task, self.task_detail, self.done, self.total = task, detail, done, total
        self.console.refresh()

    def _write(self, text: Text) -> None:
        self.write_header()
        self.console.write(text)

    def write_header(self) -> None:
        """Write the header of the block once, before its first lasting line."""
        if self.header_written:
            return
        self.header_written = True
        style = STAGE_STYLES[self.name]
        header = Text(self.console.mark(style) + " ")
        header.append(self.name, style=f"bold {style.color}")
        self.console.write(header)

    def live_line(self) -> Text:
        """The live line: a spinner, the stage, its time, and what it is doing.

        Returns:
            The line.
        """
        line = Text()
        spinner = self._spinner.render(self.console.clock())
        line.append_text(spinner if isinstance(spinner, Text) else Text(str(spinner)))
        line.append(" ")
        line.append(self.name, style=f"bold {STAGE_STYLES[self.name].color}")
        line.append(f"  {duration(self.elapsed)}", style="dim")
        if self.task:
            line.append("  " + self.console.plain(self.task))
        if self.task_detail:
            line.append(self.console.plain(" · " + self.task_detail), style="dim")
        if self.total:
            line.append(self.console.plain(f" · {self.done}/{self.total}"), style="dim")
        return line


class Console:
    """The terminal of a run, laid out by the rules of ``docs/terminal_output.md``."""

    def __init__(
        self,
        file: TextIO | None = None,
        *,
        verbosity: int = 0,
        live: bool | None = None,
        color: bool | None = None,
        width: int | None = None,
        encoding: str | None = None,
        clock: Callable[[], float] = time.perf_counter,
        force_terminal: bool | None = None,
    ) -> None:
        """Set up the output.

        Args:
            file: Where the output goes; standard output when None.
            verbosity: How many times -v was given.
            live: Whether to show the live line; None shows it on a terminal that can.
            color: Whether to colour; None decides from the terminal and ``NO_COLOR``.
            width: The width the time column ends at; None takes it from the terminal.
            encoding: The encoding of the output; None takes it from the file.
            clock: The clock that times the stages.
            force_terminal: Treat the output as a terminal, or not; None decides from the file.
        """
        self.verbosity = verbosity
        self.clock = clock
        if color is None:
            self._rich = RichConsole(
                file=file, highlight=False, emoji=False, markup=False, soft_wrap=True, force_terminal=force_terminal
            )
        else:
            self._rich = RichConsole(
                file=file,
                highlight=False,
                emoji=False,
                markup=False,
                soft_wrap=True,
                force_terminal=True if force_terminal is None and color else force_terminal,
                color_system="standard" if color else None,
                no_color=not color,
            )
        named = encoding or getattr(file, "encoding", None) or self._rich.encoding or "utf-8"
        try:
            "".join([*_PLAIN, *(style.emoji for style in STAGE_STYLES.values())]).encode(named)
            self.unicode = True
        except (UnicodeEncodeError, LookupError):
            self.unicode = False
        terminal = self._rich.is_terminal and not self._rich.is_dumb_terminal
        self.width = width or (min(self._rich.width, _TERMINAL_WIDTH) if terminal else _FILE_WIDTH)
        self.show_live = terminal if live is None else live
        self.run = _Run()
        self._stage: StageBlock | None = None
        self._live: Live | None = None

    def plain(self, text: str) -> str:
        """The text, with every symbol in its plain form when the output cannot encode it.

        Returns:
            The text as the output can carry it.
        """
        if self.unicode:
            return text
        for symbol, plain in _PLAIN.items():
            text = text.replace(symbol, plain)
        return text.encode("ascii", "replace").decode("ascii")

    def mark(self, style: StageStyle) -> str:
        """The mark of a stage: its emoji, or its plain form.

        Returns:
            The mark.
        """
        return style.emoji if self.unicode else style.ascii

    def figures(self, figures: Sequence[Figure]) -> Text:
        """Figures joined by the separator, each in the colour of its tone.

        Returns:
            The figures as one text.
        """
        text = Text()
        for index, figure in enumerate(figures):
            if index:
                text.append(self.plain(" · "))
            text.append(self.plain(figure.text), style=_TONES.get(figure.tone, ""))
        return text

    def timed(self, text: Text, seconds: float, *, dim: bool = False) -> Text:
        """A line with a time right-aligned at the width, or two spaces after it when it is too long.

        Returns:
            The line with its time.
        """
        stamp = duration(seconds)
        gap = self.width - cell_len(text.plain) - len(stamp)
        line = text.copy()
        line.append(" " * max(gap, 2) + stamp)
        if dim:
            line.stylize("dim")
        return line

    def write(self, text: Text) -> None:
        """Write one lasting line, above the live line when there is one."""
        self._rich.print(text)

    def success(self, text: str) -> None:
        """Write the line of a command that did what it was asked: a green tick and what it made."""
        line = Text()
        line.append(self.plain("✓"), style="green")
        line.append(self.plain(" " + text))
        self.write(line)

    def failure(self, text: str) -> None:
        """Write the line of a command that could not do what it was asked: a red cross and why."""
        line = Text()
        line.append(self.plain("✗ " + text), style="red")
        self.write(line)

    def header(self, command: str, subject: str, target: str | None = None, facts: Sequence[str] = ()) -> None:
        """Write the header of a run: what runs on what, where it writes and the facts of the run."""
        line = f"mqt-scpd {command} {subject}"
        if target is not None:
            line += f" → {target}"
        if facts:
            line += "  ·  " + " · ".join(facts)
        self.write(Text(self.plain(line)))
        self.write(Text())

    @contextmanager
    def stage(self, name: str) -> Generator[StageBlock]:
        """Run one stage block: its header, its lines and, on a terminal, its live line.

        Yields:
            The block, to report into.

        Raises:
            KeyError: If the step has no style in STAGE_STYLES.
        """
        if name not in STAGE_STYLES:
            msg = f"the step '{name}' has no style in console.STAGE_STYLES"
            raise KeyError(msg)
        began = self.clock()
        if self.run.began is None:
            self.run.began = began
        block = StageBlock(self, name, began)
        self._stage = block
        if not self.show_live:
            block.write_header()
        else:
            self._live = Live(
                self._live_render(),
                console=self._rich,
                transient=True,
                auto_refresh=True,
                refresh_per_second=10,
                redirect_stdout=False,
                redirect_stderr=False,
                get_renderable=self._live_render,
            )
            self._live.start()
        try:
            yield block
        finally:
            if self._live is not None:
                self._live.stop()
                self._live = None
            block.write_header()
            self._stage = None
            self.run.stages += 1

    def refresh(self) -> None:
        """Redraw the live line, when there is one."""
        if self._live is not None:
            self._live.refresh()

    def _live_render(self) -> Text:
        if self._stage is None:
            return Text()
        return self._stage.live_line()

    def closing(self, *, stopped_after: str | None, failed: str | None = None) -> None:
        """Write the closing line of a run: how many stages ran, how long, where it stopped, how many fails."""
        run = self.run
        seconds = self.clock() - run.began if run.began is not None else 0.0
        self.write(Text())
        if failed is not None:
            line = Text()
            line.append(self.plain(f"✗ stopped at {failed} after {duration(seconds)}"), style="red")
            self.write(line)
            return
        parts = [f"{run.stages} stage{'' if run.stages == 1 else 's'}", duration(seconds)]
        if stopped_after is not None:
            parts.append(f"stopped after {stopped_after}")
        if run.fails is not None:
            parts.append(f"fails {run.fails}")
        good = not run.failed and not run.fails
        line = Text()
        line.append(self.plain("✓" if good else "✗"), style="green" if good else "red")
        line.append(self.plain(" " + " · ".join(parts)))
        self.write(line)
