# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Tests of the terminal output: the stage blocks, the levels and the live progress."""

from __future__ import annotations

import io
import itertools
import re

import pytest

from mqt.scpd.console import STAGE_STYLES, Console, Figure
from mqt.scpd.steps import STEPS


class Clock:
    """A clock that a test moves by hand."""

    def __init__(self) -> None:
        """Start at 100 seconds."""
        self.now = 100.0

    def __call__(self) -> float:
        """Read the clock.

        Returns:
            The time the test has set, in seconds.
        """
        return self.now


def make(verbosity: int = 0, *, encoding: str = "utf-8") -> tuple[Console, io.StringIO, Clock]:
    """A console that writes into a string, without the live line, on a clock that the test moves.

    Returns:
        The console, the string it writes into, and its clock.
    """
    out = io.StringIO()
    clock = Clock()
    console = Console(out, verbosity=verbosity, live=False, width=72, encoding=encoding, clock=clock)
    return console, out, clock


def corridor_run(console: Console, clock: Clock) -> None:
    """Report the capacity stage and the corridor stage of a run on 17q."""
    console.header("plan", "17q", "runs/17q", ["HiGHS 1.11.0"])
    with console.stage("capacity") as stage:
        clock.now += 1.02
        stage.result([Figure("363 partitions"), Figure("62 bottlenecks"), Figure("69 chains")])
        stage.artifact("01-capacity.fb", 1_341_000)
    with console.stage("corridor") as stage:
        clock.now += 0.11
        stage.entry(0, "round 1  forward", [Figure("routed 56/58"), Figure("fails 2", "bad")])
        clock.now += 0.08
        stage.entry(0, "round 2  backward", [Figure("routed 58/58"), Figure("fails 0", "good")])
        stage.result([], fails=0)
        stage.artifact("04-corridor.fb", 60_768)
    console.closing(stopped_after="corridor")


def test_a_run_prints_one_block_per_stage() -> None:
    """The layout of a run without -v: a header, one block per stage and a closing line."""
    console, out, clock = make()
    corridor_run(console, clock)

    assert out.getvalue().splitlines() == [
        "mqt-scpd plan 17q → runs/17q  ·  HiGHS 1.11.0",
        "",
        "🧩 capacity",
        "   363 partitions · 62 bottlenecks · 69 chains                     1.02s",
        "   ✓ 01-capacity.fb  1.3 MB",
        "🧭 corridor",
        "   round 1  forward   routed 56/58 · fails 2                       0.11s",
        "   round 2  backward  routed 58/58 · fails 0                       0.19s",
        "   ✓ 04-corridor.fb  60.8 kB · fails 0",
        "",
        "✓ 2 stages · 1.21s · stopped after corridor · fails 0",
    ]


def test_a_line_shows_only_up_to_the_verbosity() -> None:
    """-v adds the steps, dimmed and indented deeper; -vv adds the items."""
    for verbosity, expected in ((0, 0), (1, 1), (2, 2)):
        console, out, clock = make(verbosity)
        with console.stage("capacity") as stage:
            clock.now += 0.5
            stage.line(1, "a step line")
            stage.line(2, "an item line")
            stage.line(3, "a solver line")
        lines = out.getvalue().splitlines()[1:]
        assert len(lines) == expected
        assert all(line.startswith("     ") for line in lines)


def test_a_solver_line_shows_at_the_highest_verbosity_only() -> None:
    """-vvv passes the own log of the solver through."""
    console, out, _ = make(3)
    with console.stage("assign") as stage:
        stage.line(3, "Presolving model")
    assert out.getvalue().splitlines()[-1] == "     Presolving model"


def test_warnings_and_failures_show_at_every_level() -> None:
    """A warning and a failure are never filtered by the verbosity."""
    console, out, _ = make(0)
    with console.stage("global") as stage:
        stage.warning("27 of 73 chains reach no launcher")
        stage.failure("the inner circuit did not solve: infeasible")
    assert out.getvalue().splitlines()[1:] == [
        "   ⚠ 27 of 73 chains reach no launcher",
        "   ✗ the inner circuit did not solve: infeasible",
    ]


def test_the_closing_line_names_a_failed_stage() -> None:
    """A run that stops at a failure says where and after how long."""
    console, out, clock = make()
    with console.stage("capacity"):
        clock.now += 2.0
    console.closing(stopped_after=None, failed="assign")
    assert out.getvalue().splitlines()[-1] == "✗ stopped at assign after 2.00s"


def test_the_closing_line_counts_the_fails_of_every_stage() -> None:
    """The fails of the stages that report them add up."""
    console, out, _ = make()
    with console.stage("corridor") as stage:
        stage.result([], fails=2)
    console.closing(stopped_after="corridor")
    assert out.getvalue().splitlines()[-1] == "✗ 1 stage · 0.00s · stopped after corridor · fails 2"


def test_a_long_line_keeps_its_time_after_two_spaces() -> None:
    """A line too long for the time column carries the time after it."""
    console, out, clock = make()
    with console.stage("assign") as stage:
        clock.now += 0.38
        stage.result([Figure("x" * 80)])
    assert out.getvalue().splitlines()[1] == "   " + "x" * 80 + "  0.38s"


@pytest.mark.parametrize(
    ("seconds", "shown"),
    [(0.0, "0.00s"), (1.234, "1.23s"), (59.994, "59.99s"), (61.0, "1m 01s"), (3190.0, "53m 10s"), (7322.0, "2h 02m")],
)
def test_a_duration_reads_at_a_glance(seconds: float, shown: str) -> None:
    """Seconds below a minute, minutes and seconds below an hour, hours and minutes above."""
    console, out, clock = make()
    with console.stage("capacity") as stage:
        clock.now += seconds
        stage.result([Figure("done")])
    assert out.getvalue().splitlines()[1].endswith(shown)


@pytest.mark.parametrize(("size", "shown"), [(840, "840 B"), (2_840, "2.8 kB"), (1_341_000, "1.3 MB")])
def test_an_artifact_size_reads_at_a_glance(size: int, shown: str) -> None:
    """Sizes are decimal units with one decimal."""
    console, out, _ = make()
    with console.stage("capacity") as stage:
        stage.artifact("01-capacity.fb", size)
    assert out.getvalue().splitlines()[1] == f"   ✓ 01-capacity.fb  {shown}"


def test_a_terminal_without_unicode_gets_the_ascii_forms() -> None:
    """Where the output cannot encode the symbols, every one of them has a plain form."""
    console, out, clock = make(encoding="ascii")
    corridor_run(console, clock)

    text = out.getvalue()
    text.encode("ascii")
    assert "[#] capacity" in text
    assert "[~] corridor" in text
    assert "   ok 01-capacity.fb  1.3 MB" in text
    assert "routed 56/58 - fails 2" in text
    assert text.splitlines()[0] == "mqt-scpd plan 17q -> runs/17q  -  HiGHS 1.11.0"


def test_the_sign_of_a_size_has_a_plain_form() -> None:
    """The multiplication sign of a size, such as the size of a grid, reads as an x in ASCII."""
    console, out, _ = make(1, encoding="ascii")
    with console.stage("capacity") as stage:
        stage.line(1, "capacity grid 25\N{MULTIPLICATION SIGN}25")
    assert "capacity grid 25x25" in out.getvalue()


def test_a_command_ends_with_a_tick_or_a_cross() -> None:
    """A command outside a run says in one line what it did, or why it could not."""
    console, out, _ = make()
    console.success("layout.svg  1.3 MB")
    console.failure("cannot read the chip input chip.json")
    assert out.getvalue().splitlines() == ["✓ layout.svg  1.3 MB", "✗ cannot read the chip input chip.json"]

    plain, text, _ = make(encoding="ascii")
    plain.success("layout.svg  1.3 MB · GDS2")
    plain.failure("no file")
    assert text.getvalue().splitlines() == ["ok layout.svg  1.3 MB - GDS2", "FAIL no file"]


def test_every_step_has_a_style() -> None:
    """A step without an emoji, a plain form and a colour cannot report; a new one fails here."""
    assert set(STEPS) <= set(STAGE_STYLES)
    for style in STAGE_STYLES.values():
        assert len(style.emoji) == 1
        # Emoji_Presentation without a variation selector is always two cells wide.
        assert "️" not in style.emoji
        assert re.fullmatch(r"\[.\]", style.ascii)
    assert len({style.emoji for style in STAGE_STYLES.values()}) == len(STAGE_STYLES)


def test_progress_never_reaches_a_file() -> None:
    """Live progress is for a terminal; in a file only the lasting lines remain."""
    console, out, _ = make()
    with console.stage("corridor") as stage:
        for done in range(1, 59):
            stage.progress("round 1  forward", "", done, 58)
        stage.result([], fails=0)
    assert out.getvalue().splitlines() == ["🧭 corridor"]


def test_the_block_header_waits_for_the_first_lasting_line_on_a_terminal() -> None:
    """On a terminal the header is written with the first lasting line, so the spinner holds its place."""
    out = io.StringIO()
    clock = Clock()
    console = Console(out, live=True, width=72, encoding="utf-8", clock=clock, force_terminal=True, color=False)
    with console.stage("capacity") as stage:
        stage.progress("medial axis", "", 1, 2)
        assert "🧩 capacity" not in visible(out.getvalue())
        clock.now += 1.0
        stage.result([Figure("363 partitions")])
    lasting = visible(out.getvalue())
    assert "🧩 capacity\n   363 partitions" in lasting


def visible(text: str) -> str:
    """The text a terminal shows once the cursor movements of the live display are done.

    Returns:
        The text without control sequences and carriage returns.
    """
    without_codes = re.sub(r"\x1b\[[0-9;?]*[A-Za-z]", "", text)
    return "\n".join(line.rsplit("\r", maxsplit=1)[-1] for line in without_codes.split("\n"))


def test_colour_marks_a_verdict_and_dims_a_step() -> None:
    """With colour on, a good verdict is green, a bad one red, and a step line dim."""
    out = io.StringIO()
    console = Console(
        out, verbosity=1, live=False, width=72, encoding="utf-8", clock=itertools.count().__next__, color=True
    )
    with console.stage("corridor") as stage:
        stage.entry(0, "round 1  forward", [Figure("fails 2", "bad")])
        stage.entry(0, "round 2  backward", [Figure("fails 0", "good")])
        stage.line(1, "the rescue placed 1 more")
    text = out.getvalue()
    assert "\x1b[31mfails 2" in text
    assert "\x1b[32mfails 0" in text
    assert "\x1b[2m" in text
