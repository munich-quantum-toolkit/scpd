# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Tests of the command line, through its entry point."""

from __future__ import annotations

import errno
import io
import json
import os
import re
import sys
from pathlib import Path
from typing import TYPE_CHECKING

import pytest

from mqt.scpd import cli, pyscpd
from mqt.scpd.artifacts import read_artifact, write_artifact
from mqt.scpd.cli import main
from mqt.scpd.export import HAS_KLAYOUT
from mqt.scpd.flatbuffers.artifacts.Artifact import ArtifactT
from mqt.scpd.flatbuffers.artifacts.GlobalRouting import GlobalRoutingT
from mqt.scpd.flatbuffers.artifacts.StageOutput import StageOutput
from mqt.scpd.planning import PlanningGeometry
from mqt.scpd.planning import planning_geometry as drawing
from mqt.scpd.run import RunDirectory

if TYPE_CHECKING:
    from collections.abc import Generator

    from mqt.scpd.flatbuffers.design.Chip import ChipT

CONFIG = str(Path(__file__).resolve().parents[2] / "fixtures" / "mini" / "config.toml")

#: The output of plan on the fixture without -v, one pattern per line, with the times removed and
#: the run path, the solver version and the file sizes masked. The form of every line is fixed; the
#: figures are what the stages make of the fixture.
PLAN_OUTPUT = (
    r"mqt-scpd plan mini → RUN  ·  HiGHS x\.y\.z",
    r"",
    r"🧩 capacity",
    r"   \d+ partitions? · \d+ bottlenecks? · \d+ chains?",
    r"   ✓ 01-capacity\.fb  SIZE",
    r"🔳 global",
    r"   no inner circuit · \d+ ring ports?",
    r"   ✓ 02-global\.fb  SIZE · fails 0",
    r"📌 assign",
    r"   \d+ ports? · \d+ launcher ends? · objective \d+\.\d\d · optimal",
    r"   ✓ 03-assign\.fb  SIZE · fails 0",
    r"🧭 corridor",
    # Any number of rounds, and the last one routes every connection.
    (
        r"(?:   round \d+ (?:forward|backward) +routed \d+/\d+ · fails \d+\n)*"
        r"   round \d+ (?:forward|backward) +routed (\d+)/\1 · fails 0"
    ),
    r"   routed (\d+)/\2 · \d+ rounds?",
    r"   ✓ 04-corridor\.fb  SIZE · fails 0",
    r"🧵 detail",
    r"   drawn (\d+)/\3 · \d+ rounds? · unrouted 0 · too close 0",
    r"   ✓ 05-detail\.fb  SIZE · fails 0",
    r"🧷 final",
    r"   pre-feedline-routing +drawn (\d+)/\4 · short \d+ · bad 0",
    r"   pre-feedline-refinement +short \d+ · long \d+",
    r"   feedline-insertion +edges (\d+)/\5 · angle cost \d+",
    r"   feedline-constrained-routing +drawn (\d+)/\6 · short \d+ · long \d+ · crossing 0 · bad 0",
    r"   feedline-constrained-refinement +short \d+ · long \d+ · bad 0",
    r"   drawn (\d+)/\7 · short \d+ · long \d+ · bad 0 · angle cost \d+",
    r"   ✓ 06-final\.fb  SIZE · fails 0",
    r"",
    r"✓ 6 stages · stopped after final · fails 0",
)

# Every test runs with the solver linked into the core.
pytestmark = pytest.mark.usefixtures("only_highs")


@pytest.fixture
def only_highs(monkeypatch: pytest.MonkeyPatch) -> Generator[None]:
    """Keep the solver of a test run from reaching a gurobipy on the machine."""
    monkeypatch.setattr("mqt.scpd.cli.register_external_solver", lambda: None)
    yield
    pyscpd.set_solver(None)


def test_doctor_reports_and_exits_zero(capsys: pytest.CaptureFixture[str]) -> None:
    """The doctor prints its report and its verdict."""
    assert main(["doctor", "-c", CONFIG]) == 0
    out = capsys.readouterr().out
    assert "doctor: OK" in out
    assert "launcher          4" in out
    assert "all_outer: 5 ports, entering at Q1.port0" in out


def test_the_chip_option_replaces_the_configured_input(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """--chip reads the chip from where the option says, for every command that loads it."""
    config = tmp_path / "config.toml"
    config.write_text(Path(CONFIG).read_text(encoding="utf-8"), encoding="utf-8")
    chip = str(Path(CONFIG).parent / "routing_config.json")
    assert main(["doctor", "-c", str(config)]) == 1
    capsys.readouterr()

    assert main(["doctor", "-c", str(config), "--chip", chip]) == 0
    assert f"chip input: {chip}" in capsys.readouterr().out
    output = tmp_path / "chip.svg"
    assert main(["plot", "-c", str(config), "--chip", chip, "-o", str(output)]) == 0
    assert output.is_file()
    if HAS_KLAYOUT:
        assert main(["render", "-c", str(config), "--chip", chip, "-o", str(tmp_path / "chip.gds")]) == 0


def test_every_command_reads_the_chip_copy_beside_the_configuration_of_a_run(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """The config.toml of a run names the original chip input; every command takes the run's chip copy instead."""
    run = tmp_path / "run"
    assert plan(run, "--stop-after", "capacity") == 0
    capsys.readouterr()
    config = str(run / "config.toml")

    assert main(["doctor", "-c", config]) == 0
    assert f"chip input: {run / '00-chip.json'}" in capsys.readouterr().out
    assert main(["plot", "-c", config, "-o", str(tmp_path / "chip.svg")]) == 0
    assert (tmp_path / "chip.svg").is_file()
    if HAS_KLAYOUT:
        assert main(["render", "-c", config, "-o", str(tmp_path / "chip.gds")]) == 0

    other = tmp_path / "other"
    assert main(["plan", "-c", config, "-o", str(other), "--no-progress", "--stop-after", "capacity"]) == 0
    assert (other / "00-chip.json").read_bytes() == (run / "00-chip.json").read_bytes()
    assert (other / "01-capacity.fb").is_file()


def test_plan_reports_a_value_the_core_refuses_against_the_configuration(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """A value that the core refuses names the configuration file, for a new run and for a resumed step."""
    config = stopping_after(tmp_path, "capacity")
    refused = config.read_text(encoding="utf-8") + "\n[stages.solver]\ntime_limit = -1.0\n"
    config.write_text(refused, encoding="utf-8")
    problem = "stages.solver: time_limit must not be negative"
    run = tmp_path / "run"

    assert main(["plan", "-c", str(config), "-o", str(run), "--no-progress"]) == 1
    assert capsys.readouterr().err == f"✗ {config}: {problem}\n"
    assert not run.exists()

    assert plan(run, "--stop-after", "capacity") == 0
    capsys.readouterr()
    (run / "config.toml").write_text(refused, encoding="utf-8")
    assert main(["plan", "-o", str(run), "--stage", "capacity", "--no-progress"]) == 1
    assert capsys.readouterr().err == f"✗ {run / 'config.toml'}: {problem}\n"


def test_plot_writes_the_layout_and_refuses_later_stages(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """The layout stage renders; a stage of a later phase names that phase."""
    output = tmp_path / "layout.svg"
    assert main(["plot", "-c", CONFIG, "--stage", "layout", "-o", str(output), "--width", "800"]) == 0
    assert output.read_text(encoding="utf-8").startswith("<svg")
    assert capsys.readouterr().out.startswith(f"✓ {output}  ")

    assert main(["plot", "-c", CONFIG, "--stage", "aligned", "-o", str(output)]) == 1
    assert "✗ stage 'aligned' arrives with phase 4" in capsys.readouterr().err


def test_render_writes_a_layout_file(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """The unrouted chip is written as GDSII where KLayout is installed, and refused with a hint where not."""
    output = tmp_path / "chip.gds"
    code = main(["render", "-c", CONFIG, "-o", str(output)])
    captured = capsys.readouterr()
    if not HAS_KLAYOUT:
        assert code == 1
        assert "mqt-scpd[klayout]" in captured.err
        return
    assert code == 0
    assert output.stat().st_size > 0
    assert "GDS2 · 4 polygons · 9 ports" in captured.out

    assert main(["render", "-c", CONFIG, "-o", str(tmp_path / "chip.svg")]) == 1
    assert "the suffix must be one of" in capsys.readouterr().err


def test_inspect_prints_an_artifact_as_json(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """An artifact file prints as JSON to stdout or to a file."""
    artifact = tmp_path / "03-global.fb"
    artifact.write_bytes(
        write_artifact(
            ArtifactT(
                producer="test",
                outputType=StageOutput.GlobalRouting,
                output=GlobalRoutingT(lattices=[], connections=[], outerRing=[], resonators=[]),
            )
        )
    )

    assert main(["inspect", str(artifact)]) == 0
    document = json.loads(capsys.readouterr().out)
    assert document == {
        "producer": "test",
        "output_type": "GlobalRouting",
        "output": {"lattices": [], "connections": [], "outer_ring": [], "resonators": []},
    }

    output = tmp_path / "03-global.json"
    assert main(["inspect", str(artifact), "-o", str(output)]) == 0
    assert json.loads(output.read_text(encoding="utf-8")) == document
    assert capsys.readouterr().out.startswith(f"✓ {output}  ")


def test_inspect_stops_quietly_when_the_reader_goes_away(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    """A reader such as ``head`` that closes the pipe early is not an error to report."""
    artifact = tmp_path / "03-global.fb"
    artifact.write_bytes(
        write_artifact(
            ArtifactT(
                producer="test",
                outputType=StageOutput.GlobalRouting,
                output=GlobalRoutingT(lattices=[], connections=[], outerRing=[], resonators=[]),
            )
        )
    )
    read, written = os.pipe()
    os.close(read)
    with os.fdopen(written, "w", encoding="utf-8") as closed:
        monkeypatch.setattr(sys, "stdout", closed)
        assert main(["inspect", str(artifact)]) == 1
        monkeypatch.undo()
    assert not capsys.readouterr().err


def test_plan_stops_quietly_when_the_reader_goes_away(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    """The header of ``plan`` goes to a pipe as well, and a reader that leaves before it is no error to report."""
    read, written = os.pipe()
    os.close(read)
    with os.fdopen(written, "w", encoding="utf-8") as closed:
        monkeypatch.setattr(sys, "stdout", closed)
        assert main(["plan", "-c", CONFIG, "-o", str(tmp_path / "run"), "--no-progress"]) == 1
        monkeypatch.undo()
    assert not capsys.readouterr().err


class _ReaderLeavesDuringAStage(io.StringIO):
    """Standard output whose reader goes away once the first stage block has begun."""

    def __init__(self) -> None:
        """Start with a reader."""
        super().__init__()
        self.stage_began = False

    def write(self, text: str) -> int:
        """Take the text until the stage block has begun, then refuse it as a pipe without a reader does.

        Returns:
            The number of characters taken.

        Raises:
            BrokenPipeError: Once the stage block has begun.
        """
        if self.stage_began:
            raise BrokenPipeError(errno.EPIPE, os.strerror(errno.EPIPE))
        self.stage_began = "capacity" in text
        return super().write(text)

    @staticmethod
    def fileno() -> int:
        """A descriptor number that main() can point at the null device.

        Returns:
            The number.
        """
        return 99


def test_plan_stops_quietly_when_the_reader_goes_away_during_a_stage(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    """A reader that leaves while a stage writes its lines is no failure of the stage.

    main() handles it as for every other command, and points standard output at the null device so
    that Python does not report the pipe again when it exits.
    """
    redirected: list[int] = []
    monkeypatch.setattr(os, "dup2", lambda _source, target: redirected.append(target))
    monkeypatch.setattr(sys, "stdout", _ReaderLeavesDuringAStage())

    assert main(["plan", "-c", CONFIG, "-o", str(tmp_path / "run"), "--no-progress"]) == 1
    monkeypatch.undo()
    assert redirected == [99]
    assert not capsys.readouterr().err


class _PipeWithoutReaderOnWindows(io.StringIO):
    """Standard output as Windows gives it once the reader of the pipe went away."""

    @staticmethod
    def write(text: str) -> int:
        """Refuse the text as Windows does.

        Raises:
            OSError: Always, with the error number that Windows gives.
        """
        del text
        raise OSError(errno.EINVAL, "Invalid argument")


def _global_artifact(tmp_path: Path) -> Path:
    """An artifact of the global step without wires.

    Returns:
        Its path.
    """
    artifact = tmp_path / "03-global.fb"
    artifact.write_bytes(
        write_artifact(
            ArtifactT(
                producer="test",
                outputType=StageOutput.GlobalRouting,
                output=GlobalRoutingT(lattices=[], connections=[], outerRing=[], resonators=[]),
            )
        )
    )
    return artifact


@pytest.mark.parametrize("command", ["inspect", "list-algorithms", "doctor", "plan"])
def test_a_command_stops_quietly_when_the_reader_goes_away_on_windows(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch, command: str
) -> None:
    """Windows reports a pipe without a reader as an invalid argument, which is not an error to report either."""
    arguments = {
        "inspect": ["inspect", str(_global_artifact(tmp_path))],
        "list-algorithms": ["list-algorithms"],
        "doctor": ["doctor", "-c", CONFIG],
        "plan": ["plan", "-c", CONFIG, "-o", str(tmp_path / "run"), "--no-progress"],
    }[command]
    monkeypatch.setattr(sys, "platform", "win32")
    monkeypatch.setattr(sys, "stdout", _PipeWithoutReaderOnWindows())

    assert main(arguments) == 1
    monkeypatch.undo()
    assert not capsys.readouterr().err


def test_an_invalid_argument_elsewhere_than_windows_is_an_error(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    """Only Windows reports a pipe without a reader as an invalid argument; elsewhere it is an error to report."""
    artifact = _global_artifact(tmp_path)
    monkeypatch.setattr(sys, "platform", "linux")
    monkeypatch.setattr(sys, "stdout", _PipeWithoutReaderOnWindows())

    assert main(["inspect", str(artifact)]) == 1
    monkeypatch.undo()
    assert capsys.readouterr().err.startswith("✗ ")


def test_problems_exit_with_one(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """A missing configuration and bytes that do not hold an artifact end with exit code 1 and a message."""
    assert main(["plot", "-c", str(tmp_path / "absent.toml"), "-o", str(tmp_path / "x.svg")]) == 1
    assert capsys.readouterr().err.startswith("✗ cannot read")

    broken = tmp_path / "broken.fb"
    broken.write_bytes(b"not an artifact")
    assert main(["inspect", str(broken)]) == 1
    assert capsys.readouterr().err.startswith("✗ ")

    assert main(["doctor", "-c", str(tmp_path / "absent.toml")]) == 1
    assert "doctor: 1 problem(s)" in capsys.readouterr().out


@pytest.mark.parametrize("option", [["--width", "0"], ["--width", "-5"], ["--tolerance", "-1"], ["--tolerance", "nan"]])
def test_plot_refuses_a_picture_without_size(tmp_path: Path, option: list[str]) -> None:
    """A width that is not positive or a negative tolerance is an argument error."""
    with pytest.raises(SystemExit) as info:
        main(["plot", "-c", CONFIG, "-o", str(tmp_path / "x.svg"), *option])
    assert info.value.code == 2


def test_wrong_arguments_print_the_usage_and_exit_with_two(capsys: pytest.CaptureFixture[str]) -> None:
    """Wrong arguments give the argparse usage message, not a line that starts with ✗."""
    with pytest.raises(SystemExit) as info:
        main(["plot", "-c", CONFIG])
    assert info.value.code == 2

    err = capsys.readouterr().err
    assert err.startswith("usage: ")
    assert "mqt-scpd plot: error: " in err


def plan(run: Path, *options: str) -> int:
    """Plan the fixture into a run directory, without the live line.

    Returns:
        The exit code.
    """
    return main(["plan", "-c", CONFIG, "-o", str(run), "--no-progress", *options])


def untimed(text: str, run: Path) -> list[str]:
    """The lines of a run's output without their times, with the run path and the solver version masked.

    Returns:
        The lines.
    """
    text = text.replace(str(run), "RUN")
    text = re.sub(r"HiGHS \d+\.\d+\.\d+", "HiGHS x.y.z", text)
    text = re.sub(r"\d+(\.\d+)? (B|kB|MB)\b", "SIZE", text)
    time = r"(\d+\.\d\ds|\d+m \d\ds|\d+h \d\dm)"
    lines = [re.sub(rf" {{2,}}{time}$", "", line) for line in text.splitlines()]
    return [re.sub(rf" · {time}(?= ·)", "", line) for line in lines]


def test_plan_runs_every_step_into_the_run_directory(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """Without -v a run shows one block per stage with its result, its rounds and its artifact."""
    run = tmp_path / "run"

    assert plan(run) == 0

    assert sorted(path.name for path in run.iterdir()) == [
        "00-chip.json",
        "01-capacity.fb",
        "01-capacity.inputs.json",
        "02-global.fb",
        "02-global.inputs.json",
        "03-assign.fb",
        "03-assign.inputs.json",
        "04-corridor.fb",
        "04-corridor.inputs.json",
        "05-detail.fb",
        "05-detail.inputs.json",
        "06-final.fb",
        "06-final.inputs.json",
        "config.toml",
    ]
    output = "\n".join(untimed(capsys.readouterr().out, run))
    assert re.fullmatch("\n".join(PLAN_OUTPUT), output), output


def test_every_line_of_a_run_follows_the_format(tmp_path: Path, capfd: pytest.CaptureFixture[str]) -> None:
    """At every level each line has the form docs/terminal_output.md gives it, and nothing writes past it."""
    run = tmp_path / "run"
    assert plan(run, "-vvv") == 0

    out, err = capfd.readouterr()
    assert not err
    time = r"(\d+\.\d\ds|\d+m \d\ds|\d+h \d\dm)"
    forms = [
        r"mqt-scpd plan \S+ → \S+  ·  .+",
        r"",
        r"(🧩|🔳|📌|🧭|🧵|🧷) (capacity|global|assign|corridor|detail|final)",
        rf"   \S.* {{2,}}{time}",
        rf"     \S.* {{2,}}{time}",
        r"     .+",
        r"   ✓ \d\d-[a-z]+\.fb  \d+(\.\d)? (B|kB|MB)( · fails \d+)?",
        rf"✓ 6 stages · {time} · stopped after final · fails 0",
    ]
    for line in out.splitlines():
        assert any(re.fullmatch(form, line) for form in forms), line
    # Each level adds to the one below it.
    assert re.search(r"^     model assignment · ", out, re.MULTILINE)
    assert re.search(r"^     chain 1 ", out, re.MULTILINE)
    assert "HiGHS" in out.split("📌 assign")[1]


@pytest.mark.parametrize("level", [0, 1, 2, 3], ids=["none", "-v", "-vv", "-vvv"])
def test_each_level_shows_its_lines_and_hides_the_ones_above(
    tmp_path: Path, capfd: pytest.CaptureFixture[str], level: int
) -> None:
    """-v adds the steps, -vv the items and -vvv the HiGHS log, as docs/terminal_output.md says."""
    run = tmp_path / "run"
    assert plan(run, *(["-" + "v" * level] if level else [])) == 0

    out = capfd.readouterr().out
    assert bool(re.search(r"^     model assignment · ", out, re.MULTILINE)) == (level >= 1)
    assert bool(re.search(r"^     chain 1 ", out, re.MULTILINE)) == (level >= 2)
    assert ("HiGHS" in out.split("📌 assign")[1]) == (level >= 3)


def test_plan_stops_after_the_step_it_is_told(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """--stop-after ends the run after that step and says so."""
    run = tmp_path / "run"

    assert plan(run, "--stop-after", "global") == 0

    assert not (run / "03-assign.fb").exists()
    assert (run / "02-global.fb").is_file()
    assert untimed(capsys.readouterr().out, run)[-1] == "✓ 2 stages · stopped after global · fails 0"


def test_plan_that_ends_with_fails_exits_with_one(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """A run whose stages all finish but leave fails ends with a cross and exit code 1."""
    config = stopping_after(tmp_path, "corridor")
    config.write_text(
        config.read_text(encoding="utf-8") + "\n[stages.capacity]\ncrossing_pitch = 1500.0\n", encoding="utf-8"
    )
    run = tmp_path / "run"

    assert main(["plan", "-c", str(config), "-o", str(run), "--no-progress"]) == 1

    assert (run / "04-corridor.fb").is_file()
    assert re.fullmatch(
        r"✗ 4 stages · stopped after corridor · fails [1-9]\d*", untimed(capsys.readouterr().out, run)[-1]
    )


@pytest.mark.parametrize("name", ["config.toml", "./config.toml"])
def test_the_header_names_the_chip_of_a_configuration_in_the_working_directory(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch, name: str
) -> None:
    """A config.toml named without its directory still names the chip after that directory."""
    chip = tmp_path / "mini"
    chip.mkdir()
    for source in Path(CONFIG).parent.iterdir():
        (chip / source.name).write_bytes(source.read_bytes())
    monkeypatch.chdir(chip)

    assert main(["plan", "-c", name, "-o", str(tmp_path / "run"), "--stop-after", "capacity", "--no-progress"]) == 0

    assert capsys.readouterr().out.splitlines()[0].startswith("mqt-scpd plan mini → ")


def test_the_configured_stop_holds_unless_the_command_says_otherwise(tmp_path: Path) -> None:
    """[run] stop_after sets where a run ends, and --stop-after overrides it."""
    config = stopping_after(tmp_path, "capacity")

    assert main(["plan", "-c", str(config), "-o", str(tmp_path / "configured"), "--no-progress"]) == 0
    assert sorted(path.name for path in (tmp_path / "configured").glob("*.fb")) == ["01-capacity.fb"]

    over = tmp_path / "overridden"
    assert main(["plan", "-c", str(config), "-o", str(over), "--no-progress", "--stop-after", "assign"]) == 0
    assert sorted(path.name for path in over.glob("*.fb")) == ["01-capacity.fb", "02-global.fb", "03-assign.fb"]


def test_a_step_resumes_a_run_from_its_own_copies(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """--stage runs one step on the configuration and chip the run carries."""
    run = tmp_path / "run"
    assert plan(run, "--stop-after", "capacity") == 0
    capsys.readouterr()

    assert main(["plan", "-o", str(run), "--stage", "global", "--no-progress"]) == 0

    assert (run / "02-global.fb").is_file()
    lines = untimed(capsys.readouterr().out, run)
    assert lines[0] == "mqt-scpd plan run → RUN  ·  HiGHS x.y.z"
    assert lines[-1] == "✓ 1 stage · stopped after global · fails 0"


def test_a_run_directory_named_from_inside_it_still_names_the_chip(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    """A run directory given as `.` names the chip after the directory, in the header and in the picture."""
    run = tmp_path / "run"
    assert plan(run, "--stop-after", "capacity") == 0
    capsys.readouterr()
    monkeypatch.chdir(run)

    assert main(["plan", "-o", ".", "--stage", "global", "--no-progress"]) == 0
    assert capsys.readouterr().out.splitlines()[0].startswith("mqt-scpd plan run → . ")

    output = tmp_path / "global.svg"
    assert main(["plot", "--run-dir", ".", "--stage", "global", "-o", str(output)]) == 0
    assert '<text class="title" ' in output.read_text(encoding="utf-8")
    assert '">run</text>' in output.read_text(encoding="utf-8")


@pytest.mark.parametrize("stage", ["global", "assign", "corridor"])
def test_a_step_refuses_a_run_whose_design_rules_changed(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], stage: str
) -> None:
    """A design rule edited in the run's config.toml stops every later step until the capacity step runs again."""
    run = tmp_path / "run"
    assert plan(run) == 0
    before = {path.name: path.read_bytes() for path in run.glob("0[1-4]-*")}
    config = run / "config.toml"
    config.write_text(
        config.read_text(encoding="utf-8").replace("min_straight_length = 100.0", "min_straight_length = 300.0"),
        encoding="utf-8",
    )
    capsys.readouterr()

    assert main(["plan", "-o", str(run), "--stage", stage, "--no-progress"]) == 1

    lines = untimed(capsys.readouterr().out, run)
    assert lines[3] == (
        "   ✗ [design_rules] in config.toml changed after the capacity step wrote 01-capacity.fb; "
        "run the capacity step again"
    )
    assert {path.name: path.read_bytes() for path in run.glob("0[1-4]-*")} == before

    assert main(["plan", "-o", str(run), "--stage", "capacity", "--no-progress"]) == 0
    for step in ("global", "assign", "corridor"):
        assert main(["plan", "-o", str(run), "--stage", step, "--no-progress"]) == 0


def test_a_step_without_its_input_fails_in_its_block(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """A missing input is a line of the block and a closing cross, not a traceback."""
    run = tmp_path / "run"
    assert plan(run, "--stop-after", "capacity") == 0
    capsys.readouterr()

    assert main(["plan", "-o", str(run), "--stage", "assign", "--no-progress"]) == 1

    lines = untimed(capsys.readouterr().out, run)
    assert lines[2:4] == ["📌 assign", "   ✗ 02-global.fb is missing; run the global step first"]
    assert lines[-1].startswith("✗ stopped at assign after ")


def test_an_error_of_the_core_is_a_line_and_not_a_traceback(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """A stage that the core refuses ends the run with the reason in its block."""
    run = tmp_path / "run"
    assert plan(run, "--stop-after", "global") == 0
    # Without its resonators, the ring carries five conventional ports, and the fixture has four
    # launchers. Only the assignment sees the ring that the global step wrote.
    artifact = read_artifact((run / "02-global.fb").read_bytes())
    assert isinstance(artifact.output, GlobalRoutingT)
    artifact.output.resonators = []
    (run / "02-global.fb").write_bytes(write_artifact(artifact))
    capsys.readouterr()

    assert main(["plan", "-o", str(run), "--stage", "assign", "--no-progress"]) == 1

    out = capsys.readouterr().out
    assert "   ✗ the ring carries 5 conventional ports and the assignment places 0 launcher ends, which need 5" in out
    assert "Traceback" not in out
    assert not (run / "03-assign.fb").exists()


def with_launcher_target(directory: Path, target: int) -> Path:
    """Write the fixture configuration with another ``[stages.assignment] launcher_target``.

    Returns:
        The path of the configuration.
    """
    config = directory / "config.toml"
    text = Path(CONFIG).read_text(encoding="utf-8")
    text = text.replace(
        'input = "routing_config.json"', f'input = "{(Path(CONFIG).parent / "routing_config.json").as_posix()}"'
    )
    config.write_text(text.replace("launcher_target = 1", f"launcher_target = {target}"), encoding="utf-8")
    return config


def test_doctor_and_plan_refuse_more_launcher_ends_than_the_chip_has_launchers(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """Counts that every ring of the global step breaks stop the doctor and the run before any stage."""
    # The three conventional ports and five launcher ends need eight launchers, and the fixture has four.
    config = with_launcher_target(tmp_path, 5)
    problem = (
        "the ring carries at least 3 conventional ports and the assignment places 5 launcher ends, "
        "which need at least 8 launchers, but the chip has 4"
    )

    assert main(["doctor", "-c", str(config)]) == 1
    assert problem in capsys.readouterr().out

    run = tmp_path / "run"
    assert plan(run) == 0
    before = sorted(path.name for path in run.glob("*.fb"))
    capsys.readouterr()
    assert main(["plan", "-c", str(config), "-o", str(run), "--no-progress"]) == 1
    assert problem in capsys.readouterr().err
    assert sorted(path.name for path in run.glob("*.fb")) == before

    # A run that ends before the assignment does not need the counts.
    assert main(["plan", "-c", str(config), "-o", str(run), "--stop-after", "global", "--no-progress"]) == 0


def test_doctor_and_plan_warn_about_a_key_that_no_stage_reads(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """A removed key fails no check, and the doctor and the run name it so that it can go."""
    config = with_launcher_target(tmp_path, 1)
    text = config.read_text(encoding="utf-8")
    config.write_text(text.replace("capacity_cells_x = 12", "capacity_cells_x = 12\nlauncher_offset_x = 0"))
    warning = "[grid] launcher_offset_x has no effect, and a later version will refuse it; remove it"

    assert main(["doctor", "-c", str(config)]) == 0
    assert f"WARNING: {warning}" in capsys.readouterr().out

    run = tmp_path / "run"
    assert main(["plan", "-c", str(config), "-o", str(run), "--stop-after", "capacity", "--no-progress"]) == 0
    assert warning in capsys.readouterr().out


def test_doctor_and_plan_refuse_a_ring_that_carries_both_ends_of_a_bridge(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """A bridge with both ends in the ring stops the doctor and a run that reaches the global step."""
    chip = json.loads((Path(CONFIG).parent / "routing_config.json").read_text(encoding="utf-8"))
    # Two ports on the coupler bar, which a bridge rule pairs.
    chip["ports"]["C12.port1"] = {"center": [900.0, 270.0], "orientation": 90.0}
    chip["ports"]["C12.port2"] = {"center": [900.0, 130.0], "orientation": 270.0}
    (tmp_path / "chip.json").write_text(json.dumps(chip), encoding="utf-8")
    text = Path(CONFIG).read_text(encoding="utf-8")
    text = text.replace('input = "routing_config.json"', 'input = "chip.json"')
    text = text.replace(
        "[ports.sequences]",
        "component = '^([^.]+)\\.port\\d+$'\n"
        "bridge_pair = '^C\\d+\\.port[12]$'\n\n"
        "[[ports.bridge_pairs]]\n"
        "first = '^(C\\d+)\\.port1$'\n"
        "second = '^(C\\d+)\\.port2$'\n\n"
        "[ports.sequences]",
    )
    text = text.replace('"Q2.port1", "Q2.port0"]', '"Q2.port1", "Q2.port0", "C12.port1", "C12.port2"]')
    config = tmp_path / "config.toml"
    config.write_text(text, encoding="utf-8")
    problem = "the ring carries both ends of the bridge 'C12.port1' - 'C12.port2'"

    assert main(["doctor", "-c", str(config)]) == 1
    assert problem in capsys.readouterr().out

    run = tmp_path / "run"
    assert main(["plan", "-c", str(config), "-o", str(run), "--no-progress"]) == 1
    assert problem in capsys.readouterr().err
    assert not list(run.glob("*.fb"))

    # A run that ends before the global step does not derive the inner circuit.
    assert main(["plan", "-c", str(config), "-o", str(run), "--stop-after", "capacity", "--no-progress"]) == 0


def test_doctor_and_plan_refuse_a_detail_grid_above_the_limit(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """A detail grid of more than 25,000,000 cells stops the doctor and a run before any stage."""
    config = with_launcher_target(tmp_path, 1)
    text = config.read_text(encoding="utf-8")
    config.write_text(
        text.replace("capacity_cells_x = 12", "capacity_cells_x = 100\ncapacity_cells_y = 100\ndetail_factor = 60"),
        encoding="utf-8",
    )
    times = "\N{MULTIPLICATION SIGN}"
    problem = (
        f"[grid] asks for a detail grid of 6000 {times} 6000 = 36000000 cells, a capacity grid of 100 {times} 100 "
        "cells times detail_factor 60; the limit is 25000000 cells"
    )

    assert main(["doctor", "-c", str(config)]) == 1
    assert problem in capsys.readouterr().out

    run = tmp_path / "run"
    assert main(["plan", "-c", str(config), "-o", str(run), "--no-progress"]) == 1
    assert problem in capsys.readouterr().err
    assert not list(run.glob("*.fb"))


def test_launchers_listed_with_the_ring_are_refused_before_any_stage(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """The doctor and a run that reaches the assignment refuse launchers listed with the ring."""
    chip = json.loads((Path(CONFIG).parent / "routing_config.json").read_text(encoding="utf-8"))
    ports = chip["ports"]
    # Chip.port1 and Chip.port3 change places, so the launchers run counter-clockwise, as the ring.
    ports["Chip.port1"], ports["Chip.port3"] = ports["Chip.port3"], ports["Chip.port1"]
    swapped = tmp_path / "chip.json"
    swapped.write_text(json.dumps(chip), encoding="utf-8")
    problem = "the chip input lists the launcher ports in the direction of the port ring"

    assert main(["doctor", "-c", CONFIG, "--chip", str(swapped)]) == 1
    assert problem in capsys.readouterr().out

    run = tmp_path / "run"
    assert main(["plan", "-c", CONFIG, "--chip", str(swapped), "-o", str(run), "--no-progress"]) == 1
    assert problem in capsys.readouterr().err
    assert not list(run.glob("*.fb"))

    # A run that ends before the assignment does not need the order.
    assert plan(run, "--chip", str(swapped), "--stop-after", "global") == 0


def test_a_refused_run_keeps_the_run_it_would_replace(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """A configuration that a step would refuse leaves the artifacts of the run directory alone."""
    run = tmp_path / "run"
    assert plan(run, "--stop-after", "global") == 0
    before = sorted(path.name for path in run.glob("*.fb"))

    config = tmp_path / "config.toml"
    text = Path(CONFIG).read_text(encoding="utf-8")
    text = text.replace(
        'input = "routing_config.json"', f'input = "{(Path(CONFIG).parent / "routing_config.json").as_posix()}"'
    )
    config.write_text(text.replace("launcher_target = 1", "launcher_target = 0"), encoding="utf-8")
    assert main(["plan", "-c", str(config), "-o", str(run), "--no-progress"]) == 1

    assert "[stages.assignment] launcher_target is missing" in capsys.readouterr().err
    assert sorted(path.name for path in run.glob("*.fb")) == before == ["01-capacity.fb", "02-global.fb"]


def test_plan_refuses_a_file_as_its_run_directory(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """A run directory that is a file is refused with a line that says so, and the file stays."""
    afile = tmp_path / "afile"
    afile.write_text("notes", encoding="utf-8")

    assert plan(afile) == 1

    assert f"{afile} is not a run directory: it is a file" in capsys.readouterr().err
    assert afile.read_text(encoding="utf-8") == "notes"


def test_resuming_takes_neither_a_configuration_nor_a_chip(tmp_path: Path) -> None:
    """A resumed step reads the copies of the run, so -c with --stage is an argument error."""
    with pytest.raises(SystemExit) as info:
        main(["plan", "-c", CONFIG, "-o", str(tmp_path), "--stage", "global"])
    assert info.value.code == 2
    with pytest.raises(SystemExit) as info:
        main(["plan", "-o", str(tmp_path), "--stage", "global", "--chip", "chip.json"])
    assert info.value.code == 2


def test_list_algorithms_names_one_line_per_stage(capsys: pytest.CaptureFixture[str]) -> None:
    """Every stage is listed with the implementations this build ships."""
    assert main(["list-algorithms"]) == 0
    assert capsys.readouterr().out.splitlines() == [
        "capacity-planner   watershed",
        "global-router      hanan-milp",
        "assigner           ordered-milp",
        "corridor-router    partition-astar",
    ]


@pytest.mark.usefixtures("vector_type")
def test_plot_draws_a_planning_stage_of_a_run(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """A planning stage is drawn from the run directory, over the chip the run carries.

    The assign and corridor pictures show the feedline chains of the run.
    """
    run = tmp_path / "run"
    assert plan(run) == 0
    capsys.readouterr()
    output = tmp_path / "corridor.svg"

    assert main(["plot", "--run-dir", str(run), "--stage", "corridor", "-o", str(output)]) == 0

    svg = output.read_text(encoding="utf-8")
    assert '<g class="l-corridor">' in svg
    assert '<g class="l-border">' in svg
    assert '<g class="l-feedline">' in svg
    assert capsys.readouterr().out.startswith(f"✓ {output}  ")

    assert main(["plot", "--run-dir", str(run), "--stage", "assign", "-o", str(output)]) == 0
    assert '<g class="l-feedline">' in output.read_text(encoding="utf-8")


@pytest.mark.parametrize(("stage", "drawn"), [("global", None), ("assign", "global"), ("corridor", "global")])
def test_plot_draws_a_stage_with_the_artifacts_it_lies_over(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch, stage: str, drawn: str | None
) -> None:
    """The global picture lies over the blocked cells; the assign and corridor pictures show the inner circuit."""
    run = tmp_path / "run"
    assert plan(run) == 0
    capsys.readouterr()
    handed: list[bytes | None] = []

    def planning_geometry(
        data: bytes,
        chip: ChipT,
        stage: str,
        capacity: bytes | None = None,
        assignment: bytes | None = None,
        global_routing: bytes | None = None,
        clearance: float = 0.0,
        phase: str | None = None,
    ) -> PlanningGeometry:
        handed.append(global_routing)
        return drawing(data, chip, stage, capacity, assignment, global_routing, clearance, phase)

    monkeypatch.setattr(cli, "planning_geometry", planning_geometry)
    output = tmp_path / f"{stage}.svg"

    assert main(["plot", "--run-dir", str(run), "--stage", stage, "-o", str(output)]) == 0

    expected = None if drawn is None else (run / "02-global.fb").read_bytes()
    assert handed == [expected]
    if stage == "global":
        assert '<g class="l-raster">' in output.read_text(encoding="utf-8")


def test_a_planning_stage_needs_a_run(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """A planning stage without a run directory, or one whose step has not run, is refused."""
    with pytest.raises(SystemExit) as info:
        main(["plot", "-c", CONFIG, "--stage", "corridor", "-o", str(tmp_path / "x.svg")])
    assert info.value.code == 2

    run = tmp_path / "run"
    assert plan(run, "--stop-after", "capacity") == 0
    capsys.readouterr()
    assert main(["plot", "--run-dir", str(run), "--stage", "corridor", "-o", str(tmp_path / "x.svg")]) == 1
    assert "04-corridor.fb is missing" in capsys.readouterr().err


def test_a_picture_refuses_an_artifact_the_run_inputs_no_longer_make(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    """A step that failed keeps its old artifact without a record, and a changed input outdates every artifact.

    Either way the picture of the step is refused, since it would show what the run inputs no longer make.
    """
    run = tmp_path / "run"
    assert plan(run, "--stop-after", "assign") == 0
    capsys.readouterr()
    output = tmp_path / "assign.svg"

    def broken(*args: object) -> bytes:
        del args
        msg = "the assignment broke"
        raise RuntimeError(msg)

    with monkeypatch.context() as patched:
        patched.setattr(pyscpd, "assign", broken)
        assert main(["plan", "-o", str(run), "--stage", "assign", "--no-progress"]) == 1
    capsys.readouterr()
    assert (run / "03-assign.fb").is_file()
    assert main(["plot", "--run-dir", str(run), "--stage", "assign", "-o", str(output)]) == 1
    assert "03-assign.fb carries no record of the inputs it was made from" in capsys.readouterr().err
    assert not output.exists()
    # inspect still prints the artifact, after a warning.
    assert main(["inspect", str(run / "03-assign.fb")]) == 0
    captured = capsys.readouterr()
    assert json.loads(captured.out)["output_type"] == "Assignment"
    assert "⚠ 03-assign.fb carries no record of the inputs it was made from" in captured.err

    assert main(["plan", "-o", str(run), "--stage", "assign", "--no-progress"]) == 0
    capsys.readouterr()
    assert main(["plot", "--run-dir", str(run), "--stage", "assign", "-o", str(output)]) == 0
    capsys.readouterr()
    assert main(["inspect", str(run / "03-assign.fb")]) == 0
    assert not capsys.readouterr().err

    config = run / "config.toml"
    config.write_text(
        config.read_text(encoding="utf-8") + "\n[stages.capacity]\ncrossing_pitch = 120.0\n", encoding="utf-8"
    )
    assert main(["plot", "--run-dir", str(run), "--stage", "assign", "-o", str(output)]) == 1
    assert "[stages.capacity] in config.toml changed after the capacity step" in capsys.readouterr().err


@pytest.mark.skipif(not HAS_KLAYOUT, reason="the export needs KLayout")
@pytest.mark.usefixtures("vector_type")
def test_render_writes_a_planning_stage_of_a_run(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """A planning stage goes into the layout file beside the artwork."""
    run = tmp_path / "run"
    assert plan(run, "--stop-after", "capacity") == 0
    capsys.readouterr()
    output = tmp_path / "capacity.gds"

    assert main(["render", "--run-dir", str(run), "--stage", "capacity", "-o", str(output)]) == 0

    assert output.stat().st_size > 0
    assert "planning shapes" in capsys.readouterr().out


def test_an_interrupted_run_says_where_it_stopped(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    """Ctrl-C ends a run with a cross in the block of the stage, a closing line and exit code 130."""

    def interrupted(*args: object, **kwargs: object) -> None:
        del args, kwargs
        raise KeyboardInterrupt

    monkeypatch.setattr(RunDirectory, "run_stage", interrupted)
    run = tmp_path / "run"

    assert plan(run) == 130

    lines = untimed(capsys.readouterr().out, run)
    assert lines[2:4] == ["🧩 capacity", "   ✗ interrupted"]
    assert lines[-1].startswith("✗ stopped at capacity after ")


def test_an_unknown_solver_is_a_line_and_not_a_traceback(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    """A backend name the core does not know ends the command with a cross and exit code 1."""
    monkeypatch.setenv("SCPD_SOLVER", "cplex")

    assert plan(tmp_path / "run") == 1

    captured = capsys.readouterr()
    assert captured.err.startswith("✗ ")
    assert "cplex" in captured.err
    assert "Traceback" not in captured.out + captured.err


def stopping_after(directory: Path, step: str) -> Path:
    """Write the fixture configuration with ``[run] stop_after`` set to a step.

    Returns:
        The path of the configuration.
    """
    config = directory / "config.toml"
    text = Path(CONFIG).read_text(encoding="utf-8")
    text = text.replace(
        'input = "routing_config.json"', f'input = "{(Path(CONFIG).parent / "routing_config.json").as_posix()}"'
    )
    config.write_text(text + f'\n[run]\nstop_after = "{step}"\n', encoding="utf-8")
    return config


def test_doctor_and_plan_refuse_an_unknown_solver_for_a_run_that_solves_nothing(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    """An unknown backend name is a mistake for every run, so the doctor and the run refuse it alike."""
    config = stopping_after(tmp_path, "capacity")
    monkeypatch.setenv("SCPD_SOLVER", "cplex")
    problem = "SCPD_SOLVER: 'cplex' is not a solver backend; use 'highs' or 'gurobi'"

    assert main(["doctor", "-c", str(config)]) == 1
    assert problem in capsys.readouterr().out

    run = tmp_path / "run"
    assert main(["plan", "-c", str(config), "-o", str(run), "--no-progress"]) == 1
    assert problem in capsys.readouterr().err
    assert not list(run.glob("*.fb"))


@pytest.mark.parametrize(("step", "code"), [("global", 0), ("assign", 1)])
def test_doctor_and_plan_need_a_reachable_solver_only_for_a_run_that_solves(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch, step: str, code: int
) -> None:
    """Gurobi without gurobipy stops the doctor and the run where a step solves, and neither elsewhere.

    The fixture's ring carries every routable port, so its global step has no inner circuit to solve.
    The assignment always solves.
    """
    config = stopping_after(tmp_path, step)
    monkeypatch.setenv("SCPD_SOLVER", "gurobi")
    monkeypatch.setitem(sys.modules, "gurobipy", None)
    problem = "SCPD_SOLVER asks for gurobi, but gurobipy is not installed"

    assert main(["doctor", "-c", str(config)]) == code
    assert (problem in capsys.readouterr().out) == (code == 1)

    run = tmp_path / "run"
    assert main(["plan", "-c", str(config), "-o", str(run), "--no-progress"]) == code
    assert (problem in capsys.readouterr().err) == (code == 1)
    written = sorted(path.name for path in run.glob("*.fb"))
    assert written == ([] if code == 1 else ["01-capacity.fb", "02-global.fb"])


@pytest.mark.parametrize(("step", "code"), [("corridor", 0), ("assign", 1)])
def test_a_resumed_step_needs_a_reachable_solver_only_where_it_solves(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch, step: str, code: int
) -> None:
    """Gurobi without gurobipy stops a resumed assignment, and a resumed corridor step runs.

    The run directory keeps the artifacts of a refused step as they were.
    """
    run = tmp_path / "run"
    assert plan(run, "--stop-after", "assign") == 0
    capsys.readouterr()
    assignment = (run / "03-assign.fb").read_bytes()
    monkeypatch.setenv("SCPD_SOLVER", "gurobi")
    monkeypatch.setitem(sys.modules, "gurobipy", None)
    problem = "SCPD_SOLVER asks for gurobi, but gurobipy is not installed"

    assert main(["plan", "-o", str(run), "--stage", step, "--no-progress"]) == code

    assert (problem in capsys.readouterr().err) == (code == 1)
    assert (run / "03-assign.fb").read_bytes() == assignment
    assert (run / "04-corridor.fb").is_file() == (code == 0)


def test_any_error_of_a_stage_ends_the_run_in_its_block(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    """An error of a kind no stage is expected to raise still ends the run with a cross in its block."""

    def broken(*args: object, **kwargs: object) -> None:
        del args, kwargs
        msg = "a solver answered with the wrong shape"
        raise TypeError(msg)

    monkeypatch.setattr(RunDirectory, "run_stage", broken)
    run = tmp_path / "run"

    assert plan(run) == 1

    lines = untimed(capsys.readouterr().out, run)
    assert lines[2:4] == ["🧩 capacity", "   ✗ a solver answered with the wrong shape"]
    assert lines[-1].startswith("✗ stopped at capacity after ")


def test_a_broken_pipe_of_a_stage_itself_ends_the_run_in_its_block(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    """A broken pipe that does not come from the output, such as one of a solver of the user's own, fails the stage."""

    def broken(*args: object, **kwargs: object) -> None:
        del args, kwargs
        msg = "the solver lost its license server"
        raise BrokenPipeError(msg)

    monkeypatch.setattr(RunDirectory, "run_stage", broken)
    run = tmp_path / "run"

    assert plan(run) == 1

    lines = untimed(capsys.readouterr().out, run)
    assert lines[2:4] == ["🧩 capacity", "   ✗ the solver lost its license server"]
    assert lines[-1].startswith("✗ stopped at capacity after ")
