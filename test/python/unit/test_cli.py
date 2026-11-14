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

import pytest

from mqt.scpd import cli, pyscpd
from mqt.scpd.artifacts import write_artifact
from mqt.scpd.cli import main
from mqt.scpd.export import HAS_KLAYOUT
from mqt.scpd.flatbuffers.artifacts.Artifact import ArtifactT
from mqt.scpd.flatbuffers.artifacts.GlobalRouting import GlobalRoutingT
from mqt.scpd.flatbuffers.artifacts.StageOutput import StageOutput
from mqt.scpd.run import BlockReporter, RunDirectory, StageResult, steps_until

CONFIG = str(Path(__file__).resolve().parents[2] / "fixtures" / "mini" / "config.toml")

#: The output of plan on the fixture without -v, one pattern per line, with the times removed and
#: the run path and the file sizes masked. The form of every line is fixed; the figures are what the
#: stages make of the fixture.
PLAN_OUTPUT = (
    r"mqt-scpd plan mini → RUN",
    r"",
    r"🧩 capacity",
    r"   \d+ partitions? · \d+ bottlenecks? · \d+ chains?",
    r"   ✓ 01-capacity\.fb  SIZE",
    r"",
    r"✓ 1 stage · stopped after capacity",
)


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
    refused = config.read_text(encoding="utf-8") + "\n[stages.capacity]\ncrossing_pitch = -1.0\n"
    config.write_text(refused, encoding="utf-8")
    problem = "stages.capacity: crossing_pitch must be positive"
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

    assert main(["plot", "-c", CONFIG, "--stage", "final", "-o", str(output)]) == 1
    assert "✗ stage 'final' arrives with phase 4" in capsys.readouterr().err


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
        write_artifact(ArtifactT(producer="test", outputType=StageOutput.GlobalRouting, output=GlobalRoutingT()))
    )

    assert main(["inspect", str(artifact)]) == 0
    document = json.loads(capsys.readouterr().out)
    assert document == {"producer": "test", "output_type": "GlobalRouting", "output": {}}

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
        write_artifact(ArtifactT(producer="test", outputType=StageOutput.GlobalRouting, output=GlobalRoutingT()))
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


def _empty_artifact(tmp_path: Path) -> Path:
    """An artifact whose output carries no fields.

    Returns:
        Its path.
    """
    artifact = tmp_path / "03-global.fb"
    artifact.write_bytes(
        write_artifact(ArtifactT(producer="test", outputType=StageOutput.GlobalRouting, output=GlobalRoutingT()))
    )
    return artifact


@pytest.mark.parametrize("command", ["inspect", "list-algorithms", "doctor", "plan"])
def test_a_command_stops_quietly_when_the_reader_goes_away_on_windows(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch, command: str
) -> None:
    """Windows reports a pipe without a reader as an invalid argument, which is not an error to report either."""
    arguments = {
        "inspect": ["inspect", str(_empty_artifact(tmp_path))],
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
    artifact = _empty_artifact(tmp_path)
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
    """The lines of a run's output without their times, with the run path masked.

    Returns:
        The lines.
    """
    text = text.replace(str(run), "RUN")
    text = re.sub(r"\d+(\.\d+)? (B|kB|MB)\b", "SIZE", text)
    time = r"(\d+\.\d\ds|\d+m \d\ds|\d+h \d\dm)"
    lines = [re.sub(rf" {{2,}}{time}$", "", line) for line in text.splitlines()]
    return [re.sub(rf" · {time}(?= ·)", "", line) for line in lines]


def test_plan_runs_every_step_into_the_run_directory(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """Without -v a run shows one block per stage with its result and its artifact."""
    run = tmp_path / "run"

    assert plan(run) == 0

    assert sorted(path.name for path in run.iterdir()) == [
        "00-chip.json",
        "01-capacity.fb",
        "01-capacity.inputs.json",
        "config.toml",
    ]
    output = "\n".join(untimed(capsys.readouterr().out, run))
    assert re.fullmatch("\n".join(PLAN_OUTPUT), output), output


def test_every_line_of_a_run_follows_the_format(tmp_path: Path, capfd: pytest.CaptureFixture[str]) -> None:
    """At every level each line has the form docs/terminal_output.md gives it, and nothing writes past it."""
    run = tmp_path / "run"
    assert plan(run, "-vv") == 0

    out, err = capfd.readouterr()
    assert not err
    time = r"(\d+\.\d\ds|\d+m \d\ds|\d+h \d\dm)"
    forms = [
        r"mqt-scpd plan \S+ → \S+",
        r"",
        r"🧩 capacity",
        rf"   \S.* {{2,}}{time}",
        rf"     \S.* {{2,}}{time}",
        r"   ✓ \d\d-[a-z]+\.fb  \d+(\.\d)? (B|kB|MB)( · fails \d+)?",
        rf"✓ 1 stage · {time} · stopped after capacity",
    ]
    for line in out.splitlines():
        assert any(re.fullmatch(form, line) for form in forms), line
    assert re.search(r"^     medial axis of \d+ cells ", out, re.MULTILINE)


@pytest.mark.parametrize("level", [0, 1, 2], ids=["none", "-v", "-vv"])
def test_each_level_shows_its_lines_and_hides_the_ones_above(
    tmp_path: Path, capfd: pytest.CaptureFixture[str], level: int
) -> None:
    """-v adds the steps of a stage, and -vv keeps them, as docs/terminal_output.md says."""
    run = tmp_path / "run"
    assert plan(run, *(["-" + "v" * level] if level else [])) == 0

    out = capfd.readouterr().out
    assert bool(re.search(r"^     medial axis of \d+ cells ", out, re.MULTILINE)) == (level >= 1)


def test_plan_stops_after_the_step_it_is_told(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """--stop-after ends the run after that step and says so."""
    run = tmp_path / "run"

    assert plan(run, "--stop-after", "capacity") == 0

    assert (run / "01-capacity.fb").is_file()
    assert untimed(capsys.readouterr().out, run)[-1] == "✓ 1 stage · stopped after capacity"


def test_plan_that_ends_with_fails_exits_with_one(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    """A run whose stages all finish but leave fails ends with a cross and exit code 1."""

    def with_fails(
        directory: RunDirectory,
        stage: str,
        config: object,
        chip: bytes,
        reporter: BlockReporter | None = None,
        verbosity: int = 0,
    ) -> StageResult:
        del config, chip, verbosity
        assert reporter is not None
        reporter.result([("1 chain", "plain")], 2)
        path = directory.artifact(stage)
        path.write_bytes(b"")
        return StageResult(stage=stage, path=path, size=0)

    monkeypatch.setattr(RunDirectory, "run_stage", with_fails)
    run = tmp_path / "run"

    assert plan(run) == 1

    lines = untimed(capsys.readouterr().out, run)
    assert lines[3:5] == ["   1 chain", "   ✓ 01-capacity.fb  SIZE · fails 2"]
    assert lines[-1] == "✗ 1 stage · stopped after capacity · fails 2"


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


def test_the_configured_stop_holds_unless_the_command_says_otherwise(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """[run] stop_after sets where a run ends, and --stop-after overrides it."""
    asked: list[str | None] = []

    def recorded(stop_after: str | None) -> list[str]:
        asked.append(stop_after)
        return steps_until(stop_after)

    monkeypatch.setattr(cli, "steps_until", recorded)
    configured = stopping_after(tmp_path, "capacity")
    plain = fixture_config(tmp_path / "plain")

    assert main(["plan", "-c", str(configured), "-o", str(tmp_path / "configured"), "--no-progress"]) == 0
    assert main(["plan", "-c", str(plain), "-o", str(tmp_path / "plain-run"), "--no-progress"]) == 0
    over = tmp_path / "overridden"
    assert main(["plan", "-c", str(plain), "-o", str(over), "--no-progress", "--stop-after", "capacity"]) == 0

    assert asked == ["capacity", "", "capacity"]
    assert sorted(path.name for path in (tmp_path / "configured").glob("*.fb")) == ["01-capacity.fb"]


def test_a_step_resumes_a_run_from_its_own_copies(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """--stage runs one step on the configuration and chip the run carries."""
    run = tmp_path / "run"
    assert plan(run, "--stop-after", "capacity") == 0
    capsys.readouterr()

    (run / "01-capacity.fb").unlink()

    assert main(["plan", "-o", str(run), "--stage", "capacity", "--no-progress"]) == 0

    assert (run / "01-capacity.fb").is_file()
    lines = untimed(capsys.readouterr().out, run)
    assert lines[0] == "mqt-scpd plan run → RUN"
    assert lines[-1] == "✓ 1 stage · stopped after capacity"


def test_a_run_directory_named_from_inside_it_still_names_the_chip(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    """A run directory given as `.` names the chip after the directory, in the header and in the picture."""
    run = tmp_path / "run"
    assert plan(run, "--stop-after", "capacity") == 0
    capsys.readouterr()
    monkeypatch.chdir(run)

    assert main(["plan", "-o", ".", "--stage", "capacity", "--no-progress"]) == 0
    assert capsys.readouterr().out.splitlines()[0] == "mqt-scpd plan run → ."

    output = tmp_path / "capacity.svg"
    assert main(["plot", "--run-dir", ".", "--stage", "capacity", "-o", str(output)]) == 0
    assert '<text class="title" ' in output.read_text(encoding="utf-8")
    assert '">run</text>' in output.read_text(encoding="utf-8")


def test_an_error_of_the_core_is_a_line_and_not_a_traceback(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """A stage that the core refuses ends the run with the reason in its block."""
    # Q2.port1 moves onto Q1.port1, so the targets of the two ports lie on one cell, which only the
    # capacity stage finds.
    chip = json.loads((Path(CONFIG).parent / "routing_config.json").read_text(encoding="utf-8"))
    chip["ports"]["Q2.port1"] = dict(chip["ports"]["Q1.port1"])
    moved = tmp_path / "chip.json"
    moved.write_text(json.dumps(chip), encoding="utf-8")
    run = tmp_path / "run"

    assert plan(run, "--chip", str(moved)) == 1

    out = capsys.readouterr().out
    assert "   ✗ the targets of the ports Q1.port1 and Q2.port1 lie on one cell of the detail grid" in out
    assert "Traceback" not in out
    assert not (run / "01-capacity.fb").exists()


def fixture_config(directory: Path) -> Path:
    """Write the fixture configuration into a directory, with the chip input at its place in the fixture.

    Returns:
        The path of the configuration.
    """
    directory.mkdir(parents=True, exist_ok=True)
    config = directory / "config.toml"
    text = Path(CONFIG).read_text(encoding="utf-8")
    config.write_text(
        text.replace(
            'input = "routing_config.json"', f'input = "{(Path(CONFIG).parent / "routing_config.json").as_posix()}"'
        ),
        encoding="utf-8",
    )
    return config


def test_doctor_and_plan_warn_about_a_key_that_no_stage_reads(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """A removed key fails no check, and the doctor and the run name it so that it can go."""
    config = fixture_config(tmp_path)
    text = config.read_text(encoding="utf-8")
    config.write_text(text.replace("capacity_cells_x = 12", "capacity_cells_x = 12\nlauncher_offset_x = 0"))
    warning = "[grid] launcher_offset_x has no effect, and a later version will refuse it; remove it"

    assert main(["doctor", "-c", str(config)]) == 0
    assert f"WARNING: {warning}" in capsys.readouterr().out

    run = tmp_path / "run"
    assert main(["plan", "-c", str(config), "-o", str(run), "--stop-after", "capacity", "--no-progress"]) == 0
    assert warning in capsys.readouterr().out


def test_doctor_and_plan_refuse_a_detail_grid_above_the_limit(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """A detail grid of more than 25,000,000 cells stops the doctor and a run before any stage."""
    config = fixture_config(tmp_path)
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


def test_a_refused_run_keeps_the_run_it_would_replace(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """A configuration that a step would refuse leaves the artifacts of the run directory alone."""
    run = tmp_path / "run"
    assert plan(run) == 0
    before = {path.name: path.read_bytes() for path in run.glob("*.fb")}

    config = fixture_config(tmp_path)
    config.write_text(
        config.read_text(encoding="utf-8") + '\n[stages.capacity]\nplanner = "other-planner"\n', encoding="utf-8"
    )
    assert main(["plan", "-c", str(config), "-o", str(run), "--no-progress"]) == 1

    assert "[stages.capacity] planner 'other-planner' is not a known implementation" in capsys.readouterr().err
    assert {path.name: path.read_bytes() for path in run.glob("*.fb")} == before
    assert sorted(before) == ["01-capacity.fb"]


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
        main(["plan", "-c", CONFIG, "-o", str(tmp_path), "--stage", "capacity"])
    assert info.value.code == 2
    with pytest.raises(SystemExit) as info:
        main(["plan", "-o", str(tmp_path), "--stage", "capacity", "--chip", "chip.json"])
    assert info.value.code == 2


def test_list_algorithms_names_one_line_per_stage(capsys: pytest.CaptureFixture[str]) -> None:
    """Every stage is listed with the implementations this build ships."""
    assert main(["list-algorithms"]) == 0
    assert capsys.readouterr().out.splitlines() == ["capacity-planner   watershed"]


@pytest.mark.usefixtures("vector_type")
def test_plot_draws_a_planning_stage_of_a_run(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """A planning stage is drawn from the run directory, over the chip the run carries.

    The capacity picture shows the borders of the partitions over the cells the grid blocked.
    """
    run = tmp_path / "run"
    assert plan(run) == 0
    capsys.readouterr()
    output = tmp_path / "capacity.svg"

    assert main(["plot", "--run-dir", str(run), "--stage", "capacity", "-o", str(output)]) == 0

    svg = output.read_text(encoding="utf-8")
    assert '<g class="l-border">' in svg
    assert '<g class="l-raster">' in svg
    assert '<g class="l-launcher">' in svg
    assert capsys.readouterr().out.startswith(f"✓ {output}  ")


def test_a_planning_stage_needs_a_run(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """A planning stage without a run directory, or one whose step has not run, is refused."""
    with pytest.raises(SystemExit) as info:
        main(["plot", "-c", CONFIG, "--stage", "capacity", "-o", str(tmp_path / "x.svg")])
    assert info.value.code == 2

    run = tmp_path / "run"
    assert plan(run) == 0
    (run / "01-capacity.fb").unlink()
    capsys.readouterr()
    assert main(["plot", "--run-dir", str(run), "--stage", "capacity", "-o", str(tmp_path / "x.svg")]) == 1
    assert "01-capacity.fb is missing" in capsys.readouterr().err


def test_a_picture_refuses_an_artifact_the_run_inputs_no_longer_make(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    """A step that failed keeps its old artifact without a record, and a changed input outdates every artifact.

    Either way the picture of the step is refused, since it would show what the run inputs no longer make.
    """
    run = tmp_path / "run"
    assert plan(run) == 0
    capsys.readouterr()
    output = tmp_path / "capacity.svg"

    def broken(*args: object) -> bytes:
        del args
        msg = "the capacity plan broke"
        raise RuntimeError(msg)

    with monkeypatch.context() as patched:
        patched.setattr(pyscpd, "plan_capacity", broken)
        assert main(["plan", "-o", str(run), "--stage", "capacity", "--no-progress"]) == 1
    capsys.readouterr()
    assert (run / "01-capacity.fb").is_file()
    assert main(["plot", "--run-dir", str(run), "--stage", "capacity", "-o", str(output)]) == 1
    assert "01-capacity.fb carries no record of the inputs it was made from" in capsys.readouterr().err
    assert not output.exists()
    # inspect still prints the artifact, after a warning.
    assert main(["inspect", str(run / "01-capacity.fb")]) == 0
    captured = capsys.readouterr()
    assert json.loads(captured.out)["output_type"] == "CapacityPlan"
    assert "⚠ 01-capacity.fb carries no record of the inputs it was made from" in captured.err

    assert main(["plan", "-o", str(run), "--stage", "capacity", "--no-progress"]) == 0
    capsys.readouterr()
    assert main(["plot", "--run-dir", str(run), "--stage", "capacity", "-o", str(output)]) == 0
    capsys.readouterr()
    assert main(["inspect", str(run / "01-capacity.fb")]) == 0
    assert not capsys.readouterr().err

    config = run / "config.toml"
    config.write_text(
        config.read_text(encoding="utf-8") + "\n[stages.capacity]\ncrossing_pitch = 120.0\n", encoding="utf-8"
    )
    assert main(["plot", "--run-dir", str(run), "--stage", "capacity", "-o", str(output)]) == 1
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


def test_any_error_of_a_stage_ends_the_run_in_its_block(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    """An error of a kind no stage is expected to raise still ends the run with a cross in its block."""

    def broken(*args: object, **kwargs: object) -> None:
        del args, kwargs
        msg = "a stage answered with the wrong shape"
        raise TypeError(msg)

    monkeypatch.setattr(RunDirectory, "run_stage", broken)
    run = tmp_path / "run"

    assert plan(run) == 1

    lines = untimed(capsys.readouterr().out, run)
    assert lines[2:4] == ["🧩 capacity", "   ✗ a stage answered with the wrong shape"]
    assert lines[-1].startswith("✗ stopped at capacity after ")


def test_a_broken_pipe_of_a_stage_itself_ends_the_run_in_its_block(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    """A broken pipe that does not come from the output fails the stage."""

    def broken(*args: object, **kwargs: object) -> None:
        del args, kwargs
        msg = "a pipe of the stage broke"
        raise BrokenPipeError(msg)

    monkeypatch.setattr(RunDirectory, "run_stage", broken)
    run = tmp_path / "run"

    assert plan(run) == 1

    lines = untimed(capsys.readouterr().out, run)
    assert lines[2:4] == ["🧩 capacity", "   ✗ a pipe of the stage broke"]
    assert lines[-1].startswith("✗ stopped at capacity after ")
