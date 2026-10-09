# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""Tests of the command line, through its entry point."""

from __future__ import annotations

import json
import re
from pathlib import Path
from typing import TYPE_CHECKING

import pytest

from mqt.scpd import pyscpd
from mqt.scpd.artifacts import write_artifact
from mqt.scpd.cli import main
from mqt.scpd.export import HAS_KLAYOUT
from mqt.scpd.flatbuffers.artifacts.Artifact import ArtifactT
from mqt.scpd.flatbuffers.artifacts.GlobalRouting import GlobalRoutingT
from mqt.scpd.flatbuffers.artifacts.StageOutput import StageOutput
from mqt.scpd.run import RunDirectory

if TYPE_CHECKING:
    from collections.abc import Generator

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
    r"   \d+ ports? on \d+ launchers? · objective \d+\.\d\d · optimal",
    r"   ✓ 03-assign\.fb  SIZE · fails 0",
    r"🧭 corridor",
    # Any number of rounds, and the last one routes every connection.
    (
        r"(?:   round \d+ (?:forward|backward) +routed \d+/\d+ · fails \d+\n)*"
        r"   round \d+ (?:forward|backward) +routed (\d+)/\1 · fails 0"
    ),
    r"   routed (\d+)/\2 · \d+ rounds?",
    r"   ✓ 04-corridor\.fb  SIZE · fails 0",
    r"",
    r"✓ 4 stages · stopped after corridor · fails 0",
)

# Every test runs with the solver linked into the core.
pytestmark = pytest.mark.usefixtures("only_highs")


@pytest.fixture
def only_highs(monkeypatch: pytest.MonkeyPatch) -> Generator[None]:
    """Keep a licensed gurobipy on the machine from becoming the solver of a test run."""
    monkeypatch.setattr("mqt.scpd.cli.register_external_solver", lambda: False)
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


def test_problems_exit_with_one(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """A missing configuration and bytes that are no artifact end with exit code 1 and a message."""
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
        "02-global.fb",
        "03-assign.fb",
        "04-corridor.fb",
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
        r"(🧩|🔳|📌|🧭) (capacity|global|assign|corridor)",
        rf"   \S.* {{2,}}{time}",
        rf"     \S.* {{2,}}{time}",
        r"     .+",
        r"   ✓ \d\d-[a-z]+\.fb  \d+(\.\d)? (B|kB|MB)( · fails \d+)?",
        rf"✓ 4 stages · {time} · stopped after corridor · fails 0",
    ]
    for line in out.splitlines():
        assert any(re.fullmatch(form, line) for form in forms), line
    # Each level adds to the one below it.
    assert re.search(r"^     model assignment · ", out, re.MULTILINE)
    assert re.search(r"^     chain 1 ", out, re.MULTILINE)
    assert "HiGHS" in out.split("📌 assign")[1]


def test_plan_stops_after_the_step_it_is_told(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """--stop-after ends the run after that step and says so."""
    run = tmp_path / "run"

    assert plan(run, "--stop-after", "global") == 0

    assert not (run / "03-assign.fb").exists()
    assert (run / "02-global.fb").is_file()
    assert untimed(capsys.readouterr().out, run)[-1] == "✓ 2 stages · stopped after global · fails 0"


def test_the_configured_stop_holds_unless_the_command_says_otherwise(tmp_path: Path) -> None:
    """[run] stop_after sets where a run ends, and --stop-after overrides it."""
    config = tmp_path / "config.toml"
    text = Path(CONFIG).read_text(encoding="utf-8")
    text = text.replace(
        'input = "routing_config.json"', f'input = "{(Path(CONFIG).parent / "routing_config.json").as_posix()}"'
    )
    config.write_text(text + '\n[run]\nstop_after = "capacity"\n', encoding="utf-8")

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
    config = tmp_path / "config.toml"
    text = Path(CONFIG).read_text(encoding="utf-8")
    text = text.replace(
        'input = "routing_config.json"', f'input = "{(Path(CONFIG).parent / "routing_config.json").as_posix()}"'
    )
    config.write_text(text.replace("launcher_target = 1", "launcher_target = 0"), encoding="utf-8")
    run = tmp_path / "run"

    assert main(["plan", "-c", str(config), "-o", str(run), "--no-progress"]) == 1

    out = capsys.readouterr().out
    assert "   ✗ [stages.assignment] launcher_target is missing" in out
    assert "Traceback" not in out
    assert not (run / "03-assign.fb").exists()


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
    assert '<g class="l-partition">' in svg
    assert '<g class="l-feedline">' in svg
    assert capsys.readouterr().out.startswith(f"✓ {output}  ")

    assert main(["plot", "--run-dir", str(run), "--stage", "assign", "-o", str(output)]) == 0
    assert '<g class="l-feedline">' in output.read_text(encoding="utf-8")


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


@pytest.mark.skipif(not HAS_KLAYOUT, reason="the export needs KLayout")
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
