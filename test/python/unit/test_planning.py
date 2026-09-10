# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""What a planning artifact turns into, and how the two renderers draw it."""

from __future__ import annotations

from pathlib import Path

import pytest

from mqt.scpd.artifacts import read_artifact, write_artifact
from mqt.scpd.chip import decode_chip
from mqt.scpd.export.klayout import PLANNING_LAYERS, write_layout
from mqt.scpd.flatbuffers.artifacts.Artifact import ArtifactT
from mqt.scpd.flatbuffers.artifacts.CapacityPlan import CapacityPlanT
from mqt.scpd.flatbuffers.artifacts.GlobalRouting import GlobalRoutingT
from mqt.scpd.flatbuffers.artifacts.GridExtent import GridExtentT
from mqt.scpd.flatbuffers.artifacts.StageOutput import StageOutput
from mqt.scpd.flatbuffers.geometry.Point import PointT
from mqt.scpd.planning import PLANNING_STAGES, PlanningError, blockade, planning_geometry
from mqt.scpd.plot import layout_svg
from mqt.scpd.run import IMPLEMENTED, RunDirectory

BENCHMARKS = Path(__file__).resolve().parents[3] / "benchmarks"

klayout = pytest.importorskip("klayout.db", reason="klayout is an optional extra")


@pytest.fixture(scope="module")
def run(tmp_path_factory: pytest.TempPathFactory) -> RunDirectory:
    """A finished planning run of the 9-qubit chip.

    Returns:
        The run directory.
    """
    directory = RunDirectory(tmp_path_factory.mktemp("run"))
    config = directory.prepare(BENCHMARKS / "9q" / "config.toml")
    for stage in IMPLEMENTED:
        directory.run_stage(stage, config)
    return directory


@pytest.fixture(scope="module")
def chip(run: RunDirectory):  # noqa: ANN201
    """The chip the run was made from.

    Returns:
        The classified chip.
    """
    from mqt.scpd.chip import classify_chip  # noqa: PLC0415

    return decode_chip(classify_chip(run.chip.read_text(encoding="utf-8"), run.load()))


def test_the_capacity_plan_carries_the_partitioning(run: RunDirectory, chip) -> None:  # noqa: ANN001
    """Everything the capacity stage decided is in the artifact, in layout units."""
    geometry = planning_geometry(run.artifact("capacity").read_bytes(), chip, "capacity")

    assert geometry.partitions
    assert geometry.borders
    assert geometry.bottlenecks
    # What the ports' own approaches block is a layer of its own; it is not chip artwork.
    assert geometry.keepout
    for ring in geometry.keepout:
        assert len(ring) >= 4
        assert ring[0] == ring[-1]
    assert geometry.launchers
    assert geometry.chains
    # A bottleneck is a line across a corridor and carries a wire budget.
    for first, second, capacity in geometry.bottlenecks:
        assert first != second
        assert capacity >= 0
    # Nothing of the other stages leaks into this one.
    assert not geometry.lattice
    assert not geometry.assignments


def test_the_global_picture_carries_the_gates_when_the_capacity_plan_is_given(
    run: RunDirectory,
    chip,  # noqa: ANN001
) -> None:
    """Where a wire may surface is decided by the free space behind the port, so the gates
    that measure it are drawn with the circuit that paid for them.
    """
    alone = planning_geometry(run.artifact("global").read_bytes(), chip, "global")
    assert not alone.bottlenecks
    assert not alone.keepout

    withgates = planning_geometry(
        run.artifact("global").read_bytes(),
        chip,
        "global",
        run.artifact("capacity").read_bytes(),
    )
    assert withgates.bottlenecks
    assert withgates.keepout
    assert (
        withgates.bottlenecks == planning_geometry(run.artifact("capacity").read_bytes(), chip, "capacity").bottlenecks
    )
    # Only the gates and the port keepout, not the rest of the capacity plan.
    assert not withgates.partitions
    assert not withgates.chains


def test_a_capacity_plan_is_the_only_thing_the_global_picture_takes_gates_from(
    run: RunDirectory,
    chip,  # noqa: ANN001
) -> None:
    """An artifact of another stage passed as the capacity plan is a problem, not a picture."""
    with pytest.raises(PlanningError, match="not a capacity plan"):
        planning_geometry(
            run.artifact("global").read_bytes(),
            chip,
            "global",
            run.artifact("global").read_bytes(),
        )


def test_the_global_routing_carries_its_lattice(run: RunDirectory, chip) -> None:  # noqa: ANN001
    """The lattice and the edges the inner circuit selected are both drawable."""
    geometry = planning_geometry(run.artifact("global").read_bytes(), chip, "global")

    assert geometry.lattice
    # What was selected is a subset of what was offered.
    assert set(geometry.inner) <= set(geometry.lattice)
    assert not geometry.partitions


def test_the_assignment_carries_a_chord_per_ring_node(run: RunDirectory, chip) -> None:  # noqa: ANN001
    """Each ring node is drawn joined to the launcher it was given."""
    geometry = planning_geometry(run.artifact("assign").read_bytes(), chip, "assign")

    assert geometry.ring
    assert len(geometry.assignments) == len(geometry.ring)
    assert not geometry.bottlenecks


def test_the_corridor_routing_carries_a_way_per_connection(run: RunDirectory, chip) -> None:  # noqa: ANN001
    """Each wire is drawn from its feed through its crossings to its target."""
    geometry = planning_geometry(
        run.artifact("corridor").read_bytes(), chip, "corridor", run.artifact("capacity").read_bytes()
    )

    assert geometry.corridors
    # A way names at least where it starts and where it ends.
    assert all(len(route) >= 2 for route in geometry.corridors)
    # Every crossing of every way is one of the slots the borders offer.
    slots = set(geometry.slots)
    assert all(point in slots for route in geometry.corridors for point in route[1:-1])
    # The partitions are drawn under it, because a way through them is what it is.
    assert geometry.partitions


def test_the_corridor_picture_draws_nothing_of_the_partitions_without_a_plan(
    run: RunDirectory,
    chip,  # noqa: ANN001
) -> None:
    """The capacity artifact is what carries the partitions; without it the ways stand alone."""
    geometry = planning_geometry(run.artifact("corridor").read_bytes(), chip, "corridor")

    assert geometry.corridors
    assert not geometry.partitions


def test_the_detail_routing_carries_a_drawn_wire_per_connection(run: RunDirectory, chip) -> None:  # noqa: ANN001
    """Each wire is the polyline of its bends, from its feed to its target."""
    geometry = planning_geometry(
        run.artifact("detail").read_bytes(), chip, "detail", run.artifact("capacity").read_bytes()
    )

    assert geometry.wires
    # A wire names at least where it starts and where it ends, and no bend repeats its neighbour.
    for wire in geometry.wires:
        assert len(wire) >= 2
        assert all(before != after for before, after in zip(wire, wire[1:], strict=False))
    # The way the corridor planned is what the wire had to follow, so the two ends agree.
    corridor = planning_geometry(
        run.artifact("corridor").read_bytes(), chip, "corridor", run.artifact("capacity").read_bytes()
    )
    assert len(geometry.wires) == len(corridor.corridors)

    def same(here: tuple[float, float], there: tuple[float, float]) -> bool:
        # The two artifacts reach the same place by different arithmetic — one stores the layout
        # point, the other multiplies the cell out again — so they agree to the last bit but one.
        return abs(here[0] - there[0]) < 1e-6 and abs(here[1] - there[1]) < 1e-6

    for wire, route in zip(geometry.wires, corridor.corridors, strict=True):
        assert same(wire[0], route[0])
        assert same(wire[-1], route[-1])
    # The partitions are drawn under it, because a way through them is what it is.
    assert geometry.partitions


def test_the_detail_picture_draws_the_clearance_the_router_actually_keeps(
    run: RunDirectory,
    chip,  # noqa: ANN001
) -> None:
    """The band is the design rule as the router converted it to whole cells, not the rule itself.

    A router that works in cells cannot keep 185 layout units; it keeps
    ``ceil(185 / cell) - 1`` cells, which is a little less. Drawing the rule would draw a clearance
    nothing keeps.
    """
    without = planning_geometry(run.artifact("detail").read_bytes(), chip, "detail")
    assert without.clearance == 0.0
    assert "l-clearance" not in layout_svg(chip, planning=without)

    with_rule = planning_geometry(run.artifact("detail").read_bytes(), chip, "detail", clearance=185.0)
    grid = read_artifact(run.artifact("detail").read_bytes()).output.grid
    cell = min(float(grid.cellWidth), float(grid.cellHeight))
    assert with_rule.clearance == blockade(185.0, cell)
    assert 0 < with_rule.clearance < 185.0
    assert with_rule.clearance % cell == pytest.approx(0.0, abs=1e-9)

    svg = layout_svg(chip, planning=with_rule)
    assert "l-clearance" in svg
    assert f"stroke-width:{with_rule.clearance:.3f}".rstrip("0").rstrip(".") in svg


def test_an_artifact_of_the_wrong_stage_is_refused(run: RunDirectory, chip) -> None:  # noqa: ANN001
    """Reading a capacity plan as an assignment says so rather than drawing nothing."""
    with pytest.raises(PlanningError, match="not an assignment"):
        planning_geometry(run.artifact("capacity").read_bytes(), chip, "assign")


def test_a_stage_that_is_not_a_planning_stage_is_refused(chip) -> None:  # noqa: ANN001
    """The message lists the stages that can be drawn."""
    empty = write_artifact(
        ArtifactT(
            producer="test",
            outputType=StageOutput.GlobalRouting,
            output=GlobalRoutingT(lattices=[], connections=[], outerRing=[], resonators=[]),
        )
    )
    with pytest.raises(PlanningError, match=", ".join(PLANNING_STAGES)):
        planning_geometry(empty, chip, "final")


def test_an_empty_plan_draws_nothing(chip) -> None:  # noqa: ANN001
    """A stage that produced nothing is a valid artifact with an empty picture."""
    empty = write_artifact(
        ArtifactT(
            producer="test",
            outputType=StageOutput.CapacityPlan,
            output=CapacityPlanT(
                capacityGrid=GridExtentT(origin=PointT()),
                detailGrid=GridExtentT(origin=PointT()),
                partitions=[],
                borders=[],
                bottlenecks=[],
                launchers=[],
                nodes=[],
                chains=[],
            ),
        )
    )
    assert planning_geometry(empty, chip, "capacity").is_empty()


@pytest.mark.parametrize("stage", list(PLANNING_STAGES))
def test_every_planning_stage_renders_to_svg(run: RunDirectory, chip, stage: str) -> None:  # noqa: ANN001
    """Each stage draws over the artwork, on layers of its own, inside the snapshot budget."""
    geometry = planning_geometry(run.artifact(stage).read_bytes(), chip, stage)
    svg = layout_svg(chip, planning=geometry)

    assert svg.startswith("<svg")
    assert svg.endswith("</svg>\n")
    # A gate carries its wire budget as a number beside it.
    for _, _, capacity in geometry.bottlenecks:
        assert f">{capacity}</text>" in svg
    # Every layer that carries something is drawn as its own group.
    for name, shapes in (
        ("keepout", geometry.keepout),
        ("partition", geometry.partitions),
        ("bottleneck", geometry.bottlenecks),
        ("inner", geometry.inner),
        ("assignment", geometry.assignments),
        ("corridor", geometry.corridors),
        ("slot", geometry.slots),
    ):
        assert (f'class="l-{name}"' in svg) == bool(shapes)
    assert len(svg.encode("utf-8")) < 10_000_000


@pytest.mark.parametrize("stage", list(PLANNING_STAGES))
def test_every_planning_stage_renders_to_gds(run: RunDirectory, chip, tmp_path: Path, stage: str) -> None:  # noqa: ANN001
    """Each stage is written beside the artwork on the planning layers, which stay switchable."""
    geometry = planning_geometry(run.artifact(stage).read_bytes(), chip, stage)
    path = tmp_path / f"{stage}.gds"

    summary = write_layout(chip, path, planning=geometry)

    assert summary.planning > 0
    assert summary.polygons > 0
    layout = klayout.Layout()
    layout.read(str(path))
    # The artwork keeps the layers it has, and every planning shape is above them.
    used = {layout.get_info(index).layer for index in layout.layer_indexes()}
    assert 1 in used
    assert used & {number for number, _, _ in PLANNING_LAYERS.values()}


def test_the_chip_still_renders_without_a_planning_stage(chip, tmp_path: Path) -> None:  # noqa: ANN001
    """`render` with no stage writes the unrouted chip exactly as it did before."""
    summary = write_layout(chip, tmp_path / "chip.gds")
    assert summary.planning == 0
