# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""What the planning stages produced, read back out of their artifacts.

This is the one place that knows how a capacity plan, a global routing and an assignment turn into
geometry. Both renderers use it: ``plot`` draws the shapes as SVG and ``render`` writes them to GDS
or OASIS on layers of their own. Neither reaches into the pipeline; both read the artifacts a run
directory holds, which is what lets either of them run against a half-finished run.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

from .artifacts import read_artifact
from .flatbuffers.artifacts.Assignment import AssignmentT
from .flatbuffers.artifacts.CapacityElement import CapacityElement
from .flatbuffers.artifacts.CapacityPlan import CapacityPlanT
from .flatbuffers.artifacts.CorridorRouting import CorridorRoutingT
from .flatbuffers.artifacts.GlobalRouting import GlobalRoutingT

if TYPE_CHECKING:
    from .flatbuffers.design.Chip import ChipT

__all__ = ["PLANNING_STAGES", "PlanningError", "PlanningGeometry", "planning_geometry"]

#: The planning stages that can be drawn, and the artifact each one is read from.
PLANNING_STAGES: dict[str, str] = {
    "capacity": "01-capacity.fb",
    "global": "02-global.fb",
    "assign": "03-assign.fb",
    "corridor": "04-corridor.fb",
}

#: A point in layout units.
Point = tuple[float, float]


class PlanningError(ValueError):
    """An artifact that does not hold what the stage it is named for produces."""


@dataclass
class PlanningGeometry:
    """The shapes one planning stage produced, in layout units.

    Each list is one layer of the picture, so a renderer can give it its own color or its own GDS
    layer without knowing what a partition or a capacity chain is.
    """

    #: The outline of every partition of the free space, as closed rings.
    partitions: list[list[Point]] = field(default_factory=list)
    #: What the ports' own approaches keep clear, as closed rings.
    keepout: list[list[Point]] = field(default_factory=list)
    #: The border between two partitions, as the run of cell edges it consists of.
    borders: list[list[Point]] = field(default_factory=list)
    #: The line across the narrowest place of a corridor, and how many wires fit through it.
    bottlenecks: list[tuple[Point, Point, int]] = field(default_factory=list)
    #: Where each launcher's wires enter the chip.
    launchers: list[Point] = field(default_factory=list)
    #: A capacity chain, as the line from each gate to the target beyond it.
    chains: list[tuple[Point, Point]] = field(default_factory=list)
    #: The Hanan lattice the inner circuit was solved on.
    lattice: list[tuple[Point, Point]] = field(default_factory=list)
    #: The lattice edges the inner circuit selected.
    inner: list[tuple[Point, Point]] = field(default_factory=list)
    #: Each assignment, as the chord from the resonator to the launcher it was given.
    assignments: list[tuple[Point, Point]] = field(default_factory=list)
    #: The outer ring, in the order the assignment consumed it.
    ring: list[Point] = field(default_factory=list)
    #: Each wire's coarse route, from its feed through its crossings to its target.
    corridors: list[list[Point]] = field(default_factory=list)
    #: Every crossing slot a border offers, taken or not.
    slots: list[Point] = field(default_factory=list)

    def is_empty(self) -> bool:
        """Whether the stage produced nothing to draw.

        Returns:
            True when every layer is empty.
        """
        return not any((
            self.partitions,
            self.keepout,
            self.borders,
            self.bottlenecks,
            self.launchers,
            self.chains,
            self.lattice,
            self.inner,
            self.assignments,
            self.ring,
            self.corridors,
            self.slots,
        ))


def _entries(items: list[Any] | None) -> list[Any]:
    """The entries of a schema list that are actually there.

    The generated model types every list as optional and every entry with it, so a reader that
    walks one has to say what an absent entry means. Here it means nothing to draw.

    Returns:
        The entries, without the absent ones.
    """
    return [item for item in (items or []) if item is not None]


def _point(value: Any) -> Point:
    """A schema point as a pair.

    Returns:
        The coordinates.
    """
    return (float(value.x), float(value.y))


def _center(chip: ChipT, index: int) -> Point | None:
    """The layout position of a port of the chip.

    Returns:
        The position, or None when the index names no port.
    """
    ports: list[Any] = chip.ports or []
    if index >= len(ports):
        return None
    port = ports[index]
    if port is None or port.center is None:
        return None
    return _point(port.center)


def _keepout(plan: CapacityPlanT, geometry: PlanningGeometry) -> None:
    """Fill the layer of what the ports' own approaches block."""
    for ring in _entries(plan.portKeepout):
        points = [_point(vertex) for vertex in _entries(ring.vertices)]
        if len(points) >= 3:
            geometry.keepout.append(points)


def _capacity(plan: CapacityPlanT, geometry: PlanningGeometry) -> None:
    """Fill the layers a capacity plan carries."""
    _keepout(plan, geometry)
    for partition in _entries(plan.partitions):
        for ring in _entries(partition.outlines):
            points = [_point(vertex) for vertex in _entries(ring.vertices)]
            if len(points) >= 3:
                geometry.partitions.append(points)
    for border in _entries(plan.borders):
        samples = [_point(sample) for sample in _entries(border.samples)]
        if samples:
            geometry.borders.append(samples)
    for bottleneck in _entries(plan.bottlenecks):
        geometry.bottlenecks.append((
            _point(bottleneck.from_),
            _point(bottleneck.to),
            int(bottleneck.capacity),
        ))
    for slot in _entries(plan.launchers):
        geometry.launchers.append(_point(slot.position))


def _chains(plan: CapacityPlanT, chip: ChipT, geometry: PlanningGeometry) -> None:
    """Fill the chain layer: one line per branch of every capacity chain.

    A chain is a tree of gates from a launcher to the targets it feeds. Drawing it as the lines
    between the ports it names is what makes a chain visible without inventing a route for it.
    """
    nodes = _entries(plan.nodes)
    bottlenecks = _entries(plan.bottlenecks)

    def position(index: int) -> Point | None:
        node = nodes[index]
        if node.kind == CapacityElement.Target:
            return _center(chip, int(node.id))
        if node.kind == CapacityElement.Bottleneck and int(node.id) < len(bottlenecks):
            gate = bottlenecks[int(node.id)]
            first, second = _point(gate.from_), _point(gate.to)
            return ((first[0] + second[0]) / 2, (first[1] + second[1]) / 2)
        return None

    def walk(index: int) -> None:
        here = position(index)
        for child in nodes[index].next or []:
            if int(child) < len(nodes):
                there = position(int(child))
                if here is not None and there is not None:
                    geometry.chains.append((here, there))
                walk(int(child))

    for root in plan.chains or []:
        if int(root) < len(nodes):
            walk(int(root))


def _global(routing: GlobalRoutingT, geometry: PlanningGeometry) -> None:
    """Fill the layers a global routing carries."""
    for lattice in _entries(routing.lattices):
        points = [_point(point) for point in _entries(lattice.points)]
        edges = [int(edge) for edge in (lattice.edges or [])]
        selected = {int(index) for index in (lattice.selected or [])}
        for index in range(0, len(edges) - 1, 2):
            first, second = edges[index], edges[index + 1]
            if first >= len(points) or second >= len(points):
                continue
            segment = (points[first], points[second])
            geometry.lattice.append(segment)
            if index // 2 in selected:
                geometry.inner.append(segment)


def _assignment(assignment: AssignmentT, chip: ChipT, geometry: PlanningGeometry) -> None:
    """Fill the layers an assignment carries."""
    ring = _entries(assignment.ring)
    launchers = _entries(assignment.launchers)
    feeds = _entries(assignment.feeds)
    for entry in ring:
        position = _center(chip, int(entry.index))
        if position is not None:
            geometry.ring.append(position)
    for index, entry in enumerate(ring):
        source = _center(chip, int(entry.index))
        if source is None:
            continue
        # A resonator that ends a feedline is fed between two launchers rather
        # than at one, so the chord is drawn to the point the artifact carries
        # and not to the launcher port it is nominally on.
        if index < len(feeds):
            target = _point(feeds[index])
        elif index < len(launchers):
            centre = _center(chip, int(launchers[index].index))
            if centre is None:
                continue
            target = centre
        else:
            continue
        geometry.assignments.append((source, target))
        if index < len(feeds) and index < len(launchers):
            centre = _center(chip, int(launchers[index].index))
            if centre is not None and target != centre:
                geometry.launchers.append(target)


def _corridor(routing: CorridorRoutingT, geometry: PlanningGeometry) -> None:
    """Fill the layers a corridor routing carries."""
    for border in _entries(routing.slots):
        geometry.slots.extend(_point(position) for position in _entries(border.positions))
    for corridor in _entries(routing.corridors):
        if not corridor.partitions or corridor.source is None or corridor.target is None:
            continue
        route = [_point(corridor.source)]
        route.extend(_point(crossing) for crossing in _entries(corridor.crossings))
        route.append(_point(corridor.target))
        geometry.corridors.append(route)


def planning_geometry(data: bytes, chip: ChipT, stage: str, capacity: bytes | None = None) -> PlanningGeometry:
    """Read one planning artifact into the shapes it describes.

    Args:
        data: The artifact bytes.
        chip: The chip the run was made from, for the positions of the ports an artifact names.
        stage: Which stage the artifact is expected to be from.
        capacity: The capacity artifact of the same run, for the stages that are drawn over the
            free space they had to fit into. The global stage reads its gates, and the corridor
            stage the partitions its wires run through.

    Returns:
        The geometry, in layout units.

    Raises:
        PlanningError: If the artifact is not the stage's own output.
    """
    artifact = read_artifact(data)
    output = artifact.output
    geometry = PlanningGeometry()

    if stage == "capacity":
        if not isinstance(output, CapacityPlanT):
            msg = "the artifact is not a capacity plan"
            raise PlanningError(msg)
        _capacity(output, geometry)
        _chains(output, chip, geometry)
    elif stage == "global":
        if not isinstance(output, GlobalRoutingT):
            msg = "the artifact is not a global routing"
            raise PlanningError(msg)
        _global(output, geometry)
        # Where a wire may surface is decided by the free space behind the port, so the gates
        # that measure it belong in the picture of the circuit that had to pay for them.
        if capacity is not None:
            plan = read_artifact(capacity).output
            if not isinstance(plan, CapacityPlanT):
                msg = "the capacity artifact is not a capacity plan"
                raise PlanningError(msg)
            for bottleneck in _entries(plan.bottlenecks):
                geometry.bottlenecks.append((
                    _point(bottleneck.from_),
                    _point(bottleneck.to),
                    int(bottleneck.capacity),
                ))
            _keepout(plan, geometry)
    elif stage == "assign":
        if not isinstance(output, AssignmentT):
            msg = "the artifact is not an assignment"
            raise PlanningError(msg)
        _assignment(output, chip, geometry)
    elif stage == "corridor":
        if not isinstance(output, CorridorRoutingT):
            msg = "the artifact is not a corridor routing"
            raise PlanningError(msg)
        _corridor(output, geometry)
        # A corridor is a way through the partitions, so the partitions are what makes the
        # picture readable at all.
        if capacity is not None:
            plan = read_artifact(capacity).output
            if not isinstance(plan, CapacityPlanT):
                msg = "the capacity artifact is not a capacity plan"
                raise PlanningError(msg)
            _capacity(plan, geometry)
    else:
        msg = f"'{stage}' is no planning stage; they are {', '.join(PLANNING_STAGES)}"
        raise PlanningError(msg)
    return geometry
