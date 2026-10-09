# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""The shapes that the planning stages produced, read back out of their artifacts.

This module is the one place that knows how a capacity plan, a global routing, an assignment and a
corridor routing turn into geometry. Both renderers use it: ``plot`` draws the shapes as SVG, and
``render`` writes them to GDSII or OASIS on layers of their own. Neither reaches into the
pipeline; both read the artifacts of a run directory, so either can draw a run that stopped early.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Protocol

from .artifacts import read_artifact
from .flatbuffers.artifacts.Assignment import AssignmentT
from .flatbuffers.artifacts.CapacityElement import CapacityElement
from .flatbuffers.artifacts.CapacityPlan import CapacityPlanT
from .flatbuffers.artifacts.CorridorRouting import CorridorRoutingT
from .flatbuffers.artifacts.GlobalRouting import GlobalRoutingT
from .run import STAGE_FILES

if TYPE_CHECKING:
    from .flatbuffers.design.Chip import ChipT

__all__ = ["PLANNING_STAGES", "Feedline", "PlanningError", "PlanningGeometry", "planning_geometry"]

#: The planning stages that can be drawn, and the artifact each is read from.
PLANNING_STAGES: dict[str, str] = dict(STAGE_FILES)

#: A point in layout units.
Point = tuple[float, float]


class _Coordinates(Protocol):
    """A point of a schema: every point type of the schemas has an x and a y."""

    x: float
    y: float


class PlanningError(ValueError):
    """An artifact that does not hold what the stage it is drawn as produces."""


@dataclass
class Feedline:
    """One feedline chain of an assignment, as the line its feedline follows."""

    #: The line: the launcher the chain starts at, the feed of each resonator of the chain in chain
    #: order, and the launcher the chain ends at. Where the chain starts or ends at a termination,
    #: the line starts or ends at the feed of the resonator there.
    points: list[Point] = field(default_factory=list)
    #: The feeds of the resonators: the terminals the feedline passes.
    terminals: list[Point] = field(default_factory=list)
    #: The ends of the line where the chain stops at a termination instead of a launcher.
    terminations: list[Point] = field(default_factory=list)
    #: What the chain runs through, such as "feedline 1: Chip.port62 → Qb7, Qb12 → Chip.port57".
    label: str = ""


@dataclass
class PlanningGeometry:
    """The shapes one planning stage produced, in layout units.

    Each list is one layer of the picture, so a renderer can give it a colour or a GDS layer of its
    own without knowing what a partition or a capacity chain is.
    """

    #: The outline of every partition of the free space, as closed rings.
    partitions: list[list[Point]] = field(default_factory=list)
    #: What the approaches of the ports keep clear, as closed rings.
    keepout: list[list[Point]] = field(default_factory=list)
    #: The border between two partitions, as the run of cell edges it consists of.
    borders: list[list[Point]] = field(default_factory=list)
    #: The line across the narrowest place of a corridor, and how many wires fit through it.
    bottlenecks: list[tuple[Point, Point, int]] = field(default_factory=list)
    #: Where the wires of each launcher enter the chip.
    launchers: list[Point] = field(default_factory=list)
    #: A capacity chain, as the line from each link to the next.
    chains: list[tuple[Point, Point]] = field(default_factory=list)
    #: The Hanan lattice the inner circuit was solved on.
    lattice: list[tuple[Point, Point]] = field(default_factory=list)
    #: The lattice edges the inner circuit selected.
    inner: list[tuple[Point, Point]] = field(default_factory=list)
    #: Each assignment, as the chord from a ring port to the point it is fed from.
    assignments: list[tuple[Point, Point]] = field(default_factory=list)
    #: The outer ring, in the order the assignment consumed it.
    ring: list[Point] = field(default_factory=list)
    #: The coarse route of each wire, from its feed over its crossings to its target.
    corridors: list[list[Point]] = field(default_factory=list)
    #: Every crossing slot a border offers, taken or not.
    slots: list[Point] = field(default_factory=list)
    #: Each feedline chain of the assignment, from its first launcher through the feeds of its
    #: resonators to its last.
    feedlines: list[Feedline] = field(default_factory=list)

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
            self.feedlines,
        ))


def _entries(items: list[Any] | None) -> list[Any]:
    """The entries of a schema list that are there.

    The generated object model types every list as optional, and every entry with it. Here an
    absent list or entry means nothing to draw.

    Returns:
        The entries, without the absent ones.
    """
    return [item for item in (items or []) if item is not None]


def _point(value: _Coordinates) -> Point:
    """A schema point as a pair.

    Returns:
        The coordinates.
    """
    return (float(value.x), float(value.y))


def _name(chip: ChipT, index: int, *, component: bool) -> str:
    """The name of a port of the chip, for the label of a feedline.

    Args:
        chip: The chip.
        index: The port index.
        component: Whether to name the component the port belongs to, where the configuration
            declares one, instead of the port.

    Returns:
        The name, or the index when the index names no port.
    """
    ports: list[Any] = chip.ports or []
    if index >= len(ports) or ports[index] is None:
        return str(index)
    port = ports[index]
    if component and port.component:
        return str(port.component)
    return str(port.label or index)


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
    """Fill the layer of what the approaches of the ports keep clear."""
    for ring in _entries(plan.portKeepout):
        points = [_point(vertex) for vertex in _entries(ring.vertices)]
        if len(points) >= 3:
            geometry.keepout.append(points)


def _bottlenecks(plan: CapacityPlanT, geometry: PlanningGeometry) -> None:
    """Fill the layer of the gates and their wire budgets."""
    for bottleneck in _entries(plan.bottlenecks):
        geometry.bottlenecks.append((_point(bottleneck.from_), _point(bottleneck.to), int(bottleneck.capacity)))


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
    _bottlenecks(plan, geometry)
    for slot in _entries(plan.launchers):
        geometry.launchers.append(_point(slot.position))


def _chains(plan: CapacityPlanT, chip: ChipT, geometry: PlanningGeometry) -> None:
    """Fill the chain layer: one line per link of every capacity chain.

    A chain is a tree of gates from a launcher to the targets it feeds. Drawn as the lines between
    the ports and the gates it names, a chain is visible without a route made up for it.
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
        # A resonator is fed between two launchers and not at one, so the chord runs to the point
        # the artifact carries and not to the launcher port the resonator was given.
        if index < len(feeds):
            target = _point(feeds[index])
        elif index < len(launchers):
            center = _center(chip, int(launchers[index].index))
            if center is None:
                continue
            target = center
        else:
            continue
        geometry.assignments.append((source, target))
        if index < len(feeds) and index < len(launchers):
            center = _center(chip, int(launchers[index].index))
            if center is not None and target != center:
                geometry.launchers.append(target)
    _feedlines(assignment, chip, geometry)


def _feedlines(assignment: AssignmentT, chip: ChipT, geometry: PlanningGeometry) -> None:
    """Fill the feedline chains of an assignment."""
    ring = _entries(assignment.ring)
    feeds = _entries(assignment.feeds)
    for number, chain in enumerate(_entries(assignment.chains), start=1):
        nodes = [int(node) for node in (chain.nodes or []) if int(node) < min(len(ring), len(feeds))]
        if not nodes:
            continue
        terminals = [_point(feeds[node]) for node in nodes]
        start = _center(chip, int(chain.start.index)) if chain.start is not None else None
        end = _center(chip, int(chain.end.index)) if chain.end is not None else None
        terminations = []
        if chain.start is None:
            terminations.append(terminals[0])
        if chain.end is None:
            terminations.append(terminals[-1])
        first = _name(chip, int(chain.start.index), component=False) if chain.start is not None else "termination"
        last = _name(chip, int(chain.end.index), component=False) if chain.end is not None else "termination"
        resonators = ", ".join(_name(chip, int(ring[node].index), component=True) for node in nodes)
        geometry.feedlines.append(
            Feedline(
                points=[*([start] if start is not None else []), *terminals, *([end] if end is not None else [])],
                terminals=terminals,
                terminations=terminations,
                label=f"feedline {number}: {first} → {resonators} → {last}",
            )
        )


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


def _plan_of(capacity: bytes) -> CapacityPlanT:
    """The capacity plan of a run, for a stage that is drawn over it.

    Returns:
        The plan.

    Raises:
        PlanningError: If the bytes are no capacity plan.
    """
    plan = read_artifact(capacity).output
    if not isinstance(plan, CapacityPlanT):
        msg = "the capacity artifact is not a capacity plan"
        raise PlanningError(msg)
    return plan


def _assignment_of(assignment: bytes) -> AssignmentT:
    """The assignment of a run, for a stage that is drawn with its feedlines.

    Returns:
        The assignment.

    Raises:
        PlanningError: If the bytes are no assignment.
    """
    chosen = read_artifact(assignment).output
    if not isinstance(chosen, AssignmentT):
        msg = "the assignment artifact is not an assignment"
        raise PlanningError(msg)
    return chosen


def planning_geometry(
    data: bytes,
    chip: ChipT,
    stage: str,
    capacity: bytes | None = None,
    assignment: bytes | None = None,
) -> PlanningGeometry:
    """Read one planning artifact into the shapes it describes.

    Args:
        data: The bytes of the artifact.
        chip: The chip of the run, for the positions of the ports an artifact names.
        stage: The stage the artifact is drawn as.
        capacity: The capacity artifact of the same run, for the stages that are drawn over the
            free space they had to fit into: the global stage over the gates it paid for, and the
            corridor stage over the partitions its wires run through.
        assignment: The assignment artifact of the same run, for the corridor stage, which is drawn
            with the feedlines that its resonator wires start at.

    Returns:
        The geometry, in layout units.

    Raises:
        PlanningError: If the stage is no planning stage, or the artifact is not its output.
    """
    if stage not in PLANNING_STAGES:
        msg = f"'{stage}' is no planning stage; the planning stages are {', '.join(PLANNING_STAGES)}"
        raise PlanningError(msg)
    output = read_artifact(data).output
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
        # The free space behind a port decides where a wire may surface, so the gates that measure
        # it belong in the picture of the circuit that had to pay for them.
        if capacity is not None:
            plan = _plan_of(capacity)
            _bottlenecks(plan, geometry)
            _keepout(plan, geometry)
    elif stage == "assign":
        if not isinstance(output, AssignmentT):
            msg = "the artifact is not an assignment"
            raise PlanningError(msg)
        _assignment(output, chip, geometry)
    else:
        if not isinstance(output, CorridorRoutingT):
            msg = "the artifact is not a corridor routing"
            raise PlanningError(msg)
        _corridor(output, geometry)
        # A corridor is a way through the partitions, so the partitions make the picture readable.
        if capacity is not None:
            _capacity(_plan_of(capacity), geometry)
        # The wire of a resonator starts at its feed, which its feedline passes.
        if assignment is not None:
            _feedlines(_assignment_of(assignment), chip, geometry)
    return geometry
