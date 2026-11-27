# Copyright (c) 2026 Chair for Design Automation, TUM
# Copyright (c) 2026 Munich Quantum Software Company GmbH
# All rights reserved.
#
# SPDX-License-Identifier: MIT
#
# Licensed under the MIT License

"""The shapes that the planning stages produced, read back out of their artifacts.

This module is the one place that knows how a capacity plan, a global routing, an assignment, a
corridor routing and a detail routing turn into geometry. Both renderers use it: ``plot`` draws the shapes as SVG, and
``render`` writes them to GDSII or OASIS on layers of their own. Neither reaches into the
pipeline; both read the artifacts of a run directory, so either can draw a run that stopped early.
"""

from __future__ import annotations

import math
import re
import struct
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Protocol

from .artifacts import read_artifact
from .flatbuffers.artifacts.Assignment import AssignmentT
from .flatbuffers.artifacts.CapacityElement import CapacityElement
from .flatbuffers.artifacts.CapacityPlan import CapacityPlanT
from .flatbuffers.artifacts.CorridorRouting import CorridorRoutingT
from .flatbuffers.artifacts.DetailRouting import DetailRoutingT
from .flatbuffers.artifacts.GlobalRouting import GlobalRoutingT
from .flatbuffers.design.AssignedRole import AssignedRole
from .run import STAGE_FILES

if TYPE_CHECKING:
    from collections.abc import Callable, Iterable

    from .flatbuffers.artifacts.GridExtent import GridExtentT
    from .flatbuffers.design.Chip import ChipT

__all__ = ["PLANNING_STAGES", "CorridorPath", "Feedline", "PlanningError", "PlanningGeometry", "planning_geometry"]

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
class CorridorPath:
    """The geometry of one routed connection."""

    #: The coarse route, from its feed over its crossings to its target cell.
    points: list[Point]
    #: The illustrative approach from the target cell into the target port, when available.
    approach: list[Point] = field(default_factory=list)


@dataclass
class PlanningGeometry:
    """The shapes one planning stage produced, in layout units.

    The lists group the shapes by their role in the picture. A renderer can give each group a
    color or a GDS layer.
    """

    #: The outline of every partition of the free space, as its closed rings. A partition with a
    #: hole has more than one ring, and a point lies in the partition where it lies inside an odd
    #: number of them.
    partitions: list[list[list[Point]]] = field(default_factory=list)
    #: A tone for each partition of ``partitions``, so that two partitions that share a border never
    #: share a tone: a small number from zero up. None for a partition the artwork closes in.
    partition_tones: list[int | None] = field(default_factory=list)
    #: The blocked cells of the planner's detail grid, as rectangles in layout units.
    blocked: list[list[Point]] = field(default_factory=list)
    #: The port approaches as closed rings, including cells the planner reopened.
    keepout: list[list[Point]] = field(default_factory=list)
    #: The border between two partitions, as the runs of cell edges it consists of, one polyline
    #: per run.
    borders: list[list[Point]] = field(default_factory=list)
    #: The borders between partitions that artwork and blocked cells close in on every side, so
    #: that no wire from a launcher reaches them, in the same form as ``borders``.
    enclosed_borders: list[list[Point]] = field(default_factory=list)
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
    #: Each feedline chain of the assignment, from its first launcher through the feeds of its
    #: resonators to its last.
    feedlines: list[Feedline] = field(default_factory=list)
    #: The coarse route and port approach of each routed wire.
    corridors: list[CorridorPath] = field(default_factory=list)
    #: Every crossing slot a border offers, taken or not.
    slots: list[Point] = field(default_factory=list)
    #: Each plain wire of the ring as the Detail stage drew it, as the polyline through its bends.
    wires: list[list[Point]] = field(default_factory=list)
    #: Each resonator wire of the ring, in the same form. The assignment tells a resonator from a
    #: plain wire.
    resonators: list[list[Point]] = field(default_factory=list)
    #: Each wire of the inner circuit, in the same form.
    inner_wires: list[list[Point]] = field(default_factory=list)
    #: The clearance that the Detail stage keeps between two wires, in layout units, or zero when
    #: the caller names no design rule. It is ``min_wire_spacing`` converted to whole cells, which is
    #: what the stage keeps, not the rule itself.
    clearance: float = 0.0

    def is_empty(self) -> bool:
        """Whether the stage produced nothing to draw.

        Returns:
            True when every layer is empty.
        """
        return not any((
            self.partitions,
            self.blocked,
            self.keepout,
            self.borders,
            self.enclosed_borders,
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
            self.wires,
            self.resonators,
            self.inner_wires,
        ))


def _entries(items: list[Any] | None) -> list[Any]:
    """The entries of a schema list that are there.

    The generated object model types every list as optional, and every entry with it. Here an
    absent list or entry means nothing to draw.

    Returns:
        The entries, without the absent ones.
    """
    return [item for item in (items or []) if item is not None]


def _indices(values: Iterable[int] | None) -> list[int]:
    """The entries of a schema vector of indices, as integers.

    The generated object model reads a vector of numbers into a NumPy array when NumPy is
    importable, and into a list otherwise. A NumPy array has no single truth value, so the vector is
    read here into a list, never tested for truth.

    Returns:
        The indices, or none when the vector is absent.
    """
    return [] if values is None else [int(value) for value in values]


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


def _corner(value: float, origin: float, step: float) -> float:
    """A layout coordinate in corner coordinates of the grid, not rounded.

    The center of cell k lies at ``origin + k * step``, so corner k, where cell k starts, lies half a
    step before it. The partition geometry of a plan runs along these corners.

    Returns:
        The coordinate, in cells from corner 0.
    """
    return (value - origin) / step + 0.5


def _border_runs(samples: list[Point], grid: GridExtentT | None) -> list[list[Point]]:
    """The cell edges of one border, joined into runs.

    A plan stores a border as the midpoints of its cell edges, in the order the grid was scanned
    rather than along the border, so they do not form a polyline. A midpoint on a whole cell coordinate
    across x lies on a vertical edge, and any other on a horizontal one; each edge spans one cell.
    Edges that share an end form one run, and a run ends where the border forks or stops.

    Args:
        samples: The midpoints, in layout units.
        grid: The detail grid the plan was made on.

    Returns:
        The runs, each as the corners it passes, in a fixed order.
    """
    if grid is None or grid.origin is None or not grid.cellWidth or not grid.cellHeight:
        return [samples] if samples else []
    origin_x, origin_y = float(grid.origin.x), float(grid.origin.y)
    width, height = float(grid.cellWidth), float(grid.cellHeight)
    edges: list[tuple[tuple[int, int], tuple[int, int]]] = []
    for x, y in samples:
        column = _corner(x, origin_x, width)
        row = _corner(y, origin_y, height)
        if abs(column - round(column)) < 0.25:
            c, r = round(column), math.floor(row)
            edges.append(((c, r), (c, r + 1)))
        else:
            c, r = math.floor(column), round(row)
            edges.append(((c, r), (c + 1, r)))

    touching: dict[tuple[int, int], list[int]] = {}
    for index, (first, second) in enumerate(edges):
        touching.setdefault(first, []).append(index)
        touching.setdefault(second, []).append(index)
    used = [False] * len(edges)
    runs: list[list[Point]] = []
    # A run starts where the border stops or forks; what is left after that are closed loops.
    ends = sorted(corner for corner, indices in touching.items() if len(indices) != 2)
    for start in [*ends, *sorted(touching)]:
        for first_edge in touching[start]:
            if used[first_edge]:
                continue
            run, corner, edge = [start], start, first_edge
            while True:
                used[edge] = True
                first, second = edges[edge]
                corner = second if first == corner else first
                run.append(corner)
                onward = [other for other in touching[corner] if not used[other]]
                if len(touching[corner]) != 2 or not onward:
                    break
                edge = onward[0]
            runs.append([(origin_x + (x - 0.5) * width, origin_y + (y - 0.5) * height) for x, y in run])
    return _bridged(runs, 3.0 * max(width, height))


def _closest_join(runs: list[list[Point]], gap: float) -> tuple[int, int, list[Point]] | None:
    """The first two runs whose ends lie at most ``gap`` apart, and the run they join into.

    Returns:
        The index of the first run, the index of the second, and the joined run, or None.
    """
    for index, run in enumerate(runs):
        for other in range(index + 1, len(runs)):
            candidate = runs[other]
            for head, tail in ((run, candidate), (run, candidate[::-1]), (candidate, run), (candidate[::-1], run)):
                if math.dist(head[-1], tail[0]) <= gap:
                    return index, other, head + tail[1:] if head[-1] == tail[0] else head + tail
    return None


def _bridged(runs: list[list[Point]], gap: float) -> list[list[Point]]:
    """Join the runs of one border whose ends lie at most ``gap`` apart.

    Where a thin strip of blocked cells lies between two partitions, they touch only in places, and
    their border falls apart into single cell edges. Joined across gaps of a few cells, the border
    reads as the one line it is.

    Returns:
        The joined runs, in the order the first of each was found.
    """
    runs = [list(run) for run in runs]
    while (join := _closest_join(runs, gap)) is not None:
        index, other, joined = join
        runs[index] = joined
        del runs[other]
    return runs


def _blocked(plan: CapacityPlanT) -> list[list[Point]]:
    """Draw the blocked detail cells as rectangles in layout units.

    Returns:
        The rectangles, as rings of four corners.
    """
    grid, words = plan.detailGrid, plan.blockedWords
    if grid is None or grid.origin is None or words is None or not grid.width or not grid.height:
        return []
    width, height = int(grid.width), int(grid.height)
    origin_x, origin_y = float(grid.origin.x), float(grid.origin.y)
    cell_width, cell_height = float(grid.cellWidth), float(grid.cellHeight)
    packed = struct.pack(f"<{len(words)}Q", *words)
    full = (1 << width) - 1
    # Runs of a row that the row above continues unchanged grow one rectangle.
    rectangles: list[list[Point]] = []
    open_runs: dict[tuple[int, int], int] = {}
    for y in range(height + 1):
        first_bit = y * width
        first_byte, skip = divmod(first_bit, 8)
        last_byte = (first_bit + width + 7) // 8
        row = (int.from_bytes(packed[first_byte:last_byte], "little") >> skip) & full if y < height else 0
        # The least significant bit is the first column.
        runs = {match.span() for match in re.finditer(r"1+", f"{row:b}"[::-1])}
        for run in list(open_runs):
            if run not in runs:
                top = open_runs.pop(run)
                x0, x1 = origin_x + (run[0] - 0.5) * cell_width, origin_x + (run[1] - 0.5) * cell_width
                y0, y1 = origin_y + (top - 0.5) * cell_height, origin_y + (y - 0.5) * cell_height
                rectangles.append([(x0, y0), (x1, y0), (x1, y1), (x0, y1)])
        for run in runs:
            open_runs.setdefault(run, y)
    return rectangles


def _enclosed(plan: CapacityPlanT) -> set[int]:
    """The labels of the partitions that artwork and blocked cells close in on every side.

    Two partitions that share a border are joined. A group of joined partitions is open when one of
    them reaches the outermost free cells of the grid, on any of its four sides; every other group
    lies closed in, inside the artwork.

    Returns:
        The labels of the partitions of the closed groups.
    """
    joined: dict[int, set[int]] = {}
    for border in _entries(plan.borders):
        first, second = int(border.first), int(border.second)
        joined.setdefault(first, set()).add(second)
        joined.setdefault(second, set()).add(first)
    extents: dict[int, tuple[float, float, float, float]] = {}
    for partition in _entries(plan.partitions):
        points = [_point(vertex) for ring in _entries(partition.outlines) for vertex in _entries(ring.vertices)]
        if points:
            xs, ys = [x for x, _ in points], [y for _, y in points]
            extents[int(partition.label)] = (min(xs), min(ys), max(xs), max(ys))
    if not extents:
        return set()
    # The partitions on the outermost free cells, one per side at least.
    outer = {
        label
        for side in range(4)
        for best in [(min if side < 2 else max)(extent[side] for extent in extents.values())]
        for label, extent in extents.items()
        if extent[side] == best
    }
    reached, pending = set(outer), list(outer)
    while pending:
        for other in joined.get(pending.pop(), set()):
            if other not in reached:
                reached.add(other)
                pending.append(other)
    return set(extents) - reached


def _tones(plan: CapacityPlanT, enclosed: set[int]) -> dict[int, int]:
    """A tone for every open partition, so that two that share a border differ.

    The partitions take tones greedily, the one with the most neighbors first and ties by label, so
    that the result depends only on the plan. Each takes the lowest tone no neighbor has yet.

    Returns:
        The tone of each open partition, by label.
    """
    neighbors: dict[int, set[int]] = {
        int(partition.label): set() for partition in _entries(plan.partitions) if int(partition.label) not in enclosed
    }
    for border in _entries(plan.borders):
        first, second = int(border.first), int(border.second)
        if first in neighbors and second in neighbors:
            neighbors[first].add(second)
            neighbors[second].add(first)
    tones: dict[int, int] = {}
    for label in sorted(neighbors, key=lambda label: (-len(neighbors[label]), label)):
        taken = {tones[other] for other in neighbors[label] if other in tones}
        tones[label] = next(tone for tone in range(len(taken) + 1) if tone not in taken)
    return tones


def _capacity(plan: CapacityPlanT, geometry: PlanningGeometry) -> None:
    """Fill the layers a capacity plan carries."""
    _keepout(plan, geometry)
    geometry.blocked = _blocked(plan)
    enclosed = _enclosed(plan)
    tones = _tones(plan, enclosed)
    for partition in _entries(plan.partitions):
        rings = [[_point(vertex) for vertex in _entries(ring.vertices)] for ring in _entries(partition.outlines)]
        rings = [ring for ring in rings if len(ring) >= 3]
        if rings:
            geometry.partitions.append(rings)
            geometry.partition_tones.append(tones.get(int(partition.label)))
    for border in _entries(plan.borders):
        samples = [_point(sample) for sample in _entries(border.samples)]
        runs = _border_runs(samples, plan.detailGrid)
        if int(border.first) in enclosed:
            geometry.enclosed_borders.extend(runs)
        else:
            geometry.borders.extend(runs)
    _bottlenecks(plan, geometry)
    for slot in _entries(plan.launchers):
        geometry.launchers.append(_point(slot.position))


def _chains(plan: CapacityPlanT, chip: ChipT, geometry: PlanningGeometry) -> None:
    """Fill the chain layer: one line per link of every capacity chain.

    The lines show the links between targets and bottlenecks in each chain.
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

    walked: set[int] = set()
    for root in _indices(plan.chains):
        pending: list[tuple[Point | None, int]] = [(None, root)]
        while pending:
            previous, index = pending.pop()
            here = position(index)
            if previous is not None and here is not None:
                geometry.chains.append((previous, here))
            if index in walked:
                continue
            walked.add(index)
            pending.extend((here, child) for child in reversed(_indices(nodes[index].next)))


def _global(routing: GlobalRoutingT, geometry: PlanningGeometry, *, lattice: bool = True) -> None:
    """Fill the layers a global routing carries.

    Args:
        routing: The global routing.
        geometry: The shapes to fill.
        lattice: Whether to draw every edge of the lattices, or only the edges the circuit selected.
    """
    for drawn in _entries(routing.lattices):
        points = [_point(point) for point in _entries(drawn.points)]
        edges = _indices(drawn.edges)
        selected = set(_indices(drawn.selected))
        for index in range(0, len(edges) - 1, 2):
            first, second = edges[index], edges[index + 1]
            if first >= len(points) or second >= len(points):
                continue
            segment = (points[first], points[second])
            if lattice:
                geometry.lattice.append(segment)
            if index // 2 in selected:
                geometry.inner.append(segment)


def _routing_of(routing: bytes) -> GlobalRoutingT:
    """The global routing of a run, for a stage that is drawn with the inner circuit.

    Returns:
        The global routing.

    Raises:
        PlanningError: If the bytes do not hold a global routing.
    """
    solved = read_artifact(routing).output
    if not isinstance(solved, GlobalRoutingT):
        msg = "the global artifact is not a global routing"
        raise PlanningError(msg)
    return solved


def _assignment(assignment: AssignmentT, chip: ChipT, geometry: PlanningGeometry) -> None:
    """Fill the layers an assignment carries."""
    ring = _entries(assignment.ring)
    launchers = _entries(assignment.launchers)
    feeds = _entries(assignment.feeds)
    for entry in ring:
        position = _center(chip, int(entry.index))
        if position is not None:
            geometry.ring.append(position)
    # A resonator is fed on its feedline, so only the feed of a conventional port is a launcher slot.
    chained = {node for chain in _entries(assignment.chains) for node in _indices(chain.nodes)}
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
        if index < len(feeds) and index < len(launchers) and index not in chained:
            center = _center(chip, int(launchers[index].index))
            if center is not None and target != center and target not in geometry.launchers:
                geometry.launchers.append(target)
    _feedlines(assignment, chip, geometry)


def _feedlines(assignment: AssignmentT, chip: ChipT, geometry: PlanningGeometry) -> None:
    """Fill the feedline chains of an assignment."""
    ring = _entries(assignment.ring)
    feeds = _entries(assignment.feeds)
    for number, chain in enumerate(_entries(assignment.chains), start=1):
        nodes = [node for node in _indices(chain.nodes) if node < min(len(ring), len(feeds))]
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


def _stub(chip: ChipT, index: int, start: Point) -> list[Point]:
    """The approach from where a corridor ends to its target port.

    A wire enters a port along the direction of the port, so the approach turns once: across to the
    axis of the port, then along it into the port.

    Returns:
        The points of the approach, or nothing when the index names no port.
    """
    port = _center(chip, index)
    if port is None or port == start:
        return []
    ports: list[Any] = chip.ports or []
    angle = math.radians(float(ports[index].orientation))
    direction = (math.cos(angle), math.sin(angle))
    along = max((start[0] - port[0]) * direction[0] + (start[1] - port[1]) * direction[1], 0.0)
    corner = (port[0] + along * direction[0], port[1] + along * direction[1])
    if math.dist(corner, start) < 1e-6 or math.dist(corner, port) < 1e-6:
        return [start, port]
    return [start, corner, port]


def _corridor(
    routing: CorridorRoutingT,
    chip: ChipT,
    plan: CapacityPlanT | None,
    assignment: AssignmentT | None,
    geometry: PlanningGeometry,
) -> None:
    """Fill the layers a corridor routing carries.

    The routing holds one corridor per connection of the assignment, in its order, so the
    assignment supplies the exact feed and the target port for each corridor.

    Raises:
        PlanningError: If the routing and assignment have different numbers of connections, or a
            slot references a border outside the Capacity plan.
    """
    routes = routing.corridors or []
    connections: list[Any] = assignment.connections or [] if assignment is not None else []
    if assignment is not None and len(connections) != len(routes):
        msg = "the corridor routing and assignment have different numbers of connections"
        raise PlanningError(msg)

    for border in _entries(routing.slots):
        if plan is not None and border.border >= len(plan.borders or []):
            msg = f"crossing slot border {border.border} is outside the Capacity plan"
            raise PlanningError(msg)
        geometry.slots.extend(_point(position) for position in _entries(border.positions))
    feeds = assignment.feeds or [] if assignment is not None else []
    for index, corridor in enumerate(routes):
        if (
            corridor is None
            or corridor.partitions is None
            or len(corridor.partitions) == 0
            or corridor.source is None
            or corridor.target is None
        ):
            continue
        source = _point(corridor.source)
        feed = feeds[index] if index < len(feeds) else None
        route = [source]
        if feed is not None and (position := _point(feed)) != source:
            route.insert(0, position)
        route.extend(_point(crossing) for crossing in _entries(corridor.crossings))
        route.append(_point(corridor.target))
        connection = connections[index] if index < len(connections) else None
        target = connection.target if connection is not None else None
        approach = _stub(chip, int(target.index), route[-1]) if target is not None else []
        geometry.corridors.append(CorridorPath(route, approach))


def _bends(path: list[Any], to_layout: Callable[[int, int], Point]) -> list[Point]:
    """One drawn wire as its bends, in layout units.

    A wire is stored cell by cell, and a run of cells in one direction is a straight line, so only
    the cells where the direction changes are kept.

    Returns:
        The corners of the polyline, ends included.
    """
    if not path:
        return []
    kept = [0]
    for index in range(1, len(path) - 1):
        before, here, after = path[index - 1], path[index], path[index + 1]
        if (here.x - before.x, here.y - before.y) != (after.x - here.x, after.y - here.y):
            kept.append(index)
    if len(path) > 1:
        kept.append(len(path) - 1)
    return [to_layout(path[index].x, path[index].y) for index in kept]


def _detail_clearance(wire_spacing: float, cell: float) -> float:
    """The clearance that the Detail stage keeps for a rule of ``wire_spacing`` on cells of ``cell``.

    The rule spans ``ceil(wire_spacing / cell)`` cells, and the stage keeps one cell less than that
    clear. As in the core, a ratio that exceeds a whole number by less than a relative 1e-12 counts
    as that whole number.

    Returns:
        The clearance in layout units, or zero when the rule or the cell is not positive.
    """
    if wire_spacing <= 0 or cell <= 0:
        return 0.0
    ratio = wire_spacing / cell
    return max(0, math.ceil(ratio - ratio * 1e-12) - 1) * cell


def _resonator_slots(assignment: AssignmentT | None) -> set[int]:
    """The connections of an assignment that carry a resonator.

    Returns:
        The indices into the connections of the assignment.
    """
    if assignment is None:
        return set()
    return {
        index
        for index, connection in enumerate(_entries(assignment.connections))
        if int(connection.sourceRole or 0) == AssignedRole.ResonatorSource
    }


def _detail(routing: DetailRoutingT, geometry: PlanningGeometry, wire_spacing: float, resonators: set[int]) -> None:
    """Fill the layers a detail routing carries."""
    grid = routing.grid
    if grid is None or grid.origin is None or not grid.width or not grid.height:
        return
    geometry.clearance = _detail_clearance(wire_spacing, min(float(grid.cellWidth), float(grid.cellHeight)))

    def to_layout(x: int, y: int) -> Point:
        # The origin of the grid is the center of cell (0, 0).
        return (
            float(grid.origin.x) + x * float(grid.cellWidth),
            float(grid.origin.y) + y * float(grid.cellHeight),
        )

    for index, wire in enumerate(_entries(routing.wires)):
        points = _bends(_entries(wire.path), to_layout)
        if points:
            (geometry.resonators if index in resonators else geometry.wires).append(points)
    for wire in _entries(routing.inner):
        points = _bends(_entries(wire.path), to_layout)
        if points:
            geometry.inner_wires.append(points)


def _plan_of(capacity: bytes) -> CapacityPlanT:
    """The capacity plan of a run, for a stage that is drawn over it.

    Returns:
        The plan.

    Raises:
        PlanningError: If the bytes do not hold a capacity plan.
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
        PlanningError: If the bytes do not hold an assignment.
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
    global_routing: bytes | None = None,
    *,
    assignment: bytes | None = None,
    clearance: float = 0.0,
) -> PlanningGeometry:
    """Read one planning artifact into the shapes it describes.

    Args:
        data: The bytes of the artifact.
        chip: The chip of the run, for the positions of the ports an artifact names.
        stage: The stage the artifact is drawn as.
        capacity: The capacity artifact of the same run, for the stages that are drawn over the
            free space they had to fit into: the global stage over the cells the grid blocked, and
            the assign, corridor and detail stages over the partitions their chords and wires run
            through.
        global_routing: The global artifact of the same run, for the assign and corridor stages,
            which are drawn with the lattice edges that the inner circuit selected.
        assignment: The assignment artifact of the same run. Corridor uses it to draw feedlines and
            the approach of each wire to its target port, and Detail to tell a resonator wire from
            a plain one.
        clearance: The design rule ``min_wire_spacing``, in layout units. The detail stage converts
            it to the whole cells it keeps clear, and a picture draws a band of that width around
            every wire, so that two wires that come too close show as two bands that overlap.

    Returns:
        The geometry, in layout units.

    Raises:
        PlanningError: If the stage or its artifact is invalid, or the Corridor artifact has an
            inconsistent connection count or border reference.
    """
    if stage not in PLANNING_STAGES:
        msg = f"'{stage}' is not a planning stage; the planning stages are {', '.join(PLANNING_STAGES)}"
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
        # The lattice lines run between the cells the grid blocked. The gates and the keep-out are
        # layers of the GDS; the SVG leaves them out, as it does in the other pictures.
        if capacity is not None:
            plan = _plan_of(capacity)
            geometry.blocked = _blocked(plan)
            _bottlenecks(plan, geometry)
            _keepout(plan, geometry)
    elif stage == "assign":
        if not isinstance(output, AssignmentT):
            msg = "the artifact is not an assignment"
            raise PlanningError(msg)
        # The plan comes first, so that a feed on a launcher slot of the plan is not marked twice.
        if capacity is not None:
            _capacity(_plan_of(capacity), geometry)
        _assignment(output, chip, geometry)
        # The inner circuit decided which ports joined the ring, so its wires belong in the picture.
        if global_routing is not None:
            _global(_routing_of(global_routing), geometry, lattice=False)
    elif stage == "detail":
        if not isinstance(output, DetailRoutingT):
            msg = "the artifact is not a detail routing"
            raise PlanningError(msg)
        chosen = _assignment_of(assignment) if assignment is not None else None
        _detail(output, geometry, clearance, _resonator_slots(chosen))
        # The seed of every wire stayed inside its partitions, so they make a drawn wire readable.
        if capacity is not None:
            _capacity(_plan_of(capacity), geometry)
    else:
        if not isinstance(output, CorridorRoutingT):
            msg = "the artifact is not a corridor routing"
            raise PlanningError(msg)
        chosen = _assignment_of(assignment) if assignment is not None else None
        plan = _plan_of(capacity) if capacity is not None else None
        _corridor(output, chip, plan, chosen, geometry)
        # A corridor is a way through the partitions, so the partitions make the picture readable.
        if plan is not None:
            _capacity(plan, geometry)
        # The wire of a resonator starts at its feed, which its feedline passes.
        if chosen is not None:
            _feedlines(chosen, chip, geometry)
        # The corridors route the outer wires; the inner circuit is the rest of the wiring.
        if global_routing is not None:
            _global(_routing_of(global_routing), geometry, lattice=False)
    return geometry
