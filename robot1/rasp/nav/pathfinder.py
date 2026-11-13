"""A* path planning on the table grid, for a holonomic base.

Public API (mm, table frame):
    Disc(x, y, radius, name)            round obstacle (opponent, pole)
    PathFinder(width, height, fixed, robot_radius)
                                        built once: borders, fixed obstacles
    plan(start, goal, obstacles)        PlanResult: .ok, .path, .status, .detail
    blocking_obstacle(x, y, obstacles)  what forbids a point ("bordure", name)
                                        or None
    point_clearance(x, y, obstacles)    margin left around a point, < 0 = too
                                        close
    shape_distance(shape, x, y)         distance to a shape, 0 inside it

Obstacle shapes: Disc, and from world.py RectZone, DiscZone, Obstacle.
In the match loop, go through nav.Replanner instead of calling plan() at
every tick.

The planner knows nothing about the LiDAR, the strategy or the team colour.
Obstacles are shapes, not points: Disc (an opponent, a pole) or any zone of
world.py with an exact distance (RectZone, DiscZone). Each grid cell keeps the
clearance of its centre, i.e. the distance to the nearest shape minus the
planning radius:

    clearance < 0                  forbidden, the robot would touch;
    0 <= clearance < soft band     allowed but costly, only around Disc
                                   obstacles (the opponent moves, walls and
                                   quarries do not);
    beyond                         free.

The table borders are an obstacle like any other.

A start inside a forbidden zone is not a failure: the robot is there, it must
be able to leave. From a forbidden cell the search only moves to cells of
strictly larger clearance, so the way out is the shortest climb out of the
zone and never a way through it.

The raw A* path is a staircase of 50 mm cells. It is smoothed by line of sight:
a corner is dropped when the straight segment that replaces it crosses no
forbidden cell and no cell costlier than the ones it replaces.
"""

import heapq
import math
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Sequence, Tuple

__all__ = [
    "Disc",
    "PathFinder",
    "PlanResult",
    "PlanStatus",
    "Point",
    "shape_distance",
]

Point = Tuple[float, float]

GRID_MM = 50.0
# Safety margin added to the planning radius. It also covers the grid: a point
# inside a free cell can be up to half a diagonal (35 mm) closer to an obstacle
# than the centre of that cell.
MARGIN_MM = 40.0
# Width of the costly band around Disc obstacles, beyond the margin, and the
# cost added at its inner edge, in cells of travel.
SOFT_BAND_MM = 150.0
SOFT_COST = 4.0
# Spacing of the samples when checking a segment against the grid.
SEGMENT_STEP_MM = GRID_MM / 2.0

_SQRT2 = math.sqrt(2.0)
_NEIGHBOURS = (
    (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
    (1, 1, _SQRT2), (1, -1, _SQRT2), (-1, 1, _SQRT2), (-1, -1, _SQRT2),
)


@dataclass(frozen=True)
class Disc:
    """A round obstacle: an opponent robot, a pole. Units: mm."""

    x: float
    y: float
    radius: float
    name: str = "disc"

    def distance(self, px: float, py: float) -> float:
        """Distance from (px, py) to the disc, 0 inside it."""
        return max(0.0, math.hypot(px - self.x, py - self.y) - self.radius)


class PlanStatus(Enum):
    """Outcome of a plan() call."""

    OK = "ok"
    START_OUTSIDE = "depart hors table"
    GOAL_OUTSIDE = "cible hors table"
    GOAL_BLOCKED = "cible dans un obstacle"
    NO_PATH = "aucun chemin"


@dataclass
class PlanResult:
    """A path, or the reason there is none."""

    status: PlanStatus
    path: List[Point] = field(default_factory=list)
    detail: str = ""
    # Number of cells of the raw A* path, before smoothing.
    raw_cells: int = 0

    @property
    def ok(self) -> bool:
        """True when a path was found."""
        return self.status is PlanStatus.OK


def shape_distance(shape, px: float, py: float) -> float:
    """Distance from (px, py) to an obstacle shape, 0 inside it.

    Accepts Disc, and the shapes of world.py: RectZone (oriented rectangle,
    centre, length along `angle`, width), DiscZone (centre, diameter) and
    Obstacle (axis-aligned rectangle, lower-left corner, width, height).

    Raises:
        TypeError: for a shape of unknown type.
    """
    if isinstance(shape, Disc):
        return shape.distance(px, py)
    if hasattr(shape, "length") and hasattr(shape, "angle"):
        dx, dy = px - shape.cx, py - shape.cy
        c, s = math.cos(shape.angle), math.sin(shape.angle)
        along = abs(dx * c + dy * s) - shape.length / 2.0
        across = abs(-dx * s + dy * c) - shape.width / 2.0
        return math.hypot(max(along, 0.0), max(across, 0.0))
    if hasattr(shape, "diameter"):
        return max(0.0, math.hypot(px - shape.cx, py - shape.cy)
                   - shape.diameter / 2.0)
    if hasattr(shape, "width") and hasattr(shape, "height"):
        dx = max(shape.x - px, 0.0, px - (shape.x + shape.width))
        dy = max(shape.y - py, 0.0, py - (shape.y + shape.height))
        return math.hypot(dx, dy)
    raise TypeError(f"forme d'obstacle inconnue : {type(shape).__name__}")


def _shape_extent(shape) -> Tuple[float, float, float]:
    """Centre and bounding radius of a shape, to bound the cells it reaches."""
    if isinstance(shape, Disc):
        return shape.x, shape.y, shape.radius
    if hasattr(shape, "length"):
        return shape.cx, shape.cy, math.hypot(shape.length, shape.width) / 2.0
    if hasattr(shape, "diameter"):
        return shape.cx, shape.cy, shape.diameter / 2.0
    return (shape.x + shape.width / 2.0, shape.y + shape.height / 2.0,
            math.hypot(shape.width, shape.height) / 2.0)


def _shape_name(shape) -> str:
    return getattr(shape, "name", type(shape).__name__)


class PathFinder:
    """A* planner on a grid of the table."""

    def __init__(
        self,
        width: float,
        height: float,
        fixed: Sequence = (),
        robot_radius: float = 120.0,
        margin: float = MARGIN_MM,
        grid: float = GRID_MM,
    ) -> None:
        """Build the static part of the grid: borders and fixed obstacles.

        Args:
            width: table size along X, mm.
            height: table size along Y, mm.
            fixed: obstacles present for the whole match (see shape_distance).
            robot_radius: planning radius of the robot, mm: centre of the
                robot to the farthest point of its outline.
            margin: safety margin added to robot_radius, mm.
            grid: cell size, mm.
        """
        self.width = float(width)
        self.height = float(height)
        self.grid = float(grid)
        self.robot_radius = float(robot_radius)
        self.margin = float(margin)
        self.inflation = self.robot_radius + self.margin
        self.cols = int(math.ceil(self.width / self.grid))
        self.rows = int(math.ceil(self.height / self.grid))
        self.fixed = tuple(fixed)

        # Clearance of each cell centre against borders and fixed obstacles,
        # flat index i = gx * rows + gy.
        self._static_clearance = [0.0] * (self.cols * self.rows)
        for gx in range(self.cols):
            for gy in range(self.rows):
                px, py = self.cell_centre(gx, gy)
                border = min(px, py, self.width - px, self.height - py)
                self._static_clearance[gx * self.rows + gy] = (
                    border - self.inflation)
        for shape in self.fixed:
            self._stamp(self._static_clearance, None, shape, soft=False)

    # ── Grid helpers ──────────────────────────────────────────────────────

    def cell_of(self, x: float, y: float) -> Tuple[int, int]:
        """Cell containing (x, y), clamped to the grid."""
        gx = min(self.cols - 1, max(0, int(x // self.grid)))
        gy = min(self.rows - 1, max(0, int(y // self.grid)))
        return gx, gy

    def cell_centre(self, gx: int, gy: int) -> Point:
        """Table coordinates of the centre of a cell."""
        return (gx + 0.5) * self.grid, (gy + 0.5) * self.grid

    def inside_table(self, x: float, y: float) -> bool:
        """True if (x, y) lies on the table."""
        return 0.0 <= x <= self.width and 0.0 <= y <= self.height

    def _stamp(self, clearance, extra_cost, shape, soft: bool) -> None:
        """Lower the clearance of the cells a shape reaches, add soft cost."""
        cx, cy, extent = _shape_extent(shape)
        reach = extent + self.inflation + (SOFT_BAND_MM if soft else 0.0)
        gx0, gy0 = self.cell_of(cx - reach, cy - reach)
        gx1, gy1 = self.cell_of(cx + reach, cy + reach)
        rows = self.rows
        for gx in range(gx0, gx1 + 1):
            px = (gx + 0.5) * self.grid
            for gy in range(gy0, gy1 + 1):
                py = (gy + 0.5) * self.grid
                value = shape_distance(shape, px, py) - self.inflation
                i = gx * rows + gy
                if value < clearance[i]:
                    clearance[i] = value
                if soft and 0.0 <= value < SOFT_BAND_MM:
                    cost = SOFT_COST * (1.0 - value / SOFT_BAND_MM)
                    if cost > extra_cost[i]:
                        extra_cost[i] = cost

    def build_grid(self, obstacles: Sequence) -> Tuple[list, list]:
        """Clearance and extra cost of every cell, with these obstacles.

        Disc obstacles get a soft band; zones (quarries, constructions) only
        their hard inflation.

        Returns:
            tuple: (clearance, extra_cost), flat lists indexed gx * rows + gy.
        """
        clearance = list(self._static_clearance)
        extra_cost = [0.0] * len(clearance)
        for shape in obstacles:
            self._stamp(clearance, extra_cost, shape,
                        soft=isinstance(shape, Disc))
        return clearance, extra_cost

    def point_clearance(self, x: float, y: float, obstacles: Sequence) -> float:
        """Exact clearance of a point: distance to the nearest obstacle or
        border, minus the planning radius and the margin."""
        best = min(x, y, self.width - x, self.height - y)
        for shape in tuple(self.fixed) + tuple(obstacles):
            best = min(best, shape_distance(shape, x, y))
        return best - self.inflation

    def blocking_obstacle(self, x: float, y: float,
                          obstacles: Sequence) -> Optional[str]:
        """Name of what forbids (x, y), or None if the point is free."""
        if min(x, y, self.width - x, self.height - y) < self.inflation:
            return "bordure"
        for shape in tuple(self.fixed) + tuple(obstacles):
            if shape_distance(shape, x, y) < self.inflation:
                return _shape_name(shape)
        return None

    # ── Planning ──────────────────────────────────────────────────────────

    def plan(self, start: Point, goal: Point,
             obstacles: Sequence = ()) -> PlanResult:
        """Plan a path from start to goal around the obstacles.

        Args:
            start: (x, y) of the robot, mm. May lie in a forbidden zone.
            goal: (x, y) to reach, mm. Must be free.
            obstacles: shapes for this call only (opponent, quarries...).

        Returns:
            PlanResult: status, and on success the smoothed path, from the
            exact start to the exact goal.
        """
        sx, sy = float(start[0]), float(start[1])
        gx_, gy_ = float(goal[0]), float(goal[1])
        if not self.inside_table(sx, sy):
            return PlanResult(PlanStatus.START_OUTSIDE,
                              detail=f"({sx:.0f}, {sy:.0f})")
        if not self.inside_table(gx_, gy_):
            return PlanResult(PlanStatus.GOAL_OUTSIDE,
                              detail=f"({gx_:.0f}, {gy_:.0f})")
        blocker = self.blocking_obstacle(gx_, gy_, obstacles)
        if blocker is not None:
            return PlanResult(PlanStatus.GOAL_BLOCKED,
                              detail=f"({gx_:.0f}, {gy_:.0f}) : {blocker}")

        clearance, extra_cost = self.build_grid(obstacles)
        start_cell = self.cell_of(sx, sy)
        goal_cell = self.cell_of(gx_, gy_)
        cells = self._astar(start_cell, goal_cell, clearance, extra_cost)
        if cells is None:
            return PlanResult(
                PlanStatus.NO_PATH,
                detail=f"({sx:.0f}, {sy:.0f}) -> ({gx_:.0f}, {gy_:.0f})")

        points = [self.cell_centre(*c) for c in cells]
        points[0] = (sx, sy)
        points[-1] = (gx_, gy_)
        path = self._smooth(points, cells, clearance, extra_cost)
        return PlanResult(PlanStatus.OK, path=path, raw_cells=len(cells))

    def _astar(self, start, goal, clearance, extra_cost):
        """A* over the cells; None when the goal cannot be reached.

        The goal cell is always enterable: its exact point was checked free,
        while its centre may sit just inside the inflation.
        """
        rows, cols = self.rows, self.cols
        start_i = start[0] * rows + start[1]
        goal_i = goal[0] * rows + goal[1]
        gx_goal, gy_goal = goal

        def heuristic(gx, gy):
            dx, dy = abs(gx - gx_goal), abs(gy - gy_goal)
            return (dx + dy) + (_SQRT2 - 2.0) * min(dx, dy)

        g_score = {start_i: 0.0}
        came_from = {}
        open_set = [(heuristic(*start), 0.0, start_i)]
        closed = set()

        while open_set:
            _, g, current = heapq.heappop(open_set)
            if current in closed:
                continue
            if current == goal_i:
                return self._rebuild(came_from, current)
            closed.add(current)
            cx, cy = divmod(current, rows)
            current_clear = clearance[current]
            escaping = current_clear < 0.0

            for dx, dy, step in _NEIGHBOURS:
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < cols and 0 <= ny < rows):
                    continue
                n = nx * rows + ny
                if n in closed:
                    continue
                n_clear = clearance[n]
                if n != goal_i:
                    if escaping:
                        # Way out of a forbidden zone: uphill only.
                        if n_clear <= current_clear:
                            continue
                    elif n_clear < 0.0:
                        continue
                if dx and dy and not escaping:
                    # No corner cutting next to a forbidden cell.
                    if (clearance[(cx + dx) * rows + cy] < 0.0
                            or clearance[cx * rows + cy + dy] < 0.0):
                        continue
                new_g = g + step * (1.0 + extra_cost[n])
                if new_g < g_score.get(n, math.inf):
                    g_score[n] = new_g
                    came_from[n] = current
                    heapq.heappush(open_set,
                                   (new_g + heuristic(nx, ny), new_g, n))
        return None

    def _rebuild(self, came_from, current) -> List[Tuple[int, int]]:
        cells = [divmod(current, self.rows)]
        while current in came_from:
            current = came_from[current]
            cells.append(divmod(current, self.rows))
        cells.reverse()
        return cells

    def _smooth(self, points, cells, clearance, extra_cost) -> List[Point]:
        """Drop the corners a straight segment can replace (line of sight).

        The way out of a forbidden zone is kept as the A* found it: smoothing
        it could cut through the zone instead of climbing out of it.
        """
        rows = self.rows
        first_free = 0
        while (first_free < len(cells) - 1
               and clearance[cells[first_free][0] * rows
                             + cells[first_free][1]] < 0.0):
            first_free += 1

        cost_of = [extra_cost[c[0] * rows + c[1]] for c in cells]
        path = list(points[:first_free + 1])
        anchor = first_free
        while anchor < len(points) - 1:
            # Farthest point in sight, searched from the end. A shortcut may
            # not get closer to the opponent than the raw path it replaces:
            # its cells may cost at most the costliest raw cell skipped.
            best = anchor + 1
            for j in range(len(points) - 1, anchor + 1, -1):
                worst_cost = max(cost_of[anchor:j + 1])
                if self._segment_clear(points[anchor], points[j],
                                       clearance, extra_cost, worst_cost,
                                       allowed=cells[-1]):
                    best = j
                    break
            path.append(points[best])
            anchor = best
        return path

    def _segment_clear(self, a: Point, b: Point, clearance, extra_cost,
                       max_cost: float, allowed) -> bool:
        """True if the segment crosses no forbidden cell and no cell costlier
        than max_cost. The goal cell (`allowed`) is exempt."""
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        steps = max(1, int(math.ceil(length / SEGMENT_STEP_MM)))
        rows = self.rows
        allowed_i = allowed[0] * rows + allowed[1]
        for k in range(steps + 1):
            t = k / steps
            gx, gy = self.cell_of(a[0] + (b[0] - a[0]) * t,
                                  a[1] + (b[1] - a[1]) * t)
            i = gx * rows + gy
            if i == allowed_i:
                continue
            if clearance[i] < 0.0 or extra_cost[i] > max_cost + 1e-9:
                return False
        return True
