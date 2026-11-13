"""Keep a path, and run the A* again only when something calls for it.

Public API (mm, s, table frame):
    Replanner(finder)                   wraps the PathFinder of the match
    update(now, pose, goal, obstacles)  call every tick: kept or new PlanResult
    ahead(pose, distance)               point `distance` mm down the path, or
                                        None
    remaining_distance(pose)            path length left to the goal
    deviation(pose)                     distance from the robot to the path
    reset()                             forget the path, replan next update()
    plans, last_reason                  A* runs so far, why the last one ran

Planning at every tick of the match loop cost 98 % of a simulated match and
would cost 10 to 20 ms per tick on the Raspberry Pi. A path stays valid until
one of these events:

    - the goal changed;
    - an obstacle appeared, disappeared, or moved more than MOVE_THRESHOLD_MM;
    - an obstacle now cuts the remaining path;
    - the robot drifted more than DEVIATION_MM away from the path;
    - the last attempt failed (retried every RETRY_PERIOD_S at most).

Between two plans the path is only followed: ahead() returns the point a
given distance down the path from the projection of the robot onto it.
"""

import math
from typing import List, Optional, Sequence, Tuple

from .pathfinder import (
    Disc,
    PathFinder,
    PlanResult,
    Point,
    shape_distance,
)

__all__ = ["Replanner"]

MOVE_THRESHOLD_MM = 100.0
DEVIATION_MM = 150.0
GOAL_CHANGE_MM = 1.0
RETRY_PERIOD_S = 0.5
# Spacing of the samples when checking the remaining path against obstacles.
CHECK_STEP_MM = 25.0


def _obstacle_key(shape) -> Tuple:
    """What identifies an obstacle between two calls, beyond its position."""
    if isinstance(shape, Disc):
        return ("disc", shape.name, round(shape.radius, 1))
    return (type(shape).__name__, getattr(shape, "name", ""))


def _obstacle_position(shape) -> Point:
    if hasattr(shape, "cx"):
        return shape.cx, shape.cy
    return shape.x, shape.y


class Replanner:
    """A path kept across ticks, replanned on events only."""

    def __init__(self, finder: PathFinder) -> None:
        """Args:
            finder: the planner to call when a new path is needed.
        """
        self.finder = finder
        self.result: Optional[PlanResult] = None
        self.goal: Optional[Point] = None
        self._obstacles: List = []
        self._last_attempt = -math.inf
        self.plans = 0               # number of A* runs, for the reports
        self.last_reason = ""        # why the last plan was made

    def update(self, now: float, pose: Point, goal: Point,
               obstacles: Sequence = ()) -> PlanResult:
        """Return the current path, replanning first if an event calls for it.

        Args:
            now: time, s (only used to pace retries after a failure).
            pose: (x, y) of the robot, mm.
            goal: (x, y) to reach, mm.
            obstacles: shapes for this tick (see PathFinder.plan).

        Returns:
            PlanResult: the path kept or the new one, or the failure.
        """
        reason = self._replan_reason(now, pose, goal, obstacles)
        if reason:
            self.result = self.finder.plan(pose, goal, obstacles)
            self.goal = (float(goal[0]), float(goal[1]))
            self._obstacles = list(obstacles)
            self._last_attempt = now
            self.plans += 1
            self.last_reason = reason
        return self.result

    def reset(self) -> None:
        """Forget the path: the next update() plans again."""
        self.result = None
        self.goal = None

    def _replan_reason(self, now, pose, goal, obstacles) -> str:
        """Why a new plan is needed, or '' to keep the current one."""
        if self.result is None or self.goal is None:
            return "pas de chemin"
        if math.hypot(goal[0] - self.goal[0],
                      goal[1] - self.goal[1]) > GOAL_CHANGE_MM:
            return "cible changee"
        if not self.result.ok:
            if now - self._last_attempt >= RETRY_PERIOD_S:
                return "nouvel essai"
            return ""
        if self._obstacles_moved(obstacles):
            return "obstacle deplace"
        if self.deviation(pose) > DEVIATION_MM:
            return "robot hors du chemin"
        if self._path_cut(pose, obstacles):
            return "chemin coupe"
        return ""

    def _obstacles_moved(self, obstacles) -> bool:
        if len(obstacles) != len(self._obstacles):
            return True
        for new, old in zip(obstacles, self._obstacles):
            if _obstacle_key(new) != _obstacle_key(old):
                return True
            (nx, ny), (ox, oy) = (_obstacle_position(new),
                                  _obstacle_position(old))
            if math.hypot(nx - ox, ny - oy) > MOVE_THRESHOLD_MM:
                return True
        return False

    def _path_cut(self, pose, obstacles) -> bool:
        """True if the rest of the path now crosses a forbidden zone.

        Only the obstacles of this tick are checked, against their exact
        shape: the borders and fixed obstacles were already planned around.
        A robot that starts inside a zone (its way out) is not a cut.
        """
        if not obstacles:
            return False
        inflation = self.finder.inflation
        for a, b in self._remaining_segments(pose):
            length = math.hypot(b[0] - a[0], b[1] - a[1])
            steps = max(1, int(math.ceil(length / CHECK_STEP_MM)))
            for k in range(steps + 1):
                t = k / steps
                x = a[0] + (b[0] - a[0]) * t
                y = a[1] + (b[1] - a[1]) * t
                for shape in obstacles:
                    d = shape_distance(shape, x, y)
                    if d < inflation and d < shape_distance(
                            shape, pose[0], pose[1]):
                        return True
        return False

    # ── Following ─────────────────────────────────────────────────────────

    def _projection(self, pose) -> Tuple[int, Point, float]:
        """Segment index, projected point and distance of the robot to the
        path."""
        path = self.result.path
        best = (0, path[0], math.inf)
        for i, (a, b) in enumerate(zip(path, path[1:])):
            dx, dy = b[0] - a[0], b[1] - a[1]
            seg2 = dx * dx + dy * dy
            t = 0.0 if seg2 == 0.0 else max(0.0, min(1.0, (
                (pose[0] - a[0]) * dx + (pose[1] - a[1]) * dy) / seg2))
            p = (a[0] + dx * t, a[1] + dy * t)
            d = math.hypot(pose[0] - p[0], pose[1] - p[1])
            if d < best[2]:
                best = (i, p, d)
        return best

    def _remaining_segments(self, pose) -> List[Tuple[Point, Point]]:
        path = self.result.path
        if len(path) < 2:
            return []
        i, p, _ = self._projection(pose)
        points = [p] + list(path[i + 1:])
        return list(zip(points, points[1:]))

    def deviation(self, pose: Point) -> float:
        """Distance from the robot to the path, mm (0 without a path)."""
        if self.result is None or not self.result.ok or len(
                self.result.path) < 2:
            return 0.0
        return self._projection(pose)[2]

    def remaining_distance(self, pose: Point) -> float:
        """Length of the path still to travel, from the projection, mm."""
        if self.result is None or not self.result.ok:
            return math.inf
        return sum(math.hypot(b[0] - a[0], b[1] - a[1])
                   for a, b in self._remaining_segments(pose))

    def ahead(self, pose: Point, distance: float) -> Optional[Point]:
        """Point `distance` mm down the path from the robot's projection.

        Returns the goal when the path ends sooner, None without a path.
        """
        if self.result is None or not self.result.ok or not self.result.path:
            return None
        left = distance
        for a, b in self._remaining_segments(pose):
            length = math.hypot(b[0] - a[0], b[1] - a[1])
            if length >= left and length > 0.0:
                t = left / length
                return a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t
            left -= length
        return self.result.path[-1]

