"""Tests of the A* planner and of the replanning on events.

    pip install -e ".[dev]"
    python -m pytest tests

Pure module: no robot, no LiDAR, no serial port.
"""

import math

import pytest

from robot1.rasp.nav import Disc, PathFinder, PlanStatus, Replanner
from robot1.rasp.world import FIELD_HEIGHT_MM, FIELD_WIDTH_MM, RectZone

RADIUS = 120.0


@pytest.fixture
def finder():
    return PathFinder(FIELD_WIDTH_MM, FIELD_HEIGHT_MM, robot_radius=RADIUS)


def path_clearance(finder, path, obstacles, skip_start=False):
    """Smallest exact clearance along a path, sampled every 10 mm."""
    worst = math.inf
    for index, (a, b) in enumerate(zip(path, path[1:])):
        if skip_start and index == 0:
            continue
        n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / 10.0))
        for k in range(n + 1):
            x = a[0] + (b[0] - a[0]) * k / n
            y = a[1] + (b[1] - a[1]) * k / n
            worst = min(worst, finder.point_clearance(x, y, obstacles))
    return worst


# ── Planner ───────────────────────────────────────────────────────────────

def test_empty_table_is_one_straight_segment(finder):
    result = finder.plan((400, 1000), (2600, 1000))
    assert result.ok
    assert result.path == [(400.0, 1000.0), (2600.0, 1000.0)]


def test_goal_too_close_to_a_border_is_refused_with_reason(finder):
    result = finder.plan((400, 1000), (10, 10))
    assert result.status is PlanStatus.GOAL_BLOCKED
    assert "bordure" in result.detail


def test_goal_off_the_table_is_refused(finder):
    assert finder.plan((400, 1000), (3500, 500)).status is \
        PlanStatus.GOAL_OUTSIDE
    assert finder.plan((-10, 500), (500, 500)).status is \
        PlanStatus.START_OUTSIDE


def test_goal_inside_an_obstacle_names_it(finder):
    quarry = RectZone("C5", 1500, 500, 320, 110, math.pi / 2)
    result = finder.plan((400, 1000), (1500, 500), [quarry])
    assert result.status is PlanStatus.GOAL_BLOCKED
    assert "C5" in result.detail


def test_borders_are_inflated(finder):
    # Along the bottom border: the path keeps the robot off the wall.
    result = finder.plan((400, 300), (2600, 300))
    assert result.ok
    assert path_clearance(finder, result.path, []) >= -1e-6


def test_opponent_on_the_way_is_avoided(finder):
    opponent = Disc(1500, 1000, 223, "adversaire")
    result = finder.plan((400, 1000), (2600, 1000), [opponent])
    assert result.ok
    assert len(result.path) > 2
    # Exact distance to the opponent: the cells may let a smoothed segment
    # graze the inflation by less than half a cell diagonal.
    assert path_clearance(finder, result.path, [opponent]) > \
        -finder.grid * math.sqrt(2) / 2


def test_oriented_quarry_is_avoided(finder):
    quarry = RectZone("C9", 1500, 1000, 320, 110, math.pi / 4)
    result = finder.plan((1000, 600), (2000, 1400), [quarry])
    assert result.ok
    assert path_clearance(finder, result.path, [quarry]) > \
        -finder.grid * math.sqrt(2) / 2


def test_start_inside_the_opponent_zone_finds_a_way_out(finder):
    # 200 mm from the opponent centre: well inside its inflated zone.
    opponent = Disc(1500, 1000, 223, "adversaire")
    result = finder.plan((1500, 800), (2600, 1000), [opponent])
    assert result.ok
    # Each step of the way out moves away from the opponent.
    distances = [math.hypot(x - 1500, y - 1000) for x, y in result.path]
    exit_index = next(i for i, (x, y) in enumerate(result.path)
                      if finder.point_clearance(x, y, [opponent]) >= 0.0)
    assert distances[:exit_index + 1] == sorted(distances[:exit_index + 1])


def test_start_against_a_border_is_not_a_failure(finder):
    assert finder.plan((80, 1000), (1500, 1000)).ok


def test_wall_of_obstacles_gives_no_path(finder):
    barrage = [Disc(1500, y, 223, f"d{y}") for y in range(0, 2001, 200)]
    result = finder.plan((400, 1000), (2600, 1000), barrage)
    assert result.status is PlanStatus.NO_PATH


# ── Replanner ─────────────────────────────────────────────────────────────

def test_path_is_kept_while_nothing_changes(finder):
    nav = Replanner(finder)
    for tick in range(20):
        nav.update(tick * 0.05, (400 + tick * 10, 1000), (2600, 1000))
    assert nav.plans == 1


def test_goal_change_replans(finder):
    nav = Replanner(finder)
    nav.update(0.0, (400, 1000), (2600, 1000))
    nav.update(0.05, (400, 1000), (2600, 1500))
    assert nav.plans == 2
    assert nav.last_reason == "cible changee"


def test_small_opponent_motion_keeps_the_path(finder):
    nav = Replanner(finder)
    nav.update(0.0, (400, 1000), (2600, 1000), [Disc(1500, 300, 223)])
    nav.update(0.05, (400, 1000), (2600, 1000), [Disc(1530, 300, 223)])
    assert nav.plans == 1


def test_opponent_moving_onto_the_path_replans(finder):
    nav = Replanner(finder)
    nav.update(0.0, (400, 1000), (2600, 1000), [Disc(1500, 300, 223)])
    nav.update(0.05, (400, 1000), (2600, 1000), [Disc(1500, 1000, 223)])
    assert nav.plans == 2
    assert path_clearance(finder, nav.result.path, [Disc(1500, 1000, 223)]) \
        > -finder.grid


def test_drift_off_the_path_replans(finder):
    nav = Replanner(finder)
    nav.update(0.0, (400, 1000), (2600, 1000))
    nav.update(0.05, (1000, 1300), (2600, 1000))
    assert nav.last_reason == "robot hors du chemin"


def test_failure_is_retried_at_a_bounded_rate(finder):
    barrage = [Disc(1500, y, 223, f"d{y}") for y in range(0, 2001, 200)]
    nav = Replanner(finder)
    for tick in range(20):                       # 1 s at 20 Hz
        nav.update(tick * 0.05, (400, 1000), (2600, 1000), barrage)
    assert not nav.result.ok
    assert nav.plans == 2                        # at 0 s and at 0.5 s


def test_ahead_walks_along_the_path(finder):
    nav = Replanner(finder)
    nav.update(0.0, (400, 1000), (2600, 1000))
    x, y = nav.ahead((600, 1000), 200.0)
    assert x == pytest.approx(800.0)
    assert y == pytest.approx(1000.0)
    assert nav.ahead((2550, 1000), 200.0) == (2600.0, 1000.0)
    assert nav.remaining_distance((600, 1000)) == pytest.approx(2000.0)
