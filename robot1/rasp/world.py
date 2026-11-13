"""Table geometry and game constants for Eurobot 2027 ("The legend of Camelot").

Source: official game rules 1.0 and general rules 1.4. Positions were taken
from the vector drawing of the official table plan, about +/-5 mm, which is
below the manufacturing tolerance of the tables themselves.

Frame (the only one in the Raspberry Pi code):
    origin (0, 0) at the FRONT-LEFT corner of the table, seen from the public;
    X to the right, towards the BLUE side   [0 .. 3000 mm]
    Y towards the back of the table         [0 .. 2000 mm]
    theta counter-clockwise, 0 = facing +X; millimetres and radians everywhere.

    (0,2000) ──────────────────────────────────────────── (3000,2000)
       │ stable                quarries                     stable │
       │      M5  T4                                    T4  M5     │
       │ ┌─────┐     M4   T3                    T3   M4     ┌─────┐│
       │ │THRONE│        M3 D2       C6       D2 M3        │THRONE││
       │ │YELLOW│        M3 D2       C5       D2 M3        │ BLUE ││
       │ └─────┘     M2   T2                    T2   M2     └─────┘│
       │      M1  T1                                    T1  M1     │
       │ stable                quarries                     stable │
    (0,0) ────────────────────── public ─────────────────── (3000,0)

YELLOW plays on the left and is the reference: every team-dependent constant
below is written for YELLOW. BLUE is its mirror across x = 1500
(x -> 3000 - x, y kept, theta -> pi - theta), computed by mirror_for_team().

Who mirrors: only the strategy layer, once, when it turns its own plan into
table coordinates. The pathfinder, the LiDAR and the Teensy never know the
team colour and always work in absolute table coordinates. Terrain is
therefore neutral: it holds both castles in absolute coordinates.

This module holds NO LiDAR link parameter: the serial port lives in
config.json through vision/lidar_config.py, and the processing thresholds in
vision/detection.py and vision/localization.py. The only thing here the LiDAR
consumes is BeaconLayout.
"""
import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Sequence, Tuple, Union

# --- Table ------------------------------------------------------------------
FIELD_WIDTH_MM = 3000
FIELD_HEIGHT_MM = 2000
CENTER_LINE_X_MM = 1500
BORDER_HEIGHT_MM = 70        # the LiDAR scans at ~350 mm and never sees them
BORDER_THICKNESS_MM = 22

# WARNING: INDEPENDENT homonym of the ROBOT_RADIUS in
# robot1/teensy_moteur/include/config.h (160.0 mm).
# Here: obstacle inflation radius for the A* (nav/pathfinder.py), a planning
# margin. There: the physical center-to-wheel-axis distance, used by the
# holonomic kinematics and the rotational odometry.
# They have no reason to be equal. NEVER "align" them.
ROBOT_RADIUS_MM = 120

# --- Match ------------------------------------------------------------------
MATCH_DURATION_S = 100.0
# PAMI: nothing scores before this time, and a PAMI leaving its stable earlier
# is removed from the match.
ATTACK_PHASE_START_S = 85.0

# --- Robot envelope (general rules) -----------------------------------------
ROBOT_MAX_HEIGHT_MM = 350          # robot + carried elements, at any time
BEACON_PLATE_Z_MM = 430            # beacon mast plate, +/-5 mm
ROBOT_START_PERIMETER_MM = 1200    # convex hull at start: circle of ~382 mm max
ROBOT_DEPLOYED_PERIMETER_MM = 1400 # convex hull at any time: ~446 mm circle

# --- Game elements ----------------------------------------------------------
STONE_LENGTH_MM = 320              # cardboard box, ~90 g
STONE_WIDTH_MM = 110
STONE_HEIGHT_MM = 110
STONE_ARUCO_ID = 13                # 4x4 dictionary, on the 4 large faces
STONES_PER_QUARRY = 3              # lying flat, stacked: 330 mm high
BALL_DIAMETER_MM = 45              # compressible foam, team colour
BALLS_PER_TEAM = 10
MAX_STONES_STORED_IN_COURTYARD = 3

# ArUco tags (4x4, 100 mm) printed on the playmat, absolute coordinates.
TABLE_ARUCO: Dict[int, Tuple[float, float]] = {
    20: (600.0, 1400.0),
    21: (2400.0, 1400.0),
    22: (600.0, 600.0),
    23: (2400.0, 600.0),
}

# --- Scoring (official game rules 1.0) --------------------------------------
PTS_STONE = 1             # per stone validly placed in a construction zone
PTS_WALL = 5              # 3 stones lying stacked, in a wall zone
PTS_TOWER = 5             # 1 standing stone, in a tower circle
PTS_GATE = 20             # 2 standing + 1 lying on top, one per castle
PTS_GRAIL_IN_CASTLE = 5
PTS_GRAIL_PER_LEVEL = 5   # level 4 = on the robot roof, > 300 mm
PTS_HOME_PARTIAL = 5      # part of the robot projection in the throne room
PTS_HOME_FULL = 10        # the whole projection in the throne room
PTS_MOAT = 5              # per opponent moat occupied by one of our PAMI
PTS_PAMI_ATTACK = 5       # once
PTS_BALL_ANY = 5          # at least 1 ball in the opponent courtyard
PTS_BALL_3_BONUS = 15     # at least 3 balls


# ==============================================================================
# Team colour and mirroring
# ==============================================================================

class TeamColor(Enum):
    BLUE = "BLUE"
    YELLOW = "YELLOW"


TeamLike = Union[TeamColor, str]


def team_color(team: TeamLike) -> TeamColor:
    """Return the TeamColor for a TeamColor or a "BLUE" / "YELLOW" string.

    Raises:
        ValueError: unknown colour. No silent default: a robot playing the
            wrong side scores nothing.
    """
    if isinstance(team, TeamColor):
        return team
    try:
        return TeamColor[str(team).upper()]
    except KeyError:
        raise ValueError(f"Unknown team colour: {team!r}") from None


def _wrap_angle(angle: float) -> float:
    """Wrap an angle into (-pi, pi]."""
    wrapped = math.atan2(math.sin(angle), math.cos(angle))
    return math.pi if wrapped == -math.pi else wrapped


def _mirror_axis(angle: float) -> float:
    """Mirror the direction of an undirected axis, result in [0, pi)."""
    return (math.pi - angle) % math.pi


def mirror_for_team(pose: Sequence[float], team: TeamLike) -> Tuple[float, ...]:
    """Turn a YELLOW-reference point or pose into table coordinates for `team`.

    Args:
        pose: (x, y) or (x, y, theta), written for YELLOW.
        team: team colour.

    Returns:
        The same tuple for YELLOW; for BLUE, x -> 3000 - x, y kept and
        theta -> pi - theta (wrapped into (-pi, pi]).
    """
    if team_color(team) is TeamColor.YELLOW:
        return tuple(float(v) for v in pose)
    if len(pose) == 2:
        x, y = pose
        return (FIELD_WIDTH_MM - float(x), float(y))
    x, y, theta = pose
    return (FIELD_WIDTH_MM - float(x), float(y), _wrap_angle(math.pi - float(theta)))


# ==============================================================================
# Typed zones
# ==============================================================================

@dataclass(frozen=True)
class Obstacle:
    """Axis-aligned rectangle, lower-left corner, as nav/pathfinder.py reads it."""
    name: str
    x: float
    y: float
    width: float
    height: float


@dataclass(frozen=True)
class RectZone:
    """Oriented rectangle.

    `angle` is the direction of the LONG side, measured from +X,
    counter-clockwise, in [0, pi): the rectangle has no front.
    """
    name: str
    cx: float
    cy: float
    length: float
    width: float
    angle: float = 0.0

    def corners(self) -> List[Tuple[float, float]]:
        """Return the 4 corners, counter-clockwise."""
        ux, uy = math.cos(self.angle), math.sin(self.angle)
        hl, hw = self.length / 2.0, self.width / 2.0
        # Rounded to the micrometre: cos(pi/2) is not exactly 0.
        return [
            (round(self.cx + sl * hl * ux - sw * hw * uy, 3),
             round(self.cy + sl * hl * uy + sw * hw * ux, 3))
            for sl, sw in ((-1, -1), (1, -1), (1, 1), (-1, 1))
        ]

    def contains(self, x: float, y: float) -> bool:
        """True if (x, y) is inside the rectangle, edges included."""
        dx, dy = x - self.cx, y - self.cy
        along = dx * math.cos(self.angle) + dy * math.sin(self.angle)
        across = -dx * math.sin(self.angle) + dy * math.cos(self.angle)
        return abs(along) <= self.length / 2.0 and abs(across) <= self.width / 2.0

    def bounding_box(self) -> Obstacle:
        """Axis-aligned bounding box, in the pathfinder's obstacle format."""
        xs, ys = zip(*self.corners())
        return Obstacle(self.name, min(xs), min(ys), max(xs) - min(xs), max(ys) - min(ys))

    def mirrored(self) -> "RectZone":
        """Mirror across x = 1500."""
        return RectZone(self.name, FIELD_WIDTH_MM - self.cx, self.cy,
                        self.length, self.width, _mirror_axis(self.angle))


@dataclass(frozen=True)
class DiscZone:
    """Circle, given by its diameter."""
    name: str
    cx: float
    cy: float
    diameter: float

    def contains(self, x: float, y: float) -> bool:
        """True if (x, y) is inside the circle, edge included."""
        return math.hypot(x - self.cx, y - self.cy) <= self.diameter / 2.0

    def mirrored(self) -> "DiscZone":
        """Mirror across x = 1500."""
        return DiscZone(self.name, FIELD_WIDTH_MM - self.cx, self.cy, self.diameter)


@dataclass(frozen=True)
class PolygonZone:
    """Simple polygon, vertices counter-clockwise."""
    name: str
    vertices: Tuple[Tuple[float, float], ...]

    def contains(self, x: float, y: float) -> bool:
        """True if (x, y) is inside the polygon (ray casting, edges not guaranteed)."""
        inside = False
        pts = self.vertices
        for i in range(len(pts)):
            (x1, y1), (x2, y2) = pts[i], pts[(i + 1) % len(pts)]
            if (y1 > y) != (y2 > y):
                x_cross = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
                if x < x_cross:
                    inside = not inside
        return inside

    def mirrored(self) -> "PolygonZone":
        """Mirror across x = 1500; order reversed to stay counter-clockwise."""
        return PolygonZone(self.name, tuple(
            (FIELD_WIDTH_MM - x, y) for x, y in reversed(self.vertices)))


@dataclass(frozen=True)
class Quarry:
    """Stack of stones lying flat at the start of the match.

    `axis` is the direction of the stones' long side (0 or pi/2). A full
    quarry is an obstacle (330 mm high), an empty one is flat floor: quarries
    are game state, owned by the strategy, never burnt into the static grid.
    """
    name: str
    cx: float
    cy: float
    axis: float
    stones: int = STONES_PER_QUARRY

    def footprint(self) -> RectZone:
        """Floor footprint of the stack."""
        return RectZone(self.name, self.cx, self.cy,
                        STONE_LENGTH_MM, STONE_WIDTH_MM, self.axis)


@dataclass(frozen=True)
class Castle:
    """Every zone owned by one team, in absolute table coordinates.

    The moats listed are the ones OF this castle: the opponent's PAMI score in
    them, ours score in the opponent castle's moats.
    A gate (2 standing stones + 1 lying on top) may be built in ANY wall
    zone, one gate per castle: there is no gate zone.
    """
    team: TeamColor
    throne_room: RectZone          # start AND end zone of the robot
    stables: Tuple[RectZone, ...]  # PAMI start zones
    courtyard: PolygonZone         # passes through the tower centres
    walls: Dict[str, RectZone] = field(default_factory=dict)
    towers: Dict[str, DiscZone] = field(default_factory=dict)
    moats: Dict[str, RectZone] = field(default_factory=dict)

    def mirrored(self, team: TeamColor) -> "Castle":
        """Mirror across x = 1500 and give the result to `team`."""
        return Castle(
            team=team,
            throne_room=self.throne_room.mirrored(),
            stables=tuple(s.mirrored() for s in self.stables),
            courtyard=self.courtyard.mirrored(),
            walls={k: z.mirrored() for k, z in self.walls.items()},
            towers={k: z.mirrored() for k, z in self.towers.items()},
            moats={k: z.mirrored() for k, z in self.moats.items()},
        )


# ==============================================================================
# 2027 layout (YELLOW reference for the castle, absolute for the rest)
# ==============================================================================

WALL_LENGTH_MM, WALL_WIDTH_MM = 400, 200
MOAT_LENGTH_MM, MOAT_WIDTH_MM = 400, 200
TOWER_DIAMETER_MM = 200
STABLE_LENGTH_MM, STABLE_WIDTH_MM = 300, 200
THRONE_ROOM_SIZE_MM = 500

_QUARTER = math.pi / 4

YELLOW_CASTLE = Castle(
    team=TeamColor.YELLOW,
    # Against the left border, centred on y = 1000: x 0-500, y 750-1250.
    throne_room=RectZone("throne_room", 250, 1000,
                         THRONE_ROOM_SIZE_MM, THRONE_ROOM_SIZE_MM, 0.0),
    stables=(
        RectZone("stable_front", 100, 150,
                 STABLE_LENGTH_MM, STABLE_WIDTH_MM, math.pi / 2),
        RectZone("stable_back", 100, 1850,
                 STABLE_LENGTH_MM, STABLE_WIDTH_MM, math.pi / 2),
    ),
    # Measured on the coloured background of the plan, +/-10 mm.
    courtyard=PolygonZone("courtyard", (
        (0, 400), (500, 400), (850, 750), (850, 1250), (500, 1600), (0, 1600),
    )),
    walls={
        "M1": RectZone("M1", 250, 400, WALL_LENGTH_MM, WALL_WIDTH_MM, 0.0),
        "M2": RectZone("M2", 680, 570, WALL_LENGTH_MM, WALL_WIDTH_MM, _QUARTER),
        "M3": RectZone("M3", 850, 1000, WALL_LENGTH_MM, WALL_WIDTH_MM, math.pi / 2),
        "M4": RectZone("M4", 680, 1430, WALL_LENGTH_MM, WALL_WIDTH_MM, 3 * _QUARTER),
        "M5": RectZone("M5", 250, 1600, WALL_LENGTH_MM, WALL_WIDTH_MM, 0.0),
    },
    towers={
        "T1": DiscZone("T1", 500, 400, TOWER_DIAMETER_MM),
        "T2": DiscZone("T2", 850, 750, TOWER_DIAMETER_MM),
        "T3": DiscZone("T3", 850, 1250, TOWER_DIAMETER_MM),
        "T4": DiscZone("T4", 500, 1600, TOWER_DIAMETER_MM),
    },
    moats={
        "D1": RectZone("D1", 820, 430, MOAT_LENGTH_MM, MOAT_WIDTH_MM, _QUARTER),
        "D2": RectZone("D2", 1050, 1000, MOAT_LENGTH_MM, MOAT_WIDTH_MM, math.pi / 2),
        "D3": RectZone("D3", 820, 1570, MOAT_LENGTH_MM, MOAT_WIDTH_MM, 3 * _QUARTER),
    },
)

# Shared by both teams, absolute coordinates, never mirrored. The 8 border
# quarries touch the border: they can only be reached from one long side.
QUARRIES: Tuple[Quarry, ...] = (
    Quarry("C1", 750, 55, 0.0),
    Quarry("C2", 1250, 55, 0.0),
    Quarry("C3", 1750, 55, 0.0),
    Quarry("C4", 2250, 55, 0.0),
    Quarry("C5", 1500, 500, math.pi / 2),
    Quarry("C6", 1500, 1500, math.pi / 2),
    Quarry("C7", 750, 1945, 0.0),
    Quarry("C8", 1250, 1945, 0.0),
    Quarry("C9", 1750, 1945, 0.0),
    Quarry("C10", 2250, 1945, 0.0),
)

# Robot start pose for YELLOW: centre of the throne room. The whole vertical
# projection of the robot must fit in the 500 x 500 mm room at the end of the
# preparation, together with the Grail.
# Heading: facing the centre of the table. To revisit once the mechanics
# decide which face carries the actuators.
START_POSE: Tuple[float, float, float] = (250.0, 1000.0, 0.0)

# Nothing on the 2027 table blocks the robot for the whole match: castle
# zones are paint on the playmat, quarries are game state (see Quarry), and
# constructions only exist once the strategy has built them.
FIXED_OBSTACLES: Tuple[Obstacle, ...] = ()


class Terrain:
    """Neutral view of the table: both castles, absolute coordinates.

    Knows no team colour of its own. The strategy asks for a castle by
    colour, and applies mirror_for_team() to its own targets.
    """

    def __init__(self):
        self.WIDTH = FIELD_WIDTH_MM
        self.HEIGHT = FIELD_HEIGHT_MM
        self.ROBOT_RADIUS = ROBOT_RADIUS_MM
        self.castles: Dict[TeamColor, Castle] = {
            TeamColor.YELLOW: YELLOW_CASTLE,
            TeamColor.BLUE: YELLOW_CASTLE.mirrored(TeamColor.BLUE),
        }
        self.quarries: Tuple[Quarry, ...] = QUARRIES
        self.table_aruco: Dict[int, Tuple[float, float]] = dict(TABLE_ARUCO)
        # Burnt once into the pathfinder's static grid.
        self.obstacles: List[Obstacle] = list(FIXED_OBSTACLES)

    def castle(self, team: TeamLike) -> Castle:
        """Return the castle of `team`."""
        return self.castles[team_color(team)]

    def opponent_castle(self, team: TeamLike) -> Castle:
        """Return the castle of the team playing against `team`."""
        own = team_color(team)
        other = TeamColor.BLUE if own is TeamColor.YELLOW else TeamColor.YELLOW
        return self.castles[other]

    # No get_static_obstacles() here: fixed obstacles are burnt once and for
    # all into PathFinder._build_static_grid(). Feeding them again to
    # create_dynamic_grid() would inflate them twice. The method existed,
    # returned a hardcoded [] and had no caller: removed on 11/09/2026.


# ==============================================================================
# BALISES — Source unique pour positions (référentiel principal du plateau) ====
# ==============================================================================

class BeaconLayout:
    """
    Positions fixes des balises de localisation (calibré pour le terrain réel 3000x2000).
    Référentiel BLEU — symétrie appliquée automatiquement en JAUNE.
    """
    BEACON_SIZE_MM = 100.0

    # Beacon center positions (x, y) in mm for the BLUE team, CDR 2026 layout.
    # 2027 plan: the fixed beacon supports stand outside the short sides
    # (x < 0 and x > 3000), around y = 60, 1000 and 1940. Only their tips are
    # drawn, so these values are approximate, and which supports belong to
    # which team is not stated.
    BEACONS = {
        1: (3050.0, 1950.0),    # haut droite (hors terrain)
        2: (3050.0,   50.0),    # bas droite  (hors terrain)
        3: (  -50.0, 1000.0),   # milieu gauche (hors terrain)
    }

    @classmethod
    def get_beacon(cls, beacon_id: int, team_color: str = "BLUE") -> tuple:
        """Retourne les coordonnées d'une balise après symétrie X si nécessaire."""
        if beacon_id not in cls.BEACONS:
            raise ValueError(f"Balise {beacon_id} non trouvée")

        x, y = cls.BEACONS[beacon_id]

        if team_color.upper() == "YELLOW":
            x = FIELD_WIDTH_MM - x   # symétrie X (gauche <-> droite)

        return (x, y)
