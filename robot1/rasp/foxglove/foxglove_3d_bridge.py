#!/usr/bin/env python3
"""
Pont Foxglove 3D - Visualisation robot holonomique en temps réel.

Publie 3 channels :
  - robot/pose     : JSON simple pour les panels Plot (x, y, theta en mm/rad)
  - robot/tf       : Transform world→robot (pour le 3D panel, en mètres)
  - robot/scene    : SceneUpdate avec cylindre représentant le robot (en mètres)
    - robot/map_scene: SceneUpdate avec la carte CDR statique (table + bords)

Compatible simulation Webots et hardware réel via init_robot().
"""

from __future__ import annotations

import sys
from pathlib import Path

# Ajouter les chemins AVANT les imports
sys.path.insert(0, str(Path(__file__).parent))                 # foxglove/
sys.path.insert(0, str(Path(__file__).parent.parent))          # rasp/
sys.path.insert(0, str(Path(__file__).parent.parent.parent))   # robot1/

import asyncio
import inspect
import json
import logging
import math
import signal
import struct
import time
import base64
from typing import Any

from loader import loader
from foxglove_websocket.server import FoxgloveServer
from utils import init_robot

try:
    from terrain_glb_loader import load_terrain_glb
except ImportError:
    load_terrain_glb = None

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)s | %(message)s",
)
logger = logging.getLogger("foxglove_3d_bridge")

Messages = loader.load_class("usb_com", "Messages")

# ─────────────────────────────────────────────
# Charger le GLB UNE FOIS au démarrage
# ─────────────────────────────────────────────
GLB_DATA_B64 = ""
GLB_PATH = Path(__file__).parent / "map_assets" / "terrain.glb"
if GLB_PATH.exists():
    try:
        import base64
        GLB_DATA_B64 = base64.b64encode(GLB_PATH.read_bytes()).decode("ascii")
        logger.info("✅ GLB chargé au démarrage: %d bytes", GLB_PATH.stat().st_size)
    except Exception as e:
        logger.error("❌ Erreur chargement GLB: %s", e)
else:
    logger.warning("⚠️  GLB non trouvé: %s", GLB_PATH)

# ─────────────────────────────────────────────
# Paramètres visuels du robot (en mètres)
# ─────────────────────────────────────────────
ROBOT_DIAMETER_M = 0.35       # 350 mm diamètre
ROBOT_HEIGHT_M   = 0.10       # 100 mm hauteur
ROBOT_COLOR      = {"r": 0.2, "g": 0.6, "b": 1.0, "a": 0.85}   # bleu
DIRECTION_COLOR  = {"r": 1.0, "g": 0.3, "b": 0.1, "a": 0.9}    # rouge-orange

# ─────────────────────────────────────────────
# Paramètres carte CDR (en mètres)
# ─────────────────────────────────────────────
TABLE_LENGTH_M   = 3.0
TABLE_WIDTH_M    = 2.0
SMALL_WALL_SIZE  = {"x": 0.022, "y": 2.044, "z": 0.07}
BIG_WALL_SIZE    = {"x": 3.0,   "y": 0.022, "z": 0.07}

TABLE_COLOR      = {"r": 0.16, "g": 0.42, "b": 0.18, "a": 0.22}
WALL_COLOR       = {"r": 0.66, "g": 0.66, "b": 0.66, "a": 0.65}
AXIS_COLOR       = {"r": 1.0, "g": 1.0, "b": 0.0, "a": 0.75}
ATTIC_COLOR      = {"r": 0.35, "g": 0.30, "b": 0.20, "a": 0.50}

ZONE_YELLOW_COLOR = {"r": 0.969, "g": 0.71,  "b": 0.0,   "a": 0.70}
ZONE_BLUE_COLOR   = {"r": 0.0,   "g": 0.357, "b": 0.549, "a": 0.70}

SUPPORT_YELLOW_COLOR = {"r": 0.969, "g": 0.71,  "b": 0.0,   "a": 0.85}
SUPPORT_BLUE_COLOR   = {"r": 0.0,   "g": 0.357, "b": 0.549, "a": 0.85}

CRATE_YELLOW_COLOR = {"r": 0.95, "g": 0.78, "b": 0.18, "a": 0.95}
CRATE_BLUE_COLOR   = {"r": 0.20, "g": 0.50, "b": 0.95, "a": 0.95}
CRATE_EMPTY_COLOR  = {"r": 0.05, "g": 0.05, "b": 0.05, "a": 0.95}

CRATE_SIZE = {"x": 0.05, "y": 0.15, "z": 0.03}
SUPPORT_SIZE = {"x": 0.122, "y": 0.082, "z": 0.14}
CALC_ZONE_SIZE = {"x": 0.45, "y": 0.20, "z": 0.022}

# Positions dérivées de map_assets/eurobot2026/table/Eurobot2026.proto
CALC_ZONES = [
    {"id": "calc_zone_yellow", "translation": (-0.225, 1.122, 0.048), "color": ZONE_YELLOW_COLOR},
    {"id": "calc_zone_blue",   "translation": (0.225,  1.122, 0.048), "color": ZONE_BLUE_COLOR},
]

BEACON_SUPPORTS = [
    {"id": "support_yellow1", "translation": (-1.594, 0.952, 0.07), "yaw": 0.0,     "color": SUPPORT_YELLOW_COLOR},
    {"id": "support_blue1",   "translation": (-1.594, 0.000, 0.07), "yaw": 0.0,     "color": SUPPORT_BLUE_COLOR},
    {"id": "support_yellow2", "translation": (-1.594,-0.952, 0.07), "yaw": 0.0,     "color": SUPPORT_YELLOW_COLOR},
    {"id": "support_blue2",   "translation": ( 1.594, 0.952, 0.07), "yaw": math.pi, "color": SUPPORT_BLUE_COLOR},
    {"id": "support_yellow3", "translation": ( 1.594, 0.000, 0.07), "yaw": math.pi, "color": SUPPORT_YELLOW_COLOR},
    {"id": "support_blue3",   "translation": ( 1.594,-0.952, 0.07), "yaw": math.pi, "color": SUPPORT_BLUE_COLOR},
]

BLUE_FIXED_BEACONS = [
    (-1.594, 0.000, 0.51),
    ( 1.594, 0.952, 0.51),
    ( 1.594,-0.952, 0.51),
]

STACK_OFFSETS_X = {
    "CrateGroup":      [-0.0752, -0.0251, 0.0251, 0.0752],
    "CrateGroup2":     [-0.0251, 0.0251],
    "EmptyCrateGroup": [-0.0501, 0.0000, 0.0501],
}

STACKS = [
    # Zone jaune
    {"id": "stack1",     "kind": "CrateGroup",      "translation": (-0.350, -0.200, 0.000), "yaw": 0.0,            "color": CRATE_YELLOW_COLOR},
    {"id": "stack2",     "kind": "CrateGroup",      "translation": (-0.400, -0.800, 0.000), "yaw": 0.0,            "color": CRATE_YELLOW_COLOR},
    {"id": "stack3",     "kind": "CrateGroup",      "translation": (-1.325,  0.200, 0.000), "yaw": 1.5707963268,   "color": CRATE_YELLOW_COLOR},
    {"id": "stack4",     "kind": "CrateGroup",      "translation": (-1.325, -0.600, 0.000), "yaw": 1.5707963268,   "color": CRATE_YELLOW_COLOR},
    {"id": "stack5",     "kind": "CrateGroup",      "translation": (-0.700,  0.675, 0.055), "yaw": 0.0,            "color": CRATE_YELLOW_COLOR},
    {"id": "stack6",     "kind": "CrateGroup2",     "translation": (-0.400,  0.725, 0.055), "yaw": 0.0,            "color": CRATE_YELLOW_COLOR},
    {"id": "stack7",     "kind": "CrateGroup2",     "translation": (-0.150,  0.775, 0.055), "yaw": 0.0,            "color": CRATE_YELLOW_COLOR},
    {"id": "stack8",     "kind": "EmptyCrateGroup", "translation": (-0.700,  0.675, 0.085), "yaw": 0.0,            "color": CRATE_EMPTY_COLOR},

    # Zone bleue
    {"id": "stack1_blue", "kind": "CrateGroup",      "translation": ( 0.350, -0.200, 0.000), "yaw": 0.0,           "color": CRATE_BLUE_COLOR},
    {"id": "stack2_blue", "kind": "CrateGroup",      "translation": ( 0.400, -0.800, 0.000), "yaw": 0.0,           "color": CRATE_BLUE_COLOR},
    {"id": "stack3_blue", "kind": "CrateGroup",      "translation": ( 1.325,  0.200, 0.000), "yaw": 1.5707963268,  "color": CRATE_BLUE_COLOR},
    {"id": "stack4_blue", "kind": "CrateGroup",      "translation": ( 1.325, -0.600, 0.000), "yaw": 1.5707963268,  "color": CRATE_BLUE_COLOR},
    {"id": "stack5_blue", "kind": "CrateGroup",      "translation": ( 0.700,  0.675, 0.055), "yaw": 0.0,           "color": CRATE_BLUE_COLOR},
    {"id": "stack6_blue", "kind": "CrateGroup2",     "translation": ( 0.400,  0.725, 0.055), "yaw": 0.0,           "color": CRATE_BLUE_COLOR},
    {"id": "stack7_blue", "kind": "CrateGroup2",     "translation": ( 0.150,  0.775, 0.055), "yaw": 0.0,           "color": CRATE_BLUE_COLOR},
    {"id": "stack8_blue", "kind": "EmptyCrateGroup", "translation": ( 0.700,  0.675, 0.085), "yaw": 0.0,           "color": CRATE_EMPTY_COLOR},
]

# ─────────────────────────────────────────────
# Utilitaires
# ─────────────────────────────────────────────

def theta_to_quaternion(theta: float) -> dict:
    """Convertit un angle yaw (rad) en quaternion (rotation autour de Z)."""
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(theta / 2.0),
        "w": math.cos(theta / 2.0),
    }


def now_ns() -> int:
    return time.time_ns()


async def maybe_await(value: Any) -> Any:
    if inspect.isawaitable(value):
        return await value
    return value


async def send_msg(server: Any, channel_id: int, payload: bytes) -> None:
    """Envoi compatible avec toutes les versions de foxglove-websocket."""
    ns = now_ns()
    n = len(inspect.signature(server.send_message).parameters)
    if n >= 4:
        result = server.send_message(channel_id, ns, ns, payload)
    elif n == 3:
        result = server.send_message(channel_id, ns, payload)
    else:
        result = server.send_message(channel_id, payload)
    await maybe_await(result)


# ─────────────────────────────────────────────
# Schemas JSON (foxglove)
# ─────────────────────────────────────────────

POSE_SCHEMA = json.dumps({
    "type": "object",
    "properties": {
        "x":     {"type": "number", "description": "Position X (mm)"},
        "y":     {"type": "number", "description": "Position Y (mm)"},
        "theta": {"type": "number", "description": "Orientation (rad)"},
    },
    "required": ["x", "y", "theta"],
})

# foxglove.FrameTransform : positionne le repère robot dans le repère monde
TF_SCHEMA = json.dumps({
    "type": "object",
    "title": "foxglove.FrameTransform",
    "properties": {
        "timestamp":        {"type": "object"},
        "parent_frame_id":  {"type": "string"},
        "child_frame_id":   {"type": "string"},
        "translation":      {"type": "object"},
        "rotation":         {"type": "object"},
    },
})

# foxglove.SceneUpdate : entités 3D (cylindre robot + flèche direction)
SCENE_SCHEMA = json.dumps({
    "type": "object",
    "title": "foxglove.SceneUpdate",
    "properties": {
        "deletions": {
            "type": "array",
            "items": {
                "type": "object",
                "additionalProperties": True,
            },
        },
        "entities": {
            "type": "array",
            "items": {
                "type": "object",
                "additionalProperties": True,
                "properties": {
                    "models": {
                        "type": "array",
                        "items": {
                            "type": "object",
                            "properties": {
                                "data": {
                                    "type": "string",
                                    "contentEncoding": "base64"
                                }
                            },
                            "additionalProperties": True
                        }
                    }
                }
            },
        },
    },
    "additionalProperties": True,
})


# ─────────────────────────────────────────────
# Schémas pour image plane
# ─────────────────────────────────────────────

COMPRESSED_IMAGE_SCHEMA = json.dumps({
    "type": "object",
    "properties": {
        "timestamp": {
            "type": "object",
            "properties": {
                "sec": {"type": "integer"},
                "nsec": {"type": "integer"},
            },
        },
        "frame_id": {"type": "string"},
        "format": {"type": "string"},
        "data": {
            "type": "string",
            "contentEncoding": "base64",
        },
    },
    "additionalProperties": True,
})

CAMERA_INFO_SCHEMA = json.dumps({
    "type": "object",
    "properties": {
        "timestamp": {
            "type": "object",
            "properties": {
                "sec": {"type": "integer"},
                "nsec": {"type": "integer"},
            },
        },
        "frame_id": {"type": "string"},
        "height": {"type": "integer"},
        "width": {"type": "integer"},
        "distortion_model": {"type": "string"},
        "D": {"type": "array", "items": {"type": "number"}},
        "K": {"type": "array", "items": {"type": "number"}},
        "P": {"type": "array", "items": {"type": "number"}},
    },
    "additionalProperties": True,
})


# ─────────────────────────────────────────────
# Construction des messages
# ─────────────────────────────────────────────

def build_pose_msg(x: float, y: float, theta: float) -> bytes:
    """robot/pose — valeurs brutes en mm et rad pour les Plot panels."""
    return json.dumps({"x": x, "y": y, "theta": theta}).encode()


def build_tf_msg(x_m: float, y_m: float, theta: float) -> bytes:
    """
    foxglove.FrameTransform  world → robot
    Foxglove 3D panel utilise ce message pour placer les entités.
    """
    ns = now_ns()
    q = theta_to_quaternion(theta)
    msg = {
        "timestamp": {
            "sec":   ns // 1_000_000_000,
            "nsec":  ns %  1_000_000_000,
        },
        "parent_frame_id": "world",
        "child_frame_id":  "robot",
        "translation": {"x": x_m, "y": y_m, "z": 0.0},
        "rotation": q,
    }
    return json.dumps(msg).encode()


def build_scene_msg(x_m: float, y_m: float, theta: float) -> bytes:
    """
    foxglove.SceneUpdate avec :
      - un cylindre bleu pour le corps du robot
      - une flèche rouge indiquant la direction (avant du robot)
    Toutes les positions sont en mètres dans le repère 'world'.
    """
    ns = now_ns()
    ts = {"sec": ns // 1_000_000_000, "nsec": ns % 1_000_000_000}
    q  = theta_to_quaternion(theta)

    # Décalage de la flèche : pointe vers l'avant du robot
    arrow_len = ROBOT_DIAMETER_M * 0.7
    tip_x = x_m + arrow_len * math.cos(theta)
    tip_y = y_m + arrow_len * math.sin(theta)

    entity = {
        "timestamp": ts,
        "frame_id":  "world",
        "id":        "robot_body",
        "lifetime":  {"sec": 0, "nsec": 100_000_000},  # 100 ms TTL
        "frame_locked": False,
        "metadata":  [],

        # ── Corps : cylindre ──────────────────────────────────────────────
        "cylinders": [
            {
                "pose": {
                    "position":    {"x": x_m, "y": y_m, "z": ROBOT_HEIGHT_M / 2},
                    "orientation": q,
                },
                "size": {
                    "x": ROBOT_DIAMETER_M,
                    "y": ROBOT_DIAMETER_M,
                    "z": ROBOT_HEIGHT_M,
                },
                "color": ROBOT_COLOR,
            }
        ],

        # ── Direction : flèche ────────────────────────────────────────────
        "arrows": [
            {
                "pose": {
                    "position":    {"x": x_m,   "y": y_m,   "z": ROBOT_HEIGHT_M},
                    "orientation": q,
                },
                "shaft_length": arrow_len,
                "shaft_diameter": 0.02,
                "head_diameter":  0.06,
                "head_length":    0.06,
                "color": DIRECTION_COLOR,
            }
        ],

        # ── Trace : point de trajectoire ─────────────────────────────────
        "points": [],
        "texts":  [],
        "lines":  [],
        "triangles":    [],
        "spheres":      [],
        "models":       [],
    }

    msg = {"deletions": [], "entities": [entity]}
    return json.dumps(msg).encode()


def build_entity(
    entity_id: str,
    ts: dict,
    *,
    cubes: list[dict] | None = None,
    cylinders: list[dict] | None = None,
    arrows: list[dict] | None = None,
) -> dict:
    """Construit une entité foxglove.SceneEntity avec les champs attendus."""
    return {
        "timestamp": ts,
        "frame_id": "world",
        "id": entity_id,
        "frame_locked": False,
        "metadata": [],
        "cubes": cubes or [],
        "cylinders": cylinders or [],
        "arrows": arrows or [],
        "points": [],
        "texts": [],
        "lines": [],
        "triangles": [],
        "spheres": [],
        "models": [],
    }


def rotate_xy(x_local: float, y_local: float, yaw: float) -> tuple[float, float]:
    """Rotation 2D dans le plan XY."""
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (x_local * c - y_local * s, x_local * s + y_local * c)


def build_stack_cubes() -> list[dict]:
    """Construit les caisses de jeu à partir du layout Eurobot2026."""
    cubes: list[dict] = []
    for stack in STACKS:
        offsets = STACK_OFFSETS_X[stack["kind"]]
        tx, ty, tz = stack["translation"]
        yaw = stack["yaw"]
        q = theta_to_quaternion(yaw)

        for lx in offsets:
            ox, oy = rotate_xy(lx, 0.0, yaw)
            cubes.append(
                {
                    "pose": {
                        "position": {
                            "x": tx + ox,
                            "y": ty + oy,
                            "z": tz + 0.015,
                        },
                        "orientation": q,
                    },
                    "size": CRATE_SIZE,
                    "color": stack["color"],
                }
            )
    return cubes


def build_map_scene_msg() -> bytes:
    """Carte CDR statique basée sur Eurobot2026.proto dans le repère world.
    
    Intègre un modèle GLB du terrain pour meilleure performance et vision 3D.
    """
    ns = now_ns()
    ts = {"sec": ns // 1_000_000_000, "nsec": ns % 1_000_000_000}

    table_cubes = [
        {
            "pose": {
                "position": {"x": 0.0, "y": 0.0, "z": 0.001},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
            "size": {"x": TABLE_LENGTH_M, "y": TABLE_WIDTH_M, "z": 0.002},
            "color": TABLE_COLOR,
        },
        {
            "pose": {
                "position": {"x": 0.0, "y": 0.775, "z": 0.0275},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
            "size": {"x": 1.8, "y": 0.45, "z": 0.055},
            "color": ATTIC_COLOR,
        },
        {
            "pose": {
                "position": {"x": 1.511, "y": 0.0, "z": 0.035},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
            "size": SMALL_WALL_SIZE,
            "color": WALL_COLOR,
        },
        {
            "pose": {
                "position": {"x": -1.511, "y": 0.0, "z": 0.035},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
            "size": SMALL_WALL_SIZE,
            "color": WALL_COLOR,
        },
        {
            "pose": {
                "position": {"x": 0.0, "y": 1.011, "z": 0.035},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
            "size": BIG_WALL_SIZE,
            "color": WALL_COLOR,
        },
        {
            "pose": {
                "position": {"x": 0.0, "y": -1.011, "z": 0.035},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
            "size": BIG_WALL_SIZE,
            "color": WALL_COLOR,
        },
    ]

    calc_zone_cubes = []
    for zone in CALC_ZONES:
        zx, zy, zz = zone["translation"]
        calc_zone_cubes.append(
            {
                "pose": {
                    "position": {"x": zx, "y": zy, "z": zz},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
                "size": CALC_ZONE_SIZE,
                "color": zone["color"],
            }
        )

    support_cubes = []
    for support in BEACON_SUPPORTS:
        sx, sy, sz = support["translation"]
        support_cubes.append(
            {
                "pose": {
                    "position": {"x": sx, "y": sy, "z": sz},
                    "orientation": theta_to_quaternion(support["yaw"]),
                },
                "size": SUPPORT_SIZE,
                "color": support["color"],
            }
        )

    beacon_cylinders = []
    for bx, by, bz in BLUE_FIXED_BEACONS:
        beacon_cylinders.append(
            {
                "pose": {
                    "position": {"x": bx, "y": by, "z": bz},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
                "size": {"x": 0.1, "y": 0.1, "z": 1.0},
                "color": {"r": 1.0, "g": 1.0, "b": 1.0, "a": 0.9},
            }
        )

    world_axis = [
        {
            "pose": {
                "position": {"x": 0.0, "y": 0.0, "z": 0.02},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
            "shaft_length": 0.35,
            "shaft_diameter": 0.01,
            "head_diameter": 0.03,
            "head_length": 0.04,
            "color": AXIS_COLOR,
        }
    ]

    entities = []

    # ────────────────────────────────────────────────────────────────────
    # Ajouter le terrain GLB si disponible (couche de base)
    # ────────────────────────────────────────────────────────────────────
    if GLB_DATA_B64:
        glb_model = {
            "pose": {
                "position": {"x": 0.0, "y": 0.0, "z": -0.001},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
            "scale": {"x": 1.0, "y": 1.0, "z": 1.0},
            "color": {"r": 1.0, "g": 1.0, "b": 1.0, "a": 1.0},
            "override_color": False,
            "media_type": "model/gltf-binary",
            "url": "",
            "data": GLB_DATA_B64,
        }
        terrain_entity = build_entity("terrain_map", ts)
        terrain_entity["models"] = [glb_model]
        entities.append(terrain_entity)

    # ────────────────────────────────────────────────────────────────────
    # Autres entités (table, zones, supports, caisses, balises, axe)
    # ────────────────────────────────────────────────────────────────────
    entities.extend([
        build_entity("cdr_table", ts, cubes=table_cubes),
        build_entity("cdr_calc_zones", ts, cubes=calc_zone_cubes),
        build_entity("cdr_beacon_supports", ts, cubes=support_cubes),
        build_entity("cdr_fixed_beacons", ts, cylinders=beacon_cylinders),
        build_entity("cdr_stacks", ts, cubes=build_stack_cubes()),
        build_entity("cdr_world_axis", ts, arrows=world_axis),
    ])

    return json.dumps({
        "frame_id": "world",
        "deletions": [],
        "entities": entities
    }).encode()


def build_compressed_image_msg(image_data_b64: str) -> bytes:
    """Construire un message CompressedImage pour Foxglove (projection plane)."""
    ns = now_ns()
    msg = {
        "timestamp": {
            "sec": ns // 1_000_000_000,
            "nsec": ns % 1_000_000_000,
        },
        "frame_id": "world",
        "format": "jpeg",
        "data": image_data_b64,
    }
    return json.dumps(msg).encode()


def build_camera_info_msg(height: int, width: int) -> bytes:
    """Construire un message CameraInfo pour Foxglove (infos de caméra orthographique)."""
    ns = now_ns()
    # Matrice de calibration simple pour projection orthographique
    # Téléobjectif avec focal très grande pour approximer orthographique
    focal_length = 10000.0
    K = [focal_length, 0, width / 2.0, 0, focal_length, height / 2.0, 0, 0, 1]
    P = [focal_length, 0, width / 2.0, 0, 0, focal_length, height / 2.0, 0, 0, 0, 1, 0]
    R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
    
    msg = {
        "timestamp": {
            "sec": ns // 1_000_000_000,
            "nsec": ns % 1_000_000_000,
        },
        "frame_id": "world",
        "height": height,
        "width": width,
        "distortion_model": "plumb_bob",
        "D": [0, 0, 0, 0, 0],
        "K": K,
        "P": P,
        "R": R,
    }
    return json.dumps(msg).encode()


# ─────────────────────────────────────────────
# Bridge principal
# ─────────────────────────────────────────────

# Charger l'image du playmat une seule fois
PLAYMAT_IMAGE_B64 = ""
PLAYMAT_WIDTH = 3000  # pixels
PLAYMAT_HEIGHT = 2000  # pixels

PLAYMAT_PATH = Path(__file__).parent / "map_assets" / "eurobot2026" / "textures" / "playmat_2026.jpg"
if PLAYMAT_PATH.exists():
    try:
        playmat_data = PLAYMAT_PATH.read_bytes()
        PLAYMAT_IMAGE_B64 = base64.b64encode(playmat_data).decode("ascii")
        # Essayer d'extraire les vraies dimensions de l'image JPEG si possible
        from PIL import Image
        img = Image.open(PLAYMAT_PATH)
        PLAYMAT_WIDTH, PLAYMAT_HEIGHT = img.size
        logger.info(f"📸 Image playmat chargée : {PLAYMAT_WIDTH}x{PLAYMAT_HEIGHT}px ({len(playmat_data)/1024:.1f} KB)")
    except Exception as e:
        logger.warning(f"⚠️  Impossible de charger playmat_2026.jpg : {e}")


async def run_bridge() -> None:
    stop_event = asyncio.Event()
    loop = asyncio.get_running_loop()

    com, mode = init_robot(logger)
    logger.info("Bridge 3D démarré en mode %s", mode)

    async with FoxgloveServer("0.0.0.0", 8765, "robot-3d-bridge") as server:

        # Enregistrement des channels
        ch_pose = await maybe_await(server.add_channel({
            "topic":          "robot/pose",
            "encoding":       "json",
            "schemaName":     "robot.Pose2D",
            "schemaEncoding": "jsonschema",
            "schema":         POSE_SCHEMA,
        }))

        ch_tf = await maybe_await(server.add_channel({
            "topic":          "robot/tf",
            "encoding":       "json",
            "schemaName":     "foxglove.FrameTransform",
            "schemaEncoding": "jsonschema",
            "schema":         TF_SCHEMA,
        }))

        ch_scene = await maybe_await(server.add_channel({
            "topic":          "robot/scene",
            "encoding":       "json",
            "schemaName":     "foxglove.SceneUpdate",
            "schemaEncoding": "jsonschema",
            "schema":         SCENE_SCHEMA,
        }))

        ch_map_scene = await maybe_await(server.add_channel({
            "topic":          "robot/map_scene",
            "encoding":       "json",
            "schemaName":     "foxglove.SceneUpdate",
            "schemaEncoding": "jsonschema",
            "schema":         SCENE_SCHEMA,
        }))

        ch_map_image = await maybe_await(server.add_channel({
            "topic":          "robot/map_image/compressed",
            "encoding":       "json",
            "schemaName":     "sensor_msgs.CompressedImage",
            "schemaEncoding": "jsonschema",
            "schema":         COMPRESSED_IMAGE_SCHEMA,
        }))

        ch_camera_info = await maybe_await(server.add_channel({
            "topic":          "robot/map_image/camera_info",
            "encoding":       "json",
            "schemaName":     "sensor_msgs.CameraInfo",
            "schemaEncoding": "jsonschema",
            "schema":         CAMERA_INFO_SCHEMA,
        }))

        logger.info("Channels créés : robot/pose | robot/tf | robot/scene | robot/map_scene | robot/map_image")
        logger.info("Foxglove sur ws://localhost:8765")

        # Republie la carte statique pour les clients qui se reconnectent.
        async def map_publisher_task() -> None:
            map_payload = build_map_scene_msg()
            while not stop_event.is_set():
                await send_msg(server, ch_map_scene, map_payload)
                await asyncio.sleep(1.0)

        map_task = asyncio.create_task(map_publisher_task())

        # ──── Publication de l'image du playmat pour projection plane ────────
        async def image_publisher_task() -> None:
            if not PLAYMAT_IMAGE_B64:
                logger.warning("⚠️  Image playmat non disponible, publication ignorée")
                return
            while not stop_event.is_set():
                img_payload = build_compressed_image_msg(PLAYMAT_IMAGE_B64)
                await send_msg(server, ch_map_image, img_payload)
                await asyncio.sleep(1.0)

        async def camera_info_publisher_task() -> None:
            if not PLAYMAT_IMAGE_B64:
                return
            while not stop_event.is_set():
                cam_payload = build_camera_info_msg(PLAYMAT_HEIGHT, PLAYMAT_WIDTH)
                await send_msg(server, ch_camera_info, cam_payload)
                await asyncio.sleep(1.0)

        image_task = asyncio.create_task(image_publisher_task())
        camera_task = asyncio.create_task(camera_info_publisher_task())

        def on_rolling_basis(data: bytes) -> None:
            if len(data) < 24:
                logger.warning("Payload trop court : %d bytes", len(data))
                return

            x, y, theta = struct.unpack("<ddd", data[:24])

            # Conversion mm → m pour la 3D
            x_m = x / 1000.0
            y_m = y / 1000.0

            def _send():
                async def _async_send():
                    try:
                        await send_msg(server, ch_pose,  build_pose_msg(x, y, theta))
                        await send_msg(server, ch_tf,    build_tf_msg(x_m, y_m, theta))
                        await send_msg(server, ch_scene, build_scene_msg(x_m, y_m, theta))
                    except Exception as exc:
                        logger.error("Erreur envoi Foxglove : %s", exc)

                future = asyncio.run_coroutine_threadsafe(_async_send(), loop)
                future.add_done_callback(
                    lambda f: logger.error("Envoi échoué : %s", f.exception())
                    if f.exception() else None
                )

            _send()

        com.add_callback(on_rolling_basis, Messages.UPDATE_ROLLING_BASIS.value)
        logger.info("Callback enregistré sur UPDATE_ROLLING_BASIS (%d)",
                    Messages.UPDATE_ROLLING_BASIS.value)

        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                loop.add_signal_handler(sig, stop_event.set)
            except NotImplementedError:
                pass

        logger.info("Bridge actif. Ctrl+C pour arrêter.")
        try:
            await stop_event.wait()
        finally:
            map_task.cancel()
            await asyncio.gather(map_task, return_exceptions=True)
            if hasattr(com, "close"):
                com.close()
            logger.info("Connexion fermée.")


def main() -> None:
    try:
        asyncio.run(run_bridge())
    except KeyboardInterrupt:
        logger.info("Arrêt utilisateur.")


if __name__ == "__main__":
    main()
