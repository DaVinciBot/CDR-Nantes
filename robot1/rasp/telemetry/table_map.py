#!/usr/bin/env python3
"""
Carte Eurobot 2027 vers Rerun : seule source de la carte statique.

Publie le terrain statique (playmat, bordures, supports et balises, pierres
des carrieres). rerun_bridge.py et sim2d appellent
log_static_map() d'ici : la geometrie n'est ecrite nulle part ailleurs. Les
cotes de table, de bordure et de pierres sont lues dans
world.py ; seuls les supports de balises et les formes d'affichage sont
definis dans ce fichier.

Usage (depuis robot1/rasp/) :
    python -m telemetry.table_map --mode local --output recording.rrd
    python -m telemetry.table_map --mode serve --port 9876
"""

import argparse
import logging
import math
from pathlib import Path

import numpy as np
import rerun as rr

from robot1.rasp.world import (
    BORDER_HEIGHT_MM,
    BORDER_THICKNESS_MM,
    FIELD_HEIGHT_MM,
    FIELD_WIDTH_MM,
    QUARRIES,
    STONE_HEIGHT_MM,
    STONE_LENGTH_MM,
    STONE_WIDTH_MM,
    BeaconLayout,
)

logger = logging.getLogger("table_map")

PLAYMAT_PATH = Path(__file__).parent / "map_assets" / "playmat2027" / "Field.png"

# ─────────────────────────────────────────────────────────────────────────────
# Terrain (mm)
# ─────────────────────────────────────────────────────────────────────────────

W = float(FIELD_WIDTH_MM)
H = float(FIELD_HEIGHT_MM)
CX = W / 2
CY = H / 2

# Couleurs RGBA uint8
C_WALL    = [168, 168, 168, 230]
C_TABLE   = [ 41, 107,  46,  80]
C_BEACON  = [255, 255, 255, 240]
C_SUP_YEL = [247, 181,   0, 220]
C_SUP_BLU = [  0,  91, 140, 220]
C_STONE   = [196, 154, 108, 255]   # kraft cardboard

# Balises : positions de world.py (dict {id: (x, y)}), triees pour que
# l'entite beacon_<i> designe toujours la meme balise.
BEACONS_MM = sorted(BeaconLayout.BEACONS.values())
BEACON_R_MM = 50.0
BEACON_H_MM = 1020.0

# Supports des balises fixes, hors table sur les petits cotes.
SUPPORTS_MM = [
    {"pos": (  -94, 1952), "color": C_SUP_YEL},
    {"pos": (  -94, 1000), "color": C_SUP_BLU},
    {"pos": (  -94,   48), "color": C_SUP_YEL},
    {"pos": ( 3094, 1952), "color": C_SUP_BLU},
    {"pos": ( 3094, 1000), "color": C_SUP_YEL},
    {"pos": ( 3094,   48), "color": C_SUP_BLU},
]


def cylinder_mesh(cx: float, cy: float, z_bot: float,
                  radius: float, height: float, n: int = 20):
    """Return (vertices, triangles) of a closed vertical cylinder."""
    angles = np.linspace(0, 2 * math.pi, n, endpoint=False)
    ca, sa = np.cos(angles), np.sin(angles)
    z_top = z_bot + height

    bot = np.column_stack([cx + radius * ca, cy + radius * sa, np.full(n, z_bot)])
    top = np.column_stack([cx + radius * ca, cy + radius * sa, np.full(n, z_top)])
    cnt = np.array([[cx, cy, z_bot], [cx, cy, z_top]])
    verts = np.vstack([bot, top, cnt]).astype(np.float32)

    bc, tc = 2 * n, 2 * n + 1
    tris = []
    for i in range(n):
        j = (i + 1) % n
        tris += [[i, j, n + j], [i, n + j, n + i]]
        tris.append([bc, j, i])
        tris.append([tc, n + i, n + j])

    return verts, np.array(tris, dtype=np.uint32)


# ─────────────────────────────────────────────────────────────────────────────
# Publicateurs Rerun
# ─────────────────────────────────────────────────────────────────────────────

def _log_playmat() -> None:
    """Textured playmat, or a plain green table if the texture is missing."""
    try:
        if not PLAYMAT_PATH.exists():
            raise FileNotFoundError(PLAYMAT_PATH)
        from PIL import Image as PILImage
        img = PILImage.open(PLAYMAT_PATH).convert("RGB")
        rr.log("world/map/playmat", rr.Mesh3D(
            vertex_positions=np.array(
                [[0., 0., 0.], [W, 0., 0.], [W, H, 0.], [0., H, 0.]], dtype=np.float32),
            triangle_indices=np.array([[0, 1, 2], [0, 2, 3]], dtype=np.uint32),
            vertex_texcoords=np.array(
                [[0., 1.], [1., 1.], [1., 0.], [0., 0.]], dtype=np.float32),
            albedo_texture=np.array(img, dtype=np.uint8),
        ), static=True)
        logger.info("Playmat %s (%dx%d)", PLAYMAT_PATH.name, img.width, img.height)
    except Exception as e:
        logger.warning("Playmat indisponible (%s) -> table verte unie", e)
        rr.log("world/map/table", rr.Boxes3D(
            centers=[[CX, CY, 1]], half_sizes=[[W / 2, H / 2, 1]], colors=[C_TABLE],
        ), static=True)


def _log_borders() -> None:
    """The 4 borders, outside the playing area; the side ones cover the corners."""
    t, hz = BORDER_THICKNESS_MM / 2.0, BORDER_HEIGHT_MM / 2.0
    centers = [
        [-t, CY, hz], [W + t, CY, hz],     # left, right
        [CX, H + t, hz], [CX, -t, hz],     # back, front
    ]
    halves = [
        [t, H / 2 + 2 * t, hz], [t, H / 2 + 2 * t, hz],
        [W / 2, t, hz], [W / 2, t, hz],
    ]
    rr.log("world/map/walls", rr.Boxes3D(
        centers=centers, half_sizes=halves, colors=[C_WALL] * len(centers),
    ), static=True)


def _log_beacons() -> None:
    """Beacon supports (base + post) and the fixed beacons themselves."""
    sup_c, sup_h, sup_col = [], [], []
    for sup in SUPPORTS_MM:
        sx, sy = sup["pos"]
        col = sup["color"]
        sup_c.append([sx, sy, 15]);       sup_h.append([61, 41, 15]);  sup_col.append(col)
        ox = 35 if sx < CX else -35
        sup_c.append([sx + ox, sy, 90]);  sup_h.append([20, 35, 75]);  sup_col.append(col)

    rr.log("world/map/beacon_supports", rr.Boxes3D(
        centers=np.array(sup_c, dtype=np.float32),
        half_sizes=np.array(sup_h, dtype=np.float32),
        colors=np.array(sup_col, dtype=np.uint8),
    ), static=True)

    for i, (bx, by) in enumerate(BEACONS_MM):
        verts, tris = cylinder_mesh(bx, by, 0.0, BEACON_R_MM, BEACON_H_MM)
        rr.log(f"world/map/beacon_{i}", rr.Mesh3D(
            vertex_positions=verts,
            triangle_indices=tris,
            vertex_colors=np.tile(C_BEACON, (len(verts), 1)).astype(np.uint8),
        ), static=True)


def log_stones() -> int:
    """Publish the stones stacked in the 10 quarries, as solid 3D boxes.

    Logged as static: this is the start-of-match layout. Once the game state
    tracks the stones, this entity has to become time-varying.

    Returns:
        Number of stones published.
    """
    centers, halves = [], []
    for quarry in QUARRIES:
        along_x = abs(math.cos(quarry.axis)) > 0.5
        half_x = (STONE_LENGTH_MM if along_x else STONE_WIDTH_MM) / 2.0
        half_y = (STONE_WIDTH_MM if along_x else STONE_LENGTH_MM) / 2.0
        for level in range(quarry.stones):
            centers.append([quarry.cx, quarry.cy, STONE_HEIGHT_MM * (level + 0.5)])
            halves.append([half_x, half_y, STONE_HEIGHT_MM / 2.0])

    rr.log("world/map/stones", rr.Boxes3D(
        centers=np.array(centers, dtype=np.float32),
        half_sizes=np.array(halves, dtype=np.float32),
        colors=[C_STONE] * len(centers),
        fill_mode="solid",
    ), static=True)
    return len(centers)


def log_static_map() -> None:
    """Publish every fixed object of the table (static=True)."""
    _log_playmat()
    _log_borders()
    _log_beacons()
    n_stones = log_stones()
    logger.info(
        "Carte statique publiée — %d balises | %d supports | %d pierres",
        len(BEACONS_MM), len(SUPPORTS_MM), n_stones,
    )


def create_blueprint():
    """Blueprint pour visualiser la map."""
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Spatial3DView(name="Terrain Eurobot 2027", origin="world"),
    )


def main():
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s | %(levelname)s | %(message)s")
    p = argparse.ArgumentParser(description="Carte Eurobot 2027 vers Rerun")
    p.add_argument("--mode", choices=["local", "serve"], default="local")
    p.add_argument("--port", type=int, default=9876)
    p.add_argument("--output", help="Enregistrer en .rrd")
    args = p.parse_args()

    rr.init("table_map", spawn=(args.mode == "local"))

    if args.mode == "serve":
        rr.serve_web(open_browser=False, web_port=args.port)
        print(f"Carte sur http://localhost:{args.port}")

    if args.output:
        rr.save(args.output)
        print(f"Enregistrement: {args.output}")

    rr.send_blueprint(create_blueprint())
    log_static_map()

    print("\nCarte complète chargée.")
    if args.mode == "local":
        print("Viewer devrait s'ouvrir automatiquement...")


if __name__ == "__main__":
    main()
