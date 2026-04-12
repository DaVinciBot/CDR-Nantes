#!/usr/bin/env python3
"""
Export Webots Map → Rerun

Charge la scène Webots complète (terrain, balises, supports, caisses, etc.)
et l'affiche en Rerun avec toutes les textures et géométries.

Usage:
    python webots_map_exporter.py --mode local --output recording.rrd
    python webots_map_exporter.py --mode serve --port 9876
"""

import sys
import argparse
import math
from pathlib import Path

import numpy as np
import rerun as rr

# Chemins
_DIR = Path(__file__).parent
WEBOTS_DIR = _DIR.parent.parent / "simulation"
WORLD_FILE = WEBOTS_DIR / "worlds" / "my_world.wbt"
PROTOS_DIR = WEBOTS_DIR / "protos" / "table"
TEXTURES_DIR = WEBOTS_DIR / "protos" / "textures"
RERUN_MAP_ASSETS = _DIR / "rerun" / "map_assets" / "eurobot2026"

# Charger positions depuis terrain_jeu.py
try:
    sys.path.insert(0, str(_DIR / ".."))
    from terrain_jeu import BeaconLayout
    BEACONS_POS = BeaconLayout.BEACONS
except ImportError:
    # Fallback: positions codées
    BEACONS_POS = [(-50.0, 1000.0), (3050.0, 1950.0), (3050.0, 50.0)]

# ─────────────────────────────────────────────────────────────────────────────
# Terrain constants (mm)
# ─────────────────────────────────────────────────────────────────────────────

W  = 3000.0
H  = 2000.0
CX = W / 2
CY = H / 2

# Couleurs
C_WALL    = [168, 168, 168, 230]
C_ATTIC   = [90, 76, 50, 180]
C_TABLE   = [41, 107, 46, 80]
C_YEL     = [242, 199, 46, 230]
C_BLU     = [51, 127, 242, 230]
C_BLK     = [20, 20, 20, 230]
C_BEACON  = [255, 255, 255, 240]
C_SUP_YEL = [247, 181, 0, 220]
C_SUP_BLU = [0, 91, 140, 220]
C_ZONE_Y  = [247, 181, 0, 120]
C_ZONE_B  = [0, 91, 140, 120]

# ─────────────────────────────────────────────────────────────────────────────
# Webots Proto → Python structs
# ─────────────────────────────────────────────────────────────────────────────

BEACONS_MM = BEACONS_POS
BEACON_R_MM = 50.0
BEACON_H_MM = 1020.0

SUPPORTS_MM = [
    {"pos": (-94, 1952), "color": C_SUP_YEL},
    {"pos": (-94, 1000), "color": C_SUP_BLU},
    {"pos": (-94, 48), "color": C_SUP_YEL},
    {"pos": (3094, 1952), "color": C_SUP_BLU},
    {"pos": (3094, 1000), "color": C_SUP_YEL},
    {"pos": (3094, 48), "color": C_SUP_BLU},
]

CALC_ZONES_MM = [
    {"pos": [1275, 2122, 11], "half": [225, 100, 11], "color": C_ZONE_Y},
    {"pos": [1725, 2122, 11], "half": [225, 100, 11], "color": C_ZONE_B},
]

ATTIC_CENTER = [1500, 1775, 27]
ATTIC_HALF = [900, 225, 27]

WALLS_MM = [
    {"c": [-11, 1000, 35], "h": [11, 1022, 35]},
    {"c": [3011, 1000, 35], "h": [11, 1022, 35]},
    {"c": [1500, 2011, 35], "h": [1500, 11, 35]},
    {"c": [1500, -11, 35], "h": [1500, 11, 35]},
]


def _cylinder_mesh(cx, cy, z_bot, radius, height, n=20):
    """Cylindre pour les balises (FixedBeacon.proto)."""
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


def _build_crates():
    """Caisses depuis rerun_bridge.py."""
    CG = [-75.2, -25.1, 25.1, 75.2]
    CG2 = [-25.1, 25.1]
    CE = [-50.1, 0.0, 50.1]
    raw = [
        ("CG", 1150, 800, 0, 0.0, C_YEL),
        ("CG", 1100, 200, 0, 0.0, C_YEL),
        ("CG", 175, 1200, 0, math.pi / 2, C_YEL),
        ("CG", 175, 400, 0, math.pi / 2, C_YEL),
        ("CG", 800, 1675, 55, 0.0, C_YEL),
        ("CG2", 1100, 1725, 55, 0.0, C_YEL),
        ("CG2", 1350, 1775, 55, 0.0, C_YEL),
        ("CE", 800, 1675, 85, 0.0, C_BLK),
        ("CG", 1850, 800, 0, 0.0, C_BLU),
        ("CG", 1900, 200, 0, 0.0, C_BLU),
        ("CG", 2825, 1200, 0, math.pi / 2, C_BLU),
        ("CG", 2825, 400, 0, math.pi / 2, C_BLU),
        ("CG", 2200, 1675, 55, 0.0, C_BLU),
        ("CG2", 1900, 1725, 55, 0.0, C_BLU),
        ("CG2", 1650, 1775, 55, 0.0, C_BLU),
        ("CE", 2200, 1675, 85, 0.0, C_BLK),
    ]
    offs = {"CG": CG, "CG2": CG2, "CE": CE}
    out = []
    for kind, tx, ty, tz, yaw, col in raw:
        c, s = math.cos(yaw), math.sin(yaw)
        for lx in offs[kind]:
            ox, oy = lx * c, lx * s
            out.append(
                {"center": [tx + ox, ty + oy, tz + 15], "half": [25, 75, 15], "color": col}
            )
    return out


CRATES_MM = _build_crates()


# ─────────────────────────────────────────────────────────────────────────────
# Publicateurs Rerun
# ─────────────────────────────────────────────────────────────────────────────


def log_static_map():
    """Publie la carte statique complète (balises, supports, caisses, etc.)."""

    print("📍 Logging Webots map assets...")

    # ── Playmat (texture) ──
    try:
        from PIL import Image as PILImage

        playmat_path = TEXTURES_DIR / "playmat_2026.jpg"
        if playmat_path.exists():
            img = PILImage.open(playmat_path).convert("RGB")
            img_arr = np.array(img, dtype=np.uint8)
            verts = np.array(
                [[0.0, 0.0, 0.0], [W, 0.0, 0.0], [W, H, 0.0], [0.0, H, 0.0]],
                dtype=np.float32,
            )
            uvs = np.array([[0.0, 1.0], [1.0, 1.0], [1.0, 0.0], [0.0, 0.0]], dtype=np.float32)
            tris = np.array([[0, 1, 2], [0, 2, 3]], dtype=np.uint32)
            rr.log(
                "world/map/playmat",
                rr.Mesh3D(
                    vertex_positions=verts,
                    triangle_indices=tris,
                    vertex_texcoords=uvs,
                    albedo_texture=img_arr,
                ),
                static=True,
            )
            print(f"  ✓ Playmat texturé {playmat_path.name}")
    except Exception as e:
        print(f"  ⚠ Playmat error: {e} → table verte fallback")
        rr.log(
            "world/map/table",
            rr.Boxes3D(
                centers=[[CX, CY, 1]], half_sizes=[[W / 2, H / 2, 1]], colors=[C_TABLE]
            ),
            static=True,
        )

    # ── Balises (cylindres FixedBeacon.proto) ──
    for i, (bx, by) in enumerate(BEACONS_MM):
        verts, tris = _cylinder_mesh(bx, by, 0.0, BEACON_R_MM, BEACON_H_MM)
        cols = np.tile(C_BEACON, (len(verts), 1)).astype(np.uint8)
        rr.log(
            f"world/map/beacon_{i}",
            rr.Mesh3D(vertex_positions=verts, triangle_indices=tris, vertex_colors=cols),
            static=True,
        )
    print(f"  ✓ {len(BEACONS_MM)} balises")

    # ── Supports (BeaconSupport.proto) ──
    support_centers = np.array([s["pos"] + [100] for s in SUPPORTS_MM], dtype=np.float32)
    support_half = np.array([[61, 41, 100]] * len(SUPPORTS_MM), dtype=np.float32)
    support_colors = np.array([s["color"] for s in SUPPORTS_MM], dtype=np.uint8)
    rr.log(
        "world/map/supports",
        rr.Boxes3D(centers=support_centers, half_sizes=support_half, colors=support_colors),
        static=True,
    )
    print(f"  ✓ {len(SUPPORTS_MM)} supports")

    # ── Zones de calcul (CalculationZone.proto) ──
    calc_centers = np.array([c["pos"] for c in CALC_ZONES_MM], dtype=np.float32)
    calc_halves = np.array([c["half"] for c in CALC_ZONES_MM], dtype=np.float32)
    calc_colors = np.array([c["color"] for c in CALC_ZONES_MM], dtype=np.uint8)
    rr.log(
        "world/map/calc_zones",
        rr.Boxes3D(centers=calc_centers, half_sizes=calc_halves, colors=calc_colors),
        static=True,
    )
    print(f"  ✓ {len(CALC_ZONES_MM)} zones calcul")

    # ── Grenier (Attic) ──
    rr.log(
        "world/map/attic",
        rr.Boxes3D(
            centers=[ATTIC_CENTER],
            half_sizes=[ATTIC_HALF],
            colors=[C_ATTIC],
        ),
        static=True,
    )
    print(f"  ✓ Grenier")

    # ── Murs (BaseTable.proto) ──
    wall_centers = np.array([w["c"] for w in WALLS_MM], dtype=np.float32)
    wall_halves = np.array([w["h"] for w in WALLS_MM], dtype=np.float32)
    wall_colors = np.array([C_WALL] * len(WALLS_MM), dtype=np.uint8)
    rr.log(
        "world/map/walls",
        rr.Boxes3D(centers=wall_centers, half_sizes=wall_halves, colors=wall_colors),
        static=True,
    )
    print(f"  ✓ {len(WALLS_MM)} murs")

    # ── Caisses (Crate*.proto) ──
    crate_centers = np.array([c["center"] for c in CRATES_MM], dtype=np.float32)
    crate_halves = np.array([c["half"] for c in CRATES_MM], dtype=np.float32)
    crate_colors = np.array([c["color"] for c in CRATES_MM], dtype=np.uint8)
    rr.log(
        "world/map/crates",
        rr.Boxes3D(centers=crate_centers, half_sizes=crate_halves, colors=crate_colors),
        static=True,
    )
    print(f"  ✓ {len(CRATES_MM)} caisses")

    print("✓ Carte complète publiée!")


def create_blueprint():
    """Blueprint pour visualiser la map."""
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Spatial3DView(name="Terrain Eurobot 2026", origin="world"),
    )


def main():
    p = argparse.ArgumentParser(description="Webots Map → Rerun")
    p.add_argument("--mode", choices=["local", "serve"], default="local")
    p.add_argument("--port", type=int, default=9876)
    p.add_argument("--output", help="Enregistrer en .rrd")
    args = p.parse_args()

    # Init Rerun
    rr.init("webots_map", spawn=(args.mode == "local"))

    if args.mode == "serve":
        rr.serve_web(open_browser=False, web_port=args.port)
        print(f"" Webots map sur http://localhost:{args.port}")

    if args.output:
        rr.save(args.output)
        print(f"Enregistrement: {args.output}")

    rr.send_blueprint(create_blueprint())
    log_static_map()

    print("\n✨ Map Webots complète chargée!")
    if args.mode == "local":
        print("Viewer devrait s'ouvrir automatiquement...")


if __name__ == "__main__":
    main()
