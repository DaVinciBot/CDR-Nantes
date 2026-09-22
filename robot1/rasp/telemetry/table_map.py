#!/usr/bin/env python3
"""
Export de la carte Eurobot 2027 vers Rerun

Publie le terrain statique (playmat, balises, supports, zones, murs) dans
Rerun. La géométrie est codée en dur dans ce fichier : aucune dépendance à un
simulateur, seule la texture du playmat est lue sur disque.

Usage (depuis robot1/rasp/) :
    python -m telemetry.table_map --mode local --output recording.rrd
    python -m telemetry.table_map --mode serve --port 9876
"""

import argparse
import math
from pathlib import Path

import numpy as np
import rerun as rr

# Chemins
_DIR = Path(__file__).parent
TEXTURES_DIR = _DIR / "map_assets" / "playmat2027" 

# Charger positions depuis world.py
try:
    from robot1.rasp.world import BeaconLayout
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
C_TABLE   = [41, 107, 46, 80]
C_BEACON  = [255, 255, 255, 240]
C_SUP_YEL = [247, 181, 0, 220]
C_SUP_BLU = [0, 91, 140, 220]
C_ZONE_Y  = [247, 181, 0, 120]
C_ZONE_B  = [0, 91, 140, 120]

# ─────────────────────────────────────────────────────────────────────────────
# Géométrie du terrain (mm)
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


# ─────────────────────────────────────────────────────────────────────────────
# Publicateurs Rerun
# ─────────────────────────────────────────────────────────────────────────────


def log_static_map():
    """Publie la carte statique complète (balises, supports, zones, murs)."""

    print("Publication de la carte statique...")

    # ── Playmat (texture) ──
    try:
        from PIL import Image as PILImage

        playmat_path = TEXTURES_DIR / "Field.png"
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
            print(f"  [OK] Playmat texturé {playmat_path.name}")
    except Exception as e:
        print(f"  [WARN] Playmat error: {e} -> table verte fallback")
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
    print(f"  [OK] {len(BEACONS_MM)} balises")

    # ── Supports (BeaconSupport.proto) ──
    support_centers = np.array([s["pos"] + [100] for s in SUPPORTS_MM], dtype=np.float32)
    support_half = np.array([[61, 41, 100]] * len(SUPPORTS_MM), dtype=np.float32)
    support_colors = np.array([s["color"] for s in SUPPORTS_MM], dtype=np.uint8)
    rr.log(
        "world/map/supports",
        rr.Boxes3D(centers=support_centers, half_sizes=support_half, colors=support_colors),
        static=True,
    )
    print(f"  [OK] {len(SUPPORTS_MM)} supports")

    # ── Zones de calcul (CalculationZone.proto) ──
    calc_centers = np.array([c["pos"] for c in CALC_ZONES_MM], dtype=np.float32)
    calc_halves = np.array([c["half"] for c in CALC_ZONES_MM], dtype=np.float32)
    calc_colors = np.array([c["color"] for c in CALC_ZONES_MM], dtype=np.uint8)
    rr.log(
        "world/map/calc_zones",
        rr.Boxes3D(centers=calc_centers, half_sizes=calc_halves, colors=calc_colors),
        static=True,
    )
    print(f"  [OK] {len(CALC_ZONES_MM)} zones calcul")

    # ── Murs (BaseTable.proto) ──
    wall_centers = np.array([w["c"] for w in WALLS_MM], dtype=np.float32)
    wall_halves = np.array([w["h"] for w in WALLS_MM], dtype=np.float32)
    wall_colors = np.array([C_WALL] * len(WALLS_MM), dtype=np.uint8)
    rr.log(
        "world/map/walls",
        rr.Boxes3D(centers=wall_centers, half_sizes=wall_halves, colors=wall_colors),
        static=True,
    )
    print(f"  [OK] {len(WALLS_MM)} murs")

    print("[OK] Carte complète publiée!")


def create_blueprint():
    """Blueprint pour visualiser la map."""
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Spatial3DView(name="Terrain Eurobot 2027", origin="world"),
    )


def main():
    p = argparse.ArgumentParser(description="Carte Eurobot 2027 vers Rerun")
    p.add_argument("--mode", choices=["local", "serve"], default="local")
    p.add_argument("--port", type=int, default=9876)
    p.add_argument("--output", help="Enregistrer en .rrd")
    args = p.parse_args()

    # Init Rerun
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
