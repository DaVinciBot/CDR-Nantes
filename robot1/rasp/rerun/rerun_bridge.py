#!/usr/bin/env python3
"""
Rerun Bridge — Eurobot 2026 — Visualisation complète.

Architecture de visualisation pour la Raspberry Pi Eurobot 2026 :
  • La Raspberry Pi exécute ce script (données temps réel → Rerun SDK)
  • Modes de visualisation :
    - LOCAL (local) : Viewer s'ouvre directement sur la Rasp (seulement pour tests)
    - SERVE (serve) : Serveur WebSocket actif (accès distant depuis PC portable)
    - CONNECT (connect) : Se connecte à un Rerun Viewer externe (gRPC)

RECOMMANDÉ EN PRODUCTION :
  Lancer en mode « serve » sur la Rasp (ex. port 9876)
  Puis ouvrir le navigateur du PC : http://RaspIP:9876


Repère : coin bas-gauche = (0, 0) mm, identique au code Teensy.
  X : 0 → 3000 mm  (axe long du terrain)
  Y : 0 → 2000 mm  (axe court)
  Z : hauteur mm

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Sources de données :
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  Teensy  → odométrie (x, y, θ)
  Lidar   → nuage de points brut (angle, distance, intensité)
            balises détectées
            position calculée via trilatération
  PathFinding → trajectoire planifiée, point d'arrivée, position fusionnée

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Formes 3D (fidèles aux protos Webots) :
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  Balises (FixedBeacon.proto)   → cylindres vrais (Mesh3D triangulé, r=50mm h=1020mm)
  Supports (BeaconSupport.proto)→ L-shape approx (2 boîtes)
  Caisses (Crate*.proto)        → boîtes colorées (jaune/bleu/noir)
  Grenier                       → boîte texturée
  Murs    (BaseTable.proto)     → boîtes fines
  Playmat (BaseTable.proto)     → quad texturé (Mesh3D + PIL)
  Zones calcul                  → boîtes semi-transparentes
  Robot                         → cylindre vrai + flèche de direction

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Usage en développement (Rasp locale) :
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    # Mode simulation (test sans hardware)
    python rerun/rerun_bridge.py --mode local --sim

    # Mode serveur WebSocket (accès PC distant)
    python rerun/rerun_bridge.py --mode serve --host 0.0.0.0 --port 9876

    # Avec hardware Lidar
    python rerun/rerun_bridge.py --mode serve --with-lidar --port 9876

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Accès depuis PC portable :
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    1. Noter l'IP de la Rasp (ex. 192.168.1.100)
    2. Ouvrir navigateur : http://192.168.1.100:9876
    3. La visualisation s'affiche en direct dans le navigateur
    4. Historique disponible (timeline) pour rejouer les capteurs

Dépendances :
    pip install rerun-sdk Pillow numpy
"""
from __future__ import annotations

import argparse
import math
import struct
import threading
import time
import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
import rerun as rr
import rerun.blueprint as rrb

logging.basicConfig(level=logging.INFO, format="%(asctime)s | %(levelname)s | %(message)s")
logger = logging.getLogger("rerun_bridge")

# ─────────────────────────────────────────────────────────────────────────────
# Chemins
# ─────────────────────────────────────────────────────────────────────────────

_DIR = Path(__file__).parent  # /robot1/rasp/rerun/
PLAYMAT_PATH = _DIR / "map_assets" / "eurobot2026" / "textures" / "playmat_2026.jpg"

# ─────────────────────────────────────────────────────────────────────────────
# Terrain (mm, coin bas-gauche = origine)
# ─────────────────────────────────────────────────────────────────────────────

W  = 3000.0
H  = 2000.0
CX = W / 2    # 1500
CY = H / 2    # 1000

# Diagonale du terrain (pour vue polaire Lidar)
TERRAIN_DIAG_MM = math.sqrt(W**2 + H**2)  # ≈ 3606 mm

# ─────────────────────────────────────────────────────────────────────────────
# Couleurs RGBA uint8
# ─────────────────────────────────────────────────────────────────────────────

C_WALL    = [168, 168, 168, 230]
C_ATTIC   = [ 90,  76,  50, 180]
C_TABLE   = [ 41, 107,  46,  80]
C_YEL     = [242, 199,  46, 230]
C_BLU     = [ 51, 127, 242, 230]
C_BLK     = [ 20,  20,  20, 230]
C_BEACON  = [255, 255, 255, 240]
C_SUP_YEL = [247, 181,   0, 220]
C_SUP_BLU = [  0,  91, 140, 220]
C_ZONE_Y  = [247, 181,   0, 120]
C_ZONE_B  = [  0,  91, 140, 120]
C_ROBOT   = [ 51, 153, 255, 220]
C_LIDAR_P = [255,  80,  80, 255]
C_FUSED   = [ 80, 255, 120, 255]
C_TARGET  = [255, 220,  50, 255]
C_TRAJ    = [200, 200, 255, 200]

# ─────────────────────────────────────────────────────────────────────────────
# Objets du terrain (positions mm, repère coin bas-gauche)
# Conversion depuis proto Webots centré :
#   x_mm = x_proto * 1000 + 1500
#   y_mm = y_proto * 1000 + 1000
# ─────────────────────────────────────────────────────────────────────────────

# FixedBeacon.proto : Cylinder { height 1, radius 0.05 }
# Charger positions depuis terrain_jeu.py (source unique)
try:
    import sys
    sys.path.insert(0, str(_DIR.parent))
    from terrain_jeu import BeaconLayout
    # Convertir dict {1:(x,y), 2:(x,y)} → list [(x,y), ...]
    BEACONS_MM = sorted([pos for pos in BeaconLayout.BEACONS.values()])
    logger.info(f"✓ Balises depuis terrain_jeu: {BEACONS_MM}")
except (ImportError, AttributeError) as e:
    # Fallback si terrain_jeu indisponible
    logger.warning(f"terrain_jeu indisponible ({e}) → positions par défaut")
    BEACONS_MM = [(-50.0, 1000.0), (3050.0, 1950.0), (3050.0, 50.0)]
BEACON_R_MM = 50.0
BEACON_H_MM = 1020.0   # proto height=1m, centré à 0.51m → base z≈0, sommet z≈1020

# BeaconSupport.proto : boite ~122×82×140mm
# x_proto = ±1.594m → x_mm = -94 ou 3094
# y_proto = 0, ±0.952m → y_mm = 1000, 48, 1952
SUPPORTS_MM = [
    {"pos": (  -94, 1952), "color": C_SUP_YEL},
    {"pos": (  -94, 1000), "color": C_SUP_BLU},
    {"pos": (  -94,   48), "color": C_SUP_YEL},
    {"pos": ( 3094, 1952), "color": C_SUP_BLU},
    {"pos": ( 3094, 1000), "color": C_SUP_YEL},
    {"pos": ( 3094,   48), "color": C_SUP_BLU},
]

# CalculationZone.proto : ~450×200×22mm
# x_proto = ±0.225m → x_mm = 1275 ou 1725  ;  y_proto = 1.122m → y_mm = 2122
CALC_ZONES_MM = [
    {"pos": [1275, 2122, 11], "half": [225, 100, 11], "color": C_ZONE_Y},
    {"pos": [1725, 2122, 11], "half": [225, 100, 11], "color": C_ZONE_B},
]

# Attic : Box { size 1.8 0.45 0.055 } center (0, 0.775, 0.0275)m
ATTIC_CENTER = [1500, 1775, 27]
ATTIC_HALF   = [ 900,  225, 27]

# Murs (BaseTable.proto)
WALLS_MM = [
    {"c": [   -11, 1000,  35], "h": [  11, 1022,  35]},
    {"c": [  3011, 1000,  35], "h": [  11, 1022,  35]},
    {"c": [  1500, 2011,  35], "h": [1500,   11,  35]},
    {"c": [  1500,   -11, 35], "h": [1500,   11,  35]},
]

# Caisses (Crate.proto : 50×150×30mm)
def _build_crates():
    CG  = [-75.2, -25.1, 25.1, 75.2]
    CG2 = [-25.1, 25.1]
    CE  = [-50.1, 0.0, 50.1]
    raw = [
        ("CG",  1150,  800,  0,  0.0,          C_YEL),
        ("CG",  1100,  200,  0,  0.0,          C_YEL),
        ("CG",   175, 1200,  0,  math.pi/2,    C_YEL),
        ("CG",   175,  400,  0,  math.pi/2,    C_YEL),
        ("CG",   800, 1675, 55,  0.0,          C_YEL),
        ("CG2", 1100, 1725, 55,  0.0,          C_YEL),
        ("CG2", 1350, 1775, 55,  0.0,          C_YEL),
        ("CE",   800, 1675, 85,  0.0,          C_BLK),
        ("CG",  1850,  800,  0,  0.0,          C_BLU),
        ("CG",  1900,  200,  0,  0.0,          C_BLU),
        ("CG",  2825, 1200,  0,  math.pi/2,    C_BLU),
        ("CG",  2825,  400,  0,  math.pi/2,    C_BLU),
        ("CG",  2200, 1675, 55,  0.0,          C_BLU),
        ("CG2", 1900, 1725, 55,  0.0,          C_BLU),
        ("CG2", 1650, 1775, 55,  0.0,          C_BLU),
        ("CE",  2200, 1675, 85,  0.0,          C_BLK),
    ]
    offs = {"CG": CG, "CG2": CG2, "CE": CE}
    out = []
    for kind, tx, ty, tz, yaw, col in raw:
        c, s = math.cos(yaw), math.sin(yaw)
        # Inverser dimensions si yaw != 0 (rotation 90°)
        half_dims = [75, 25, 15] if abs(yaw) > 0.01 else [25, 75, 15]
        for lx in offs[kind]:
            ox, oy = lx*c, lx*s
            out.append({"center": [tx+ox, ty+oy, tz+15], "half": half_dims, "color": col})
    return out

CRATES_MM = _build_crates()

# ─────────────────────────────────────────────────────────────────────────────
# Générateur de maillage cylindre (FixedBeacon + robot)
# ─────────────────────────────────────────────────────────────────────────────

def _cylinder_mesh(cx: float, cy: float, z_bot: float,
                   radius: float, height: float, n: int = 20):
    """
    Retourne (vertices float32 Nx3, indices uint32 Mx3).
    Cylindre droit axe Z, centre base = (cx, cy, z_bot).
    """
    angles = np.linspace(0, 2*math.pi, n, endpoint=False)
    ca, sa = np.cos(angles), np.sin(angles)
    z_top  = z_bot + height

    # Anneaux bas (0..n-1) et haut (n..2n-1), centres (2n = bas, 2n+1 = haut)
    bot = np.column_stack([cx + radius*ca, cy + radius*sa, np.full(n, z_bot)])
    top = np.column_stack([cx + radius*ca, cy + radius*sa, np.full(n, z_top)])
    cnt = np.array([[cx, cy, z_bot], [cx, cy, z_top]])
    verts = np.vstack([bot, top, cnt]).astype(np.float32)

    bc, tc = 2*n, 2*n+1
    tris = []
    for i in range(n):
        j = (i+1) % n
        tris += [[i, j, n+j], [i, n+j, n+i]]  # flanc
        tris.append([bc, j, i])                  # bouchon bas
        tris.append([tc, n+i, n+j])              # bouchon haut

    return verts, np.array(tris, dtype=np.uint32)


# ─────────────────────────────────────────────────────────────────────────────
# Carte statique
# ─────────────────────────────────────────────────────────────────────────────

def log_static_map() -> None:
    """Publie tous les objets fixes (static=True, envoyés une seule fois)."""

    _log_playmat()

    # Murs
    rr.log("world/map/walls", rr.Boxes3D(
        centers=[w["c"] for w in WALLS_MM],
        half_sizes=[w["h"] for w in WALLS_MM],
        colors=[C_WALL] * len(WALLS_MM),
    ), static=True)

    # Grenier
    rr.log("world/map/attic", rr.Boxes3D(
        centers=[ATTIC_CENTER], half_sizes=[ATTIC_HALF], colors=[C_ATTIC],
    ), static=True)

    # Zones de calcul
    rr.log("world/map/calc_zones", rr.Boxes3D(
        centers=[z["pos"] for z in CALC_ZONES_MM],
        half_sizes=[z["half"] for z in CALC_ZONES_MM],
        colors=[z["color"] for z in CALC_ZONES_MM],
    ), static=True)

    # Supports de balises : approximation L-shape (2 boîtes par support)
    sup_c, sup_h, sup_col = [], [], []
    for sup in SUPPORTS_MM:
        sx, sy = sup["pos"]
        col = sup["color"]
        # Pied horizontal
        sup_c.append([sx, sy, 15])
        sup_h.append([61, 41, 15])
        sup_col.append(col)
        # Corps vertical (penché vers l'intérieur du terrain)
        ox = 35 if sx < CX else -35
        sup_c.append([sx + ox, sy, 90])
        sup_h.append([20, 35, 75])
        sup_col.append(col)

    rr.log("world/map/beacon_supports", rr.Boxes3D(
        centers=np.array(sup_c, dtype=np.float32),
        half_sizes=np.array(sup_h, dtype=np.float32),
        colors=np.array(sup_col, dtype=np.uint8),
    ), static=True)

    # Balises — cylindres vrais (Mesh3D triangulé)
    for i, (bx, by) in enumerate(BEACONS_MM):
        verts, tris = _cylinder_mesh(bx, by, 0.0, BEACON_R_MM, BEACON_H_MM)
        cols = np.tile(C_BEACON, (len(verts), 1)).astype(np.uint8)
        rr.log(f"world/map/beacon_{i}", rr.Mesh3D(
            vertex_positions=verts,
            triangle_indices=tris,
            vertex_colors=cols,
        ), static=True)

    # Caisses
    rr.log("world/map/crates", rr.Boxes3D(
        centers=np.array([c["center"] for c in CRATES_MM], dtype=np.float32),
        half_sizes=np.array([c["half"] for c in CRATES_MM], dtype=np.float32),
        colors=np.array([c["color"] for c in CRATES_MM], dtype=np.uint8),
    ), static=True)

    logger.info("Carte statique publiée — %d balises | %d supports | %d caisses",
                len(BEACONS_MM), len(SUPPORTS_MM), len(CRATES_MM))


def _log_playmat() -> None:
    if not PLAYMAT_PATH.exists():
        logger.warning("Playmat introuvable → table verte")
        rr.log("world/map/table", rr.Boxes3D(
            centers=[[CX, CY, 1]], half_sizes=[[W/2, H/2, 1]], colors=[C_TABLE],
        ), static=True)
        return
    try:
        from PIL import Image as PILImage
        img = PILImage.open(PLAYMAT_PATH).convert("RGB")
        img_arr = np.array(img, dtype=np.uint8)
        verts = np.array([
            [0., 0., 0.], [W, 0., 0.], [W, H, 0.], [0., H, 0.]
        ], dtype=np.float32)
        uvs   = np.array([[0.,1.],[1.,1.],[1.,0.],[0.,0.]], dtype=np.float32)
        tris  = np.array([[0,1,2],[0,2,3]], dtype=np.uint32)
        rr.log("world/map/playmat", rr.Mesh3D(
            vertex_positions=verts,
            triangle_indices=tris,
            vertex_texcoords=uvs,
            albedo_texture=img_arr,
        ), static=True)
        logger.info("Playmat %s (%dx%d)", PLAYMAT_PATH.name, img.width, img.height)
    except Exception as e:
        logger.error("Erreur playmat : %s", e)
        rr.log("world/map/table", rr.Boxes3D(
            centers=[[CX, CY, 1]], half_sizes=[[W/2, H/2, 1]], colors=[C_TABLE],
        ), static=True)


# ─────────────────────────────────────────────────────────────────────────────
# Robot (cylindre vrai + flèche)
# ─────────────────────────────────────────────────────────────────────────────

ROBOT_R_MM   = 175.0
ROBOT_H_MM   = 100.0
ARROW_L_MM   = 250.0
_rv, _rt = _cylinder_mesh(0, 0, 0, ROBOT_R_MM, ROBOT_H_MM)  # template centré à (0,0)


def _log_robot(path: str, x: float, y: float, theta: float, color: list) -> None:
    verts = _rv.copy()
    verts[:, 0] += x
    verts[:, 1] += y
    cols = np.tile(color[:3], (len(verts), 1)).astype(np.uint8)
    rr.log(f"{path}/body", rr.Mesh3D(vertex_positions=verts,
                                      triangle_indices=_rt, vertex_colors=cols))
    dx, dy = ARROW_L_MM * math.cos(theta), ARROW_L_MM * math.sin(theta)
    rr.log(f"{path}/arrow", rr.Arrows3D(
        origins=[[x, y, ROBOT_H_MM+20]], vectors=[[dx, dy, 0.0]], colors=[color]))


# ─────────────────────────────────────────────────────────────────────────────
# État partagé thread-safe
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class _State:
    odom_x: float = CX
    odom_y: float = CY
    odom_theta: float = 0.0
    lidar_cloud: list   = field(default_factory=list)   # [(angle_rad, dist_mm, qual)]
    lidar_beacons: list = field(default_factory=list)   # dicts beacon candidates
    lidar_x: float = 0.0
    lidar_y: float = 0.0
    lidar_theta: float = 0.0
    lidar_conf: float = 0.0
    lidar_ok: bool = False
    target_x: float = CX
    target_y: float = CY
    target_theta: float = 0.0
    trajectory: list = field(default_factory=list)      # [(x,y)]
    obstacles: list = field(default_factory=list)       # [{"x": x, "y": y, "radius": r}, ...]
    fused_x: float = CX
    fused_y: float = CY
    fused_theta: float = 0.0
    _lock: threading.RLock = field(default_factory=threading.RLock)

    def snap(self):
        with self._lock:
            s = _State()
            for f in self.__dataclass_fields__:
                if f != "_lock":
                    setattr(s, f, getattr(self, f))
            return s


_st = _State()


# ─────────────────────────────────────────────────────────────────────────────
# API publique
# ─────────────────────────────────────────────────────────────────────────────

def update_odom(x: float, y: float, theta: float):
    with _st._lock:
        _st.odom_x, _st.odom_y, _st.odom_theta = x, y, theta

def update_lidar_cloud(pts):
    with _st._lock: _st.lidar_cloud = list(pts)

def update_lidar_beacons(cands):
    with _st._lock: _st.lidar_beacons = list(cands)

def update_lidar_pose(x: float, y: float, theta: float, conf: float, ok: bool = True):
    with _st._lock:
        _st.lidar_x, _st.lidar_y, _st.lidar_theta = x, y, theta
        _st.lidar_conf, _st.lidar_ok = conf, ok

def update_target(x: float, y: float, theta: float = None):
    with _st._lock:
        _st.target_x, _st.target_y = x, y
        if theta is not None:
            _st.target_theta = theta

def update_trajectory(pts):
    with _st._lock: _st.trajectory = list(pts)

def update_obstacles(obstacles):
    """Update detected obstacles (adversary robots, static obstacles).
    
    Args:
        obstacles: List of dicts {"x": float, "y": float, "radius": float}
                   Positions in mm, radius in mm
    """
    with _st._lock: _st.obstacles = list(obstacles)

def update_fused(x: float, y: float, theta: float):
    with _st._lock: _st.fused_x, _st.fused_y, _st.fused_theta = x, y, theta


# ─────────────────────────────────────────────────────────────────────────────
# Callbacks factories
# ─────────────────────────────────────────────────────────────────────────────

def make_odom_callback():
    """com.add_callback(make_odom_callback(), Messages.UPDATE_ROLLING_BASIS.value)"""
    def cb(data: bytes):
        if len(data) >= 24:
            x, y, t = struct.unpack("<ddd", data[:24])
            update_odom(x, y, t)
    return cb


def make_lidar_poll():
    """
    Retourne une fonction à appeler dans la boucle principale.
    Récupère le nuage, les balises et la pose depuis lidar_logic.
    """
    from lidar.lidar_logic import (
        get_latest_scan_data, get_latest_beacon_candidates, get_corrected_pose,
    )
    def poll():
        update_lidar_cloud(get_latest_scan_data())
        update_lidar_beacons(get_latest_beacon_candidates())
        p = get_corrected_pose()
        if p:
            update_lidar_pose(p.x, p.y, p.theta, p.confidence, p.is_localized)
    return poll


# ─────────────────────────────────────────────────────────────────────────────
# Publication à 20 Hz
# ─────────────────────────────────────────────────────────────────────────────

def _log_lidar_polar(pts_cloud: list) -> None:
    """
    Publie une vue 2D POLAIRE du Lidar dans Rerun.
    
    Vue radar 2D pure:
    - X axis: angle en radiants (-π à +π)
    - Y axis: distance (mm, normalisée par diagonale terrain)
    - Cercles concentriques = références distance (500, 1000, 1500, ...mm)
    """
    if not pts_cloud:
        rr.log("sensors/lidar/polar_view", rr.Clear(recursive=False))
        rr.log("sensors/lidar/reference_circles", rr.Clear(recursive=False))
        return
    
    try:
        pts = np.array(pts_cloud, dtype=np.float32)  # [(angle_rad, dist_mm, qual), ...]
        
        # Vérifier format
        if pts.ndim != 2 or pts.shape[1] < 2:
            logger.warning(f"Format polaire invalide: shape={pts.shape}, clearing")
            rr.log("sensors/lidar/polar_view", rr.Clear(recursive=False))
            return
        
        ang = pts[:, 0]  # angle en radiants
        dist = pts[:, 1]  # distance en mm
        qual = pts[:, 2] if pts.shape[1] > 2 else np.ones(len(pts))  # qualité optionnelle
        
        # Couleur par intensité (qualité): gris→cyan
        t = np.clip(qual / 15.0, 0, 1)
        colors = np.column_stack([
            np.zeros(len(t), np.uint8),
            (t * 200).astype(np.uint8),
            (255 - t * 80).astype(np.uint8),
            np.full(len(t), 220, np.uint8),
        ])
        
        # Publier points polaires 2D
        rr.log("sensors/lidar/polar_view", rr.Points2D(
            positions=np.column_stack([ang, dist]).astype(np.float32),
            colors=colors, radii=np.full(len(ang), 8.0),
        ))
        
        # Ajouter grille de référence (cercles tous les 500 mm jusqu'à TERRAIN_DIAG_MM)
        grid_pts = []
        grid_col = []
        
        # Lignes radiales (tous les 45° = π/4) - MOINS DENSES
        for angle in np.arange(-math.pi, math.pi, math.pi/4):
            for d in np.arange(0, TERRAIN_DIAG_MM + 500, 500):  # Aussi moins dense (500 mm au lieu de 100)
                grid_pts.append([angle, d])
                grid_col.append([100, 120, 150, 120])  # Bleu grisâtre
        
        # Cercles concentriques (tous les 500 mm) - MOINS DE POINTS PAR CERCLE
        for d in np.arange(500, TERRAIN_DIAG_MM + 500, 500):
            for angle in np.linspace(-math.pi, math.pi, 60):  # Réduit de 120 à 60 points
                grid_pts.append([angle, d])
                grid_col.append([120, 140, 170, 140])  # Bleu plus clair
        
        if grid_pts:
            rr.log("sensors/lidar/reference_circles", rr.Points2D(
                positions=np.array(grid_pts, dtype=np.float32),
                colors=np.array(grid_col, dtype=np.uint8),
                radii=np.full(len(grid_pts), 3.0),  # Un peu plus gros pour visibilité
            ))
        
        logger.debug(f"Polaire: {len(ang)} pts, angle range [{ang.min():.2f}, {ang.max():.2f}] rad, dist range [{dist.min():.0f}, {dist.max():.0f}] mm")
    
    except Exception as e:
        logger.error(f"Erreur polaire: {e}", exc_info=True)
        rr.log("sensors/lidar/polar_view", rr.Clear(recursive=False))




def _publish(s: _State) -> None:

    # ── Odométrie Teensy (robot bleu) ─────────────────────────────────────────
    _log_robot("world/robot/odom", s.odom_x, s.odom_y, s.odom_theta, C_ROBOT)

    # ── Nuage Lidar (projeté en coordonnées monde) ────────────────────────────
    if s.lidar_cloud:
        pts = np.array(s.lidar_cloud, dtype=np.float32)
        ang, dist, qual = pts[:,0], pts[:,1], pts[:,2]
        ct, st = math.cos(s.odom_theta), math.sin(s.odom_theta)
        # repère lidar → monde  (convention: x=dist*sin, y=dist*cos en repère lidar)
        xl = dist * np.sin(ang)
        yl = dist * np.cos(ang)
        xw = s.odom_x + ct*xl - st*yl
        yw = s.odom_y + st*xl + ct*yl
        zw = np.full(len(xw), 30.0)

        t   = np.clip(qual / 15.0, 0, 1)
        col = np.column_stack([
            np.zeros(len(t), np.uint8),
            (t * 200).astype(np.uint8),
            (255 - t * 80).astype(np.uint8),
            np.full(len(t), 180, np.uint8),
        ])
        rr.log("world/lidar/cloud", rr.Points3D(
            positions=np.column_stack([xw, yw, zw]).astype(np.float32),
            colors=col, radii=np.full(len(xw), 8.0),
        ))
    else:
        rr.log("world/lidar/cloud", rr.Clear(recursive=False))

    # ── Vue polaire Lidar (radar) ──────────────────────────────────────────────
    _log_lidar_polar(s.lidar_cloud)

    # ── Balises détectées (diamants orange) ───────────────────────────────────
    if s.lidar_beacons:
        ct, st = math.cos(s.odom_theta), math.sin(s.odom_theta)
        bpts = []
        for bc in s.lidar_beacons:
            xr, yr = float(bc["x_r"]), float(bc["y_r"])
            bpts.append([s.odom_x + ct*xr - st*yr,
                         s.odom_y + st*xr + ct*yr, 150.0])
        rr.log("world/lidar/beacons_detected", rr.Points3D(
            positions=np.array(bpts, np.float32),
            colors=[[255,170,0,255]] * len(bpts),
            radii=np.full(len(bpts), 55.0),
        ))
    else:
        rr.log("world/lidar/beacons_detected", rr.Clear(recursive=False))

    # ── Position calculée par Lidar (robot rouge) ─────────────────────────────
    if s.lidar_ok:
        _log_robot("world/lidar/pose", s.lidar_x, s.lidar_y, s.lidar_theta, C_LIDAR_P)
    else:
        rr.log("world/lidar/pose/body",  rr.Clear(recursive=False))
        rr.log("world/lidar/pose/arrow", rr.Clear(recursive=False))

    # ── Position fusionnée (robot vert) ───────────────────────────────────────
    _log_robot("world/robot/fused", s.fused_x, s.fused_y, s.fused_theta, C_FUSED)

    # ── Cible PathFinding (point jaune avec flèche) ───────────────────────────────────────
    rr.log("world/pathfinding/target", rr.Points3D(
        positions=[[s.target_x, s.target_y, 60.0]],
        colors=[C_TARGET], radii=[80.0],
    ))
    # Flèche directionnelle sur la cible (si theta défini)
    if s.target_theta is not None:
        target_arrow_len = 150.0
        tdx = target_arrow_len * math.cos(s.target_theta)
        tdy = target_arrow_len * math.sin(s.target_theta)
        rr.log("world/pathfinding/target_arrow", rr.Arrows3D(
            origins=[[s.target_x, s.target_y, 60.0]],
            vectors=[[tdx, tdy, 0.0]],
            colors=[C_TARGET]
        ))

    # ── Trajectoire (ligne pointillée) ────────────────────────────────────────
    if len(s.trajectory) >= 2:
        pts = [[x, y, 50.0] for x, y in s.trajectory]
        rr.log("world/pathfinding/trajectory", rr.LineStrips3D(
            strips=[pts], colors=[C_TRAJ], radii=[8.0],
        ))
    else:
        rr.log("world/pathfinding/trajectory", rr.Clear(recursive=False))

    # ── Robots adversaires et obstacles (sphères rouges) ───────────────────────
    if s.obstacles:
        obs_pts = []
        obs_radii = []
        for obs in s.obstacles:
            obs_pts.append([obs.get("x", 0), obs.get("y", 0), obs.get("radius", 100)])
            obs_radii.append(obs.get("radius", 100))
        rr.log("world/obstacles/adversaries", rr.Points3D(
            positions=np.array(obs_pts, dtype=np.float32),
            colors=[[255, 50, 50, 200]] * len(obs_pts),  # Red with transparency
            radii=np.array(obs_radii, dtype=np.float32),
        ))
    else:
        rr.log("world/obstacles/adversaries", rr.Clear(recursive=False))

    # ── Courbes temporelles ───────────────────────────────────────────────────
    rr.log("data/teensy/x_mm",      rr.Scalars(s.odom_x))
    rr.log("data/teensy/y_mm",      rr.Scalars(s.odom_y))
    rr.log("data/teensy/theta_deg", rr.Scalars(math.degrees(s.odom_theta)))

    rr.log("data/lidar/confidence", rr.Scalars(s.lidar_conf))
    rr.log("data/lidar/nb_points",  rr.Scalars(float(len(s.lidar_cloud))))
    rr.log("data/lidar/nb_beacons", rr.Scalars(float(len(s.lidar_beacons))))
    if s.lidar_ok:
        rr.log("data/lidar/x_mm",   rr.Scalars(s.lidar_x))
        rr.log("data/lidar/y_mm",   rr.Scalars(s.lidar_y))

    rr.log("data/fused/x_mm",       rr.Scalars(s.fused_x))
    rr.log("data/fused/y_mm",       rr.Scalars(s.fused_y))

    if s.lidar_ok:
        err = math.hypot(s.odom_x - s.lidar_x, s.odom_y - s.lidar_y)
        rr.log("data/fusion/ecart_odom_lidar_mm", rr.Scalars(err))


def publish_loop(hz: float = 20.0, lidar_poll=None) -> None:
    dt = 1.0 / hz
    while True:
        if lidar_poll:
            try: lidar_poll()
            except Exception as e: logger.debug("lidar poll: %s", e)
        _publish(_st.snap())
        time.sleep(dt)


# ─────────────────────────────────────────────────────────────────────────────
# Simulation
# ─────────────────────────────────────────────────────────────────────────────

def simulation_loop(radius_mm: float = 500.0, speed: float = 0.4) -> None:
    """Robot en cercle + faux nuage Lidar + trajectoire de test."""
    t = 0.0
    logger.info("Simulation — R=%.0f mm", radius_mm)

    update_trajectory([
        (CX+600, CY), (CX+600, CY+400), (CX-200, CY+400), (CX-200, CY), (CX+600, CY)
    ])
    update_target(CX+600, CY+400)

    while True:
        a = t * speed
        ox = CX + radius_mm * math.cos(a)
        oy = CY + radius_mm * math.sin(a)
        ot = a + math.pi/2
        update_odom(ox, oy, ot)

        lx = ox + 20*math.sin(t*2.5)
        ly = oy + 20*math.cos(t*2.5)
        conf = 0.85 + 0.1*math.sin(t*0.8)
        update_lidar_pose(lx, ly, ot+0.03, conf)
        update_fused(0.6*lx+0.4*ox, 0.6*ly+0.4*oy, ot)

        # Faux nuage (360 points, murs + bruit)
        pts = []
        for deg in range(0, 360, 2):
            ar = math.radians(deg)
            ca, sa = math.cos(ar), math.sin(ar)
            d = min(
                abs((W-ox)/ca) if ca > 0.001 else 9999,
                abs(-ox/ca)    if ca < -0.001 else 9999,
                abs((H-oy)/sa) if sa > 0.001 else 9999,
                abs(-oy/sa)    if sa < -0.001 else 9999,
                3500.0,
            ) + 30*(np.random.rand()-0.5)
            pts.append((ar, max(50, d), 8+int(5*np.random.rand())))
        update_lidar_cloud(pts)

        t += 0.05
        time.sleep(0.05)


# ─────────────────────────────────────────────────────────────────────────────
# Blueprint
# ─────────────────────────────────────────────────────────────────────────────

def create_blueprint() -> rrb.Blueprint:
    return rrb.Blueprint(
        rrb.Vertical(
            rrb.Horizontal(
                rrb.Spatial3DView(name="Terrain Eurobot 2026", origin="world"),
                rrb.Spatial2DView(name="Lidar Polaire (Radar)", origin="sensors/lidar"),
                column_shares=[1, 1],
            ),
            rrb.Horizontal(
                rrb.TimeSeriesView(name="Position Teensy (mm)", origin="data/teensy"),
                rrb.TimeSeriesView(name="Lidar",                origin="data/lidar"),
                rrb.TimeSeriesView(name="Fusionné",             origin="data/fused"),
                rrb.TimeSeriesView(name="Écart odom↔lidar",    origin="data/fusion"),
                column_shares=[3, 2, 2, 2],
            ),
            row_shares=[1, 1],
        ),
    )


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main() -> None:
    p = argparse.ArgumentParser(description="Rerun Bridge — Eurobot 2026")
    p.add_argument("--mode", choices=["local","serve","connect"], default="local",
                   help="local: Viewer local | serve: WebSocket serveur | connect: gRPC distant")
    p.add_argument("--host", default="0.0.0.0",
                   help="Host pour serveur (0.0.0.0 = accès depuis n'importe où)")
    p.add_argument("--port", type=int, default=9876,
                   help="Port WebSocket ou gRPC")
    p.add_argument("--sim",         action="store_true", help="Simulation robot en cercle")
    p.add_argument("--with-lidar",  action="store_true", help="Active polling lidar hardware")
    args = p.parse_args()

    rr.init("eurobot_2026", spawn=(args.mode == "local"))

    if args.mode == "serve":
        logger.info("🌐 Mode SERVE activé")
        logger.info(f"   Serveur Rerun WebSocket sur {args.host}:{args.port}")
        logger.info(f"   Accédez depuis un navigateur : http://[RaspIP]:{args.port}")
        rr.serve_web(open_browser=False, web_port=args.port)
    elif args.mode == "connect":
        logger.info(f"🔗 Connexion gRPC à {args.host}:{args.port}")
        rr.connect_grpc(f"{args.host}:{args.port}")

    rr.send_blueprint(create_blueprint())
    log_static_map()

    lidar_poll = None
    if args.with_lidar:
        try:
            lidar_poll = make_lidar_poll()
            logger.info("✓ Lidar polling activé")
        except ImportError:
            logger.warning("⚠ lidar_logic indisponible")

    # ─────────────────────────────────────────────────────────────────────────
    # Initialiser source d'odométrie (Teensy hardware OU simulation)
    # ─────────────────────────────────────────────────────────────────────────
    
    if args.sim:
        logger.info("=" * 70)
        logger.info("🎮 MODE SIMULATION activé (robot en cercle)")
        logger.info("=" * 70)
        threading.Thread(target=simulation_loop, daemon=True).start()
    else:
        logger.info("=" * 70)
        logger.info("🔌 Initialisation hardware Teensy...")
        try:
            import sys
            sys.path.insert(0, str(_DIR.parent))
            from utils import init_robot
            from loader import loader
            Messages = loader.load_class("usb_com", "Messages")
            com, mode = init_robot(logger)
            com.add_callback(make_odom_callback(), Messages.UPDATE_ROLLING_BASIS.value)
            logger.info("✅ Teensy connecté —— Odométrie affichée en direct")
            logger.info(f"   Mode: {mode}")
            logger.info("=" * 70)
        except Exception as e:
            if args.with_lidar:
                logger.warning("⚠️  Teensy non détecté — Mode Lidar seul activé")
                logger.info(f"   ({e})")
                logger.info("=" * 70)
                # Poster une position par défaut au centre du terrain
                update_odom(1500.0, 1000.0, 0.0)
            else:
                logger.error("❌ Teensy indisponible : %s", e)
                logger.error("   Options:")
                logger.error("   • Branchez le Teensy en USB")
                logger.error("   • Ou lancez avec --sim pour simulation")
                logger.error("   • Ou lancez avec --with-lidar pour Lidar seul")
                logger.info("=" * 70)
                sys.exit(1)

    logger.info("▶ Publication Rerun à 20 Hz")
    logger.info(f"   Position Teensy: world/robot/odom (cylindre bleu)")
    logger.info(f"   Nuage Lidar: world/lidar/cloud{' ✓' if lidar_poll else ''}")
    logger.info("=" * 70)
    publish_loop(hz=20.0, lidar_poll=lidar_poll)


if __name__ == "__main__":
    main()
