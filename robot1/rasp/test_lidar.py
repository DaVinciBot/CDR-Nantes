"""
test_lidar.py — debug détection adversaire (terrain test 1040×1040mm).
LiDAR au centre du terrain. Branche en USB puis : python test_lidar.py
"""

import math
import time
import csv
import sys
import os
import logging

ROOT = os.path.dirname(os.path.abspath(__file__))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import lidar.lidar_detection as lidar

# ── LOGGING ──────────────────────────────────────────────────────────────────
# DEBUG → fichier (tout, y compris les logs internes du module lidar)
# INFO+ → console (lisible)
log_path = f"lidar_debug_{int(time.time())}.log"
logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s.%(msecs)03d [%(name)s] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S",
    handlers=[
        logging.FileHandler(log_path),
        logging.StreamHandler(sys.stdout),
    ],
)
logging.getLogger().handlers[1].setLevel(logging.INFO)

# ── POSE ROBOT ───────────────────────────────────────────────────────────────
# Centre du terrain test 1040×1040
ROBOT_X     = 520
ROBOT_Y     = 520
ROBOT_THETA = 0.0   # 0 = câble LiDAR pointe vers +y (avant du robot)

lidar.update_robot_pose(ROBOT_X, ROBOT_Y, ROBOT_THETA)
lidar.start()
time.sleep(1.5)

# ── CSV ──────────────────────────────────────────────────────────────────────
csv_path = f"lidar_run_{int(time.time())}.csv"
f = open(csv_path, "w", newline="")
w = csv.writer(f)
w.writerow(["t_s", "detected", "devant_mm", "droite_mm",
            "dist_mm", "conf", "fps"])

# ── HEADER ───────────────────────────────────────────────────────────────────
print(f"\n  Log DEBUG → {log_path}")
print(f"  CSV       → {csv_path}")
print(f"  Robot     → ({ROBOT_X}, {ROBOT_Y}, θ={math.degrees(ROBOT_THETA):.0f}°)")
print(f"  Terrain   → {lidar.FIELD_W}×{lidar.FIELD_H}mm")
print(f"  Détection → {lidar.MIN_DIST_MM}–{lidar.DETECT_DIST_MM}mm")
print(f"  Offset    → {lidar.ANGLE_OFFSET_DEG}°")
print()
print(f"  {'t':>5} {'devant':>7} {'droite':>7} {'dist':>6} {'conf':>5} {'fps':>5}")
print(f"  {'-'*45}")

# ── BOUCLE ───────────────────────────────────────────────────────────────────
t0 = time.time()
last_fps_t = t0
frame_count = 0
fps = 0.0
det_count = 0
total_count = 0

# Stats pour résumé
dists = []
xs = []
ys = []

try:
    while True:
        now = time.time()
        t_rel = now - t0
        frame_count += 1
        total_count += 1

        if now - last_fps_t >= 1.0:
            fps = frame_count / (now - last_fps_t)
            frame_count = 0
            last_fps_t = now

        opp = lidar.get_opponent()

        if opp:
            det_count += 1
            x, y, conf = opp
            dx, dy = x - ROBOT_X, y - ROBOT_Y

            # Repère robot (marche quel que soit ROBOT_THETA)
            cos_t = math.cos(ROBOT_THETA)
            sin_t = math.sin(ROBOT_THETA)
            devant =  dx * sin_t + dy * cos_t
            droite =  dx * cos_t - dy * sin_t
            dist = math.hypot(devant, droite)

            dists.append(dist)
            xs.append(devant)
            ys.append(droite)

            print(f"  {t_rel:5.1f} {devant:+7.0f} {droite:+7.0f} {dist:6.0f} "
                  f"{conf:5.2f} {fps:5.1f}")
            w.writerow([f"{t_rel:.2f}", 1, f"{devant:.1f}", f"{droite:.1f}",
                        f"{dist:.0f}", f"{conf:.2f}", f"{fps:.1f}"])
        else:
            print(f"  {t_rel:5.1f}     ---     ---    ---   ---  {fps:5.1f}")
            w.writerow([f"{t_rel:.2f}", 0, "", "", "", "", f"{fps:.1f}"])

        f.flush()
        time.sleep(0.1)

except KeyboardInterrupt:
    pass
finally:
    f.close()
    lidar.stop()

    # ── RÉSUMÉ ───────────────────────────────────────────────────────────────
    rate = 100 * det_count / max(total_count, 1)
    print(f"\n  ─── Résumé ({t_rel:.0f}s) ───")
    print(f"  Détections : {det_count}/{total_count} ({rate:.0f}%)")

    if len(dists) > 1:
        import statistics
        print(f"  Distance   : moy={statistics.mean(dists):.0f}mm  "
              f"σ={statistics.stdev(dists):.0f}mm")
        print(f"  Devant     : moy={statistics.mean(xs):.0f}mm  "
              f"σ={statistics.stdev(xs):.0f}mm")
        print(f"  Droite     : moy={statistics.mean(ys):.0f}mm  "
              f"σ={statistics.stdev(ys):.0f}mm")
    elif len(dists) == 1:
        print(f"  Distance   : {dists[0]:.0f}mm (1 mesure)")

    print(f"  CSV        : {csv_path}")
    print(f"  Log        : {log_path}")