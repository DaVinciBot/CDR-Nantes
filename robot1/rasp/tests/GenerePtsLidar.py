"""Test de detection.py sur un scan fictif au format acquisition.py.
Lancer depuis la racine CDR-Nantes : python test_detection_scan.py
Aucun capteur LiDAR ni moteur n'est utilise.
"""
import math
import time
from robot1.rasp.vision import detection

# Pose du LiDAR dans le repere du terrain (mm, radians).
ROBOT_X, ROBOT_Y, ROBOT_THETA = 700.0, 1000.0, 0.0

# Convertit une position (x, y) du terrain en mesure LiDAR (qualite, angle, distance).
# Convention de detection.py : angle 0 = +Y ; angle 90 = +X.
def point_lidar(x, y, qualite=15):
    dx, dy = x - ROBOT_X, y - ROBOT_Y
    angle = math.degrees(math.atan2(dx, dy)) % 360.0
    distance = math.hypot(dx, dy)
    return (qualite, round(angle, 3), round(distance, 3))

# 1) Pierre centree vers (1300, 1200), 7 points sur une face de 60 mm.
pierre = [point_lidar(1300, y) for y in range(1170, 1231, 10)]
# 2) Adversaire centre vers (1700, 1000), 11 points sur une face de 200 mm.
adversaire = [point_lidar(1700, y) for y in range(900, 1101, 20)]
# 3) Obstacle centre vers (1195, 600), 7 points sur une face de 90 mm.
obstacle = [point_lidar(1195, y) for y in range(555, 646, 15)]
# 4) Bruit : 2 points isoles, 1 mauvaise qualite, 1 distance nulle.
bruit = [point_lidar(1800, 1700), point_lidar(1820, 1720),
         (3, 110.0, 600.0), (15, 70.0, 0.0)]
scan = pierre + adversaire + obstacle + bruit

# Parametres uniquement pour la simulation. 150 mm = rayon fictif du robot.
detection.configure(
    stone_positions=[(1300, 1200)],
    cluster_gap_mm=55,
    opponent_min_span_mm=100,
    opponent_max_span_mm=450,
    robot_radius_mm=150,
    avoidance_margin_mm=50,
    stone_physical_radius_mm=50,
)
detection.update_robot_pose(ROBOT_X, ROBOT_Y, ROBOT_THETA)

print(f"SCAN : {len(scan)} mesures (qualite, angle_degres, distance_mm)")
print("8 premieres mesures :")
for mesure in scan[:8]:
    print(" ", mesure)

# Le suivi adverse requiert deux scans concordants.
detection.process_scan(scan)
time.sleep(0.03)
detection.process_scan(scan)

print("\nGROUPES :")
for c in detection.get_clusters():
    print(f"  {c.categorie:22} centre=({c.x_mm:.0f}, {c.y_mm:.0f}) "
          f"longueur={c.longueur_mm:.0f} mm, largeur={c.largeur_mm:.0f} mm, "
          f"points={len(c.points)}")

print("\nADVERSAIRE :", detection.get_opponent())
print("DETAIL ADVERSAIRE :", detection.get_opponent_details())
print("\nOBSTACLES :", [(c.categorie, round(c.x_mm), round(c.y_mm))
                         for c in detection.get_obstacles()])
print("\nOBJETS POUR EVITEMENT :")
for obj in detection.get_avoidance_objects():
    print(" ", {k: (round(v, 1) if isinstance(v, float) else v)
                for k, v in obj.items()})

categories = {c.categorie for c in detection.get_clusters()}
assert 'pierre_possible' in categories, 'Pierre non detectee'
assert 'adversaire_possible' in categories, 'Adversaire non detecte'
assert 'obstacle' in categories, 'Obstacle non detecte'
assert detection.get_opponent() is not None, 'Adversaire non confirme'
print("\nTEST REUSSI : pierre, adversaire et obstacle identifies.")
