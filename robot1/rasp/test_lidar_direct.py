#!/usr/bin/env python3
"""
Test LIDAR - Accès direct aux points (version simple)
Récupère la liste des 360 points en tuples (angle, distance)
"""

import sys
import time
from pathlib import Path

# Ajouter les chemins nécessaires
WORKSPACE = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(WORKSPACE / "robot1" / "rasp"))
sys.path.insert(0, str(WORKSPACE / "robot1" / "rasp" / "simu"))

from utils import create_com
from simu.lidar_manager import LidarManager


# Connexion
com = create_com()
if not com:
    print("❌ Erreur de connexion")
    exit(1)

lidar = LidarManager(com)
print("✅ LIDAR initialisé\n")

# Attendre premier scan
while lidar.scans_received == 0:
    time.sleep(0.01)

print("Premier scan reçu!\n")

try:
    while True:
        # Attendre nouveau scan
        old = lidar.scans_received
        while lidar.scans_received == old:
            time.sleep(0.01)
        
        # Récupérer les points: liste de tuples (angle, distance)
        points = lidar.points  # [(0, dist0), (1, dist1), ..., (359, dist359)]
        
        print(f"Scan #{lidar.scans_received}: {len(points)} points")
        print(f"  Exemples: {points[0]}, {points[90]}, {points[180]}, {points[270]}")
        
except KeyboardInterrupt:
    print("\n🛑 Arrêt")
    com.close()
