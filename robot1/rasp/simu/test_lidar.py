#!/usr/bin/env python3
"""
Test du système LIDAR dans Webots
Réception et affichage des données LIDAR - 360 points (comme RPLIDAR A2M8)
"""

import sys
import time
import struct
import math
from pathlib import Path

# Configuration paths
WORKSPACE = Path(__file__).resolve().parent.parent.parent.parent
sys.path.insert(0, str(WORKSPACE / "robot1" / "rasp"))
sys.path.insert(0, str(WORKSPACE / "common" / "usb_com" / "python"))

from robot_context import create_com
from loader import loader

Messages = loader.load_class('usb_com', 'Messages')

# Buffer pour reconstituer les 360 points à partir des 4 messages
lidar_buffer = {
    'part1': None,  # Angles 0-89
    'part2': None,  # Angles 90-179
    'part3': None,  # Angles 180-269
    'part4': None,  # Angles 270-359 + timestamp
}

def parse_lidar_part(payload, part_num):
    """
    Décode une partie des données LIDAR (90 points).
    
    part_num: 1, 2, 3, ou 4
    Retourne une liste de 90 distances (part 1-3) ou un dict avec distances+timestamp (part 4)
    """
    if part_num < 4:
        # Parties 1-3: 90 uint16 (180 bytes)
        expected_size = 90 * 2
        if len(payload) != expected_size:
            print(f"⚠️  LIDAR Part{part_num}: Taille incorrecte ({len(payload)} != {expected_size})")
            return None
        
        data = struct.unpack('<90H', payload)
        return list(data)
    else:
        # Partie 4: 90 uint16 + 1 uint32 timestamp (180 + 4 = 184 bytes)
        expected_size = 90 * 2 + 4
        if len(payload) != expected_size:
            print(f"⚠️  LIDAR Part4: Taille incorrecte ({len(payload)} != {expected_size})")
            return None
        
        data = struct.unpack('<90HI', payload)
        distances = list(data[0:90])
        timestamp = data[90]
        return {'distances': distances, 'timestamp': timestamp}


def display_lidar_ascii(distances):
    """
    Affichage ASCII art du LIDAR (vue du dessus, robot au centre)
    Format: 360 points, 1° de résolution (comme RPLIDAR A2M8)
    """
    print("\n╔═══════════════════════════════════════════════════════════════════╗")
    print("║               LIDAR SCAN - 360 points (RPLIDAR A2M8)             ║")
    print("╠═══════════════════════════════════════════════════════════════════╣")
    
    # Regrouper par secteurs de 45° pour affichage condensé
    sector_names = [
        "  0°- 45° [AVANT-DROITE]",
        " 45°- 90° [DROITE-AVANT]",
        " 90°-135° [DROITE-ARRIERE]",
        "135°-180° [ARRIERE-DROITE]",
        "180°-225° [ARRIERE-GAUCHE]",
        "225°-270° [GAUCHE-ARRIERE]",
        "270°-315° [GAUCHE-AVANT]",
        "315°-360° [AVANT-GAUCHE]"
    ]
    
    max_display_range = 3000  # 3m pour l'affichage
    bar_width = 40
    
    for sector_idx in range(8):
        # Récupérer les 45 points du secteur
        start_angle = sector_idx * 45
        end_angle = start_angle + 45
        sector_points = distances[start_angle:end_angle]
        
        # Filtrer les points valides (> 0)
        valid_points = [d for d in sector_points if d > 0]
        
        if valid_points:
            min_dist = min(valid_points)
            max_dist = max(valid_points)
            avg_dist = sum(valid_points) // len(valid_points)
            num_valid = len(valid_points)
        else:
            min_dist = 0
            max_dist = 0
            avg_dist = 0
            num_valid = 0
        
        # Construction de la barre visuelle
        if avg_dist == 0:
            bar = '·' * bar_width
            stats = "Aucune détection"
        else:
            avg_bar = int((avg_dist / max_display_range) * bar_width)
            avg_bar = min(avg_bar, bar_width)
            bar = '█' * avg_bar + '·' * (bar_width - avg_bar)
            stats = f"min:{min_dist:4d} avg:{avg_dist:4d} max:{max_dist:4d}mm ({num_valid:2d}/45pts)"
        
        print(f"║ {sector_names[sector_idx]}: {bar} {stats} ║")
    
    print("╚═══════════════════════════════════════════════════════════════════╝")
    print("Légende: █ = distance moyenne | · = hors portée ou pas de détection")

def process_complete_scan():
    """Traite un scan complet une fois les 4 parties reçues"""
    # Vérifier que toutes les parties sont présentes
    if None in lidar_buffer.values():
        return
    
    # Reconstituer les 360 points
    distances = (
        lidar_buffer['part1'] +
        lidar_buffer['part2'] +
        lidar_buffer['part3'] +
        lidar_buffer['part4']['distances']
    )
    timestamp = lidar_buffer['part4']['timestamp']
    
    # Réinitialiser le buffer
    lidar_buffer['part1'] = None
    lidar_buffer['part2'] = None
    lidar_buffer['part3'] = None
    lidar_buffer['part4'] = None
    
    # Affichage des données
    display_lidar_ascii(distances)
    
    # Analyse des obstacles proches (< 500mm)
    obstacles_angles = []
    for angle in range(360):
        dist = distances[angle]
        if 0 < dist < 500:  # Obstacle entre 0 et 50cm
            obstacles_angles.append((angle, dist))
    
    if obstacles_angles:
        print("\n⚠️  OBSTACLES PROCHES détectés (<500mm) :")
        # Regrouper les angles consécutifs
        if len(obstacles_angles) <= 10:
            for angle, dist in obstacles_angles:
                print(f"   • Angle {angle:3d}° : {dist}mm")
        else:
            print(f"   • {len(obstacles_angles)} points d'obstacles détectés")
            # Afficher les 5 plus proches
            obstacles_sorted = sorted(obstacles_angles, key=lambda x: x[1])
            print("   Les 5 plus proches :")
            for angle, dist in obstacles_sorted[:5]:
                print(f"      - Angle {angle:3d}° : {dist}mm")
    else:
        print("\n✅ Aucun obstacle proche (<500mm)")

    
    # Statistiques globales
    valid_distances = [d for d in distances if d > 0]
    if valid_distances:
        min_global = min(valid_distances)
        max_global = max(valid_distances)
        avg_global = sum(valid_distances) // len(valid_distances)
        print(f"\n📊 Statistiques: {len(valid_distances)}/360 points valides")
        print(f"   Min: {min_global}mm | Moy: {avg_global}mm | Max: {max_global}mm")
    
    print(f"\n⏱️  Timestamp: {timestamp}ms\n")

# Callbacks pour chaque partie
def on_lidar_part1(payload):
    result = parse_lidar_part(payload, 1)
    if result:
        lidar_buffer['part1'] = result
        process_complete_scan()

def on_lidar_part2(payload):
    result = parse_lidar_part(payload, 2)
    if result:
        lidar_buffer['part2'] = result
        process_complete_scan()

def on_lidar_part3(payload):
    result = parse_lidar_part(payload, 3)
    if result:
        lidar_buffer['part3'] = result
        process_complete_scan()

def on_lidar_part4(payload):
    result = parse_lidar_part(payload, 4)
    if result:
        lidar_buffer['part4'] = result
        process_complete_scan()

def main():
    print("🔧 Initialisation test LIDAR Webots...")
    
    # Connexion au port série virtuel (COM2 pour Webots)
    com = create_com()
    
    if com is None:
        print("❌ Impossible de créer la communication")
        return
    
    # Enregistrement des 4 callbacks LIDAR (4 parties pour les 360 points)
    com.add_callback(on_lidar_part1, Messages.LIDAR_SCAN_PART1.value)
    com.add_callback(on_lidar_part2, Messages.LIDAR_SCAN_PART2.value)
    com.add_callback(on_lidar_part3, Messages.LIDAR_SCAN_PART3.value)
    com.add_callback(on_lidar_part4, Messages.LIDAR_SCAN_PART4.value)
    print(f"✅ Callbacks LIDAR enregistrés (IDs: {Messages.LIDAR_SCAN_PART1.value}-{Messages.LIDAR_SCAN_PART4.value})")
    
    print("\n📡 En attente de données LIDAR (360 points, comme RPLIDAR A2M8)...")
    print("   Les données sont envoyées en 4 parties (90 points chacune)")
    print("   (Lancez la simulation Webots avec le LIDAR activé)\n")
    
    try:
        # Boucle d'écoute - les callbacks sont appelés automatiquement par le thread de réception
        while True:
            time.sleep(0.1)  # 10Hz - Le thread interne gère la réception
    
    except KeyboardInterrupt:
        print("\n\n🛑 Arrêt du test LIDAR")
    finally:
        com.close()

if __name__ == '__main__':
    main()
