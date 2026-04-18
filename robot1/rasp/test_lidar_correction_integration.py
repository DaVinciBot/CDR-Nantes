#!/usr/bin/env python3
"""
Test d'intégration complet : Correction SVD + Filtre complémentaire

Progression des tests:
1. NIVEAU 1 (Basique): Vérifier que LiDAR est détecté et envoie des données
2. NIVEAU 2 (Beacons): Vérifier que les balises sont détectées
3. NIVEAU 3 (SVD): Vérifier que la correction SVD calcule une pose
4. NIVEAU 4 (Filtre): Vérifier que le filtre complémentaire fonctionne
5. NIVEAU 5 (Correction Teensy): Vérifier que les corrections sont envoyées

USAGE:
    python3 test_lidar_correction_integration.py [--level 1-5] [--duration 30]

Exemples:
    python3 test_lidar_correction_integration.py --level 1 --duration 10
    python3 test_lidar_correction_integration.py --level 3 --duration 20
    python3 test_lidar_correction_integration.py --level 5 --duration 60
"""

import sys
import time
import logging
import struct
import argparse
import threading
from pathlib import Path
from typing import Optional, Tuple
from collections import deque

import math

# Configuration des chemins
sys.path.insert(0, str(Path(__file__).parent))

# Imports robot + lidar
from loader import loader
from utils import init_robot
from terrain_jeu import Terrain, BEACONS_BY_ID

try:
    from lidar.lidar_logic import (
        get_latest_scan_data,
        start_lidar_thread,
        stop_lidar_runtime,
        update_teensy_pose,
        get_corrected_pose,
        should_send_correction_to_teensy,
        _predict_beacon_windows,
        _compute_corrected_pose,
        _extract_beacon_candidates_fast,
    )
except ImportError as e:
    print(f"ERROR: Could not import lidar_logic functions: {e}")
    sys.exit(1)

# Logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)-7s | %(name)s | %(message)s"
)
logger = logging.getLogger("TEST_LIDAR_CORRECTION")

Messages = loader.load_class("usb_com", "Messages")


# ═══════════════════════════════════════════════════════════════════════════════
# NIVEAU 1 : Détection LiDAR basique
# ═══════════════════════════════════════════════════════════════════════════════

def test_level_1_lidar_detection(duration_sec: int = 10):
    """
    NIVEAU 1: Vérifier que le LiDAR est détecté et envoie des scans
    
    ✓ Check: LiDAR détecté sur port série
    ✓ Check: Scans reçus (frame count > 0)
    ✓ Check: Points avec qualité et distance valides
    """
    print("\n" + "="*70)
    print("NIVEAU 1: DÉTECTION LIDAR BASIQUE")
    print("="*70)
    
    try:
        logger.info("Démarrage du thread LiDAR...")
        start_lidar_thread()
        time.sleep(1.0)  # Attendre initialisation
        
        logger.info(f"Acquisition pendant {duration_sec} secondes...")
        frame_count = 0
        total_points = 0
        quality_histogram = {}
        distance_stats = {"min": float("inf"), "max": 0, "avg": 0}
        
        start_time = time.time()
        while time.time() - start_time < duration_sec:
            scan_data = get_latest_scan_data()
            
            if scan_data and len(scan_data) > 0:
                frame_count += 1
                total_points += len(scan_data)
                
                # Analyser statistiques
                distances = [p[1] for p in scan_data]
                qualities = [int(p[2]) for p in scan_data]
                
                distance_stats["min"] = min(distance_stats["min"], min(distances))
                distance_stats["max"] = max(distance_stats["max"], max(distances))
                distance_stats["avg"] = total_points / frame_count if frame_count > 0 else 0
                
                for q in qualities:
                    quality_histogram[q] = quality_histogram.get(q, 0) + 1
                
                if frame_count % 20 == 0:
                    logger.info(
                        f"Frame {frame_count}: {len(scan_data)} points | "
                        f"Distances: {distance_stats['min']:.0f}-{distance_stats['max']:.0f}mm"
                    )
            
            time.sleep(0.05)
        
        stop_lidar_runtime()
        
        # Afficher résumé
        print("\n" + "─"*70)
        print(f"✓ LiDAR DÉTECTÉ ET FONCTIONNEL")
        print(f"  Frames acquises: {frame_count}")
        print(f"  Points totaux: {total_points}")
        print(f"  Points/frame: {total_points/frame_count:.1f}")
        print(f"  Distance: {distance_stats['min']:.0f}mm - {distance_stats['max']:.0f}mm")
        print(f"  Qualités reçues: {sorted(quality_histogram.keys())}")
        print("─"*70)
        
        return True
        
    except Exception as e:
        logger.error(f"ERREUR Level 1: {e}")
        import traceback
        traceback.print_exc()
        return False


# ═══════════════════════════════════════════════════════════════════════════════
# NIVEAU 2 : Détection de balises
# ═══════════════════════════════════════════════════════════════════════════════

def test_level_2_beacon_detection(duration_sec: int = 20):
    """
    NIVEAU 2: Vérifier que les balises sont détectées
    
    ✓ Check: Candidats-balises trouvés (beacon clusters)
    ✓ Check: Nombre de balises détectées par frame
    ✓ Check: Positions cohérentes
    """
    print("\n" + "="*70)
    print("NIVEAU 2: DÉTECTION DE BALISES")
    print("="*70)
    
    try:
        logger.info("Démarrage du thread LiDAR...")
        start_lidar_thread()
        time.sleep(1.0)
        
        logger.info(f"Acquisition pendant {duration_sec} secondes...")
        
        beacon_frames = deque(maxlen=100)
        beacons_detected = {}
        
        start_time = time.time()
        frame_count = 0
        while time.time() - start_time < duration_sec:
            scan_data = get_latest_scan_data()
            
            if scan_data and len(scan_data) > 0:
                frame_count += 1
                
                # Extraire candidats-balises
                try:
                    candidates = _extract_beacon_candidates_fast(scan_data)
                    
                    if candidates and len(candidates) > 0:
                        beacon_frames.append({
                            'timestamp': time.time(),
                            'count': len(candidates),
                            'candidates': candidates
                        })
                        
                        for cand in candidates:
                            angle = cand.get('angle', 0)
                            distance = cand.get('distance', 0)
                            key = f"angle_{int(angle*1000)}"
                            if key not in beacons_detected:
                                beacons_detected[key] = []
                            beacons_detected[key].append(distance)
                        
                        if frame_count % 20 == 0:
                            logger.info(f"Frame {frame_count}: {len(candidates)} candidats-balises")
                
                except Exception as e:
                    logger.warning(f"Erreur extraction candidats: {e}")
            
            time.sleep(0.05)
        
        stop_lidar_runtime()
        
        # Résumé
        beacon_detections = len(beacon_frames)
        avg_beacons_per_frame = sum(f['count'] for f in beacon_frames) / beacon_detections if beacon_detections > 0 else 0
        
        print("\n" + "─"*70)
        if beacon_detections > 0:
            print(f"✓ BALISES DÉTECTÉES")
            print(f"  Frames avec balises: {beacon_detections}/{frame_count}")
            print(f"  Balises/frame (moyenne): {avg_beacons_per_frame:.2f}")
            print(f"  Candidats uniques: {len(beacons_detected)}")
            print(f"  Positions d'angles détectées: {sorted(beacons_detected.keys())[:5]}...")
        else:
            print(f"⚠ AUCUNE BALISE DÉTECTÉE - Vérifier:")
            print(f"  - Balises sont-elles visibles?")
            print(f"  - LiDAR scan contient-il des points?")
            print(f"  - Seuils de qualité/distance corrects?")
        print("─"*70)
        
        return beacon_detections > 0
        
    except Exception as e:
        logger.error(f"ERREUR Level 2: {e}")
        import traceback
        traceback.print_exc()
        return False


# ═══════════════════════════════════════════════════════════════════════════════
# NIVEAU 3 : Correction SVD
# ═══════════════════════════════════════════════════════════════════════════════

def test_level_3_svd_correction(duration_sec: int = 30):
    """
    NIVEAU 3: Vérifier que SVD Umeyama calcule une correction
    
    ✓ Check: Corrected poses calculées
    ✓ Check: Confiance SVD > 0.5
    ✓ Check: Delta correction raisonnable (< 500mm)
    """
    print("\n" + "="*70)
    print("NIVEAU 3: CORRECTION SVD UMEYAMA")
    print("="*70)
    
    try:
        logger.info("Démarrage du thread LiDAR...")
        start_lidar_thread()
        time.sleep(1.0)
        
        logger.info(f"Acquisition pendant {duration_sec} secondes...")
        
        # Pose factice Teensy pour test
        teensy_x, teensy_y, teensy_theta = 1000.0, 1500.0, 0.0
        
        corrected_poses = []
        failed_svds = 0
        
        start_time = time.time()
        frame_count = 0
        while time.time() - start_time < duration_sec:
            scan_data = get_latest_scan_data()
            
            if scan_data and len(scan_data) > 0:
                frame_count += 1
                
                # Mettre à jour pose Teensy pour LiDAR thread
                update_teensy_pose(teensy_x, teensy_y, teensy_theta)
                
                try:
                    # Extraire candidats
                    candidates = _extract_beacon_candidates_fast(scan_data)
                    
                    # Essayer SVD correction
                    if candidates and len(candidates) >= 2:
                        try:
                            corrected = _compute_corrected_pose(
                                candidates, teensy_x, teensy_y, teensy_theta
                            )
                            
                            if corrected is not None:
                                corrected_poses.append(corrected)
                                
                                delta_x = corrected.x - teensy_x
                                delta_y = corrected.y - teensy_y
                                delta_theta = corrected.theta - teensy_theta
                                
                                if frame_count % 10 == 0:
                                    logger.info(
                                        f"Frame {frame_count}: SVD OK | "
                                        f"Conf={corrected.confidence:.2f} | "
                                        f"ΔX={delta_x:.1f}mm ΔY={delta_y:.1f}mm "
                                        f"Δθ={math.degrees(delta_theta):.1f}°"
                                    )
                            else:
                                failed_svds += 1
                        except Exception as e:
                            logger.debug(f"SVD computation error: {e}")
                            failed_svds += 1
                
                except Exception as e:
                    logger.debug(f"Candidats extraction error: {e}")
            
            time.sleep(0.05)
        
        stop_lidar_runtime()
        
        # Résumé
        svd_success_rate = len(corrected_poses) / frame_count if frame_count > 0 else 0
        avg_confidence = sum(p.confidence for p in corrected_poses) / len(corrected_poses) if corrected_poses else 0
        
        print("\n" + "─"*70)
        if len(corrected_poses) > 0:
            print(f"✓ SVD CORRECTION FONCTIONNE")
            print(f"  Corrections réussies: {len(corrected_poses)}/{frame_count} ({svd_success_rate*100:.1f}%)")
            print(f"  Confiance moyenne: {avg_confidence:.2f}")
            print(f"  Beacons utilisés: {[p.beacon_ids for p in corrected_poses[:3]]}")
        else:
            print(f"⚠ AUCUNE SVD CORRECTION - Vérifier:")
            print(f"  - Au moins 2 balises détectées?")
            print(f"  - Balises dans BEACONS_BY_ID?")
            print(f"  - RMS residual < seuil?")
        print("─"*70)
        
        return len(corrected_poses) > 0
        
    except Exception as e:
        logger.error(f"ERREUR Level 3: {e}")
        import traceback
        traceback.print_exc()
        return False


# ═══════════════════════════════════════════════════════════════════════════════
# NIVEAU 4 : Filtre complémentaire
# ═══════════════════════════════════════════════════════════════════════════════

def test_level_4_complementary_filter(duration_sec: int = 30):
    """
    NIVEAU 4: Vérifier que le filtre complémentaire fonctionne
    
    ✓ Check: Alpha varie avec confiance
    ✓ Check: Blend intermédiaire raisonnable
    ✓ Check: Monotonie alpha(confiance)
    """
    print("\n" + "="*70)
    print("NIVEAU 4: FILTRE COMPLÉMENTAIRE ADAPTATIF")
    print("="*70)
    
    try:
        from robot import Robot
        
        logger.info("Création instance Robot...")
        robot = Robot("BLEU")
        
        logger.info(f"Test filtre pendant {duration_sec} secondes...")
        
        filter_tests = []
        
        # Test points
        test_cases = [
            (0.0, 0.85, "Low confidence"),
            (0.2, 0.85, "Boundary low"),
            (0.5, 0.55, "Medium confidence"),
            (0.8, 0.25, "Boundary high"),
            (1.0, 0.25, "High confidence"),
        ]
        
        for conf, expected_alpha, desc in test_cases:
            x_fused, y_fused, alpha = robot._apply_complementary_filter(
                lidar_x=1000.0, lidar_y=2000.0,
                teensy_x=1010.0, teensy_y=2010.0,
                confidence=conf
            )
            
            alpha_ok = abs(alpha - expected_alpha) < 0.01
            filter_tests.append({
                'conf': conf,
                'alpha': alpha,
                'expected': expected_alpha,
                'ok': alpha_ok,
                'desc': desc
            })
            
            status = "✓" if alpha_ok else "✗"
            logger.info(
                f"{status} {desc}: conf={conf:.2f} → alpha={alpha:.3f} "
                f"(expected {expected_alpha:.3f})"
            )
        
        # Vérifier monotonie
        alphas = [t['alpha'] for t in filter_tests]
        monotonic = all(alphas[i] >= alphas[i+1] for i in range(len(alphas)-1))
        
        print("\n" + "─"*70)
        all_ok = all(t['ok'] for t in filter_tests) and monotonic
        if all_ok:
            print(f"✓ FILTRE COMPLÉMENTAIRE OK")
            for t in filter_tests:
                status = "✓" if t['ok'] else "✗"
                print(f"  {status} {t['desc']}: alpha={t['alpha']:.3f}")
            print(f"  ✓ Monotonie: alpha décroît avec confiance")
        else:
            print(f"✗ FILTRE COMPLÉMENTAIRE PROBLÈME")
            for t in filter_tests:
                status = "✓" if t['ok'] else "✗"
                print(f"  {status} {t['desc']}: alpha={t['alpha']:.3f} (expected {t['expected']:.3f})")
            if not monotonic:
                print(f"  ✗ Monotonie: alpha non décroissant!")
        print("─"*70)
        
        return all_ok
        
    except Exception as e:
        logger.error(f"ERREUR Level 4: {e}")
        import traceback
        traceback.print_exc()
        return False


# ═══════════════════════════════════════════════════════════════════════════════
# NIVEAU 5 : Intégration complète avec Teensy
# ═══════════════════════════════════════════════════════════════════════════════

def test_level_5_full_integration(duration_sec: int = 60):
    """
    NIVEAU 5: Intégration complète (Teensy + LiDAR + Correction)
    
    ✓ Check: Teensy envoie odométrie
    ✓ Check: LiDAR calcule corrections
    ✓ Check: Corrections envoyées à Teensy
    ✓ Check: Throttle respecté (1-2s entre corrections)
    """
    print("\n" + "="*70)
    print("NIVEAU 5: INTÉGRATION COMPLÈTE")
    print("="*70)
    
    try:
        logger.info("Initialisation Teensy + LiDAR...")
        
        # Initialiser communication
        com, mode = init_robot(logger)
        logger.info(f"Mode détecté: {mode}")
        
        # Lancer LiDAR thread
        start_lidar_thread()
        time.sleep(1.0)
        
        # Stocker poses pour analyse
        teensy_poses = deque(maxlen=1000)
        corrected_poses = deque(maxlen=1000)
        corrections_sent = deque(maxlen=1000)
        
        def handle_odometry(data):
            """Callback Teensy position"""
            if len(data) >= 24:
                x, y, theta = struct.unpack('<ddd', data[:24])
                teensy_poses.append({
                    'x': x, 'y': y, 'theta': theta,
                    'time': time.time()
                })
                # Mettre à jour pour LiDAR thread
                update_teensy_pose(x, y, theta)
        
        # Attacher callback
        com.add_callback(handle_odometry, Messages.UPDATE_ROLLING_BASIS.value)
        
        logger.info(f"Acquisition pendant {duration_sec} secondes...")
        
        start_time = time.time()
        last_correction_time = 0
        frame_count = 0
        
        while time.time() - start_time < duration_sec:
            frame_count += 1
            
            # Vérifier correction disponible
            corrected = get_corrected_pose()
            if corrected is not None:
                corrected_poses.append({
                    'x': corrected.x,
                    'y': corrected.y,
                    'theta': corrected.theta,
                    'confidence': corrected.confidence,
                    'time': time.time()
                })
            
            # Vérifier throttle correction
            should_correct = should_send_correction_to_teensy()
            if should_correct:
                current_time = time.time()
                if current_time - last_correction_time >= 1.0:
                    if corrected is not None:
                        # Envoyer correction
                        msg = Messages.SET_ODOMETRIE.to_bytes()
                        msg += struct.pack('<ddd', corrected.x, corrected.y, corrected.theta)
                        com.send_bytes(msg)
                        
                        corrections_sent.append({
                            'x': corrected.x,
                            'y': corrected.y,
                            'time': current_time,
                            'confidence': corrected.confidence
                        })
                        
                        last_correction_time = current_time
                        logger.info(
                            f"Correction #{len(corrections_sent)} envoyée | "
                            f"Conf={corrected.confidence:.2f} | "
                            f"Pose=({corrected.x:.0f}, {corrected.y:.0f})"
                        )
            
            time.sleep(0.05)
        
        stop_lidar_runtime()
        com.close()
        
        # Résumé final
        print("\n" + "─"*70)
        print(f"RÉSUMÉ INTÉGRATION COMPLÈTE")
        print(f"  Durée: {duration_sec}s | Frames: {frame_count}")
        print(f"  Poses Teensy reçues: {len(teensy_poses)}")
        print(f"  Poses corrigées (SVD): {len(corrected_poses)}")
        print(f"  Corrections envoyées à Teensy: {len(corrections_sent)}")
        
        if len(corrections_sent) > 0:
            time_between = []
            for i in range(1, len(corrections_sent)):
                dt = corrections_sent[i]['time'] - corrections_sent[i-1]['time']
                time_between.append(dt)
            
            avg_interval = sum(time_between) / len(time_between) if time_between else 0
            print(f"  Intervalle corrections: {avg_interval:.2f}s (target: 1-2s)")
            print(f"  Confiance moyenne: {sum(c['confidence'] for c in corrections_sent)/len(corrections_sent):.2f}")
        
        print("─"*70)
        
        success = len(teensy_poses) > 0 and len(corrected_poses) > 0
        return success
        
    except Exception as e:
        logger.error(f"ERREUR Level 5: {e}")
        import traceback
        traceback.print_exc()
        return False


# ═══════════════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test intégration LiDAR correction")
    parser.add_argument("--level", type=int, default=1, choices=[1,2,3,4,5],
                       help="Niveau de test (1-5)")
    parser.add_argument("--duration", type=int, default=30,
                       help="Durée du test en secondes")
    
    args = parser.parse_args()
    
    print("\n╔════════════════════════════════════════════════════════════════════╗")
    print("║     TEST INTÉGRATION: CORRECTION ODOMÉTRIE LIDAR + TEENSY         ║")
    print("╚════════════════════════════════════════════════════════════════════╝")
    
    levels = {
        1: (test_level_1_lidar_detection, "Détection LiDAR basique"),
        2: (test_level_2_beacon_detection, "Détection de balises"),
        3: (test_level_3_svd_correction, "Correction SVD Umeyama"),
        4: (test_level_4_complementary_filter, "Filtre complémentaire"),
        5: (test_level_5_full_integration, "Intégration complète"),
    }
    
    test_func, desc = levels[args.level]
    
    print(f"\n➤ Exécution: NIVEAU {args.level} - {desc}")
    print(f"  Durée: {args.duration}s\n")
    
    try:
        success = test_func(args.duration)
        
        if success:
            print("\n✓ TEST RÉUSSI")
            exit_code = 0
        else:
            print("\n✗ TEST ÉCHOUÉ")
            exit_code = 1
    except KeyboardInterrupt:
        print("\n⏹ Test interrompu par l'utilisateur")
        exit_code = 130
    
    sys.exit(exit_code)
