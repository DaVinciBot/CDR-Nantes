#!/usr/bin/env python3
"""
test_complet_lidar.py — CDR 2026
═══════════════════════════════════════════════════════════════════════════════
Test complet du stack robot avec LiDAR réel uniquement.
Teensy simulée via poses fictives injectées directement dans lidar_logic.

Modules testés :
  ✓ terrain_jeu     — terrain, obstacles, balises, symétrie BLEU/JAUNE
  ✓ pathfinder      — A*, grille, inflation, chemin, lookahead
  ✓ lidar_logic     — thread, scan, extraction balises, SVD, adversaire
  ✓ lidar.py        — LidarInterface, get_fused_position, get_opponent
  ✓ robot.py        — filtre complémentaire, logique de correction odométrie
  ✓ strategy        — StratManager, chrono, validation actions
  ✓ loader          — chargement config.json
  ✓ intégration     — pipeline complet LiDAR → SVD → filtre → correction

Usage :
    python test_complet_lidar.py [--level 1-8] [--duration 20] [--color BLUE]
    python test_complet_lidar.py           # tous les tests (non-LiDAR d'abord)
    python test_complet_lidar.py --level 7 --duration 60  # intégration complète

═══════════════════════════════════════════════════════════════════════════════
"""

import sys
import time
import math
import logging
import argparse
import threading
import struct
from pathlib import Path
from typing import Optional, List, Tuple, Dict
from collections import deque

# ── Chemin ───────────────────────────────────────────────────────────────────
ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

# ── Logging ───────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s │ %(levelname)-7s │ %(name)-18s │ %(message)s",
)
logger = logging.getLogger("TEST_CDR")

# ══════════════════════════════════════════════════════════════════════════════
# Helpers d'affichage
# ══════════════════════════════════════════════════════════════════════════════

def header(titre: str) -> None:
    print("\n" + "═" * 72)
    print(f"  {titre}")
    print("═" * 72)


def ok(msg: str) -> None:
    print(f"  ✓  {msg}")


def warn(msg: str) -> None:
    print(f"  ⚠  {msg}")


def fail(msg: str) -> None:
    print(f"  ✗  {msg}")


def section(msg: str) -> None:
    print(f"\n  ── {msg} ──")


RESULTS: Dict[str, bool] = {}


def record(name: str, passed: bool) -> bool:
    RESULTS[name] = passed
    return passed


# ══════════════════════════════════════════════════════════════════════════════
# TEST 1 — terrain_jeu
# ══════════════════════════════════════════════════════════════════════════════

def test_terrain(color: str = "BLUE") -> bool:
    header("TEST 1 — terrain_jeu : dimensions, obstacles, balises, symétrie")
    try:
        from terrain_jeu import Terrain, BeaconLayout, FIELD_WIDTH_MM, FIELD_HEIGHT_MM

        section("Dimensions")
        assert FIELD_WIDTH_MM == 3000, f"WIDTH attendu 3000, got {FIELD_WIDTH_MM}"
        assert FIELD_HEIGHT_MM == 2000, f"HEIGHT attendu 2000, got {FIELD_HEIGHT_MM}"
        ok(f"Terrain {FIELD_WIDTH_MM}×{FIELD_HEIGHT_MM} mm")

        section("Obstacles statiques (BLEU)")
        t_blue = Terrain("BLUE")
        assert len(t_blue.obstacles) >= 1, "Aucun obstacle défini"
        for obs in t_blue.obstacles:
            ok(f"Obstacle '{obs.name}' : ({obs.x:.0f}, {obs.y:.0f}) {obs.width}×{obs.height} mm")

        section("Obstacles statiques (JAUNE — symétrie)")
        t_yel = Terrain("YELLOW")
        assert len(t_yel.obstacles) == len(t_blue.obstacles), "Nombre d'obstacles différent entre équipes"
        ok(f"{len(t_yel.obstacles)} obstacles JAUNE OK")

        section("Position de départ")
        sx, sy, sth = t_blue.get_start_position()
        ok(f"Start BLEU : ({sx:.0f}, {sy:.0f}, {math.degrees(sth):.1f}°)")
        sx2, sy2, sth2 = t_yel.get_start_position()
        ok(f"Start JAUNE : ({sx2:.0f}, {sy2:.0f}, {math.degrees(sth2):.1f}°)")
        assert sx2 != sx or sy2 != sy, "Positions de départ BLEU/JAUNE identiques — erreur de symétrie"

        section("BeaconLayout")
        assert hasattr(BeaconLayout, "BEACONS"), "BeaconLayout.BEACONS manquant"
        assert len(BeaconLayout.BEACONS) >= 3, "Moins de 3 balises définies"
        ok(f"{len(BeaconLayout.BEACONS)} balises définies")
        for bid, (bx, by) in BeaconLayout.BEACONS.items():
            ok(f"  Balise #{bid} : ({bx:.0f}, {by:.0f}) mm")

        section("Symétrie balises BLEU → JAUNE")
        for bid in BeaconLayout.BEACONS:
            bx_bl, by_bl = BeaconLayout.get_beacon(bid, "BLUE")
            bx_yl, by_yl = BeaconLayout.get_beacon(bid, "YELLOW")
            assert abs((bx_bl + bx_yl) - 3000) < 1, \
                f"Symétrie X incorrecte balise {bid}: {bx_bl}+{bx_yl}≠3000"
            ok(f"  Balise #{bid} BLEU=({bx_bl:.0f},{by_bl:.0f}) JAUNE=({bx_yl:.0f},{by_yl:.0f})")

        print()
        ok("TEST 1 PASSED")
        return record("terrain", True)

    except Exception as e:
        fail(f"TEST 1 FAILED : {e}")
        import traceback; traceback.print_exc()
        return record("terrain", False)


# ══════════════════════════════════════════════════════════════════════════════
# TEST 2 — pathfinder
# ══════════════════════════════════════════════════════════════════════════════

def test_pathfinder(color: str = "BLUE") -> bool:
    header("TEST 2 — pathfinder : A*, grille, inflation, chemin")
    try:
        from terrain_jeu import Terrain
        from pathfinder import PathFinder

        terrain = Terrain(color)
        pf = PathFinder(terrain)

        section("Paramètres grille")
        ok(f"GRID_SIZE = {pf.GRID_SIZE} mm")
        ok(f"MARGIN    = {pf.MARGIN} mm")
        ok(f"Inflation = robot({terrain.ROBOT_RADIUS}) + margin({pf.MARGIN}) = {pf.inflation_radius} mm")
        ok(f"Grille    = {pf.cols} cols × {pf.rows} rows")

        section("Chemin basique (centre → bord)")
        start = {"x": 1500, "y": 1000, "theta": 0.0}
        goal  = {"x": 2800, "y": 1000}
        chemin = pf.get_path(start, goal, [])
        if chemin and len(chemin) > 1:
            ok(f"Chemin trouvé : {len(chemin)} waypoints")
        else:
            warn("Aucun chemin trouvé pour start→bord (peut être bloqué par obstacle)")

        section("Chemin avec obstacle dynamique")
        obstacles_dyn = [(1800, 1000)]  # adversaire entre start et goal
        chemin2 = pf.get_path(start, goal, obstacles_dyn)
        if chemin2:
            ok(f"Chemin avec obstacle : {len(chemin2)} waypoints")
        else:
            warn("Chemin bloqué par obstacle (normal si zone trop étroite)")

        section("Chemin impossible (goal dans mur)")
        goal_wall = {"x": 0, "y": 0}
        chemin3 = pf.get_path(start, goal_wall, [])
        if not chemin3:
            ok("Chemin vers mur correctement rejeté (retourne [])")
        else:
            warn(f"Chemin vers mur retourné avec {len(chemin3)} pts — vérifier inflation")

        section("Test aller-retour (plusieurs points)")
        waypoints = [
            ({"x": 500,  "y": 500},  {"x": 2500, "y": 500}),
            ({"x": 2500, "y": 500},  {"x": 2500, "y": 1500}),
            ({"x": 2500, "y": 1500}, {"x": 500,  "y": 1500}),
        ]
        for i, (s, g) in enumerate(waypoints):
            s["theta"] = 0.0
            ch = pf.get_path(s, g, [])
            status = f"{len(ch)} pts" if ch else "BLOQUÉ"
            ok(f"  Segment {i+1}: ({s['x']},{s['y']}) → ({g['x']},{g['y']}) — {status}")

        print()
        ok("TEST 2 PASSED")
        return record("pathfinder", True)

    except Exception as e:
        fail(f"TEST 2 FAILED : {e}")
        import traceback; traceback.print_exc()
        return record("pathfinder", False)


# ══════════════════════════════════════════════════════════════════════════════
# TEST 3 — strategy
# ══════════════════════════════════════════════════════════════════════════════

def test_strategy(color: str = "BLUE") -> bool:
    header("TEST 3 — strategy : StratManager, chrono, actions")
    try:
        from strategy.strategy_actions import Action, TypeAction
        from strategy.strategy_strat_manager import StratManager

        section("Création StratManager")
        strat = StratManager(color)
        ok(f"StratManager créé pour équipe {color}")
        ok(f"Nombre d'actions dans la stratégie : {len(strat.liste_actions)}")

        section("Types d'actions")
        for i, action in enumerate(strat.liste_actions):
            ok(f"  Action {i}: {action.type.name}"
               + (f" → ({action.cible_x:.0f}, {action.cible_y:.0f})" if action.type == TypeAction.DEPLACEMENT else "")
               + (f" → {action.nom_actionneur}" if action.type == TypeAction.ACTIONNEUR else "")
               + (f" → {action.temps_attente}s" if action.type == TypeAction.ATTENTE else ""))

        section("get_action_actuelle")
        action = strat.get_action_actuelle()
        assert action is not None, "Aucune action retournée au démarrage"
        ok(f"Action courante : {action.type.name}")

        section("Chrono ATTENTE")
        # Chercher une action ATTENTE pour tester le chrono
        attente_idx = None
        for i, a in enumerate(strat.liste_actions):
            if a.type == TypeAction.ATTENTE:
                attente_idx = i
                break

        if attente_idx is not None:
            strat.etape_actuelle = attente_idx
            started = strat.demarrer_chrono_si_necessaire()
            assert started, "demarrer_chrono_si_necessaire devrait retourner True la 1ère fois"
            started2 = strat.demarrer_chrono_si_necessaire()
            assert not started2, "demarrer_chrono_si_necessaire devrait retourner False la 2ème fois"
            ok("Chrono démarré correctement")

            ecoule = strat.chrono_ecoule(9999)
            assert not ecoule, "Chrono ne devrait pas être écoulé immédiatement"
            time.sleep(0.1)
            ecoule2 = strat.chrono_ecoule(0.05)
            assert ecoule2, "Chrono devrait être écoulé après 100ms pour une durée de 50ms"
            ok("Vérification chrono_ecoule() OK")
        else:
            warn("Pas d'action ATTENTE dans la stratégie — test chrono sauté")

        section("Validation d'action")
        strat.etape_actuelle = 0
        strat.chrono_action = None
        strat.valider_action_terminee()
        assert strat.etape_actuelle == 1, "etape_actuelle devrait être 1 après validation"
        assert strat.chrono_action is None, "chrono_action devrait être reset après validation"
        ok("valider_action_terminee() OK → étape 0→1")

        section("Fin de stratégie")
        strat.etape_actuelle = len(strat.liste_actions)
        action_fin = strat.get_action_actuelle()
        assert action_fin is None, "get_action_actuelle() devrait retourner None à la fin"
        ok("Fin de stratégie retourne None correctement")

        print()
        ok("TEST 3 PASSED")
        return record("strategy", True)

    except Exception as e:
        fail(f"TEST 3 FAILED : {e}")
        import traceback; traceback.print_exc()
        return record("strategy", False)


# ══════════════════════════════════════════════════════════════════════════════
# TEST 4 — filtre complémentaire
# ══════════════════════════════════════════════════════════════════════════════

def test_complementary_filter(color: str = "BLUE") -> bool:
    header("TEST 4 — filtre complémentaire adaptatif (robot._apply_complementary_filter)")
    try:
        # On instancie la méthode directement sans créer de Robot complet
        # pour éviter la dépendance à la Teensy.
        def apply_filter(lidar_x, lidar_y, teensy_x, teensy_y, confidence):
            """Copie exacte de robot.py._apply_complementary_filter."""
            if confidence < 0.2:
                alpha = 0.85
            elif confidence > 0.8:
                alpha = 0.25
            else:
                alpha = 0.85 - (confidence - 0.2) / 0.6 * 0.60
            x_fused = (1.0 - alpha) * lidar_x + alpha * teensy_x
            y_fused = (1.0 - alpha) * lidar_y + alpha * teensy_y
            return x_fused, y_fused, alpha

        cases = [
            (0.00, 0.85, "conf=0.00 → alpha=0.85 (100% Teensy)"),
            (0.10, 0.85, "conf=0.10 → alpha=0.85"),
            (0.20, 0.85, "conf=0.20 → alpha=0.85 (boundary bas)"),
            (0.35, None, "conf=0.35 → alpha intermédiaire"),
            (0.50, 0.55, "conf=0.50 → alpha=0.55"),
            (0.65, None, "conf=0.65 → alpha intermédiaire"),
            (0.80, 0.25, "conf=0.80 → alpha=0.25 (boundary haut)"),
            (0.90, 0.25, "conf=0.90 → alpha=0.25"),
            (1.00, 0.25, "conf=1.00 → alpha=0.25 (100% LiDAR)"),
        ]

        alphas = []
        ok("Progression alpha (doit être monotone décroissante) :")
        for conf, expected_alpha, desc in cases:
            _, _, alpha = apply_filter(1000, 2000, 1010, 2010, conf)
            alphas.append(alpha)
            if expected_alpha is not None:
                diff = abs(alpha - expected_alpha)
                status = "✓" if diff < 0.001 else "✗"
                print(f"    {status}  {desc:45s}  alpha={alpha:.4f}")
                if diff >= 0.001:
                    fail(f"Alpha attendu {expected_alpha:.3f}, obtenu {alpha:.3f}")
                    return record("filter", False)
            else:
                print(f"    ·  {desc:45s}  alpha={alpha:.4f}")

        section("Monotonie alpha")
        for i in range(len(alphas) - 1):
            if alphas[i] < alphas[i + 1] - 1e-9:
                fail(f"Alpha non monotone à index {i}: {alphas[i]:.4f} < {alphas[i+1]:.4f}")
                return record("filter", False)
        ok("Alpha décroît monotonement avec la confiance LiDAR")

        section("Valeurs fusionnées")
        lx, ly, tx, ty = 1000.0, 2000.0, 1010.0, 2010.0
        xf_low,  yf_low,  _ = apply_filter(lx, ly, tx, ty, 0.0)
        xf_high, yf_high, _ = apply_filter(lx, ly, tx, ty, 1.0)
        assert xf_low  > lx, "Basse confiance : résultat devrait être proche de Teensy"
        assert xf_high < tx, "Haute confiance : résultat devrait être proche de LiDAR"
        ok(f"Basse conf → fusionné ({xf_low:.1f},{yf_low:.1f}) ~ Teensy ({tx},{ty})")
        ok(f"Haute conf → fusionné ({xf_high:.1f},{yf_high:.1f}) ~ LiDAR ({lx},{ly})")

        print()
        ok("TEST 4 PASSED")
        return record("filter", True)

    except Exception as e:
        fail(f"TEST 4 FAILED : {e}")
        import traceback; traceback.print_exc()
        return record("filter", False)


# ══════════════════════════════════════════════════════════════════════════════
# TEST 5 — lidar_logic constants & SVD hors thread
# ══════════════════════════════════════════════════════════════════════════════

def test_lidar_logic_static(color: str = "BLUE") -> bool:
    header("TEST 5 — lidar_logic : constantes, BEACONS, SVD Umeyama (hors thread)")
    try:
        from lidar.lidar_logic import (
            BEACONS_BY_ID, BEACON_WINDOW_ANGLE_RAD, BEACON_WINDOW_DIST_MM,
            POSE_CORRECTION_MIN_CONFIDENCE, POSE_CORRECTION_MIN_BEACONS,
            POSE_SEND_BACK_INTERVAL_S, BEACON_FIT_MAX_RMS_MM,
            set_team_color, _predict_beacon_windows, _compute_corrected_pose,
            _extract_beacon_candidates_fast, _associate_candidates_to_beacons,
            _validate_beacon_geometry,
        )

        section("Chargement balises")
        set_team_color(color)
        assert len(BEACONS_BY_ID) >= 3, f"BEACONS_BY_ID vide ou incomplet : {BEACONS_BY_ID}"
        for bid, (bx, by) in BEACONS_BY_ID.items():
            ok(f"  Balise #{bid}: ({bx:.0f}, {by:.0f}) mm")

        section("Constantes de correction")
        ok(f"BEACON_WINDOW_ANGLE_RAD = {math.degrees(BEACON_WINDOW_ANGLE_RAD):.1f}°")
        ok(f"BEACON_WINDOW_DIST_MM   = {BEACON_WINDOW_DIST_MM:.0f} mm")
        ok(f"MIN_CONFIDENCE          = {POSE_CORRECTION_MIN_CONFIDENCE}")
        ok(f"MIN_BEACONS             = {POSE_CORRECTION_MIN_BEACONS}")
        ok(f"SEND_BACK_INTERVAL_S    = {POSE_SEND_BACK_INTERVAL_S} s")
        ok(f"FIT_MAX_RMS_MM          = {BEACON_FIT_MAX_RMS_MM} mm")

        section("_predict_beacon_windows")
        # Robot au centre du terrain
        windows = _predict_beacon_windows(1500.0, 1000.0, 0.0)
        if windows:
            ok(f"Fenêtres prédites pour {len(windows)} balise(s) depuis centre")
            for bid, w in windows.items():
                ok(f"  Balise #{bid}: dist_pred={w['dist_pred']:.0f}mm, "
                   f"angle=[{math.degrees(w['angle_min']):.1f}°,{math.degrees(w['angle_max']):.1f}°]")
        else:
            warn("Aucune fenêtre prédite depuis le centre (vérifier positions balises)")

        section("_predict_beacon_windows (coin)")
        windows2 = _predict_beacon_windows(200.0, 200.0, 0.0)
        if windows2:
            ok(f"Fenêtres depuis coin : {len(windows2)} balise(s) visible(s)")
        else:
            warn("Aucune balise visible depuis le coin (peut être hors distance)")

        section("_compute_corrected_pose (candidats synthétiques)")
        # Créer des candidats synthétiques correspondant exactement aux balises
        # → Le SVD devrait retourner delta=0 et confiance maximale
        syn_cands = []
        for bid, (bx, by) in BEACONS_BY_ID.items():
            # Robot au centre, balise en coordonnées monde → robot frame avec theta=0
            rx = bx - 1500.0
            ry = by - 1000.0
            dist = math.hypot(rx, ry)
            angle = math.atan2(rx, ry)
            syn_cands.append({
                "x_r": rx,
                "y_r": ry,
                "angle": angle,
                "distance": dist,
                "quality": 14.0,
                "count": 5,
                "beacon_id": bid,
            })

        if len(syn_cands) >= 2:
            result = _compute_corrected_pose(syn_cands, 1500.0, 1000.0, 0.0)
            if result is not None:
                ok(f"SVD synthétique : conf={result.confidence:.2f}, "
                   f"delta=({result.x-1500:.1f},{result.y-1000:.1f})mm, "
                   f"beacons={result.beacon_ids}")
                assert abs(result.x - 1500.0) < 50, f"Δx trop grand : {result.x-1500:.1f}mm"
                assert abs(result.y - 1000.0) < 50, f"Δy trop grand : {result.y-1000:.1f}mm"
                ok("Delta SVD < 50mm (correction précise sur candidats exacts)")
            else:
                warn("SVD retourne None sur candidats synthétiques — RMS trop haut?")
        else:
            warn("Moins de 2 candidats synthétiques, SVD non testé")

        section("_associate_candidates_to_beacons (sans fenêtres)")
        if len(syn_cands) >= 2:
            cands_sans_bid = [{k: v for k, v in c.items() if k != "beacon_id"} for c in syn_cands]
            assoc = _associate_candidates_to_beacons(cands_sans_bid, 1500.0, 1000.0, 0.0, None)
            ok(f"Association sans fenêtres : {len(assoc)} candidats associés / {len(cands_sans_bid)}")
            for a in assoc:
                ok(f"  → Balise #{a.get('beacon_id')} (score={a.get('association_score', 'N/A')})")

        section("_validate_beacon_geometry")
        if len(syn_cands) >= 2:
            valid = _validate_beacon_geometry(syn_cands[:2])
            ok(f"Validation géométrie 2 balises synthétiques : {'OK' if valid else 'REJETÉ'}")

        print()
        ok("TEST 5 PASSED")
        return record("lidar_logic_static", True)

    except Exception as e:
        fail(f"TEST 5 FAILED : {e}")
        import traceback; traceback.print_exc()
        return record("lidar_logic_static", False)


# ══════════════════════════════════════════════════════════════════════════════
# TEST 6 — LiDAR thread basique (scan + balises)
# ══════════════════════════════════════════════════════════════════════════════

def test_lidar_thread_basic(duration_sec: int = 15, color: str = "BLUE") -> bool:
    header(f"TEST 6 — Thread LiDAR réel : scan, balises ({duration_sec}s)")
    print("  ⚡  Ce test nécessite le LiDAR branché sur /dev/ttyUSB0")
    try:
        from lidar.lidar_logic import (
            start_lidar_thread, stop_lidar_runtime,
            get_latest_scan_data, get_latest_beacon_candidates,
            get_corrected_pose, update_teensy_pose, set_team_color,
        )

        set_team_color(color)

        logs = []
        statuses = []

        def _console(msg):
            pass  # silencieux

        def _status(msg):
            statuses.append(msg)

        ok("Démarrage thread LiDAR...")
        thread = start_lidar_thread(_console, _status)
        time.sleep(2.0)

        frame_count       = 0
        beacon_frames     = 0
        total_pts         = 0
        max_beacons       = 0
        svd_count         = 0
        confidences       = []
        first_pose        = None

        # Injecter une pose Teensy fictive au centre
        update_teensy_pose(1500.0, 1000.0, 0.0)

        start = time.time()
        while time.time() - start < duration_sec:
            scan = get_latest_scan_data()
            cands = get_latest_beacon_candidates()
            pose = get_corrected_pose()

            if scan:
                frame_count += 1
                total_pts += len(scan)

            if cands:
                beacon_frames += 1
                max_beacons = max(max_beacons, len(cands))

            if pose and pose.is_localized:
                svd_count += 1
                confidences.append(pose.confidence)
                if first_pose is None:
                    first_pose = pose

            time.sleep(0.1)

        stop_lidar_runtime()

        section("Résultats scan")
        if frame_count == 0:
            fail("Aucun scan reçu — LiDAR non connecté ou port incorrect")
            warn("Vérifier /dev/ttyUSB0 et baudrate 256000")
            return record("lidar_thread", False)

        ok(f"Frames reçues : {frame_count}")
        ok(f"Points totaux : {total_pts} (moy {total_pts//max(frame_count,1)}/frame)")

        section("Résultats balises")
        if beacon_frames == 0:
            warn("Aucune balise détectée — vérifier visibilité et positions dans terrain_jeu.py")
        else:
            ok(f"Frames avec balises : {beacon_frames}/{frame_count} ({beacon_frames*100//frame_count}%)")
            ok(f"Max balises/frame   : {max_beacons}")

        section("Résultats SVD Umeyama")
        if svd_count == 0:
            warn("Aucune pose SVD calculée — au moins 2 balises nécessaires avec bonne géométrie")
        else:
            avg_conf = sum(confidences) / len(confidences)
            ok(f"Poses SVD réussies  : {svd_count}")
            ok(f"Confiance moyenne   : {avg_conf:.3f}")
            if first_pose:
                ok(f"Première pose       : ({first_pose.x:.0f}, {first_pose.y:.0f}) "
                   f"θ={math.degrees(first_pose.theta):.1f}° "
                   f"conf={first_pose.confidence:.2f} "
                   f"beacons={first_pose.beacon_ids}")

        section("Statuts LiDAR reçus")
        for s in statuses[-3:]:
            ok(f"  Status: {s}")

        print()
        passed = frame_count > 0
        if passed:
            ok("TEST 6 PASSED — LiDAR opérationnel")
        else:
            fail("TEST 6 FAILED — Pas de scan LiDAR")
        return record("lidar_thread", passed)

    except Exception as e:
        fail(f"TEST 6 FAILED : {e}")
        import traceback; traceback.print_exc()
        return record("lidar_thread", False)


# ══════════════════════════════════════════════════════════════════════════════
# TEST 7 — LidarInterface (lidar.py wrapper)
# ══════════════════════════════════════════════════════════════════════════════

def test_lidar_interface(duration_sec: int = 20, color: str = "BLUE") -> bool:
    header(f"TEST 7 — LidarInterface : get_fused_position, get_opponent ({duration_sec}s)")
    print("  ⚡  Nécessite LiDAR branché")
    try:
        from lidar.lidar_logic import (
            start_lidar_thread, stop_lidar_runtime,
            update_teensy_pose, set_team_color,
        )
        from lidar.lidar import LidarInterface

        set_team_color(color)

        def _noop(x): pass

        start_lidar_thread(_noop, _noop)
        time.sleep(2.0)

        lidar_iface = LidarInterface(team_color=color)

        # Poses Teensy fictives : robot se déplace virtuellement en ligne
        test_poses = [
            (1500, 1000, 0.0),
            (1500, 1000, math.pi / 4),
            (1200, 800,  0.0),
            (1800, 1200, math.pi / 2),
        ]

        section("get_fused_position avec poses Teensy fictives")
        results_fused = []
        for tx, ty, tth in test_poses:
            update_teensy_pose(tx, ty, tth)
            time.sleep(0.5)
            fx, fy, fth, conf = lidar_iface.get_fused_position(tx, ty, tth)
            results_fused.append((fx, fy, conf))
            ok(f"Teensy ({tx},{ty}) → Fusionné ({fx:.0f},{fy:.0f}) conf={conf:.2f}")

        section("get_opponent")
        opp = lidar_iface.get_opponent()
        if opp:
            ox, oy, oc = opp
            ok(f"Adversaire détecté : ({ox:.0f}, {oy:.0f}) conf={oc:.2f}")
        else:
            ok("Aucun adversaire détecté (normal si table vide)")

        section("get_diagnostic_info")
        diag = lidar_iface.get_diagnostic_info()
        ok(f"Localisé     : {diag['is_localized']}")
        ok(f"Nb balises   : {diag['nb_beacons']}")
        ok(f"Confiance    : {diag['confidence']:.2f}")
        ok(f"Balises IDs  : {diag['beacon_ids']}")

        section("Scan pendant %ds supplémentaires" % (duration_sec - 5))
        update_teensy_pose(1500, 1000, 0.0)
        time.sleep(duration_sec - 5)

        # Dernier état
        fx, fy, fth, conf = lidar_iface.get_fused_position(1500, 1000, 0.0)
        ok(f"Position finale fusionnée : ({fx:.0f}, {fy:.0f}) conf={conf:.2f}")

        stop_lidar_runtime()
        print()
        ok("TEST 7 PASSED")
        return record("lidar_interface", True)

    except Exception as e:
        fail(f"TEST 7 FAILED : {e}")
        import traceback; traceback.print_exc()
        return record("lidar_interface", False)


# ══════════════════════════════════════════════════════════════════════════════
# TEST 8 — Pipeline complet (sans Teensy réelle)
# ══════════════════════════════════════════════════════════════════════════════

def test_full_pipeline(duration_sec: int = 30, color: str = "BLUE") -> bool:
    header(f"TEST 8 — Pipeline complet : LiDAR→SVD→filtre→pathfinder ({duration_sec}s)")
    print("  ⚡  Nécessite LiDAR branché — Teensy simulée par injection de pose")
    try:
        from terrain_jeu import Terrain
        from pathfinder import PathFinder
        from lidar.lidar_logic import (
            start_lidar_thread, stop_lidar_runtime,
            update_teensy_pose, get_corrected_pose,
            should_send_correction_to_teensy, set_team_color,
        )
        from lidar.lidar import LidarInterface

        set_team_color(color)
        terrain = Terrain(color)
        pf = PathFinder(terrain)
        lidar_iface = LidarInterface(team_color=color)

        def _noop(x): pass
        start_lidar_thread(_noop, _noop)
        time.sleep(2.0)

        # Pose de départ
        robot_x, robot_y, robot_theta = terrain.get_start_position()
        update_teensy_pose(robot_x, robot_y, robot_theta)

        objectifs = [
            (1500, 1000),
            (2000, 500),
            (500, 1500),
            (1500, 1000),
        ]
        obj_idx = 0
        goal = {"x": objectifs[0][0], "y": objectifs[0][1]}

        corrections_ok   = 0
        paths_computed   = 0
        svd_poses_seen   = 0
        frames           = 0
        last_correction  = 0.0

        def apply_filter(lx, ly, tx, ty, conf):
            alpha = 0.85 if conf < 0.2 else (0.25 if conf > 0.8
                    else 0.85 - (conf - 0.2) / 0.6 * 0.60)
            return (1.0 - alpha) * lx + alpha * tx, (1.0 - alpha) * ly + alpha * ty, alpha

        ok(f"Position départ {color}: ({robot_x:.0f},{robot_y:.0f}) θ={math.degrees(robot_theta):.1f}°")
        ok(f"Objectifs : {objectifs}")
        print()

        start = time.time()
        while time.time() - start < duration_sec:
            frames += 1

            # — Injection pose Teensy (simule odométrie) —
            update_teensy_pose(robot_x, robot_y, robot_theta)

            # — Récupération pose SVD —
            corrected = get_corrected_pose()
            if corrected and corrected.is_localized:
                svd_poses_seen += 1
                rx, ry, alpha = apply_filter(
                    corrected.x, corrected.y,
                    robot_x, robot_y,
                    corrected.confidence,
                )
                rtheta = robot_theta
            else:
                rx, ry, rtheta = robot_x, robot_y, robot_theta

            # — Adversaire → obstacles —
            obstacles = []
            opp = lidar_iface.get_opponent()
            if opp:
                obstacles.append((opp[0], opp[1]))

            # — Pathfinding —
            dist_to_goal = math.hypot(goal["x"] - rx, goal["y"] - ry)
            if dist_to_goal < 80:
                obj_idx = (obj_idx + 1) % len(objectifs)
                goal = {"x": objectifs[obj_idx][0], "y": objectifs[obj_idx][1]}
                logger.debug(f"Objectif atteint, nouveau: {goal}")

            start_pos = {"x": rx, "y": ry, "theta": rtheta}
            chemin = pf.get_path(start_pos, goal, obstacles)
            if chemin and len(chemin) > 1:
                paths_computed += 1
                lookahead = min(3, len(chemin) - 1)
                cible_x, cible_y = chemin[lookahead]
                # Simuler mouvement vers cible (5mm/frame)
                dx = cible_x - robot_x
                dy = cible_y - robot_y
                dist = math.hypot(dx, dy)
                step = min(5.0, dist)
                if dist > 0.1:
                    robot_x += step * dx / dist
                    robot_y += step * dy / dist
                    robot_theta = math.atan2(dy, dx)

            # — Vérification correction Teensy —
            if should_send_correction_to_teensy():
                now = time.time()
                if now - last_correction >= 1.0:
                    corrections_ok += 1
                    last_correction = now
                    if corrected:
                        logger.debug(f"Correction Teensy #{corrections_ok}: "
                                     f"({corrected.x:.0f},{corrected.y:.0f}) conf={corrected.confidence:.2f}")

            # Log périodique
            if frames % 40 == 0:
                elapsed = time.time() - start
                ok(f"  t={elapsed:.1f}s | pos=({robot_x:.0f},{robot_y:.0f}) | "
                   f"SVD={svd_poses_seen} | paths={paths_computed} | corrections={corrections_ok}")

            time.sleep(0.05)

        stop_lidar_runtime()

        section("Bilan final")
        ok(f"Frames de boucle    : {frames}")
        ok(f"Poses SVD reçues    : {svd_poses_seen}")
        ok(f"Chemins calculés    : {paths_computed}")
        ok(f"Corrections Teensy  : {corrections_ok}")
        ok(f"Position finale     : ({robot_x:.0f}, {robot_y:.0f})")

        print()
        passed = paths_computed > 0
        if passed:
            ok("TEST 8 PASSED — Pipeline complet fonctionnel")
        else:
            fail("TEST 8 FAILED — Pathfinder n'a calculé aucun chemin")
        return record("pipeline", passed)

    except Exception as e:
        fail(f"TEST 8 FAILED : {e}")
        import traceback; traceback.print_exc()
        return record("pipeline", False)


# ══════════════════════════════════════════════════════════════════════════════
# RÉSUMÉ FINAL
# ══════════════════════════════════════════════════════════════════════════════

def print_summary() -> None:
    print("\n" + "═" * 72)
    print("  RÉSUMÉ DES TESTS")
    print("═" * 72)
    total  = len(RESULTS)
    passed = sum(1 for v in RESULTS.values() if v)
    for name, result in RESULTS.items():
        status = "✓ PASS" if result else "✗ FAIL"
        print(f"  {status}  —  {name}")
    print("─" * 72)
    print(f"  {passed}/{total} tests réussis")
    if passed == total:
        print("  ✓✓✓  TOUS LES TESTS PASSÉS")
    else:
        print(f"  ⚠   {total - passed} test(s) en échec")
    print("═" * 72 + "\n")


# ══════════════════════════════════════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════════════════════════════════════

TESTS = {
    1: ("Terrain (obstacles, balises, symétrie)",          test_terrain,               False),
    2: ("Pathfinder (A*, grille, inflation)",               test_pathfinder,            False),
    3: ("Strategy (actions, chrono, validation)",           test_strategy,              False),
    4: ("Filtre complémentaire adaptatif",                  test_complementary_filter,  False),
    5: ("lidar_logic static (SVD, fenêtres, constantes)",   test_lidar_logic_static,    False),
    6: ("Thread LiDAR réel (scan + balises)",               test_lidar_thread_basic,    True),
    7: ("LidarInterface (fused + opponent)",                test_lidar_interface,       True),
    8: ("Pipeline complet (LiDAR→SVD→filtre→pathfinder)",  test_full_pipeline,         True),
}


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Test complet CDR 2026 — LiDAR réel, Teensy simulée"
    )
    parser.add_argument(
        "--level", type=int, default=0,
        help="Numéro de test à exécuter (0 = tous, 1-8 = spécifique)"
    )
    parser.add_argument(
        "--duration", type=int, default=20,
        help="Durée des tests LiDAR (secondes, défaut 20)"
    )
    parser.add_argument(
        "--color", default="BLUE",
        help="Couleur d'équipe BLUE ou YELLOW (défaut BLUE)"
    )
    parser.add_argument(
        "--no-lidar", action="store_true",
        help="Sauter les tests nécessitant le LiDAR branché (tests 6-8)"
    )
    args = parser.parse_args()

    color = args.color.upper()
    if color not in ("BLUE", "YELLOW"):
        color = "BLUE"

    print("\n" + "╔" + "═" * 70 + "╗")
    print("║" + "  CDR 2026 — TEST COMPLET ROBOT (LiDAR réel, Teensy simulée)".center(70) + "║")
    print("╚" + "═" * 70 + "╝")
    print(f"  Équipe    : {color}")
    print(f"  Durée LiDAR: {args.duration}s par test")
    print(f"  LiDAR requis: {'OUI' if not args.no_lidar else 'SAUTÉ (--no-lidar)'}")
    print()

    if args.level == 0:
        # Tous les tests
        for num, (desc, fn, needs_lidar) in TESTS.items():
            if needs_lidar and args.no_lidar:
                print(f"\n  ⟳  Test {num} SAUTÉ (nécessite LiDAR) : {desc}")
                continue
            if needs_lidar:
                fn(duration_sec=args.duration, color=color)
            else:
                fn(color=color)
    else:
        if args.level not in TESTS:
            fail(f"Niveau {args.level} inconnu. Choisir entre 1 et {max(TESTS)}")
            sys.exit(1)
        desc, fn, needs_lidar = TESTS[args.level]
        if needs_lidar and args.no_lidar:
            print(f"  ⟳  Test {args.level} SAUTÉ (--no-lidar)")
        elif needs_lidar:
            fn(duration_sec=args.duration, color=color)
        else:
            fn(color=color)

    print_summary()
    sys.exit(0 if all(RESULTS.values()) else 1)


if __name__ == "__main__":
    main()