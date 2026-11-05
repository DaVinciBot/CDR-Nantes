#!/usr/bin/env python3
"""Tests CDR avec acquisition LiDAR commune.

Placer dans robot1/rasp/tests/test_complet_lidar.py.
Depuis la racine CDR-Nantes :
  python -m robot1.rasp.tests.test_complet_lidar --level 6 --duration 20
  python -m robot1.rasp.tests.test_complet_lidar --no-lidar

Niveaux : 1 terrain, 2 navigation, 3 strategie, 4 formule de reference,
5 localisation synthetique, 6 acquisition reelle, 7 detection observee,
8 integration localisation + detection + navigation (adaptation requise).
Par defaut : niveau 6 uniquement. --level 0 selectionne tous les niveaux.

7 et 8 : robot IMMOBILE, fournir sa pose mesuree via --x --y --theta-deg.
Le niveau 8 exige localization.process_scan(mesures) synchrone : il ne demarre
jamais l'ancien lecteur et n'applique aucune correction aux moteurs.
Le niveau 7 observe les candidats : il ne certifie pas leur identification.
"""

import sys
import time
import math
import logging
import argparse
from pathlib import Path
from typing import Dict

# Cherche la racine du depot depuis le fichier, pas depuis le terminal.
for parent in Path(__file__).resolve().parents:
    if (parent / "robot1" / "rasp").is_dir():
        sys.path.insert(0, str(parent))
        break

# ── Logging ───────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)-7s | %(name)-18s | %(message)s",
)
logger = logging.getLogger("TEST_CDR")

# ══════════════════════════════════════════════════════════════════════════════
# Helpers d'affichage
# ══════════════════════════════════════════════════════════════════════════════

def header(titre: str) -> None:
    print("\n" + "=" * 72)
    print(f"  {titre}")
    print("=" * 72)


def ok(msg: str) -> None:
    print(f"  [OK]  {msg}")


def warn(msg: str) -> None:
    print(f"  [WARN]  {msg}")


def fail(msg: str) -> None:
    print(f"  [FAIL]  {msg}")


def section(msg: str) -> None:
    print(f"\n  -- {msg} --")


RESULTS: Dict[str, bool] = {}


def record(name: str, passed: bool) -> bool:
    RESULTS[name] = passed
    return passed


# ══════════════════════════════════════════════════════════════════════════════
# TEST 1 — world
# ══════════════════════════════════════════════════════════════════════════════

def test_terrain(color: str = "BLUE") -> bool:
    header("TEST 1 — world : dimensions, obstacles, balises, symétrie")
    try:
        from robot1.rasp.world import Terrain, BeaconLayout, FIELD_WIDTH_MM, FIELD_HEIGHT_MM

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

        section("Symétrie balises BLEU -> JAUNE")
        for bid in BeaconLayout.BEACONS:
            bx_bl, by_bl = BeaconLayout.get_beacon(bid, "BLUE")
            bx_yl, by_yl = BeaconLayout.get_beacon(bid, "YELLOW")
            assert abs((bx_bl + bx_yl) - 3000) < 1, \
                f"Symétrie X incorrecte balise {bid}: {bx_bl}+{bx_yl}!=3000"
            # AJOUT : vérifie que la symétrie selon X conserve aussi la coordonnée Y.
            assert abs(by_bl - by_yl) < 1, "La symetrie X ne doit pas modifier Y"
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
        from robot1.rasp.world import Terrain
        from robot1.rasp.nav import PathFinder

        terrain = Terrain(color)
        pf = PathFinder(terrain)

        section("Paramètres grille")
        ok(f"GRID_SIZE = {pf.GRID_SIZE} mm")
        ok(f"MARGIN    = {pf.MARGIN} mm")
        ok(f"Inflation = robot({terrain.ROBOT_RADIUS}) + margin({pf.MARGIN}) = {pf.inflation_radius} mm")
        ok(f"Grille    = {pf.cols} cols × {pf.rows} rows")

        section("Chemin basique (centre -> bord)")
        start = {"x": 1500, "y": 1000, "theta": 0.0}
        goal  = {"x": 2800, "y": 1000}
        chemin = pf.get_path(start, goal, [])
        # AJOUT : l’absence de chemin entre les positions libres fait maintenant échouer le test.
        assert chemin and len(chemin) > 1, "Aucun chemin entre deux positions libres"
        if chemin and len(chemin) > 1:
            ok(f"Chemin trouvé : {len(chemin)} waypoints")
        else:
            warn("Aucun chemin trouvé pour start->bord (peut être bloqué par obstacle)")

        section("Chemin avec obstacle dynamique")
        obstacles_dyn = [(1800, 1000)]  # adversaire entre start et goal
        # AJOUT : inspecte la grille, au lieu de seulement afficher qu’un chemin a été calculé.
        grille_dyn = pf.create_dynamic_grid(obstacles_dyn)
        # Repère la case correspondant à l’obstacle dynamique injecté.
        gx, gy = pf._to_grid(1800, 1000)
        # AJOUT : exige que cette case soit réellement marquée comme occupée.
        assert grille_dyn[gy][gx] == 1, "Obstacle dynamique absent de la grille"
        chemin2 = pf.get_path(start, goal, obstacles_dyn)
        if chemin2:
            ok(f"Chemin avec obstacle : {len(chemin2)} waypoints")
        else:
            warn("Chemin bloqué par obstacle (normal si zone trop étroite)")

        section("Chemin impossible (goal hors terrain)")
        # MODIFICATION : utilise un objectif vraiment hors terrain ; (0,0) n’était pas forcément un mur.
        goal_wall = {"x": -1000, "y": -1000}  # hors terrain
        chemin3 = pf.get_path(start, goal_wall, [])
        # AJOUT : un chemin vers cet objectif invalide doit être rejeté.
        assert not chemin3, "Un chemin hors terrain a ete accepte"
        if not chemin3:
            ok("Chemin hors terrain correctement rejeté (retourne [])")
        else:
            warn(f"Chemin hors terrain retourné avec {len(chemin3)} pts — vérifier inflation")

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
            ok(f"  Segment {i+1}: ({s['x']},{s['y']}) -> ({g['x']},{g['y']}) — {status}")

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
        from robot1.rasp.strategy import Action, TypeAction
        from robot1.rasp.strategy import StratManager

        section("Création StratManager")
        strat = StratManager(color)
        ok(f"StratManager créé pour équipe {color}")
        ok(f"Nombre d'actions dans la stratégie : {len(strat.liste_actions)}")

        section("Types d'actions")
        for i, action in enumerate(strat.liste_actions):
            ok(f"  Action {i}: {action.type.name}"
               + (f" -> ({action.cible_x:.0f}, {action.cible_y:.0f})" if action.type == TypeAction.DEPLACEMENT else "")
               + (f" -> {action.nom_actionneur}" if action.type == TypeAction.ACTIONNEUR else "")
               + (f" -> {action.temps_attente}s" if action.type == TypeAction.ATTENTE else ""))

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
        ok("valider_action_terminee() OK -> étape 0->1")

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
    header("TEST 4 — courbe de référence uniquement, PAS le filtre du robot")
    try:
        # Copie locale de la courbe, volontairement autonome : la méthode
        # d'origine Robot._apply_complementary_filter n'existe plus, et la
        # courbe reste la référence documentée en interne.
        def apply_filter(lidar_x, lidar_y, teensy_x, teensy_y, confidence):
            """Courbe alpha adaptative : 0.85 sous conf 0.2, 0.25 au-dessus de 0.8."""
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
            (0.00, 0.85, "conf=0.00 -> alpha=0.85 (85% Teensy)"),
            (0.10, 0.85, "conf=0.10 -> alpha=0.85"),
            (0.20, 0.85, "conf=0.20 -> alpha=0.85 (boundary bas)"),
            (0.35, None, "conf=0.35 -> alpha intermédiaire"),
            (0.50, 0.55, "conf=0.50 -> alpha=0.55"),
            (0.65, None, "conf=0.65 -> alpha intermédiaire"),
            (0.80, 0.25, "conf=0.80 -> alpha=0.25 (boundary haut)"),
            (0.90, 0.25, "conf=0.90 -> alpha=0.25"),
            (1.00, 0.25, "conf=1.00 -> alpha=0.25 (75% LiDAR)"),
        ]

        alphas = []
        ok("Progression alpha (doit être monotone décroissante) :")
        for conf, expected_alpha, desc in cases:
            _, _, alpha = apply_filter(1000, 2000, 1010, 2010, conf)
            alphas.append(alpha)
            if expected_alpha is not None:
                diff = abs(alpha - expected_alpha)
                status = "[OK]" if diff < 0.001 else "[FAIL]"
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
        ok(f"Basse conf -> fusionné ({xf_low:.1f},{yf_low:.1f}) ~ Teensy ({tx},{ty})")
        ok(f"Haute conf -> fusionné ({xf_high:.1f},{yf_high:.1f}) ~ LiDAR ({lx},{ly})")

        print()
        ok("TEST 4 PASSED")
        return record("filter", True)

    except Exception as e:
        fail(f"TEST 4 FAILED : {e}")
        import traceback; traceback.print_exc()
        return record("filter", False)


# ══════════════════════════════════════════════════════════════════════════════
# TEST 5 — localization constants & SVD hors thread
# ══════════════════════════════════════════════════════════════════════════════

# RENOMMAGE : remplace test_lidar_logic_static ; aucun module lidar_logic.py n’est utilisé.
def test_localization_static(color: str = "BLUE") -> bool:
    header("TEST 5 — localization : constantes, BEACONS, SVD Umeyama (hors thread)")
    try:
        from robot1.rasp.vision.localization import (
            BEACONS_BY_ID, BEACON_WINDOW_ANGLE_RAD, BEACON_WINDOW_DIST_MM,
            POSE_CORRECTION_MIN_CONFIDENCE, POSE_CORRECTION_MIN_BEACONS,
            POSE_SEND_BACK_INTERVAL_S, BEACON_FIT_MAX_RMS_MM,
            set_team_color, _predict_beacon_windows, _compute_corrected_pose,
            _extract_beacon_candidates_fast, _associate_candidates_to_beacons,
            _validate_beacon_geometry,
        )

        section("Chargement balises")
        set_team_color(color)
        # CORRECTION : relit le dictionnaire dans le module après le changement de couleur.
        from robot1.rasp.vision import localization
        # set_team_color() réaffecte le dictionnaire ; un ancien import direct gardait l’ancienne couleur.
        BEACONS_BY_ID = localization.BEACONS_BY_ID
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
                assert abs(result.x - 1500.0) < 50, f"deltax trop grand : {result.x-1500:.1f}mm"
                assert abs(result.y - 1000.0) < 50, f"deltay trop grand : {result.y-1000:.1f}mm"
                ok("Delta SVD < 50mm (correction précise sur candidats exacts)")
            else:
                # MODIFICATION : un rejet de candidats synthétiques exacts est un échec, pas un avertissement.
                raise AssertionError("SVD retourne None sur des candidats exacts")
        else:
            warn("Moins de 2 candidats synthétiques, SVD non testé")

        section("_associate_candidates_to_beacons (sans fenêtres)")
        if len(syn_cands) >= 2:
            cands_sans_bid = [{k: v for k, v in c.items() if k != "beacon_id"} for c in syn_cands]
            assoc = _associate_candidates_to_beacons(cands_sans_bid, 1500.0, 1000.0, 0.0, None)
            # AJOUT : exige que tous les candidats synthétiques aient été associés.
            assert len(assoc) == len(cands_sans_bid), "Association incomplète"
            ok(f"Association sans fenêtres : {len(assoc)} candidats associés / {len(cands_sans_bid)}")
            for a in assoc:
                ok(f"  -> Balise #{a.get('beacon_id')} (score={a.get('association_score', 'N/A')})")

        section("_validate_beacon_geometry")
        if len(syn_cands) >= 2:
            valid = _validate_beacon_geometry(syn_cands[:2])
            # AJOUT : fait échouer le test si la géométrie exacte des balises est rejetée.
            assert valid, "Géométrie exacte rejetée"
            ok(f"Validation géométrie 2 balises synthétiques : {'OK' if valid else 'REJETÉ'}")

        print()
        ok("TEST 5 PASSED")
        return record("localization_static", True)

    except Exception as e:
        fail(f"TEST 5 FAILED : {e}")
        import traceback; traceback.print_exc()
        return record("localization_static", False)




# AJOUT : vérifie le scan déjà lu par acquisition.py, sans ouvrir de connexion.
# ══════════════════════════════════════════════════════════════════════════════
# OUTILS COMMUNS — validation et exécution des tests LiDAR
# ══════════════════════════════════════════════════════════════════════════════

def _validate_scan(scan):
    # Un scan publié doit porter un numéro positif.
    assert scan.numero > 0, "Numero de scan invalide"
    # Refuse un horodatage NaN ou infini.
    assert math.isfinite(scan.timestamp), "Horodatage invalide"
    # Exige des données de moins de 0,5 seconde ; seuil à adapter à votre cadence.
    assert 0 <= scan.age() <= 0.5, "Scan perime"
    # Un scan vide ne permet pas de valider la réception des mesures.
    assert scan.mesures, "Scan vide"
    # Parcourt tous les points du scan pour vérifier leur format.
    for mesure in scan.mesures:
        # Chaque point doit contenir exactement trois valeurs.
        assert len(mesure) == 3, "Format attendu : (qualite, degres, mm)"
        # Décompose le point dans l’ordre (qualité, angle en degrés, distance en mm).
        qualite, angle, distance = mesure
        # Vérifie que le filtrage a retiré les valeurs NaN et infinies.
        assert all(math.isfinite(v) for v in mesure), "Valeur non finie"
        # Vérifie que les distances nulles ou négatives ont été éliminées.
        assert distance > 0, "Distance nulle ou negative"
        # Contrôle la plage des angles bruts du pilote : [0°, 360°[.
        assert 0 <= angle < 360, "Angle hors de [0, 360["
        # Refuse une qualité négative ; ne filtre pas les faibles qualités à ce stade.
        assert qualite >= 0, "Qualite negative"


# REMPLACE les anciens tests matériels : un seul lecteur pour les niveaux 6, 7 et 8.
def _hardware_test(level, args):
    """Un seul proprietaire du capteur ; chaque scan traite une seule fois."""
    # Associe le niveau à son nom dans le résumé final.
    name = {6: "acquisition", 7: "detection_observation", 8: "integration"}[level]
    # Affiche le niveau choisi et la durée demandée.
    titres = {
        6: "Acquisition LiDAR commune : scans, format, fraîcheur",
        7: "Détection adverse : traitement des scans communs",
        8: "Pipeline commun : LiDAR -> localisation + détection -> navigation",
    }
    header(f"TEST {level} — {titres[level]} ({args.duration}s)")
    print("  Nécessite le LiDAR branché ; aucun mouvement commandé.")
    section("Préparation des modules")
    # Permet le nettoyage même si un import ou une initialisation échoue.
    acquisition = None
    # Par défaut, le test échoue ; seuls les contrôles réussis autorisent un succès.
    passed = False
    # Protège ce bloc pour pouvoir signaler les erreurs et nettoyer les ressources.
    try:
        # Utilise le nouveau lecteur commun au lieu de start_lidar_thread().
        from robot1.rasp.vision import acquisition
        # Récupère le vrai port configuré et la vitesse, sans les écrire en dur dans le test.
        from robot1.rasp.vision.lidar_config import PORT, BAUDRATE
        # Prépare les composants optionnels ; ils ne sont pas requis au niveau 6.
        detection = localization = pf = None
        # Active la détection uniquement pour les niveaux 7 et 8.
        if level >= 7:
            # Exige la pose réelle du robot immobile pour projeter correctement les points.
            if args.x is None or args.y is None or args.theta_deg is None:
                # Interrompt ce niveau avec une explication claire, plutôt que poursuivre avec des données manquantes.
                raise RuntimeError("Fournir la pose REELLE du robot immobile : --x --y --theta-deg")
            # Importe le traitement ; ne démarre pas son ancien lecteur.
            from robot1.rasp.vision import detection
            # Compatible avec le wrapper propose et avec l'ancien traitement pur.
            # Cherche la nouvelle fonction publique qui reçoit les mesures en paramètre.
            process_detection = getattr(detection, "process_scan", None)
            # Vérifie qu’une fonction de traitement est disponible.
            if not callable(process_detection):
                # Compatibilité : utilise le calcul existant si le wrapper public n’est pas encore ajouté.
                process_detection = getattr(detection, "_process_scan", None)
            # Vérifie qu’une fonction de traitement est disponible.
            if not callable(process_detection):
                # Interrompt ce niveau avec une explication claire, plutôt que poursuivre avec des données manquantes.
                raise RuntimeError("Aucun traitement de scan disponible dans detection.py")
            # Convertit l’orientation de la ligne de commande en radians pour les calculs.
            theta = math.radians(args.theta_deg)
            # Fournit la pose réelle à la détection ; aucune commande de mouvement n’est envoyée.
            detection.update_robot_pose(args.x, args.y, theta)
        # La localisation et le calcul de chemin sont réservés au test d’intégration.
        if level == 8:
            # Importe les calculs de localisation, sans lancer leur ancien thread.
            from robot1.rasp.vision import localization
            # Charge la description du terrain utilisée par la navigation.
            from robot1.rasp.world import Terrain
            # Charge le calculateur de chemin, sans importer le contrôleur des moteurs.
            from robot1.rasp.nav import PathFinder
            # Refuse de continuer si la localisation n’a pas été adaptée à une entrée de scan commune.
            if not callable(getattr(localization, "process_scan", None)):
                # Interrompt ce niveau avec une explication claire, plutôt que poursuivre avec des données manquantes.
                raise RuntimeError(
                    "localization.process_scan(mesures) manque. Adapter ce module avant "
                    "le niveau 8 ; ne pas redemarrer start_lidar_thread().")
            # Le niveau 8 exige aussi un objectif réel de navigation, choisi dans une zone libre.
            if args.goal_x is None or args.goal_y is None:
                # Interrompt ce niveau avec une explication claire, plutôt que poursuivre avec des données manquantes.
                raise RuntimeError("Fournir aussi --goal-x et --goal-y (objectif libre sur le terrain)")
            # Charge les coordonnées des balises de la bonne équipe.
            localization.set_team_color(args.color)
            # Injecte la pose mesurée comme référence odométrique ; la Teensy n’est pas sollicitée.
            localization.update_teensy_pose(args.x, args.y, theta)
            # Construit la grille de navigation à partir du terrain.
            pf = PathFinder(Terrain(args.color))

        # Affiche la connexion que le test va effectivement utiliser.
        section("Démarrage de l’acquisition commune")
        ok(f"Port configure : {PORT} ; vitesse : {BAUDRATE} bauds")
        # Démarre l’unique lecture et attend le premier scan valide.
        if not acquisition.start():
            # Interrompt ce niveau avec une explication claire, plutôt que poursuivre avec des données manquantes.
            raise RuntimeError(acquisition.get_status()[1] or "Aucun scan valide au demarrage")
        # Initialise les compteurs : scans, points, candidats, poses et chemins.
        count = total_points = candidates = poses = paths = 0
        # Mémorise le dernier numéro traité pour ne pas recompter le même scan.
        last_num = 0
        # Permet de vérifier ensuite que les horodatages progressent.
        last_timestamp = None
        # Évite de compter plusieurs fois une ancienne pose de localisation.
        last_pose_time = 0.0
        # Conserve une référence au scan précédent pour vérifier la stabilité de son numéro.
        previous_scan = None
        # Limite les messages de progression à environ un par seconde.
        last_log = 0.0
        # Mesure la durée avec une horloge insensible aux changements de l’heure système.
        section("Observation des scans")
        started = time.monotonic()
        # Observe les scans pendant la durée demandée, sans simuler de déplacement du robot.
        while time.monotonic() - started < args.duration:
            # Vérifie à chaque tour si l’acquisition a signalé une erreur.
            _, error = acquisition.get_status()
            # Une erreur matérielle doit faire échouer le test, pas laisser un faux succès.
            if error:
                # Interrompt ce niveau avec une explication claire, plutôt que poursuivre avec des données manquantes.
                raise RuntimeError(error)
            # Consulte le dernier scan : cet appel ne déclenche pas une nouvelle mesure.
            scan = acquisition.get_latest_scan()
            # Une absence de scan récent est considérée comme une perte des données.
            if scan is None:
                # Interrompt ce niveau avec une explication claire, plutôt que poursuivre avec des données manquantes.
                raise RuntimeError("Perte de donnees : aucun scan recent")
            # Si aucun nouveau scan n’est arrivé, ne relance pas les traitements.
            if scan.numero == last_num:
                # Attend brièvement pour ne pas faire tourner inutilement le processeur.
                time.sleep(0.01)
                # Passe au prochain tour de boucle sans recompter les mêmes mesures.
                continue
            # Vérifie le format et la validité de tous les points du nouveau scan.
            _validate_scan(scan)
            # Un scan publié doit porter un numéro positif.
            assert scan.numero > last_num, "Numero de scan non croissant"
            # Compare les dates seulement à partir du deuxième scan observé.
            if last_timestamp is not None:
                # Les scans successifs ne doivent pas être horodatés dans le désordre.
                assert scan.timestamp >= last_timestamp, "Horodatage non croissant"
                # Le snapshot precedent doit rester lisible apres publication du suivant.
                # Vérifie la stabilité du numéro de l’ancien snapshot ; ne prouve pas toute son immutabilité.
                assert previous_scan.numero == last_num, "Ancien scan modifie"
            # Mémorise le numéro et la date du scan qui vient d’être accepté.
            last_num, last_timestamp = scan.numero, scan.timestamp
            # Garde cette référence pour le contrôle au prochain scan.
            previous_scan = scan
            # Compte un scan distinct observé, et non un simple tour de boucle.
            count += 1
            # Additionne les points des scans observés pour calculer une moyenne.
            total_points += len(scan.mesures)
            # Prépare les obstacles de ce tour ; la liste sera alimentée par la détection.
            obstacles = []
            # Active la détection uniquement pour les niveaux 7 et 8.
            if level >= 7:
                # Meme objet mesures pour les deux traitements ; aucune nouvelle lecture.
                # Transmet les mesures déjà acquises au traitement de l’adversaire.
                process_detection(scan.mesures)
                # Récupère le candidat produit ou suivi par la détection.
                opponent = detection.get_opponent()
                # N’exploite un candidat que si le module en fournit un.
                if opponent is not None:
                    # Décompose le candidat : coordonnées du terrain et indice de confiance.
                    ox, oy, confidence = opponent
                    # Vérifie que les coordonnées et la confiance sont des nombres finis.
                    assert all(math.isfinite(v) for v in opponent), "Candidat invalide"
                    # Compte les scans avec candidat ; ce n’est pas le nombre de robots distincts.
                    candidates += 1
                    # Présente le candidat à la navigation comme un obstacle dynamique.
                    obstacles = [(ox, oy)]
            # La localisation et le calcul de chemin sont réservés au test d’intégration.
            if level == 8:
                # Contrat : entree brute (qualite, degres, mm), calcul synchrone,
                # publication via get_corrected_pose(), horodatage time.time().
                # Transmet exactement les mêmes mesures à la localisation ; aucune seconde acquisition.
                localization.process_scan(scan.mesures)
                # Lit la pose publiée par le traitement de localisation adapté.
                pose = localization.get_corrected_pose()
                # N’exploite que les résultats explicitement déclarés localisés.
                if pose is not None and pose.is_localized:
                    # Le contrat actuel de PoseState utilise time.time(), différent de l’horloge des scans.
                    age = time.time() - pose.last_update_time
                    # Ne compte qu’une pose récente et plus nouvelle que celle déjà comptée.
                    if 0 <= age <= 0.5 and pose.last_update_time > last_pose_time:
                        # Mémorise la date de cette nouvelle pose pour éviter les doublons.
                        last_pose_time = pose.last_update_time
                        # Refuse une position ou une orientation non numérique.
                        assert all(math.isfinite(v) for v in (pose.x, pose.y, pose.theta))
                        # Applique le seuil de confiance défini par le module de localisation.
                        assert pose.confidence >= localization.POSE_CORRECTION_MIN_CONFIDENCE
                        # Compte une nouvelle pose acceptée ; ne certifie pas sa précision physique.
                        poses += 1
                        # Calcule un chemin depuis la pose LiDAR vers l’objectif en tenant compte du candidat.
                        path = pf.get_path(
                            {"x": pose.x, "y": pose.y, "theta": pose.theta},
                            {"x": args.goal_x, "y": args.goal_y}, obstacles)
                        # Un chemin exploitable pour ce test doit contenir au moins deux positions.
                        if path and len(path) > 1:
                            # Compte les calculs de chemin réussis ; aucun mouvement n’est exécuté.
                            paths += 1
            # Récupère l’heure de suivi pour limiter l’affichage.
            now = time.monotonic()
            # Affiche la progression environ une fois par seconde.
            if now - last_log >= 1.0:
                # Mémorise la date du dernier affichage.
                last_log = now
                # Affiche numéro, quantité, fraîcheur et un point d’exemple du scan.
                ok(f"Scan #{scan.numero} : {len(scan.mesures)} points ; "
                   f"age={scan.age():.3f}s ; exemple={scan.mesures[0]}")
        # Un seul scan ne suffit pas à démontrer une lecture qui continue.
        section("Résultats acquisition")
        assert count >= 2, "Moins de deux scans distincts observes"
        # Exige une acquisition encore opérationnelle à la fin de l’observation.
        assert acquisition.get_status()[0], "Acquisition inactive en fin de test"
        # Résume les scans distincts observés, la quantité moyenne de points et la cadence observée.
        ok(f"{count} scans distincts observes, {total_points / count:.0f} points/scan "
           f"en moyenne ; {count / args.duration:.1f} scans observes/s")
        # Précise que le polling peut manquer des scans ; ce n’est pas la cadence exhaustive du matériel.
        print("  Cette frequence est celle des scans observes par le test, pas une mesure exhaustive du capteur.")
        # Active la détection uniquement pour les niveaux 7 et 8.
        if level >= 7:
            # Résume les exécutions de la détection et les scans avec candidat.
            section("Résultats détection adverse")
            ok(f"Traitement detection execute sur {count} scans ; {candidates} avec candidat")
            # Évite de confondre exécution du code et validation réelle de la reconnaissance adverse.
            warn("Un candidat peut etre un mur. Verification visuelle necessaire ; "
                 "ce test ne certifie pas la reconnaissance de l'adversaire.")
        # La localisation et le calcul de chemin sont réservés au test d’intégration.
        if level == 8:
            # CHANGEMENT : le niveau 8 échoue si aucune nouvelle pose valide n’a été obtenue.
            section("Résultats localisation et navigation")
            assert poses > 0, "Aucune NOUVELLE pose localisee et recente : integration non validee"
            # CHANGEMENT : un calcul de chemin depuis une pose LiDAR récente est aussi requis.
            assert paths > 0, "Aucun chemin calcule depuis une pose LiDAR recente"
            # Résume l’intégration logicielle et rappelle l’absence de commande moteur.
            ok(f"{poses} poses recentes ; {paths} chemins ; aucune commande moteur envoyee")
            # La validité logicielle ne remplace pas une comparaison avec la position mesurée.
            warn("La precision de localisation doit etre comparee a la position reelle.")
        # Autorise le succès seulement après tous les contrôles du niveau choisi.
        passed = True
    except Exception as exc:
        # Affiche la cause de l’échec dans la console.
        fail(str(exc))
        # Conserve la trace complète pour diagnostiquer le problème.
        logger.exception("Test %s en echec", level)
    finally:
        # Ne tente l’arrêt que si le module a pu être importé.
        section("Arrêt et libération du LiDAR")
        if acquisition is not None:
            # Protège ce bloc pour pouvoir signaler les erreurs et nettoyer les ressources.
            try:
                # Demande l’arrêt, attend le thread et libère le port, y compris après un échec.
                acquisition.stop()
                # Vérifie que l’acquisition n’est plus active après son arrêt.
                assert not acquisition.get_status()[0], "Acquisition toujours active"
                # Vérifie qu’un ancien scan n’est plus présenté comme disponible après arrêt.
                assert acquisition.get_latest_scan() is None, "Scan encore publie apres arret"
                # Confirme les contrôles de fermeture, pas seulement l’appel de stop().
                ok("Acquisition arretee et dernier scan efface")
            except Exception as exc:
                # Signale séparément une erreur pendant la fermeture du capteur.
                fail(f"Arret incomplet : {exc}")
                # Par défaut, le test échoue ; seuls les contrôles réussis autorisent un succès.
                passed = False
    # Enregistre le résultat final, en tenant compte des erreurs de nettoyage.
    print()
    if passed:
        ok(f"TEST {level} PASSED — {name}")
    else:
        fail(f"TEST {level} FAILED — {name}")
    return record(name, passed)


# ══════════════════════════════════════════════════════════════════════════════
# TEST 6 — Acquisition LiDAR commune (capteur réel)
# ══════════════════════════════════════════════════════════════════════════════

def test_acquisition(duration_sec: float = 20, color: str = "BLUE", args=None) -> bool:
    """Teste uniquement la nouvelle lecture, sans dépendre des calculs de balises."""
    # Réutilise les paramètres de la commande, ou prépare ceux d’un appel direct.
    if args is None:
        args = argparse.Namespace(duration=duration_sec, color=color)
    # Le moteur commun n’active ni détection ni localisation au niveau 6.
    return _hardware_test(6, args)


# ══════════════════════════════════════════════════════════════════════════════
# TEST 7 — Détection adverse depuis les scans de l’acquisition commune
# ══════════════════════════════════════════════════════════════════════════════

def test_detection(duration_sec: float = 20, color: str = "BLUE", args=None) -> bool:
    """Exécute la détection ; ne certifie pas à lui seul l’identification adverse."""
    # Le robot reste immobile ; X, Y et orientation doivent correspondre au réel.
    if args is None:
        args = argparse.Namespace(duration=duration_sec, color=color,
                                  x=None, y=None, theta_deg=None)
    # La détection reçoit les mesures déjà acquises ; detection.start() n’est pas appelé.
    return _hardware_test(7, args)


# ══════════════════════════════════════════════════════════════════════════════
# TEST 8 — Pipeline complet avec acquisition commune (sans Teensy réelle)
# ══════════════════════════════════════════════════════════════════════════════

def test_full_pipeline(duration_sec: float = 30, color: str = "BLUE", args=None) -> bool:
    """Teste localisation, détection et navigation sur les mêmes mesures réelles.

    Prérequis : localization.process_scan(mesures) adapté et défauts corrigés.
    Pose réelle et objectif libre à fournir ; aucun déplacement fictif ou réel.
    Le test exige une nouvelle pose valide et un chemin depuis une telle pose.
    """
    if args is None:
        args = argparse.Namespace(duration=duration_sec, color=color,
                                  x=None, y=None, theta_deg=None,
                                  goal_x=None, goal_y=None)
    # Le test refuse l’ancien lecteur de localization.py ; seul acquisition.py lit.
    return _hardware_test(8, args)


# ══════════════════════════════════════════════════════════════════════════════
# RÉSUMÉ FINAL
# ══════════════════════════════════════════════════════════════════════════════

def print_summary() -> None:
    """Présentation conservée : un statut par test et bilan global."""
    header("RÉSUMÉ DES TESTS")
    total = len(RESULTS)
    passed = sum(1 for result in RESULTS.values() if result)
    for name, result in RESULTS.items():
        status = "[OK] PASS" if result else "[FAIL] FAIL"
        print(f"  {status}  —  {name}")
    print("-" * 72)
    print(f"  {passed}/{total} tests réussis")
    if total and passed == total:
        print("  OK  TOUS LES TESTS EXÉCUTÉS SONT PASSÉS")
    else:
        print(f"  [WARN]  {total - passed} test(s) en échec")
    print("  Le niveau 4 teste une formule locale, pas le filtre du robot.")
    print("  Détection et précision de localisation restent à vérifier physiquement.")
    print("=" * 72 + "\n")


# ══════════════════════════════════════════════════════════════════════════════
# MAIN — sélection des tests (présentation du document d’origine)
# ══════════════════════════════════════════════════════════════════════════════

# Chaque niveau possède maintenant sa fonction et sa section, comme les tests 1-5.
# Le booléen indique si le capteur réel est nécessaire.
TESTS = {
    1: ("Terrain (obstacles, balises, symétrie)", test_terrain, False),
    2: ("Pathfinder (A*, grille, inflation)", test_pathfinder, False),
    3: ("Stratégie (actions, chrono, validation)", test_strategy, False),
    4: ("Filtre complémentaire : formule de référence", test_complementary_filter, False),
    5: ("Localisation statique (SVD, fenêtres, constantes)", test_localization_static, False),
    6: ("Acquisition LiDAR commune (scans et fraîcheur)", test_acquisition, True),
    7: ("Détection adverse sur scans communs", test_detection, True),
    8: ("Pipeline commun (localisation, détection, navigation)", test_full_pipeline, True),
}


# REMPLACEMENT : organise les nouveaux niveaux et vérifie les arguments fournis.
def main():
    # Construit l’aide de la commande à partir de la description du fichier.
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    # CHANGEMENT : niveau 6 par défaut ; 0 = tous ; le niveau 7 existe désormais.
    parser.add_argument("--level", type=int, choices=range(9), default=6)
    # Accepte une durée en secondes, éventuellement décimale.
    parser.add_argument("--duration", type=float, default=20)
    # Accepte BLUE/YELLOW et normalise la casse au lieu de corriger silencieusement une faute.
    parser.add_argument("--color", type=str.upper, choices=("BLUE", "YELLOW"), default="BLUE")
    # Cette option sélectionne uniquement les cinq tests sans capteur branché.
    parser.add_argument("--no-lidar", action="store_true", help="Executer seulement les niveaux 1 a 5")
    # AJOUT : coordonnée réelle X du robot immobile, en millimètres.
    parser.add_argument("--x", type=float, help="Position reelle X du robot immobile en mm")
    # AJOUT : coordonnée réelle Y du robot immobile, en millimètres.
    parser.add_argument("--y", type=float, help="Position reelle Y du robot immobile en mm")
    # AJOUT : orientation réelle ; la conversion en radians est faite ensuite.
    parser.add_argument("--theta-deg", type=float, help="Orientation reelle selon la convention du code, en degres")
    # AJOUT : objectif X requis pour le calcul de chemin au niveau 8.
    parser.add_argument("--goal-x", type=float, help="Objectif de navigation X, mm (niveau 8)")
    # AJOUT : objectif Y requis pour le calcul de chemin au niveau 8.
    parser.add_argument("--goal-y", type=float, help="Objectif de navigation Y, mm (niveau 8)")
    # Lit et convertit les options saisies dans le terminal.
    args = parser.parse_args()
    # Refuse une durée nulle, négative, NaN ou infinie.
    if not math.isfinite(args.duration) or args.duration <= 0:
        # Explique l’erreur et quitte avec un code d’échec.
        parser.error("--duration doit etre strictement positive et finie")
    # Vérifie les cinq arguments numériques de position et d’objectif.
    for field in ("x", "y", "theta_deg", "goal_x", "goal_y"):
        # Récupère la valeur de l’argument dont le nom est dans field.
        value = getattr(args, field)
        # Les arguments absents restent permis jusqu’au contrôle du niveau concerné.
        if value is not None and not math.isfinite(value):
            # Indique précisément quel argument contient une valeur non finie.
            parser.error(f"--{field.replace('_', '-')} doit etre fini")
    print("\n" + "+" + "=" * 70 + "+")
    print("|" + "  CDR — TEST COMPLET ROBOT / ACQUISITION LiDAR COMMUNE".center(70) + "|")
    print("+" + "=" * 70 + "+")
    print(f"  Équipe      : {args.color}")
    print(f"  Durée LiDAR : {args.duration}s par test")
    print(f"  LiDAR requis: {'NON (--no-lidar)' if args.no_lidar else 'selon le niveau'}")
    print("  Robot immobile ; aucune commande moteur envoyée.")
    # Sans LiDAR : 1-5 ; niveau 0 : tous ; sinon : seulement le niveau demandé.
    levels = list(range(1, 6)) if args.no_lidar else (list(TESTS) if args.level == 0 else [args.level])
    # Efface les anciens résultats avant une nouvelle exécution du lanceur.
    RESULTS.clear()
    # Chaque fonction de test figure explicitement dans TESTS.
    for level in levels:
        description, function, needs_lidar = TESTS[level]
        if needs_lidar:
            function(duration_sec=args.duration, color=args.color, args=args)
        else:
            function(color=args.color)
    # Rétablit le même bilan détaillé que dans le document initial.
    print_summary()
    # Renvoie 0 uniquement si au moins un test a été exécuté et si tous ont réussi.
    return 0 if RESULTS and all(RESULTS.values()) else 1


# Ne lance les tests automatiquement que si ce fichier est exécuté comme programme.
if __name__ == "__main__":
    # Transmet au terminal le code de succès/échec, utile pour l’automatisation.
    raise SystemExit(main())
