"""
test_lidar.py
─────────────
Script de test de la détection adversaire LiDAR — 3 étapes.
Lance depuis la racine du projet :  python test_lidar.py

Pas de robot, pas de Teensy nécessaire.
Le LiDAR doit être branché en USB.
"""

import math
import time
import sys
import os

# ── Permet de lancer depuis n'importe où dans le projet ──────────────────────
ROOT = os.path.dirname(os.path.abspath(__file__))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import lidar.lidar_detection as lidar

# ── Couleurs terminal (fonctionne sur Linux/Mac, ignoré si non supporté) ──────
R  = "\033[91m"   # rouge
G  = "\033[92m"   # vert
Y  = "\033[93m"   # jaune
B  = "\033[94m"   # bleu
W  = "\033[97m"   # blanc
DIM = "\033[2m"   # grisé
RST = "\033[0m"   # reset

def titre(texte: str) -> None:
    print(f"\n{B}{'─'*55}{RST}")
    print(f"{W}  {texte}{RST}")
    print(f"{B}{'─'*55}{RST}")

def ok(texte: str)   -> None: print(f"  {G}✓  {texte}{RST}")
def warn(texte: str) -> None: print(f"  {Y}⚠  {texte}{RST}")
def err(texte: str)  -> None: print(f"  {R}✗  {texte}{RST}")
def info(texte: str) -> None: print(f"  {DIM}{texte}{RST}")

def attendre_entree(msg: str = "Appuie sur Entrée pour continuer...") -> None:
    print(f"\n  {Y}→ {msg}{RST}")
    input("    ")

def barre_confiance(conf: float, largeur: int = 20) -> str:
    rempli = int(conf * largeur)
    vide   = largeur - rempli
    couleur = G if conf > 0.6 else (Y if conf > 0.3 else R)
    return f"{couleur}{'█' * rempli}{'░' * vide}{RST} {conf:.2f}"


# ══════════════════════════════════════════════════════════════════════════════
# ÉTAPE 1 — Détection basique : passe la main devant le LiDAR
# ══════════════════════════════════════════════════════════════════════════════

def etape1() -> bool:
    titre("ÉTAPE 1 — Détection basique (passe la main devant)")
    print()
    info("Pose fictive utilisée : x=1500 mm, y=1000 mm, θ=0° (centre du terrain).")
    info("Le LiDAR doit être posé à plat, dégagé.")
    print()
    attendre_entree("Appuie sur Entrée quand le LiDAR tourne et que tu es prêt...")

    lidar.update_robot_pose(1500.0, 1000.0, 0.0)
    lidar.start()
    time.sleep(1.5)   # laisse le LiDAR démarrer

    print()
    print(f"  Détection en cours — passe lentement la main à 30–80 cm du LiDAR.")
    print(f"  {DIM}(Ctrl+C pour passer à l'étape suivante){RST}")
    print()

    nb_detections = 0
    nb_scans      = 0
    t_debut       = time.time()
    duree_test    = 20   # secondes

    try:
        while time.time() - t_debut < duree_test:
            nb_scans += 1
            opp = lidar.get_opponent()

            if opp:
                nb_detections += 1
                ox, oy, conf = opp
                barre = barre_confiance(conf)
                print(f"\r  {G}Détecté{RST}  x={ox:6.0f} mm  y={oy:6.0f} mm  conf={barre}   ", end="", flush=True)
            else:
                print(f"\r  {DIM}(rien détecté — bouge la main){RST}                                  ", end="", flush=True)

            time.sleep(0.15)

    except KeyboardInterrupt:
        pass

    print()
    taux = nb_detections / max(nb_scans, 1) * 100

    if taux > 40:
        ok(f"Détection OK — taux={taux:.0f}%  ({nb_detections}/{nb_scans} scans)")
        return True
    elif taux > 10:
        warn(f"Détection partielle — taux={taux:.0f}%. Monte MIN_QUALITY à 1 ou baisse CLUSTER_MIN_PTS.")
        return True
    else:
        err(f"Aucune détection ({taux:.0f}%). Vérifie le port USB, le baudrate, ou que tu as bien bougé la main.")
        return False


# ══════════════════════════════════════════════════════════════════════════════
# ÉTAPE 2 — Vérification des coordonnées terrain
# ══════════════════════════════════════════════════════════════════════════════

def etape2() -> bool:
    titre("ÉTAPE 2 — Vérification des coordonnées terrain")
    print()
    info("On va fixer une pose connue et vérifier que l'objet détecté")
    info("apparaît aux bonnes coordonnées dans le repère terrain.")
    print()
    print(f"  {W}Scénario :{RST}")
    print(f"    • LiDAR posé en  x=500 mm, y=1000 mm, face vers les x+ (θ=0°)")
    print(f"    • Pose un objet (boîte, pied de chaise...) à ~1000 mm devant")
    print(f"    • Attendu : x ≈ 1500 mm, y ≈ 1000 mm")
    print()

    attendre_entree("Pose l'objet et appuie sur Entrée...")

    lidar.update_robot_pose(500.0, 1000.0, 0.0)
    time.sleep(0.3)

    print()
    print(f"  Lecture pendant 5 secondes...")
    print()

    mesures = []
    t0 = time.time()
    while time.time() - t0 < 5.0:
        opp = lidar.get_opponent()
        if opp:
            mesures.append(opp)
        time.sleep(0.15)

    if not mesures:
        err("Aucune détection. Essaie un objet plus grand ou vérifie l'orientation du LiDAR.")
        return False

    xs    = [m[0] for m in mesures]
    ys    = [m[1] for m in mesures]
    confs = [m[2] for m in mesures]
    x_moy = sum(xs) / len(xs)
    y_moy = sum(ys) / len(ys)
    c_moy = sum(confs) / len(confs)

    print(f"  {W}Résultats ({len(mesures)} détections) :{RST}")
    print(f"    x moyen = {x_moy:7.0f} mm   (attendu ≈ 1500 mm)")
    print(f"    y moyen = {y_moy:7.0f} mm   (attendu ≈ 1000 mm)")
    print(f"    confiance moy = {barre_confiance(c_moy)}")
    print()

    erreur_x = abs(x_moy - 1500)
    erreur_y = abs(y_moy - 1000)

    if erreur_x < 200 and erreur_y < 200:
        ok(f"Coordonnées cohérentes  (erreur x={erreur_x:.0f} mm, y={erreur_y:.0f} mm)")
        return True
    else:
        warn(f"Décalage important  (erreur x={erreur_x:.0f} mm, y={erreur_y:.0f} mm)")
        _diagnostiquer_offset(x_moy, y_moy)
        return False


def _diagnostiquer_offset(x_moy: float, y_moy: float) -> None:
    """Suggère un offset d'orientation si les coordonnées sont pivotées."""
    print()
    print(f"  {Y}Diagnostic orientation :{RST}")

    # L'objet est à ~1000 mm devant en x. Si on le retrouve sur l'axe Y,
    # c'est que le LiDAR est monté pivoté de 90°.
    if abs(y_moy - 1500) < 300 and abs(x_moy - 1000) < 300:
        warn("LiDAR probablement monté à 90°. Essaie LIDAR_OFFSET_RAD = math.radians(90) dans lidar_detection.py")
    elif abs(x_moy - (-500 + 500)) < 300:
        warn("LiDAR probablement monté à 180°. Essaie LIDAR_OFFSET_RAD = math.radians(180)")
    else:
        warn("Ajuste LIDAR_OFFSET_RAD dans lidar_detection.py par pas de 90° jusqu'à ce que ça colle.")

    info("Ligne à ajouter dans _process_scan(), juste après  rad = math.radians(angle_deg) :")
    info("    rad += LIDAR_OFFSET_RAD   # en haut du fichier : LIDAR_OFFSET_RAD = math.radians(90)")


# ══════════════════════════════════════════════════════════════════════════════
# ÉTAPE 3 — Test en mouvement (LiDAR bougé à la main)
# ══════════════════════════════════════════════════════════════════════════════

def etape3() -> None:
    titre("ÉTAPE 3 — Stabilité en mouvement (LiDAR bougé à la main)")
    print()
    info("Tu vas faire bouger le LiDAR lentement comme si c'était le robot.")
    info("On simule les poses que le robot aurait en se déplaçant sur le terrain.")
    info("Un objet fixe doit rester détecté en permanence.")
    print()
    print(f"  {W}Instructions :{RST}")
    print(f"    1. Pose un objet fixe à ~1 m du LiDAR")
    print(f"    2. Bouge doucement le LiDAR (translate, tourne-le légèrement)")
    print(f"    3. L'objet doit rester détecté et ses coordonnées doivent")
    print(f"       rester stables (±300 mm max de variation)")
    print()
    print(f"  {W}Poses simulées :{RST} la pose sera mise à jour toutes les 3s")
    print(f"    pour simuler un robot qui avance sur le terrain.")
    print()

    attendre_entree("Pose l'objet et appuie sur Entrée pour démarrer...")

    # Séquence de poses qui simule un robot qui avance
    poses_simulation = [
        (500,  1000, 0.0,  "Départ (x=500, y=1000, θ=0°)"),
        (800,  1000, 0.0,  "Avance (x=800, y=1000, θ=0°)"),
        (1100, 1000, 0.0,  "Avance (x=1100, y=1000, θ=0°)"),
        (1100, 1000, 0.3,  "Rotation légère (θ=17°)"),
        (800,  1200, 0.1,  "Déplacement (x=800, y=1200)"),
    ]

    historique_x   = []
    historique_y   = []
    nb_detecte     = 0
    nb_total       = 0

    print()
    print(f"  {'Pose simulée':<38} {'Détection':<10} {'x':>7} {'y':>7} {'conf':>6}")
    print(f"  {'─'*70}")

    try:
        for px, py, pt, label in poses_simulation:
            lidar.update_robot_pose(float(px), float(py), float(pt))
            print(f"\n  {DIM}{label}{RST}")

            t0 = time.time()
            while time.time() - t0 < 3.0:
                nb_total += 1
                opp = lidar.get_opponent()

                if opp:
                    nb_detecte += 1
                    ox, oy, conf = opp
                    historique_x.append(ox)
                    historique_y.append(oy)
                    barre = barre_confiance(conf)
                    print(f"\r    {G}Détecté{RST}  x={ox:6.0f} mm  y={oy:6.0f} mm  conf={barre}   ", end="", flush=True)
                else:
                    print(f"\r    {R}Pas de détection{RST}                                             ", end="", flush=True)

                time.sleep(0.15)

    except KeyboardInterrupt:
        pass

    print()
    print()

    # ── Résumé ────────────────────────────────────────────────────────────────
    taux = nb_detecte / max(nb_total, 1) * 100

    print(f"  {W}Résumé :{RST}")
    if taux > 60:
        ok(f"Détection stable — taux={taux:.0f}%")
    elif taux > 30:
        warn(f"Détection instable — taux={taux:.0f}%. Monte MAX_MISSED_SCANS à 20 dans lidar_detection.py")
    else:
        err(f"Détection trop faible — taux={taux:.0f}%. L'objet est peut-être sorti du champ.")

    if len(historique_x) > 5:
        variation_x = max(historique_x) - min(historique_x)
        variation_y = max(historique_y) - min(historique_y)
        print()
        info(f"Variation x = {variation_x:.0f} mm   Variation y = {variation_y:.0f} mm")
        if variation_x < 400 and variation_y < 400:
            ok("Coordonnées stables — la détection suit correctement le mouvement.")
        else:
            warn("Forte variation des coordonnées. Normal si tu as bougé l'objet,")
            warn("sinon vérifie que l'odométrie est bien mise à jour dans update_robot_pose().")


# ══════════════════════════════════════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════════════════════════════════════

def main():
    print()
    print(f"{B}{'═'*55}{RST}")
    print(f"{W}   TEST DÉTECTION LIDAR — CDR 2026{RST}")
    print(f"{B}{'═'*55}{RST}")
    print()
    info("Ce script ne nécessite pas de robot ni de Teensy.")
    info("Branche juste le LiDAR en USB avant de lancer.")
    print()

    try:
        # ── Étape 1 ───────────────────────────────────────────────────────────
        succes1 = etape1()
        if not succes1:
            err("Étape 1 échouée — vérifie le branchement LiDAR avant de continuer.")
            attendre_entree("Appuie sur Entrée pour quand même continuer vers l'étape 2, ou Ctrl+C pour quitter...")

        # ── Étape 2 ───────────────────────────────────────────────────────────
        attendre_entree("Prêt pour l'étape 2 ? Appuie sur Entrée...")
        etape2()

        # ── Étape 3 ───────────────────────────────────────────────────────────
        attendre_entree("Prêt pour l'étape 3 ? Appuie sur Entrée...")
        etape3()

    except KeyboardInterrupt:
        print()
        warn("Test interrompu.")

    finally:
        print()
        info("Arrêt du LiDAR...")
        lidar.stop()
        print()
        print(f"{B}{'═'*55}{RST}")
        print(f"{W}   TEST TERMINÉ{RST}")
        print(f"{B}{'═'*55}{RST}")
        print()


if __name__ == "__main__":
    main()