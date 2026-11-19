"""Tests de robustesse de detection.py, sans capteur ni moteurs.
Depuis CDR-Nantes : python -m robot1.rasp.tests.GenerePtsLidar_CasDifficiles
Placez ce fichier dans robot1/rasp/tests/.
Chaque scénario est indépendant. Résultats observés et non tests automatiques
sur des objets réels.
"""
import math
import time
from robot1.rasp.vision import detection

RX, RY = 700.0, 1000.0

def mesure(x, y, q=15):
    dx, dy = x - RX, y - RY
    return (q, math.degrees(math.atan2(dx, dy)) % 360, math.hypot(dx, dy))

def face(x, y, taille_mm, pas_mm=20):
    n = max(2, round(taille_mm / pas_mm))
    return [mesure(x, y - taille_mm / 2 + taille_mm * i / n) for i in range(n + 1)]

def config():
    detection.configure(stone_positions=[(1300, 1200)], cluster_gap_mm=55,
                        opponent_min_span_mm=100, opponent_max_span_mm=450,
                        robot_radius_mm=150, avoidance_margin_mm=50,
                        stone_physical_radius_mm=50)
    detection.update_robot_pose(RX, RY, 0.0)

def reset():
    # Fonction interne existante, utilisee seulement dans ce script de test.
    detection._clear_results()

def montrer(titre, scans):
    print('\n' + '=' * 65)
    print(titre)
    reset()
    config()
    for scan in scans:
        detection.process_scan(scan)
        time.sleep(0.02)
    groupes = detection.get_clusters()
    for c in groupes:
        print(f'  {c.categorie:21s} centre=({c.x_mm:.0f},{c.y_mm:.0f}) '
              f'taille={max(c.longueur_mm,c.largeur_mm):.0f}mm '
              f'points={len(c.points)}')
    if not groupes:
        print('  Aucun groupe significatif publie')
    print('  Adversaire suivi :', detection.get_opponent())
    print('  Nombre objets evitement :',len(detection.get_avoidance_objects()))

if __name__ == '__main__':
    # 1. Deux points isoles et points de qualite insuffisante.
    montrer('1 - Bruit : points isoles et mauvaise qualite', [
        [mesure(1800, 1700), mesure(1800, 1550), mesure(1200, 1400, 2),
         (15, 30.0, 0.0)]
    ])
    # 2. Objet assez grand en points mais trop petit pour etre un adversaire.
    petit = face(1700, 1000, 60, 10)
    montrer('2 - Face de 60 mm : PAS un adversaire', [petit, petit])
    # 3. Face au-dela des 450 mm adversaire, mais sous 650 mm objet.
    grand = face(1700, 1000, 500, 25)
    montrer('3 - Face de 500 mm : PAS un adversaire', [grand, grand])
    # 4. Deux surfaces proches dans l espace: voir si elles fusionnent.
    proches = face(1500, 900, 60, 15) + face(1530, 990, 60, 15)
    montrer('4 - Deux objets proches : observer le nombre de groupes', [proches, proches])
    # 5. Pierre en position initiale puis deplacee : le plan n'est pas un suivi.
    pierre_initiale = face(1300, 1200, 60, 10)
    montrer('5A - Pierre a sa position initiale', [pierre_initiale])
    pierre_deplacee = face(1300, 1450, 60, 10)
    montrer('5B - Pierre deplacee de 250 mm : sa categorie change', [pierre_deplacee])
    # 6. La cible adverse est confirme puis saute de 1000 mm en 10 ms.
    initial = face(1200, 1000, 200, 20)
    saute = face(2200, 1000, 200, 20)
    montrer('6A - Adversaire stable : confirmation', [initial, initial])
    print('\n' + '=' * 65)
    print('6B - Saut de 1000 mm : la NOUVELLE position doit etre refusee')
    detection.process_scan(saute)
    print('  Adversaire suivi apres saut :', detection.get_opponent())
    print('  Groupes visibles du nouveau scan :',
          [(c.categorie, round(c.x_mm), round(c.y_mm)) for c in detection.get_clusters()])
    print('\nFIN : comparer les categories et les centres aux attentes ci-dessus.')
