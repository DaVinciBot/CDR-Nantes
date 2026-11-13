# sim2d — simulateur 2D cinématique

Fait tourner **le code de production de `robot1/rasp/` sans robot**. Seuls les backends changent :
le lien USB vers la Teensy est remplacé par un robot simulé, les GPIO par des boutons simulés,
l'horloge murale par une horloge virtuelle.

## Lancement

**Depuis la racine du dépôt** (`CDR-Nantes/`), contrairement à `tools/` et `tests/` qui se lancent
depuis `robot1/rasp/`. Prérequis : `pip install -e .` à la racine.

```bash
python -m robot1.sim2d.run --scenario carre --no-noise
python -m robot1.sim2d.run --scenario ligne
python -m robot1.sim2d.run --scenario match --color YELLOW
python -m robot1.sim2d.run --scenario match --speed 1 --rerun
```

Pas besoin de régler `ROBOT_MODE` : `run.py` force `sim2d` avant tout import, ce qui garantit
aussi qu'un run simulé n'ouvre jamais un port série par accident.

## Options

| Option | Défaut | Effet |
|---|---|---|
| `--scenario carre\|ligne\|match` | `carre` | voir ci-dessous |
| `--color BLUE\|YELLOW` | `BLUE` | couleur d'équipe (scénario `match`), injectée à la place du switch |
| `--seed N` | `1` | graine du bruit. Même graine = même run, au chiffre près |
| `--speed max\|<float>` | `max` | `max` = aussi vite que le CPU le permet ; `20` = 20 s simulées par seconde réelle |
| `--no-noise` | bruit actif | supprime la marche aléatoire, ne laisse que la dérive systématique |
| `--rerun` | non | ouvre la vue Rerun |
| `--accel A` | `900` | accélération du profil du scénario `ligne`, mm/s² |
| `--budget S` | `1.0` | budget de temps mur pour un match, en secondes |
| `--verbose` | non | logs de la boucle de production |

## Les trois scénarios

Ils répondent à des questions différentes, et chacun **imprime un verdict et sort en code retour
non nul en cas d'échec** — pour être enchaînés dans une campagne de runs sans framework de test.

**`carre`** teste le simulateur lui-même. Il pilote le lien directement, sans pathfinding ni
stratégie : 4 côtés de 800 mm, un quart de tour à chaque coin. Il vérifie que la cinématique déplace
le robot et que l'odométrie dérive exactement comme le modèle le dit. Avec `--no-noise` la
vérification est exacte ; sinon la tolérance est dérivée du modèle de bruit (3 sigma).

**`ligne`** teste la loi de commande de la Teensy. Trois lignes droites (0°, 30°, 90°, 1,2 à
1,6 m), chacune pilotée par un profil trapézoïdal envoyé dans `SET_TARGET` à chaque tour : pose du
profil et vitesse prise une demi-période plus loin (voir Pièges). Il vérifie que la base atteint
1 m/s et qu'elle s'arrête sans dépasser le but au-delà de la zone morte du firmware (10 mm),
mesuré sur l'odométrie, ce que la Teensy asservit. Le profil est écrit pour ce scénario seulement :
ce n'est pas le générateur de la boucle de match.

**`match`** teste la boucle de production, inchangée, sur les 100 s réglementaires : couleur,
tirette, stratégie, pathfinding, arrêt. Il vérifie qu'elle survit et mesure ce qu'elle coûte,
en séparant le temps passé dans la simulation de celui passé dans `app.py::update()`.

## Lire la sortie

Deux poses, et il ne faut jamais les confondre :

- la **pose vraie** est où le robot est réellement. Seul le simulateur la connaît.
- la **pose odométrique** est ce que la Teensy rapporterait, dérive comprise. C'est la seule que la
  boucle de match voit.

L'écart entre les deux est l'intérêt de l'exercice. Dans la vue Rerun, la pose vraie est en cyan
(`world/robot/truth`), l'estimée dans les entités habituelles du pont, et les courbes d'erreur sont
sous `data/sim2d/`.

## Ce qui est simulé, et ce qui ne l'est pas

**Simulé** :

- géométrie de table, cinématique holonome 3 roues (W1 120°, W2 240°, W3 0°, rayon 160 mm) ;
- **la loi de commande de la Teensy** (`teensy.py`), copie ligne à ligne de
  `Holonomic_Basis::handle()` et de `WheelCommandShaper::step()` (`lib/motion`), exécutée toutes
  les 15 ms comme l'ISR, sur la pose odométrique :
  - cible extrapolée à la vitesse de `SET_TARGET`, vitesse expirée après 150 ms (la référence
    s'arrête alors au dernier point extrapolé) ;
  - zone morte 10 mm / 0,15 rad, dans laquelle l'anticipation reste ;
  - P + anticipation, plafond de freinage `√(2·a·d)` sous sa forme par tick, cinématique
    inverse, normalisation à 350 rpm, limiteur synchronisé à 320 rpm/s par roue ;
- dérive d'odométrie ;
- le protocole USB au niveau message : `SET_TARGET` (pose et vitesse), `SET_ODOMETRIE`, et
  `ODOMETRY` à 33 Hz, un échantillon tous les deux ticks de commande comme le firmware de match
  prévu. Une boucle à 20 Hz en reçoit une ou deux par tour. La vitesse publiée est la vitesse
  vraie en repère table, faute d'estimateur de vitesse simulé.

La boucle de position suit le firmware de match **spécifié**, pas le firmware de test
`src/main.cpp` : sorties bornées en unités physiques (±1000 mm/s, ±3 rad/s), là où `main.cpp`
borne à ±350 et donne au PID θ une zone morte de sortie de 2 rad/s (avec `KP_THETA = 1`, aucune
rotation sous 2 rad d'erreur). La zone morte X/Y de 5 mm/s est reprise. Toute modification de
`handle()` ou de `lib/motion` est à reporter dans `teensy.py`.

**Pas simulé** : les drivers MKS (une roue tourne exactement à sa consigne, sans boucle interne
ni rampe propre), la latence RS485 et USB, l'estimateur de pose, l'EKF, la physique des
actionneurs, le glissement des roues, la 3D. Le simulateur dit si la boucle de match pilote
correctement la loi de commande 2027 ; le temps de stabilisation et le dépassement du vrai robot,
et les gains, se valident au banc.

Trois niveaux de confiance dans les constantes, à connaître avant de croire un résultat :

1. **Démontré** — la cinématique inverse et directe sont réciproques à 1e-14. C'est de la géométrie.
2. **Issu du firmware** — rayon, diamètre de roue, 350 rpm, 320 rpm/s par roue, 1000 mm/s,
   3 rad/s, 1000 mm/s², `KP = 1`, période de 15 ms, zone morte, 150 ms : lus dans
   `teensy_moteur/include/config.h` et `holonomic_basis.cpp`. Le gain lui-même reste à régler sur
   le robot.
3. **Inventé** — **tout le modèle de dérive**. Valeurs plausibles, jamais mesurées. Elles seront
   recalées sur de vrais logs du robot, une fois ceux-ci rejouables (`replay`).

## Modules

| Fichier | Rôle |
|---|---|
| `clock.py` | `VirtualClock` (n'attend jamais) et `ScaledClock` (cadencée sur le temps réel) |
| `kinematics.py` | cinématique inverse et directe des 3 roues, constantes de `config.h` |
| `teensy.py` | `TeensyController` : loi de commande du firmware, de `SET_TARGET` aux rpm des roues |
| `robot.py` | `SimulatedRobot` : pose vraie, pose odométrique, `DriftModel`, ticks de 15 ms, échantillons `ODOMETRY` |
| `transport.py` | `Sim2dCom`, retourné par `get_com_class()` en mode `sim2d` |
| `viz.py` | vue Rerun, best effort : son absence n'empêche jamais un run |
| `run.py` | ligne de commande et scénarios |

## Ajouter un scénario

1. Écrire `scenario_<nom>(args) -> int` dans `run.py` : construire l'horloge avec `build_clock()`,
   le robot avec `build_robot()`, puis mener la boucle.
2. **La boucle appelle toujours, dans cet ordre** : `sim.step(TICK_S)`,
   `com.publish_odometry(clock.now())`, puis le code testé, puis `clock.sleep(TICK_S)`.
   La boucle reste maîtresse du temps — rien ne doit dormir ailleurs.
3. Ajouter le nom aux `choices` de `--scenario`, et imprimer un verdict chiffré avec un code retour.

## Pièges

- **Profil à 1000 mm/s² = limiteur saturé.** Le limiteur par roue (320 rpm/s) autorise
  1005 mm/s² à 90° et à 30°, où une roue porte toute la vitesse : un profil à 1000 ne laisse
  aucune marge à la boucle de position pendant les rampes. Le retard s'accumule, et plus Kp est
  grand, plus le dépassement est grand, parce que le plafond de freinage ne voit que la distance à
  la référence mobile, pas au but : 19 mm avec `KP = 1`, 120 mm avec `KP = 4`, 169 mm avec
  `KP = 8`. À 900 mm/s², 10 mm au plus pour les mêmes gains. D'où `--accel 900` par défaut ;
  `--accel 1000` reproduit l'échec.
- **Envoyer la vitesse du milieu de période.** La Teensy extrapole la pose reçue à vitesse
  constante jusqu'au message suivant. Envoyer `v(t)` fait rouler le robot ~25 mm/s trop vite sur
  toute la rampe de freinage (23 mm de dépassement à 0°, `KP = 1`). Envoyer la pose `p(t)` avec
  la vitesse `v(t + 25 ms)` rend l'extrapolation exacte à accélération constante.
- **Ne jamais asservir sur la pose vraie.** Aucun contrôleur réel ne la connaît. Le premier modèle
  le faisait : comme `app.py` s'arrête en commandant « va à ta pose estimée », le robot poursuivait
  sa propre dérive et sortait de la table de 1,2 m par match.
- **Un carré fermé ne mesure pas une erreur d'échelle** : elle s'annule entre aller et retour.
  Mesurer la distance parcourue, pas la position finale.
- **Le budget « 100 s en moins d'1 s » est tenu (0,3 à 0,9 s selon la couleur), mais A\* en
  représente l'essentiel ; la simulation, 0,07 ms par tick.** Il ne tombe
  aujourd'hui que sur 25 % des ticks parce que le stub de stratégie s'arrête au bout de ~25 s. Avec
  une vraie stratégie, compter 3,8 s. Correctif prévu dans la ré-API du pathfinder, pas ici.
- **Aucun scan LiDAR n'est simulé** : `match` ne teste **pas** l'évitement.
  `detection.use_pushed_scans()` est appelé pour n'ouvrir aucun LiDAR, et rien n'est poussé. Ne
  rien conclure sur l'évitement tant que le simulateur ne produit pas de scans.
