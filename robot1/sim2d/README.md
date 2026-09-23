# sim2d — simulateur 2D cinématique

Fait tourner **le code de production de `robot1/rasp/` sans robot**. Seuls les backends changent :
le lien USB vers la Teensy est remplacé par un robot simulé, les GPIO par des boutons simulés,
l'horloge murale par une horloge virtuelle.

## Lancement

**Depuis la racine du dépôt** (`CDR-Nantes/`), contrairement à `tools/` et `tests/` qui se lancent
depuis `robot1/rasp/`. Prérequis : `pip install -e .` à la racine.

```bash
python -m robot1.sim2d.run --scenario carre --no-noise
python -m robot1.sim2d.run --scenario match --color YELLOW
python -m robot1.sim2d.run --scenario match --speed 1 --rerun
```

Pas besoin de régler `ROBOT_MODE` : `run.py` force `sim2d` avant tout import, ce qui garantit
aussi qu'un run simulé n'ouvre jamais un port série par accident.

## Options

| Option | Défaut | Effet |
|---|---|---|
| `--scenario carre\|match` | `carre` | voir ci-dessous |
| `--color BLUE\|YELLOW` | `BLUE` | couleur d'équipe (scénario `match`), injectée à la place du switch |
| `--seed N` | `1` | graine du bruit. Même graine = même run, au chiffre près |
| `--speed max\|<float>` | `max` | `max` = aussi vite que le CPU le permet ; `20` = 20 s simulées par seconde réelle |
| `--no-noise` | bruit actif | supprime la marche aléatoire, ne laisse que la dérive systématique |
| `--rerun` | non | ouvre la vue Rerun |
| `--budget S` | `1.0` | budget de temps mur pour un match, en secondes |
| `--verbose` | non | logs de la boucle de production |

## Les deux scénarios

Ils répondent à deux questions différentes, et chacun **imprime un verdict et sort en code retour
non nul en cas d'échec** — pour être enchaînés dans une campagne (lot 5) sans framework de test.

**`carre`** teste le simulateur lui-même. Il pilote le lien directement, sans pathfinding ni
stratégie : 4 côtés de 800 mm, un quart de tour à chaque coin. Il vérifie que la cinématique déplace
le robot et que l'odométrie dérive exactement comme le modèle le dit. Avec `--no-noise` la
vérification est exacte ; sinon la tolérance est dérivée du modèle de bruit (3 sigma).

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

**Simulé** : géométrie de table, cinématique holonome 3 roues (W1 120°, W2 240°, W3 0°, rayon
160 mm), plafonds de vitesse des roues, dérive d'odométrie, et le protocole USB au niveau message
(`SET_TARGET_POSITION`, `SET_ODOMETRIE`, `UPDATE_ROLLING_BASIS` à 10 Hz comme la vraie Teensy).

**Pas simulé, et ça ne le sera pas** : le firmware, l'EKF, les steppers, la physique des
actionneurs, la 3D, aucun moteur physique. Le déplacement est une approche proportionnelle bornée,
**pas le PID de `holonomic_basis.cpp`** : rien ici ne prédit un dépassement, un temps de
stabilisation ou un glissement de roue. Ça se valide au banc et sur le robot.

Trois niveaux de confiance dans les constantes, à connaître avant de croire un résultat :

1. **Démontré** — la cinématique inverse et directe sont réciproques à 1e-14. C'est de la géométrie.
2. **Issu du matériel** — rayon, diamètre de roue, 100 rpm, 200 mm/s, 2 rad/s, lus dans
   `teensy_moteur/include/config.h`.
3. **Inventé** — gains d'approche, accélérations, zones mortes, et **tout le modèle de dérive**.
   Valeurs plausibles, jamais mesurées. Elles seront recalées sur de vrais logs au lot 8 (`replay`).

À noter, trouvé en écrivant le bridage : à 100 rpm sur des roues de 60 mm une roue plafonne à
314,2 mm/s, donc la rotation pure plafonne à **1,963 rad/s**. Le `ROBOT_MAX_RAD_S = 2.0` de
`config.h` est donc 1,9 % au-dessus du réalisable.

## Modules

| Fichier | Rôle |
|---|---|
| `clock.py` | `VirtualClock` (n'attend jamais) et `ScaledClock` (cadencée sur le temps réel) |
| `kinematics.py` | cinématique inverse et directe des 3 roues, bridage proportionnel |
| `robot.py` | `SimulatedRobot` : pose vraie, pose odométrique, `DriftModel` |
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

- **Ne jamais asservir sur la pose vraie.** Aucun contrôleur réel ne la connaît. Le premier modèle
  le faisait : comme `app.py` s'arrête en commandant « va à ta pose estimée », le robot poursuivait
  sa propre dérive et sortait de la table de 1,2 m par match.
- **Un carré fermé ne mesure pas une erreur d'échelle** : elle s'annule entre aller et retour.
  Mesurer la distance parcourue, pas la position finale.
- **Le budget « 100 s en moins d'1 s » est tenu à 0,98 s, mais A\* en représente 98 %.** Il ne tombe
  aujourd'hui que sur 25 % des ticks parce que le stub de stratégie s'arrête au bout de ~25 s. Avec
  une vraie stratégie, compter 3,8 s. Correctif prévu dans la ré-API du pathfinder, pas ici.
- **Le lot 2 n'est pas fait** : aucun scan LiDAR n'est produit, donc `match` ne teste **pas**
  l'évitement. `detection.use_pushed_scans()` est appelé pour n'ouvrir aucun LiDAR, et rien n'est
  poussé. Ne pas conclure quoi que ce soit sur l'évitement avant le lot 2.
