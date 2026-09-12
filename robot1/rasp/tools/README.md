# tools/

Outillage, **hors chemin de production**. Rien ici n'est importe par `main.py` /
`app.py` : ce sont des scripts qu'on lance a la main.

```text
tools/
├── bringup/        bring-up materiel : liaison serie, detection USB, moteurs
└── manual/         pilotage manuel du robot complet : consignes x/y/theta
```

Aucun de ces dossiers n'a d'`__init__.py`, volontairement : ce ne sont pas des packages
du robot, juste des namespaces pour pouvoir les lancer avec `-m` depuis `robot1/rasp/`.

```bash
cd robot1/rasp
python -m tools.bringup.test_usb_detection
python -m tools.bringup.test_communication
python -m tools.manual.move_robot
```

Le `-m` est requis : c'est ce qui met `robot1/rasp` dans `sys.path`, donc ce qui rend
`tools.bringup.<script>` resolvable. Le code du robot, lui, s'importe en
`from robot1.rasp.comm import get_com_config` depuis l'etape B (11/09/2026) : ces scripts ne
dependent plus du repertoire courant pour ca, uniquement pour se trouver eux-memes.

Voir [bringup/README.md](bringup/README.md) pour le detail de chaque script de bring-up.

## manual/

- `move_robot.py` - deplacement manuel x/y/theta, interactif ou par liste de points.
  Descendu de la racine de `rasp/` le 11/09/2026 : c'est de l'outillage, pas du chemin
  de production. `test_simple_traj.py` est reste a la racine pour l'instant.
