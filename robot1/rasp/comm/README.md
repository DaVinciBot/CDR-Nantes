# comm/

Point d'entree du lien de communication avec la Teensy.

```text
comm/
├── __init__.py      # re-exports
├── context.py       # mode d'execution + creation de l'instance Com
└── pose_stub.py     # mode pose_stub : source de pose generee, sans robot
```

La liaison USB elle-meme vit dans `common/usb_com/` (package `usb_com`, rendu importable
par `pip install -e .` a la racine du depot). `comm/` ne porte que les decisions cote
robot : quel mode tourne, et quelle classe de transport cela implique.

## Utilisation

```python
from robot1.rasp.comm import init_robot

com, mode = init_robot(logger)   # "HARDWARE", "DUMMY" ou "POSE_STUB"
```

## Mode `pose_stub`

```bash
ROBOT_MODE=pose_stub python -m telemetry.rerun_bridge
```

Emet une pose qui bouge, sans robot, sans Teensy et sans EKF : un cercle de 400 mm
parcouru a vitesse constante, cap tangent, publie en `UPDATE_ROLLING_BASIS` a 50 Hz.
Sert a developper le haut niveau contre le contrat de pose des aujourd'hui.

Les trames sont construites avec le vrai `encode_frame` et injectees dans le vrai
thread recepteur : le stub exerce le decodeur reel, il ne l'imite pas. Rien n'est
modifie dans `common/usb_com/`, et `app.py` n'est pas touche non plus.

Pas encore de champ `sigma` : il n'a de sens qu'avec un EKF regle (reglage
prevu en janvier), et une `sigma` non validee est pire qu'absente puisqu'on lui fait
confiance. Le rejeu de log attend le format de log JSONL de `sim2d` ; d'ici la,
surcharger `pose_at()` suffit.

Detail et justifications : `pose_stub.py`.

## API

| Symbole | Role |
| --------- | ------ |
| `init_robot(logger=None)` | Ouvre le lien et retourne `(com, mode)`. Point d'entree normal. |
| `create_com(logger=None)` | Construit l'instance `Com` sans journaliser le mode. |
| `get_mode()` | Retourne `"hardware"`, `"dummy"` ou `"pose_stub"` d'apres `ROBOT_MODE`. Leve `ValueError` sinon. |
| `get_com_config()` | `serial_config` de `config.json`, plus `mode` et `enable_dummy`. |
| `get_com_class()` | Classe de transport a instancier. Point de branchement du futur mode `sim2d`. |
| `CONFIG_PATH` | Chemin de `config.json`, expose pour les outils de diagnostic. |

Le detail des modes et du comportement en cas d'echec est dans la section
« Mode d'execution » du [README parent](../README.md).

## Ajouter un mode `sim2d`

Deux endroits dans `context.py`, et rien d'autre : ajouter `"sim2d"` a `VALID_MODES`,
puis retourner sa classe de transport dans `get_com_class()`. C'est exactement ce qu'a
fait `pose_stub` : le prendre comme modele.
