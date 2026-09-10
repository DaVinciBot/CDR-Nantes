# comm/

Point d'entree du lien de communication avec la Teensy.

```text
comm/
├── __init__.py      # re-exports
└── context.py       # mode d'execution + creation de l'instance Com
```

La liaison USB elle-meme vit dans `common/usb_com/` (package `usb_com`, rendu importable
par `pip install -e .` a la racine du depot). `comm/` ne porte que les decisions cote
robot : quel mode tourne, et quelle classe de transport cela implique.

## Utilisation

```python
from comm import init_robot

com, mode = init_robot(logger)   # mode vaut "HARDWARE" ou "DUMMY"
```

## API

| Symbole | Role |
| --------- | ------ |
| `init_robot(logger=None)` | Ouvre le lien et retourne `(com, mode)`. Point d'entree normal. |
| `create_com(logger=None)` | Construit l'instance `Com` sans journaliser le mode. |
| `get_mode()` | Retourne `"hardware"` ou `"dummy"` d'apres `ROBOT_MODE`. Leve `ValueError` sinon. |
| `get_com_config()` | `serial_config` de `config.json`, plus `mode` et `enable_dummy`. |
| `get_com_class()` | Classe de transport a instancier. Point de branchement du futur mode `sim2d`. |
| `CONFIG_PATH` | Chemin de `config.json`, expose pour les outils de diagnostic. |

Le detail des modes et du comportement en cas d'echec est dans la section
« Mode d'execution » du [README parent](../README.md).

## Ajouter un mode `sim2d`

Deux endroits dans `context.py`, et rien d'autre : ajouter `"sim2d"` a `VALID_MODES`,
puis retourner sa classe de transport dans `get_com_class()`.
