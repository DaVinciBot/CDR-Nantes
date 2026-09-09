# utils/

Point d'entree du lien de communication avec la Teensy.

```text
utils/
├── __init__.py         # re-exports
└── robot_context.py    # mode d'execution + creation de l'instance Com
```

## Utilisation

```python
from utils import init_robot

com, mode = init_robot(logger)   # mode vaut "HARDWARE" ou "DUMMY"
```

## API

| Symbole | Role |
|---------|------|
| `init_robot(logger=None)` | Ouvre le lien et retourne `(com, mode)`. Point d'entree normal. |
| `create_com(logger=None)` | Construit l'instance `Com` sans journaliser le mode. |
| `get_mode()` | Retourne `"hardware"` ou `"dummy"` d'apres `ROBOT_MODE`. Leve `ValueError` sinon. |
| `get_com_config()` | `serial_config` de `config.json`, plus `mode` et `enable_dummy`. |
| `get_com_class()` | Classe de transport a instancier. Point de branchement du futur mode `sim2d`. |

Le detail des modes et du comportement en cas d'echec est dans la section
« Mode d'execution » du [README parent](../README.md).

## Ajouter un mode `sim2d`

Deux endroits dans `robot_context.py`, et rien d'autre : ajouter `"sim2d"` a `VALID_MODES`,
puis retourner sa classe de transport dans `get_com_class()`.
