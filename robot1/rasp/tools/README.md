# tools/

Outillage, **hors chemin de production**. Rien ici n'est importe par `main.py` /
`app.py` : ce sont des scripts qu'on lance a la main.

```text
tools/
└── bringup/        bring-up materiel : liaison serie, detection USB, moteurs
```

`tools/` et `tools/bringup/` n'ont volontairement pas d'`__init__.py` : ce ne sont pas
des packages du robot, juste des namespaces pour pouvoir les lancer avec `-m` depuis
`robot1/rasp/`.

```bash
cd robot1/rasp
python -m tools.bringup.test_usb_detection
python -m tools.bringup.test_communication
```

Le `-m` est requis : c'est ce qui met `robot1/rasp` dans `sys.path`, donc ce qui rend
`from comm import get_com_config` resolvable depuis ces scripts.

Voir [bringup/README.md](bringup/README.md) pour le detail de chaque script.
