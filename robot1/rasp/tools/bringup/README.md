# tools/bringup/ - bring-up materiel

> Outils qu'on lance a la main, face au robot, dans l'ordre. Ils testent la **liaison** et le
> **mouvement**, pas le code Python : les tests du code sont dans [../../tests/](../../tests/).
> Toutes les commandes se lancent depuis `robot1/rasp/` (le `-m` est requis,
> cf. [../../README.md](../../README.md)).

> Etat au 11/09/2026. La version precedente de ce README documentait des options
> (`--motor`, `--steps`, `--speed`, `--duration`, `--filter`, `--no-crc`, `--timeout`,
> `--list-all`) qui **n'existent dans aucun de ces scripts** : aucun n'a d'argparse. Elle
> montrait aussi des sorties inventees. Ce qui suit decrit ce que le code fait.

## Dans quel ordre

| # | Script | Ce qu'il fait vraiment | Materiel requis |
| --- | --- | --- | --- |
| 1 | `test_usb_detection.py` | liste tous les ports serie, repere la Teensy sur `vid=0x16c0` et imprime le bloc `serial_number`/`vid`/`pid` a recopier dans `config.json` | Teensy branchee |
| 2 | `test_communication.py` | ouvre le lien via `usb_com.Com`, remet l'odometrie a zero, envoie 4 positions cibles, compte les `ODOMETRY` recus | Teensy flashee |
| 3 | `test_debug_messages.py` | meme connexion, en `DEBUG`, ecoute 15 s et hexdumpe chaque message recu | Teensy flashee |
| 4 | `test_length_messages.py` | **serie brute, sans la classe `Com`** : decoupe le flux sur le delimiteur `0x00`, decode le COBS, verifie le CRC et detaille les `ODOMETRY` (pose, vitesse, horodatage) | Teensy flashee |
| 5 | `test_one_motor.py` | envoie 4 consignes de position (X, puis X+Y, puis rotation, puis retour) et laisse 5 s entre chaque | moteurs alimentes |

Aucun de ces scripts ne prend d'argument. Le seul reglage est `ROBOT_MODE`, plus `TEENSY_PORT`
pour `test_length_messages.py`.


- **`test_one_motor.py` ne teste pas un moteur.** Il envoie des consignes holonomes
  (`SET_TARGET`) qui font tourner **les trois roues** a la fois. Il n'y a aucun moyen
  d'adresser une roue seule dans le protocole actuel. C'est un script de « premier mouvement »,
  pas un test de moteur isole.
- **`test_length_messages.py` ne mesure pas des tailles de payload par type de message.** Il
  decode la trame sur le fil. C'est le seul outil qui voit le cadrage (COBS, CRC) tel qu'il
  arrive de la Teensy.

Renommer les deux demande de les verifier devant le robot : reporte.

## Configuration

Les scripts lisent `config.json`, section `serial_config` (numero de serie, vid, pid, baudrate).
`test_length_messages.py` n'ouvre pas le lien par `Com` mais **retrouve le port avec la meme regle**
(vid + pid + numero de serie) : il portait `/dev/ttyACM0` en dur jusqu'au 11/09/2026. Pour forcer
un device :

```bash
TEENSY_PORT=/dev/ttyACM0 python -m tools.bringup.test_length_messages
```

Le mode d'execution est explicite et se lit dans `ROBOT_MODE` : `hardware` (defaut) ou `dummy`.
Toute autre valeur leve `ValueError`.

### Sans Teensy branchee (poste de dev)

```bash
ROBOT_MODE=dummy python -m tools.bringup.test_communication
```

Le lien devient une boucle locale : les envois partent, **rien ne revient** (le dummy n'emet pas
d'`ODOMETRY`), donc le script finit sur « aucun message recu ». C'est le
comportement attendu hors robot, pas une panne.

Sans cette variable, l'absence de Teensy leve `ComError: No Device found!`. C'est voulu : un robot
qui ne parle pas a sa Teensy ne doit pas sembler demarrer.

## Checklist de validation

1. [ ] `test_usb_detection.py` -> Teensy detectee, identifiants coherents avec `config.json`
2. [ ] `test_communication.py` -> messages envoyes **et** recus non nuls
3. [ ] `test_length_messages.py` -> COBS et CRC OK, `ODOMETRY` de la taille attendue
4. [ ] `test_one_motor.py` -> les trois roues tournent, la position remontee evolue
5. [ ] `python -m tools.manual.move_robot` -> pilotage interactif x/y/theta

La fiche de validation complete du lien USB (T-USB-01, T-USB-02) est tenue dans le suivi de
validation interne, hors depot.

## Depannage

**Port serie introuvable.** Lancer `test_usb_detection.py` : il liste *tous* les ports, meme non
Teensy. Sous Linux, verifier l'appartenance au groupe `dialout` :
`sudo usermod -a -G dialout $USER`, puis rouvrir la session.

**CRC invalide.** Le CRC se desactive dans `config.json` (`"enable_crc": false`), pas en ligne de
commande. A ne faire qu'en debug, et a remettre ensuite.

**Rien ne revient de la Teensy.** Verifier dans l'ordre : firmware flashe, baudrate 115200,
et que le firmware emet bien des `ODOMETRY`. `test_length_messages.py` tranche : s'il ne voit
aucune trame (aucun `0x00`), rien ne sort de la Teensy. Le firmware et la Rasp doivent avoir le
meme format de trame : une Teensy flashee avant le passage au COBS n'est pas comprise.

## Ce qui a disparu le 11/09/2026

`test_serial_raw.py`, **copie octet pour octet de `test_usb_detection.py`** (meme MD5). Son nom
promettait un dump serie brut, il listait des ports USB. Le vrai dump brut est
`test_length_messages.py`.

## Ajouter un nouveau test

Aucun `sys.path` a bricoler : `usb_com` vient de `pip install -e .` (racine du depot) et
`robot1.rasp.comm` s'importe de partout depuis l'etape B.

```python
#!/usr/bin/env python3
"""Description du test."""

import logging

from usb_com import Com, Messages, pack_message, payload_size, unpack_message

from robot1.rasp.comm import get_com_config
```
