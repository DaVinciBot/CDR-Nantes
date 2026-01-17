# 🔄 Basculer entre Simulation et Hardware

## Utilisation simple
Faire la commande python switch_mode.py ( ou python3) pour changer de mode que ce soit sur pc ou la raspery 

## Configuration modifiée

Le script modifie `config.json` :

**Simulation** : port COM1, pour Webots  
**Hardware** : serial_number + vid/pid, pour Teensy USB réel

Tous les scripts Python (`test_program.py`, `test_communication.py`, etc.) utilisent automatiquement la configuration active.

Pour initaliser chaque fichier python pour différence rasp/simulation 
Mettre ces quelques lignes : 

# Initialisation automatique (simulation ou hardware)
import sys
from pathlib import Path
from loader import loader

sys.path.insert(0, str(Path(__file__).parent))
from robot_context import init_robot

com, mode = init_robot(logger)

def handle_position(data: bytes) -> None:
    """Callback pour recevoir la position du robot."""
    if len(data) >= 24:
        x, y, theta = struct.unpack('<ddd', data[:24])
        logger.info(f" Position robot: X={x:.2f}mm, Y={y:.2f}mm, θ={theta:.4f}rad")
    else:
        logger.warning(f" Message trop court: {len(data)} bytes")

def send_position(x, y, theta, com, description=""):
    """Envoie une position cible au robot."""
    msg = Messages.SET_TARGET_POSITION.to_bytes()
    msg += struct.pack('<ddd', x, y, theta)
    com.send_bytes(msg)
    logger.info(f"📤 {description}: X={x}mm, Y={y}mm, θ={theta}rad")

com.add_callback(handle_position, Messages.UPDATE_ROLLING_BASIS.value)

Dectete automatique le mode a réaliser et permetre d'avoir les différentes commandes pour envoyer et recevoir les différents messages 