#!/usr/bin/env python3
"""
Module de détection automatique : Simulation vs Hardware réel.

Permet d'utiliser le même code Python pour la simulation Webots et le robot réel.
La détection se fait automatiquement selon le contexte d'exécution.
"""

import os
import sys
from pathlib import Path


def is_simulation() -> bool:
    """Détecte si on est en mode simulation ou sur le robot réel.
    
    Critères de détection :
    1. Variable d'environnement ROBOT_MODE=simulation
    2. Fichier .simulation_mode présent
    3. Exécution depuis le dossier simulation/
    
    Returns:
        bool: True si en simulation, False si sur hardware réel
    """
    # 1. Variable d'environnement (priorité maximale)
    robot_mode = os.environ.get('ROBOT_MODE', '').lower()
    if robot_mode == 'simulation':
        return True
    elif robot_mode == 'hardware':
        return False
    
    # 2. Fichier marqueur .simulation_mode
    current_dir = Path.cwd()
    if (current_dir / '.simulation_mode').exists():
        return True
    
    # 3. Détection par chemin (si on est dans simulation/)
    if 'simulation' in str(current_dir).lower():
        return True
    
    # Par défaut : hardware réel
    return False


def get_com_config() -> dict:
    """Retourne la configuration COM adaptée au contexte.
    
    Returns:
        dict: Configuration avec les paramètres appropriés
    """
    if is_simulation():
        # Configuration simulation (port COM virtuel)
        return {
            'mode': 'simulation',
            'port': 'COM1',
            'serial_number': None,
            'vid': None,
            'pid': None,
            'baudrate': 115200,
            'enable_crc': True,
            'enable_dummy': False
        }
    else:
        # Configuration hardware réel (USB Teensy)
        return {
            'mode': 'hardware',
            'port': None,  # Détection automatique
            'serial_number': 18421350,
            'vid': 5824,
            'pid': 1155,
            'baudrate': 115200,
            'enable_crc': True,
            'enable_dummy': False
        }


def get_com_class():
    """Retourne la classe Com appropriée selon le contexte.
    
    Returns:
        class: WebotsComBridge en simulation, Com en hardware
    """
    if is_simulation():
        # Import de la classe simulation
        sys.path.insert(0, str(Path(__file__).parent))
        from webots_com import WebotsComBridge
        return WebotsComBridge
    else:
        # Import de la classe hardware réelle
        from loader import loader
        return loader.load_class('usb_com', 'Com')


def create_com(logger=None):
    """Crée une instance Com adaptée au contexte (simulation ou hardware).
    
    Args:
        logger: Logger optionnel pour les messages
        
    Returns:
        Com instance configurée pour le contexte actuel
    """
    config = get_com_config()
    ComClass = get_com_class()
    
    if config['mode'] == 'simulation':
        # Instanciation pour simulation (WebotsComBridge)
        return ComClass(
            port=config['port'],
            baudrate=config['baudrate'],
            enable_crc=config['enable_crc'],
            logger=logger
        )
    else:
        # Instanciation pour hardware réel (Com)
        return ComClass(
            logger=logger,
            serial_number=config['serial_number'],
            vid=config['vid'],
            pid=config['pid'],
            baudrate=config['baudrate'],
            enable_crc=config['enable_crc'],
            enable_dummy=config['enable_dummy']
        )


if __name__ == '__main__':
    # Test de détection
    print(f"Mode détecté : {'SIMULATION' if is_simulation() else 'HARDWARE'}")
    print(f"Configuration : {get_com_config()}")
