#!/usr/bin/env python3
"""
Module de détection automatique : Simulation vs Hardware réel.

Permet d'utiliser le même code Python pour la simulation Webots et le robot réel.
La détection se fait automatiquement selon le contexte d'exécution.
"""

import os
import sys
import json
from pathlib import Path


def is_simulation() -> bool:
    """Détecte si on est en mode simulation ou sur le robot réel.
    
    Critères de détection (par ordre de priorité) :
    1. Variable d'environnement ROBOT_MODE=simulation
    2. Fichier .simulation_mode présent
    3. Lecture de config.json (port=COM1 → simulation)
    4. Exécution depuis le dossier simulation/
    
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
    
    # 3. Lecture de config.json (chercher dans le parent)
    config_file = Path(__file__).parent.parent / 'config.json'
    if config_file.exists():
        try:
            with open(config_file, 'r') as f:
                config = json.load(f)
            # Si port=COM1 → simulation
            if config.get('serial_config', {}).get('port') == 'COM1':
                return True
            # Si serial_number présent → hardware
            if config.get('serial_config', {}).get('serial_number') is not None:
                return False
        except:
            pass
    
    # 4. Détection par chemin (si on est dans simulation/)
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
        # Import de la classe simulation depuis utils
        from .webots_com import WebotsComBridge
        return WebotsComBridge
    else:
        # Import de la classe hardware réelle
        sys.path.insert(0, str(Path(__file__).parent.parent))
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


def init_robot(logger=None):
    """Initialise le robot avec détection automatique du mode.
    
    Simplifie l'initialisation en une seule ligne dans vos programmes.
    
    Args:
        logger: Logger optionnel
        
    Returns:
        tuple: (com, mode_string) où mode_string est "SIMULATION" ou "HARDWARE"
        
    Example:
        from utils.robot_context import init_robot
        com, mode = init_robot(logger)
        logger.info(f"Mode: {mode}")
    """
    mode_str = "SIMULATION" if is_simulation() else "HARDWARE"
    if logger:
        logger.info(f"🤖 Mode détecté : {mode_str}")
        logger.info("=" * 70)
    
    com = create_com(logger=logger)
    
    if logger:
        logger.info("✅ Connexion établie!")
        logger.info("=" * 70)
    
    return com, mode_str


# Alias pour compatibilité
get_config = get_com_config
RobotContext = create_com


if __name__ == '__main__':
    # Test de détection
    print(f"Mode détecté : {'SIMULATION' if is_simulation() else 'HARDWARE'}")
    print(f"Configuration : {get_com_config()}")
