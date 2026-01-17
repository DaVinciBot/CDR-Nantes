#!/usr/bin/env python3
"""
Script de bascule rapide entre mode simulation et hardware.
Usage: python switch_mode.py [simulation|hardware]
"""

import json
import sys
from pathlib import Path

# Configuration simulation
SIMULATION_CONFIG = {
    "usb_com": {
        "module_path": "../../common/usb_com/python",
        "classes": {
            "Com": "com.com.Com",
            "DummyCom": "com.dummy.DummyCom",
            "Messages": "messages.Messages"
        }
    },
    "teensy": {
        "module_path": "../../common/teensy",
        "classes": {
            "BaseTeensy": "base_teensy.BaseTeensy",
            "GPIOTeensy": "gpio_teensy.GPIOTeensy"
        }
    },
    "serial_config": {
        "port": "COM1",
        "serial_number": None,
        "vid": None,
        "pid": None,
        "baudrate": 115200,
        "enable_crc": True,
        "enable_dummy": False
    }
}

# Configuration hardware
HARDWARE_CONFIG = {
    "usb_com": {
        "module_path": "../../common/usb_com/python",
        "classes": {
            "Com": "com.com.Com",
            "DummyCom": "com.dummy.DummyCom",
            "Messages": "messages.Messages"
        }
    },
    "teensy": {
        "module_path": "../../common/teensy",
        "classes": {
            "BaseTeensy": "base_teensy.BaseTeensy",
            "GPIOTeensy": "gpio_teensy.GPIOTeensy"
        }
    },
    "serial_config": {
        "serial_number": 18421350,
        "vid": 5824,
        "pid": 1155,
        "baudrate": 115200,
        "enable_crc": True,
        "enable_dummy": False
    }
}

def get_current_mode(config_path):
    """Détecte le mode actuel."""
    try:
        with open(config_path, 'r') as f:
            config = json.load(f)
        if config.get('serial_config', {}).get('port') == 'COM1':
            return 'simulation'
        else:
            return 'hardware'
    except:
        return None

def main():
    config_path = Path(__file__).parent / 'config.json'
    
    # Si argument fourni
    if len(sys.argv) > 1:
        mode = sys.argv[1].lower()
        if mode not in ['simulation', 'hardware']:
            print(" Mode invalide. Utilisez : simulation ou hardware")
            sys.exit(1)
    else:
        # Mode interactif
        current = get_current_mode(config_path)
        if current == 'simulation':
            print(" Mode actuel : SIMULATION")
            print()
            response = input("Voulez-vous basculer vers HARDWARE ? (o/N) ")
            if response.lower() in ['o', 'oui']:
                mode = 'hardware'
            else:
                print(" Annulé")
                return
        elif current == 'hardware':
            print(" Mode actuel : HARDWARE")
            print()
            response = input("Voulez-vous basculer vers SIMULATION ? (o/N) ")
            if response.lower() in ['o', 'oui']:
                mode = 'simulation'
            else:
                print(" Annulé")
                return
        else:
            print(" Impossible de détecter le mode actuel")
            return
    
    # Appliquer la configuration
    if mode == 'simulation':
        print(" Bascule vers SIMULATION...")
        config = SIMULATION_CONFIG
        with open(config_path, 'w') as f:
            json.dump(config, f, indent=4)
        print(" Mode SIMULATION activé (COM1)")
        print("   Utilisez avec Webots")
    elif mode == 'hardware':
        print(" Bascule vers HARDWARE...")
        config = HARDWARE_CONFIG
        with open(config_path, 'w') as f:
            json.dump(config, f, indent=4)
        print(" Mode HARDWARE activé (Teensy USB)")
        print("   Serial: 18421350, VID: 5824, PID: 1155")

if __name__ == '__main__':
    main()
