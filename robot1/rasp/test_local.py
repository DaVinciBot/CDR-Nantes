#!/usr/bin/env python3
"""
Test local Rerun — Aucune Raspberry Pi nécessaire.

Lance une simulation avec un robot qui tourne en cercle
pour valider que la carte et le robot s'affichent correctement.

Usage:
    python test_local.py
"""

import sys
import subprocess
from pathlib import Path

# Vérifier que rerun-sdk est installé
try:
    import rerun as rr
except ImportError:
    print("rerun-sdk non installé.")
    print("Installe avec : pip install rerun-sdk")
    sys.exit(1)

# Lancer le bridge en mode local + simulation
_DIR = Path(__file__).parent
bridge_script = _DIR / "rerun" / "rerun_bridge.py"

subprocess.run([
    sys.executable, str(bridge_script),
    "--mode", "local",
    "--sim",
])
