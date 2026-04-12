#!/usr/bin/env python3
"""
Launcher pour Rerun Bridge — Visualisation Teensy + Lidar.

Usage (depuis /robot1/rasp/) :
    # Mode simulation (test)
    python lidar/launch_rerun.py --sim

    # Mode serveur (Rasp → visualisation depuis PC distant)
    python lidar/launch_rerun.py --serve --port 9876

    # Mode serveur avec Lidar hardware
    python lidar/launch_rerun.py --serve --with-lidar --port 9876
"""

import subprocess
import sys
from pathlib import Path

def main():
    # Le script se trouve dans lidar/, remonter vers rerun/
    current_dir = Path(__file__).parent  # robot1/rasp/lidar/
    rerun_script = current_dir.parent / "rerun" / "rerun_bridge.py"
    
    if not rerun_script.exists():
        print(f"❌ Erreur: {rerun_script} introuvable")
        sys.exit(1)
    
    # Passer tous les arguments au bridge
    subprocess.run([sys.executable, str(rerun_script)] + sys.argv[1:])

if __name__ == "__main__":
    main()
