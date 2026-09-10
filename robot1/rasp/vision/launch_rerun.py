#!/usr/bin/env python3
"""
Launcher pour le pont Rerun — visualisation Teensy + LiDAR.

Usage (depuis robot1/rasp/) :
    # Mode simulation (test)
    python vision/launch_rerun.py --sim

    # Mode serveur (Rasp -> visualisation depuis PC distant)
    python vision/launch_rerun.py --serve --port 9876

    # Mode serveur avec LiDAR hardware
    python vision/launch_rerun.py --serve --with-lidar --port 9876

Raccourci : ce launcher ne fait que relayer vers
    python -m telemetry.rerun_bridge <args>
"""

import subprocess
import sys
from pathlib import Path

RASP_DIR = Path(__file__).resolve().parent.parent


def main():
    # Lance le pont comme module, depuis robot1/rasp : c'est ce qui rend
    # world / comm / vision importables sans toucher a sys.path.
    subprocess.run(
        [sys.executable, "-m", "telemetry.rerun_bridge"] + sys.argv[1:],
        cwd=str(RASP_DIR),
    )


if __name__ == "__main__":
    main()
