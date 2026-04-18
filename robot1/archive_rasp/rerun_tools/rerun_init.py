#!/usr/bin/env python3
"""
Initialisation de la visualisation Rerun pour Eurobot 2026.

Cet utilitaire helper facilite le lancement de Rerun depuis différents contextes.

Usage :
    from rerun_init import setup_rerun, get_rerun_bridge

    # Initialiser le serveur Rerun (mode production)
    setup_rerun(mode="serve", port=9876, with_lidar=True)

    # Ou simplement importer le bridge
    from rerun.rerun_bridge import publish_loop, log_static_map
"""

from pathlib import Path
import subprocess
import sys
import logging

logger = logging.getLogger("rerun_init")

# ─────────────────────────────────────────────────────────────────────────────
# Chemins
# ─────────────────────────────────────────────────────────────────────────────

RASP_DIR = Path(__file__).parent  # /robot1/rasp/
RERUN_DIR = RASP_DIR / "rerun"
RERUN_BRIDGE = RERUN_DIR / "rerun_bridge.py"


# ─────────────────────────────────────────────────────────────────────────────
# API publique
# ─────────────────────────────────────────────────────────────────────────────

def setup_rerun(mode: str = "serve", 
                host: str = "0.0.0.0", 
                port: int = 9876, 
                with_lidar: bool = False,
                sim: bool = False) -> None:
    """
    Lance le serveur Rerun avec les paramètres spécifiés.
    
    Args:
        mode: "local" (Viewer local), "serve" (WebSocket), ou "connect" (gRPC)
        host: Adresse bind (0.0.0.0 pour tous les interfaces)
        port: Port WebSocket ou gRPC
        with_lidar: Activer polling Lidar hardware
        sim: Mode simulation (robot en cercle)
    
    Exemple:
        setup_rerun(mode="serve", port=9876, with_lidar=True)
    """
    
    if not RERUN_BRIDGE.exists():
        raise FileNotFoundError(f"Bridge introuvable: {RERUN_BRIDGE}")
    
    cmd = [sys.executable, str(RERUN_BRIDGE), "--mode", mode, "--host", host, "--port", str(port)]
    
    if with_lidar:
        cmd.append("--with-lidar")
    if sim:
        cmd.append("--sim")
    
    logger.info(f"Lancement Rerun: {' '.join(cmd)}")
    subprocess.run(cmd)


def get_rerun_bridge():
    """
    Retourne le chemin vers rerun_bridge.py pour un import direct.
    """
    return RERUN_BRIDGE


def launch_serve_default() -> None:
    """
    Lance Rerun en mode serveur avec config par défaut (0.0.0.0:9876).
    Idéal pour production sur Raspberry Pi.
    """
    setup_rerun(mode="serve", host="0.0.0.0", port=9876)


def launch_test_local() -> None:
    """
    Lance Rerun en mode simulation local (pas de hardware nécessaire).
    """
    setup_rerun(mode="local", sim=True)


# ─────────────────────────────────────────────────────────────────────────────
# Scripts directs
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import argparse
    
    p = argparse.ArgumentParser(description="Rerun Init Helper — Eurobot 2026")
    p.add_argument("--mode", choices=["local", "serve", "connect"], default="serve")
    p.add_argument("--port", type=int, default=9876)
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--with-lidar", action="store_true")
    p.add_argument("--sim", action="store_true")
    args = p.parse_args()
    
    logging.basicConfig(level=logging.INFO)
    setup_rerun(mode=args.mode, host=args.host, port=args.port, 
                with_lidar=args.with_lidar, sim=args.sim)
