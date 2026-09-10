"""LiDAR perception: opponent detection and beacon-based pose correction.

Three layers, each in its own module:

    detection.py     opponent detection only (thread + clustering + smoothing).
                     The only layer the match loop (app.py) uses today.
    localization.py  beacon extraction + SVD Umeyama pose correction.
                     Computed and exposed, but NOT applied by app.py.
    interface.py     LidarInterface, object wrapper over localization.
    gui.py           Tkinter/matplotlib debug view  ->  python -m vision

Importing this package pulls in numpy and rplidar (localization.py needs them).
The GUI is optional on purpose: a headless Raspberry Pi has no Tk.
"""

from . import detection
from .interface import LidarInterface
from .localization import (
    OpponentState,
    PoseState,
    get_corrected_pose,
    get_latest_beacon_candidates,
    get_latest_opponent,
    get_latest_pose,
    get_latest_scan_data,
    start_lidar_thread,
    stop_lidar_runtime,
)

try:
    from .gui import LidarApp, run_gui
except Exception:  # tkinter / matplotlib absent (headless Raspberry Pi)
    LidarApp = None

    def run_gui(*_args, **_kwargs):
        raise RuntimeError("Lidar GUI dependencies are unavailable in this environment.")


__all__ = [
    "LidarApp",
    "LidarInterface",
    "OpponentState",
    "PoseState",
    "detection",
    "get_corrected_pose",
    "get_latest_beacon_candidates",
    "get_latest_opponent",
    "get_latest_pose",
    "get_latest_scan_data",
    "run_gui",
    "start_lidar_thread",
    "stop_lidar_runtime",
]
