"""Backward-compatible launcher.

Deprecated: use lidar_gui.py or main.py.
"""

try:
    from .lidar_gui import LidarApp, run_gui
except ImportError:
    from lidar_gui import LidarApp, run_gui


if __name__ == "__main__":
    run_gui()
