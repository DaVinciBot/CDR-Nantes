"""Reusable lidar module for holonomic robot projects."""

try:
    from .lidar_logic import (
        start_lidar_thread,
        stop_lidar_runtime,
        get_latest_scan_data,
        get_latest_beacon_candidates,
        get_latest_pose,
        get_latest_opponent,
        PoseState,
        OpponentState,
    )
except ImportError:
    from lidar_logic import (
        start_lidar_thread,
        stop_lidar_runtime,
        get_latest_scan_data,
        get_latest_beacon_candidates,
        get_latest_pose,
        get_latest_opponent,
        PoseState,
        OpponentState,
    )

# Navigation bridge for pathfinding integration
try:
    from .lidar_navigation_bridge import (
        LidarNavigationBridge,
        OpponentTrack,
        PathfindingState,
    )
except ImportError:
    from lidar_navigation_bridge import (
        LidarNavigationBridge,
        OpponentTrack,
        PathfindingState,
    )

try:
    from .lidar_gui import LidarApp, run_gui
except Exception:
    try:
        from lidar_gui import LidarApp, run_gui
    except Exception:
        LidarApp = None

        def run_gui(*_args, **_kwargs):
            raise RuntimeError("Lidar GUI dependencies are unavailable in this environment.")

__all__ = [
    "start_lidar_thread",
    "stop_lidar_runtime",
    "get_latest_scan_data",
    "get_latest_beacon_candidates",
    "LidarNavigationBridge",
    "OpponentTrack",
    "PathfindingState",
    "LidarApp",
    "run_gui",
]
