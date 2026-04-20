"""Bridge module: Lidar pose + opponent tracking for pathfinding.

This module is headless and can be used without launching the GUI.
It provides a single integration surface to:
- expose robot pose from lidar (via PoseEngine in lidar_logic)
- track opponent from lidar point clouds
- package state for pathfinding layer
"""

import math
import os
import sys
import threading
import time
from dataclasses import asdict, dataclass
from typing import Any, Dict, Optional, Sequence, Tuple

ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

# Import pose and scan data from lidar_logic
try:
    from .lidar_logic import (
        get_corrected_pose,
        get_latest_opponent,
        get_latest_scan_data,
        PoseState,
        OpponentState,
    )
except ImportError:
    from lidar_logic import (
        get_corrected_pose,
        get_latest_opponent,
        get_latest_scan_data,
        PoseState,
        OpponentState,
    )


@dataclass
class OpponentTrack:
    """Latest tracked opponent state in world coordinates."""

    x: float
    y: float
    vx: float
    vy: float
    radius: float
    confidence: float
    timestamp: float


@dataclass
class PathfindingState:
    """Payload consumed by pathfinding."""

    robot_x: float
    robot_y: float
    robot_theta: float
    robot_confidence: float
    robot_nb_beacons: int
    robot_last_lidar_age: float
    localization_stale: bool
    opponent: Optional[OpponentTrack]
    timestamp: float

    def as_dict(self) -> Dict[str, Any]:
        payload = asdict(self)
        return payload


class LidarNavigationBridge:
    """Simplified bridge exposing lidar-based pose and opponent data for pathfinding.
    
    All pose calculation (trilateration, beacon fitting) is now handled by
    PoseEngine in lidar_logic.py. This class simply retrieves and packages
    the calculated pose and opponent coordinates.

    Typical usage:
        bridge = LidarNavigationBridge()
        
        # Get fused state for pathfinding
        state = bridge.get_state()
        opp_xy = bridge.get_opponent_coordinates()
        
        # Or get serialized payload
        payload = bridge.get_pathfinding_payload()
    """

    def __init__(self, team_color: str = "blue", opponent_timeout_s: float = 0.70):
        """Initialize bridge (team_color kept for API compatibility)."""
        self._opponent_timeout_s = max(0.1, float(opponent_timeout_s))
        self._lock = threading.Lock()
        self._last_opponent: Optional[OpponentTrack] = None

    def initialize_pose(self, x_mm: float, y_mm: float, theta_rad: float) -> None:
        """DEPRECATED: Pose is now calculated by PoseEngine, not set manually."""
        pass

    def set_team_color(self, color: str) -> None:
        """DEPRECATED: Team color is handled by BeaconLayout in terrain_jeu.py."""
        pass

    def update_teensy_odometry(
        self,
        x_mm: float,
        y_mm: float,
        theta_rad: float,
        timestamp: Optional[float] = None,
    ) -> None:
        """DEPRECATED: Odometry fusion not yet implemented.
        Currently only lidar pose is available."""
        pass

    def process_lidar_scan(self, scan_points) -> None:
        """DEPRECATED: Scans are processed by lidar_logic runtime.
        Call get_corrected_pose() instead."""
        pass

    def get_state(self) -> PathfindingState:
        """Return latest lidar-based robot state plus tracked opponent."""
        lidar_pose = get_corrected_pose()
        lidar_opponent = get_latest_opponent()
        now = time.time()
        
        # Convert PoseState to PathfindingState
        return PathfindingState(
            robot_x=float(lidar_pose.x),
            robot_y=float(lidar_pose.y),
            robot_theta=float(lidar_pose.theta),
            robot_confidence=float(lidar_pose.confidence),
            robot_nb_beacons=len(lidar_pose.beacon_ids or []),
            robot_last_lidar_age=0.0,  # Not tracked yet
            localization_stale=not lidar_pose.is_localized,
            opponent=self._convert_opponent(lidar_opponent, now),
            timestamp=now,
        )

    def get_opponent_coordinates(self) -> Optional[Tuple[float, float]]:
        """Return (x, y) of opponent for pathfinding, or None if unavailable."""
        state = self.get_state()
        if state.opponent is None:
            return None
        return float(state.opponent.x), float(state.opponent.y)

    def get_pathfinding_payload(self, prediction_horizon_s: float = 0.0) -> Dict[str, Any]:
        """Return a serializable payload for pathfinding and strategy layers."""
        state = self.get_state()
        payload = state.as_dict()

        opp = payload.get("opponent")
        horizon = max(0.0, float(prediction_horizon_s))
        if opp is not None and horizon > 0.0:
            opp["x_pred"] = float(opp["x"] + opp["vx"] * horizon)
            opp["y_pred"] = float(opp["y"] + opp["vy"] * horizon)

        return payload

    @staticmethod
    def _convert_opponent(opp_state: OpponentState, now: float) -> Optional[OpponentTrack]:
        """Convert OpponentState from lidar_logic to OpponentTrack."""
        if opp_state is None or opp_state.confidence <= 0.0:
            return None
        
        return OpponentTrack(
            x=float(opp_state.x),
            y=float(opp_state.y),
            vx=0.0,  # Velocity tracking not yet implemented
            vy=0.0,
            radius=0.0,  # Radius not tracked in OpponentState
            confidence=float(opp_state.confidence),
            timestamp=now,
        )


__all__ = [
    "OpponentTrack",
    "PathfindingState",
    "LidarNavigationBridge",
]
