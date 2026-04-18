"""
lidar.py
Wrapper simplifié pour encapsuler la localisation et tracking adversaire.

Fournit une interface unique pour :
- Fusion pose lidar (trilatération beacons) + odométrie Teensy
- Tracking robot adverse
- Exporte pour pathfinding
"""

import logging
import threading
import time
from typing import Optional, Tuple

try:
    from .lidar_navigation_bridge import LidarNavigationBridge, OpponentTrack
    from .lidar_logic import (
        get_latest_pose, 
        get_latest_opponent,
        get_latest_scan_data,
        get_latest_beacon_candidates,
    )
except ImportError:
    from lidar_navigation_bridge import LidarNavigationBridge, OpponentTrack
    from lidar_logic import (
        get_latest_pose, 
        get_latest_opponent,
        get_latest_scan_data,
        get_latest_beacon_candidates,
    )


class LidarInterface:
    """
    Interface simplifiée pour accéder à la localisation fusionnée et tracking adversaire.
    
    Usage:
        lidar = LidarInterface(team_color="BLUE")
        
        # Lors de chaque update robot
        pose_x, pose_y, pose_theta, confidence = lidar.get_fused_position(imu_theta)
        opp_x, opp_y, opp_conf = lidar.get_opponent()
    """
    
    def __init__(self, team_color: str = "BLUE", opponent_timeout_s: float = 0.70):
        """
        Initialize the lidar bridge.
        
        Args:
            team_color: "BLUE" or "YELLOW" (used for beacon symmetry)
            opponent_timeout_s: Timeout for opponent track validity
        """
        self.logger = logging.getLogger("LIDAR_INTERFACE")
        self.team_color = team_color.upper()
        self._bridge = LidarNavigationBridge(
            team_color=self.team_color,
            opponent_timeout_s=opponent_timeout_s
        )
        self._lock = threading.Lock()
        self.logger.info(f"LidarInterface initialized for team {self.team_color}")
    
    def get_fused_position(self, teensy_x: float, teensy_y: float, 
                          teensy_theta: float) -> Tuple[float, float, float, float]:
        """
        Get fused robot position: blend lidar (x,y) with teensy odometry.
        
        Performs simple alpha-blending:
        - x_fused = alpha * lidar_x + (1-alpha) * teensy_x
        - y_fused = alpha * lidar_y + (1-alpha) * teensy_y
        - theta_fused = teensy_theta (always from Teensy IMU)
        
        Args:
            teensy_x: X position (mm) from Teensy odometry
            teensy_y: Y position (mm) from Teensy odometry
            teensy_theta: Robot orientation (rad) from Teensy IMU
        
        Returns:
            (x_mm, y_mm, theta_rad, confidence)
            confidence: Blended based on lidar beacon visibility (0.0 = no beacons)
        """
        alpha_xy = 0.35  # Weight for lidar X,Y (higher = trust lidar more)
        
        with self._lock:
            # Get lidar pose estimate
            lidar_pose = get_latest_pose()
            
            # If no lidar localization, use teensy odometry as-is
            if not lidar_pose or not lidar_pose.is_localized:
                return teensy_x, teensy_y, teensy_theta, 0.0
            
            # Blend X, Y from lidar and teensy
            fused_x = alpha_xy * lidar_pose.x + (1.0 - alpha_xy) * teensy_x
            fused_y = alpha_xy * lidar_pose.y + (1.0 - alpha_xy) * teensy_y
            
            # Theta always from Teensy IMU
            fused_theta = teensy_theta
            
            # Confidence from lidar (how many beacons used)
            confidence = lidar_pose.confidence
            
            return (fused_x, fused_y, fused_theta, confidence)
    
    def get_opponent(self) -> Optional[Tuple[float, float, float]]:
        """
        Get opponent (robot adverse) position if available.
        
        Returns:
            (x_mm, y_mm, confidence) or None if no opponent detected
        """
        with self._lock:
            opponent = get_latest_opponent()
            
            if not opponent or opponent.confidence < 0.1:
                return None
            
            return (opponent.x, opponent.y, opponent.confidence)
    
    def get_obstacles(self, robot_x: float, robot_y: float, robot_theta: float) -> list:
        """
        Get list of obstacle points in absolute world coordinates.
        
        Converts relative lidar scan points to absolute coordinates.
        Points are filtered to exclude beacons and keep valid obstacles only.
        
        Args:
            robot_x: Robot X position (mm)
            robot_y: Robot Y position (mm)
            robot_theta: Robot orientation (rad)
        
        Returns:
            List of (x, y) tuples in mm absolute coordinates
        """
        import math
        
        with self._lock:
            try:
                scan_data = get_latest_scan_data()
            except Exception:
                return []
            
            if not scan_data:
                return []
            
            obstacles = []
            cos_theta = math.cos(robot_theta)
            sin_theta = math.sin(robot_theta)
            
            # Convert points from relative to absolute coordinates
            for point in scan_data:
                try:
                    x_rel = float(point.get('x', 0.0) if isinstance(point, dict) else getattr(point, 'x', 0.0))
                    y_rel = float(point.get('y', 0.0) if isinstance(point, dict) else getattr(point, 'y', 0.0))
                    
                    # Rotate from lidar frame to robot frame, then translate
                    x_abs = robot_x + (x_rel * cos_theta - y_rel * sin_theta)
                    y_abs = robot_y + (x_rel * sin_theta + y_rel * cos_theta)
                    
                    obstacles.append((x_abs, y_abs))
                except (ValueError, TypeError, AttributeError):
                    continue
            
            # Add opponent if detected
            opp = self.get_opponent()
            if opp:
                opp_x, opp_y, _ = opp
                obstacles.append((opp_x, opp_y))
            
            return obstacles
    
    def get_diagnostic_info(self) -> dict:
        """
        Get debug/diagnostic info about localization state.
        
        Returns dict with:
            - is_localized: bool
            - nb_beacons: int
            - confidence: float
            - beacon_ids: list
            - last_update_time: float
        """
        with self._lock:
            pose = get_latest_pose()
            
            if not pose:
                return {
                    "is_localized": False,
                    "nb_beacons": 0,
                    "confidence": 0.0,
                    "beacon_ids": [],
                    "last_update_time": 0.0
                }
            
            return {
                "is_localized": pose.is_localized,
                "nb_beacons": len(pose.beacon_ids or []),
                "confidence": pose.confidence,
                "beacon_ids": pose.beacon_ids or [],
                "last_update_time": pose.last_update_time
            }
