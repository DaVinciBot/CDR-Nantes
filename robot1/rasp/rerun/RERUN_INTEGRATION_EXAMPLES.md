# Rerun Integration Examples

This file contains **copy-paste ready examples** for integrating Rerun into existing robot code.

---

## Example 1: Minimal Integration in robot.py

**Goal:** Display robot position in Rerun with minimal code changes.

### Before (without Rerun):
```python
# robot.py
class Robot:
    def update(self):
        # Read Teensy position
        with self.lock:
            x, y, theta = self.x, self.y, self.theta
        
        # Process...
        obstacles = self.lidar.get_obstacles(x, y, theta)
```

### After (with Rerun):
```python
# robot.py
import sys
from pathlib import Path

# Add at top of file
sys.path.insert(0, str(Path(__file__).parent / "rerun"))
try:
    from rerun_bridge import update_fused, update_target, update_trajectory
    RERUN_AVAILABLE = True
except ImportError:
    RERUN_AVAILABLE = False
    print("⚠️  Rerun not available")

class Robot:
    def update(self):
        # Read Teensy position
        with self.lock:
            x, y, theta = self.x, self.y, self.theta
        
        # Process...
        obstacles = self.lidar.get_obstacles(x, y, theta)
        
        # Send to Rerun visualization (added 2 lines)
        if RERUN_AVAILABLE:
            update_fused(x, y, theta)
```

---

## Example 2: Full Integration with LiDAR + PathFinding

**Goal:** Display position, LiDAR data, and trajectory.

### robot.py Enhancement:
```python
# robot.py

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent / "rerun"))
try:
    from rerun_bridge import (
        update_fused, 
        update_target, 
        update_trajectory,
    )
    RERUN_AVAILABLE = True
except ImportError:
    RERUN_AVAILABLE = False

from lidar.lidar_logic import (
    update_teensy_pose,        # Signal LiDAR thread
    get_corrected_pose,         # Get LiDAR-corrected position
)

class Robot:
    def __init__(self):
        self.x = 1500.0
        self.y = 1000.0
        self.theta = 0.0
        self.lock = threading.RLock()
        self._last_target_update = 0.0
        self._last_trajectory_update = 0.0
        # ... other initialization
    
    def update(self):
        """Main control loop (called 20 Hz)."""
        
        # 1. Read current Teensy odometry
        with self.lock:
            teensy_x, teensy_y, teensy_theta = self.x, self.y, self.theta
        
        # 2. Signal LiDAR with Teensy position
        update_teensy_pose(teensy_x, teensy_y, teensy_theta)
        
        # 3. Get LiDAR-corrected pose
        corrected_pose = get_corrected_pose()
        
        # 4. Compute fused position (complementary filter)
        if corrected_pose is not None and corrected_pose.is_localized:
            # Use LiDAR correction if available and confident
            confidence = corrected_pose.confidence
            
            # Adaptive blending: high confidence → more LiDAR, low → more Teensy
            alpha = 0.4 + 0.4 * min(confidence, 1.0)  # Range [0.4, 0.8]
            
            rx = alpha * corrected_pose.x + (1 - alpha) * teensy_x
            ry = alpha * corrected_pose.y + (1 - alpha) * teensy_y
        else:
            # Fallback: Teensy only
            rx, ry = teensy_x, teensy_y
        
        # Always use Teensy IMU for angle
        rtheta = teensy_theta
        
        # 5. Update Rerun with fused position
        if RERUN_AVAILABLE:
            update_fused(rx, ry, rtheta)
        
        # 6. Obstacle detection & navigation
        obstacles = self.lidar.get_obstacles(rx, ry, rtheta)
        action = self.strategie.get_action_actuelle()
        
        # 7. If action or target changed, update Rerun
        current_time = time.time()
        if self.strategie.target_changed():
            target_x, target_y = self.strategie.get_target()
            if RERUN_AVAILABLE:
                update_target(target_x, target_y)
            self._last_target_update = current_time
        
        if self.strategie.path_changed() and (current_time - self._last_trajectory_update) > 0.2:
            # Update trajectory at most every 200ms to avoid spam
            trajectory = self.strategie.get_planned_path()
            if RERUN_AVAILABLE:
                update_trajectory(trajectory)
            self._last_trajectory_update = current_time
        
        # 8. Continue with normal robot control
        self.move_to_target()
        self.lidar.update()
        self.strategie.update()
```

---

## Example 3: Strategy Integration

**Goal:** Automatically send target and trajectory updates when strategy replans.

### strategy/action_handler.py (excerpt):
```python
# strategy/action_handler.py

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "rerun"))
try:
    from rerun_bridge import update_target, update_trajectory
    RERUN_AVAILABLE = True
except ImportError:
    RERUN_AVAILABLE = False

class ActionHandler:
    def __init__(self):
        self._target = (1500, 1000)
        self._trajectory = []
        self._replanned = False
    
    def execute_action(self, action_type, params):
        """Execute a navigation action."""
        
        if action_type == "GO_TO_POINT":
            x, y = params["x"], params["y"]
            
            # Replan path
            self._trajectory = self.plan_path(x, y)
            self._target = (x, y)
            self._replanned = True
            
            # Update Rerun visualization
            if RERUN_AVAILABLE:
                update_target(x, y)
                update_trajectory(self._trajectory)
        
        elif action_type == "FOLLOW_TRAJECTORY":
            trajectory = params["points"]
            self._trajectory = trajectory
            self._target = trajectory[-1] if trajectory else self._target
            self._replanned = True
            
            # Update Rerun visualization
            if RERUN_AVAILABLE:
                update_target(*self._target)
                update_trajectory(self._trajectory)
    
    def plan_path(self, goal_x, goal_y):
        """Plan collision-free path using RRT*."""
        
        start = (self.current_x, self.current_y, self.current_theta)
        goal = (goal_x, goal_y)
        obstacles = self.get_obstacles()
        
        # RRT* planning algorithm...
        path = rrt_star_plan(start, goal, obstacles)
        
        return path
```

---

## Example 4: Pathfinder Integration

**Goal:** Send trajectory updates as soon as pathfinding completes.

### pathfinder.py (excerpt):
```python
# pathfinder.py

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "rerun"))
try:
    from rerun_bridge import update_trajectory
    RERUN_AVAILABLE = True
except ImportError:
    RERUN_AVAILABLE = False

def compute_trajectory(start, goal, obstacles):
    """Compute collision-free path.
    
    Args:
        start: (x, y, theta) current pose
        goal: (x, y) goal point
        obstacles: List of Obstacle objects
    
    Returns:
        List of (x, y) waypoints
    """
    
    # Planning algorithm...
    path = rrt_star_plan(start, goal, obstacles)
    
    if path and len(path) >= 2:
        # Smooth the path
        path = smooth_path(path)
        
        # Send to Rerun for visualization
        if RERUN_AVAILABLE:
            update_trajectory(path)
            print(f"✓ Trajectory {len(path)} pts → Rerun")
    else:
        # No path found
        if RERUN_AVAILABLE:
            update_trajectory([])  # Clear trajectory
        print("✗ No path found")
    
    return path
```

---

## Example 5: LiDAR Integration

**Goal:** Ensure LiDAR data flows to Rerun correctly.

### lidar_logic.py (excerpt) - Already implemented:
```python
# lidar/lidar_logic.py

# This is already called by rerun_bridge.py's make_lidar_poll()

def get_latest_scan_data():
    """Get raw LiDAR points from latest scan.
    
    Returns:
        List of [angle_rad, distance_mm, quality] tuples
    
    Called by: rerun_bridge.publish_loop() via make_lidar_poll()
    At rate: 20 Hz
    """
    with _scan_lock:
        return list(_latest_scan) if _latest_scan else []


def get_latest_beacon_candidates():
    """Get detected beacon candidates.
    
    Returns:
        List of {"x_r": float, "y_r": float, "id": int} dicts
    
    Called by: rerun_bridge.publish_loop() via make_lidar_poll()
    At rate: 20 Hz
    """
    with _beacon_lock:
        return list(_beacon_candidates) if _beacon_candidates else []


def get_corrected_pose():
    """Get LiDAR-corrected position from trilateration.
    
    Returns:
        PoseState(x, y, theta, confidence, is_localized, beacon_ids)
        or None if not available
    
    Called by: rerun_bridge.publish_loop() via make_lidar_poll()
             AND: robot.py::update()
    At rate: 20 Hz
    """
    with _corrected_pose_lock:
        return _corrected_pose
```

### Verification script:
```python
# test/verify_lidar_rerun.py

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from lidar.lidar_logic import (
    get_latest_scan_data,
    get_latest_beacon_candidates,
    get_corrected_pose,
)

# Quick check that LiDAR data reaches functions
for i in range(10):
    pts = get_latest_scan_data()
    beacons = get_latest_beacon_candidates()
    pose = get_corrected_pose()
    
    print(f"[{i}] Cloud: {len(pts) if pts else 0} pts | "
          f"Beacons: {len(beacons) if beacons else 0} | "
          f"Pose: {pose.confidence:.2f if pose else 'None'}")
    
    time.sleep(0.1)
```

---

## Example 6: Testing Without Hardware

**Goal:** Test Rerun integration using simulation.

### test/test_rerun_integration.py:
```python
#!/usr/bin/env python3
"""
Test Rerun integration using simulation mode.
Verifies all update functions are working.
"""

import sys
import time
import threading
import math
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))
sys.path.insert(0, str(Path(__file__).parent.parent / "rerun"))

from rerun_bridge import (
    update_odom,
    update_lidar_cloud,
    update_lidar_beacons,
    update_lidar_pose,
    update_target,
    update_trajectory,
    update_obstacles,
    update_fused,
)

def simulate_robot_motion():
    """Simulate robot moving in a circle."""
    
    for t in range(200):  # 10 seconds at 20 Hz
        # Circular motion
        angle = 2 * math.pi * t / 100
        x = 1500 + 500 * math.cos(angle)
        y = 1000 + 500 * math.sin(angle)
        theta = angle
        
        # Update odometry
        update_odom(x, y, theta)
        
        # Simulate LiDAR cloud (circle around robot)
        cloud = []
        for i in range(36):
            scan_angle = 2 * math.pi * i / 36
            distance = 200 + 50 * math.sin(5 * scan_angle)
            quality = 14 + int(5 * (0.5 + 0.5 * math.cos(scan_angle)))
            cloud.append([scan_angle, distance, quality])
        update_lidar_cloud(cloud)
        
        # Simulate beacon detections
        beacons = []
        for beacon_id in [1, 2, 3]:
            # Beacon relative position
            br_x = 300 * math.cos(beacon_id * 2)
            br_y = 300 * math.sin(beacon_id * 2)
            beacons.append({"x_r": br_x, "y_r": br_y, "id": beacon_id})
        update_lidar_beacons(beacons)
        
        # Simulate LiDAR pose (with some noise)
        x_lidar = x + 20 * math.sin(t * 0.05)
        y_lidar = y + 20 * math.cos(t * 0.05)
        confidence = 0.7 + 0.2 * math.sin(t * 0.01)
        update_lidar_pose(x_lidar, y_lidar, theta, confidence, ok=True)
        
        # Simulate navigation target
        target_angle = 2 * math.pi * (t / 100 + 0.5)
        tx = 1500 + 600 * math.cos(target_angle)
        ty = 1000 + 600 * math.sin(target_angle)
        update_target(tx, ty)
        
        # Simulate trajectory (path ahead)
        trajectory = []
        for dt in range(1, 6):
            future_angle = 2 * math.pi * ((t + dt*3) / 100)
            future_x = 1500 + 500 * math.cos(future_angle)
            future_y = 1000 + 500 * math.sin(future_angle)
            trajectory.append((future_x, future_y))
        update_trajectory(trajectory)
        
        # Fused position (90% LiDAR, 10% odometry)
        rx = 0.9 * x_lidar + 0.1 * x
        ry = 0.9 * y_lidar + 0.1 * y
        update_fused(rx, ry, theta)
        
        # Simulate obstacles (red spheres)
        obstacles = []
        for i in range(2):
            obs_angle = 2 * math.pi * (i / 2 + t / 200)
            obs_x = 1500 + 700 * math.cos(obs_angle)
            obs_y = 1000 + 700 * math.sin(obs_angle)
            obstacles.append({"x": obs_x, "y": obs_y, "radius": 150})
        update_obstacles(obstacles)
        
        time.sleep(1.0 / 20)  # 20 Hz
        
        if t % 20 == 0:
            print(f"[{t}] x={x:.0f} y={y:.0f} θ={math.degrees(theta):.1f}° "
                  f"| cloud={len(cloud)} | beacons={len(beacons)} | "
                  f"lidar_conf={confidence:.2f}")


if __name__ == "__main__":
    print("=" * 70)
    print("Rerun Integration Test - Simulation Mode")
    print("=" * 70)
    print("\nStart this script, then open browser:")
    print("  http://localhost:9876")
    print("\nYou should see:")
    print("  ✓ Blue cylinder moving in a circle (odometry)")
    print("  ✓ Red cylinder following (LiDAR pose)")
    print("  ✓ Green cylinder slightly offset (fused)")
    print("  ✓ Yellow point and dotted line (target + trajectory)")    print("  ✓ Red spheres rotating around (adversary robots/obstacles)")    print("  ✓ LiDAR polar view with rays")
    print("  ✓ Orange diamonds (beacon detections)")
    print("\nRunning simulation for 10 seconds...")
    print("=" * 70 + "\n")
    
    simulate_robot_motion()
    
    print("\n" + "=" * 70)
    print("✓ Test completed successfully!")
    print("=" * 70)
```

### Run the test:
```bash
# Terminal 1: Start Rerun bridge
cd /robot1/rasp/rerun
python rerun_bridge.py --mode serve --port 9876

# Terminal 2: Run test (on same machine or from elsewhere)
cd /robot1/rasp
python test/test_rerun_integration.py

# Terminal 3 (optional): View in browser
# Open: http://localhost:9876
```

---

## Example 7: Conditional Rerun Updates

**Goal:** Only send updates when data changes significantly (reduce network traffic).

### robot.py snippet:
```python
class Robot:
    def __init__(self):
        # ... other init
        self._last_fused_update = (0.0, 0.0, 0.0)  # x, y, theta
        self._last_target_update = (0.0, 0.0)
        self._fused_update_threshold_mm = 10.0  # Only update if moved >10mm
        self._target_update_threshold_mm = 50.0
        self._last_update_time = 0.0
    
    def update(self):
        """Main control loop."""
        
        # ... compute rx, ry, rtheta (fused position)
        
        # Only send if moved significantly OR haven't sent in 1 second
        current_time = time.time()
        dist_moved = math.hypot(
            rx - self._last_fused_update[0],
            ry - self._last_fused_update[1]
        )
        angle_changed = abs(self._normalize_angle(rtheta - self._last_fused_update[2]))
        
        if (dist_moved > self._fused_update_threshold_mm or 
            angle_changed > 0.1 or 
            (current_time - self._last_update_time) > 1.0):
            
            if RERUN_AVAILABLE:
                update_fused(rx, ry, rtheta)
            
            self._last_fused_update = (rx, ry, rtheta)
            self._last_update_time = current_time
        
        # Similar for target updates...
```

---

## Example 8: Error Handling

**Goal:** Gracefully handle Rerun failures without crashing robot.

### robot.py snippet:
```python
def safe_rerun_update():
    """Wrapper for Rerun updates with error handling."""
    
    if not RERUN_AVAILABLE:
        return
    
    try:
        update_fused(rx, ry, rtheta)
    except Exception as e:
        logger.warning(f"Rerun update failed: {e}")
        # Continue robot operation normally
        # Don't crash if visualization is unavailable
    
    try:
        if self.strategie.path_changed():
            update_target(*self.strategie.get_target())
            update_trajectory(self.strategie.get_planned_path())
    except Exception as e:
        logger.warning(f"Rerun trajectory update failed: {e}")
        # Continue normally
```

---

## Example 9: Performance Monitoring

**Goal:** Monitor Rerun update performance to identify bottlenecks.

### debug/monitor_rerun.py:
```python
#!/usr/bin/env python3
"""
Monitor Rerun update performance.
"""

import sys
import time
from pathlib import Path
from collections import deque
import statistics

sys.path.insert(0, str(Path(__file__).parent.parent))
sys.path.insert(0, str(Path(__file__).parent.parent / "rerun"))

from rerun_bridge import update_fused, update_target, update_trajectory

# Track update times
fused_times = deque(maxlen=100)
target_times = deque(maxlen=100)

def profile_rerun_updates():
    """Monitor Rerun update performance."""
    
    for i in range(1000):
        # Profile fused position update
        t0 = time.perf_counter()
        update_fused(1500.0 + i, 1000.0, 0.5)
        t1 = time.perf_counter()
        fused_times.append((t1 - t0) * 1000)  # Convert to ms
        
        # Profile target update (every 10th iteration)
        if i % 10 == 0:
            t0 = time.perf_counter()
            update_target(1700.0, 1000.0)
            t1 = time.perf_counter()
            target_times.append((t1 - t0) * 1000)
        
        # Profile trajectory update (every 20th iteration)
        if i % 20 == 0:
            t0 = time.perf_counter()
            trajectory = [(1500+j, 1000) for j in range(10)]
            update_trajectory(trajectory)
            t1 = time.perf_counter()
        
        if i % 100 == 0:
            avg_fused = statistics.mean(fused_times) if fused_times else 0
            max_fused = max(fused_times) if fused_times else 0
            avg_target = statistics.mean(target_times) if target_times else 0
            
            print(f"[{i}] Fused: {avg_fused:.2f}ms avg, {max_fused:.2f}ms max | "
                  f"Target: {avg_target:.2f}ms avg")
        
        time.sleep(0.01)

if __name__ == "__main__":
    print("Monitoring Rerun update performance for 10 seconds...")
    profile_rerun_updates()
    print("✓ Complete")
```

---

## Example 10: Obstacle Detection Integration

**Goal:** Display adversary robots and detected obstacles.

### robot.py snippet - LiDAR obstacle detection:

```python
# robot.py

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent / "rerun"))
try:
    from rerun_bridge import update_obstacles, update_fused
    RERUN_AVAILABLE = True
except ImportError:
    RERUN_AVAILABLE = False

class Robot:
    def update(self):
        """Main control loop."""
        
        # ... existing code ...
        
        # Detect obstacles from LiDAR
        obstacles = self.lidar.detect_obstacles(rx, ry, rtheta)
        
        # Convert to Rerun format
        obs_list = []
        for obs in obstacles:
            obs_list.append({
                "x": obs.x,           # mm
                "y": obs.y,           # mm
                "radius": obs.radius  # mm
            })
        
        # Send to visualization
        if RERUN_AVAILABLE:
            update_obstacles(obs_list)  # Red spheres in viewer
            update_fused(rx, ry, rtheta)
```

### lidar/obstacle_detector.py - Simple obstacle detection:

```python
# lidar/obstacle_detector.py

import math
from dataclasses import dataclass

@dataclass
class Obstacle:
    x: float      # mm
    y: float      # mm
    radius: float # mm (typical: 150mm for robot)

def detect_obstacles_from_cloud(lidar_cloud, robot_x, robot_y, robot_theta):
    """Detect obstacles from raw LiDAR cloud using clustering.
    
    Args:
        lidar_cloud: List of [angle_rad, distance_mm, quality]
        robot_x, robot_y, robot_theta: Current robot pose
    
    Returns:
        List of Obstacle objects
    """
    
    if not lidar_cloud:
        return []
    
    # Convert to world coordinates
    points_world = []
    for angle_rad, distance_mm, quality in lidar_cloud:
        if distance_mm < 3000:  # Filter far points
            # Transform from robot frame to world frame
            x_robot = distance_mm * math.sin(angle_rad)
            y_robot = distance_mm * math.cos(angle_rad)
            
            cos_t = math.cos(robot_theta)
            sin_t = math.sin(robot_theta)
            
            x_world = robot_x + cos_t * x_robot - sin_t * y_robot
            y_world = robot_y + sin_t * x_robot + cos_t * y_robot
            
            points_world.append((x_world, y_world))
    
    if not points_world:
        return []
    
    # Simple clustering (group close points)
    clusters = []
    used = set()
    
    for i, (x, y) in enumerate(points_world):
        if i in used:
            continue
        
        # Start new cluster
        cluster = [(x, y)]
        used.add(i)
        
        # Find nearby points
        for j in range(i + 1, len(points_world)):
            if j not in used:
                x2, y2 = points_world[j]
                dist = math.hypot(x - x2, y - y2)
                if dist < 200:  # Within 200mm
                    cluster.append((x2, y2))
                    used.add(j)
        
        clusters.append(cluster)
    
    # Convert clusters to obstacles
    obstacles = []
    for cluster in clusters:
        if len(cluster) >= 3:  # Min 3 points
            # Compute centroid
            cx = sum(p[0] for p in cluster) / len(cluster)
            cy = sum(p[1] for p in cluster) / len(cluster)
            
            # Compute radius (max distance from centroid)
            radius = max(
                math.hypot(cx - p[0], cy - p[1]) 
                for p in cluster
            )
            
            obstacles.append(Obstacle(
                x=cx,
                y=cy,
                radius=max(100, radius + 50)  # Min 100mm, add margin
            ))
    
    return obstacles
```

### Usage in robot.py:

```python
from lidar.obstacle_detector import detect_obstacles_from_cloud

class Robot:
    def update(self):
        """Main control loop (20 Hz)."""
        
        # ... get lidar_cloud ...
        
        # Detect obstacles
        obstacles = detect_obstacles_from_cloud(
            lidar_cloud, 
            rx, ry, rtheta
        )
        
        # Send to Rerun
        if RERUN_AVAILABLE:
            obs_dict = [
                {"x": obs.x, "y": obs.y, "radius": obs.radius}
                for obs in obstacles
            ]
            update_obstacles(obs_dict)
```

**Visualization Result:**
- Red spheres appear in 3D view where obstacles detected
- Sphere size = obstacle radius
- Sphere color intensity shows confidence
- Helps debug obstacle avoidance

---

## Checklist: Adding Rerun to Your Code

- [ ] Import Rerun functions at top of file
- [ ] Add `RERUN_AVAILABLE` try-except block
- [ ] In main update loop, call `update_fused(x, y, theta)`
- [ ] In strategy code, call `update_target(x, y, theta)` when goal changes (theta optional)
- [ ] In pathfinding code, call `update_trajectory()` when path updates
- [ ] In obstacle detection code, call `update_obstacles()` with detected obstacles
- [ ] Test with `--mode serve --port 9876`
- [ ] Verify visualization in browser: `http://localhost:9876`
- [ ] Check "robot moves in viewer" as you move physically
- [ ] Check red spheres appear when obstacles detected
- [ ] Verify yellow arrow points to target goal
- [ ] Verify times series plots update smoothly
- [ ] Set up auto-launch systemd service for deployment

