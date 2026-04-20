# Rerun Data Flow & Integration Guide

## Overview

The Rerun Bridge (`rerun_bridge.py`) serves as the **central visualization hub** for all robot telemetry. It operates in **background mode** receiving data updates from multiple sources and publishing them at **20 Hz** to a real-time 3D/2D visualization.

### Key Architecture
```
┌─────────────────────────────────────────────────────────────────┐
│                      RASPBERRY PI                              │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   ┌──────────────┐  ┌──────────────┐  ┌──────────────┐         │
│   │   Teensy     │  │   LiDAR      │  │  PathFinding │         │
│   │  (Odometry)  │  │  (LiDAR Data)│  │  (Trajectory)│         │
│   └──────┬───────┘  └──────┬───────┘  └──────┬───────┘         │
│          │                 │                 │                  │
│          │ USB Callback    │ Polling Thread  │ Direct Call      │
│          ▼                 ▼                 ▼                  │
│   ┌─────────────────────────────────────────────────┐          │
│   │   Rerun Bridge (Thread-Safe State)              │          │
│   │   ┌─────────────────────────────────────────┐   │          │
│   │   │  _State (Dataclass)                     │   │          │
│   │   │  • odom_x, odom_y, odom_theta           │   │          │
│   │   │  • lidar_cloud, lidar_beacons           │   │          │
│   │   │  • lidar_x, lidar_y, lidar_theta        │   │          │
│   │   │  • target_x, target_y                   │   │          │
│   │   │  • trajectory [(x,y), ...]              │   │          │
│   │   │  • fused_x, fused_y, fused_theta        │   │          │
│   │   │  • _lock (RLock)                        │   │          │
│   │   └─────────────────────────────────────────┘   │          │
│   │                                                   │          │
│   │   ┌─────────────────────────────────────────┐   │          │
│   │   │  publish_loop() @ 20 Hz                 │   │          │
│   │   │  • Reads _State snapshot                │   │          │
│   │   │  • Renders 3D scene                     │   │          │
│   │   │  • Logs time series data                │   │          │
│   │   │  • Publishes to Rerun SDK               │   │          │
│   │   └─────────────────────────────────────────┘   │          │
│   └─────────────────────────────────────────────────┘          │
│          │ Rerun Protocol (WebSocket/gRPC)                     │
│          ▼                                                      │
│   ┌─────────────────────────────────────────────┐              │
│   │  Rerun Viewer (Web Browser)                 │              │
│   │  • 3D Terrain Visualization                 │              │
│   │  • LiDAR Polar View                         │              │
│   │  • Time Series Plots                        │              │
│   │  • Real-time Update @ 20 Hz                 │              │
│   └─────────────────────────────────────────────┘              │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│                      PC LAPTOP                                  │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  Firefox/Chrome: http://RaspIP:9876                            │
│  Displays live visualization with full interactivity           │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## Public API (Data Update Functions)

### 1. **Odometry Updates** (from Teensy)
```python
def update_odom(x: float, y: float, theta: float):
    """Update robot odometry (Teensy position estimate).
    
    Args:
        x: Position X in mm (0 to 3000)
        y: Position Y in mm (0 to 2000)
        theta: Orientation in radians
    
    Visualization: Blue cylinder with arrow in world/robot/odom
    Updates: Time series plots (data/teensy/*)
    """
    with _st._lock:
        _st.odom_x, _st.odom_y, _st.odom_theta = x, y, theta
```

**When Called:**
- USB callback from Teensy `UPDATE_ROLLING_BASIS` messages
- Default: ~10 Hz (every 100ms from Teensy firmware)

---

### 2. **LiDAR Raw Cloud**
```python
def update_lidar_cloud(pts):
    """Update raw LiDAR scan data.
    
    Args:
        pts: List of [angle_rad, distance_mm, quality] tuples
             Example: [[0.1, 500, 15], [0.2, 510, 14], ...]
    
    Visualization: 
        - 3D point cloud in world/lidar/cloud
        - 2D polar view in sensors/lidar/polar_view
    """
    with _st._lock:
        _st.lidar_cloud = list(pts)
```

**When Called:**
- Polling function from `lidar_logic.py::get_latest_scan_data()`
- Default: ~20 Hz polling in main loop

---

### 3. **LiDAR Detected Beacons**
```python
def update_lidar_beacons(cands):
    """Update beacon detection results.
    
    Args:
        cands: List of beacon dictionaries
               Example: [{"x_r": 200, "y_r": -150, "id": 1}, ...]
               (Position in robot frame relative to odometry)
    
    Visualization: Orange diamonds in world/lidar/beacons_detected
    """
    with _st._lock:
        _st.lidar_beacons = list(cands)
```

**When Called:**
- Polling function from `lidar_logic.py::get_latest_beacon_candidates()`
- Default: ~20 Hz polling in main loop

---

### 4. **LiDAR Corrected Pose**
```python
def update_lidar_pose(x: float, y: float, theta: float, 
                      conf: float, ok: bool = True):
    """Update position calculated by LiDAR SVD trilateration.
    
    Args:
        x: Position X in mm
        y: Position Y in mm
        theta: Orientation in radians
        conf: Confidence score (0.0 to 1.0)
        ok: True if localization is valid
    
    Visualization: Red cylinder (only if ok=True) in world/lidar/pose
    Updates: Time series plots (data/lidar/*)
    """
    with _st._lock:
        _st.lidar_x, _st.lidar_y, _st.lidar_theta = x, y, theta
        _st.lidar_conf, _st.lidar_ok = conf, ok
```

**When Called:**
- Polling function from `lidar_logic.py::get_corrected_pose()`
- Default: ~20 Hz polling in main loop

---

### 5. **PathFinding Target**
```python
def update_target(x: float, y: float, theta: float = None):
    """Update navigation target point with optional direction.
    
    Args:
        x: Target X in mm
        y: Target Y in mm
        theta: Target orientation in radians (optional)
    
    Visualization: 
        - Yellow point in world/pathfinding/target
        - Yellow arrow (if theta provided) in world/pathfinding/target_arrow
    """
    with _st._lock:
        _st.target_x, _st.target_y = x, y
        if theta is not None:
            _st.target_theta = theta
```

**When Called:**
- From `pathfinder.py` when new path is planned
- Or from `strategy/` when goals are updated
- Frequency: Variable, depends on strategy updates

---

### 6. **Planned Trajectory**
```python
def update_trajectory(pts):
    """Update planned path trajectory.
    
    Args:
        pts: List of waypoints [(x1, y1), (x2, y2), ...]
             Can be empty to clear
    
    Visualization: Dotted line in world/pathfinding/trajectory
    """
    with _st._lock:
        _st.trajectory = list(pts)
```

**When Called:**
- From `pathfinder.py` after path planning
- Frequency: Variable, depends on replanning frequency

---

### 7. **Obstacles & Adversaries**
```python
def update_obstacles(obstacles):
    """Update detected obstacles (adversary robots, static obstacles).
    
    Args:
        obstacles: List of dicts
                  [{"x": 500, "y": 1500, "radius": 150},
                   {"x": 800, "y": 1200, "radius": 150}]
                   (Position and radius in mm)
    
    Visualization: Red spheres in world/obstacles/adversaries
    """
    with _st._lock:
        _st.obstacles = list(obstacles)
```

**When Called:**
- From `robot.py` when LiDAR detects adversary robots
- Or from strategy code when static obstacles are identified
- Frequency: Variable, depends on detection updates

---

### 8. **Fused Position** (Best Estimate)
```python
def update_fused(x: float, y: float, theta: float):
    """Update best position estimate (fusion of Teensy + LiDAR).
    
    Args:
        x: Fused X in mm
        y: Fused Y in mm
        theta: Fused orientation in radians
    
    Visualization: Green cylinder in world/robot/fused
    Updates: Time series plots (data/fused/*)
    
    Note: This is typically computed by robot.py's complementary filter
          combining Teensy odometry with LiDAR corrections
    """
    with _st._lock:
        _st.fused_x, _st.fused_y, _st.fused_theta = x, y, theta
```

**When Called:**
- From `robot.py::update()` after fusion computation
- Default: 20 Hz robot update loop

---

## How Data Flows (Real Example)

### **Scenario: Complete cycle at 20 Hz**

```
┌─ Main Loop (robot.py) ──────────────────────────────┐
│                                                       │
│  1. Read Teensy USB Message                          │
│     → Teensy callback triggered                      │
│     → Calls: make_odom_callback()                    │
│     → Extracts x,y,θ from 24 bytes                   │
│     → Calls: rerun_bridge.update_odom(x, y, θ)      │
│                                                       │
│  2. In parallel: LiDAR polling (from rerun_bridge)   │
│     → Calls: get_latest_scan_data()                  │
│     → Calls: rerun_bridge.update_lidar_cloud(pts)    │
│     → Calls: get_latest_beacon_candidates()         │
│     → Calls: rerun_bridge.update_lidar_beacons(cand) │
│     → Calls: get_corrected_pose()                    │
│     → Calls: rerun_bridge.update_lidar_pose(...)     │
│                                                       │
│  3. ComplementaryFilter fusion (robot.py)            │
│     → Blend Teensy position + LiDAR correction      │
│     → Calls: rerun_bridge.update_fused(rx,ry,rθ)     │
│                                                       │
│  4. PathFinding updates (strategy/)                  │
│     → If replanning triggered:                       │
│     → Calls: rerun_bridge.update_target(tx, ty)      │
│     → Calls: rerun_bridge.update_trajectory(path)    │
│                                                       │
│  5. At same time: publish_loop() @ 20 Hz             │
│     → Reads thread-safe _State snapshot              │
│     → Renders all 3D objects                         │
│     → Publishes to Rerun viewer                      │
│                                                       │
└─────────────────────────────────────────────────────┘
```

---

## Starting the Bridge

### **Mode 1: Local Viewer (Development on Rasp)**
```bash
cd /robot1/rasp/rerun
python rerun_bridge.py --mode local --sim
# → Viewer opens directly on Rasp (if X11 available)
```

### **Mode 2: WebSocket Server (PC Laptop Access) - RECOMMENDED**
```bash
# On Raspberry Pi:
cd /robot1/rasp/rerun
python rerun_bridge.py --mode serve --host 0.0.0.0 --port 9876 --with-lidar

# Output:
# ✓ Lidar polling activé
# 🌐 Mode SERVE activé
#    Serveur Rerun WebSocket sur 0.0.0.0:9876
#    Accédez depuis un navigateur : http://[RaspIP]:9876

# On PC Laptop:
# Open browser: http://192.168.1.50:9876  (replace with actual Rasp IP)
```

### **Mode 3: Connect to External Viewer (Advanced)**
```bash
# Start remote Rerun viewer on PC:
rerun --serve --port 8812

# On Raspberry Pi:
python rerun_bridge.py --mode connect --host 192.168.1.15 --port 8812
```

---

## Integration with robot.py

### **Complete Example: Adding Rerun Updates**

```python
# robot.py

import sys
from pathlib import Path

# Import Rerun API from bridge
sys.path.insert(0, str(Path(__file__).parent / "rerun"))
from rerun_bridge import (
    update_odom,
    update_target,
    update_trajectory,
    update_fused,
)

class Robot:
    def __init__(self):
        self.x = 1500.0
        self.y = 1000.0
        self.theta = 0.0
        # ... other init
    
    def update(self):
        """Called 20 times per second."""
        
        # Step 1: Read current position
        with self.lock:
            teensy_x, teensy_y, teensy_theta = self.x, self.y, self.theta
        
        # Step 2: Signal LiDAR that Teensy position updated
        from lidar.lidar_logic import update_teensy_pose
        update_teensy_pose(teensy_x, teensy_y, teensy_theta)
        
        # Step 3: Get LiDAR-corrected position
        from lidar.lidar_logic import get_corrected_pose
        corrected = get_corrected_pose()
        
        # Step 4: Compute fused position
        if corrected and corrected.is_localized:
            # Blend LiDAR correction with Teensy odometry
            alpha = 0.6  # 60% LiDAR, 40% Teensy
            rx = alpha * corrected.x + (1-alpha) * teensy_x
            ry = alpha * corrected.y + (1-alpha) * teensy_y
            rtheta = teensy_theta  # Use IMU
        else:
            # Fallback to Teensy only
            rx, ry, rtheta = teensy_x, teensy_y, teensy_theta
        
        # Step 5: Update Rerun visualization
        update_fused(rx, ry, rtheta)
        
        # Step 6: Strategy/Navigation
        obstacles = self.lidar.get_obstacles(rx, ry, rtheta)
        action = self.strategie.get_action_actuelle()
        
        # Step 7: If action requires new path, update Rerun
        if self.strategie.path_changed():
            target_x, target_y = self.strategie.get_target()
            trajectory_pts = self.strategie.get_planned_path()
            
            # These calls happen in parallel with other systems
            update_target(target_x, target_y)
            update_trajectory(trajectory_pts)
        
        # Continue with normal robot control...
        self.move_to_target()
```

---

## Integration with PathFinding

### **Example: pathfinder.py**

```python
# pathfinder.py

def plan_path(start: Tuple[float, float, float], 
              goal: Tuple[float, float], 
              obstacles: List) -> List[Tuple[float, float]]:
    """Plan a collision-free path.
    
    Returns path as list of waypoints.
    """
    
    # ... pathfinding algorithm ...
    path = compute_rrt_star(start, goal, obstacles)
    
    # Send updated path to Rerun visualization
    if path:
        from rerun_bridge import update_trajectory
        update_trajectory(path)
    
    return path


def set_navigation_goal(x: float, y: float):
    """Set a new navigation target."""
    
    # Send target to Rerun visualization
    from rerun_bridge import update_target
    update_target(x, y)
    
    # ... rest of navigation logic ...
```

---

## Thread Safety

**All `update_*()` functions are thread-safe** due to the `_lock` (RLock) in `_State`:

```python
# Thread 1: Main robot loop
update_fused(123.4, 456.7, 0.5)  # ← Acquires lock

# Thread 2: LiDAR polling thread (concurrent)
update_lidar_cloud(pts)           # ← Waits for lock, then acquires
update_lidar_beacons(cands)       # ← Already has lock

# Thread 3: Rerun publish loop
_st.snap()                        # ← Takes snapshot under lock
_publish(snapshot)                # ← Publishes without lock
```

No busy-wait; uses standard Python threading primitives.

---

## Visualization Layout

### **Default Blueprint (4 Views)**

```
┌─────────────────────────────────────┬──────────────────┐
│                                     │                  │
│   3D Terrain View                   │  2D Polar View   │
│   (world/)                          │  (sensors/lidar) │
│                                     │                  │
│   • Blue cylinder: Teensy odom      │   • LiDAR points │
│   • Red cylinder: LiDAR pose        │   • Concentric   │
│   • Green cylinder: Fused position  │     reference    │
│   • Yellow point: Navigation target │     circles      │
│   • Dotted line: Planned trajectory │                  │
│   • Orange diamonds: Detected beacons                  │
│   • Gray cylinders: Fixed beacons                      │
│   • Blue boxes: Fixed field obstacles                  │
│                                     │                  │
├─────────────────────────────────────┼──────────────────┤
│  Position Timeline (Teensy)         │  LiDAR Stats     │
│  • X_mm, Y_mm, theta_deg            │  • Confidence    │
│                                     │  • Num Points    │
│  Fused Timeline                     │  • Num Beacons   │
│  • X_mm, Y_mm                       │                  │
│                                     │  Error Timeline  │
│  Fusion Quality                     │  • Odom↔LiDAR    │
│  • Ecart_odom_lidar_mm              │    distance      │
│                                     │                  │
└─────────────────────────────────────┴──────────────────┘
```

---

## Performance Notes

### **CPU/Memory Impact**

- **Rendering cost:** ~15% CPU (on Rasp Pi 4) for full 3D scene at 20 Hz
- **Network bandwidth:** ~500 KB/s at 20 Hz publishing rate
- **WebSocket latency:** <50ms for localhost, <200ms for WiFi

### **Optimization Tips**

1. **Reduce publishing rate if needed:**
   ```python
   publish_loop(hz=10.0)  # Instead of 20 Hz
   ```

2. **Disable LiDAR polling if not needed:**
   ```bash
   # Launch without --with-lidar to skip LiDAR thread overhead
   python rerun_bridge.py --mode serve --port 9876
   ```

3. **Use simulation mode for testing:**
   ```bash
   python rerun_bridge.py --mode local --sim
   ```

---

## Debugging: Data Flow Verification

### **Check Teensy Odometry:**
```python
# In robot.py main loop:
print(f"Teensy: x={robot.x:.1f} y={robot.y:.1f} θ={robot.theta:.3f}")
# → Should match "Teensy (mm)" plot in Rerun viewer
```

### **Check LiDAR Cloud:**
```python
# In lidar polling:
pts = get_latest_scan_data()
print(f"LiDAR: {len(pts)} points")
# → Should see point cloud in 3D view + polar plot
```

### **Check Fused Position:**
```python
# In robot.py update():
print(f"Fused: x={rx:.1f} y={ry:.1f} θ={rtheta:.3f}")
# → Should match "Fused (mm)" plot in Rerun viewer
```

### **Check Network (from PC):**
```bash
# Verify WebSocket connection from PC:
curl -i http://RaspIP:9876/health
# Should return 200 OK

# Or use browser console:
# console.log("Connected to Rerun at ws://RaspIP:9876")
```

---

## Common Issues

| Problem | Solution |
|---------|----------|
| "No points in point cloud" | Check `update_lidar_cloud()` is being called with valid data |
| "Beacons not showing" | Verify `update_lidar_beacons()` is called with valid beacon dicts |
| "Red cylinder not visible" | Check `update_lidar_pose(..., ok=True)` is called |
| "WebSocket won't connect" | Check Rasp firewall: `sudo ufw allow 9876` |
| "High CPU usage" | Reduce Hz rate or disable --with-lidar |
| "Trajectory line not showing" | Verify path has ≥2 points and `update_trajectory()` is called |

---

## Next Steps

1. **Test with hardware:**
   ```bash
   python rerun_bridge.py --mode serve --host 0.0.0.0 --port 9876 --with-lidar
   ```

2. **Monitor from PC:**
   ```
   http://RaspIP:9876
   ```

3. **Add custom updates in strategy:**
   ```python
   from rerun_bridge import update_target, update_trajectory
   # Call in your strategy code when goals change
   ```

4. **Set up auto-launch on Rasp boot** (see next section)

---

## Auto-Launch on Raspberry Pi Boot

Create systemd service file:

```bash
sudo nano /etc/systemd/system/rerun-bridge.service
```

```ini
[Unit]
Description=Rerun Bridge - Eurobot 2026 Visualization
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/CDR-Nantes/robot1/rasp
Environment="PATH=/home/pi/.local/bin:/usr/local/bin:/usr/bin"
ExecStart=/usr/bin/python3 /home/pi/CDR-Nantes/robot1/rasp/rerun/rerun_bridge.py --mode serve --host 0.0.0.0 --port 9876 --with-lidar
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

Enable and start:
```bash
sudo systemctl daemon-reload
sudo systemctl enable rerun-bridge.service
sudo systemctl start rerun-bridge.service

# Check status:
sudo systemctl status rerun-bridge.service

# View logs:
sudo journalctl -u rerun-bridge.service -f
```

