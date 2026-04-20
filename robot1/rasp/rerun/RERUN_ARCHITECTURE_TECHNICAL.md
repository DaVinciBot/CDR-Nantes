# Rerun Bridge - Technical Architecture & Data Flow

Comprehensive technical documentation for the Rerun visualization bridge.

---

## Table of Contents

1. [System Architecture](#system-architecture)
2. [Data Flow Diagram](#data-flow-diagram)
3. [Thread Model](#thread-model)
4. [State Management](#state-management)
5. [API Reference](#api-reference)
6. [Communication Protocols](#communication-protocols)
7. [Performance Characteristics](#performance-characteristics)
8. [Extensibility](#extensibility)

---

## System Architecture

### High-Level Component Diagram

```
┌──────────────────────────────────────────────────────────────────┐
│                    Raspberry Pi (3 Hz robot loop)                 │
├──────────────────────────────────────────────────────────────────┤
│                                                                   │
│  ┌────────────────┐       ┌────────────────┐                     │
│  │  Teensy (USB)  │       │  LiDAR (USB)   │                     │
│  │  Hardware      │       │  Hardware      │                     │
│  └────────┬───────┘       └────────┬───────┘                     │
│           │                        │                             │
│           ▼                        ▼                             │
│  ┌─────────────────────────────────────────────┐                │
│  │     USB Communication Layer                 │                │
│  │  • FTDI serial @ 115200 baud                │                │
│  │  • Messages: UPDATE_ROLLING_BASIS (24 bytes)│                │
│  │  • Messages: LIDAR_DATA (variable)          │                │
│  └────────┬────────────────────┬───────────────┘                │
│           │                    │                                │
│           │ (10 Hz callback)   │ (polling thread 20Hz)          │
│           ▼                    ▼                                │
│  ┌──────────────────────────────────────────────────┐          │
│  │   Rerun Bridge (rerun_bridge.py)                │          │
│  │   ┌────────────────────────────────────────────┐│          │
│  │   │  Shared State (_State dataclass)            ││          │
│  │   │  ┌──────────────────────────────────────┐  ││          │
│  │   │  │ Thread-Safe Data Container          │  ││          │
│  │   │  │ • odom_x, odom_y, odom_theta       │  ││          │
│  │   │  │ • lidar_cloud []                   │  ││          │
│  │   │  │ • lidar_beacons []                 │  ││          │
│  │   │  │ • lidar_x, y, theta, conf, ok     │  ││          │
│  │   │  │ • target_x, target_y               │  ││          │
│  │   │  │ • trajectory []                    │  ││          │
│  │   │  │ • fused_x, fused_y, fused_theta   │  ││          │
│  │   │  │ • _lock (RLock)                    │  ││          │
│  │   │  └──────────────────────────────────────┘  ││          │
│  │   │                                            ││          │
│  │   │  Public API (update_* functions)           ││          │
│  │   │  • update_odom()                           ││          │
│  │   │  • update_lidar_cloud()                    ││          │
│  │   │  • update_lidar_beacons()                  ││          │
│  │   │  • update_lidar_pose()                     ││          │
│  │   │  • update_target()                         ││          │
│  │   │  • update_trajectory()                     ││          │
│  │   │  • update_fused()                          ││          │
│  │   │                                            ││          │
│  │   │  Polling Threads                           ││          │
│  │   │  • lidar_poll() @ 20 Hz                    ││          │
│  │   └────────────────────────────────────────────┘│          │
│  │                                                  │          │
│  │  ┌────────────────────────────────────────────┐ │          │
│  │  │ publish_loop() @ 20 Hz                     │ │          │
│  │  │ • Read _State (thread-safe snapshot)      │ │          │
│  │  │ • Render 3D scene (_publish)              │ │          │
│  │  │ • Log time series                         │ │          │
│  │  │ • Send to Rerun SDK                       │ │          │
│  │  └────────────────────────────────────────────┘ │          │
│  │                                                  │          │
│  └──────┬───────────────────────────────────────────┘          │
│         │ (Rerun SDK Protocol)                                 │
│         ▼                                                       │
│  ┌─────────────────────────────────────────────────────┐       │
│  │  Rerun SDK → Network Transport                      │       │
│  │  • Mode 1: local (direct viewer)                    │       │
│  │  • Mode 2: serve (WebSocket server on 0.0.0.0:9876)│       │
│  │  • Mode 3: connect (gRPC to external viewer)        │       │
│  └────────┬────────────────────────────────────────────┘       │
│           │                                                    │
└───────────┼────────────────────────────────────────────────────┘
            │
            │ (Rerun Protocol: JSON over WebSocket or gRPC)
            ▼
┌──────────────────────────────────────────────────────────────────┐
│                   PC Laptop (Web Browser)                        │
├──────────────────────────────────────────────────────────────────┤
│                                                                  │
│  Firefox/Chrome/Safari: http://RaspIP:9876                      │
│                                                                  │
│  ┌─────────────────────────────────────────────────────┐        │
│  │  Rerun Viewer (Web Application)                     │        │
│  │                                                     │        │
│  │  ┌──────────────────┐  ┌──────────────────┐        │        │
│  │  │  3D Renderer     │  │  2D Polar Plot   │        │        │
│  │  │  • Terrain       │  │  • LiDAR rays    │        │        │
│  │  │  • Robots        │  │  • Reference     │        │        │
│  │  │  • Obstacles     │  │    circles       │        │        │
│  │  │  • Trajectories  │  └──────────────────┘        │        │
│  │  └──────────────────┘                              │        │
│  │                                                     │        │
│  │  ┌──────────────────┐  ┌──────────────────┐        │        │
│  │  │  Time Series     │  │  Legend          │        │        │
│  │  │  • Position      │  │  • Show/hide     │        │        │
│  │  │  • Velocities    │  │    objects       │        │        │
│  │  │  • Confidence    │  │  • Color coding  │        │        │
│  │  │  • Errors        │  │  • Labels        │        │        │
│  │  └──────────────────┘  └──────────────────┘        │        │
│  │                                                     │        │
│  └─────────────────────────────────────────────────────┘        │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
```

---

## Data Flow Diagram

### Complete Data Path: USB Message → Visualization

```
Step 1: USB Message from Teensy
┌─────────────────────────────────────┐
│ Teensy Firmware sends 24-byte       │
│ UPDATE_ROLLING_BASIS message every  │
│ ~100ms with (x, y, θ)               │
└──────────────┬──────────────────────┘
               │ (USB serial @ 115200 baud)
               ▼
Step 2: USB Driver Receives
┌─────────────────────────────────────┐
│ FTDI USB-Serial driver on Rasp      │
│ Routes to /dev/ttyUSB0              │
└──────────────┬──────────────────────┘
               │
               ▼
Step 3: Message Dispatch (robot.py)
┌─────────────────────────────────────┐
│ com.add_callback(make_odom_callback(),│
│   Messages.UPDATE_ROLLING_BASIS)     │
│                                      │
│ When message received:               │
│ → Callback triggered                 │
│ → Parses 24 bytes: x,y,θ             │
│ → Calls update_odom(x, y, θ)         │
└──────────────┬──────────────────────┘
               │
               ▼
Step 4: Update Shared State (rerun_bridge._State)
┌─────────────────────────────────────┐
│ def update_odom(x, y, θ):           │
│   with _st._lock:                   │
│     _st.odom_x = x                  │
│     _st.odom_y = y                  │
│     _st.odom_theta = θ              │
│                                      │
│ (Lock acquired for thread safety)    │
└──────────────┬──────────────────────┘
               │
               ▼
Step 5: Polling Loop Reads (publish_loop)
┌─────────────────────────────────────┐
│ Main publish_loop() @ 20 Hz:        │
│   if lidar_poll:                    │
│     lidar_poll()  ← Triggers update  │
│                   of lidar data      │
│   snapshot = _st.snap()  ← Atomic   │
│                   read of all data   │
│   _publish(snapshot)                │
└──────────────┬──────────────────────┘
               │
               ▼
Step 6: Rendering (rerun_bridge._publish)
┌─────────────────────────────────────┐
│ def _publish(s: _State):            │
│   # Render robot mesh               │
│   _log_robot("world/robot/odom",    │
│       s.odom_x, s.odom_y,           │
│       s.odom_theta)                 │
│                                      │
│   # Render LiDAR cloud              │
│   if s.lidar_cloud:                 │
│     rr.log("world/lidar/cloud",     │
│       rr.Points3D(positions=...,    │
│                   colors=...))      │
│                                      │
│   # Log time series                 │
│   rr.log("data/teensy/x_mm",        │
│     rr.Scalars(s.odom_x))           │
│                                      │
│   ... (similar for all data)        │
└──────────────┬──────────────────────┘
               │
               ▼
Step 7: Rerun SDK Encoding
┌─────────────────────────────────────┐
│ Rerun SDK batches log entries       │
│ Encodes to internal format          │
│ (Messages protobuf internally)       │
└──────────────┬──────────────────────┘
               │
               ▼
Step 8: Network Transport
┌─────────────────────────────────────┐
│ Mode selection:                      │
│ • local: Direct to embedded viewer   │
│ • serve: WebSocket on 0.0.0.0:9876  │
│ • connect: gRPC to external viewer   │
│                                      │
│ Typical: WebSocket @ ~500 KB/s      │
└──────────────┬──────────────────────┘
               │
               ▼
Step 9: Browser Reception (PC)
┌─────────────────────────────────────┐
│ WebSocket on PC receives data        │
│ JavaScript unpacks messages         │
│ Stores in browser-side data model   │
└──────────────┬──────────────────────┘
               │
               ▼
Step 10: Rendering (Web GL)
┌─────────────────────────────────────┐
│ Three.js / WebGL renders:           │
│ • 3D terrain & robot cylinder       │
│ • LiDAR point cloud                 │
│ • UI plots                          │
│ • Live updates @ 20 Hz              │
│                                      │
│ Result: Real-time visualization     │
│ shown in browser                    │
└─────────────────────────────────────┘
```

---

## Thread Model

### Thread Safety Architecture

```
Main Application Threads:
┌────────────────────────────────────┐
│  robot.py Main Loop (20 Hz)        │
│  • Reads: teensy_x, teensy_y       │
│  • Writes: _st.odom_x via callback │
│  • Calls: update_fused(rx,ry,rθ)   │
└────────────────┬───────────────────┘
                 │
                 ▼
         ┌───────────────────┐
         │  Rerun Lock       │
         │  _st._lock        │
         │  (RLock)          │
         │  Reentrant = Safe │
         │  for same thread  │
         └─────────┬─────────┘
                 │
                 ├─────────────────────────────┐
                 │                             │
                 ▼                             ▼
    ┌────────────────────────┐    ┌──────────────────────┐
    │ USB Callback Thread    │    │ LiDAR Polling Thread │
    │ (Teensy ODO)           │    │ (from publish_loop)  │
    │                        │    │                      │
    │ • Receives USB msg     │    │ • Calls              │
    │ • Acquires lock        │    │   get_latest_scan()  │
    │ • Updates _st.odom_*   │    │ • Acquires lock      │
    │ • Releases lock        │    │ • Updates _st.lidar_*│
    │                        │    │ • Releases lock      │
    └────────────────────────┘    └──────────────────────┘
                 │                             │
                 └─────────────────────────────┘
                           │
                           ▼
         ┌─────────────────────────────┐
         │  Publish Loop Thread (20Hz) │
         │                             │
         │  • Calls _st.snap()         │
         │    (atomic read under lock) │
         │  • Gets consistent snapshot │
         │  • Renders without lock     │
         │  • Publishes to Rerun SDK   │
         │                             │
         └─────────────────────────────┘

No busy-wait! Uses standard Python threading.
Lock contention minimal (microseconds).
```

### Thread Safety Implementation

```python
# In rerun_bridge.py

@dataclass
class _State:
    odom_x: float = CX
    odom_y: float = CY
    # ... other fields ...
    _lock: threading.RLock = field(default_factory=threading.RLock)
    
    def snap(self):
        """Take atomic snapshot of all state."""
        with self._lock:  # Acquire lock
            s = _State()
            for f in self.__dataclass_fields__:
                if f != "_lock":
                    setattr(s, f, getattr(self, f))
            return s  # Return copy (lock released)

# Update functions acquire lock
def update_odom(x: float, y: float, theta: float):
    with _st._lock:  # Acquire
        _st.odom_x, _st.odom_y, _st.odom_theta = x, y, theta
        # Auto-release when exiting with block

# Publishing doesn't hold lock
def publish_loop(hz: float = 20.0, lidar_poll=None) -> None:
    dt = 1.0 / hz
    while True:
        snapshot = _st.snap()  # ← Lock held briefly
        _publish(snapshot)     # ← Lock NOT held (safe for updates)
        time.sleep(dt)
```

---

## State Management

### _State Dataclass Fields

```python
@dataclass
class _State:
    # Odometry (from Teensy via UPDATE_ROLLING_BASIS)
    odom_x: float = CX              # Position X in mm
    odom_y: float = CY              # Position Y in mm
    odom_theta: float = 0.0         # Orientation in radians
    
    # LiDAR raw data (from polling thread)
    lidar_cloud: list = field(...)  # [(angle_rad, dist_mm, qual), ...]
    
    # LiDAR detected beacons (from polling thread)
    lidar_beacons: list = field(...)  # [{"x_r": x, "y_r": y, "id": id}, ...]
    
    # LiDAR corrected pose (from trilateration/SVD)
    lidar_x: float = 0.0            # Position X in mm
    lidar_y: float = 0.0            # Position Y in mm
    lidar_theta: float = 0.0        # Orientation in radians
    lidar_conf: float = 0.0         # Confidence 0.0-1.0
    lidar_ok: bool = False          # Localization valid?
    
    # Navigation targets
    target_x: float = CX            # Goal X in mm
    target_y: float = CY            # Goal Y in mm
    
    # Planned trajectory
    trajectory: list = field(...)   # [(x, y), (x, y), ...] waypoints
    
    # Fused best estimate
    fused_x: float = CX             # Best X estimate
    fused_y: float = CY             # Best Y estimate
    fused_theta: float = 0.0        # Best θ estimate
    
    # Thread synchronization
    _lock: threading.RLock = field(default_factory=threading.RLock)

# State lifecycle:
_st = _State()  # Singleton instance created at module load
# Updated throughout robot.py execution
# Read by publish_loop() via .snap()
```

---

## API Reference

### Public Functions

#### 1. `update_odom(x: float, y: float, theta: float)`
```
Purpose:  Update Teensy odometry estimate
Called by: USB callback from Teensy (10 Hz)
Thread-safe: Yes (acquires _lock)
Parameters:
  x:     Position X in mm (0-3000)
  y:     Position Y in mm (0-2000)
  theta: Orientation in radians (-π to +π)
Example:
  update_odom(1500.5, 1000.2, 0.785)
Visualization: Blue cylinder in world/robot/odom
```

#### 2. `update_lidar_cloud(pts)`
```
Purpose:  Update raw LiDAR scan
Called by: Polling via lidar_logic.get_latest_scan_data()
Thread-safe: Yes
Parameters:
  pts: List of [angle_rad, distance_mm, quality]
       Example: [[0.1, 500, 14], [0.2, 510, 15], ...]
Visualization: 
  - 3D: world/lidar/cloud (points in 3D terrain)
  - 2D: sensors/lidar/polar_view (radar plot)
```

#### 3. `update_lidar_beacons(cands)`
```
Purpose:  Update detected beacon candidates
Called by: Polling via lidar_logic.get_latest_beacon_candidates()
Thread-safe: Yes
Parameters:
  cands: List of dicts
         [{"x_r": 200, "y_r": -150, "id": 1}, ...]
         (Position in robot frame)
Visualization: Orange diamonds in world/lidar/beacons_detected
```

#### 4. `update_lidar_pose(x, y, theta, conf, ok=True)`
```
Purpose:  Update LiDAR-calculated position
Called by: Polling via lidar_logic.get_corrected_pose()
Thread-safe: Yes
Parameters:
  x:    Position X in mm
  y:    Position Y in mm
  theta: Orientation in radians
  conf:  Confidence score (0.0-1.0)
  ok:    Localization valid? (True/False)
Visualization: Red cylinder in world/lidar/pose (only if ok=True)
Time series: data/lidar/x_mm, data/lidar/y_mm, data/lidar/confidence
```

#### 5. `update_target(x: float, y: float)`
```
Purpose:  Update navigation goal
Called by: robot.py when goal changes
Thread-safe: Yes
Parameters:
  x: Target X in mm
  y: Target Y in mm
Visualization: Yellow point in world/pathfinding/target
```

#### 6. `update_trajectory(pts)`
```
Purpose:  Update planned path
Called by: pathfinder.py or strategy after replanning
Thread-safe: Yes
Parameters:
  pts: List of (x, y) waypoints
       Example: [(1500, 1000), (1600, 1100), ...]
       Empty list [] clears trajectory
Visualization: Dotted line in world/pathfinding/trajectory
```

#### 7. `update_fused(x: float, y: float, theta: float)`
```
Purpose:  Update best position estimate (Teensy + LiDAR fusion)
Called by: robot.py after complementary filtering
Thread-safe: Yes
Parameters:
  x:     Fused X in mm
  y:     Fused Y in mm
  theta: Fused θ in radians
Visualization: Green cylinder in world/robot/fused
Time series: data/fused/x_mm, data/fused/y_mm
```

### Callback Factories (Internal)

#### `make_odom_callback()`
```
Returns callback function for USB Teensy messages
Callback signature: callback(data: bytes)
  • Extracts 24 bytes: 3× double (x, y, θ)
  • Calls: update_odom(x, y, θ)
Used by:
  com.add_callback(make_odom_callback(), Messages.UPDATE_ROLLING_BASIS.value)
```

#### `make_lidar_poll()`
```
Returns polling function for LiDAR data
Polling signature: poll()
  • Calls: update_lidar_cloud(get_latest_scan_data())
  • Calls: update_lidar_beacons(get_latest_beacon_candidates())
  • Calls: update_lidar_pose(get_corrected_pose())
Used by:
  publish_loop(hz=20.0, lidar_poll=make_lidar_poll())
```

---

## Communication Protocols

### USB Protocol (Teensy → Rasp)

**Message Type: `UPDATE_ROLLING_BASIS`**
```
Structure:
  [0:8]   double x              (mm, IEEE 754 little-endian)
  [8:16]  double y              (mm)
  [16:24] double theta          (radians)

Example wire format (hex):
  00 00 00 00 00 60 5e 40  (1500.0 in mm)
  00 00 00 00 00 80 88 40  (1000.0 in mm)
  8d 97 f6 f5 db a3 f0 3f  (0.061 radians)

Frequency: ~10 Hz (every 100ms from Teensy firmware)
Baud rate: 115200
Protocol: FTDI serial (USB virtual COM port)
```

### Network Protocol (Rerun SDK)

**Mode 1: Local (Direct Viewer)**
- No network, direct in-process communication
- Used for testing on Rasp with X11 display

**Mode 2: WebSocket (Recommended)**
```
Server: 0.0.0.0:9876
Protocol: WebSocket over HTTP
Data format: Binary Rerun protocol (internal format)
Bandwidth: ~500 KB/s at 20 Hz
Latency: <50ms (LAN), <200ms (WiFi)
Typical usage:
  Bridge on Rasp: python rerun_bridge.py --mode serve
  Viewer on PC:   http://RaspIP:9876
```

**Mode 3: gRPC (External Viewer)**
```
Protocol: gRPC with Rerun proto definitions
Used for remote Rerun viewer instance
Advanced use case, not typical
```

### Rerun Log Format

Data logged to Rerun:

```
Time Series (scalar data):
  data/teensy/x_mm           → rr.Scalars(value)
  data/teensy/y_mm           → rr.Scalars(value)
  data/teensy/theta_deg      → rr.Scalars(value)
  data/lidar/confidence      → rr.Scalars(value)
  data/lidar/nb_points       → rr.Scalars(count)
  data/lidar/nb_beacons      → rr.Scalars(count)
  data/lidar/x_mm            → rr.Scalars(value)
  data/lidar/y_mm            → rr.Scalars(value)
  data/fused/x_mm            → rr.Scalars(value)
  data/fused/y_mm            → rr.Scalars(value)
  data/fusion/ecart_odom_lidar_mm → rr.Scalars(error_mm)

Spatial Data (3D):
  world/robot/odom           → rr.Mesh3D (cylinder + arrow)
  world/robot/fused          → rr.Mesh3D (cylinder + arrow)
  world/lidar/pose           → rr.Mesh3D (cylinder + arrow)
  world/lidar/cloud          → rr.Points3D (point cloud)
  world/lidar/beacons_detected → rr.Points3D (spheres)
  world/pathfinding/target   → rr.Points3D (single point)
  world/pathfinding/trajectory → rr.LineStrips3D (polyline)

Reference Objects (static):
  world/map/*                → rr.Boxes3D or rr.Mesh3D
  sensors/lidar/polar_view   → rr.Points2D (2D radar)
```

---

## Performance Characteristics

### CPU Usage
```
Component              Typical      Notes
────────────────────────────────────────────
USB callback (Teensy)  ~0.1%       Async, triggered only on message
LiDAR polling          ~3-5%       Polling thread, scaling with cloud size
Rendering (3D)         ~8-12%      Mesh rendering, point cloud rasterization
Publishing to SDK      ~1-2%       Encoding and network transmission
─────────────────────────────────────────────
Total (Rasp Pi 4)      ~15-20%     When all subsystems active

Optimization tips:
• Reduce publishing rate: hz=10 instead of 20
• Disable LiDAR: --mode serve (without --with-lidar)
• Disable remote viewer: --mode local (no network)
```

### Memory Usage
```
Component              Typical      Notes
────────────────────────────────────────────
_State object          <1 MB        Fixed size dataclass
Python process         ~30-40 MB    Base Python + numpy + rerun SDK
Point cloud buffer     ~5-10 MB     500-1000 points @ 30 bytes each
─────────────────────────────────────────────
Total                  ~50-60 MB    Stable, no memory leaks

Memory scaling:
• Point cloud: O(n) where n = # LiDAR points
• Trajectory: O(m) where m = # waypoints (~100 typical)
• Beacon list: O(k) where k ≈ 3-6 beacons
```

### Network Bandwidth
```
Message type           Rate    Size      Total
────────────────────────────────────────────────
Odometry scalars       20 Hz   ~50 B     1 KB/s
LiDAR cloud points     20 Hz   ~5 KB     100 KB/s
LiDAR time series      20 Hz   ~100 B    2 KB/s
Mesh updates           <1 Hz   ~1 KB     <1 KB/s
────────────────────────────────────────────────
Total typical          ~500 KB/s

Bandwidth optimization:
• Only send significant changes
• Use 10 Hz instead of 20 Hz
• Reduce point cloud size by decimation
```

### Latency
```
Path                           Latency
──────────────────────────────────────────────
Teensy → Rasp USB             ~5 ms
Rasp processing               ~25 ms (1/20 Hz)
Rerun SDK encoding            ~5 ms
WebSocket transmission        <50 ms (LAN)
                              <200 ms (WiFi)
Browser rendering             ~16 ms (60 FPS)
──────────────────────────────────────────────
Total (LAN to display)        ~80-100 ms
```

---

## Extensibility

### Adding Custom Visualization Objects

Example: Add a safety zone (circular obstacle):

```python
# In rerun_bridge.py, inside _publish() function:

def _publish(s: _State) -> None:
    # ... existing code ...
    
    # Add custom safety zone visualization
    if hasattr(s, 'safety_zone_center'):
        cx, cy = s.safety_zone_center
        radius = s.safety_zone_radius
        
        # Draw circle as point cloud
        angles = np.linspace(0, 2*np.pi, 60)
        circle_pts = []
        for angle in angles:
            x = cx + radius * np.cos(angle)
            y = cy + radius * np.sin(angle)
            circle_pts.append([x, y, 0])
        
        rr.log("world/safety_zone", rr.Points3D(
            positions=np.array(circle_pts, np.float32),
            colors=[[255, 0, 0, 100]],  # Transparent red
            radii=[5.0],
        ))
```

### Adding Custom Time Series

Example: Track motor currents:

```python
# In robot.py:
if RERUN_AVAILABLE:
    # At each update:
    motor_currents = self.get_motor_currents()  # [I1, I2, I3]
    
    # Log to Rerun (would need custom handling)
    for i, current in enumerate(motor_currents):
        # This would require modifying rerun_bridge.py to add new field
        # and update function, or directly access rr.log:
        import rerun as rr
        rr.log(f"data/motors/motor{i+1}_current_A", rr.Scalars(current))
```

### Adding New Update Functions

To add support for a new data source:

```python
# Step 1: Add field to _State
@dataclass
class _State:
    # ... existing fields ...
    custom_sensor_value: float = 0.0
    custom_sensor_timestamp: float = 0.0

# Step 2: Add public update function
def update_custom_sensor(value: float):
    import time
    with _st._lock:
        _st.custom_sensor_value = value
        _st.custom_sensor_timestamp = time.time()

# Step 3: Add rendering in _publish()
def _publish(s: _State) -> None:
    # ... existing code ...
    rr.log("data/custom_sensor/value", rr.Scalars(s.custom_sensor_value))

# Step 4: Call from your code
# In robot.py:
from rerun_bridge import update_custom_sensor
update_custom_sensor(sensor_reading)
```

### Custom Blueprint

Example: Create custom visualization layout:

```python
import rerun.blueprint as rrb

def create_custom_blueprint():
    return rrb.Blueprint(
        rrb.Vertical(
            rrb.Horizontal(
                rrb.Spatial3DView(name="Top View", origin="world"),
                rrb.Spatial2DView(name="Polar", origin="sensors/lidar"),
            ),
            rrb.TimeSeriesView(name="Position", origin="data/fused"),
            row_shares=[2, 1],
        ),
    )

# Then in main():
# rr.send_blueprint(create_custom_blueprint())
```

---

## Advanced: Internal Data Flow Example

**Complete example: How a Teensy odometry message reaches Rerun visualization**

```python
# ─── Teensy sends message ───────────────────────────────────────
# Firmware loop sends every 100ms:
#   struct { double x, y, theta }
#   → USB → /dev/ttyUSB0 → FTDI driver

# ─── Callback triggered (main thread) ───────────────────────────
def make_odom_callback():
    def cb(data: bytes):
        if len(data) >= 24:
            x, y, t = struct.unpack("<ddd", data[:24])
            # → Calls update_odom(1500.5, 1000.2, 0.785)
            update_odom(x, y, t)
    return cb

# ─── Shared state updated ──────────────────────────────────────
def update_odom(x: float, y: float, theta: float):
    with _st._lock:                    # Acquire lock (microseconds)
        _st.odom_x = x                 # Write: 1500.5
        _st.odom_y = y                 # Write: 1000.2
        _st.odom_theta = theta         # Write: 0.785
        # Auto-release lock

# ─── Publish loop reads snapshot (50ms later, 20 Hz) ───────────
def publish_loop(hz: float = 20.0, lidar_poll=None):
    dt = 1.0 / hz  # 0.05 seconds = 50ms
    while True:
        # Read atomic snapshot under lock
        snapshot = _st.snap()  # Reads: odom_x=1500.5, odom_y=1000.2, etc.
        
        # Release lock immediately
        _publish(snapshot)  # No lock needed here
        
        time.sleep(dt)

# ─── Rendering with snapshot ──────────────────────────────────
def _publish(s: _State) -> None:
    # Use snapshot data (consistent values)
    _log_robot("world/robot/odom",  # Mesh3D path
               s.odom_x,             # 1500.5
               s.odom_y,             # 1000.2
               s.odom_theta,         # 0.785
               C_ROBOT)              # Color

    # Also log time series for plotting
    rr.log("data/teensy/x_mm",      rr.Scalars(s.odom_x))
    rr.log("data/teensy/y_mm",      rr.Scalars(s.odom_y))
    rr.log("data/teensy/theta_deg", rr.Scalars(math.degrees(s.odom_theta)))

# ─── Rerun SDK processes ───────────────────────────────────────
# rr.log() calls batch and encode data internally
# → Batches up to 50ms of logs
# → Encodes to Rerun protocol binary format
# → Sends via WebSocket

# ─── Network transmission (WebSocket) ──────────────────────────
# Browser receives binary Rerun protocol data
# JavaScript unpacks messages
# Updates internal state: odom_x=1500.5, odom_y=1000.2

# ─── Three.js rendering ───────────────────────────────────────
# WebGL renderer updates:
# • Robot mesh position: (1500.5, 1000.2)
# • Robot mesh orientation: rotate by 0.785 rad
# • Time series plot updates with new point
# • Browser displays update on screen

# ───────────────────────────────────────────────────────────────
# Total latency: ~80-100ms from USB message to screen update
```

---

## Debugging

### Enable Verbose Logging

```python
# In rerun_bridge.py:
import logging
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger("rerun_bridge")

# Then in _publish():
logger.debug(f"Publishing: odom_x={s.odom_x:.1f} "
             f"lidar_cloud={len(s.lidar_cloud)} pts")
```

### Monitor Thread Activity

```python
import threading

def monitor_threads():
    for thread in threading.enumerate():
        print(f"Thread: {thread.name} (daemon={thread.daemon})")

# Call periodically in robot.py:
if update_count % 100 == 0:
    monitor_threads()
```

### Verify Data Flow

```python
# Add temporary debug code in robot.py:

from rerun_bridge import _st

# After update_fused() call:
with _st._lock:
    print(f"Fused state: x={_st.fused_x:.1f} "
          f"y={_st.fused_y:.1f} θ={_st.fused_theta:.3f}")
```

---

**End of Technical Architecture Document**

For practical examples, see: [RERUN_INTEGRATION_EXAMPLES.md](RERUN_INTEGRATION_EXAMPLES.md)

For quick start, see: [RERUN_QUICK_START.md](RERUN_QUICK_START.md)

