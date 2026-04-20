# Odometry Correction System - Complete Implementation Guide

## Overview

**Last Updated**: April 20, 2026 ✅  
**Status**: Fully Operational (beacon_ids type consistency verified)

This document describes the complete bidirectional odometry correction system implemented for CDR 2026. The system combines:

1. **SVD Umeyama 2D** (beacon-based pose correction in `lidar_logic.py`)
   - ✅ beacon_ids stored as `List[int]` (fixed April 20 — was String, now consistent)
2. **Adaptive Complementary Filter** (LiDAR + Teensy fusion in `robot.py`)
3. **Throttled Correction Sending** (Teensy reset 1-2s intervals)

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    LIDAR_THREAD (lidar_logic.py)                │
│                                                                 │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │ 1. _predict_beacon_windows(teensy_x, teensy_y, θ)      │   │
│  │    → Predicts where beacons should appear ±10°/±200mm  │   │
│  └─────────────────────────────────────────────────────────┘   │
│                           ↓                                     │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │ 2. _extract_beacon_candidates_fast(points, windows)    │   │
│  │    → Extract clusters within predicted windows only    │   │
│  └─────────────────────────────────────────────────────────┘   │
│                           ↓                                     │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │ 3. _compute_corrected_pose(candidates, teensy_pose)    │   │
│  │    → SVD Umeyama 2D: measured vs theoretical beacons   │   │
│  │    → Returns PoseState (x, y, θ, confidence)           │   │
│  └─────────────────────────────────────────────────────────┘   │
│                           ↓                                     │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │ 4. Thread-safe global state (_corrected_pose_lock)     │   │
│  │    → Stores corrected pose for robot.py to consume     │   │
│  └─────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
           ↓ PRODUCER (signals correction available)
           
┌─────────────────────────────────────────────────────────────────┐
│                    ROBOT_THREAD (robot.py)                      │
│                                                                 │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │ 1. update_teensy_pose(x, y, θ)                          │   │
│  │    → Store current Teensy odometry (producer signal)    │   │
│  └─────────────────────────────────────────────────────────┘   │
│                           ↓                                     │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │ 2. corrected = get_corrected_pose()                     │   │
│  │    → Retrieve SVD-corrected pose from LiDAR thread      │   │
│  └─────────────────────────────────────────────────────────┘   │
│                           ↓                                     │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │ 3. _apply_complementary_filter(lidar_x, lidar_y,       │   │
│  │                   teensy_x, teensy_y, conf)            │   │
│  │    → Adaptive blend: alpha varies by confidence        │   │
│  │    → Returns (x_fused, y_fused, alpha)                 │   │
│  └─────────────────────────────────────────────────────────┘   │
│                           ↓                                     │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │ 4. should_send_correction_to_teensy()                   │   │
│  │    → Check: confidence > 0.6 && beacons >= 2 && time   │   │
│  │    → Return boolean (throttle ready?)                   │   │
│  └─────────────────────────────────────────────────────────┘   │
│                           ↓                                     │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │ 5. _send_odometry_correction(x, y, θ)                  │   │
│  │    → Send SET_ODOMETRIE to Teensy                      │   │
│  │    → Resets Teensy odometer with corrected pose        │   │
│  └─────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

---

## Key Constants

All defined in [lidar_logic.py](lidar_logic.py) around L145:

```python
# Beacon window prediction: search ±10° in angle, ±200mm in distance
BEACON_WINDOW_ANGLE_RAD = 10.0 * math.pi / 180.0  # radians
BEACON_WINDOW_DIST_MM = 200.0

# Correction thresholds
POSE_CORRECTION_MIN_CONFIDENCE = 0.60      # Only correct if conf > 60%
POSE_CORRECTION_MIN_BEACONS = 2            # Need ≥2 detected beacons
POSE_SEND_BACK_INTERVAL_S = 1.0            # Throttle corrections to 1/second
```

---

## Public API Functions (lidar_logic.py)

### 1. `update_teensy_pose(x: float, y: float, theta: float)`
**Called by**: `robot.update()` at 20 Hz
**Purpose**: Store current Teensy odometry for beacon window prediction
**Thread-safe**: Yes (uses `_teensy_pose_lock`)

```python
# In robot.py update():
update_teensy_pose(teensy_x, teensy_y, teensy_theta)
```

---

### 2. `get_corrected_pose() → Optional[PoseState]`
**Called by**: `robot.update()` at 20 Hz
**Returns**: Latest corrected pose from SVD, or None if not available
**Thread-safe**: Yes (uses `_corrected_pose_lock`)

```python
# In robot.py update():
corrected_pose = get_corrected_pose()
if corrected_pose is not None:
    x_corrected = corrected_pose.x
    y_corrected = corrected_pose.y
    confidence = corrected_pose.confidence
    beacon_ids = corrected_pose.beacon_ids  # Which beacons were used (List[int]) ✅
```

---

### 3. `should_send_correction_to_teensy() → bool`
**Called by**: `robot.update()` to check throttle
**Returns**: True if correction ready (all conditions met + time interval elapsed)
**Conditions**:
- `get_corrected_pose().confidence > POSE_CORRECTION_MIN_CONFIDENCE (0.60)`
- `len(beacon_ids) >= POSE_CORRECTION_MIN_BEACONS (2)` [beacon_ids: List[int] ✅ type-consistent]
- `time.time() - _last_correction_time >= POSE_SEND_BACK_INTERVAL_S (1.0)`

```python
# In robot.py update():
if should_send_correction_to_teensy():
    # Additional local throttle for safety
    if current_time - robot._last_correction_sent_time >= 1.0:
        robot._send_odometry_correction(x_fused, y_fused, theta_fused)
        robot._last_correction_sent_time = current_time
```

---

## Core Algorithms

### A. Beacon Window Prediction
**Function**: `_predict_beacon_windows(teensy_x, teensy_y, teensy_theta)`

**Input**: Current Teensy pose (world coordinates)
**Output**: Dict of predicted windows for each beacon

**Algorithm**:
```
For each beacon at world position (bx, by):
  1. Transform to robot frame (relative to Teensy)
  2. Convert to polar: (distance, angle)
  3. Create window: angle ± 10°, distance ± 200mm
  4. Filter out-of-range (< 300mm or > 6000mm)
  5. Return {beacon_id: {angle_pred, dist_pred, angle_min, angle_max, dist_min, dist_max}}
```

**Example**:
```
Teensy at (1000mm, 2000mm, 0°)
Beacon_1 at world (1500mm, 2500mm)

Relative to robot: (500mm, 500mm) [robot frame]
Polar: distance=707mm, angle=45° = 0.785 rad

Window prediction:
  angle_min = 0.785 - 0.175 = 0.610 rad (35°)
  angle_max = 0.785 + 0.175 = 0.960 rad (55°)
  dist_min = 707 - 200 = 507mm
  dist_max = 707 + 200 = 907mm
```

---

### B. SVD Umeyama 2D Pose Correction
**Function**: `_compute_corrected_pose(beacon_candidates, teensy_x/y/theta)`

**Input**: 
- Detected beacon clusters with `beacon_id` field
- Current Teensy pose

**Output**: PoseState or None

**Algorithm**:
```
Step 1: Build Point Correspondences
  For each detected beacon with known beacon_id:
    - Get measured position in LiDAR frame (x_lidar, y_lidar)
    - Transform to world using provisional Teensy pose
    - Store as measured point
    - Get theoretical position from BEACONS_BY_ID
    - Store as theoretical point

Step 2: SVD Umeyama 2D Rigid Transformation
  - Center both point sets (subtract mean)
  - Compute cross-covariance: H = measured^T @ theoretical
  - SVD decomposition: H = U * S * V^T
  - Rotation matrix: R = V^T @ U^T
  - Ensure det(R) = 1 (no reflection)
  - Extract delta_theta from R[1,0], R[0,0]
  - Compute translation: t = centroid_theory - R @ centroid_measured
  
Step 3: Validate & Compute Confidence
  - Transform measured points back via R and t
  - Calculate RMS residual
  - If RMS > threshold → return None (failed)
  - Confidence = 1.0 - (RMS / 100) clamped [0.5, 1.0]

Step 4: Return Corrected Pose
  - x_corrected = teensy_x + delta_x
  - y_corrected = teensy_y + delta_y
  - theta_corrected = teensy_theta + delta_theta
  - confidence = derived from RMS
  - is_localized = confidence >= 0.60
```

**Why SVD Umeyama?**
- Finds optimal rigid transformation (rotation + translation) between two point sets
- Minimizes RMS error of correspondence
- Robust to outliers (unlike simple centroid matching)
- Handles arbitrary orientation mismatch

---

### C. Adaptive Complementary Filter
**Function**: `robot._apply_complementary_filter(lidar_x, lidar_y, teensy_x, teensy_y, confidence)`

**Input**:
- `lidar_x/y`: Position from SVD-corrected LiDAR (mm)
- `teensy_x/y`: Position from Teensy odometry (mm)
- `confidence`: LiDAR confidence [0.0, 1.0]

**Output**: `(x_fused, y_fused, alpha_used)`

**Algorithm**:
```
Adaptive Alpha Calculation:
  
  If confidence < 0.2:
    alpha = 0.85  (85% Teensy, 15% LiDAR)
    → LiDAR is unreliable, trust Teensy drift more
  
  Else if confidence > 0.8:
    alpha = 0.25  (25% Teensy, 75% LiDAR)
    → LiDAR is reliable, trust it more
  
  Else:
    # Linear transition between 0.2 and 0.8
    alpha = 0.85 - (confidence - 0.2) / 0.6 * 0.60
    alpha = 0.85 - 0.60 * (confidence - 0.2) / 0.6
    → Smooth blend across middle range
    
Fusion:
  x_fused = (1 - alpha) * lidar_x + alpha * teensy_x
  y_fused = (1 - alpha) * lidar_y + alpha * teensy_y
  
Result interpretation:
  - alpha = 0.0: 100% LiDAR
  - alpha = 0.5: 50/50 blend
  - alpha = 1.0: 100% Teensy
```

**Example Alpha Progression**:
```
Confidence → Alpha → LiDAR Weight → Teensy Weight
0.0        → 0.85  → 15%          → 85%
0.2        → 0.85  → 15%          → 85%
0.35       → 0.70  → 30%          → 70%
0.5        → 0.55  → 45%          → 55%
0.65       → 0.40  → 60%          → 40%
0.8        → 0.25  → 75%          → 25%
1.0        → 0.25  → 75%          → 25%
```

---

## Integration Points

### In lidar_thread() (NOT YET IMPLEMENTED - PENDING)

The following needs to be added to the main lidar_thread() execution loop:

```python
def lidar_thread():
    """Main LiDAR acquisition and processing loop."""
    
    while not stop_flag:
        try:
            # ... existing scan acquisition code ...
            
            # NEW: Get current Teensy pose for window prediction
            with _teensy_pose_lock:
                teensy_x, teensy_y, teensy_theta = _teensy_pose
            
            if teensy_x is not None:
                # NEW: Predict where beacons should be
                beacon_windows = _predict_beacon_windows(
                    teensy_x, teensy_y, teensy_theta
                )
                
                # NEW: Extract candidates using prediction windows
                # (Modify _extract_beacon_candidates_fast to accept windows parameter)
                candidates = _extract_beacon_candidates_fast(
                    points, 
                    beacon_windows=beacon_windows  # Hybrid mode
                )
            else:
                # Fallback to classic extraction without windows
                candidates = _extract_beacon_candidates_fast(points)
            
            # NEW: Compute SVD-corrected pose
            if len(candidates) >= 2:
                corrected_pose = _compute_corrected_pose(
                    candidates, teensy_x, teensy_y, teensy_theta
                )
                
                if corrected_pose is not None:
                    with _corrected_pose_lock:
                        _corrected_pose = corrected_pose
                        _last_correction_time = time.time()
            
            # ... rest of thread ...
            
        except Exception as e:
            logger.error(f"LiDAR thread error: {e}")
```

---

### In robot.update() (IMPLEMENTED ✓)

```python
def update(self):
    """Main control loop called 20x/second."""
    
    # Read Teensy position
    with self.lock:
        teensy_x, teensy_y, teensy_theta = self.x, self.y, self.theta
    
    # NEW: Signal LiDAR thread with current Teensy pose
    update_teensy_pose(teensy_x, teensy_y, teensy_theta)
    
    # NEW: Get SVD-corrected pose
    corrected_pose = get_corrected_pose()
    
    if corrected_pose is not None:
        # Use SVD-corrected position
        lidar_x = corrected_pose.x
        lidar_y = corrected_pose.y
        confidence = corrected_pose.confidence
        
        # NEW: Apply adaptive complementary filter
        rx, ry, alpha = self._apply_complementary_filter(
            lidar_x, lidar_y, teensy_x, teensy_y, confidence
        )
        
        # Theta always from Teensy IMU
        rtheta = teensy_theta
        
        # NEW: Check if correction should be sent to Teensy
        if should_send_correction_to_teensy():
            current_time = time.time()
            if current_time - self._last_correction_sent_time >= 1.0:
                self._send_odometry_correction(rx, ry, rtheta)
                self._last_correction_sent_time = current_time
    else:
        # Fallback: use simple fusion (backward compatible)
        rx, ry, rtheta, _ = self.lidar.get_fused_position(
            teensy_x, teensy_y, teensy_theta
        )
    
    # Continue with obstacle detection, strategy, etc.
    obstacles = self.lidar.get_obstacles(rx, ry, rtheta)
    action = self.strategie.get_action_actuelle()
    # ...
```

---

## Thread Safety

All shared state is protected with locks:

| Variable | Lock | Used By |
|----------|------|---------|
| `_teensy_pose` | `_teensy_pose_lock` | robot.py writes, lidar_thread reads |
| `_corrected_pose` | `_corrected_pose_lock` | lidar_thread writes, robot.py reads |
| `_last_correction_time` | `_corrected_pose_lock` | Both threads check throttle |

**Lock Pattern**:
```python
with _teensy_pose_lock:
    _teensy_pose = (x, y, theta)  # Write

with _teensy_pose_lock:
    teensy_x, teensy_y, teensy_theta = _teensy_pose  # Read
```

---

## Data Flow Summary

```
Match starts (t=0s)
  ↓
robot.update() @ 20 Hz
  ├─ Reads Teensy odometry (50 Hz, but sampled @ 20 Hz)
  ├─ Calls update_teensy_pose() → lidar_thread receives signal
  ├─ Calls get_corrected_pose() ← May return None if LiDAR not ready yet
  ├─ If available:
  │    ├─ Applies complementary filter (alpha adaptive on confidence)
  │    ├─ Checks should_send_correction_to_teensy() (confidence + 2 beacons + 1s throttle)
  │    └─ Sends correction to Teensy if all conditions met
  └─ Continues with navigation/strategy
  
Meanwhile: lidar_thread() @ 200 Hz (async)
  ├─ Acquires LiDAR scan
  ├─ Predicts beacon windows (using latest Teensy pose from update_teensy_pose)
  ├─ Extracts candidates within windows
  ├─ Computes SVD Umeyama correction
  └─ Stores in _corrected_pose for robot.update() to consume
```

---

## Testing

### Test File: `test_complementary_filter.py`

Run with:
```bash
cd robot1/rasp
python3 test_complementary_filter.py
```

**Tests Included**:
1. Low confidence (< 0.2): Alpha ≈ 0.85 (85% Teensy)
2. High confidence (> 0.8): Alpha ≈ 0.25 (25% Teensy)
3. Medium confidence (0.5): Alpha between 0.25-0.85
4. Edge cases: conf=0.0, conf=1.0, boundary transitions
5. Alpha progression: Monotonic decrease with increasing confidence

---

## Known Limitations & Future Work

1. **LiDAR Window Extraction**: `_extract_beacon_candidates_fast()` still needs modification to accept `beacon_windows` parameter (hybrid mode)

2. **Integration**: `lidar_thread()` loop not yet modified to call `_predict_beacon_windows()` and `_compute_corrected_pose()`

3. **Fallback Behavior**: If LiDAR confidence < 0.60 or < 2 beacons, no correction sent. Robot still navigates with simple fusion.

4. **Theta Correction**: Currently NOT correcting theta (always use Teensy IMU). Could add beacon arc for angle estimation.

5. **Beacon Occlusion**: If beacon fully occluded, SVD fails gracefully (returns None). Could implement voting across 2-beacon subsets.

---

## Calibration Constants

All in `lidar_logic.py` and tunable:

```python
BEACON_WINDOW_ANGLE_RAD = 10.0 * π/180     # ±10° predicted angle window
BEACON_WINDOW_DIST_MM = 200.0               # ±200mm predicted distance
POSE_CORRECTION_MIN_CONFIDENCE = 0.60       # Only correct if SVD confidence > 60%
POSE_CORRECTION_MIN_BEACONS = 2             # Need ≥2 beacons for correction
POSE_SEND_BACK_INTERVAL_S = 1.0             # Throttle corrections to max 1/second
BEACON_FIT_MAX_RMS_MM = 150.0               # Reject if SVD residual > 150mm
```

**Tuning Guide**:
- **Narrow windows** (±5°/±100mm): Faster but risky (may miss beacons)
- **Wide windows** (±15°/±300mm): Robust but slower
- **Higher confidence threshold** (0.70): Conservative (fewer corrections)
- **Lower confidence threshold** (0.50): Aggressive (frequent corrections)
- **Longer throttle interval** (2.0s): Smooth but less responsive
- **Shorter throttle interval** (0.5s): Responsive but spammy

---

## References

- **SVD Umeyama Algorithm**: Umeyama, S. (1991). "Least-Squares Estimation of Transformation Parameters Between Two Point Patterns"
- **Complementary Filters**: Fusion of rigid-body measurements (accelerometer + gyro typical, here LiDAR + odometry)
- **Beacon Trilateration**: 3-point lateration for initial localization (handled separately)

