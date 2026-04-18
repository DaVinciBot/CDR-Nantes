# CDR 2026 ROBOT CODEBASE ANALYSIS
**Date**: April 18, 2026 | **Status**: Pre-Match Readiness Assessment

---

## 1. FINALIZATION POINTS (Code Marked TODO/FIXME or Incomplete)

### 1.1 HARDWARE INTEGRATION GAPS

| File | Line | Issue | Severity | Details |
|------|------|-------|----------|---------|
| **robot.py** | 74-75 | Tirette (start pin) not reading | 🔴 CRITICAL | `time.sleep(3)` placeholder - should read GPIO instead |
| **robot.py** | 127-128 | Dynamic lookahead not implemented | 🟡 MEDIUM | Lookahead hardcoded to `min(4, len(chemin)-1)` instead of scaling with speed |
| **robot.py** | 143-145 | Actuator feedback missing | 🟡 MEDIUM | No state machine for arm/actuator validation - just passes instantly |
| **robot.py** | 166 | Arm cleanup incomplete | 🟡 MEDIUM | `self.actionneurs.ranger_bras()` commented out |
| **strategy_strat_manager.py** | 1-5 | Missing import | 🔴 CRITICAL | No `import time` but methods use `time.time()` (lines 50, 58) |

### 1.2 STRATEGY/ACTION VALIDATION

| File | Line | Issue | Severity | Details |
|------|------|-------|----------|---------|
| **strategy_strat_manager.py** | 18-32 | Example strategy only | 🟡 MEDIUM | Hardcoded test actions - real match strategy needs definition |
| **strategy_actions.py** | 1-16 | No validation logic | 🟡 MEDIUM | Action types exist but no state validation before transitions |
| **robot.py** | 104-149 | No path timeout | 🟡 MEDIUM | If path blocked indefinitely, robot waits without backoff strategy |

### 1.3 ACTIONNEUR INTEGRATION

| File | Line | Issue | Severity | Details |
|------|------|-------|----------|---------|
| **robot.py** | 40 | `self.actionneurs` never instantiated | 🔴 CRITICAL | Referenced in line 143 & 166 but never created in `__init__` |
| **robot.py** | 143 | Immediate validation (HACK) | 🟡 MEDIUM | Comment says "on valide instantanément pour tester la boucle" - needs real feedback |

---

## 2. TESTING POINTS (What Must Be Validated Before Match)

### 2.1 LOCALIZATION PIPELINE (LiDAR + Odometry Fusion)

| Component | Test | Expected Result | Risk |
|-----------|------|-----------------|------|
| **Beacon Detection** | Verify 3 beacons detected from known position | ≥2 beacons @ confidence ≥0.50 | If only 1 beacon found, confidence drops to 0.20 |
| **Beacon Trilateration** | Calculate pose from 3 beacons vs ground truth | Error < ±50mm, ±5° | RMS error tolerance: 80mm (line 101) |
| **2-Beacon Fallback** | Test with only 2 beacons visible | Confidence = 0.50 (line 100) | Limited accuracy - only use for emergency |
| **Fusion Alpha** | Verify LiDAR (0.35) vs Teensy (0.65) blending | Smooth transitions, no jerks | If alpha wrong, robot drifts or oscillates |
| **Opponent Detection** | Track opponent robot in arena | Timeout: 0.70s (line 46 lidar.py) | If not seen for 0.70s, marked invalid |

### 2.2 PATHFINDING & NAVIGATION

| Component | Test | Expected Result | Risk |
|-----------|------|-----------------|------|
| **A* Algorithm** | Path finding with obstacles | Finds route in <100ms | Raspberry Pi CPU limited |
| **Grid Inflation** | Obstacle margins correct | Robot stays ≥40mm from walls | margin = 40mm (pathfinder.py line 8) + ROBOT_RADIUS (120mm) = 160mm safety zone |
| **Lookahead Tracking** | Follow waypoint at lookahead_index=4 | Smooth curved trajectories | Hardcoded - should adapt to velocity |
| **Goal Detection** | Reach target within threshold | Must reach within 50mm | Line 111 robot.py: `if distance_restante < 50.0` |
| **Blocked Path Recovery** | If path blocked >2 frames | Log warning, try new path | Current: just stops without retry |

### 2.3 ACTUATOR CONTROL & STATE MACHINE

| Component | Test | Expected Result | Risk |
|-----------|------|-----------------|------|
| **Arm Movement** | Raise/lower arm, read position | State verified via feedback | CURRENTLY MISSING - validates instantly (line 145) |
| **Gripper/Pump** | Activate and confirm operation | Pneumatic pressure confirmed | No sensors in current code |
| **Action Sequencing** | Execute 3-action chain (move→arm↓→move) | Actions run in order | Strategy is placeholder (line 23-31 strat_manager.py) |
| **Timing Accuracy** | Wait 1.0s for arm to settle | Chrono must be accurate ±100ms | Uses `time.time()` - sufficient precision |

### 2.4 EMERGENCY PROCEDURES

| Component | Test | Expected Result | Risk |
|-----------|------|-----------------|------|
| **90s Timeout** | Match ends at exactly 90.0s | TEMPS_MATCH_SECONDES = 90.0 (main.py line 15) | Uses time.time() - may drift if CPU loaded |
| **Ctrl+C Shutdown** | Press Ctrl+C, motors stop, LiDAR shuts down | `stopper_tout()` called (robot.py line 165) | INCOMPLETE - no arm cleanup (line 166 commented) |
| **USB Disconnect Recovery** | Unplug Teensy mid-match | Error logged, safe state | No recovery mechanism - match ends |

---

## 3. MISSING DATA / CALIBRATION REQUIRED

### 3.1 LIDAR CALIBRATION (lidar_logic.py)

#### Connection Parameters
```python
PORT = '/dev/ttyUSB0'                  # Line 50 - PLATFORM SPECIFIC (Linux only)
BAUDRATE = 256000                      # Line 51 - Must match hardware
TIMEOUT = 3                            # Line 52 - Seconds to wait for connection
MIN_DIST = 50                          # Line 53 - Below 50mm filtered out
MAX_DIST = 12000                       # Line 54 - Above 12m filtered out  
MIN_QUAL = 1                           # Line 55 - Quality threshold (0-15)
```

**STATUS**: ⚠️ `/dev/ttyUSB0` hardcoded - Will fail on Windows or different USB port
**ACTION NEEDED**: Make port configurable OR detect auto

#### Beacon Detection Thresholds (Critical for Trilateration)
```python
BEACON_QUAL_MIN = 2                    # Line 74 - Minimum signal quality
BEACON_ANG_GAP_RAD = math.radians(2.5) # Line 75 - 2.5° max angular gap
BEACON_DIST_GAP_MM = 90.0             # Line 76 - 0.90 × 100mm (10cm beacon)
BEACON_MIN_RETURNS_PER_CLUSTER = 2     # Line 77 - Points needed per beacon
BEACON_MAX_CANDIDATES = 3               # Line 78 - Exactly 3 beacons on field ✓
BEACON_FIT_MAX_RMS_MM = 80.0           # Line 79 - Was 140, tightened to 80
BEACON_GEOM_TOL_MM = 70.0              # Line 80 - Was 180, tightened to 70
BEACON_FACE_MIN_LEN_MM = 65.0          # Line 82 - 0.65 × 100mm minimum
BEACON_FACE_MAX_LEN_MM = 105.0         # Line 83 - 1.05 × 100mm maximum
```

**CONCERN**: Lines 79-80 show recent tightening from 140→80mm and 180→70mm
**ACTION NEEDED**: Validate these are correct for real beacons. If beacons smaller or further, may not localize

#### Pose Calculation Thresholds
```python
AUTO_POSE_MIN_CONFIDENCE = 0.20        # Line 85 - Minimum to declare "localized"
TWO_BEACON_BASE_CONFIDENCE = 0.50      # Line 86 - Only 2 beacons detected
POSE_CONTINUITY_RMS_TOL_MM = 28.0      # Line 87 - Reject jumps >28mm between frames
POSE_BLEND_ALPHA_XY = 0.35             # Line 88 - 35% LiDAR, 65% Teensy odometry
POSE_BLEND_ALPHA_THETA = 0.30          # Line 89 - 30% LiDAR angle, 70% IMU
POSE_DEADBAND_MM = 15.0                # Line 90 - Ignore drifts <15mm
POSE_DEADBAND_DEG = 1.5                # Line 91 - Ignore drifts <1.5°
POSE_MAX_JUMP_MM = 260.0               # Line 92 - Reject jumps >260mm
POSE_MAX_JUMP_DEG = 28.0               # Line 93 - Reject angle jumps >28°
FUSION_MIN_CONFIDENCE = 0.14           # Line 100 - Minimum to use LiDAR correction
```

**KEY POINT**: Line 88 alpha=0.35 means odometry weighted 65% - **LiDAR is secondary**
**VALIDATION NEEDED**: Test actual localization error vs these thresholds

#### Opponent Detection Thresholds
```python
ROBOT_MIN_RADIUS_MM = 60.0             # Line 104 - Too small = noise
ROBOT_MAX_RADIUS_MM = 220.0            # Line 105 - Robot is ~240mm → may miss
OPPONENT_BEACON_EXCLUSION_MM = 200.0   # Line 106 - Avoid confusing with beacons
OPPONENT_MAX_MISSED = 12               # Line 107 - Lost after 12 frames (~0.6s @ 20Hz)
```

**ISSUE**: ROBOT_MAX_RADIUS_MM = 220mm but typical robot ~240mm wide
**ACTION NEEDED**: Increase to 250mm or verify actual opponent size

### 3.2 PATHFINDING CALIBRATION (pathfinder.py)

```python
GRID_SIZE = 50                         # Line 3 - Cell resolution = 50mm
MARGIN = 40                            # Line 4 - Safety margin around obstacles
inflation_radius = ROBOT_RADIUS + MARGIN = 120 + 40 = 160mm  # Line 5
```

**CALCULATION**: 
- Grid cells: 3000mm width ÷ 50mm = 60 cells × 40 cells (2000mm height)
- Inflation zone: Robot stays ≥160mm from any obstacle
- **VALIDATION NEEDED**: Test with real obstacles to verify 160mm is sufficient

#### Dynamic Parameters NOT YET IMPLEMENTED
```python
lookahead_index = min(4, len(chemin) - 1)  # robot.py line 127
```

**ISSUE**: Hardcoded to 4 waypoints ahead
**SHOULD BE**: Proportional to robot velocity (not implemented - TODO line 127)
- If moving at 1000mm/s and update at 20Hz: 50mm per frame
- 4 waypoints × 50mm cells = 200mm lookahead
- At higher speed, should be 6-8 waypoints

### 3.3 STRATEGY CALIBRATION (strategy_strat_manager.py)

Current hardcoded example (lines 18-32):
```python
Action(DEPLACEMENT, cible_x=500, cible_y=500)      # Move away from start
Action(DEPLACEMENT, cible_x=1200, cible_y=800)     # Move to task area
Action(ACTIONNEUR, nom_actionneur="BAISSER_BRAS")  # Lower arm
Action(ATTENTE, temps_attente=1.0)                 # Wait 1.0s
Action(DEPLACEMENT, cible_x=200, cible_y=200)      # Return
```

**STATUS**: ⚠️ Test strategy only - real match strategy undefined
**CALIBRATIONS NEEDED**:
- [ ] Actual task positions from field layout
- [ ] Arm timing (currently 1.0s - needs validation)
- [ ] Number of tasks × total time budget (90s)
- [ ] Backup strategy if opponent blocks primary path

### 3.4 FUSION PARAMETERS (lidar.py)

```python
alpha_xy = 0.35              # Line 82 - Fusion weight for X,Y position
opponent_timeout_s = 0.70    # Line 46 - Opponent validity timeout
opponent confidence < 0.1    # Line 114 - Minimum opponent confidence to track
```

**VALIDATION NEEDED**:
- [ ] Test alpha=0.35 with mixed LiDAR/odometry data
- [ ] Verify opponent timeout doesn't cause false tracking losses
- [ ] Check confidence thresholds during full match simulation

---

## 4. INTEGRATION GAPS (Modules Not Connected)

### 4.1 MISSING MODULE: Actionneur/Bras Controller

| Reference | Location | Status | Impact |
|-----------|----------|--------|--------|
| `self.actionneurs` | robot.py line 143 | ❌ UNDEFINED | Actuator feedback never happens |
| `ranger_bras()` | robot.py line 166 | ❌ NOT CALLED | Arm never closes/retracts on shutdown |
| Action type ACTIONNEUR | strategy_actions.py line 8 | ✓ Defined | But no hardware backend |

**MISSING INTEGRATION**:
- No class instantiation for actuator control
- No feedback mechanism (sensor reading)
- No error handling if arm movement fails

**REQUIRED**:
1. Create `ActionneurController` class (possibly in `teensy_actuator/`)
2. Add to `robot.py __init__()`: `self.actionneurs = ActionneurController(...)`
3. Implement `_handle_actuator_feedback()` callback
4. Add validation in robot.py line 143:
```python
if self.actionneurs.is_action_done():
    self.strategie.valider_action_terminee()
```

### 4.2 MISSING MODULE: Import Statement in Strategy Manager

**File**: `robot1/rasp/strategy/strategy_strat_manager.py`
**Line 1-5**: Missing `import time`
**Uses**: Lines 50, 58

```python
# CURRENT (BROKEN):
import logging
from .actions import Action, TypeAction

class StratManager:
    def demarrer_chrono_si_necessaire(self):
        self.chrono_action = time.time()  # NameError: time not defined!
```

**FIX**:
```python
import logging
import time  # ADD THIS LINE
from .actions import Action, TypeAction
```

### 4.3 INCOMPLETE: Callback Attachment

**File**: `robot.py` line 45
```python
self.com.add_callback(self._handle_position, self.Messages.UPDATE_ROLLING_BASIS.value)
```

**Status**: Position callback attached ✓
**Missing**: No callback for:
- [ ] Actuator status updates
- [ ] Button/sensor events
- [ ] Error conditions (motor stall, timeout)

### 4.4 MISSING: Terrain Obstacle-Pathfinder Sync

**Current Flow**:
1. `Terrain` object created with static obstacles (terrain_jeu.py)
2. `PathFinder` inflates obstacles into grid (pathfinder.py lines 18-20)
3. Dynamic obstacles from LiDAR added at pathfind time

**ISSUE**: If terrain obstacles change, grid is stale
**STATUS**: OK for static obstacles, but if any obstacles added dynamically, pathfinder needs rebuild

### 4.5 INCOMPLETE: opponent → pathfinding integration

**Current**:
```python
obstacles = self.lidar.get_obstacles(rx, ry, rtheta)  # robot.py line 108
chemin = self.cerveau.get_path({'x': rx, 'y': ry}, objectif, obstacles)  # line 109
```

**ISSUE**: Opponent position from LiDAR NOT passed to pathfinder
**Should be**:
```python
opp = self.lidar.get_opponent()  
if opp:
    opp_x, opp_y, conf = opp
    obstacles.append((opp_x, opp_y))  # Add opponent as obstacle
```

**MISSING**: Opponent radius not considered (should expand obstacle by ROBOT_MAX_RADIUS_MM ≈ 220mm)

### 4.6 INCOMPLETE: Lidar Runtime Control

**File**: `lidar_logic.py` lines 677-691
```python
def start_lidar_thread():
    lidar_obj = RPLidar(PORT, baudrate=BAUDRATE, timeout=TIMEOUT)
```

**Issue**: PORT hardcoded to `/dev/ttyUSB0` (Linux-only)
**No auto-detection** or fallback for:
- Windows (COM ports)
- Different USB devices
- Multiple LiDARs

**Fix needed**: Implement USB device detection (as used in `utils/robot_context.py` line 126)

---

## 5. SPECIFIC CALIBRATION VALUES TO TEST

### 5.1 Field Geometry (terrain_jeu.py)

```python
FIELD_WIDTH_MM = 3000     # Line 8
FIELD_HEIGHT_MM = 2000    # Line 9
ROBOT_RADIUS_MM = 120     # Line 10

# Obstacles (Example):
"Grenier"       at (600, 1550)  1800×450mm  # Line 32
"Nid Adverse"   at (0, 1550)    600×450mm   # Line 33
```

**BEACON POSITIONS** (Line 67-70):
```python
Beacon 1: (3050.0, 1950.0)   # Top-right corner (outside field!)
Beacon 2: (3050.0, 50.0)     # Bottom-right corner (outside field!)
Beacon 3: (-50.0, 1000.0)    # Left side midpoint (outside field!)
```

**⚠️ WARNING**: All 3 beacons are 50mm OUTSIDE the field boundaries
- This requires wide LiDAR range (calculated: POSE_MAX_DIST_MM ≈ 4800mm)
- Beacons visible only from center of field

**TEST**: Verify beacons can be seen from typical robot positions

### 5.2 Match Timing

```python
TEMPS_MATCH_SECONDES = 90.0    # main.py line 15 - Fixed
Update frequency: 20 Hz (50ms sleep) # main.py line 59
Match check: if temps_ecoule >= 90.0 # main.py line 47
```

**CONCERN**: CPU load might cause frame drops - verify actual Hz
**TEST**: Log timestamps for 90s, check variance

### 5.3 GPIO Ports (Not Yet Implemented)

**Tirette (Start button)**: robot.py line 74
- Currently: `time.sleep(3)` placeholder
- Should read: GPIO pin (pin number NOT specified!)
- **ACTION**: Define which GPIO pin on Raspberry Pi

**Arm Feedback** (if available):
- Not currently read
- Would need GPIO or analog input

---

## 6. SUMMARY TABLE: READINESS BY COMPONENT

| System | Completeness | Calibration | Testing | Blockers |
|--------|--------------|-------------|---------|----------|
| **Localization (LiDAR)** | 95% | 🟡 PARTIAL | 🔴 NEEDED | Beacon positions need field validation |
| **Navigation (Pathfinder)** | 90% | 🟡 PARTIAL | 🔴 NEEDED | Dynamic lookahead not implemented |
| **Odometry (Teensy)** | 100% | ✓ DONE | 🟡 BASIC | Callback working |
| **Strategy/State Machine** | 60% | 🔴 MISSING | 🔴 NEEDED | Real strategy not defined; import time missing |
| **Actuators** | 40% | ❌ NONE | ❌ NOT STARTED | Class not instantiated; no feedback |
| **Emergency Stop** | 80% | ✓ DONE | 🟡 BASIC | Arm cleanup commented out |
| **Hardware Detection** | 50% | 🔴 MISSING | 🔴 NEEDED | USB port hardcoded |

---

## 7. CRITICAL PATH TO MATCH-READY

### BLOCKING ISSUES (Must Fix Before Match)
1. ❌ **strategy_strat_manager.py**: Add `import time` (Line 1)
2. ❌ **robot.py**: Implement `self.actionneurs` initialization
3. ❌ **robot.py**: Read actual GPIO for tirette, don't sleep
4. ❌ **strategy_strat_manager.py**: Replace placeholder strategy with real actions
5. ❌ **lidar_logic.py**: Make PORT auto-detectable or configurable

### HIGH-PRIORITY (Should Fix Before Match)
6. 🟡 **robot.py line 127**: Implement dynamic lookahead
7. 🟡 **robot.py line 143**: Add actuator feedback validation
8. 🟡 **lidar_logic.py**: Validate beacon detection thresholds (80mm RMS)
9. 🟡 **robot.py line 108**: Add opponent to obstacles list
10. 🟡 **pathfinder.py**: Test 160mm safety margin in real arena

### MEDIUM-PRIORITY (Nice to Have)
11. 🔵 Add USB auto-detection for LiDAR port
12. 🔵 Implement arm cleanup on shutdown
13. 🔵 Add velocity-based parameter scaling
14. 🔵 Implement path timeout recovery

---

## 8. LINE-BY-LINE REFERENCE

### Quick Lookup: All Hardcoded Values
```
robot.py
  Line 44:  time.sleep(3)               ← Tirette placeholder
  Line 111: distance_restante < 50.0    ← Goal tolerance (mm)
  Line 127: lookahead_index = min(4,...) ← Hardcoded lookahead

pathfinder.py  
  Line 3:   GRID_SIZE = 50              ← Cell size (mm)
  Line 4:   MARGIN = 40                 ← Safety margin (mm)

terrain_jeu.py
  Line 8-10: FIELD_WIDTH/HEIGHT/RADIUS  ← 3000×2000 mm, robot 120mm
  Line 67-70: Beacon positions          ← All outside field!

lidar_logic.py
  Line 50-55: LiDAR connection params   ← PORT hardcoded!
  Line 74-100: Thresholds               ← Many calibration values
  
lidar.py
  Line 82:  alpha_xy = 0.35             ← Fusion weight
  Line 46:  opponent_timeout_s = 0.70   ← Opponent validity

main.py
  Line 15:  TEMPS_MATCH_SECONDES = 90.0 ← Match duration
  Line 59:  time.sleep(0.05)            ← 20 Hz update rate

strategy_strat_manager.py
  Line 1-5: Missing import time         ← BREAKS CODE!
  Line 23-31: Placeholder strategy       ← Example waypoints only
```

---

## APPENDIX: BEACON SYMMETRY LOGIC

Both BLUE and YELLOW teams get correct beacon positions via symmetry:

```python
# terrain_jeu.py line 66-70 (BeaconLayout)
if team_color.upper() == "YELLOW":
    x = FIELD_WIDTH_MM - x  # Horizontal flip
```

**Example**: 
- BLUE Beacon 1 at (3050, 1950) → outside top-right
- YELLOW Beacon 1 at (3000 - 3050, 1950) = (-50, 1950) → outside top-left ✓

Symmetry correctly applied ✓

---

**END OF ANALYSIS**

*Generated for CDR 2026 - Pre-Match Validation*
*All line numbers refer to workspace files as of April 18, 2026*
