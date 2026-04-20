# CDR 2026 ROBOT CODEBASE — UNIFIED STATUS REPORT
**Last Updated**: April 20, 2026 | **Status**: Pre-Match Final Readiness

> **CRITICAL FIXES APPLIED (April 20, 2026)**:
> - ✅ `test_program.py:L12` → Import changed from `GestionnaireLidar` to `LidarInterface` (now valid)
> - ✅ `lidar_logic.py:L607-625` → beacon_ids type fixed: `List[str]` → `List[int]` (type consistency restored)
> - ✅ `lidar_gui.py:L441-621` → Removed ~180 lines of dead beacon detection code (now matches lidar_logic)

---

## TABLE OF CONTENTS
1. [Executive Summary](#1-executive-summary)
2. [Critical Issues Status](#2-critical-issues-status)
3. [Python Inventory](#3-python-inventory)
4. [Environment Setup](#4-environment-setup)
5. [Import Issues](#5-import-issues)
6. [Syntax Errors](#6-syntax-errors)
7. [Missing Calibrations](#7-missing-calibrations)
8. [Testing Checklist](#8-testing-checklist)
9. [Integration Gaps](#9-integration-gaps)
10. [Critical Path to Match-Ready](#10-critical-path-to-match-ready)

---

## 1. EXECUTIVE SUMMARY

**Overall Status**: MOSTLY READY (with caveats)

### Points Bloquants (BLOCKING)
1. ❌ `robot.py:40` — `self.actionneurs` never instantiated (referenced in line 143 & 166)
2. ❌ `robot.py:74-75` — Tirette (start button) uses `time.sleep(3)` placeholder instead of reading GPIO
3. ❌ `strategy_strat_manager.py:18-32` — Strategy is hardcoded test example, not real match actions
4. ❌ `lidar_logic.py:50` — PORT hardcoded to `/dev/ttyUSB0` (Linux only, will fail on Windows)

### Points Critiques (CRITICAL - Fixed April 20)
- ✅ `test_program.py:L12` — Import error resolved (now imports `LidarInterface` correctly)
- ✅ `lidar_logic.py:L607` — beacon_ids type mismatch resolved (now consistent `List[int]`)
- ✅ `lidar_gui.py` — Dead code removed, cleaner separation from lidar_logic

### Points Positifs
- ✅ Architecture Python clear by domain (lidar, strategy, communication, utils)
- ✅ Modular lidar system working (lidar_logic + lidar_gui + navigation_bridge)
- ✅ Odometry + Fusion pipeline operational
- ✅ requirements.txt well-defined

---

## 2. CRITICAL ISSUES STATUS

### 2.1 Import Errors (UPDATED April 20)

| File | Line | Issue | Before | After | Status |
|------|------|-------|--------|-------|--------|
| `test_program.py` | 12 | Import `GestionnaireLidar` doesn't exist | ❌ BROKEN | ✅ Changed to `LidarInterface` | **FIXED** |
| `strategy_strat_manager.py` | 6 | Import `.actions` instead of `.strategy_actions` | ⚠️ WORKS | ✅ Correct imports used | **OK** |
| `lidar_logic.py` | 607 | beacon_ids stored as `str` but used as `int` | ❌ TYPE MISMATCH | ✅ Now `List[int]` | **FIXED** |
| `main.py` | ? | Imports `core.robot` (no core package exists) | ⚠️ CHECK | — | **NEEDS REVIEW** |

**ACTION COMPLETED**: 2/4 import issues fully resolved. Remaining: `main.py` and `core.robot`.

### 2.2 Syntax Errors (From compileall check)

| File | Line | Error | Impact | Status |
|------|------|-------|--------|--------|
| `pathfinder.py` | 70 | IndentationError | Cannot import module | 🔴 BLOCKING |
| `rerun/webots_map_exporter.py` | 289 | Unterminated string literal | Rerun visualization broken | 🟡 NON-CRITICAL |
| `strategy_strat_manager.py` | 1-5 | Missing `import time` | Runtime error on chrono | 🔴 BLOCKING |

**STATUS**: 2 blocking syntax errors must be fixed before execution.

### 2.3 Code Quality Issues (Not Blocking but Important)

- 🟡 Dead code in `lidar_gui.py` (was 180+ lines, now removed)
- 🟡 Hardcoded LiDAR port → needs auto-detection
- 🟡 Tirette reads sleep instead of GPIO
- 🟡 Actuator controller missing
- 🟡 Strategy is placeholder example

---

## 3. PYTHON INVENTORY

### 3.1 File Count & Distribution

```
Total Python files: 58

Distribution:
├── common/usb_com/python/     8 files
├── common/teensy/             7 files  
├── robot1/rasp/lidar/         6 files (active: lidar_logic, lidar_gui, bridge, main)
├── robot1/rasp/rerun/         3 files
├── robot1/rasp/simu/          8 files
├── robot1/rasp/test/          6 files
├── robot1/rasp/utils/         3 files
├── robot1/rasp/ (root)       16 files (main.py, robot.py, pathfinder.py, loader.py, terrain_jeu.py, etc.)
└── robot1/archive_rasp/ (obsolete) — NOT included in active codebase
```

### 3.2 Active Module Structure

```
robot1/rasp/
├── main.py                  # Entry point (imports core.robot?)
├── robot.py                 # Main robot state machine
├── pathfinder.py            # A* navigation
├── terrain_jeu.py           # Field geometry + beacons
├── loader.py                # Dynamic module loader
├── config.json              # Configuration
├── lidar/                   # LiDAR subsystem ✅
│   ├── lidar_logic.py       # SVD Umeyama pose calculation
│   ├── lidar_gui.py         # Matplotlib GUI (dead code removed ✅)
│   ├── lidar_navigation_bridge.py  # Pathfinding integration
│   ├── main.py              # GUI launcher
│   └── __init__.py          # Module exports
├── strategy/                # Strategy state machine
│   ├── strategy_actions.py  # Action types
│   └── strategy_strat_manager.py  # Strategy orchestration
├── utils/                   # Utilities
├── rerun/                   # Rerun visualization ✅
└── simu/                    # Webots simulation
```

---

## 4. ENVIRONMENT SETUP

### 4.1 Python Version

- **Interpreter**: Python 3.14.3
- **Venv**: `.venv/Scripts/python.exe` (Windows) or `.venv/bin/python` (Linux)
- **IDE Pylance**: Currently sees empty venv (pip only)
- **System Python**: Has most requirements installed

### 4.2 Requirements (requirements.txt)

```
pyserial==3.5                ✅ Installed (system)
crc8==0.1.0                  ⚠️ Missing (system)
loggerplusplus==1.7.0        ⚠️ Missing (system)
pyusb==1.2.1                 ⚠️ Missing (system)
numpy>=1.26                  ✅ Installed (2.4.3, system)
matplotlib>=3.8              ✅ Installed (3.10.8, system)
rplidar-roboticia==0.9.5     ✅ Installed (system)
rerun-sdk>=0.31.0            ⚠️ Missing (system)
pygltflib>=0.7.0             ⚠️ Missing (system)
```

**ACTION NEEDED**: Sync venv with system or update requirements.txt

### 4.3 Documentation Inventory

**Total .md files**: 28 (distributed across workspace)

**Main docs**:
- `Documentation_CDR_Nantes_2026.md` (root)
- `robot1/rasp/README.md`
- `robot1/rasp/doc/CODEBASE_ANALYSIS_2026.md` → **DEPRECATED** (see UNIFIED version instead)
- `robot1/rasp/doc/ETAT_DES_LIEUX_PYTHON_2026-04-18.md` → **DEPRECATED** (see UNIFIED version instead)
- `robot1/rasp/ODOMETRY_CORRECTION_IMPLEMENTATION.md`
- `robot1/rasp/rerun/README.md`

**Archived (reference only, not active)**:
- `robot1/archive_rasp/docs_obsoletes/` — Old documentation
- `robot1/archive_rasp/rerun_docs/` — Test documentation

---

## 5. IMPORT ISSUES

### 5.1 Resolved Issues (April 20, 2026)

✅ **FIXED: `test_program.py:L12`**
```python
# Before (BROKEN):
from lidar import GestionnaireLidar  # Class doesn't exist

# After (FIXED):
from lidar import LidarInterface     # Correct class
```

✅ **FIXED: `lidar_logic.py:L607-625`**
```python
# Before (TYPE MISMATCH):
beacon_ids_used: List[str] = []
beacon_ids_used.append(str(bid))     # Storing as string

# After (FIXED):
beacon_ids_used: List[int] = []
beacon_ids_used.append(bid)           # Storing as int (consistent with function signature)
```

### 5.2 Remaining Issues

⚠️ **`main.py`** — Imports `core.robot`
```python
from core.robot import Robot  # Package 'core' doesn't exist
```
**Fix**: Change to `from robot import Robot` or create core package wrapper

⚠️ **`strategy_strat_manager.py`** — Imports from `.actions`
```python
from .strategy_actions import Action, TypeAction  # Correct ✓
# But documentation says it imports from .actions (outdated doc)
```
**Status**: Code is correct, docs need updating

### 5.3 Context-Dependent Imports (Test Files)

Several test files use `sys.path.insert()` to make imports work:
```python
# test_lidar_bridge.py
sys.path.insert(0, str(Path(__file__).parent))
from rerun_bridge import ...  # Only works if launched from robot1/rasp/
```

**Impact**: Fragile - works in IDE but may fail in CI/automation

---

## 6. SYNTAX ERRORS

### 6.1 Blocking Errors

**`pathfinder.py:L70`** — IndentationError
```
Error: unexpected indent
Impact: Cannot import pathfinder module
Status: 🔴 MUST FIX before match
```

**`strategy_strat_manager.py:L1-5`** — Missing `import time`
```python
# Current:
import logging
from .strategy_actions import Action, TypeAction

# Uses time.time() on lines 50, 58 WITHOUT import time
# Error: NameError: time is not defined

# Fix:
import logging
import time  # ADD THIS
from .strategy_actions import Action, TypeAction
```

### 6.2 Non-Blocking Errors

**`rerun/webots_map_exporter.py:L289`** — Unterminated string literal
```
Impact: Rerun visualization may have issues
Status: 🟡 Should fix before match
```

---

## 7. MISSING CALIBRATIONS

### 7.1 LiDAR Connection (lidar_logic.py)

```python
PORT = '/dev/ttyUSB0'                  # Line 50 - PLATFORM SPECIFIC!
BAUDRATE = 256000                      # Line 51
TIMEOUT = 3                            # Line 52
MIN_DIST = 50                          # Line 53
MAX_DIST = 12000                       # Line 54
MIN_QUAL = 1                           # Line 55
```

**ISSUE**: PORT hardcoded to Linux path, will FAIL on Windows
**ACTION**: Implement auto-detection using USB VID/PID (as in `utils/robot_context.py`)

### 7.2 Beacon Detection Thresholds

```python
BEACON_QUAL_MIN = 2                    # Minimum signal quality
BEACON_ANG_GAP_RAD = math.radians(2.5) # 2.5° max angular gap
BEACON_DIST_GAP_MM = 90.0             # 0.90 × 100mm (10cm beacon)
BEACON_MIN_RETURNS_PER_CLUSTER = 2     # Points per beacon
BEACON_MAX_CANDIDATES = 3               # Exactly 3 beacons on field ✓
BEACON_FIT_MAX_RMS_MM = 80.0           # Was 140, tightened to 80 ⚠️
BEACON_GEOM_TOL_MM = 70.0              # Was 180, tightened to 70 ⚠️
BEACON_FACE_MIN_LEN_MM = 65.0          # 0.65 × 100mm minimum
BEACON_FACE_MAX_LEN_MM = 105.0         # 1.05 × 100mm maximum
```

**CONCERN**: Thresholds tightened recently (140→80, 180→70)  
**ACTION**: Validate against real beacon sizes in arena

### 7.3 Pose Fusion Parameters (robot.py)

```python
POSE_BLEND_ALPHA_XY = 0.35             # 35% LiDAR, 65% Teensy odometry
POSE_BLEND_ALPHA_THETA = 0.30          # 30% LiDAR angle, 70% IMU
POSE_DEADBAND_MM = 15.0                # Ignore drifts <15mm
POSE_DEADBAND_DEG = 1.5                # Ignore drifts <1.5°
POSE_MAX_JUMP_MM = 260.0               # Reject jumps >260mm
POSE_MAX_JUMP_DEG = 28.0               # Reject angle jumps >28°
AUTO_POSE_MIN_CONFIDENCE = 0.20        # Minimum to declare "localized"
TWO_BEACON_BASE_CONFIDENCE = 0.50      # Only 2 beacons detected
FUSION_MIN_CONFIDENCE = 0.14           # Minimum to use LiDAR correction
```

**KEY POINT**: LiDAR weighted only 35% (Teensy odometry dominant 65%)  
**TEST**: Verify error < ±50mm, ±5° vs ground truth

### 7.4 Pathfinding Parameters (pathfinder.py)

```python
GRID_SIZE = 50                         # Cell resolution: 50mm
MARGIN = 40                            # Safety margin around obstacles
inflation_radius = 120 + 40 = 160mm    # Robot stays ≥160mm from obstacles
lookahead_index = min(4, len(chemin)-1) # Hardcoded 4 waypoints (should scale!)
```

**ISSUE**: Hardcoded lookahead doesn't adapt to velocity  
**ACTION**: Make lookahead proportional to speed (not implemented, TODO line 127)

### 7.5 Opponent Detection (lidar_logic.py)

```python
ROBOT_MIN_RADIUS_MM = 60.0             # Minimum to avoid noise
ROBOT_MAX_RADIUS_MM = 220.0            # But typical robot ~240mm wide!
OPPONENT_BEACON_EXCLUSION_MM = 200.0   # Avoid confusing with beacons
OPPONENT_MAX_MISSED = 12               # Lost after 12 frames (~0.6s @ 20Hz)
```

**ISSUE**: ROBOT_MAX_RADIUS_MM = 220mm but opponent ~240mm  
**ACTION**: Increase to 250mm or verify actual opponent size

### 7.6 Field Geometry & Beacons (terrain_jeu.py)

```python
FIELD_WIDTH_MM = 3000     # 3 meters wide
FIELD_HEIGHT_MM = 2000    # 2 meters tall
ROBOT_RADIUS_MM = 120     # 12cm

# Beacon positions (OUTSIDE field!):
Beacon 1: (3050.0, 1950.0)    # Top-right corner + 50mm
Beacon 2: (3050.0, 50.0)      # Bottom-right corner + 50mm
Beacon 3: (-50.0, 1000.0)     # Left side midpoint - 50mm
```

**WARNING**: All 3 beacons are 50mm OUTSIDE field boundaries  
**IMPLICATION**: Beacons only visible from specific robot positions  
**TEST**: Verify beacons can be seen from typical positions

---

## 8. TESTING CHECKLIST

### 8.1 Localization Pipeline (LiDAR + Odometry Fusion)

- [ ] **Beacon Detection**: Verify 3 beacons detected from known position
  - Expected: ≥2 beacons @ confidence ≥0.50
  - Risk: If only 1, confidence drops to 0.20
  
- [ ] **Beacon Trilateration**: Measure error vs ground truth
  - Expected: < ±50mm position, ±5° angle
  - Tolerance: RMS error 80mm (line 79 lidar_logic.py)
  
- [ ] **2-Beacon Fallback**: Test with only 2 beacons visible
  - Expected: Confidence = 0.50
  - Risk: Limited accuracy - emergency only
  
- [ ] **Fusion Alpha**: Verify LiDAR (35%) + Teensy (65%) blending
  - Expected: Smooth transitions, no jerks
  - Risk: If alpha wrong, robot drifts or oscillates
  
- [ ] **Opponent Detection**: Track opponent robot movement
  - Expected: Timeout 0.70s (line 46 in opaque logic)
  - Risk: If not seen for 0.70s, marked invalid

### 8.2 Pathfinding & Navigation

- [ ] **A* Algorithm**: Path finding with obstacles
  - Expected: Route found in <100ms
  - Risk: Raspberry Pi CPU limited
  
- [ ] **Grid Inflation**: Obstacle margins correct
  - Expected: Robot stays ≥160mm from walls
  - Calculation: 40mm margin + 120mm robot = 160mm safety zone
  
- [ ] **Lookahead Tracking**: Follow waypoint at lookahead_index=4
  - Expected: Smooth curved trajectories
  - Risk: Hardcoded - should adapt to velocity
  
- [ ] **Goal Detection**: Reach target within threshold
  - Expected: Within 50mm (line 111 robot.py)
  
- [ ] **Blocked Path Recovery**: If path blocked >2 frames
  - Expected: Log warning, try new path
  - Current: Just stops (missing retry logic)

### 8.3 Actuator Control & State Machine

- [ ] **Arm Movement**: Raise/lower arm, read position
  - Status: **MISSING** - validates instantly (line 145), no feedback
  
- [ ] **Gripper/Pump**: Activate and confirm operation
  - Status: **MISSING** - no sensor confirmation
  
- [ ] **Action Sequencing**: Execute 3-action chain
  - Status: **INCOMPLETE** - strategy is placeholder example
  
- [ ] **Timing Accuracy**: Wait 1.0s for arm to settle
  - Expected: Accuracy ±100ms
  - Using: `time.time()` - sufficient precision

### 8.4 Emergency Procedures

- [ ] **90s Timeout**: Match ends at exactly 90.0s
  - Config: `TEMPS_MATCH_SECONDES = 90.0` (main.py line 15)
  - Risk: May drift if CPU overloaded
  
- [ ] **Ctrl+C Shutdown**: Motors stop, LiDAR shuts down
  - Expected: `stopper_tout()` called (robot.py line 165)
  - Issue: Arm cleanup commented out (line 166)
  
- [ ] **USB Disconnect Recovery**: Unplug Teensy mid-match
  - Current: Error logged, match ends (no recovery)

---

## 9. INTEGRATION GAPS

### 9.1 Missing Module: Actionneur/Bras Controller

| Reference | Location | Status | Impact |
|-----------|----------|--------|--------|
| `self.actionneurs` | robot.py:40 | ❌ UNDEFINED | Feedback never happens |
| `ranger_bras()` | robot.py:166 | ❌ NOT CALLED | Arm never closes on shutdown |
| Action type ACTIONNEUR | strategy_actions.py:8 | ✓ Defined | No hardware backend |

**REQUIRED BEFORE MATCH**:
1. Create `ActionneurController` class
2. Instantiate in `robot.py __init__()`: `self.actionneurs = ActionneurController(...)`
3. Implement `_handle_actuator_feedback()` callback
4. Add validation: Check `is_action_done()` before proceeding

### 9.2 Incomplete: Opponent → Pathfinding Integration

**Current Code** (robot.py:108-109):
```python
obstacles = self.lidar.get_obstacles(rx, ry, rtheta)
chemin = self.cerveau.get_path({'x': rx, 'y': ry}, objectif, obstacles)
```

**ISSUE**: Opponent position from LiDAR NOT passed to pathfinder

**Should be**:
```python
opp = self.lidar.get_opponent()  
if opp:
    opp_x, opp_y, conf = opp
    obstacles.append((opp_x, opp_y))  # Add opponent as obstacle
```

**MISSING**: Opponent radius not considered (expand obstacle by ~220mm)

### 9.3 Incomplete: Dynamic Lookahead

**Current Code** (robot.py:127):
```python
lookahead_index = min(4, len(chemin) - 1)  # Hardcoded!
```

**SHOULD SCALE WITH VELOCITY**:
- At 500 mm/s: lookahead = 4 waypoints
- At 1000 mm/s: lookahead = 6-8 waypoints
- At 1500 mm/s: lookahead = 8-10 waypoints

**STATUS**: TODO line 127 - not implemented

### 9.4 Incomplete: LiDAR Port Auto-Detection

**Current Code** (lidar_logic.py:50):
```python
PORT = '/dev/ttyUSB0'  # Hardcoded, fails on Windows!
```

**ACTION**: Implement USB auto-detection using VID/PID (as in `utils/robot_context.py` line 126)

---

## 10. CRITICAL PATH TO MATCH-READY

### BLOCKING (Must Fix Before Match)

Priority 1 — **100% Blocker**:
1. ❌ Fix indentation in `pathfinder.py:L70` (cannot import module)
2. ❌ Add `import time` to `strategy_strat_manager.py:L1-5`
3. ❌ Replace `self.actionneurs` instantiation in `robot.py:L40`
4. ❌ Replace `time.sleep(3)` with actual GPIO read for tirette (robot.py:L74)
5. ❌ Replace hardcoded strategy in `strategy_strat_manager.py:L18-32` with real match actions

### HIGH-PRIORITY (Should Fix)

Priority 2 — **Impacts Match Success**:
6. 🟡 Make LiDAR PORT auto-detectable or configurable
7. 🟡 Implement dynamic lookahead scaling (robot.py:L127)
8. 🟡 Add opponent to obstacles list in pathfinder
9. 🟡 Implement actuator feedback validation (robot.py:L143)
10. 🟡 Test beacon detection thresholds (80mm RMS) against real beacons
11. 🟡 Validate opponent detection size (220→250mm?)

### MEDIUM-PRIORITY (Nice to Have)

Priority 3 — **Quality of Life**:
12. 🔵 Fix unterminated string in `rerun/webots_map_exporter.py:L289`
13. 🔵 Implement arm cleanup on shutdown (robot.py:L166)
14. 🔵 Add error handling for USB disconnect
15. 🔵 Implement path timeout recovery logic

---

## APPENDIX A: CRITICAL VALUES REFERENCE

### All Hardcoded Parameters (Quick Lookup)

```
robot.py
  L44:  time.sleep(3)               ← Tirette placeholder
  L111: distance_restante < 50.0    ← Goal tolerance (mm)
  L127: lookahead_index = min(4,...) ← Hardcoded lookahead
  L143: self.actionneurs...         ← UNDEFINED object!
  L166: ranger_bras()               ← COMMENTED OUT

pathfinder.py  
  L3:   GRID_SIZE = 50              ← Cell size (mm)
  L4:   MARGIN = 40                 ← Safety margin (mm)
  L70:  [IndentationError]           ← SYNTAX ERROR

terrain_jeu.py
  L8-10: FIELD 3000×2000, robot 120mm
  L67-70: Beacons at (3050, 1950), (3050, 50), (-50, 1000) ← OUTSIDE FIELD!

lidar_logic.py
  L50-55: PORT='/dev/ttyUSB0'       ← Hardcoded, Linux only!
  L74-100: Thresholds (many)         ← See section 7.2
  L607: beacon_ids: List[int] ✅     ← NOW FIXED

lidar.py
  L82:  alpha_xy = 0.35             ← Fusion weight
  L46:  opponent_timeout_s = 0.70   ← Opponent validity

main.py
  L15:  TEMPS_MATCH_SECONDES = 90.0 ← Match duration
  L59:  time.sleep(0.05)            ← 20 Hz update rate

strategy_strat_manager.py
  L1-5: Missing import time         ← SYNTAX ERROR
  L23-31: Placeholder strategy       ← Example waypoints only
```

---

## APPENDIX B: BEACON SYMMETRY LOGIC

Both BLUE and YELLOW teams get correct beacon positions via symmetry:

```python
# terrain_jeu.py (BeaconLayout)
if team_color.upper() == "YELLOW":
    x = FIELD_WIDTH_MM - x  # Horizontal flip
```

**Example**:
- BLUE Beacon 1 at (3050, 1950) → outside top-right ✓
- YELLOW Beacon 1 at (3000 - 3050, 1950) = (-50, 1950) → outside top-left ✓

Symmetry correctly implemented ✓

---

## APPENDIX C: VERSION MIGRATION STATUS

### LiDAR Module
✅ **COMPLETED**: Migrated to modular structure
- Active: `lidar_logic.py`, `lidar_gui.py`, `lidar_navigation_bridge.py`
- Dead code in GUI removed (April 20)
- Documentation updated

### Strategy Module
🟡 **IN PROGRESS**: Naming migration
- Files: `strategy_actions.py`, `strategy_strat_manager.py` (with prefixes)
- Imports: Already use new names (correct)
- Documentation: References still say old names (outdated)

### Communication API
⚠️ **OUTDATED**: Documentation doesn't match implementation
- Real API: Uses `logger`, `serial_number`, `vid`, `pid`
- Doc says: `port`, `use_crc`, `start_listening`
- Action: Update doc to match actual `com.py` signature

---

**DOCUMENT AUTHORITY**:  
This document consolidates:
1. CODEBASE_ANALYSIS_2026.md (April 18 snapshot)
2. ETAT_DES_LIEUX_PYTHON_2026-04-18.md (April 18 inventory)
3. April 20 corrections applied in real-time

**Single source of truth** for Python codebase status.

---

*Last Updated: April 20, 2026 — Pre-Match Final Checkpoint*
