# Robot1 Raspberry Pi - Python

**Status**: ✅ MOSTLY READY (April 20, 2026 — Critical bugs fixed)

**For complete codebase status**, see [doc/CODEBASE_ANALYSIS_2026_UNIFIED.md](doc/CODEBASE_ANALYSIS_2026_UNIFIED.md)

## Critical Fixes Applied (April 20)

- ✅ `test_program.py:L12` → Import fixed: `GestionnaireLidar` → `LidarInterface`
- ✅ `lidar_logic.py:L607` → beacon_ids type fixed: `List[str]` → `List[int]`
- ✅ `lidar_gui.py` → Dead code removed, cleaner architecture

## Structure

```text
robot1/rasp/
├── main.py                     # Entry point
├── robot.py                    # Main state machine
├── terrain_jeu.py              # Field geometry + beacons
├── pathfinder.py               # A* navigation
├── loader.py                   # Dynamic module loader
├── switch_mode.py              # Simulation ↔ Hardware toggle
├── config.json                 # USB config
├── lidar/                      # ✅ LiDAR subsystem (SEE README)
│   ├── lidar_logic.py          # SVD Umeyama pose calculation
│   ├── lidar_gui.py            # Matplotlib GUI visualization
│   ├── lidar_navigation_bridge.py  # Pathfinding integration
│   ├── main.py                 # GUI launcher
│   └── README.md               # Complete LiDAR documentation
├── strategy/                   # Strategy state machine
│   ├── strategy_actions.py     # Action types
│   └── strategy_strat_manager.py  # Orchestration
├── utils/                      # Utilities
├── rerun/                      # Rerun 3D visualization ✅
├── test/                       # Hardware tests
├── simu/                       # Webots simulation
└── doc/                        # Documentation
    ├── CODEBASE_ANALYSIS_2026_UNIFIED.md  # ← Single source of truth
    └── ODOMETRY_CORRECTION_IMPLEMENTATION.md
```

## LiDAR Module (Updated April 20)

**For complete module documentation**, see [lidar/README.md](lidar/README.md)

### Quick Start

```bash
cd robot1/rasp

# Launch GUI (debugging)
python -m lidar.main

# Rerun 3D visualization
python rerun/rerun_bridge.py --mode serve --with-lidar --port 9876
```

### Key Changes (April 20)

- ✅ Dead beacon detection code removed (~180 lines)
- ✅ Clear separation: lidar_logic calculates, lidar_gui displays
- ✅ beacon_ids type consistency: all `List[int]` now
- ✅ New README documenting full architecture

## Lancement module lidar GUI

```bash
cd robot1/rasp
python -m lidar.main
```

## Bascule simulation/hardware

```bash
cd robot1/rasp
python switch_mode.py simulation
python switch_mode.py hardware
```

## Documentation de reference

- Etat global Python: `doc/ETAT_DES_LIEUX_PYTHON_2026-04-18.md`
- Plan de reduction: `doc/PLAN_REDUCTION_RASP.md`

## Archives (cleanup)

Les fichiers retires de la surface active sont conserves ici:

- `robot1/archive_rasp/tests_lidar/`
- `robot1/archive_rasp/docs_obsoletes/`
- `robot1/archive_rasp/rerun_docs/`
- `robot1/archive_rasp/rerun_tools/`
