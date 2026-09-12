"""High-level Python stack of robot 1, running on the Raspberry Pi 5.

Public modules, imported as ``from robot1.rasp.<module> import ...``::

    comm        Teensy USB link; reads ROBOT_MODE. Insertion point for sim2d.
    nav         A* path finding on a 50 mm grid, obstacle inflation.
    vision      LiDAR: opponent detection, SVD beacon relocalisation, debug GUI.
    strategy    action sequence state machine (hardcoded stub for now).
    telemetry   Rerun publishing, best-effort.
    world       table geometry, fixed obstacles, beacons, BLUE/YELLOW symmetry.
    app         match loop and state machine (class Robot); main.py is the entry point.

Deliberately holds no code and re-exports nothing: ``app`` imports gpiozero at module
level, so any re-export here would make ``import robot1.rasp`` fail off the Raspberry Pi.

``tools/`` and ``tests/`` sit next to these modules but are outside the package (no
__init__.py): they are launch namespaces, not robot dependencies, and are run with
``python -m`` from robot1/rasp/.
"""
