"""Robot 1 (three-wheel holonomic) - package root.

Layout::

    robot1/
        rasp/               Raspberry Pi 5 high-level stack (Python)
        sim2d/              headless kinematic simulator      (planned)
        replay/             sensor-log replay harness         (planned)
        teensy_moteur/      firmware, NOT a Python package
        teensy_capteur/     firmware, NOT a Python package
        teensy_actuator/    firmware, NOT a Python package

Deliberately holds no code and imports nothing. Importing ``robot1.sim2d`` must not
drag in ``robot1.rasp`` and its hardware dependencies (gpiozero, pyserial, rplidar):
a headless simulator that only runs on the robot is worthless. Anything re-exported
here would be loaded by every sibling package, so keep this file empty.
"""
