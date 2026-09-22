"""Headless 2D simulator for the high level code.

Simulates the world and the sensors, never the firmware: no Teensy, no EKF, no
steppers, no actuator physics, no 3D and no physics engine. What runs inside is
the production code of robot1.rasp, unchanged; only the backends differ.

Entry point: python -m robot1.sim2d.run  (from the repository root)
Design, lots and exit criteria: doc_ref/PLAN_REFONTE_HAUT_NIVEAU_2027.md, 14.

This file holds a docstring and nothing else, on purpose. Importing anything
here would make `import robot1.sim2d` drag the whole simulator in, exactly what
the empty __init__ of robot1/rasp/ avoids for the robot.
"""
