"""LiDAR perception: opponent detection and beacon-based pose correction.

Deliberately empty of any import: importing one layer must never drag the
others in. app.py only wants detection, and used to pull the 1000 lines of
localization plus a tkinter probe with it.

    detection.py     opponent detection (thread, clustering, smoothing).
                     The only layer the match loop (app.py) uses.
    localization.py  beacon extraction and SVD Umeyama pose correction.
                     FROZEN, see the module docstring and vision/README.md.
    lidar_config.py  serial link shared by both, read from config.json.
    gui.py           tkinter/matplotlib debug view  ->  python -m vision

Import the layer you need, never the package:

    from robot1.rasp.vision import detection
    from robot1.rasp.vision.localization import get_corrected_pose
"""
