"""Estimators of the board, replayed on a PC from the inputs logged by LOG_TICK.

    python -m robot1.replay.estimators.complementary <run folder>   # non-regression
    python -m robot1.replay.estimators.ekf <run folder>...          # offline EKF
    python -m robot1.replay.estimators.ekf_tune ...                 # Q / R sweep

The estimators are not rewritten in Python: the C++ of the board runs on the
PC through a small tool, tools/est_replay of teensy_moteur, built by the
"replay" environment of PlatformIO (native.py drives it).
"""
