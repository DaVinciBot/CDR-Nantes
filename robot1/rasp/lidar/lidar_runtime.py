"""Backward-compatible shim.

Deprecated: import lidar_logic instead of lidar_runtime.
"""

try:
    from .lidar_logic import *  # noqa: F401,F403
except ImportError:
    from lidar_logic import *  # noqa: F401,F403
