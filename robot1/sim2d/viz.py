"""Rerun view of a simulated run, true pose against estimated pose.

Best effort, exactly like telemetry/ is for the robot: if rerun is missing or
the viewer will not start, the simulation runs anyway. A visualisation that can
abort a run would make the headless campaigns of lot 5 fragile for no reason.

The table, the robot, the target and the trajectory are already published by
telemetry/rerun_bridge.py, which the match loop feeds on its own. All that is
added here is what only the simulator knows: the true pose, and the odometry
error it produces.
"""

import logging
import math

logger = logging.getLogger("SIM2D_VIZ")

C_TRUTH = [0, 200, 255, 255]     # cyan, the pose only the simulator knows
ARROW_MM = 260.0
Z_MM = 60.0

_enabled = False
_rr = None
_bridge = None


def init(spawn: bool = True) -> bool:
    """Start a Rerun session and log the static table.

    Args:
        spawn: open the local viewer. False only logs, for a headless run.

    Returns:
        bool: True if the visualisation is live.
    """
    global _enabled, _rr, _bridge
    try:
        import rerun as rr

        from robot1.rasp.telemetry import rerun_bridge
    except Exception as exc:                       # noqa: BLE001
        logger.warning("Rerun indisponible (%s) : simulation sans vue.", exc)
        return False

    try:
        rr.init("sim2d")
        if spawn:
            # The bridge owns the Rerun session on the robot, so it also owns
            # the knowledge of where the viewer actually lives.
            rerun_bridge.spawn_viewer()
        rr.send_blueprint(rerun_bridge.create_blueprint())
        rerun_bridge.log_static_map()
    except Exception as exc:                       # noqa: BLE001
        logger.warning("Rerun n'a pas demarre (%s) : simulation sans vue.", exc)
        return False

    _rr, _bridge, _enabled = rr, rerun_bridge, True
    logger.info("Vue Rerun active.")
    return True


def is_enabled() -> bool:
    """Return whether a Rerun session is live."""
    return _enabled


def publish(sim_time: float, sim_robot) -> None:
    """Publish one frame at the given simulated time.

    Args:
        sim_time: simulated time, in seconds. Drives the Rerun timeline, so
            scrubbing follows the match and not the wall clock.
        sim_robot: the SimulatedRobot, read for its true pose and its error.
    """
    if not _enabled:
        return
    try:
        _rr.set_time("sim_time", duration=sim_time)

        x, y, theta = sim_robot.true_pose()
        _rr.log("world/robot/truth/point", _rr.Points3D(
            positions=[[x, y, Z_MM]],
            colors=[C_TRUTH],
            radii=[80.0],
        ))
        _rr.log("world/robot/truth/arrow", _rr.Arrows3D(
            origins=[[x, y, Z_MM]],
            vectors=[[ARROW_MM * math.cos(theta),
                      ARROW_MM * math.sin(theta), 0.0]],
            colors=[C_TRUTH],
        ))

        _rr.log("data/sim2d/erreur_position_mm",
                _rr.Scalars(sim_robot.position_error_mm()))
        _rr.log("data/sim2d/erreur_cap_deg",
                _rr.Scalars(math.degrees(sim_robot.heading_error_rad())))
        _rr.log("data/sim2d/distance_parcourue_mm",
                _rr.Scalars(sim_robot.distance_travelled_mm))

        # The match loop has already filled the shared state (odometry, target,
        # trajectory, obstacles): flush it on the same simulated timestamp.
        _bridge.publish_once()
    except Exception as exc:                       # noqa: BLE001
        logger.debug("Publication Rerun ignoree : %s", exc)
