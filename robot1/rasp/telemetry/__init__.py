"""Telemetry: the run log, and the Rerun visualisation bridge.

The run log (logger, log_format, log_reader) writes to disk what went through
the link and what the robot decided. trial adds what makes a run a trial: its
type and its ground truth. They depend on nothing but the standard library and
usb_com.

Rerun does no fusion of its own. It receives values already computed by the
high level (app.py, nav, vision) and displays them.

Importing rerun_bridge pulls in the rerun SDK, which is optional on a dev
machine. Callers that tolerate its absence should guard the import:

    try:
        from robot1.rasp.telemetry import rerun_bridge
    except ImportError:
        rerun_bridge = None

The package itself is deliberately kept import-light, so that `import telemetry`
never drags the SDK in.
"""

__all__ = [
    "log_format", "log_reader", "logger", "rerun_bridge", "table_map", "trial",
]
