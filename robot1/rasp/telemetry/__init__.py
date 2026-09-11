"""Telemetry: Rerun visualisation bridge.

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

__all__ = ["rerun_bridge", "table_map"]
