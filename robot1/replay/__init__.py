"""Offline tools for the run logs: clock alignment, loading, Rerun export, report.

Run on a PC, never on the robot. They read a run folder written by
robot1.rasp.telemetry.logger and leave its files untouched. Each tool is run
as a module, from anywhere:

    python -m robot1.replay.align    <run folder>
    python -m robot1.replay.report   <run folder>
    python -m robot1.replay.to_rerun <run folder>

pandas and the Rerun SDK come with the "analysis" extra of pyproject.toml.
"""
