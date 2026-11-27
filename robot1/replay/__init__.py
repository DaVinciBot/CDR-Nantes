"""Offline tools for the run logs: clock alignment, loading, Rerun export, report,
replay of the estimators.

Run on a PC, never on the robot. They read a run folder written by
robot1.rasp.telemetry.logger and leave its files untouched. Each tool is run
as a module, from anywhere:

    python -m robot1.replay.align    <run folder>
    python -m robot1.replay.report   <run folder>
    python -m robot1.replay.to_rerun <run folder>
    python -m robot1.replay.estimators.complementary <run folder>
    python -m robot1.replay.estimators.ekf <run folder>...
    python -m robot1.replay.estimators.ekf_tune --tune ... --validate ... --grid ...

pandas and the Rerun SDK come with the "analysis" extra of pyproject.toml.
"""
