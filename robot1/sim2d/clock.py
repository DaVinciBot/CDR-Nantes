"""Simulated time sources.

Both satisfy robot1.rasp.clock.Clock, so the match loop cannot tell them from
the wall clock. VirtualClock runs as fast as the CPU allows, which is what a
100 s match in under a second requires; ScaledClock paces simulated time
against real time, which is what watching the run in Rerun requires.
"""

import time


class VirtualClock:
    """Simulated time. sleep() advances a counter instead of blocking.

    Nothing here ever waits, so a run costs exactly what it computes.
    """

    def __init__(self, start: float = 0.0) -> None:
        """Build the clock.

        Args:
            start: initial simulated time, in seconds.
        """
        self.t = start

    def now(self) -> float:
        """Return the current simulated time, in seconds."""
        return self.t

    def sleep(self, seconds: float) -> None:
        """Advance simulated time, without waiting.

        Args:
            seconds: duration to skip. Negative values are ignored.
        """
        if seconds > 0.0:
            self.t += seconds


class ScaledClock:
    """Simulated time paced against the wall clock.

    speed=1.0 plays in real time, speed=10.0 ten times faster. The wait is
    computed from the accumulated simulated time rather than per call, so a
    slow tick is absorbed by the next ones instead of shifting the whole run.
    """

    def __init__(self, speed: float = 1.0, start: float = 0.0) -> None:
        """Build the clock.

        Args:
            speed: simulated seconds per real second. Must be > 0.
            start: initial simulated time, in seconds.

        Raises:
            ValueError: if speed is not strictly positive.
        """
        if speed <= 0.0:
            raise ValueError(f"speed must be > 0, got {speed}")
        self.speed = speed
        self.t = start
        self._t0_sim = start
        self._t0_wall = time.time()

    def now(self) -> float:
        """Return the current simulated time, in seconds."""
        return self.t

    def sleep(self, seconds: float) -> None:
        """Advance simulated time, then wait for real time to catch up.

        Args:
            seconds: simulated duration to advance. Negative values are
                ignored.
        """
        if seconds > 0.0:
            self.t += seconds
        target_wall = self._t0_wall + (self.t - self._t0_sim) / self.speed
        remaining = target_wall - time.time()
        if remaining > 0.0:
            time.sleep(remaining)
