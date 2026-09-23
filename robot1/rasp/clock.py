"""Time source for the match loop.

Production runs on the wall clock. sim2d substitutes a virtual clock, and that
is what makes two things possible: simulating a 100 s match in under a second,
and replaying a run exactly. Both break as soon as a module reads time.time()
on its own, because the speed gating of vision/detection.py then compares
timestamps it did not produce.

Hardware behaviour is unchanged: RealClock is a thin wrapper over the time
module and it is the default everywhere.
"""

import time
from typing import Protocol, runtime_checkable


@runtime_checkable
class Clock(Protocol):
    """Minimal time contract shared by the real and the simulated loops."""

    def now(self) -> float:
        """Return the current time, in seconds."""

    def sleep(self, seconds: float) -> None:
        """Wait for a duration expressed in this clock's own time base."""


class RealClock:
    """Wall clock, backed by the time module."""

    def now(self) -> float:
        """Return the wall clock time, in seconds since the epoch."""
        return time.time()

    def sleep(self, seconds: float) -> None:
        """Block the calling thread for the given number of seconds."""
        time.sleep(seconds)


# Shared default, so that a module-level fallback costs no allocation and every
# consumer that was not given a clock demonstrably uses the same one.
DEFAULT_CLOCK = RealClock()
