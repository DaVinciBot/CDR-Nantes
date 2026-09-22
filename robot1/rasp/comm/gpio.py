"""GPIO access behind a single dispatch point.

gpiozero only installs on a Raspberry Pi, and app.py used to import it at
module level: `import robot1.rasp.app` failed on a development machine, which
made the whole match loop untestable off-robot. The import now happens inside
get_button(), and only in hardware mode.

Hardware behaviour is unchanged. Outside hardware mode the button is simulated
and says so in the log: a simulated start cord must never pass for a real one.
"""

import logging

from .context import HARDWARE_MODE, get_mode

logger = logging.getLogger("ROBOT_GPIO")

# Preset states for simulated pins, keyed by BCM pin number.
_fake_states: dict[int, bool] = {}


def set_fake_button(pin: int, pressed: bool) -> None:
    """Preset what a simulated button reports for one pin.

    Only meaningful outside hardware mode: this is how sim2d picks the team
    colour before building the Robot, instead of reading a physical switch.

    Args:
        pin: BCM pin number.
        pressed: state FakeButton will report through is_pressed.
    """
    _fake_states[pin] = pressed


class FakeButton:
    """Stand-in for gpiozero.Button outside hardware mode.

    Implements only what app.py uses: is_pressed, wait_for_release, close.
    """

    def __init__(self, pin: int, *, pressed: bool = True) -> None:
        """Build a simulated button.

        Args:
            pin: BCM pin number, kept for logging only.
            pressed: initial state reported by is_pressed.
        """
        self.pin = pin
        self.closed = False
        self._pressed = pressed

    @property
    def is_pressed(self) -> bool:
        """Return the simulated pin state."""
        return self._pressed

    def wait_for_release(self) -> None:
        """Return at once: there is no cord to pull in simulation."""
        logger.warning(
            "GPIO simule (pin %d) : depart immediat, pas de tirette.",
            self.pin,
        )
        self._pressed = False

    def close(self) -> None:
        """Release the simulated pin."""
        self.closed = True


def get_button(pin: int, *, pull_up: bool = True):
    """Return the button object for one pin, according to ROBOT_MODE.

    Args:
        pin: BCM pin number.
        pull_up: forwarded to gpiozero in hardware mode.

    Returns:
        gpiozero.Button in hardware mode, FakeButton otherwise.

    Raises:
        ImportError: in hardware mode on a machine without gpiozero. The
            failure is loud on purpose: a robot that cannot read its start
            cord must not look like a robot that is ready to play.
    """
    if get_mode() == HARDWARE_MODE:
        from gpiozero import Button

        return Button(pin, pull_up=pull_up)

    pressed = _fake_states.get(pin, True)
    logger.info("GPIO simule : pin %d, pressed=%s.", pin, pressed)
    return FakeButton(pin, pressed=pressed)
