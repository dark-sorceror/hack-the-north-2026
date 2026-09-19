"""Physical e-stop button and vacuum relay on the Pi's 40-pin header.

gpiozero, not RPi.GPIO: RPi.GPIO does not run on the Pi 5, whose header GPIO
sits behind the RP1 chip. gpiozero's default backend, lgpio, talks to the
kernel's gpiochip device and works on the Pi 4 and the Pi 5 alike.

gpiozero is imported lazily, inside the constructors, so this module imports
on a laptop with nothing installed. Tests pass `device=` (a fake with the same
two or three members) or a gpiozero MockFactory as `pin_factory=`.

Pin numbers are BCM GPIO numbers (GPIO17), not physical header positions
(pin 11). gpiozero also accepts "BOARD11" or "GPIO17" strings.
"""

from __future__ import annotations

import logging
import time
from typing import Any, Callable

log = logging.getLogger(__name__)


def _gpiozero() -> Any:
    try:
        import gpiozero
    except ImportError as exc:
        raise ImportError(
            "gpiozero is not importable. On Raspberry Pi OS it is preinstalled for "
            "the system python3; in a venv, create it with --system-site-packages."
        ) from exc
    return gpiozero


class EstopButton:
    """An e-stop switch between a GPIO pin and GND, with the internal pull-up.

    normally_closed=True (recommended): the contacts are closed at rest, so the
    pin reads LOW; pressing opens them and the pull-up takes the pin HIGH. A
    cut wire, a pulled connector or a dead switch also reads HIGH, i.e. as a
    press. With a normally-open button the same faults read as "not pressed"
    and the e-stop silently stops working.

    Debounce: the tripped level must hold for `debounce_s` before it counts, so
    a spike on a long wire does not stop the robot. The default (15 ms) means
    two consecutive polls at the bridge's 50 Hz tick; a real press lasts far
    longer. Contact bounce on press is harmless (the first stable level
    latches); release is reported at once, since releasing clears nothing.
    """

    def __init__(
        self,
        pin: int | str | None = None,
        *,
        normally_closed: bool = True,
        debounce_s: float = 0.015,
        pin_factory: Any = None,
        device: Any = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if device is None:
            if pin is None:
                raise ValueError("EstopButton needs a pin (or a device)")
            # pull_up=True: switch to GND; value == 1 while the contacts are closed.
            device = _gpiozero().DigitalInputDevice(pin, pull_up=True, pin_factory=pin_factory)
        self.device = device
        self.normally_closed = normally_closed
        self.debounce_s = debounce_s
        self.clock = clock
        self._tripped_since: float | None = None

    def contacts_closed(self) -> bool:
        return bool(self.device.value)

    def tripped_now(self) -> bool:
        """The raw, undebounced reading."""
        closed = self.contacts_closed()
        return not closed if self.normally_closed else closed

    def __call__(self) -> bool:
        """Debounced: True while the button is pressed (or the wire is cut)."""
        now = self.clock()
        if not self.tripped_now():
            self._tripped_since = None
            return False
        if self._tripped_since is None:
            self._tripped_since = now
        return now - self._tripped_since >= self.debounce_s

    def close(self) -> None:
        self.device.close()


class VacuumRelay:
    """The vacuum pump's relay (or an LED standing in for it).

    OFF when constructed, OFF on stop() and close(). set_vacuum() is the
    VacuumDriver interface CompositeDriver expects.

    active_high has no default on purpose: most hobby relay modules switch ON
    when their input is pulled LOW, and a wrong guess runs the pump from boot.
    If the relay clicks on at startup, the polarity is wrong.

    Note what the bridge's own stop() does NOT do: e-stop, watchdog and
    disconnect leave the vacuum as it is, per HardwareDriver's contract —
    dropping a held object is its own decision. The relay goes off at process
    start, on explicit stop()/close(), and at a clean exit.
    """

    def __init__(
        self,
        pin: int | str | None = None,
        *,
        active_high: bool,
        pin_factory: Any = None,
        device: Any = None,
    ) -> None:
        if device is None:
            if pin is None:
                raise ValueError("VacuumRelay needs a pin (or a device)")
            device = _gpiozero().DigitalOutputDevice(
                pin, active_high=active_high, initial_value=False, pin_factory=pin_factory
            )
        self.device = device
        self.active_high = active_high
        self.is_on = False
        self.device.off()

    def set_vacuum(self, on: bool) -> None:
        if on:
            self.device.on()
        else:
            self.device.off()
        self.is_on = bool(on)

    def stop(self) -> None:
        self.set_vacuum(False)

    def close(self) -> None:
        """Off, then release the pin. gpiozero's close leaves the pin a floating
        input, so an active-high relay needs a pull-down to stay off after
        exit."""
        try:
            self.stop()
        finally:
            self.device.close()


class VacuumOverlay:
    """Any HardwareDriver, with set_vacuum also driving a real relay.

    For running FakeTankDriver on a real Pi with a real relay (scripts/fake_pi.py
    --vacuum-pin). The real stack uses CompositeDriver(vacuum=relay) instead.
    stop() is the inner driver's alone: the vacuum is left as it is.
    """

    def __init__(self, driver: Any, relay: VacuumRelay) -> None:
        self.driver = driver
        self.relay = relay

    @property
    def counts_per_rev(self) -> int:
        return self.driver.counts_per_rev

    def set_wheels(self, left_mps: float, right_mps: float) -> None:
        self.driver.set_wheels(left_mps, right_mps)

    def set_joints(self, targets: dict[str, float]) -> None:
        self.driver.set_joints(targets)

    def set_gripper(self, position: float) -> None:
        self.driver.set_gripper(position)

    def set_vacuum(self, on: bool) -> None:
        self.relay.set_vacuum(on)
        self.driver.set_vacuum(on)

    def read_state(self) -> dict[str, Any]:
        return self.driver.read_state()

    def stop(self) -> None:
        self.driver.stop()
