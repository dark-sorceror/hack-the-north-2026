"""Is the Pi's supply healthy? Ask the Pi's own PMIC.

WHAT THIS IS NOT. It is not the battery's charge. Nothing on this robot can
measure the pack: the DDSM115 protocol carries current, rpm, position,
temperature and error bits but no bus voltage (drivers.py), and the Pi has no
ADC. A percentage needs a divider or an INA219/ADS1115 on I2C; until one is
fitted, claiming a percentage would be making one up.

WHAT IT IS. The 5 V rail going into the Pi, and the firmware's undervoltage
flags. That rail is what actually fails: a sagging pack drags the converter's
output down, the PMIC cuts USB power, and the peripherals vanish. On the night
this was written a flat pack (10 V on a 12 V system) took the robot down
repeatedly -- LED green, then red about ten seconds in as the lidar and camera
drew -- and it was diagnosed with a multimeter, because nothing on the
dashboard showed it. get_throttled would have said so an hour earlier: it warns
while the robot still works, and it remembers (bit 16) that it dipped at all.

    throttled bit 0    undervoltage RIGHT NOW (below about 4.63 V)
    throttled bit 16   undervoltage has happened since boot

Off a Pi (a laptop sim, a dev box) vcgencmd does not exist: everything reads
None and the dashboard leaves the chip out. Stdlib only, like the rest of the
Pi side.
"""

from __future__ import annotations

import logging
import subprocess
import threading
import time

LOG = logging.getLogger("power")

UNDERVOLTAGE_NOW = 0x1
UNDERVOLTAGE_EVER = 0x10000

# The input rail, by firmware preference. Older Pi 5 firmware reports no
# EXT5V_V; HDMI_V hangs off the same 5 V input and tracks it within a few mV.
RAIL_NAMES = ("EXT5V_V", "HDMI_V")


def _run(args: list[str], timeout: float = 2.0) -> str | None:
    try:
        out = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.SubprocessError):
        return None
    return out.stdout if out.returncode == 0 else None


def read_throttled() -> int | None:
    """The firmware's throttle/undervoltage word, or None off a Pi."""
    text = _run(["vcgencmd", "get_throttled"])
    if not text or "=" not in text:
        return None
    try:
        return int(text.strip().split("=", 1)[1], 0)
    except ValueError:
        return None


def read_rail_volts() -> float | None:
    """Volts on the 5 V input rail, or None if the firmware won't say."""
    text = _run(["vcgencmd", "pmic_read_adc"])
    if not text:
        return None
    found: dict[str, float] = {}
    for line in text.splitlines():
        # e.g. "      HDMI_V volt(23)=4.91512000V"
        name, _, rest = line.strip().partition(" ")
        if "volt(" not in rest or "=" not in rest:
            continue
        try:
            found[name] = float(rest.split("=", 1)[1].rstrip("V"))
        except ValueError:
            continue
    for want in RAIL_NAMES:
        if want in found:
            return found[want]
    return None


class PowerMonitor:
    """Samples the PMIC on a thread. Cheap: vcgencmd twice every period_s."""

    def __init__(self, period_s: float = 2.0,
                 clock=time.monotonic, start: bool = True) -> None:
        self.period_s = period_s
        self.clock = clock
        self.volts: float | None = None
        self.undervoltage: bool | None = None       # right now
        self.dipped: bool = False                   # at any point since boot
        self.available = False                      # is this even a Pi
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        if start:
            self.start()

    def start(self) -> None:
        if self._thread is not None:
            return
        self.sample()                               # one now, so the first state has it
        self._thread = threading.Thread(target=self._loop, name="power", daemon=True)
        self._thread.start()

    def sample(self) -> None:
        word = read_throttled()
        if word is None:
            self.available = False
            self.undervoltage = None
            self.volts = None
            return
        was = self.undervoltage
        self.available = True
        self.undervoltage = bool(word & UNDERVOLTAGE_NOW)
        self.dipped = self.dipped or bool(word & (UNDERVOLTAGE_NOW | UNDERVOLTAGE_EVER))
        self.volts = read_rail_volts()
        if self.undervoltage and not was:
            LOG.warning("UNDERVOLTAGE: the 5 V rail has dropped%s. The battery is sagging; "
                        "the Pi will reset if it gets worse",
                        "" if self.volts is None else f" to {self.volts:.2f} V")

    def _loop(self) -> None:
        while not self._stop.wait(self.period_s):
            try:
                self.sample()
            except Exception:                       # a monitor must never stop the bridge
                LOG.debug("power sample failed", exc_info=True)

    def close(self) -> None:
        self._stop.set()
        t = self._thread
        if t is not None:
            t.join(timeout=1.0)
            self._thread = None
