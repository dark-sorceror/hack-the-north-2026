"""Physical e-stop and vacuum relay (bridge/gpio.py).

Every scenario runs twice when gpiozero is importable: once against a fake pin
object with the same two members EstopButton uses, and once against real
gpiozero devices on gpiozero's MockFactory, driving the mock pins the way the
wiring would. Without gpiozero (the laptop venv) the MockFactory half skips.
"""

import importlib.util
import logging
import subprocess
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from retriever.bridge.drivers import CompositeDriver
from retriever.bridge.fake_driver import FakeTankDriver
from retriever.bridge.gpio import EstopButton, VacuumOverlay, VacuumRelay
from retriever.bridge.protocol import (
    Act,
    Estop,
)
from retriever.bridge.server import BridgeCore, HardwareDriver

SRC = Path(__file__).resolve().parents[1] / "src"
HAVE_GPIOZERO = importlib.util.find_spec("gpiozero") is not None
logging.getLogger("retriever.bridge").setLevel(logging.CRITICAL)


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


class FakeInput:
    """gpiozero DigitalInputDevice(pull_up=True), as far as EstopButton cares:
    value is 1 while the switch contacts are closed (pin pulled to GND)."""

    def __init__(self, closed):
        self.value = 1 if closed else 0
        self.closed_calls = 0

    def close(self):
        self.closed_calls += 1


class FakeOutput:
    def __init__(self):
        self.value = 1          # deliberately ON: VacuumRelay must switch it off
        self.is_closed = False

    def on(self):
        self.value = 1

    def off(self):
        self.value = 0

    def close(self):
        self.is_closed = True


# ---------------------------------------------------------------- the wiring, two ways


class FakePinWiring:
    def relay(self, active_high=True):
        dev = FakeOutput()
        relay = VacuumRelay(device=dev, active_high=active_high)
        return relay, (lambda: bool(dev.value)), dev


class MockFactoryWiring:
    def __init__(self):
        from gpiozero.pins.mock import MockFactory

        self.factory = MockFactory()

    def relay(self, active_high=True, pin=27):
        relay = VacuumRelay(pin, active_high=active_high, pin_factory=self.factory)
        p = self.factory.pin(pin)
        # "energised" in terms of the pin's electrical level
        return relay, (lambda: p.state == active_high), relay.device


# ---------------------------------------------------------------- scenarios


class EstopScenarios:
    """Run against both wirings by the subclasses below."""

    def wiring(self):
        raise NotImplementedError

    def setUp(self):
        self.clock = Clock()
        self.w = self.wiring()

    def test_relay_is_off_at_start_follows_set_vacuum_and_is_off_after_stop(self):
        for active_high in (True, False):
            with self.subTest(active_high=active_high):
                relay, energised, _ = self.w.relay(active_high=active_high)
                self.assertFalse(energised())
                relay.set_vacuum(True)
                self.assertTrue(energised())
                self.assertTrue(relay.is_on)
                relay.stop()
                self.assertFalse(energised())
                relay.set_vacuum(True)
                relay.close()                               # off first, then released
                self.assertFalse(relay.is_on)

    def test_relay_behind_the_bridge(self):
        """Act.vacuum drives the relay; e-stop leaves it as it is (HardwareDriver
        contract: letting go is its own decision); close() turns it off."""
        relay, energised, _ = self.w.relay(active_high=True)
        drv = VacuumOverlay(FakeTankDriver(clock=self.clock), relay)
        self.assertIsInstance(drv, HardwareDriver)
        core = BridgeCore(drv, self.clock())
        core.connected(self.clock())
        core.handle(Act(seq=1, vacuum=True), self.clock())
        self.assertTrue(energised())
        self.assertTrue(drv.driver.vacuum)
        core.handle(Estop(reason="test"), self.clock())
        self.assertTrue(energised())
        relay.close()
        self.assertFalse(energised())


class TestEstopWithFakePin(EstopScenarios, unittest.TestCase):
    def wiring(self):
        return FakePinWiring()


@unittest.skipUnless(HAVE_GPIOZERO, "gpiozero not importable here; the fake-pin half covers it")
class TestEstopWithGpiozeroMockFactory(EstopScenarios, unittest.TestCase):
    def wiring(self):
        return MockFactoryWiring()


# ---------------------------------------------------------------- the parts


class TestEstopInput(unittest.TestCase):
    def test_debounce_uses_its_own_clock(self):
        clock = Clock()
        dev = FakeInput(closed=True)
        btn = EstopButton(device=dev, debounce_s=0.03, clock=clock)
        dev.value = 0
        self.assertFalse(btn())                           # first sighting
        clock.t += 0.02
        self.assertFalse(btn())
        clock.t += 0.02
        self.assertTrue(btn())
        dev.value = 1
        self.assertFalse(btn())                           # release is immediate

    def test_constructors_need_a_pin_or_a_device(self):
        with self.assertRaises(ValueError):
            EstopButton()
        with self.assertRaises(ValueError):
            VacuumRelay(active_high=True)

    def test_relay_polarity_must_be_chosen(self):
        with self.assertRaises(TypeError):
            VacuumRelay(device=FakeOutput())               # no active_high: refused

    def test_composite_driver_routes_vacuum_to_the_relay(self):
        class Base:
            counts_per_rev = 4096

            def set_wheels(self, left, right):
                pass

            def read_ticks(self):
                return (0, 0)

            def battery(self):
                return None

            def stop(self):
                pass

        dev = FakeOutput()
        relay = VacuumRelay(device=dev, active_high=True)
        drv = CompositeDriver(Base(), {}, vacuum=relay)
        drv.set_vacuum(True)
        self.assertEqual(dev.value, 1)
        drv.stop()
        self.assertEqual(dev.value, 1)                    # stop() leaves the vacuum
        relay.close()
        self.assertEqual((dev.value, dev.is_closed), (0, True))


class TestGpioModuleIsLazy(unittest.TestCase):
    def test_importing_it_pulls_in_nothing_outside_the_stdlib(self):
        code = (
            f"import sys; sys.path.insert(0, {str(SRC)!r})\n"
            "before = set(sys.modules)\n"
            "import retriever.bridge.gpio\n"
            "new = {m.split('.')[0] for m in set(sys.modules) - before}\n"
            "print(sorted(new - set(sys.stdlib_module_names) - {'retriever'}))\n"
        )
        out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True,
                             timeout=30)
        self.assertEqual(out.returncode, 0, out.stderr)
        self.assertEqual(out.stdout.strip(), "[]")


if __name__ == "__main__":
    unittest.main()
