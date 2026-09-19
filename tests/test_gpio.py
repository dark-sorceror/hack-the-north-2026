"""Physical e-stop and vacuum relay (bridge/gpio.py) and the server's estop_input hook.

Every scenario runs twice when gpiozero is importable: once against a fake pin
object with the same two members EstopButton uses, and once against real
gpiozero devices on gpiozero's MockFactory, driving the mock pins the way the
wiring would. Without gpiozero (the laptop venv) the MockFactory half skips.
"""

import importlib.util
import logging
import socket
import subprocess
import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from retriever.bridge.drivers import CompositeDriver
from retriever.bridge.fake_driver import FakeTankDriver
from retriever.bridge.gpio import EstopButton, VacuumOverlay, VacuumRelay
from retriever.bridge.protocol import (
    SERVER_MESSAGES,
    Act,
    ClearEstop,
    Estop,
    Hello,
    State,
    decode,
    encode,
)
from retriever.bridge.server import BridgeCore, BridgeServer, HardwareDriver, ServerThread

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
    def button(self, normally_closed=True, **kw):
        dev = FakeInput(closed=normally_closed)          # at rest: NC closed, NO open
        btn = EstopButton(device=dev, normally_closed=normally_closed, **kw)
        # Pressing opens NC contacts / closes NO contacts. A cut wire == open.
        press = lambda: setattr(dev, "value", 0 if normally_closed else 1)  # noqa: E731
        release = lambda: setattr(dev, "value", 1 if normally_closed else 0)  # noqa: E731
        return btn, press, release

    def relay(self, active_high=True):
        dev = FakeOutput()
        relay = VacuumRelay(device=dev, active_high=active_high)
        return relay, (lambda: bool(dev.value)), dev


class MockFactoryWiring:
    def __init__(self):
        from gpiozero.pins.mock import MockFactory

        self.factory = MockFactory()

    def button(self, normally_closed=True, pin=17, **kw):
        btn = EstopButton(pin, normally_closed=normally_closed, pin_factory=self.factory, **kw)
        p = self.factory.pin(pin)
        # Switch between the pin and GND, internal pull-up: closed contacts
        # drive the pin low, open contacts let the pull-up take it high.
        close_contacts, open_contacts = p.drive_low, p.drive_high
        if normally_closed:
            close_contacts()                               # NC at rest: closed
            return btn, open_contacts, close_contacts
        return btn, close_contacts, open_contacts

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

    def make(self, normally_closed=True, debounce_s=0.02, pressed_at_start=False):
        btn, self.press, self.release = self.w.button(
            normally_closed=normally_closed, debounce_s=debounce_s, clock=self.clock)
        self.addCleanup(btn.close)
        if pressed_at_start:
            self.press()
        self.drv = FakeTankDriver(clock=self.clock)
        self.core = BridgeCore(self.drv, self.clock(), timeout_s=0.3, motion_timeout_s=0.5,
                               estop_input=btn)
        self.core.connected(self.clock())
        return btn

    def advance(self, seconds, every=0.02, msg=None):
        for _ in range(int(round(seconds / every))):
            self.clock.t += every
            if msg is not None:
                self.core.handle(msg, self.clock())
            self.core.tick(self.clock())

    def drive(self, seq=1):
        self.core.handle(Act(seq=seq, base_vx=0.3), self.clock())

    def test_press_latches_stops_motors_and_release_does_not_clear(self):
        self.make()
        self.drive()
        self.advance(0.1, msg=Act(seq=1, base_vx=0.3))
        self.assertNotEqual(self.drv.wheel_speeds, (0.0, 0.0))
        stops = self.drv.stop_count

        self.press()
        self.advance(0.06, msg=Act(seq=2, base_vx=0.3))    # laptop still streaming
        self.assertTrue(self.core.estop)
        self.assertIn("physical", self.core.estop_reason)
        self.assertEqual(self.drv.wheel_speeds, (0.0, 0.0))
        self.assertEqual(self.drv.stop_count, stops + 1)  # latched once, not every tick

        self.release()
        self.advance(1.0, msg=Act(seq=3, base_vx=0.3))
        self.assertTrue(self.core.estop)                   # releasing clears nothing
        self.assertEqual(self.drv.wheel_speeds, (0.0, 0.0))
        self.assertTrue(self.core.snapshot(self.clock()).estop)

        self.assertIsNone(self.core.handle(ClearEstop(), self.clock()))
        self.assertFalse(self.core.estop)                  # the laptop clears it
        self.drive(seq=4)
        self.assertNotEqual(self.drv.wheel_speeds, (0.0, 0.0))

    def test_clear_is_refused_while_the_button_is_down(self):
        self.make()
        self.press()
        self.advance(0.1)
        refused = self.core.handle(ClearEstop(), self.clock())
        self.assertIn("still pressed", refused)
        self.assertTrue(self.core.estop)
        self.drive(seq=5)
        self.assertEqual(self.drv.wheel_speeds, (0.0, 0.0))

    def test_stops_the_motors_with_no_laptop_at_all(self):
        """The link is dead (never alive, here): the tick loop alone latches it."""
        btn, press, _ = self.w.button(debounce_s=0.02, clock=self.clock)
        self.addCleanup(btn.close)
        drv = FakeTankDriver(clock=self.clock)
        core = BridgeCore(drv, self.clock(), estop_input=btn)
        stops = drv.stop_count
        press()
        for _ in range(5):
            self.clock.t += 0.02
            core.tick(self.clock())
        self.assertTrue(core.estop)
        self.assertGreater(drv.stop_count, stops)
        self.assertTrue(core.watchdog_tripped)

    def test_press_mid_drive_after_the_laptop_died(self):
        """Laptop gone silent 100 ms ago, watchdog not yet tripped: the button
        still stops the base before the watchdog would have."""
        self.make()
        self.drive()
        self.advance(0.1)                                  # silence, < 300 ms
        self.assertNotEqual(self.drv.wheel_speeds, (0.0, 0.0))
        self.press()
        self.advance(0.06)
        self.assertTrue(self.core.estop)
        self.assertFalse(self.core.watchdog_tripped)
        self.assertEqual(self.drv.wheel_speeds, (0.0, 0.0))

    def test_a_glitch_shorter_than_the_debounce_is_ignored(self):
        self.make(debounce_s=0.05)
        self.drive()
        self.press()
        self.advance(0.02, msg=Act(seq=1, base_vx=0.3))
        self.release()
        self.advance(0.2, msg=Act(seq=1, base_vx=0.3))
        self.assertFalse(self.core.estop)
        self.assertNotEqual(self.drv.wheel_speeds, (0.0, 0.0))

    def test_held_at_startup_latches_on_the_first_ticks(self):
        btn = self.make(pressed_at_start=True)
        self.advance(0.06)
        self.assertTrue(self.core.estop)
        self.assertTrue(btn())
        self.core.handle(Act(seq=1, base_vx=0.3), self.clock())
        self.assertEqual(self.drv.wheel_speeds, (0.0, 0.0))

    def test_normally_open_wiring(self):
        btn = self.make(normally_closed=False)
        self.advance(0.1)
        self.assertFalse(self.core.estop)
        self.press()                                       # NO: pressing closes contacts
        self.advance(0.06)
        self.assertTrue(self.core.estop)
        self.release()
        self.advance(0.06)
        self.assertFalse(btn())
        self.assertTrue(self.core.estop)

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
    def test_an_unreadable_input_counts_as_pressed(self):
        clock = Clock()
        drv = FakeTankDriver(clock=clock)

        def broken():
            raise OSError("gpiochip gone")

        core = BridgeCore(drv, clock(), estop_input=broken)
        core.tick(clock())
        self.assertTrue(core.estop)
        self.assertIn("unreadable", core.estop_reason)
        self.assertIsNotNone(core.handle(ClearEstop(), clock()))
        self.assertTrue(core.estop)

    def test_no_input_changes_nothing(self):
        clock = Clock()
        core = BridgeCore(FakeTankDriver(clock=clock), clock())
        self.assertFalse(core.poll_estop_input(clock()))

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


# ---------------------------------------------------------------- over TCP


class TestEstopOverTheWire(unittest.TestCase):
    def serve(self, button):
        self.drv = FakeTankDriver()
        self.server = BridgeServer(self.drv, "127.0.0.1", 0, timeout_ms=5000,
                                   motion_timeout_ms=5000, state_hz=100.0, estop_input=button)
        self.st = ServerThread(self.server)
        self.port = self.st.start()
        self.addCleanup(self.st.stop)

    def states(self, sock):
        buf = b""
        while True:
            chunk = sock.recv(65536)
            if not chunk:
                return
            buf += chunk
            while b"\n" in buf:
                raw, _, buf = buf.partition(b"\n")
                yield decode(raw, SERVER_MESSAGES)

    def until(self, it, pred, timeout=2.0):
        deadline = time.monotonic() + timeout
        for m in it:
            if pred(m):
                return m
            if time.monotonic() > deadline:
                break
        raise AssertionError("not seen on the wire")

    def test_button_press_reaches_the_motors_and_the_laptop(self):
        dev = FakeInput(closed=True)
        self.serve(EstopButton(device=dev, debounce_s=0.02))
        sock = socket.create_connection(("127.0.0.1", self.port), timeout=2.0)
        self.addCleanup(sock.close)
        it = self.states(sock)
        self.assertIsInstance(next(it), Hello)
        sock.sendall(encode(Act(seq=1, base_vx=0.2)))
        self.until(it, lambda m: isinstance(m, State) and m.seq == 1)
        self.assertNotEqual(self.st.call(lambda: self.drv.wheel_speeds), (0.0, 0.0))

        dev.value = 0                                       # press: NC contacts open
        self.until(it, lambda m: isinstance(m, State) and m.estop)
        self.assertEqual(self.st.call(lambda: self.drv.wheel_speeds), (0.0, 0.0))
        dev.value = 1                                       # release
        time.sleep(0.1)
        self.assertTrue(self.st.call(lambda: self.server.core.estop))

    def test_button_works_with_no_client_connected(self):
        dev = FakeInput(closed=True)
        self.serve(EstopButton(device=dev, debounce_s=0.02))
        stops = self.st.call(lambda: self.drv.stop_count)
        dev.value = 0
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and not self.st.call(lambda: self.server.core.estop):
            time.sleep(0.01)
        self.assertTrue(self.st.call(lambda: self.server.core.estop))
        self.assertGreater(self.st.call(lambda: self.drv.stop_count), stops)


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
