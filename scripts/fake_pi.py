#!/usr/bin/env python3
"""The Pi's bridge process: serve the laptop until SIGINT or SIGTERM, then stop the motors.

The same entry point runs on the Pi and, with `--driver fake`, on a laptop with no robot, so
everything above the bridge is exercised over a real socket long before a motor turns. It
needs nothing installed (it puts ../src on sys.path itself), because the Pi has nothing
installed. SIGTERM is handled like Ctrl-C, so `systemctl stop` stops the motors too, not
just the process. One log line at start says where it listens, which driver, and every
timeout, which is the first thing to check when the robot stops "for no reason".

A real e-stop button and vacuum relay work with either driver, so the fake tank
can run on a Pi 4 with real GPIO (BCM numbers):

    python3 scripts/fake_pi.py --estop-pin 17 --vacuum-pin 27
    python3 scripts/fake_pi.py --estop-pin 17 --vacuum-pin 27 --vacuum-active-low
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import signal
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from retriever.bridge.drivers import build_real_driver  # noqa: E402
from retriever.bridge.fake_driver import FakeTankDriver  # noqa: E402
from retriever.bridge.gpio import EstopButton, VacuumOverlay, VacuumRelay  # noqa: E402
from retriever.bridge.protocol import DEFAULT_PORT  # noqa: E402
from retriever.bridge.server import BridgeServer, HardwareDriver  # noqa: E402
from retriever.navigation.kinematics import TankGeometry  # noqa: E402

logger = logging.getLogger("fake_pi")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    """The command line; defaults are the robot's own numbers."""
    defaults = TankGeometry()
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--host", default="", help='address to listen on ("" = all)')
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="0 picks a free port")
    parser.add_argument("--driver", choices=("fake", "real"), default="fake")
    parser.add_argument("--wheel-port", default="/dev/ttyUSB0")
    parser.add_argument("--arm-port", default=None)
    parser.add_argument("--estop-pin", default=None,
                        help="BCM GPIO of a physical e-stop switch to GND (e.g. 17)")
    parser.add_argument(
        "--estop-normally-open", action="store_true",
        help="the e-stop is normally-open (default: normally-closed, fail-safe)")
    parser.add_argument("--vacuum-pin", default=None,
                        help="BCM GPIO driving the vacuum relay, or an LED (e.g. 27)")
    parser.add_argument("--vacuum-active-low", action="store_true",
                        help="the relay switches ON when its input is LOW (most relay modules)")
    parser.add_argument("--timeout-ms", type=float, default=300.0, help="watchdog")
    parser.add_argument("--motion-timeout-ms", type=float, default=500.0, help="motion deadman")
    parser.add_argument("--state-hz", type=float, default=50.0)
    parser.add_argument("--max-wheel-mps", type=float, default=0.8)
    parser.add_argument("--wheel-radius", type=float, default=defaults.wheel_radius_m)
    parser.add_argument("--track-width", type=float, default=defaults.track_width_m)
    parser.add_argument("--scrub-factor", type=float, default=defaults.scrub_factor)
    parser.add_argument("-v", "--verbose", action="store_true", help="debug logging")
    return parser.parse_args(argv)


async def serve(server: BridgeServer, where: str, settings: str) -> None:
    """Run `server` until SIGINT or SIGTERM, then close it, which stops the motors."""
    loop = asyncio.get_running_loop()
    port = await server.start()
    stop_requested = asyncio.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop_requested.set)

    async def close_when_asked() -> None:
        await stop_requested.wait()
        logger.info("stopping: motors to zero, then exit")
        await server.close()

    closer = loop.create_task(close_when_asked())
    logger.info("bridge listening on %s port %d; %s", where, port, settings)
    try:
        await server.serve_forever()
    finally:
        closer.cancel()


def main(argv: list[str] | None = None) -> int:
    """Build the driver and the server from the command line, and serve."""
    args = parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    button = relay = None
    if args.estop_pin is not None or args.vacuum_pin is not None:
        try:
            if args.estop_pin is not None:
                button = EstopButton(_pin(args.estop_pin),
                                     normally_closed=not args.estop_normally_open)
            if args.vacuum_pin is not None:
                relay = VacuumRelay(_pin(args.vacuum_pin),
                                    active_high=not args.vacuum_active_low)
        except Exception as exc:  # ImportError, GPIOPinInUse, BadPinFactory, ...
            print(f"\n  GPIO setup failed: {exc}\n", file=sys.stderr)
            _close(button)
            return 2
        if button is not None:
            pressed = "PRESSED - latched until the button is up and the laptop clears it" \
                if button.tripped_now() else "ok"
            logger.info(
                "e-stop on GPIO%s (%s): %s", args.estop_pin,
                "normally-open" if args.estop_normally_open else "normally-closed", pressed)

    try:
        geo = TankGeometry(args.wheel_radius, args.track_width, args.scrub_factor)
        server = BridgeServer(
            build_driver(args, geo, relay),
            args.host,
            args.port,
            geo=geo,
            timeout_ms=args.timeout_ms,
            motion_timeout_ms=args.motion_timeout_ms,
            state_hz=args.state_hz,
            max_wheel_mps=args.max_wheel_mps,
            estop_input=button,
        )
    except (NotImplementedError, ValueError) as exc:
        print(f"fake_pi: {exc}", file=sys.stderr)
        _close(relay, button)
        return 2
    settings = (
        f"{args.driver} driver, watchdog {args.timeout_ms:g} ms, "
        f"motion deadman {args.motion_timeout_ms:g} ms, state {args.state_hz:g} Hz, "
        f"max wheel {args.max_wheel_mps:g} m/s"
    )
    try:
        asyncio.run(serve(server, args.host or "every address", settings))
    except OSError as exc:  # most often: the port is already taken by another bridge
        print(f"fake_pi: {exc}", file=sys.stderr)
        return 1
    finally:
        _close(relay, button)   # relay off, pins released
    return 0


def build_driver(
    args: argparse.Namespace, geo: TankGeometry, relay: VacuumRelay | None = None
) -> HardwareDriver:
    """The fake driver, or the real hardware (which raises NotImplementedError for now)."""
    if args.driver == "real":
        return build_real_driver(args.wheel_port, args.arm_port, geo)
    driver: HardwareDriver = FakeTankDriver(geo)
    if relay is not None:
        driver = VacuumOverlay(driver, relay)
    return driver


def _close(*parts: object) -> None:
    for part in parts:
        if part is not None:
            part.close()


def _pin(spec: str) -> int | str:
    """"17" -> 17 (BCM); anything else ("GPIO17", "BOARD11") goes to gpiozero as is."""
    return int(spec) if spec.isdigit() else spec


if __name__ == "__main__":
    sys.exit(main())
