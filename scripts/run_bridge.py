#!/usr/bin/env python3
"""The Pi's bridge process: serve the laptop until SIGINT or SIGTERM, then stop the motors.

The same entry point runs on the Pi and, with `--driver fake`, on a laptop with no robot, so
everything above the bridge is exercised over a real socket long before a motor turns. It
needs nothing installed (it puts ../src on sys.path itself), because the Pi has nothing
installed. SIGTERM is handled like Ctrl-C, so `systemctl stop` stops the motors too, not
just the process. One log line at start says where it listens, which driver, and every
timeout, which is the first thing to check when the robot stops "for no reason".

On the Pi: the fake needs nothing installed. `--driver real` needs pyserial
(hardware/nav_pi/setup.sh installs python3-serial) for the DDSM115 wheels:

    python3 scripts/run_bridge.py --driver real --wheel-port /dev/serial/by-id/<RS485 adapter>

Check the wheels with tools/diagnostics/wheel_check.py first: left/right IDs, forward
direction and counts/rev are unverified, and --wheel-radius must be measured.

A real e-stop button and vacuum relay work with either driver, so the fake tank
can run on a Pi 4 with real GPIO (BCM numbers):

    python3 scripts/run_bridge.py --estop-pin 17 --vacuum-pin 27
    python3 scripts/run_bridge.py --estop-pin 17 --vacuum-pin 27 --vacuum-active-low
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import signal
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from retriever.bridge.drivers import (  # noqa: E402
    DDSM115_COUNTS_PER_REV,
    build_real_driver,
    parse_id_list,
)
from retriever.bridge.fake_driver import FakeTankDriver  # noqa: E402
from retriever.bridge.gpio import EstopButton, VacuumOverlay, VacuumRelay  # noqa: E402
from retriever.bridge.power import PowerMonitor  # noqa: E402
from retriever.bridge.protocol import DEFAULT_PORT  # noqa: E402
from retriever.bridge.safety import add_lidar_args, lidar_from_args  # noqa: E402
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
    parser.add_argument(
        "--wheel-port", default=None,
        help="real: the DDSM115 USB-RS485 adapter (default: $DDSM115_PORT, else "
             "the one WCH adapter; use /dev/serial/by-id/... to pick between two)")
    parser.add_argument("--left-ids", default="1,2", help="real: left-side DDSM115 motor IDs")
    parser.add_argument("--right-ids", default="3,4", help="real: right-side DDSM115 motor IDs")
    parser.add_argument(
        "--wheel-flipped-ids", default="1,2",
        help="real: mirror-mounted motors, which get -rpm (measured on the robot: 1,2; "
             "the teammate's 3,4 drove it back-end first)")
    parser.add_argument(
        "--wheel-counts-per-rev", type=int, default=DDSM115_COUNTS_PER_REV,
        help="real: DDSM115 position counts per wheel turn (wheel_check.py rev)")
    parser.add_argument(
        "--wheel-reply-timeout-ms", type=float, default=10.0,
        help="real: wait per motor reply; a call blocks at most 4x (this + 10 ms)")
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
    parser.add_argument(
        "--wheel-radius", type=float, default=defaults.wheel_radius_m,
        help="metres. The default is a placeholder: MEASURE the tyre (diameter/2)")
    parser.add_argument("--track-width", type=float, default=defaults.track_width_m)
    parser.add_argument("--scrub-factor", type=float, default=defaults.scrub_factor)
    parser.add_argument("--imu", choices=("auto", "mpu", "d435i", "none"), default="auto",
                        help="gyro heading for the laptop's odometry: an MPU-6050/9250 on the "
                             "Pi's I2C, or the D435i's IMU over raw HID (auto: the MPU if it "
                             "answers, else the camera; naming one makes it required)")
    parser.add_argument("-v", "--verbose", action="store_true", help="debug logging")
    # Lidar safety bubble: all off unless --lidar-port or --fake-lidar is given.
    add_lidar_args(parser)
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

    lidar = None
    imu = None
    try:
        geo = TankGeometry(args.wheel_radius, args.track_width, args.scrub_factor)
        driver = build_driver(args, geo, relay)
        lidar, bubble = lidar_from_args(args, driver)   # (None, None) unless asked for
        # The Pi's own supply. Off a Pi (vcgencmd missing) this reads unavailable
        # and nothing is sent, so the sim is unaffected.
        power = PowerMonitor()
        if power.available:
            logger.info("power: 5 V rail %s, undervoltage %s",
                        "unknown" if power.volts is None else f"{power.volts:.2f} V",
                        "YES" if power.undervoltage else "no")
        # A gyro gives heading that ignores the skid-steer's wheel slip: the robot's
        # own MPU-6050/9250 (bridge/mpu.py), or the camera's IMU (bridge/imu.py).
        if args.imu != "none":
            if args.imu in ("auto", "mpu"):
                from retriever.bridge.mpu import MpuImu

                try:
                    imu = MpuImu().start()
                    logger.info("gyro: %s at 0x%02x on %s (heading for the laptop's odometry)",
                                imu.part, imu.address, imu.bus)
                except Exception as exc:
                    if args.imu == "mpu":
                        print(f"\n  {exc}\n", file=sys.stderr)
                        _close(lidar, relay, button)
                        return 2
                    logger.info("gyro: no MPU on I2C (%s); trying the camera's", exc)
            if imu is None and args.imu in ("auto", "d435i"):
                from retriever.bridge.imu import D435iImu, find_hidraw

                try:
                    if args.imu == "d435i" or find_hidraw() is not None:
                        imu = D435iImu().start()
                        logger.info("gyro: D435i IMU on %s (heading for the laptop's odometry)",
                                    imu.path)
                except Exception as exc:  # no camera, no permission: the wheels still do heading
                    if args.imu == "d435i":
                        print(f"\n  {exc}\n", file=sys.stderr)
                        _close(lidar, relay, button)
                        return 2
                    logger.warning("gyro: not used (%s)", exc)
            if imu is None:
                logger.warning("gyro: none found; heading falls back to the wheels, which slip")
        server = BridgeServer(
            driver,
            args.host,
            args.port,
            geo=geo,
            timeout_ms=args.timeout_ms,
            motion_timeout_ms=args.motion_timeout_ms,
            state_hz=args.state_hz,
            max_wheel_mps=args.max_wheel_mps,
            estop_input=button,
            lidar=lidar,
            bubble=bubble,
            scan_hz=args.scan_hz,
            imu=imu,
            power=power,
        )
    except (ConnectionError, FileNotFoundError, ImportError, NotImplementedError,
            RuntimeError, ValueError) as exc:
        print(f"fake_pi: {exc}", file=sys.stderr)
        _close(lidar, imu, relay, button)
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
        # server.close() already ran driver.stop(); close() also frees the wheel port
        _close(driver if hasattr(driver, "close") else None, relay, button, imu)
    return 0


def build_driver(
    args: argparse.Namespace, geo: TankGeometry, relay: VacuumRelay | None = None
) -> HardwareDriver:
    """The fake driver, or the real hardware."""
    if args.driver == "real":
        return build_real_driver(
            args.wheel_port, geo=geo, vacuum=relay,
            left_ids=parse_id_list(args.left_ids), right_ids=parse_id_list(args.right_ids),
            flipped_ids=parse_id_list(args.wheel_flipped_ids),
            wheel_counts_per_rev=args.wheel_counts_per_rev,
            wheel_reply_timeout_s=args.wheel_reply_timeout_ms / 1000.0)
    driver: HardwareDriver = FakeTankDriver(geo)
    if relay is not None:
        driver = VacuumOverlay(driver, relay)
    return driver


def _close(*parts: object) -> None:
    for part in parts:
        if part is not None:
            try:
                part.close()
            except Exception as exc:  # one part failing must not skip the rest
                print(f"closing {type(part).__name__}: {exc}", file=sys.stderr)


def _pin(spec: str) -> int | str:
    """"17" -> 17 (BCM); anything else ("GPIO17", "BOARD11") goes to gpiozero as is."""
    return int(spec) if spec.isdigit() else spec


if __name__ == "__main__":
    sys.exit(main())
