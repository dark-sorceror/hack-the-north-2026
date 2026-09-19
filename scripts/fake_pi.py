#!/usr/bin/env python3
"""The Pi's bridge process: serve the laptop until SIGINT or SIGTERM, then stop the motors.

The same entry point runs on the Pi and, with `--driver fake`, on a laptop with no robot, so
everything above the bridge is exercised over a real socket long before a motor turns. It
needs nothing installed (it puts ../src on sys.path itself), because the Pi has nothing
installed. SIGTERM is handled like Ctrl-C, so `systemctl stop` stops the motors too, not
just the process. One log line at start says where it listens, which driver, and every
timeout, which is the first thing to check when the robot stops "for no reason".
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
    try:
        geo = TankGeometry(args.wheel_radius, args.track_width, args.scrub_factor)
        server = BridgeServer(
            build_driver(args, geo),
            args.host,
            args.port,
            geo=geo,
            timeout_ms=args.timeout_ms,
            motion_timeout_ms=args.motion_timeout_ms,
            state_hz=args.state_hz,
            max_wheel_mps=args.max_wheel_mps,
        )
    except (NotImplementedError, ValueError) as exc:
        print(f"fake_pi: {exc}", file=sys.stderr)
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
    return 0


def build_driver(args: argparse.Namespace, geo: TankGeometry) -> HardwareDriver:
    """The fake driver, or the real hardware (which raises NotImplementedError for now)."""
    if args.driver == "real":
        return build_real_driver(args.wheel_port, args.arm_port, geo)
    return FakeTankDriver(geo)


if __name__ == "__main__":
    sys.exit(main())
