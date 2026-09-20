#!/usr/bin/env python3
"""Drive the robot from the Mac with W/A/S/D, in a browser tab.

    .venv/bin/python scripts/teleop.py --sim                   # no robot: fake Pi, fake lidar room
    .venv/bin/python scripts/teleop.py --bridge PI_IP:7777     # the real robot
    .venv/bin/python scripts/teleop.py --bridge PI_IP --record # + log every state and scan

On the Pi, the bridge with the real wheels:

    python3 scripts/run_bridge.py --driver real --wheel-port /dev/serial/by-id/<RS485 adapter>

Opens http://localhost:8791. Hold keys to drive, let go to stop; the page
lists the rest. Gear 1 (0.15 m/s) is the default, for the first time the real
wheels touch the floor. --listen 0.0.0.0 lets a phone on the same network
drive it with the on-screen pad: only do that on a network you trust.
"""

from __future__ import annotations

import argparse
import sys
import time
import webbrowser
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from retriever.bridge.client import BridgeRobot, parse_address  # noqa: E402
from retriever.bridge.protocol import DEFAULT_PORT  # noqa: E402
from retriever.teleop import (  # noqa: E402
    DriveRecorder,
    TeleopConfig,
    TeleopSession,
    parse_gears,
    serve,
)

ROOT = Path(__file__).resolve().parents[1]


def sim_bridge() -> tuple[object, int]:
    """A BridgeServer on a free local port with the fake tank, a fake lidar in a
    room with two chairs and a box, and the real safety bubble."""
    from retriever.bridge.fake_driver import FakeTankDriver
    from retriever.bridge.lidar import FakeLidar, FakeTankPose, FakeWorld
    from retriever.bridge.safety import BubbleConfig, SafetyBubble
    from retriever.bridge.server import BridgeServer, ServerThread

    legs = []
    for cx, cy in ((1.6, 0.5), (0.8, -1.3)):                      # two chairs: four thin legs each
        legs += [(cx + dx, cy + dy, 0.015) for dx in (-0.22, 0.22) for dy in (-0.22, 0.22)]
    world = FakeWorld.room(-2.0, -2.4, 3.5, 2.2, posts=legs)
    world.add_wall(2.2, -0.8, 2.6, -0.8).add_wall(2.6, -0.8, 2.6, -0.3)   # a box's two near faces
    world.add_wall(2.6, -0.3, 2.2, -0.3).add_wall(2.2, -0.3, 2.2, -0.8)
    driver = FakeTankDriver()
    bubble = SafetyBubble(BubbleConfig())
    lidar = FakeLidar(world, pose_fn=FakeTankPose(driver), mount=bubble.config.mount, noise_m=0.01)
    server = BridgeServer(driver, "127.0.0.1", 0, lidar=lidar, bubble=bubble)
    thread = ServerThread(server)
    return thread, thread.start()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--bridge", metavar="HOST[:PORT]", help=f"the Pi (port {DEFAULT_PORT})")
    src.add_argument("--sim", action="store_true", help="an in-process fake Pi; no robot needed")
    ap.add_argument("--port", type=int, default=8791, help="the page's port")
    ap.add_argument("--listen", default="127.0.0.1", help="0.0.0.0 to drive from a phone")
    ap.add_argument("--gears", default=None, help='m/s:rad/s per gear, e.g. "0.1:0.5,0.2:0.9"')
    ap.add_argument("--record", nargs="?", const="auto", default=None, metavar="PATH",
                    help="log states + scans as JSON lines (default: data/drives/<time>.jsonl)")
    ap.add_argument("--scrub-factor", type=float, default=None,
                    help="odometry's turn-slip factor, from the page's turn calibration "
                         "(overrides the Pi's until its bridge runs with --scrub-factor too)")
    ap.add_argument("--footprint", default="0.25,0.25,0.20",
                    help="front,rear,half-width in m, for drawing the robot")
    ap.add_argument("--map-min-range", type=float, default=0.45, metavar="M",
                    help="lidar returns closer than this to the robot's centre never reach "
                         "the map (default 0.45: the arm reaches 0.42, so anything nearer is "
                         "the robot itself). The Pi's safety bubble still sees them")
    ap.add_argument("--claw", type=float, default=0.0, metavar="M",
                    help="how much of --footprint's FRONT is arm rather than chassis. The "
                         "safety bubble uses the whole outline either way; this only stops "
                         "the page drawing the arm as if it were bodywork")
    ap.add_argument("--camera", default="auto", metavar="URL",
                    help="MJPEG stream to show on the page. 'auto' (default) points at the "
                         "robot Pi's port 8790, where scripts/camera_stream.py serves it; "
                         "'off' hides the panel")
    ap.add_argument("--clearance", type=float, default=0.02, metavar="M",
                    help="metres the route keeps clear PAST the body (default 0.02). The "
                         "bubble still sweeps the real footprint, so this only decides how "
                         "narrow a gap a route may aim at")
    ap.add_argument("--wide-gaps", action="store_true",
                    help="plan with the circumscribed radius (clear at every heading, so it "
                         "can always turn on the spot) instead of the body half width: "
                         "safer, but it refuses gaps the robot actually fits through")
    ap.add_argument("--arm", default=None, metavar="USER@HOST",
                    help="the arm board, e.g. sunrise@10.0.0.112; enables the G key")
    ap.add_argument("--arm-python", default="~/hiwonder-SoArm-101/.venv/bin/python",
                    help="the interpreter on the arm board that has lerobot")
    ap.add_argument("--no-open", action="store_true", help="don't open a browser tab")
    args = ap.parse_args()

    try:
        config = TeleopConfig(gears=parse_gears(args.gears)) if args.gears else TeleopConfig()
        footprint = tuple(float(v) for v in args.footprint.split(","))
        if len(footprint) != 3:
            raise ValueError("--footprint wants front,rear,half-width")
    except ValueError as exc:
        print(f"\n  {exc}\n", file=sys.stderr)
        return 2

    sim = None
    if args.sim:
        sim, bport = sim_bridge()
        host = "127.0.0.1"
    else:
        host, bport = parse_address(args.bridge)

    recorder = None
    if args.record:
        path = (ROOT / "data" / "drives" / time.strftime("drive-%Y%m%d-%H%M%S.jsonl")
                if args.record == "auto" else Path(args.record))
        recorder = DriveRecorder(path)

    def connect() -> BridgeRobot:
        try:
            return BridgeRobot(host, bport, connect_timeout_s=2.0, scrub_factor=args.scrub_factor)
        except ConnectionError as exc:
            if "refused" in str(exc).lower():   # the Pi answered: nothing on the port
                raise ConnectionError(
                    f"the Pi at {host} is up but its bridge isn't listening on {bport}. With "
                    "--driver real that means the wheel adapter isn't plugged in (or the "
                    "motors are off); it starts by itself once it is. On the Pi: "
                    "journalctl -u retriever-bridge -n 5") from None
            raise

    camera_url: str | None
    if args.camera == "off" or (args.camera == "auto" and not args.bridge):
        camera_url = None                      # a sim has no camera to show
    elif args.camera == "auto":
        camera_url = f"http://{args.bridge.split(':')[0]}:8790/stream.mjpg"
    else:
        camera_url = args.camera

    session = TeleopSession(connect, config, recorder, footprint=footprint,
                            tight_gaps=not args.wide_gaps,
                            clearance_m=args.clearance,
                            camera_url=camera_url, claw_m=args.claw,
                            map_min_range_m=args.map_min_range).start()
    if args.arm:
        from retriever.teleop import ArmRunner
        session.arm = ArmRunner(args.arm, python=args.arm_python)
        print(f"  arm: {args.arm}  (press G on the page to grab)")
    try:
        httpd = serve(session, args.listen, args.port)
    except OSError as exc:
        print(f"\n  can't serve the page on {args.listen}:{args.port}: {exc}\n", file=sys.stderr)
        session.close()
        return 1
    url = f"http://localhost:{httpd.server_port}"
    print(f"\n  teleop: {url}   robot: {'simulated' if sim else f'{host}:{bport}'}")
    if recorder:
        print(f"  recording to {recorder.path}")
    if not session.wait_connected(3.0):
        print(f"  not connected yet ({session.link_detail or 'waiting'}); retrying every second")
    print("  ctrl-c to quit (the robot stops)\n")
    if not args.no_open:
        webbrowser.open(url)
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        httpd.shutdown()
        session.close()
        if sim is not None:
            sim.stop()
    print("  stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
