#!/usr/bin/env python3
"""Bench check for the RPLIDAR A2M12: is it wired, talking, spinning, and mounted
the way the safety bubble thinks it is?

    python3 scripts/lidar_check.py                                # auto port; MOTOCTL tied high
    python3 scripts/lidar_check.py --port /dev/ttyAMA0 --motor-pin 18   # the robot's wiring
    python3 scripts/lidar_check.py --motor-pin 18 --find-front         # measure the mount yaw
    python3 scripts/lidar_check.py --motor-pin 18 --record-mask 10     # propose a self-mask
    python3 scripts/lidar_check.py --fake                              # no hardware at all

It prints the lidar's info and health, spins it up, then twice a second the
nearest return in each 45-degree sector of the ROBOT (with the --lidar-*
mount flags; without them, of the lidar itself), the revolution rate, and how
many rays came back. Needs pyserial (python3-serial; nav-pi/setup.sh installs it)
and, with --motor-pin, gpiozero (preinstalled on Raspberry Pi OS).

Angles: counter-clockwise degrees, 0 = the lidar's 0 mark (the side AWAY from
its cable). The RPLIDAR's own numbers run clockwise (raw = 360 - ours); this
script and the bridge convert once, at the parser.

The motor ALWAYS stops on exit, Ctrl-C and SIGTERM when this script controls
it (--motor-pin). With MOTOCTL tied high it cannot. With Slamtec's USB adapter
MOTOCTL follows DTR, so the motor spins again as soon as the port closes:
unplug the adapter to stop it.
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from retriever.bridge.lidar import (
    DEFAULT_BAUD,
    FakeLidar,
    FakeWorld,
    LidarError,
    LidarMount,
    RPLidarLink,
    ScanAssembler,
    bring_up,
    default_motor,
    find_ports,
    scan_to_base,
)
from retriever.bridge.safety import (
    Footprint,
    format_mask,
    parse_footprint,
    propose_mask,
)

SECTORS = ["front", "front-left", "left", "rear-left",
           "rear", "rear-right", "right", "front-right"]


class _Terminate(Exception):
    pass


def _on_sigterm(signum: int, frame: object) -> None:
    raise _Terminate()


def sectors(scan, mount: LidarMount) -> list[float | None]:
    """Nearest return per 45-degree sector of the base frame, from the base origin."""
    best: list[float | None] = [None] * 8
    for x, y, _ in scan_to_base(scan, mount):
        i = int(((math.degrees(math.atan2(y, x)) + 22.5) % 360.0) // 45.0)
        r = math.hypot(x, y)
        if best[i] is None or r < best[i]:
            best[i] = r
    return best


def report(scan, mount: LidarMount, out=print) -> None:
    rays = sum(scan.samples) or scan.n_raw
    pct = 100.0 * len(scan.points) / rays if rays else 0.0
    hz = f"{scan.rev_hz:5.1f} rev/s" if scan.rev_hz else "  ?   rev/s"
    out(f"{hz}  {len(scan.points):4d} points ({pct:3.0f}% of {rays} rays returned)")
    parts = []
    for name, r in zip(SECTORS, sectors(scan, mount)):
        parts.append(f"{name} {'  —  ' if r is None else f'{r:5.2f}'}")
    out("   " + "  ".join(parts))


def find_front(scan, mount: LidarMount, out=print) -> None:
    """The nearest return and the yaw that would put it dead ahead."""
    if not scan.points:
        out("   no returns")
        return
    a, r, _ = min(scan.points, key=lambda p: p[1])
    deg = math.degrees(a)
    raw = (360.0 - deg) % 360.0
    yaw = (deg if mount.inverted else -deg + 360.0) % 360.0
    yaw = yaw - 360.0 if yaw > 180.0 else yaw
    out(f"   nearest: {r:.2f} m at {deg:5.1f} deg (RPLIDAR raw {raw:5.1f} deg cw). "
        f"If that is the object straight in front of the robot: --lidar-yaw-deg {yaw:.0f}"
        + (" (with --lidar-inverted)" if mount.inverted else ""))


def fake_scans(args, stop_at: float):
    """FakeLidar in a room, with the robot's 'arm' as a self part, for running
    this script with no hardware."""
    world = FakeWorld.room(-2.0, -1.5, 2.5, 1.5, posts=[(0.8, 0.3, 0.1)])
    lidar = FakeLidar(world, rate_hz=8.0, beams=340, noise_m=0.005,
                      self_parts=[(0.12, 0.25, 0.04), (-0.15, -0.25, 0.04)])
    while time.monotonic() < stop_at:
        time.sleep(lidar.period)
        yield lidar.scan_now()


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", default=None,
                    help="serial port; default: search (/dev/ttyAMA0 first)")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="A2M12: 256000")
    ap.add_argument("--motor-pin", default=None,
                    help="BCM GPIO wired to MOTOCTL (18 on the robot)")
    ap.add_argument("--motor", choices=["gpio", "adapter", "external"], default=None)
    ap.add_argument("--fake", action="store_true", help="simulated lidar, no hardware")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="stop after S seconds (0: Ctrl-C)")
    ap.add_argument("--every", type=float, default=0.5, help="seconds between reports")
    ap.add_argument("--find-front", action="store_true",
                    help="print the nearest return and the --lidar-yaw-deg that puts it ahead")
    ap.add_argument("--record-mask", type=float, default=0.0, metavar="SECONDS",
                    help="record for SECONDS with nothing near the robot, "
                         "then propose --lidar-mask")
    ap.add_argument("--near", type=float, default=None,
                    help="--record-mask: returns nearer than this (m, from the lidar) are the "
                         "robot; default: farthest footprint corner + 0.10 m")
    ap.add_argument("--max-gap", type=float, default=30.0,
                    help="--record-mask: join two masked sectors across a gap this wide "
                         "(deg) if its rays returned nothing (the middle of a wheel)")
    ap.add_argument("--lidar-x", type=float, default=0.0)
    ap.add_argument("--lidar-y", type=float, default=0.0)
    ap.add_argument("--lidar-yaw-deg", type=float, default=0.0)
    ap.add_argument("--lidar-inverted", action="store_true")
    ap.add_argument("--footprint", default="0.25,0.25,0.20", help="FRONT,REAR,HALF_WIDTH (m)")
    args = ap.parse_args(argv)

    mount = LidarMount(args.lidar_x, args.lidar_y, math.radians(args.lidar_yaw_deg),
                       args.lidar_inverted)
    fp: Footprint = parse_footprint(args.footprint)
    recording = args.record_mask > 0
    duration = args.record_mask if recording else args.duration
    stop_at = time.monotonic() + duration if duration > 0 else math.inf

    signal.signal(signal.SIGTERM, _on_sigterm)
    link = motor = None
    kept = []
    try:
        if args.fake:
            print("fake lidar: a 4.5 x 3 m room, a post, and two self parts on the robot")
            scans = fake_scans(args, stop_at)
        else:
            port = args.port
            if port is None:
                found = find_ports()
                if not found:
                    print("no serial port found. On the Pi 5 header UART: add "
                          "dtoverlay=uart0-pi5 to /boot/firmware/config.txt and reboot; "
                          "or pass --port.")
                    return 2
                port = found[0]
                print(f"ports: {', '.join(found)}  -> using {port}")
            pin = args.motor_pin
            pin = int(pin) if pin and pin.isdigit() else pin
            try:
                motor = default_motor(port, pin, args.motor)
            except ImportError as exc:
                print(f"can't drive the motor pin: {exc}")
                return 2
            print(f"{port} @ {args.baud}, motor: {type(motor).__name__}"
                  + (f" on GPIO{pin}" if pin is not None else ""))
            link = RPLidarLink(port, args.baud, motor=motor)
            link.open()
            info, health = bring_up(link, log_fn=lambda m: print("  " + m))
            print(f"  {info}")
            print(f"  health: {health}")
            print("  spinning up (the first revolution takes ~1.5 s)...")

            def real_scans():
                asm = ScanAssembler()
                last = time.monotonic()
                while time.monotonic() < stop_at:
                    ms = link.read_measurements()
                    now = time.monotonic()
                    if ms:
                        for m in ms:
                            scan = asm.add(m, now)
                            if scan is not None:
                                last = now
                                yield scan
                    if now - last > 6.0:
                        raise LidarError("no complete revolution for 6 s "
                                         f"({link.parser.bad} bad bytes). Motor spinning? "
                                         "Right baud? TX/RX crossed?")

            scans = real_scans()

        if recording:
            print(f"recording {args.record_mask:.1f} s: keep everything else > 1 m away "
                  "(walls 1-3 m away are ideal); move the arm through its carry poses now")
        next_report = 0.0
        for scan in scans:
            if recording:
                kept.append(scan)
            now = time.monotonic()
            if now >= next_report:
                next_report = now + args.every
                report(scan, mount)
                if args.find_front:
                    find_front(scan, mount)
    except KeyboardInterrupt:
        print("\nCtrl-C")
    except _Terminate:
        print("\nSIGTERM")
    except LidarError as exc:
        print(f"\nlidar: {exc}")
        return 1
    finally:
        if link is not None:
            link.close()                 # STOP (laser off), motor off, port closed
        if motor is not None:
            motor.close()                # GPIO released; MOTOCTL's pull-down keeps it off
        if link is not None:
            if getattr(motor, "controllable", False) and type(motor).__name__ == "AdapterMotor":
                print("motor: USB adapter - it spins again now the port is closed "
                      "(MOTOCTL follows DTR). Unplug the adapter to stop it.")
            elif getattr(motor, "controllable", False):
                print("motor stopped.")
            else:
                print("laser stopped; MOTOCTL is not ours (tied high), so the motor keeps "
                      "spinning while powered.")

    if recording:
        near = args.near
        if near is None:
            near = 0.10 + max(math.hypot(cx - mount.x_m, cy - mount.y_m)
                              for cx, cy in fp.corners())
        mask = propose_mask(kept, near_m=near, max_gap_deg=args.max_gap)
        print(f"\n{len(kept)} revolutions; returns nearer than {near:.2f} m "
              "counted as the robot")
        if not mask:
            print("nothing to mask: no part of the robot is in the scan plane")
        else:
            total = sum((b - a) % 360.0 for a, b in mask) or 360.0
            print(f"proposed self-mask ({len(mask)} sectors, {total:.0f} deg blind):")
            for a, b in mask:
                mid = math.radians(a + ((b - a) % 360.0) / 2.0)
                bearing = math.degrees(mount.bearing(mid))
                side = SECTORS[int(((bearing + 22.5) % 360.0) // 45.0)]
                print(f"   {a:5.0f} .. {b:5.0f} deg   ({side} of the robot)")
            print(f"\n   --lidar-mask {format_mask(mask)}")
            print("   masked sectors are BLIND: the bubble allows only creep toward them.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
