#!/usr/bin/env python3
"""Capture and replay fixed arm poses on the SO-ARM-101 follower.

Scripted poses for things a learned policy should not have to do: parking the
arm for driving, and presenting the object to a person. Both are fully
determined, so making ACT learn them only adds inconsistency to the dataset.

    ./arm_poses.py capture rest          # torque off - pose by hand, press ENTER
    ./arm_poses.py capture carry
    ./arm_poses.py capture handoff
    ./arm_poses.py list
    ./arm_poses.py goto carry --hold-gripper
    ./arm_poses.py seq carry handoff --hold-gripper

Poses live in ~/arm_poses.json, in the same units the policy uses: degrees for
the five body joints, 0-100 for the gripper.

--hold-gripper leaves the gripper untouched. Use it any time the arm is
carrying something: without it, moving to a pose also drives the gripper to
that pose's opening and the object drops.

Run it inside the arm venv:
    cd ~/hiwonder-SoArm-101 && ./.venv/bin/python ~/arm_poses.py ...
"""

import argparse
import json
import math
import sys
import time
from pathlib import Path

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import OperatingMode
from lerobot.motors.hiwonder import HiwonderMotorsBus

PORT = "/dev/soarm_follower"
CAL = Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower/my_follower_arm.json"
POSES = Path.home() / "arm_poses.json"

BODY = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
GRIPPER = "gripper"

# Must match SOFollower exactly, or a scripted pose and a policy action mean
# different things: body joints in degrees, gripper 0-100.
MOTORS = {
    "shoulder_pan": Motor(1, "hx30hm", MotorNormMode.DEGREES),
    "shoulder_lift": Motor(2, "hx30hm", MotorNormMode.DEGREES),
    "elbow_flex": Motor(3, "hx30hm", MotorNormMode.DEGREES),
    "wrist_flex": Motor(4, "hx30hm", MotorNormMode.DEGREES),
    "wrist_roll": Motor(5, "hx30hm", MotorNormMode.DEGREES),
    GRIPPER: Motor(6, "hx30hm", MotorNormMode.RANGE_0_100),
}


def connect() -> HiwonderMotorsBus:
    if not CAL.exists():
        sys.exit(f"no calibration at {CAL}")
    cal = {k: MotorCalibration(**v) for k, v in json.loads(CAL.read_text()).items()}
    bus = HiwonderMotorsBus(port=PORT, motors=MOTORS, calibration=cal)
    bus.connect()
    if not bus.is_calibrated:
        bus.disconnect()
        sys.exit("motors disagree with the calibration file - refusing to move the arm")
    return bus


def load_poses() -> dict:
    return json.loads(POSES.read_text()) if POSES.exists() else {}


def save_poses(p: dict) -> None:
    POSES.write_text(json.dumps(p, indent=2, sort_keys=True) + "\n")


def cmd_capture(args) -> None:
    bus = connect()
    try:
        bus.disable_torque()
        time.sleep(0.1)
        print(f"Torque off. Move the arm to the '{args.name}' pose, then press ENTER.")
        input()
        pos = bus.sync_read("Present_Position", num_retry=3)
        poses = load_poses()
        poses[args.name] = {k: round(float(v), 2) for k, v in pos.items()}
        save_poses(poses)
        print(f"saved '{args.name}':")
        for k, v in poses[args.name].items():
            print(f"    {k:<15} {v:8.2f}")
        print(f"\n-> {POSES}")
    finally:
        bus.disconnect(disable_torque=True)


def cmd_list(_args) -> None:
    poses = load_poses()
    if not poses:
        sys.exit(f"no poses saved yet in {POSES}")
    names = sorted(poses)
    print(f"{'joint':<15}" + "".join(f"{n:>12}" for n in names))
    for j in BODY + [GRIPPER]:
        print(f"{j:<15}" + "".join(f"{poses[n].get(j, float('nan')):>12.2f}" for n in names))


def move_to(bus, target: dict, duration: float, rate: float, hold_gripper: bool) -> None:
    """Interpolate from the current pose to `target` with a smooth ramp.

    Writing the goal in one step makes the servos lunge at full speed; the arm
    slams and anything held gets thrown. Stepping the goal spreads it out, and
    the cosine ramp keeps acceleration off the endpoints.
    """
    joints = list(BODY) if hold_gripper else list(BODY) + [GRIPPER]
    joints = [j for j in joints if j in target]

    start = bus.sync_read("Present_Position", num_retry=3)
    steps = max(2, int(duration * rate))
    dt = 1.0 / rate

    for i in range(1, steps + 1):
        # cosine ease-in/ease-out over [0, 1]
        a = 0.5 - 0.5 * math.cos(math.pi * i / steps)
        goal = {j: start[j] + (target[j] - start[j]) * a for j in joints}
        bus.sync_write("Goal_Position", goal)
        time.sleep(dt)


def cmd_goto(args) -> None:
    poses = load_poses()
    if args.name not in poses:
        sys.exit(f"unknown pose '{args.name}'. known: {sorted(poses) or 'none'}")
    bus = connect()
    try:
        for m in bus.motors:
            bus.write("Operating_Mode", m, OperatingMode.POSITION.value)
        bus.enable_torque()
        time.sleep(0.1)
        print(f"moving to '{args.name}' over {args.duration}s"
              f"{' (gripper held)' if args.hold_gripper else ''}")
        move_to(bus, poses[args.name], args.duration, args.rate, args.hold_gripper)
        print("done")
    finally:
        # Leave torque ON: releasing it here would drop whatever is held and
        # let the arm fall. Cut power or run 'capture' to go slack.
        bus.disconnect(disable_torque=False)


def cmd_seq(args) -> None:
    poses = load_poses()
    missing = [n for n in args.names if n not in poses]
    if missing:
        sys.exit(f"unknown poses: {missing}. known: {sorted(poses)}")
    bus = connect()
    try:
        for m in bus.motors:
            bus.write("Operating_Mode", m, OperatingMode.POSITION.value)
        bus.enable_torque()
        time.sleep(0.1)
        for name in args.names:
            print(f"-> {name}")
            move_to(bus, poses[name], args.duration, args.rate, args.hold_gripper)
            time.sleep(args.dwell)
        print("sequence complete")
    finally:
        bus.disconnect(disable_torque=False)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    c = sub.add_parser("capture", help="record the current pose by name")
    c.add_argument("name")
    c.set_defaults(func=cmd_capture)

    sub.add_parser("list", help="show saved poses").set_defaults(func=cmd_list)

    g = sub.add_parser("goto", help="move smoothly to a saved pose")
    g.add_argument("name")
    g.add_argument("--duration", type=float, default=3.0, help="seconds for the move")
    g.add_argument("--rate", type=float, default=50.0, help="goal updates per second")
    g.add_argument("--hold-gripper", action="store_true", help="do not move the gripper")
    g.set_defaults(func=cmd_goto)

    s = sub.add_parser("seq", help="move through several poses in order")
    s.add_argument("names", nargs="+")
    s.add_argument("--duration", type=float, default=3.0)
    s.add_argument("--rate", type=float, default=50.0)
    s.add_argument("--dwell", type=float, default=0.5, help="pause between poses")
    s.add_argument("--hold-gripper", action="store_true")
    s.set_defaults(func=cmd_seq)

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\ninterrupted - torque left as-is")
