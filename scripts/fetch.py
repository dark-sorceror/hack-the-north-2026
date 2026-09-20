#!/usr/bin/env python3
"""Autonomous fetch: run the ACT policy until it has the object, then hand it over.

Replaces the `lerobot-record` + press-q workflow. There is no human in the loop:
once this is on the robot nobody can reach a keyboard, so the script decides for
itself when the grasp has happened and moves straight into the handoff.

    ./.venv/bin/python ~/fetch.py --checkpoint ~/checkpoints/checkpoints/020000/pretrained_model
    ./.venv/bin/python ~/fetch.py --probe          # log gripper telemetry, never auto-stop
    ./.venv/bin/python ~/fetch.py --no-handoff     # stop at the grasp

Why not lerobot-record
----------------------
`lerobot-record` builds a dataset: parquet writing, image writing and (by
default) AV1 video encoding, none of which inference needs. On this board the
encode stalled for tens of seconds after the operator pressed q, delaying the
handoff. It also needs a keyboard. This script keeps only the control loop.

Doing the handoff in the SAME process is the other win. Previously the policy
exited, released /dev/soarm_follower, and a second process reconnected -- but
`connect()` handshakes every declared motor, and a gripper latched in overload
protection does not answer, so the reconnect failed. Here the bus is never
dropped, so a latched gripper is simply never spoken to again.

Output
------
One JSON object per line on stdout, for the Pi5 to consume directly:

    {"event": "ready", "load_s": 6.6}
    {"event": "step", "n": 12, "hz": 0.97, "grip_pos": 11.4, "grip_current": 480}
    {"event": "grasped", "n": 34, "reason": "gripper_unresponsive"}
    {"event": "done", "status": "succeeded", "elapsed_s": 41.2}

Exit code 0 = object grasped (and handed off, unless --no-handoff).
Exit code 1 = timed out or failed. The arm keeps torque either way.
"""

import argparse
import json
import math
import sys
import time
from pathlib import Path

import torch

from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.datasets.feature_utils import build_dataset_frame, combine_feature_dicts
from lerobot.datasets.pipeline_features import (
    aggregate_pipeline_dataset_features,
    create_initial_features,
)
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.factory import make_pre_post_processors
from lerobot.policies.utils import make_robot_action
from lerobot.processor import make_default_processors
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.utils.constants import OBS_STR
from lerobot.utils.control_utils import predict_action

PORT = "/dev/soarm_follower"
ROBOT_ID = "my_follower_arm"
POSES = Path.home() / "arm_poses.json"

# Both cameras share USB bus 001. Uncompressed YUYV exceeds USB 2.0 bandwidth
# and starves the second camera, so MJPG is mandatory, not an optimisation.
BIRDSEYE = "/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920_222E1FAF-video-index0"
CLAW = "/dev/v4l/by-id/usb-icSpring_icspring_camera_202404160005-video-index0"

BODY = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
GRIPPER = "gripper"

# Candidate registers for grasp detection, tried in order at startup. Names vary
# between control tables, so probe rather than assume; if none answer we fall
# back to timeout-only termination instead of crashing mid-demo.
GRIP_REGISTERS = ["Present_Current", "Present_Load"]


def emit(**kw):
    """One JSON object per line. The Pi5 parses these as action feedback."""
    print(json.dumps(kw), flush=True)


def build_robot():
    cams = {
        "birdseye": OpenCVCameraConfig(index_or_path=BIRDSEYE, width=640, height=480,
                                       fps=30, fourcc="MJPG"),
        "claw": OpenCVCameraConfig(index_or_path=CLAW, width=640, height=480,
                                   fps=30, fourcc="MJPG"),
    }
    cfg = SO101FollowerConfig(
        port=PORT,
        id=ROBOT_ID,
        cameras=cams,
        # Never go limp on disconnect: the whole point is to keep holding the
        # object through the drive home.
        disable_torque_on_disconnect=False,
    )
    return SO101Follower(cfg)


def load_policy(ckpt: str, device: str):
    """Load the policy without any dataset.

    `make_policy()` insists on dataset metadata or a sim env, but the checkpoint
    already carries everything needed: `from_pretrained` for the weights, and
    the processor pipelines read their normalisation stats straight out of the
    checkpoint's safetensors.
    """
    policy = ACTPolicy.from_pretrained(ckpt)
    policy.eval()
    policy.config.device = device
    from lerobot.configs.policies import PreTrainedConfig
    cfg = PreTrainedConfig.from_pretrained(ckpt)
    cfg.pretrained_path = ckpt
    cfg.device = device
    pre, post = make_pre_post_processors(
        policy_cfg=cfg,
        pretrained_path=ckpt,
        preprocessor_overrides={"device_processor": {"device": device}},
    )
    return policy, pre, post


def probe_grip_register(robot):
    """Find a readable gripper load/current register, or None."""
    for reg in GRIP_REGISTERS:
        try:
            v = robot.bus.read(reg, GRIPPER, normalize=False)
            emit(event="grip_register", name=reg, value=int(v))
            return reg
        except Exception as e:
            emit(event="grip_register_unavailable", name=reg, error=str(e)[:80])
    return None


def read_grip(robot, reg):
    if reg is None:
        return None
    try:
        return int(robot.bus.read(reg, GRIPPER, normalize=False))
    except Exception:
        return None


def handoff(robot, duration: float, rate: float):
    """Move the five body joints to the saved handoff pose. Never the gripper.

    The gripper is holding the object, most likely latched in overload
    protection, in which case it answers nothing. Writing to it would at best
    open it and drop the object, at worst raise.
    """
    if not POSES.exists():
        emit(event="handoff_skipped", reason=f"no {POSES}")
        return False
    poses = json.loads(POSES.read_text())
    if "handoff" not in poses:
        emit(event="handoff_skipped", reason="no 'handoff' pose saved")
        return False
    target = poses["handoff"]

    joints = [j for j in BODY if j in target]
    start = {j: robot.bus.read("Present_Position", j) for j in joints}
    steps = max(2, int(duration * rate))
    dt = 1.0 / rate
    emit(event="handoff", joints=joints, duration_s=duration)
    for i in range(1, steps + 1):
        # Cosine ease so the arm does not lunge and fling what it is holding.
        a = 0.5 - 0.5 * math.cos(math.pi * i / steps)
        goal = {j: start[j] + (target[j] - start[j]) * a for j in joints}
        robot.bus.sync_write("Goal_Position", goal)
        time.sleep(dt)
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--checkpoint", default=str(Path.home() /
                    "checkpoints/checkpoints/020000/pretrained_model"))
    ap.add_argument("--task", default="pick up the goose")
    ap.add_argument("--fps", type=float, default=30.0,
                    help="target control rate; actual is inference-bound (~1 Hz on CPU)")
    ap.add_argument("--timeout", type=float, default=45.0,
                    help="abort if no grasp is detected within this many seconds")
    ap.add_argument("--grip-threshold", type=int, default=None,
                    help="grasp when the gripper register exceeds this for "
                         "--grip-frames consecutive steps. Run --probe first to "
                         "pick a value from real data.")
    ap.add_argument("--grip-frames", type=int, default=3)
    ap.add_argument("--settle-steps", type=int, default=5,
                    help="ignore grasp detection for this many steps at the start, "
                         "so the initial torque spike is not mistaken for a grasp")
    ap.add_argument("--probe", action="store_true",
                    help="log telemetry, never auto-stop; run to the timeout")
    ap.add_argument("--no-handoff", action="store_true")
    ap.add_argument("--handoff-duration", type=float, default=6.0)
    ap.add_argument("--device", default="cpu")
    args = ap.parse_args()

    t0 = time.perf_counter()
    emit(event="loading", checkpoint=args.checkpoint)
    policy, pre, post = load_policy(args.checkpoint, args.device)
    device = torch.device(args.device)

    robot = build_robot()
    robot.connect()

    _, robot_action_processor, robot_observation_processor = make_default_processors()
    features = combine_feature_dicts(
        aggregate_pipeline_dataset_features(
            pipeline=robot_observation_processor,
            initial_features=create_initial_features(observation=robot.observation_features),
            use_videos=True,
        ),
    )
    # Action feature names must match training order or make_robot_action maps
    # joint values onto the wrong motors.
    action_features = combine_feature_dicts(
        aggregate_pipeline_dataset_features(
            pipeline=robot_action_processor,
            initial_features=create_initial_features(action=robot.action_features),
            use_videos=True,
        ),
    )
    all_features = {**features, **action_features}

    grip_reg = probe_grip_register(robot)
    if grip_reg is None:
        emit(event="warning",
             detail="no gripper register readable; grasp detection limited to "
                    "unresponsiveness and timeout")

    policy.reset()
    pre.reset()
    post.reset()
    emit(event="ready", load_s=round(time.perf_counter() - t0, 1),
         grip_register=grip_reg, timeout_s=args.timeout,
         grip_threshold=args.grip_threshold, probe=args.probe)

    period = 1.0 / args.fps
    start = time.perf_counter()
    n = 0
    over = 0
    grasped = False
    reason = None

    try:
        while True:
            elapsed = time.perf_counter() - start
            if elapsed >= args.timeout:
                reason = "timeout"
                break
            loop_t = time.perf_counter()

            try:
                obs = robot.get_observation()
            except (ConnectionError, RuntimeError) as e:
                # A gripper that trips overload protection stops answering every
                # register read, which fails the whole sync_read. Empirically
                # that trip is what a successful grasp looks like on this arm.
                if n > args.settle_steps and not args.probe:
                    grasped, reason = True, "gripper_unresponsive"
                    emit(event="grasped", n=n, reason=reason, detail=str(e)[:120])
                    break
                # Sleep before retrying: a latched gripper fails every read, and
                # a bare `continue` would spin the serial port flat out.
                emit(event="read_error", n=n, error=str(e)[:120])
                time.sleep(0.25)
                continue

            obs_processed = robot_observation_processor(obs)
            frame = build_dataset_frame(all_features, obs_processed, prefix=OBS_STR)

            action_values = predict_action(
                observation=frame,
                policy=policy,
                device=device,
                preprocessor=pre,
                postprocessor=post,
                use_amp=False,
                task=args.task,
                robot_type=robot.robot_type,
            )
            act = make_robot_action(action_values, all_features)
            act = robot_action_processor((act, obs))
            robot.send_action(act)

            n += 1
            grip = read_grip(robot, grip_reg)
            hz = round(1.0 / max(time.perf_counter() - loop_t, 1e-6), 2)
            emit(event="step", n=n, t=round(elapsed, 1), hz=hz,
                 grip_pos=round(float(obs.get(f"{GRIPPER}.pos", float("nan"))), 2),
                 grip_raw=grip)

            if (not args.probe and args.grip_threshold is not None
                    and grip is not None and n > args.settle_steps):
                over = over + 1 if grip >= args.grip_threshold else 0
                if over >= args.grip_frames:
                    grasped, reason = True, "grip_threshold"
                    emit(event="grasped", n=n, reason=reason, grip_raw=grip)
                    break

            slack = period - (time.perf_counter() - loop_t)
            if slack > 0:
                time.sleep(slack)

    except KeyboardInterrupt:
        reason = "interrupted"
        emit(event="interrupted", n=n)

    did_handoff = False
    if grasped and not args.no_handoff:
        try:
            did_handoff = handoff(robot, args.handoff_duration, rate=50.0)
        except Exception as e:
            emit(event="handoff_failed", error=str(e)[:160])

    # disable_torque_on_disconnect=False, so the arm keeps holding the object.
    try:
        robot.disconnect()
    except Exception as e:
        emit(event="disconnect_warning", error=str(e)[:120])

    status = "succeeded" if grasped else "aborted"
    emit(event="done", status=status, reason=reason, steps=n,
         handoff=did_handoff, elapsed_s=round(time.perf_counter() - start, 1))
    return 0 if grasped else 1


if __name__ == "__main__":
    sys.exit(main())
