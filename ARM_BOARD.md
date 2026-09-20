# Arm Board (RDK S100) — Integration Guide

This board owns **the arm and nothing else**. It has no navigation, no voice, no
mission logic. It exposes two operations to the rest of the robot:

1. **pickup** — run the trained ACT policy until the object is grasped
2. **handoff** — raise the arm to present the object to the person

Everything below is the contract for calling those two things.

---

## 1. The integration contract

### Command A — pickup

```bash
cd ~/hiwonder-SoArm-101 && ./.venv/bin/python ~/fetch.py --no-handoff
```

| | |
|---|---|
| Blocks for | ~7 s model load + up to 45 s rollout |
| Exit `0` | object is grasped, arm is holding it |
| Exit `1` | aborted (timeout or failure); arm holds position, no object |
| stdout | one JSON object per line (see §4) |

Autonomous — no keyboard, no operator. It decides for itself when the grasp has
happened.

### Command B — handoff

```bash
cd ~/hiwonder-SoArm-101 && ./.venv/bin/python ~/arm_poses.py goto handoff --hold-gripper --duration 6
```

| | |
|---|---|
| Blocks for | ~6 s (set by `--duration`) |
| Exit `0` | arm is at the handoff pose, still holding the object |
| stdout | human-readable progress, not machine-parsed |

`--hold-gripper` is **mandatory** here. See §5.

### Required sequence

```
nav: drive to the object
  └─> pickup          (wait for exit 0)
nav: drive back to the person        ← arm holds the object the whole way
  └─> handoff         (wait for exit 0)
person takes the object
```

**Only one of these may run at a time.** Both open `/dev/soarm_follower`, and
Linux will happily let two processes fight over the same serial port and corrupt
each other's traffic. If you parallelise anything, do not parallelise these.

---

## 2. Software stack — this is NOT upstream lerobot

> **Do not `pip install lerobot`. Do not swap in upstream lerobot. Do not assume
> Feetech servos.** Anything written against stock lerobot will not drive this
> arm correctly.

The arm runs a **Hiwonder fork** of lerobot, because the servos are Hiwonder
**HX-30HM**, not the Feetech STS3215 that upstream lerobot and the stock
SO-ARM-101 assume.

| | |
|---|---|
| Repo | `https://github.com/Hiwonder-official/hiwonder-SoArm-101.git` |
| Pinned commit | `a24998f7ba3c77ea445b48c92ad15c14a50e492a` |
| Reports version | `lerobot 0.5.1` (but it is **not** the upstream 0.5.1) |
| Checkout | `~/hiwonder-SoArm-101`, installed editable into `./.venv` |
| Servos | Hiwonder HX-30HM (`motor_model: "hx30hm"`, model number 777) |
| Bus class | `HiwonderMotorsBus(FeetechMotorsBus)` — reuses the STS/SMS control table |

The robot config is registered under the name `so101_follower`, which is
misleading: it is `SOFollower` with `motor_model = "hx30hm"`. The name matches
upstream but the behaviour does not.

Note also that lerobot 0.5.x **exists only as git tags** — PyPI stops at 0.4.4.
Anything that tries to resolve `lerobot==0.5.1` from PyPI will fail or silently
install something older.

### ⚠ The board's lerobot has uncommitted local patches

Five files in `~/hiwonder-SoArm-101` are modified in the working tree and
committed nowhere:

```
src/lerobot/motors/motors_bus.py            # ping + sync_read retry logic
src/lerobot/robots/so_follower/so_follower.py
src/lerobot/teleoperators/so_leader/so_leader.py
src/lerobot/scripts/lerobot_record.py       # rate-limited slow-loop warning
src/lerobot/utils/control_utils.py          # headless stdin controls
```

These are load-bearing. The `motors_bus.py` retry patches took connect failures
from roughly 1-in-3 down to 0-in-20, and without them mid-episode reads crash
with `TypeError: 'NoneType' object is not subscriptable`.

What they fix:

| File | Change |
|---|---|
| `motors_bus.py` | per-ping retry, whole-scan retry loop, and an `_all_present()` guard in `_sync_read` that raises `ConnectionError` instead of `TypeError` when a servo returns nothing |
| `so_follower.py`, `so_leader.py` | `sync_read` with `num_retry=3` |
| `lerobot_record.py` | rate-limited slow-loop warning (once per 5 s) instead of a flood |
| `control_utils.py` | headless stdin controls (`n` end, `r` re-record, `q` stop) |

**Consequences for integration:**
- A fresh clone of the fork at `a24998f7` will be *less reliable* than this board
- Re-cloning, `git checkout .`, or `git stash` in that directory **will break the arm**

All five are captured in `scripts/lerobot-board-patches.patch`. To rebuild the
board's software state from scratch:

```bash
git clone https://github.com/Hiwonder-official/hiwonder-SoArm-101.git
cd hiwonder-SoArm-101
git checkout a24998f7ba3c77ea445b48c92ad15c14a50e492a
git apply /path/to/scripts/lerobot-board-patches.patch
uv sync                      # needs Python 3.12; 3.14 breaks draccus
```

Then restore the calibration file (below) — without it the arm will not move
correctly even with the patches applied.

### Calibration is machine-local and must not be regenerated

```
~/.cache/huggingface/lerobot/calibration/robots/so_follower/my_follower_arm.json
```

This file encodes a physically re-seated wrist horn and a raised gripper
`range_min`. Re-running `lerobot-calibrate` produces *different* numbers and
invalidates the trained policy, because the policy learned joint values in these
units. Back it up; do not regenerate it.

---

## 3. Reaching the board

```bash
ssh sunrise@10.0.0.112        # password: ask Jay
```

- User `sunrise`, home `/home/sunrise`
- Arm venv: `~/hiwonder-SoArm-101/.venv/bin/python` (always use the full path;
  the system `python3` does not have lerobot)
- Both scripts live in `~`, not in the repo checkout
- Run them with `cwd = ~/hiwonder-SoArm-101` — that is how they were tested

> **The IP is a DHCP lease and will change.** `10.0.0.112` is `wlan0` on
> whatever network the board last joined. Before the demo, either reserve it on
> the router or switch to the direct-Ethernet plan in §8. Do not hardcode
> `10.0.0.112` in integration code — read it from config.

---

## 4. Machine-readable output from `fetch.py`

One JSON object per line on stdout. Parse these for progress; the exit code is
the authoritative result.

```json
{"event":"loading","checkpoint":"/home/sunrise/checkpoints/checkpoints/020000/pretrained_model"}
{"event":"grip_register","name":"Present_Current","value":112}
{"event":"ready","load_s":6.6,"grip_register":"Present_Current","timeout_s":45.0}
{"event":"step","n":12,"t":12.4,"hz":0.97,"grip_pos":11.4,"grip_raw":480}
{"event":"grasped","n":34,"reason":"gripper_unresponsive"}
{"event":"done","status":"succeeded","reason":"gripper_unresponsive","steps":34,"handoff":false,"elapsed_s":41.2}
```

| event | meaning |
|---|---|
| `loading` / `ready` | model loading; `ready` means the control loop is starting |
| `step` | one control iteration. `hz` is the real loop rate, `grip_raw` is gripper load |
| `grasped` | success detected, loop is exiting |
| `read_error` | a servo read failed; transient unless followed by `grasped` |
| `done` | terminal. `status` is `succeeded` or `aborted` |

**`hz` will read ~1.0, not 30.** That is expected and not a bug — see §6.

Useful flags: `--timeout SECONDS`, `--probe` (log telemetry, never auto-stop),
`--checkpoint DIR`, `--no-handoff` (stop at the grasp instead of continuing into
the handoff pose).

---

## 5. Constraints you must respect

These are not style preferences. Each one corresponds to a failure we hit.

**Never `Ctrl-C` the arm scripts.** It orphans the process holding the serial
port and skips the clean shutdown. Send `SIGTERM` and let them exit, or let them
run to completion.

**Do not power-cycle the arm between pickup and handoff.** The gripper holds the
object by tripping its overload protection and latching. Cutting power releases
it and the object drops.

**Always pass `--hold-gripper` to the handoff.** A latched gripper stops
answering *every* register read. `connect()` handshakes every declared motor, so
a normal connect fails outright with `Missing motor IDs: 6`. `--hold-gripper`
removes the gripper from both the motor table and the calibration, so it is
never addressed at all.

**The arm keeps torque after both commands.** This is deliberate
(`disable_torque_on_disconnect=false`) — releasing torque drops the object and
lets the arm fall. It draws current while idle; don't leave it parked for long.

**Both cameras must use MJPG.** They share USB bus 001 and uncompressed YUYV
exceeds USB 2.0 bandwidth, which silently starves the second camera. Already set
in the scripts; just don't "clean it up".

---

## 6. Performance, and why it still works

Inference runs at **0.8–1.0 Hz on CPU**, against a nominal 30 Hz target. The BPU
(the board's NPU) is not yet wired up.

This works anyway because ACT is an *action-chunking* policy: each inference
emits 100 future actions covering roughly 3 seconds of motion. So a ~1 Hz
inference rate still produces continuous movement. Budget **30–45 s** for a
pickup and don't treat the slow-loop warnings as errors.

---

## 7. Recovery

| Symptom | Cause | Fix |
|---|---|---|
| `Missing motor IDs: 6` | gripper latched in overload | add `--hold-gripper`, or power-cycle the arm if you don't need to keep holding |
| `Missing motor IDs: 1` (or others) at connect | flaky handshake | retry; the bus has retry patches but isn't perfect |
| Port busy / garbled reads | two processes on the serial port | ensure only one command runs at a time |
| `fetch.py` exits 1 immediately | camera or checkpoint missing | check the `loading`/`ready` lines |
| Arm goes limp | torque released | re-run; check nothing passed `disable_torque_on_disconnect=true` |
| `ValueError: Magnitude N exceeds 2047` | calibration/horn problem | do **not** recalibrate blindly — see §2 |

Fallback path if `fetch.py` misbehaves: `~/run_goose.sh 020000` is the older
operator-driven flow (policy + automatic handoff), but it **requires a human to
press `q`**, so it is not usable on the robot. Keep it for bench testing only.

---

## 8. Status — read this before integrating

**Working and tested on hardware:**
- ACT policy inference; picks up the goose, and recovers after a failed grasp
- The handoff pose, including with the gripper latched in overload
- Checkpoints at 10k and 20k steps (`~/checkpoints/checkpoints/{010000,020000}/`)

**Written, imports verified, NOT yet run on hardware:**
- `fetch.py` itself. It replaces a workflow that required a human keypress, and
  its grasp detection defaults to "the gripper stopped responding", which is
  what a successful grasp looks like on this arm — but that has not been
  exercised end to end yet. **Test it before you depend on it.** The older
  `run_goose.sh` path is proven but needs an operator.

**Planned, not built:**
- A WebSocket action server on the board so the Pi5 can trigger these over the
  network instead of over SSH, with goal/feedback/result semantics like a ROS2
  action. Until that exists, **SSH + exit code is the interface.**
- Direct Ethernet between Pi5 and board (`eth1` already holds a static
  `192.168.127.10/24`) to remove the WiFi dependency entirely.
- BPU acceleration, to lift inference off the CPU.

---

## 9. What this board does not do

Navigation, obstacle avoidance, voice, wake-word, mission sequencing, deciding
*when* to pick up or *where* to drive. It picks up and it hands over. Everything
else belongs to the Pi5.
