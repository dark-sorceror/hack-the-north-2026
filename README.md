<div align="center">

# Retriever

**Tell it what you dropped. It maps the room, routes around the furniture, drives there, picks it up, and brings it back.**

A voice-controlled retrieval robot built at **Hack the North 2026**.
Four computers, one of which runs the cloud coordinator and a hard-real-time safety
interlock on the same chip — on purpose, because it runs QNX.

<!-- TODO: add demo.gif here -->

</div>

---

## What it is

You say *"bring me my bottle."* Retriever hears you through the speaker it answers with,
works out which of three bottles on the floor is **yours**, plans a route around the chairs,
drives there, grasps it with a learned policy, and brings it back.

The interesting part was never picking things up. It was **deciding**:

- **Which one is yours.** A text-prompted detector finds *a* bottle. DINOv2 embeddings plus
  a hue histogram decide it's *your* bottle, by the margin over the runner-up.
- **Whether it's sure enough to act.** When the top two matches are too close to call, it
  asks instead of guessing. Across both evaluation runs, *accepted & wrong* was **0**.
- **Whether it should be moving at all.** Eight independent layers can stop this robot. The
  last one reaches it on a single optocoupled wire with no network path at all.

---

## Architecture

```
                  ☁  CLOUD — LLM planner
                     picks WHICH skill runs next · object memory
                     in no control loop, ever
                              │ HTTPS, initiated from below
 ┌────────────────────────────┴───────────────────────────────────┐
 │  CENTRAL Pi 5 — QNX 8.0 — eyes, ears, voice, and the interlock │
 │                                                                 │
 │   ┌── normal priority ────────────────────────────────────┐    │
 │   │  microphone in · speaker out                          │    │
 │   │  Qwen-Omni: image + audio + language, in ONE call     │    │
 │   │  D435i → object detection: WHICH thing did they mean? │    │
 │   │  depth on that box → where it is → a goal pose        │    │
 │   │  network-facing, allocates, allowed to crash          │    │
 │   └───────────────────────────────────────────────────────┘    │
 │   ┌── SCHED_FIFO 50 ──────────────────────────────────────┐    │
 │   │  the same camera → TFLite person detection, on-device │    │
 │   │  lineguard → 20 Hz heartbeat on GPIO17                │    │
 │   │  hard real-time, trusted, must never be starved       │    │
 │   └───────────────────────────────────────────────────────┘    │
 │                                                                 │
 │   One camera, two consumers at opposite ends of the priority   │
 │   scale. A microkernel is what makes that safe on one chip.    │
 └──────┬──────────────────┬──────────────────────┬───────────────┘
  goal  │            grasp │                      │ ONE optocoupled wire
  pose  │                  │                      │ no network path at all
 ┌──────┴────────────┐  ┌──┴───────────────┐      │
 │ NAV Pi — the spine│  │ S100 — the hand  │      │
 │                   │  │ SO-101 arm       │      │
 │ 4× DDSM115  RS485 │  │ ACT inference,   │      │
 │ RPLIDAR A2M12     │  │ local            │      │
 │ MPU-6050 gyro I²C │  │ trained on       │      │
 │                   │  │ Baseten          │      │
 │ watchdog 300 ms   │  └──────────────────┘      │
 │ deadman 500 ms    │◄────────────────────────────┘
 │ e-stop · bubble   │      ┌──────────────────────┐
 │ stdlib-only Python│◄:7777►│ MAC                 │
 └───────────────────┘      │ occupancy map        │
                            │ costmap · A*         │
                            │ pure pursuit         │
                            │ dashboard · teleop   │
                            └──────────────────────┘
```

### Why the coordinator and the interlock share a chip

On a best-effort Linux box, putting a network-facing cloud coordinator next to a
safety-critical control loop would be reckless — the coordinator allocates, blocks on I/O,
and is exactly the workload that makes something else miss a deadline.

QNX is a hard-real-time microkernel with strict priority scheduling, so the interlock at
`SCHED_FIFO 50` **cannot** be starved by anything the coordinator does. It's the same
property that has QNX running infotainment beside safety-critical systems in 275 million
vehicles.

We measured it with both running: **20–30 µs of scheduling lateness on a 25 ms tick.**

### Three rules we never bent

| Rule | Why |
|---|---|
| **The model decides WHAT, never HOW** | An LLM picks which skill runs next; control laws and a learned policy pick the actual motion. No model output becomes a velocity. |
| **Every layer below the brain can only *stop* the robot** | None of them invents motion, so a bug upstairs can never be papered over by a bug downstairs. |
| **The cloud is in no control loop** | The spine never talks to it. Lose the network and the robot gets *quieter*, not less safe. |

---

## Run it, with no robot

Core interfaces are stdlib-only on purpose — no torch, no hardware, no venv:

```bash
python3 -m unittest discover -s tests -t .      # 903 tests
```

Drive a simulated robot through a simulated room, in a browser:

```bash
python3 scripts/teleop.py --sim                 # then open http://127.0.0.1:8791
```

Hold **W/A/S/D** to drive. **Click the map** and it plans a route and follows it.
**Shift-click** for a round trip. **H** goes home. The simulated lidar drops ~60% of its
rays, adds noise, and lets black objects return nothing — because an idealised sensor hides
the bugs that matter.

<details>
<summary><b>On the real robot</b></summary>

```bash
# on the Pi (one command turns a fresh Pi into a robot that boots into the bridge)
bash nav-pi/setup.sh --driver real --imu mpu --lidar-port /dev/ttyAMA0

# on the laptop
python3 scripts/teleop.py --bridge hao.local:7777
```

`scripts/pi_doctor.sh` says in plain language why the Pi can't be reached.
`scripts/link_test.py` measures whether the 300 ms watchdog survives your network.
</details>

---

## From a sentence to wheel speeds

The LLM's job ends at choosing a skill and a destination. Everything after that is
classical autonomy — which is the point: each half is doing what it's actually good at, and
the seam between them is a coordinate.

```
"bring me my bottle"
   └─► LLM picks the skill ──► goto(x, y)
                                  │
       lidar scan ──► log-odds grid ──► distance field ──► costmap ──► A* ──► string pull
          at the        hit +0.9          3 ms, exact       lethal     1 ms     keeps
          pose it       miss −0.3         to 4.5e-7         0.35 m     420      clearance
          was taken                       cells             + fade     cells
                             ▲                                            │
          depth-camera points join here ─┘                                ▼
          block-only, expires ~2 s                           pure pursuit, 20 Hz ──► wheels
```

A model that hallucinates a destination gets a route planned to it, checked against a map,
and refused by a safety bubble if it can't be reached — rather than a hallucinated
trajectory going straight to a motor.

<details>
<summary><b>The four decisions that carry the navigation</b></summary>

**Hits weigh three times misses.** A chair leg is 2 cm thick in a 5 cm cell, and most beams
crossing that cell miss the leg. With equal weights the leg gets erased and the robot drives
into it. A cell that took a return is never cleared by the same scan.

**Unknown is never free.** Only beams that came *back* clear anything. A black or chrome
object looks exactly like empty space to a lidar, so unseen directions get creep speed — in
the map, the planner and the safety bubble alike.

**Inflate by the circumscribed radius.** Corner-to-centre, because a tank sweeps its corners
when it spins. On this chassis (43.5 × 38 cm) that's 0.29 m, giving a 0.35 m lethal zone.
Two things fall out free: a chair's four legs merge into one obstacle, and gaps under
0.70 m close — the honest answer for a 38 cm robot with odometry drift.

**Speed is what the corner allows.** The first rule divided top speed by curvature and had
the robot crawling at 0.1 m/s round a half-metre radius. Now speed is the least of what the
base can turn, what the bend allows sideways, and the distance left. A tight S-curve went
from **40 s to 14 s**, at 19 cm off the line.

The Euclidean distance transform that inflation needs is a separable two-pass implementation
in NumPy — SciPy and OpenCV both have one, and neither is on the Pi-side stack.
</details>

### Heading comes from a gyro, because the tyres lie

A four-wheel skid-steer turns by dragging all four tyres sideways. Ours claims **1.77×** the
rotation it actually produces, so it realised only **56%** of every turn rate commanded.
Wheel odometry gets distance right and heading catastrophically wrong.

An MPU-6050 on I²C measures rotation directly at 200 Hz. Yaw rate is taken about the
**gravity vector** the accelerometer reports, so the board can be mounted at any angle — it
measured the real mount at 2° off level and compensated. **Zero-velocity updates** hold the
heading and re-learn the bias whenever the wheels stop, because a stopped skid-steer is not
turning.

> Drift at standstill: **7.5 °/s → 0.000° over 10 seconds.**

### The grasp is the one part that's learned

Everything above the wrist is deterministic. The last 15 cm — approach, align, close without
knocking the thing over — is an **ACT (Action Chunking Transformer)** policy: demonstrations
teleoperated on the SO-101, trained on Baseten, checkpoint pulled down, **inference running
locally on the S100**. A grasp cannot wait for a network round trip, and ACT emits a chunk of
future actions per forward pass precisely so it doesn't have to.

---

## The safety chain

Eight layers. Each works when everything above it is dead, and every one is a **stop**,
never a steer.

| Layer | Trips when | Budget |
|---|---|---|
| The page | keys released, tab hidden, page closed | immediate |
| Teleop server | nothing from the page | 300 ms |
| Bridge client | the laptop process dies | immediate |
| Pi watchdog | no message at all | 300 ms |
| Pi motion deadman | heartbeats but no commands | 500 ms |
| Lidar safety bubble | a command it could not stop from | per command |
| E-stop latch | page, GPIO button, or missing heartbeat | latched |
| **QNX interlock** | a person too close — **or it dies** | ~250 ms |

The bubble is the only layer that reasons about geometry: it slides the robot's inflated
footprint along the commanded arc and refuses anything it couldn't stop from in time, at the
speed it's going. It can only ever *lower* a command.

### A heartbeat, not a level

The obvious wiring is "hold the line LOW while it's safe." We built it, then found three
ways it lies:

1. **A GPIO output keeps its last value after the process that set it dies** — a crashed
   supervisor keeps saying "OK" forever.
2. **An unpowered Pi can clamp the line near 0 V** through its protection diodes, which the
   robot reads as "OK".
3. **After a reboot the pin is an input with a pull-down**, fighting the robot's pull-up.

So `lineguard` **toggles** GPIO17 at 20 Hz while — and only while — SAFE verdicts keep
arriving, and the robot treats **any steady level** as STOP. Crash, hang, power loss, cut
wire and reboot all produce the same thing: a steady line.

```
kill -9 detector    →  STOP after 225 ms
kill -9 lineguard   →  pin freezes → no edges → robot stops
pull the power      →  optocoupler LED dark → line high → robot stops
```

---

## Measured

Every number came off the machine or a recording of it. Nothing here is a datasheet figure.

<table>
<tr><td valign="top" width="50%">

**Driving**
| | |
|---|---|
| Arrival from a clicked spot | **8 cm** |
| Heading drift, 10 s at standstill | **0.000°** |
| Turn slip (wheels vs gyro) | **1.77×** |
| Home error, reversing vs turning round | **5 cm** vs 86 cm |
| Tight S-curve, after the corner rule | **40 s → 14 s** |
| Chassis / turning circle | 43.5 × 38 cm / 0.29 m |

**Map and planner**
| | |
|---|---|
| Distance transform, 16 m map | **3 ms** |
| …error vs brute force | 4.5e-7 cells |
| A\* | **1 ms**, 420 cells |
| Clearance past a chair leg | 0.69 m |

</td><td valign="top" width="50%">

**QNX central Pi**
| | |
|---|---|
| SSD-MobileNet v1 quant, 4 threads | **38.5 ms** mean, 39.1 p99 |
| Live frame capture → STOP on the pin | **81–94 ms** |
| Detector rate | 17 fps |
| Interlock lateness, coordinator running | **20–30 µs** on a 25 ms tick |
| `kill -9` the detector | STOP after 225 ms |

**Link and process**
| | |
|---|---|
| Bridge state rate | 50–77 Hz |
| State gaps | p50 20 ms, p99 81 ms |
| Phone hotspot RTT | p50 22 ms, max 141 ms |
| Tests | **903** |

</td></tr>
</table>

---

## Repository layout

```
src/retriever/
  bridge/        the laptop↔Pi wire, and everything that stops the robot
    protocol.py    strict JSON lines: unknown fields and vy are errors
    server.py      watchdog · deadman · e-stop latch   (no sockets, no clock — testable exactly)
    safety.py      the lidar bubble: only ever lowers a command
    ddsm115.py     RS485 hub motors · mpu.py  I²C gyro · power.py  PMIC undervoltage
  navigation/
    grid.py        log-odds occupancy grid, distance field
    pathplan.py    costmap, A*, string pulling
    navigator.py   map keeper + route follower + camera layer
    pursuit.py     Catmull-Rom curves, pure pursuit, reverse retrace
    odometry.py    encoder unwrap + exact twist integration, fused with the gyro
    avoid.py       reactive VFH-lite, kept underneath the map
  perception/
    projection.py      the ONLY file allowed to write the camera's sines and cosines
    depth_obstacles.py depth grid → block-only obstacle points in the base frame
    identity.py        DINOv2 + hue margin gate — "is this *yours*?"
  planner/  skills/  voice/            the language side
scripts/         teleop · depth_publisher · wheel_check · link_test · pi_doctor
nav-pi/          setup.sh — one command from a fresh Pi to a robot that boots into the bridge
tests/           903 tests, incl. a subset that runs on the Pi's own Python
```

**The one rule:** nothing above `backends/` knows which backend it's talking to. The fake
robot, the fake Pi and the real one satisfy one interface — which is why four people could
work in parallel against one physical robot, and why the first real drive took minutes.

**The second rule, learned the hard way:** every seam between machines is a documented wire
format, not an import. The depth camera moved between machines five times in two days — nav
Pi, perception Pi, Mac, back again, and finally the QNX central Pi — and not one
consumer changed — which is why merging two Pis into one was a half-hour job instead of a
rewrite.

---

## Built as a progression

This repo is 20 stacked pull requests, in the order the robot actually came up:

```
#1  python skeleton            #11 drive controllers
#2  core interfaces            #12 obstacle avoidance
#3  tank kinematics            #13 teleop
#4  pi bridge                  #14 room map + A*
#5  pi setup + boot service    #15 pure pursuit
#6  link test / doctor         #16 gyro heading
#7  e-stop + relay             #17 curve speed
#8  DDSM115 wheels             #18 I²C IMU
#9  untethered on boot         #19 camera projection
#10 lidar safety bubble        #20 camera obstacles
```

Each PR is one working rung. Every one has a test plan, and the suite is green at each
merge.

---

## Sponsor tracks

| Track | Which computer it owns |
|---|---|
| **QNX** | The central Pi. QNX 8.0 running the cloud/voice coordinator and a hard-real-time safety interlock on one chip, with TFLite from `oss.qnx.com` on-device. Measured: 20–30 µs of scheduling lateness with both running. |
| **Huawei OMNI Live** | The voice path on that same board. Vision, speech and language in one `qwen3.5-omni-flash` call — the frame, the raw audio clip and the spoken answer are a single round trip, because *"bring me that one"* is unanswerable without the picture. |
| **OpenAI API** | The decision-maker in the cloud. A tool-calling loop where the tool list *is* the entire action space — the model cannot emit a velocity, only a skill and a destination, which the lidar → map → A\* pipeline turns into motion. |
| **Baseten** | Trains the ACT policy for the last 15 cm. Train in the cloud, own the weights, run inference on the S100 next to the arm. |

---

## Disclosure

Hack the North's build window is **12:00 AM EDT Sat 19 Sept → 8:00 AM EDT Sun 20 Sept**.

Parts of the instance-identity stack, the navigation geometry and the dashboard exist in a
prior personal repo and were ported in. **Everything on the robot this weekend** — the QNX
central Pi, the DDSM115 driver, teleop, the map and route planner, the gyro and
zero-velocity updates, the depth obstacle layer, the grasp policy — was built on site.

## Team

Hao Yan · Jay · Axzyl · Gisoo

<!-- TODO: roles, and links -->

<div align="center">

**Devpost** · **Demo video**

<!-- TODO: add the Devpost and demo video links -->

</div>
