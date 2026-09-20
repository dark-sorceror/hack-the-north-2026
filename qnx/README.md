# QNX safety supervisor

A second Raspberry Pi 5 running QNX 8.0 watches for people with an on-device model and
holds the robot's e-stop line. The robot's AI runs in the cloud and on a laptop; this
doesn't. Kill it, unplug it or cut the wire, and the robot stops.

```
camera frames ─► detector.py (TFLite, oss.qnx.com) ─► SAFE/STOP datagrams ─► lineguard (C, SCHED_FIFO 50)
                                                                               │ GPIO17: 20 Hz square wave while SAFE
                                                                               ▼
                                                   optocoupler ─► robot Pi GPIO17 ─► bridge HeartbeatInput ─► e-stop latch
```

**Why a heartbeat and not a level.** A GPIO output keeps its last value after the process
that set it is gone, so "LOW means OK" survives a crash. `lineguard` toggles the pin only
while SAFE verdicts keep arriving, and the robot's bridge (`--estop-mode heartbeat`, see
`docs/pi-hardware.md`) stops on any constant level:

| Failure | What the robot sees |
|---|---|
| Person too close (box ≥ 45% of frame height) | detector sends STOP → LED held off |
| Detector crashes, hangs, camera stalls | no SAFE for 200 ms → LED held off |
| `lineguard` crashes or hangs | pin frozen at its last level |
| QNX Pi reboots or loses power | pin reverts to input with pull-down, LED off |
| Wire cut | robot's pull-up holds HIGH |

After any STOP, toggling resumes only after 1 s of continuous SAFE (`-H`), so a detector
that flickers at its threshold, or briefly loses someone who fills the frame, can't
restart the robot between two frames. That hold lives in `lineguard`, not in the AI.

## Wiring (QNX side)

QNX Pi GPIO17 (physical pin 11) → 1 kΩ → optocoupler LED anode; LED cathode → QNX Pi GND
(pin 9). That's about 2 mA through a ~1.2 V optocoupler LED; check your part's datasheet.
The transistor side goes to the robot Pi (see `docs/pi-hardware.md`). The optocoupler
isolates the two boards, so their grounds are **not** joined. With a bare wire and a
resistor, a powered-off QNX Pi can clamp the line near 0 V through its protection diodes,
which the robot could read as "OK".

## Setup (on a QNX Pi 5 quick-start card)

Everything builds on the Pi: the image ships clang, Python 3.14 and `apk` pointed at
repo.oss.qnx.com. No SDP or cross-compiler is needed.

```
scp -r qnx qnxuser@<pi>:supervisor          # default password qnxuser
ssh qnxuser@<pi> 'sh supervisor/setup.sh'   # apk add, models, test clip, make
ssh qnxuser@<pi> 'supervisor/run.sh start'  # then: status | stop
```

`setup.sh` installs `python3-tflite-runtime`, `python3-numpy` and `python3-opencv` from
oss.qnx.com, downloads SSD-MobileNet v1 (quantised COCO) and EfficientDet-Lite0, and
builds a test clip (`make_approach.py` zooms into a person in a photo, then back out).
`run.sh start` accepts `MODEL=`, `SOURCE=` (a video file or an image glob) and `FPS=`.

## Measured on the Pi 5 (qnxpi36, QNX 8.0.0), 2026-09-19

| | |
|---|---|
| SSD-MobileNet v1 quant, 300×300, 4 threads | 38.5 ms mean, 39.1 ms p99 (`bench.py`, still image) |
| EfficientDet-Lite0, 320×320, 4 threads | 61.6 ms mean, 62.5 ms p99 |
| Detector in the loop, 15 fps | 40.8 ms p50, 41.1 ms p99 inference, ~233% of a core |
| Frame capture → `lineguard` stops toggling | 48–65 ms |
| `lineguard` wake-up jitter with the AI saturating the CPU | 20–30 µs within a run; the mean lateness varies between runs (0.07–1 ms), which looks like timer-tick phase |
| `kill -9` the detector | STOP after 225 ms (200 ms deadline + one tick) |
| `kill -9` `lineguard` | pin frozen at a constant level |

`kill_test.sh` reproduces the last two, and reads the pin from the RP1 registers with
`gpio-rp1`.

Worst case, person to motors stopping: ≤ 67 ms frame period + ~41 ms inference + ≤ 25 ms
tick + the bridge's heartbeat timeout (100 ms) + one bridge poll (20 ms). About 250 ms,
independent of WiFi, the laptop and the cloud.

## Not done yet

- **A live camera.** Frames come from files for now. QNX cameras (Camera Module 3, USB
  UVC) go through the Sensor Framework's `camapi`, whose headers ship with the SDP, not
  the image. GStreamer on the image has no QNX camera source, and OpenCV has no V4L2.
  With the headers, a small C frame tap feeding `detector.py` is the missing piece.
  `/dev/sensor/camera2` (a file camera playing `simulator_camera.mov`) is there to test
  it against.
- **Autostart at boot.** Start it by hand with `run.sh` for now.

## Files

- `lineguard.c`: the fail-safe output; the only part that has to be right.
- `detector.py`: TFLite person detection → SAFE/STOP verdicts.
- `run.sh`, `setup.sh`, `Makefile`: run, install and build on the Pi.
- `bench.py`, `make_approach.py`, `kill_test.sh`: the measurements above.
