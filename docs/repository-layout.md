# Repository layout

```text
src/retriever/          Reusable Python runtime
  mission.py           Mission sequencing and navigation/arm interfaces
  perception/floor.py  Camera stream reader and floor-plane detector
  voice/               OMNI client, audio, and Mac/QNX/OpenAI adapters
  bridge/              Drivers, protocol, watchdogs, and lidar checks
  navigation/          Mapping, odometry, planning, and path following
  planner/             Tool-calling planner
  skills/              Skill library
  backends/            Robot interface and simulation backends
scripts/               Launchers, dashboard, camera tools, drive replay
hardware/
  nav_pi/              Navigation board provisioning and boot service
  qnx/                 Camera and supervisor code for QNX
  so101/               Arm code, board patches, and training launchers
tools/
  deploy/              Navigation board deployment
  diagnostics/         Connection, wheel, and lidar checks
  legacy/              Earlier setup experiments and manual hardware checks
tests/                 Automated hardware-free tests
docs/                  Development, setup, and repository documentation
```

## Renamed entry points

| Previous location | Current location |
| --- | --- |
| `nav-pi/` | `hardware/nav_pi/` |
| `qnx/` | `hardware/qnx/` |
| `SO-101/` | `hardware/so101/` |
| `scripts/fake_pi.py` | `scripts/run_bridge.py` |
| `scripts/central_pi.py` | `scripts/run_mission.py`; logic in `src/retriever/mission.py` |
| `scripts/replay_pi.py` | `scripts/replay_drive.py` |
| `scripts/cam_stream.py` | `scripts/camera_stream.py` |
| `scripts/floor_detector.py` | `scripts/detect_floor.py`; logic in `src/retriever/perception/floor.py` |
| `scripts/mac_voice.py` | `scripts/voice_mac.py` |
| `scripts/qnx_voice.py` | `scripts/voice_qnx.py` |
| `scripts/openai_voice.py` | `scripts/voice_openai.py` |
| `scripts/pi_deploy.sh` | `tools/deploy/nav_pi.sh` |
| `scripts/pi_doctor.sh` | `tools/diagnostics/pi_doctor.sh` |
| Root SSH setup helpers | `tools/legacy/ssh/` |

The old container files targeted the removed `robot_app` implementation and have been removed. The Python package name and wire protocols remain unchanged.
