# Development

The distribution is **goosetriever**. Python imports remain `retriever` to keep the existing interfaces stable.

## Local environment

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e ".[camera,test]"
python scripts/teleop.py --sim
python -m unittest discover -s tests -t .
```

Use `.[audio,perception]` for laptop microphone/speaker and YOLO-based camera experiments. Model weights are downloaded separately when the detector loads them. The arm has its own pinned Hiwonder/LeRobot environment; see [the arm guide](../hardware/so101/ARM_BOARD.md).

## Entry points

| Command | Purpose |
| --- | --- |
| `python scripts/teleop.py --sim` | Navigation simulation and dashboard |
| `python scripts/run_bridge.py --driver fake` | Standalone bridge with fake wheels |
| `python scripts/run_mission.py --dry-run --once "fetch the goose"` | Exercise mission sequencing without hardware |
| `python scripts/detect_floor.py --help` | Object detection and floor-plane projection |
| `python scripts/camera_view.py --help` | Annotated camera viewer |
| `python scripts/camera_stream.py --help` | MJPEG camera server |
| `python scripts/voice_mac.py --help` | OMNI voice using laptop audio |
| `python scripts/voice_qnx.py --help` | OMNI voice using the QNX audio helper over SSH |
| `python scripts/voice_openai.py --help` | Alternative OpenAI voice adapter |
| `python scripts/replay_drive.py --help` | Replay a recorded drive without moving hardware |
| `bash tools/deploy/nav_pi.sh --help` | Deploy to the navigation Pi |
| `bash tools/diagnostics/pi_doctor.sh --help` | Diagnose the navigation Pi connection |

Run commands from the repository root. Python launch scripts locate the source package relative to their own location. `.env.example` lists supported environment variables; it is not automatically loaded.

The mission CLI currently uses console input. Voice adapters are available separately; a single voice-enabled mission launcher is still integration work. The QNX stream publisher and board audio helper must already be present on the board. A fresh clone alone does not reconstruct those board-local dependencies or the arm calibration and checkpoints.

## Naming and layout

- Python files and importable directories use `snake_case`.
- Board names are `nav_pi`, `qnx`, and `so101` under `hardware/`.
- Runtime logic lives in `src/retriever/`; `scripts/` contains launchers and operator applications.
- Provisioning, deployment, and diagnostics live under `tools/`.
- Old setup experiments live under `tools/legacy/` and are not part of the supported startup path.
- Keep credentials, recordings, downloaded weights, and generated outputs out of Git.

The existing board checkout remains `~/retriever`; moving local files does not rename directories or services on deployed hardware. Rerun setup when deploying the new bridge launcher so the service points to `scripts/run_bridge.py`.
