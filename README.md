# Voice-controlled multi-Pi robot

A runnable **simulation-first starter** for continuous voice commands, WebSocket communication, and a fetch-and-return task sequence. Python 3.11+; Raspberry Pi OS 64-bit is the intended Pi environment. No ROS dependency.

## Layout

- **Central Pi 5:** `robot_app.voice` captures microphone audio, plays model speech, and runs the task coordinator.
- **Cloud:** `robot_app.cloud` relays the Yibu Qwen realtime protocol and records token usage. Only this process needs the provider key.
- **Hardware Pi 5:** `robot_app.hardware` exposes the robot command protocol. Currently its driver simulates navigation and the SO-101 arm.
- **Depth-camera Pi:** `robot_app.camera` captures RGB/depth frames, detects the requested object, and returns a calibrated `map` pose. Set `CAMERA_URL` to this service. In the local demo the hardware simulator also supplies simulated camera observations.

The `vla_core/` and `hardware_bridge/` paths are pure-Python compatibility facades for the maintained `robot_app` implementation. ROS 2 is not required for Phase 1.

## Run the local simulation first

From the project directory on each Pi:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[test]'
export ROBOT_TOKEN='replace-with-the-same-random-secret-at-least-24-characters'
```

Generate a token with `python -c 'import secrets; print(secrets.token_urlsafe(32))'`. Variables in `.env.example` are documentation; `.env` is not loaded automatically.

Terminal 1:

```bash
python -m robot_app.hardware
```

Terminal 2, with the same token:

```bash
python -m robot_app.coordinator --target bottle
```

Expected: `state: completed`, `simulated: true`. No provider key, microphone, ROS, motors, or API credit needed. The sequence saves the start, locates, approaches, reobserves, grasps, verifies, stows, and returns. It holds the object at the start; release/handover is not implemented.

## Enable continuous voice

On the cloud host (or a separate local terminal for initial testing):

```bash
export ROBOT_TOKEN='your-shared-secret'
export YIBU_API_KEY='your-private-Yibu-key'
export YIBU_AUDIT_LOG='artifacts/yibu_api_calls.jsonl'
python -m robot_app.cloud --host 0.0.0.0
```

On the central Pi:

```bash
sudo apt-get install libportaudio2
python -m pip install -e '.[audio]'
export ROBOT_TOKEN='your-shared-secret'
export VOICE_URL='ws://127.0.0.1:8765'
export ROBOT_URL='ws://127.0.0.1:8766'
export CAMERA_URL="$ROBOT_URL"
export VOICE_OUTPUT_RATE=48000
python -m robot_app.voice --list-devices
python -m robot_app.voice --input-device 0 --output-device 0
```

## Enable the depth camera

Install the detector dependencies on the depth-camera Pi:

```bash
python -m pip install -e '.[camera]'
```

The `pyrealsense2` Python package does not currently publish a compatible wheel for every Raspberry Pi ARM64/Python combination. If installation of `.[camera]` succeeds but the camera service reports that `pyrealsense2` is unavailable, install the Intel RealSense SDK and Python bindings using the SDK's Raspberry Pi ARM64 instructions, or run `robot_app.camera` on a supported host that has the RealSense camera attached. The separate optional extra `.[camera-realsense]` is available only where a compatible pip wheel exists.

Start the camera service with the same `ROBOT_TOKEN` as the central Pi:

```bash
python -m robot_app.camera --host 0.0.0.0 --port 8767 --frame map \
  --translation 0 0 0
```

The service uses an Intel RealSense RGB/depth stream and YOLO-World. The detector's pixel-box center is deprojected using the depth measurement, then the configured translation is applied. The camera must be calibrated so the returned frame is the robot navigation frame; the coordinator rejects stale, uncertain, or mismatched observations. Set `CAMERA_URL=ws://CAMERA_PI_IP:8767` on the central Pi.

### Bridge the board birdseye feed

The board camera dashboard at `http://10.0.0.112:8766/` publishes JPEG frames at `ws://10.0.0.112:8765/birdseye`. To expose that feed to the VLA camera protocol, install the camera extra and run the bridge with a measured pixel-to-map calibration:

```bash
python -m pip install -e '.[camera]'
export ROBOT_TOKEN='same-secret-used-by-the-central-pi'
python -m robot_app.camera_bridge --host 0.0.0.0 --port 8767 \
  --map-center 0 0 --meters-per-pixel 0.005 -0.005 --object-height 0
```

Set `CAMERA_URL=ws://CAMERA_BRIDGE_IP:8767` on the central Pi. The bridge provides planar map coordinates from the image; it does not provide depth. Measure `--map-center`, `--meters-per-pixel`, and `--object-height` for the actual camera and robot frame before allowing motion.

Verify the camera Pi from the central Pi before starting voice:

```bash
export CAMERA_URL=ws://CAMERA_PI_IP:8767
python -m robot_app.camera_check --target bottle --frame map
```

Success means the authenticated WebSocket worked, a fresh RGB/depth frame was captured, YOLO-World found the requested object, depth produced a 3D point, and the pose passed freshness, confidence, and frame checks. A failure means the printed error must be fixed before testing robot motion.

The full deployment has three independent links:

| Link | Process | Purpose |
| --- | --- | --- |
| Microphone to voice relay | `robot_app.voice` -> `VOICE_URL` | Streams microphone audio and receives speech/tool events |
| Central Pi to robot Pi | coordinator -> `ROBOT_URL` | Saves pose, approaches, grasps, verifies, stows, and returns |
| Central Pi to camera Pi | coordinator -> `CAMERA_URL` | Captures frames and returns calibrated object observations |

The microphone is connected to the central Pi running `robot_app.voice`; the camera is connected to the camera Pi running `robot_app.camera`. They do not need to be physically attached to the same Raspberry Pi. To check the microphone before voice, run `python -m robot_app.voice --list-devices`, then start voice with the selected input and output device IDs. The voice process must print `Listening continuously`.

From the central Pi, verify all three network links before starting the microphone workflow:

```bash
export ROBOT_TOKEN='same-secret-used-everywhere'
export VOICE_URL=ws://CLOUD_RELAY_IP:8765
export ROBOT_URL=ws://ROBOT_PI_IP:8766
export CAMERA_URL=ws://CAMERA_PI_IP:8767
python -m robot_app.central_check --target bottle --frame map
```

This performs no robot movement and does not open the microphone. It verifies voice session negotiation, the robot's starting-pose response, and one live camera detection. Only after it succeeds should you run `python -m robot_app.voice` on the central Pi.

Replace device indices using the device list, or omit both to use defaults. Audio is mono signed 16-bit PCM: microphone 16 kHz, speaker 24 kHz by default. Set `VOICE_OUTPUT_RATE=48000` for the Jabra SPEAK 510 on the Raspberry Pi; that device rejects 24 kHz. Hardware must support the selected rates. Use an echo-cancelling USB speakerphone or configure system acoustic echo cancellation: the application does not implement AEC.

The microphone streams continuously; server VAD determines speech turns. No push-to-talk. Try “Bring me the bottle” or “Stop.” The model receives `fetch_object` and `stop_robot` tools; only validated tool calls enter the coordinator. Speech-start events clear buffered playback. Physical stopping through voice depends on cloud connectivity and recognition; an independent local stop remains a hardware integration requirement.

Live verification on September 19, 2026 accepted the audio/tool configuration and exercised a text-triggered `fetch_object` call through the cloud relay and simulated hardware, followed by provider speech audio. The Jabra microphone and speaker streams also opened successfully on Windows. **Spoken-command recognition and audible playback still need a human test.** A rejected configuration fails explicitly. No automatic reconnect or command replay is attempted.

### Initial Windows/Jabra voice test

Open three PowerShell terminals in this project directory. Use the same Python environment in each. Dependencies on the checked machine are already installed; for a fresh environment run `python -m pip install -e '.[audio,test]'`.

Generate a shared secret once with `python -c "import secrets; print(secrets.token_urlsafe(32))"` and copy its output into all three terminals below.

Terminal 1 (simulated robot):

```powershell
$env:ROBOT_TOKEN = 'PASTE_SHARED_SECRET'
python -m robot_app.hardware
```

Terminal 2 (voice relay):

```powershell
$env:ROBOT_TOKEN = 'PASTE_SHARED_SECRET'
$credential = Get-Credential -UserName 'Yibu' -Message 'Enter your Yibu API key as the password'
$env:YIBU_API_KEY = $credential.GetNetworkCredential().Password
python -m robot_app.cloud
```

Terminal 3 (microphone and speaker):

```powershell
$env:ROBOT_TOKEN = 'PASTE_SHARED_SECRET'
$env:VOICE_URL = 'ws://127.0.0.1:8765'
$env:ROBOT_URL = 'ws://127.0.0.1:8766'
$env:CAMERA_URL = $env:ROBOT_URL
python -m robot_app.voice --list-devices
python -m robot_app.voice --input-device 1 --output-device 4
```

On the checked Windows machine, devices 1 and 4 are the Jabra SPEAK 510 using MME; both opened at the required rates. Device numbers can change after reconnecting. The WASAPI Jabra input did not accept 16 kHz, so use the verified MME pair for this test.

Wait for `Listening continuously`, unmute the Jabra, and say:

1. “Hello. Say hello back.” Confirm that you hear the reply through the Jabra.
2. “Bring me the bottle.” Confirm terminal output contains `"state": "completed"` and `"simulated": true`, and the spoken reply identifies the simulation.
3. “Stop.” Confirm terminal output contains `"state": "stopped"`.

The simulator finishes quickly, so saying stop afterward checks the stop command but does not prove interruption during movement. Press Ctrl+C in the voice terminal first, then stop the other two services. If the microphone cannot open, check Windows microphone access for desktop apps and close other applications using the device. No physical robot movement is implemented.

## Separate machines and cloud deployment

Start the simulated hardware service on the hardware Pi with `--host 0.0.0.0`. Set `ROBOT_URL=ws://HARDWARE_PI_IP:8766` on the central Pi. When a camera service exists, set `CAMERA_URL=ws://CAMERA_PI_IP:8767`. Plain `ws://` is for a trusted private lab network only; use a private encrypted network or TLS for shared networks.

### Raspberry Pi 5 deployment checklist

The central Pi runs the microphone client only. Keep `YIBU_API_KEY` on the cloud relay; it is not needed on the central Pi.

Install the central Pi:

```bash
sudo apt update
sudo apt install -y python3-venv python3-pip libportaudio2
cd ~/VLA-HTN
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e '.[audio]'
python -m robot_app.voice --list-devices
```

For a one-Pi simulation, run hardware locally and use `ROBOT_URL=ws://127.0.0.1:8766`. For separate machines, use the following roles:

| Machine | Command/configuration |
| --- | --- |
| Cloud relay | `python -m robot_app.cloud --host 0.0.0.0`; keep `YIBU_API_KEY` here |
| Hardware Pi | `python -m robot_app.hardware --host 0.0.0.0` |
| Central Pi | `python -m robot_app.voice --input-device INPUT_ID --output-device OUTPUT_ID` |

On the central Pi, set the same `ROBOT_TOKEN` used by the other services, set `VOICE_URL=ws://CLOUD_RELAY_IP:8765`, `ROBOT_URL=ws://HARDWARE_PI_IP:8766`, and `CAMERA_URL=ws://CAMERA_PI_IP:8766`. Confirm the Pi can reach both ports before starting voice:

```bash
nc -vz CLOUD_RELAY_IP 8765
nc -vz HARDWARE_PI_IP 8766
```

The Pi should print `Listening continuously`. Say “Bring me the bottle”; the simulator should produce `"state": "completed"` and `"simulated": true`. Do not connect a real robot until this network and audio test passes. The current `hardware.py` is simulated and does not drive motors or an arm.

For a cloud VM:

```bash
docker build -t robot-voice .
docker run --rm -p 127.0.0.1:8765:8765 \
  -e ROBOT_TOKEN -e YIBU_API_KEY \
  -v robot-usage:/app/artifacts robot-voice
```

From the central Pi, keep an SSH tunnel open:

```bash
ssh -N -L 8765:127.0.0.1:8765 user@YOUR_CLOUD_HOST
```

Then use `VOICE_URL=ws://127.0.0.1:8765`. The tunnel encrypts the cloud connection and the cloud port remains private. The container is provided but has not been built in this workspace. Keep the same token on the cloud and robot services; this initial design is for one robot, not a multi-tenant service.

### Task decomposition and realtime checks

The maintained application includes an optional DashScope Qwen 3.5 Plus decomposition
adapter. Keep the key on the coordinator host, then enable it for an end-to-end run:

```bash
export DASHSCOPE_API_KEY='your-private-dashscope-key'
export VLA_DECOMPOSER_ENABLED=1
export ROBOT_WATCHDOG_TIMEOUT=30
python -m robot_app.coordinator --target bottle
```

The response is validated before execution. The coordinator runs a local safety check
before every robot command and sends the independent `stop` command if a command exceeds
the watchdog deadline. Decomposition is disabled by default, so offline tests never make
paid calls.

Run the five pick/place/deliver decomposition samples with:

```bash
python -m robot_app.benchmarks --json
```

Run the complete offline Phase 1 check without an API key or hardware:

```bash
export VLA_PROVIDER=mock
python -m robot_app.vla_integration_test --report artifacts/vla_integration_report.json
```

The report verifies simulated task completion, decomposition latency, object-location
latency, feedback latency, frame rate, and the active safety watchdog. A passing report
is a software-readiness result only; it does not validate Nav2, Arm 101, camera wiring,
or Raspberry Pi motor control.

`robot_app.action_monitor.RealtimeActionMonitor` paces camera feedback at 30 FPS and
records maximum feedback latency. The RealSense source is configured for 30 FPS, but
the 500 ms feedback target and physical camera/robot deployment still require a live
hardware measurement.

### Phase 2 hardware bridge

The hardware service now supports an opt-in bridge backend. It keeps the same validated
WebSocket command protocol while delegating navigation and manipulation to separate
Pi-side bridge processes:

```bash
export ROBOT_TOKEN='same-secret-used-by-the-bridges'
export NAV2_BRIDGE_URL='ws://NAV2_PI:8770'
export ARM101_BRIDGE_URL='ws://ARM101_PI:8771'
python -m robot_app.hardware --host 0.0.0.0 --backend bridge
```

The Nav2 bridge owns localization and base navigation; the Arm 101 bridge owns grasp,
verification, and stow. Both must implement authenticated JSON `health`, action, and
`stop` messages. The application does not guess a motor or serial protocol.

Run the non-motion preflight from the coordinator Pi before enabling movement:

```bash
python -m robot_app.phase2_check \
  --robot-url ws://HARDWARE_PI:8766 \
  --camera-url ws://CAMERA_PI:8767 \
  --nav2-url ws://NAV2_PI:8770 \
  --arm-url ws://ARM101_PI:8771 \
  --target bottle --frame map
```

The command checks robot pose, camera detection and calibration, and both bridge health
responses. It never sends navigation, grasp, or stop-motion commands.

## Hardware integration contract

Every WebSocket connection carries one command and one completion acknowledgement. Stop uses a separate connection so it can interrupt a running command. There are no automatic retries. Commands carry unique IDs and Unix expiry times; synchronize clocks on all Pis. The service rejects duplicates during its process lifetime, expired requests, and simultaneous movements. Command disconnect or deadline expiry requests a driver stop.

Request example:

```json
{"id":"unique-command-id","expires_at":1790000120,"action":"locate","target":"bottle","pose":null}
```

The example timestamp must be replaced with the current time plus the allowed deadline. Response:

```json
{"id":"unique-command-id","ok":true,"simulated":false,"result":{"target":"bottle","pose":{"frame":"map","x":1.2,"y":0.4,"z":0.7,"yaw":0},"observed_at":1790000000,"confidence":0.95},"error":null}
```

Distances are metres and angles radians. `observed_at` is the actual measurement time, not the time cached data was sent. Camera observations must already be transformed into the navigation frame using calibrated camera extrinsics and current localization. The coordinator rejects observations older than two seconds, confidence below 0.7, non-finite values, or mismatched frames. A detector's confidence is not a substitute for calibration or uncertainty estimation.

Implement a driver replacing `SimulatedHardware` only after verifying the controller:

| Action | Required real behavior/result |
| --- | --- |
| `save_start` | Return measured localized `pose` |
| `locate` | Return a fresh object pose, timestamp, confidence; fail if ambiguous |
| `approach` | Plan a collision-aware base pose within arm reach; don't drive the base to the object's XYZ |
| `grasp` | Use calibrated transforms, inverse kinematics or a trained policy, and joint/gripper limits |
| `verify_grasp` | Return measured `held: true/false` |
| `stow` | Move the arm into a verified carrying configuration |
| `return_start` | Navigate to the saved pose; report completion only after arrival |
| `stop` | Stop base motion and safely hold/stow the arm as appropriate |

The driver must expose `simulated = False`, an async `execute(Request) -> dict`, and an async `stop()`. Operations must allow concurrent stop/cancellation and enforce local watchdogs. USB serial is a likely SO-101 connection, but **no real arm or base driver is included yet**. Qwen supplies semantic task requests, not a trained VLA motor policy. Depth processing, SLAM/navigation, grasp planning, and camera-to-arm calibration remain hardware work.

## Usage reporting

```bash
python -m robot_app.audit --log artifacts/yibu_api_calls.jsonl --out-dir artifacts/summary
```

This creates `usage_summary.json` and `usage_by_model_key_purpose.csv`. Ledger records follow the linked `yibu_call_audit_v1` schema and preserve unknown usage as null. One completed response produces one record; incomplete/failed sessions produce an additional unknown-usage record. Thus call counts describe response records, not WebSocket sessions. No prompts, audio, transcripts, or full keys are intentionally logged. Original usage metadata is preserved. Summary layout is custom; the linked package's summarizer can also consume the ledger.

The organizer guide asks teams to return reports by September 20, 2026, 11:59 PM America/Toronto. Review reports and mention this custom per-response accounting when submitting them. Nothing is emailed automatically.

Sources:
- [User-supplied Yibu documentation, pinned commit](https://github.com/7nr754rpby-cmyk/OMNI-Live-Build-the-Next-Generation-of-Real-Time-Multimodal-AI/blob/6244ce145d2689544c0929a371adff47f0916383/docs/yibuapi-usage-reporting.md)
- [Qwen realtime client events](https://www.alibabacloud.com/help/en/model-studio/client-events)
- [SO-101 setup and calibration](https://huggingface.co/docs/lerobot/so101)

## Verification

```bash
python -m pytest -q
```

Tests exercise actual loopback WebSockets with simulated hardware/provider traffic, including fetch/return, cancellation, disconnect, duplicate/expired requests, failed grasps, invalid observations, and usage accounting. No paid provider calls are made.
