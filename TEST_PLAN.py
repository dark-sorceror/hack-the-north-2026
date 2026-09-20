#!/usr/bin/env python3
"""
TEST PLAN: Voice → Vision Signal Path
Testing the first 2 steps of the fetch sequence
"""

PLAN = r"""
╔════════════════════════════════════════════════════════════════════════════════╗
║                                 TEST PLAN                                     ║
║                     Voice → Vision Signal Path (Steps 1-2)                    ║
╚════════════════════════════════════════════════════════════════════════════════╝

OVERVIEW
════════════════════════════════════════════════════════════════════════════════

Goal: Test that when a user says "grab an object", the system:
  1. Recognizes the voice command
  2. Signals the camera to detect the object
  3. Returns the detected object's 3D pose

Expected Flow:
  
  USER SPEAKS:     "Hey robot, grab a bottle"
                           ↓
  VOICE DETECTS:   fetch_object(object="bottle")
                           ↓
  COORDINATOR:     Call camera: locate(target="bottle")
                           ↓
  CAMERA:          Detect object → return pose
                           ↓
  RESULT:          {"pose": {x:1.2, y:0.5, z:0.7}, "confidence": 0.92}


TEST PHASES
════════════════════════════════════════════════════════════════════════════════

┌──────────────────────────────────────────────────────────────────────────────┐
│ PHASE 0: SETUP (One-time, ~30 minutes)                                      │
├──────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│ Task:    Install pyrealsense2 on camera Pi                                  │
│ Command: python quickstart.py --install                                     │
│          (or run interactively: python quickstart.py)                        │
│                                                                              │
│ What it does:                                                                │
│   1. Clones librealsense from GitHub                                         │
│   2. Runs cmake with Python bindings enabled                                 │
│   3. Compiles (make -j4)                                                     │
│   4. Installs system libraries (sudo make install)                           │
│   5. Installs Python package (setup.py install)                              │
│                                                                              │
│ Expected output:                                                             │
│   ✓ pyrealsense2 ready                                                       │
│                                                                              │
│ Troubleshooting:                                                             │
│   - If build fails, check disk space: ssh gisooj@gisoopi.local 'df -h'      │
│   - If pybind11 fails, check cmake version: cmake --version                 │
│   - Retry: python quickstart.py --install                                   │
│                                                                              │
└──────────────────────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────────────────────┐
│ PHASE 1: START CAMERA SERVICE                                                │
├──────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│ Task:    Start camera WebSocket server on Pi                                 │
│ Command: python quickstart.py --start-camera                                │
│                                                                              │
│ What it does:                                                                │
│   - Starts robot_app.camera on gisoopi.local:8767                           │
│   - Listens for "locate" requests                                           │
│   - Runs indefinitely until Ctrl+C                                          │
│                                                                              │
│ Expected output:                                                             │
│   ✓ Token configured                                                        │
│   ✓ WebSocket server listening on 0.0.0.0:8767                             │
│   Ready to receive: locate requests                                         │
│                                                                              │
│ Terminal usage:                                                              │
│   # Terminal 1 (Pi)                                                          │
│   $ python quickstart.py --start-camera                                     │
│   # This blocks; service runs in foreground                                 │
│   # Press Ctrl+C to stop                                                    │
│                                                                              │
│ Background mode (optional):                                                  │
│   $ ssh gisooj@gisoopi.local 'cd ~/VLA-HTN && nohup bash -c \\             │
│     "source venv/bin/activate && \\                                        │
│      export ROBOT_TOKEN=test-token-12345678901234567890 && \\              │
│      python -m robot_app.camera" > camera.log 2>&1 &'                      │
│                                                                              │
│   Check logs:                                                                │
│   $ ssh gisooj@gisoopi.local 'tail -f ~/VLA-HTN/camera.log'                │
│                                                                              │
└──────────────────────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────────────────────┐
│ PHASE 2: TEST VISION DETECTION (Step 2 of fetch)                            │
├──────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│ Task:    Test that camera can detect objects                                │
│ Command: python quickstart.py --test-vision                                 │
│          (or: python test_vision_integration.py --mode vision)              │
│                                                                              │
│ What it does:                                                                │
│   - Connects to camera service on Pi                                        │
│   - Sends: {"action": "locate", "target": "bottle"}                         │
│   - Repeats for: bottle, cup, book, remote, phone, glass                   │
│   - Shows detection results                                                 │
│                                                                              │
│ Expected output:                                                             │
│   ✓ bottle        Detected (conf: 92%)                                      │
│                  Pose: (1.20, 0.50, 0.70)                                  │
│   ✓ cup          Detected (conf: 87%)                                      │
│                  Pose: (1.10, -0.30, 0.60)                                 │
│   ✗ book         Not found                                                 │
│   ...                                                                       │
│                                                                              │
│   [RESULTS]                                                                 │
│   Detected: 3/6                                                             │
│   Failed:   3/6                                                             │
│                                                                              │
│ What "Not found" means:                                                     │
│   - Object was not visible in camera frame                                 │
│   - YOLO confidence < 0.25 threshold                                       │
│   - Try moving object closer or pointing camera at it                      │
│                                                                              │
│ Troubleshooting:                                                             │
│   - Camera not responding?                                                  │
│     → Check camera service is running: python quickstart.py --start-camera │
│   - All objects showing "Not found"?                                        │
│     → Point camera at object and try again                                 │
│     → Make sure RealSense has power (USB bus)                              │
│   - Confidence too low (< 0.25)?                                           │
│     → YOLO-World threshold is 0.25 by default                             │
│     → Try clearer/closer shots                                             │
│                                                                              │
└──────────────────────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────────────────────┐
│ PHASE 3: TEST COORDINATOR (Full Steps 1-8)                                   │
├──────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│ Task:    Test full fetch sequence with simulated hardware                   │
│ Command: python quickstart.py --test-coordinator                            │
│          (or: python test_vision_integration.py --mode coordinator)         │
│                                                                              │
│ What it does:                                                                │
│   - Simulates: "User says fetch a bottle"                                   │
│   - Runs all 8 steps:                                                       │
│     1. save_start (get home pose)                                           │
│     2. locate (detect bottle via camera Pi)        ← REAL                   │
│     3. approach (simulated nav)                    ← SIMULATED             │
│     4. locate (re-verify detection)                ← REAL                   │
│     5. grasp (simulated arm)                       ← SIMULATED             │
│     6. verify_grasp (simulated arm)                ← SIMULATED             │
│     7. stow (simulated arm)                        ← SIMULATED             │
│     8. return_start (simulated nav)                ← SIMULATED             │
│                                                                              │
│ Expected output:                                                             │
│   ✓ FETCH SEQUENCE COMPLETED                                               │
│                                                                              │
│   [RESULTS]                                                                 │
│   State: completed                                                          │
│   Simulated: true                                                           │
│   Steps completed: save_start, locate, approach, locate, grasp, ...       │
│                                                                              │
│   [LATENCIES]                                                               │
│   save_start               12.5ms                                           │
│   locate                   450.3ms  ← Real camera detection                 │
│   approach                 100.1ms  ← Simulated                            │
│   locate                   430.8ms  ← Real camera re-verify                │
│   grasp                    2000.5ms ← Simulated arm                        │
│   verify_grasp             15.2ms   ← Simulated                            │
│   stow                     1000.3ms ← Simulated                            │
│   return_start             105.0ms  ← Simulated                            │
│                                                                              │
│ Total end-to-end: ~4.1 seconds                                             │
│                                                                              │
│ Troubleshooting:                                                             │
│   - "Target not detected" error?                                            │
│     → Camera didn't find the object                                         │
│     → Try with a bottle/cup that's clearly visible                         │
│     → Check object is within 1 meter of camera                             │
│   - Timeout error?                                                          │
│     → Camera service not running or not reachable                          │
│     → Check: python quickstart.py --start-camera                           │
│   - "Connection refused"?                                                   │
│     → Pi not reachable or service crashed                                  │
│     → Check SSH: ssh gisooj@gisoopi.local echo OK                          │
│                                                                              │
└──────────────────────────────────────────────────────────────────────────────┘


CRITICAL TEST POINTS
════════════════════════════════════════════════════════════════════════════════

✓ TEST 1: Camera Service Responsive
  Verify:   curl ws://gisoopi.local:8767 returns WebSocket handshake
  Success:  Camera service is running and accepting connections
  
✓ TEST 2: Object Detection Works
  Verify:   Detect "bottle" with confidence > 0.7
  Success:  YOLO-World model loads and runs
  
✓ TEST 3: Pose Conversion Works
  Verify:   3D point (x, y, z) returned with correct frame
  Success:  RealSense depth + camera calibration working
  
✓ TEST 4: Coordinator Orchestration Works
  Verify:   All 8 steps execute in sequence
  Success:  Coordinator can call camera → nav2 → arm in order


COMMAND CHEAT SHEET
════════════════════════════════════════════════════════════════════════════════

# Interactive setup (recommended for first time)
python quickstart.py

# Install pyrealsense2
python quickstart.py --install

# Start camera service (blocks until Ctrl+C)
python quickstart.py --start-camera

# Test vision only
python test_vision_integration.py --mode vision

# Test full coordinator
python test_vision_integration.py --mode coordinator

# Show demo flow diagram
python test_vision_integration.py --mode demo

# Direct camera service start (if not using quickstart)
ssh gisooj@gisoopi.local
cd ~/VLA-HTN && source venv/bin/activate
export ROBOT_TOKEN='test-token-12345678901234567890'
python -m robot_app.camera --host 0.0.0.0 --port 8767

# Check if camera is responding
python -c "
import asyncio
from robot_app.protocol import Request, rpc
async def test():
    try:
        resp = await rpc('ws://gisoopi.local:8767', Request(action='locate', target='bottle'))
        print('✓ Camera is responding')
    except Exception as e:
        print(f'✗ Error: {e}')
asyncio.run(test())
"


NEXT STEPS AFTER TESTS PASS
════════════════════════════════════════════════════════════════════════════════

Once Steps 1-2 tests pass, you can:

1. IMPLEMENT NAV2 BRIDGE (Step 3: approach, Step 8: return_start)
   - Copy nav2_bridge_server.py to your Nav2 Pi
   - Edit Nav2Controller to connect to real Nav2 stack
   - Run: python nav2_bridge_server.py

2. IMPLEMENT ARM101 BRIDGE (Step 5: grasp, Step 6: verify, Step 7: stow)
   - Copy arm101_bridge_server.py to your Arm 101 Pi
   - Edit Arm101Controller to connect to real arm hardware
   - Run: python arm101_bridge_server.py

3. END-TO-END TEST with all components
   - Start: Camera service
   - Start: Nav2 bridge
   - Start: Arm101 bridge
   - Run: python -m robot_app.coordinator --target bottle
   - Verify: Robot fetches real object

4. VOICE INTEGRATION (Optional)
   - Run: python -m robot_app.voice
   - Speak: "Hey robot, grab a bottle"
   - Observe: Full end-to-end with voice


═══════════════════════════════════════════════════════════════════════════════
                           HAPPY TESTING! 🚀
═══════════════════════════════════════════════════════════════════════════════
"""

if __name__ == "__main__":
    print(PLAN)
