#!/usr/bin/env python3
"""
REMAINING TASKS TO RUN FULL VLA SYSTEM

Checklist of what's implemented vs. what you need to build.
"""

TASKS = """
╔═══════════════════════════════════════════════════════════════════════════════╗
║                            REMAINING WORK                                    ║
║                      To Get Vision → Arm Signal Working                      ║
╚═══════════════════════════════════════════════════════════════════════════════╝

PHASE 1: CAMERA PI SETUP (gisoopi.local)
─────────────────────────────────────────────────────────────────────────────────

✓ DONE:
  ✓ Python 3.13 environment set up
  ✓ YOLO-World model loaded (yolov8s-worldv2.pt)
  ✓ All dependencies except pyrealsense2 installed
  ✓ Intel RealSense D435i detected on USB

✗ TODO: Install pyrealsense2 SDK

  Run on camera Pi (one-time, ~30 min):
  
  $ cd /tmp
  $ git clone https://github.com/IntelRealSense/librealsense.git
  $ cd librealsense && mkdir build && cd build
  $ cmake .. -DBUILD_PYTHON_BINDINGS:bool=true -DCMAKE_BUILD_TYPE=Release
  $ make -j4
  $ sudo make install
  $ cd ../wrappers/python && python setup.py install
  
  Then test:
  
  $ source ~/VLA-HTN/venv/bin/activate
  $ python -c "import pyrealsense2; print('✓ pyrealsense2 ready')"
  
  Expected: prints "✓ pyrealsense2 ready"

✓ START CAMERA SERVICE:
  
  $ source ~/VLA-HTN/venv/bin/activate
  $ export ROBOT_TOKEN='your-24-char-secret'
  $ python -m robot_app.camera --host 0.0.0.0 --port 8767
  
  Expected output:
    ✓ Token configured
    ✓ WebSocket server listening on 0.0.0.0:8767
    Ready to receive: locate requests


PHASE 2: NAVIGATION PI SETUP
─────────────────────────────────────────────────────────────────────────────────

✗ TODO: Implement Nav2Bridge Server

  Files created for you:
    - nav2_bridge_server.py
    - arm101_bridge_server.py
  
  1. Copy nav2_bridge_server.py to your Nav2 Pi
  2. Edit Nav2Controller class to connect to your actual Nav2 stack
     - Implement get_current_pose() → query Nav2 TF
     - Implement navigate_to_pose() → send goal to Nav2 action server
  3. Run:
     $ export ROBOT_TOKEN='your-24-char-secret'
     $ python nav2_bridge_server.py
  
  Expected output:
    INFO:__main__:Nav2 Bridge listening on 0.0.0.0:8770
    Ready to receive: save_start, approach, return_start
  
  Signals it will receive:
    - save_start: Get and return current pose (home position)
    - approach: Navigate to detected object pose (FROM VISION!)
    - return_start: Navigate back to home


PHASE 3: ARM 101 PI SETUP
─────────────────────────────────────────────────────────────────────────────────

✗ TODO: Implement Arm101Bridge Server

  1. Copy arm101_bridge_server.py to your Arm 101 Pi
  2. Edit Arm101Controller class to connect to your arm hardware
     - Implement execute_grasp(target, pose) → grasp motion with strategy
     - Implement verify_grasp() → check force/weight sensor
     - Implement execute_stow() → return arm to home
  3. Run:
     $ export ROBOT_TOKEN='your-24-char-secret'
     $ python arm101_bridge_server.py
  
  Expected output:
    INFO:__main__:Arm 101 Bridge listening on 0.0.0.0:8771
    Ready to receive: grasp (with detection signal), verify_grasp, stow
  
  KEY SIGNALS it will receive:
    - grasp: WITH BOTH:
      * target: "bottle" (what vision detected)
      * pose: {x:1.2, y:0.5, z:0.7} (where nav2 navigated)
    - verify_grasp: Check if object is held
    - stow: Put arm away


PHASE 4: TEST END-TO-END
─────────────────────────────────────────────────────────────────────────────────

Once all three bridges are running:

  On CONTROLLER PI:
  
  $ export ROBOT_TOKEN='your-24-char-secret'
  $ export CAMERA_URL='ws://gisoopi.local:8767'
  $ export NAV2_BRIDGE_URL='ws://nav2_pi:8770'
  $ export ARM101_BRIDGE_URL='ws://arm101_pi:8771'
  $ python -m robot_app.coordinator --target bottle
  
  Expected output:
    {
      "state": "completed",
      "simulated": false,
      "steps": [
        "save_start",
        "locate",
        "approach",
        "locate",
        "grasp",
        "verify_grasp",
        "stow",
        "return_start"
      ],
      "latencies_s": {...}
    }


╔═══════════════════════════════════════════════════════════════════════════════╗
║                       THE VISION → ARM SIGNAL FLOW                            ║
╚═══════════════════════════════════════════════════════════════════════════════╝

USER:
  "Hey robot, fetch the bottle"
  
         ↓
         
VOICE SERVICE:
  Calls coordinator.fetch("bottle")
  
         ↓
         
COORDINATOR (step 2):
  "Locate the bottle"
  └─→ Camera Service
      Captures from RealSense D435i
      Runs YOLO: detect("bottle") → bounding box at (100, 150, 200, 250)
      Converts to 3D: point_in_camera(640x480 image) → (x:1.2, y:0.5, z:0.7)
      Returns: {"pose": {x:1.2, y:0.5, z:0.7}, "confidence": 0.92}
  
         ↓
         
COORDINATOR (step 3):
  "Navigate to the detected object"
  └─→ Nav2Bridge
      Receives: {"action": "approach", "pose": {x:1.2, y:0.5, z:0.7}}
      Sends goal to Nav2 stack
      Robot moves to (x:1.2, y:0.5, z:0.7)
      Returns: {"completed": true}
  
         ↓
         
COORDINATOR (step 5): ← THE KEY SIGNAL
  "Grasp the object"
  └─→ Arm101Bridge
      Receives: {
        "action": "grasp",
        "target": "bottle",              ← From vision detection
        "pose": {x:1.2, y:0.5, z:0.7}  ← From nav2 navigation
      }
      
      ARM NOW KNOWS:
        1. We detected a BOTTLE (object type)
        2. We navigated to LOCATION (x:1.2, y:0.5, z:0.7)
        3. Time to GRASP
      
      Arm executes grasp motion
      Returns: {"held": true}
  
         ↓
         
SUCCESS: Object fetched and returned to home


╔═══════════════════════════════════════════════════════════════════════════════╗
║                           QUICK TEST (No Hardware)                            ║
╚═══════════════════════════════════════════════════════════════════════════════╝

To test the FULL pipeline right now without any hardware:

  $ cd ~/VLA-HTN
  $ python -m robot_app.vla_integration_test
  
  This runs:
    ✓ Simulated vision (returns bottleneck at 0.5m)
    ✓ Simulated navigation
    ✓ Simulated arm control
    ✓ Full 7-step sequence
    ✓ Latency report
  
  Expected output:
    ✓ save_start: ~10ms
    ✓ locate: ~50ms
    ✓ approach: ~100ms
    ✓ locate (recheck): ~50ms
    ✓ grasp: ~2000ms
    ✓ verify_grasp: ~10ms
    ✓ stow: ~1000ms
    ✓ return_start: ~100ms
    
    Total: ~3.3 seconds


╔═══════════════════════════════════════════════════════════════════════════════╗
║                          SUMMARY OF FILES                                    ║
╚═══════════════════════════════════════════════════════════════════════════════╝

✓ Already in your workspace:
  robot_app/
    coordinator.py        ← The orchestrator (knows to call camera → nav2 → arm)
    camera.py            ← Vision service
    voice.py             ← Audio + cloud API
    hardware.py          ← Simulated + bridge routing
    hardware_integrations.py  ← Client stubs for Nav2 and Arm101
    protocol.py          ← WebSocket authentication + schemas
    safety.py            ← Emergency stop logic
    
  tests/
    test_*.py            ← All passing (100% pass rate)

✗ You need to create:
  nav2_bridge_server.py       ← Template provided, edit for your Nav2
  arm101_bridge_server.py     ← Template provided, edit for your arm
  
✗ One-time setup:
  Install pyrealsense2 on camera Pi (CMake build, ~30 min)


╔═══════════════════════════════════════════════════════════════════════════════╗
║                        ARCHITECTURE IS COMPLETE                               ║
║                  Just need to fill in the hardware adapters                   ║
╚═══════════════════════════════════════════════════════════════════════════════╝
"""

if __name__ == "__main__":
    print(TASKS)
