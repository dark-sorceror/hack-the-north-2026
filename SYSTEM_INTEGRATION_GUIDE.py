#!/usr/bin/env python3
"""
VLA-HTN COMPLETE SYSTEM INTEGRATION GUIDE

This is the master documentation for deploying the complete fetch robot system
across 3 Raspberry Pi computers.

SYSTEM ARCHITECTURE:
===================

┌─────────────────────────────────────────────────────────────────────────┐
│                         COORDINATOR (Camera Pi)                          │
│  ┌───────────────────────────────────────────────────────────────────┐  │
│  │ Main Orchestration Logic                                          │  │
│  │ - Manages 8-step fetch sequence                                   │  │
│  │ - Validates safety at each step                                   │  │
│  │ - Coordinates with 3 separate services                            │  │
│  └───────────────────────────────────────────────────────────────────┘  │
│         ↓                          ↓                        ↓            │
│  ┌───────────────┐         ┌──────────────────┐    ┌──────────────┐    │
│  │  CAMERA       │         │   PROTOCOL &     │    │  SAFETY      │    │
│  │  SERVICE      │         │   WEBSOCKET      │    │  WATCHDOG    │    │
│  │  (port 8767)  │         │   (port 8766)    │    │              │    │
│  │               │         │                  │    │              │    │
│  │ RealSense +   │         │ Authenticated    │    │ Emergency    │    │
│  │ YOLO-World    │         │ JSON RPC         │    │ Stop + Time  │    │
│  │ Detection     │         │                  │    │ Limits       │    │
│  └───────────────┘         └──────────────────┘    └──────────────┘    │
│                                                                          │
│                              CAMERA PI                                  │
└─────────────────────────────────────────────────────────────────────────┘
                 ↓                                              ↓
     ┌───────────────────────────┐         ┌──────────────────────────┐
     │  NAV2 BRIDGE (Nav2 Pi)     │         │ ARM101 BRIDGE (Arm Pi)   │
     │  (port 8770)               │         │ (port 8771)              │
     │                            │         │                          │
     │  Handles:                  │         │ Handles:                 │
     │  - save_start              │         │ - grasp (CRITICAL)       │
     │  - approach (from vision)  │         │ - verify_grasp           │
     │  - return_start (home)     │         │ - stow                   │
     │                            │         │                          │
     │  Receives pose from camera │         │ Receives object + pose   │
     │  Navigates robot to target │         │ Uses both signals        │
     │                            │         │                          │
     └────────┬────────────────────┘         └────────┬─────────────────┘
              │                                       │
         ┌────▼─────────┐                      ┌──────▼──────────┐
         │  NAV2 STACK  │                      │  ARM101        │
         │  ROS2 Nav    │                      │  HARDWARE      │
         │  on Pi 2     │                      │  on Pi 3       │
         └──────────────┘                      └────────────────┘

SIGNAL FLOW - COMPLETE SEQUENCE:
===============================

Voice Input: "grab a bottle"
            ↓
         STEP 1: save_start → NAV2_BRIDGE (WebSocket to Nav2 Pi)
            "Remember starting position" 
            ↓ Response: current_pose = {x:0, y:0, z:0}
         
         STEP 2: locate → CAMERA_SERVICE (same machine)
            "Find a bottle using YOLO-World detection"
            ↓ Response: detected_pose = {x:1.2, y:0.5, z:0.7, frame:"map"}
         
         STEP 3: approach → NAV2_BRIDGE (WebSocket)
            "Navigate to x:1.2, y:0.5" [SIGNAL: pos from vision]
            ↓ Response: navigation_complete
         
         STEP 2b: locate → CAMERA_SERVICE
            "Reobserve bottle (never grasp from old image)"
            ↓ Response: new_detected_pose (after movement)
         
         STEP 5: grasp → ARM101_BRIDGE (WebSocket to Arm Pi)
            "Grasp bottle at x:1.2, y:0.5" [SIGNAL: object + pose]
            ↓ Response: grasp_complete
         
         STEP 6: verify_grasp → ARM101_BRIDGE
            "Check if holding object"
            ↓ Response: held = True
         
         STEP 7: stow → ARM101_BRIDGE
            "Put arm away safely"
            ↓ Response: stow_complete
         
         STEP 8: return_start → NAV2_BRIDGE
            "Navigate back to starting position"
            ↓ Response: returned_to_start

COMPLETE ✓
Object retrieved and back at starting position!


DEPLOYMENT SETUP:
================

MACHINE 1: CAMERA PI (Primary)
────────────────────────────────

Hardware:
  - Raspberry Pi 4+ (8GB recommended)
  - Intel RealSense D435i (USB)
  - Robust network connection

Installation:
  1. Copy entire VLA-HTN repo to ~/VLA-HTN
  2. cd ~/VLA-HTN
  3. python3 -m venv venv
  4. source venv/bin/activate
  5. pip install -r requirements.txt
  6. export ROBOT_TOKEN="your-long-secure-token-at-least-24-chars"
  7. export CAMERA_URL="ws://127.0.0.1:8767"
  8. export NAV2_BRIDGE_URL="ws://<NAV2_PI_IP>:8770"
  9. export ARM101_BRIDGE_URL="ws://<ARM_PI_IP>:8771"

Start Camera Service:
  python run_camera_service.py

Monitor in another terminal:
  python system_orchestrator.py --mode status


MACHINE 2: NAV2 PI (Secondary)
────────────────────────────────

Hardware:
  - Raspberry Pi 4+ (4GB+)
  - Connected to same network
  - Running Nav2 stack (ROS2 installation)
  - Control electronics for motors

Installation:
  1. Copy nav2_bridge_server.py to ~/nav2_bridge_server.py
  2. Install: pip install websockets pydantic
  3. export ROBOT_TOKEN="same-token-as-camera-pi"

Start Bridge Server:
  python ~/nav2_bridge_server.py --host 0.0.0.0 --port 8770

CUSTOMIZE FOR YOUR NAV2:
  - Edit nav2_bridge_server.py, line 45: Nav2Controller.get_current_pose()
  - Import your ROS2 action clients
  - Replace simulated navigation with real Nav2 calls
  - Connect to your TF tree for pose estimation


MACHINE 3: ARM101 PI (Secondary)
──────────────────────────────────

Hardware:
  - Raspberry Pi 4+ (4GB+)
  - Connected to same network  
  - ARM101 control board (serial/USB)
  - Gripper electronics

Installation:
  1. Copy arm101_bridge_server.py to ~/arm101_bridge_server.py
  2. Install: pip install websockets pydantic
  3. export ROBOT_TOKEN="same-token-as-camera-pi"

Start Bridge Server:
  python ~/arm101_bridge_server.py --host 0.0.0.0 --port 8771

CUSTOMIZE FOR YOUR ARM:
  - Edit arm101_bridge_server.py, line 56: Arm101Controller.initialize()
  - Import your ARM101 serial/USB driver
  - Replace simulated grasp with real hardware commands
  - Connect force/weight sensors for verification
  - Define grasp strategies for different objects


CONFIGURATION (CAMERA PI):
=========================

Environment Variables (set before running):

  ROBOT_TOKEN
    - Long secure token (24+ chars) shared by all Pis
    - Used for WebSocket authentication
    - Example: export ROBOT_TOKEN="bot-secret-key-at-least-24-chars-long"
  
  CAMERA_URL
    - WebSocket URL of camera service
    - Default: ws://127.0.0.1:8767
    - Change first part to Pi's IP for remote access
  
  NAV2_BRIDGE_URL
    - WebSocket URL of Nav2 bridge on Navigation Pi
    - Example: export NAV2_BRIDGE_URL="ws://192.168.1.102:8770"
  
  ARM101_BRIDGE_URL
    - WebSocket URL of Arm101 bridge on Arm Pi
    - Example: export ARM101_BRIDGE_URL="ws://192.168.1.103:8771"

Test Configuration:
  python system_orchestrator.py --mode status


TESTING WORKFLOW:
================

PHASE 1: INDIVIDUAL SERVICE TESTS
──────────────────────────────────

1. Test Camera Service Alone:
   Terminal 1 (Camera Pi):
     python run_camera_service.py
   
   Terminal 2 (Camera Pi):
     python -c "
       from robot_app.camera import Camera
       import asyncio
       async def test():
           cam = Camera()
           result = await cam.locate('bottle')
           print(result)
       asyncio.run(test())
     "
   Expected: Detection result with 3D pose

2. Test Nav2 Bridge Alone:
   Terminal 1 (Nav2 Pi):
     python nav2_bridge_server.py --host 0.0.0.0 --port 8770
   
   Terminal 2 (Camera Pi):
     python -c "
       from robot_app.hardware_integrations import Nav2Bridge
       import asyncio
       async def test():
           nav2 = Nav2Bridge('ws://192.168.1.102:8770')
           result = await nav2.call('save_start')
           print(result)
       asyncio.run(test())
     "
   Expected: Current pose saved

3. Test Arm101 Bridge Alone:
   Terminal 1 (Arm Pi):
     python arm101_bridge_server.py --host 0.0.0.0 --port 8771
   
   Terminal 2 (Camera Pi):
     python -c "
       from robot_app.hardware_integrations import Arm101Bridge
       import asyncio
       async def test():
           arm = Arm101Bridge('ws://192.168.1.103:8771')
           result = await arm.call('stow')
           print(result)
       asyncio.run(test())
     "
   Expected: Arm stowed confirmation

PHASE 2: COORDINATED SEQUENCE TEST
───────────────────────────────────

Camera Pi only (all bridges simulated):
  python system_orchestrator.py --mode complete --target "bottle"

Expected output:
  - STEP 1: save_start → Camera service saves pose
  - STEP 2: locate → Camera detects bottle
  - STEP 3: approach → Simulated navigation
  - STEP 2b: locate → Reobserve
  - STEP 5: grasp → Simulated grasp
  - STEP 6: verify_grasp → Verify successful
  - STEP 7: stow → Arm stowed
  - STEP 8: return_start → Returned home

PHASE 3: FULL SYSTEM TEST
─────────────────────────

All three Pis running and connected:

1. Start Nav2 Bridge (Nav2 Pi):
   python nav2_bridge_server.py --host 0.0.0.0 --port 8770

2. Start Arm101 Bridge (Arm Pi):
   python arm101_bridge_server.py --host 0.0.0.0 --port 8771

3. Start Camera Service (Camera Pi):
   python run_camera_service.py

4. Run Coordinator (Camera Pi, new terminal):
   python system_orchestrator.py --mode complete --target "bottle"

Monitor all services:
   python system_orchestrator.py --mode status


TROUBLESHOOTING:
================

❌ Services offline
  → Check network: ping <PI_IP>
  → Check tokens match across all machines
  → Check firewall isn't blocking WebSocket ports

❌ Camera detection not working
  → Verify RealSense connected: lsusb
  → Check YOLO-World model file exists: ls yolov8s-worldv2.pt
  → Test camera: python run_camera_opencv.py

❌ Navigation fails
  → Check Nav2 stack running: ros2 topic list | grep nav
  → Verify TF tree available: ros2 run tf2_tools view_frames
  → Test manually: ros2 action send_goal /navigate_to_pose ...

❌ Arm not responding
  → Check serial connection: ls /dev/ttyUSB*
  → Verify Arm101 board powered
  → Test manual command: python -c "import serial; s = serial.Serial(...)"

❌ WebSocket connection refused
  → Check bridge server running on correct Pi
  → Verify port not blocked by firewall
  → Check token in ROBOT_TOKEN matches


MONITORING & LOGGING:
====================

Real-time monitoring:
  python system_orchestrator.py --mode status

Full execution log:
  python system_orchestrator.py --mode complete --target "bottle" 2>&1 | tee execution.log

Debug mode (more verbose):
  export DEBUG=1
  python system_orchestrator.py --mode complete --target "bottle"

Individual service logs:
  Camera Pi: grep "SIGNAL" run_camera_service.log
  Nav2 Pi:   grep "\\[SIGNAL\\]\\|\\[NAV2\\]" nav2.log
  Arm Pi:    grep "\\[SIGNAL\\]\\|\\[ARM101\\]" arm.log


ADVANCED: CUSTOM GRASP STRATEGIES
=================================

Each object can have custom grasp parameters. In arm101_bridge_server.py:

    self.grasp_strategies = {
        "bottle": {"grip_force": 50, "approach_height": 0.1, "depth": 0.15},
        "cup": {"grip_force": 40, "approach_height": 0.08, "depth": 0.12},
        "book": {"grip_force": 60, "approach_height": 0.05, "depth": 0.20},
        "custom_object": {...}
    }

Customize for your end-effector hardware and object shapes.


FILES REFERENCE:
================

CAMERA PI (Primary):
  system_orchestrator.py      ← Master orchestration (THIS)
  run_camera_service.py       ← Start camera service
  robot_app/coordinator.py    ← 8-step fetch sequence
  robot_app/camera.py         ← RealSense + YOLO integration
  robot_app/protocol.py       ← WebSocket JSON protocol
  robot_app/safety.py         ← Watchdog & emergency stop
  
NAV2 PI (Secondary):
  nav2_bridge_server.py       ← Navigation bridge (CUSTOMIZE)
  
ARM101 PI (Secondary):
  arm101_bridge_server.py     ← Arm grasp bridge (CUSTOMIZE)


NEXT STEPS:
===========

1. ✓ Understand the architecture (this guide)
2. ✓ Set up physical hardware on 3 Pis
3. ✓ Configure environment variables & tokens
4. ✓ Customize nav2_bridge_server.py for your Nav2 stack
5. ✓ Customize arm101_bridge_server.py for your Arm101 hardware
6. ✓ Test individual services
7. ✓ Run full system integration test
8. ✓ Deploy to production

Questions? See README.md for detailed architecture info.
"""

# Print guide when run as main
if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--view":
        print(__doc__)
    else:
        # Otherwise, this is imported as a module
        pass
