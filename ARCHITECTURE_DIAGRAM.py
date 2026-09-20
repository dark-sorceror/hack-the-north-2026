#!/usr/bin/env python3
"""
END-TO-END PIPELINE ARCHITECTURE
VLA System: Voice → Detection → Navigation → Grasp

What's Implemented:
  ✓ Protocol (WebSocket, authentication)
  ✓ Coordinator orchestration
  ✓ Camera service with YOLO
  ✓ Hardware service (bridge routing)
  ✓ Voice service (Qwen integration)

What Needs Setup:
  ✗ Nav2 Bridge Server (on navigation Pi)
  ✗ Arm 101 Bridge Server (on arm Pi)
  ✗ pyrealsense2 SDK (on camera Pi)
  
"""

ARCHITECTURE = r"""
╔═══════════════════════════════════════════════════════════════════════════════╗
║                           DISTRIBUTED VLA SYSTEM                             ║
║                    3-4 Raspberry Pi Components                                ║
╚═══════════════════════════════════════════════════════════════════════════════╝

COMPONENT 1: CONTROLLER PI (Main Orchestrator)
  ┌─────────────────────────────────────────────────────────────────┐
  │ robot_app.voice.Audio                                           │
  │   - Listen via USB Jabra mic                                    │
  │   - Send to Qwen cloud API (realtime)                           │
  │   - Receive tool calls: fetch_object, stop_robot               │
  └─────────────────────────────────────────────────────────────────┘
              │
              │ (tool call) fetch_object(object_name="bottle")
              │
  ┌─────────────────────────────────────────────────────────────────┐
  │ robot_app.coordinator.Coordinator                              │
  │   - Orchestrates the 7-step fetch sequence                     │
  │   - Routes to: camera_pi, nav2_pi, arm101_pi                   │
  └─────────────────────────────────────────────────────────────────┘
              │
              ├─────────────────────────────────────────────┐
              │                                             │
              ▼                                             ▼
  
  
COMPONENT 2: CAMERA PI (Vision & Detection)
  ┌─────────────────────────────────────────────────────────────────┐
  │ robot_app.camera.CameraService (WebSocket on 8767)             │
  │   - Listens for "locate" requests                              │
  │   - Captures from Intel RealSense D435i (/dev/video0)          │
  │   - Runs YOLO-World v8s for object detection                   │
  │   - Returns: {pose, confidence, observed_at}                   │
  │                                                                 │
  │   Request: {"action": "locate", "target": "bottle"}            │
  │   Response: {                                                   │
  │     "pose": {"frame": "map", "x": 1.2, "y": 0.5, "z": 0.7},  │
  │     "confidence": 0.92,                                         │
  │     "observed_at": <timestamp>                                 │
  │   }                                                             │
  └─────────────────────────────────────────────────────────────────┘
  

COMPONENT 3: NAVIGATION PI (Nav2)
  ┌─────────────────────────────────────────────────────────────────┐
  │ Nav2BridgeServer (Custom WebSocket on 8770)                    │
  │   - Listens for: save_start, approach, return_start            │
  │   - Routes to Nav2 stack on real hardware                      │
  │                                                                 │
  │   Requests:                                                    │
  │   - save_start: {} → {"pose": starting_pose}                   │
  │   - approach:   {"pose": target_pose} → {"completed": true}   │
  │   - return_start: {"pose": home_pose} → {"completed": true}   │
  └─────────────────────────────────────────────────────────────────┘
  

COMPONENT 4: ARM 101 PI (Manipulation)
  ┌─────────────────────────────────────────────────────────────────┐
  │ Arm101BridgeServer (Custom WebSocket on 8771)                  │
  │   - Listens for: grasp, verify_grasp, stow                     │
  │   - Routes to Arm101 real hardware                             │
  │                                                                 │
  │   Requests:                                                    │
  │   - grasp: {"target": "bottle", "pose": {...}} → {}            │
  │   - verify_grasp: {} → {"held": true}                          │
  │   - stow: {} → {"completed": true}                             │
  └─────────────────────────────────────────────────────────────────┘


╔═══════════════════════════════════════════════════════════════════════════════╗
║                        THE FETCH SEQUENCE (Step-by-Step)                      ║
╚═══════════════════════════════════════════════════════════════════════════════╝

USER: "Hey robot, fetch the bottle"
  │
  └─→ Voice Service (Qwen) decodes → tool_call(fetch_object, object="bottle")
      │
      └─→ Coordinator.fetch("bottle")
          │
          STEP 1: save_start (NAV2)
          ├─→ Request: {"action": "save_start"}
          ├─→ Nav2Bridge responds: {"pose": {x:0, y:0, z:0, frame:"map"}}
          └─→ Coordinator remembers: home_pose = (0, 0, 0)
          
          STEP 2: locate (CAMERA)
          ├─→ Request: {"action": "locate", "target": "bottle"}
          ├─→ CameraService:
          │   ├─ Captures frame from RealSense
          │   ├─ Runs YOLO detect("bottle") → bounding box
          │   ├─ Converts pixel → camera 3D point
          │   └─ Returns: {pose: {x:1.2, y:0.5, z:0.7}, confidence: 0.92}
          └─→ Coordinator validates: confidence ≥ 0.7 ✓, age < 2s ✓
          
          STEP 3: approach (NAV2) ← THIS IS THE KEY SIGNAL TO ARM
          ├─→ Request: {"action": "approach", "pose": {x:1.2, y:0.5, z:0.7}}
          │   ↳ This says: "Navigate to where the bottle was detected"
          ├─→ Nav2Bridge moves robot to (x:1.2, y:0.5, z:0.7)
          └─→ Nav2Bridge responds: {"completed": true}
          
          STEP 4: locate (CAMERA) - RE-VERIFY
          ├─→ Request: {"action": "locate", "target": "bottle"}
          │   ↳ After moving, re-detect to get fresh observation
          ├─→ CameraService captures again
          └─→ Returns updated pose (should be closer now)
          
          STEP 5: grasp (ARM101) ← ARM RECEIVES THE SIGNAL + POSE
          ├─→ Request: {"action": "grasp", "target": "bottle", "pose": {...}}
          │   ↳ The ARM NOW KNOWS:
          │      - Object is "bottle" (detected by vision)
          │      - We navigated to pose (x:1.2, y:0.5, z:0.7)
          │      - Object should be at that location
          │      - Time to grasp
          ├─→ Arm101Bridge executes grasp motion
          └─→ Arm101Bridge responds: {"completed": true}
          
          STEP 6: verify_grasp (ARM101)
          ├─→ Request: {"action": "verify_grasp"}
          ├─→ Arm101Bridge returns: {"held": true}
          └─→ Coordinator checks: if not held, ABORT with error
          
          STEP 7: stow (ARM101)
          ├─→ Request: {"action": "stow"}
          └─→ Arm101Bridge tucks arm away
          
          STEP 8: return_start (NAV2)
          ├─→ Request: {"action": "return_start", "pose": home_pose}
          └─→ Nav2Bridge navigates back to (0, 0, 0)
          
          ✓ TASK COMPLETE: Robot fetched the bottle and returned home


╔═══════════════════════════════════════════════════════════════════════════════╗
║                      WHAT YOU NEED TO BUILD                                  ║
╚═══════════════════════════════════════════════════════════════════════════════╝

PART 1: NAV2 BRIDGE SERVER (nav2_pi.local)
─────────────────────────────────────────────

This runs on a Raspberry Pi with Nav2 stack installed.
It listens for WebSocket connections and routes commands to Nav2.

Location: /path/to/nav2_pi/nav2_bridge_server.py

Template:

    import asyncio
    import json
    from websockets.asyncio.server import serve
    from robot_app.protocol import authenticated
    
    async def handle_nav2_command(ws, path):
        if not authenticated(ws):
            await ws.close(1008, "Unauthorized")
            return
        
        try:
            message = json.loads(await ws.recv())
            action = message.get("action")
            
            if action == "save_start":
                # Get current pose from Nav2
                current_pose = get_nav2_current_pose()
                await ws.send(json.dumps({
                    "ok": True,
                    "result": {"pose": current_pose.to_dict()}
                }))
            
            elif action == "approach":
                # Send goal to Nav2
                target_pose = message.get("pose")
                nav2_result = await send_nav2_goal(target_pose)
                await ws.send(json.dumps({
                    "ok": True,
                    "result": {"completed": True}
                }))
            
            elif action == "return_start":
                # Navigate back to home
                home_pose = message.get("pose")
                await send_nav2_goal(home_pose)
                await ws.send(json.dumps({
                    "ok": True,
                    "result": {"completed": True}
                }))
        
        except Exception as e:
            await ws.send(json.dumps({"ok": False, "error": str(e)}))
    
    async def main():
        async with serve(handle_nav2_command, "0.0.0.0", 8770,
                        additional_headers={"Authorization": "Bearer ..."}):
            print("Nav2 Bridge listening on :8770")
            await asyncio.Future()
    
    asyncio.run(main())


PART 2: ARM 101 BRIDGE SERVER (arm101_pi.local)
─────────────────────────────────────────────────

This runs on a Raspberry Pi with Arm 101 hardware.
It listens for WebSocket connections and routes commands to the arm.

Location: /path/to/arm101_pi/arm101_bridge_server.py

Template:

    import asyncio
    import json
    from websockets.asyncio.server import serve
    from robot_app.protocol import authenticated
    
    async def handle_arm_command(ws, path):
        if not authenticated(ws):
            await ws.close(1008, "Unauthorized")
            return
        
        try:
            message = json.loads(await ws.recv())
            action = message.get("action")
            
            if action == "grasp":
                # THE KEY SIGNAL: Coordinator tells us what to grasp
                target = message.get("target")  # "bottle", "cup", etc.
                pose = message.get("pose")       # Where nav2 navigated to
                
                print(f"Received signal: GRASP detected '{target}' at pose {pose}")
                # Now execute grasp with arm hardware
                arm_result = await execute_grasp(target, pose)
                await ws.send(json.dumps({
                    "ok": True,
                    "result": {"completed": True}
                }))
            
            elif action == "verify_grasp":
                # Check if object is held (sensor feedback)
                held = arm_held_object()
                await ws.send(json.dumps({
                    "ok": True,
                    "result": {"held": held}
                }))
            
            elif action == "stow":
                # Put arm away
                await execute_stow()
                await ws.send(json.dumps({
                    "ok": True,
                    "result": {"completed": True}
                }))
        
        except Exception as e:
            await ws.send(json.dumps({"ok": False, "error": str(e)}))
    
    async def main():
        async with serve(handle_arm_command, "0.0.0.0", 8771,
                        additional_headers={"Authorization": "Bearer ..."}):
            print("Arm 101 Bridge listening on :8771")
            await asyncio.Future()
    
    asyncio.run(main())


╔═══════════════════════════════════════════════════════════════════════════════╗
║                         TESTING THE FLOW                                      ║
╚═══════════════════════════════════════════════════════════════════════════════╝

1. Start simulated mode (no hardware):
   
   $ export ROBOT_TOKEN='your-24-char-secret'
   $ python -m robot_app.vla_integration_test
   
   Expected: E2E in ~2 seconds, all steps succeed

2. Start with one real Pi (camera):
   
   On camera Pi:
   $ export ROBOT_TOKEN='your-24-char-secret'
   $ python -m robot_app.camera --host 0.0.0.0 --port 8767
   
   On controller:
   $ export ROBOT_TOKEN='your-24-char-secret'
   $ export CAMERA_URL='ws://gisoopi.local:8767'
   $ python -c "import asyncio; from robot_app.protocol import rpc, Request; print(asyncio.run(rpc('ws://gisoopi.local:8767', Request(action='locate', target='bottle'))))"
   
   Expected: Detection results with pose, confidence

3. Full end-to-end (all components):
   
   On camera Pi:
   $ python -m robot_app.camera --host 0.0.0.0 --port 8767
   
   On nav2 Pi:
   $ python nav2_bridge_server.py
   
   On arm101 Pi:
   $ python arm101_bridge_server.py
   
   On controller:
   $ export ROBOT_TOKEN='your-24-char-secret'
   $ export CAMERA_URL='ws://gisoopi.local:8767'
   $ export NAV2_BRIDGE_URL='ws://nav2_pi:8770'
   $ export ARM101_BRIDGE_URL='ws://arm101_pi:8771'
   $ python -m robot_app.coordinator --target bottle
   
   Expected: Full fetch sequence, arm receives signal to grasp


╔═══════════════════════════════════════════════════════════════════════════════╗
║                    SUMMARY: What's Already Done vs TODO                       ║
╚═══════════════════════════════════════════════════════════════════════════════╝

✓ DONE: Coordinator logic (knows to call camera, nav2, arm in sequence)
✓ DONE: Camera service (runs YOLO, returns pose + confidence)
✓ DONE: Voice service (listens to Qwen, calls coordinator)
✓ DONE: Protocol (WebSocket, auth, schemas)
✓ DONE: Safety validation (watchdog, emergency stop)

✗ TODO: Nav2BridgeServer (you need to build this for your Nav2 Pi)
✗ TODO: Arm101BridgeServer (you need to build this for your Arm 101 Pi)
✗ TODO: Install pyrealsense2 on camera Pi
✗ TODO: Test on actual hardware

The signal flow from vision to arm is: 
  Coordinator → Camera (detect) → Nav2 (approach) → Arm (grasp with target+pose)

The arm receives BOTH the object name AND the navigation pose from the coordinator.
"""

if __name__ == "__main__":
    print(ARCHITECTURE)
