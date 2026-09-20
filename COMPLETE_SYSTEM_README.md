# VLA-HTN: Complete Robot Integration System

**Vision → Navigation → Manipulation on Distributed Raspberry Pi Network**

This is the complete implementation of a distributed robot system that:
1. **Detects objects** using RealSense camera + YOLO-World vision AI
2. **Navigates** to detected objects using Nav2 on a separate Pi
3. **Grasps objects** using Arm101 hardware on a third Pi  
4. **Returns home** after completing the task

## Quick Start (5 minutes)

### For Testing with Simulated Hardware:
```bash
# On your desktop/camera Pi
python system_orchestrator.py --mode complete --target "bottle"
```

Expected output:
```
STEP 1: save_start → Navigation saves current position
STEP 2: locate → Camera detects bottle 
STEP 3: approach → Robot navigates to bottle location
STEP 2b: locate → Camera reobserves after moving
STEP 5: grasp → Arm executes grasp motion
STEP 6: verify_grasp → Confirms object is held
STEP 7: stow → Arm puts object safely away
STEP 8: return_start → Robot returns to starting position
✓ COMPLETE
```

### For Real Hardware (3 Raspberry Pis):

**Pi 1 (Camera):**
```bash
export ROBOT_TOKEN="your-secure-token-at-least-24-chars"
export NAV2_BRIDGE_URL="ws://192.168.1.102:8770"
export ARM101_BRIDGE_URL="ws://192.168.1.103:8771"

python run_camera_service.py  # Terminal 1
python system_orchestrator.py --mode complete --target "bottle"  # Terminal 2
```

**Pi 2 (Navigation):**
```bash
export ROBOT_TOKEN="your-secure-token-at-least-24-chars"  # MUST MATCH
python nav2_bridge_server.py --host 0.0.0.0 --port 8770
```

**Pi 3 (Manipulation):**
```bash
export ROBOT_TOKEN="your-secure-token-at-least-24-chars"  # MUST MATCH
python arm101_bridge_server.py --host 0.0.0.0 --port 8771
```

---

## System Architecture

```
                    CAMERA PI (Primary)
                    ===================
         
         ┌────────────────────────────┐
         │   COORDINATOR              │
         │   (8-step fetch sequence)  │
         └────────┬───────────────────┘
                  │
         ┌────────┴────────┬──────────────┐
         │                 │              │
    STEP 2,2b:         STEP 1,3,8:     STEP 5,6,7:
      locate          navigate to       grasp 
      (CAMERA)        object (NAV2)    object (ARM)
         │                 │              │
    ┌────▼────┐      ┌─────▼────┐    ┌───▼──────┐
    │ YOLO    │      │ Nav2 Pi  │    │ Arm Pi   │
    │ World   │      │ Bridge   │    │ Bridge   │
    │ Detection      │ Server   │    │ Server   │
    │ (8767)  │      │ (8770)   │    │ (8771)   │
    └─────────┘      └──────────┘    └──────────┘
         │                 │              │
    Intel RealSense  ROS2 Nav2 Stack  Arm101 Hardware
    USB Camera       on Pi 2           on Pi 3
```

## Files Overview

### Core System Files
- **system_orchestrator.py** - Master control: starts/monitors all services
- **test_system_integration.py** - End-to-end test suite
- **deployment_checklist.py** - Verify deployment readiness
- **SYSTEM_INTEGRATION_GUIDE.py** - Detailed deployment instructions

### Robot Application (`robot_app/`)
- **coordinator.py** - 8-step fetch sequence orchestrator
- **camera.py** - RealSense + YOLO-World object detection
- **hardware_integrations.py** - Bridge clients for Nav2/Arm101
- **protocol.py** - WebSocket JSON-RPC protocol definition
- **safety.py** - Watchdog timer + emergency stop
- **task_decomposer.py** - Optional: VLA task planning

### Bridge Servers (Customize for your hardware)
- **nav2_bridge_server.py** - Run on Navigation Pi
- **arm101_bridge_server.py** - Run on Arm101 Pi

### Service Runners
- **run_camera_service.py** - Start camera detection service

---

## The Signal Flow: How Vision → Navigation → Manipulation Works

### Complete Sequence

```
USER VOICE INPUT:
  "grab a bottle"
       ↓
STEP 1: save_start → NAV2_BRIDGE
  "Remember this starting position"
  WebSocket: {action: "save_start"}
  Response: {ok: true, result: {pose: {x:0, y:0, z:0}}}
       ↓
STEP 2: locate → CAMERA_SERVICE (same machine)
  "Where is a bottle?"
  WebSocket: {action: "locate", target: "bottle"}
  Response: {ok: true, result: {pose: {x:1.2, y:0.5, z:0.7}, confidence: 0.92}}
       ↓
STEP 3: approach → NAV2_BRIDGE (SIGNAL from vision)
  "Navigate to x:1.2, y:0.5" ← POSE FROM VISION
  WebSocket: {action: "approach", pose: {x:1.2, y:0.5, z:0.7}}
  Response: {ok: true, result: {completed: true}}
       ↓
STEP 2b: locate → CAMERA_SERVICE
  "Reobserve bottle from new location (never grasp from old image)"
  Response: {ok: true, result: {pose: {x:1.2, y:0.5, z:0.7, ...}}}
       ↓
STEP 5: grasp → ARM101_BRIDGE (CRITICAL SIGNAL from BOTH vision + navigation)
  "Grasp bottle at x:1.2, y:0.5" ← OBJECT FROM VISION, POSE FROM NAVIGATION
  WebSocket: {
    action: "grasp",
    target: "bottle",              ← DETECTED BY VISION
    pose: {x:1.2, y:0.5, z:0.7}  ← NAVIGATED BY NAV2
  }
  Response: {ok: true, result: {held: true}}
       ↓
STEP 6: verify_grasp → ARM101_BRIDGE
  "Check if object is being held"
  Response: {ok: true, result: {held: true}}
       ↓
STEP 7: stow → ARM101_BRIDGE
  "Put arm away safely with object"
  Response: {ok: true, result: {completed: true}}
       ↓
STEP 8: return_start → NAV2_BRIDGE
  "Return to starting position"
  WebSocket: {action: "return_start", pose: {x:0, y:0, z:0}}
  Response: {ok: true, result: {completed: true}}
       ↓
DONE ✓
Object retrieved and back at starting position!
```

## Key Insight: The Three Signals

This system successfully integrates three separate Pi services through **three key signals**:

1. **Vision Signal** (YOLO Detection)
   - Camera service detects object
   - Returns 3D pose in map frame
   - Used by: Navigation (where to go) + Arm (what to grasp)

2. **Navigation Signal** (Nav2 Movement)
   - Nav2 navigates to detected location
   - Arm knows "robot is at detection spot"
   - No need for separate object re-detection at arm level

3. **Manipulation Signal** (Combined)
   - Arm receives BOTH:
     - Object class from vision ("bottle", "cup", etc.)
     - Position from navigation (x, y, z)
   - Uses both to execute appropriate grasp strategy

This three-signal integration is the core of the complete system.

---

## Installation & Setup

### Prerequisites
- 3x Raspberry Pi 4+ (8GB+ recommended)
- Intel RealSense D435i camera
- Arm101 control board + gripper
- Nav2 stack (ROS2) on Navigation Pi
- Network connection between all Pis

### Camera Pi Setup
```bash
cd ~/VLA-HTN
python3 -m venv venv
source venv/bin/activate

# Install dependencies
pip install websockets pydantic ultralytics opencv-python numpy pillow

# Optional: Real RealSense support (requires build tools)
pip install pyrealsense2

# Set security token
export ROBOT_TOKEN="your-long-secure-token-minimum-24-characters"
```

### Nav2 Pi Setup
```bash
pip install websockets pydantic

# Copy bridge server from camera Pi
scp user@camera_pi:/path/to/nav2_bridge_server.py .

# IMPORTANT: Edit nav2_bridge_server.py
# - Line 45-50: Nav2Controller.get_current_pose()
# - Line 52-65: Nav2Controller.navigate_to_pose()
# - Connect to your actual Nav2 action servers

export ROBOT_TOKEN="same-token-as-camera-pi"
python nav2_bridge_server.py --host 0.0.0.0 --port 8770
```

### Arm101 Pi Setup
```bash
pip install websockets pydantic

# Copy bridge server from camera Pi
scp user@camera_pi:/path/to/arm101_bridge_server.py .

# IMPORTANT: Edit arm101_bridge_server.py
# - Line 60-65: Arm101Controller.initialize()
# - Line 77: execute_grasp() - connect to arm hardware
# - Line 98: verify_grasp() - read force/weight sensors
# - Line 107: execute_stow() - home position

export ROBOT_TOKEN="same-token-as-camera-pi"
python arm101_bridge_server.py --host 0.0.0.0 --port 8771
```

---

## Testing

### Test Individual Services
```bash
# Test vision detection only
python test_system_integration.py --step vision

# Test navigation only
python test_system_integration.py --step navigation

# Test manipulation only
python test_system_integration.py --step manipulation
```

### Test Complete Sequence
```bash
# With simulated hardware
python test_system_integration.py --full

# With real hardware (all Pis running)
python test_system_integration.py --full --target "cup"
```

### Monitor System Health
```bash
python system_orchestrator.py --mode status
```

---

## Customization

### For Your Vision Model
In `robot_app/camera.py`:
```python
# Change detection model
detector = YoloWorldDetector(model="yolov8m-worldv2.pt")  # Larger model

# Add custom object classes
detector.set_classes(["bottle", "cup", "book", "phone", "custom_obj"])
```

### For Your Nav2 Stack
In `nav2_bridge_server.py`:
```python
async def get_current_pose(self):
    # Query your actual Nav2 TF tree
    from tf2_ros import TransformListener
    transform = tf_buffer.lookup_transform('map', 'base_footprint', ...)
    return extract_pose(transform)

async def navigate_to_pose(self, target_pose, timeout):
    # Use your Nav2 action client
    from nav2_simple_commander.robot_navigator import BasicNavigator
    navigator.goToPose(create_pose_stamped(...))
```

### For Your Arm Hardware
In `arm101_bridge_server.py`:
```python
async def execute_grasp(self, target, pose):
    # Connect to your arm control interface
    strategy = self.grasp_strategies.get(target)
    
    # Execute grasp with strategy
    await self.arm_client.grasp(
        object_class=target,
        pose=pose,
        grip_force=strategy['grip_force']
    )
```

---

## Troubleshooting

### Services Offline
```bash
# Check each Pi is reachable
ping 192.168.1.102  # Nav2 Pi
ping 192.168.1.103  # Arm Pi

# Check services are running
ssh user@192.168.1.102 "ps aux | grep bridge"
```

### Vision Detection Fails
```bash
# Check RealSense is connected
lsusb | grep RealSense

# Test camera directly
python -c "
from robot_app.camera import Camera
import asyncio
async def test():
    cam = Camera()
    result = await cam.locate('bottle')
    print(result)
asyncio.run(test())
"
```

### Navigation Fails
```bash
# Check Nav2 is running
ros2 topic list | grep nav

# Test bridge connection
python -c "
from robot_app.hardware_integrations import Nav2Bridge
import asyncio
async def test():
    nav = Nav2Bridge('ws://192.168.1.102:8770')
    result = await nav.call('save_start')
    print(result)
asyncio.run(test())
"
```

### Arm Not Responding
```bash
# Check arm connection
ls /dev/ttyUSB*

# Test bridge connection
python -c "
from robot_app.hardware_integrations import Arm101Bridge
import asyncio
async def test():
    arm = Arm101Bridge('ws://192.168.1.103:8771')
    result = await arm.call('stow')
    print(result)
asyncio.run(test())
"
```

---

## Development & Debugging

### View Complete Signal Flow
```bash
python system_orchestrator.py --mode complete --target "bottle" 2>&1 | grep "\[SIGNAL\]"
```

### Run with Debug Output
```bash
DEBUG=1 python system_orchestrator.py --mode complete --target "bottle"
```

### Interactive Testing
```bash
python system_orchestrator.py --mode interactive
```

---

## Performance

### Typical Latencies
- Vision detection: 0.5-1.5 seconds
- Navigation: 5-30 seconds (depends on distance)
- Grasp execution: 1-3 seconds
- Total task time: 8-35 seconds

### Optimization Tips
- Use smaller YOLO-World model for faster detection
- Pre-compute grasp strategies for common objects
- Implement object prediction (anticipatory grasping)
- Add parallel task execution where possible

---

## Security

All WebSocket connections use Bearer token authentication:
```
Authorization: Bearer <ROBOT_TOKEN>
```

Required:
- Token must be 24+ characters
- Same on all Pis
- Store in `ROBOT_TOKEN` environment variable
- Never commit to git

Example:
```bash
export ROBOT_TOKEN="vla-htn-bot-secret-key-minimum-24-chars-required"
```

---

## Documentation

- **SYSTEM_INTEGRATION_GUIDE.py** - Complete deployment guide
- **deployment_checklist.py** - Verify setup readiness
- **system_orchestrator.py** - Master orchestrator
- **test_system_integration.py** - Test suite

---

## Contributing

To add new capabilities:
1. Extend `robot_app/coordinator.py` for new steps
2. Add handlers in bridge servers (nav2_bridge_server.py, arm101_bridge_server.py)
3. Add tests to `test_system_integration.py`
4. Update documentation

---

## License & Attribution

This is a research/educational project for distributed robotic systems integration.

---

## Next Steps

1. **Read the guides:**
   - SYSTEM_INTEGRATION_GUIDE.py - Detailed architecture
   - deployment_checklist.py - Verify readiness

2. **Test locally:**
   - python system_orchestrator.py --mode complete --target "bottle"

3. **Deploy to Pis:**
   - Run deployment_checklist.py
   - Customize bridge servers
   - Test each service individually
   - Run full end-to-end test

4. **Customize for your hardware:**
   - Nav2: Edit nav2_bridge_server.py
   - Arm: Edit arm101_bridge_server.py
   - Vision: Adjust detection model in robot_app/camera.py

---

**Questions?** Check the detailed SYSTEM_INTEGRATION_GUIDE.py or run:
```bash
python system_orchestrator.py --mode status
```
