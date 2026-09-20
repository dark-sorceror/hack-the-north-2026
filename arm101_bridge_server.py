"""
Arm 101 Bridge Server - Run on your Arm 101 Control Pi

This server listens for grasp commands via WebSocket from the coordinator.
It receives the detected object name AND the navigation pose as signals.

KEY SIGNAL FLOW:
  Coordinator calls: grasp(target="bottle", pose={x:1.2, y:0.5, z:0.7})
  This tells the arm:
    1. We detected a "bottle" using vision
    2. We navigated to position (x:1.2, y:0.5, z:0.7)
    3. Now grasp the object at that location

SETUP:
  1. Copy this file to your arm101_pi
  2. Install: pip install websockets pydantic
  3. Configure ARM101_HARDWARE_PORT, ARM101_GRIPPER_PIN, etc.
  4. Run: python arm101_bridge_server.py
  
EXPECTED BEHAVIOR:
  - Listens on 0.0.0.0:8771
  - Authenticates all requests with Bearer token
  - Routes grasp, verify_grasp, stow to your arm hardware
"""

import argparse
import asyncio
import json
import logging
import os
import time
from typing import Optional

from websockets.asyncio.server import serve
from websockets.exceptions import ConnectionClosed

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def get_token():
    """Get authentication token from environment."""
    token = os.getenv("ROBOT_TOKEN", "").strip()
    if not token or len(token) < 24:
        raise ValueError("ROBOT_TOKEN must be set to at least 24 characters")
    return token


def authenticated(ws):
    """Check WebSocket has valid Bearer token."""
    auth = ws.request.headers.get("Authorization", "").split()
    if len(auth) != 2 or auth[0] != "Bearer":
        return False
    return auth[1] == get_token()


class Arm101Controller:
    """Interface to your Arm 101 hardware (customize for your setup)."""
    
    def __init__(self):
        # TODO: Initialize your hardware interfaces
        self.gripper_pin = None  # GPIO or serial port
        self.arm_client = None   # Your arm control interface
        self.held_object = False
        self.last_grasp_target = None
        self.grasp_history = []
        self.simulated = True  # Set to False when real hardware is connected
        
        # Grasp strategies for different object types
        self.grasp_strategies = {
            "bottle": {"grip_force": 50, "approach_height": 0.1, "depth": 0.15},
            "cup": {"grip_force": 40, "approach_height": 0.08, "depth": 0.12},
            "book": {"grip_force": 60, "approach_height": 0.05, "depth": 0.20},
            "default": {"grip_force": 45, "approach_height": 0.10, "depth": 0.15},
        }
        
    async def initialize(self):
        """Initialize arm hardware connections."""
        # TODO: Connect to Arm 101 serial/network interface
        # Example (serial):
        # import serial
        # self.arm_client = serial.Serial('/dev/ttyUSB0', baudrate=115200)
        
        # Example (ROS action client):
        # from rclpy.action import ActionClient
        # from arm101_interfaces.action import GraspObject
        # self.arm_client = ActionClient(node, GraspObject, 'grasp')
        
        logger.info("[ARM101] Controller initialized (SIMULATED MODE)")
        logger.info("[ARM101] To connect real hardware: customize initialize() in arm101_bridge_server.py")
    
    def _get_strategy(self, target: str):
        """Get grasp strategy for target object."""
        return self.grasp_strategies.get(target.lower(), self.grasp_strategies["default"])
    
    async def execute_grasp(self, target: str, pose: dict) -> bool:
        """
        Execute grasp motion with full signal integration.
        
        SIGNALS FROM SYSTEM:
          - target: What was detected by vision ("bottle", "cup", etc.)
          - pose: Where nav2 navigated to (x, y, z in map frame)
        
        THIS IS THE CRITICAL INTEGRATION POINT:
          VISION (detected bottle) + NAVIGATION (went to x,y,z) → GRASP
        """
        logger.info("\n" + "=" * 70)
        logger.info("[ARM101] *** RECEIVING COMBINED VISION + NAVIGATION SIGNAL ***")
        logger.info("=" * 70)
        logger.info(f"[SIGNAL] Detected object: '{target}'")
        logger.info(f"[SIGNAL] Navigation position: x={pose.get('x'):.3f}, y={pose.get('y'):.3f}, z={pose.get('z'):.3f}")
        logger.info(f"[SIGNAL] Frame: {pose.get('frame', 'unknown')}")
        logger.info("=" * 70)
        
        strategy = self._get_strategy(target)
        logger.info(f"\n[ARM101] Using grasp strategy for '{target}':")
        logger.info(f"  - Grip force: {strategy['grip_force']}%")
        logger.info(f"  - Approach height: {strategy['approach_height']:.2f}m")
        logger.info(f"  - Grasp depth: {strategy['depth']:.2f}m")
        
        # TODO: Send grasp command to your Arm 101
        # Example (using pose + strategy):
        # arm_motion = await self.arm_client.plan_grasp(
        #     strategy=strategy,
        #     target_pose=pose,
        #     object_class=target
        # )
        # success = await arm_motion.execute()
        
        logger.info(f"\n[ARM101] [SIMULATED] Executing grasp motion for '{target}'...")
        
        # Simulate: Move to pose, approach, grasp, retract
        await asyncio.sleep(0.5)  # Move to approach
        logger.info("[ARM101] [SIMULATED] ├─ Moving to approach position")
        
        await asyncio.sleep(1.0)  # Execute grasp
        logger.info("[ARM101] [SIMULATED] ├─ Executing grasp motion")
        
        await asyncio.sleep(0.5)  # Retract
        logger.info("[ARM101] [SIMULATED] └─ Retracting with object")
        
        self.held_object = True
        self.last_grasp_target = target
        
        self.grasp_history.append({
            "target": target,
            "pose": pose,
            "timestamp": time.time(),
            "success": True
        })
        
        logger.info(f"\n[ARM101] Grasp complete. Now holding: '{target}'")
        logger.info("=" * 70 + "\n")
        return True
    
    async def verify_grasp(self) -> bool:
        """Check if object is held (force sensor, weight sensor, etc.)."""
        # TODO: Query your force/weight sensor
        # Example:
        # force = await self.gripper.read_force()
        # held = force > MIN_FORCE_THRESHOLD
        
        logger.info(f"\n[ARM101] Verifying grasp of '{self.last_grasp_target or 'unknown'}'...")
        logger.info("[ARM101] [SIMULATED] Reading force sensor...")
        
        # Simulate verification
        await asyncio.sleep(0.5)
        held = self.held_object
        
        logger.info(f"[ARM101] Grip verification: {'SECURE' if held else 'FAILED'}")
        return held
    
    async def execute_stow(self) -> bool:
        """Put arm and gripper away safely."""
        # TODO: Send stow/home command to your arm
        # Example (ROS):
        # home_goal = HomeArm.Goal()
        # await self.arm_client.send_goal_async(home_goal)
        
        logger.info(f"\n[ARM101] Stowing arm (holding '{self.last_grasp_target or 'object'}')")
        logger.info("[ARM101] [SIMULATED] Moving to home position...")
        
        # Simulate stow
        await asyncio.sleep(1.0)
        self.held_object = False
        self.last_grasp_target = None
        
        logger.info("[ARM101] Arm stowed, ready for next task")
        logger.info("=" * 70 + "\n")
        return True


class Arm101BridgeServer:
    """WebSocket server for coordinator → Arm 101 bridge."""
    
    def __init__(self, arm_controller: Optional[Arm101Controller] = None):
        self.arm = arm_controller or Arm101Controller()
    
    async def handle_request(self, ws, path):
        """Handle incoming WebSocket request."""
        # Check authentication
        if not authenticated(ws):
            logger.warning(f"Unauthorized connection from {ws.remote_address}")
            await ws.close(1008, "Unauthorized")
            return
        
        try:
            # Receive request (timeout after 5 seconds)
            request_json = await asyncio.wait_for(ws.recv(), timeout=5.0)
            request = json.loads(request_json)
            logger.info(f"Received: {request.get('action')}")
            
            action = request.get("action")
            response = {"ok": False, "error": "Unknown action"}
            
            # STEP 5: grasp (MAIN SIGNAL)
            if action == "grasp":
                response = await self.handle_grasp(request)
            
            # STEP 6: verify_grasp
            elif action == "verify_grasp":
                response = await self.handle_verify_grasp(request)
            
            # STEP 7: stow
            elif action == "stow":
                response = await self.handle_stow(request)
            
            else:
                response = {"ok": False, "error": f"Arm101 doesn't handle '{action}'"}
            
            # Send response
            await ws.send(json.dumps(response))
            logger.info(f"Responded: {response}")
        
        except asyncio.TimeoutError:
            logger.error("Request timeout")
            await ws.send(json.dumps({"ok": False, "error": "Timeout"}))
        
        except json.JSONDecodeError:
            logger.error("Invalid JSON")
            await ws.send(json.dumps({"ok": False, "error": "Invalid JSON"}))
        
        except ConnectionClosed:
            logger.info("Connection closed")
        
        except Exception as e:
            logger.error(f"Error: {e}")
            await ws.send(json.dumps({"ok": False, "error": str(e)}))
    
    async def handle_grasp(self, request: dict) -> dict:
        """
        Handle grasp command with vision + navigation signals.
        
        THIS IS THE CRITICAL STEP:
          request = {
            "action": "grasp",
            "target": "bottle",                    # ← From vision detection
            "pose": {x: 1.2, y: 0.5, z: 0.7}    # ← From nav2 navigation
          }
        """
        try:
            target = request.get("target")
            pose = request.get("pose")
            
            if not target or not pose:
                return {"ok": False, "error": "Missing target or pose"}
            
            # Execute grasp with signals
            success = await self.arm.execute_grasp(target, pose)
            
            if success:
                return {
                    "ok": True,
                    "result": {
                        "completed": True,
                        "target": target,
                        "held": self.arm.held_object
                    }
                }
            else:
                return {"ok": False, "error": "Grasp execution failed"}
        
        except Exception as e:
            return {"ok": False, "error": str(e)}
    
    async def handle_verify_grasp(self, request: dict) -> dict:
        """Verify the object is held."""
        try:
            held = await self.arm.verify_grasp()
            
            return {
                "ok": True,
                "result": {
                    "held": held,
                    "target": self.arm.last_grasp_target
                }
            }
        
        except Exception as e:
            return {"ok": False, "error": str(e)}
    
    async def handle_stow(self, request: dict) -> dict:
        """Stow the arm and gripper."""
        try:
            success = await self.arm.execute_stow()
            
            if success:
                return {
                    "ok": True,
                    "result": {
                        "completed": True
                    }
                }
            else:
                return {"ok": False, "error": "Stow failed"}
        
        except Exception as e:
            return {"ok": False, "error": str(e)}


async def main():
    """Start the bridge server."""
    parser = argparse.ArgumentParser(description="Arm 101 Bridge Server")
    parser.add_argument("--host", default="0.0.0.0", help="Listen address")
    parser.add_argument("--port", type=int, default=8771, help="Listen port")
    args = parser.parse_args()
    
    # Validate token
    try:
        token = get_token()
        logger.info(f"Token: {token[:8]}...{token[-8:]}")
    except ValueError as e:
        logger.error(f"Configuration error: {e}")
        return
    
    # Initialize Arm 101 controller
    arm = Arm101Controller()
    await arm.initialize()
    
    # Create and start server
    server = Arm101BridgeServer(arm)
    
    async with serve(server.handle_request, args.host, args.port):
        logger.info(f"Arm 101 Bridge listening on {args.host}:{args.port}")
        logger.info(f"Ready to receive: grasp (with detection signal), verify_grasp, stow")
        try:
            await asyncio.Future()  # Run forever
        except KeyboardInterrupt:
            logger.info("Shutting down")


if __name__ == "__main__":
    asyncio.run(main())
