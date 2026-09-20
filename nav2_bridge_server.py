"""
Nav2 Bridge Server - Run on your Navigation Pi

This server listens for coordinator commands via WebSocket and routes them to Nav2.
It also handles pose transformations and feedback.

SETUP:
  1. Copy this file to your nav2_pi
  2. Install: pip install websockets pydantic
  3. Configure NAV2_ACTION_SERVER, NAV2_LOCAL_COSTMAP, NAV2_GLOBAL_COSTMAP
  4. Run: python nav2_bridge_server.py
  
EXPECTED BEHAVIOR:
  - Listens on 0.0.0.0:8770
  - Authenticates all requests with Bearer token
  - Routes save_start, approach, return_start to your Nav2 stack
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


class Nav2Controller:
    """Interface to your Nav2 stack (customize for your setup)."""
    
    def __init__(self):
        # TODO: Customize these for your Nav2 setup
        self.current_pose = {"x": 0.0, "y": 0.0, "z": 0.0, "frame": "map"}
        self.nav2_goal_client = None  # Initialize your Nav2 action client here
        self.navigation_history = []
        self.simulated = True  # Set to False when real Nav2 is connected
        
    async def get_current_pose(self):
        """Get robot's current pose from Nav2 or odometry."""
        # TODO: Query your Nav2 TF or odometry
        # Example (ROS2):
        # from tf2_ros import TransformListener
        # transform = tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        # pose = extract_pose_from_transform(transform)
        
        logger.info(f"[NAV2] Current pose: {self.current_pose}")
        return self.current_pose
    
    async def navigate_to_pose(self, target_pose: dict, timeout: float = 120.0) -> bool:
        """Send a navigation goal to Nav2."""
        logger.info(f"[NAV2] Navigation goal received: {target_pose}")
        logger.info(f"[SIGNAL] Robot navigating from {self.current_pose} to {target_pose}")
        
        # TODO: Send goal to your Nav2 action server (requires rclpy)
        # Example (ROS2):
        # from nav2_simple_commander.robot_navigator import BasicNavigator
        # navigator = BasicNavigator()
        # goal_pose = create_pose_stamped(self, target_pose['x'], target_pose['y'])
        # navigator.goToPose(goal_pose)
        # while not navigator.isTaskComplete():
        #     await asyncio.sleep(0.1)
        
        # Simulated navigation for testing
        logger.info("[NAV2] [SIMULATED] Moving to target...")
        await asyncio.sleep(2.0)  # Simulate 2 seconds of navigation
        self.current_pose = target_pose
        
        self.navigation_history.append({
            "target": target_pose,
            "timestamp": time.time(),
            "success": True
        })
        
        logger.info(f"[NAV2] [SIMULATED] Reached target: {self.current_pose}")
        return True
    
    async def initialize(self):
        """Initialize Nav2 action clients."""
        # TODO: Connect to real Nav2 stack
        # Example (ROS2):
        # import rclpy
        # rclpy.init()
        # self.node = rclpy.create_node('nav2_bridge')
        # self.nav2_goal_client = ActionClient(...)
        
        logger.info("[NAV2] Controller initialized (SIMULATED MODE)")
        logger.info("[NAV2] To connect real Nav2: customize initialize() in nav2_bridge_server.py")


class Nav2BridgeServer:
    """WebSocket server for coordinator → Nav2 bridge."""
    
    def __init__(self, nav2_controller: Optional[Nav2Controller] = None):
        self.nav2 = nav2_controller or Nav2Controller()
    
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
            
            # STEP 1: save_start
            if action == "save_start":
                response = await self.handle_save_start()
            
            # STEP 3: approach
            elif action == "approach":
                response = await self.handle_approach(request)
            
            # STEP 8: return_start
            elif action == "return_start":
                response = await self.handle_return_start(request)
            
            else:
                response = {"ok": False, "error": f"Nav2 doesn't handle '{action}'"}
            
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
    
    async def handle_save_start(self) -> dict:
        """Get current pose and save as starting position."""
        try:
            current_pose = await self.nav2.get_current_pose()
            logger.info(f"Saved starting pose: {current_pose}")
            return {
                "ok": True,
                "result": {
                    "pose": current_pose
                }
            }
        except Exception as e:
            return {"ok": False, "error": f"Failed to get pose: {e}"}
    
    async def handle_approach(self, request: dict) -> dict:
        """Navigate to the target pose (from camera detection)."""
        try:
            target_pose = request.get("pose")
            if not target_pose:
                return {"ok": False, "error": "Missing target pose"}
            
            logger.info("=" * 70)
            logger.info("[SIGNAL] APPROACHING DETECTED OBJECT")
            logger.info("=" * 70)
            logger.info(f"  From position: x={self.current_pose.get('x'):.3f}, y={self.current_pose.get('y'):.3f}")
            logger.info(f"  To position:   x={target_pose.get('x'):.3f}, y={target_pose.get('y'):.3f}")
            logger.info("  This pose was detected by the VISION system")
            logger.info("=" * 70)
            
            # Navigate to the detected object location
            success = await self.nav2.navigate_to_pose(target_pose, timeout=120.0)
            
            if success:
                logger.info("[NAV2] Successfully reached detection location")
                return {
                    "ok": True,
                    "result": {
                        "completed": True,
                        "pose": target_pose,
                        "simulated": self.nav2.simulated
                    }
                }
            else:
                return {"ok": False, "error": "Navigation failed"}
        
        except Exception as e:
            logger.error(f"[NAV2] Error in approach: {e}")
            return {"ok": False, "error": str(e)}
    
    async def handle_return_start(self, request: dict) -> dict:
        """Navigate back to the starting position."""
        try:
            home_pose = request.get("pose")
            if not home_pose:
                return {"ok": False, "error": "Missing home pose"}
            
            logger.info(f"Returning to start: {home_pose}")
            success = await self.nav2.navigate_to_pose(home_pose, timeout=120.0)
            
            if success:
                return {
                    "ok": True,
                    "result": {
                        "completed": True,
                        "pose": home_pose
                    }
                }
            else:
                return {"ok": False, "error": "Return navigation failed"}
        
        except Exception as e:
            return {"ok": False, "error": str(e)}


async def main():
    """Start the bridge server."""
    parser = argparse.ArgumentParser(description="Nav2 Bridge Server")
    parser.add_argument("--host", default="0.0.0.0", help="Listen address")
    parser.add_argument("--port", type=int, default=8770, help="Listen port")
    args = parser.parse_args()
    
    # Validate token
    try:
        token = get_token()
        logger.info(f"Token: {token[:8]}...{token[-8:]}")
    except ValueError as e:
        logger.error(f"Configuration error: {e}")
        return
    
    # Initialize Nav2 controller
    nav2 = Nav2Controller()
    await nav2.initialize()
    
    # Create and start server
    server = Nav2BridgeServer(nav2)
    
    async with serve(server.handle_request, args.host, args.port):
        logger.info(f"Nav2 Bridge listening on {args.host}:{args.port}")
        logger.info(f"Ready to receive: save_start, approach, return_start")
        try:
            await asyncio.Future()  # Run forever
        except KeyboardInterrupt:
            logger.info("Shutting down")


if __name__ == "__main__":
    asyncio.run(main())
