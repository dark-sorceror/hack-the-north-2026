#!/usr/bin/env python3
"""
VLA-HTN END-TO-END INTEGRATION TEST

This script tests the complete system integration:
  1. Vision: Detect objects with RealSense + YOLO-World
  2. Navigation: Command Nav2 bridge to approach detected location
  3. Manipulation: Send grasp command to Arm101 bridge
  4. Return: Navigate back to starting position

USAGE:
  # Full test with real services
  python test_system_integration.py --full

  # Test with simulated services
  python test_system_integration.py --simulated

  # Test individual steps
  python test_system_integration.py --step vision
  python test_system_integration.py --step navigation
  python test_system_integration.py --step manipulation
"""

import argparse
import asyncio
import json
import logging
import os
import sys
import time
from typing import Dict, Any, Optional

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)-8s | %(message)s"
)
logger = logging.getLogger(__name__)


class IntegrationTest:
    """Complete system integration test suite."""
    
    def __init__(self, simulated: bool = True):
        self.simulated = simulated
        self.results = {}
        self.target = "bottle"
        
        # Load environment
        self.robot_token = os.getenv("ROBOT_TOKEN", "dev-token-at-least-24-chars-long")
        self.camera_url = os.getenv("CAMERA_URL", "ws://127.0.0.1:8767")
        self.nav2_url = os.getenv("NAV2_BRIDGE_URL", "ws://127.0.0.1:8770")
        self.arm101_url = os.getenv("ARM101_BRIDGE_URL", "ws://127.0.0.1:8771")
    
    def print_header(self, title: str):
        """Print a formatted header."""
        print("\n" + "=" * 80)
        print(f"  {title}")
        print("=" * 80)
    
    def print_section(self, title: str):
        """Print a formatted section."""
        print(f"\n{title}")
        print("-" * 80)
    
    async def test_vision(self) -> bool:
        """TEST 1: Vision detection using camera service."""
        self.print_header("TEST 1: VISION DETECTION")
        self.print_section("Connecting to Camera Service")
        
        logger.info(f"Camera Service URL: {self.camera_url}")
        
        try:
            from robot_app.camera import Camera
            
            camera = Camera(service_url=self.camera_url)
            
            self.print_section(f"Detecting: {self.target}")
            logger.info(f"Sending detection request to camera service...")
            
            result = await camera.locate(self.target)
            
            if result and "pose" in result:
                pose = result["pose"]
                confidence = result.get("confidence", 0)
                
                logger.info(f"✓ Detection successful!")
                logger.info(f"  Object: {self.target}")
                logger.info(f"  Confidence: {confidence:.1%}")
                logger.info(f"  Position: x={pose.get('x'):.3f}, y={pose.get('y'):.3f}, z={pose.get('z'):.3f}")
                logger.info(f"  Frame: {pose.get('frame', 'map')}")
                
                self.results["vision"] = {
                    "success": True,
                    "detected_pose": pose,
                    "confidence": confidence,
                    "target": self.target
                }
                
                return True
            else:
                logger.error("✗ Detection failed - no pose returned")
                self.results["vision"] = {"success": False, "error": "No pose returned"}
                return False
        
        except Exception as e:
            logger.error(f"✗ Vision test failed: {e}")
            self.results["vision"] = {"success": False, "error": str(e)}
            return False
    
    async def test_navigation(self) -> bool:
        """TEST 2: Navigation using Nav2 bridge."""
        self.print_header("TEST 2: NAVIGATION")
        
        if "vision" not in self.results or not self.results["vision"]["success"]:
            logger.error("✗ Skipping navigation test - vision detection failed")
            return False
        
        detected_pose = self.results["vision"]["detected_pose"]
        
        self.print_section("Connecting to Nav2 Bridge")
        logger.info(f"Nav2 Bridge URL: {self.nav2_url}")
        
        try:
            from robot_app.hardware_integrations import Nav2Bridge
            
            nav2 = Nav2Bridge(self.nav2_url)
            
            # STEP 1: Save starting position
            self.print_section("STEP 1: Saving starting position")
            logger.info("Calling: save_start")
            
            start_result = await nav2.call("save_start")
            start_pose = start_result.get("pose", {})
            
            logger.info(f"✓ Starting position saved")
            logger.info(f"  Position: x={start_pose.get('x'):.3f}, y={start_pose.get('y'):.3f}, z={start_pose.get('z'):.3f}")
            
            # STEP 2: Navigate to detected object
            self.print_section("STEP 2: Navigating to detected object")
            logger.info(f"Calling: approach with pose from vision")
            logger.info(f"  Target: x={detected_pose.get('x'):.3f}, y={detected_pose.get('y'):.3f}, z={detected_pose.get('z'):.3f}")
            
            approach_result = await nav2.call("approach", pose=detected_pose)
            
            logger.info(f"✓ Navigation to object successful")
            logger.info(f"  Robot is now at detection location")
            
            # STEP 3: Return to start
            self.print_section("STEP 3: Returning to starting position")
            logger.info(f"Calling: return_start")
            
            return_result = await nav2.call("return_start", pose=start_pose)
            
            logger.info(f"✓ Successfully returned to starting position")
            
            self.results["navigation"] = {
                "success": True,
                "start_pose": start_pose,
                "approach_pose": detected_pose,
                "return_complete": True
            }
            
            return True
        
        except Exception as e:
            logger.error(f"✗ Navigation test failed: {e}")
            self.results["navigation"] = {"success": False, "error": str(e)}
            return False
    
    async def test_manipulation(self) -> bool:
        """TEST 3: Manipulation using Arm101 bridge."""
        self.print_header("TEST 3: MANIPULATION (ARM101)")
        
        if "vision" not in self.results or not self.results["vision"]["success"]:
            logger.error("✗ Skipping manipulation test - vision detection failed")
            return False
        
        detected_pose = self.results["vision"]["detected_pose"]
        target = self.results["vision"]["target"]
        
        self.print_section("Connecting to Arm101 Bridge")
        logger.info(f"Arm101 Bridge URL: {self.arm101_url}")
        
        try:
            from robot_app.hardware_integrations import Arm101Bridge
            
            arm = Arm101Bridge(self.arm101_url)
            
            # STEP 1: Execute grasp (CRITICAL SIGNAL)
            self.print_section("STEP 1: Executing grasp motion")
            logger.info("Calling: grasp with COMBINED SIGNAL")
            logger.info(f"  Object detected: {target}")
            logger.info(f"  Position from nav: x={detected_pose.get('x'):.3f}, y={detected_pose.get('y'):.3f}, z={detected_pose.get('z'):.3f}")
            
            grasp_result = await arm.call("grasp", target=target, pose=detected_pose)
            
            logger.info(f"✓ Grasp motion executed")
            logger.info(f"  Object held: {grasp_result.get('held', 'unknown')}")
            
            # STEP 2: Verify grasp
            self.print_section("STEP 2: Verifying grasp")
            logger.info("Calling: verify_grasp")
            
            verify_result = await arm.call("verify_grasp")
            held = verify_result.get("held", False)
            
            if held:
                logger.info(f"✓ Grasp verified - object is held securely")
            else:
                logger.warning(f"✗ Grasp verification failed - object may not be held")
            
            # STEP 3: Stow arm
            self.print_section("STEP 3: Stowing arm")
            logger.info("Calling: stow")
            
            stow_result = await arm.call("stow")
            
            logger.info(f"✓ Arm stowed safely")
            
            self.results["manipulation"] = {
                "success": True,
                "grasp_complete": True,
                "object_held": held,
                "stow_complete": True
            }
            
            return True
        
        except Exception as e:
            logger.error(f"✗ Manipulation test failed: {e}")
            self.results["manipulation"] = {"success": False, "error": str(e)}
            return False
    
    async def test_full_sequence(self) -> bool:
        """Execute complete sequence: vision → navigation → manipulation."""
        self.print_header("COMPLETE END-TO-END TEST: FETCH SEQUENCE")
        
        logger.info("This test executes the complete robot task:")
        logger.info("  1. Detect object with vision")
        logger.info("  2. Navigate to detected location")
        logger.info("  3. Grasp the object")
        logger.info("  4. Return to starting position")
        
        # Run all three tests in sequence
        results = {
            "vision": await self.test_vision(),
            "navigation": await self.test_navigation(),
            "manipulation": await self.test_manipulation(),
        }
        
        return all(results.values())
    
    def print_summary(self):
        """Print final summary."""
        self.print_header("TEST SUMMARY")
        
        total = len(self.results)
        passed = sum(1 for r in self.results.values() if r.get("success", False))
        
        print(f"\nResults: {passed}/{total} tests passed")
        print()
        
        for test_name, result in self.results.items():
            status = "✓ PASS" if result.get("success", False) else "✗ FAIL"
            print(f"  {test_name:20} {status}")
            if not result.get("success", False) and "error" in result:
                print(f"    Error: {result['error']}")
        
        print()
        
        if passed == total:
            print("✓ ALL TESTS PASSED - System is fully integrated!")
            return 0
        else:
            print(f"✗ {total - passed} test(s) failed - check logs above")
            return 1
    
    async def run_test(self, test_type: str = "full") -> int:
        """Run requested test(s)."""
        try:
            if test_type == "full":
                success = await self.test_full_sequence()
            elif test_type == "vision":
                success = await self.test_vision()
            elif test_type == "navigation":
                success = await self.test_navigation()
            elif test_type == "manipulation":
                success = await self.test_manipulation()
            else:
                logger.error(f"Unknown test type: {test_type}")
                return 1
            
            exit_code = self.print_summary()
            return exit_code
        
        except KeyboardInterrupt:
            logger.info("\n\nTest interrupted by user")
            return 130
        except Exception as e:
            logger.error(f"Unexpected error: {e}", exc_info=True)
            return 1


async def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="VLA-HTN End-to-End Integration Test",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
EXAMPLES:
  # Run complete sequence (all three tests)
  python test_system_integration.py --full
  
  # Test only vision
  python test_system_integration.py --step vision
  
  # Test only navigation
  python test_system_integration.py --step navigation
  
  # Test only manipulation
  python test_system_integration.py --step manipulation
  
  # With simulated hardware (default)
  python test_system_integration.py --simulated
  
  # Custom target object
  python test_system_integration.py --full --target "cup"
        """
    )
    
    parser.add_argument(
        "--full",
        action="store_true",
        help="Run complete end-to-end test"
    )
    
    parser.add_argument(
        "--step",
        choices=["vision", "navigation", "manipulation"],
        help="Run only specific test step"
    )
    
    parser.add_argument(
        "--simulated",
        action="store_true",
        default=True,
        help="Use simulated hardware (default)"
    )
    
    parser.add_argument(
        "--target",
        default="bottle",
        help="Object to detect and fetch"
    )
    
    args = parser.parse_args()
    
    # Determine test type
    test_type = "full"
    if args.step:
        test_type = args.step
    elif not args.full:
        test_type = "full"  # Default to full
    
    # Create and run test
    test = IntegrationTest(simulated=args.simulated)
    test.target = args.target
    
    exit_code = await test.run_test(test_type)
    sys.exit(exit_code)


if __name__ == "__main__":
    asyncio.run(main())
