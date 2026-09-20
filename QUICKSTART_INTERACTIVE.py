#!/usr/bin/env python3
"""
VLA-HTN QUICK START GUIDE

Get the complete system up and running in 5 steps.

STEP 1: VERIFY SETUP (1 minute)
STEP 2: TEST VISION (1 minute)
STEP 3: TEST NAVIGATION (1 minute)
STEP 4: TEST MANIPULATION (1 minute)
STEP 5: RUN COMPLETE SYSTEM (2 minutes)

Total time: ~5 minutes for simulated hardware
"""

import os
import subprocess
import sys
from pathlib import Path


def print_header(title):
    print("\n" + "=" * 80)
    print(f"  {title}")
    print("=" * 80)


def print_step(num, title, instructions):
    print(f"\n{'─' * 80}")
    print(f"STEP {num}: {title}")
    print(f"{'─' * 80}")
    print(instructions)


def run_command(cmd, description=""):
    """Run a command and return success/failure."""
    if description:
        print(f"\n→ {description}")
    print(f"  $ {cmd}")
    try:
        result = subprocess.run(
            cmd,
            shell=True,
            capture_output=True,
            text=True,
            timeout=30
        )
        if result.returncode == 0:
            if result.stdout:
                print(f"  Output: {result.stdout[:200]}")
            return True
        else:
            if result.stderr:
                print(f"  Error: {result.stderr[:200]}")
            return False
    except subprocess.TimeoutExpired:
        print("  ERROR: Command timed out")
        return False
    except Exception as e:
        print(f"  ERROR: {e}")
        return False


def main():
    """Run quick start guide."""
    
    print_header("VLA-HTN COMPLETE SYSTEM - QUICK START")
    
    print("""
This guide will help you get the complete robot system running
in just 5 minutes with simulated hardware, or 1 hour with real Pis.

WHAT WILL HAPPEN:
  1. Verify your setup is complete
  2. Test vision (object detection)
  3. Test navigation (movement to target)
  4. Test manipulation (grasping)
  5. Run the complete fetch sequence

REQUIREMENTS:
  ✓ Python 3.10+ with venv
  ✓ Dependencies installed (pip install -r requirements.txt)
  ✓ ROBOT_TOKEN environment variable set
    export ROBOT_TOKEN="your-secure-token-at-least-24-chars"

Let's begin!
""")
    
    input("Press Enter to continue...")
    
    # ===== STEP 1 =====
    print_step(
        1,
        "VERIFY SETUP",
        """
This checks that your system is properly configured.

What it verifies:
  ✓ Environment variables (ROBOT_TOKEN, URLs)
  ✓ Python dependencies
  ✓ Hardware devices
  ✓ Network connectivity
  ✓ Required files

RUN THIS:
  python deployment_checklist.py --check all
""")
    
    if input("\nRun verification? (y/n): ").lower() == 'y':
        run_command(
            "python deployment_checklist.py --check all",
            "Verifying deployment..."
        )
    
    # ===== STEP 2 =====
    print_step(
        2,
        "TEST VISION (Object Detection)",
        """
This tests the camera service's ability to detect objects
using YOLO-World on your RealSense camera (or simulated).

What it does:
  1. Connects to camera service
  2. Sends "find a bottle" command
  3. Gets back 3D pose of detected object
  4. Reports detection confidence

Expected output:
  ✓ Object detected: bottle (confidence: 92%)
  ✓ Position: x=1.2, y=0.5, z=0.7

RUN THIS:
  python test_system_integration.py --step vision
""")
    
    if input("\nRun vision test? (y/n): ").lower() == 'y':
        run_command(
            "python test_system_integration.py --step vision",
            "Testing vision detection..."
        )
    
    # ===== STEP 3 =====
    print_step(
        3,
        "TEST NAVIGATION (Movement to Target)",
        """
This tests the navigation system's ability to move the robot
to the location detected by vision (or simulated).

What it does:
  1. Connects to Nav2 bridge
  2. Saves starting position
  3. Navigates to detected object location
  4. Returns to starting position

Expected output:
  ✓ Starting position saved: x=0, y=0
  ✓ Navigating to: x=1.2, y=0.5
  ✓ Returned to start

RUN THIS:
  python test_system_integration.py --step navigation
""")
    
    if input("\nRun navigation test? (y/n): ").lower() == 'y':
        run_command(
            "python test_system_integration.py --step navigation",
            "Testing navigation..."
        )
    
    # ===== STEP 4 =====
    print_step(
        4,
        "TEST MANIPULATION (Grasping)",
        """
This tests the arm's ability to grasp the detected object.

What it does:
  1. Connects to Arm101 bridge
  2. Receives detected object type (from vision)
  3. Receives detected position (from navigation)
  4. Executes grasp motion
  5. Verifies object is held
  6. Stows arm safely

CRITICAL SIGNAL INTEGRATION:
  The arm receives BOTH:
    - Object class from VISION ("bottle")
    - Position from NAVIGATION (x:1.2, y:0.5)
  
  This allows object-specific grasp strategies!

Expected output:
  ✓ Grasp signal received: bottle at x=1.2, y=0.5
  ✓ Executing grasp motion
  ✓ Verification: object held = True
  ✓ Arm stowed

RUN THIS:
  python test_system_integration.py --step manipulation
""")
    
    if input("\nRun manipulation test? (y/n): ").lower() == 'y':
        run_command(
            "python test_system_integration.py --step manipulation",
            "Testing manipulation..."
        )
    
    # ===== STEP 5 =====
    print_step(
        5,
        "RUN COMPLETE SYSTEM (End-to-End Test)",
        """
This runs the COMPLETE 8-step fetch sequence:

  STEP 1: save_start → Remember starting position
  STEP 2: locate → Find object with vision
  STEP 3: approach → Navigate to object (SIGNAL from vision)
  STEP 2b: locate → Reobserve after moving
  STEP 5: grasp → Grasp object (SIGNAL from vision + navigation)
  STEP 6: verify_grasp → Confirm object is held
  STEP 7: stow → Put arm away safely
  STEP 8: return_start → Return to starting position

THE COMPLETE SIGNAL FLOW:
  User: "grab a bottle"
    ↓
  Vision detects bottle at (1.2, 0.5, 0.7)
    ↓
  Navigation moves to (1.2, 0.5)
    ↓
  Arm receives: object="bottle", pose=(1.2, 0.5, 0.7)
    ↓
  Arm executes grasp with proper strategy for bottle shape
    ↓
  Robot returns home with object
    ↓
  COMPLETE ✓

RUN THIS:
  python test_system_integration.py --full
""")
    
    if input("\nRun complete end-to-end test? (y/n): ").lower() == 'y':
        run_command(
            "python test_system_integration.py --full",
            "Running complete end-to-end test..."
        )
    
    # ===== SUMMARY =====
    print_header("QUICK START COMPLETE!")
    
    print("""
🎉 If all tests passed, your system is working correctly!

WHAT YOU'VE VERIFIED:
  ✓ Vision detection works
  ✓ Navigation movement works
  ✓ Arm manipulation works
  ✓ Complete 8-step sequence works
  ✓ Vision → Navigation → Arm signals flow correctly

NEXT STEPS:
  1. Read COMPLETE_SYSTEM_README.md for full documentation
  2. For real hardware deployment, see SYSTEM_INTEGRATION_GUIDE.py
  3. Deploy to three Raspberry Pis as described in IMPLEMENTATION_SUMMARY.md

FOR REAL HARDWARE DEPLOYMENT:
  
  Camera Pi:
    python run_camera_service.py &
    python system_orchestrator.py --mode complete --target "bottle"
  
  Nav2 Pi:
    python nav2_bridge_server.py --host 0.0.0.0 --port 8770
  
  Arm101 Pi:
    python arm101_bridge_server.py --host 0.0.0.0 --port 8771

MONITORING:
  Watch the complete system:
    python system_orchestrator.py --mode status
  
  Run interactive mode for manual testing:
    python system_orchestrator.py --mode interactive

TROUBLESHOOTING:
  If something fails, check the logs for [SIGNAL] tags:
    grep "\\[SIGNAL\\]" *.log
    grep "\\[NAV2\\]" nav2_bridge.log
    grep "\\[ARM101\\]" arm101_bridge.log

FILES REFERENCE:
  COMPLETE_SYSTEM_README.md - Quick overview
  SYSTEM_INTEGRATION_GUIDE.py - Detailed deployment
  IMPLEMENTATION_SUMMARY.md - What was built
  system_orchestrator.py - Master control
  test_system_integration.py - Test suite
  deployment_checklist.py - Setup verification

SUPPORT:
  Questions? Start with COMPLETE_SYSTEM_README.md
  Need architecture details? See SYSTEM_INTEGRATION_GUIDE.py
  Need setup help? Run deployment_checklist.py
""")
    
    print("\n✓ Quick start guide complete. Good luck with your robot system!")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nQuick start cancelled.")
        sys.exit(0)
