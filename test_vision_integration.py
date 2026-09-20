#!/usr/bin/env python3
"""
VOICE → VISION INTEGRATION TEST

This script tests the first 2 steps of the fetch sequence:
  1. Voice command received: "grab a bottle"
  2. Vision detects object

USAGE:
  # Test vision detection (requires camera service running)
  python test_vision_integration.py --mode vision
  
  # Test full coordinator (with simulated nav2/arm)
  python test_vision_integration.py --mode coordinator
  
  # Start camera service on Pi
  python test_vision_integration.py --start-camera-pi
"""

import argparse
import asyncio
import json
import os
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from robot_app.coordinator import Coordinator
from robot_app.protocol import Request, rpc


async def test_vision_detection():
    """Test just the vision detection step (step 2 of fetch)."""
    print("\n" + "="*80)
    print("VISION DETECTION TEST (Step 2: Locate)")
    print("="*80)
    
    camera_url = os.getenv("CAMERA_URL", "ws://127.0.0.1:8767")
    token = os.getenv("ROBOT_TOKEN", "test-token-12345678901234567890")
    
    print(f"\n[CONFIG]")
    print(f"  Camera URL: {camera_url}")
    print(f"  Token: {token[:8]}...{token[-8:]}")
    
    # Test objects
    test_objects = ["bottle", "cup", "book", "remote", "phone", "glass"]
    
    print(f"\n[TEST] Detecting objects...")
    successful = 0
    failed = 0
    
    for obj in test_objects:
        try:
            print(f"\n  Testing: {obj:<15} ... ", end="", flush=True)
            
            request = Request(action="locate", target=obj)
            response = await rpc(camera_url, request, timeout=10)
            
            if response.ok:
                result = response.result
                pose = result.get("pose", {})
                confidence = result.get("confidence", 0)
                
                print(f"✓ Detected (conf: {confidence:.1%})")
                print(f"              Pose: ({pose.get('x'):.2f}, {pose.get('y'):.2f}, {pose.get('z'):.2f})")
                successful += 1
            else:
                print(f"✗ Not found")
                failed += 1
        
        except asyncio.TimeoutError:
            print(f"✗ Timeout")
            failed += 1
            break
        
        except ConnectionRefusedError:
            print(f"\n\n✗ CAMERA SERVICE NOT RUNNING")
            print(f"   Start it with:")
            print(f"     ssh gisooj@gisoopi.local")
            print(f"     cd ~/VLA-HTN && source venv/bin/activate")
            print(f"     export ROBOT_TOKEN='test-token-12345678901234567890'")
            print(f"     python -m robot_app.camera --host 0.0.0.0 --port 8767")
            return False
        
        except Exception as e:
            print(f"✗ Error: {e}")
            failed += 1
    
    print(f"\n[RESULTS]")
    print(f"  Detected: {successful}/{len(test_objects)}")
    print(f"  Failed:   {failed}/{len(test_objects)}")
    
    return successful > 0


async def test_voice_to_coordinator():
    """Test voice command triggering coordinator fetch sequence."""
    print("\n" + "="*80)
    print("VOICE → COORDINATOR TEST (Steps 1-8)")
    print("="*80)
    
    camera_url = os.getenv("CAMERA_URL", "ws://127.0.0.1:8767")
    token = os.getenv("ROBOT_TOKEN", "test-token-12345678901234567890")
    
    print(f"\n[CONFIG]")
    print(f"  Camera URL: {camera_url}")
    print(f"  Token: {token[:8]}...{token[-8:]}")
    
    print(f"\n[VOICE COMMAND] User: \"Hey robot, grab a bottle\"")
    print(f"[STEP 1] Voice service recognizes: fetch_object(object='bottle')")
    print(f"[STEP 2+] Coordinator.fetch('bottle') running...")
    
    coordinator = Coordinator(camera_url=camera_url)
    
    try:
        result = await coordinator.fetch("bottle")
        
        print(f"\n✓ FETCH SEQUENCE COMPLETED")
        print(f"\n[RESULTS]")
        print(f"  State: {result.get('state')}")
        print(f"  Simulated: {result.get('simulated')}")
        print(f"  Steps completed: {', '.join(result.get('steps', []))}")
        
        latencies = result.get('latencies_s', {})
        if latencies:
            print(f"\n[LATENCIES]")
            for step, times in latencies.items():
                avg_ms = sum(times) * 1000 / len(times)
                print(f"  {step:<20} {avg_ms:.1f}ms")
        
        return True
    
    except Exception as e:
        print(f"\n✗ FETCH FAILED: {e}")
        return False


def start_camera_on_pi():
    """Start camera service on remote Pi."""
    print("\n" + "="*80)
    print("STARTING CAMERA SERVICE ON PI")
    print("="*80)
    
    token = os.getenv("ROBOT_TOKEN", "test-token-12345678901234567890")
    pi_host = os.getenv("PI_HOST", "gisooji@gisoopi.local")
    
    cmd = f"""
    ssh {pi_host} "
    cd ~/VLA-HTN && \
    source venv/bin/activate && \
    export ROBOT_TOKEN='{token}' && \
    python -m robot_app.camera --host 0.0.0.0 --port 8767
    "
    """
    
    print(f"\n[COMMAND]")
    print(f"  ssh {pi_host}")
    print(f"  cd ~/VLA-HTN && source venv/bin/activate")
    print(f"  export ROBOT_TOKEN='...'")
    print(f"  python -m robot_app.camera --host 0.0.0.0 --port 8767")
    
    print(f"\n[START] Running camera service...")
    print(f"(Press Ctrl+C to stop)\n")
    
    try:
        subprocess.run(cmd, shell=True, check=True)
    except KeyboardInterrupt:
        print(f"\n[STOP] Camera service stopped")
    except subprocess.CalledProcessError as e:
        print(f"\n✗ ERROR: {e}")


def show_demo_flow():
    """Show the demonstration flow."""
    print("""
    ╔════════════════════════════════════════════════════════════════════════════╗
    ║                  VOICE → VISION TEST FLOW                                 ║
    ╚════════════════════════════════════════════════════════════════════════════╝
    
    STEP-BY-STEP:
    ─────────────
    
    1. USER SPEAKS:
       "Hey robot, grab a bottle"
    
    2. VOICE SERVICE (robot_app.voice):
       - Records audio via USB microphone (Jabra)
       - Sends to Qwen cloud API (realtime)
       - Receives tool_call: fetch_object(object="bottle")
    
    3. COORDINATOR (robot_app.coordinator):
       - Receives: Coordinator.fetch("bottle")
       - Calls: save_start() → returns home pose
       - Calls: locate(target="bottle") ← VISION SIGNAL
    
    4. CAMERA SERVICE (robot_app.camera):
       - Receives: locate(target="bottle")
       - Captures frame from RealSense D435i
       - Runs YOLO-World: detect("bottle") → bounding box
       - Converts pixel → 3D: (x:1.2, y:0.5, z:0.7)
       - Returns: {pose, confidence:0.92, observed_at}
    
    5. COORDINATOR continues:
       - validate_observation(pose, confidence, age)
       - Call: approach(pose) ← Send to Nav2
       - Call: locate(target) again (re-verify)
       - Call: grasp(target, pose) ← Send to Arm101 with SIGNAL
       - Call: verify_grasp()
       - Call: stow()
       - Call: return_start(pose)
    
    6. RESULT:
       Object detected and coordinates extracted for arm
    
    ═════════════════════════════════════════════════════════════════════════════
    
    TESTING:
    ────────
    
    • Test vision only:
      python test_vision_integration.py --mode vision
    
    • Test coordinator (full flow):
      python test_vision_integration.py --mode coordinator
    
    • Start camera service:
      python test_vision_integration.py --start-camera-pi
    
    ═════════════════════════════════════════════════════════════════════════════
    """)


async def main():
    parser = argparse.ArgumentParser(
        description="Voice → Vision Integration Test",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Test vision detection
  python test_vision_integration.py --mode vision
  
  # Test full coordinator
  python test_vision_integration.py --mode coordinator
  
  # Start camera on Pi (in separate terminal)
  python test_vision_integration.py --start-camera-pi
        """
    )
    
    parser.add_argument(
        "--mode",
        choices=["vision", "coordinator", "demo"],
        default="demo",
        help="Test mode"
    )
    parser.add_argument(
        "--start-camera-pi",
        action="store_true",
        help="Start camera service on remote Pi (runs indefinitely)"
    )
    parser.add_argument(
        "--camera-url",
        default="ws://127.0.0.1:8767",
        help="Camera service WebSocket URL"
    )
    parser.add_argument(
        "--token",
        default="test-token-12345678901234567890",
        help="Robot authentication token"
    )
    
    args = parser.parse_args()
    
    # Set environment
    os.environ["CAMERA_URL"] = args.camera_url
    os.environ["ROBOT_TOKEN"] = args.token
    
    if args.start_camera_pi:
        start_camera_on_pi()
        return
    
    if args.mode == "demo":
        show_demo_flow()
        return
    
    if args.mode == "vision":
        success = await test_vision_detection()
        sys.exit(0 if success else 1)
    
    if args.mode == "coordinator":
        success = await test_voice_to_coordinator()
        sys.exit(0 if success else 1)


if __name__ == "__main__":
    asyncio.run(main())
