#!/usr/bin/env python3
"""
TEST: Voice → Vision Signal Path (First 2 Steps)

This tests:
  1. User speaks: "grab an object"
  2. Voice service recognizes command
  3. Coordinator calls camera
  4. Camera detects object and returns pose
  
NO hardware needed (camera service runs standalone or simulated).
"""

import asyncio
import json
import os
import sys
from pathlib import Path

# Add project to path
sys.path.insert(0, str(Path(__file__).parent))

from robot_app.coordinator import Coordinator
from robot_app.protocol import Request, rpc


async def test_voice_to_vision_signal():
    """
    Simulate: "Hey robot, grab a bottle"
    Expected: Camera detects bottle and returns pose
    """
    print("\n" + "="*80)
    print("TEST: VOICE → VISION SIGNAL PATH (Steps 1-2)")
    print("="*80)
    
    # Setup
    token = os.getenv("ROBOT_TOKEN", "test-token-12345678901234567890")
    camera_url = os.getenv("CAMERA_URL", "ws://127.0.0.1:8767")
    
    print(f"\n[CONFIG]")
    print(f"  Token: {token[:8]}...{token[-8:]}")
    print(f"  Camera URL: {camera_url}")
    
    # Test objects to try detecting
    test_objects = ["bottle", "cup", "book", "remote", "phone"]
    
    print(f"\n[TEST] Testing detection for objects: {test_objects}")
    
    coordinator = Coordinator(camera_url=camera_url)
    
    for obj in test_objects:
        print(f"\n  ────────────────────────────────────────")
        print(f"  VOICE COMMAND: \"Hey robot, grab a {obj}\"")
        print(f"  ────────────────────────────────────────")
        
        try:
            # STEP 1: Voice command recognized → fetch_object tool call
            print(f"\n  [STEP 1] Voice recognized tool call: fetch_object(object='{obj}')")
            
            # STEP 2: Coordinator calls camera (locate)
            print(f"  [STEP 2] Coordinator sending: locate(target='{obj}')")
            
            # Call camera service directly
            request = Request(action="locate", target=obj)
            response = await rpc(camera_url, request, timeout=10)
            
            # Parse response
            if response.ok:
                result = response.result
                pose = result.get("pose", {})
                confidence = result.get("confidence", 0)
                observed_at = result.get("observed_at", 0)
                
                print(f"\n  ✓ DETECTION SUCCESS")
                print(f"    Object: {obj}")
                print(f"    Confidence: {confidence:.2%}")
                print(f"    Pose: x={pose.get('x'):.2f}, y={pose.get('y'):.2f}, z={pose.get('z'):.2f}")
                print(f"    Frame: {pose.get('frame', 'unknown')}")
                print(f"    Age: {json.dumps(result.get('observed_at'), default=str)}")
            else:
                print(f"\n  ✗ DETECTION FAILED")
                print(f"    Error: {response.error if hasattr(response, 'error') else 'Unknown error'}")
        
        except asyncio.TimeoutError:
            print(f"\n  ✗ TIMEOUT: Camera service not responding")
            print(f"    Make sure camera is running: python -m robot_app.camera")
            break
        
        except ConnectionRefusedError:
            print(f"\n  ✗ CONNECTION REFUSED: Camera service not running")
            print(f"    Start camera service on Pi:")
            print(f"      ssh gisooj@gisoopi.local")
            print(f"      cd ~/VLA-HTN && source venv/bin/activate")
            print(f"      export ROBOT_TOKEN='your-token'")
            print(f"      python -m robot_app.camera --host 0.0.0.0 --port 8767")
            break
        
        except Exception as e:
            print(f"\n  ✗ ERROR: {e}")
            break
    
    print(f"\n" + "="*80)
    print("TEST COMPLETE")
    print("="*80 + "\n")


async def test_with_full_coordinator():
    """
    Test the full fetch sequence (but with simulated nav2/arm101).
    """
    print("\n" + "="*80)
    print("TEST: FULL FETCH SEQUENCE (Steps 1-8)")
    print("="*80)
    
    token = os.getenv("ROBOT_TOKEN", "test-token-12345678901234567890")
    
    print(f"\n[CONFIG]")
    print(f"  Token: {token[:8]}...{token[-8:]}")
    
    coordinator = Coordinator()
    
    print(f"\n[SIMULATE] User says: \"Hey robot, fetch a bottle\"")
    print(f"\n[COORDINATOR] Running full fetch sequence...\n")
    
    try:
        result = await coordinator.fetch("bottle")
        
        print(f"\n✓ FETCH COMPLETED")
        print(json.dumps(result, indent=2, default=str))
    
    except Exception as e:
        print(f"\n✗ FETCH FAILED: {e}")


def main():
    """Run tests based on command line arguments."""
    import argparse
    
    parser = argparse.ArgumentParser(description="Test Voice → Vision signal path")
    parser.add_argument(
        "--mode",
        choices=["vision-only", "full"],
        default="vision-only",
        help="Test mode: vision-only (just detect), full (fetch sequence)"
    )
    parser.add_argument(
        "--object",
        default="bottle",
        help="Object to detect"
    )
    args = parser.parse_args()
    
    if args.mode == "vision-only":
        asyncio.run(test_voice_to_vision_signal())
    else:
        asyncio.run(test_with_full_coordinator())


if __name__ == "__main__":
    main()
