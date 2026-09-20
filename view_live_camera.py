#!/usr/bin/env python3
"""
LIVE CAMERA VIEWER - Display real-time camera feed with object detection from Raspberry Pi

Usage:
    1. Start camera service on Pi:
       $ ssh gisooj@gisoopi.local
       $ cd ~/VLA-HTN
       $ source venv/bin/activate
       $ export ROBOT_TOKEN='test-token-12345678901234567890'
       $ python -m robot_app.camera --host 0.0.0.0 --port 8767
    
    2. Run this viewer from your desktop:
       $ export ROBOT_TOKEN='test-token-12345678901234567890'
       $ export CAMERA_URL='ws://gisoopi.local:8767'
       $ python view_live_camera.py

    3. Enter object names to detect in real-time (e.g., "bottle", "cup", "person")
"""

import asyncio
import json
import os
import sys
from pathlib import Path
import time

sys.path.insert(0, str(Path(__file__).parent))

from robot_app.protocol import Request, rpc


async def view_live_camera():
    """View live camera feed with object detection."""
    
    camera_url = os.getenv("CAMERA_URL", "ws://gisoopi.local:8767")
    
    print("\n" + "="*90)
    print("🎥 LIVE CAMERA VIEWER - Object Detection Feed")
    print("="*90)
    print(f"\nCamera Service: {camera_url}")
    print("\nControls:")
    print("  - Enter object name to detect (e.g., 'bottle', 'cup', 'person', 'hand')")
    print("  - Type 'list' to see detection history")
    print("  - Type 'quit' or 'exit' to stop")
    print("  - Press Enter repeatedly to keep detecting the same object\n")
    
    detection_history = []
    current_target = None
    last_detection_time = 0
    frame_count = 0
    fps_counter = 0
    fps_time = time.time()
    
    try:
        while True:
            # Get user input (non-blocking)
            if not current_target:
                print("\n" + "-"*90)
                user_input = input("Enter object to detect (or 'quit'): ").strip()
                
                if user_input.lower() in ['quit', 'exit', 'q']:
                    print("\n✓ Camera viewer closed")
                    break
                elif user_input.lower() == 'list':
                    print("\n📋 Recent Detections:")
                    if detection_history:
                        for i, det in enumerate(detection_history[-10:], 1):
                            print(f"  {i}. {det['target']} @ {det['time']:.1f}s | "
                                  f"conf: {det['confidence']:.0%} | "
                                  f"pos: ({det['x']:.2f}, {det['y']:.2f}, {det['z']:.2f})")
                    else:
                        print("  (No detections yet)")
                    continue
                elif user_input:
                    current_target = user_input
                else:
                    continue
            
            # Detect current target
            print(f"\n🔍 Detecting '{current_target}'...", end=" ", flush=True)
            
            try:
                start_time = time.time()
                response = await asyncio.wait_for(
                    rpc(
                        camera_url,
                        Request(action="locate", target=current_target),
                        timeout=10
                    ),
                    timeout=15
                )
                
                result = response.result
                detection_time = time.time() - start_time
                frame_count += 1
                
                # Extract data
                pose = result.get("pose", {})
                confidence = result.get("confidence", 0)
                
                # Store in history
                detection_entry = {
                    'target': current_target,
                    'confidence': confidence,
                    'x': pose.get('x', 0),
                    'y': pose.get('y', 0),
                    'z': pose.get('z', 0),
                    'frame': pose.get('frame', 'unknown'),
                    'time': time.time(),
                }
                detection_history.append(detection_entry)
                
                # Calculate FPS
                fps_counter += 1
                elapsed = time.time() - fps_time
                if elapsed >= 1.0:
                    fps = fps_counter / elapsed
                    fps_time = time.time()
                    fps_counter = 0
                else:
                    fps = frame_count / (time.time() - last_detection_time + 0.001)
                
                # Display result
                confidence_bar = "█" * int(confidence * 20) + "░" * (20 - int(confidence * 20))
                print(f"✓")
                print(f"\n  📍 Object: {current_target}")
                print(f"  🎯 Position (map frame):")
                print(f"     X: {pose.get('x', 0):7.3f}m  (East)")
                print(f"     Y: {pose.get('y', 0):7.3f}m  (North)")
                print(f"     Z: {pose.get('z', 0):7.3f}m  (Up)")
                print(f"  📊 Confidence: [{confidence_bar}] {confidence:.0%}")
                print(f"  ⏱️  Detection time: {detection_time:.2f}s")
                print(f"  🎬 FPS: {fps:.1f}")
                print(f"  📦 Total detections: {len(detection_history)}")
                
                # Confidence validation
                if confidence < 0.5:
                    print(f"  ⚠️  WARNING: Low confidence - may not be reliable for grasping")
                
                last_detection_time = time.time()
                
                # Ask if user wants to continue with same target
                print(f"\nPress Enter to detect '{current_target}' again, or enter new target:")
                user_input = input("> ").strip()
                
                if user_input.lower() in ['quit', 'exit', 'q']:
                    print("\n✓ Camera viewer closed")
                    break
                elif user_input.lower() == 'list':
                    print("\n📋 Recent Detections:")
                    if detection_history:
                        for i, det in enumerate(detection_history[-10:], 1):
                            print(f"  {i}. {det['target']} | conf: {det['confidence']:.0%} | "
                                  f"pos: ({det['x']:.2f}, {det['y']:.2f}, {det['z']:.2f})")
                    else:
                        print("  (No detections yet)")
                elif user_input:
                    current_target = user_input
                
            except asyncio.TimeoutError:
                print(f"⏱️  Timeout - detection took too long")
            except Exception as e:
                print(f"✗ Error: {e}")
                current_target = None
    
    except KeyboardInterrupt:
        print("\n\n✓ Viewer interrupted by user")
    except Exception as e:
        print(f"\n✗ Fatal error: {e}")
        print(f"\nTroubleshooting:")
        print(f"  1. Is camera service running on the Pi?")
        print(f"     $ python -m robot_app.camera --host 0.0.0.0 --port 8767")
        print(f"  2. Check CAMERA_URL: {camera_url}")
        print(f"  3. Check ROBOT_TOKEN is set: {os.getenv('ROBOT_TOKEN', 'NOT SET')}")
        import traceback
        traceback.print_exc()


async def main():
    await view_live_camera()


if __name__ == "__main__":
    asyncio.run(main())
