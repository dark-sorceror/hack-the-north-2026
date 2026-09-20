#!/usr/bin/env python3
"""Run camera service on Pi and test object detection."""
import asyncio
import json
import os
import sys
from pathlib import Path

async def main():
    # Set token
    os.environ["ROBOT_TOKEN"] = "test-token-12345678901234567890"
    
    print("=" * 70)
    print("VLA CAMERA SERVICE - RASPBERRY PI")
    print("=" * 70)
    
    try:
        from robot_app.camera import CameraDriver, RealSenseSource, YoloWorldDetector, CameraCalibration
        from robot_app.hardware import HardwareService
        from robot_app.protocol import Request, rpc, token
        from websockets.asyncio.server import serve
        
        print("\n✓ Modules imported")
        token()
        print("✓ Token configured")
        
        # Initialize camera components
        print("\nInitializing camera hardware...")
        try:
            source = RealSenseSource(width=640, height=480, fps=30)
            print("✓ RealSense camera initialized")
        except Exception as e:
            print(f"✗ Camera initialization failed: {e}")
            print("  Attempting to install pyrealsense2...")
            sys.exit(1)
        
        print("✓ YOLO detector loading...")
        detector = YoloWorldDetector("yolov8s-worldv2.pt", confidence=0.25)
        print("✓ YOLO detector ready")
        
        calibration = CameraCalibration(frame="map", translation=(0, 0, 0.05))
        camera = CameraDriver(source, detector, calibration)
        print("✓ Camera driver created")
        
        # Start camera service
        print("\nStarting camera WebSocket service on port 8767...")
        async with serve(HardwareService(camera).handler, "0.0.0.0", 8767, max_size=65536) as server:
            print("✓ Camera service listening on 0.0.0.0:8767")
            print("\nAttempting object detection (timeout 15s)...")
            
            try:
                # Test locate request
                reply = await asyncio.wait_for(
                    rpc("ws://127.0.0.1:8767", Request(action="locate", target="bottle")),
                    timeout=15
                )
                
                observation = reply.result
                print("\n" + "=" * 70)
                print("DETECTION RESULT")
                print("=" * 70)
                print(f"✓ Object detected!")
                print(f"\n  Target: {observation['target']}")
                print(f"  Pose:")
                print(f"    Frame: {observation['pose']['frame']}")
                print(f"    X: {observation['pose']['x']:.3f} m")
                print(f"    Y: {observation['pose']['y']:.3f} m")
                print(f"    Z: {observation['pose']['z']:.3f} m")
                print(f"    Yaw: {observation['pose'].get('yaw', 0):.3f} rad")
                print(f"\n  Confidence: {observation['confidence']:.2%}")
                print(f"  Observed at: {observation['observed_at']}")
                print("\n" + "=" * 70)
                
            except asyncio.TimeoutError:
                print("✗ Detection timeout (15s)")
            except Exception as e:
                print(f"✗ Detection failed: {e}")
        
        print("\n✓ Camera service stopped")
        
    except ImportError as e:
        print(f"✗ Import error: {e}")
        print("\nInstalling pyrealsense2 from source...")
        os.system("cd /tmp && git clone https://github.com/IntelRealSense/librealsense.git")
        os.system("cd /tmp/librealsense && mkdir -p build && cd build && cmake .. -DBUILD_PYTHON_BINDINGS:bool=true && make -j4")
        sys.exit(1)
    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    asyncio.run(main())
