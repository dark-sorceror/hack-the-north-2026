#!/usr/bin/env python3
"""Run camera service on Pi using OpenCV instead of RealSense SDK."""
import asyncio
import os
import sys

async def main():
    os.environ["ROBOT_TOKEN"] = "test-token-12345678901234567890"
    
    print("=" * 70)
    print("VLA CAMERA SERVICE - RASPBERRY PI (OpenCV Backend)")
    print("=" * 70)
    
    try:
        import cv2
        import numpy as np
        from robot_app.hardware import HardwareService
        from robot_app.camera import CameraDriver, Detection, CapturedFrame, CameraCalibration
        from robot_app.protocol import Request, rpc, token
        from websockets.asyncio.server import serve
        
        print("\n✓ OpenCV imported")
        token()
        print("✓ Token configured")
        
        # Create a simple camera source using OpenCV
        class OpenCVSource:
            def __init__(self, device=0, width=640, height=480):
                self.cap = cv2.VideoCapture(device)
                if not self.cap.isOpened():
                    raise RuntimeError(f"Cannot open camera device {device}")
                # Try to set resolution and FPS
                for _ in range(3):
                    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                    self.cap.set(cv2.CAP_PROP_FPS, 30)
                # Get actual resolution
                actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
                print(f"  ✓ OpenCV camera opened: {actual_width}x{actual_height}@{actual_fps:.1f}fps")
            
            def capture(self):
                import time
                ret, frame = self.cap.read()
                if not ret:
                    raise RuntimeError("Failed to capture frame")
                # Convert BGR to RGB
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                return CapturedFrame(frame_rgb, time.time())
            
            def point_in_camera(self, frame, pixel_x, pixel_y):
                # Simple depth estimation (assume object is at 0.5m)
                # In real scenario, would use stereo or depth sensor
                return (float(pixel_x) / 640 * 0.5, float(pixel_y) / 480 * 0.5, 0.5)
            
            def close(self):
                self.cap.release()
        
        print("\nInitializing camera...")
        source = None
        for device_id in [0, 1, 2, 3, 4, 5]:
            try:
                source = OpenCVSource(device=device_id, width=640, height=480)
                break
            except RuntimeError as e:
                print(f"  ✗ Device {device_id}: {e}")
        
        if source is None:
            raise RuntimeError("Could not open any camera device")
        
        print("✓ YOLO detector loading...")
        from robot_app.camera import YoloWorldDetector
        detector = YoloWorldDetector("yolov8s-worldv2.pt", confidence=0.25)
        print("✓ YOLO detector ready")
        
        calibration = CameraCalibration(frame="map", translation=(0, 0, 0.05))
        camera = CameraDriver(source, detector, calibration)
        print("✓ Camera driver created")
        
        # Start camera service
        print("\nStarting camera WebSocket service on port 8767...")
        async with serve(HardwareService(camera).handler, "0.0.0.0", 8767, max_size=65536) as server:
            print("✓ Camera service listening on 0.0.0.0:8767")
            print("\n" + "-" * 70)
            print("WAITING FOR DETECTION REQUESTS (Press Ctrl+C to stop)")
            print("-" * 70)
            print("\nTo test from another Pi, run:")
            print("  python -c \"import asyncio; from robot_app.protocol import Request, rpc; asyncio.run(rpc('ws://gisoopi.local:8767', Request(action='locate', target='bottle')))\"")
            print()
            
            # Keep service running
            await asyncio.Future()
        
    except ImportError as e:
        print(f"✗ Import error: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n✓ Camera service stopped")
