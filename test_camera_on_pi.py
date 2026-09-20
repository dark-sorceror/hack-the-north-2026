#!/usr/bin/env python3
"""Test camera and object detection on Raspberry Pi.

Usage:
  ssh gisooj@gisoopi.local "cd ~/VLA-HTN && python test_camera_on_pi.py"
"""
import asyncio
import json
import sys
from pathlib import Path

async def test_camera_hardware():
    """Test RealSense camera detection and basic capture."""
    print("=" * 60)
    print("TESTING REALSENSE CAMERA")
    print("=" * 60)
    
    try:
        import pyrealsense2 as rs
        print("✓ pyrealsense2 imported")
        
        # Detect cameras
        ctx = rs.context()
        devices = ctx.query_devices()
        
        if len(devices) == 0:
            print("✗ No RealSense cameras detected")
            return False
        
        print(f"✓ Found {len(devices)} RealSense device(s)")
        
        for i, device in enumerate(devices):
            serial = device.get_info(rs.camera_info.serial_number)
            name = device.get_info(rs.camera_info.name)
            print(f"  Device {i}: {name} (Serial: {serial})")
        
        # Try to start pipeline
        print("\nTesting pipeline startup...")
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        try:
            profile = pipeline.start(config)
            print("✓ Pipeline started successfully")
            
            # Capture a frame
            print("Capturing test frame...")
            frames = pipeline.wait_for_frames(timeout_ms=5000)
            if frames:
                print(f"✓ Captured frame set with {frames.size()} streams")
            
            pipeline.stop()
            print("✓ Pipeline stopped cleanly")
            return True
        except Exception as e:
            print(f"✗ Pipeline error: {e}")
            return False
            
    except ImportError as e:
        print(f"✗ Import error: {e}")
        return False
    except Exception as e:
        print(f"✗ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        return False


async def test_yolo_detector():
    """Test YOLO object detection."""
    print("\n" + "=" * 60)
    print("TESTING YOLO OBJECT DETECTION")
    print("=" * 60)
    
    try:
        from ultralytics import YOLOWorld
        import numpy as np
        
        print("✓ Ultralytics imported")
        
        # Check model file
        model_path = "yolov8s-worldv2.pt"
        if not Path(model_path).exists():
            print(f"✗ Model file not found: {model_path}")
            return False
        
        print(f"✓ Model file found: {model_path}")
        
        # Load model
        print("Loading YOLO model...")
        model = YOLOWorld(model_path)
        print("✓ YOLO model loaded")
        
        # Test detection on dummy image
        print("\nTesting detection...")
        dummy_image = np.zeros((480, 640, 3), dtype=np.uint8)
        model.set_classes(["bottle"])
        results = model.predict(dummy_image, conf=0.25, verbose=False)
        
        print(f"✓ Detection executed: {len(results)} result(s)")
        
        # Test with actual camera frame if available
        try:
            import pyrealsense2 as rs
            print("\nTesting detection with real camera frame...")
            
            ctx = rs.context()
            devices = ctx.query_devices()
            if devices:
                pipeline = rs.pipeline()
                config = rs.config()
                config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
                pipeline.start(config)
                
                frames = pipeline.wait_for_frames(timeout_ms=5000)
                color_frame = frames.get_color_frame()
                
                if color_frame:
                    import numpy as np
                    frame_data = np.asanyarray(color_frame.get_data())
                    
                    results = model.predict(frame_data, conf=0.25, verbose=False)
                    if results and results[0].boxes:
                        print(f"✓ Detected {len(results[0].boxes)} object(s) in camera frame!")
                        for conf in results[0].boxes.conf:
                            print(f"  - Confidence: {conf:.2f}")
                    else:
                        print("✓ No objects detected in camera frame (this is OK)")
                
                pipeline.stop()
        except Exception as e:
            print(f"⚠ Real camera test skipped: {e}")
        
        return True
        
    except ImportError as e:
        print(f"✗ Import error: {e}")
        return False
    except Exception as e:
        print(f"✗ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        return False


async def test_camera_service():
    """Test camera service RPC."""
    print("\n" + "=" * 60)
    print("TESTING CAMERA SERVICE")
    print("=" * 60)
    
    try:
        from robot_app.camera import CameraDriver, RealSenseSource, YoloWorldDetector, CameraCalibration
        from robot_app.hardware import HardwareService
        from robot_app.protocol import Request, rpc, token
        from websockets.asyncio.server import serve
        import os
        
        print("✓ Camera service modules imported")
        
        # Set token
        os.environ["ROBOT_TOKEN"] = "test-token-12345678901234567890"
        token()
        print("✓ Token configured")
        
        # Check hardware availability
        try:
            source = RealSenseSource()
            detector = YoloWorldDetector()
            calibration = CameraCalibration(frame="map", translation=(0, 0, 0))
            
            camera = CameraDriver(source, detector, calibration)
            print("✓ Camera driver initialized")
            
            # Start service
            print("\nStarting camera service...")
            async with serve(HardwareService(camera).handler, "127.0.0.1", 8767, max_size=65536) as server:
                print(f"✓ Camera service listening on port 8767")
                
                # Test locate request
                print("\nSending locate request...")
                reply = await asyncio.wait_for(
                    rpc("ws://127.0.0.1:8767", Request(action="locate", target="bottle")),
                    timeout=15
                )
                
                observation = reply.result
                print(f"✓ Detection succeeded!")
                print(f"  Target: {observation['target']}")
                print(f"  Pose: x={observation['pose']['x']:.2f}, y={observation['pose']['y']:.2f}, z={observation['pose']['z']:.2f}")
                print(f"  Confidence: {observation['confidence']:.2f}")
                print(f"  Frame: {observation['pose']['frame']}")
                
            return True
            
        except RuntimeError as e:
            print(f"✗ Camera not available: {e}")
            print("  (This is expected if RealSense camera is not connected)")
            return False
            
    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return False


async def main():
    print("\n" + "=" * 60)
    print("VLA RASPBERRY PI HARDWARE TEST")
    print("=" * 60 + "\n")
    
    results = {}
    
    results["camera_hardware"] = await test_camera_hardware()
    results["yolo_detector"] = await test_yolo_detector()
    results["camera_service"] = await test_camera_service()
    
    print("\n" + "=" * 60)
    print("TEST SUMMARY")
    print("=" * 60)
    
    for test_name, passed in results.items():
        status = "✓ PASS" if passed else "✗ FAIL"
        print(f"{status}: {test_name}")
    
    all_passed = all(results.values())
    print(f"\nOverall: {'✓ ALL TESTS PASSED' if all_passed else '✗ SOME TESTS FAILED'}\n")
    
    return 0 if all_passed else 1


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
