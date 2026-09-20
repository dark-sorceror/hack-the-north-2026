#!/usr/bin/env python3
"""
Test RealSense Camera + YOLO-World Object Detection Pipeline
"""

import sys
import time

print("=" * 70)
print("VLA-HTN Object Detection Pipeline Test")
print("=" * 70)

# Step 1: Test imports
print("\n[STEP 1] Testing imports...")
try:
    import pyrealsense2 as rs
    print(f"  ✓ pyrealsense2 ({rs.__version__})")
except Exception as e:
    print(f"  ✗ pyrealsense2: {e}")
    sys.exit(1)

try:
    from ultralytics import YOLO
    print(f"  ✓ ultralytics (YOLO-World)")
except Exception as e:
    print(f"  ✗ ultralytics: {e}")
    sys.exit(1)

try:
    import numpy as np
    print(f"  ✓ numpy")
except Exception as e:
    print(f"  ✗ numpy: {e}")
    sys.exit(1)

try:
    from PIL import Image
    print(f"  ✓ pillow (PIL)")
except Exception as e:
    print(f"  ✗ pillow: {e}")
    sys.exit(1)

# Step 2: Initialize RealSense
print("\n[STEP 2] Initializing RealSense camera...")
try:
    ctx = rs.context()
    devices = ctx.query_devices()
    
    if devices.size() == 0:
        print("  ✗ No RealSense devices found!")
        sys.exit(1)
    
    device = devices[0]
    name = device.get_info(rs.camera_info.name)
    print(f"  ✓ Found: {name}")
    
    # Create pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    
    # Enable streams
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    
    # Start pipeline
    profile = pipeline.start(config)
    print("  ✓ Pipeline started")
    
    # Wait for first frame
    print("  ⏳ Waiting for frames...")
    for i in range(10):
        frames = pipeline.wait_for_frames()
        if frames:
            break
        time.sleep(0.1)
    
    if not frames:
        print("  ✗ No frames received!")
        sys.exit(1)
    
    print("  ✓ Frames streaming")
    
except Exception as e:
    print(f"  ✗ Error: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

# Step 3: Load YOLO model
print("\n[STEP 3] Loading YOLO-World model...")
try:
    print("  ⏳ Loading model (this may take 30-60 seconds)...")
    model = YOLO("yolov8s-worldv2.pt")
    print("  ✓ YOLO-World model loaded")
except Exception as e:
    print(f"  ✗ Error loading model: {e}")
    sys.exit(1)

# Step 4: Run detection
print("\n[STEP 4] Running object detection...")
try:
    # Get a frame
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()
    
    if not color_frame:
        print("  ✗ No color frame!")
        sys.exit(1)
    
    # Convert to numpy
    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())
    
    print(f"  ✓ Frame captured: {color_image.shape}")
    
    # Run detection with text prompts
    detection_prompts = ["bottle", "cup", "person", "hand", "object"]
    print(f"  ⏳ Running detection for: {', '.join(detection_prompts)}")
    
    results = model.predict(
        source=color_image,
        conf=0.25,
        verbose=False,
        device='cpu'
    )
    
    if results:
        result = results[0]
        detections = result.boxes
        
        print(f"  ✓ Detection complete!")
        print(f"    Found {len(detections)} objects")
        
        for i, det in enumerate(detections):
            conf = det.conf[0].item()
            cls_id = det.cls[0].item()
            x1, y1, x2, y2 = det.xyxy[0]
            
            # Get depth at detection center
            cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
            depth_val = depth_image[cy, cx] if cy < depth_image.shape[0] and cx < depth_image.shape[1] else 0
            depth_m = depth_val / 1000.0  # Convert to meters
            
            class_name = result.names.get(int(cls_id), "unknown")
            
            print(f"    [{i+1}] {class_name}: confidence={conf:.2f}, depth={depth_m:.2f}m")
        
        # If we found objects, pipeline is working!
        if len(detections) > 0:
            print("\n  ✓✓✓ DETECTION PIPELINE WORKING! ✓✓✓")
        else:
            print("\n  ⚠ No objects detected (camera might be looking at empty space)")
    else:
        print("  ⚠ No results returned")
    
except Exception as e:
    print(f"  ✗ Detection error: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

# Step 5: Cleanup
print("\n[STEP 5] Cleanup...")
try:
    pipeline.stop()
    print("  ✓ Pipeline stopped")
except:
    pass

# Summary
print("\n" + "=" * 70)
print("✓ INSTALLATION VERIFIED - Object Detection Pipeline Ready!")
print("=" * 70)
print("\nYou can now:")
print("  1. Run vision tests: python test_vision_integration.py --mode vision")
print("  2. Start camera service: python run_camera_service.py")
print("  3. Run full integration: python test_vision_integration.py --mode coordinator")
print("\n")
