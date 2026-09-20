#!/usr/bin/env python3
"""Test RealSense installation and object detection pipeline"""

import sys
import os

print("=" * 60)
print("VLA-HTN Installation Verification")
print("=" * 60)

# Test 1: Check Python environment
print("\n[1/4] Python Environment")
print(f"  Python: {sys.version.split()[0]}")
print(f"  Executable: {sys.executable}")

# Test 2: Check core dependencies
print("\n[2/4] Core Dependencies")
deps_ok = True
core_deps = {
    'numpy': 'Array operations',
    'pydantic': 'Data validation',
    'websockets': 'WebSocket communication',
    'ultralytics': 'YOLO-World detection',
}

for dep, description in core_deps.items():
    try:
        __import__(dep)
        print(f"  ✓ {dep:20} ({description})")
    except ImportError:
        print(f"  ✗ {dep:20} MISSING")
        deps_ok = False

# Test 3: Check camera/audio dependencies  
print("\n[3/4] Camera & Audio Dependencies")
camera_deps = {
    'pillow': 'Image processing',
    'sounddevice': 'Audio capture',
}

for dep, description in camera_deps.items():
    try:
        __import__(dep)
        print(f"  ✓ {dep:20} ({description})")
    except ImportError:
        print(f"  ✗ {dep:20} MISSING")
        deps_ok = False

# Test 4: Check RealSense
print("\n[4/4] RealSense SDK")
try:
    import pyrealsense2 as rs
    print(f"  ✓ pyrealsense2        (version: {rs.__version__})")
    
    # Try to enumerate devices
    try:
        ctx = rs.context()
        devices = ctx.query_devices()
        num_devices = devices.size()
        print(f"  ✓ Device enumeration  ({num_devices} device(s) available)")
        
        if num_devices > 0:
            for i in range(num_devices):
                dev = devices[i]
                name = dev.get_info(rs.camera_info.name)
                print(f"    - Device {i}: {name}")
    except Exception as e:
        print(f"  ⚠ Device enumeration failed: {e}")
        
except ImportError as e:
    print(f"  ✗ pyrealsense2        FAILED: {e}")
    deps_ok = False

# Test 5: Check project modules
print("\n[5/5] Project Modules")
project_ok = True
try:
    import robot_app
    print(f"  ✓ robot_app           (imported)")
except ImportError as e:
    print(f"  ✗ robot_app           FAILED: {e}")
    project_ok = False

try:
    import vla_core
    print(f"  ✓ vla_core            (imported)")
except ImportError as e:
    print(f"  ✗ vla_core            FAILED: {e}")
    project_ok = False

# Summary
print("\n" + "=" * 60)
if deps_ok and project_ok:
    print("✓ INSTALLATION COMPLETE - Ready for testing!")
    print("\nNext steps:")
    print("  1. Run object detection test:")
    print("     $ python test_hardware_detection.py")
    print("  2. Run vision integration test:")
    print("     $ python test_vision_integration.py --mode vision")
    sys.exit(0)
else:
    print("✗ Installation incomplete - some dependencies missing")
    sys.exit(1)
