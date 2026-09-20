#!/usr/bin/env python3
import sys
import os

print("Current Python:", sys.executable)
print("\nPYTHONPATH entries:")
for p in sys.path:
    if p.strip():
        print(f"  {p}")

print("\nLD_LIBRARY_PATH:", os.environ.get('LD_LIBRARY_PATH', 'NOT SET'))
print("PYTHONPATH env:", os.environ.get('PYTHONPATH', 'NOT SET'))

# Try importing pyrealsense2
print("\nAttempting to import pyrealsense2...")
try:
    import pyrealsense2
    print("SUCCESS! Version:", pyrealsense2.__version__)
except Exception as e:
    print(f"FAILED: {e}")
    import traceback
    traceback.print_exc()
    
    # Try direct import from build dir
    print("\n\nTrying to import from build directory directly...")
    sys.path.insert(0, '/home/gisooj/VLA-HTN/librealsense/build/Release')
    try:
        import pyrealsense2
        print("SUCCESS after adding build dir! Version:", pyrealsense2.__version__)
    except Exception as e2:
        print(f"Still failed: {e2}")
