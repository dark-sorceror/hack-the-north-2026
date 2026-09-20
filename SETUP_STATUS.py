#!/usr/bin/env python3
"""
VLA-HTN Camera System - Setup Status Report
Generated: 2026-09-20

WHAT'S BEEN DONE ✓
==================

1. Virtual Environment
   ✓ Created: ~/VLA-HTN/venv
   ✓ Python: 3.13.5
   ✓ Status: Ready to use

2. System Dependencies  
   ✓ libssl-dev, libusb-1.0-0-dev, libudev-dev, pkg-config
   ✓ cmake, build-essential, git, wget
   ✓ libgtk-3-dev, libgl1-mesa-dev, libglu1-mesa-dev
   ✓ Status: All installed

3. Python Camera Packages
   ✓ numpy>=2,<3          (array operations)
   ✓ pillow>=10,<13       (image processing)
   ✓ ultralytics>=8.3,<9  (YOLO-World object detection)
   ✓ sounddevice>=0.5,<0.6 (microphone input)
   ✓ websockets>=15,<16   (WebSocket communication)
   ✓ pydantic>=2.10,<3    (data validation)
   ✓ Status: All installed

4. Project Installation
   ✓ Installed in editable mode (pip install -e .)
   ✓ Modules ready: robot_app, vla_core
   ✓ Status: Ready to import

5. RealSense SDK - Building from Source
   ✓ Source cloned: ~/VLA-HTN/librealsense
   ✓ CMake configured: build/ directory ready
   ✓ Python bindings enabled: -DBUILD_PYTHON_BINDINGS:bool=true
   ✓ Compilation started: make -j4
   ⏳ ETA: 15-20 minutes (still running)
   

WHAT NEEDS YOUR ACTION
======================

OPTION 1: Auto-Setup Script (Recommended)
──────────────────────────────────────────
This script handles all remaining steps with sudo:

   $ cd ~/VLA-HTN
   $ bash setup_realsense_pi.sh

The script will:
1. Wait for make to complete
2. Run: sudo make install (will prompt for sudo password)
3. Configure PYTHONPATH in ~/.bashrc
4. Verify installation
5. Show final status

You'll need to provide your Raspberry Pi sudo password once when prompted.


OPTION 2: Manual Steps
──────────────────────
If you prefer to run commands manually:

   # 1. Wait for compilation to finish (check if you see "Built target" messages)
   $ cd ~/VLA-HTN/librealsense/build
   $ # Monitor with: watch "ps aux | grep make"
   
   # 2. Once done (process ends), install:
   $ sudo make install
   
   # 3. Add PYTHONPATH to ~/.bashrc:
   $ echo 'export PYTHONPATH=$PYTHONPATH:/usr/local/lib' >> ~/.bashrc
   $ source ~/.bashrc
   
   # 4. Verify:
   $ python3 -c "import pyrealsense2; print(pyrealsense2.__version__)"


TESTING YOUR SETUP
==================

Once compilation completes, test with:

   $ cd ~/VLA-HTN
   $ source venv/bin/activate
   
   # Check all camera dependencies
   $ python3 -c "import numpy; import pillow; import ultralytics; import pyrealsense2; print('✓ All ready')"
   
   # Run quick camera test
   $ python3 test_hardware_detection.py
   
   # Run full vision test
   $ python3 test_vision_integration.py --mode vision


IMPORTANT NOTES
===============

1. RealSense Compilation
   - Building from SOURCE (not pre-built wheels)
   - Reason: Pre-built wheels don't support ARM64
   - Time: Takes 15-20 min on Pi 5
   - Status: STILL RUNNING - do NOT interrupt

2. PYTHONPATH Configuration
   - Must add /usr/local/lib to Python path
   - Installed via: sudo make install
   - Added to ~/.bashrc for persistence

3. Virtual Environment
   - Always activate before running code:
     $ source ~/VLA-HTN/venv/bin/activate
   - This is now configured in most startup scripts

4. Camera Hardware
   - Connect D435i to BLUE USB 3.0 port
   - Resolution: 640x480 @ 30 FPS
   - Depth stream enabled
   - Tested OK previously

5. YOLO-World Model
   - Located: ~/VLA-HTN/yolov8s-worldv2.pt
   - Text-prompt based (can detect any object)
   - Default confidence: 0.25


QUICK REFERENCE
================

Activate environment:
   $ cd ~/VLA-HTN && source venv/bin/activate

Start camera service:
   $ python run_camera_service.py

Run tests:
   $ python test_vision_integration.py --mode vision
   $ python test_voice_vision_signal.py

View project structure:
   $ python FILE_INVENTORY.py


NEXT STEPS
==========

1. Wait for RealSense compilation to complete
   (You'll see "Built target" messages stop appearing)

2. Run setup script:
   $ bash ~/VLA-HTN/setup_realsense_pi.sh
   
   OR manually run sudo make install

3. Verify installation works:
   $ source ~/VLA-HTN/venv/bin/activate
   $ python3 -c "import pyrealsense2; print(pyrealsense2.__version__)"

4. Test camera detection:
   $ python3 test_hardware_detection.py

5. Review test plan and continue with integration tests:
   $ python START_HERE.py
   $ python test_vision_integration.py --mode vision


Questions? Check:
  - ARCHITECTURE_DIAGRAM.py (system overview)
  - TEST_PLAN.py (testing strategy)
  - QUICKSTART_SUMMARY.py (quick reference)
"""

if __name__ == "__main__":
    import sys
    lines = __doc__.strip().split("\n")
    for line in lines:
        print(line)
    
    # Optional: Show compilation status if running locally
    try:
        import subprocess
        result = subprocess.run(
            ["ps", "aux"],
            capture_output=True,
            text=True,
            timeout=2
        )
        if "make" in result.stdout:
            print("\n" + "="*60)
            print("COMPILATION STATUS")
            print("="*60)
            for line in result.stdout.split("\n"):
                if "make" in line.lower():
                    print(f"  {line}")
    except:
        pass
