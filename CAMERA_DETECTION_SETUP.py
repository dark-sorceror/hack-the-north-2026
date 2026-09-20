#!/usr/bin/env python3
"""
Camera Detection - SSH Setup Guide & Local Commands

Since SSH key authentication isn't configured, here are your options:
"""

setup_guide = """
╔════════════════════════════════════════════════════════════════════════════════╗
║                      CAMERA DETECTION - SSH SETUP                              ║
╚════════════════════════════════════════════════════════════════════════════════╝

The detection script needs SSH access without passwords. Here's how to set it up:

OPTION 1: SSH Key-Based Authentication (RECOMMENDED - One-time setup)
════════════════════════════════════════════════════════════════════════════════

Step 1: Generate SSH key on Windows (PowerShell as Admin)
────────────────────────────────────────────────────────
  ssh-keygen -t rsa -b 4096 -f $env:USERPROFILE\\.ssh\\id_rsa -N ""
  
  This creates:
    ~/.ssh/id_rsa (private key - NEVER share)
    ~/.ssh/id_rsa.pub (public key)

Step 2: Copy public key to Raspberry Pi
──────────────────────────────────────────
  # You'll need to enter password ONE TIME
  $ ssh-copy-id -i ~/.ssh/id_rsa.pub gisooj@gisoopi.local
  
  Or manually (if ssh-copy-id doesn't work):
  $ ssh gisooj@gisoopi.local
  $ mkdir -p ~/.ssh
  $ cat >> ~/.ssh/authorized_keys
  (paste contents of ~/.ssh/id_rsa.pub and press Ctrl+D)

Step 3: Test SSH key authentication
───────────────────────────────────
  ssh gisooj@gisoopi.local "echo 'SSH key works!'"
  
  Should NOT ask for password

Step 4: Run camera detection
─────────────────────────────
  python detect_camera.py --host gisoopi.local --test-stream


OPTION 2: Run Commands Directly on Raspberry Pi
════════════════════════════════════════════════════════════════════════════════

If you're SSH'd directly into the Pi, run these commands:

# Check camera detection via USB
lsusb | grep -i realense

# Check if pyrealsense2 is installed
python -c "import pyrealsense2; print('pyrealsense2 version:', pyrealsense2.__version__)"

# Test camera detection
python << 'EOF'
import pyrealsense2 as rs
ctx = rs.context()
devices = ctx.query_devices()
print(f"Found {len(devices)} RealSense device(s)")
for i, dev in enumerate(devices):
    print(f"  Device {i}: {dev.get_info(rs.camera_info.name)}")
EOF

# Test camera streaming (from ~/VLA-HTN directory)
cd ~/VLA-HTN && source venv/bin/activate
python << 'EOF'
from robot_app.camera import RealSenseSource
source = RealSenseSource()
frame = source.capture()
print(f"✓ Captured frame: {frame.shape}")
source.release()
EOF


OPTION 3: Manual SSH Connection (Password each time)
════════════════════════════════════════════════════════════════════════════════

If you don't want to set up keys, connect manually and run:

$ ssh gisooj@gisoopi.local

Then run the commands from OPTION 2 directly on the Pi


════════════════════════════════════════════════════════════════════════════════
                        QUICK TROUBLESHOOTING
════════════════════════════════════════════════════════════════════════════════

"Cannot connect to gisoopi.local"
  └─ Try:
     • ping gisoopi.local
     • ssh gisooj@gisoopi.local
     • Check if Pi is on same network as your computer

"lsusb: command not found"
  └─ Check if camera is connected via: dmesg | tail -20

"pyrealsense2 not installed"
  └─ Install with: python quickstart.py --install

"Camera not detected by pyrealsense2"
  └─ Try:
     • Reconnect USB cable
     • Check power supply
     • Run: python quickstart.py --install (rebuild from source)


════════════════════════════════════════════════════════════════════════════════
                       CAMERA DETECTION COMMANDS
════════════════════════════════════════════════════════════════════════════════

Run these in order (after SSH is set up):

# 1. Check USB connection
ssh gisooj@gisoopi.local "lsusb"

# 2. Check pyrealsense2 installed
ssh gisooj@gisoopi.local "python -c 'import pyrealsense2; print(pyrealsense2.__version__)'"

# 3. Test camera visibility
ssh gisooj@gisoopi.local "cd ~/VLA-HTN && source venv/bin/activate && python -c 'import pyrealsense2 as rs; ctx = rs.context(); print(f\"Cameras: {len(ctx.query_devices())}\")'"

# 4. Test streaming
ssh gisooj@gisoopi.local "cd ~/VLA-HTN && source venv/bin/activate && python << 'EOF'
from robot_app.camera import RealSenseSource
source = RealSenseSource()
frame = source.capture()
print(f'Frame shape: {frame.shape}')
source.release()
EOF"

# 5. Start camera service
ssh gisooj@gisoopi.local "cd ~/VLA-HTN && source venv/bin/activate && export ROBOT_TOKEN='test-token-12345678901234567890' && nohup python -m robot_app.camera --host 0.0.0.0 --port 8767 > camera.log 2>&1 &"


════════════════════════════════════════════════════════════════════════════════
                              WHAT TO EXPECT
════════════════════════════════════════════════════════════════════════════════

After successful setup:

1. lsusb should show:
   Bus 002 Device 002: ID 8086:0b3a Intel Corp. RealSense D435i

2. pyrealsense2 should be: 2.53.x.x or higher

3. Camera detection should show:
   Found 1 RealSense device(s)
   Device 0: Intel RealSense D435i

4. Camera streaming should show:
   Frame shape: (480, 640, 3)

5. Camera service should start without errors and listen on 0.0.0.0:8767


════════════════════════════════════════════════════════════════════════════════
"""

print(setup_guide)

# Also provide Python one-liner for testing
print("\n" + "="*80)
print("COPY-PASTE COMMAND (after SSH key setup):")
print("="*80)
print("""
# Test all 5 steps:
ssh gisooj@gisoopi.local "lsusb" && \\
ssh gisooj@gisoopi.local "python -c 'import pyrealsense2; print(pyrealsense2.__version__)'" && \\
echo "✓ Basic checks passed"
""")
