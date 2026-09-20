#!/usr/bin/env python3
"""
Simple Camera Detection Script (Works with password prompts in PowerShell)
"""

import subprocess
import sys

def run_ssh_interactive(host: str, cmd: str):
    """Run SSH command with interactive password prompt"""
    full_cmd = f'ssh gisooj@{host} "{cmd}"'
    print(f"→ Running: {full_cmd}")
    print()
    result = subprocess.run(full_cmd, shell=True)
    return result.returncode

def main():
    host = "gisoopi.local"
    
    print("╔════════════════════════════════════════════════════════════════════════════════╗")
    print("║                     CAMERA DETECTION ON RASPBERRY PI                          ║")
    print("╚════════════════════════════════════════════════════════════════════════════════╝\n")
    
    print(f"Target Pi: {host}\n")
    
    checks = [
        ("Check USB devices", "lsusb | grep -i realense || echo 'Checking all USB devices:' && lsusb"),
        ("Check pyrealsense2", "python -c 'import pyrealsense2; print(f\"✓ pyrealsense2 version: {pyrealsense2.__version__}\")'"),
        ("Test camera visibility", "cd ~/VLA-HTN && source venv/bin/activate && python -c 'import pyrealsense2 as rs; ctx = rs.context(); devices = ctx.query_devices(); print(f\"Found {len(devices)} RealSense camera(s)\")'"),
    ]
    
    print("Step 1: Check Camera Detection\n")
    print("="*80 + "\n")
    
    for step_name, cmd in checks:
        print(f"\n{step_name}:")
        print("-" * 80)
        run_ssh_interactive(host, cmd)
    
    print("\n" + "="*80)
    print("\nStep 2: Test Camera Streaming\n")
    print("="*80 + "\n")
    
    streaming_test = """
cd ~/VLA-HTN && source venv/bin/activate && python << 'EOF'
import sys
try:
    from robot_app.camera import RealSenseSource
    print("→ Initializing RealSense camera...")
    source = RealSenseSource()
    
    print("→ Capturing frame...")
    frame = source.capture()
    
    if frame is not None:
        print(f"✓ Success! Captured frame shape: {frame.shape}")
    else:
        print("✗ No frame captured")
        sys.exit(1)
    
    source.release()
    print("✓ Camera streaming test passed")
except Exception as e:
    print(f"✗ Error: {e}")
    sys.exit(1)
EOF
"""
    
    run_ssh_interactive(host, streaming_test)
    
    print("\n" + "="*80)
    print("\nStep 3: Start Camera Service\n")
    print("="*80 + "\n")
    
    service_cmd = """cd ~/VLA-HTN && source venv/bin/activate && export ROBOT_TOKEN='test-token-12345678901234567890' && python -m robot_app.camera --host 0.0.0.0 --port 8767"""
    
    print("Starting camera service (Press Ctrl+C to stop):\n")
    run_ssh_interactive(host, service_cmd)
    
    print("\n" + "="*80)
    print("\nNOTE: Service is running on gisoopi.local:8767")
    print("To run tests from Windows:")
    print("  python test_vision_integration.py --mode vision")
    print("="*80 + "\n")

if __name__ == "__main__":
    main()
