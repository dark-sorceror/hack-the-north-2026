#!/usr/bin/env python3
"""
Camera Detection & Configuration Utility

Detects RealSense camera on Raspberry Pi and tests connectivity.
Runs diagnostics and configures the camera service.
"""

import sys
import subprocess
import asyncio
import json
import os
from typing import Optional

# ANSI colors for output
class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    RESET = '\033[0m'
    BOLD = '\033[1m'

def print_header(text: str):
    print(f"\n{Colors.BOLD}{Colors.BLUE}{'='*80}{Colors.RESET}")
    print(f"{Colors.BOLD}{Colors.BLUE}{text:^80}{Colors.RESET}")
    print(f"{Colors.BOLD}{Colors.BLUE}{'='*80}{Colors.RESET}\n")

def print_status(status: str, message: str):
    if status == "✓":
        print(f"{Colors.GREEN}✓{Colors.RESET} {message}")
    elif status == "✗":
        print(f"{Colors.RED}✗{Colors.RESET} {message}")
    elif status == "⚠":
        print(f"{Colors.YELLOW}⚠{Colors.RESET} {message}")
    elif status == "→":
        print(f"{Colors.BLUE}→{Colors.RESET} {message}")

def run_ssh_command(host: str, cmd: str) -> tuple[int, str, str]:
    """Run command on remote Raspberry Pi via SSH"""
    try:
        result = subprocess.run(
            ["ssh", host, cmd],
            capture_output=True,
            text=True,
            timeout=30
        )
        return result.returncode, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        return -1, "", "SSH command timed out"
    except Exception as e:
        return -1, "", str(e)

def check_ssh_connection(host: str) -> bool:
    """Test if we can SSH to the Raspberry Pi"""
    code, _, _ = run_ssh_command(host, "echo 'connected'")
    return code == 0

def detect_camera_on_pi(host: str) -> dict:
    """Detect RealSense camera on Raspberry Pi"""
    info = {
        "ssh_connected": False,
        "camera_detected_lsusb": False,
        "pyrealsense2_installed": False,
        "camera_visible_to_pyrealsense2": False,
        "usb_devices": [],
        "pyrealsense2_info": "",
        "errors": []
    }
    
    print_header("STEP 1: SSH Connection Test")
    
    # Test SSH connection
    if not check_ssh_connection(host):
        print_status("✗", f"Cannot connect to {host}")
        print_status("→", f"Make sure you're on the same network")
        print_status("→", f"Try: ping {host}")
        info["errors"].append(f"SSH connection to {host} failed")
        return info
    
    print_status("✓", f"Connected to {host}")
    info["ssh_connected"] = True
    
    # Check camera via lsusb
    print_header("STEP 2: Detect Camera via USB")
    code, stdout, stderr = run_ssh_command(host, "lsusb | grep -i 'realense\\|intel' || echo 'Not found in filtered list'")
    
    if "Intel" in stdout or "RealSense" in stdout or "8086" in stdout:
        print_status("✓", "RealSense camera detected in USB devices")
        info["camera_detected_lsusb"] = True
    else:
        print_status("⚠", "RealSense not found in lsusb output")
    
    # Show all USB devices for reference
    print_status("→", "All USB devices:")
    code, stdout, stderr = run_ssh_command(host, "lsusb")
    for line in stdout.strip().split('\n'):
        print(f"  {line}")
        if any(x in line.lower() for x in ['realense', 'intel', '8086', 'depth']):
            info["usb_devices"].append(line)
    
    # Check pyrealsense2 installation
    print_header("STEP 3: Check pyrealsense2 Installation")
    code, stdout, stderr = run_ssh_command(host, "python -c 'import pyrealsense2; print(pyrealsense2.__version__)'")
    
    if code == 0:
        print_status("✓", f"pyrealsense2 installed: {stdout.strip()}")
        info["pyrealsense2_installed"] = True
        info["pyrealsense2_info"] = stdout.strip()
    else:
        print_status("✗", "pyrealsense2 not installed")
        print_status("→", "Install with: bash setup_realsense_pi.sh")
        info["errors"].append("pyrealsense2 not installed")
    
    # Test if camera is visible to pyrealsense2
    print_header("STEP 4: Test pyrealsense2 Camera Detection")
    
    test_script = """
import sys
try:
    import pyrealsense2 as rs
    ctx = rs.context()
    devices = ctx.query_devices()
    print(f"Found {len(devices)} RealSense device(s)")
    for i, dev in enumerate(devices):
        print(f"Device {i}: {dev.get_info(rs.camera_info.name)}")
        print(f"  Serial: {dev.get_info(rs.camera_info.serial_number)}")
        print(f"  Product ID: {dev.get_info(rs.camera_info.product_id)}")
except ImportError:
    print("ERROR: pyrealsense2 not available")
    sys.exit(1)
except Exception as e:
    print(f"ERROR: {e}")
    sys.exit(1)
"""
    
    code, stdout, stderr = run_ssh_command(
        host,
        f"cd ~/VLA-HTN && source venv/bin/activate && python -c \"{test_script}\""
    )
    
    if code == 0:
        print_status("✓", "Camera is visible to pyrealsense2")
        print(stdout)
        info["camera_visible_to_pyrealsense2"] = True
    else:
        print_status("✗", "Camera not visible to pyrealsense2")
        if stderr:
            print(f"  Error: {stderr}")
        info["errors"].append("Camera not visible to pyrealsense2")
    
    return info

def test_camera_streaming(host: str, timeout: int = 10) -> bool:
    """Test if camera can stream frames"""
    print_header("STEP 5: Test Camera Streaming")
    
    test_script = """
import sys
sys.path.insert(0, '/root/VLA-HTN')
try:
    import pyrealsense2 as rs
    from robot_app.camera import RealSenseSource
    
    print("→ Initializing RealSense pipeline...")
    source = RealSenseSource()
    print("✓ RealSense initialized")
    
    print("→ Capturing test frame...")
    frame = source.capture()
    
    if frame is not None and len(frame) > 0:
        print(f"✓ Captured frame: RGB {frame.shape}")
    else:
        print("✗ No frame captured")
        sys.exit(1)
    
    source.release()
    print("✓ Camera streaming test passed")
except Exception as e:
    print(f"✗ Error: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
"""
    
    cmd = f"cd ~/VLA-HTN && source venv/bin/activate && timeout {timeout} python -c \"{test_script}\""
    code, stdout, stderr = run_ssh_command(host, cmd)
    
    print(stdout)
    if stderr:
        print(stderr)
    
    return code == 0

def start_camera_service(host: str) -> bool:
    """Start camera service on Raspberry Pi"""
    print_header("STEP 6: Starting Camera Service")
    
    print_status("→", f"Starting camera service on {host}:8767...")
    print_status("→", "The service will run in the background")
    
    # Note: This should be run in a separate terminal/screen session
    cmd = """
cd ~/VLA-HTN && source venv/bin/activate && \\
export ROBOT_TOKEN='test-token-12345678901234567890' && \\
nohup python -m robot_app.camera --host 0.0.0.0 --port 8767 > camera.log 2>&1 &
echo $! > camera.pid
sleep 2
cat camera.pid
"""
    
    code, stdout, stderr = run_ssh_command(host, cmd)
    
    if code == 0:
        print_status("✓", f"Camera service started (PID: {stdout.strip()})")
        print_status("→", "View logs with: ssh {host} tail -f ~/VLA-HTN/camera.log")
        return True
    else:
        print_status("✗", "Failed to start camera service")
        if stderr:
            print(f"Error: {stderr}")
        return False

def check_camera_service_running(host: str) -> bool:
    """Check if camera service is running"""
    print_header("STEP 7: Verify Camera Service")
    
    cmd = "ps aux | grep 'python.*camera' | grep -v grep"
    code, stdout, stderr = run_ssh_command(host, cmd)
    
    if code == 0 and stdout.strip():
        print_status("✓", "Camera service is running")
        print(stdout.strip())
        return True
    else:
        print_status("⚠", "Camera service not found running")
        print_status("→", "Check logs: ssh {host} tail ~/VLA-HTN/camera.log")
        return False

def test_camera_websocket(host: str, port: int = 8767) -> bool:
    """Test camera WebSocket connection"""
    print_header("STEP 8: Test WebSocket Connection")
    
    test_script = f"""
import asyncio
import websockets
import json

async def test():
    try:
        uri = "ws://{host}:{port}/camera"
        print(f"→ Connecting to {{uri}}...")
        
        async with websockets.connect(uri) as ws:
            print("✓ Connected to camera WebSocket")
            
            # Send test request
            request = {{
                "type": "detect",
                "target": "bottle",
                "token": "test-token-12345678901234567890"
            }}
            await ws.send(json.dumps(request))
            print("→ Sent detection request...")
            
            # Receive response
            response = await asyncio.wait_for(ws.recv(), timeout=5.0)
            data = json.loads(response)
            print(f"✓ Received response: {{data.get('type', 'unknown')}}")
            
            if 'detections' in data:
                print(f"  Found {len(data['detections'])} detection(s)")
    
    except asyncio.TimeoutError:
        print("⚠ WebSocket response timeout (camera may still be initializing)")
    except Exception as e:
        print(f"✗ WebSocket test failed: {{e}}")
        return False
    
    return True

asyncio.run(test())
"""
    
    cmd = f"cd ~/VLA-HTN && source venv/bin/activate && python -c \"{test_script}\""
    code, stdout, stderr = run_ssh_command(host, cmd)
    
    print(stdout)
    if stderr and "asyncio" not in stderr.lower():
        print(stderr)
    
    return "✓" in stdout or code == 0

def main():
    import argparse
    
    parser = argparse.ArgumentParser(
        description="Detect and configure camera on Raspberry Pi"
    )
    parser.add_argument(
        "--host",
        default="gisoopi.local",
        help="Raspberry Pi hostname or IP (default: gisoopi.local)"
    )
    parser.add_argument(
        "--detect-only",
        action="store_true",
        help="Only detect camera, don't start service"
    )
    parser.add_argument(
        "--start-service",
        action="store_true",
        help="Start camera service after detection"
    )
    parser.add_argument(
        "--test-stream",
        action="store_true",
        help="Test camera streaming capability"
    )
    parser.add_argument(
        "--test-websocket",
        action="store_true",
        help="Test WebSocket connection"
    )
    
    args = parser.parse_args()
    
    print_header(f"RealSense Camera Detection & Configuration")
    print_status("→", f"Target: {args.host}")
    
    # Run detection
    info = detect_camera_on_pi(args.host)
    
    # Summary
    print_header("DETECTION SUMMARY")
    print_status("✓" if info["ssh_connected"] else "✗", "SSH Connection")
    print_status("✓" if info["camera_detected_lsusb"] else "✗", "Camera in USB Devices")
    print_status("✓" if info["pyrealsense2_installed"] else "✗", "pyrealsense2 Installed")
    print_status("✓" if info["camera_visible_to_pyrealsense2"] else "✗", "Camera Visible to SDK")
    
    if info["errors"]:
        print(f"\n{Colors.RED}Issues found:{Colors.RESET}")
        for err in info["errors"]:
            print_status("✗", err)
    
    # Optional tests
    if args.test_stream and info["camera_visible_to_pyrealsense2"]:
        test_camera_streaming(args.host)
    
    # Start service
    if args.start_service and info["camera_visible_to_pyrealsense2"]:
        start_camera_service(args.host)
        # Check if running
        asyncio.sleep(2)
        check_camera_service_running(args.host)
    
    if args.test_websocket:
        test_camera_websocket(args.host)
    
    # Next steps
    print_header("NEXT STEPS")
    
    if not info["ssh_connected"]:
        print_status("1", "Fix SSH connection:")
        print(f"   ping {args.host}")
        print(f"   ssh gisooj@{args.host}")
    elif not info["pyrealsense2_installed"]:
        print_status("1", "Install pyrealsense2:")
        print("   bash setup_realsense_pi.sh")
    elif not info["camera_visible_to_pyrealsense2"]:
        print_status("1", "Camera not visible - check:")
        print("   • USB cable connection")
        print("   • Power supply")
        print(f"   • SSH to {args.host} and run: dmesg | tail -20")
    elif not args.start_service:
        print_status("1", "Start camera service:")
        print(f"   python detect_camera.py --host {args.host} --start-service")
    else:
        print_status("✓", "Camera is ready!")
        print_status("2", "Test vision detection:")
        print("   python test_vision_integration.py --mode vision")
    
    print()

if __name__ == "__main__":
    main()
