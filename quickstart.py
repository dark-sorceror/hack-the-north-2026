#!/usr/bin/env python3
"""
QUICK START: Voice → Vision Testing Guide

This guides you through setting up and testing the first 2 steps of the fetch sequence.

STEPS:
  1. Install pyrealsense2 on camera Pi (~30 min)
  2. Start camera service on Pi
  3. Run vision detection tests
  4. Test coordinator with simulated hardware
"""

import subprocess
import sys
import os


def run_command(cmd, description, show_output=True):
    """Run a shell command and return success status."""
    print(f"\n{'='*80}")
    print(f"{description}")
    print(f"{'='*80}\n")
    
    if show_output:
        try:
            result = subprocess.run(cmd, shell=True, check=True)
            return result.returncode == 0
        except subprocess.CalledProcessError as e:
            print(f"\n✗ Command failed with exit code {e.returncode}")
            return False
    else:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        print(result.stdout)
        if result.stderr:
            print("STDERR:", result.stderr)
        return result.returncode == 0


def step_1_install_pyrealsense():
    """Install pyrealsense2 on camera Pi."""
    print("\n" + "="*80)
    print("STEP 1: Install pyrealsense2 on Camera Pi")
    print("="*80)
    
    pi_host = input("\nEnter camera Pi hostname (default: gisooj@gisoopi.local): ").strip()
    if not pi_host:
        pi_host = "gisooj@gisoopi.local"
    
    token = input("Enter ROBOT_TOKEN (or press Enter for default): ").strip()
    if not token:
        token = "test-token-12345678901234567890"
    
    print(f"\n[INFO] Will install pyrealsense2 on: {pi_host}")
    print(f"[INFO] This takes about 30 minutes and requires compilation")
    
    confirm = input("\nContinue? (y/n): ").strip().lower()
    if confirm != 'y':
        return False
    
    # Build commands
    cmds = [
        f"ssh {pi_host} 'cd /tmp && rm -rf librealsense'",
        f"ssh {pi_host} 'cd /tmp && git clone https://github.com/IntelRealSense/librealsense.git'",
        f"ssh {pi_host} 'cd /tmp/librealsense && mkdir -p build && cd build && cmake .. -DBUILD_PYTHON_BINDINGS:bool=true -DCMAKE_BUILD_TYPE=Release'",
        f"ssh {pi_host} 'cd /tmp/librealsense/build && make -j4'",
        f"ssh {pi_host} 'cd /tmp/librealsense/build && sudo make install'",
        f"ssh {pi_host} 'cd /tmp/librealsense/wrappers/python && python setup.py install'",
    ]
    
    for i, cmd in enumerate(cmds, 1):
        print(f"\n[{i}/{len(cmds)}] ", end="", flush=True)
        if not run_command(cmd, f"Building pyrealsense2 ({i}/{len(cmds)})"):
            print(f"\n✗ Build failed at step {i}")
            return False
    
    # Verify
    print("\n[VERIFY] Testing pyrealsense2 import...")
    verify_cmd = f"ssh {pi_host} 'source ~/VLA-HTN/venv/bin/activate && python -c \"import pyrealsense2; print(\\\"✓ pyrealsense2 ready\\\")\"'"
    
    if run_command(verify_cmd, "Verifying pyrealsense2", show_output=True):
        print("\n✓ STEP 1 COMPLETE: pyrealsense2 installed")
        return True
    else:
        print("\n✗ STEP 1 FAILED: pyrealsense2 verification failed")
        return False


def step_2_start_camera_service():
    """Start camera service on Pi."""
    print("\n" + "="*80)
    print("STEP 2: Start Camera Service on Pi")
    print("="*80)
    
    pi_host = input("\nEnter camera Pi hostname (default: gisooj@gisoopi.local): ").strip()
    if not pi_host:
        pi_host = "gisooj@gisoopi.local"
    
    token = input("Enter ROBOT_TOKEN (or press Enter for default): ").strip()
    if not token:
        token = "test-token-12345678901234567890"
    
    print(f"\n[INFO] Starting camera service on {pi_host}")
    print(f"[INFO] The service will run until you press Ctrl+C")
    print(f"\nTo run in background, use:")
    print(f"  nohup ssh {pi_host} 'export ROBOT_TOKEN=\\\"{token}\\\" && python -m robot_app.camera' > camera.log 2>&1 &")
    
    cmd = f"ssh {pi_host} 'cd ~/VLA-HTN && source venv/bin/activate && export ROBOT_TOKEN=\\\"{token}\\\" && python -m robot_app.camera --host 0.0.0.0 --port 8767'"
    
    print(f"\n[START] Running camera service (Ctrl+C to stop)...\n")
    return run_command(cmd, "Camera Service", show_output=True)


def step_3_test_vision():
    """Test vision detection."""
    print("\n" + "="*80)
    print("STEP 3: Test Vision Detection")
    print("="*80)
    
    pi_host = input("\nEnter camera Pi hostname (default: gisooji.local): ").strip()
    if not pi_host:
        pi_host = "gisooji.local"
    
    token = input("Enter ROBOT_TOKEN (or press Enter for default): ").strip()
    if not token:
        token = "test-token-12345678901234567890"
    
    camera_url = f"ws://{pi_host}:8767"
    
    print(f"\n[INFO] Testing vision detection")
    print(f"[INFO] Camera URL: {camera_url}")
    
    env = os.environ.copy()
    env["CAMERA_URL"] = camera_url
    env["ROBOT_TOKEN"] = token
    
    cmd = f"python test_vision_integration.py --mode vision --camera-url {camera_url} --token {token}"
    
    return run_command(cmd, "Vision Detection Test", show_output=True)


def step_4_test_coordinator():
    """Test coordinator (full sequence with simulated hardware)."""
    print("\n" + "="*80)
    print("STEP 4: Test Coordinator")
    print("="*80)
    
    pi_host = input("\nEnter camera Pi hostname (default: gisooji.local): ").strip()
    if not pi_host:
        pi_host = "gisooji.local"
    
    token = input("Enter ROBOT_TOKEN (or press Enter for default): ").strip()
    if not token:
        token = "test-token-12345678901234567890"
    
    camera_url = f"ws://{pi_host}:8767"
    
    print(f"\n[INFO] Testing full coordinator sequence")
    print(f"[INFO] Camera URL: {camera_url}")
    
    cmd = f"python test_vision_integration.py --mode coordinator --camera-url {camera_url} --token {token}"
    
    return run_command(cmd, "Coordinator Test", show_output=True)


def main():
    """Interactive setup wizard."""
    print("""
    ╔════════════════════════════════════════════════════════════════════════════╗
    ║                   VOICE → VISION INTEGRATION TEST                         ║
    ║                          QUICK START GUIDE                                ║
    ╚════════════════════════════════════════════════════════════════════════════╝
    
    This will guide you through:
      1. Install pyrealsense2 on camera Pi (~30 min)
      2. Start camera service on Pi
      3. Test vision detection
      4. Test coordinator (full sequence)
    
    ═════════════════════════════════════════════════════════════════════════════
    """)
    
    while True:
        print("\n[MENU]")
        print("  1. Install pyrealsense2 (one-time setup)")
        print("  2. Start camera service on Pi")
        print("  3. Test vision detection")
        print("  4. Test coordinator")
        print("  5. Show demo flow")
        print("  0. Exit")
        
        choice = input("\nSelect option (0-5): ").strip()
        
        if choice == "1":
            if not step_1_install_pyrealsense():
                print("\n✗ Installation failed")
        elif choice == "2":
            step_2_start_camera_service()
        elif choice == "3":
            if not step_3_test_vision():
                print("\n✗ Vision test failed")
        elif choice == "4":
            if not step_4_test_coordinator():
                print("\n✗ Coordinator test failed")
        elif choice == "5":
            run_command("python test_vision_integration.py --mode demo", "Demo Flow")
        elif choice == "0":
            print("\n✓ Goodbye!")
            break
        else:
            print("\n✗ Invalid option")


if __name__ == "__main__":
    if len(sys.argv) > 1:
        # Command line mode
        if sys.argv[1] == "--install":
            step_1_install_pyrealsense()
        elif sys.argv[1] == "--start-camera":
            step_2_start_camera_service()
        elif sys.argv[1] == "--test-vision":
            step_3_test_vision()
        elif sys.argv[1] == "--test-coordinator":
            step_4_test_coordinator()
        else:
            print(f"Unknown option: {sys.argv[1]}")
    else:
        # Interactive mode
        main()
