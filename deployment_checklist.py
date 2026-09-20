#!/usr/bin/env python3
"""
VLA-HTN DEPLOYMENT CHECKLIST

Use this script to verify all components are correctly configured
before running the complete system on 3 Raspberry Pis.

USAGE:
  python deployment_checklist.py --check all      # Check everything
  python deployment_checklist.py --check config   # Check environment
  python deployment_checklist.py --check hardware # Check hardware
  python deployment_checklist.py --check network  # Check network
  python deployment_checklist.py --generate-script # Generate Pi setup scripts
"""

import argparse
import json
import logging
import os
import subprocess
import sys
from pathlib import Path
from typing import Dict, Any, List, Tuple

logging.basicConfig(
    level=logging.INFO,
    format="%(levelname)-8s | %(message)s"
)
logger = logging.getLogger(__name__)


class DeploymentChecker:
    """Verify deployment configuration."""
    
    def __init__(self):
        self.checks = {}
        self.warnings = []
        self.errors = []
    
    def check_environment(self) -> bool:
        """Check environment variables."""
        logger.info("\n" + "=" * 80)
        logger.info("CHECKING ENVIRONMENT VARIABLES")
        logger.info("=" * 80)
        
        required = {
            "ROBOT_TOKEN": ("Authentication token", 24),
            "CAMERA_URL": ("Camera service URL", None),
            "NAV2_BRIDGE_URL": ("Nav2 bridge URL", None),
            "ARM101_BRIDGE_URL": ("Arm101 bridge URL", None),
        }
        
        all_ok = True
        
        for var, (desc, min_len) in required.items():
            value = os.getenv(var)
            
            if not value:
                logger.warning(f"  ✗ {var}: NOT SET")
                self.warnings.append(f"{var} not set (using defaults)")
                all_ok = False
            elif min_len and len(value) < min_len:
                logger.error(f"  ✗ {var}: Too short (need {min_len}+ chars)")
                self.errors.append(f"{var} too short")
                all_ok = False
            else:
                if var == "ROBOT_TOKEN":
                    display = f"{value[:8]}...{value[-8:]}"
                else:
                    display = value
                logger.info(f"  ✓ {var}: {display}")
        
        self.checks["environment"] = all_ok
        return all_ok
    
    def check_python_environment(self) -> bool:
        """Check Python and dependencies."""
        logger.info("\n" + "=" * 80)
        logger.info("CHECKING PYTHON ENVIRONMENT")
        logger.info("=" * 80)
        
        all_ok = True
        
        # Check Python version
        import sys as sys_module
        py_version = sys_module.version_info
        if py_version >= (3, 10):
            logger.info(f"  ✓ Python {py_version.major}.{py_version.minor}.{py_version.micro}")
        else:
            logger.warning(f"  ✗ Python {py_version.major}.{py_version.minor} (recommended 3.10+)")
            all_ok = False
        
        # Check key dependencies
        required_packages = [
            ("websockets", "WebSocket client/server"),
            ("pydantic", "Data validation"),
            ("ultralytics", "YOLO detection"),
            ("numpy", "Numerical computing"),
            ("PIL", "Image processing"),
        ]
        
        for package, desc in required_packages:
            try:
                __import__(package)
                logger.info(f"  ✓ {package}: {desc}")
            except ImportError:
                logger.warning(f"  ✗ {package}: NOT INSTALLED ({desc})")
                self.warnings.append(f"Missing package: {package}")
                all_ok = False
        
        # Check RealSense (optional but recommended)
        try:
            import pyrealsense2
            logger.info(f"  ✓ pyrealsense2: Intel RealSense SDK")
        except ImportError:
            logger.warning(f"  ⚠ pyrealsense2: NOT INSTALLED (needed for real camera)")
            self.warnings.append("Missing pyrealsense2 - simulated camera will be used")
        
        self.checks["python"] = all_ok
        return all_ok
    
    def check_hardware(self) -> bool:
        """Check for hardware devices."""
        logger.info("\n" + "=" * 80)
        logger.info("CHECKING HARDWARE")
        logger.info("=" * 80)
        
        all_ok = True
        
        # Check USB devices
        try:
            result = subprocess.run(
                ["lsusb"],
                capture_output=True,
                text=True,
                timeout=5
            )
            usb_output = result.stdout
            
            # Look for RealSense
            if "RealSense" in usb_output or "Intel" in usb_output:
                logger.info("  ✓ Intel RealSense camera detected")
            else:
                logger.warning("  ⚠ RealSense camera not detected (USB may not be connected)")
                self.warnings.append("RealSense camera not found")
                all_ok = False
            
            # Look for audio devices
            if "Jabra" in usb_output or "SPEAK" in usb_output:
                logger.info("  ✓ Microphone detected")
            else:
                logger.warning("  ⚠ Microphone not detected")
        
        except Exception as e:
            logger.warning(f"  ⚠ Could not check USB devices: {e}")
        
        # Check network
        try:
            result = subprocess.run(
                ["ip", "link", "show"],
                capture_output=True,
                text=True,
                timeout=5
            )
            if "eth0" in result.stdout or "wlan0" in result.stdout:
                logger.info("  ✓ Network interface detected")
            else:
                logger.warning("  ⚠ No network interface found")
                all_ok = False
        except Exception as e:
            logger.warning(f"  ⚠ Could not check network: {e}")
        
        self.checks["hardware"] = all_ok
        return all_ok
    
    def check_network_connectivity(self) -> bool:
        """Check network connectivity to other Pis."""
        logger.info("\n" + "=" * 80)
        logger.info("CHECKING NETWORK CONNECTIVITY")
        logger.info("=" * 80)
        
        camera_url = os.getenv("CAMERA_URL", "ws://127.0.0.1:8767")
        nav2_url = os.getenv("NAV2_BRIDGE_URL", "ws://127.0.0.1:8770")
        arm101_url = os.getenv("ARM101_BRIDGE_URL", "ws://127.0.0.1:8771")
        
        urls = {
            "Camera": camera_url,
            "Nav2": nav2_url,
            "Arm101": arm101_url,
        }
        
        all_ok = True
        
        for name, url in urls.items():
            try:
                # Extract host from URL
                host = url.replace("ws://", "").replace("wss://", "").split(":")[0]
                port = url.split(":")[-1]
                
                # Try to ping or connect
                if host in ("127.0.0.1", "localhost"):
                    logger.info(f"  ✓ {name}: Local ({url})")
                else:
                    result = subprocess.run(
                        ["ping", "-c", "1", "-W", "2", host],
                        capture_output=True,
                        timeout=5
                    )
                    if result.returncode == 0:
                        logger.info(f"  ✓ {name}: Reachable ({url})")
                    else:
                        logger.warning(f"  ✗ {name}: Not reachable ({url})")
                        self.warnings.append(f"{name} Pi not reachable")
                        all_ok = False
            
            except Exception as e:
                logger.warning(f"  ⚠ {name}: Could not check ({e})")
        
        self.checks["network"] = all_ok
        return all_ok
    
    def check_files(self) -> bool:
        """Check required files exist."""
        logger.info("\n" + "=" * 80)
        logger.info("CHECKING FILES")
        logger.info("=" * 80)
        
        required_files = [
            ("robot_app/coordinator.py", "Coordinator logic"),
            ("robot_app/camera.py", "Camera service"),
            ("robot_app/hardware_integrations.py", "Hardware bridges"),
            ("nav2_bridge_server.py", "Nav2 bridge template"),
            ("arm101_bridge_server.py", "Arm101 bridge template"),
            ("system_orchestrator.py", "System orchestrator"),
            ("test_system_integration.py", "Integration tests"),
            ("yolov8s-worldv2.pt", "YOLO-World weights"),
        ]
        
        all_ok = True
        
        for filepath, desc in required_files:
            if Path(filepath).exists():
                logger.info(f"  ✓ {filepath}")
            else:
                logger.error(f"  ✗ {filepath}: NOT FOUND")
                self.errors.append(f"Missing: {filepath}")
                all_ok = False
        
        self.checks["files"] = all_ok
        return all_ok
    
    def print_summary(self) -> int:
        """Print final summary."""
        logger.info("\n" + "=" * 80)
        logger.info("DEPLOYMENT CHECK SUMMARY")
        logger.info("=" * 80)
        
        passed = sum(1 for v in self.checks.values() if v)
        total = len(self.checks)
        
        print(f"\nResults: {passed}/{total} checks passed\n")
        
        for check, passed in self.checks.items():
            status = "✓" if passed else "✗"
            print(f"  {status} {check.upper()}")
        
        if self.errors:
            print("\nERRORS (MUST FIX):")
            for error in self.errors:
                print(f"  ✗ {error}")
        
        if self.warnings:
            print("\nWARNINGS (SHOULD FIX):")
            for warning in self.warnings:
                print(f"  ⚠ {warning}")
        
        print()
        
        if self.errors:
            print("❌ DEPLOYMENT NOT READY - fix errors above")
            return 1
        elif self.warnings:
            print("⚠️  DEPLOYMENT READY WITH WARNINGS - some features may be limited")
            return 0
        else:
            print("✓ DEPLOYMENT READY - system is fully configured!")
            return 0
    
    async def run_all_checks(self) -> int:
        """Run all checks."""
        self.check_environment()
        self.check_python_environment()
        self.check_files()
        self.check_hardware()
        self.check_network_connectivity()
        
        return self.print_summary()
    
    async def run_check(self, check_type: str) -> int:
        """Run specific check."""
        if check_type == "environment":
            self.check_environment()
        elif check_type == "hardware":
            self.check_hardware()
        elif check_type == "network":
            self.check_network_connectivity()
        elif check_type == "python":
            self.check_python_environment()
        elif check_type == "files":
            self.check_files()
        elif check_type == "all":
            await self.run_all_checks()
        else:
            logger.error(f"Unknown check: {check_type}")
            return 1
        
        return self.print_summary()


def generate_pi_setup_scripts():
    """Generate setup scripts for each Pi."""
    
    scripts = {
        "camera_pi_setup.sh": """#!/bin/bash
# VLA-HTN Camera Pi Setup Script

echo "Installing VLA-HTN on Camera Pi..."

# Create directory
mkdir -p ~/VLA-HTN
cd ~/VLA-HTN

# Clone repo (or sync)
# git clone <repo-url> .

# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install dependencies
pip install --upgrade pip
pip install websockets pydantic ultralytics opencv-python numpy pillow

# Optional: Install RealSense support
# pip install pyrealsense2

# Set environment token (change this!)
export ROBOT_TOKEN="your-secure-token-at-least-24-characters-long"

# Start camera service
echo "Starting camera service..."
python run_camera_service.py &

# In another terminal, run tests:
# python system_orchestrator.py --mode status
""",

        "nav2_pi_setup.sh": """#!/bin/bash
# VLA-HTN Nav2 Pi Setup Script

echo "Installing VLA-HTN on Nav2 Pi..."

# Install basic dependencies
pip install websockets pydantic

# Create working directory
mkdir -p ~/nav2_bridge
cd ~/nav2_bridge

# Copy bridge server
# scp camera_pi:~/VLA-HTN/nav2_bridge_server.py .

# Set environment token (MUST MATCH CAMERA PI!)
export ROBOT_TOKEN="your-secure-token-at-least-24-characters-long"

# IMPORTANT: Customize nav2_bridge_server.py for your Nav2 stack
# Edit the Nav2Controller class to connect to your actual Nav2 services

# Start bridge server
echo "Starting Nav2 bridge server..."
python nav2_bridge_server.py --host 0.0.0.0 --port 8770

# Monitor logs:
# tail -f nav2_bridge.log
""",

        "arm101_pi_setup.sh": """#!/bin/bash
# VLA-HTN Arm101 Pi Setup Script

echo "Installing VLA-HTN on Arm101 Pi..."

# Install basic dependencies
pip install websockets pydantic

# Create working directory
mkdir -p ~/arm101_bridge
cd ~/arm101_bridge

# Copy bridge server
# scp camera_pi:~/VLA-HTN/arm101_bridge_server.py .

# Set environment token (MUST MATCH CAMERA PI!)
export ROBOT_TOKEN="your-secure-token-at-least-24-characters-long"

# IMPORTANT: Customize arm101_bridge_server.py for your ARM101 hardware
# Edit the Arm101Controller class to connect to your actual arm hardware
# - Serial/USB connection
# - GPIO pins for gripper
# - Grasp strategies for different objects

# Start bridge server
echo "Starting Arm101 bridge server..."
python arm101_bridge_server.py --host 0.0.0.0 --port 8771

# Monitor logs:
# tail -f arm101_bridge.log
""",

        "quick_test.sh": """#!/bin/bash
# Quick integration test

echo "VLA-HTN Quick Test"
echo "=================="

# Test 1: Check services are running
echo "Checking service connectivity..."
python system_orchestrator.py --mode status

# Test 2: Run vision only
echo -e "\\nTesting vision detection..."
python test_system_integration.py --step vision

# Test 3: Run full sequence
echo -e "\\nTesting complete sequence..."
python test_system_integration.py --full
"""
    }
    
    output_dir = Path(".")
    
    for filename, content in scripts.items():
        filepath = output_dir / filename
        with open(filepath, "w") as f:
            f.write(content)
        
        # Make scripts executable
        os.chmod(filepath, 0o755)
        logger.info(f"Generated: {filename}")
    
    logger.info("\nGenerated setup scripts. Configure each Pi by:")
    logger.info("  1. Edit camera_pi_setup.sh, nav2_pi_setup.sh, arm101_pi_setup.sh")
    logger.info("  2. Update ROBOT_TOKEN to same value on all Pis")
    logger.info("  3. Customize bridge servers for your hardware")
    logger.info("  4. Run scripts on each Pi")


async def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="VLA-HTN Deployment Checklist",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
EXAMPLES:
  # Check everything
  python deployment_checklist.py --check all
  
  # Check only environment variables
  python deployment_checklist.py --check environment
  
  # Check only hardware
  python deployment_checklist.py --check hardware
  
  # Check network connectivity
  python deployment_checklist.py --check network
  
  # Generate setup scripts for 3 Pis
  python deployment_checklist.py --generate-script
        """
    )
    
    parser.add_argument(
        "--check",
        choices=["all", "environment", "python", "hardware", "network", "files"],
        default="all",
        help="Which check to run"
    )
    
    parser.add_argument(
        "--generate-script",
        action="store_true",
        help="Generate Pi setup scripts"
    )
    
    args = parser.parse_args()
    
    if args.generate_script:
        generate_pi_setup_scripts()
        return 0
    
    checker = DeploymentChecker()
    
    if args.check == "all":
        exit_code = await checker.run_all_checks()
    else:
        exit_code = await checker.run_check(args.check)
    
    sys.exit(exit_code)


if __name__ == "__main__":
    import asyncio
    asyncio.run(main())
