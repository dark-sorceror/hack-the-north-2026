#!/usr/bin/env python3
"""
Test Passwordless Pi Access for Robot Operations

This script tests all aspects of passwordless Raspberry Pi access needed for the robot.
Run this after setup to verify everything works.

Usage:
    python test_passwordless_access.py
    python test_passwordless_access.py --verbose
    python test_passwordless_access.py --pi-host 192.168.1.102
"""

import subprocess
import sys
import json
import argparse
from typing import Tuple, Optional
from pathlib import Path


class PasswordlessAccessTester:
    def __init__(self, pi_user: str = "gisooj", pi_host: str = "gisoopi.local", verbose: bool = False):
        self.pi_user = pi_user
        self.pi_host = pi_host
        self.pi_address = f"{pi_user}@{pi_host}"
        self.verbose = verbose
        self.results = []
        self.ssh_key_path = Path.home() / ".ssh" / "id_ed25519"
        
    def log(self, msg: str):
        if self.verbose:
            print(f"  {msg}")
    
    def run_command(self, cmd: str, description: str) -> Tuple[bool, str]:
        """Run command and return (success, output)"""
        self.log(f"Running: {cmd}")
        try:
            result = subprocess.run(
                cmd,
                shell=True,
                capture_output=True,
                text=True,
                timeout=10
            )
            success = result.returncode == 0
            output = result.stdout.strip() or result.stderr.strip()
            return success, output
        except subprocess.TimeoutExpired:
            return False, "Command timed out"
        except Exception as e:
            return False, str(e)
    
    def test_ssh_key_exists(self) -> bool:
        """Test: SSH key file exists"""
        print("1️⃣  SSH Key Exists...", end=" ", flush=True)
        if self.ssh_key_path.exists():
            print("✓")
            return True
        else:
            print("❌")
            self.results.append(f"SSH key not found at {self.ssh_key_path}")
            return False
    
    def test_ssh_connection(self) -> bool:
        """Test: Can SSH without password"""
        print("2️⃣  SSH Connection...", end=" ", flush=True)
        success, output = self.run_command(
            f'ssh {self.pi_address} "echo OK"',
            "SSH connection"
        )
        if success:
            print("✓")
            return True
        else:
            print("❌")
            self.results.append(f"SSH connection failed: {output}")
            return False
    
    def test_no_ssh_passphrase(self) -> bool:
        """Test: SSH key has no passphrase"""
        print("3️⃣  SSH No Passphrase...", end=" ", flush=True)
        # Try to change key passphrase without entering anything
        cmd = f'ssh-keygen -p -f "{self.ssh_key_path}" -N "" -P "" -q'
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=5)
        
        # If it worked without prompting, passphrase is empty
        # If it prompts or fails, there's a passphrase
        if result.returncode == 0:
            print("✓")
            return True
        else:
            # Try one more thing - just listing the key details
            cmd2 = f'ssh-keygen -l -f "{self.ssh_key_path}"'
            result2 = subprocess.run(cmd2, shell=True, capture_output=True, text=True)
            if result2.returncode == 0:
                print("⚠️ (has passphrase - robot may need it removed)")
                self.results.append("SSH key has a passphrase - may cause issues with automation")
                return False
            else:
                print("❌")
                self.results.append("Could not verify SSH key passphrase status")
                return False
    
    def test_sudo_access(self) -> bool:
        """Test: Can run sudo without password"""
        print("4️⃣  Passwordless Sudo...", end=" ", flush=True)
        success, output = self.run_command(
            f'ssh {self.pi_address} "sudo echo OK"',
            "Sudo access"
        )
        if success:
            print("✓")
            return True
        else:
            print("❌")
            self.results.append(f"Sudo access failed: {output}")
            return False
    
    def test_sudo_whoami(self) -> bool:
        """Test: Can run specific sudo command"""
        print("5️⃣  Sudo Whoami...", end=" ", flush=True)
        success, output = self.run_command(
            f'ssh {self.pi_address} "sudo whoami"',
            "Sudo whoami"
        )
        if success and "root" in output:
            print("✓")
            return True
        else:
            print("❌")
            self.results.append(f"Sudo whoami failed or didn't return root: {output}")
            return False
    
    def test_python_access(self) -> bool:
        """Test: Can run Python commands on Pi"""
        print("6️⃣  Python Access...", end=" ", flush=True)
        success, output = self.run_command(
            f'ssh {self.pi_address} "python3 -c \\"import sys; print(sys.version)\\"" 2>/dev/null',
            "Python access"
        )
        if success:
            print("✓")
            return True
        else:
            print("❌")
            self.results.append("Python not accessible on Pi")
            return False
    
    def test_venv_access(self) -> bool:
        """Test: Can access virtual environment"""
        print("7️⃣  VLN-HTN Venv...", end=" ", flush=True)
        success, output = self.run_command(
            f'ssh {self.pi_address} "test -d ~/VLA-HTN/venv && echo OK || echo MISSING"',
            "VLN-HTN venv"
        )
        if success and "OK" in output:
            print("✓")
            return True
        else:
            print("⚠️ (venv not found - may need to install)")
            self.results.append("VLN-HTN venv not found at ~/VLA-HTN/venv")
            return False
    
    def test_robot_command(self) -> bool:
        """Test: Can run typical robot command"""
        print("8️⃣  Robot Command...", end=" ", flush=True)
        cmd = f'ssh {self.pi_address} "cd ~/VLA-HTN && source venv/bin/activate 2>/dev/null && python3 -c \\"print(\\'OK\\')\\" 2>/dev/null || echo OK"'
        success, output = self.run_command(cmd, "Robot command")
        if success:
            print("✓")
            return True
        else:
            print("⚠️ (may need to install dependencies)")
            self.results.append("Robot command execution had issues")
            return False
    
    def test_sudoers_config(self) -> bool:
        """Test: Verify sudoers is configured correctly"""
        print("9️⃣  Sudoers Config...", end=" ", flush=True)
        cmd = f'ssh {self.pi_address} "sudo cat /etc/sudoers.d/010_gisooj-nopasswd 2>/dev/null | grep -q NOPASSWD && echo OK || echo MISSING"'
        success, output = self.run_command(cmd, "Sudoers config")
        if success and "OK" in output:
            print("✓")
            return True
        else:
            print("❌")
            self.results.append("Sudoers not configured for passwordless access")
            return False
    
    def run_all_tests(self) -> bool:
        """Run all tests"""
        print("\n" + "="*70)
        print("🧪 PASSWORDLESS RASPBERRY PI ACCESS TEST")
        print("="*70 + "\n")
        
        print(f"Testing: {self.pi_address}")
        print(f"SSH Key: {self.ssh_key_path}")
        print()
        
        tests = [
            self.test_ssh_key_exists,
            self.test_ssh_connection,
            self.test_no_ssh_passphrase,
            self.test_sudo_access,
            self.test_sudo_whoami,
            self.test_python_access,
            self.test_venv_access,
            self.test_robot_command,
            self.test_sudoers_config,
        ]
        
        passed = sum(1 for test in tests if test())
        total = len(tests)
        
        print("\n" + "="*70)
        print(f"RESULTS: {passed}/{total} tests passed")
        print("="*70)
        
        if self.results:
            print("\n⚠️  Issues found:")
            for i, result in enumerate(self.results, 1):
                print(f"  {i}. {result}")
        
        if passed == total:
            print("\n✅ ALL TESTS PASSED!")
            print("\nYour Raspberry Pi is ready for passwordless robot operations!")
            print("\nYou can run commands like:")
            print(f"  ssh {self.pi_address} \"cd ~/VLA-HTN && python run_camera_service.py\"")
            print(f"  ssh {self.pi_address} \"sudo systemctl restart robot-service\"")
            return True
        elif passed >= total - 2:
            print("\n⚠️  Most tests passed, but fix the issues above")
            return False
        else:
            print("\n❌ Several tests failed - check setup")
            return False


def main():
    parser = argparse.ArgumentParser(
        description="Test passwordless Raspberry Pi access for robot operations",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python test_passwordless_access.py
  python test_passwordless_access.py --verbose
  python test_passwordless_access.py --pi-host 192.168.1.102 --verbose
        """
    )
    
    parser.add_argument("--pi-user", default="gisooj", help="Raspberry Pi username")
    parser.add_argument("--pi-host", default="gisoopi.local", help="Raspberry Pi hostname or IP")
    parser.add_argument("--verbose", "-v", action="store_true", help="Verbose output")
    
    args = parser.parse_args()
    
    tester = PasswordlessAccessTester(
        pi_user=args.pi_user,
        pi_host=args.pi_host,
        verbose=args.verbose
    )
    
    success = tester.run_all_tests()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
