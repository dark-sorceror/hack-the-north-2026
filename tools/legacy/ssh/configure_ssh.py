#!/usr/bin/env python3
"""
Automated Passwordless Raspberry Pi Setup for Robot Operations

Configures:
1. SSH key without passphrase
2. Passwordless sudo on Pi
3. Optional auto-login at boot

Usage:
    python tools/legacy/ssh/configure_ssh.py [OPTIONS]

Options:
    --passphrase TEXT       Current SSH key passphrase to remove
    --pi-user TEXT         Raspberry Pi username (default: gisooj)
    --pi-host TEXT         Raspberry Pi hostname (default: gisoopi.local)
    --auto-login          Enable auto-login at boot
    --test-only           Only test, don't configure
"""

import os
import sys
import subprocess
import argparse
from pathlib import Path


class PasswordlessPiSetup:
    def __init__(self, pi_user="gisooj", pi_host="gisoopi.local"):
        self.pi_user = pi_user
        self.pi_host = pi_host
        self.pi_address = f"{pi_user}@{pi_host}"
        self.ssh_key_path = Path.home() / ".ssh" / "id_ed25519"
        
    def run_local(self, cmd, description=None):
        """Run command locally (Windows PowerShell)"""
        if description:
            print(f"\n📍 {description}")
        print(f"   → {cmd}")
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        if result.returncode != 0:
            print(f"   ❌ Error: {result.stderr}")
            return False
        print(f"   ✓ {result.stdout.strip()}")
        return True
    
    def run_ssh(self, cmd, description=None):
        """Run command on Pi via SSH"""
        if description:
            print(f"\n🔧 {description}")
        full_cmd = f'ssh {self.pi_address} "{cmd}"'
        print(f"   → ssh ... {cmd[:60]}{'...' if len(cmd) > 60 else ''}")
        result = subprocess.run(full_cmd, shell=True, capture_output=True, text=True)
        if result.returncode != 0:
            print(f"   ❌ Error: {result.stderr}")
            return False
        if result.stdout:
            print(f"   ✓ {result.stdout.strip()}")
        else:
            print(f"   ✓ Command executed")
        return True
    
    def step1_check_ssh_key(self):
        """Check if SSH key exists"""
        print("\n" + "="*70)
        print("STEP 1: Verify SSH Key Exists")
        print("="*70)
        
        if not self.ssh_key_path.exists():
            print(f"❌ SSH key not found: {self.ssh_key_path}")
            print("\nGenerate new key:")
            print(f'  ssh-keygen -t ed25519 -f {self.ssh_key_path} -N ""')
            return False
        
        print(f"✓ SSH key found: {self.ssh_key_path}")
        
        # Check for passphrase
        check_cmd = f'ssh-keygen -l -p -f "{self.ssh_key_path}"'
        result = subprocess.run(check_cmd, shell=True, capture_output=True, text=True, input="n\n")
        
        if "Enter passphrase" in result.stdout or "Enter passphrase" in result.stderr:
            print("⚠️  SSH key has a passphrase (will need to remove for robot)")
            return True
        
        print("✓ SSH key has no passphrase (good for robot!)")
        return True
    
    def step2_remove_passphrase(self, current_passphrase):
        """Remove passphrase from SSH key"""
        print("\n" + "="*70)
        print("STEP 2: Remove SSH Key Passphrase")
        print("="*70)
        
        if not current_passphrase:
            print("❌ No passphrase provided. Use --passphrase to specify current passphrase")
            print("\nCommand:")
            print(f'  python tools/legacy/ssh/configure_ssh.py --passphrase "YOUR_PASSPHRASE"')
            return False
        
        # Remove passphrase using ssh-keygen
        key_path = str(self.ssh_key_path)
        cmd = f'ssh-keygen -p -f "{key_path}" -N "" -P "{current_passphrase}"'
        
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        
        if result.returncode != 0:
            print(f"❌ Failed to remove passphrase: {result.stderr}")
            return False
        
        print("✓ SSH key passphrase removed")
        return True
    
    def step3_test_ssh(self):
        """Test SSH connection without prompts"""
        print("\n" + "="*70)
        print("STEP 3: Test SSH Connection")
        print("="*70)
        
        return self.run_ssh("echo 'SSH connection successful!'", "Testing SSH...")
    
    def step4_configure_sudo(self):
        """Configure passwordless sudo on Pi"""
        print("\n" + "="*70)
        print("STEP 4: Configure Passwordless Sudo on Raspberry Pi")
        print("="*70)
        
        sudoers_entry = f"{self.pi_user} ALL=(ALL) NOPASSWD:ALL"
        sudoers_file = "/etc/sudoers.d/010_gisooj-nopasswd"
        
        # Use echo with sudo tee (more reliable than direct echo)
        cmd = f'echo "{sudoers_entry}" | sudo tee {sudoers_file} > /dev/null'
        
        return self.run_ssh(cmd, "Adding passwordless sudo entry...")
    
    def step5_test_sudo(self):
        """Test passwordless sudo"""
        print("\n" + "="*70)
        print("STEP 5: Test Passwordless Sudo")
        print("="*70)
        
        return self.run_ssh("sudo echo 'Sudo works without password!'", "Testing sudo...")
    
    def step6_enable_autologin(self):
        """Enable auto-login at boot"""
        print("\n" + "="*70)
        print("STEP 6: Enable Auto-Login at Boot")
        print("="*70)
        
        # Enable auto-login to CLI
        return self.run_ssh(
            "sudo raspi-config nonint do_boot_behaviour B1",
            "Enabling auto-login to CLI..."
        )
    
    def test_robot_operations(self):
        """Test full robot operation pipeline"""
        print("\n" + "="*70)
        print("FINAL TEST: Robot Operations")
        print("="*70)
        
        tests = [
            ("whoami", "Current user"),
            ("sudo whoami", "Sudo access"),
            ("pwd", "Working directory"),
            ("echo $HOME", "Home directory"),
        ]
        
        all_passed = True
        for cmd, desc in tests:
            if not self.run_ssh(cmd, desc):
                all_passed = False
        
        return all_passed
    
    def run_full_setup(self, current_passphrase=None, auto_login=False, test_only=False):
        """Run complete passwordless setup"""
        print("\n")
        print("╔" + "="*68 + "╗")
        print("║" + " "*15 + "🔓 PASSWORDLESS RASPBERRY PI SETUP" + " "*18 + "║")
        print("╚" + "="*68 + "╝")
        
        if test_only:
            print("\n⚠️  TEST MODE - No changes will be made")
        
        # Step 1: Verify SSH key exists
        if not self.step1_check_ssh_key():
            return False
        
        # Step 2: Remove passphrase (if provided)
        if current_passphrase:
            if not self.step2_remove_passphrase(current_passphrase):
                return False
        
        # Step 3: Test SSH
        if not self.step3_test_ssh():
            print("\n❌ SSH test failed. Cannot proceed.")
            return False
        
        if test_only:
            print("\n✓ SSH connection works! (test mode only)")
            return True
        
        # Step 4: Configure passwordless sudo
        if not self.step4_configure_sudo():
            return False
        
        # Step 5: Test sudo
        if not self.step5_test_sudo():
            print("\n⚠️  Sudo test failed. Check sudoers configuration.")
            return False
        
        # Step 6: Optional - Enable auto-login
        if auto_login:
            if not self.step6_enable_autologin():
                print("\n⚠️  Auto-login failed (may require manual setup)")
        
        # Final test
        if not self.test_robot_operations():
            print("\n⚠️  Some tests failed")
            return False
        
        return True


def main():
    parser = argparse.ArgumentParser(
        description="Passwordless Raspberry Pi Setup for Robot Operations",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Remove SSH passphrase and configure sudo
  python tools/legacy/ssh/configure_ssh.py --passphrase "my-passphrase"
  
  # Also enable auto-login at boot
  python tools/legacy/ssh/configure_ssh.py --passphrase "my-passphrase" --auto-login
  
  # Test connection without making changes
  python tools/legacy/ssh/configure_ssh.py --test-only
  
  # Custom Pi user/host
  python tools/legacy/ssh/configure_ssh.py --pi-user pi --pi-host 192.168.1.102 --passphrase "pass"
        """
    )
    
    parser.add_argument("--passphrase", help="Current SSH key passphrase to remove")
    parser.add_argument("--pi-user", default="gisooj", help="Raspberry Pi username (default: gisooj)")
    parser.add_argument("--pi-host", default="gisoopi.local", help="Raspberry Pi hostname (default: gisoopi.local)")
    parser.add_argument("--auto-login", action="store_true", help="Enable auto-login at boot")
    parser.add_argument("--test-only", action="store_true", help="Test only, don't configure")
    
    args = parser.parse_args()
    
    setup = PasswordlessPiSetup(pi_user=args.pi_user, pi_host=args.pi_host)
    
    success = setup.run_full_setup(
        current_passphrase=args.passphrase,
        auto_login=args.auto_login,
        test_only=args.test_only
    )
    
    print("\n" + "="*70)
    if success:
        print("✅ PASSWORDLESS SETUP COMPLETE!")
        print("\nYour Raspberry Pi can now run robot operations without password prompts:")
        print(f"  ssh {args.pi_user}@{args.pi_host} 'sudo python ~/VLA-HTN/run_camera_service.py'")
    else:
        print("❌ SETUP FAILED - See errors above")
    print("="*70 + "\n")
    
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
