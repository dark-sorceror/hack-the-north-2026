📋 PASSWORDLESS RASPBERRY PI SETUP FOR ROBOT
═════════════════════════════════════════════════════════════════════════════

Your robot needs to connect to the Raspberry Pi without waiting for password
prompts. This guide sets up complete passwordless access.

═════════════════════════════════════════════════════════════════════════════
🚀 QUICK START (5 MINUTES)
═════════════════════════════════════════════════════════════════════════════

1. First time setup on Windows PowerShell (required once):

   # Find your SSH key passphrase (from when you created the key)
   # Then run:
   
   python passwordless_pi_setup.py --passphrase "YOUR_PASSPHRASE"

2. Verify everything works:

   python test_passwordless_access.py


That's it! ✓


═════════════════════════════════════════════════════════════════════════════
📚 THREE WAYS TO SETUP
═════════════════════════════════════════════════════════════════════════════

METHOD 1: Python Script (Cross-platform, Recommended)
──────────────────────────────────────────────────────

From Windows PowerShell:

    python passwordless_pi_setup.py --passphrase "current_passphrase"
    
Options:
    --passphrase TEXT     Current SSH key passphrase to remove
    --pi-user TEXT        Raspberry Pi username (default: gisooj)
    --pi-host TEXT        Raspberry Pi hostname (default: gisoopi.local)
    --auto-login         Enable auto-login at boot
    --test-only          Test without making changes


METHOD 2: PowerShell Script (Windows-native)
─────────────────────────────────────────────

    .\passwordless_pi_setup.ps1 -Passphrase "your_passphrase"
    
Options:
    -Passphrase TEXT      Current SSH key passphrase to remove
    -PiUser TEXT         Raspberry Pi username (default: gisooj)
    -PiHost TEXT         Raspberry Pi hostname (default: gisoopi.local)
    -AutoLogin           Enable auto-login at boot
    -TestOnly            Test without making changes


METHOD 3: Manual Commands (Full control)
─────────────────────────────────────────

Step 1 - Remove SSH key passphrase (Windows PowerShell):
    
    ssh-keygen -p -f $env:USERPROFILE\.ssh\id_ed25519 -N "" -P "current_passphrase"

Step 2 - Configure passwordless sudo (run via SSH):
    
    ssh gisooj@gisoopi.local 'echo "gisooj ALL=(ALL) NOPASSWD:ALL" | sudo tee /etc/sudoers.d/010_gisooj-nopasswd > /dev/null'

Step 3 - Test both work:
    
    ssh gisooj@gisoopi.local "echo OK"
    ssh gisooj@gisoopi.local "sudo echo OK"


═════════════════════════════════════════════════════════════════════════════
✅ WHAT EACH SETUP DOES
═════════════════════════════════════════════════════════════════════════════

1. REMOVES SSH KEY PASSPHRASE
   ├─ Current: ssh gisooj@gisoopi.local → Asks for key passphrase
   └─ After:   ssh gisooj@gisoopi.local → No prompt! ✓

2. CONFIGURES PASSWORDLESS SUDO
   ├─ Current: ssh gisooj@gisoopi.local "sudo echo test" → Asks for password
   └─ After:   ssh gisooj@gisoopi.local "sudo echo test" → No prompt! ✓

3. VERIFIES EVERYTHING WORKS
   ├─ Tests SSH connection
   ├─ Tests sudo access
   └─ Tests robot command execution

4. OPTIONAL: ENABLES AUTO-LOGIN AT BOOT
   ├─ Pi boots directly to login (--auto-login flag)
   └─ Robot can start automatically after reboot


═════════════════════════════════════════════════════════════════════════════
🧪 TESTING THE SETUP
═════════════════════════════════════════════════════════════════════════════

Run this to verify everything works:

    python test_passwordless_access.py

This tests:
    ✓ SSH key exists and is readable
    ✓ Can SSH without password
    ✓ SSH key has no passphrase
    ✓ Can run sudo without password
    ✓ Sudo returns root access
    ✓ Python is available on Pi
    ✓ VLA-HTN venv exists
    ✓ Can run robot commands
    ✓ Sudoers is configured correctly


═════════════════════════════════════════════════════════════════════════════
🤖 USING PASSWORDLESS ACCESS IN ROBOT CODE
═════════════════════════════════════════════════════════════════════════════

Python:
├─ import subprocess
├─ result = subprocess.run(
│      ["ssh", "gisooj@gisoopi.local", "python ~/VLA-HTN/run_camera_service.py"],
│      capture_output=True,
│      text=True
│  )
└─ print(result.stdout)

PowerShell:
├─ ssh gisooj@gisoopi.local "cd ~/VLA-HTN && python run_camera_service.py"
└─ Or with error handling:
   try { $output = ssh ... } catch { Write-Error "Failed: $_" }

Bash:
├─ ssh gisooj@gisoopi.local "cd ~/VLA-HTN && source venv/bin/activate && python test.py"
└─ Or with sudo:
   ssh gisooj@gisoopi.local "sudo systemctl restart robot-service"


═════════════════════════════════════════════════════════════════════════════
❌ TROUBLESHOOTING
═════════════════════════════════════════════════════════════════════════════

Problem: "SSH still asks for passphrase"
─────────────────────────────────────────

Cause: SSH key still has a passphrase
Solution:
    ssh-keygen -p -f $env:USERPROFILE\.ssh\id_ed25519 -N "" -P "current_passphrase"

Or if you forgot the passphrase:
    # Generate new key (old one won't work)
    Remove-Item $env:USERPROFILE\.ssh\id_ed25519*
    ssh-keygen -t ed25519 -f $env:USERPROFILE\.ssh\id_ed25519 -N ""
    # Then copy to Pi again


Problem: "sudo: password required"
──────────────────────────────────

Cause: Sudoers not configured for passwordless
Solution:
    ssh gisooj@gisoopi.local 'echo "gisooj ALL=(ALL) NOPASSWD:ALL" | sudo tee /etc/sudoers.d/010_gisooj-nopasswd > /dev/null'

Or verify it's set:
    ssh gisooj@gisoopi.local "sudo cat /etc/sudoers.d/010_gisooj-nopasswd"
    # Should show: gisooj ALL=(ALL) NOPASSWD:ALL


Problem: "Cannot connect to gisoopi.local"
──────────────────────────────────────────

Cause: Network issue or wrong hostname
Solution:
    # Check network
    ping gisoopi.local
    
    # Try IP address instead
    ssh gisooj@192.168.1.102 "echo OK"
    
    # Or check if SSH is running
    ssh gisooj@192.168.1.102 "sudo systemctl status ssh"


Problem: "Connection refused"
──────────────────────────────

Cause: Pi not accessible or SSH not running
Solution:
    # Make sure Pi is on and connected to network
    ping gisoopi.local
    
    # On Pi, check SSH status
    ssh gisooj@192.168.1.102 "sudo systemctl status ssh"
    
    # On Pi, restart SSH
    ssh gisooj@192.168.1.102 "sudo systemctl restart ssh"


═════════════════════════════════════════════════════════════════════════════
🔑 HOW THIS WORKS (TECHNICAL)
═════════════════════════════════════════════════════════════════════════════

SSH Passwordless Authentication:
├─ Windows (client): SSH private key (~/.ssh/id_ed25519)
├─ Raspberry Pi (server): SSH public key (in ~/.ssh/authorized_keys)
├─ Connection: Private key proves identity without password
└─ Result: ssh gisooj@gisoopi.local connects without password ✓

Key Passphrase:
├─ Passphrase: Extra password protecting the private key
├─ Without passphrase: Key can be used immediately
├─ With passphrase: Key prompts for password on first use
├─ For robot: Must be removed so no interactive prompts occur
└─ Result: SSH works without any user interaction ✓

Passwordless Sudo:
├─ Sudoers file: /etc/sudoers.d/010_gisooj-nopasswd
├─ Entry: "gisooj ALL=(ALL) NOPASSWD:ALL"
├─ Meaning: User 'gisooj' can run any command as root without password
└─ Result: sudo commands work without password prompt ✓

End-to-End:
├─ Windows (robot) → SSH (no password)
├─ Pi (server) ← Accepts connection with public key
├─ Robot runs command with sudo ← No password prompt
└─ Result: Fully automated, passwordless operation ✓


═════════════════════════════════════════════════════════════════════════════
🛡️ SECURITY CONSIDERATIONS
═════════════════════════════════════════════════════════════════════════════

⚠️  RISKS OF REMOVING KEY PASSPHRASE:
    • Anyone with your private key can SSH to your Pi
    • Keep private key file secure (don't share, don't commit to git)
    • Use firewall to restrict who can SSH to Pi
    
✓  MITIGATIONS:
    • Keep Pi on secure network (not exposed to internet)
    • Use firewall rules to restrict SSH access
    • Monitor SSH logs: ssh gisooj@gisoopi.local "sudo tail -f /var/log/auth.log"
    • Consider using SSH key with IP/command restrictions
    • Rotate keys periodically

⚠️  RISKS OF PASSWORDLESS SUDO:
    • Any local user could potentially run root commands
    • Use firewall and network security
    • Run robot service as dedicated user (if possible)
    
✓  FOR ROBOT-ONLY SYSTEMS:
    • This setup is fine for autonomous robots
    • Robot runs as dedicated user
    • Network is isolated/secure
    • Consider physical security (Pi location)


═════════════════════════════════════════════════════════════════════════════
📖 DETAILED GUIDES
═════════════════════════════════════════════════════════════════════════════

PASSWORDLESS_PI_SETUP.md
├─ Complete step-by-step guide
├─ Explains what each command does
├─ Includes security considerations
└─ Troubleshooting for each step

QUICK_START_PASSWORDLESS.md
├─ Quick reference guide
├─ Copy-paste commands
├─ 3-minute setup
└─ Common issues


═════════════════════════════════════════════════════════════════════════════
✨ NEXT STEPS
═════════════════════════════════════════════════════════════════════════════

1. Run setup:
   python passwordless_pi_setup.py --passphrase "YOUR_PASSPHRASE"

2. Test it works:
   python test_passwordless_access.py

3. Update robot code to use SSH commands:
   subprocess.run(["ssh", "gisooj@gisoopi.local", "command"])

4. Deploy robot tasks that run without password prompts

5. Monitor and maintain:
   - Check SSH logs regularly
   - Keep key secure
   - Update Pi OS periodically


═════════════════════════════════════════════════════════════════════════════

Questions? Check the troubleshooting section or run:
    python passwordless_pi_setup.py --help
    python test_passwordless_access.py --help
    python test_passwordless_access.py --verbose

