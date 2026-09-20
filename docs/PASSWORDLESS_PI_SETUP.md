# 🔓 Raspberry Pi Passwordless Setup for Robot Operations

## Overview
This guide makes your Raspberry Pi completely passwordless for robot operations, so it can:
- SSH without password prompts
- Run sudo commands without password
- Auto-login at boot (optional)
- Execute robot tasks unattended

## ✅ Prerequisites
- SSH key already configured (✓ id_ed25519 exists)
- Access to the Pi via SSH
- Pi username: `gisooj`
- Pi hostname: `gisoopi.local`

---

## STEP 1: Remove SSH Key Passphrase (Windows - PowerShell)

Your SSH key currently has a passphrase. Remove it so robot operations don't get stuck on prompts:

```powershell
# Open PowerShell (NOT as admin)
ssh-keygen -p -f $env:USERPROFILE\.ssh\id_ed25519 -N "" -P "YOUR_CURRENT_PASSPHRASE"
```

**If you forgot your passphrase:**
```powershell
# Create new key with no passphrase
Remove-Item $env:USERPROFILE\.ssh\id_ed25519*
ssh-keygen -t ed25519 -f $env:USERPROFILE\.ssh\id_ed25519 -N ""
# Then copy new public key to Pi
```

**Verify passphrase removed:**
```powershell
ssh gisooj@gisoopi.local "echo 'SSH works without passphrase prompt!'"
# Should NOT ask for any passphrase or password
```

---

## STEP 2: Configure Passwordless Sudo on Raspberry Pi

SSH into your Pi and run these commands:

```bash
# SSH into your Pi (now without passphrase!)
ssh gisooj@gisoopi.local

# Once on the Pi, run:
echo "gisooj ALL=(ALL) NOPASSWD:ALL" | sudo tee /etc/sudoers.d/010_gisooj-nopasswd

# Verify it works
sudo echo "Sudo works without password!"

# Exit Pi
exit
```

**Manual Alternative (if echo doesn't work):**
```bash
ssh gisooj@gisoopi.local

# Open sudoers editor
sudo visudo -f /etc/sudoers.d/010_gisooj-nopasswd

# Add this line:
gisooj ALL=(ALL) NOPASSWD:ALL

# Save (Ctrl+X if using nano, then Y, then Enter)
exit
```

---

## STEP 3: Optional - Auto-Login at Boot (Full Unattended Mode)

Run this on the Raspberry Pi to enable auto-login:

```bash
ssh gisooj@gisoopi.local << 'EOF'

# Enable auto-login for Raspberry Pi
sudo raspi-config nonint do_boot_behaviour B2  # Auto-login to desktop

# Or to just auto-login to CLI (recommended for robot):
sudo raspi-config nonint do_boot_behaviour B1  # Auto-login to CLI

# Verify
echo "Auto-login configured!"

EOF
```

---

## STEP 4: Test Full Passwordless Setup (Windows PowerShell)

```powershell
# Test 1: SSH without password
ssh gisooj@gisoopi.local "echo 'SSH test passed!'"

# Test 2: SSH + sudo without passwords
ssh gisooj@gisoopi.local "sudo echo 'Sudo test passed!'"

# Test 3: Run robot commands without prompts
ssh gisooj@gisoopi.local "cd ~/VLA-HTN && source venv/bin/activate && python test_microphone_vla_detection.py"

# Test 4: Multiple commands
ssh gisooj@gisoopi.local << 'EOF'
echo "Test started..."
whoami
sudo whoami
pwd
echo "All tests passed!"
EOF
```

---

## STEP 5: Robot Operations Script

Run this from Windows to execute robot tasks on Pi without passwords:

```powershell
# PowerShell script to run robot task
$pi_host = "gisooj@gisoopi.local"

# Example: Start camera service
ssh $pi_host "cd ~/VLA-HTN && source venv/bin/activate && python run_camera_service.py"

# Example: Run voice detection
ssh $pi_host "cd ~/VLA-HTN && source venv/bin/activate && python test_microphone_vla_detection.py"

# Example: Run with sudo if needed
ssh $pi_host "cd ~/VLA-HTN && source venv/bin/activate && sudo python run_camera_service.py"
```

---

## ✅ Verification Checklist

- [ ] SSH key passphrase removed (tested with `ssh gisooj@gisoopi.local "echo OK"`)
- [ ] Passwordless sudo configured (tested with `sudo echo OK`)
- [ ] No password prompts when running SSH commands
- [ ] Robot can SSH and execute commands unattended
- [ ] Optional: Auto-login at boot is enabled (if desired)

---

## 🚨 Security Notes

⚠️ **Be Aware:**
- Removing SSH key passphrase = anyone with your key can SSH to Pi
- Passwordless sudo = any local account can run root commands
- Auto-login at boot = anyone with physical access can use the Pi

**For robot-only setup (recommended):**
- Keep key passphrase removed only for robot operations
- Limit sudo access to specific commands if needed
- Don't enable auto-login unless Pi is in secure location

---

## Troubleshooting

**Q: "ssh: Permission denied (publickey)"**
```powershell
# Verify key file exists
Get-Item $env:USERPROFILE\.ssh\id_ed25519
Get-Item $env:USERPROFILE\.ssh\id_ed25519.pub

# Try explicit key
ssh -i $env:USERPROFILE\.ssh\id_ed25519 gisooj@gisoopi.local
```

**Q: "sudo: password required"**
```bash
# On Pi, check sudoers file
sudo cat /etc/sudoers.d/010_gisooj-nopasswd

# Should show: gisooj ALL=(ALL) NOPASSWD:ALL
```

**Q: "SSH still asks for passphrase"**
```powershell
# Check if key has passphrase
ssh-keygen -l -f $env:USERPROFILE\.ssh\id_ed25519

# Remove passphrase (see STEP 1)
ssh-keygen -p -f $env:USERPROFILE\.ssh\id_ed25519 -N "" -P "YOUR_PASSPHRASE"
```

**Q: "Cannot connect to gisoopi.local"**
```powershell
# Try IP address instead
ssh gisooj@192.168.1.102

# Or check network
ping gisoopi.local
```

---

## Quick Command Reference

```bash
# From Windows PowerShell:

# Remove SSH key passphrase
ssh-keygen -p -f $env:USERPROFILE\.ssh\id_ed25519 -N "" -P "current_passphrase"

# Test SSH
ssh gisooj@gisoopi.local "echo OK"

# Configure passwordless sudo
ssh gisooj@gisoopi.local 'echo "gisooj ALL=(ALL) NOPASSWD:ALL" | sudo tee /etc/sudoers.d/010_gisooj-nopasswd'

# Test sudo
ssh gisooj@gisoopi.local "sudo echo OK"

# Enable auto-login
ssh gisooj@gisoopi.local "sudo raspi-config nonint do_boot_behaviour B1"
```

---

## Next Steps

1. **Run STEP 1** - Remove SSH key passphrase
2. **Run STEP 2** - Configure passwordless sudo on Pi
3. **Run STEP 4** - Test everything works
4. **Optional STEP 3** - Enable auto-login if robot runs unattended
5. Update robot scripts to use passwordless SSH (see STEP 5)
