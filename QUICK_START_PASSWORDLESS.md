# ⚡ QUICK START: Passwordless Raspberry Pi for Robot

## 🚀 The 3-Minute Setup

### Option A: PowerShell (Recommended for Windows)

```powershell
# 1. Open PowerShell (NOT as admin)
cd C:\Users\jgiso\OneDrive\Desktop\Documentos\VLA-HTN

# 2. First, TEST that SSH works
ssh gisooj@gisoopi.local "echo OK"

# 3. If SSH key has a passphrase, REMOVE IT:
ssh-keygen -p -f $env:USERPROFILE\.ssh\id_ed25519 -N "" -P "YOUR_CURRENT_PASSPHRASE"

# 4. Configure passwordless sudo on Pi:
ssh gisooj@gisoopi.local 'echo "gisooj ALL=(ALL) NOPASSWD:ALL" | sudo tee /etc/sudoers.d/010_gisooj-nopasswd > /dev/null'

# 5. TEST both work without prompts:
ssh gisooj@gisoopi.local "echo 'SSH OK'"
ssh gisooj@gisoopi.local "sudo echo 'Sudo OK'"

# DONE! ✓
```

### Option B: Python Script (Cross-Platform)

```powershell
# Run the automated setup
python passwordless_pi_setup.py --passphrase "YOUR_CURRENT_PASSPHRASE"

# Or with auto-login enabled:
python passwordless_pi_setup.py --passphrase "YOUR_CURRENT_PASSPHRASE" --auto-login

# Just test without changing anything:
python passwordless_pi_setup.py --test-only
```

### Option C: PowerShell Script

```powershell
# Run the PowerShell setup script
.\passwordless_pi_setup.ps1 -Passphrase "YOUR_CURRENT_PASSPHRASE"

# Or with auto-login:
.\passwordless_pi_setup.ps1 -Passphrase "YOUR_CURRENT_PASSPHRASE" -AutoLogin

# Just test:
.\passwordless_pi_setup.ps1 -TestOnly
```

---

## ✅ Verify It Works

After setup, run these from PowerShell - should NOT ask for any passwords:

```powershell
# Test 1: SSH without password
ssh gisooj@gisoopi.local "echo 'SSH test passed'"

# Test 2: SSH + Sudo without passwords
ssh gisooj@gisoopi.local "sudo whoami"

# Test 3: Run robot task
ssh gisooj@gisoopi.local "cd ~/VLA-HTN && source venv/bin/activate && python run_camera_service.py"
```

---

## 🤖 Using in Robot Scripts

### Python
```python
import subprocess

def run_on_pi(command):
    """Run command on Pi without password prompts"""
    return subprocess.run(
        ["ssh", "gisooj@gisoopi.local", command],
        capture_output=True,
        text=True
    )

# Example
result = run_on_pi("cd ~/VLA-HTN && python test_microphone_vla_detection.py")
print(result.stdout)
```

### PowerShell
```powershell
# Simple SSH call
ssh gisooj@gisoopi.local "python ~/VLA-HTN/run_camera_service.py"

# With error handling
try {
    $output = ssh gisooj@gisoopi.local "sudo systemctl status robot-camera-service"
    Write-Host $output
} catch {
    Write-Error "Failed to run command: $_"
}
```

---

## 🔍 Troubleshooting

### "SSH still asks for passphrase"
```powershell
# Remove passphrase from key
ssh-keygen -p -f $env:USERPROFILE\.ssh\id_ed25519 -N "" -P "your_passphrase"
```

### "sudo: password required"
```powershell
# Verify sudoers is set correctly
ssh gisooj@gisoopi.local "sudo cat /etc/sudoers.d/010_gisooj-nopasswd"

# Should show: gisooj ALL=(ALL) NOPASSWD:ALL
```

### "Cannot connect to gisoopi.local"
```powershell
# Try with IP address
ssh gisooj@192.168.1.102 "echo OK"

# Or check network
ping gisoopi.local
```

### "Connection refused"
```powershell
# Make sure Pi is on and SSH is enabled
ssh gisooj@gisoopi.local "echo OK"

# If that fails, verify from Pi:
ssh gisooj@gisoopi.local "sudo systemctl status ssh"
```

---

## 📋 What Was Set Up

✓ **SSH Key Authentication** - No password needed, key-based access  
✓ **Passphrase Removed** - SSH key works without prompts  
✓ **Passwordless Sudo** - Robot can run root commands  
✓ **Verified End-to-End** - Both SSH and sudo tested  
✓ **Optional Auto-Login** - Pi boots directly to login (if enabled)  

---

## 🛡️ Security Reminders

⚠️ **Removing key passphrase** = Anyone with your key can SSH to Pi  
⚠️ **Passwordless sudo** = Any local user can run root commands  
⚠️ **Auto-login** = Anyone with physical access can use Pi  

For production robots, consider:
- Keep Pi in secure location (physical security)
- Use firewall to limit SSH access
- Run robot services as dedicated user (not full sudo)
- Monitor SSH logs: `ssh gisooj@gisoopi.local "sudo tail -f /var/log/auth.log"`

---

## 📖 Full Documentation

See `PASSWORDLESS_PI_SETUP.md` for detailed step-by-step instructions
