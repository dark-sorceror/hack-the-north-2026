📚 SSH PASSWORDLESS ACCESS - SETUP COMPLETE
═══════════════════════════════════════════════════════════════

✅ CURRENT STATUS
─────────────────────────────────────────────────────────────
✓ SSH key generated: ~/.ssh/id_ed25519
✓ Public key installed on Pi
✓ Key-based authentication working

Your key fingerprint:
  SHA256:wwN8eMOSQyqFiIQEAIkvlUcUDE48glPz3kklFTVsiMU vla-robot


🔑 QUICK START - 3 Ways to Connect
═══════════════════════════════════════════════════════════════

METHOD 1: Simple SSH (One-liner, no password)
──────────────────────────────────────────────
ssh gisooj@gisoopi.local

This now uses your SSH key automatically. You'll be prompted once for the
key's passphrase, which is normal and secure.


METHOD 2: Explicit Key Usage
──────────────────────────────────────────────
ssh -i $HOME\.ssh\id_ed25519 gisooj@gisoopi.local

Specify the key file explicitly (same result as Method 1).


METHOD 3: Via SCP (File Transfer)
──────────────────────────────────────────────
scp test_microphone_vla_detection.py gisooj@gisoopi.local:~/VLA-HTN/
scp pyproject.toml gisooj@gisoopi.local:~/VLA-HTN/


🚀 USE WITH YOUR SCRIPTS
═══════════════════════════════════════════════════════════════

Test the microphone → VLA → detection pipeline:
  ssh gisooj@gisoopi.local "cd ~/VLA-HTN && source venv/bin/activate && python test_microphone_vla_detection.py"

No password needed anymore!


OPTIONAL: Remove Key Passphrase (One-time)
═════════════════════════════════════════════════════════════════

If you want to avoid even the key passphrase prompt, you can remove the
passphrase from your local key. WARNING: This makes your key less secure
if the file is compromised.

ssh-keygen -p -f $HOME\.ssh\id_ed25519 -N "" -P "YOUR_CURRENT_PASSPHRASE"

Then test:
  ssh gisooj@gisoopi.local "echo 'No passphrase needed!'"


📋 SSH CONFIG FILE LOCATION
─────────────────────────────────────────────────────────────
~/.ssh/config

If you want to further customize SSH behavior, edit this file.


✅ TESTING PASSWORDLESS ACCESS
═════════════════════════════════════════════════════════════════

Copy and paste these tests:

# Test 1: Simple command
ssh gisooj@gisoopi.local "whoami"

# Test 2: File transfer
scp -r ~/Desktop/test_file.txt gisooj@gisoopi.local:~/

# Test 3: Run Python on Pi
ssh gisooj@gisoopi.local "cd ~/VLA-HTN && python --version"

# Test 4: Full microphone → VLA → detection pipeline
ssh gisooj@gisoopi.local "cd ~/VLA-HTN && source venv/bin/activate && python test_microphone_vla_detection.py"


🔐 SECURITY INFO
═════════════════════════════════════════════════════════════════

What you have:
  - Ed25519 SSH key (256-bit, secure modern standard)
  - Public key on Pi's authorized_keys
  - Key passphrase protects your private key locally

When you SSH:
  1. SSH agent checks for your key
  2. Prompts for key passphrase (first time in session)
  3. Caches passphrase in agent memory
  4. Subsequent commands use cached passphrase
  5. No Pi password needed


⚙️ TROUBLESHOOTING
═════════════════════════════════════════════════════════════════

❌ "Permission denied (publickey,password)"
  → Verify key is in authorized_keys:
    ssh gisooj@gisoopi.local "cat ~/.ssh/authorized_keys"

❌ "Could not open a connection to your authentication agent"
  → SSH agent not running (Windows specific):
    Start-Service ssh-agent
    ssh-add $HOME\.ssh\id_ed25519

❌ "Passphrase prompt every time"
  → This is normal! Passphrase is for your local key security
  → To cache it: ssh-add $HOME\.ssh\id_ed25519

❌ "Host key verification failed"
  → Add -o StrictHostKeyChecking=accept-new to accept the Pi's host key


🎯 NEXT STEPS
═════════════════════════════════════════════════════════════════

1. Test passwordless SSH:
   ssh gisooj@gisoopi.local "echo 'Success!'"

2. Run your microphone detection test:
   ssh gisooj@gisoopi.local "cd ~/VLA-HTN && source venv/bin/activate && python test_microphone_vla_detection.py"

3. Update your deployment scripts to use:
   ssh gisooj@gisoopi.local "your-command-here"
   (no password prompts!)

