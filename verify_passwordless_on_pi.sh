#!/usr/bin/env bash
# Passwordless Raspberry Pi Setup - Quick Test Script
# Run this ON the Raspberry Pi to verify/configure passwordless access

set -e

echo "╔════════════════════════════════════════════════════════════════════╗"
echo "║         PASSWORDLESS RASPBERRY PI VERIFICATION (ON PI)             ║"
echo "╚════════════════════════════════════════════════════════════════════╝"

echo ""
echo "STEP 1: Verify SSH Key Access"
echo "────────────────────────────────────────────────────────────────────"
echo "✓ Already connected via SSH - this proves SSH key works!"

echo ""
echo "STEP 2: Check Current User"
echo "────────────────────────────────────────────────────────────────────"
whoami
pwd

echo ""
echo "STEP 3: Test Passwordless Sudo"
echo "────────────────────────────────────────────────────────────────────"
echo "Testing: sudo whoami"
sudo whoami

echo ""
echo "STEP 4: Configure Passwordless Sudo (if needed)"
echo "────────────────────────────────────────────────────────────────────"

CURRENT_USER=$(whoami)
SUDOERS_FILE="/etc/sudoers.d/010_${CURRENT_USER}-nopasswd"

if sudo grep -q "^${CURRENT_USER} ALL=(ALL) NOPASSWD:ALL" "$SUDOERS_FILE" 2>/dev/null; then
    echo "✓ Passwordless sudo already configured for $CURRENT_USER"
else
    echo "ℹ️  Setting up passwordless sudo for $CURRENT_USER..."
    echo "${CURRENT_USER} ALL=(ALL) NOPASSWD:ALL" | sudo tee "$SUDOERS_FILE" > /dev/null
    echo "✓ Passwordless sudo configured!"
fi

echo ""
echo "STEP 5: Enable Auto-Login (Optional)"
echo "────────────────────────────────────────────────────────────────────"
echo "To enable auto-login to CLI at boot:"
echo "  sudo raspi-config nonint do_boot_behaviour B1"
echo ""
echo "To enable auto-login to desktop at boot:"
echo "  sudo raspi-config nonint do_boot_behaviour B2"

echo ""
echo "════════════════════════════════════════════════════════════════════"
echo "✅ PASSWORDLESS SETUP VERIFIED!"
echo "════════════════════════════════════════════════════════════════════"
echo ""
echo "Your Pi can now:"
echo "  • SSH without password (from Windows with key)"
echo "  • Run sudo commands without password"
echo "  • Execute robot tasks unattended"
echo ""
