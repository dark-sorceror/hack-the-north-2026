# Passwordless Raspberry Pi Setup - PowerShell Script
# 🔓 Configures SSH and sudo for robot operations without password prompts

param(
    [string]$PiUser = "gisooj",
    [string]$PiHost = "gisoopi.local",
    [string]$Passphrase = "",
    [switch]$AutoLogin = $false,
    [switch]$TestOnly = $false
)

$ErrorActionPreference = "Continue"

# Color output
function Write-Success { Write-Host "✓ $args" -ForegroundColor Green }
function Write-Error { Write-Host "❌ $args" -ForegroundColor Red }
function Write-Warning { Write-Host "⚠️  $args" -ForegroundColor Yellow }
function Write-Info { Write-Host "ℹ️  $args" -ForegroundColor Cyan }
function Write-Step { Write-Host "`n========================================================================" -ForegroundColor Magenta; Write-Host "$args" -ForegroundColor Magenta; Write-Host "========================================================================" -ForegroundColor Magenta }

# Paths
$SSH_KEY = "$env:USERPROFILE\.ssh\id_ed25519"
$PI_ADDRESS = "${PiUser}@${PiHost}"

Write-Host "`n"
Write-Host "╔══════════════════════════════════════════════════════════════════════╗" -ForegroundColor Cyan
Write-Host "║         🔓 PASSWORDLESS RASPBERRY PI SETUP FOR ROBOT OPERATIONS      ║" -ForegroundColor Cyan
Write-Host "╚══════════════════════════════════════════════════════════════════════╝" -ForegroundColor Cyan

if ($TestOnly) {
    Write-Warning "TEST MODE - No changes will be made"
}

Write-Info "Settings:"
Write-Info "  Pi User: $PiUser"
Write-Info "  Pi Host: $PiHost"
Write-Info "  SSH Key: $SSH_KEY"
Write-Info "  Auto-Login: $(if($AutoLogin) { 'Enabled' } else { 'Disabled' })"

# ============================================================================
# STEP 1: Verify SSH Key Exists
# ============================================================================
Write-Step "STEP 1: Verify SSH Key Exists"

if (-not (Test-Path $SSH_KEY)) {
    Write-Error "SSH key not found: $SSH_KEY"
    Write-Info "Generate new key with:"
    Write-Host "  ssh-keygen -t ed25519 -f ""$SSH_KEY"" -N """"" -ForegroundColor Yellow
    exit 1
}

Write-Success "SSH key found: $SSH_KEY"

# ============================================================================
# STEP 2: Remove SSH Key Passphrase
# ============================================================================
if ($Passphrase -and -not $TestOnly) {
    Write-Step "STEP 2: Remove SSH Key Passphrase"
    
    Write-Info "Removing passphrase from SSH key..."
    $RemoveCmd = "ssh-keygen -p -f ""$SSH_KEY"" -N """" -P ""$Passphrase"""
    
    $output = & cmd /c $RemoveCmd 2>&1
    
    if ($LASTEXITCODE -eq 0) {
        Write-Success "SSH key passphrase removed"
    } else {
        Write-Error "Failed to remove passphrase"
        Write-Error $output
        Write-Info "Make sure you entered the correct current passphrase"
        exit 1
    }
} elseif ($Passphrase) {
    Write-Warning "Passphrase provided but in TEST MODE - skipping"
}

# ============================================================================
# STEP 3: Test SSH Connection
# ============================================================================
Write-Step "STEP 3: Test SSH Connection"

Write-Info "Testing SSH connection to ${PiAddress}..."
$TestOutput = & ssh $PI_ADDRESS "echo 'SSH connection successful!'" 2>&1

if ($LASTEXITCODE -eq 0) {
    Write-Success "SSH connection works!"
} else {
    Write-Error "SSH connection failed"
    Write-Error $TestOutput
    exit 1
}

if ($TestOnly) {
    Write-Success "`nSSH test passed! (test mode - stopping here)"
    exit 0
}

# ============================================================================
# STEP 4: Configure Passwordless Sudo
# ============================================================================
Write-Step "STEP 4: Configure Passwordless Sudo on Raspberry Pi"

Write-Info "Adding passwordless sudo entry..."

$SudoersEntry = "$PiUser ALL=(ALL) NOPASSWD:ALL"
$SudoersCmd = "echo '$SudoersEntry' | sudo tee /etc/sudoers.d/010_gisooj-nopasswd > /dev/null"

$SudoOutput = & ssh $PI_ADDRESS $SudoersCmd 2>&1

if ($LASTEXITCODE -eq 0) {
    Write-Success "Passwordless sudo configured"
} else {
    Write-Error "Failed to configure sudo"
    Write-Error $SudoOutput
    exit 1
}

# ============================================================================
# STEP 5: Test Passwordless Sudo
# ============================================================================
Write-Step "STEP 5: Test Passwordless Sudo"

Write-Info "Testing sudo access..."
$SudoTestOutput = & ssh $PI_ADDRESS "sudo echo 'Sudo works without password!'" 2>&1

if ($LASTEXITCODE -eq 0) {
    Write-Success "Passwordless sudo works!"
} else {
    Write-Warning "Sudo test failed - check sudoers configuration"
    Write-Error $SudoTestOutput
}

# ============================================================================
# STEP 6: Optional - Enable Auto-Login
# ============================================================================
if ($AutoLogin) {
    Write-Step "STEP 6: Enable Auto-Login at Boot"
    
    Write-Info "Enabling auto-login to CLI..."
    $AutoLoginOutput = & ssh $PI_ADDRESS "sudo raspi-config nonint do_boot_behaviour B1" 2>&1
    
    if ($LASTEXITCODE -eq 0) {
        Write-Success "Auto-login enabled"
    } else {
        Write-Warning "Auto-login failed (may require manual setup)"
    }
}

# ============================================================================
# FINAL TEST: Robot Operations
# ============================================================================
Write-Step "FINAL TEST: Robot Operations"

$tests = @(
    @{cmd = "whoami"; desc = "Current user"},
    @{cmd = "sudo whoami"; desc = "Sudo access"},
    @{cmd = "pwd"; desc = "Working directory"},
    @{cmd = 'echo $HOME'; desc = "Home directory"}
)

$allPassed = $true
foreach ($test in $tests) {
    Write-Info $test.desc
    $output = & ssh $PI_ADDRESS $test.cmd 2>&1
    
    if ($LASTEXITCODE -eq 0) {
        Write-Success "$($test.desc): $output"
    } else {
        Write-Error "$($test.desc): Failed"
        $allPassed = $false
    }
}

# ============================================================================
# Summary
# ============================================================================
Write-Host "`n========================================================================" -ForegroundColor Cyan
if ($allPassed) {
    Write-Success "PASSWORDLESS SETUP COMPLETE!"
    Write-Host "`nYour Raspberry Pi can now run robot operations without password prompts:" -ForegroundColor Green
    Write-Host "  ssh $PI_ADDRESS 'sudo python ~/VLA-HTN/run_camera_service.py'" -ForegroundColor Yellow
    Write-Host "  ssh $PI_ADDRESS 'cd ~/VLA-HTN && source venv/bin/activate && python test_microphone_vla_detection.py'" -ForegroundColor Yellow
} else {
    Write-Error "Some tests failed - see errors above"
}
Write-Host "========================================================================`n" -ForegroundColor Cyan

exit 0
