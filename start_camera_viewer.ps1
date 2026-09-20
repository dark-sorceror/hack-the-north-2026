#!/usr/bin/env pwsh
<#
.SYNOPSIS
Start camera service on Raspberry Pi and launch live viewer on Windows

.DESCRIPTION
This script:
1. SSHes to the Raspberry Pi
2. Starts the camera service with object detection
3. Launches a live viewer on your Windows machine

.EXAMPLE
PS> .\start_camera_viewer.ps1

.NOTES
Requires:
- SSH access to gisoopi.local (or modify $PI_HOST)
- Camera service Python environment set up on Pi
- ROBOT_TOKEN environment variable set
#>

$PI_HOST = "gisooj@gisoopi.local"
$PI_PATH = "~/VLA-HTN"
$CAMERA_PORT = "8767"
$ROBOT_TOKEN = "test-token-12345678901234567890"

Write-Host "================================" -ForegroundColor Cyan
Write-Host "🎥 VLA-HTN CAMERA VIEWER STARTUP" -ForegroundColor Cyan
Write-Host "================================`n" -ForegroundColor Cyan

# Step 1: Start camera service on Pi
Write-Host "[1/3] Starting camera service on Raspberry Pi..." -ForegroundColor Yellow
Write-Host "      Host: $PI_HOST" -ForegroundColor Gray
Write-Host "      Port: $CAMERA_PORT" -ForegroundColor Gray
Write-Host "      Path: $PI_PATH`n" -ForegroundColor Gray

$startCommand = @"
cd $PI_PATH && `
source venv/bin/activate && `
export ROBOT_TOKEN='$ROBOT_TOKEN' && `
echo '✓ Environment ready' && `
python -m robot_app.camera --host 0.0.0.0 --port $CAMERA_PORT 2>&1 | head -50
"@

# Run camera service startup
ssh $PI_HOST $startCommand

# Check if it started successfully
$checkCommand = @"
netstat -tuln | grep $CAMERA_PORT || echo 'Port not yet listening - may still be starting'
"@

Write-Host "`n[2/3] Checking camera service status..." -ForegroundColor Yellow
ssh $PI_HOST $checkCommand

# Step 3: Start viewer
Write-Host "`n[3/3] Launching camera viewer on Windows..." -ForegroundColor Yellow

# Set environment variables
$env:ROBOT_TOKEN = $ROBOT_TOKEN
$env:CAMERA_URL = "ws://gisoopi.local:$CAMERA_PORT"

Write-Host "       Camera URL: $env:CAMERA_URL`n" -ForegroundColor Gray

# Launch viewer
python view_live_camera.py

Write-Host "`n================================" -ForegroundColor Green
Write-Host "✓ Camera viewer closed" -ForegroundColor Green
Write-Host "================================`n" -ForegroundColor Green
