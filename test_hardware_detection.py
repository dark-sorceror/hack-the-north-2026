#!/usr/bin/env python3
"""Detect and test hardware devices on Raspberry Pi."""
import subprocess
import os

print("=" * 60)
print("RASPBERRY PI HARDWARE DETECTION")
print("=" * 60)

# USB Devices
print("\n1. USB DEVICES:")
result = subprocess.run(["lsusb"], capture_output=True, text=True)
for line in result.stdout.split("\n"):
    if "Intel" in line or "Jabra" in line or "SPEAK" in line:
        print(f"   ✓ {line.strip()}")

# Video devices
print("\n2. VIDEO DEVICES (/dev/video*):")
try:
    video_devs = sorted([f for f in os.listdir("/dev") if f.startswith("video")])[:6]
    print(f"   ✓ Found {len(video_devs)} video device nodes: {', '.join(video_devs)}")
except Exception as e:
    print(f"   ✗ Error: {e}")

# Audio devices
print("\n3. AUDIO DEVICES (ARECORD):")
result = subprocess.run(["arecord", "-l"], capture_output=True, text=True)
for line in result.stdout.split("\n"):
    if "card" in line or "Jabra" in line:
        print(f"   ✓ {line.strip()}")

# Test sounddevice
print("\n4. SOUNDDEVICE TEST:")
try:
    import sounddevice as sd
    devices = sd.query_devices()
    mics = [d for d in devices if d["max_input_channels"] > 0]
    if mics:
        for mic in mics[:3]:
            print(f"   ✓ {mic['name']} (channels: {mic['max_input_channels']})")
    else:
        print("   ✗ No microphone devices found")
except ImportError:
    print("   ✗ sounddevice not installed")
except Exception as e:
    print(f"   ✗ Error: {e}")

# Test pyrealsense2 if available
print("\n5. REALSENSE CAMERA TEST:")
try:
    import pyrealsense2 as rs
    ctx = rs.context()
    devices = ctx.query_devices()
    if devices:
        for i, dev in enumerate(devices):
            name = dev.get_info(rs.camera_info.name)
            serial = dev.get_info(rs.camera_info.serial_number)
            print(f"   ✓ Device {i}: {name} (Serial: {serial})")
    else:
        print("   ✗ No RealSense devices detected via SDK")
except ImportError:
    print("   ✗ pyrealsense2 not installed")
except Exception as e:
    print(f"   ✗ Error: {e}")

# Test OpenCV camera access
print("\n6. OPENCV CAMERA ACCESS TEST:")
try:
    import cv2
    cap = cv2.VideoCapture(0)
    if cap.isOpened():
        ret, frame = cap.read()
        if ret:
            print(f"   ✓ Captured frame: {frame.shape}")
        else:
            print(f"   ⚠ Camera opened but no frame captured")
        cap.release()
    else:
        print(f"   ✗ Could not open /dev/video0")
except Exception as e:
    print(f"   ✗ Error: {e}")

print("\n" + "=" * 60)
