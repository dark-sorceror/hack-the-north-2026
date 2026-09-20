#!/usr/bin/env python3
"""Check the RealSense camera on the Raspberry Pi: enumerate, stream, capture.

Run by hand on the board - it needs a camera plugged in:
  ssh gisooj@gisoopi.local "cd ~/VLA-HTN && python tests/hardware/test_camera_on_pi.py"
"""
import asyncio
import sys


async def test_camera_hardware():
    """Test RealSense camera detection and basic capture."""
    print("=" * 60)
    print("TESTING REALSENSE CAMERA")
    print("=" * 60)

    try:
        import pyrealsense2 as rs
        print("✓ pyrealsense2 imported")

        # Detect cameras
        ctx = rs.context()
        devices = ctx.query_devices()

        if len(devices) == 0:
            print("✗ No RealSense cameras detected")
            return False

        print(f"✓ Found {len(devices)} RealSense device(s)")

        for i, device in enumerate(devices):
            serial = device.get_info(rs.camera_info.serial_number)
            name = device.get_info(rs.camera_info.name)
            print(f"  Device {i}: {name} (Serial: {serial})")

        # Try to start pipeline
        print("\nTesting pipeline startup...")
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        try:
            pipeline.start(config)
            print("✓ Pipeline started successfully")

            # Capture a frame
            print("Capturing test frame...")
            frames = pipeline.wait_for_frames(timeout_ms=5000)
            if frames:
                print(f"✓ Captured frame set with {frames.size()} streams")

            pipeline.stop()
            print("✓ Pipeline stopped cleanly")
            return True
        except Exception as e:
            print(f"✗ Pipeline error: {e}")
            return False

    except ImportError as e:
        print(f"✗ Import error: {e}")
        return False
    except Exception as e:
        print(f"✗ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        return False


async def main():
    print("\n" + "=" * 60)
    print("REALSENSE HARDWARE TEST")
    print("=" * 60 + "\n")

    passed = await test_camera_hardware()

    print("\n" + "=" * 60)
    print("TEST SUMMARY")
    print("=" * 60)
    print(f"{'✓ PASS' if passed else '✗ FAIL'}: camera_hardware\n")

    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
