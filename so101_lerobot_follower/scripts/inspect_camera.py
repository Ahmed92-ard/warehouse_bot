#!/usr/bin/env python3
"""Linux V4L2 USB Camera Inspector for SO-101 Follower Camera Setup."""

import sys
import subprocess
import glob


def inspect():
    print("=" * 60)
    print("SO-101 Follower USB Camera Inspector (V4L2)")
    print("=" * 60)

    devices = sorted(glob.glob("/dev/video*"))
    if not devices:
        print("No /dev/video* devices detected on this host.")
        return

    print(f"Found {len(devices)} video device(s): {', '.join(devices)}\n")

    # Run v4l2-ctl --list-devices
    try:
        res = subprocess.run(["v4l2-ctl", "--list-devices"], capture_output=True, text=True, check=True)
        print("--- V4L2 Devices ---")
        print(res.stdout)
    except Exception as e:
        print(f"v4l2-ctl --list-devices returned error: {e}")

    # Inspect each video device formats
    for dev in devices:
        print(f"\n--- Formats & Modes for {dev} ---")
        try:
            res = subprocess.run(["v4l2-ctl", f"-d{dev}", "--list-formats-ext"], capture_output=True, text=True)
            if res.returncode == 0:
                print(res.stdout)
            else:
                print(f"Device {dev} does not support format query: {res.stderr.strip()}")
        except Exception as e:
            print(f"Could not query {dev}: {e}")

    print("=" * 60)
    print("Recommendation for Zebronics Zeb-Sharp Pro / USB Camera:")
    print("Default device: /dev/video0")
    print("Supported formats: MJPEG (preferred for 30/60fps) or YUYV")
    print("Recommended stream resolution: 640x480 @ 30 FPS (JPEG quality: 75)")
    print("=" * 60)


if __name__ == "__main__":
    inspect()

