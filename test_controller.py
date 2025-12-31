#!/usr/bin/env python3
"""
Xbox Controller Test Script
Tests if your Xbox controller is detected and shows input events.

Usage:
    python3 test_controller.py
"""

import sys
import time

def test_controller():
    print("=" * 50)
    print("XBOX CONTROLLER TEST")
    print("=" * 50)
    print()

    # Check for inputs library
    try:
        from inputs import devices, get_gamepad
        print("[OK] 'inputs' library installed")
    except ImportError:
        print("[ERROR] 'inputs' library not found")
        print("Install with: pip install inputs")
        return False

    # Check for gamepads
    print()
    print(f"Gamepads detected: {len(devices.gamepads)}")

    if not devices.gamepads:
        print("[ERROR] No gamepad found!")
        print()
        print("Troubleshooting:")
        print("  1. Make sure Xbox controller is connected via USB")
        print("  2. Check: ls /dev/input/js*")
        print("  3. Check: cat /proc/bus/input/devices | grep -A5 Xbox")
        return False

    for i, gp in enumerate(devices.gamepads):
        print(f"  [{i}] {gp.name}")

    print()
    print("[OK] Controller detected!")
    print()
    print("Testing inputs (move sticks, press buttons for 15 seconds)...")
    print("-" * 50)

    start = time.time()
    try:
        while time.time() - start < 15:
            events = get_gamepad()
            for event in events:
                if event.ev_type != 'Sync':
                    print(f"{event.code:20} = {event.state}")
    except KeyboardInterrupt:
        print("\nInterrupted")

    print("-" * 50)
    print("Test complete!")
    return True


if __name__ == "__main__":
    success = test_controller()
    sys.exit(0 if success else 1)
