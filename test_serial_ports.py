#!/usr/bin/env python3
"""
Serial Port Detection Script
Detects and identifies connected serial devices (Teensy, Arduino, etc.)

Usage:
    python3 test_serial_ports.py
"""

import sys

def test_serial_ports():
    print("=" * 50)
    print("SERIAL PORT DETECTION")
    print("=" * 50)
    print()

    try:
        import serial.tools.list_ports
        print("[OK] 'pyserial' library installed")
    except ImportError:
        print("[ERROR] 'pyserial' library not found")
        print("Install with: pip install pyserial")
        return False

    print()
    ports = list(serial.tools.list_ports.comports())

    if not ports:
        print("[WARNING] No serial ports found!")
        print()
        print("Troubleshooting:")
        print("  1. Connect your devices via USB")
        print("  2. Check permissions: sudo usermod -a -G dialout $USER")
        print("  3. Log out and back in after adding to dialout group")
        return False

    print(f"Found {len(ports)} serial port(s):")
    print("-" * 50)

    teensys = []
    arduinos = []
    others = []

    for port in ports:
        desc = port.description.lower() if port.description else ''
        hwid = port.hwid if port.hwid else ''

        print(f"\nDevice: {port.device}")
        print(f"  Description: {port.description}")
        print(f"  HWID: {port.hwid}")
        print(f"  VID:PID: {port.vid}:{port.pid}" if port.vid else "  VID:PID: N/A")

        if 'teensy' in desc:
            teensys.append(port.device)
            print("  Type: TEENSY (AR4 Robot)")
        elif 'arduino' in desc:
            if 'uno' in desc:
                arduinos.append(port.device)
                print("  Type: ARDUINO UNO (Tube Feeder)")
            else:
                arduinos.append(port.device)
                print("  Type: ARDUINO")
        else:
            others.append(port.device)
            print("  Type: OTHER")

    print()
    print("-" * 50)
    print("SUMMARY:")
    print(f"  Teensy (Robots): {teensys if teensys else 'None found'}")
    print(f"  Arduino (Feeder): {arduinos if arduinos else 'None found'}")
    print(f"  Other: {others if others else 'None'}")
    print()

    if teensys or arduinos:
        print("[OK] Devices detected!")
        return True
    else:
        print("[WARNING] No Teensy or Arduino found")
        return False


if __name__ == "__main__":
    success = test_serial_ports()
    sys.exit(0 if success else 1)
