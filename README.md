# Xbox Toolbox

**AR4 Multi-Robot Control via Xbox Controller**

A standalone tool for controlling multiple AR4 robots, linear tracks (J7), and tube feeders (J9) using an Xbox controller. Features a dark cyberpunk-themed GUI with LED status indicators.

![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)
![Platform](https://img.shields.io/badge/Platform-Linux-green.svg)
![License](https://img.shields.io/badge/License-MIT-yellow.svg)

## Features

- **Multi-Robot Control**: Switch between Robot 1, Robot 2, or control both simultaneously
- **Xbox Controller Support**: Full gamepad integration with analog sticks, triggers, and buttons
- **Linear Track (J7)**: Control external linear axis via X/Y buttons
- **Tube Feeder (J9)**: Arduino-controlled tube feeding system via LB/RB bumpers
- **Joint & Cartesian Modes**: Toggle between joint-space and Cartesian jogging
- **Dark Cyber Theme**: Modern GUI with LED indicators and visual feedback
- **Auto-Save Window Position**: Remembers window size and location
- **Auto-Kill Previous Instances**: Only one instance runs at a time

## Button Mapping

| Button | Function |
|--------|----------|
| **A** | Joint Mode |
| **B** | Cartesian Mode |
| **Back** | Cycle Robot: R1 → R2 → Both |
| **Start** | EMERGENCY STOP ALL |
| **Left Stick** | J1/J2 (Joint) or X/Y (Cartesian) |
| **Right Stick** | J3/J4 (Joint) or Z/Rz (Cartesian) |
| **D-pad** | J5/J6 (Joint) or Rx/Ry (Cartesian) |
| **X** | J7 Track Negative |
| **Y** | J7 Track Positive |
| **LB** | J9 Tube Feeder Retract |
| **RB** | J9 Tube Feeder Feed |
| **LT** | Decrease Speed |
| **RT** | Increase Speed |

## Installation

### Prerequisites

- Python 3.8+
- Linux (tested on Ubuntu)
- Xbox controller (USB connected)

### Install Dependencies

```bash
pip install -r requirements.txt
```

Or manually:
```bash
pip install inputs pyserial
```

### Permissions

Add your user to the `dialout` group for serial port access:
```bash
sudo usermod -a -G dialout $USER
```
Log out and back in for changes to take effect.

## Usage

### Quick Start

```bash
# Using the run script
./run.sh

# Or directly
python3 xbox_toolbox.py
```

### Testing

Test your Xbox controller:
```bash
python3 test_controller.py
```

Test serial port detection:
```bash
python3 test_serial_ports.py
```

## Hardware Setup

### Robots (AR4)
- Connect via USB (Teensy 4.1)
- Each robot appears as a separate serial port
- Auto-detected by the toolbox

### Tube Feeder (J9)
- Arduino Uno with DM322T stepper driver
- NEMA 17 stepper motor
- See [tube_feeder documentation](../tube_feeder/README.md) for wiring

### Linear Track (J7)
- Connected to robot's J7 axis output
- Controlled via robot serial commands

## Configuration

Window position and size are automatically saved to `.xbox_toolbox_config.json` when you close the application.

## Troubleshooting

### Xbox Controller Not Detected

1. Check if connected:
   ```bash
   ls /dev/input/js*
   ```

2. Check device info:
   ```bash
   cat /proc/bus/input/devices | grep -A5 Xbox
   ```

3. Run the test script:
   ```bash
   python3 test_controller.py
   ```

### Serial Ports Not Found

1. Check connections:
   ```bash
   python3 test_serial_ports.py
   ```

2. Check permissions:
   ```bash
   ls -la /dev/ttyACM* /dev/ttyUSB*
   ```

3. Add to dialout group (see Installation)

### Sticks Not Responding

- The deadzone is set to 0.25 - push sticks past 25% to register
- Check the action display shows stick values when moving

## File Structure

```
xbox-toolbox/
├── xbox_toolbox.py      # Main application
├── test_controller.py   # Xbox controller test script
├── test_serial_ports.py # Serial port detection script
├── run.sh               # Launch script
├── requirements.txt     # Python dependencies
└── README.md            # This file
```

## License

MIT License - See LICENSE file for details.

## Related Projects

- [AR4 Robot Arm](https://github.com/SkyentificGit/AR4) - Original AR4 project
- [Tube Feeder](../tube_feeder/) - Arduino tube feeder controller

---

Built for AR4 dual-robot systems with Xbox controller integration.
