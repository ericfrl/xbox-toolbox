# Xbox Toolbox

**FRL Multi-Robot Control via Xbox Controller**

A standalone tool for controlling multiple FRL robots, linear tracks (J7), and tube feeders (J9) using an Xbox controller. Features a dark cyberpunk-themed GUI with LED status indicators and pathway training/playback.

![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)
![Platform](https://img.shields.io/badge/Platform-Linux-green.svg)
![License](https://img.shields.io/badge/License-MIT-yellow.svg)

## Features

- **Multi-Robot Control**: Switch between Robot 1, Robot 2, or control both simultaneously
- **Xbox Controller Support**: Full gamepad integration with analog sticks, triggers, and buttons
- **Train/Move Modes**: Record waypoints and playback pathways for automated motion
- **Linear Track (J7)**: Control external linear axis via X/Y buttons
- **Tube Feeder (J9)**: Arduino-controlled tube feeding system via LB/RB bumpers
- **Joint & Cartesian Modes**: Toggle between joint-space and Cartesian jogging
- **Pathway Persistence**: Save/load pathways as JSON files for reuse
- **Dark Cyber Theme**: Modern GUI with LED indicators and visual feedback
- **Auto-Save Window Position**: Remembers window size and location
- **Auto-Kill Previous Instances**: Only one instance runs at a time

## Operation Modes

### Move Mode (Default)
Standard jogging mode for manual robot control.

### Train Mode
Record waypoints to create automated pathways:
1. Click the **MOVE** button to switch to **TRAIN** mode
2. Use sticks/d-pad to position the robot
3. Press **A** to add a waypoint at current position
4. Repeat to build a pathway
5. Press **Start** or click **Play** to execute the pathway

## Button Mapping

### Move Mode (Default)

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

### Train Mode

| Button | Function |
|--------|----------|
| **A** | Add Waypoint |
| **B** | Delete Last Waypoint |
| **Start** | Toggle Playback |
| **Back** | Cycle Robot: R1 → R2 → Both |
| **Sticks/D-pad** | Position robot (same as Move mode) |
| **X/Y** | J7 Track (same as Move mode) |
| **LB/RB** | Tube Feeder (same as Move mode) |
| **LT/RT** | Adjust Speed (same as Move mode) |

## Pathway Panel (Train Mode)

When in Train mode, a pathway panel appears with:
- **Waypoint Counter**: Shows number of recorded waypoints
- **Name Field**: Enter a name for the pathway
- **Save**: Save pathway to `~/.xbox_toolbox_pathways/`
- **Load**: Load a previously saved pathway
- **Clear**: Delete all waypoints from current pathway
- **Loop**: Toggle continuous playback
- **Play/Stop**: Start or stop pathway execution

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

## Pathway Data Format

Pathways are saved as JSON files in `~/.xbox_toolbox_pathways/`:

```json
{
  "name": "pathway_1",
  "robot_mode": "both",
  "created": "2025-12-31T12:00:00",
  "waypoints": [
    {
      "r1": {"joints": [0.0, -15.0, 30.0, 0.0, 45.0, 0.0], "j7": 100.0},
      "r2": {"joints": [0.0, -15.0, 30.0, 0.0, 45.0, 0.0], "j7": 50.0},
      "feeder": 25.5
    }
  ]
}
```

## Related Projects

- [Tube Feeder](../tube_feeder/) - Arduino tube feeder controller
- [Heartbeat](../Heartbeat/) - FRL System Monitor Dashboard

---

Built for FRL dual-robot systems with Xbox controller integration.
