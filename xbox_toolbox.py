#!/usr/bin/env python3
"""
Xbox Toolbox for AR4 Multi-Robot Control
=========================================
Standalone tool for controlling:
- Robot 1 (6-axis + J7 linear track + J9 tube feeder)
- Robot 2 (6-axis + J7 linear track)
- Coordinated dual-robot operations

All via Xbox controller with a simple Tkinter GUI.

Usage:
    python3 xbox_toolbox.py

Button Mapping:
    Back        - Cycle device: R1 → R2 → Both
    Start       - EMERGENCY STOP ALL
    A           - Joint mode
    B           - Cartesian mode
    X           - J7 negative (linear track)
    Y           - J7 positive (linear track)
    Left Stick  - J1/J2 (Joint) or X/Y (Cartesian)
    Right Stick - J3/J4 (Joint) or Z/Rz (Cartesian)
    D-pad       - J5/J6 (Joint) or Rx/Ry (Cartesian)
    LB          - J9 retract (tube feeder reverse)
    RB          - J9 feed (tube feeder forward)
    LT          - Decrease speed
    RT          - Increase speed

Note: No gripper - robot has permanent magnet end effector
J9 = Tube Feeder (Arduino-controlled external axis)
"""

import sys
import os
import time
import threading
import json
import ctypes
import serial
import serial.tools.list_ports
import tkinter as tk
from tkinter import ttk, messagebox

# =============================================================================
# CONFIGURATION
# =============================================================================

DEFAULT_BAUDRATE = 115200
FEEDER_BAUDRATE = 115200

# Default COM ports (will be auto-detected or user-selected)
ROBOT1_PORT = None
ROBOT2_PORT = None
FEEDER_PORT = None

# Jogging parameters
DEFAULT_SPEED = 25
DEFAULT_ACCEL = 10
DEFAULT_DECEL = 10

# =============================================================================
# TUBE FEEDER CONTROLLER (from tube_feeder/feeder_test.py)
# =============================================================================

class TubeFeederController:
    """Controller for Arduino-based tube feeder."""

    def __init__(self, port=None, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.connected = False
        self.reading = False
        self.read_thread = None
        self.position = 0.0
        self.last_response = ""

    def find_arduino(self):
        """Auto-detect Arduino Uno port."""
        ports = serial.tools.list_ports.comports()
        for port in ports:
            desc = port.description.lower() if port.description else ''
            if 'arduino' in desc and 'uno' in desc:
                return port.device
        for port in ports:
            desc = port.description.lower() if port.description else ''
            if 'arduino' in desc and 'teensy' not in desc:
                return port.device
        for port in ports:
            if port.vid == 0x2341:  # Arduino vendor ID
                if port.pid in [0x0043, 0x0001, 0x0243]:
                    return port.device
        return None

    def connect(self):
        """Connect to the Arduino."""
        if self.port is None:
            self.port = self.find_arduino()
            if self.port is None:
                return False
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=0.1)
            time.sleep(2)
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            self.connected = True
            self.reading = True
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.read_thread.start()
            return True
        except serial.SerialException:
            return False

    def disconnect(self):
        """Disconnect from Arduino."""
        self.reading = False
        if self.serial and self.serial.is_open:
            self.serial.close()
        self.connected = False

    def _read_loop(self):
        """Background thread to read serial responses."""
        while self.reading and self.serial and self.serial.is_open:
            try:
                if self.serial.in_waiting:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.last_response = line
                        if line.startswith("POS:"):
                            try:
                                self.position = float(line[4:])
                            except:
                                pass
            except:
                pass
            time.sleep(0.01)

    def send(self, command):
        """Send a command to the Arduino."""
        if not self.connected:
            return False
        try:
            cmd = command.strip() + '\n'
            self.serial.write(cmd.encode())
            self.serial.flush()
            return True
        except:
            return False

    def feed(self, mm):
        return self.send(f"F{mm}")

    def retract(self, mm):
        return self.send(f"R{mm}")

    def set_speed(self, mm_per_sec):
        return self.send(f"S{mm_per_sec}")

    def jog_forward(self):
        return self.send("J+")

    def jog_reverse(self):
        return self.send("J-")

    def stop(self):
        return self.send("STOP")

    def home(self):
        return self.send("HOME")

    def get_position(self):
        return self.send("POS")


# =============================================================================
# ROBOT CONTROLLER
# =============================================================================

class RobotController:
    """Controller for AR4 robot via serial."""

    def __init__(self, name="Robot", port=None, baudrate=115200):
        self.name = name
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.connected = False
        self.reading = False
        self.read_thread = None

        # Current position (from encoder feedback)
        self.joints = [0.0] * 6  # J1-J6
        self.j7_pos = 0.0  # Linear track
        self.cartesian = [0.0] * 6  # X, Y, Z, Rx, Ry, Rz

        # Jogging state
        self.jogging = False
        self.last_response = ""

    def find_teensy(self, exclude_ports=None):
        """Auto-detect Teensy port for AR4."""
        exclude = exclude_ports or []
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.device in exclude:
                continue
            desc = port.description.lower() if port.description else ''
            if 'teensy' in desc:
                return port.device
        return None

    def connect(self):
        """Connect to the robot."""
        if self.port is None:
            return False
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=0.1)
            time.sleep(0.5)
            self.serial.reset_input_buffer()
            self.connected = True
            self.reading = True
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.read_thread.start()
            return True
        except serial.SerialException as e:
            print(f"[{self.name}] Connection error: {e}")
            return False

    def disconnect(self):
        """Disconnect from robot."""
        self.reading = False
        self.stop_jog()
        if self.serial and self.serial.is_open:
            self.serial.close()
        self.connected = False

    def _read_loop(self):
        """Background thread to read serial responses."""
        while self.reading and self.serial and self.serial.is_open:
            try:
                if self.serial.in_waiting:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.last_response = line
                        self._parse_response(line)
            except:
                pass
            time.sleep(0.005)

    def _parse_response(self, line):
        """Parse position feedback from robot."""
        # AR4 sends position data in format: A1.23B4.56C... etc.
        try:
            if line.startswith("A"):
                # Parse joint angles
                pass  # Implement based on actual AR4 response format
        except:
            pass

    def send(self, command):
        """Send a command to the robot."""
        if not self.connected:
            return False
        try:
            cmd = command if command.endswith('\n') else command + '\n'
            self.serial.write(cmd.encode())
            self.serial.flush()
            return True
        except:
            return False

    def stop_jog(self):
        """Stop all jogging motion."""
        self.jogging = False
        return self.send("S")

    def jog_joint(self, joint, direction, speed=25, accel=10, decel=10):
        """
        Start live jogging a joint.
        joint: 1-6 for main joints, 7 for linear track
        direction: +1 or -1
        """
        # Live jog command format: LJ + joint code
        # Code: J1- = 10, J1+ = 11, J2- = 20, J2+ = 21, etc.
        code = joint * 10 + (1 if direction > 0 else 0)
        self.jogging = True
        return self.send(f"LJ{code}S{speed}A{accel}D{decel}")

    def jog_cartesian(self, axis, direction, speed=25, accel=10, decel=10):
        """
        Start live jogging in Cartesian space.
        axis: 'X', 'Y', 'Z', 'Rx', 'Ry', 'Rz'
        direction: +1 or -1
        """
        axis_map = {'X': 1, 'Y': 2, 'Z': 3, 'Rx': 4, 'Ry': 5, 'Rz': 6}
        if axis not in axis_map:
            return False
        code = axis_map[axis] * 10 + (1 if direction > 0 else 0)
        self.jogging = True
        return self.send(f"LC{code}S{speed}A{accel}D{decel}")

    def jog_j7(self, direction, speed=25):
        """Jog linear track (J7)."""
        code = 70 + (1 if direction > 0 else 0)
        self.jogging = True
        return self.send(f"LJ{code}S{speed}")

    def emergency_stop(self):
        """Emergency stop."""
        self.jogging = False
        return self.send("ES")

    def get_position(self):
        """Request current position."""
        return self.send("GP")


# =============================================================================
# XBOX CONTROLLER (Cross-platform)
# =============================================================================

class XboxController:
    """Cross-platform Xbox controller interface."""

    # Button constants
    BTN_A = 'A'
    BTN_B = 'B'
    BTN_X = 'X'
    BTN_Y = 'Y'
    BTN_LB = 'LB'
    BTN_RB = 'RB'
    BTN_BACK = 'BACK'
    BTN_START = 'START'
    DPAD_UP = 'DPAD_UP'
    DPAD_DOWN = 'DPAD_DOWN'
    DPAD_LEFT = 'DPAD_LEFT'
    DPAD_RIGHT = 'DPAD_RIGHT'

    def __init__(self):
        self.connected = False
        self.platform = sys.platform
        self._xinput = None
        self._controller_idx = None

        # Current state
        self.buttons = set()
        self.prev_buttons = set()
        self.left_stick = (0.0, 0.0)
        self.right_stick = (0.0, 0.0)
        self.left_trigger = 0.0
        self.right_trigger = 0.0

        # Callbacks
        self.on_button_press = None
        self.on_button_release = None
        self.on_stick_move = None
        self.on_trigger = None

        # Deadzone
        self.deadzone = 0.15

        # Polling thread
        self._polling = False
        self._poll_thread = None

    def connect(self):
        """Initialize controller connection."""
        if self.platform == 'win32':
            return self._connect_windows()
        else:
            return self._connect_linux()

    def _connect_windows(self):
        """Connect using Windows XInput."""
        try:
            for dll in ["xinput1_4.dll", "xinput1_3.dll", "xinput9_1_0.dll"]:
                try:
                    self._xinput = ctypes.WinDLL(dll)
                    break
                except OSError:
                    continue

            if self._xinput is None:
                return False

            # Define structures
            class XINPUT_GAMEPAD(ctypes.Structure):
                _fields_ = [
                    ("wButtons", ctypes.c_ushort),
                    ("bLeftTrigger", ctypes.c_ubyte),
                    ("bRightTrigger", ctypes.c_ubyte),
                    ("sThumbLX", ctypes.c_short),
                    ("sThumbLY", ctypes.c_short),
                    ("sThumbRX", ctypes.c_short),
                    ("sThumbRY", ctypes.c_short),
                ]

            class XINPUT_STATE(ctypes.Structure):
                _fields_ = [
                    ("dwPacketNumber", ctypes.c_uint),
                    ("Gamepad", XINPUT_GAMEPAD)
                ]

            self._XINPUT_STATE = XINPUT_STATE
            self._xinput.XInputGetState.argtypes = [ctypes.c_uint, ctypes.POINTER(XINPUT_STATE)]
            self._xinput.XInputGetState.restype = ctypes.c_uint

            # Find controller
            state = XINPUT_STATE()
            for i in range(4):
                if self._xinput.XInputGetState(i, ctypes.byref(state)) == 0:
                    self._controller_idx = i
                    self.connected = True
                    return True
            return False

        except Exception as e:
            print(f"XInput error: {e}")
            return False

    def _connect_linux(self):
        """Connect using inputs library on Linux."""
        try:
            from inputs import devices
            if devices.gamepads:
                self.connected = True
                return True
            return False
        except ImportError:
            print("Install 'inputs' package: pip install inputs")
            return False
        except Exception:
            return False

    def start_polling(self):
        """Start background polling thread."""
        if not self.connected:
            return False
        self._polling = True
        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._poll_thread.start()
        return True

    def stop_polling(self):
        """Stop polling thread."""
        self._polling = False

    def _poll_loop(self):
        """Main polling loop."""
        if self.platform == 'win32':
            self._poll_windows()
        else:
            self._poll_linux()

    def _poll_windows(self):
        """Windows XInput polling."""
        # Button masks
        BTN_MASKS = {
            0x1000: self.BTN_A,
            0x2000: self.BTN_B,
            0x4000: self.BTN_X,
            0x8000: self.BTN_Y,
            0x0100: self.BTN_LB,
            0x0200: self.BTN_RB,
            0x0020: self.BTN_BACK,
            0x0010: self.BTN_START,
            0x0001: self.DPAD_UP,
            0x0002: self.DPAD_DOWN,
            0x0004: self.DPAD_LEFT,
            0x0008: self.DPAD_RIGHT,
        }

        while self._polling:
            state = self._XINPUT_STATE()
            if self._xinput.XInputGetState(self._controller_idx, ctypes.byref(state)) != 0:
                self.connected = False
                time.sleep(0.5)
                continue

            gp = state.Gamepad

            # Parse buttons
            new_buttons = set()
            for mask, name in BTN_MASKS.items():
                if gp.wButtons & mask:
                    new_buttons.add(name)

            # Detect presses and releases
            pressed = new_buttons - self.prev_buttons
            released = self.prev_buttons - new_buttons

            for btn in pressed:
                if self.on_button_press:
                    self.on_button_press(btn)

            for btn in released:
                if self.on_button_release:
                    self.on_button_release(btn)

            self.prev_buttons = self.buttons
            self.buttons = new_buttons

            # Parse sticks (normalize to -1.0 to 1.0)
            def normalize(val, deadzone=8000):
                if abs(val) < deadzone:
                    return 0.0
                return max(-1.0, min(1.0, val / 32767.0))

            self.left_stick = (normalize(gp.sThumbLX), normalize(gp.sThumbLY))
            self.right_stick = (normalize(gp.sThumbRX), normalize(gp.sThumbRY))

            if self.on_stick_move:
                self.on_stick_move(self.left_stick, self.right_stick)

            # Parse triggers (0-255 to 0.0-1.0)
            self.left_trigger = gp.bLeftTrigger / 255.0
            self.right_trigger = gp.bRightTrigger / 255.0

            if self.on_trigger:
                self.on_trigger(self.left_trigger, self.right_trigger)

            time.sleep(0.008)  # ~120Hz

    def _poll_linux(self):
        """Linux inputs library polling."""
        try:
            from inputs import get_gamepad

            # Event code mappings
            BTN_MAP = {
                'BTN_SOUTH': self.BTN_A,
                'BTN_EAST': self.BTN_B,
                'BTN_WEST': self.BTN_X,
                'BTN_NORTH': self.BTN_Y,
                'BTN_TL': self.BTN_LB,
                'BTN_TR': self.BTN_RB,
                'BTN_SELECT': self.BTN_BACK,
                'BTN_START': self.BTN_START,
            }

            while self._polling:
                try:
                    events = get_gamepad()
                    for event in events:
                        # Buttons
                        if event.code in BTN_MAP:
                            btn = BTN_MAP[event.code]
                            if event.state == 1:
                                self.buttons.add(btn)
                                if self.on_button_press:
                                    self.on_button_press(btn)
                            else:
                                self.buttons.discard(btn)
                                if self.on_button_release:
                                    self.on_button_release(btn)

                        # D-pad
                        elif event.code == 'ABS_HAT0X':
                            self.buttons.discard(self.DPAD_LEFT)
                            self.buttons.discard(self.DPAD_RIGHT)
                            if event.state == -1:
                                self.buttons.add(self.DPAD_LEFT)
                                if self.on_button_press:
                                    self.on_button_press(self.DPAD_LEFT)
                            elif event.state == 1:
                                self.buttons.add(self.DPAD_RIGHT)
                                if self.on_button_press:
                                    self.on_button_press(self.DPAD_RIGHT)

                        elif event.code == 'ABS_HAT0Y':
                            self.buttons.discard(self.DPAD_UP)
                            self.buttons.discard(self.DPAD_DOWN)
                            if event.state == -1:
                                self.buttons.add(self.DPAD_UP)
                                if self.on_button_press:
                                    self.on_button_press(self.DPAD_UP)
                            elif event.state == 1:
                                self.buttons.add(self.DPAD_DOWN)
                                if self.on_button_press:
                                    self.on_button_press(self.DPAD_DOWN)

                        # Left stick
                        elif event.code == 'ABS_X':
                            self.left_stick = (event.state / 32767.0, self.left_stick[1])
                        elif event.code == 'ABS_Y':
                            self.left_stick = (self.left_stick[0], -event.state / 32767.0)

                        # Right stick
                        elif event.code == 'ABS_RX':
                            self.right_stick = (event.state / 32767.0, self.right_stick[1])
                        elif event.code == 'ABS_RY':
                            self.right_stick = (self.right_stick[0], -event.state / 32767.0)

                        # Triggers
                        elif event.code == 'ABS_Z':
                            self.left_trigger = event.state / 255.0
                        elif event.code == 'ABS_RZ':
                            self.right_trigger = event.state / 255.0

                        if self.on_stick_move:
                            self.on_stick_move(self.left_stick, self.right_stick)
                        if self.on_trigger:
                            self.on_trigger(self.left_trigger, self.right_trigger)

                except Exception as e:
                    time.sleep(0.1)

        except Exception as e:
            print(f"Linux gamepad error: {e}")


# =============================================================================
# MAIN TOOLBOX APPLICATION
# =============================================================================

class XboxToolbox:
    """Main application combining Xbox controller with multi-robot control."""

    # Device modes (no separate feeder mode - J9 is integrated)
    MODE_ROBOT1 = 0
    MODE_ROBOT2 = 1
    MODE_BOTH = 2
    MODE_NAMES = ["Robot 1", "Robot 2", "Both Robots"]

    # Movement modes
    MOVE_JOINT = 0
    MOVE_CARTESIAN = 1

    # Config file for saving window geometry
    CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), '.xbox_toolbox_config.json')

    def __init__(self, root):
        self.root = root
        self.root.title("Xbox Toolbox - AR4 Multi-Robot Control")
        self.root.resizable(True, True)

        # Load saved geometry or use default
        self._load_geometry()

        # Controllers
        self.xbox = XboxController()
        self.robot1 = RobotController("Robot 1")
        self.robot2 = RobotController("Robot 2")
        self.feeder = TubeFeederController()

        # State
        self.device_mode = self.MODE_ROBOT1
        self.move_mode = self.MOVE_JOINT
        self.speed = DEFAULT_SPEED
        self.current_jog = None  # Track what's currently jogging

        # Build GUI
        self._build_gui()

        # Bind Xbox callbacks
        self.xbox.on_button_press = self._on_button_press
        self.xbox.on_button_release = self._on_button_release
        self.xbox.on_stick_move = self._on_stick_move
        self.xbox.on_trigger = self._on_trigger

        # Start status update loop
        self._update_status()

    def _load_geometry(self):
        """Load saved window geometry from config file."""
        try:
            if os.path.exists(self.CONFIG_FILE):
                with open(self.CONFIG_FILE, 'r') as f:
                    config = json.load(f)
                    geometry = config.get('geometry', '600x800+100+100')
                    self.root.geometry(geometry)
            else:
                self.root.geometry("600x800+100+100")
        except Exception:
            self.root.geometry("600x800+100+100")

    def _save_geometry(self):
        """Save current window geometry to config file."""
        try:
            geometry = self.root.geometry()
            config = {'geometry': geometry}
            with open(self.CONFIG_FILE, 'w') as f:
                json.dump(config, f)
        except Exception:
            pass

    def _build_gui(self):
        """Build the Tkinter GUI with dark cyber theme."""
        # === DARK THEME COLORS ===
        self.colors = {
            'bg_dark': '#1a1a2e',
            'bg_mid': '#16213e',
            'bg_light': '#0f3460',
            'accent': '#e94560',
            'accent2': '#00d9ff',
            'success': '#00ff88',
            'warning': '#ffaa00',
            'text': '#ffffff',
            'text_dim': '#888899',
            'border': '#0f3460'
        }

        # Configure root window
        self.root.configure(bg=self.colors['bg_dark'])

        # Style configuration
        style = ttk.Style()
        style.theme_use('clam')

        # Configure dark theme styles
        style.configure("Dark.TFrame", background=self.colors['bg_dark'])
        style.configure("Dark.TLabelframe", background=self.colors['bg_dark'],
                       foreground=self.colors['accent2'], bordercolor=self.colors['border'])
        style.configure("Dark.TLabelframe.Label", background=self.colors['bg_dark'],
                       foreground=self.colors['accent2'], font=("Consolas", 10, "bold"))
        style.configure("Dark.TLabel", background=self.colors['bg_dark'],
                       foreground=self.colors['text'], font=("Consolas", 10))
        style.configure("Title.TLabel", background=self.colors['bg_dark'],
                       foreground=self.colors['accent'], font=("Consolas", 20, "bold"))
        style.configure("Mode.TLabel", background=self.colors['bg_mid'],
                       foreground=self.colors['accent2'], font=("Consolas", 16, "bold"))
        style.configure("Action.TLabel", background=self.colors['bg_dark'],
                       foreground=self.colors['success'], font=("Consolas", 12, "bold"))
        style.configure("Dark.TButton", background=self.colors['bg_light'],
                       foreground=self.colors['text'], font=("Consolas", 9, "bold"),
                       bordercolor=self.colors['accent2'], padding=5)
        style.map("Dark.TButton",
                 background=[('active', self.colors['accent2']), ('pressed', self.colors['accent'])],
                 foreground=[('active', self.colors['bg_dark'])])
        style.configure("Dark.TCombobox", fieldbackground=self.colors['bg_mid'],
                       background=self.colors['bg_light'], foreground=self.colors['text'])
        style.configure("Dark.Horizontal.TScale", background=self.colors['bg_dark'],
                       troughcolor=self.colors['bg_mid'])

        # Main frame
        main_frame = ttk.Frame(self.root, style="Dark.TFrame", padding="15")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # === HEADER ===
        header_frame = tk.Frame(main_frame, bg=self.colors['bg_dark'])
        header_frame.pack(fill=tk.X, pady=(0, 15))

        title_label = tk.Label(header_frame, text="⎮ XBOX TOOLBOX ⎮",
                              font=("Consolas", 24, "bold"),
                              fg=self.colors['accent'], bg=self.colors['bg_dark'])
        title_label.pack(side=tk.LEFT)

        subtitle = tk.Label(header_frame, text="AR4 Multi-Robot Control",
                           font=("Consolas", 10),
                           fg=self.colors['text_dim'], bg=self.colors['bg_dark'])
        subtitle.pack(side=tk.LEFT, padx=15, pady=8)

        # === STATUS BAR (Top) ===
        status_bar = tk.Frame(main_frame, bg=self.colors['bg_mid'], height=50)
        status_bar.pack(fill=tk.X, pady=(0, 10))
        status_bar.pack_propagate(False)

        # LED indicators
        self._create_led_indicator(status_bar, "XBOX", 0)
        self._create_led_indicator(status_bar, "R1", 1)
        self._create_led_indicator(status_bar, "R2", 2)
        self._create_led_indicator(status_bar, "J9", 3)

        # === ACTIVE MODE DISPLAY ===
        mode_display = tk.Frame(main_frame, bg=self.colors['bg_mid'], height=80)
        mode_display.pack(fill=tk.X, pady=5)
        mode_display.pack_propagate(False)

        self.mode_label = tk.Label(mode_display, text="◉ ROBOT 1",
                                   font=("Consolas", 22, "bold"),
                                   fg=self.colors['accent2'], bg=self.colors['bg_mid'])
        self.mode_label.pack(pady=10)

        self.movement_mode_label = tk.Label(mode_display, text="[ JOINT MODE ]",
                                            font=("Consolas", 12),
                                            fg=self.colors['success'], bg=self.colors['bg_mid'])
        self.movement_mode_label.pack()

        # === CONNECTION PANELS (Grid) ===
        conn_frame = tk.Frame(main_frame, bg=self.colors['bg_dark'])
        conn_frame.pack(fill=tk.X, pady=10)

        # Robot 1
        r1_panel = self._create_connection_panel(conn_frame, "ROBOT 1", 0)
        self.r1_port_var = r1_panel['port_var']
        self.r1_port_combo = r1_panel['combo']
        self.r1_status_led = r1_panel['led']
        r1_panel['button'].configure(command=self._connect_robot1)
        r1_panel['refresh'].configure(command=self._refresh_ports)

        # Robot 2
        r2_panel = self._create_connection_panel(conn_frame, "ROBOT 2", 1)
        self.r2_port_var = r2_panel['port_var']
        self.r2_port_combo = r2_panel['combo']
        self.r2_status_led = r2_panel['led']
        r2_panel['button'].configure(command=self._connect_robot2)

        # J9 Tube Feeder
        j9_panel = self._create_connection_panel(conn_frame, "J9 FEEDER", 2)
        self.feeder_port_var = j9_panel['port_var']
        self.feeder_port_combo = j9_panel['combo']
        self.feeder_status_led = j9_panel['led']
        j9_panel['button'].configure(command=self._connect_feeder)

        # === SPEED CONTROL ===
        speed_frame = tk.Frame(main_frame, bg=self.colors['bg_dark'])
        speed_frame.pack(fill=tk.X, pady=10)

        tk.Label(speed_frame, text="SPEED", font=("Consolas", 10, "bold"),
                fg=self.colors['accent2'], bg=self.colors['bg_dark']).pack(side=tk.LEFT)

        self.speed_var = tk.IntVar(value=DEFAULT_SPEED)
        self.speed_canvas = tk.Canvas(speed_frame, width=300, height=25,
                                      bg=self.colors['bg_mid'], highlightthickness=0)
        self.speed_canvas.pack(side=tk.LEFT, padx=10)
        self._draw_speed_bar()

        self.speed_label = tk.Label(speed_frame, text=f"{DEFAULT_SPEED}%",
                                    font=("Consolas", 14, "bold"),
                                    fg=self.colors['success'], bg=self.colors['bg_dark'])
        self.speed_label.pack(side=tk.LEFT, padx=10)

        # === CURRENT ACTION ===
        action_frame = tk.Frame(main_frame, bg=self.colors['bg_light'], height=50)
        action_frame.pack(fill=tk.X, pady=10)
        action_frame.pack_propagate(False)

        tk.Label(action_frame, text="▶", font=("Consolas", 16),
                fg=self.colors['accent'], bg=self.colors['bg_light']).pack(side=tk.LEFT, padx=10)

        self.action_label = tk.Label(action_frame, text="IDLE",
                                     font=("Consolas", 14, "bold"),
                                     fg=self.colors['text'], bg=self.colors['bg_light'])
        self.action_label.pack(side=tk.LEFT, padx=5, pady=12)

        # === EMERGENCY STOP ===
        estop_btn = tk.Button(main_frame, text="⚠ EMERGENCY STOP ALL ⚠",
                             bg=self.colors['accent'], fg='white',
                             font=("Consolas", 14, "bold"),
                             activebackground='#ff0000', activeforeground='white',
                             relief=tk.FLAT, cursor='hand2', height=2,
                             command=self._emergency_stop_all)
        estop_btn.pack(fill=tk.X, pady=10)

        # === BUTTON MAPPING ===
        map_frame = tk.Frame(main_frame, bg=self.colors['bg_mid'])
        map_frame.pack(fill=tk.X, pady=5)

        mapping_lines = [
            ("A/B", "Mode", self.colors['success']),
            ("Back", "Robot", self.colors['warning']),
            ("Start", "E-STOP", self.colors['accent']),
            ("L-Stick", "J1/J2", self.colors['accent2']),
            ("R-Stick", "J3/J4", self.colors['accent2']),
            ("D-pad", "J5/J6", self.colors['accent2']),
            ("X/Y", "J7 Track", self.colors['success']),
            ("LB/RB", "J9 Feed", self.colors['warning']),
        ]

        for i, (btn, action, color) in enumerate(mapping_lines):
            tk.Label(map_frame, text=btn, font=("Consolas", 8, "bold"),
                    fg=color, bg=self.colors['bg_mid']).grid(row=0, column=i*2, padx=3, pady=5)
            tk.Label(map_frame, text=action, font=("Consolas", 8),
                    fg=self.colors['text_dim'], bg=self.colors['bg_mid']).grid(row=1, column=i*2, padx=3)

        # === LOG ===
        log_frame = tk.Frame(main_frame, bg=self.colors['bg_dark'])
        log_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        tk.Label(log_frame, text="─── LOG ───", font=("Consolas", 9),
                fg=self.colors['text_dim'], bg=self.colors['bg_dark']).pack()

        self.log_text = tk.Text(log_frame, height=6, font=("Consolas", 9),
                               bg=self.colors['bg_mid'], fg=self.colors['text'],
                               insertbackground=self.colors['text'],
                               relief=tk.FLAT, state=tk.DISABLED)
        self.log_text.pack(fill=tk.BOTH, expand=True)

        # === XBOX CONNECT BUTTON ===
        xbox_btn = tk.Button(main_frame, text="🎮 CONNECT XBOX CONTROLLER",
                            bg=self.colors['bg_light'], fg=self.colors['accent2'],
                            font=("Consolas", 11, "bold"),
                            activebackground=self.colors['accent2'],
                            activeforeground=self.colors['bg_dark'],
                            relief=tk.FLAT, cursor='hand2',
                            command=self._connect_xbox)
        xbox_btn.pack(fill=tk.X, pady=5)

        # Populate ports
        self._refresh_ports()

    def _create_led_indicator(self, parent, label, col):
        """Create an LED-style status indicator."""
        frame = tk.Frame(parent, bg=self.colors['bg_mid'])
        frame.pack(side=tk.LEFT, padx=20, pady=10)

        led = tk.Canvas(frame, width=12, height=12, bg=self.colors['bg_mid'], highlightthickness=0)
        led.create_oval(2, 2, 10, 10, fill='#333344', outline=self.colors['border'])
        led.pack(side=tk.LEFT, padx=5)

        tk.Label(frame, text=label, font=("Consolas", 9, "bold"),
                fg=self.colors['text_dim'], bg=self.colors['bg_mid']).pack(side=tk.LEFT)

        # Store LED references
        if label == "XBOX":
            self.xbox_led = led
        elif label == "R1":
            self.r1_led = led
        elif label == "R2":
            self.r2_led = led
        elif label == "J9":
            self.feeder_led = led

    def _create_connection_panel(self, parent, title, col):
        """Create a connection panel for a device."""
        panel = tk.Frame(parent, bg=self.colors['bg_light'], padx=10, pady=8)
        panel.grid(row=0, column=col, padx=5, sticky='ew')
        parent.columnconfigure(col, weight=1)

        # Title with LED
        title_frame = tk.Frame(panel, bg=self.colors['bg_light'])
        title_frame.pack(fill=tk.X)

        led = tk.Canvas(title_frame, width=10, height=10, bg=self.colors['bg_light'], highlightthickness=0)
        led.create_oval(1, 1, 9, 9, fill='#333344', outline='#444455')
        led.pack(side=tk.LEFT, padx=(0, 5))

        tk.Label(title_frame, text=title, font=("Consolas", 10, "bold"),
                fg=self.colors['accent2'], bg=self.colors['bg_light']).pack(side=tk.LEFT)

        # Port selection
        port_var = tk.StringVar()
        combo = ttk.Combobox(panel, textvariable=port_var, width=12, font=("Consolas", 9))
        combo.pack(fill=tk.X, pady=5)

        # Buttons
        btn_frame = tk.Frame(panel, bg=self.colors['bg_light'])
        btn_frame.pack(fill=tk.X)

        connect_btn = tk.Button(btn_frame, text="Connect", font=("Consolas", 8),
                               bg=self.colors['bg_mid'], fg=self.colors['text'],
                               relief=tk.FLAT, cursor='hand2')
        connect_btn.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=1)

        refresh_btn = tk.Button(btn_frame, text="⟳", font=("Consolas", 10),
                               bg=self.colors['bg_mid'], fg=self.colors['accent2'],
                               relief=tk.FLAT, cursor='hand2', width=3)
        refresh_btn.pack(side=tk.RIGHT, padx=1)

        return {'port_var': port_var, 'combo': combo, 'led': led,
                'button': connect_btn, 'refresh': refresh_btn}

    def _draw_speed_bar(self):
        """Draw the speed bar visualization."""
        self.speed_canvas.delete("all")
        speed = self.speed_var.get()
        width = int(290 * speed / 100)

        # Background
        self.speed_canvas.create_rectangle(5, 5, 295, 20, fill=self.colors['bg_dark'], outline='')

        # Gradient effect (multiple rectangles)
        if speed > 0:
            color = self.colors['success'] if speed < 70 else (self.colors['warning'] if speed < 90 else self.colors['accent'])
            self.speed_canvas.create_rectangle(5, 5, 5 + width, 20, fill=color, outline='')

    def _set_led(self, led_canvas, connected):
        """Set LED indicator state."""
        led_canvas.delete("all")
        color = self.colors['success'] if connected else '#333344'
        glow = self.colors['success'] if connected else self.colors['border']
        led_canvas.create_oval(2, 2, 10, 10, fill=color, outline=glow)

    def _log(self, message):
        """Add message to log."""
        self.log_text.configure(state=tk.NORMAL)
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)

    def _refresh_ports(self):
        """Refresh available serial ports."""
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.r1_port_combo['values'] = ports
        self.r2_port_combo['values'] = ports
        self.feeder_port_combo['values'] = ports

        # Auto-select if only one Teensy
        teensys = []
        arduinos = []
        for p in serial.tools.list_ports.comports():
            desc = p.description.lower() if p.description else ''
            if 'teensy' in desc:
                teensys.append(p.device)
            elif 'arduino' in desc:
                arduinos.append(p.device)

        if len(teensys) >= 1:
            self.r1_port_var.set(teensys[0])
        if len(teensys) >= 2:
            self.r2_port_var.set(teensys[1])
        if arduinos:
            self.feeder_port_var.set(arduinos[0])

    def _connect_xbox(self):
        """Connect Xbox controller."""
        if self.xbox.connect():
            self.xbox.start_polling()
            self._log("Xbox controller connected!")
            self._set_led(self.xbox_led, True)
        else:
            self._log("Xbox controller not found - check connection")
            self._set_led(self.xbox_led, False)

    def _connect_robot1(self):
        """Connect Robot 1."""
        port = self.r1_port_var.get()
        if not port:
            self._log("Robot 1: No port selected")
            return

        self.robot1.port = port
        if self.robot1.connect():
            self._log(f"Robot 1 connected on {port}")
        else:
            self._log(f"Robot 1 connection failed on {port}")

    def _connect_robot2(self):
        """Connect Robot 2."""
        port = self.r2_port_var.get()
        if not port:
            self._log("Robot 2: No port selected")
            return

        self.robot2.port = port
        if self.robot2.connect():
            self._log(f"Robot 2 connected on {port}")
        else:
            self._log(f"Robot 2 connection failed on {port}")

    def _connect_feeder(self):
        """Connect Tube Feeder."""
        port = self.feeder_port_var.get()
        if not port:
            # Try auto-detect
            self.feeder.port = None
        else:
            self.feeder.port = port

        if self.feeder.connect():
            self._log(f"Tube Feeder (J9) connected on {self.feeder.port}")
        else:
            self._log("Tube Feeder connection failed")

    def _on_speed_change(self, val):
        """Handle speed slider change."""
        self.speed = int(float(val))
        self.speed_var.set(self.speed)
        self.speed_label.config(text=f"{self.speed}%")
        self._draw_speed_bar()

    def _emergency_stop_all(self):
        """Emergency stop all devices."""
        self._log("!!! EMERGENCY STOP ALL !!!")
        self.action_label.config(text="EMERGENCY STOP", foreground="red")

        if self.robot1.connected:
            self.robot1.emergency_stop()
        if self.robot2.connected:
            self.robot2.emergency_stop()
        if self.feeder.connected:
            self.feeder.stop()

        self.current_jog = None

    def _cycle_device_mode(self):
        """Cycle through device modes (R1 -> R2 -> Both)."""
        self.device_mode = (self.device_mode + 1) % 3
        mode_icons = ["◉ ROBOT 1", "◉ ROBOT 2", "◉◉ BOTH ROBOTS"]
        self.mode_label.config(text=mode_icons[self.device_mode])
        self._log(f"Switched to: {self.MODE_NAMES[self.device_mode]}")

        # Stop any current jogging when switching
        self._stop_all_jog()

    def _set_move_mode(self, mode):
        """Set movement mode (joint or cartesian)."""
        self.move_mode = mode
        mode_name = "[ JOINT MODE ]" if mode == self.MOVE_JOINT else "[ CARTESIAN MODE ]"
        self.movement_mode_label.config(text=mode_name)
        self._log(f"Movement mode: {'Joint' if mode == self.MOVE_JOINT else 'Cartesian'}")

    def _stop_all_jog(self):
        """Stop all jogging motion."""
        if self.robot1.connected:
            self.robot1.stop_jog()
        if self.robot2.connected:
            self.robot2.stop_jog()
        if self.feeder.connected:
            self.feeder.stop()
        self.current_jog = None
        self.action_label.config(text="IDLE", fg=self.colors['text'])

    def _on_button_press(self, button):
        """Handle Xbox button press."""
        # Back = cycle device
        if button == XboxController.BTN_BACK:
            self._cycle_device_mode()

        # Start = emergency stop
        elif button == XboxController.BTN_START:
            self._emergency_stop_all()

        # A = joint mode
        elif button == XboxController.BTN_A:
            self._set_move_mode(self.MOVE_JOINT)

        # B = cartesian mode
        elif button == XboxController.BTN_B:
            self._set_move_mode(self.MOVE_CARTESIAN)

        # X = J7 negative (linear track)
        elif button == XboxController.BTN_X:
            self._jog_j7(-1)

        # Y = J7 positive (linear track)
        elif button == XboxController.BTN_Y:
            self._jog_j7(+1)

        # LB = J9 retract (tube feeder reverse)
        elif button == XboxController.BTN_LB:
            self._jog_j9(-1)

        # RB = J9 feed (tube feeder forward)
        elif button == XboxController.BTN_RB:
            self._jog_j9(+1)

        # D-pad
        elif button == XboxController.DPAD_UP:
            self._jog_dpad('up')
        elif button == XboxController.DPAD_DOWN:
            self._jog_dpad('down')
        elif button == XboxController.DPAD_LEFT:
            self._jog_dpad('left')
        elif button == XboxController.DPAD_RIGHT:
            self._jog_dpad('right')

    def _on_button_release(self, button):
        """Handle Xbox button release."""
        # Stop jogging when X/Y (J9), bumpers (J7), or d-pad released
        if button in [XboxController.BTN_X, XboxController.BTN_Y,
                      XboxController.BTN_LB, XboxController.BTN_RB,
                      XboxController.DPAD_UP, XboxController.DPAD_DOWN,
                      XboxController.DPAD_LEFT, XboxController.DPAD_RIGHT]:
            self._stop_all_jog()

    def _on_stick_move(self, left_stick, right_stick):
        """Handle stick movement."""
        lx, ly = left_stick
        rx, ry = right_stick

        # Apply deadzone
        dz = 0.25  # Increased deadzone for better control
        if abs(lx) < dz: lx = 0
        if abs(ly) < dz: ly = 0
        if abs(rx) < dz: rx = 0
        if abs(ry) < dz: ry = 0

        # Determine what to jog based on stick position
        new_jog = None

        # Prioritize the strongest input
        max_val = max(abs(lx), abs(ly), abs(rx), abs(ry))

        if max_val < dz:
            # No significant input - stop jogging
            if self.current_jog is not None and self.current_jog[0] in ('J', 'C'):
                # Only stop if it was a stick-initiated jog (not button)
                if self.current_jog[1] in (1, 2, 3, 4, 'X', 'Y', 'Z', 'Rz'):
                    self._stop_all_jog()
            return

        # Update action display to show stick input (for debugging)
        self.action_label.config(text=f"Stick: L({lx:.1f},{ly:.1f}) R({rx:.1f},{ry:.1f})",
                                 fg=self.colors['accent2'])

        if self.move_mode == self.MOVE_JOINT:
            # Joint mode: L=J1/J2, R=J3/J4
            if abs(lx) >= abs(ly) and abs(lx) >= abs(rx) and abs(lx) >= abs(ry):
                new_jog = ('J', 1, 1 if lx > 0 else -1)
            elif abs(ly) >= abs(lx) and abs(ly) >= abs(rx) and abs(ly) >= abs(ry):
                new_jog = ('J', 2, 1 if ly > 0 else -1)
            elif abs(rx) >= abs(lx) and abs(rx) >= abs(ly) and abs(rx) >= abs(ry):
                new_jog = ('J', 3, 1 if rx > 0 else -1)
            elif abs(ry) >= abs(lx) and abs(ry) >= abs(ly) and abs(ry) >= abs(rx):
                new_jog = ('J', 4, 1 if ry > 0 else -1)
        else:
            # Cartesian mode: L=X/Y, R=Z/Rz
            if abs(lx) >= abs(ly) and abs(lx) >= abs(rx) and abs(lx) >= abs(ry):
                new_jog = ('C', 'Y', 1 if lx > 0 else -1)
            elif abs(ly) >= abs(lx) and abs(ly) >= abs(rx) and abs(ly) >= abs(ry):
                new_jog = ('C', 'X', 1 if ly > 0 else -1)
            elif abs(rx) >= abs(lx) and abs(rx) >= abs(ly) and abs(rx) >= abs(ry):
                new_jog = ('C', 'Rz', 1 if rx > 0 else -1)
            elif abs(ry) >= abs(lx) and abs(ry) >= abs(ly) and abs(ry) >= abs(rx):
                new_jog = ('C', 'Z', 1 if ry > 0 else -1)

        # Only start new jog if different from current
        if new_jog != self.current_jog:
            if self.current_jog is not None:
                self._stop_all_jog()

            if new_jog is not None:
                self._start_jog(new_jog)

    def _on_trigger(self, left_trigger, right_trigger):
        """Handle trigger input for speed adjustment (rate-limited)."""
        # Rate limit: only change speed every 200ms
        now = time.time()
        if not hasattr(self, '_last_speed_change'):
            self._last_speed_change = 0

        if now - self._last_speed_change < 0.2:  # 200ms rate limit
            return

        # RT increases speed, LT decreases
        if right_trigger > 0.5:
            new_speed = min(100, self.speed + 1)  # Slower increment
            if new_speed != self.speed:
                self.speed = new_speed
                self.speed_var.set(new_speed)
                self.speed_label.config(text=f"{self.speed}%")
                self._draw_speed_bar()
                self._last_speed_change = now

        elif left_trigger > 0.5:
            new_speed = max(1, self.speed - 1)  # Slower decrement
            if new_speed != self.speed:
                self.speed = new_speed
                self.speed_var.set(new_speed)
                self.speed_label.config(text=f"{self.speed}%")
                self._draw_speed_bar()
                self._last_speed_change = now

    def _start_jog(self, jog_spec):
        """Start jogging based on spec."""
        self.current_jog = jog_spec
        jog_type, axis, direction = jog_spec

        robots = []
        if self.device_mode == self.MODE_ROBOT1:
            robots = [self.robot1]
        elif self.device_mode == self.MODE_ROBOT2:
            robots = [self.robot2]
        elif self.device_mode == self.MODE_BOTH:
            robots = [self.robot1, self.robot2]

        action_text = ""

        for robot in robots:
            if not robot.connected:
                continue

            if jog_type == 'J':
                # Joint jog
                robot.jog_joint(axis, direction, self.speed)
                dir_str = "+" if direction > 0 else "-"
                action_text = f"Jogging J{axis}{dir_str}"
            elif jog_type == 'C':
                # Cartesian jog
                robot.jog_cartesian(axis, direction, self.speed)
                dir_str = "+" if direction > 0 else "-"
                action_text = f"Jogging {axis}{dir_str}"

        if action_text:
            self.action_label.config(text=action_text, foreground="blue")

    def _jog_j7(self, direction):
        """Jog J7 (linear track) on selected robot(s)."""
        robots = []
        if self.device_mode == self.MODE_ROBOT1:
            robots = [self.robot1]
        elif self.device_mode == self.MODE_ROBOT2:
            robots = [self.robot2]
        elif self.device_mode == self.MODE_BOTH:
            robots = [self.robot1, self.robot2]

        for robot in robots:
            if robot.connected:
                robot.jog_j7(direction, self.speed)

        dir_str = "+" if direction > 0 else "-"
        self.action_label.config(text=f"Jogging J7{dir_str} (Track)", foreground="blue")
        self.current_jog = ('J', 7, direction)

    def _jog_j9(self, direction):
        """Jog J9 (tube feeder) - Arduino-controlled external axis."""
        if not self.feeder.connected:
            self._log("Tube Feeder (J9) not connected")
            return

        if direction > 0:
            self.feeder.jog_forward()
            self.action_label.config(text="J9+ (Tube Feed)", foreground="blue")
        else:
            self.feeder.jog_reverse()
            self.action_label.config(text="J9- (Tube Retract)", foreground="blue")
        self.current_jog = ('J', 9, direction)

    def _jog_dpad(self, direction):
        """Handle D-pad jogging (J5/J6 or Rx/Ry)."""
        robots = []
        if self.device_mode == self.MODE_ROBOT1:
            robots = [self.robot1]
        elif self.device_mode == self.MODE_ROBOT2:
            robots = [self.robot2]
        elif self.device_mode == self.MODE_BOTH:
            robots = [self.robot1, self.robot2]

        if self.move_mode == self.MOVE_JOINT:
            # D-pad = J5/J6
            if direction == 'up':
                for r in robots:
                    if r.connected: r.jog_joint(5, -1, self.speed)
                self.action_label.config(text="Jogging J5-", foreground="blue")
                self.current_jog = ('J', 5, -1)
            elif direction == 'down':
                for r in robots:
                    if r.connected: r.jog_joint(5, +1, self.speed)
                self.action_label.config(text="Jogging J5+", foreground="blue")
                self.current_jog = ('J', 5, +1)
            elif direction == 'left':
                for r in robots:
                    if r.connected: r.jog_joint(6, -1, self.speed)
                self.action_label.config(text="Jogging J6-", foreground="blue")
                self.current_jog = ('J', 6, -1)
            elif direction == 'right':
                for r in robots:
                    if r.connected: r.jog_joint(6, +1, self.speed)
                self.action_label.config(text="Jogging J6+", foreground="blue")
                self.current_jog = ('J', 6, +1)
        else:
            # Cartesian: D-pad = Rx/Ry
            if direction == 'up':
                for r in robots:
                    if r.connected: r.jog_cartesian('Rx', +1, self.speed)
                self.action_label.config(text="Jogging Rx+", foreground="blue")
                self.current_jog = ('C', 'Rx', +1)
            elif direction == 'down':
                for r in robots:
                    if r.connected: r.jog_cartesian('Rx', -1, self.speed)
                self.action_label.config(text="Jogging Rx-", foreground="blue")
                self.current_jog = ('C', 'Rx', -1)
            elif direction == 'left':
                for r in robots:
                    if r.connected: r.jog_cartesian('Ry', -1, self.speed)
                self.action_label.config(text="Jogging Ry-", foreground="blue")
                self.current_jog = ('C', 'Ry', -1)
            elif direction == 'right':
                for r in robots:
                    if r.connected: r.jog_cartesian('Ry', +1, self.speed)
                self.action_label.config(text="Jogging Ry+", foreground="blue")
                self.current_jog = ('C', 'Ry', +1)

    def _update_status(self):
        """Periodic status update with LED indicators."""
        # Update LED indicators
        self._set_led(self.xbox_led, self.xbox.connected)
        self._set_led(self.r1_led, self.robot1.connected)
        self._set_led(self.r1_status_led, self.robot1.connected)
        self._set_led(self.r2_led, self.robot2.connected)
        self._set_led(self.r2_status_led, self.robot2.connected)
        self._set_led(self.feeder_led, self.feeder.connected)
        self._set_led(self.feeder_status_led, self.feeder.connected)

        # Schedule next update
        self.root.after(500, self._update_status)

    def on_close(self):
        """Clean up on window close."""
        self._log("Shutting down...")
        self._save_geometry()  # Save window position/size
        self.xbox.stop_polling()
        self.robot1.disconnect()
        self.robot2.disconnect()
        self.feeder.disconnect()
        self.root.destroy()


# =============================================================================
# MAIN ENTRY POINT
# =============================================================================

def kill_previous_instances():
    """Kill any previous instances of xbox_toolbox.py"""
    import subprocess
    current_pid = os.getpid()
    try:
        # Find all xbox_toolbox processes
        result = subprocess.run(['pgrep', '-f', 'xbox_toolbox.py'],
                                capture_output=True, text=True)
        pids = result.stdout.strip().split('\n')
        for pid in pids:
            if pid and int(pid) != current_pid:
                try:
                    os.kill(int(pid), 9)  # SIGKILL
                except (ProcessLookupError, ValueError):
                    pass
    except Exception:
        pass


def main():
    # Kill any previous instances first
    kill_previous_instances()

    root = tk.Tk()
    app = XboxToolbox(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
