#!/usr/bin/env python3
"""
Xbox Controller Interface
=========================
Cross-platform Xbox controller support for Linux (inputs library) and Windows (XInput).
"""

import sys
import time
import threading
import ctypes


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

                        # Triggers (Xbox Series uses 0-1023, older uses 0-255)
                        elif event.code == 'ABS_Z':
                            self.left_trigger = min(1.0, event.state / 1023.0)
                        elif event.code == 'ABS_RZ':
                            self.right_trigger = min(1.0, event.state / 1023.0)

                        if self.on_stick_move:
                            self.on_stick_move(self.left_stick, self.right_stick)
                        if self.on_trigger:
                            self.on_trigger(self.left_trigger, self.right_trigger)

                except Exception as e:
                    time.sleep(0.1)

        except Exception as e:
            print(f"Linux gamepad error: {e}")
