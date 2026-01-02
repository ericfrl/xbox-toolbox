#!/usr/bin/env python3
"""
Robot Controller for AR4
========================
Serial communication controller for AR4 robot arm.
Handles connection, jogging, position feedback, and movement commands.
"""

import time
import threading
import serial
import serial.tools.list_ports


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
        # AR4 sends ALL data in one line:
        # A{j1}B{j2}C{j3}D{j4}E{j5}F{j6}G{X}H{Y}I{Z}J{Rz}K{Ry}L{Rx}M...P{j7}Q{j8}R{j9}
        try:
            if 'A' in line and 'B' in line:
                # Parse using index-based extraction like FRL.py
                j1_idx = line.find('A')
                j2_idx = line.find('B')
                j3_idx = line.find('C')
                j4_idx = line.find('D')
                j5_idx = line.find('E')
                j6_idx = line.find('F')
                x_idx = line.find('G')
                y_idx = line.find('H')
                z_idx = line.find('I')
                rz_idx = line.find('J')
                ry_idx = line.find('K')
                rx_idx = line.find('L')
                m_idx = line.find('M')
                p_idx = line.find('P')
                q_idx = line.find('Q')
                r_idx = line.find('R')

                # Parse joints (A-F)
                if j1_idx >= 0 and j2_idx > j1_idx:
                    self.joints[0] = float(line[j1_idx+1:j2_idx])
                if j2_idx >= 0 and j3_idx > j2_idx:
                    self.joints[1] = float(line[j2_idx+1:j3_idx])
                if j3_idx >= 0 and j4_idx > j3_idx:
                    self.joints[2] = float(line[j3_idx+1:j4_idx])
                if j4_idx >= 0 and j5_idx > j4_idx:
                    self.joints[3] = float(line[j4_idx+1:j5_idx])
                if j5_idx >= 0 and j6_idx > j5_idx:
                    self.joints[4] = float(line[j5_idx+1:j6_idx])
                if j6_idx >= 0 and x_idx > j6_idx:
                    self.joints[5] = float(line[j6_idx+1:x_idx])

                # Parse Cartesian (G=X, H=Y, I=Z, J=Rz, K=Ry, L=Rx)
                if x_idx >= 0 and y_idx > x_idx:
                    self.cartesian[0] = float(line[x_idx+1:y_idx])  # X
                if y_idx >= 0 and z_idx > y_idx:
                    self.cartesian[1] = float(line[y_idx+1:z_idx])  # Y
                if z_idx >= 0 and rz_idx > z_idx:
                    self.cartesian[2] = float(line[z_idx+1:rz_idx])  # Z
                if rz_idx >= 0 and ry_idx > rz_idx:
                    self.cartesian[5] = float(line[rz_idx+1:ry_idx])  # Rz
                if ry_idx >= 0 and rx_idx > ry_idx:
                    self.cartesian[4] = float(line[ry_idx+1:rx_idx])  # Ry
                if rx_idx >= 0 and m_idx > rx_idx:
                    self.cartesian[3] = float(line[rx_idx+1:m_idx])  # Rx

                # Parse J7 (P index)
                if p_idx >= 0 and q_idx > p_idx:
                    self.j7_pos = float(line[p_idx+1:q_idx])

        except Exception:
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

    def move_and_wait(self, command, timeout=60):
        """
        Send a move command and wait for Teensy to respond (move complete).

        Args:
            command: The MJ command string
            timeout: Max seconds to wait for move to complete

        Returns:
            (success, response) tuple
        """
        if not self.connected:
            return False, "Not connected"

        try:
            # Stop background reading temporarily
            self.reading = False
            time.sleep(0.05)

            # Clear any pending data
            self.serial.reset_input_buffer()

            # Send command
            cmd = command if command.endswith('\n') else command + '\n'
            self.serial.write(cmd.encode())
            self.serial.flush()

            # Save original timeout and set new one
            original_timeout = self.serial.timeout
            self.serial.timeout = timeout

            # Wait for response (blocks until move complete)
            response = self.serial.readline().decode('utf-8', errors='ignore').strip()

            # Restore original timeout
            self.serial.timeout = original_timeout

            # Restart background reading
            self.reading = True
            if self.read_thread is None or not self.read_thread.is_alive():
                self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
                self.read_thread.start()

            if not response:
                return False, "Timeout"
            elif response.startswith('E'):
                return False, response
            else:
                self.last_response = response
                return True, response

        except Exception as e:
            self.reading = True
            return False, str(e)

    def stop_jog(self):
        """Stop all jogging motion."""
        self.jogging = False
        return self.send("S")

    def jog_joint(self, joint, direction, speed=25, accel=10, decel=10):
        """
        Start live jogging a joint.
        joint: 1-6 for main joints, 7 for linear track
        direction: +1 or -1
        speed: 1-100 (will be scaled to robot's range: 1-25)
        """
        # Scale 1-100% to robot range 1-25
        robot_speed = max(1, int(speed * 25 / 100))
        # Live jog command format: LJ + joint code
        # Code: J1- = 10, J1+ = 11, J2- = 20, J2+ = 21, etc.
        code = joint * 10 + (1 if direction > 0 else 0)
        self.jogging = True
        return self.send(f"LJ{code}S{robot_speed}A{accel}D{decel}")

    def jog_cartesian(self, axis, direction, speed=25, accel=10, decel=10):
        """
        Start live jogging in Cartesian space.
        axis: 'X', 'Y', 'Z', 'Rx', 'Ry', 'Rz'
        direction: +1 or -1
        speed: 1-100 (will be scaled to robot's range: 1-25)
        """
        # Scale 1-100% to robot range 1-25
        robot_speed = max(1, int(speed * 25 / 100))
        axis_map = {'X': 1, 'Y': 2, 'Z': 3, 'Rx': 4, 'Ry': 5, 'Rz': 6}
        if axis not in axis_map:
            return False
        code = axis_map[axis] * 10 + (1 if direction > 0 else 0)
        self.jogging = True
        return self.send(f"LC{code}S{robot_speed}A{accel}D{decel}")

    def jog_j7(self, direction, speed=25, accel=10, decel=10):
        """Jog linear track (J7)."""
        # Scale 1-100% to robot range 1-25
        robot_speed = max(1, int(speed * 25 / 100))
        code = 70 + (1 if direction > 0 else 0)
        self.jogging = True
        return self.send(f"LJ{code}S{robot_speed}A{accel}D{decel}")

    def emergency_stop(self):
        """Emergency stop."""
        self.jogging = False
        return self.send("ES")

    def get_position(self):
        """Request current position."""
        return self.send("GP")
