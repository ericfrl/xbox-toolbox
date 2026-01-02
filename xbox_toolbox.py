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
import re
import serial
import serial.tools.list_ports
import tkinter as tk
from tkinter import ttk, messagebox

# Import modular components
from FRL_robot_controller import RobotController
from FRL_xbox_controller import XboxController

# Import from shared location
sys.path.insert(0, os.path.expanduser('~/.frl'))
from FRL_tube_feeder import TubeFeederController

# =============================================================================
# CONFIGURATION
# =============================================================================

DEFAULT_BAUDRATE = 115200
FEEDER_BAUDRATE = 115200

# Default ports - use udev symlinks when available
FRL_ROBOT1_PORT = "/dev/frl_robot1"
FRL_ROBOT2_PORT = "/dev/frl_robot2"
FRL_FEEDER_PORT = "/dev/frl_feeder"

# Jogging parameters
DEFAULT_SPEED = 25
DEFAULT_ACCEL = 10
DEFAULT_DECEL = 10


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

    # Operation modes (Train/Move)
    OP_MOVE = 0
    OP_TRAIN = 1
    OP_NAMES = ["Move", "Train"]

    # Config file for saving window geometry
    CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), '.xbox_toolbox_config.json')

    # Pathway storage directory
    PATHWAY_DIR = os.path.expanduser('~/.xbox_toolbox_pathways')

    def __init__(self, root):
        self.root = root
        # Set title based on theme (so user can tell which version)
        theme_mode = getattr(root, '_theme_mode', 'cyber')
        if theme_mode == 'darkly':
            self.root.title("Xbox Toolbox - Dashboard Mode")
        else:
            self.root.title("Xbox Toolbox - FRL Multi-Robot Control")
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
        self.smoothness = 50  # 1-100: higher = gentler accel/decel
        self.current_jog = None  # Track what's currently jogging

        # Train/Move mode state
        self.operation_mode = self.OP_MOVE  # Start in Move mode
        self.pathway = {'name': '', 'robot_mode': 'r1', 'waypoints': []}
        self.playback_active = False
        self.playback_loop = False
        self.playback_thread = None

        # Feeder has independent speed control (mm/sec)
        self.feeder_speed = 10  # Default 10 mm/sec

        # Ensure pathway directory exists
        os.makedirs(self.PATHWAY_DIR, exist_ok=True)

        # Build GUI
        self._build_gui()

        # Bind Xbox callbacks
        self.xbox.on_button_press = self._on_button_press
        self.xbox.on_button_release = self._on_button_release
        self.xbox.on_stick_move = self._on_stick_move
        self.xbox.on_trigger = self._on_trigger

        # Start status update loop
        self._update_status()

    def _load_config(self):
        """Load saved configuration from config file."""
        self._config = {}
        try:
            if os.path.exists(self.CONFIG_FILE):
                with open(self.CONFIG_FILE, 'r') as f:
                    self._config = json.load(f)
                    geometry = self._config.get('geometry', '600x800+100+100')
                    self.root.geometry(geometry)
            else:
                self.root.geometry("600x800+100+100")
        except Exception:
            self.root.geometry("600x800+100+100")

    def _save_config(self):
        """Save current configuration to config file."""
        try:
            self._config['geometry'] = self.root.geometry()
            # Save port selections
            self._config['robot1_port'] = self.r1_port_var.get()
            self._config['robot2_port'] = self.r2_port_var.get()
            self._config['feeder_port'] = self.feeder_port_var.get()
            # Save speed settings
            self._config['speed'] = self.speed
            self._config['smoothness'] = self.smoothness
            self._config['feeder_speed'] = self.feeder_speed
            with open(self.CONFIG_FILE, 'w') as f:
                json.dump(self._config, f, indent=2)
        except Exception:
            pass

    def _apply_saved_ports(self):
        """Apply saved port selections after GUI is built."""
        if hasattr(self, '_config'):
            # Restore port selections if they exist in available ports
            ports = [p.device for p in serial.tools.list_ports.comports()]

            saved_r1 = self._config.get('robot1_port', '')
            if saved_r1 and saved_r1 in ports:
                self.r1_port_var.set(saved_r1)

            saved_r2 = self._config.get('robot2_port', '')
            if saved_r2 and saved_r2 in ports:
                self.r2_port_var.set(saved_r2)

            saved_feeder = self._config.get('feeder_port', '')
            if saved_feeder and saved_feeder in ports:
                self.feeder_port_var.set(saved_feeder)

            # Restore speed
            saved_speed = self._config.get('speed', DEFAULT_SPEED)
            self.speed = saved_speed
            self.speed_var.set(saved_speed)
            self.speed_label.config(text=f"{saved_speed}%")
            self._draw_speed_bar()

            # Restore smoothness
            saved_smooth = self._config.get('smoothness', 50)
            self.smoothness = saved_smooth
            self.smooth_var.set(saved_smooth)
            self.smooth_label.config(text=f"{saved_smooth}%")
            self._draw_smooth_bar()

            # Restore feeder speed
            saved_feeder_speed = self._config.get('feeder_speed', 10)
            self.feeder_speed = saved_feeder_speed
            self.feeder_speed_var.set(saved_feeder_speed)
            self.feeder_speed_label.config(text=f"{saved_feeder_speed} mm/s")

    def _load_geometry(self):
        """Load saved window geometry from config file (legacy wrapper)."""
        self._load_config()

    def _save_geometry(self):
        """Save current window geometry to config file (legacy wrapper)."""
        self._save_config()

    def _build_gui(self):
        """Build the Tkinter GUI with theme support."""
        # Detect theme mode
        self.theme_mode = getattr(self.root, '_theme_mode', 'cyber')

        # === THEME COLORS ===
        if self.theme_mode == 'cyborg':
            # ttkbootstrap cyborg theme - black with vibrant multi-color accents
            self.colors = {
                'bg_dark': '#060606',
                'bg_mid': '#1a1a1a',
                'bg_light': '#2d2d2d',
                'accent': '#ff3366',    # hot pink for title
                'accent2': '#00ffcc',   # cyan/teal
                'accent3': '#bf5fff',   # purple/violet
                'accent4': '#ffff00',   # yellow
                'success': '#39ff14',   # neon green
                'warning': '#ff8800',   # orange
                'danger': '#ff2222',    # red for E-STOP
                'robot1': '#00aaff',    # blue for robot 1
                'robot2': '#ff66aa',    # pink for robot 2
                'feeder': '#ffcc00',    # gold for feeder
                'text': '#ffffff',
                'text_dim': '#888888',
                'border': '#404040'
            }
        elif self.theme_mode == 'darkly':
            # ttkbootstrap darkly theme colors
            self.colors = {
                'bg_dark': '#222222',
                'bg_mid': '#303030',
                'bg_light': '#444444',
                'accent': '#375a7f',    # darkly primary
                'accent2': '#00bc8c',   # darkly success
                'success': '#00bc8c',
                'warning': '#f39c12',
                'text': '#ffffff',
                'text_dim': '#adb5bd',
                'border': '#444444'
            }
        elif self.theme_mode == 'superhero':
            # ttkbootstrap superhero - dark blue/orange
            self.colors = {
                'bg_dark': '#2b3e50',
                'bg_mid': '#3a5068',
                'bg_light': '#4e6680',
                'accent': '#df691a',    # superhero primary (orange)
                'accent2': '#5cb85c',   # superhero success
                'success': '#5cb85c',
                'warning': '#f0ad4e',
                'text': '#ffffff',
                'text_dim': '#8a9dae',
                'border': '#4e6680'
            }
        elif self.theme_mode == 'vapor':
            # ttkbootstrap vapor - purple/pink
            self.colors = {
                'bg_dark': '#1a1a2e',
                'bg_mid': '#2d2d44',
                'bg_light': '#3f3f5a',
                'accent': '#ea39b8',    # vapor primary (pink)
                'accent2': '#3cf281',   # vapor success (neon green)
                'success': '#3cf281',
                'warning': '#ffe700',
                'text': '#ffffff',
                'text_dim': '#8a8aa3',
                'border': '#3f3f5a'
            }
        elif self.theme_mode == 'solar':
            # ttkbootstrap solar - dark blue/yellow
            self.colors = {
                'bg_dark': '#002b36',
                'bg_mid': '#073642',
                'bg_light': '#094959',
                'accent': '#b58900',    # solar primary (yellow)
                'accent2': '#2aa198',   # solar success (cyan)
                'success': '#2aa198',
                'warning': '#cb4b16',
                'text': '#ffffff',
                'text_dim': '#839496',
                'border': '#094959'
            }
        else:
            # Custom cyber theme (default) - black background with colorful accents
            self.colors = {
                'bg_dark': '#0a0a0a',
                'bg_mid': '#1a1a1a',
                'bg_light': '#2a2a2a',
                'accent': '#ff3366',       # hot pink
                'accent2': '#00ffcc',      # cyan
                'accent3': '#bf5fff',      # purple
                'success': '#39ff14',      # neon green
                'warning': '#ff8800',      # orange
                'danger': '#cc0000',       # red
                'robot1': '#00aaff',       # blue
                'robot2': '#ff66aa',       # pink
                'feeder': '#ffcc00',       # gold
                'text': '#ffffff',
                'text_dim': '#888899',
                'border': '#333333'
            }

        # Configure root window
        self.root.configure(bg=self.colors['bg_dark'])

        # Style configuration
        style = ttk.Style()
        if self.theme_mode != 'darkly':
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

        subtitle = tk.Label(header_frame, text="FRL Multi-Robot Control",
                           font=("Consolas", 10),
                           fg=self.colors['text_dim'], bg=self.colors['bg_dark'])
        subtitle.pack(side=tk.LEFT, padx=15, pady=8)

        # Help button
        help_btn = tk.Button(header_frame, text="?",
                            font=("Consolas", 14, "bold"),
                            bg=self.colors['bg_light'], fg=self.colors['accent2'],
                            activebackground=self.colors['accent2'],
                            activeforeground=self.colors['bg_dark'],
                            relief=tk.FLAT, cursor='hand2', width=3,
                            command=self._show_help)
        help_btn.pack(side=tk.RIGHT, padx=5)

        # Restart button
        restart_btn = tk.Button(header_frame, text="↻",
                               font=("Consolas", 14, "bold"),
                               bg=self.colors['bg_light'], fg=self.colors['warning'],
                               activebackground=self.colors['warning'],
                               activeforeground=self.colors['bg_dark'],
                               relief=tk.FLAT, cursor='hand2', width=3,
                               command=self._restart)
        restart_btn.pack(side=tk.RIGHT, padx=5)

        # === STATUS BAR (Top) ===
        status_bar = tk.Frame(main_frame, bg=self.colors['bg_mid'], height=50)
        status_bar.pack(fill=tk.X, pady=(0, 10))
        status_bar.pack_propagate(False)

        # LED indicators
        self._create_led_indicator(status_bar, "XBOX", 0)
        self._create_led_indicator(status_bar, "R1", 1)
        self._create_led_indicator(status_bar, "R2", 2)
        self._create_led_indicator(status_bar, "FEED", 3)

        # === ACTIVE MODE DISPLAY ===
        mode_display = tk.Frame(main_frame, bg=self.colors['bg_mid'], height=120)
        mode_display.pack(fill=tk.X, pady=5)
        mode_display.pack_propagate(False)

        # Top row: Robot mode and Train/Move toggle
        mode_top = tk.Frame(mode_display, bg=self.colors['bg_mid'])
        mode_top.pack(fill=tk.X, pady=5)

        self.mode_label = tk.Label(mode_top, text="◉ ROBOT 1",
                                   font=("Consolas", 22, "bold"),
                                   fg=self.colors.get('robot1', self.colors['accent2']), bg=self.colors['bg_mid'])
        self.mode_label.pack(side=tk.LEFT, padx=20)

        # Train/Move toggle button
        self.train_move_btn = tk.Button(mode_top, text="▶ MOVE",
                                        font=("Consolas", 14, "bold"),
                                        bg=self.colors['success'], fg=self.colors['bg_dark'],
                                        activebackground=self.colors['accent'],
                                        relief=tk.FLAT, cursor='hand2', width=10,
                                        command=self._toggle_operation_mode)
        self.train_move_btn.pack(side=tk.RIGHT, padx=20)

        self.movement_mode_label = tk.Label(mode_display, text="[ JOINT MODE ]",
                                            font=("Consolas", 12),
                                            fg=self.colors['success'], bg=self.colors['bg_mid'])
        self.movement_mode_label.pack()

        # === PATHWAY PANEL (for Train mode) ===
        self.pathway_frame = tk.Frame(main_frame, bg=self.colors['bg_light'])
        self.pathway_frame.pack(fill=tk.X, pady=5)

        # Pathway info row
        pathway_info = tk.Frame(self.pathway_frame, bg=self.colors['bg_light'])
        pathway_info.pack(fill=tk.X, padx=10, pady=5)

        tk.Label(pathway_info, text="PATHWAY", font=("Consolas", 10, "bold"),
                fg=self.colors['accent2'], bg=self.colors['bg_light']).pack(side=tk.LEFT)

        self.waypoint_count_label = tk.Label(pathway_info, text="Waypoints: 0",
                                             font=("Consolas", 10),
                                             fg=self.colors['text'], bg=self.colors['bg_light'])
        self.waypoint_count_label.pack(side=tk.LEFT, padx=20)

        # Playback indicator
        self.playback_label = tk.Label(pathway_info, text="",
                                       font=("Consolas", 10, "bold"),
                                       fg=self.colors['accent'], bg=self.colors['bg_light'])
        self.playback_label.pack(side=tk.RIGHT, padx=10)

        # Loop checkbox
        self.loop_var = tk.BooleanVar(value=False)
        loop_cb = tk.Checkbutton(pathway_info, text="Loop", variable=self.loop_var,
                                 font=("Consolas", 9), bg=self.colors['bg_light'],
                                 fg=self.colors['text'], selectcolor=self.colors['bg_mid'],
                                 activebackground=self.colors['bg_light'],
                                 command=self._toggle_loop)
        loop_cb.pack(side=tk.RIGHT, padx=5)

        # Pathway name and buttons row
        pathway_ctrl = tk.Frame(self.pathway_frame, bg=self.colors['bg_light'])
        pathway_ctrl.pack(fill=tk.X, padx=10, pady=5)

        tk.Label(pathway_ctrl, text="Name:", font=("Consolas", 9),
                fg=self.colors['text_dim'], bg=self.colors['bg_light']).pack(side=tk.LEFT)

        self.pathway_name_var = tk.StringVar(value="pathway_1")
        self.pathway_name_entry = tk.Entry(pathway_ctrl, textvariable=self.pathway_name_var,
                                           font=("Consolas", 10), width=15,
                                           bg=self.colors['bg_mid'], fg=self.colors['text'],
                                           insertbackground=self.colors['text'])
        self.pathway_name_entry.pack(side=tk.LEFT, padx=5)

        # Button style for pathway controls
        btn_cfg = {'font': ("Consolas", 9), 'bg': self.colors['bg_mid'],
                   'fg': self.colors['text'], 'relief': tk.FLAT, 'cursor': 'hand2', 'width': 6}

        self.save_btn = tk.Button(pathway_ctrl, text="Save", command=self._save_pathway, **btn_cfg)
        self.save_btn.pack(side=tk.LEFT, padx=2)

        self.load_btn = tk.Button(pathway_ctrl, text="Load", command=self._show_load_dialog, **btn_cfg)
        self.load_btn.pack(side=tk.LEFT, padx=2)

        self.clear_btn = tk.Button(pathway_ctrl, text="Clear", command=self._clear_pathway, **btn_cfg)
        self.clear_btn.pack(side=tk.LEFT, padx=2)

        # Play/Stop button
        self.play_btn = tk.Button(pathway_ctrl, text="▶ Play",
                                  font=("Consolas", 9, "bold"),
                                  bg=self.colors['success'], fg=self.colors['bg_dark'],
                                  relief=tk.FLAT, cursor='hand2', width=8,
                                  command=self._toggle_playback)
        self.play_btn.pack(side=tk.RIGHT, padx=2)

        # Initially hide pathway panel (shown only in Train mode)
        self.pathway_frame.pack_forget()

        # === CONNECTION PANELS (Grid) ===
        conn_frame = tk.Frame(main_frame, bg=self.colors['bg_dark'])
        conn_frame.pack(fill=tk.X, pady=10)

        # Robot 1 (blue)
        r1_color = self.colors.get('robot1', self.colors['accent2'])
        r1_panel = self._create_connection_panel(conn_frame, "ROBOT 1", 0, r1_color)
        self.r1_port_var = r1_panel['port_var']
        self.r1_port_combo = r1_panel['combo']
        self.r1_status_led = r1_panel['led']
        r1_panel['button'].configure(command=self._connect_robot1)
        r1_panel['refresh'].configure(command=self._refresh_ports)

        # Robot 2 (pink)
        r2_color = self.colors.get('robot2', self.colors['accent2'])
        r2_panel = self._create_connection_panel(conn_frame, "ROBOT 2", 1, r2_color)
        self.r2_port_var = r2_panel['port_var']
        self.r2_port_combo = r2_panel['combo']
        self.r2_status_led = r2_panel['led']
        r2_panel['button'].configure(command=self._connect_robot2)

        # Tube Feeder (gold)
        feeder_color = self.colors.get('feeder', self.colors['accent2'])
        j9_panel = self._create_connection_panel(conn_frame, "TUBE FEEDER", 2, feeder_color)
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

        # === SMOOTHNESS CONTROL ===
        smooth_frame = tk.Frame(main_frame, bg=self.colors['bg_dark'])
        smooth_frame.pack(fill=tk.X, pady=5)

        tk.Label(smooth_frame, text="SMOOTH", font=("Consolas", 10, "bold"),
                fg=self.colors['accent2'], bg=self.colors['bg_dark']).pack(side=tk.LEFT)

        self.smooth_var = tk.IntVar(value=50)
        self.smooth_canvas = tk.Canvas(smooth_frame, width=300, height=25,
                                       bg=self.colors['bg_mid'], highlightthickness=0)
        self.smooth_canvas.pack(side=tk.LEFT, padx=10)
        self._draw_smooth_bar()

        self.smooth_label = tk.Label(smooth_frame, text="50%",
                                     font=("Consolas", 14, "bold"),
                                     fg=self.colors['accent2'], bg=self.colors['bg_dark'])
        self.smooth_label.pack(side=tk.LEFT, padx=10)

        # Smoothness preset buttons
        for val, name in [(20, "Snappy"), (50, "Normal"), (80, "Gentle")]:
            tk.Button(smooth_frame, text=name, font=("Consolas", 8),
                     bg=self.colors['bg_mid'], fg=self.colors['text'],
                     relief=tk.FLAT, width=6, cursor='hand2',
                     command=lambda v=val: self._set_smoothness(v)).pack(side=tk.LEFT, padx=2)

        # === FEEDER SPEED CONTROL ===
        feeder_speed_frame = tk.Frame(main_frame, bg=self.colors['bg_dark'])
        feeder_speed_frame.pack(fill=tk.X, pady=5)

        tk.Label(feeder_speed_frame, text="FEEDER", font=("Consolas", 10, "bold"),
                fg=self.colors['warning'], bg=self.colors['bg_dark']).pack(side=tk.LEFT)

        # Decrease button
        tk.Button(feeder_speed_frame, text="-", font=("Consolas", 12, "bold"),
                 bg=self.colors['bg_light'], fg=self.colors['text'],
                 relief=tk.FLAT, width=2, cursor='hand2',
                 command=self._decrease_feeder_speed).pack(side=tk.LEFT, padx=(10, 2))

        self.feeder_speed_var = tk.IntVar(value=self.feeder_speed)
        self.feeder_speed_label = tk.Label(feeder_speed_frame, text=f"{self.feeder_speed} mm/s",
                                           font=("Consolas", 12, "bold"),
                                           fg=self.colors['warning'], bg=self.colors['bg_dark'],
                                           width=10)
        self.feeder_speed_label.pack(side=tk.LEFT, padx=5)

        # Increase button
        tk.Button(feeder_speed_frame, text="+", font=("Consolas", 12, "bold"),
                 bg=self.colors['bg_light'], fg=self.colors['text'],
                 relief=tk.FLAT, width=2, cursor='hand2',
                 command=self._increase_feeder_speed).pack(side=tk.LEFT, padx=2)

        # Preset buttons for common speeds
        tk.Label(feeder_speed_frame, text="  Presets:", font=("Consolas", 9),
                fg=self.colors['text_dim'], bg=self.colors['bg_dark']).pack(side=tk.LEFT, padx=(15, 5))
        for spd in [5, 10, 25, 50]:
            tk.Button(feeder_speed_frame, text=str(spd), font=("Consolas", 9),
                     bg=self.colors['bg_mid'], fg=self.colors['text'],
                     relief=tk.FLAT, width=3, cursor='hand2',
                     command=lambda s=spd: self._set_feeder_speed(s)).pack(side=tk.LEFT, padx=1)

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
                             bg='#cc0000', fg='white',
                             font=("Consolas", 14, "bold"),
                             activebackground='#ff0000', activeforeground='white',
                             relief=tk.FLAT, cursor='hand2', height=2,
                             command=self._emergency_stop_all)
        estop_btn.pack(fill=tk.X, pady=10)

        # === BUTTON MAPPING ===
        map_frame = tk.Frame(main_frame, bg=self.colors['bg_mid'])
        map_frame.pack(fill=tk.X, pady=5)

        # Labels for button mapping that will change based on mode
        self.mapping_labels = {}

        mapping_lines = [
            ("A", "Joint", self.colors['success']),
            ("B", "Cartsn", self.colors['success']),
            ("Back", "Robot", self.colors['warning']),
            ("Start", "E-STOP", self.colors['accent']),
            ("Sticks", "J1-4", self.colors['accent2']),
            ("D-pad", "J5/J6", self.colors['accent2']),
            ("X/Y", "Track", self.colors['success']),
            ("LB/RB", "Feeder", self.colors['warning']),
        ]

        for i, (btn, action, color) in enumerate(mapping_lines):
            tk.Label(map_frame, text=btn, font=("Consolas", 8, "bold"),
                    fg=color, bg=self.colors['bg_mid']).grid(row=0, column=i*2, padx=3, pady=5)
            lbl = tk.Label(map_frame, text=action, font=("Consolas", 8),
                    fg=self.colors['text_dim'], bg=self.colors['bg_mid'])
            lbl.grid(row=1, column=i*2, padx=3)
            # Store labels that change in Train mode
            if btn in ["A", "B", "Start"]:
                self.mapping_labels[btn] = lbl

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
                            bg=self.colors['bg_light'], fg=self.colors['success'],
                            font=("Consolas", 11, "bold"),
                            activebackground=self.colors['success'],
                            activeforeground=self.colors['bg_dark'],
                            relief=tk.FLAT, cursor='hand2',
                            command=self._connect_xbox)
        xbox_btn.pack(fill=tk.X, pady=5)

        # Populate ports and restore saved selections
        self._refresh_ports()
        self._apply_saved_ports()

        # Auto-connect to FRL devices on startup
        self.root.after(500, self._auto_connect_frl)

    def _auto_connect_frl(self):
        """Auto-connect to FRL udev symlinks and Xbox controller on startup."""
        connected = []

        # Xbox controller
        if self.xbox.connect():
            self.xbox.start_polling()
            connected.append("Xbox")

        # Robot 1
        if os.path.exists(FRL_ROBOT1_PORT):
            self.r1_port_var.set(FRL_ROBOT1_PORT)
            self.robot1.port = FRL_ROBOT1_PORT
            if self.robot1.connect():
                connected.append("Robot 1")

        # Robot 2
        if os.path.exists(FRL_ROBOT2_PORT):
            self.r2_port_var.set(FRL_ROBOT2_PORT)
            self.robot2.port = FRL_ROBOT2_PORT
            if self.robot2.connect():
                connected.append("Robot 2")

        # Feeder
        if os.path.exists(FRL_FEEDER_PORT):
            self.feeder_port_var.set(FRL_FEEDER_PORT)
            self.feeder.port = FRL_FEEDER_PORT
            if self.feeder.connect():
                connected.append("Feeder")

        if connected:
            self._log(f"Auto-connected: {', '.join(connected)}")
        else:
            self._log("No devices found - connect manually")

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
        elif label == "FEED":
            self.feeder_led = led

    def _create_connection_panel(self, parent, title, col, title_color=None):
        """Create a connection panel for a device."""
        panel = tk.Frame(parent, bg=self.colors['bg_light'], padx=10, pady=8)
        panel.grid(row=0, column=col, padx=5, sticky='ew')
        parent.columnconfigure(col, weight=1)

        # Use device-specific color or fallback
        if title_color is None:
            title_color = self.colors['accent2']

        # Title with LED
        title_frame = tk.Frame(panel, bg=self.colors['bg_light'])
        title_frame.pack(fill=tk.X)

        led = tk.Canvas(title_frame, width=10, height=10, bg=self.colors['bg_light'], highlightthickness=0)
        led.create_oval(1, 1, 9, 9, fill='#333344', outline='#444455')
        led.pack(side=tk.LEFT, padx=(0, 5))

        tk.Label(title_frame, text=title, font=("Consolas", 10, "bold"),
                fg=title_color, bg=self.colors['bg_light']).pack(side=tk.LEFT)

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

    def _draw_smooth_bar(self):
        """Draw the smoothness bar visualization."""
        self.smooth_canvas.delete("all")
        smooth = self.smooth_var.get()
        width = int(290 * smooth / 100)

        # Background
        self.smooth_canvas.create_rectangle(5, 5, 295, 20, fill=self.colors['bg_dark'], outline='')

        # Bar color based on smoothness level
        if smooth > 0:
            color = self.colors['accent2']
            self.smooth_canvas.create_rectangle(5, 5, 5 + width, 20, fill=color, outline='')

    def _set_smoothness(self, value):
        """Set smoothness value (1-100)."""
        self.smoothness = max(1, min(100, value))
        self.smooth_var.set(self.smoothness)
        self.smooth_label.config(text=f"{self.smoothness}%")
        self._draw_smooth_bar()

    def _get_accel_decel(self):
        """Convert smoothness to accel/decel values for robot (1-25 range)."""
        # High smoothness = low accel/decel = gentle motion
        # smoothness 100 → accel 1 (very gentle)
        # smoothness 50 → accel 13 (medium)
        # smoothness 1 → accel 25 (snappy)
        accel = max(1, int(26 - (self.smoothness * 25 / 100)))
        return accel, accel  # Same value for accel and decel

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
        """Refresh available serial ports, prioritizing FRL udev symlinks."""
        # Get standard ports
        ports = [p.device for p in serial.tools.list_ports.comports()]

        # Check for FRL udev symlinks and add them to the front
        frl_ports = []
        for frl_port in [FRL_ROBOT1_PORT, FRL_ROBOT2_PORT, FRL_FEEDER_PORT]:
            if os.path.exists(frl_port):
                frl_ports.append(frl_port)

        # Combine: FRL symlinks first, then standard ports
        all_ports = frl_ports + [p for p in ports if p not in frl_ports]

        self.r1_port_combo['values'] = all_ports
        self.r2_port_combo['values'] = all_ports
        self.feeder_port_combo['values'] = all_ports

        # Auto-select FRL symlinks if they exist
        if os.path.exists(FRL_ROBOT1_PORT):
            self.r1_port_var.set(FRL_ROBOT1_PORT)
        if os.path.exists(FRL_ROBOT2_PORT):
            self.r2_port_var.set(FRL_ROBOT2_PORT)
        if os.path.exists(FRL_FEEDER_PORT):
            self.feeder_port_var.set(FRL_FEEDER_PORT)

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

    def _set_feeder_speed(self, speed):
        """Set the feeder speed."""
        self.feeder_speed = max(1, min(100, speed))
        self.feeder_speed_var.set(self.feeder_speed)
        self.feeder_speed_label.config(text=f"{self.feeder_speed} mm/s")
        self._log(f"Feeder speed: {self.feeder_speed} mm/s")

    def _increase_feeder_speed(self):
        """Increase feeder speed by 5 mm/s."""
        self._set_feeder_speed(self.feeder_speed + 5)

    def _decrease_feeder_speed(self):
        """Decrease feeder speed by 5 mm/s."""
        self._set_feeder_speed(self.feeder_speed - 5)

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
        mode_colors = [
            self.colors.get('robot1', self.colors['accent2']),
            self.colors.get('robot2', self.colors['accent2']),
            self.colors.get('accent3', self.colors['accent2'])  # purple for both
        ]
        self.mode_label.config(text=mode_icons[self.device_mode], fg=mode_colors[self.device_mode])
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
        # Back = cycle device (always available)
        if button == XboxController.BTN_BACK:
            self._cycle_device_mode()

        # Start behavior depends on mode
        elif button == XboxController.BTN_START:
            if self.operation_mode == self.OP_TRAIN:
                # In Train mode: toggle playback
                self._toggle_playback()
            else:
                # In Move mode: emergency stop
                self._emergency_stop_all()

        # A button behavior depends on mode
        elif button == XboxController.BTN_A:
            if self.operation_mode == self.OP_TRAIN:
                # In Train mode: add waypoint
                self._capture_waypoint()
            else:
                # In Move mode: switch to joint mode
                self._set_move_mode(self.MOVE_JOINT)

        # B button behavior depends on mode
        elif button == XboxController.BTN_B:
            if self.operation_mode == self.OP_TRAIN:
                # In Train mode: delete last waypoint
                self._delete_last_waypoint()
            else:
                # In Move mode: switch to cartesian mode
                self._set_move_mode(self.MOVE_CARTESIAN)

        # X = J7 negative (linear track) - always available for positioning
        elif button == XboxController.BTN_X:
            self._jog_j7(-1)

        # Y = J7 positive (linear track) - always available for positioning
        elif button == XboxController.BTN_Y:
            self._jog_j7(+1)

        # LB = J9 retract (tube feeder reverse) - always available
        elif button == XboxController.BTN_LB:
            self._jog_j9(-1)

        # RB = J9 feed (tube feeder forward) - always available
        elif button == XboxController.BTN_RB:
            self._jog_j9(+1)

        # D-pad - always available for positioning
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
            # No significant input - ALWAYS stop jogging for safety
            if self.current_jog is not None:
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

        # Start new jog or resend if direction changed or enough time passed
        now = time.time()
        if not hasattr(self, '_last_jog_send'):
            self._last_jog_send = 0

        if new_jog != self.current_jog:
            # Direction changed - stop and start new jog
            if self.current_jog is not None:
                self._stop_all_jog()
            if new_jog is not None:
                self._start_jog(new_jog)
                self._last_jog_send = now
        elif new_jog is not None and (now - self._last_jog_send) > 0.5:
            # Same direction but resend periodically to keep robot moving
            self._start_jog(new_jog)
            self._last_jog_send = now

    def _on_trigger(self, left_trigger, right_trigger):
        """Handle trigger input for speed adjustment - increment/decrement with persistence."""
        # Initialize state
        if not hasattr(self, '_trigger_cooldown'):
            self._trigger_cooldown = 0

        now = time.time()
        if now - self._trigger_cooldown < 0.15:  # 150ms between changes
            return

        speed_changed = False

        # RT increases speed when pressed past 20%
        if right_trigger > 0.2:
            new_speed = min(100, self.speed + 5)
            if new_speed != self.speed:
                self.speed = new_speed
                speed_changed = True
                self._trigger_cooldown = now

        # LT decreases speed when pressed past 20%
        if left_trigger > 0.2 and not speed_changed:
            new_speed = max(1, self.speed - 5)
            if new_speed != self.speed:
                self.speed = new_speed
                speed_changed = True
                self._trigger_cooldown = now

        if speed_changed:
            self.speed_var.set(self.speed)
            self.speed_label.config(text=f"{self.speed}%")
            self._draw_speed_bar()
            # If currently jogging, resend with new speed
            if self.current_jog is not None:
                self._start_jog(self.current_jog)

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

        accel, decel = self._get_accel_decel()

        for robot in robots:
            if not robot.connected:
                continue

            if jog_type == 'J':
                # Joint jog
                robot.jog_joint(axis, direction, self.speed, accel, decel)
                dir_str = "+" if direction > 0 else "-"
                action_text = f"Jogging J{axis}{dir_str}"
            elif jog_type == 'C':
                # Cartesian jog
                robot.jog_cartesian(axis, direction, self.speed, accel, decel)
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

        accel, decel = self._get_accel_decel()
        for robot in robots:
            if robot.connected:
                robot.jog_j7(direction, self.speed, accel, decel)

        dir_str = "+" if direction > 0 else "-"
        self.action_label.config(text=f"Jogging J7{dir_str} (Track)", foreground="blue")
        self.current_jog = ('J', 7, direction)

    def _jog_j9(self, direction):
        """Jog J9 (tube feeder) - Arduino-controlled external axis."""
        if not self.feeder.connected:
            self._log("Tube Feeder not connected")
            return

        # Set feeder speed before jogging
        self.feeder.set_speed(self.feeder_speed)

        if direction > 0:
            self.feeder.jog_forward()
            self.action_label.config(text=f"Tube Feed + ({self.feeder_speed}mm/s)", foreground="blue")
        else:
            self.feeder.jog_reverse()
            self.action_label.config(text=f"Tube Retract - ({self.feeder_speed}mm/s)", foreground="blue")
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

        accel, decel = self._get_accel_decel()

        if self.move_mode == self.MOVE_JOINT:
            # D-pad = J5/J6
            if direction == 'up':
                for r in robots:
                    if r.connected: r.jog_joint(5, -1, self.speed, accel, decel)
                self.action_label.config(text="Jogging J5-", foreground="blue")
                self.current_jog = ('J', 5, -1)
            elif direction == 'down':
                for r in robots:
                    if r.connected: r.jog_joint(5, +1, self.speed, accel, decel)
                self.action_label.config(text="Jogging J5+", foreground="blue")
                self.current_jog = ('J', 5, +1)
            elif direction == 'left':
                for r in robots:
                    if r.connected: r.jog_joint(6, -1, self.speed, accel, decel)
                self.action_label.config(text="Jogging J6-", foreground="blue")
                self.current_jog = ('J', 6, -1)
            elif direction == 'right':
                for r in robots:
                    if r.connected: r.jog_joint(6, +1, self.speed, accel, decel)
                self.action_label.config(text="Jogging J6+", foreground="blue")
                self.current_jog = ('J', 6, +1)
        else:
            # Cartesian: D-pad = Rx/Ry
            if direction == 'up':
                for r in robots:
                    if r.connected: r.jog_cartesian('Rx', +1, self.speed, accel, decel)
                self.action_label.config(text="Jogging Rx+", foreground="blue")
                self.current_jog = ('C', 'Rx', +1)
            elif direction == 'down':
                for r in robots:
                    if r.connected: r.jog_cartesian('Rx', -1, self.speed, accel, decel)
                self.action_label.config(text="Jogging Rx-", foreground="blue")
                self.current_jog = ('C', 'Rx', -1)
            elif direction == 'left':
                for r in robots:
                    if r.connected: r.jog_cartesian('Ry', -1, self.speed, accel, decel)
                self.action_label.config(text="Jogging Ry-", foreground="blue")
                self.current_jog = ('C', 'Ry', -1)
            elif direction == 'right':
                for r in robots:
                    if r.connected: r.jog_cartesian('Ry', +1, self.speed, accel, decel)
                self.action_label.config(text="Jogging Ry+", foreground="blue")
                self.current_jog = ('C', 'Ry', +1)

    # =========================================================================
    # HELP DIALOG
    # =========================================================================

    def _show_help(self):
        """Show help dialog with button mappings and usage info."""
        help_win = tk.Toplevel(self.root)
        help_win.title("Xbox Toolbox Help")
        help_win.geometry("500x600")
        help_win.configure(bg=self.colors['bg_dark'])
        help_win.transient(self.root)

        # Scrollable text area
        text_frame = tk.Frame(help_win, bg=self.colors['bg_dark'])
        text_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        scrollbar = tk.Scrollbar(text_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        help_text = tk.Text(text_frame, font=("Consolas", 10),
                           bg=self.colors['bg_mid'], fg=self.colors['text'],
                           wrap=tk.WORD, yscrollcommand=scrollbar.set)
        help_text.pack(fill=tk.BOTH, expand=True)
        scrollbar.config(command=help_text.yview)

        # Help content
        content = """
═══════════════════════════════════════════════
           XBOX TOOLBOX - HELP
═══════════════════════════════════════════════

OPERATION MODES
───────────────────────────────────────────────
• MOVE MODE (Default): Manual robot jogging
• TRAIN MODE: Record waypoints for playback

Click the MOVE/TRAIN button to toggle modes.

═══════════════════════════════════════════════
MOVE MODE - BUTTON MAPPING
═══════════════════════════════════════════════

  A          → Joint Mode
  B          → Cartesian Mode
  Back       → Cycle Robot: R1 → R2 → Both
  Start      → EMERGENCY STOP ALL

  Left Stick  → J1/J2 (Joint) or X/Y (Cartesian)
  Right Stick → J3/J4 (Joint) or Z/Rz (Cartesian)
  D-pad       → J5/J6 (Joint) or Rx/Ry (Cartesian)

  X          → J7 Track Negative
  Y          → J7 Track Positive
  LB         → Tube Feeder Retract
  RB         → Tube Feeder Feed

  LT         → Decrease Speed
  RT         → Increase Speed

═══════════════════════════════════════════════
TRAIN MODE - BUTTON MAPPING
═══════════════════════════════════════════════

  A          → Add Waypoint (capture position)
  B          → Delete Last Waypoint
  Start      → Toggle Playback

  All other buttons work the same as Move mode
  for positioning the robot.

═══════════════════════════════════════════════
PATHWAY PANEL (Train Mode)
═══════════════════════════════════════════════

When in Train mode, a pathway panel appears:

  • Waypoint Counter: Shows recorded waypoints
  • Name Field: Enter pathway name
  • Save: Save to ~/.xbox_toolbox_pathways/
  • Load: Load a saved pathway
  • Clear: Delete all waypoints
  • Loop: Toggle continuous playback
  • Play/Stop: Execute or stop pathway

═══════════════════════════════════════════════
QUICK START
═══════════════════════════════════════════════

1. Click "Connect Xbox Controller"
2. Select ports and connect robots/feeder
3. Use sticks and buttons to jog

For pathway recording:
1. Click MOVE to switch to TRAIN mode
2. Position robot, press A to add waypoint
3. Repeat to build pathway
4. Enter name, click Save
5. Click Play or press Start to execute

═══════════════════════════════════════════════
TROUBLESHOOTING
═══════════════════════════════════════════════

Xbox not detected:
  • Ensure controller is connected via USB/BT
  • Click "Connect Xbox Controller" button

Serial ports not found:
  • Check USB connections
  • Run: sudo usermod -a -G dialout $USER
  • Log out and back in

Sticks not responding:
  • Deadzone is 25% - push sticks further
  • Check action display for values

═══════════════════════════════════════════════
"""
        help_text.insert('1.0', content)
        help_text.config(state=tk.DISABLED)

        # Close button
        tk.Button(help_win, text="Close", font=("Consolas", 11),
                 bg=self.colors['accent2'], fg=self.colors['bg_dark'],
                 command=help_win.destroy, width=10).pack(pady=10)

    # =========================================================================
    # TRAIN/MOVE MODE FUNCTIONS
    # =========================================================================

    def _toggle_operation_mode(self):
        """Toggle between Train and Move modes."""
        if self.playback_active:
            self._stop_playback()

        self.operation_mode = self.OP_TRAIN if self.operation_mode == self.OP_MOVE else self.OP_MOVE

        if self.operation_mode == self.OP_TRAIN:
            # Entering Train mode
            self.train_move_btn.config(text="● TRAIN", bg=self.colors['accent'])
            self.pathway_frame.pack(fill=tk.X, pady=5, after=self.train_move_btn.master.master)
            # Update pathway robot mode to match current device mode
            mode_map = {self.MODE_ROBOT1: 'r1', self.MODE_ROBOT2: 'r2', self.MODE_BOTH: 'both'}
            self.pathway['robot_mode'] = mode_map[self.device_mode]
            # Update button mapping labels for Train mode
            self.mapping_labels['A'].config(text="AddWP")
            self.mapping_labels['B'].config(text="DelWP")
            self.mapping_labels['Start'].config(text="Play")
            self._log("Entered TRAIN mode - A=Add waypoint, B=Delete last")
        else:
            # Entering Move mode
            self.train_move_btn.config(text="▶ MOVE", bg=self.colors['success'])
            self.pathway_frame.pack_forget()
            # Restore button mapping labels for Move mode
            self.mapping_labels['A'].config(text="Joint")
            self.mapping_labels['B'].config(text="Cartsn")
            self.mapping_labels['Start'].config(text="E-STOP")
            self._log("Entered MOVE mode")

    def _toggle_loop(self):
        """Toggle loop playback mode."""
        self.playback_loop = self.loop_var.get()

    def _capture_waypoint(self):
        """Capture current robot position as a waypoint."""
        waypoint = {'r1': None, 'r2': None, 'feeder': 0.0}

        # Request fresh position data from robot(s)
        if self.device_mode in [self.MODE_ROBOT1, self.MODE_BOTH]:
            if self.robot1.connected:
                self.robot1.get_position()
                time.sleep(0.1)  # Brief wait for response

        if self.device_mode in [self.MODE_ROBOT2, self.MODE_BOTH]:
            if self.robot2.connected:
                self.robot2.get_position()
                time.sleep(0.1)

        # Request feeder position
        if self.feeder.connected:
            self.feeder.get_position()
            time.sleep(0.05)

        # Capture from robot(s) based on current device mode
        if self.device_mode in [self.MODE_ROBOT1, self.MODE_BOTH]:
            if self.robot1.connected:
                waypoint['r1'] = {
                    'joints': list(self.robot1.joints),
                    'cartesian': list(self.robot1.cartesian),
                    'j7': self.robot1.j7_pos
                }

        if self.device_mode in [self.MODE_ROBOT2, self.MODE_BOTH]:
            if self.robot2.connected:
                waypoint['r2'] = {
                    'joints': list(self.robot2.joints),
                    'cartesian': list(self.robot2.cartesian),
                    'j7': self.robot2.j7_pos
                }

        # Capture feeder position
        if self.feeder.connected:
            waypoint['feeder'] = self.feeder.position

        self.pathway['waypoints'].append(waypoint)
        count = len(self.pathway['waypoints'])
        self.waypoint_count_label.config(text=f"Waypoints: {count}")

        # Debug: show what was captured
        if waypoint.get('r1'):
            j = waypoint['r1']['joints']
            c = waypoint['r1']['cartesian']
            self._log(f"WP#{count} R1 joints: [{j[0]:.1f},{j[1]:.1f},{j[2]:.1f},{j[3]:.1f},{j[4]:.1f},{j[5]:.1f}]")
            self._log(f"WP#{count} R1 cart: X={c[0]:.1f} Y={c[1]:.1f} Z={c[2]:.1f}")
        if waypoint.get('r2'):
            j = waypoint['r2']['joints']
            c = waypoint['r2']['cartesian']
            self._log(f"WP#{count} R2 joints: [{j[0]:.1f},{j[1]:.1f},{j[2]:.1f},{j[3]:.1f},{j[4]:.1f},{j[5]:.1f}]")
            self._log(f"WP#{count} R2 cart: X={c[0]:.1f} Y={c[1]:.1f} Z={c[2]:.1f}")

        self.action_label.config(text=f"Waypoint #{count} added", fg=self.colors['success'])

    def _delete_last_waypoint(self):
        """Delete the last waypoint from the pathway."""
        if self.pathway['waypoints']:
            self.pathway['waypoints'].pop()
            count = len(self.pathway['waypoints'])
            self.waypoint_count_label.config(text=f"Waypoints: {count}")
            self._log(f"Deleted last waypoint (now {count})")
            self.action_label.config(text=f"Waypoint deleted ({count} left)", fg=self.colors['warning'])
        else:
            self._log("No waypoints to delete")

    def _clear_pathway(self):
        """Clear all waypoints from the current pathway."""
        self.pathway['waypoints'] = []
        self.waypoint_count_label.config(text="Waypoints: 0")
        self._log("Pathway cleared")
        self.action_label.config(text="Pathway cleared", fg=self.colors['text'])

    def _save_pathway(self):
        """Save the current pathway to a file."""
        name = self.pathway_name_var.get().strip()
        if not name:
            self._log("Error: Pathway name required")
            return

        if not self.pathway['waypoints']:
            self._log("Error: No waypoints to save")
            return

        # Update pathway metadata
        self.pathway['name'] = name
        mode_map = {self.MODE_ROBOT1: 'r1', self.MODE_ROBOT2: 'r2', self.MODE_BOTH: 'both'}
        self.pathway['robot_mode'] = mode_map[self.device_mode]
        self.pathway['created'] = time.strftime('%Y-%m-%dT%H:%M:%S')

        # Save to file
        filepath = os.path.join(self.PATHWAY_DIR, f"{name}.json")
        try:
            with open(filepath, 'w') as f:
                json.dump(self.pathway, f, indent=2)
            self._log(f"Pathway saved: {name} ({len(self.pathway['waypoints'])} waypoints)")
            self.action_label.config(text=f"Saved: {name}", fg=self.colors['success'])
        except Exception as e:
            self._log(f"Save failed: {e}")

    def _show_load_dialog(self):
        """Show a dialog to select and load a pathway."""
        # List available pathways
        try:
            files = [f[:-5] for f in os.listdir(self.PATHWAY_DIR) if f.endswith('.json')]
        except:
            files = []

        if not files:
            self._log("No saved pathways found")
            return

        # Simple selection dialog
        dialog = tk.Toplevel(self.root)
        dialog.title("Load Pathway")
        dialog.geometry("300x200")
        dialog.configure(bg=self.colors['bg_dark'])
        dialog.transient(self.root)
        dialog.grab_set()

        tk.Label(dialog, text="Select Pathway:", font=("Consolas", 11),
                bg=self.colors['bg_dark'], fg=self.colors['text']).pack(pady=10)

        listbox = tk.Listbox(dialog, font=("Consolas", 10),
                            bg=self.colors['bg_mid'], fg=self.colors['text'],
                            selectbackground=self.colors['accent2'])
        listbox.pack(fill=tk.BOTH, expand=True, padx=10)

        for name in sorted(files):
            listbox.insert(tk.END, name)

        def load_selected():
            selection = listbox.curselection()
            if selection:
                name = listbox.get(selection[0])
                self._load_pathway(name)
                dialog.destroy()

        tk.Button(dialog, text="Load", font=("Consolas", 10),
                 bg=self.colors['success'], fg=self.colors['bg_dark'],
                 command=load_selected).pack(pady=10)

    def _load_pathway(self, name):
        """Load a pathway from file."""
        filepath = os.path.join(self.PATHWAY_DIR, f"{name}.json")
        try:
            with open(filepath, 'r') as f:
                self.pathway = json.load(f)
            self.pathway_name_var.set(name)
            count = len(self.pathway['waypoints'])
            self.waypoint_count_label.config(text=f"Waypoints: {count}")
            self._log(f"Loaded pathway: {name} ({count} waypoints)")
            self.action_label.config(text=f"Loaded: {name}", fg=self.colors['success'])
        except Exception as e:
            self._log(f"Load failed: {e}")

    def _toggle_playback(self):
        """Toggle pathway playback."""
        if self.playback_active:
            self._stop_playback()
        else:
            self._start_playback()

    def _start_playback(self):
        """Start playing back the pathway."""
        if not self.pathway['waypoints']:
            self._log("No waypoints to play")
            return

        self.playback_active = True
        self.play_btn.config(text="■ Stop", bg=self.colors['accent'])
        self.playback_label.config(text="▶ PLAYING")
        self._log("Playback started")

        # Start playback in background thread
        self.playback_thread = threading.Thread(target=self._playback_loop, daemon=True)
        self.playback_thread.start()

    def _stop_playback(self):
        """Stop pathway playback."""
        self.playback_active = False
        self.play_btn.config(text="▶ Play", bg=self.colors['success'])
        self.playback_label.config(text="")
        self._stop_all_jog()
        self._log("Playback stopped")

    def _playback_loop(self):
        """Background thread for pathway playback."""
        while self.playback_active:
            for i, waypoint in enumerate(self.pathway['waypoints']):
                if not self.playback_active:
                    return

                # Update UI on main thread
                self.root.after(0, lambda idx=i: self.playback_label.config(
                    text=f"▶ {idx+1}/{len(self.pathway['waypoints'])}"))

                # Move robot(s) to waypoint and wait for completion
                success = self._move_to_waypoint(waypoint)

                if not success:
                    self.root.after(0, lambda: self._log("Move failed - stopping playback"))
                    self.root.after(0, self._stop_playback)
                    return

                if not self.playback_active:
                    return

            # Check if we should loop
            if not self.playback_loop:
                self.root.after(0, self._stop_playback)
                return

    def _move_to_waypoint(self, waypoint):
        """
        Move robot(s) to a waypoint position and wait for completion.

        Returns:
            True if all moves completed successfully, False otherwise
        """
        # Scale speed 1-100% to robot range 1-25
        robot_speed = max(1, int(self.speed * 25 / 100))
        accel, decel = self._get_accel_decel()

        success = True

        # Move Robot 1 and wait for completion
        if waypoint.get('r1') and self.robot1.connected:
            r1_data = waypoint['r1']

            # Use Cartesian if available, otherwise fall back to joints
            if 'cartesian' in r1_data and any(r1_data['cartesian']):
                cart = r1_data['cartesian']
                # MJ command with Cartesian (matches FRL.py format)
                cmd = f"MJX{cart[0]:.3f}Y{cart[1]:.3f}Z{cart[2]:.3f}"
                cmd += f"Rz{cart[5]:.3f}Ry{cart[4]:.3f}Rx{cart[3]:.3f}"
                cmd += f"Sp{robot_speed}Ac{accel}Dc{decel}Rm100W0Lm000000"
            else:
                # Fallback: try joint command (may not work)
                joints = r1_data['joints']
                cmd = f"MJ{joints[0]:.3f}A{joints[1]:.3f}B{joints[2]:.3f}"
                cmd += f"C{joints[3]:.3f}D{joints[4]:.3f}E{joints[5]:.3f}"
                cmd += f"F{robot_speed}G{accel}H{decel}"

            self.root.after(0, lambda c=cmd: self._log(f"R1: {c[:60]}..."))

            ok, response = self.robot1.move_and_wait(cmd)
            if ok:
                self.root.after(0, lambda: self._log("R1: Complete"))
            else:
                self.root.after(0, lambda r=response: self._log(f"R1 Error: {r}"))
                success = False
        elif waypoint.get('r1'):
            self.root.after(0, lambda: self._log("R1: NOT CONNECTED"))

        # Check if still playing before Robot 2
        if not self.playback_active:
            return False

        # Move Robot 2 and wait for completion
        if waypoint.get('r2') and self.robot2.connected:
            r2_data = waypoint['r2']

            # Use Cartesian if available, otherwise fall back to joints
            if 'cartesian' in r2_data and any(r2_data['cartesian']):
                cart = r2_data['cartesian']
                # MJ command with Cartesian (matches FRL.py format)
                cmd = f"MJX{cart[0]:.3f}Y{cart[1]:.3f}Z{cart[2]:.3f}"
                cmd += f"Rz{cart[5]:.3f}Ry{cart[4]:.3f}Rx{cart[3]:.3f}"
                cmd += f"Sp{robot_speed}Ac{accel}Dc{decel}Rm100W0Lm000000"
            else:
                joints = r2_data['joints']
                cmd = f"MJ{joints[0]:.3f}A{joints[1]:.3f}B{joints[2]:.3f}"
                cmd += f"C{joints[3]:.3f}D{joints[4]:.3f}E{joints[5]:.3f}"
                cmd += f"F{robot_speed}G{accel}H{decel}"

            self.root.after(0, lambda c=cmd: self._log(f"R2: {c[:60]}..."))

            ok, response = self.robot2.move_and_wait(cmd)
            if ok:
                self.root.after(0, lambda: self._log("R2: Complete"))
            else:
                self.root.after(0, lambda r=response: self._log(f"R2 Error: {r}"))
                success = False
        elif waypoint.get('r2'):
            self.root.after(0, lambda: self._log("R2: NOT CONNECTED"))

        # Move feeder (no wait - it's fast)
        feeder_pos = waypoint.get('feeder', 0.0)
        if self.feeder.connected and feeder_pos != self.feeder.position:
            delta = feeder_pos - self.feeder.position
            if delta > 0:
                self.feeder.feed(abs(delta))
            else:
                self.feeder.retract(abs(delta))

        return success

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

    def _restart(self):
        """Restart the application with same arguments."""
        import subprocess
        self._log("Restarting...")
        self._save_geometry()
        self.xbox.stop_polling()
        self.robot1.disconnect()
        self.robot2.disconnect()
        self.feeder.disconnect()
        # Relaunch with same arguments
        subprocess.Popen([sys.executable] + sys.argv, start_new_session=True)
        self.root.destroy()

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
    import argparse

    parser = argparse.ArgumentParser(description='Xbox Toolbox for AR4 Multi-Robot Control')
    parser.add_argument('--theme', type=str, default='cyber',
                        choices=['cyber', 'darkly', 'cyborg', 'superhero', 'vapor', 'solar'],
                        help='UI theme: cyber (default custom) or ttkbootstrap themes')
    args = parser.parse_args()

    # Kill any previous instances first
    kill_previous_instances()

    # Create root window based on theme
    if args.theme != 'cyber':
        try:
            import ttkbootstrap as ttkb
            root = ttkb.Window(themename=args.theme)
            root._theme_mode = args.theme
        except ImportError:
            print("Warning: ttkbootstrap not installed, falling back to cyber theme")
            root = tk.Tk()
            root._theme_mode = 'cyber'
    else:
        root = tk.Tk()
        root._theme_mode = 'cyber'

    app = XboxToolbox(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
