#!/usr/bin/env python3
"""Robot Control GUI - State machine control interface using tkinter"""

import sys
import os
import json
import struct
import queue
import threading
from dora import Node

import tkinter as tk
from tkinter import ttk

# State definitions (must match mimicv2_state_manager)
STATE_NAMES = {
    0: "INIT",
    1: "SERVO_OFF",
    2: "READY",
    3: "RUN",
}

# State command definitions
STATE_CMD_SERVO_OFF = 0
STATE_CMD_STOP = 1
STATE_CMD_READY = 2
STATE_CMD_RUN = 3


class RobotControlGUI:
    def __init__(self, root, robot_config, cmd_queue):
        self.root = root
        self.cmd_queue = cmd_queue
        self.robot_config = robot_config
        self.axis_count = robot_config.get("axis_count", 0)
        self.axes = robot_config.get("axes", [])
        self.running = True

        # State
        self.current_state = 0
        self.progress = 0

        # Per-axis data (position, torque)
        self.axis_data = []
        for _ in range(self.axis_count):
            self.axis_data.append({
                'position': 0.0,
                'torque': 0.0
            })

        self.setup_ui()

    def setup_ui(self):
        self.root.title(f"Robot Controller - {self.robot_config['robot_name']}")
        self.root.geometry("500x400")
        self.root.resizable(True, True)

        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Header
        header = ttk.Label(main_frame, text=f"{self.robot_config['robot_name']}",
                          font=("Helvetica", 14, "bold"))
        header.pack(pady=(0, 10))

        # State display frame
        state_frame = ttk.LabelFrame(main_frame, text="State", padding="10")
        state_frame.pack(fill=tk.X, pady=5)

        state_inner = ttk.Frame(state_frame)
        state_inner.pack(fill=tk.X)

        ttk.Label(state_inner, text="Current State:", font=("Helvetica", 11)).pack(side=tk.LEFT)
        self.state_label = ttk.Label(state_inner, text="INIT", font=("Helvetica", 14, "bold"),
                                     foreground="gray")
        self.state_label.pack(side=tk.LEFT, padx=10)

        # Progress bar (for READY interpolation)
        self.progress_var = tk.IntVar(value=0)
        self.progress_bar = ttk.Progressbar(state_frame, variable=self.progress_var,
                                            maximum=100, length=200)
        self.progress_bar.pack(fill=tk.X, pady=(5, 0))

        # Control buttons frame
        btn_frame = ttk.LabelFrame(main_frame, text="Control", padding="10")
        btn_frame.pack(fill=tk.X, pady=5)

        btn_inner = ttk.Frame(btn_frame)
        btn_inner.pack(fill=tk.X)

        self.servo_off_btn = ttk.Button(btn_inner, text="Servo OFF",
                                        command=self.servo_off_cmd, width=12)
        self.servo_off_btn.pack(side=tk.LEFT, expand=True, padx=2)

        self.stop_btn = ttk.Button(btn_inner, text="Stop",
                                   command=self.stop_cmd, width=12)
        self.stop_btn.pack(side=tk.LEFT, expand=True, padx=2)

        self.ready_btn = ttk.Button(btn_inner, text="Ready",
                                    command=self.ready_cmd, width=12)
        self.ready_btn.pack(side=tk.LEFT, expand=True, padx=2)

        self.run_btn = ttk.Button(btn_inner, text="Run",
                                  command=self.run_cmd, width=12)
        self.run_btn.pack(side=tk.LEFT, expand=True, padx=2)

        # Axis data table
        table_frame = ttk.LabelFrame(main_frame, text="Motor Status", padding="5")
        table_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        # Column headers
        header_frame = ttk.Frame(table_frame)
        header_frame.pack(fill=tk.X)

        ttk.Label(header_frame, text="ID", width=6, anchor=tk.CENTER,
                  font=("Courier", 10, "bold")).pack(side=tk.LEFT)
        ttk.Label(header_frame, text="Name", width=15, anchor=tk.CENTER,
                  font=("Courier", 10, "bold")).pack(side=tk.LEFT)
        ttk.Label(header_frame, text="Position (rev)", width=15, anchor=tk.CENTER,
                  font=("Courier", 10, "bold")).pack(side=tk.LEFT)
        ttk.Label(header_frame, text="Torque (Nm)", width=15, anchor=tk.CENTER,
                  font=("Courier", 10, "bold")).pack(side=tk.LEFT)

        ttk.Separator(table_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=2)

        # Data rows
        self.row_labels = []
        for i, axis in enumerate(self.axes):
            row_frame = ttk.Frame(table_frame)
            row_frame.pack(fill=tk.X, pady=1)

            row_data = {}

            # ID
            ttk.Label(row_frame, text=f"{axis['device_id']}", width=6, anchor=tk.CENTER,
                      font=("Courier", 10)).pack(side=tk.LEFT)

            # Name
            ttk.Label(row_frame, text=f"{axis['name']}", width=15, anchor=tk.W,
                      font=("Courier", 10)).pack(side=tk.LEFT)

            # Position
            pos_lbl = ttk.Label(row_frame, text="0.000", width=15, anchor=tk.E,
                               font=("Courier", 10))
            pos_lbl.pack(side=tk.LEFT)
            row_data['position'] = pos_lbl

            # Torque
            torq_lbl = ttk.Label(row_frame, text="0.000", width=15, anchor=tk.E,
                                font=("Courier", 10))
            torq_lbl.pack(side=tk.LEFT)
            row_data['torque'] = torq_lbl

            self.row_labels.append(row_data)

        # Bottom frame with Quit button
        bottom_frame = ttk.Frame(main_frame)
        bottom_frame.pack(fill=tk.X, pady=5)

        ttk.Button(bottom_frame, text="Quit", command=self.quit_app).pack(side=tk.RIGHT)

        # Bind close event
        self.root.protocol("WM_DELETE_WINDOW", self.quit_app)

    def servo_off_cmd(self):
        print("[gui] Servo OFF command")
        self.cmd_queue.put(("state_command", STATE_CMD_SERVO_OFF))

    def stop_cmd(self):
        print("[gui] Stop command")
        self.cmd_queue.put(("state_command", STATE_CMD_STOP))

    def ready_cmd(self):
        print("[gui] Ready command")
        self.cmd_queue.put(("state_command", STATE_CMD_READY))

    def run_cmd(self):
        print("[gui] Run command")
        self.cmd_queue.put(("state_command", STATE_CMD_RUN))

    def quit_app(self):
        print("[gui] Quit")
        self.running = False
        self.cmd_queue.put(("quit", None))
        self.root.quit()

    def update_state_status(self, state, progress):
        """Update state display"""
        self.current_state = state
        self.progress = progress

        state_name = STATE_NAMES.get(state, f"?{state}")
        self.state_label.config(text=state_name)

        # Color coding
        if state == 1:  # SERVO_OFF
            self.state_label.config(foreground="gray")
        elif state == 2:  # READY
            if progress < 100:
                self.state_label.config(foreground="orange")
            else:
                self.state_label.config(foreground="blue")
        elif state == 3:  # RUN
            self.state_label.config(foreground="green")
        else:
            self.state_label.config(foreground="gray")

        self.progress_var.set(progress)

    def update_motor_display(self, positions, torques):
        """Update motor position and torque display"""
        for i in range(min(len(positions), len(self.row_labels))):
            self.row_labels[i]['position'].config(text=f"{positions[i]:.4f}")
        for i in range(min(len(torques), len(self.row_labels))):
            self.row_labels[i]['torque'].config(text=f"{torques[i]:.4f}")


def load_robot_config():
    config_path = os.environ.get("ROBOT_CONFIG", "robot_config/mimic_v2.json")
    if not os.path.isabs(config_path):
        config_path = os.path.join(os.getcwd(), config_path)
    try:
        with open(config_path, 'r') as f:
            robot_config = json.load(f)
        print(f"[gui] Loaded config: {robot_config['robot_name']} ({robot_config['axis_count']} axes)")
        return robot_config
    except Exception as e:
        print(f"[gui] ERROR: Failed to load config: {e}")
        sys.exit(1)


def main():
    print("[gui] Starting robot_control_gui")

    robot_config = load_robot_config()
    node = Node("robot_control_gui")

    cmd_queue = queue.Queue()  # GUI -> dora thread
    status_queue = queue.Queue()  # dora thread -> GUI

    root = tk.Tk()
    gui = RobotControlGUI(root, robot_config, cmd_queue)

    # Dora event processing in separate thread
    def dora_thread():
        # Send robot_config immediately at thread start
        config_json = json.dumps(robot_config)
        node.send_output("robot_config", config_json.encode('utf-8'))
        print("[gui] Sent robot_config")

        while gui.running:
            # Process commands from GUI
            try:
                while True:
                    cmd, data = cmd_queue.get_nowait()
                    if cmd == "state_command":
                        node.send_output("state_command", bytes([data]))
                    elif cmd == "quit":
                        return
            except queue.Empty:
                pass

            # Process dora events
            try:
                event = node.next(timeout=0.005)
                if event is None:
                    continue

                if event["type"] == "STOP":
                    print("[gui] Received stop signal")
                    status_queue.put(("stop", None))
                    return
                elif event["type"] == "INPUT":
                    if event["id"] == "state_status":
                        raw = bytes(event["value"].to_pylist())
                        if len(raw) >= 2:
                            state = raw[0]
                            progress = raw[1]
                            status_queue.put(("state_status", (state, progress)))

                    elif event["id"] == "motor_display":
                        raw = bytes(event["value"].to_pylist())
                        if len(raw) >= 1:
                            count = raw[0]
                            offset = 1
                            positions = []
                            torques = []

                            # Positions
                            for i in range(count):
                                if offset + 8 <= len(raw):
                                    pos = struct.unpack('d', raw[offset:offset+8])[0]
                                    positions.append(pos)
                                    offset += 8

                            # Torques
                            for i in range(count):
                                if offset + 8 <= len(raw):
                                    torq = struct.unpack('d', raw[offset:offset+8])[0]
                                    torques.append(torq)
                                    offset += 8

                            status_queue.put(("motor_display", (positions, torques)))

            except Exception as e:
                print(f"[gui] Dora error: {e}")

    # GUI update from status_queue
    def update_gui():
        if not gui.running:
            return
        try:
            while True:
                msg_type, data = status_queue.get_nowait()
                if msg_type == "stop":
                    gui.quit_app()
                    return
                elif msg_type == "state_status":
                    state, progress = data
                    gui.update_state_status(state, progress)
                elif msg_type == "motor_display":
                    positions, torques = data
                    gui.update_motor_display(positions, torques)
        except queue.Empty:
            pass
        root.after(10, update_gui)

    # Start dora thread
    thread = threading.Thread(target=dora_thread, daemon=True)
    thread.start()

    root.after(10, update_gui)
    root.mainloop()
    print("[gui] Finished")


if __name__ == "__main__":
    main()
