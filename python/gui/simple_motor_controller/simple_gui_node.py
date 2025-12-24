#!/usr/bin/env python3
"""Simple GUI node for multi-axis servo control using tkinter"""

import sys
import os
import json
import struct
import queue
import threading
from dora import Node

import tkinter as tk
from tkinter import ttk

# Mode names from moteus
MODE_NAMES = {
    0: "Stop",
    1: "Fault",
    2: "Enab",
    3: "PZS",
    4: "PWM",
    5: "Volt",
    6: "VFoc",
    7: "VDQ",
    8: "Curr",
    9: "Pos",
    10: "Tmo",
    11: "ZOff",
}


class MotorControlGUI:
    def __init__(self, root, robot_config, cmd_queue):
        self.root = root
        self.cmd_queue = cmd_queue
        self.robot_config = robot_config
        self.axis_count = robot_config.get("axis_count", 0)
        self.axes = robot_config.get("axes", [])
        self.running = True

        # Data storage
        self.servo_on_mask = 0
        self.cycle_times = []

        # Per-axis data
        self.axis_data = []
        for _ in range(self.axis_count):
            self.axis_data.append({
                'mode': 0, 'position': 0.0, 'velocity': 0.0,
                'd_current': 0.0, 'q_current': 0.0, 'torque': 0.0, 'motor_temp': 0.0
            })

        # Axis selection checkboxes
        self.axis_selected = []

        # Position command entries
        self.position_entries = []

        self.setup_ui()

    def setup_ui(self):
        self.root.title(f"Motor Controller - {self.robot_config['robot_name']}")
        self.root.geometry("750x450")
        self.root.resizable(True, True)

        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Header
        header = ttk.Label(main_frame, text=f"{self.robot_config['robot_name']} ({self.axis_count} axes)",
                          font=("Helvetica", 14, "bold"))
        header.pack(pady=(0, 10))

        # Control buttons frame
        btn_frame = ttk.Frame(main_frame)
        btn_frame.pack(fill=tk.X, pady=5)

        self.servo_on_btn = ttk.Button(btn_frame, text="Servo ON", command=self.servo_on_cmd)
        self.servo_on_btn.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=5)

        self.servo_off_btn = ttk.Button(btn_frame, text="Servo OFF", command=self.servo_off_cmd)
        self.servo_off_btn.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=5)

        self.set_zero_btn = ttk.Button(btn_frame, text="Set Zero", command=self.set_zero_cmd)
        self.set_zero_btn.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=5)

        # Separator
        ttk.Separator(main_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=5)

        # Axis data table
        table_frame = ttk.LabelFrame(main_frame, text="Axis Status", padding="5")
        table_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        # Column headers
        columns = ["", "ID", "Mode", "Position", "Velocity", "Id", "Iq", "Torque", "Temp"]
        col_widths = [3, 4, 5, 10, 10, 8, 8, 8, 6]

        header_frame = ttk.Frame(table_frame)
        header_frame.pack(fill=tk.X)

        for i, (col, width) in enumerate(zip(columns, col_widths)):
            lbl = ttk.Label(header_frame, text=col, width=width, anchor=tk.CENTER,
                           font=("Courier", 9, "bold"))
            lbl.pack(side=tk.LEFT)
            if i > 0:
                sep = ttk.Separator(header_frame, orient=tk.VERTICAL)
                sep.pack(side=tk.LEFT, fill=tk.Y, padx=1)

        ttk.Separator(table_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=2)

        # Data rows
        self.row_labels = []
        for i, axis in enumerate(self.axes):
            row_frame = ttk.Frame(table_frame)
            row_frame.pack(fill=tk.X, pady=1)

            row_data = {}

            # Checkbox
            var = tk.BooleanVar(value=True)
            self.axis_selected.append(var)
            cb = ttk.Checkbutton(row_frame, variable=var, width=1)
            cb.pack(side=tk.LEFT)

            # ID
            id_lbl = ttk.Label(row_frame, text=f"{axis['device_id']}", width=4, anchor=tk.CENTER,
                              font=("Courier", 10))
            id_lbl.pack(side=tk.LEFT)
            self._add_sep(row_frame)

            # Mode
            mode_lbl = ttk.Label(row_frame, text="Stop", width=5, anchor=tk.CENTER,
                                font=("Courier", 10))
            mode_lbl.pack(side=tk.LEFT)
            row_data['mode'] = mode_lbl
            self._add_sep(row_frame)

            # Position (rev)
            pos_lbl = ttk.Label(row_frame, text="0.000", width=10, anchor=tk.E,
                               font=("Courier", 10))
            pos_lbl.pack(side=tk.LEFT)
            row_data['position'] = pos_lbl
            self._add_sep(row_frame)

            # Velocity (rev/s)
            vel_lbl = ttk.Label(row_frame, text="0.000", width=10, anchor=tk.E,
                               font=("Courier", 10))
            vel_lbl.pack(side=tk.LEFT)
            row_data['velocity'] = vel_lbl
            self._add_sep(row_frame)

            # d_current (A)
            id_cur_lbl = ttk.Label(row_frame, text="0.00", width=8, anchor=tk.E,
                                  font=("Courier", 10))
            id_cur_lbl.pack(side=tk.LEFT)
            row_data['d_current'] = id_cur_lbl
            self._add_sep(row_frame)

            # q_current (A)
            iq_cur_lbl = ttk.Label(row_frame, text="0.00", width=8, anchor=tk.E,
                                  font=("Courier", 10))
            iq_cur_lbl.pack(side=tk.LEFT)
            row_data['q_current'] = iq_cur_lbl
            self._add_sep(row_frame)

            # Torque (Nm)
            torq_lbl = ttk.Label(row_frame, text="0.000", width=8, anchor=tk.E,
                                font=("Courier", 10))
            torq_lbl.pack(side=tk.LEFT)
            row_data['torque'] = torq_lbl
            self._add_sep(row_frame)

            # Temperature (C)
            temp_lbl = ttk.Label(row_frame, text="--", width=6, anchor=tk.E,
                                font=("Courier", 10))
            temp_lbl.pack(side=tk.LEFT)
            row_data['motor_temp'] = temp_lbl

            self.row_labels.append(row_data)

            # Position command row (below status row)
            cmd_frame = ttk.Frame(table_frame)
            cmd_frame.pack(fill=tk.X, pady=1)

            # Spacer for checkbox column
            ttk.Label(cmd_frame, text="", width=3).pack(side=tk.LEFT)

            # Label
            ttk.Label(cmd_frame, text="Cmd:", width=4, font=("Courier", 9)).pack(side=tk.LEFT)

            # Position entry
            pos_var = tk.StringVar(value="0.0")
            pos_entry = ttk.Entry(cmd_frame, textvariable=pos_var, width=12, font=("Courier", 10))
            pos_entry.pack(side=tk.LEFT, padx=2)
            self.position_entries.append(pos_var)

            # Go button (move to position)
            go_btn = ttk.Button(cmd_frame, text="Go", width=4,
                               command=lambda idx=i: self.go_position_cmd(idx))
            go_btn.pack(side=tk.LEFT, padx=2)

            # Separator between axes
            if i < len(self.axes) - 1:
                ttk.Separator(table_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=3)

        # Separator
        ttk.Separator(main_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=5)

        # Cycle time frame
        cycle_frame = ttk.Frame(main_frame)
        cycle_frame.pack(fill=tk.X)

        ttk.Label(cycle_frame, text="Cycle:").pack(side=tk.LEFT)
        self.cycle_current_label = ttk.Label(cycle_frame, text="--- us", width=10,
                                             font=("Courier", 10))
        self.cycle_current_label.pack(side=tk.LEFT, padx=5)

        ttk.Label(cycle_frame, text="Avg:").pack(side=tk.LEFT)
        self.cycle_avg_label = ttk.Label(cycle_frame, text="--- us", width=10,
                                         font=("Courier", 10))
        self.cycle_avg_label.pack(side=tk.LEFT, padx=5)

        # Quit button
        ttk.Button(cycle_frame, text="Quit", command=self.quit_app).pack(side=tk.RIGHT, padx=5)

        # Bind close event
        self.root.protocol("WM_DELETE_WINDOW", self.quit_app)

    def _add_sep(self, parent):
        sep = ttk.Separator(parent, orient=tk.VERTICAL)
        sep.pack(side=tk.LEFT, fill=tk.Y, padx=1)

    def get_selected_mask(self):
        mask = 0
        for i, var in enumerate(self.axis_selected):
            if var.get():
                mask |= (1 << i)
        return mask

    def servo_on_cmd(self):
        mask = self.get_selected_mask()
        if mask == 0:
            print("[gui] No axes selected")
            return
        print(f"[gui] Sending Servo ON (mask=0x{mask:02x})")
        self.cmd_queue.put(("servo_on", mask))
        self.servo_on_mask |= mask

    def servo_off_cmd(self):
        mask = self.get_selected_mask()
        if mask == 0:
            print("[gui] No axes selected")
            return
        print(f"[gui] Sending Servo OFF (mask=0x{mask:02x})")
        self.cmd_queue.put(("servo_off", mask))
        self.servo_on_mask &= ~mask

    def set_zero_cmd(self):
        mask = self.get_selected_mask()
        if mask == 0:
            print("[gui] No axes selected")
            return
        print(f"[gui] Sending Set Zero (mask=0x{mask:02x})")
        self.cmd_queue.put(("set_zero", mask))

    def go_position_cmd(self, axis_index):
        """Send position command for a single axis"""
        try:
            pos = float(self.position_entries[axis_index].get())
        except ValueError:
            print(f"[gui] Invalid position value for axis {axis_index}")
            return
        print(f"[gui] Go position: axis={axis_index}, pos={pos}")
        self.cmd_queue.put(("go_position", (axis_index, pos)))

    def quit_app(self):
        print("[gui] Quit")
        self.running = False
        self.cmd_queue.put(("quit", None))
        self.root.quit()

    def update_motor_status(self, statuses):
        """Update all motor status data"""
        for i, status in enumerate(statuses):
            if i >= len(self.row_labels):
                break
            row = self.row_labels[i]

            # Mode
            mode_name = MODE_NAMES.get(status['mode'], f"?{status['mode']}")
            fg = "green" if status['mode'] == 9 else ("red" if status['mode'] == 1 else "black")
            row['mode'].config(text=mode_name, foreground=fg)

            # Position
            row['position'].config(text=f"{status['position']:.5f}")

            # Velocity
            row['velocity'].config(text=f"{status['velocity']:.5f}")

            # d_current
            row['d_current'].config(text=f"{status['d_current']:.3f}")

            # q_current
            row['q_current'].config(text=f"{status['q_current']:.3f}")

            # Torque
            row['torque'].config(text=f"{status['torque']:.4f}")

            # Motor temperature
            if status['motor_temp'] > 0:
                row['motor_temp'].config(text=f"{status['motor_temp']:.1f}")
            else:
                row['motor_temp'].config(text="--")

    def update_cycle_time(self, cycle_us):
        self.cycle_times.append(cycle_us)
        if len(self.cycle_times) > 100:
            self.cycle_times.pop(0)

        avg = sum(self.cycle_times) / len(self.cycle_times)
        self.cycle_current_label.config(text=f"{cycle_us} us")
        self.cycle_avg_label.config(text=f"{avg:.0f} us")


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
    print("[gui] Starting")

    robot_config = load_robot_config()
    node = Node("gui")

    cmd_queue = queue.Queue()  # GUI -> dora thread
    status_queue = queue.Queue()  # dora thread -> GUI

    root = tk.Tk()
    gui = MotorControlGUI(root, robot_config, cmd_queue)

    # Dora event processing in separate thread
    def dora_thread():
        config_sent = False
        while gui.running:
            # Process commands from GUI
            try:
                while True:
                    cmd, data = cmd_queue.get_nowait()
                    if cmd == "servo_on":
                        node.send_output("servo_on", bytes([data]))
                    elif cmd == "servo_off":
                        node.send_output("servo_off", bytes([data]))
                    elif cmd == "set_zero":
                        node.send_output("set_zero", bytes([data]))
                    elif cmd == "go_position":
                        axis_index, pos = data
                        payload = struct.pack('B', axis_index) + struct.pack('d', pos)
                        node.send_output("position_command", payload)
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
                    if event["id"] == "tick":
                        # Send robot_config on first tick
                        if not config_sent:
                            config_json = json.dumps(robot_config)
                            node.send_output("robot_config", config_json.encode('utf-8'))
                            print("[gui] Sent robot_config")
                            config_sent = True
                        # Forward tick as query to moteus_communication
                        node.send_output("query", bytes([0]))

                    elif event["id"] == "cycle_time_us":
                        raw = bytes(event["value"].to_pylist())
                        if len(raw) >= 8:
                            cycle_us = struct.unpack('q', raw[:8])[0]
                            status_queue.put(("cycle_time", cycle_us))

                    elif event["id"] == "motor_status":
                        raw = bytes(event["value"].to_pylist())
                        if len(raw) >= 1:
                            count = raw[0]
                            statuses = []
                            offset = 1
                            for i in range(count):
                                if offset + 49 <= len(raw):
                                    mode = raw[offset]
                                    offset += 1
                                    pos = struct.unpack('d', raw[offset:offset+8])[0]
                                    offset += 8
                                    vel = struct.unpack('d', raw[offset:offset+8])[0]
                                    offset += 8
                                    d_cur = struct.unpack('d', raw[offset:offset+8])[0]
                                    offset += 8
                                    q_cur = struct.unpack('d', raw[offset:offset+8])[0]
                                    offset += 8
                                    torq = struct.unpack('d', raw[offset:offset+8])[0]
                                    offset += 8
                                    temp = struct.unpack('d', raw[offset:offset+8])[0]
                                    offset += 8
                                    statuses.append({
                                        'mode': mode, 'position': pos, 'velocity': vel,
                                        'd_current': d_cur, 'q_current': q_cur,
                                        'torque': torq, 'motor_temp': temp
                                    })
                            status_queue.put(("motor_status", statuses))
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
                elif msg_type == "cycle_time":
                    gui.update_cycle_time(data)
                elif msg_type == "motor_status":
                    gui.update_motor_status(data)
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
