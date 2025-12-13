#!/usr/bin/env python3
"""CUI node for multi-axis servo control - dynamic node (start manually in separate terminal)"""

import sys
import os
import json
import tty
import termios
import select
import struct
from dora import Node

# Statistics
cycle_times = []
last_torques = []

# Motion limits (adjust these values as needed)
VELOCITY_LIMIT = 100.0   # rev/s
ACCEL_LIMIT = 200.0      # rev/s^2
TORQUE_LIMIT = 10       # Nm

# Robot config
robot_config = None
axis_count = 0
device_ids = []  # device_id list for display

def load_robot_config():
    """Load robot configuration from JSON file"""
    global robot_config, axis_count, last_torques, device_ids

    config_path = os.environ.get("ROBOT_CONFIG", "robot_config/single_axis.json")

    # Handle relative paths
    if not os.path.isabs(config_path):
        config_path = os.path.join(os.getcwd(), config_path)

    try:
        with open(config_path, 'r') as f:
            robot_config = json.load(f)
        axis_count = robot_config.get("axis_count", 0)
        last_torques = [0.0] * axis_count
        device_ids = [axis['device_id'] for axis in robot_config.get("axes", [])]
        print(f"[cui] Loaded config: {robot_config['robot_name']} ({axis_count} axes)")
        for axis in robot_config.get("axes", []):
            print(f"  - {axis['name']}: device_id={axis['device_id']}, motdir={axis['motdir']}")
    except Exception as e:
        print(f"[cui] ERROR: Failed to load config: {e}")
        sys.exit(1)

def print_help():
    """Print help message"""
    print("\n=== Multi-Axis Moteus Controller ===")
    print(f"Robot: {robot_config['robot_name']} ({axis_count} axes)")
    print("[1] Servo ON  [0] Servo OFF  [q] Quit")
    print("====================================")

def update_status():
    """Update status display on single line using \\r"""
    # Build torque string: ID1:+0.123 ID2:-0.456 ...
    torque_parts = []
    for i, dev_id in enumerate(device_ids):
        torque = last_torques[i] if i < len(last_torques) else 0.0
        torque_parts.append(f"ID{dev_id}:{torque:+.3f}")
    torque_str = " ".join(torque_parts)

    # Cycle time
    if cycle_times:
        avg = sum(cycle_times) / len(cycle_times)
        cycle_str = f"cyc:{cycle_times[-1]}us avg:{avg:.0f}us"
    else:
        cycle_str = "cyc:--- avg:---"

    # Print single line with \r to overwrite
    line = f"\r[{torque_str} | {cycle_str}] > "
    print(line + " " * 10, end="", flush=True)

def main():
    print("[cui] Starting (dynamic node)")

    # Load robot configuration
    load_robot_config()

    node = Node("cui")

    # Send robot config to moteus_communication on startup
    config_json = json.dumps(robot_config)
    node.send_output("robot_config", config_json.encode('utf-8'))
    print("[cui] Sent robot_config to moteus_communication")

    print_help()

    # Save terminal settings
    old_settings = termios.tcgetattr(sys.stdin)

    try:
        # Set terminal to raw mode (non-canonical, no echo)
        tty.setcbreak(sys.stdin.fileno())

        while True:
            # Check dora events with timeout
            dora_event = node.next(timeout=0.05)

            if dora_event is not None:
                if dora_event["type"] == "STOP":
                    print("\n[cui] Received stop signal")
                    break
                elif dora_event["type"] == "INPUT":
                    if dora_event["id"] == "cycle_time_us":
                        raw = bytes(dora_event["value"].to_pylist())
                        if len(raw) >= 8:
                            cycle_us = struct.unpack('q', raw[:8])[0]  # int64_t
                            cycle_times.append(cycle_us)
                            # Keep only last 100 samples
                            if len(cycle_times) > 100:
                                cycle_times.pop(0)
                            update_status()
                    elif dora_event["id"] == "motor_torques":
                        raw = bytes(dora_event["value"].to_pylist())
                        # Format: [axis_count][torque1 (8 bytes)][torque2 (8 bytes)]...
                        if len(raw) >= 1:
                            count = raw[0]
                            offset = 1
                            for i in range(min(count, axis_count)):
                                if offset + 8 <= len(raw):
                                    last_torques[i] = struct.unpack('d', raw[offset:offset+8])[0]
                                    offset += 8

            # Check keyboard input (non-blocking)
            if select.select([sys.stdin], [], [], 0)[0]:
                c = sys.stdin.read(1)

                if c == '1':
                    print("\n[cui] Sending Servo ON (all axes)")
                    node.send_output("velocity_limit", struct.pack('d', VELOCITY_LIMIT))
                    node.send_output("accel_limit", struct.pack('d', ACCEL_LIMIT))
                    node.send_output("torque_limit", struct.pack('d', TORQUE_LIMIT))
                    node.send_output("servo_on", bytes([1]))
                    print_help()
                elif c == '0':
                    print("\n[cui] Sending Servo OFF (all axes)")
                    node.send_output("servo_off", bytes([1]))
                    print_help()
                elif c in ('q', 'Q', '\x1b'):  # q, Q, or ESC
                    print("\n[cui] Quit")
                    break
                elif c in ('h', 'H', '?'):
                    print_help()

    finally:
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    print("[cui] Finished")

if __name__ == "__main__":
    main()
