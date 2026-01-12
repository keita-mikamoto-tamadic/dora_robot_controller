#!/usr/bin/env python3
"""
simple_cui.py - シンプルなCUIコントローラ

キー操作:
  s: SERVO ON (OFF -> STOP)
  o: SERVO OFF
  r: RUN
  t: STOP
  q: 終了
"""

import json
import struct
import sys
import os
import select
import termios
import tty
from dora import Node

def main():
    # robot_config読み込み
    config_path = os.environ.get("ROBOT_CONFIG", "robot_config/mimic_v2.json")
    with open(config_path) as f:
        config = json.load(f)

    # dora dynamic nodeとして初期化
    node = Node("cui")

    # config送信
    config_bytes = json.dumps(config).encode()
    node.send_output("robot_config", config_bytes)
    print(f"Config sent: {len(config['axes'])} axes")

    print("\n=== Simple CUI ===")
    print("s: SERVO ON  o: SERVO OFF  r: RUN  t: STOP")
    print("e: READY  i: INIT POS  q: quit\n")

    state_names = ["OFF", "STOP", "READY", "RUN"]

    # 端末設定保存
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        # 非カノニカルモード（1文字ずつ読める）
        tty.setcbreak(fd)

        for event in node:
            # キー入力チェック（ノンブロッキング）
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                cmd = None

                if key == 's':
                    cmd = 3  # SERVO_ON
                    print("-> SERVO ON")
                elif key == 'o':
                    cmd = 2  # SERVO_OFF
                    print("-> SERVO OFF")
                elif key == 'r':
                    cmd = 1  # RUN
                    print("-> RUN")
                elif key == 't':
                    cmd = 0  # STOP
                    print("-> STOP")
                elif key == 'e':
                    cmd = 5  # READY
                    print("-> READY")
                elif key == 'i':
                    cmd = 4  # INIT_POSITION_RESET
                    print("-> INIT POS RESET")
                elif key == 'q':
                    print("Quit")
                    break

                if cmd is not None:
                    node.send_output("state_command", bytes([cmd]))

            # イベント処理
            if event["type"] == "INPUT":
                id = event["id"]
                data = bytes(event["value"].to_pylist())

                if id == "state_status" and len(data) >= 1:
                    st = data[0]
                    if st < len(state_names):
                        print(f"State: {state_names[st]}")

                elif id == "motor_status" and len(data) >= 1:
                    count = data[0]
                    positions = []
                    velocities = []
                    torques = []
                    for i in range(count):
                        off = 1 + i * 25
                        if off + 24 <= len(data):
                            pos = struct.unpack('d', data[off:off+8])[0]
                            vel = struct.unpack('d', data[off+8:off+16])[0]
                            torque = struct.unpack('d', data[off+16:off+24])[0]
                            positions.append(f"{pos:.2f}")
                            velocities.append(f"{vel:.2f}")
                            torques.append(f"{torque:.2f}")
                    print(f"\rPos: [{', '.join(positions)}]                    ", end='')
                    print(f"\nVel: [{', '.join(velocities)}]                    ", end='')
                    print(f"\nTorque: [{', '.join(torques)}]                    ", end='')
                    print("\033[F\033[F", end='', flush=True)  # カーソルを2行上に戻す

    finally:
        # 端末設定復元
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

if __name__ == "__main__":
    main()
