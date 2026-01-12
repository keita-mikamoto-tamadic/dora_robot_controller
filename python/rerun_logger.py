#!/usr/bin/env python3
"""
rerun_logger.py - RUN状態で1000ポイント収集してCSV保存

起動方法:
  python3 python/rerun_logger.py

dataflow_simple.yml起動後に別ターミナルで起動すること。
RUN状態になると自動的に1000ポイント収集し、python/log/に保存。

再生方法:
  python3 python/rerun_logger.py python/log/YYYYMMDD_HHMMSS.csv
"""

import struct
import sys
from datetime import datetime
from pathlib import Path
import rerun as rr
from dora import Node


LOG_DIR = Path(__file__).parent / "log"
MAX_POINTS = 1000  # 1000ポイント = 約4秒 (4ms * 1000)

# state_statusの状態値
STATE_RUN = 3

# motor_status フォーマット: [count(1)][per axis: pos(8) + vel(8) + torque(8) + fault(1)] = 25 bytes/axis
MOTOR_STATUS_AXIS_SIZE = 25  # pos(8) + vel(8) + torque(8) + fault(1)
WHEEL_R_INDEX = 2
WHEEL_L_INDEX = 5


def save_csv(data_list, filepath):
    """データをCSVに保存"""
    with open(filepath, 'w') as f:
        f.write("timestamp,pitch,pitch_rate,error,integral,wheel_vel,control_output,"
                "wheel_r_pos,wheel_r_vel,wheel_r_torque,wheel_l_pos,wheel_l_vel,wheel_l_torque\n")
        for row in data_list:
            f.write(",".join(f"{v:.6f}" for v in row) + "\n")
    print(f"[rerun_logger] Saved: {filepath}")


def play_csv(filepath):
    """CSVデータをrerunで再生"""
    rr.init("mimic_balancing")
    rr.spawn()

    with open(filepath, 'r') as f:
        lines = f.readlines()[1:]  # ヘッダースキップ

    for line in lines:
        vals = [float(x) for x in line.strip().split(',')]
        # 新フォーマット: 13列
        if len(vals) >= 13:
            (timestamp, pitch, pitch_rate, error, integral, wheel_vel, control,
             wheel_r_pos, wheel_r_vel, wheel_r_torque,
             wheel_l_pos, wheel_l_vel, wheel_l_torque) = vals[:13]
        else:
            # 旧フォーマット互換: 7列
            timestamp, pitch, pitch_rate, error, integral, wheel_vel, control = vals[:7]
            wheel_r_pos = wheel_r_vel = wheel_r_torque = 0.0
            wheel_l_pos = wheel_l_vel = wheel_l_torque = 0.0

        rr.set_time("time", timestamp=timestamp)
        rr.log("pid/pitch", rr.Scalars([pitch]))
        rr.log("pid/pitch_rate", rr.Scalars([pitch_rate]))
        rr.log("pid/error", rr.Scalars([error]))
        rr.log("pid/integral", rr.Scalars([integral]))
        rr.log("pid/wheel_vel", rr.Scalars([wheel_vel]))
        rr.log("pid/control_output", rr.Scalars([control]))
        # motor data
        rr.log("motor/wheel_r/position", rr.Scalars([wheel_r_pos]))
        rr.log("motor/wheel_r/velocity", rr.Scalars([wheel_r_vel]))
        rr.log("motor/wheel_r/torque", rr.Scalars([wheel_r_torque]))
        rr.log("motor/wheel_l/position", rr.Scalars([wheel_l_pos]))
        rr.log("motor/wheel_l/velocity", rr.Scalars([wheel_l_vel]))
        rr.log("motor/wheel_l/torque", rr.Scalars([wheel_l_torque]))

    print(f"[rerun_logger] Loaded {len(lines)} points into rerun viewer.")
    print("Press Ctrl+C to exit.")
    try:
        while True:
            pass
    except KeyboardInterrupt:
        pass


def parse_motor_status(data, axis_index):
    """motor_statusから指定軸のデータを取得"""
    if len(data) < 1:
        return (0.0, 0.0, 0.0)

    count = data[0]
    if axis_index >= count:
        return (0.0, 0.0, 0.0)

    # オフセット計算: 1 (count) + axis_index * 25
    offset = 1 + axis_index * MOTOR_STATUS_AXIS_SIZE
    if len(data) < offset + 24:  # pos(8) + vel(8) + torque(8)
        return (0.0, 0.0, 0.0)

    pos, vel, torque = struct.unpack('3d', data[offset:offset+24])
    return (pos, vel, torque)


def collect_data():
    """Doraノードとしてデータ収集"""
    LOG_DIR.mkdir(exist_ok=True)

    node = Node("rerun_logger")
    print("[rerun_logger] Started. Waiting for RUN state...")

    collecting = False
    data_buffer = []
    current_state = 0

    # motor_statusの最新データを保持
    latest_motor = {
        'wheel_r': (0.0, 0.0, 0.0),  # (pos, vel, torque)
        'wheel_l': (0.0, 0.0, 0.0),
    }

    for event in node:
        if event["type"] == "INPUT":
            id = event["id"]
            data = bytes(event["value"].to_pylist())

            # state_statusでRUN状態を検出
            if id == "state_status" and len(data) >= 1:
                current_state = data[0]
                if current_state == STATE_RUN and not collecting:
                    collecting = True
                    data_buffer = []
                    print(f"[rerun_logger] RUN detected. Collecting {MAX_POINTS} points...")

            # motor_statusを更新
            if id == "motor_status":
                latest_motor['wheel_r'] = parse_motor_status(data, WHEEL_R_INDEX)
                latest_motor['wheel_l'] = parse_motor_status(data, WHEEL_L_INDEX)

            # debug_dataを収集（motor_statusも合わせて保存）
            if id == "debug_data" and len(data) >= 56 and collecting:
                debug_vals = struct.unpack('7d', data[:56])
                # debug_data + motor_status を結合
                wheel_r = latest_motor['wheel_r']
                wheel_l = latest_motor['wheel_l']
                combined = debug_vals + wheel_r + wheel_l  # 7 + 3 + 3 = 13 values
                data_buffer.append(combined)

                if len(data_buffer) % 250 == 0:
                    print(f"[rerun_logger] Collected {len(data_buffer)}/{MAX_POINTS}")

                if len(data_buffer) >= MAX_POINTS:
                    collecting = False
                    filename = datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv"
                    filepath = LOG_DIR / filename
                    save_csv(data_buffer, filepath)
                    data_buffer = []
                    print(f"[rerun_logger] Ready for next RUN...")
                    print(f"[rerun_logger] To view: python3 python/rerun_logger.py {filepath}")


def main():
    if len(sys.argv) > 1:
        # 引数あり → CSV再生モード
        csv_path = Path(sys.argv[1])
        if not csv_path.exists():
            print(f"Error: File not found: {csv_path}")
            sys.exit(1)
        play_csv(csv_path)
    else:
        # 引数なし → 収集モード
        collect_data()


if __name__ == "__main__":
    main()
