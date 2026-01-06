# position_commands 拡張計画

## 目的
`position_commands`を拡張して、軸ごとに`position, velocity, kp_scale, kd_scale`を送れるようにする。
これにより位置制御と速度制御を同じメッセージ形式で統一的に扱える。

## 現状
- `send_position_commands`: 位置のみ送信（kp_scale=1, kd_scale=1固定）
- `send_velocity_commands`: 別関数として存在（削除予定）
- `build_position_frame`: kp_scale=1.0, kd_scale=1.0がハードコード

## 新フォーマット
```
[1 byte count]
[軸0: position(8) + velocity(8) + kp_scale(8) + kd_scale(8)]
[軸1: ...]
...
```

位置制御: `position=目標位置, velocity=0, kp_scale=1, kd_scale=1`
速度制御: `position=NaN, velocity=目標速度, kp_scale=0, kd_scale=1`

## 変更ファイル

### 1. helpers.hpp
- `AxisCommand`構造体を追加
- `send_position_commands`の新シグネチャを追加
- `send_velocity_commands`は削除

### 2. helpers.cc
- `send_position_commands(vector<AxisCommand>)`を実装
- 旧`send_position_commands(vector<double>)`は互換性のため残す（kp=1,kd=1で呼び出し）
- `send_velocity_commands`は削除

### 3. moteus_communication/main.cc
- `build_position_frame`に`kp_scale, kd_scale`引数を追加
- `PositionMode::Format`に`kp_scale, kd_scale`を追加
- `position_commands`ハンドラで新フォーマットをパース

### 4. state_ready.cc
- `tick_interpolation`: 全軸分の`AxisCommand`を作成して送信
  - 股膝: position=補間値, velocity=0, kp_scale=1, kd_scale=1
  - ホイール: position=NaN, velocity=0, kp_scale=0, kd_scale=1
- `tick_ready_hold`: 同様

### 5. state_run.cc
- `tick_run`: 全軸分の`AxisCommand`を作成して送信
  - 股膝: position=初期位置, velocity=0, kp_scale=1, kd_scale=1
  - ホイール: position=NaN, velocity=PID出力, kp_scale=0, kd_scale=1

### 6. dataflow_robot.yml
- `velocity_commands`の出力/入力を削除（不要になる）
