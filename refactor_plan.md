# mimic-controller リファクタリング計画

## 現状の問題点

### 1. メッセージフローが複雑すぎる
現在の1サイクル（5ms）で流れるメッセージ:
```
tick → state_manager
    ├─ position_commands → moteus_communication (保存するだけ)
    ├─ query → moteus_communication (これがCANフレーム送信トリガー)
    └─ state_status → GUI

moteus_communication → can_frames → canfd_txrx
                                        ↓
                                   rx_frames
                                        ↓
moteus_communication → motor_status → state_manager
                                        ↓
                                   motor_display → GUI
```

**問題:**
- `position_commands`と`query`が分離しているため、順序問題が発生
- `motor_display`が毎回`motor_status`受信時に送信される（冗長）
- `robot_config`がstate_manager経由で転送される（不要な中継）

### 2. moteus_communicationの責務が曖昧
- position_commandsで位置を保存
- queryでCANフレーム構築＆送信
- rx_framesでモーターステータス解析
- servo_on_delayの管理

**問題:** 単位変換とCAN送受信が混在

### 3. canfd_txrxのグループ処理
- Group1送信 → Group1受信待ち → Group2送信 → Group2受信待ち
- シーケンシャル処理で最大8msかかる

---

## 新アーキテクチャ設計

### 基本方針
1. **メッセージを統合・削減** - 1 tickで最小限のメッセージ
2. **責務を明確化** - 各ノードの役割をシンプルに
3. **タイミング問題を解消** - 分離したメッセージを統合

### 新しいデータフロー

```
┌─────────────────────────────────────────────────┐
│              Dora Timer (5ms)                   │
└───────────────────┬─────────────────────────────┘
                    │ tick
                    ▼
┌─────────────────────────────────────────────────┐
│         state_manager (状態機械)                │
│  - 状態遷移管理                                 │
│  - 制御アルゴリズム実行                         │
│  - 位置/速度指令生成                            │
└───────────────────┬─────────────────────────────┘
                    │ motor_commands
                    │ (position + velocity + control_mode)
                    ▼
┌─────────────────────────────────────────────────┐
│         moteus_driver (moteusドライバ)          │
│  - CANフレーム構築                              │
│  - 単位変換 (rad ↔ rev)                        │
│  - CAN送受信                                    │
│  - servo_on_delay管理                           │
│  [RT_PRIORITY: 80, CPU_AFFINITY: 1]            │
└───────────────────┬─────────────────────────────┘
                    │ motor_status
                    │ (position + velocity + torque + fault)
                    ▼
              [state_manager]
                    │
                    │ state_status + motor_display
                    ▼
              [robot_control_gui]
```

### 変更点

| 現在 | 新設計 |
|------|--------|
| 4ノード (state_manager, moteus_communication, canfd_txrx, GUI) | 3ノード (state_manager, moteus_driver, GUI) |
| 7種類のメッセージ/tick | 3種類のメッセージ/tick |
| position_commands + query分離 | motor_commands統合 |
| canfd_txrx別ノード | moteus_driverに統合 |

---

## 実装計画

### Phase 1: moteus_driver新規作成

**新ノード: `cpp/motor/moteus_driver/`**

```cpp
// 入力
struct MotorCommand {
    uint8_t control_mode;  // 0=servo_off, 1=stop, 2=position, 3=velocity
    double position;       // rad (position/stopモード)
    double velocity;       // rad/s (velocityモード)
    double kp_scale;       // 位置ゲイン
    double kd_scale;       // 速度ゲイン
};

// 出力
struct MotorStatus {
    double position;       // rad
    double velocity;       // rad/s
    double torque;         // Nm
    uint8_t mode;          // moteus mode
    uint8_t fault;         // fault code
};
```

**処理フロー:**
```cpp
on_motor_commands(commands):
    for each axis:
        // 単位変換
        pos_rev = rad_to_rev(commands[i].position) * motdir
        vel_rev = rad_to_rev(commands[i].velocity) * motdir

        // servo_on_delay処理
        if (servo_on_delay[i] > 0):
            build_nan_position_frame()
            servo_on_delay[i]--
        else:
            build_position_frame(pos_rev, vel_rev, ...)

    // CAN送信 (Group1)
    for frame in group1:
        can_send(frame)
    rx1 = can_receive_multiple(group1_ids, timeout=2ms)

    // CAN送信 (Group2)
    for frame in group2:
        can_send(frame)
    rx2 = can_receive_multiple(group2_ids, timeout=2ms)

    // レスポンス解析
    parse_responses(rx1 + rx2)
    send_motor_status()
```

### Phase 2: state_manager簡素化

**削除するもの:**
- `send_position_commands()` → `send_motor_commands()`に統合
- `send_query()` → 不要（motor_commandsで自動トリガー）
- `forward_robot_config()` → GUIから直接moteus_driverへ

**新しいヘルパー:**
```cpp
struct AxisCommand {
    ControlMode mode;  // SERVO_OFF, STOP, POSITION, VELOCITY
    double position;
    double velocity;
    double kp_scale;
    double kd_scale;
};

void send_motor_commands(const std::vector<AxisCommand>& commands);
```

### Phase 3: dataflow_robot.yml更新

```yaml
nodes:
  - id: imu_node
    path: cpp/sensor/imu/sony/imu_node
    inputs:
      tick: dora/timer/millis/1
    outputs:
      - imu_data

  - id: robot_control_gui
    path: dynamic
    inputs:
      state_status: state_manager/state_status
      motor_display: state_manager/motor_display
      imu_data: imu_node/imu_data
    outputs:
      - robot_config
      - state_command

  - id: state_manager
    path: cpp/state_machine/mimicv2_state_manager/mimicv2_state_manager_node
    inputs:
      tick: dora/timer/millis/5
      robot_config: robot_control_gui/robot_config
      motor_status: moteus_driver/motor_status
      state_command: robot_control_gui/state_command
      imu_data: imu_node/imu_data
    outputs:
      - motor_commands
      - state_status
      - motor_display
      - set_position

  - id: moteus_driver
    path: cpp/motor/moteus_driver/moteus_driver_node
    inputs:
      robot_config: robot_control_gui/robot_config  # 直接受信
      motor_commands: state_manager/motor_commands
      set_position: state_manager/set_position
    outputs:
      - motor_status
    env:
      CAN_INTERFACE: "can0"
      RX_TIMEOUT_US: "2000"
      CPU_AFFINITY: "1"
      RT_PRIORITY: "80"
```

---

## メッセージフォーマット

### motor_commands (state_manager → moteus_driver)
```
[1 byte: count]
[per axis: 1 byte mode + 8 bytes position + 8 bytes velocity + 8 bytes kp + 8 bytes kd]
= 1 + 33 * n bytes
```

### motor_status (moteus_driver → state_manager)
```
[1 byte: count]
[per axis: 8 bytes position + 8 bytes velocity + 8 bytes torque + 1 byte mode + 1 byte fault]
= 1 + 26 * n bytes
```

### robot_config (GUI → moteus_driver, state_manager)
JSON形式（変更なし）

---

## 削除するファイル

- `cpp/communications/comm_manager/moteus_communication/` (全体)
- `cpp/communications/canfd/canfd_txrx/` (全体)

---

## 新規作成ファイル

```
cpp/motor/moteus_driver/
├── main.cc           # メインノード
├── can_interface.cc  # CAN通信 (canfd_txrxから移植)
├── can_interface.hpp
├── moteus_protocol.cc # フレーム構築 (moteus_communicationから移植)
├── moteus_protocol.hpp
├── Makefile
└── README.md
```

---

## 移行手順

1. **moteus_driver骨格作成** - CAN初期化、基本ループ
2. **can_interfaceの移植** - canfd_txrxからCAN送受信コードを移植
3. **moteus_protocolの移植** - フレーム構築関数を移植
4. **統合テスト** - 単体でCAN通信確認
5. **state_manager修正** - motor_commands出力に変更
6. **dataflow更新** - 新構成でテスト
7. **旧ノード削除** - moteus_communication, canfd_txrx削除

---

## 期待される改善

| 項目 | 現在 | 改善後 |
|------|------|--------|
| 1サイクルのメッセージ数 | 7+ | 3 |
| ノード間ホップ数 | 3 (SM→MC→CAN→MC) | 1 (SM→Driver) |
| タイミング問題 | position_commands/query分離 | 統合メッセージで解消 |
| コード量 | ~1000行 (2ノード) | ~600行 (1ノード) |
| レイテンシ | ~4.5ms | ~3ms |

---

## リスクと対策

| リスク | 対策 |
|--------|------|
| CAN通信が動かない | canfd_txrxのコードを慎重に移植、段階的テスト |
| servo_on_delayが壊れる | 現在のロジックをそのまま移植 |
| 単位変換ミス | 既存の変換関数を流用 |
| RT優先度問題 | 同じ設定を維持 |

---

## 確認事項

1. **ControlModeの定義**: SERVO_OFF, STOP, POSITION, VELOCITYの4種類で十分か？
2. **set_positionの扱い**: 現状維持（OutputExact）でよいか？
3. **GUI側の変更**: motor_statusフォーマット変更に対応必要
4. **IMUノード**: 変更なし（独立したまま）
