#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>
#include <utility>
#include <cmath>

// Control mode (matches moteus_driver)
enum class ControlMode : uint8_t
{
    SERVO_OFF = 0,
    STOP = 1,      // NaN position (hold current)
    POSITION = 2,  // Position control
    VELOCITY = 3   // Velocity control
};

// 軸ごとの制御コマンド
struct AxisCommand
{
    ControlMode mode;
    double position;   // 目標位置 (rad)、速度制御時は無視
    double velocity;   // 目標速度 (rad/s)
    double kp_scale;   // 位置ゲインスケール
    double kd_scale;   // 速度ゲインスケール

    // サーボオフ
    static AxisCommand servo_off()
    {
        return {ControlMode::SERVO_OFF, 0.0, 0.0, 0.0, 0.0};
    }

    // 停止（現在位置保持）
    static AxisCommand stop()
    {
        return {ControlMode::STOP, 0.0, 0.0, 1.0, 1.0};
    }

    // 位置制御
    static AxisCommand position_control(double pos)
    {
        return {ControlMode::POSITION, pos, 0.0, 1.0, 1.0};
    }

    // 速度制御
    static AxisCommand velocity_control(double vel, double kd = 1.0)
    {
        return {ControlMode::VELOCITY, 0.0, vel, 0.0, kd};
    }
};

// motor_commands を送信 (position_commands + query を統合)
void send_motor_commands(void* dora_context, const std::vector<AxisCommand>& commands);

// set_position (エンコーダゼロ点設定)
void send_set_position(void* dora_context, const std::vector<double>& positions);

// 状態ステータス送信
void send_state_status(void* dora_context, uint8_t progress = 0);

// モーター表示情報送信
void send_motor_display(void* dora_context);

// robot_config転送
void forward_robot_config(void* dora_context, const char* data, size_t len);
