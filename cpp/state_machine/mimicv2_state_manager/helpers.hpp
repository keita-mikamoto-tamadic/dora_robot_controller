#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>
#include <utility>
#include <cmath>

// 軸ごとの制御コマンド
// 位置制御: position=目標位置, velocity=0, kp_scale=1, kd_scale=1
// 速度制御: position=NaN, velocity=目標速度, kp_scale=0, kd_scale=1
struct AxisCommand
{
    double position;   // 目標位置 (rad)、速度制御時はNaN
    double velocity;   // 目標速度 (rad/s)
    double kp_scale;   // 位置ゲインスケール (0で位置制御無効)
    double kd_scale;   // 速度ゲインスケール

    // 位置制御用コンストラクタ
    static AxisCommand position_control(double pos)
    {
        return {pos, 0.0, 1.0, 1.0};
    }

    // 速度制御用コンストラクタ
    static AxisCommand velocity_control(double vel, double kd = 30.0)
    {
        return {std::nan(""), vel, 0.0, kd};
    }
};

void send_servo_on(void* dora_context, uint8_t mask);
void send_servo_off(void* dora_context, uint8_t mask);
void send_position_commands(void* dora_context, const std::vector<double>& positions);
void send_position_commands(void* dora_context, const std::vector<AxisCommand>& commands);
void send_set_position(void* dora_context, const std::vector<double>& positions);
void send_state_status(void* dora_context, uint8_t progress = 0);
void send_motor_display(void* dora_context);
void send_query(void* dora_context);
void forward_robot_config(void* dora_context, const char* data, size_t len);
