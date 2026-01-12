#include "state_run.hpp"
#include "types.hpp"
#include "helpers.hpp"
#include "../../control/pid_controller/pid_control.hpp"
#include <iostream>
#include <vector>

// PIDコントローラ（倒立制御用）
// 初期ゲイン: Kp=5.0, Ki=0.0, Kd=0.1 (控えめな値から開始)
static PidController g_balance_pid(5.0, 0.0, 0.1);

// ホイール軸インデックス
static constexpr int WHEEL_R_INDEX = 2;
static constexpr int WHEEL_L_INDEX = 5;

// 制御パラメータ
static constexpr double DT = 0.005;  // 5ms tick

void cmd_run(void* dora_context)
{
    if (g_current_state == State::READY && !g_interpolating)
    {
        g_current_state = State::RUN;
        g_holding = false;
        g_balance_pid.reset();  // PIDをリセット
        send_state_status(dora_context, 100);
        std::cout << "[state_manager] RUN: Running (PID balance control)" << std::endl;
    }
}

void tick_run(void* dora_context)
{
    // IMUデータが有効でない場合は安全のため初期位置保持（速度0）
    if (!g_imu_data.valid)
    {
        std::vector<AxisCommand> commands(g_axes.size());
        for (size_t i = 0; i < g_axes.size(); ++i)
        {
            if (i == WHEEL_R_INDEX || i == WHEEL_L_INDEX)
            {
                commands[i] = AxisCommand::velocity_control(0.0);
            }
            else
            {
                commands[i] = AxisCommand::position_control(g_axes[i].initial_position);
            }
        }
        send_motor_commands(dora_context, commands);
        return;
    }

    // 倒立制御: pitch角をゼロに保つ
    // 目標: pitch = 0 (直立)
    double pitch_error = 0.0 - g_imu_data.pitch;

    // PID出力を計算 → ホイール速度指令 (rad/s)
    double wheel_velocity_rad_s = g_balance_pid.compute(pitch_error, DT);

    // 全軸分のコマンドを作成
    std::vector<AxisCommand> commands(g_axes.size());
    for (size_t i = 0; i < g_axes.size(); ++i)
    {
        if (i == WHEEL_R_INDEX)
        {
            // wheel_rは正方向
            commands[i] = AxisCommand::velocity_control(wheel_velocity_rad_s);
        }
        else if (i == WHEEL_L_INDEX)
        {
            // wheel_lは逆方向
            commands[i] = AxisCommand::velocity_control(-wheel_velocity_rad_s);
        }
        else
        {
            // 股・膝は初期位置を保持
            commands[i] = AxisCommand::position_control(g_axes[i].initial_position);
        }
    }
    send_motor_commands(dora_context, commands);
}
