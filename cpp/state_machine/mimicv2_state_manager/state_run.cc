#include "state_run.hpp"
#include "types.hpp"
#include "helpers.hpp"
#include "../../control/pid_controller/pid_control.hpp"
#include <iostream>
#include <vector>
#include <utility>

// PIDコントローラ（倒立制御用）
// 初期ゲイン: Kp=5.0, Ki=0.0, Kd=0.1 (控えめな値から開始)
static PidController g_balance_pid(5.0, 0.0, 0.1);

// ホイール軸インデックス
static constexpr int WHEEL_R_INDEX = 2;
static constexpr int WHEEL_L_INDEX = 5;

// 制御パラメータ
static constexpr double DT = 0.005;  // 5ms tick
static constexpr double KP_SCALE = 0.0;  // 純粋な速度制御
static constexpr double KD_SCALE = 1.0;

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
    // IMUデータが有効でない場合は安全のため初期位置保持
    if (!g_imu_data.valid)
    {
        std::vector<double> positions(g_axes.size());
        for (size_t i = 0; i < g_axes.size(); ++i)
        {
            positions[i] = g_axes[i].initial_position;
        }
        send_position_commands(dora_context, positions);
        return;
    }

    // 倒立制御: pitch角をゼロに保つ
    // 目標: pitch = 0 (直立)
    double pitch_error = 0.0 - g_imu_data.pitch;

    // PID出力を計算 → ホイール速度指令 (rad/s)
    double wheel_velocity_rad_s = g_balance_pid.compute(pitch_error, DT);

    // ホイールに速度指令 (rad/s)
    // wheel_rとwheel_lは左右逆向きに回転
    std::vector<std::pair<int, double>> wheel_cmds = {
        {WHEEL_R_INDEX, wheel_velocity_rad_s},
        {WHEEL_L_INDEX, -wheel_velocity_rad_s}
    };
    send_velocity_commands(dora_context, wheel_cmds, KP_SCALE, KD_SCALE);

    // 他の軸は初期位置を保持
    std::vector<double> positions(g_axes.size());
    for (size_t i = 0; i < g_axes.size(); ++i)
    {
        positions[i] = g_axes[i].initial_position;
    }
    send_position_commands(dora_context, positions);
}
