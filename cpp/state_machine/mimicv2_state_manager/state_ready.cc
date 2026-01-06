#include "state_ready.hpp"
#include "types.hpp"
#include "helpers.hpp"
#include "timing_manager.hpp"
#include <iostream>
#include <vector>

extern TimingManager g_timing;

// ホイール軸インデックス
static constexpr int WHEEL_R_INDEX = 2;
static constexpr int WHEEL_L_INDEX = 5;

void start_interpolation()
{
    g_interp_start_positions.resize(g_axes.size());
    g_interp_target_positions.resize(g_axes.size());

    for (size_t i = 0; i < g_axes.size(); ++i)
    {
        g_interp_start_positions[i] = g_axes[i].current_position;
        g_interp_target_positions[i] = g_axes[i].initial_position;
    }

    g_timing.reset();
    g_interpolating = true;
    g_holding = false;

    std::cout << "[state_manager] Interpolation start (" << g_interpolation_time << "s)" << std::endl;
}

double interpolate(double start, double target, double t)
{
    if (t >= 1.0) return target;
    if (t <= 0.0) return start;
    return start + (target - start) * t;
}

void cmd_ready(void* dora_context)
{
    uint8_t all_mask = (1 << g_axes.size()) - 1;

    if (g_current_state == State::SERVO_OFF)
    {
        send_servo_on(dora_context, all_mask);
    }

    g_current_state = State::READY;
    g_holding = false;
    start_interpolation();
    send_state_status(dora_context, 0);
    std::cout << "[state_manager] READY: Moving to initial positions" << std::endl;
}

void tick_interpolation(void* dora_context)
{
    double elapsed = g_timing.getElapsedTime();
    double t = elapsed / g_interpolation_time;
    if (t > 1.0) t = 1.0;

    // 全軸分のコマンドを作成
    std::vector<AxisCommand> commands(g_axes.size());
    for (size_t i = 0; i < g_axes.size(); ++i)
    {
        if (i == WHEEL_R_INDEX || i == WHEEL_L_INDEX)
        {
            // ホイールは速度制御（指令値0）
            commands[i] = AxisCommand::velocity_control(0.0);
        }
        else
        {
            // 股・膝は位置制御で補間
            double pos = interpolate(g_interp_start_positions[i], g_interp_target_positions[i], t);
            commands[i] = AxisCommand::position_control(pos);
        }
    }
    send_position_commands(dora_context, commands);

    uint8_t progress = static_cast<uint8_t>(t * 100);
    send_state_status(dora_context, progress);

    if (t >= 1.0)
    {
        g_interpolating = false;
        std::cout << "[state_manager] Interpolation complete" << std::endl;
        send_state_status(dora_context, 100);
    }
}

void tick_ready_hold(void* dora_context)
{
    // 全軸分のコマンドを作成
    std::vector<AxisCommand> commands(g_axes.size());
    for (size_t i = 0; i < g_axes.size(); ++i)
    {
        if (i == WHEEL_R_INDEX || i == WHEEL_L_INDEX)
        {
            // ホイールは速度制御（指令値0）
            commands[i] = AxisCommand::velocity_control(0.0);
        }
        else
        {
            // 股・膝は位置制御で初期姿勢保持
            commands[i] = AxisCommand::position_control(g_axes[i].initial_position);
        }
    }
    send_position_commands(dora_context, commands);
}
