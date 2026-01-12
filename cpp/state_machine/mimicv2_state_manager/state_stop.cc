#include "state_stop.hpp"
#include "types.hpp"
#include "helpers.hpp"
#include <iostream>
#include <limits>

void cmd_stop(void* dora_context)
{
    g_interpolating = false;
    g_holding = true;

    if (g_current_state == State::SERVO_OFF)
    {
        // Initialize hold_positions to NaN (will hold current on first motor_status)
        for (size_t i = 0; i < g_axes.size(); ++i)
        {
            g_hold_positions[i] = std::numeric_limits<double>::quiet_NaN();
        }
    }
    else
    {
        // From other states: Hold at current position immediately
        for (size_t i = 0; i < g_axes.size(); ++i)
        {
            g_hold_positions[i] = g_axes[i].current_position;
        }
    }

    g_current_state = State::STOP;
    send_state_status(dora_context, 100);
    std::cout << "[state_manager] STOP: Holding at current position" << std::endl;
}

void tick_hold(void* dora_context)
{
    // STOP状態では全軸STOPモード（現在位置保持）
    std::vector<AxisCommand> commands(g_axes.size());
    for (size_t i = 0; i < g_axes.size(); ++i)
    {
        commands[i] = AxisCommand::stop();
    }
    send_motor_commands(dora_context, commands);
}
