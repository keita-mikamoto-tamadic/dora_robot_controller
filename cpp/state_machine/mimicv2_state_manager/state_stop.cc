#include "state_stop.hpp"
#include "types.hpp"
#include "helpers.hpp"
#include <iostream>

void cmd_stop(void* dora_context)
{
    uint8_t all_mask = (1 << g_axes.size()) - 1;

    g_interpolating = false;
    g_holding = true;

    for (size_t i = 0; i < g_axes.size(); ++i)
    {
        g_hold_positions[i] = g_axes[i].current_position;
    }

    if (g_current_state == State::SERVO_OFF)
    {
        send_servo_on(dora_context, all_mask);
    }

    g_current_state = State::READY;
    send_state_status(dora_context, 100);
    std::cout << "[state_manager] STOP: Holding at current position" << std::endl;
}

void tick_hold(void* dora_context)
{
    send_position_commands(dora_context, g_hold_positions);
}
