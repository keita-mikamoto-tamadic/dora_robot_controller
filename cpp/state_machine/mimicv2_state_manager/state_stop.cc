#include "state_stop.hpp"
#include "types.hpp"
#include "helpers.hpp"
#include <iostream>
#include <limits>

void cmd_stop(void* dora_context)
{
    uint8_t all_mask = (1 << g_axes.size()) - 1;

    g_interpolating = false;
    g_holding = true;  // Enable position commands in STOP state

    if (g_current_state == State::SERVO_OFF)
    {
        // From SERVO_OFF: Send servo_on to enable servos
        // moteus_communication will use servo_on_delay for safe startup
        send_servo_on(dora_context, all_mask);

        // Initialize hold_positions to NaN
        // Will be set from current_position after servo_on_delay expires
        for (size_t i = 0; i < g_axes.size(); ++i)
        {
            g_hold_positions[i] = std::numeric_limits<double>::quiet_NaN();
        }
    }
    else
    {
        // From other states (READY, RUN): Hold at current position immediately
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
    send_position_commands(dora_context, g_hold_positions);
}
