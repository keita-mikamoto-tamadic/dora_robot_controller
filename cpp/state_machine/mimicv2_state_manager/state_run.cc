#include "state_run.hpp"
#include "types.hpp"
#include "helpers.hpp"
#include <iostream>
#include <vector>

void cmd_run(void* dora_context)
{
    if (g_current_state == State::READY && !g_interpolating)
    {
        g_current_state = State::RUN;
        g_holding = false;
        send_state_status(dora_context, 100);
        std::cout << "[state_manager] RUN: Running" << std::endl;
    }
}

void tick_run(void* dora_context)
{
    // 将来の制御ロジック用
    // 今は初期位置を保持
    std::vector<double> positions(g_axes.size());
    for (size_t i = 0; i < g_axes.size(); ++i)
    {
        positions[i] = g_axes[i].initial_position;
    }
    send_position_commands(dora_context, positions);
}
