extern "C"
{
#include "node_api.h"
}

#include "helpers.hpp"
#include "types.hpp"
#include <iostream>
#include <cstring>
#include <vector>

// motor_commands を送信
// Format: [1 byte count][per axis: mode(1) + position(8) + velocity(8) + kp(8) + kd(8)]
void send_motor_commands(void* dora_context, const std::vector<AxisCommand>& commands)
{
    size_t per_axis = 1 + 4 * sizeof(double);  // mode + position + velocity + kp + kd
    std::vector<uint8_t> buffer(1 + commands.size() * per_axis);
    size_t offset = 0;

    buffer[offset++] = static_cast<uint8_t>(commands.size());
    for (const auto& cmd : commands)
    {
        buffer[offset++] = static_cast<uint8_t>(cmd.mode);
        std::memcpy(buffer.data() + offset, &cmd.position, sizeof(double));
        offset += sizeof(double);
        std::memcpy(buffer.data() + offset, &cmd.velocity, sizeof(double));
        offset += sizeof(double);
        std::memcpy(buffer.data() + offset, &cmd.kp_scale, sizeof(double));
        offset += sizeof(double);
        std::memcpy(buffer.data() + offset, &cmd.kd_scale, sizeof(double));
        offset += sizeof(double);
    }

    std::string output_id = "motor_commands";
    dora_send_output(dora_context,
                     const_cast<char*>(output_id.c_str()), output_id.length(),
                     reinterpret_cast<char*>(buffer.data()), offset);
}

void send_set_position(void* dora_context, const std::vector<double>& positions)
{
    std::vector<uint8_t> buffer(1 + positions.size() * sizeof(double));
    size_t offset = 0;

    buffer[offset++] = static_cast<uint8_t>(positions.size());
    for (const auto& pos : positions)
    {
        std::memcpy(buffer.data() + offset, &pos, sizeof(double));
        offset += sizeof(double);
    }

    std::string output_id = "set_position";
    dora_send_output(dora_context,
                     const_cast<char*>(output_id.c_str()), output_id.length(),
                     reinterpret_cast<char*>(buffer.data()), offset);
}

void send_state_status(void* dora_context, uint8_t progress)
{
    uint8_t buffer[2];
    buffer[0] = static_cast<uint8_t>(g_current_state);
    buffer[1] = progress;

    std::string output_id = "state_status";
    dora_send_output(dora_context,
                     const_cast<char*>(output_id.c_str()), output_id.length(),
                     reinterpret_cast<char*>(buffer), 2);
}

void send_motor_display(void* dora_context)
{
    size_t n = g_axes.size();
    // Format: [1 byte count][positions (n*8)][torques (n*8)][faults (n*1)]
    std::vector<uint8_t> buffer(1 + n * 16 + n);
    size_t offset = 0;

    buffer[offset++] = static_cast<uint8_t>(n);

    for (const auto& axis : g_axes)
    {
        std::memcpy(buffer.data() + offset, &axis.current_position, sizeof(double));
        offset += sizeof(double);
    }

    for (const auto& axis : g_axes)
    {
        std::memcpy(buffer.data() + offset, &axis.current_torque, sizeof(double));
        offset += sizeof(double);
    }

    for (const auto& axis : g_axes)
    {
        buffer[offset++] = axis.fault;
    }

    std::string output_id = "motor_display";
    dora_send_output(dora_context,
                     const_cast<char*>(output_id.c_str()), output_id.length(),
                     reinterpret_cast<char*>(buffer.data()), offset);
}

void forward_robot_config(void* dora_context, const char* data, size_t len)
{
    std::string output_id = "robot_config_out";
    dora_send_output(dora_context,
                     const_cast<char*>(output_id.c_str()), output_id.length(),
                     const_cast<char*>(data), len);
}
