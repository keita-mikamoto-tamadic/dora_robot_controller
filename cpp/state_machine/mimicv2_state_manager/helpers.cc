extern "C"
{
#include "node_api.h"
}

#include "helpers.hpp"
#include "types.hpp"
#include <iostream>
#include <cstring>
#include <vector>

void send_servo_on(void* dora_context, uint8_t mask)
{
    std::string output_id = "servo_on";
    char data[1] = {static_cast<char>(mask)};
    dora_send_output(dora_context,
                     const_cast<char*>(output_id.c_str()), output_id.length(),
                     data, 1);
    std::cout << "[state_manager] Servo ON mask: 0x" << std::hex << static_cast<int>(mask) << std::dec << std::endl;
}

void send_servo_off(void* dora_context, uint8_t mask)
{
    std::string output_id = "servo_off";
    char data[1] = {static_cast<char>(mask)};
    dora_send_output(dora_context,
                     const_cast<char*>(output_id.c_str()), output_id.length(),
                     data, 1);
    std::cout << "[state_manager] Servo OFF mask: 0x" << std::hex << static_cast<int>(mask) << std::dec << std::endl;
}

void send_position_command(void* dora_context, uint8_t axis_index, double position)
{
    uint8_t buffer[9];
    buffer[0] = axis_index;
    std::memcpy(buffer + 1, &position, sizeof(double));

    std::string output_id = "position_command";
    dora_send_output(dora_context,
                     const_cast<char*>(output_id.c_str()), output_id.length(),
                     reinterpret_cast<char*>(buffer), 9);
}

void send_position_commands(void* dora_context, const std::vector<double>& positions)
{
    // Format: [1 byte count][pos0 (8 bytes)][pos1 (8 bytes)]...
    std::vector<uint8_t> buffer(1 + positions.size() * sizeof(double));
    size_t offset = 0;

    buffer[offset++] = static_cast<uint8_t>(positions.size());
    for (const auto& pos : positions)
    {
        std::memcpy(buffer.data() + offset, &pos, sizeof(double));
        offset += sizeof(double);
    }

    std::string output_id = "position_commands";
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
    std::vector<uint8_t> buffer(1 + n * 16);
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

    std::string output_id = "motor_display";
    dora_send_output(dora_context,
                     const_cast<char*>(output_id.c_str()), output_id.length(),
                     reinterpret_cast<char*>(buffer.data()), offset);
}

void send_query(void* dora_context)
{
    std::string query_id = "query";
    uint8_t query_data[1] = {0xFF};
    dora_send_output(dora_context,
                     const_cast<char*>(query_id.c_str()), query_id.length(),
                     reinterpret_cast<char*>(query_data), 1);
}

void forward_robot_config(void* dora_context, const char* data, size_t len)
{
    std::string output_id = "robot_config_out";
    dora_send_output(dora_context,
                     const_cast<char*>(output_id.c_str()), output_id.length(),
                     const_cast<char*>(data), len);
}
