extern "C"
{
#include "node_api.h"
}

#include <iostream>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <limits>
#include <vector>
#include <string>
#include <sched.h>

#include "mjbots/moteus/moteus_protocol.h"
#include "nlohmann/json.hpp"

using namespace mjbots::moteus;
using json = nlohmann::json;

// Axis configuration
struct AxisConfig
{
    int index;
    std::string name;
    int device_id;
    int motdir;  // Motor direction: 1 or -1
    double hold_position;  // Current hold position
    double current_position;  // Latest position from moteus (raw, no motdir)
    double velocity_limit;  // rev/s
    double accel_limit;     // rev/s^2
    double torque_limit;    // Nm
};

// Static configuration (set once on robot_config input)
static std::vector<AxisConfig> g_axes;
static bool g_config_received = false;

// Build position command frame for moteus
size_t build_position_frame(uint8_t* buffer, int motor_id, double position, double velocity,
                            double velocity_limit, double accel_limit, double torque_limit)
{
    CanData frame;
    WriteCanData writer(&frame);

    // Position mode command
    PositionMode::Command cmd;
    cmd.position = position;
    cmd.velocity = velocity;
    cmd.maximum_torque = torque_limit;
    cmd.feedforward_torque = 0.0;
    cmd.kp_scale = 1.0;
    cmd.kd_scale = 1.0;
    cmd.stop_position = std::numeric_limits<double>::quiet_NaN();
    cmd.watchdog_timeout = std::numeric_limits<double>::quiet_NaN();
    cmd.velocity_limit = velocity_limit;
    cmd.accel_limit = accel_limit;

    PositionMode::Format fmt;
    fmt.maximum_torque = Resolution::kFloat;
    fmt.velocity_limit = Resolution::kFloat;
    fmt.accel_limit = Resolution::kFloat;
    PositionMode::Make(&writer, cmd, fmt);

    // Also request query with extended data
    Query::Format query_fmt;
    query_fmt.q_current = Resolution::kFloat;
    query_fmt.d_current = Resolution::kFloat;
    query_fmt.motor_temperature = Resolution::kFloat;
    Query::Make(&writer, query_fmt);

    // Pack: [4 bytes arb_id][1 byte len][data...]
    size_t offset = 0;
    uint32_t arb_id = 0x8000 | motor_id;
    std::memcpy(buffer + offset, &arb_id, 4);
    offset += 4;

    buffer[offset] = frame.size;
    offset += 1;

    std::memcpy(buffer + offset, frame.data, frame.size);
    offset += frame.size;

    return offset;
}

// Build stop command frame using StopMode from moteus_protocol.h
size_t build_stop_frame(uint8_t* buffer, int motor_id)
{
    CanData frame;
    WriteCanData writer(&frame);

    // Use the official StopMode::Make
    StopMode::Command cmd;
    StopMode::Format fmt;
    StopMode::Make(&writer, cmd, fmt);

    // Also request query with extended data (so we get status even when stopped)
    Query::Format query_fmt;
    query_fmt.q_current = Resolution::kFloat;
    query_fmt.d_current = Resolution::kFloat;
    query_fmt.motor_temperature = Resolution::kFloat;
    Query::Make(&writer, query_fmt);

    // Pack: [4 bytes arb_id][1 byte len][data...]
    size_t offset = 0;
    uint32_t arb_id = 0x8000 | motor_id;
    std::memcpy(buffer + offset, &arb_id, 4);
    offset += 4;

    buffer[offset] = frame.size;
    offset += 1;

    std::memcpy(buffer + offset, frame.data, frame.size);
    offset += frame.size;

    return offset;
}

// Build set output exact frame (set zero point)
size_t build_set_output_exact_frame(uint8_t* buffer, int motor_id, double position)
{
    CanData frame;
    WriteCanData writer(&frame);

    // Use OutputExact to set current position exactly (no rounding)
    OutputExact::Command cmd;
    cmd.position = position;
    OutputExact::Format fmt;
    OutputExact::Make(&writer, cmd, fmt);

    // Pack: [4 bytes arb_id][1 byte len][data...]
    size_t offset = 0;
    uint32_t arb_id = 0x8000 | motor_id;
    std::memcpy(buffer + offset, &arb_id, 4);
    offset += 4;

    buffer[offset] = frame.size;
    offset += 1;

    std::memcpy(buffer + offset, frame.data, frame.size);
    offset += frame.size;

    return offset;
}

// Build axis_config message for canfd_txrx
// Format: [1 byte axis_count][device_id 1][device_id 2]...
void send_axis_config(void* dora_context)
{
    uint8_t buffer[256];
    size_t offset = 0;

    buffer[offset++] = static_cast<uint8_t>(g_axes.size());
    for (const auto& axis : g_axes)
    {
        buffer[offset++] = static_cast<uint8_t>(axis.device_id);
    }

    std::string output_id = "axis_config";
    dora_send_output(dora_context,
                     const_cast<char*>(output_id.c_str()), output_id.length(),
                     reinterpret_cast<char*>(buffer), offset);
}
// Build stop frames for all axes
void send_all_stop_frames(void* dora_context)
{
    uint8_t buffer[1024];
    size_t offset = 0;

    buffer[offset++] = static_cast<uint8_t>(g_axes.size());

    for (const auto& axis : g_axes)
    {
        size_t frame_len = build_stop_frame(buffer + offset, axis.device_id);
        offset += frame_len;
    }

    std::string output_id = "can_frames";
    dora_send_output(dora_context,
                     const_cast<char*>(output_id.c_str()), output_id.length(),
                     reinterpret_cast<char*>(buffer), offset);
}

int main()
{
    std::cout << "[moteus_communication] Starting" << std::endl;

    // Set CPU affinity if specified
    const char* cpu_affinity_env = std::getenv("CPU_AFFINITY");
    if (cpu_affinity_env) {
        int cpu_id = std::atoi(cpu_affinity_env);
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(cpu_id, &cpuset);
        if (sched_setaffinity(0, sizeof(cpuset), &cpuset) == 0) {
            std::cout << "[moteus_communication] CPU affinity set to core " << cpu_id << std::endl;
        } else {
            std::cerr << "[moteus_communication] Failed to set CPU affinity to core " << cpu_id << std::endl;
        }
    }

    // Set RT priority (SCHED_FIFO) if specified
    const char* rt_priority_env = std::getenv("RT_PRIORITY");
    if (rt_priority_env) {
        int priority = std::atoi(rt_priority_env);
        struct sched_param param;
        param.sched_priority = priority;
        if (sched_setscheduler(0, SCHED_FIFO, &param) == 0) {
            std::cout << "[moteus_communication] RT priority set to SCHED_FIFO " << priority << std::endl;
        } else {
            std::cerr << "[moteus_communication] Failed to set RT priority (need CAP_SYS_NICE)" << std::endl;
        }
    }

    auto dora_context = init_dora_context_from_env();
    if (dora_context == NULL)
    {
        std::cerr << "[moteus_communication] Failed to init dora context" << std::endl;
        return 1;
    }

    uint8_t servo_on_mask = 0;  // Bitmask for servo on/off per axis

    std::cout << "[moteus_communication] Waiting for robot_config..." << std::endl;

    while (true)
    {
        void *event = dora_next_event(dora_context);
        if (event == NULL)
        {
            break;
        }

        enum DoraEventType ty = read_dora_event_type(event);

        if (ty == DoraEventType_Stop)
        {
            std::cout << "[moteus_communication] Received stop signal" << std::endl;
            if (g_config_received)
            {
                send_all_stop_frames(dora_context);
            }
            free_dora_event(event);
            break;
        }
        else if (ty == DoraEventType_Input)
        {
            char *id_ptr;
            size_t id_len;
            read_dora_input_id(event, &id_ptr, &id_len);
            std::string input_id(id_ptr, id_len);

            char *data_ptr;
            size_t data_len;
            read_dora_input_data(event, &data_ptr, &data_len);

            if (input_id == "robot_config")
            {
                // Parse JSON config from CUI
                if (!g_config_received && data_len > 0)
                {
                    try
                    {
                        std::string json_str(data_ptr, data_len);
                        json config = json::parse(json_str);

                        g_axes.clear();
                        for (const auto& axis_json : config["axes"])
                        {
                            AxisConfig axis;
                            axis.index = axis_json["index"].get<int>();
                            axis.name = axis_json["name"].get<std::string>();
                            axis.device_id = axis_json["device_id"].get<int>();
                            axis.motdir = axis_json["motdir"].get<int>();
                            axis.hold_position = std::numeric_limits<double>::quiet_NaN();
                            axis.current_position = std::numeric_limits<double>::quiet_NaN();
                            // Per-axis limits (with defaults)
                            axis.velocity_limit = axis_json.value("velocity_limit", 2.0);
                            axis.accel_limit = axis_json.value("accel_limit", 10.0);
                            axis.torque_limit = axis_json.value("torque_limit", 0.5);
                            g_axes.push_back(axis);
                        }

                        g_config_received = true;
                        std::cout << "[moteus_communication] Config received: "
                                  << config["robot_name"].get<std::string>()
                                  << " (" << g_axes.size() << " axes)" << std::endl;

                        for (const auto& axis : g_axes)
                        {
                            std::cout << "  - " << axis.name
                                      << ": device_id=" << axis.device_id
                                      << ", motdir=" << axis.motdir
                                      << ", limits(v/a/t)=" << axis.velocity_limit
                                      << "/" << axis.accel_limit
                                      << "/" << axis.torque_limit << std::endl;
                        }

                        // Send axis_config to canfd_txrx
                        send_axis_config(dora_context);
                        std::cout << "[moteus_communication] Sent axis_config to canfd_txrx" << std::endl;
                    }
                    catch (const std::exception& e)
                    {
                        std::cerr << "[moteus_communication] Failed to parse config: " << e.what() << std::endl;
                    }
                }
            }
            else if (input_id == "servo_on")
            {
                if (g_config_received && data_len >= 1)
                {
                    uint8_t mask = static_cast<uint8_t>(data_ptr[0]);
                    // Reset hold positions for newly enabled axes
                    for (size_t i = 0; i < g_axes.size() && i < 8; ++i)
                    {
                        if ((mask & (1 << i)) && !(servo_on_mask & (1 << i)))
                        {
                            g_axes[i].hold_position = std::numeric_limits<double>::quiet_NaN();
                        }
                    }
                    servo_on_mask |= mask;
                    std::cout << "[moteus_communication] Servo ON mask: 0x"
                              << std::hex << static_cast<int>(servo_on_mask) << std::dec << std::endl;
                }
            }
            else if (input_id == "servo_off")
            {
                if (g_config_received && data_len >= 1)
                {
                    uint8_t mask = static_cast<uint8_t>(data_ptr[0]);
                    servo_on_mask &= ~mask;
                    std::cout << "[moteus_communication] Servo OFF mask: 0x"
                              << std::hex << static_cast<int>(servo_on_mask) << std::dec << std::endl;
                }
            }
            else if (input_id == "set_zero")
            {
                // Set zero point for selected axes
                // Format: [1 byte mask]
                if (g_config_received && data_len >= 1)
                {
                    uint8_t mask = static_cast<uint8_t>(data_ptr[0]);
                    std::cout << "[moteus_communication] Set zero mask: 0x"
                              << std::hex << static_cast<int>(mask) << std::dec << std::endl;

                    uint8_t buffer[1024];
                    size_t offset = 1;  // Reserve first byte for count
                    uint8_t count = 0;

                    // Build set_output_nearest frames for selected axes
                    // Send current position so that it becomes the new zero point
                    for (size_t i = 0; i < g_axes.size() && i < 8; ++i)
                    {
                        if (mask & (1 << i))
                        {
                            // Send 0.0 to set current position as zero
                            offset += build_set_output_exact_frame(buffer + offset, g_axes[i].device_id, 0.0);
                            count++;
                            std::cout << "[moteus_communication] Set zero for axis " << i
                                      << " (device_id=" << g_axes[i].device_id << ")" << std::endl;
                        }
                    }
                    buffer[0] = count;  // Write actual count

                    if (count > 0)
                    {
                        std::string output_id = "can_frames";
                        dora_send_output(dora_context,
                                         const_cast<char*>(output_id.c_str()), output_id.length(),
                                         reinterpret_cast<char*>(buffer), offset);
                    }
                }
            }
            else if (input_id == "position_command")
            {
                // Position command for a single axis
                // Format: [1 byte axis_index][8 bytes position (double)]
                if (g_config_received && data_len >= 9)
                {
                    uint8_t axis_index = static_cast<uint8_t>(data_ptr[0]);
                    double position;
                    std::memcpy(&position, data_ptr + 1, sizeof(double));

                    if (axis_index < g_axes.size())
                    {
                        g_axes[axis_index].hold_position = position;
                    }
                }
            }
            else if (input_id == "position_commands")
            {
                // Batch position commands for all axes
                // Format: [1 byte count][pos0 (8 bytes)][pos1 (8 bytes)]...
                if (g_config_received && data_len >= 1)
                {
                    uint8_t count = static_cast<uint8_t>(data_ptr[0]);
                    size_t offset = 1;
                    for (uint8_t i = 0; i < count && i < g_axes.size(); ++i)
                    {
                        if (offset + sizeof(double) > data_len) break;
                        double position;
                        std::memcpy(&position, data_ptr + offset, sizeof(double));
                        g_axes[i].hold_position = position;
                        offset += sizeof(double);
                    }
                }
            }
            else if (input_id == "tick" || input_id == "query")
            {
                if (g_config_received)
                {
                    // Send frames for all axes: position for servo_on, stop for servo_off
                    uint8_t buffer[1024];
                    size_t offset = 0;
                    buffer[offset++] = static_cast<uint8_t>(g_axes.size());

                    for (size_t i = 0; i < g_axes.size(); ++i)
                    {
                        if (servo_on_mask & (1 << i))
                        {
                            // Position command with per-axis limits
                            double pos = g_axes[i].hold_position;
                            if (!std::isnan(pos))
                            {
                                pos *= g_axes[i].motdir;
                            }
                            offset += build_position_frame(buffer + offset, g_axes[i].device_id, pos, 0.0,
                                                           g_axes[i].velocity_limit,
                                                           g_axes[i].accel_limit,
                                                           g_axes[i].torque_limit);
                        }
                        else
                        {
                            // Stop command
                            offset += build_stop_frame(buffer + offset, g_axes[i].device_id);
                        }
                    }

                    std::string output_id = "can_frames";
                    dora_send_output(dora_context,
                                     const_cast<char*>(output_id.c_str()), output_id.length(),
                                     reinterpret_cast<char*>(buffer), offset);
                }
            }
            else if (input_id == "rx_frames")
            {
                // RX frames from canfd_txrx
                // Format: [1 byte frame_count][frame1...][frame2...]
                // Each frame: [8 bytes timestamp][4 bytes arb_id][1 byte len][data...]
                if (g_config_received && data_len >= 1)
                {
                    uint8_t frame_count = static_cast<uint8_t>(data_ptr[0]);
                    size_t offset = 1;

                    // Per-axis data: mode, position, velocity, d_current, q_current, torque, motor_temp
                    struct AxisStatus {
                        uint8_t mode;
                        double position;
                        double velocity;
                        double d_current;
                        double q_current;
                        double torque;
                        double motor_temp;
                    };
                    std::vector<AxisStatus> statuses(g_axes.size());
                    for (auto& s : statuses) {
                        s.mode = 0;
                        s.position = 0.0;
                        s.velocity = 0.0;
                        s.d_current = 0.0;
                        s.q_current = 0.0;
                        s.torque = 0.0;
                        s.motor_temp = 0.0;
                    }

                    for (uint8_t i = 0; i < frame_count && offset < data_len; ++i)
                    {
                        if (offset + 13 > data_len) break;

                        // Skip timestamp (8 bytes)
                        offset += 8;

                        uint32_t arb_id;
                        std::memcpy(&arb_id, data_ptr + offset, 4);
                        offset += 4;

                        uint8_t frame_len = static_cast<uint8_t>(data_ptr[offset]);
                        offset += 1;

                        if (offset + frame_len > data_len) break;

                        // Find matching axis by arb_id (response = device_id << 8)
                        for (size_t j = 0; j < g_axes.size(); ++j)
                        {
                            uint32_t expected_id = g_axes[j].device_id << 8;
                            if (arb_id == expected_id && frame_len > 0)
                            {
                                const uint8_t* frame_data = reinterpret_cast<const uint8_t*>(data_ptr + offset);
                                Query::Result result = Query::Parse(frame_data, frame_len);

                                // Save raw position for set_zero
                                g_axes[j].current_position = result.position;

                                int motdir = g_axes[j].motdir;
                                statuses[j].mode = static_cast<uint8_t>(result.mode);
                                statuses[j].position = std::isnan(result.position) ? 0.0 : result.position * motdir;
                                statuses[j].velocity = std::isnan(result.velocity) ? 0.0 : result.velocity * motdir;
                                statuses[j].d_current = std::isnan(result.d_current) ? 0.0 : result.d_current;
                                statuses[j].q_current = std::isnan(result.q_current) ? 0.0 : result.q_current * motdir;
                                statuses[j].torque = std::isnan(result.torque) ? 0.0 : result.torque * motdir;
                                statuses[j].motor_temp = std::isnan(result.motor_temperature) ? 0.0 : result.motor_temperature;
                                break;
                            }
                        }

                        offset += frame_len;
                    }

                    // Output motor_status
                    // Format: [1 byte count][axis1: mode(1) + pos(8) + vel(8) + d_cur(8) + q_cur(8) + torq(8) + temp(8)]...
                    // Per axis: 1 + 6*8 = 49 bytes
                    uint8_t status_buffer[1 + 64 * 49];
                    size_t status_offset = 0;
                    status_buffer[status_offset++] = static_cast<uint8_t>(statuses.size());
                    for (const auto& s : statuses)
                    {
                        status_buffer[status_offset++] = s.mode;
                        std::memcpy(status_buffer + status_offset, &s.position, sizeof(double));
                        status_offset += sizeof(double);
                        std::memcpy(status_buffer + status_offset, &s.velocity, sizeof(double));
                        status_offset += sizeof(double);
                        std::memcpy(status_buffer + status_offset, &s.d_current, sizeof(double));
                        status_offset += sizeof(double);
                        std::memcpy(status_buffer + status_offset, &s.q_current, sizeof(double));
                        status_offset += sizeof(double);
                        std::memcpy(status_buffer + status_offset, &s.torque, sizeof(double));
                        status_offset += sizeof(double);
                        std::memcpy(status_buffer + status_offset, &s.motor_temp, sizeof(double));
                        status_offset += sizeof(double);
                    }

                    std::string output_id = "motor_status";
                    dora_send_output(dora_context,
                                     const_cast<char*>(output_id.c_str()), output_id.length(),
                                     reinterpret_cast<char*>(status_buffer), status_offset);
                }
            }
        }

        free_dora_event(event);
    }

    free_dora_context(dora_context);
    std::cout << "[moteus_communication] Finished" << std::endl;
    return 0;
}
