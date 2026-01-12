/**
 * moteus_driver - Unified moteus motor driver node
 *
 * Combines moteus_communication + canfd_txrx into single node.
 * Receives motor commands, sends CAN frames, returns motor status.
 *
 * Inputs:
 *   - robot_config: JSON configuration with axis info
 *   - motor_commands: Position/velocity commands for all axes
 *   - set_position: Set encoder zero point (OutputExact)
 *
 * Outputs:
 *   - motor_status: Current position/velocity/torque/fault for all axes
 */

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

#include "can_interface.hpp"
#include "moteus_protocol.hpp"
#include "nlohmann/json.hpp"

using json = nlohmann::json;

// Constants
static constexpr double TWO_PI = 2.0 * M_PI;
static constexpr int GROUP_BOUNDARY = 80;  // device_id < 80 = right leg, >= 80 = left leg
static constexpr int SERVO_ON_DELAY_CYCLES = 5;

// Unit conversion
inline double rev_to_rad(double rev) { return rev * TWO_PI; }
inline double rad_to_rev(double rad) { return rad / TWO_PI; }

// Control mode enum (matches state_manager)
enum class ControlMode : uint8_t
{
    SERVO_OFF = 0,
    STOP = 1,      // NaN position (hold current)
    POSITION = 2,  // Position control
    VELOCITY = 3   // Velocity control
};

// Per-axis configuration
struct AxisConfig
{
    int index;
    std::string name;
    int device_id;
    int motdir;            // Motor direction: 1 or -1
    double velocity_limit; // rev/s
    double accel_limit;    // rev/s^2
    double torque_limit;   // Nm
    int servo_on_delay;    // Cycles remaining before position control
    int8_t previous_fault; // For fault change detection
};

// Global state
static std::vector<AxisConfig> g_axes;
static bool g_config_received = false;
static CanInterface* g_can = nullptr;

// Parse robot_config JSON
static void handle_robot_config(const char* data, size_t len)
{
    try
    {
        std::string json_str(data, len);
        json config = json::parse(json_str);

        g_axes.clear();

        if (config.contains("axes"))
        {
            int idx = 0;
            for (const auto& axis_json : config["axes"])
            {
                AxisConfig axis;
                axis.index = idx++;
                axis.name = axis_json.value("name", "unknown");
                axis.device_id = axis_json["device_id"].get<int>();
                axis.motdir = axis_json["motdir"].get<int>();
                axis.velocity_limit = axis_json.value("velocity_limit", 2.0);
                axis.accel_limit = axis_json.value("accel_limit", 10.0);
                axis.torque_limit = axis_json.value("torque_limit", 0.5);
                axis.servo_on_delay = 0;
                axis.previous_fault = 0;
                g_axes.push_back(axis);
            }
        }

        g_config_received = true;
        std::cout << "[moteus_driver] Loaded " << g_axes.size() << " axes" << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "[moteus_driver] Failed to parse robot_config: " << e.what() << std::endl;
    }
}

// Send motor status to state_manager
// Format: [1 byte count][per axis: pos(8) + vel(8) + torque(8) + mode(1) + fault(1)]
static void send_motor_status(void* dora_context, const std::vector<MotorQueryResult>& results)
{
    // Per axis: 8 + 8 + 8 + 1 + 1 = 26 bytes
    std::vector<uint8_t> buffer(1 + g_axes.size() * 26);
    size_t offset = 0;

    buffer[offset++] = static_cast<uint8_t>(g_axes.size());

    for (size_t i = 0; i < g_axes.size(); ++i)
    {
        const auto& result = results[i];
        int motdir = g_axes[i].motdir;

        // Convert from revolutions to radians, apply motor direction
        double position_rad = rev_to_rad(result.position * motdir);
        double velocity_rad_s = rev_to_rad(result.velocity * motdir);
        double torque_nm = result.torque * motdir;

        std::memcpy(buffer.data() + offset, &position_rad, sizeof(double));
        offset += sizeof(double);
        std::memcpy(buffer.data() + offset, &velocity_rad_s, sizeof(double));
        offset += sizeof(double);
        std::memcpy(buffer.data() + offset, &torque_nm, sizeof(double));
        offset += sizeof(double);
        buffer[offset++] = result.mode;
        buffer[offset++] = static_cast<uint8_t>(result.fault);
    }

    std::string output_id = "motor_status";
    dora_send_output(dora_context,
                     const_cast<char*>(output_id.c_str()), output_id.length(),
                     reinterpret_cast<char*>(buffer.data()), offset);
}

// Process motor_commands input
// Format: [1 byte count][per axis: mode(1) + position(8) + velocity(8) + kp(8) + kd(8)]
static void handle_motor_commands(void* dora_context, const char* data, size_t len)
{
    if (!g_config_received || len < 1) return;

    uint8_t count = static_cast<uint8_t>(data[0]);
    size_t offset = 1;
    size_t per_axis = 1 + 4 * sizeof(double);  // mode + position + velocity + kp + kd

    // Parse commands and build CAN frames
    struct AxisCommand
    {
        ControlMode mode;
        double position;  // rad
        double velocity;  // rad/s
        double kp_scale;
        double kd_scale;
    };
    std::vector<AxisCommand> commands(g_axes.size());

    for (size_t i = 0; i < g_axes.size() && i < count; ++i)
    {
        if (offset + per_axis > len) break;

        commands[i].mode = static_cast<ControlMode>(data[offset++]);
        std::memcpy(&commands[i].position, data + offset, sizeof(double));
        offset += sizeof(double);
        std::memcpy(&commands[i].velocity, data + offset, sizeof(double));
        offset += sizeof(double);
        std::memcpy(&commands[i].kp_scale, data + offset, sizeof(double));
        offset += sizeof(double);
        std::memcpy(&commands[i].kd_scale, data + offset, sizeof(double));
        offset += sizeof(double);

        // Handle servo_on_delay transitions
        if (commands[i].mode == ControlMode::POSITION || commands[i].mode == ControlMode::VELOCITY)
        {
            // If transitioning from SERVO_OFF, start delay
            if (g_axes[i].servo_on_delay == 0)
            {
                // Check if this is a new servo-on (we'll track via delay counter)
            }
        }
        else if (commands[i].mode == ControlMode::SERVO_OFF)
        {
            g_axes[i].servo_on_delay = SERVO_ON_DELAY_CYCLES;  // Reset for next servo-on
        }
    }

    // Split axes into groups
    std::vector<size_t> group1_indices, group2_indices;
    for (size_t i = 0; i < g_axes.size(); ++i)
    {
        if (g_axes[i].device_id < GROUP_BOUNDARY)
        {
            group1_indices.push_back(i);
        }
        else
        {
            group2_indices.push_back(i);
        }
    }

    // Results storage
    std::vector<MotorQueryResult> results(g_axes.size());
    for (auto& r : results)
    {
        r.mode = 0;
        r.position = 0;
        r.velocity = 0;
        r.torque = 0;
        r.q_current = 0;
        r.fault = 0;
    }

    // Process Group 1 (right leg)
    if (!group1_indices.empty())
    {
        std::vector<uint32_t> expected_ids;
        for (size_t idx : group1_indices)
        {
            const auto& cmd = commands[idx];
            const auto& axis = g_axes[idx];
            uint8_t frame_data[64];
            size_t frame_len = 0;

            if (cmd.mode == ControlMode::SERVO_OFF)
            {
                frame_len = build_servo_off_frame(frame_data, axis.device_id);
            }
            else if (cmd.mode == ControlMode::STOP || g_axes[idx].servo_on_delay > 0)
            {
                frame_len = build_nan_position_frame(frame_data, axis.device_id);
                if (g_axes[idx].servo_on_delay > 0)
                {
                    g_axes[idx].servo_on_delay--;
                }
            }
            else
            {
                // Position or velocity control
                double pos_rev = rad_to_rev(cmd.position) * axis.motdir;
                double vel_rev = rad_to_rev(cmd.velocity) * axis.motdir;
                if (cmd.mode == ControlMode::VELOCITY)
                {
                    pos_rev = std::numeric_limits<double>::quiet_NaN();
                }
                frame_len = build_position_frame(frame_data, axis.device_id,
                                                  pos_rev, vel_rev,
                                                  cmd.kp_scale, cmd.kd_scale,
                                                  axis.velocity_limit, axis.accel_limit,
                                                  axis.torque_limit);
            }

            uint32_t arb_id = 0x8000 | axis.device_id;
            g_can->send(arb_id, frame_data, frame_len);
            expected_ids.push_back(static_cast<uint32_t>(axis.device_id) << 8);
        }

        // Receive responses
        std::vector<CanInterface::RxFrame> rx_frames;
        g_can->receiveMultiple(expected_ids, rx_frames);

        // Parse responses
        for (size_t i = 0; i < group1_indices.size(); ++i)
        {
            size_t axis_idx = group1_indices[i];
            if (rx_frames[i].len > 0)
            {
                parse_query_response(rx_frames[i].data, rx_frames[i].len, results[axis_idx]);

                // Fault change detection
                if (results[axis_idx].fault != g_axes[axis_idx].previous_fault)
                {
                    if (results[axis_idx].fault != 0)
                    {
                        std::cout << "[moteus_driver] FAULT - Axis " << axis_idx
                                  << " (" << g_axes[axis_idx].name << "): code "
                                  << static_cast<int>(results[axis_idx].fault) << std::endl;
                    }
                    else
                    {
                        std::cout << "[moteus_driver] FAULT CLEARED - Axis " << axis_idx
                                  << " (" << g_axes[axis_idx].name << ")" << std::endl;
                    }
                    g_axes[axis_idx].previous_fault = results[axis_idx].fault;
                }
            }
        }
    }

    // Process Group 2 (left leg)
    if (!group2_indices.empty())
    {
        std::vector<uint32_t> expected_ids;
        for (size_t idx : group2_indices)
        {
            const auto& cmd = commands[idx];
            const auto& axis = g_axes[idx];
            uint8_t frame_data[64];
            size_t frame_len = 0;

            if (cmd.mode == ControlMode::SERVO_OFF)
            {
                frame_len = build_servo_off_frame(frame_data, axis.device_id);
            }
            else if (cmd.mode == ControlMode::STOP || g_axes[idx].servo_on_delay > 0)
            {
                frame_len = build_nan_position_frame(frame_data, axis.device_id);
                if (g_axes[idx].servo_on_delay > 0)
                {
                    g_axes[idx].servo_on_delay--;
                }
            }
            else
            {
                double pos_rev = rad_to_rev(cmd.position) * axis.motdir;
                double vel_rev = rad_to_rev(cmd.velocity) * axis.motdir;
                if (cmd.mode == ControlMode::VELOCITY)
                {
                    pos_rev = std::numeric_limits<double>::quiet_NaN();
                }
                frame_len = build_position_frame(frame_data, axis.device_id,
                                                  pos_rev, vel_rev,
                                                  cmd.kp_scale, cmd.kd_scale,
                                                  axis.velocity_limit, axis.accel_limit,
                                                  axis.torque_limit);
            }

            uint32_t arb_id = 0x8000 | axis.device_id;
            g_can->send(arb_id, frame_data, frame_len);
            expected_ids.push_back(static_cast<uint32_t>(axis.device_id) << 8);
        }

        std::vector<CanInterface::RxFrame> rx_frames;
        g_can->receiveMultiple(expected_ids, rx_frames);

        for (size_t i = 0; i < group2_indices.size(); ++i)
        {
            size_t axis_idx = group2_indices[i];
            if (rx_frames[i].len > 0)
            {
                parse_query_response(rx_frames[i].data, rx_frames[i].len, results[axis_idx]);

                if (results[axis_idx].fault != g_axes[axis_idx].previous_fault)
                {
                    if (results[axis_idx].fault != 0)
                    {
                        std::cout << "[moteus_driver] FAULT - Axis " << axis_idx
                                  << " (" << g_axes[axis_idx].name << "): code "
                                  << static_cast<int>(results[axis_idx].fault) << std::endl;
                    }
                    else
                    {
                        std::cout << "[moteus_driver] FAULT CLEARED - Axis " << axis_idx
                                  << " (" << g_axes[axis_idx].name << ")" << std::endl;
                    }
                    g_axes[axis_idx].previous_fault = results[axis_idx].fault;
                }
            }
        }
    }

    // Send motor status
    send_motor_status(dora_context, results);
}

// Handle set_position (encoder zero point)
static void handle_set_position(void* dora_context, const char* data, size_t len)
{
    if (!g_config_received || len < 1) return;

    uint8_t count = static_cast<uint8_t>(data[0]);
    size_t offset = 1;

    for (size_t i = 0; i < count && i < g_axes.size(); ++i)
    {
        if (offset + sizeof(double) > len) break;

        double position_rad;
        std::memcpy(&position_rad, data + offset, sizeof(double));
        offset += sizeof(double);

        double position_rev = rad_to_rev(position_rad) * g_axes[i].motdir;

        uint8_t frame_data[64];
        size_t frame_len = build_set_output_exact_frame(frame_data, g_axes[i].device_id, position_rev);

        uint32_t arb_id = 0x8000 | g_axes[i].device_id;
        g_can->send(arb_id, frame_data, frame_len);

        std::cout << "[moteus_driver] Set position axis " << i << " = " << position_rad << " rad" << std::endl;
    }
}

int main()
{
    std::cout << "[moteus_driver] Starting" << std::endl;

    // Get configuration from environment
    const char* can_interface = std::getenv("CAN_INTERFACE");
    if (!can_interface) can_interface = "can0";

    const char* rx_timeout_env = std::getenv("RX_TIMEOUT_US");
    int rx_timeout_us = rx_timeout_env ? std::atoi(rx_timeout_env) : 2000;

    // Set CPU affinity
    const char* cpu_affinity_env = std::getenv("CPU_AFFINITY");
    if (cpu_affinity_env)
    {
        int cpu_id = std::atoi(cpu_affinity_env);
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(cpu_id, &cpuset);
        if (sched_setaffinity(0, sizeof(cpuset), &cpuset) == 0)
        {
            std::cout << "[moteus_driver] CPU affinity set to core " << cpu_id << std::endl;
        }
    }

    // Set RT priority
    const char* rt_priority_env = std::getenv("RT_PRIORITY");
    if (rt_priority_env)
    {
        int priority = std::atoi(rt_priority_env);
        struct sched_param param;
        param.sched_priority = priority;
        if (sched_setscheduler(0, SCHED_FIFO, &param) == 0)
        {
            std::cout << "[moteus_driver] RT priority set to SCHED_FIFO " << priority << std::endl;
        }
    }

    // Initialize CAN interface
    g_can = new CanInterface(can_interface, rx_timeout_us);
    if (!g_can->init())
    {
        std::cerr << "[moteus_driver] Failed to initialize CAN interface" << std::endl;
        return 1;
    }

    // Initialize Dora
    auto dora_context = init_dora_context_from_env();
    if (dora_context == NULL)
    {
        std::cerr << "[moteus_driver] Failed to init dora context" << std::endl;
        delete g_can;
        return 1;
    }

    // Main event loop
    while (true)
    {
        void* event = dora_next_event(dora_context);
        if (event == NULL) break;

        enum DoraEventType ty = read_dora_event_type(event);

        if (ty == DoraEventType_Stop)
        {
            std::cout << "[moteus_driver] Stop signal" << std::endl;

            // Send servo_off to all axes
            for (const auto& axis : g_axes)
            {
                uint8_t frame_data[64];
                size_t frame_len = build_servo_off_frame(frame_data, axis.device_id);
                uint32_t arb_id = 0x8000 | axis.device_id;
                g_can->send(arb_id, frame_data, frame_len);
            }

            free_dora_event(event);
            break;
        }
        else if (ty == DoraEventType_Input)
        {
            char* id_ptr;
            size_t id_len;
            read_dora_input_id(event, &id_ptr, &id_len);
            std::string input_id(id_ptr, id_len);

            char* data_ptr;
            size_t data_len;
            read_dora_input_data(event, &data_ptr, &data_len);

            if (input_id == "robot_config")
            {
                handle_robot_config(data_ptr, data_len);
            }
            else if (input_id == "motor_commands")
            {
                handle_motor_commands(dora_context, data_ptr, data_len);
            }
            else if (input_id == "set_position")
            {
                handle_set_position(dora_context, data_ptr, data_len);
            }
        }

        free_dora_event(event);
    }

    // Cleanup
    free_dora_context(dora_context);
    delete g_can;
    std::cout << "[moteus_driver] Finished" << std::endl;
    return 0;
}
