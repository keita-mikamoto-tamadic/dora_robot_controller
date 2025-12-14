extern "C"
{
#include "node_api.h"
}

#include <iostream>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <string>
#include <sched.h>

#include "nlohmann/json.hpp"
#include "timing_manager.hpp"

using json = nlohmann::json;

// State definitions
enum class State : uint8_t
{
    INIT = 0,
    SERVO_OFF = 1,
    READY = 2,
    RUN = 3
};

// State command definitions (from GUI)
enum class StateCommand : uint8_t
{
    SERVO_OFF = 0,
    STOP = 1,
    READY = 2,
    RUN = 3
};

// Axis configuration
struct AxisConfig
{
    int index;
    std::string name;
    int device_id;
    int motdir;
    double initial_position;  // Target position for ready state (rev)
    double current_position;  // Current position from motor_status
    double current_torque;    // Current torque from motor_status
};

// Global state
static State g_current_state = State::INIT;
static std::vector<AxisConfig> g_axes;
static bool g_config_received = false;
static double g_interpolation_time = 2.0;  // seconds

// Interpolation state
static std::vector<double> g_interp_start_positions;
static std::vector<double> g_interp_target_positions;
static bool g_interpolating = false;

// Stop/hold state
static std::vector<double> g_hold_positions;
static bool g_holding = false;

// Timing manager for real-time interpolation
static TimingManager g_timing;

const char* state_to_string(State s)
{
    switch (s)
    {
        case State::INIT: return "INIT";
        case State::SERVO_OFF: return "SERVO_OFF";
        case State::READY: return "READY";
        case State::RUN: return "RUN";
        default: return "UNKNOWN";
    }
}

// Send servo_on command (bitmask for all axes)
void send_servo_on(void* dora_context, uint8_t mask)
{
    std::string output_id = "servo_on";
    char data[1] = {static_cast<char>(mask)};
    dora_send_output(dora_context,
                     const_cast<char*>(output_id.c_str()), output_id.length(),
                     data, 1);
}

// Send servo_off command (bitmask for all axes)
void send_servo_off(void* dora_context, uint8_t mask)
{
    std::string output_id = "servo_off";
    char data[1] = {static_cast<char>(mask)};
    dora_send_output(dora_context,
                     const_cast<char*>(output_id.c_str()), output_id.length(),
                     data, 1);
}

// Send position command for a single axis
// Format: [1 byte axis_index][8 bytes position (double)]
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

// Send state_status to GUI
// Format: [1 byte current_state][1 byte progress_percent]
void send_state_status(void* dora_context, uint8_t progress = 0)
{
    uint8_t buffer[2];
    buffer[0] = static_cast<uint8_t>(g_current_state);
    buffer[1] = progress;

    std::string output_id = "state_status";
    dora_send_output(dora_context,
                     const_cast<char*>(output_id.c_str()), output_id.length(),
                     reinterpret_cast<char*>(buffer), 2);
}

// Send motor_display to GUI
// Format: [1 byte axis_count][position(8)×N][torque(8)×N]
void send_motor_display(void* dora_context)
{
    size_t n = g_axes.size();
    std::vector<uint8_t> buffer(1 + n * 16);  // 1 + N * (8 + 8)
    size_t offset = 0;

    buffer[offset++] = static_cast<uint8_t>(n);

    // Positions
    for (const auto& axis : g_axes)
    {
        std::memcpy(buffer.data() + offset, &axis.current_position, sizeof(double));
        offset += sizeof(double);
    }

    // Torques
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

// Forward robot_config to moteus_communication
void forward_robot_config(void* dora_context, const char* data, size_t len)
{
    std::string output_id = "robot_config_out";
    dora_send_output(dora_context,
                     const_cast<char*>(output_id.c_str()), output_id.length(),
                     const_cast<char*>(data), len);
}

// Start interpolation to target positions
void start_interpolation()
{
    g_interp_start_positions.resize(g_axes.size());
    g_interp_target_positions.resize(g_axes.size());

    for (size_t i = 0; i < g_axes.size(); ++i)
    {
        g_interp_start_positions[i] = g_axes[i].current_position;
        g_interp_target_positions[i] = g_axes[i].initial_position;
    }

    g_timing.reset();  // Reset timing for interpolation
    g_interpolating = true;
    g_holding = false;

    std::cout << "[state_manager] Starting interpolation ("
              << g_interpolation_time << "s)" << std::endl;
}

// Calculate current interpolated position (time-based)
double interpolate(double start, double target, double t)
{
    if (t >= 1.0) return target;
    if (t <= 0.0) return start;
    return start + (target - start) * t;
}

int main()
{
    std::cout << "[state_manager] Starting mimicv2_state_manager" << std::endl;

    // Set CPU affinity if specified
    const char* cpu_affinity_env = std::getenv("CPU_AFFINITY");
    if (cpu_affinity_env)
    {
        int cpu_id = std::atoi(cpu_affinity_env);
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(cpu_id, &cpuset);
        if (sched_setaffinity(0, sizeof(cpuset), &cpuset) == 0)
        {
            std::cout << "[state_manager] CPU affinity set to core " << cpu_id << std::endl;
        }
    }

    auto dora_context = init_dora_context_from_env();
    if (dora_context == NULL)
    {
        std::cerr << "[state_manager] Failed to init dora context" << std::endl;
        return 1;
    }

    std::cout << "[state_manager] Waiting for robot_config..." << std::endl;

    while (true)
    {
        void* event = dora_next_event(dora_context);
        if (event == NULL)
        {
            break;
        }

        enum DoraEventType ty = read_dora_event_type(event);

        if (ty == DoraEventType_Stop)
        {
            std::cout << "[state_manager] Received stop signal" << std::endl;
            // Send servo_off for all axes before exit
            if (g_config_received)
            {
                uint8_t all_mask = (1 << g_axes.size()) - 1;
                send_servo_off(dora_context, all_mask);
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
                // Parse JSON config
                if (!g_config_received && data_len > 0)
                {
                    try
                    {
                        std::string json_str(data_ptr, data_len);
                        json config = json::parse(json_str);

                        // Get interpolation time
                        g_interpolation_time = config.value("interpolation_time", 2.0);

                        g_axes.clear();
                        for (const auto& axis_json : config["axes"])
                        {
                            AxisConfig axis;
                            axis.index = axis_json["index"].get<int>();
                            axis.name = axis_json["name"].get<std::string>();
                            axis.device_id = axis_json["device_id"].get<int>();
                            axis.motdir = axis_json["motdir"].get<int>();
                            axis.initial_position = axis_json.value("initial_position", 0.0);
                            axis.current_position = 0.0;
                            axis.current_torque = 0.0;
                            g_axes.push_back(axis);
                        }

                        g_hold_positions.resize(g_axes.size(), 0.0);

                        g_config_received = true;
                        std::cout << "[state_manager] Config received: "
                                  << config["robot_name"].get<std::string>()
                                  << " (" << g_axes.size() << " axes)"
                                  << ", interp_time=" << g_interpolation_time << "s" << std::endl;

                        for (const auto& axis : g_axes)
                        {
                            std::cout << "  - " << axis.name
                                      << ": device_id=" << axis.device_id
                                      << ", initial_pos=" << axis.initial_position << std::endl;
                        }

                        // Forward config to moteus_communication
                        forward_robot_config(dora_context, data_ptr, data_len);

                        // Transition from INIT to SERVO_OFF
                        g_current_state = State::SERVO_OFF;
                        send_state_status(dora_context, 100);
                        std::cout << "[state_manager] State: INIT -> SERVO_OFF" << std::endl;
                    }
                    catch (const std::exception& e)
                    {
                        std::cerr << "[state_manager] Failed to parse config: " << e.what() << std::endl;
                    }
                }
            }
            else if (input_id == "state_command")
            {
                if (g_config_received && data_len >= 1)
                {
                    StateCommand cmd = static_cast<StateCommand>(static_cast<uint8_t>(data_ptr[0]));
                    uint8_t all_mask = (1 << g_axes.size()) - 1;

                    switch (cmd)
                    {
                        case StateCommand::SERVO_OFF:
                            // Always allowed: turn off all servos
                            send_servo_off(dora_context, all_mask);
                            g_interpolating = false;
                            g_holding = false;
                            g_current_state = State::SERVO_OFF;
                            send_state_status(dora_context, 100);
                            std::cout << "[state_manager] State -> SERVO_OFF (command)" << std::endl;
                            break;

                        case StateCommand::STOP:
                            // Hold current position with servo ON
                            if (g_current_state == State::READY || g_current_state == State::RUN)
                            {
                                g_interpolating = false;
                                g_holding = true;
                                // Capture current positions as hold positions
                                for (size_t i = 0; i < g_axes.size(); ++i)
                                {
                                    g_hold_positions[i] = g_axes[i].current_position;
                                }
                                std::cout << "[state_manager] STOP: Holding positions" << std::endl;
                                // Stay in current state but stop motion
                            }
                            break;

                        case StateCommand::READY:
                            if (g_current_state == State::SERVO_OFF)
                            {
                                // Servo on all axes
                                send_servo_on(dora_context, all_mask);
                                g_current_state = State::READY;
                                // Start interpolation to initial positions
                                start_interpolation();
                                send_state_status(dora_context, 0);
                                std::cout << "[state_manager] State: SERVO_OFF -> READY" << std::endl;
                            }
                            else if (g_holding)
                            {
                                // Resume from stop: restart interpolation
                                start_interpolation();
                                g_current_state = State::READY;
                                std::cout << "[state_manager] Resuming READY from STOP" << std::endl;
                            }
                            break;

                        case StateCommand::RUN:
                            if (g_current_state == State::READY && !g_interpolating)
                            {
                                // Only allow RUN after interpolation complete
                                g_current_state = State::RUN;
                                g_holding = false;
                                send_state_status(dora_context, 100);
                                std::cout << "[state_manager] State: READY -> RUN" << std::endl;
                            }
                            else if (g_holding && g_current_state == State::RUN)
                            {
                                // Resume from stop in run state
                                g_holding = false;
                                std::cout << "[state_manager] Resuming RUN from STOP" << std::endl;
                            }
                            break;
                    }
                }
            }
            else if (input_id == "motor_status")
            {
                // Parse motor status from moteus_communication
                // Format: [1 byte count][axis: mode(1) + pos(8) + vel(8) + d_cur(8) + q_cur(8) + torq(8) + temp(8)]...
                if (g_config_received && data_len >= 1)
                {
                    uint8_t count = static_cast<uint8_t>(data_ptr[0]);
                    size_t offset = 1;

                    for (uint8_t i = 0; i < count && i < g_axes.size(); ++i)
                    {
                        if (offset + 49 > data_len) break;

                        // Skip mode (1 byte)
                        offset += 1;

                        // Position (8 bytes)
                        double position;
                        std::memcpy(&position, data_ptr + offset, sizeof(double));
                        offset += sizeof(double);

                        // Skip velocity (8 bytes)
                        offset += sizeof(double);

                        // Skip d_current (8 bytes)
                        offset += sizeof(double);

                        // Skip q_current (8 bytes)
                        offset += sizeof(double);

                        // Torque (8 bytes)
                        double torque;
                        std::memcpy(&torque, data_ptr + offset, sizeof(double));
                        offset += sizeof(double);

                        // Skip motor_temp (8 bytes)
                        offset += sizeof(double);

                        g_axes[i].current_position = position;
                        g_axes[i].current_torque = torque;
                    }

                    // Send motor display to GUI
                    send_motor_display(dora_context);
                }
            }
            else if (input_id == "tick")
            {
                if (g_config_received)
                {
                    // Send query to moteus_communication to trigger CAN frames
                    std::string query_id = "query";
                    uint8_t query_data[1] = {0xFF};
                    dora_send_output(dora_context,
                                     const_cast<char*>(query_id.c_str()), query_id.length(),
                                     reinterpret_cast<char*>(query_data), 1);

                    // Update timing
                    g_timing.update();

                    if (g_interpolating && g_current_state == State::READY)
                    {
                        // Calculate interpolation progress based on real time
                        double t = g_timing.getElapsedTime() / g_interpolation_time;
                        if (t > 1.0) t = 1.0;

                        for (size_t i = 0; i < g_axes.size(); ++i)
                        {
                            double pos = interpolate(g_interp_start_positions[i],
                                                     g_interp_target_positions[i],
                                                     t);
                            send_position_command(dora_context, static_cast<uint8_t>(i), pos);
                        }

                        // Calculate progress percentage
                        uint8_t progress = static_cast<uint8_t>(t * 100);
                        if (progress > 100) progress = 100;
                        send_state_status(dora_context, progress);

                        // Check if interpolation complete
                        if (t >= 1.0)
                        {
                            g_interpolating = false;
                            std::cout << "[state_manager] Interpolation complete" << std::endl;
                            send_state_status(dora_context, 100);
                        }
                    }
                    else if (g_holding)
                    {
                        // Hold current positions
                        for (size_t i = 0; i < g_axes.size(); ++i)
                        {
                            send_position_command(dora_context, static_cast<uint8_t>(i), g_hold_positions[i]);
                        }
                    }
                    else if (g_current_state == State::RUN)
                    {
                        // RUN state: Future control logic goes here
                        // For now, just hold the initial positions
                        for (size_t i = 0; i < g_axes.size(); ++i)
                        {
                            send_position_command(dora_context, static_cast<uint8_t>(i), g_axes[i].initial_position);
                        }
                    }
                }
            }
        }

        free_dora_event(event);
    }

    free_dora_context(dora_context);
    std::cout << "[state_manager] Finished" << std::endl;
    return 0;
}
