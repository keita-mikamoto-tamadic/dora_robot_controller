extern "C"
{
#include "node_api.h"
}

#include <iostream>
#include <cstring>
#include <cstdlib>
#include <sched.h>

#include "types.hpp"
#include "helpers.hpp"
#include "timing_manager.hpp"
#include "state_init.hpp"
#include "state_ready.hpp"
#include "state_stop.hpp"
#include "state_run.hpp"

// Global state definitions
State g_current_state = State::INIT;
std::vector<AxisConfig> g_axes;
bool g_config_received = false;
double g_interpolation_time = 5.0;
std::vector<double> g_interp_start_positions;
std::vector<double> g_interp_target_positions;
bool g_interpolating = false;
std::vector<double> g_hold_positions;
bool g_holding = false;
TimingManager g_timing;
ImuData g_imu_data = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false, 0};

static void handle_state_command(void* dora_context, const char* data, size_t len)
{
    if (!g_config_received || len < 1) return;

    StateCommand cmd = static_cast<StateCommand>(static_cast<uint8_t>(data[0]));

    switch (cmd)
    {
        case StateCommand::SERVO_OFF:
        {
            uint8_t all_mask = (1 << g_axes.size()) - 1;
            send_servo_off(dora_context, all_mask);
            g_interpolating = false;
            g_holding = false;
            g_current_state = State::SERVO_OFF;
            send_state_status(dora_context, 100);
            std::cout << "[state_manager] -> SERVO_OFF" << std::endl;
            break;
        }
        case StateCommand::STOP:
            cmd_stop(dora_context);
            break;
        case StateCommand::READY:
            cmd_ready(dora_context);
            break;
        case StateCommand::RUN:
            cmd_run(dora_context);
            break;
        case StateCommand::INIT_POSITION_RESET:
        {
            // Reset moteus encoder positions to predefined init positions
            // hip_pitch_r/l: -7.5deg = -0.130899 rad
            // knee_r/l: -136deg = -2.373644 rad
            // wheel_r/l: 0deg = 0.0 rad
            const double init_positions[6] = {
                -0.130899,  // hip_pitch_r: -7.5° = -0.02083 * 2π rad
                -2.373644,  // knee_r: -136° = -0.37778 * 2π rad
                 0.0,       // wheel_r: 0°
                -0.130899,  // hip_pitch_l: -7.5°
                -2.373644,  // knee_l: -136°
                 0.0        // wheel_l: 0°
            };

            std::vector<double> positions;
            for (size_t i = 0; i < g_axes.size() && i < 6; ++i)
            {
                positions.push_back(init_positions[i]);
            }

            // Send set_position to moteus_communication (uses OutputExact)
            send_set_position(dora_context, positions);

            std::cout << "[state_manager] Init position reset sent" << std::endl;
            break;
        }
    }
}

static void handle_motor_status(void* dora_context, const char* data, size_t len)
{
    if (!g_config_received || len < 1) return;

    uint8_t count = static_cast<uint8_t>(data[0]);
    size_t offset = 1;
    bool fault_detected = false;

    for (uint8_t i = 0; i < count && i < g_axes.size(); ++i)
    {
        if (offset + 50 > len) break;  // Updated: now 50 bytes per axis (added fault)

        offset += 1;  // mode

        double position;
        std::memcpy(&position, data + offset, sizeof(double));
        offset += sizeof(double);

        offset += sizeof(double);  // velocity
        offset += sizeof(double);  // d_current
        offset += sizeof(double);  // q_current

        double torque;
        std::memcpy(&torque, data + offset, sizeof(double));
        offset += sizeof(double);

        offset += sizeof(double);  // motor_temp

        int8_t fault = static_cast<int8_t>(data[offset]);
        offset += 1;  // fault

        g_axes[i].current_position = position;
        g_axes[i].current_torque = torque;

        // Detect fault and trigger STOP state
        if (fault != 0)
        {
            fault_detected = true;
        }
    }

    // If any fault detected, transition to STOP state for safety
    if (fault_detected && g_current_state != State::STOP && g_current_state != State::SERVO_OFF)
    {
        std::cout << "[state_manager] FAULT DETECTED - Transitioning to STOP state" << std::endl;

        // Initialize hold positions to current positions
        g_interpolating = false;
        g_holding = false;
        for (size_t i = 0; i < g_axes.size(); ++i)
        {
            g_hold_positions[i] = g_axes[i].current_position;
        }

        g_current_state = State::STOP;
        send_state_status(dora_context, 0);
    }

    send_motor_display(dora_context);
}

// IMUデータメッセージサイズ: 61バイト
// [valid(1)][q0-q3(16)][gx-gz(12)][ax-az(12)][roll,pitch,yaw(12)][timestamp(8)]
static constexpr size_t IMU_MESSAGE_SIZE = 61;

static uint32_t g_imu_log_counter = 0;

static void handle_imu_data(const char* data, size_t len)
{
    if (len < IMU_MESSAGE_SIZE) return;

    size_t offset = 0;

    g_imu_data.valid = (data[offset++] != 0);
    std::memcpy(&g_imu_data.q0, data + offset, sizeof(float)); offset += sizeof(float);
    std::memcpy(&g_imu_data.q1, data + offset, sizeof(float)); offset += sizeof(float);
    std::memcpy(&g_imu_data.q2, data + offset, sizeof(float)); offset += sizeof(float);
    std::memcpy(&g_imu_data.q3, data + offset, sizeof(float)); offset += sizeof(float);
    std::memcpy(&g_imu_data.gx, data + offset, sizeof(float)); offset += sizeof(float);
    std::memcpy(&g_imu_data.gy, data + offset, sizeof(float)); offset += sizeof(float);
    std::memcpy(&g_imu_data.gz, data + offset, sizeof(float)); offset += sizeof(float);
    std::memcpy(&g_imu_data.ax, data + offset, sizeof(float)); offset += sizeof(float);
    std::memcpy(&g_imu_data.ay, data + offset, sizeof(float)); offset += sizeof(float);
    std::memcpy(&g_imu_data.az, data + offset, sizeof(float)); offset += sizeof(float);
    std::memcpy(&g_imu_data.roll, data + offset, sizeof(float)); offset += sizeof(float);
    std::memcpy(&g_imu_data.pitch, data + offset, sizeof(float)); offset += sizeof(float);
    std::memcpy(&g_imu_data.yaw, data + offset, sizeof(float)); offset += sizeof(float);
    std::memcpy(&g_imu_data.timestamp_ns, data + offset, sizeof(uint64_t));

    // 1000回に1回表示（約1秒毎）
/*     if (++g_imu_log_counter >= 1000)
    {
        g_imu_log_counter = 0;
        std::cout << "[IMU] roll=" << g_imu_data.roll << " pitch=" << g_imu_data.pitch << " yaw=" << g_imu_data.yaw << std::endl;
    } */
}

static void handle_tick(void* dora_context)
{
    if (!g_config_received) return;

    g_timing.update();

    // Position commands BEFORE query
    if (g_current_state == State::STOP)
    {
        // STOP state: hold all axes at current position
        tick_hold(dora_context);
    }
    else if (g_current_state == State::READY)
    {
        if (g_interpolating)
        {
            tick_interpolation(dora_context);
        }
        else
        {
            // 補間完了後: 股膝は位置制御、ホイールは速度制御(0)
            tick_ready_hold(dora_context);
        }
    }
    else if (g_holding)
    {
        tick_hold(dora_context);
    }
    else if (g_current_state == State::RUN)
    {
        tick_run(dora_context);
    }

    // Query AFTER position commands
    send_query(dora_context);
}

int main()
{
    std::cout << "[state_manager] Starting" << std::endl;

    auto dora_context = init_dora_context_from_env();
    if (dora_context == NULL)
    {
        std::cerr << "[state_manager] Failed to init dora context" << std::endl;
        return 1;
    }

    while (true)
    {
        void* event = dora_next_event(dora_context);
        if (event == NULL) break;

        enum DoraEventType ty = read_dora_event_type(event);

        if (ty == DoraEventType_Stop)
        {
            std::cout << "[state_manager] Stop signal" << std::endl;
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
                handle_robot_config(dora_context, data_ptr, data_len);
            else if (input_id == "state_command")
                handle_state_command(dora_context, data_ptr, data_len);
            else if (input_id == "motor_status")
                handle_motor_status(dora_context, data_ptr, data_len);
            else if (input_id == "imu_data")
                handle_imu_data(data_ptr, data_len);
            else if (input_id == "tick")
                handle_tick(dora_context);
        }

        free_dora_event(event);
    }

    free_dora_context(dora_context);
    std::cout << "[state_manager] Finished" << std::endl;
    return 0;
}
