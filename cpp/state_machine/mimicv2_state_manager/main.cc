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
    }
}

static void handle_motor_status(void* dora_context, const char* data, size_t len)
{
    if (!g_config_received || len < 1) return;

    uint8_t count = static_cast<uint8_t>(data[0]);
    size_t offset = 1;

    for (uint8_t i = 0; i < count && i < g_axes.size(); ++i)
    {
        if (offset + 49 > len) break;

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

        g_axes[i].current_position = position;
        g_axes[i].current_torque = torque;
    }

    send_motor_display(dora_context);
}

static void handle_tick(void* dora_context)
{
    if (!g_config_received) return;

    g_timing.update();

    // Position commands BEFORE query
    if (g_interpolating && g_current_state == State::READY)
    {
        tick_interpolation(dora_context);
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
            else if (input_id == "tick")
                handle_tick(dora_context);
        }

        free_dora_event(event);
    }

    free_dora_context(dora_context);
    std::cout << "[state_manager] Finished" << std::endl;
    return 0;
}
