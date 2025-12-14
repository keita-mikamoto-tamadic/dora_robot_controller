#pragma once

#include <cstdint>
#include <string>
#include <vector>

enum class State : uint8_t
{
    INIT = 0,
    SERVO_OFF = 1,
    READY = 2,
    RUN = 3
};

enum class StateCommand : uint8_t
{
    SERVO_OFF = 0,
    STOP = 1,
    READY = 2,
    RUN = 3
};

struct AxisConfig
{
    int index;
    std::string name;
    int device_id;
    int motdir;
    double initial_position;
    double current_position;
    double current_torque;
};

// Global state
extern State g_current_state;
extern std::vector<AxisConfig> g_axes;
extern bool g_config_received;
extern double g_interpolation_time;
extern std::vector<double> g_interp_start_positions;
extern std::vector<double> g_interp_target_positions;
extern bool g_interpolating;
extern std::vector<double> g_hold_positions;
extern bool g_holding;
