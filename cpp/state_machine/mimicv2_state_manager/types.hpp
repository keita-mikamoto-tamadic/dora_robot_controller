#pragma once

#include <cstdint>
#include <string>
#include <vector>

enum class State : uint8_t
{
    INIT = 0,
    SERVO_OFF = 1,
    READY = 2,
    RUN = 3,
    STOP = 4
};

enum class StateCommand : uint8_t
{
    SERVO_OFF = 0,
    STOP = 1,
    READY = 2,
    RUN = 3,
    INIT_POSITION_RESET = 4
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
    uint8_t fault;  // moteus fault code (0 = no fault)
};

// IMUデータ構造体
struct ImuData
{
    float q0, q1, q2, q3;    // クォータニオン（Madgwickフィルタ済み）
    float gx, gy, gz;         // 角速度 (rad/s)
    float ax, ay, az;         // 加速度 (m/s²)
    float roll, pitch, yaw;   // オイラー角 (rad)
    bool valid;               // データ有効フラグ
    uint64_t timestamp_ns;    // 受信時刻（ナノ秒）
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
extern ImuData g_imu_data;
