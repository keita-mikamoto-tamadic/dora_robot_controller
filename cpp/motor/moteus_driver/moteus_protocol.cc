#include "moteus_protocol.hpp"
#include "mjbots/moteus/moteus_protocol.h"
#include <cstring>
#include <cmath>
#include <limits>

using namespace mjbots::moteus;

size_t build_position_frame(uint8_t* buffer, int motor_id, double position, double velocity,
                            double kp_scale, double kd_scale,
                            double velocity_limit, double accel_limit, double torque_limit)
{
    CanData frame;
    WriteCanData writer(&frame);

    PositionMode::Command cmd;
    cmd.position = position;
    cmd.velocity = velocity;
    cmd.maximum_torque = torque_limit;
    cmd.feedforward_torque = 0.0;
    cmd.kp_scale = kp_scale;
    cmd.kd_scale = kd_scale;
    cmd.stop_position = std::numeric_limits<double>::quiet_NaN();
    cmd.watchdog_timeout = std::numeric_limits<double>::quiet_NaN();
    cmd.velocity_limit = velocity_limit;
    cmd.accel_limit = accel_limit;

    PositionMode::Format fmt;
    fmt.maximum_torque = Resolution::kFloat;
    fmt.velocity_limit = Resolution::kFloat;
    fmt.accel_limit = Resolution::kFloat;
    fmt.kp_scale = Resolution::kFloat;
    fmt.kd_scale = Resolution::kFloat;
    PositionMode::Make(&writer, cmd, fmt);

    // Request query response
    Query::Format query_fmt;
    query_fmt.q_current = Resolution::kFloat;
    query_fmt.fault = Resolution::kInt8;
    Query::Make(&writer, query_fmt);

    // Return raw CAN data (arb_id will be added by caller)
    std::memcpy(buffer, frame.data, frame.size);
    return frame.size;
}

size_t build_nan_position_frame(uint8_t* buffer, int motor_id)
{
    CanData frame;
    WriteCanData writer(&frame);

    PositionMode::Command cmd;
    cmd.position = std::numeric_limits<double>::quiet_NaN();
    cmd.velocity = 0.0;
    cmd.maximum_torque = std::numeric_limits<double>::quiet_NaN();
    cmd.feedforward_torque = 0.0;
    cmd.kp_scale = 1.0;
    cmd.kd_scale = 1.0;
    cmd.stop_position = std::numeric_limits<double>::quiet_NaN();
    cmd.watchdog_timeout = std::numeric_limits<double>::quiet_NaN();
    cmd.velocity_limit = std::numeric_limits<double>::quiet_NaN();
    cmd.accel_limit = std::numeric_limits<double>::quiet_NaN();

    PositionMode::Format fmt;
    PositionMode::Make(&writer, cmd, fmt);

    Query::Format query_fmt;
    query_fmt.q_current = Resolution::kFloat;
    query_fmt.fault = Resolution::kInt8;
    Query::Make(&writer, query_fmt);

    std::memcpy(buffer, frame.data, frame.size);
    return frame.size;
}

size_t build_servo_off_frame(uint8_t* buffer, int motor_id)
{
    CanData frame;
    WriteCanData writer(&frame);

    StopMode::Command cmd;
    StopMode::Format fmt;
    StopMode::Make(&writer, cmd, fmt);

    Query::Format query_fmt;
    query_fmt.q_current = Resolution::kFloat;
    query_fmt.fault = Resolution::kInt8;
    Query::Make(&writer, query_fmt);

    std::memcpy(buffer, frame.data, frame.size);
    return frame.size;
}

size_t build_set_output_exact_frame(uint8_t* buffer, int motor_id, double position)
{
    CanData frame;
    WriteCanData writer(&frame);

    OutputExact::Command cmd;
    cmd.position = position;
    OutputExact::Format fmt;
    OutputExact::Make(&writer, cmd, fmt);

    std::memcpy(buffer, frame.data, frame.size);
    return frame.size;
}

bool parse_query_response(const uint8_t* data, size_t len, MotorQueryResult& result)
{
    if (len == 0) return false;

    Query::Result parsed = Query::Parse(data, len);

    result.mode = static_cast<uint8_t>(parsed.mode);
    result.position = std::isnan(parsed.position) ? 0.0 : parsed.position;
    result.velocity = std::isnan(parsed.velocity) ? 0.0 : parsed.velocity;
    result.torque = std::isnan(parsed.torque) ? 0.0 : parsed.torque;
    result.q_current = std::isnan(parsed.q_current) ? 0.0 : parsed.q_current;
    result.fault = parsed.fault;

    return true;
}
