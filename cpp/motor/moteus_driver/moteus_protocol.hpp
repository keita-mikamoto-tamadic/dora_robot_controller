#ifndef MOTEUS_PROTOCOL_HPP
#define MOTEUS_PROTOCOL_HPP

#include <cstdint>
#include <cstddef>

/**
 * Build position/velocity command frame for moteus
 *
 * For position control: kp_scale=1.0, kd_scale=1.0
 * For velocity control: kp_scale=0.0, kd_scale>0, position=NaN
 *
 * @param buffer Output buffer (must be at least 64 bytes)
 * @param motor_id Moteus device ID
 * @param position Target position (revolutions), NaN for velocity control
 * @param velocity Target velocity (rev/s)
 * @param kp_scale Position gain scale (0 for velocity control)
 * @param kd_scale Velocity gain scale
 * @param velocity_limit Maximum velocity (rev/s)
 * @param accel_limit Maximum acceleration (rev/s^2)
 * @param torque_limit Maximum torque (Nm)
 * @return Number of bytes written to buffer
 */
size_t build_position_frame(uint8_t* buffer, int motor_id, double position, double velocity,
                            double kp_scale, double kd_scale,
                            double velocity_limit, double accel_limit, double torque_limit);

/**
 * Build NaN position control frame (hold current position)
 * This is "stop" mode: position control with NaN position
 *
 * @param buffer Output buffer
 * @param motor_id Moteus device ID
 * @return Number of bytes written
 */
size_t build_nan_position_frame(uint8_t* buffer, int motor_id);

/**
 * Build servo off frame (complete power-off, no torque)
 *
 * @param buffer Output buffer
 * @param motor_id Moteus device ID
 * @return Number of bytes written
 */
size_t build_servo_off_frame(uint8_t* buffer, int motor_id);

/**
 * Build set output exact frame (set encoder zero point)
 *
 * @param buffer Output buffer
 * @param motor_id Moteus device ID
 * @param position Position to set (revolutions)
 * @return Number of bytes written
 */
size_t build_set_output_exact_frame(uint8_t* buffer, int motor_id, double position);

/**
 * Parsed motor status from query response
 */
struct MotorQueryResult
{
    uint8_t mode;
    double position;     // revolutions
    double velocity;     // rev/s
    double torque;       // Nm
    double q_current;    // A
    int8_t fault;
};

/**
 * Parse moteus query response
 *
 * @param data Raw CAN data
 * @param len Data length
 * @param result Output parsed result
 * @return true on success
 */
bool parse_query_response(const uint8_t* data, size_t len, MotorQueryResult& result);

#endif // MOTEUS_PROTOCOL_HPP
