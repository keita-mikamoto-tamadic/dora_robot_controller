#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>
#include <utility>

void send_servo_on(void* dora_context, uint8_t mask);
void send_servo_off(void* dora_context, uint8_t mask);
void send_position_commands(void* dora_context, const std::vector<double>& positions);
void send_set_position(void* dora_context, const std::vector<double>& positions);
void send_state_status(void* dora_context, uint8_t progress = 0);
void send_motor_display(void* dora_context);
void send_query(void* dora_context);
void forward_robot_config(void* dora_context, const char* data, size_t len);

// Send velocity commands for specific axes (for wheel velocity control)
// velocities: vector of (axis_index, velocity_rad_s) pairs
// kp_scale: position control gain (0 for pure velocity control)
// kd_scale: derivative gain (default 1.0)
void send_velocity_commands(void* dora_context,
                            const std::vector<std::pair<int, double>>& velocities,
                            double kp_scale, double kd_scale = 1.0);
