#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>

void send_servo_on(void* dora_context, uint8_t mask);
void send_servo_off(void* dora_context, uint8_t mask);
void send_position_commands(void* dora_context, const std::vector<double>& positions);
void send_state_status(void* dora_context, uint8_t progress = 0);
void send_motor_display(void* dora_context);
void send_query(void* dora_context);
void forward_robot_config(void* dora_context, const char* data, size_t len);
