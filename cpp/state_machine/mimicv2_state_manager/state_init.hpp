#pragma once

#include <cstddef>

// INIT -> SERVO_OFF: robot_config受信時の処理
void handle_robot_config(void* dora_context, const char* data, size_t len);
