#include "state_init.hpp"
#include "types.hpp"
#include "helpers.hpp"
#include "nlohmann/json.hpp"
#include <iostream>

using json = nlohmann::json;

void handle_robot_config(void* dora_context, const char* data, size_t len)
{
    if (g_config_received || len == 0) return;

    try
    {
        std::string json_str(data, len);
        json config = json::parse(json_str);

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

        std::cout << "[state_manager] Config: " << config["robot_name"].get<std::string>()
                  << " (" << g_axes.size() << " axes), interp=" << g_interpolation_time << "s" << std::endl;

        forward_robot_config(dora_context, data, len);

        // INIT -> SERVO_OFF
        g_current_state = State::SERVO_OFF;
        send_state_status(dora_context, 100);
        std::cout << "[state_manager] INIT -> SERVO_OFF" << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "[state_manager] Config error: " << e.what() << std::endl;
    }
}
