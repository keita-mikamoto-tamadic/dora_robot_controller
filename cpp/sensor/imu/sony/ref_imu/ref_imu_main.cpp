#include <rclcpp/rclcpp.hpp>
#include "alubus_sensor_node/spresense_imu_receiver.hpp"
#include <memory>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<SpresenseImuReceiver>();
        RCLCPP_INFO(node->get_logger(), "Spresense IMU Receiver Node started");
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Error: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}