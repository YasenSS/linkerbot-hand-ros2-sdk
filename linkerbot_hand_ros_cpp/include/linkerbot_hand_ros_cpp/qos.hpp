#pragma once
#include <rclcpp/qos.hpp>
#include <rclcpp/version.h>
namespace linkerbot_hand_ros_cpp {
    inline rclcpp::QoS state_qos() { return rclcpp::QoS(1).best_effort(); }
    inline rclcpp::QoS command_qos() { return rclcpp::QoS(1).best_effort(); }
    inline rclcpp::QoS status_qos() { return rclcpp::QoS(32).reliable(); }
    inline rclcpp::QoS metadata_qos() { return rclcpp::QoS(1).reliable().transient_local(); }
    inline auto service_qos() {
#if RCLCPP_VERSION_MAJOR >= 28
        return rclcpp::ServicesQoS();
#else
        return rmw_qos_profile_services_default;
#endif
    }
} // namespace linkerbot_hand_ros_cpp
