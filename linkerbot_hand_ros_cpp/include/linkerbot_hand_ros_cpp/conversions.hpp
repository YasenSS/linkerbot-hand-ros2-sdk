#pragma once
#include "linkerbot_hand_ros_cpp/driver_config.hpp"
#include <linkerbot_hand_msgs/msg/hand_capabilities.hpp>
#include <linkerbot_hand_msgs/msg/hand_command_status.hpp>
#include <linkerbot_hand_msgs/msg/hand_device_info.hpp>
#include <linkerbot_hand_msgs/msg/hand_fault_status_state.hpp>
#include <linkerbot_hand_msgs/msg/hand_force_field.hpp>
#include <linkerbot_hand_msgs/msg/hand_joint_state.hpp>
#include <linkerbot_hand_msgs/msg/hand_normal_force_resultant.hpp>
#include <linkerbot_hand_msgs/msg/hand_tactile_sensor_state.hpp>
#include <linkerbot_hand_msgs/msg/hand_tactile_state.hpp>
#include <linkerbot_hand_msgs/msg/hand_telemetry.hpp>

namespace linkerbot_hand_ros_cpp {
    namespace msg = linkerbot_hand_msgs::msg;
    std::string semantic_name(hand::CommandSemantic value);
    std::string semantic_name(hand::TelemetrySemantic value);
    builtin_interfaces::msg::Time project_time(std::chrono::steady_clock::time_point value, const rclcpp::Time &now);
    msg::SampleMetadata metadata(const hand::SampleMeta &value, const rclcpp::Time &now);
    msg::HandCapabilities capabilities(const hand::DeviceCapabilities &caps,
                                       const std::optional<hand::DeviceIdentity> &identity);
    msg::HandDeviceInfo device_info(const hand::DeviceIdentity &value, const rclcpp::Time &now, uint64_t generation);
    msg::HandJointState angles(const hand::AngleSample &value, const hand::DeviceCapabilities &caps,
                               const rclcpp::Time &now);
    msg::HandTelemetry telemetry(const hand::RawVectorSample &value, const hand::TelemetryCapability &caps,
                                 const rclcpp::Time &now);
    msg::HandForceField force_field(const hand::ForceFieldSample &value, const rclcpp::Time &now);
    msg::HandTactileState tactile(const hand::ForceSample &value, const hand::TelemetryCapability &caps,
                                  const rclcpp::Time &now);
    msg::HandTactileSensorState sensors(const hand::ForceSample &value, const rclcpp::Time &now);
    msg::HandNormalForceResultant normal_force(const hand::NormalForceResultantSample &value, const rclcpp::Time &now);
    uint8_t command_stage(hand::CommandStage value);
    std::string error_code(const std::exception &error);
    bool state_supported(const std::string &name, const hand::DeviceCapabilities &caps,
                         const std::optional<hand::DeviceIdentity> &identity = {});
    msg::HandFaultStatusState fault_statuses(const hand::FaultSample &sample, const hand::DeviceCapabilities &caps,
                                             const std::optional<hand::DeviceIdentity> &identity,
                                             const rclcpp::Time &now);
} // namespace linkerbot_hand_ros_cpp
