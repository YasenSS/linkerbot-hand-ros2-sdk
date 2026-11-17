#pragma once
#include "linkerbot_hand_ros_cpp/conversions.hpp"
#include <linkerbot_hand_msgs/msg/hand_joint_command.hpp>
#include <linkerbot_hand_msgs/msg/hand_raw_command.hpp>
namespace linkerbot_hand_ros_cpp {
    hand::CommandHandle submit_angles(hand::HandDevice &, const msg::HandJointCommand &);
    hand::CommandHandle submit_raw(hand::HandDevice &, hand::CommandSemantic, const std::vector<double> &);
    hand::CommandHandle submit_operation(hand::HandDevice &, hand::DeviceOperation);
    hand::RawVectorSample read_raw(hand::HandDevice &, hand::TelemetrySemantic, std::chrono::milliseconds timeout);
    std::string family_name(hand::CommandFamily value);
    msg::HandCommandStatus command_status(const hand::CommandHandle &handle, uint64_t client_id, uint64_t sequence,
                                          const builtin_interfaces::msg::Time &published,
                                          const builtin_interfaces::msg::Time &received, const rclcpp::Time &now);
} // namespace linkerbot_hand_ros_cpp
