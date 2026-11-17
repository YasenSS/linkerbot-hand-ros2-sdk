#pragma once
#include "linkerbot_hand_ros_cpp/conversions.hpp"
#include <linkerbot_hand_msgs/msg/hand_joint_command.hpp>
#include <linkerbot_hand_msgs/msg/hand_raw_command.hpp>
namespace linkerbot_hand_ros_cpp {
    hand::CommandHandle submit_angles(hand::HandDevice & /*device*/, const msg::HandJointCommand & /*command*/);
    hand::CommandHandle submit_raw(hand::HandDevice & /*device*/, hand::CommandSemantic /*semantic*/,
                                   const std::vector<double> & /*values*/);
    hand::CommandHandle submit_operation(hand::HandDevice & /*device*/, hand::DeviceOperation /*operation*/);
    hand::RawVectorSample read_raw(hand::HandDevice & /*device*/, hand::TelemetrySemantic /*semantic*/,
                                   std::chrono::milliseconds timeout);
    std::string family_name(hand::CommandFamily value);
    struct CommandContext {
        uint64_t client_id{};
        uint64_t sequence{};
        builtin_interfaces::msg::Time published;
        builtin_interfaces::msg::Time received;
    };
    msg::HandCommandStatus command_status(const hand::CommandHandle &handle, const CommandContext &context,
                                          const rclcpp::Time &now);
} // namespace linkerbot_hand_ros_cpp
