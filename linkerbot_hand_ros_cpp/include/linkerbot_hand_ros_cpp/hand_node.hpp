#pragma once
#include "linkerbot_hand_ros_cpp/driver_config.hpp"
#include <functional>
#include <memory>
namespace linkerbot_hand_ros_cpp {
    class HandNode : public rclcpp_lifecycle::LifecycleNode {
    public:
        using DeviceFactory = std::function<std::unique_ptr<hand::HandDevice>(const hand::HandDeviceConfig &)>;
        using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
        explicit HandNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions(), DeviceFactory factory = {});
        ~HandNode() override;
        HandNode(const HandNode &) = delete;
        HandNode &operator=(const HandNode &) = delete;
        HandNode(HandNode &&) = delete;
        HandNode &operator=(HandNode &&) = delete;
        CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;
        CallbackReturn on_error(const rclcpp_lifecycle::State &state) override;

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
} // namespace linkerbot_hand_ros_cpp
