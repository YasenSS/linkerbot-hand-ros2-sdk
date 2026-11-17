#pragma once

#include <linkerbot/hand/api/device.hpp>
#include <linkerbot/hand/discovery.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace linkerbot_hand_ros_cpp {
    namespace hand = linkerbot::hand;

    struct DriverConfig {
        hand::HandDeviceConfig device;
        hand::HandRuntimeConfig runtime;
        hand::DiscoveryConfig discovery;
        bool discover = false;
        double publish_rate_hz = 100.0;
        double diagnostic_rate_hz = 1.0;
        size_t max_tracked_commands = 1024;
        hand::PollingPlan polling;
        std::string transport_json;
        std::string angle_feedback = "query_only";
        bool auto_reconnect = false;
        double reconnect_initial_delay_s = 0.1;
        double reconnect_max_delay_s = 2.0;
    };

    void declare_parameters(rclcpp_lifecycle::LifecycleNode &node);
    DriverConfig read_config(const rclcpp_lifecycle::LifecycleNode &node,
                             const std::vector<rclcpp::Parameter> &overrides = {});
    std::unique_ptr<hand::HandDevice> make_device(const DriverConfig &config);
    bool dynamic_parameter(const std::string &name);
    void apply_discovery_configuration(hand::DiscoveryConfig &config, const std::string &text,
                                       const std::filesystem::path &base = {});
    std::string discovery_configuration(const hand::DiscoveryConfig &config);
    std::string physical_channel(const DriverConfig &config);
    std::string normalize_namespace(const std::string &value, const std::string &parent = "/");
    void validate_polling(const hand::PollingPlan &plan, const hand::DeviceCapabilities &caps);
    hand::Model parse_model(const std::string &input);
    std::string model_name(hand::Model value);
    std::string side_name(hand::HandSide value);
} // namespace linkerbot_hand_ros_cpp
