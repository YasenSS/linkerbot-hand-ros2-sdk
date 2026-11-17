#include "linkerbot_hand_ros_cpp/hand_node.hpp"
#include <exception>
#include <memory>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    int result = 0;
    try {
        auto node = std::make_shared<linkerbot_hand_ros_cpp::HandNode>();
        rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
        executor.add_node(node->get_node_base_interface());
        executor.spin();
        // Stop executor callbacks before releasing the one device owner. Device
        // cleanup does not require an operational ROS context after SIGINT.
        executor.remove_node(node->get_node_base_interface());
        // Synchronize with deferred signal shutdown before destroying ROS entities.
        rclcpp::shutdown();
        node.reset();
    } catch (const std::exception &error) {
        RCLCPP_ERROR(rclcpp::get_logger("linkerbot_hand_ros_cpp"), "%s", error.what());
        result = 1;
    }
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return result;
}
