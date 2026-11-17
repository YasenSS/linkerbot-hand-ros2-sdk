"""Start a C++ multi-hand container, including discovery/deployment entry points."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from linkerbot_hand_ros_cpp.launch_support import build_multi


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare("linkerbot_hand_ros_cpp"), "config", "multi_hand.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=""),
            DeclareLaunchArgument("discover", default_value="false"),
            DeclareLaunchArgument("channels", default_value=""),
            DeclareLaunchArgument("discovery_config", default_value=""),
            DeclareLaunchArgument("output", default_value=""),
            DeclareLaunchArgument("autostart", default_value=""),
            OpaqueFunction(function=build_multi, kwargs={"default_config": config}),
        ]
    )
