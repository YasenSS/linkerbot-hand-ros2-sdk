"""Start the C++ driver and optionally configure/activate its lifecycle."""

import json
from importlib.resources import files

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.events import Shutdown, matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def _boolean(value, name):
    if value not in ("true", "false"):
        raise ValueError(f"{name} must be true or false")
    return value == "true"


PARAMETERS = json.loads(
    files("linkerbot_hand_ros_cpp").joinpath("public_parameters.json").read_text()
)


def _capture(context):
    context.launch_configurations["_cpp_explicit_arguments"] = json.dumps(
        list(context.launch_configurations)
    )
    return []


def _build(context):
    overrides = {}
    explicit = json.loads(
        context.launch_configurations.get("_cpp_explicit_arguments", "[]")
    )
    for name, spec in PARAMETERS.items():
        value = LaunchConfiguration(name).perform(context)
        if value or name in explicit:
            converters = {
                "str": str,
                "int": int,
                "float": float,
                "bool": lambda v, name=name: _boolean(v, name),
            }
            overrides[name] = converters[spec["type"]](value)
    channels = LaunchConfiguration("channels").perform(context)
    if channels:
        overrides["channels"] = [v.strip() for v in channels.split(",") if v.strip()]
    discovery_config = LaunchConfiguration("discovery_config").perform(context)
    if discovery_config:
        overrides["discovery_config"] = discovery_config
    discover = LaunchConfiguration("discover").perform(context)
    if discover:
        overrides["discover"] = _boolean(discover, "discover")
        if overrides["discover"] and set(explicit) & {
            "model",
            "side",
            "transport_json",
        }:
            raise ValueError(
                "discovery conflicts with explicit model/side/transport_json"
            )
    node = LifecycleNode(
        package="linkerbot_hand_ros_cpp",
        executable="hand_node",
        name="linkerbot_hand",
        namespace=LaunchConfiguration("namespace"),
        parameters=[LaunchConfiguration("params_file"), overrides],
        output="screen",
    )
    if not _boolean(LaunchConfiguration("autostart").perform(context), "autostart"):
        return [node]

    def change(transition):
        return EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=matches_action(node),
                transition_id=transition,
            )
        )

    # Only configure->inactive autostarts. A later manual deactivate stays
    # inactive and does not accidentally re-open the device.
    handlers = [
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=node,
                start_state="configuring",
                goal_state="inactive",
                entities=[change(Transition.TRANSITION_ACTIVATE)],
            )
        )
    ]
    for start, goal in (("configuring", "unconfigured"), ("activating", "inactive")):
        handlers.append(
            RegisterEventHandler(
                OnStateTransition(
                    target_lifecycle_node=node,
                    start_state=start,
                    goal_state=goal,
                    entities=[
                        EmitEvent(event=Shutdown(reason="C++ driver startup failed"))
                    ],
                )
            )
        )
    return [*handlers, node, change(Transition.TRANSITION_CONFIGURE)]


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare("linkerbot_hand_ros_cpp"), "config", "hand.yaml"]
    )
    return LaunchDescription(
        [
            OpaqueFunction(function=_capture),
            DeclareLaunchArgument("params_file", default_value=default_config),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("autostart", default_value="true"),
            *[
                DeclareLaunchArgument(name, default_value="")
                for name in (*PARAMETERS, "discover", "channels", "discovery_config")
            ],
            OpaqueFunction(function=_build),
        ]
    )
