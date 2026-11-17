"""Run the installed launch file without activating any hardware connection."""

import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState


def test_manual_launch_keeps_configured_driver_inactive(tmp_path):
    os.environ.setdefault("ROS_DOMAIN_ID", "89")
    namespace = f"cpp_launch_test_{os.getpid()}"
    log_path = tmp_path / "launch.log"
    with log_path.open("w") as log:
        process = subprocess.Popen(
            [
                "ros2",
                "launch",
                "linkerbot_hand_ros_cpp",
                "hand.launch.py",
                "autostart:=false",
                f"namespace:={namespace}",
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        rclpy.init()
        node = rclpy.create_node("cpp_launch_test_client")
        try:
            prefix = f"/{namespace}/linkerbot_hand"
            get_state = node.create_client(GetState, f"{prefix}/get_state")
            change_state = node.create_client(ChangeState, f"{prefix}/change_state")
            assert get_state.wait_for_service(timeout_sec=10), log_path.read_text()
            assert change_state.wait_for_service(timeout_sec=5), log_path.read_text()

            def call(client, request):
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=5)
                assert future.done(), log_path.read_text()
                return future.result()

            assert (
                call(get_state, GetState.Request()).current_state.id
                == State.PRIMARY_STATE_UNCONFIGURED
            )
            request = ChangeState.Request()
            request.transition.id = Transition.TRANSITION_CONFIGURE
            assert call(change_state, request).success, log_path.read_text()
            # Configure constructs the facade only; autostart=false must not
            # open can0, even after lifecycle events have propagated.
            time.sleep(0.2)
            assert (
                call(get_state, GetState.Request()).current_state.id
                == State.PRIMARY_STATE_INACTIVE
            )
            request.transition.id = Transition.TRANSITION_CLEANUP
            assert call(change_state, request).success
        finally:
            node.destroy_node()
            rclpy.shutdown()
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
        assert process.returncode == 0, log_path.read_text()


def test_autostart_configuration_failure_shuts_down(tmp_path):
    os.environ.setdefault("ROS_DOMAIN_ID", "89")
    log_path = Path(tmp_path) / "failure.log"
    with log_path.open("w") as log:
        process = subprocess.Popen(
            [
                "ros2",
                "launch",
                "linkerbot_hand_ros_cpp",
                "hand.launch.py",
                "model:=invalid_model",
                f"namespace:=cpp_launch_failure_{os.getpid()}",
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            process.wait(timeout=15)
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        assert "unsupported model: invalid_model" in log_path.read_text()
        assert process.returncode == 0, log_path.read_text()


def test_packaged_multi_config_preserves_child_settings_and_container_parameters(
    tmp_path,
):
    from rcl_interfaces.srv import GetParameters, ListParameters

    path = tmp_path / "multi.log"
    with path.open("w") as stream:
        process = subprocess.Popen(
            ["ros2", "launch", "linkerbot_hand_ros_cpp", "multi_hand.launch.py"],
            stdout=stream,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        rclpy.init()
        node = rclpy.create_node("cpp_multi_launch_test_client")
        try:

            def call(service_type, name, request):
                client = node.create_client(service_type, name)
                assert client.wait_for_service(timeout_sec=10), path.read_text()
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=5)
                assert future.done(), path.read_text()
                result = future.result()
                node.destroy_client(client)
                return result

            for namespace, model, channel in (
                ("l6_left", "l6", "can0"),
                ("o6_right", "o6", "can1"),
            ):
                prefix = f"/{namespace}/linkerbot_hand"
                assert (
                    call(
                        GetState, prefix + "/get_state", GetState.Request()
                    ).current_state.id
                    == State.PRIMARY_STATE_UNCONFIGURED
                )
                request = GetParameters.Request()
                request.names = ["model", "transport_json"]
                values = call(GetParameters, prefix + "/get_parameters", request).values
                assert values[0].string_value == model
                assert channel in values[1].string_value
            values = call(
                ListParameters,
                "/linkerbot_hand_container/list_parameters",
                ListParameters.Request(),
            ).result.names
            assert {
                "device_namespaces",
                "autostart",
                "discovery_mode",
                "discovery_config_json",
                "driver_parameters_json",
                "discovery_namespace",
                "discovery_output",
                "deployment_config",
                "autostart_override",
            } <= set(values)
        finally:
            node.destroy_node()
            rclpy.shutdown()
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
        assert process.returncode == 0, path.read_text()
