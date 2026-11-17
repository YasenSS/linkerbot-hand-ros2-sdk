"""Exercise configuration entry points without importing either Python SDK."""

import json
from pathlib import Path
from unittest.mock import patch

import pytest
import yaml
from launch import LaunchContext
from launch.substitutions import TextSubstitution

from linkerbot_hand_ros_cpp import launch_support as support


def manual():
    return {
        "/linkerbot_hand_container": {
            "ros__parameters": {
                "device_namespaces": ["left", "right"],
                "autostart": False,
            }
        },
        "/**": {"ros__parameters": {"angles_hz": 25.0}},
        "/left/linkerbot_hand": {
            "ros__parameters": {
                "model": "l6",
                "transport_json": '{"backend":"socketcan","channel":"can4"}',
            }
        },
        "/right/linkerbot_hand": {
            "ros__parameters": {"model": "o6", "side": "right", "channel": "can5"}
        },
    }


def deployment():
    return {
        "schema": "linkerbot_hand_ros.discovery",
        "version": 1,
        "autostart": False,
        "discovery": {"backends": ["pcan"], "channels": ["pcan:PCAN_USBBUS1"]},
        "hands": [
            {
                "namespace": "//left/",
                "match": {
                    "backend": "pcan",
                    "channel": "pcan:PCAN_USBBUS1",
                    "model": "L6",
                    "side": "left",
                    "wire_protocol": "can",
                    "address": {
                        "scheme": "classic_can_id",
                        "request_id": 40,
                        "response_id": 40,
                        "is_extended_id": False,
                    },
                },
                "parameters": {"angles_hz": 25.0},
            }
        ],
    }


def test_packaged_and_explicit_multi_configs_have_identical_child_overrides(
    tmp_path, monkeypatch
):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "logs"))
    path = tmp_path / "hands.yaml"
    path.write_text(yaml.safe_dump(manual()))
    context = LaunchContext()
    context.launch_configurations.update(
        {
            name: ""
            for name in (
                "config",
                "channels",
                "discovery_config",
                "output",
                "autostart",
            )
        }
    )
    context.launch_configurations["discover"] = "false"
    with patch.object(support, "Node", side_effect=lambda **kwargs: kwargs):
        default = support.build_multi(context, TextSubstitution(text=str(path)))[0]
        context.launch_configurations["config"] = str(path)
        explicit = support.build_multi(context, TextSubstitution(text="unused"))[0]
    assert default == explicit
    values = {key: value for row in default["parameters"] for key, value in row.items()}
    devices = json.loads(values["device_parameters_json"])
    assert devices["/left"]["angles_hz"] == 25.0
    assert devices["/left"]["transport_json"].endswith('"can4"}')
    assert devices["/right"]["side"] == "right"
    assert values["autostart"] is False
    assert values["device_namespaces"] == ["/left", "/right"]


def test_manual_config_rejects_normalized_namespace_and_transport_alias_duplicates():
    value = manual()
    value["/linkerbot_hand_container"]["ros__parameters"]["device_namespaces"] = [
        "left",
        "//left/",
    ]
    with pytest.raises(ValueError, match="duplicate namespace"):
        support.manual_configuration(value)
    value = manual()
    value["/right/linkerbot_hand"]["ros__parameters"]["transport_json"] = (
        '{"backend":"python_can","interface_type":"socketcan","interface_name":"can4"}'
    )
    with pytest.raises(ValueError, match="duplicate physical channel"):
        support.manual_configuration(value)


@pytest.mark.parametrize(
    "parameters",
    [
        {"angle_feedback": "adaptive"},
        {"mailbox_capacity": 10},
        {"route_queue_capacity": 10},
        {"recovery_timeout_s": 3.0},
        {"not_a_parameter": 1},
        {"channel": "can9"},
    ],
)
def test_deployment_rejects_unsupported_runtime_settings_before_scan(
    tmp_path, parameters
):
    value = deployment()
    value["hands"][0]["parameters"] = parameters
    path = tmp_path / "hands.yaml"
    path.write_text(yaml.safe_dump(value))
    with pytest.raises(ValueError):
        support.load_configuration(path)


def test_deployment_preserves_runtime_settings_and_normalizes_namespaces(tmp_path):
    value = deployment()
    value["hands"][0]["parameters"]["angle_feedback"] = "query_only"
    path = tmp_path / "hands.yaml"
    path.write_text(yaml.safe_dump(value))
    actual = support.load_configuration(path)
    assert actual["discovery"] == value["discovery"]
    assert actual["hands"][0]["namespace"] == "/left"
    assert actual["hands"][0]["parameters"] == value["hands"][0]["parameters"]


def test_deployment_duplicate_namespaces_and_unknown_identity_fields_reject(tmp_path):
    value = deployment()
    other = json.loads(json.dumps(value["hands"][0]))
    other["namespace"] = "/left"
    other["match"]["channel"] = "pcan:PCAN_USBBUS2"
    value["hands"].append(other)
    path = tmp_path / "hands.yaml"
    path.write_text(yaml.safe_dump(value))
    with pytest.raises(ValueError, match="duplicate namespace"):
        support.load_configuration(path)
    value = deployment()
    value["hands"][0]["match"]["build_time"] = "unavailable"
    path.write_text(yaml.safe_dump(value))
    with pytest.raises(ValueError, match="unsupported_operation"):
        support.load_configuration(path)


def _container(arguments):
    import os
    import subprocess

    from ament_index_python.packages import get_package_prefix

    executable = (
        Path(get_package_prefix("linkerbot_hand_ros_cpp"))
        / "lib/linkerbot_hand_ros_cpp/hand_container"
    )
    env = dict(
        os.environ, ROS_LOG_DIR="/tmp/linkerbot-cpp-preflight-logs", ROS_DOMAIN_ID="89"
    )
    result = subprocess.run(
        check=False,
        args=[str(executable), "--ros-args", *arguments],
        text=True,
        capture_output=True,
        timeout=8,
        env=env,
    )
    assert result.returncode != 0
    return result.stdout + result.stderr


def test_installed_container_rejects_feedback_policy_without_scanning_any_device():
    # Even a regressed preflight can only attempt a nonexistent channel.
    scan = json.dumps(
        {"backends": ["socketcan"], "channels": ["cpp_test_no_such_interface"]}
    )
    output = _container(
        [
            "-p",
            "discovery_mode:=multi",
            "-p",
            "discovery_config_json:='" + scan + "'",
            "-p",
            'driver_parameters_json:=\'{"angle_feedback":"adaptive"}\'',
            "-p",
            "discovery_output:=/tmp/linkerbot-cpp-unused-preflight-output.yaml",
        ]
    )
    assert "unsupported_operation" in output and "feedback-policy control" in output


def test_installed_container_refuses_existing_output_before_discovery(tmp_path):
    path = tmp_path / "hands.yaml"
    path.write_text("keep existing deployment")
    output = _container(
        ["-p", "discovery_mode:=multi", "-p", f"discovery_output:={path}"]
    )
    assert "refusing overwrite" in output
    assert path.read_text() == "keep existing deployment"


def test_installed_container_file_alias_rejects_duplicate_yaml_before_discovery(
    tmp_path,
):
    path = tmp_path / "hands.yaml"
    path.write_text("schema: first\nschema: second\n")
    output = _container(
        ["-p", "discovery_mode:=file", "-p", f"deployment_config:={path}"]
    )
    assert "duplicate YAML key" in output


@pytest.mark.parametrize(
    "content", ["&root {child: *root}", "root: " + "[" * 66 + "0" + "]" * 66]
)
def test_installed_container_rejects_cyclic_or_deep_yaml_before_discovery(
    tmp_path, content
):
    path = tmp_path / "hands.yaml"
    path.write_text(content)
    output = _container(
        ["-p", "discovery_mode:=file", "-p", f"deployment_config:={path}"]
    )
    assert "YAML nesting exceeds 64 levels" in output
