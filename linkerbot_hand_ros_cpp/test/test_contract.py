"""Check public parity against the protected Python package without importing it."""

import ast
import json
import os
import signal
import subprocess
import time
from importlib.resources import files
from pathlib import Path

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from lifecycle_msgs.srv import GetState
from rcl_interfaces.srv import ListParameters

ROOT = Path(__file__).resolve().parents[2]
PYTHON = Path(
    os.environ.get("LINKERBOT_PYTHON_ROS_BASELINE", str(ROOT / "linkerbot_hand_ros"))
)


def test_all_python_launch_parameters_have_cpp_entries():
    tree = ast.parse((PYTHON / "launch/hand.launch.py").read_text())
    rows = next(
        node.value
        for node in tree.body
        if isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Name) and target.id == "_ARGUMENTS"
            for target in node.targets
        )
    )
    expected = {row.elts[0].value for row in rows.elts}
    actual = json.loads(
        files("linkerbot_hand_ros_cpp").joinpath("public_parameters.json").read_text()
    )
    assert expected <= actual.keys()


def test_client_public_methods_and_tool_options_are_preserved():
    source = PYTHON / "linkerbot_hand_ros"
    target = ROOT / "linkerbot_hand_ros_cpp/linkerbot_hand_ros_cpp"

    def methods(path):
        tree = ast.parse(path.read_text())
        cls = next(
            node
            for node in tree.body
            if isinstance(node, ast.ClassDef) and node.name == "HandCommandClient"
        )
        return {
            node.name: (
                ast.dump(node.args),
                ast.dump(node.returns) if node.returns else None,
                tuple(ast.dump(value) for value in node.decorator_list),
            )
            for node in cls.body
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
            and (not node.name.startswith("_") or node.name == "__init__")
        }

    expected_methods = methods(source / "client.py")
    actual_methods = methods(target / "client.py")
    assert expected_methods.keys() <= actual_methods.keys()
    assert all(
        actual_methods[name] == signature
        for name, signature in expected_methods.items()
    )
    for name in (
        "angle_command",
        "angle_controller",
        "state_monitor",
        "frequency_benchmark",
    ):
        executable = (
            Path(get_package_prefix("linkerbot_hand_ros_cpp"))
            / "lib/linkerbot_hand_ros_cpp"
            / name
        )
        result = subprocess.run(
            check=False,
            args=[str(executable), "--help"],
            text=True,
            capture_output=True,
            timeout=10,
        )
        assert result.returncode == 0, result.stderr
        tree = ast.parse((source / f"examples/{name}.py").read_text())
        flags = {
            arg.value
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "add_argument"
            for arg in node.args
            if isinstance(arg, ast.Constant)
            and isinstance(arg.value, str)
            and arg.value.startswith("--")
        }
        assert all(flag in result.stdout for flag in flags)
    for path in target.rglob("*.py"):
        tree = ast.parse(path.read_text())
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom) and node.module:
                assert node.module.split(".")[0] not in {
                    "linkerbot_hand",
                    "linkerbot_hand_ros",
                }


@pytest.fixture
def public_node(tmp_path):
    os.environ.setdefault("ROS_DOMAIN_ID", "89")
    namespace = f"cpp_public_contract_{os.getpid()}"
    path = tmp_path / "node.log"
    with path.open("w") as stream:
        process = subprocess.Popen(
            [
                "ros2",
                "launch",
                "linkerbot_hand_ros_cpp",
                "hand.launch.py",
                "autostart:=false",
                f"namespace:={namespace}",
            ],
            stdout=stream,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        rclpy.init()
        node = rclpy.create_node("cpp_public_contract_client")
        prefix = f"/{namespace}/linkerbot_hand"
        try:
            service = node.create_client(GetState, f"{prefix}/get_state")
            assert service.wait_for_service(timeout_sec=10), path.read_text()
            yield node, prefix
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


def test_all_public_services_topics_and_parameters_exist_without_device(public_node):
    node, prefix = public_node
    expected_topics, expected_services = python_ros_contract()
    deadline = time.monotonic() + 5
    observed_topics, observed_services = {}, {}
    while time.monotonic() < deadline:
        observed_services = dict(node.get_service_names_and_types())
        observed_topics = dict(node.get_topic_names_and_types())
        if all(
            prefix + name in observed_services for name in expected_services
        ) and all(
            (prefix + name if name.startswith("/") and name != "/diagnostics" else name)
            in observed_topics
            for name in expected_topics
        ):
            break
        rclpy.spin_once(node, timeout_sec=0.05)
    for name, service_type in expected_services.items():
        assert observed_services.get(prefix + name) == [
            "linkerbot_hand_msgs/srv/" + service_type
        ]
    for name, topic_type in expected_topics.items():
        actual = name if name == "/diagnostics" else prefix + name
        assert observed_topics.get(actual) == [topic_type]
    service = node.create_client(ListParameters, f"{prefix}/list_parameters")
    assert service.wait_for_service(timeout_sec=5)
    future = service.call_async(ListParameters.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=5)
    assert future.done()
    expected = json.loads(
        files("linkerbot_hand_ros_cpp").joinpath("public_parameters.json").read_text()
    )
    assert expected.keys() <= set(future.result().result.names)


def test_multi_hand_configuration_failure_rolls_back_before_activation(tmp_path):
    # The second model has no SDK backend. All configure steps must finish
    # before ANY activation, so this test never opens a physical interface.
    namespaces = [f"group_good_{os.getpid()}", f"group_missing_{os.getpid()}"]
    config = tmp_path / "hands.yaml"
    config.write_text(
        "/linkerbot_hand_container:\n  ros__parameters:\n"
        f"    device_namespaces: [{', '.join(namespaces)}]\n    autostart: true\n"
        f"/{namespaces[0]}/linkerbot_hand:\n  ros__parameters:\n    model: l6\n    side: left\n    channel: can10\n"
        f"/{namespaces[1]}/linkerbot_hand:\n  ros__parameters:\n    model: o20\n    channel: can11\n"
    )
    result = subprocess.run(
        check=False,
        args=[
            "ros2",
            "launch",
            "linkerbot_hand_ros_cpp",
            "multi_hand.launch.py",
            f"config:={config}",
        ],
        text=True,
        capture_output=True,
        timeout=15,
    )
    assert "no C++ HandDevice backend" in result.stdout + result.stderr
    assert "group configure failed" in result.stdout + result.stderr


def test_deployment_parser_rejects_duplicate_keys_and_unknown_schema(tmp_path):
    from linkerbot_hand_ros_cpp.launch_support import load_configuration

    path = tmp_path / "bad.yaml"
    path.write_text("schema: one\nschema: two\n")
    with pytest.raises(ValueError, match="duplicate"):
        load_configuration(path)
    path.write_text("schema: unsupported\nversion: 123\n")
    with pytest.raises(ValueError, match="schema"):
        load_configuration(path)


def _assignment(path, name):
    tree = ast.parse(path.read_text())
    for node in tree.body:
        if isinstance(node, (ast.Assign, ast.AnnAssign)):
            targets = node.targets if isinstance(node, ast.Assign) else [node.target]
            if any(
                isinstance(target, ast.Name) and target.id == name for target in targets
            ):
                return node.value
    raise AssertionError(f"missing baseline declaration: {name}")


def python_ros_contract():
    """Derive the baseline from ROS source only, without importing its SDK."""
    source = PYTHON / "linkerbot_hand_ros"
    topics = {}
    for filename in (
        "state_publisher.py",
        "command_interface.py",
        "driver_interfaces.py",
    ):
        tree = ast.parse((source / filename).read_text())
        for call in ast.walk(tree):
            if not isinstance(call, ast.Call) or not isinstance(
                call.func, ast.Attribute
            ):
                continue
            if (
                call.func.attr
                not in {
                    "create_publisher",
                    "create_lifecycle_publisher",
                    "create_subscription",
                }
                or len(call.args) < 2
            ):
                continue
            message, name = call.args[:2]
            if (
                isinstance(message, ast.Name)
                and isinstance(name, ast.Constant)
                and isinstance(name.value, str)
            ):
                suffix = name.value.removeprefix("~")
                package = (
                    "diagnostic_msgs"
                    if message.id == "DiagnosticArray"
                    else "linkerbot_hand_msgs"
                )
                topics[suffix] = f"{package}/msg/{message.id}"
    for semantic in _assignment(source / "command_interface.py", "_RAW_COMMANDS").elts:
        topics["/command/" + semantic.attr.lower()] = (
            "linkerbot_hand_msgs/msg/HandRawCommand"
        )
    for semantic in _assignment(
        source / "state_publisher.py", "_SAMPLE_ATTRIBUTES"
    ).keys:
        topics["/state/" + semantic.attr.lower()] = (
            "linkerbot_hand_msgs/msg/HandTelemetry"
        )
    rates = _assignment(source / "driver_config.py", "POLL_RATE_PARAMETERS")
    for name in rates.keys:
        if name.value.startswith("force_") and name.value != "force_hz":
            topics["/state/" + name.value.removesuffix("_hz")] = (
                "linkerbot_hand_msgs/msg/HandForceField"
            )
    services = {
        "/get_device_info": "GetDeviceInfo",
        "/get_stall_protection": "GetStallProtection",
        "/recover": "Recover",
    }
    for name in ("_STALL_SETTERS", "_CONFIG_SETTERS"):
        for key in _assignment(source / "management.py", name).keys:
            if key is not None:
                services["/set_" + key.value] = "SetRawValues"
    for value in _assignment(source / "management.py", "_RAW_GETTERS").values:
        services["/" + value.value] = "GetRawValues"
    tree = ast.parse((source / "command_router.py").read_text())
    operations = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "DiscreteOperation"
    )
    for node in operations.body:
        if isinstance(node, ast.Assign):
            services["/" + node.value.value] = "SubmitCommand"
    return topics, services


def test_baseline_contract_extraction_is_complete():
    topics, services = python_ros_contract()
    assert len(services) == 24
    assert topics["/command/angles"] == "linkerbot_hand_msgs/msg/HandJointCommand"
    assert (
        topics["/state/fault_statuses"]
        == "linkerbot_hand_msgs/msg/HandFaultStatusState"
    )
    assert topics["/state/force_px"] == "linkerbot_hand_msgs/msg/HandForceField"


def test_multi_launch_arguments_cover_python_baseline():
    def arguments(path):
        return {
            node.args[0].value
            for node in ast.walk(ast.parse(path.read_text()))
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "DeclareLaunchArgument"
            and node.args
            and isinstance(node.args[0], ast.Constant)
        }

    target = ROOT / "linkerbot_hand_ros_cpp/launch/multi_hand.launch.py"
    assert arguments(PYTHON / "launch/multi_hand.launch.py") <= arguments(target)


def test_installed_helpers_import_with_python_sdk_and_driver_explicitly_blocked():
    script = r"""
import importlib
import importlib.abc
import sys
class NoPythonSdk(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {'linkerbot_hand', 'linkerbot_hand_ros'}:
            raise ImportError('protected dependency is unavailable: ' + fullname)
sys.meta_path.insert(0, NoPythonSdk())
for name in ('client', 'benchmark', 'driver_readiness', 'qos', 'launch_support',
             'examples.angle_command', 'examples.angle_controller',
             'examples.state_monitor', 'examples.frequency_benchmark'):
    importlib.import_module('linkerbot_hand_ros_cpp.' + name)
"""
    result = subprocess.run(
        check=False,
        args=["/usr/bin/python3", "-c", script],
        text=True,
        capture_output=True,
        timeout=10,
    )
    assert result.returncode == 0, result.stderr
