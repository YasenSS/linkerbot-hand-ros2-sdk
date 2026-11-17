"""Prepare multi-device configurations without opening any device."""

import json
import re
from importlib.resources import files
from pathlib import Path

import yaml
from launch_ros.actions import Node


class StrictLoader(yaml.SafeLoader):
    pass


def _mapping(loader, node):
    result = {}
    for key, value in node.value:
        name = loader.construct_object(key)
        if name in result:
            raise ValueError(f"duplicate YAML key: {name}")
        result[name] = loader.construct_object(value)
    return result


StrictLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _mapping)


def normalize_namespace(value):
    if not isinstance(value, str):
        raise TypeError("namespace must be a string")
    name = "/".join(part for part in value.split("/") if part)
    if not re.fullmatch(r"[A-Za-z_][A-Za-z_0-9]*(/[A-Za-z_][A-Za-z_0-9]*)*", name):
        raise ValueError(f"invalid namespace: {value!r}")
    return "/" + name


def _parameters(value, *, discovered=False):
    if not isinstance(value, dict):
        raise TypeError("hand parameters must be a mapping")
    public = json.loads(
        files("linkerbot_hand_ros_cpp").joinpath("public_parameters.json").read_text()
    )
    extra = {"use_sim_time", "discover", "channels", "discovery_config"}
    unknown = value.keys() - public.keys() - extra
    if unknown:
        raise ValueError(
            f"unsupported_operation: unknown hand parameters {sorted(unknown)}"
        )
    if discovered and value.keys() & {
        "model",
        "side",
        "transport_json",
        "channel",
        "address",
        "host_address",
        "discover",
        "channels",
        "discovery_config",
    }:
        raise ValueError("discovered hand parameters must not override its endpoint")
    if value.get("angle_feedback", "query_only") != "query_only":
        raise ValueError(
            "unsupported_operation: C++ SDK has no angle_feedback policy control"
        )
    for name, default in (
        ("mailbox_capacity", 256),
        ("route_queue_capacity", 256),
        ("recovery_timeout_s", 2.0),
    ):
        if name in value and value[name] != default:
            raise ValueError(
                f"unsupported_operation: no independent SDK field for {name}"
            )
    if str(value.get("model", "")).lower() == "o20":
        raise ValueError(
            "unsupported_operation: model o20 has no C++ HandDevice backend"
        )
    return dict(value)


def physical_channel(parameters):
    transport = parameters.get("transport_json", "")
    if not transport:
        return ("socketcan", parameters.get("channel", "can0"))
    value = json.loads(transport, object_pairs_hook=_unique_json)
    if not isinstance(value, dict):
        raise TypeError("transport_json must be an object")
    backend = value.get("backend")
    if backend == "socketcan":
        return (backend, value.get("channel"))
    if backend == "python_can":
        return (value.get("interface_type"), value.get("interface_name"))
    if backend == "vendor_canfd":
        return (
            "minicanfd",
            value.get("device_index", 0),
            value.get("channel_index", 0),
        )
    raise ValueError(f"unsupported_operation: transport backend={backend}")


def _unique_json(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def load_configuration(path):
    with Path(path).open() as stream:
        value = yaml.load(stream, Loader=StrictLoader)
    if not isinstance(value, dict):
        raise TypeError("configuration must be a mapping")
    if "schema" in value:
        if (
            value["schema"] != "linkerbot_hand_ros.discovery"
            or type(value.get("version")) is not int
            or value["version"] != 1
        ):
            raise ValueError("unsupported deployment schema/version")
        if value.keys() - {"schema", "version", "autostart", "discovery", "hands"}:
            raise ValueError("unknown deployment fields")
        if "autostart" in value and not isinstance(value["autostart"], bool):
            raise ValueError("autostart must be boolean")
        if "discovery" in value and not isinstance(value["discovery"], dict):
            raise ValueError("discovery must be a mapping")
        hands = value.get("hands")
        if not isinstance(hands, list) or not hands:
            raise ValueError("deployment hands must be a nonempty list")
        namespaces, endpoints = set(), set()
        for hand in hands:
            if not isinstance(hand, dict) or set(hand) != {
                "namespace",
                "match",
                "parameters",
            }:
                raise ValueError("hand requires namespace, match, parameters")
            name = normalize_namespace(hand["namespace"])
            match = hand["match"]
            if (
                not isinstance(match, dict)
                or not {
                    "backend",
                    "channel",
                    "model",
                    "side",
                    "wire_protocol",
                    "address",
                }
                <= match.keys()
            ):
                raise ValueError("incomplete discovery identity match")
            supported = {
                "backend",
                "channel",
                "model",
                "side",
                "wire_protocol",
                "address",
                "serial_number",
                "product_code",
            }
            if match.keys() - supported:
                raise ValueError(
                    "unsupported_operation: no C++ SDK identity field for match"
                )
            if not isinstance(match["address"], dict) or match["address"].keys() - {
                "scheme",
                "request_id",
                "response_id",
                "is_extended_id",
                "node_id",
                "host_id",
            }:
                raise ValueError("unsupported_operation: unsupported address match")
            endpoint = (match["backend"], match["channel"])
            if name in namespaces or endpoint in endpoints:
                raise ValueError("duplicate namespace or physical channel")
            namespaces.add(name)
            endpoints.add(endpoint)
            hand["namespace"] = name
            hand["parameters"] = _parameters(hand["parameters"], discovered=True)
    return value


def manual_configuration(content):
    """Return explicit child overrides for both packaged and user YAML files."""
    container = next(
        (v for k, v in content.items() if k.strip("/") == "linkerbot_hand_container"),
        {},
    )
    values = container.get("ros__parameters", {})
    names = values.get("device_namespaces", [])
    if not isinstance(names, list) or not names:
        raise ValueError("device_namespaces must be nonempty and unique")
    devices, endpoints = {}, set()
    common = content.get("/**", {}).get("ros__parameters", {})
    for name in names:
        normalized = normalize_namespace(name)
        if normalized in devices:
            raise ValueError("duplicate namespace")
        child = next(
            (
                v
                for k, v in content.items()
                if "/" + "/".join(part for part in k.split("/") if part)
                == normalized + "/linkerbot_hand"
            ),
            None,
        )
        if not isinstance(child, dict) or not isinstance(
            child.get("ros__parameters"), dict
        ):
            raise TypeError(f"missing node parameters: {normalized}/linkerbot_hand")
        # Full SDK validation is performed by each node during configure.
        parameters = dict(common, **child["ros__parameters"])
        endpoint = physical_channel(parameters)
        if endpoint in endpoints:
            raise ValueError("duplicate physical channel")
        endpoints.add(endpoint)
        devices[normalized] = parameters
    return values, devices


def build_multi(context, default_config):
    from launch.substitutions import LaunchConfiguration

    def get(name):
        return LaunchConfiguration(name).perform(context)

    discover = get("discover")
    if discover not in ("true", "false"):
        raise ValueError("discover must be true or false")
    path = get("config")
    parameters = []
    if not path and discover == "false":
        path = default_config.perform(context)
    if path:
        path = str(Path(path).resolve())
        content = load_configuration(path)
        if "schema" in content:
            parameters.append(
                {
                    "deployment_json": json.dumps(content),
                    "deployment_base_dir": str(Path(path).parent),
                }
            )
        else:
            if discover == "true":
                raise ValueError("manual config conflicts with discover:=true")
            values, devices = manual_configuration(content)
            parameters.extend(
                [
                    values,
                    {
                        "device_namespaces": list(devices),
                        "device_parameters_json": json.dumps(devices),
                    },
                ]
            )
    options = {"discover": discover == "true"}
    if get("channels"):
        options["channels"] = [
            v.strip() for v in get("channels").split(",") if v.strip()
        ]
    if get("discovery_config"):
        options["discovery_config"] = str(Path(get("discovery_config")).resolve())
    if get("output"):
        options["output"] = str(Path(get("output")).resolve())
    if (
        discover == "false"
        and not any("deployment_json" in p for p in parameters)
        and set(options) - {"discover"}
    ):
        raise ValueError("channels/discovery_config/output require discovery")
    if get("autostart"):
        if get("autostart") not in ("true", "false"):
            raise ValueError("autostart must be true or false")
        options["autostart"] = get("autostart") == "true"
    parameters.append(options)
    return [
        Node(
            package="linkerbot_hand_ros_cpp",
            executable="hand_container",
            name="linkerbot_hand_container",
            parameters=parameters,
            output="screen",
        )
    ]
