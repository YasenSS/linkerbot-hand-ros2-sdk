#include "linkerbot_hand_ros_cpp/driver_config.hpp"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <jsoncpp/json/json.h>
#include <limits>
#include <linkerbot/hand/adapter_transport.hpp>
#include <map>
#include <set>
#include <yaml-cpp/yaml.h>

namespace linkerbot_hand_ros_cpp {
    namespace {
        struct Rate {
            const char *name;
            hand::TelemetrySemantic semantic;
            double initial;
        };
        const Rate kRates[] = {
            {"angles_hz", hand::TelemetrySemantic::kAngles, 100.0},
            {"speeds_hz", hand::TelemetrySemantic::kMeasuredSpeeds, 0.0},
            {"current_hz", hand::TelemetrySemantic::kCurrents, 0.0},
            {"temperature_hz", hand::TelemetrySemantic::kTemperatures, 0.0},
            {"faults_hz", hand::TelemetrySemantic::kFaults, 0.0},
            {"voltage_hz", hand::TelemetrySemantic::kVoltages, 0.0},
            {"torques_hz", hand::TelemetrySemantic::kTorques, 0.0},
            {"force_hz", hand::TelemetrySemantic::kForce, 0.0},
            {"force_fns_hz", hand::TelemetrySemantic::kForceFns, 0.0},
            {"force_ft_hz", hand::TelemetrySemantic::kForceFt, 0.0},
            {"force_fts_hz", hand::TelemetrySemantic::kForceFts, 0.0},
            {"force_fd_hz", hand::TelemetrySemantic::kForceFd, 0.0},
            {"force_fds_hz", hand::TelemetrySemantic::kForceFds, 0.0},
            {"force_px_hz", hand::TelemetrySemantic::kForcePx, 0.0},
        };
        double positive(double value, const std::string &name, double maximum = 1000.0) {
            if (!std::isfinite(value) || value < 0.001 || value > maximum) {
                throw hand::ValidationError(name + " must be finite and in 0.001.." + std::to_string(maximum));
            }
            return value;
        }
        std::chrono::milliseconds duration(double value, const std::string &name) {
            return std::chrono::milliseconds(static_cast<int64_t>(std::ceil(1000.0 * positive(value, name, 300.0))));
        }
    } // namespace

    hand::Model parse_model(const std::string &input) {
        std::string value = input;
        std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) { return std::tolower(c); });
        if (value == "o20") {
            throw hand::UnsupportedOperationError("model o20: no C++ HandDevice backend is exposed by SDK 0.3");
        }
        if (value == "l6") {
            return hand::Model::kL6;
        }
        if (value == "o6") {
            return hand::Model::kO6;
        }
        if (value == "l20lite" || value == "l20_lite" || value == "l20-lite") {
            return hand::Model::kL20Lite;
        }
        if (value == "l20") {
            return hand::Model::kL20;
        }
        if (value == "l30") {
            return hand::Model::kL30;
        }
        if (value == "o30") {
            return hand::Model::kO30;
        }
        throw hand::ValidationError("unsupported model: " + value);
    }
    std::string model_name(hand::Model value) {
        switch (value) {
        case hand::Model::kL6:
            return "l6";
        case hand::Model::kO6:
            return "o6";
        case hand::Model::kL20Lite:
            return "l20lite";
        case hand::Model::kL20:
            return "l20";
        case hand::Model::kL30:
            return "l30";
        case hand::Model::kO30:
            return "o30";
        default:
            return "unknown";
        }
    }
    std::string side_name(hand::HandSide value) {
        if (value == hand::HandSide::kLeft) {
            return "left";
        }
        if (value == hand::HandSide::kRight) {
            return "right";
        }
        return "not_applicable";
    }
    void declare_parameters(rclcpp_lifecycle::LifecycleNode &node) {
        // Keep public parameters discoverable. The node callback applies supported
        // live updates and rejects static changes until cleanup.
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = false;
        auto declare = [&](const std::string &name, const auto &value) {
            node.declare_parameter(name, value, descriptor);
        };
        declare("transport_json", std::string());
        declare("angle_feedback", std::string("query_only"));
        declare("mailbox_capacity", int64_t(256));
        declare("route_queue_capacity", int64_t(256));
        declare("auto_reconnect", false);
        declare("reconnect_initial_delay_s", 0.1);
        declare("reconnect_max_delay_s", 2.0);
        declare("recovery_max_attempts", int64_t(1));
        declare("recovery_timeout_s", 2.0);
        declare("recovery_retry_delay_s", 0.1);
        declare("force_fn_hz", 0.0);
        declare("discovery_config", std::string());
        declare("model", std::string("l6"));
        declare("side", std::string("left"));
        declare("channel", std::string("can0"));
        declare("address", int64_t(-1));
        declare("host_address", int64_t(-1));
        declare("angle_mapping_path", std::string());
        declare("discover", false);
        declare("channels", std::vector<std::string>{});
        declare("discovery_timeout_s", 10.0);
        declare("command_timeout_s", 0.1);
        declare("identity_timeout_s", 1.0);
        declare("transaction_timeout_s", 0.1);
        declare("shutdown_timeout_s", 2.0);
        declare("command_rate_hz", 250.0);
        declare("discrete_queue_capacity", int64_t(16));
        declare("max_tracked_commands", int64_t(1024));
        declare("publish_rate_hz", 100.0);
        declare("diagnostic_rate_hz", 1.0);
        for (const auto &rate : kRates) {
            declare(rate.name, rate.initial);
        }
        for (const auto &value : node.get_node_parameters_interface()->get_parameter_overrides()) {
            if (!node.has_parameter(value.first)) {
                throw hand::UnsupportedOperationError("unknown hand parameter: " + value.first);
            }
        }
    }
    DriverConfig read_config(const rclcpp_lifecycle::LifecycleNode &node,
                             const std::vector<rclcpp::Parameter> &overrides) {
        auto parameter = [&](const std::string &name) {
            for (const auto &value : overrides) {
                if (value.get_name() == name) {
                    return value;
                }
            }
            return node.get_parameter(name);
        };
        DriverConfig result;
        auto number = [&](const char *name) { return parameter(name).as_double(); };
        auto text = [&](const char *name) { return parameter(name).as_string(); };
        result.discover = parameter("discover").as_bool();
        auto &cfg = result.device;
        cfg.model = parse_model(text("model"));
        const auto side = text("side");
        if (side != "left" && side != "right" && side != "" && side != "not_applicable") {
            throw hand::ValidationError("side must be left, right or not_applicable");
        }
        cfg.side = side == "left"    ? hand::HandSide::kLeft
                   : side == "right" ? hand::HandSide::kRight
                                     : hand::HandSide::kUnknown;
        cfg.interface_name = text("channel");
        if (cfg.interface_name.empty()) {
            throw hand::ValidationError("channel must not be empty");
        }
        auto address = [&](const char *name) -> std::optional<uint32_t> {
            const auto value = parameter(name).as_int();
            if (value < -1 || value > std::numeric_limits<uint32_t>::max()) {
                throw hand::ValidationError(std::string(name) + " is out of range");
            }
            if (value == -1) {
                return {};
            }
            return static_cast<uint32_t>(value);
        };
        cfg.address = address("address");
        cfg.host_address = address("host_address");
        if (!text("angle_mapping_path").empty()) {
            cfg.angle_mapping_path = text("angle_mapping_path");
        }
        cfg.command_timeout = duration(number("command_timeout_s"), "command_timeout_s");
        cfg.identity_timeout = duration(number("identity_timeout_s"), "identity_timeout_s");
        if (number("transaction_timeout_s") == 0) {
            throw hand::UnsupportedOperationError(
                "transaction_timeout_s=0: C++ SDK has no protocol-default response-window selector");
        }
        cfg.transaction_timeout = duration(number("transaction_timeout_s"), "transaction_timeout_s");
        cfg.shutdown_timeout = duration(number("shutdown_timeout_s"), "shutdown_timeout_s");
        cfg.command_rate_hz = positive(number("command_rate_hz"), "command_rate_hz");
        auto capacity = [&](const char *name) {
            const auto value = parameter(name).as_int();
            if (value < 1 || value > 65536) {
                throw hand::ValidationError(std::string(name) + " must be 1..65536");
            }
            return static_cast<size_t>(value);
        };
        cfg.discrete_queue_capacity = capacity("discrete_queue_capacity");
        result.max_tracked_commands = capacity("max_tracked_commands");
        result.publish_rate_hz = number("publish_rate_hz");
        if (result.publish_rate_hz != 0) {
            positive(result.publish_rate_hz, "publish_rate_hz");
        }
        result.diagnostic_rate_hz = number("diagnostic_rate_hz");
        if (result.diagnostic_rate_hz != 0) {
            positive(result.diagnostic_rate_hz, "diagnostic_rate_hz", 100.0);
        }
        const double fn = number("force_fn_hz");
        const double force = number("force_hz");
        if (!std::isfinite(fn) || fn < 0 || fn > 1000 || (fn > 0 && fn < 0.001)) {
            throw hand::ValidationError("force_fn_hz out of range");
        }
        if (force > 0 && (fn > 0 || number("force_fns_hz") > 0)) {
            throw hand::ValidationError("force_hz conflicts with force_fn_hz/force_fns_hz");
        }
        for (const auto &rate : kRates) {
            const auto hz = std::string(rate.name) == "force_hz" ? std::max(force, fn) : number(rate.name);
            if (!std::isfinite(hz) || hz < 0 || hz > 1000 || (hz > 0 && hz < 0.001)) {
                throw hand::ValidationError(std::string(rate.name) + " must be zero or in 0.001..1000");
            }
            if (hz > 0) {
                hand::PollingChannel channel;
                channel.semantic = rate.semantic;
                channel.interval = std::chrono::milliseconds(static_cast<int64_t>(std::ceil(1000.0 / hz)));
                result.polling.push_back(std::move(channel));
            }
        }
        result.transport_json = text("transport_json");
        result.angle_feedback = text("angle_feedback");
        if (result.angle_feedback != "query_only") {
            if (result.angle_feedback != "adaptive" && result.angle_feedback != "command_feedback_only") {
                throw hand::ValidationError("unknown angle_feedback policy");
            }
            throw hand::UnsupportedOperationError("angle_feedback=" + result.angle_feedback +
                                                  ": C++ SDK exposes no feedback-policy control; use query_only");
        }
        for (const auto *name : {"mailbox_capacity", "route_queue_capacity"}) {
            if (parameter(name).as_int() != 256) {
                throw hand::UnsupportedOperationError(std::string(name) +
                                                      ": C++ HandDeviceConfig has no corresponding field");
            }
        }
        const auto attempts = parameter("recovery_max_attempts").as_int();
        if (attempts < 1 || attempts > 100) {
            throw hand::ValidationError("recovery_max_attempts must be 1..100");
        }
        cfg.recovery_attempts = static_cast<uint32_t>(attempts);
        cfg.recovery_backoff = duration(number("recovery_retry_delay_s"), "recovery_retry_delay_s");
        if (number("recovery_timeout_s") != 2.0) {
            throw hand::UnsupportedOperationError("recovery_timeout_s: SDK recover() shares shutdown_timeout; "
                                                  "independent recovery budget is unavailable");
        }
        result.auto_reconnect = parameter("auto_reconnect").as_bool();
        result.reconnect_initial_delay_s =
            positive(number("reconnect_initial_delay_s"), "reconnect_initial_delay_s", 300.0);
        result.reconnect_max_delay_s = positive(number("reconnect_max_delay_s"), "reconnect_max_delay_s", 300.0);
        if (result.reconnect_max_delay_s < result.reconnect_initial_delay_s) {
            throw hand::ValidationError("reconnect delay maximum is less than initial delay");
        }
        // Construction is deliberately zero I/O, including vendor transport factories.
        if (!result.discover && !result.transport_json.empty()) {
            (void)make_device(result);
        }
        auto &runtime = result.runtime;
        runtime.angle_mapping_path = cfg.angle_mapping_path;
        runtime.command_timeout = cfg.command_timeout;
        runtime.identity_timeout = cfg.identity_timeout;
        runtime.transaction_timeout = cfg.transaction_timeout;
        runtime.shutdown_timeout = cfg.shutdown_timeout;
        runtime.command_rate_hz = cfg.command_rate_hz;
        runtime.discrete_queue_capacity = cfg.discrete_queue_capacity;
        runtime.recovery_attempts = cfg.recovery_attempts;
        runtime.recovery_backoff = cfg.recovery_backoff;
        result.discovery.backends = {"socketcan"};
        result.discovery.channels = parameter("channels").as_string_array();
        result.discovery.total_timeout = duration(number("discovery_timeout_s"), "discovery_timeout_s");
        const auto discovery_file = text("discovery_config");
        if (!discovery_file.empty()) {
            std::ifstream stream(discovery_file);
            if (!stream) {
                throw hand::ValidationError("cannot read discovery_config");
            }
            std::string content((std::istreambuf_iterator<char>(stream)), {});
            apply_discovery_configuration(result.discovery, content,
                                          std::filesystem::absolute(discovery_file).parent_path());
        }
        if (!result.discover) {
            validate_polling(result.polling, hand::declared_capabilities(cfg.model, cfg.side));
        }
        return result;
    }
    void apply_discovery_configuration(hand::DiscoveryConfig &config, const std::string &text,
                                       const std::filesystem::path &base) {
        const auto settings = YAML::Load(text);
        if (!settings.IsMap()) {
            throw hand::ValidationError("discovery_config must contain a map");
        }
        auto check = [](const YAML::Node &map, std::initializer_list<std::string> allowed) {
            if (!map.IsMap()) {
                throw hand::ValidationError("discovery section must be a map");
            }
            std::set<std::string> seen;
            for (const auto &pair : map) {
                const auto key = pair.first.as<std::string>();
                if (!seen.insert(key).second) {
                    throw hand::ValidationError("duplicate discovery key: " + key);
                }
                if (std::find(allowed.begin(), allowed.end(), key) == allowed.end()) {
                    throw hand::UnsupportedOperationError("discovery field " + key +
                                                          ": no C++ SDK configuration mapping");
                }
            }
        };
        check(settings, {"backends", "channels", "total_timeout_s", "response_timeout_s", "send_timeout_s",
                         "inter_request_delay_s", "route_queue_capacity", "l30_node_ids", "o30_request_ids", "pcan",
                         "slcan", "minicanfd"});
        auto budget = [&](const YAML::Node &node, const char *key, auto &target) {
            if (node[key]) {
                target = duration(node[key].as<double>(), key);
            }
        };
        if (settings["backends"]) {
            config.backends = settings["backends"].as<std::vector<std::string>>();
        }
        if (settings["channels"]) {
            config.channels = settings["channels"].as<std::vector<std::string>>();
        }
        budget(settings, "total_timeout_s", config.total_timeout);
        budget(settings, "response_timeout_s", config.response_timeout);
        budget(settings, "send_timeout_s", config.send_timeout);
        if (settings["inter_request_delay_s"] && settings["inter_request_delay_s"].as<double>() == 0) {
            config.inter_request_delay = std::chrono::milliseconds(0);
        } else {
            budget(settings, "inter_request_delay_s", config.inter_request_delay);
        }
        if (settings["route_queue_capacity"]) {
            auto value = settings["route_queue_capacity"].as<int64_t>();
            if (value < 1 || value > 65536) {
                throw hand::ValidationError("discovery route_queue_capacity out of range");
            }
            config.route_queue_capacity = value;
        }
        if (settings["l30_node_ids"]) {
            config.l30_node_ids.clear();
            for (const auto &node : settings["l30_node_ids"]) {
                const auto value = node.as<int>();
                if (value < 1 || value > 31) {
                    throw hand::ValidationError("L30 node id must be 1..31");
                }
                config.l30_node_ids.push_back(value);
            }
        }
        if (settings["o30_request_ids"]) {
            config.o30_request_ids = settings["o30_request_ids"].as<std::vector<uint32_t>>();
        }
        auto path = [&](const YAML::Node &node) {
            auto value = std::filesystem::path(node.as<std::string>());
            return value.is_relative() && !base.empty() ? base / value : value;
        };
        auto protocols = [](const YAML::Node &node, auto &target) {
            if (!node) {
                return;
            }
            target.clear();
            for (const auto &value : node) {
                const auto name = value.template as<std::string>();
                if (name != "can" && name != "can_fd") {
                    throw hand::ValidationError("probe_protocols must use can or can_fd");
                }
                target.push_back(name == "can" ? hand::TransportKind::kCan : hand::TransportKind::kCanFd);
            }
        };
        for (const auto *backend : {"pcan", "slcan"}) {
            const auto section = settings[backend];
            if (!section) {
                continue;
            }
            check(section,
                  {"channels", "bitrate", "nominal_bitrate", "data_bitrate", "probe_protocols", "library_path",
                   "fd_timing", "serial_baud", "command_timeout_s", "startup_delay_s", "protocol", "bus_options"});
            const bool pcan = std::string(backend) == "pcan";
            if (section["bus_options"] && section["bus_options"].size()) {
                throw hand::UnsupportedOperationError(
                    std::string(backend) + ".bus_options: generic python-can plugin options have no C++ mapping");
            }
            if (pcan) {
                auto &target = config.pcan;
                if (section["channels"]) {
                    target.channels = section["channels"].as<std::vector<std::string>>();
                }
                if (section["bitrate"]) {
                    target.nominal_bitrate = section["bitrate"].as<uint32_t>();
                }
                if (section["nominal_bitrate"]) {
                    target.nominal_bitrate = section["nominal_bitrate"].as<uint32_t>();
                }
                if (section["data_bitrate"]) {
                    target.data_bitrate = section["data_bitrate"].as<uint32_t>();
                }
                if (section["library_path"]) {
                    target.library_path = path(section["library_path"]);
                }
                protocols(section["probe_protocols"], target.probe_protocols);
                if (section["fd_timing"]) {
                    const auto timing = section["fd_timing"];
                    check(timing, {"f_clock", "nom_brp", "nom_tseg1", "nom_tseg2", "nom_sjw", "data_brp", "data_tseg1",
                                   "data_tseg2", "data_sjw"});
                    std::map<std::string, uint32_t *> fields{{"f_clock", &target.fd_timing.clock_hz},
                                                             {"nom_brp", &target.fd_timing.nominal_prescaler},
                                                             {"nom_tseg1", &target.fd_timing.nominal_tseg1},
                                                             {"nom_tseg2", &target.fd_timing.nominal_tseg2},
                                                             {"nom_sjw", &target.fd_timing.nominal_sjw},
                                                             {"data_brp", &target.fd_timing.data_prescaler},
                                                             {"data_tseg1", &target.fd_timing.data_tseg1},
                                                             {"data_tseg2", &target.fd_timing.data_tseg2},
                                                             {"data_sjw", &target.fd_timing.data_sjw}};
                    for (auto &field : fields) {
                        if (timing[field.first]) {
                            *field.second = timing[field.first].as<uint32_t>();
                        }
                    }
                }
                for (const auto *key : {"serial_baud", "command_timeout_s", "startup_delay_s", "protocol"}) {
                    if (section[key]) {
                        throw hand::UnsupportedOperationError(std::string("pcan.") + key + ": no matching SDK field");
                    }
                }
            } else {
                auto &target = config.slcan;
                if (section["channels"]) {
                    target.channels = section["channels"].as<std::vector<std::string>>();
                }
                if (section["bitrate"]) {
                    target.nominal_bitrate = section["bitrate"].as<uint32_t>();
                }
                if (section["nominal_bitrate"]) {
                    target.nominal_bitrate = section["nominal_bitrate"].as<uint32_t>();
                }
                if (section["data_bitrate"]) {
                    target.data_bitrate = section["data_bitrate"].as<uint32_t>();
                }
                if (section["serial_baud"]) {
                    target.serial_baud = section["serial_baud"].as<uint32_t>();
                }
                budget(section, "command_timeout_s", target.command_timeout);
                if (section["startup_delay_s"] && section["startup_delay_s"].as<double>() == 0) {
                    target.startup_delay = std::chrono::milliseconds(0);
                } else {
                    budget(section, "startup_delay_s", target.startup_delay);
                }
                protocols(section["probe_protocols"], target.probe_protocols);
                if (section["protocol"]) {
                    const auto name = section["protocol"].as<std::string>();
                    if (name != "lawicel_classic" && name != "canable_fd") {
                        throw hand::ValidationError("unknown SLCAN protocol");
                    }
                    target.protocol =
                        name == "canable_fd" ? hand::SLCANProtocol::kCanableFd : hand::SLCANProtocol::kLawicelClassic;
                }
                if (section["fd_timing"] || section["library_path"]) {
                    throw hand::UnsupportedOperationError("SLCAN SDK exposes no fd_timing/library_path option");
                }
            }
        }
        if (const auto section = settings["minicanfd"]) {
            check(section, {"library_path", "linux_channels", "config"});
            if (section["library_path"]) {
                config.minicanfd.library_path = path(section["library_path"]);
            }
            if (section["linux_channels"]) {
                config.minicanfd.channels.clear();
                for (const auto &entry : section["linux_channels"]) {
                    for (const auto &channel : entry.second) {
                        config.minicanfd.channels.push_back({entry.first.as<uint32_t>(), channel.as<uint32_t>()});
                    }
                }
            }
            if (section["config"]) {
                const auto options = section["config"];
                check(options, {"nominal_bitrate", "data_bitrate", "nom_baud", "dat_baud"});
                if (options["nominal_bitrate"]) {
                    config.minicanfd.nominal_bitrate = options["nominal_bitrate"].as<uint32_t>();
                }
                if (options["nom_baud"]) {
                    config.minicanfd.nominal_bitrate = options["nom_baud"].as<uint32_t>();
                }
                if (options["data_bitrate"]) {
                    config.minicanfd.data_bitrate = options["data_bitrate"].as<uint32_t>();
                }
                if (options["dat_baud"]) {
                    config.minicanfd.data_bitrate = options["dat_baud"].as<uint32_t>();
                }
            }
        }
    }

    bool dynamic_parameter(const std::string &name) {
        if (name == "publish_rate_hz" || name == "diagnostic_rate_hz" || name == "angle_feedback" ||
            name == "force_fn_hz") {
            return true;
        }
        for (const auto &rate : kRates) {
            if (name == rate.name) {
                return true;
            }
        }
        return false;
    }

    std::unique_ptr<hand::HandDevice> make_device(const DriverConfig &configuration) {
        auto cfg = configuration.device;
        if (configuration.transport_json.empty()) {
            return std::make_unique<hand::HandDevice>(cfg);
        }
        Json::CharReaderBuilder builder;
        builder["rejectDupKeys"] = true;
        builder["failIfExtra"] = true;
        builder["allowComments"] = false;
        auto reader = std::unique_ptr<Json::CharReader>(builder.newCharReader());
        Json::Value value;
        std::string errors;
        const auto &text = configuration.transport_json;
        if (!reader->parse(text.data(), text.data() + text.size(), &value, &errors) || !value.isObject()) {
            throw hand::ValidationError("transport_json must be a strict JSON object: " + errors);
        }
        auto string = [&](const char *key, std::string initial = {}) {
            if (!value.isMember(key)) {
                return initial;
            }
            if (!value[key].isString() || value[key].asString().empty()) {
                throw hand::ValidationError(std::string(key) + " must be a nonempty string");
            }
            return value[key].asString();
        };
        auto integer = [&](const char *key, uint32_t initial, uint32_t maximum = UINT32_MAX) {
            if (!value.isMember(key)) {
                return initial;
            }
            if (!value[key].isUInt() || value[key].asUInt() > maximum) {
                throw hand::ValidationError(std::string(key) + " is out of range");
            }
            return value[key].asUInt();
        };
        auto allowed = [&](std::initializer_list<std::string> keys) {
            for (const auto &key : value.getMemberNames()) {
                if (std::find(keys.begin(), keys.end(), key) == keys.end()) {
                    throw hand::UnsupportedOperationError("transport_json." + key + ": no C++ SDK mapping");
                }
            }
        };
        const auto caps = hand::declared_capabilities(cfg.model, cfg.side);
        const auto backend = string("backend");
        const auto nominal = integer("nominal_bitrate", caps.transport.nominal_bitrate.value_or(1000000));
        const auto data = integer("data_bitrate", caps.transport.data_bitrate.value_or(5000000));
        if ((caps.transport.nominal_bitrate && nominal != *caps.transport.nominal_bitrate) ||
            (caps.transport.data_bitrate && data != *caps.transport.data_bitrate)) {
            throw hand::ValidationError("transport bitrate disagrees with device capability");
        }
        if (backend == "socketcan") {
            allowed(
                {"backend", "channel", "nominal_bitrate", "data_bitrate", "auto_reconfigure", "receive_own_messages"});
            cfg.interface_name = string("channel");
            if (cfg.interface_name.empty()) {
                throw hand::ValidationError("socketcan.channel is required");
            }
            for (const auto *key : {"auto_reconfigure", "receive_own_messages"}) {
                if (value.isMember(key)) {
                    if (!value[key].isBool()) {
                        throw hand::ValidationError(std::string(key) + " must be boolean");
                    }
                    if (value[key].asBool()) {
                        throw hand::UnsupportedOperationError(std::string(key) + ": no HandDevice transport option");
                    }
                }
            }
            return std::make_unique<hand::HandDevice>(cfg);
        }
        if (backend == "vendor_canfd") {
            allowed({"backend", "device_index", "channel_index", "library_path", "nominal_bitrate", "data_bitrate",
                     "config_flags", "model", "can_type", "frame_type"});
            if (caps.transport.kind != hand::TransportKind::kCanFd) {
                throw hand::UnsupportedOperationError("vendor_canfd requires a CAN FD device");
            }
            linkerbot::can::VendorCanFdTransport::Config transport;
            transport.device_index = integer("device_index", 0);
            transport.channel_index = integer("channel_index", 0);
            transport.nom_baud = nominal;
            transport.dat_baud = data;
            transport.config_flags = integer("config_flags", 7, 255);
            transport.model = integer("model", 0, 255);
            transport.cantype = integer("can_type", 1, 255);
            if (value.isMember("frame_type") && integer("frame_type", 4, 255) != 4) {
                throw hand::UnsupportedOperationError("vendor frame_type override is not applied by SDK");
            }
            if (value.isMember("library_path")) {
                transport.library_path = string("library_path");
            }
            return std::make_unique<hand::HandDevice>(cfg, hand::make_minicanfd_transport_factory(transport));
        }
        if (backend == "python_can") {
            allowed({"backend", "interface_name", "interface_type", "nominal_bitrate"});
            const auto type = string("interface_type");
            const auto channel = string("interface_name");
            if (caps.transport.kind != hand::TransportKind::kCan) {
                throw hand::UnsupportedOperationError("python_can manual interface is classic CAN only");
            }
            if (type == "socketcan") {
                if (channel.empty()) {
                    throw hand::ValidationError("interface_name is required");
                }
                cfg.interface_name = channel;
                return std::make_unique<hand::HandDevice>(cfg);
            }
            if (type == "pcan") {
                linkerbot::can::PcanConfig transport;
                if (channel.rfind("PCAN_USBBUS", 0) != 0) {
                    throw hand::UnsupportedOperationError("PCAN channel name is not mapped: " + channel);
                }
                const auto number_text = channel.substr(11);
                size_t consumed = 0;
                unsigned long number;
                try {
                    number = std::stoul(number_text, &consumed);
                } catch (...) {
                    throw hand::ValidationError("invalid PCAN USB channel");
                }
                if (consumed != number_text.size() || number < 1 || number > 16) {
                    throw hand::ValidationError("PCAN USB channel must be 1..16");
                }
                transport.channel = number <= 8 ? 0x50 + number : 0x500 + number;
                transport.nominal_bitrate = nominal;
                return std::make_unique<hand::HandDevice>(cfg, hand::make_pcan_transport_factory(transport));
            }
            throw hand::UnsupportedOperationError("python_can interface_type=" + type +
                                                  ": no corresponding C++ transport factory");
        }
        throw hand::UnsupportedOperationError("transport backend=" + backend + ": no corresponding C++ factory");
    }

    void validate_polling(const hand::PollingPlan &plan, const hand::DeviceCapabilities &caps) {
        for (const auto &channel : plan) {
            const auto *capability = caps.telemetry_channel(channel.semantic);
            if (!capability || !capability->can_query || capability->read_mode == hand::ReadMode::kCommandCache ||
                capability->read_mode == hand::ReadMode::kCommandResponse) {
                throw hand::PollingPlanError("requested channel does not support active polling");
            }
        }
    }
} // namespace linkerbot_hand_ros_cpp
