#include "linkerbot/hand/api/config.hpp"
#include "linkerbot/hand/capabilities.hpp"
#include "linkerbot/hand/discovery.hpp"
#include "linkerbot/hand/errors.hpp"
#include "linkerbot_hand_ros_cpp/conversions.hpp"
#include "linkerbot_hand_ros_cpp/driver_config.hpp"
#include "linkerbot_hand_ros_cpp/hand_node.hpp"
#include "linkerbot_hand_ros_cpp/posix.h"
#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstddef>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <json/config.h>
#include <json/forwards.h>
#include <json/reader.h>
#include <json/value.h>
#include <json/writer.h>
#include <lifecycle_msgs/msg/state.hpp>
#include <limits>
#include <memory>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <set>
#include <sstream>
#include <string>
#include <unistd.h>
#include <utility>
#include <vector>
#include <yaml-cpp/node/node.h>
#include <yaml-cpp/node/parse.h>
#include <yaml-cpp/yaml.h> // IWYU pragma: keep

namespace driver = linkerbot_hand_ros_cpp;
namespace hand = linkerbot::hand;
using namespace std::chrono_literals;

namespace {
    Json::Value parse(const std::string &text) {
        Json::CharReaderBuilder builder;
        builder["rejectDupKeys"] = true;
        builder["failIfExtra"] = true;
        builder["allowComments"] = false;
        std::istringstream stream(text);
        Json::Value value;
        std::string errors;
        if (!Json::parseFromStream(builder, stream, &value, &errors)) {
            throw hand::ValidationError(errors);
        }
        return value;
    }
    struct YamlWork {
        YAML::Node source;
        Json::Value *target;
        size_t depth;
    };
    Json::Value yaml_scalar(const YAML::Node &node) {
        if (node.IsNull()) {
            return {};
        }
        if (node.Tag() != "!") {
            try {
                return parse(node.Scalar());
            } catch (const hand::ValidationError &) {
                return node.Scalar();
            }
        }
        return node.Scalar();
    }
    void expand_yaml_map(const YamlWork &item, std::vector<YamlWork> &pending) {
        *item.target = Json::Value(Json::objectValue);
        for (const auto &pair : item.source) {
            const auto key = pair.first.as<std::string>();
            if (item.target->isMember(key)) {
                throw hand::ValidationError("duplicate YAML key: " + key);
            }
            pending.push_back({pair.second, &(*item.target)[key], item.depth + 1});
        }
    }
    void expand_yaml_sequence(const YamlWork &item, std::vector<YamlWork> &pending) {
        *item.target = Json::Value(Json::arrayValue);
        if (item.source.size() > std::numeric_limits<Json::ArrayIndex>::max()) {
            throw hand::ValidationError("YAML sequence is too large");
        }
        const auto count = static_cast<Json::ArrayIndex>(item.source.size());
        item.target->resize(count);
        for (Json::ArrayIndex i = 0; i < count; ++i) {
            pending.push_back({item.source[i], &(*item.target)[i], item.depth + 1});
        }
    }
    Json::Value yaml_json(const YAML::Node &node) {
        Json::Value result;
        std::vector<YamlWork> pending{{node, &result, 0}};
        while (!pending.empty()) {
            const auto item = pending.back();
            pending.pop_back();
            if (item.depth > 64) {
                throw hand::ValidationError("YAML nesting exceeds 64 levels or contains a cyclic alias");
            }
            if (item.source.IsMap()) {
                expand_yaml_map(item, pending);
            } else if (item.source.IsSequence()) {
                expand_yaml_sequence(item, pending);
            } else {
                *item.target = yaml_scalar(item.source);
            }
        }
        return result;
    }
    std::vector<rclcpp::Parameter> parameters(const Json::Value &values) {
        if (!values.isObject()) {
            throw hand::ValidationError("device parameters must be an object");
        }
        std::vector<rclcpp::Parameter> result;
        for (const auto &key : values.getMemberNames()) {
            const auto &value = values[key];
            if (value.isBool()) {
                result.emplace_back(key, value.asBool());
            } else if (value.type() == Json::intValue || value.type() == Json::uintValue) {
                result.emplace_back(key, value.asInt64());
            } else if (value.isDouble()) {
                result.emplace_back(key, value.asDouble());
            } else if (value.isString()) {
                result.emplace_back(key, value.asString());
            } else if (value.isArray()) {
                std::vector<std::string> items;
                for (const auto &item : value) {
                    if (!item.isString()) {
                        throw hand::UnsupportedOperationError("container only supports string-array parameter " + key);
                    }
                    items.push_back(item.asString());
                }
                result.emplace_back(key, items);
            } else {
                throw hand::UnsupportedOperationError("container parameter " + key + ": unsupported value type");
            }
        }
        return result;
    }
    std::string lower(std::string value) {
        std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) { return std::tolower(c); });
        return value;
    }
    Json::Value match(const hand::DiscoveryCandidate &candidate) {
        Json::Value result;
        result["backend"] = candidate.channel().backend();
        result["channel"] = candidate.channel().name();
        result["model"] = driver::model_name(candidate.model());
        result["side"] =
            candidate.model() == hand::Model::kL30 ? "not_applicable" : driver::side_name(candidate.side());
        result["wire_protocol"] = candidate.wire_protocol() == hand::TransportKind::kCan ? "can" : "can_fd";
        auto &address = result["address"];
        address["scheme"] = candidate.address().scheme();
        address["request_id"] = candidate.address().request_id();
        address["response_id"] = candidate.address().response_id();
        address["is_extended_id"] = candidate.address().extended_id();
        if (const auto node_id = candidate.address().node_id()) {
            address["node_id"] = node_id.value();
        }
        if (const auto host_id = candidate.address().host_id()) {
            address["host_id"] = host_id.value();
        }
        if (const auto serial_number = candidate.identity().serial_number) {
            result["serial_number"] = serial_number.value();
        }
        if (const auto product_code = candidate.identity().product_code) {
            result["product_code"] = product_code.value();
        }
        return result;
    }
    bool matches(const Json::Value &expected, const Json::Value &actual) {
        if (!expected.isObject()) {
            throw hand::ValidationError("match must be an object");
        }
        std::vector<std::pair<const Json::Value *, const Json::Value *>> pending{{&expected, &actual}};
        while (!pending.empty()) {
            const auto [left, right] = pending.back();
            pending.pop_back();
            for (const auto &key : left->getMemberNames()) {
                if (!right->isMember(key)) {
                    throw hand::UnsupportedOperationError("match." + key + ": no C++ SDK identity field");
                }
                if ((*left)[key].isObject()) {
                    pending.emplace_back(&(*left)[key], &(*right)[key]);
                } else if (key == "model") {
                    if (lower((*left)[key].asString()) != lower((*right)[key].asString())) {
                        return false;
                    }
                } else if ((*left)[key] != (*right)[key]) {
                    return false;
                }
            }
        }
        return true;
    }
    void save(const std::filesystem::path &destination, const Json::Value &value) {
        if (std::filesystem::exists(destination)) {
            throw hand::ValidationError("output exists; refusing overwrite: " + destination.string());
        }
        auto temporary = destination.string() + ".tmp.XXXXXX";
        const int fd = driver::make_temporary_file(temporary.data());
        if (fd < 0) {
            throw hand::ValidationError("cannot create deployment temporary file");
        }
        try {
            const auto text = Json::writeString(Json::StreamWriterBuilder(), value);
            size_t offset = 0;
            while (offset < text.size()) {
                auto count = ::write(fd, text.data() + offset, text.size() - offset);
                if (count < 0 && errno == EINTR) {
                    continue;
                }
                if (count <= 0) {
                    throw hand::InternalError("deployment write failed");
                }
                offset += count;
            }
            if (::fsync(fd) != 0) {
                throw hand::InternalError("deployment fsync failed");
            }
            if (::link(temporary.c_str(), destination.c_str()) != 0) {
                throw hand::ValidationError("deployment output already exists or cannot be installed");
            }
            ::close(fd);
            ::unlink(temporary.c_str());
        } catch (...) {
            ::close(fd);
            ::unlink(temporary.c_str());
            throw;
        }
    }
} // namespace

namespace {
    template <typename Range, typename Predicate, typename Failure>
    void reject_first(const Range &items, const Predicate &invalid, const Failure &failure) {
        const auto found = std::find_if(items.begin(), items.end(), invalid);
        if (found != items.end()) {
            failure(*found);
        }
    }
    struct ContainerSession {
        ~ContainerSession() = default;
        explicit ContainerSession(std::vector<std::shared_ptr<driver::HandNode>> &storage) : nodes(storage) {}
        ContainerSession(const ContainerSession &) = delete;
        ContainerSession &operator=(const ContainerSession &) = delete;
        ContainerSession(ContainerSession &&) = delete;
        ContainerSession &operator=(ContainerSession &&) = delete;

        std::vector<std::shared_ptr<driver::HandNode>> &nodes;
        std::shared_ptr<rclcpp::Node> owner;
        std::string deployment_text;
        std::string mode;
        Json::Value common_parameters;
        Json::Value device_parameters;
        std::vector<std::string> names;
        std::string autostart_override;
        std::string output_path;
        Json::Value deployment;
        std::vector<hand::DiscoveryCandidate> selected;
        std::vector<Json::Value> runtime_parameters;
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> configuration;
        bool discover = false;
        bool autostart = true;
        hand::DiscoveryConfig discovery;

        void initialize() {
            owner = std::make_shared<rclcpp::Node>(
                "linkerbot_hand_container",
                rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
            auto declare = [&](const std::string &key, const auto &initial) {
                if (!owner->has_parameter(key)) {
                    owner->declare_parameter(key, initial);
                }
            };
            declare("device_namespaces", std::vector<std::string>{});
            declare("device_parameters_json", std::string("{}"));
            declare("deployment_json", std::string());
            declare("deployment_base_dir", std::string());
            declare("discover", false);
            declare("autostart", true);
            for (const auto *name : {"discovery_mode", "discovery_output", "deployment_config", "autostart_override"}) {
                declare(name, std::string());
            }
            declare("discovery_config_json", std::string("{}"));
            declare("driver_parameters_json", std::string("{}"));
            declare("discovery_namespace", std::string("/"));
            declare("channels", std::vector<std::string>{});
            declare("discovery_config", std::string());
            declare("output", std::string("hands.yaml"));
        }

        void read_startup() {
            deployment_text = owner->get_parameter("deployment_json").as_string();
            mode = owner->get_parameter("discovery_mode").as_string();
            if (!mode.empty() && mode != "single" && mode != "multi" && mode != "file") {
                throw hand::ValidationError("unknown discovery startup mode");
            }
            if (mode == "file") {
                if (!deployment_text.empty()) {
                    throw hand::ValidationError("deployment_config conflicts with deployment_json");
                }
                const auto path = std::filesystem::absolute(owner->get_parameter("deployment_config").as_string());
                deployment_text =
                    Json::writeString(Json::StreamWriterBuilder(), yaml_json(YAML::LoadFile(path.string())));
                owner->set_parameter(rclcpp::Parameter("deployment_base_dir", path.parent_path().string()));
            }
            discover = owner->get_parameter("discover").as_bool() || !deployment_text.empty() || !mode.empty();
            common_parameters = parse(owner->get_parameter("driver_parameters_json").as_string());
            if (!common_parameters.isObject()) {
                throw hand::ValidationError("driver_parameters_json must be an object");
            }
            device_parameters = parse(owner->get_parameter("device_parameters_json").as_string());
            names = owner->get_parameter("device_namespaces").as_string_array();
            autostart = owner->get_parameter("autostart").as_bool();
            autostart_override = owner->get_parameter("autostart_override").as_string();
            if (!autostart_override.empty() && autostart_override != "true" && autostart_override != "false") {
                throw hand::ValidationError("autostart_override must be true or false");
            }
            output_path = owner->get_parameter("discovery_output").as_string();
            if (output_path.empty()) {
                output_path = owner->get_parameter("output").as_string();
            }
            if (discover && deployment_text.empty() && mode != "single" && std::filesystem::exists(output_path)) {
                throw hand::ValidationError("output exists; refusing overwrite: " + output_path);
            }
        }

        void parse_deployment() {
            deployment = parse(deployment_text);
            if (deployment["version"].asInt() != 1 ||
                deployment["schema"].asString() != "linkerbot_hand_ros.discovery") {
                throw hand::ValidationError("invalid deployment schema");
            }
            if (deployment.isMember("autostart") &&
                (owner->get_node_parameters_interface()->get_parameter_overrides().count("autostart") == 0u)) {
                autostart = deployment["autostart"].asBool();
            }
            reject_first(
                deployment.getMemberNames(),
                [&](const auto &key) {
                    return key != "schema" && key != "version" && key != "autostart" && key != "discovery" &&
                           key != "hands";
                },
                [&](const auto &key) { throw hand::ValidationError("unknown deployment field: " + key); });
            if (!deployment["hands"].isArray() || deployment["hands"].empty()) {
                throw hand::ValidationError("deployment hands must be nonempty");
            }
        }

        void configure_discovery() {
            rclcpp::NodeOptions options;
            options.use_global_arguments(false);
            options.parameter_overrides(
                {rclcpp::Parameter("discover", true),
                 rclcpp::Parameter("channels", owner->get_parameter("channels").as_string_array()),
                 rclcpp::Parameter("discovery_config", owner->get_parameter("discovery_config").as_string())});
            configuration = std::make_shared<rclcpp_lifecycle::LifecycleNode>("cpp_discovery_configuration", options);
            driver::declare_parameters(*configuration);
            discovery = driver::read_config(*configuration).discovery;
            const auto discovery_json = owner->get_parameter("discovery_config_json").as_string();
            if (discovery_json != "{}") {
                if (!deployment_text.empty() || !owner->get_parameter("discovery_config").as_string().empty()) {
                    throw hand::ValidationError("discovery_config_json conflicts with other discovery settings");
                }
                driver::apply_discovery_configuration(discovery, discovery_json);
            }
            auto common_values = parameters(common_parameters);
            for (const auto &value : common_values) {
                const auto &key = value.get_name();
                if (key == "model" || key == "side" || key == "channel" || key == "transport_json" ||
                    key == "address" || key == "host_address" || key == "discover" || key == "channels" ||
                    key == "discovery_config") {
                    throw hand::ValidationError("discovered hand runtime parameters cannot override its endpoint");
                }
                if (!configuration->has_parameter(key)) {
                    throw hand::UnsupportedOperationError("unknown parameter: " + key);
                }
            }
            // Runtime validation uses discovery=true to avoid selecting a model or
            // opening a transport before the identity scan has resolved the endpoint.
            (void)driver::read_config(*configuration, common_values);
        }

        void validate_deployment() {
            if (!owner->get_parameter("discovery_config").as_string().empty()) {
                throw hand::ValidationError("deployment discovery conflicts with discovery_config");
            }
            if (!deployment["discovery"].isNull()) {
                driver::apply_discovery_configuration(
                    discovery, Json::writeString(Json::StreamWriterBuilder(), deployment["discovery"]),
                    owner->get_parameter("deployment_base_dir").as_string());
            }
            const auto channels = owner->get_parameter("channels").as_string_array();
            if (!channels.empty()) {
                if (deployment["discovery"].isMember("channels") && channels != discovery.channels) {
                    throw hand::ValidationError("channels conflicts with deployment discovery.channels");
                }
                discovery.channels = channels;
            }
            std::set<std::string> bindings;
            std::set<std::string> physical;
            for (const auto &binding : deployment["hands"]) {
                validate_binding(binding, bindings, physical);
            }
        }

        void validate_binding(const Json::Value &binding, std::set<std::string> &bindings,
                              std::set<std::string> &physical) {
            const auto ns = driver::normalize_namespace(binding["namespace"].asString(), owner->get_namespace());
            if (!bindings.insert(ns).second) {
                throw hand::ValidationError("duplicate namespace");
            }
            if (!binding.isObject() || binding.size() != 3 || !binding.isMember("namespace") ||
                !binding.isMember("match") || !binding.isMember("parameters")) {
                throw hand::ValidationError("hand requires namespace, match, parameters");
            }
            const auto &endpoint = binding["match"];
            validate_endpoint(endpoint);
            const auto key = endpoint["backend"].asString() + ":" + endpoint["channel"].asString();
            if (!physical.insert(key).second) {
                throw hand::ValidationError("duplicate physical channel");
            }
            validate_binding_parameters(binding);
        }

        static void validate_endpoint(const Json::Value &endpoint) {
            for (const auto *key : {"backend", "channel", "model", "side", "wire_protocol"}) {
                if (!endpoint[key].isString() || endpoint[key].asString().empty()) {
                    throw hand::ValidationError("incomplete discovery identity match");
                }
            }
            if (!endpoint["address"].isObject()) {
                throw hand::ValidationError("match.address must be an object");
            }
            reject_first(
                endpoint["address"].getMemberNames(),
                [&](const auto &key) {
                    return key != "scheme" && key != "request_id" && key != "response_id" && key != "is_extended_id" &&
                           key != "node_id" && key != "host_id";
                },
                [&](const auto &key) {
                    throw hand::UnsupportedOperationError("match.address." + key + ": no C++ SDK field");
                });
            reject_first(
                endpoint.getMemberNames(),
                [&](const auto &key) {
                    return key != "backend" && key != "channel" && key != "model" && key != "side" &&
                           key != "wire_protocol" && key != "address" && key != "serial_number" &&
                           key != "product_code";
                },
                [&](const auto &key) {
                    throw hand::UnsupportedOperationError("match." + key + ": no C++ SDK identity field");
                });
        }

        static void validate_binding_parameters(const Json::Value &binding) {
            auto values = parameters(binding["parameters"]);
            for (const auto &value : values) {
                const auto &key = value.get_name();
                if (key == "model" || key == "side" || key == "channel" || key == "transport_json" ||
                    key == "address" || key == "host_address" || key == "discover" || key == "channels" ||
                    key == "discovery_config") {
                    throw hand::ValidationError("discovery parameters must not override endpoint: " + key);
                }
            }
            values.emplace_back("model", lower(binding["match"]["model"].asString()));
            values.emplace_back("side", binding["match"]["side"].asString());
            rclcpp::NodeOptions preflight_options;
            preflight_options.use_global_arguments(false);
            preflight_options.parameter_overrides(values);
            auto preflight =
                std::make_shared<rclcpp_lifecycle::LifecycleNode>("cpp_deployment_preflight", preflight_options);
            driver::declare_parameters(*preflight);
            reject_first(
                values, [&](const auto &value) { return !preflight->has_parameter(value.get_name()); },
                [&](const auto &value) {
                    throw hand::UnsupportedOperationError("unknown parameter: " + value.get_name());
                });
            (void)driver::read_config(*preflight);
        }

        static const hand::DiscoveryCandidate &find_binding(const Json::Value &expected,
                                                            const hand::DiscoveryReport &report) {
            const hand::DiscoveryCandidate *found = nullptr;
            for (const auto &candidate : report.candidates()) {
                if (candidate.status() == hand::DiscoveryStatus::kVerified && matches(expected, match(candidate))) {
                    if (found != nullptr) {
                        throw hand::AmbiguousHandError("deployment binding matched multiple devices");
                    }
                    found = &candidate;
                }
            }
            if (found == nullptr) {
                throw hand::NoHandFoundError("deployment binding has no matching device");
            }
            return *found;
        }
        void select_candidates(const hand::DiscoveryReport &report) {
            names.clear();
            if (!deployment_text.empty()) {
                for (const auto &binding : deployment["hands"]) {
                    const auto *found = &find_binding(binding["match"], report);
                    selected.push_back(*found);
                    names.push_back(binding["namespace"].asString());
                    auto runtime = common_parameters;
                    for (const auto &key : binding["parameters"].getMemberNames()) {
                        runtime[key] = binding["parameters"][key];
                    }
                    runtime_parameters.push_back(std::move(runtime));
                }
            } else if (mode == "single") {
                const auto &candidate = report.require_one();
                selected.push_back(candidate);
                names.push_back(owner->get_parameter("discovery_namespace").as_string());
                runtime_parameters.push_back(common_parameters);
            } else {
                size_t index = 0;
                for (const auto &candidate : report.candidates()) {
                    if (candidate.status() == hand::DiscoveryStatus::kVerified) {
                        selected.push_back(candidate);
                        names.push_back(driver::model_name(candidate.model()) + "_" + std::to_string(++index));
                        runtime_parameters.push_back(common_parameters);
                    }
                }
            }
            if (selected.empty()) {
                throw hand::NoHandFoundError("no verified hands found");
            }
        }

        void save_deployment(const hand::DiscoveryReport &report) {
            for (auto &name : names) {
                if (!(mode == "single" && name == "/")) {
                    name = driver::normalize_namespace(name, owner->get_namespace());
                }
            }
            std::set<std::string> physical;
            Json::Value output;
            output["schema"] = "linkerbot_hand_ros.discovery";
            output["version"] = 1;
            output["autostart"] = autostart;
            output["hands"] = Json::Value(Json::arrayValue);
            output["discovery"] = parse(driver::discovery_configuration(discovery));
            output["discovery"]["channels"] = Json::Value(Json::arrayValue);
            for (size_t i = 0; i < selected.size(); ++i) {
                const auto &candidate = selected[i];
                const auto key = candidate.channel().backend() + ":" + candidate.channel().name();
                if (!physical.insert(key).second) {
                    throw hand::AmbiguousHandError("more than one device selected on physical channel " + key);
                }
                const auto count =
                    std::count_if(report.candidates().begin(), report.candidates().end(), [&](const auto &observed) {
                        return observed.channel().backend() + ":" + observed.channel().name() == key;
                    });
                if (count != 1) {
                    throw hand::AmbiguousHandError("selected channel contains additional endpoints");
                }
                output["discovery"]["channels"].append(candidate.channel().name());
                Json::Value binding;
                binding["namespace"] = names[i];
                binding["match"] = match(candidate);
                binding["parameters"] = runtime_parameters[i];
                output["hands"].append(binding);
            }
            if (deployment_text.empty() && mode != "single") {
                save(output_path, output);
            }
        }

        void create_nodes() {
            if (names.empty()) {
                throw hand::ValidationError("device_namespaces must be nonempty");
            }
            std::set<std::string> unique;
            for (size_t i = 0; i < names.size(); ++i) {
                const auto ns = mode == "single" && names[i] == "/"
                                    ? std::string("/")
                                    : driver::normalize_namespace(names[i], owner->get_namespace());
                if (!unique.insert(ns).second) {
                    throw hand::ValidationError("duplicate namespace");
                }
                rclcpp::NodeOptions options;
                // Launch supplies container options through /** parameter files. Child
                // settings have already been resolved into device_parameters_json; do
                // not leak owner parameters (autostart, output, ...) into hand nodes.
                options.use_global_arguments(false);
                options.arguments({"--ros-args", "-r", "__node:=linkerbot_hand", "-r", "__ns:=" + ns});
                driver::HandNode::DeviceFactory factory;
                if (discover) {
                    const auto candidate = selected[i];
                    auto values = parameters(runtime_parameters[i]);
                    values.emplace_back("model", driver::model_name(candidate.model()));
                    values.emplace_back(
                        "side", candidate.model() == hand::Model::kL30 ? "" : driver::side_name(candidate.side()));
                    options.parameter_overrides(values);
                    factory = [candidate](const hand::HandDeviceConfig &cfg) {
                        hand::HandRuntimeConfig runtime;
                        runtime.angle_mapping_path = cfg.angle_mapping_path;
                        runtime.command_timeout = cfg.command_timeout;
                        runtime.identity_timeout = cfg.identity_timeout;
                        runtime.transaction_timeout = cfg.transaction_timeout;
                        runtime.shutdown_timeout = cfg.shutdown_timeout;
                        runtime.command_rate_hz = cfg.command_rate_hz;
                        runtime.discrete_queue_capacity = cfg.discrete_queue_capacity;
                        runtime.recovery_attempts = cfg.recovery_attempts;
                        runtime.recovery_backoff = cfg.recovery_backoff;
                        return candidate.create_hand_device(runtime);
                    };
                } else if (device_parameters.isMember(names[i])) {
                    options.parameter_overrides(parameters(device_parameters[names[i]]));
                }
                nodes.push_back(std::make_shared<driver::HandNode>(options, factory));
                reject_first(
                    options.parameter_overrides(),
                    [&](const auto &value) { return !nodes.back()->has_parameter(value.get_name()); },
                    [&](const auto &value) {
                        throw hand::UnsupportedOperationError("unknown parameter: " + value.get_name());
                    });
            }
        }

        void validate_physical_nodes() {
            std::set<std::string> physical;
            for (const auto &node : nodes) {
                driver::DriverConfig endpoint;
                endpoint.device.interface_name = node->get_parameter("channel").as_string();
                endpoint.transport_json = node->get_parameter("transport_json").as_string();
                const auto key = driver::physical_channel(endpoint);
                if (!physical.insert(key).second) {
                    throw hand::ValidationError("duplicate physical channel: " + key);
                }
            }
        }

        void activate_nodes() {
            if (std::any_of(nodes.begin(), nodes.end(), [](const auto &node) {
                    return node->configure().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
                })) {
                throw hand::LifecycleError("group configure failed");
            }
            if (std::any_of(nodes.begin(), nodes.end(), [](const auto &node) {
                    return node->activate().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
                })) {
                throw hand::LifecycleError("group activate failed");
            }
        }

        void spin() {
            rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                              std::max<size_t>(4, nodes.size() * 4));
            executor.add_node(owner);
            for (const auto &node : nodes) {
                executor.add_node(node->get_node_base_interface());
            }
            executor.spin();
        }

        void discover_devices() {
            configure_discovery();
            if (!deployment_text.empty()) {
                parse_deployment();
                validate_deployment();
            }
            if (!autostart_override.empty()) {
                autostart = autostart_override == "true";
            }
            const auto report = hand::discover_hands(discovery);
            for (const auto &failure : report.failures()) {
                RCLCPP_WARN(owner->get_logger(), "%s: %s", failure.code().c_str(), failure.message().c_str());
            }
            select_candidates(report);
            save_deployment(report);
        }

        void run() {
            initialize();
            read_startup();
            if (discover) {
                discover_devices();
            }
            create_nodes();
            if (!discover) {
                validate_physical_nodes();
            }
            if (!autostart_override.empty()) {
                autostart = autostart_override == "true";
            }
            if (autostart) {
                activate_nodes();
            }
            spin();
        }
    };
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    std::vector<std::shared_ptr<driver::HandNode>> nodes;
    int status = 0;
    try {
        ContainerSession session{nodes};
        session.run();
    } catch (const std::exception &error) {
        RCLCPP_ERROR(rclcpp::get_logger("linkerbot_hand_container"), "%s: %s", driver::error_code(error).c_str(),
                     error.what());
        status = 1;
    }
    // A signal may invalidate the context before its shutdown callbacks finish.
    if (!rclcpp::ok()) {
        rclcpp::shutdown();
    }
    for (auto it = nodes.rbegin(); it != nodes.rend(); ++it) {
        try {
            if ((*it)->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                (*it)->deactivate();
            }
        } catch (const std::exception &error) {
            RCLCPP_ERROR((*it)->get_logger(), "rollback: %s", error.what());
            status = 1;
        }
        it->reset();
    }
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return status;
}
