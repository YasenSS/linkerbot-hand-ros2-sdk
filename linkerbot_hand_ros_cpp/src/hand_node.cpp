#include "linkerbot_hand_ros_cpp/hand_node.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "linkerbot/hand/api/commands.hpp"
#include "linkerbot/hand/api/device.hpp"
#include "linkerbot/hand/api/readings.hpp"
#include "linkerbot/hand/api/snapshot.hpp"
#include "linkerbot/hand/capabilities.hpp"
#include "linkerbot/hand/discovery.hpp"
#include "linkerbot/hand/errors.hpp"
#include "linkerbot_hand_msgs/msg/hand_capabilities.hpp"
#include "linkerbot_hand_msgs/msg/hand_command_status.hpp"
#include "linkerbot_hand_msgs/msg/hand_device_info.hpp"
#include "linkerbot_hand_msgs/msg/hand_force_field.hpp"
#include "linkerbot_hand_msgs/msg/hand_joint_command.hpp"
#include "linkerbot_hand_msgs/msg/hand_joint_state.hpp"
#include "linkerbot_hand_msgs/msg/hand_normal_force_resultant.hpp"
#include "linkerbot_hand_msgs/msg/hand_raw_command.hpp"
#include "linkerbot_hand_msgs/msg/hand_tactile_sensor_state.hpp"
#include "linkerbot_hand_msgs/msg/hand_tactile_state.hpp"
#include "linkerbot_hand_msgs/msg/hand_telemetry.hpp"
#include "linkerbot_hand_ros_cpp/commands.hpp"
#include "linkerbot_hand_ros_cpp/conversions.hpp"
#include "linkerbot_hand_ros_cpp/driver_config.hpp"
#include "linkerbot_hand_ros_cpp/public_contract.hpp"
#include "linkerbot_hand_ros_cpp/qos.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include <algorithm>
#include <atomic>
#include <bits/chrono.h>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <exception>
#include <ios>
#include <limits>
#include <linkerbot_hand_msgs/msg/hand_command_status_batch.hpp>
#include <linkerbot_hand_msgs/msg/hand_fault_status_state.hpp>
#include <linkerbot_hand_msgs/msg/hand_stall_protection_state.hpp>
#include <linkerbot_hand_msgs/srv/get_device_info.hpp>
#include <linkerbot_hand_msgs/srv/get_raw_values.hpp>
#include <linkerbot_hand_msgs/srv/get_stall_protection.hpp>
#include <linkerbot_hand_msgs/srv/recover.hpp>
#include <linkerbot_hand_msgs/srv/set_raw_values.hpp>
#include <linkerbot_hand_msgs/srv/submit_command.hpp>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription_base.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <shared_mutex>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

namespace linkerbot_hand_ros_cpp {
    namespace srv = linkerbot_hand_msgs::srv;
    using namespace std::chrono_literals;

    struct HandNode::Impl {
        HandNode &node;
        DeviceFactory factory;
        DriverConfig config;
        std::unique_ptr<hand::HandDevice> device;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback;
        std::atomic<bool> worker_stop{true};
        std::thread cache_worker;
        std::mutex publication_mutex;
        rclcpp::CallbackGroup::SharedPtr state_group;

        // Lifecycle owns an exclusive lease; bounded reads own a shared lease.
        // Topic/timer callbacks try-lock, so they never wait behind recovery/close.
        std::shared_mutex gate;
        bool configured = false;
        bool active = false;
        bool interfaces_created = false;
        rclcpp::CallbackGroup::SharedPtr management_group;
        rclcpp::TimerBase::SharedPtr publish_timer, diagnostic_timer;
        std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions;
        std::vector<rclcpp::ServiceBase::SharedPtr> services;

        template <typename T> using Publisher = typename rclcpp::Publisher<T>::SharedPtr;
        Publisher<msg::HandCapabilities> capabilities_pub;
        Publisher<msg::HandDeviceInfo> info_pub;
        Publisher<msg::HandJointState> angles_pub;
        Publisher<msg::HandFaultStatusState> fault_status_pub;
        Publisher<diagnostic_msgs::msg::DiagnosticArray> support_pub;
        Publisher<msg::HandCommandStatusBatch> status_pub;
        Publisher<diagnostic_msgs::msg::DiagnosticArray> diagnostic_pub;
        std::map<hand::TelemetrySemantic, Publisher<msg::HandTelemetry>> telemetry_pubs;
        std::map<std::string, Publisher<msg::HandForceField>> field_pubs;
        Publisher<msg::HandTelemetry> force_pub;
        Publisher<msg::HandTactileState> tactile_pub;
        Publisher<msg::HandTactileSensorState> sensors_pub;
        Publisher<msg::HandNormalForceResultant> normal_pub;
        Publisher<msg::HandStallProtectionState> stall_pub;
        std::optional<msg::HandCapabilities> published_capabilities;
        uint64_t published_identity_generation = 0;
        std::optional<std::chrono::steady_clock::time_point> published_identity_timestamp;
        using Signature =
            std::tuple<uint64_t, uint64_t, bool, bool, bool, hand::TelemetryValueOrigin, hand::TelemetryDelivery>;
        std::map<std::string, Signature> published;
        struct MockPublication {
            size_t subscribers{};
            uint64_t generation{};
            std::chrono::steady_clock::time_point last;
        };
        std::map<std::string, MockPublication> mocked;

        template <typename T>
        void unavailable_state(const std::string &name, const Publisher<T> &pub, const T &message, bool supported,
                               uint64_t generation) {
            const auto count = pub->get_subscription_count();
            if (supported || !count) {
                mocked.erase(name);
                return;
            }
            const auto now = std::chrono::steady_clock::now();
            const auto previous = mocked.find(name);
            // Best-effort state is volatile. Repeat the SAME unavailable sample at a
            // bounded rate so late subscribers and dropped DDS packets can observe it.
            if (previous != mocked.end() && previous->second.subscribers == count &&
                previous->second.generation == generation && now - previous->second.last < 1s) {
                return;
            }
            pub->publish(message);
            if (previous == mocked.end() || previous->second.generation != generation) {
                report(hand::UnsupportedOperationError("state/" + name + ": no SDK/model projection"));
            }
            mocked[name] = {count, generation, now};
        }

        struct Tracked {
            hand::CommandHandle handle;
            uint64_t client_id;
            uint64_t sequence;
            builtin_interfaces::msg::Time published, received;
            size_t history_size = 0;
            uint64_t sent_frames = 0;
        };
        std::mutex tracker_mutex;
        std::vector<Tracked> tracked;
        std::mutex error_mutex;
        std::string last_error_code, last_error_message;

        rcl_interfaces::msg::SetParametersResult update_parameters(const std::vector<rclcpp::Parameter> &parameters) {
            rcl_interfaces::msg::SetParametersResult result;
            try {
                std::unique_lock<std::shared_mutex> const lease(gate);
                for (const auto &value : parameters) {
                    const auto &name = value.get_name();
                    if (name == "use_sim_time") {
                        continue;
                    }
                    if (name == "mailbox_capacity" || name == "route_queue_capacity" || name == "recovery_timeout_s") {
                        throw hand::UnsupportedOperationError(name +
                                                              ": SDK exposes no independent configuration interface");
                    }
                    if (configured && !dynamic_parameter(name)) {
                        throw hand::LifecycleError(name + ": cleanup before changing static parameters");
                    }
                }
                auto next = read_config(node, parameters);
                // Allocate replacement ROS timers before changing the SDK. If allocation
                // fails, both the previous SDK plan and ROS configuration remain intact.
                auto next_timers = configured ? make_timers(next) : TimerPair{};
                if (active && device) {
                    validate_polling(next.polling, device->get_capabilities());
                    device->stop_polling();
                    try {
                        if (!next.polling.empty()) {
                            device->start_polling(next.polling);
                        }
                    } catch (...) {
                        const auto cause = std::current_exception();
                        try {
                            if (!config.polling.empty()) {
                                device->start_polling(config.polling);
                            }
                        } catch (const std::exception &rollback) {
                            active = false;
                            report(rollback);
                        }
                        std::rethrow_exception(cause);
                    }
                }
                config = std::move(next);
                if (configured) {
                    publish_timer = std::move(next_timers.first);
                    diagnostic_timer = std::move(next_timers.second);
                }
                result.successful = true;
            } catch (const std::exception &error) {
                result.successful = false;
                result.reason = error_code(error) + ": " + error.what();
                report(error);
            }
            return result;
        }
        using TimerPair = std::pair<rclcpp::TimerBase::SharedPtr, rclcpp::TimerBase::SharedPtr>;
        TimerPair make_timers(const DriverConfig &next) {
            auto publish = node.create_wall_timer(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double>(1.0 / std::max(100.0, next.publish_rate_hz))),
                [this] { publish_state(); }, state_group);
            auto diagnostic = node.create_wall_timer(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double>(1.0 / std::max(1.0, next.diagnostic_rate_hz))),
                [this] { publish_diagnostics(); }, state_group);
            return {std::move(publish), std::move(diagnostic)};
        }
        void timers() {
            auto next = make_timers(config);
            publish_timer = std::move(next.first);
            diagnostic_timer = std::move(next.second);
        }
        void start_worker() {
            worker_stop = false;
            cache_worker = std::thread([this] {
                auto next_recovery = std::chrono::steady_clock::now();
                double backoff = 0.1;
                while (!worker_stop) {
                    bool reconnect = false;
                    {
                        std::shared_lock<std::shared_mutex> const lease(gate, std::try_to_lock);
                        if (lease.owns_lock() && active && device) {
                            reconnect = config.auto_reconnect && !device->get_health().connected();
                            if (!reconnect) {
                                try {
                                    const auto previous = device->get_snapshot();
                                    (void)device->wait_for_snapshot(previous, 20ms);
                                } catch (const std::exception &error) {
                                    report(error);
                                }
                                backoff = config.reconnect_initial_delay_s;
                            }
                        }
                    }
                    if (reconnect && std::chrono::steady_clock::now() >= next_recovery) {
                        std::unique_lock<std::shared_mutex> const lease(gate, std::try_to_lock);
                        if (lease.owns_lock() && active && device && !worker_stop) {
                            try {
                                device->recover();
                                published.clear();
                                mocked.clear();
                                published_capabilities.reset();
                                metadata_publish();
                                support_status();
                                clear_error();
                                backoff = config.reconnect_initial_delay_s;
                            } catch (const std::exception &error) {
                                report(error);
                            }
                            next_recovery = std::chrono::steady_clock::now() +
                                            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                                std::chrono::duration<double>(backoff));
                            backoff = std::min(config.reconnect_max_delay_s, backoff * 2);
                        }
                    }
                    publish_state();
                    std::this_thread::sleep_for(2ms);
                }
            });
        }

        explicit Impl(HandNode &owner, DeviceFactory make) : node(owner), factory(std::move(make)) {
            state_group = node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            management_group = node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            capabilities_pub = publisher<msg::HandCapabilities>("~/capabilities", metadata_qos());
            info_pub = publisher<msg::HandDeviceInfo>("~/device_info", metadata_qos());
            status_pub = publisher<msg::HandCommandStatusBatch>("~/command/status", status_qos());
            diagnostic_pub = publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", rclcpp::QoS(10));
            support_pub = publisher<diagnostic_msgs::msg::DiagnosticArray>("~/interface_support", metadata_qos());
            create_interfaces();
        }
        template <typename T> Publisher<T> publisher(const std::string &topic, const rclcpp::QoS &qos) {
            // Plain publishers deliberately allow declared metadata in inactive state.
            // All device state and command status publications are explicitly gated.
            return rclcpp::create_publisher<T>(node, topic, qos);
        }
        void report(const std::exception &error) {
            std::lock_guard<std::mutex> const lock(error_mutex);
            last_error_code = error_code(error);
            last_error_message = error.what();
            RCLCPP_WARN(node.get_logger(), "%s: %s", last_error_code.c_str(), error.what());
        }
        void clear_error() {
            std::lock_guard<std::mutex> const lock(error_mutex);
            last_error_code.clear();
            last_error_message.clear();
        }
        void require_active() const {
            if (!active || !device) {
                throw hand::DeviceNotReadyError("lifecycle node is not active");
            }
        }
        template <typename Response> void failure(Response &response, const std::exception &error) {
            response.error_code = error_code(error);
            response.message = error.what();
            report(error);
        }
        std::chrono::milliseconds timeout(double seconds) const {
            const double shutdown = std::chrono::duration<double>(config.device.shutdown_timeout).count();
            if (!std::isfinite(seconds) || seconds <= 0 || seconds >= shutdown / 2.0) {
                throw hand::ValidationError("timeout_s must be positive and less than half shutdown_timeout_s");
            }
            return std::chrono::milliseconds(static_cast<int64_t>(std::ceil(seconds * 1000.0)));
        }
        template <typename Service, typename Callback> void service(const std::string &name, Callback callback) {
            services.push_back(node.create_service<Service>(
                name,
                [this, callback](const std::shared_ptr<typename Service::Request> request,
                                 std::shared_ptr<typename Service::Response> response) {
                    try {
                        std::shared_lock<std::shared_mutex> const lease(gate, std::try_to_lock);
                        if (!lease.owns_lock()) {
                            throw hand::DeviceNotReadyError("lifecycle transition in progress");
                        }
                        require_active();
                        callback(*request, *response);
                    } catch (const std::exception &error) {
                        failure(*response, error);
                    } catch (...) {
                        response->error_code = "internal";
                        response->message = "non-standard SDK exception";
                    }
                },
                service_qos(), management_group));
        }
        template <typename Submit>
        uint64_t track(Submit submit, uint64_t client_id = 0, uint64_t sequence = 0,
                       builtin_interfaces::msg::Time stamp = builtin_interfaces::msg::Time()) {
            const auto received = node.now();
            std::lock_guard<std::mutex> const lock(tracker_mutex);
            // Reserve tracking capacity BEFORE submitting: never lose an accepted
            // command merely because the ROS result tracker is full.
            if (tracked.size() >= config.max_tracked_commands) {
                throw hand::QueueFullError("ROS command tracker is full");
            }
            auto handle = submit();
            const auto id = handle.command_id();
            tracked.push_back({std::move(handle), client_id, sequence, stamp, received, 0, 0});
            clear_error();
            return id;
        }
        void metadata_publish() {
            if (!device) {
                return;
            }
            const auto caps = device->get_capabilities();
            const auto identity = device->get_identity();
            auto message = capabilities(caps, identity);
            // Compare all projected identity/descriptor fields, including changes
            // observed after activation, without serializing on the polling path.
            if (!published_capabilities || message != *published_capabilities) {
                capabilities_pub->publish(message);
                published_capabilities = std::move(message);
                support_status();
            }
            const auto health = device->get_health();
            if (identity && (health.connection_generation != published_identity_generation ||
                             published_identity_timestamp != identity->timestamp)) {
                info_pub->publish(device_info(*identity, node.now(), health.connection_generation));
                published_identity_generation = health.connection_generation;
                published_identity_timestamp = identity->timestamp;
            }
        }
        void reject(const std::exception &error, const CommandContext &context, const std::string &family) {
            report(error);
            msg::HandCommandStatusBatch batch;
            msg::HandCommandStatus status;
            status.client_id = context.client_id;
            status.user_sequence = context.sequence;
            status.publisher_stamp = context.published;
            status.driver_received_stamp = context.received;
            status.family = family;
            status.stage = msg::HandCommandStatus::STAGE_FAILED;
            status.error_code = error_code(error);
            status.error_message = error.what();
            // Zero explicitly means rejected before SDK admission, not a fake SDK ID.
            batch.statuses.push_back(std::move(status));
            status_pub->publish(batch);
        }
        void support_status() {
            diagnostic_msgs::msg::DiagnosticArray array;
            array.header.stamp = node.now();
            auto entry = [&](const std::string &name, bool supported, const std::string &detail) {
                diagnostic_msgs::msg::DiagnosticStatus status;
                status.name = std::string(node.get_fully_qualified_name()) + "/" + name;
                status.level = supported ? 0 : 2;
                status.message = supported ? "implemented" : "unsupported_operation: " + detail;
                array.status.push_back(std::move(status));
            };
            const auto caps =
                device ? std::optional<hand::DeviceCapabilities>(device->get_capabilities()) : std::nullopt;
            for (auto semantic : kCommandInterfaces) {
                entry("command/" + semantic_name(semantic), caps && caps->supports(semantic),
                      "current SDK/model has no command implementation");
            }
            for (auto semantic : kStateInterfaces) {
                entry("state/" + semantic_name(semantic),
                      caps && state_supported(semantic_name(semantic), *caps, device->get_identity()),
                      "current SDK/model exposes no telemetry projection");
            }
            for (auto semantic : kGetterInterfaces) {
                entry("get_" + semantic_name(semantic), caps && caps->supports(semantic),
                      "current SDK/model exposes no getter");
            }
            for (const auto *name : {"get_device_info", "recover", "get_stall_protection", "clear_faults", "enable",
                                     "disable", "calibrate", "save_parameters"}) {
                const auto wire = caps ? std::optional<msg::HandCapabilities>(capabilities(*caps, {})) : std::nullopt;
                const bool supported =
                    std::string(name) == "recover"
                        ? bool(device)
                        : wire && ((std::string(name) == "get_device_info" && wire->operations.read_device_info) ||
                                   (std::string(name) == "get_stall_protection" &&
                                    wire->operations.read_stall_protection) ||
                                   (std::string(name) == "clear_faults" && wire->operations.clear_faults) ||
                                   (std::string(name) == "enable" && wire->operations.enable) ||
                                   (std::string(name) == "disable" && wire->operations.disable) ||
                                   (std::string(name) == "calibrate" && wire->operations.calibrate) ||
                                   (std::string(name) == "save_parameters" && wire->operations.save_parameters));
                entry(name, supported, "current SDK/model does not support this operation");
            }
            {
                for (const auto *name :
                     {"fault_statuses", "tactile", "tactile_sensors", "normal_force_resultant", "stall_protection"}) {
                    entry("state/" + std::string(name), caps && state_supported(name, *caps, device->get_identity()),
                          "no SDK/model projection or verified protocol definition");
                }
                for (const auto &field : field_pubs) {
                    entry("state/force_" + field.first, caps && state_supported("force_" + field.first, *caps),
                          "no SDK/model force field");
                }
            }
            for (auto semantic : {hand::CommandSemantic::kStallDetectionTimes, hand::CommandSemantic::kStallThresholds,
                                  hand::CommandSemantic::kStallHoldCurrents, hand::CommandSemantic::kLimitCompensations,
                                  hand::CommandSemantic::kPositionKp, hand::CommandSemantic::kPositionKd,
                                  hand::CommandSemantic::kTorqueKd}) {
                entry("set_" + semantic_name(semantic), caps && caps->supports(semantic), "no SDK/model setter");
            }
            for (const auto *name :
                 {"angle_feedback/adaptive", "angle_feedback/command_feedback_only", "parameter/mailbox_capacity",
                  "parameter/route_queue_capacity", "parameter/recovery_timeout_s", "model/o20",
                  "identity/extended_fields", "tactile/descriptor_window_flags", "diagnostics/transport_bus_errors",
                  "diagnostics/transport_pending_rx", "diagnostics/transport_estimated_utilization",
                  "diagnostics/discrete_queue_depth", "diagnostics/channel_command_feedback_samples",
                  "diagnostics/channel_origin", "diagnostics/force_field_rates"}) {
                entry(name, false, "SDK 0.3 has no corresponding public data/control interface");
            }
            support_pub->publish(array);
        }
        void create_interfaces() {
            if (interfaces_created) {
                return;
            }
            subscriptions.push_back(node.create_subscription<msg::HandJointCommand>(
                "~/command/angles", command_qos(), [this](msg::HandJointCommand::ConstSharedPtr command) {
                    try {
                        std::shared_lock<std::shared_mutex> const lease(gate, std::try_to_lock);
                        if (!lease.owns_lock()) {
                            throw hand::DeviceNotReadyError("lifecycle transition in progress");
                        }
                        require_active();
                        track([&] { return submit_angles(*device, *command); }, command->client_id,
                              command->user_sequence, command->publisher_stamp);
                    } catch (const std::exception &error) {
                        reject(error,
                               {command->client_id, command->user_sequence, command->publisher_stamp, node.now()},
                               "angle_target");
                    }
                }));
            for (auto semantic : kCommandInterfaces) {
                if (semantic == hand::CommandSemantic::kAngles) {
                    continue;
                }
                const auto name = semantic_name(semantic);
                subscriptions.push_back(node.create_subscription<msg::HandRawCommand>(
                    "~/command/" + name, command_qos(), [this, semantic](msg::HandRawCommand::ConstSharedPtr command) {
                        try {
                            std::shared_lock<std::shared_mutex> const lease(gate, std::try_to_lock);
                            if (!lease.owns_lock()) {
                                throw hand::DeviceNotReadyError("lifecycle transition in progress");
                            }
                            require_active();
                            if (command->client_id == 0u) {
                                throw hand::ValidationError("client_id must be nonzero");
                            }
                            track([&] { return submit_raw(*device, semantic, command->raw); }, command->client_id,
                                  command->user_sequence, command->publisher_stamp);
                        } catch (const std::exception &error) {
                            reject(error,
                                   {command->client_id, command->user_sequence, command->publisher_stamp, node.now()},
                                   semantic_name(semantic));
                        }
                    }));
                if (semantic == hand::CommandSemantic::kStallDetectionTimes ||
                    semantic == hand::CommandSemantic::kStallThresholds ||
                    semantic == hand::CommandSemantic::kStallHoldCurrents ||
                    semantic == hand::CommandSemantic::kLimitCompensations ||
                    semantic == hand::CommandSemantic::kPositionKp || semantic == hand::CommandSemantic::kPositionKd ||
                    semantic == hand::CommandSemantic::kTorqueKd) {
                    service<srv::SetRawValues>("~/set_" + name, [this, semantic](const auto &request, auto &response) {
                        response.command_id =
                            track([&] { return submit_raw(*device, semantic, request.raw); }, 0, request.user_sequence);
                        response.accepted = true;
                        response.message = "command accepted; delivery is asynchronous";
                    });
                }
            }
            angles_pub = publisher<msg::HandJointState>("~/state/angles", state_qos());
            for (auto semantic : kStateInterfaces) {
                if (semantic != hand::TelemetrySemantic::kForce) {
                    telemetry_pubs.emplace(
                        semantic, publisher<msg::HandTelemetry>("~/state/" + semantic_name(semantic), state_qos()));
                }
            }
            for (auto semantic : kGetterInterfaces) {
                service<srv::GetRawValues>(
                    "~/get_" + semantic_name(semantic), [this, semantic](const auto &request, auto &response) {
                        const auto current = device->get_capabilities();
                        const auto *capability = current.telemetry_channel(semantic);
                        if (!capability) {
                            throw hand::UnsupportedOperationError(semantic_name(semantic) +
                                                                  ": current SDK/model exposes no getter");
                        }
                        const auto sample = read_raw(*device, semantic, timeout(request.timeout_s));
                        response.state = telemetry(sample, *capability, node.now());
                        response.success = true;
                        response.message = "read completed; inspect sample origin and freshness";
                    });
            }
            force_pub = publisher<msg::HandTelemetry>("~/state/force", state_qos());
            tactile_pub = publisher<msg::HandTactileState>("~/state/tactile", state_qos());
            sensors_pub = publisher<msg::HandTactileSensorState>("~/state/tactile_sensors", state_qos());
            normal_pub = publisher<msg::HandNormalForceResultant>("~/state/normal_force_resultant", state_qos());
            fault_status_pub = publisher<msg::HandFaultStatusState>("~/state/fault_statuses", state_qos());
            for (const auto name : hand::kForceFields) {
                field_pubs.emplace(std::string(name),
                                   publisher<msg::HandForceField>("~/state/force_" + std::string(name), state_qos()));
            }
            stall_pub = publisher<msg::HandStallProtectionState>("~/state/stall_protection", state_qos());
            service<srv::GetStallProtection>("~/get_stall_protection", [this](const auto &request, auto &response) {
                const auto caps = device->get_capabilities();
                for (auto semantic :
                     {hand::TelemetrySemantic::kStallDetectionTimes, hand::TelemetrySemantic::kStallThresholds,
                      hand::TelemetrySemantic::kStallHoldCurrents}) {
                    if (!caps.supports(semantic)) {
                        throw hand::UnsupportedOperationError("SDK/model does not expose complete stall protection");
                    }
                }
                const auto deadline = std::chrono::steady_clock::now() + timeout(request.timeout_s);
                for (auto semantic :
                     {hand::TelemetrySemantic::kStallDetectionTimes, hand::TelemetrySemantic::kStallThresholds,
                      hand::TelemetrySemantic::kStallHoldCurrents}) {
                    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
                        deadline - std::chrono::steady_clock::now());
                    if (remaining <= 0ms) {
                        throw hand::TimeoutError("stall read exceeded overall timeout");
                    }
                    (void)read_raw(*device, semantic, remaining);
                }
                response.state = stall_message(device->get_snapshot(), caps);
                response.success = true;
            });
            service<srv::GetDeviceInfo>("~/get_device_info", [this](const auto &request, auto &response) {
                const auto identity = device->get_device_info(timeout(request.timeout_s));
                response.info = device_info(identity, node.now(), device->get_health().connection_generation);
                response.success = true;
            });
            for (auto operation :
                 {hand::DeviceOperation::kClearFaults, hand::DeviceOperation::kEnable, hand::DeviceOperation::kDisable,
                  hand::DeviceOperation::kCalibrate, hand::DeviceOperation::kSaveParameters}) {
                const std::string name = operation == hand::DeviceOperation::kClearFaults ? "clear_faults"
                                         : operation == hand::DeviceOperation::kEnable    ? "enable"
                                         : operation == hand::DeviceOperation::kDisable   ? "disable"
                                         : operation == hand::DeviceOperation::kCalibrate ? "calibrate"
                                                                                          : "save_parameters";
                service<srv::SubmitCommand>("~/" + name, [this, operation](const auto &, auto &response) {
                    response.command_id = track([&] { return submit_operation(*device, operation); });
                    response.accepted = true;
                    response.message = "command accepted; delivery is asynchronous";
                });
            }
            services.push_back(node.create_service<srv::Recover>(
                "~/recover",
                [this](const std::shared_ptr<srv::Recover::Request> &request,
                       const std::shared_ptr<srv::Recover::Response> &response) {
                    try {
                        std::unique_lock<std::shared_mutex> const lease(gate);
                        require_active();
                        const double budget = std::chrono::duration<double>(config.device.shutdown_timeout).count();
                        if (!std::isfinite(request->timeout_s) || request->timeout_s <= 0) {
                            throw hand::ValidationError("timeout_s must be positive and finite");
                        }
                        if (request->timeout_s < budget) {
                            throw hand::UnsupportedOperationError(
                                "SDK recover() has no per-call timeout; timeout_s must be >= shutdown_timeout_s");
                        }
                        device->recover();
                        published.clear();
                        mocked.clear();
                        published_capabilities.reset();
                        metadata_publish();
                        support_status();
                        response->success = true;
                        response->message = "identity-verified recovery completed; targets not replayed";
                        clear_error();
                    } catch (const std::exception &error) {
                        failure(*response, error);
                    }
                },
                service_qos(), management_group));
            interfaces_created = true;
            support_status();
        }
        msg::HandStallProtectionState stall_message(const hand::HandSnapshot &snapshot,
                                                    const hand::DeviceCapabilities &caps) {
            msg::HandStallProtectionState result;
            const auto unavailable =
                metadata(hand::SampleMeta::unavailable(device ? device->get_health().connection_generation
                                                              : snapshot.connection_generation),
                         node.now());
            result.detection_times.meta = unavailable;
            result.thresholds.meta = unavailable;
            result.hold_currents.meta = unavailable;
            result.detection_times.semantic = "stall_detection_times";
            result.thresholds.semantic = "stall_thresholds";
            result.hold_currents.semantic = "stall_hold_currents";
            auto convert = [&](const auto &sample, hand::TelemetrySemantic semantic, msg::HandTelemetry &target) {
                if (sample) {
                    if (const auto *capability = caps.telemetry_channel(semantic)) {
                        target = telemetry(*sample, *capability, node.now());
                    }
                }
            };
            convert(snapshot.stall_detection_times, hand::TelemetrySemantic::kStallDetectionTimes,
                    result.detection_times);
            convert(snapshot.stall_thresholds, hand::TelemetrySemantic::kStallThresholds, result.thresholds);
            convert(snapshot.stall_hold_currents, hand::TelemetrySemantic::kStallHoldCurrents, result.hold_currents);
            result.channel_names = result.detection_times.channel_names;
            return result;
        }
        bool changed(const std::string &key, const hand::SampleMeta &meta) {
            const Signature signature{meta.sequence, meta.connection_generation, meta.valid,
                                      meta.stale,    meta.within_declared_range, meta.origin,
                                      meta.delivery};
            const auto found = published.find(key);
            if (found != published.end() && found->second == signature) {
                return false;
            }
            published[key] = signature;
            return true;
        }
        void publish_status() {
            std::lock_guard<std::mutex> const lock(tracker_mutex);
            msg::HandCommandStatusBatch batch;
            auto flush = [&] {
                if (!batch.statuses.empty()) {
                    status_pub->publish(batch);
                    batch.statuses.clear();
                }
            };
            for (auto it = tracked.begin(); it != tracked.end();) {
                auto message =
                    command_status(it->handle, {it->client_id, it->sequence, it->published, it->received}, node.now());
                // Use the same history snapshot for change detection and terminality,
                // otherwise an ACK arriving between reads could be silently discarded.
                const bool done = message.stage == msg::HandCommandStatus::STAGE_FAILED ||
                                  message.stage == msg::HandCommandStatus::STAGE_SUPERSEDED ||
                                  message.stage == msg::HandCommandStatus::STAGE_ACKNOWLEDGED ||
                                  (message.stage == msg::HandCommandStatus::STAGE_DISPATCHED &&
                                   it->handle.ack_expectation() == hand::AckExpectation::kNone);
                if (message.transitions.size() != it->history_size || message.sent_frames != it->sent_frames) {
                    it->history_size = message.transitions.size();
                    it->sent_frames = message.sent_frames;
                    batch.statuses.push_back(std::move(message));
                    if (batch.statuses.size() == 512) {
                        flush();
                    }
                }
                if (done) {
                    it = tracked.erase(it);
                } else {
                    ++it;
                }
            }
            flush();
        }
        void publish_state() {
            std::shared_lock<std::shared_mutex> const lease(gate, std::try_to_lock);
            if (!lease.owns_lock() || !active || !device) {
                return;
            }
            std::lock_guard<std::mutex> const publication(publication_mutex);
            try {
                publish_status();
                metadata_publish();
                if (config.publish_rate_hz == 0) {
                    return;
                }
                const auto snapshot = device->get_snapshot();
                const auto caps = device->get_capabilities();
                const auto now = node.now();
                // An empty SDK snapshot can precede the first telemetry sample. Health
                // still identifies the active connection generation for mock messages.
                const auto generation = device->get_health().connection_generation;
                const auto unavailable = hand::SampleMeta::unavailable(generation);
                auto unsupported = [&](const std::string &name) {
                    return !state_supported(name, caps, device->get_identity());
                };
                msg::HandFaultStatusState missing_faults;
                missing_faults.meta = metadata(unavailable, now);
                if (snapshot.faults && unsupported("fault_statuses")) {
                    missing_faults = fault_statuses(*snapshot.faults, caps, device->get_identity(), now);
                }
                unavailable_state("fault_statuses", fault_status_pub, missing_faults, !unsupported("fault_statuses"),
                                  generation);
                for (const auto &item : telemetry_pubs) {
                    msg::HandTelemetry message;
                    message.semantic = semantic_name(item.first);
                    message.meta = metadata(unavailable, now);
                    unavailable_state(message.semantic, item.second, message, caps.supports(item.first), generation);
                }
                msg::HandTelemetry missing_force;
                missing_force.semantic = "force";
                missing_force.meta = metadata(unavailable, now);
                unavailable_state("force", force_pub, missing_force, !unsupported("force"), generation);
                msg::HandTactileState missing_tactile;
                missing_tactile.meta = metadata(unavailable, now);
                unavailable_state("tactile", tactile_pub, missing_tactile, !unsupported("tactile"), generation);
                msg::HandTactileSensorState missing_sensors;
                missing_sensors.meta = metadata(unavailable, now);
                unavailable_state("tactile_sensors", sensors_pub, missing_sensors, !unsupported("tactile_sensors"),
                                  generation);
                msg::HandNormalForceResultant missing_normal;
                missing_normal.meta = metadata(unavailable, now);
                unavailable_state("normal_force_resultant", normal_pub, missing_normal,
                                  !unsupported("normal_force_resultant"), generation);
                for (const auto &item : field_pubs) {
                    msg::HandForceField message;
                    message.name = item.first;
                    message.meta = metadata(unavailable, now);
                    const bool observed =
                        snapshot.force && std::any_of(snapshot.force->fields.begin(), snapshot.force->fields.end(),
                                                      [&](const auto &field) { return field.name == item.first; });
                    unavailable_state("force_" + item.first, item.second, message,
                                      observed || !unsupported("force_" + item.first), generation);
                }
                unavailable_state("stall_protection", stall_pub, stall_message(snapshot, caps),
                                  !unsupported("stall_protection"), generation);
                if (snapshot.angles && angles_pub &&
                    (config.angle_feedback != "query_only" ||
                     snapshot.angles->meta.delivery != hand::TelemetryDelivery::kCommandResponse) &&
                    changed("angles", snapshot.angles->meta)) {
                    angles_pub->publish(angles(*snapshot.angles, caps, now));
                }
                auto raw = [&](const auto &sample, hand::TelemetrySemantic semantic) {
                    const auto pub = telemetry_pubs.find(semantic);
                    const auto *cap = caps.telemetry_channel(semantic);
                    if (sample && cap && pub != telemetry_pubs.end() &&
                        changed(semantic_name(semantic), sample->meta)) {
                        pub->second->publish(telemetry(*sample, *cap, now));
                    }
                };
                raw(snapshot.measured_speeds, hand::TelemetrySemantic::kMeasuredSpeeds);
                raw(snapshot.configured_speeds, hand::TelemetrySemantic::kConfiguredSpeeds);
                raw(snapshot.configured_accelerations, hand::TelemetrySemantic::kConfiguredAccelerations);
                raw(snapshot.currents, hand::TelemetrySemantic::kCurrents);
                raw(snapshot.current_limits, hand::TelemetrySemantic::kCurrentLimits);
                raw(snapshot.position_kp, hand::TelemetrySemantic::kPositionKp);
                raw(snapshot.position_kd, hand::TelemetrySemantic::kPositionKd);
                raw(snapshot.torque_kd, hand::TelemetrySemantic::kTorqueKd);
                raw(snapshot.torques, hand::TelemetrySemantic::kTorques);
                raw(snapshot.torque_limits, hand::TelemetrySemantic::kTorqueLimits);
                raw(snapshot.temperatures, hand::TelemetrySemantic::kTemperatures);
                raw(snapshot.voltages, hand::TelemetrySemantic::kVoltages);
                raw(snapshot.motion_times, hand::TelemetrySemantic::kMotionTimes);
                raw(snapshot.limit_compensations, hand::TelemetrySemantic::kLimitCompensations);
                raw(snapshot.stall_detection_times, hand::TelemetrySemantic::kStallDetectionTimes);
                raw(snapshot.stall_thresholds, hand::TelemetrySemantic::kStallThresholds);
                raw(snapshot.stall_hold_currents, hand::TelemetrySemantic::kStallHoldCurrents);
                if (snapshot.faults && state_supported("fault_statuses", caps, device->get_identity()) &&
                    changed("fault_statuses", snapshot.faults->meta)) {
                    fault_status_pub->publish(fault_statuses(*snapshot.faults, caps, device->get_identity(), now));
                }
                if (snapshot.faults && changed("faults", snapshot.faults->meta)) {
                    const auto pub = telemetry_pubs.find(hand::TelemetrySemantic::kFaults);
                    const auto *cap = caps.telemetry_channel(hand::TelemetrySemantic::kFaults);
                    if (pub != telemetry_pubs.end() && (cap != nullptr)) {
                        hand::RawVectorSample sample;
                        sample.raw.assign(snapshot.faults->raw.begin(), snapshot.faults->raw.end());
                        sample.meta = snapshot.faults->meta;
                        pub->second->publish(telemetry(sample, *cap, now));
                    }
                }
                if (snapshot.force) {
                    const auto &sample = *snapshot.force;
                    if (changed("force", sample.meta)) {
                        if (tactile_pub && !sample.tactile.empty()) {
                            if (const auto *cap = caps.telemetry_channel(hand::TelemetrySemantic::kForce)) {
                                tactile_pub->publish(tactile(sample, *cap, now));
                            }
                        }
                        if (sensors_pub && !sample.sensors.empty()) {
                            sensors_pub->publish(sensors(sample, now));
                        }
                        if (force_pub && state_supported("force", caps) && sample.tactile.empty() &&
                            sample.sensors.empty()) {
                            if (const auto *cap = caps.telemetry_channel(hand::TelemetrySemantic::kForce)) {
                                hand::RawVectorSample vector;
                                vector.raw.assign(sample.raw.begin(), sample.raw.end());
                                vector.meta = sample.meta;
                                force_pub->publish(telemetry(vector, *cap, now));
                            }
                        }
                    }
                    for (const auto &field : sample.fields) {
                        const auto found = field_pubs.find(field.name);
                        if (found != field_pubs.end() && changed("force/" + field.name, field.meta)) {
                            found->second->publish(force_field(field, now));
                        }
                    }
                }
                if (normal_pub && snapshot.normal_force_resultant &&
                    changed("normal_force_resultant", snapshot.normal_force_resultant->meta)) {
                    normal_pub->publish(normal_force(*snapshot.normal_force_resultant, now));
                }
                if (stall_pub &&
                    (snapshot.stall_detection_times || snapshot.stall_thresholds || snapshot.stall_hold_currents)) {
                    // Do not short-circuit: record all three independent signatures.
                    bool updated = false;
                    if (snapshot.stall_detection_times) {
                        updated |= changed("stall/time", snapshot.stall_detection_times->meta);
                    }
                    if (snapshot.stall_thresholds) {
                        updated |= changed("stall/threshold", snapshot.stall_thresholds->meta);
                    }
                    if (snapshot.stall_hold_currents) {
                        updated |= changed("stall/current", snapshot.stall_hold_currents->meta);
                    }
                    if (updated) {
                        stall_pub->publish(stall_message(snapshot, caps));
                    }
                }
            } catch (const std::exception &error) {
                report(error);
            }
        }
        void publish_diagnostics() {
            std::shared_lock<std::shared_mutex> const lease(gate, std::try_to_lock);
            if (!lease.owns_lock() || config.diagnostic_rate_hz == 0) {
                return;
            }
            diagnostic_msgs::msg::DiagnosticArray array;
            array.header.stamp = node.now();
            diagnostic_msgs::msg::DiagnosticStatus status;
            status.name = std::string(node.get_fully_qualified_name()) + ": hand_driver";
            const auto health = device ? device->get_health() : hand::HandHealth{};
            status.hardware_id = device ? model_name(device->get_capabilities().model) : "undiscovered";
            auto value = [&](const std::string &key, const auto &data) {
                diagnostic_msgs::msg::KeyValue item;
                item.key = key;
                std::ostringstream stream;
                stream << std::boolalpha << data;
                item.value = stream.str();
                status.values.push_back(std::move(item));
            };
            const char *connections[] = {"closed", "connecting", "connected", "reconnecting", "faulted"};
            const char *lifecycles[] = {"closed", "opening", "open", "closing", "recovering", "faulted"};
            value("model", status.hardware_id);
            value("lifecycle_state", lifecycles[static_cast<size_t>(health.lifecycle_state)]);
            value("connection_state", connections[static_cast<size_t>(health.connection_state)]);
            value("connected", health.connected());
            value("identity_verified", health.identity_verified);
            value("polling_active", health.polling_active);
            value("connection_generation", health.connection_generation);
            value("reconnect_count", health.reconnect_count);
            value("consecutive_errors", health.consecutive_errors);
            value("pending_commands", health.pending_commands());
            value("superseded_commands", health.superseded_commands());
            const auto unknown = std::numeric_limits<double>::quiet_NaN();
            auto instant = [&](const auto &time) {
                return time ? std::chrono::duration<double>(time->time_since_epoch()).count() : unknown;
            };
            value("discrete_queue_depth", unknown);
            value("last_successful_send_at", instant(health.last_successful_send));
            value("last_successful_receive_at", instant(health.last_successful_receive));
            value("last_successful_transaction_at", instant(health.last_successful_transaction));
            value("recent_error_type", health.recent_error ? health.recent_error->error_type : "unknown");
            value("transport_bus_errors", unknown);
            value("transport_pending_rx", unknown);
            value("transport_estimated_utilization", unknown);
            auto seconds = [](const auto &duration) {
                return duration ? std::chrono::duration<double>(*duration).count()
                                : std::numeric_limits<double>::quiet_NaN();
            };
            value("transaction_count", health.transaction_latency.count);
            value("transaction_p50_s", seconds(health.transaction_latency.p50));
            value("transaction_p95_s", seconds(health.transaction_latency.p95));
            value("transaction_p99_s", seconds(health.transaction_latency.p99));
            value("transport_tx_frames", health.session.tx_frames);
            value("transport_rx_frames", health.session.rx_frames);
            value("transport_tx_bytes", health.session.tx_bytes);
            value("transport_rx_bytes", health.session.rx_bytes);
            value("transport_route_misses", health.session.route_misses);
            value("transport_pending_tx", health.session.pending_tx);
            value("transport_dropped_frames", health.session.overflows);
            for (const auto &family : health.command_families) {
                const auto prefix = "command_" + family.family;
                value(prefix + "_pending", family.pending);
                value(prefix + "_accepted", family.accepted);
                value(prefix + "_dispatched", family.dispatched);
                value(prefix + "_acknowledged", family.acknowledged);
                value(prefix + "_failed", family.failed);
                value(prefix + "_superseded", family.superseded);
            }
            status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            status.message = "operational";
            if (!active) {
                status.level = 1;
                status.message = "lifecycle node is inactive";
            } else if (!health.connected() || !health.identity_verified || health.consecutive_errors >= 3) {
                status.level = 2;
                status.message = "device is not operational";
            } else {
                const auto snapshot = device->get_snapshot();
                if (!snapshot.angles || !snapshot.angles->meta.valid || snapshot.angles->meta.stale ||
                    (health.consecutive_errors != 0u)) {
                    status.level = 1;
                    status.message = "waiting for fresh feedback or SDK I/O recovery";
                }
            }
            {
                std::lock_guard<std::mutex> const lock(error_mutex);
                value("recent_error_code", last_error_code.empty() && health.recent_error
                                               ? std::string(hand::error_code_name(health.recent_error->code))
                                               : last_error_code);
                value("recent_error_message", last_error_message.empty() && health.recent_error
                                                  ? health.recent_error->message
                                                  : last_error_message);
                if (!last_error_code.empty() && active) {
                    status.level = std::max<uint8_t>(status.level, 1);
                    status.message = last_error_message;
                }
            }
            for (const auto &channel : health.channels) {
                value("channel_" + channel.channel + "_requested_hz", channel.requested_hz);
                value("channel_" + channel.channel + "_achieved_hz", channel.achieved_hz);
                value("channel_" + channel.channel + "_timeouts", channel.timeouts);
                value("channel_" + channel.channel + "_completed_reads", channel.completed_reads);
                value("channel_" + channel.channel + "_feedback_samples", channel.feedback_samples);
                value("channel_" + channel.channel + "_command_feedback_samples", unknown);
                value("channel_" + channel.channel + "_feedback_policy",
                      channel.channel == "angles" ? config.angle_feedback : "unknown");
                value("channel_" + channel.channel + "_origin", "unknown");
                value("channel_" + channel.channel + "_missed_deadlines", channel.missed_deadlines);
                value("channel_" + channel.channel + "_age_s", seconds(channel.data_age.age));
                value("channel_" + channel.channel + "_valid", channel.data_age.valid);
                value("channel_" + channel.channel + "_stale", channel.data_age.stale);
                value("channel_" + channel.channel + "_sequence", channel.data_age.sequence);
                value("channel_" + channel.channel + "_generation", channel.data_age.generation);
            }
            if (device) {
                const auto caps = device->get_capabilities();
                std::string unavailable_states;
                for (const auto *name : {"fault_statuses", "force", "tactile", "tactile_sensors",
                                         "normal_force_resultant", "stall_protection"}) {
                    if (!state_supported(name, caps, device->get_identity())) {
                        unavailable_states += std::string(name) + " ";
                    }
                }
                for (auto semantic : kStateInterfaces) {
                    if (!state_supported(semantic_name(semantic), caps, device->get_identity())) {
                        unavailable_states += semantic_name(semantic) + " ";
                    }
                }
                for (const auto &field : field_pubs) {
                    if (!state_supported("force_" + field.first, caps)) {
                        unavailable_states += "force_" + field.first + " ";
                    }
                }
                value("unsupported_state_interfaces", unavailable_states);
                value("unsupported_state_error_code", unavailable_states.empty() ? "none" : "unsupported_operation");
                for (const auto &field : caps.force_fields()) {
                    const auto prefix = "force_" + std::string(field.name);
                    const auto parameter_name = prefix + "_hz";
                    double requested =
                        node.has_parameter(parameter_name) ? node.get_parameter(parameter_name).as_double() : unknown;
                    if (field.name == "fn") {
                        requested = std::max(requested, node.get_parameter("force_hz").as_double());
                    }
                    value(prefix + "_requested_hz", requested);
                    double query_rate = field.query_channel ? 0.0 : unknown;
                    if (const auto query_channel = field.query_channel) {
                        for (const auto &query : config.polling) {
                            if (query.semantic == query_channel.value_or(hand::TelemetrySemantic::kForce)) {
                                query_rate = 1000.0 / static_cast<double>(query.interval.count());
                            }
                        }
                    }
                    value(prefix + "_query_hz", query_rate);
                    value(prefix + "_achieved_hz", unknown);
                    value(prefix + "_query_channel",
                          field.query_channel
                              ? semantic_name(field.query_channel.value_or(hand::TelemetrySemantic::kForce))
                              : "unknown");
                    value(prefix + "_supported", field.supported.has_value()
                                                     ? (field.supported.value_or(false) ? "true" : "false")
                                                     : "unknown");
                }
            }
            array.status.push_back(std::move(status));
            diagnostic_pub->publish(array);
        }
        void destroy_interfaces() {
            subscriptions.clear();
            services.clear();
            angles_pub.reset();
            telemetry_pubs.clear();
            field_pubs.clear();
            force_pub.reset();
            tactile_pub.reset();
            sensors_pub.reset();
            normal_pub.reset();
            stall_pub.reset();
            interfaces_created = false;
            published.clear();
            mocked.clear();
            published_capabilities.reset();
            published_identity_generation = 0;
            published_identity_timestamp.reset();
        }
        void close() {
            active = false;
            worker_stop = true;
            if (cache_worker.joinable()) {
                cache_worker.join();
            }
            if (device) {
                device->close();
            }
            // Closing terminalizes accepted setters; publish their last results before
            // retiring handles. A failed close keeps all ownership for a later retry.
            if (node.get_node_base_interface()->get_context()->is_valid()) {
                publish_status();
                metadata_publish();
            } else {
                std::lock_guard<std::mutex> const lock(tracker_mutex);
                tracked.clear();
            }
        }
        void cleanup() {
            close();
            publish_timer.reset();
            diagnostic_timer.reset();
            published.clear();
            mocked.clear();
            published_capabilities.reset();
            published_identity_generation = 0;
            published_identity_timestamp.reset();
            device.reset();
            support_status();
            configured = false;
        }
    };

    HandNode::HandNode(const rclcpp::NodeOptions &options, DeviceFactory factory)
        : LifecycleNode("linkerbot_hand", options) {
        linkerbot_hand_ros_cpp::declare_parameters(*this);
        impl_ = std::make_unique<Impl>(*this, std::move(factory));
        impl_->parameter_callback = add_on_set_parameters_callback(
            [this](const auto &parameters) { return impl_->update_parameters(parameters); });
    }
    HandNode::~HandNode() {
        try {
            std::unique_lock<std::shared_mutex> const lease(impl_->gate);
            impl_->cleanup();
        } catch (const std::exception &error) {
            RCLCPP_ERROR(get_logger(), "driver cleanup: %s", error.what());
        }
    }
    HandNode::CallbackReturn HandNode::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
        std::unique_lock<std::shared_mutex> const lease(impl_->gate);
        try {
            if (impl_->configured || impl_->device) {
                throw hand::LifecycleError("cleanup previous configuration first");
            }
            impl_->config = read_config(*this);
            impl_->tracked.reserve(impl_->config.max_tracked_commands);
            if (!impl_->config.discover) {
                impl_->device = impl_->factory ? impl_->factory(impl_->config.device) : make_device(impl_->config);
                if (!impl_->device) {
                    throw hand::InternalError("device factory returned null");
                }
                impl_->create_interfaces();
                impl_->metadata_publish();
                impl_->support_status();
            }
            impl_->timers();
            impl_->configured = true;
            return CallbackReturn::SUCCESS;
        } catch (const std::exception &error) {
            impl_->report(error);
            try {
                impl_->cleanup();
            } catch (const std::exception &cleanup_error) {
                impl_->report(cleanup_error);
            }
            return CallbackReturn::FAILURE;
        }
    }
    HandNode::CallbackReturn HandNode::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
        std::unique_lock<std::shared_mutex> const lease(impl_->gate);
        try {
            if (!impl_->configured) {
                throw hand::LifecycleError("driver is not configured");
            }
            if (!impl_->device) {
                hand::PlatformDiscoverySource source;
                const auto report = source.scan(impl_->config.discovery);
                // Identity discovery intentionally happens only during activation.
                const auto &candidate = report.require_one();
                impl_->device = candidate.create_hand_device(impl_->config.runtime);
            }
            validate_polling(impl_->config.polling, impl_->device->get_capabilities());
            impl_->device->open();
            impl_->create_interfaces();
            if (!impl_->config.polling.empty()) {
                impl_->device->start_polling(impl_->config.polling);
            }
            impl_->published.clear();
            impl_->mocked.clear();
            impl_->published_identity_generation = 0;
            impl_->metadata_publish();
            impl_->support_status();
            impl_->active = true;
            impl_->start_worker();
            impl_->clear_error();
            return CallbackReturn::SUCCESS;
        } catch (const std::exception &error) {
            impl_->report(error);
            try {
                impl_->close();
            } catch (const std::exception &cleanup_error) {
                impl_->report(cleanup_error);
            }
            return CallbackReturn::FAILURE;
        }
    }
    HandNode::CallbackReturn HandNode::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
        std::unique_lock<std::shared_mutex> const lease(impl_->gate);
        try {
            impl_->close();
            return CallbackReturn::SUCCESS;
        } catch (const std::exception &error) {
            impl_->report(error);
            return CallbackReturn::ERROR;
        }
    }
    HandNode::CallbackReturn HandNode::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) {
        std::unique_lock<std::shared_mutex> const lease(impl_->gate);
        try {
            impl_->cleanup();
            return CallbackReturn::SUCCESS;
        } catch (const std::exception &error) {
            impl_->report(error);
            return CallbackReturn::FAILURE;
        }
    }
    HandNode::CallbackReturn HandNode::on_shutdown(const rclcpp_lifecycle::State &state) { return on_cleanup(state); }
    HandNode::CallbackReturn HandNode::on_error(const rclcpp_lifecycle::State &state) { return on_cleanup(state); }
} // namespace linkerbot_hand_ros_cpp
