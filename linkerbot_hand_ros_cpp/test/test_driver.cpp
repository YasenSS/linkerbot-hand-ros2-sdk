#include "linkerbot/can/message.hpp"
#include "linkerbot/hand/api/commands.hpp"
#include "linkerbot/hand/api/device.hpp"
#include "linkerbot/hand/api/readings.hpp"
#include "linkerbot/hand/capabilities.hpp"
#include "linkerbot/hand/discovery.hpp"
#include "linkerbot/hand/errors.hpp"
#include "linkerbot/hand/identity.hpp"
#include "linkerbot_hand_ros_cpp/commands.hpp"
#include "linkerbot_hand_ros_cpp/conversions.hpp"
#include "linkerbot_hand_ros_cpp/driver_config.hpp"
#include "linkerbot_hand_ros_cpp/hand_node.hpp"
#include "linkerbot_hand_ros_cpp/posix.h"
#include "linkerbot_hand_ros_cpp/qos.hpp"
#include <algorithm>
#include <atomic>
#include <bits/chrono.h>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <future>
#include <gtest/gtest.h>
#include <lifecycle_msgs/msg/state.hpp>
#include <limits>
#include <linkerbot/can/fake_transport.hpp>
#include <linkerbot_hand_msgs/msg/hand_capabilities.hpp>
#include <linkerbot_hand_msgs/msg/hand_command_status.hpp>
#include <linkerbot_hand_msgs/msg/hand_command_status_batch.hpp>
#include <linkerbot_hand_msgs/msg/hand_fault_status_state.hpp>
#include <linkerbot_hand_msgs/msg/hand_force_field.hpp>
#include <linkerbot_hand_msgs/msg/hand_joint_command.hpp>
#include <linkerbot_hand_msgs/msg/hand_joint_state.hpp>
#include <linkerbot_hand_msgs/msg/hand_normal_force_resultant.hpp>
#include <linkerbot_hand_msgs/msg/hand_raw_command.hpp>
#include <linkerbot_hand_msgs/msg/hand_tactile_sensor_state.hpp>
#include <linkerbot_hand_msgs/msg/hand_telemetry.hpp>
#include <linkerbot_hand_msgs/msg/sample_metadata.hpp>
#include <linkerbot_hand_msgs/srv/get_device_info.hpp>
#include <linkerbot_hand_msgs/srv/get_raw_values.hpp>
#include <linkerbot_hand_msgs/srv/recover.hpp>
#include <map>
#include <memory>
#include <optional>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

namespace driver = linkerbot_hand_ros_cpp;
namespace hand = linkerbot::hand;
namespace msg = linkerbot_hand_msgs::msg;
using namespace std::chrono_literals;

// Exercise the actual SDK codec/runtime through an in-memory CAN endpoint.
// No fake runtime mode is exposed in the installed driver.
namespace {
    std::vector<linkerbot::can::Message> reply(const linkerbot::can::Message &request) {
        if (request.data[0] == 0xC0) {
            const std::string identity = "LHL6-1-123-L";
            std::vector<linkerbot::can::Message> result;
            for (size_t part = 0; part < 4; ++part) {
                linkerbot::can::Message response{};
                response.id = request.id;
                response.dlc = 8;
                response.data[0] = 0xC0;
                response.data[1] = part;
                for (size_t i = 0; i < 6 && part * 6 + i < identity.size(); ++i) {
                    response.data[i + 2] = identity[part * 6 + i];
                }
                result.push_back(response);
            }
            return result;
        }
        auto response = request;
        response.dlc = 7;
        if (request.data[0] == 0xC1) {
            response.dlc = 5;
        } else if (request.data[0] == 0xC2 || request.data[0] == 0xC4) {
            response.dlc = 4;
        }
        for (size_t i = 1; i < response.dlc; ++i) {
            response.data[i] = request.dlc > 1 ? request.data[i] : uint8_t(64);
        }
        return {response};
    }

} // namespace

class DriverTest : public testing::Test {
protected:
    static void SetUpTestSuite() {
        driver::set_environment("ROS_DOMAIN_ID", "89", 0);
        int const argc = 0;
        rclcpp::init(argc, nullptr);
    }
    static void TearDownTestSuite() { rclcpp::shutdown(); }
    std::atomic<int> transports{0};
    std::atomic<int> requests{0};
    hand::HandDevice *sdk = nullptr;
    static std::string topic(const std::string &suffix) {
        return "/cpp_driver_test_" + std::to_string(getpid()) + "/linkerbot_hand/" + suffix;
    }
    std::shared_ptr<driver::HandNode> node;
    std::shared_ptr<rclcpp::Node> client;
    rclcpp::executors::SingleThreadedExecutor executor;

    void make(std::vector<rclcpp::Parameter> parameters = {}) {
        parameters.emplace_back("angle_mapping_path", "/tmp/linkerbot-ros-cpp-test-mapping-unused.toml");
        parameters.emplace_back("command_timeout_s", 1.0);
        rclcpp::NodeOptions options;
        options.parameter_overrides(parameters);
        options.arguments({"--ros-args", "-r", "__ns:=/cpp_driver_test_" + std::to_string(getpid())});
        node = std::make_shared<driver::HandNode>(options, [this](const auto &cfg) {
            auto result = std::make_unique<hand::HandDevice>(cfg, [this] {
                ++transports;
                auto transport = std::make_unique<linkerbot::can::FakeTransport>();
                transport->on_request(0x28, [this](const auto &request) {
                    ++requests;
                    return reply(request);
                });
                return transport;
            });
            sdk = result.get();
            return result;
        });
        client = std::make_shared<rclcpp::Node>("cpp_driver_test_client");
        executor.add_node(node->get_node_base_interface());
        executor.add_node(client);
    }
    template <typename Predicate> bool until(Predicate predicate, std::chrono::seconds limit = 5s) {
        const auto deadline = std::chrono::steady_clock::now() + limit;
        while (std::chrono::steady_clock::now() < deadline) {
            executor.spin_some();
            if (predicate()) {
                return true;
            }
            std::this_thread::sleep_for(5ms);
        }
        return predicate();
    }
    void TearDown() override {
        if (node) {
            executor.remove_node(node->get_node_base_interface());
            node.reset();
        }
        if (client) {
            executor.remove_node(client);
            client.reset();
        }
    }
};

TEST_F(DriverTest, ConfigureDoesNotOpenTransportAndMetadataIsLatched) {
    make();
    ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(transports, 0);
    msg::HandCapabilities received;
    auto sub = client->create_subscription<msg::HandCapabilities>(
        topic("capabilities"), driver::metadata_qos(),
        [&](const msg::HandCapabilities::ConstSharedPtr &value) { received = *value; });
    ASSERT_TRUE(until([&] { return !received.joints.empty(); }));
    EXPECT_FALSE(received.identity.verified);
    EXPECT_EQ(received.joints.size(), 6u);
    EXPECT_EQ(node->cleanup().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    EXPECT_EQ(transports, 0);
}
TEST_F(DriverTest, ParameterUpdatesBoundLifecycleLockWait) {
    std::promise<void> entered;
    std::promise<void> proceed;
    auto ready = entered.get_future();
    auto release = proceed.get_future().share();
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("angle_mapping_path", "/tmp/unused-mapping.toml")});
    auto isolated = std::make_shared<driver::HandNode>(options, [&](const auto &cfg) {
        entered.set_value();
        (void)release.wait_for(2s);
        return std::make_unique<hand::HandDevice>(cfg,
                                                  [] { return std::make_unique<linkerbot::can::FakeTransport>(); });
    });
    auto configure = std::async(std::launch::async, [&] { return isolated->configure().id(); });
    const auto started = ready.wait_for(2s);
    auto update = std::async(std::launch::async,
                             [&] { return isolated->set_parameter(rclcpp::Parameter("publish_rate_hz", 20.0)); });
    const auto completed = update.wait_for(200ms);
    proceed.set_value();
    const auto result = update.get();
    EXPECT_EQ(configure.get(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(started, std::future_status::ready);
    EXPECT_EQ(completed, std::future_status::ready);
    EXPECT_FALSE(result.successful);
    EXPECT_NE(result.reason.find("conflicts with device activity"), std::string::npos);
}

TEST_F(DriverTest, RealSdkPublishesAnglesAndCommandDelivery) {
    make();
    node->configure();
    ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    EXPECT_EQ(transports, 1);
    bool feedback = false;
    bool delivered = false;
    auto state = client->create_subscription<msg::HandJointState>(
        topic("state/angles"), driver::state_qos(), [&](const msg::HandJointState::ConstSharedPtr &value) {
            feedback = value->meta.valid && !value->meta.stale && value->raw.size() == 6 &&
                       value->meta.connection_generation != 0;
        });
    auto status = client->create_subscription<msg::HandCommandStatusBatch>(
        topic("command/status"), driver::status_qos(), [&](const msg::HandCommandStatusBatch::ConstSharedPtr &value) {
            for (const auto &item : value->statuses) {
                if (item.client_id == 42 && item.user_sequence == 7) {
                    for (const auto &change : item.transitions) {
                        if (change.stage == msg::HandCommandStatus::STAGE_DISPATCHED) {
                            delivered = true;
                        }
                    }
                }
            }
        });
    auto pub = client->create_publisher<msg::HandJointCommand>(topic("command/angles"), driver::command_qos());
    ASSERT_TRUE(until([&] { return feedback && pub->get_subscription_count() == 1; }));
    msg::HandJointCommand command;
    command.client_id = 42;
    command.user_sequence = 7;
    command.publisher_stamp = client->now();
    command.representation = msg::HandJointCommand::REPRESENTATION_RAW;
    command.values.assign(6, 64);
    pub->publish(command);
    ASSERT_TRUE(until([&] { return delivered; }));
    ASSERT_EQ(node->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    EXPECT_EQ(transports, 2);
    node->deactivate();
    node->cleanup();
}
TEST_F(DriverTest, InactiveServicesRejectWithoutIo) {
    make();
    node->configure();
    auto service = client->create_client<linkerbot_hand_msgs::srv::GetDeviceInfo>(topic("get_device_info"));
    ASSERT_TRUE(until([&] { return service->service_is_ready(); }));
    auto request = std::make_shared<linkerbot_hand_msgs::srv::GetDeviceInfo::Request>();
    request->timeout_s = 0.2;
    auto future = service->async_send_request(request);
    ASSERT_TRUE(until([&] { return future.wait_for(0ms) == std::future_status::ready; }));
    const auto response = future.get();
    EXPECT_FALSE(response->success);
    EXPECT_EQ(response->error_code, "device_not_ready");
    EXPECT_EQ(transports, 0);
}
TEST_F(DriverTest, RejectsInvalidPollingBeforeDeviceIo) {
    make({rclcpp::Parameter("speeds_hz", 100.0)});
    EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    EXPECT_EQ(transports, 0);
}
TEST_F(DriverTest, RejectsNanBeforeIo) {
    make({rclcpp::Parameter("angles_hz", std::numeric_limits<double>::quiet_NaN())});
    EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    EXPECT_EQ(transports, 0);
}
TEST_F(DriverTest, RejectsInvalidAddressBeforeIo) {
    make({rclcpp::Parameter("address", int64_t(-2))});
    EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    EXPECT_EQ(transports, 0);
}
TEST_F(DriverTest, RejectsUnrepresentablePollingPeriodBeforeIo) {
    make({rclcpp::Parameter("angles_hz", 1e-300)});
    EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    EXPECT_EQ(transports, 0);
}
TEST_F(DriverTest, InvalidAnglesNeverReachTransport) {
    make({rclcpp::Parameter("angles_hz", 0.0)});
    node->configure();
    ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    const auto frames = requests.load();
    msg::HandJointCommand command;
    command.client_id = 1;
    command.representation = msg::HandJointCommand::REPRESENTATION_RAW;
    command.values.assign(6, 1.5);
    EXPECT_THROW(driver::submit_angles(*sdk, command), hand::ValidationError);
    command.values.assign(6, std::numeric_limits<double>::infinity());
    EXPECT_THROW(driver::submit_angles(*sdk, command), hand::ValidationError);
    command.values.assign(6, 256);
    EXPECT_THROW(driver::submit_angles(*sdk, command), hand::ValidationError);
    command.values.assign(5, 64);
    EXPECT_THROW(driver::submit_angles(*sdk, command), hand::ValidationError);
    command.values.assign(6, 64);
    command.client_id = 0;
    EXPECT_THROW(driver::submit_angles(*sdk, command), hand::ValidationError);
    command.client_id = 1;
    command.representation = msg::HandJointCommand::REPRESENTATION_PERCENT;
    command.values.assign(6, 100.001);
    EXPECT_THROW(driver::submit_angles(*sdk, command), hand::ValidationError);
    EXPECT_EQ(requests.load(), frames);
}
TEST_F(DriverTest, IdentityMismatchRollsBackActivation) {
    make({rclcpp::Parameter("model", "o6")});
    ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    // The endpoint reports L6, while this node was configured for O6.
    EXPECT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_FALSE(sdk->is_open());
    EXPECT_EQ(node->cleanup().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}
TEST_F(DriverTest, ActiveInfoServiceValidatesTimeoutAndReturnsIdentity) {
    make();
    node->configure();
    node->activate();
    auto service = client->create_client<linkerbot_hand_msgs::srv::GetDeviceInfo>(topic("get_device_info"));
    ASSERT_TRUE(until([&] { return service->service_is_ready(); }));
    auto request = std::make_shared<linkerbot_hand_msgs::srv::GetDeviceInfo::Request>();
    request->timeout_s = 0.2;
    auto future = service->async_send_request(request);
    ASSERT_TRUE(until([&] { return future.wait_for(0ms) == std::future_status::ready; }));
    const auto response = future.get();
    ASSERT_TRUE(response->success) << response->message;
    EXPECT_EQ(response->info.model, "l6");
    EXPECT_TRUE(response->info.has_serial_number);
    request->timeout_s = -1;
    auto rejected = service->async_send_request(request);
    ASSERT_TRUE(until([&] { return rejected.wait_for(0ms) == std::future_status::ready; }));
    EXPECT_EQ(rejected.get()->error_code, "validation");
}
TEST_F(DriverTest, RecoveryHonorsConfiguredBudgetAndChangesGeneration) {
    make();
    node->configure();
    node->activate();
    const auto generation = sdk->get_health().connection_generation;
    auto service = client->create_client<linkerbot_hand_msgs::srv::Recover>(topic("recover"));
    ASSERT_TRUE(until([&] { return service->service_is_ready(); }));
    auto request = std::make_shared<linkerbot_hand_msgs::srv::Recover::Request>();
    request->timeout_s = 0.01;
    auto rejected = service->async_send_request(request);
    ASSERT_TRUE(until([&] { return rejected.wait_for(0ms) == std::future_status::ready; }));
    EXPECT_EQ(rejected.get()->error_code, "unsupported_operation");
    EXPECT_EQ(sdk->get_health().connection_generation, generation);
    request->timeout_s = 2.0;
    auto future = service->async_send_request(request);
    ASSERT_TRUE(until([&] { return future.wait_for(0ms) == std::future_status::ready; }));
    const auto response = future.get();
    ASSERT_TRUE(response->success) << response->message;
    EXPECT_GT(sdk->get_health().connection_generation, generation);
    EXPECT_TRUE(sdk->get_health().polling_active);
}
TEST_F(DriverTest, DiscoveryConfigureDoesNotProbe) {
    make({rclcpp::Parameter("discover", true)});
    EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(transports, 0);
}
TEST_F(DriverTest, EndpointsAndSchedulingParametersAreReadOnly) {
    make();
    node->configure();
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("channel", "can1")).successful);
    EXPECT_EQ(node->get_parameter("channel").as_string(), "can0");
}
TEST(Conversions, EveryModelHasConsistentRosCapabilities) {
    for (auto model : hand::supported_models()) {
        const auto caps = hand::declared_capabilities(model, hand::HandSide::kLeft);
        const auto message = driver::capabilities(caps, {});
        EXPECT_EQ(message.joints.size(), caps.joints.size());
        EXPECT_EQ(message.commands.size(), caps.commands.size());
        EXPECT_FALSE(message.identity.verified);
        for (const auto &command : message.commands) {
            EXPECT_EQ(command.raw_ranges.size(), command.channel_names.size());
            EXPECT_EQ(command.percent_mappings.size(), command.channel_names.size());
        }
    }
}
TEST(Conversions, UnknownPhysicalValuesRemainInvalid) {
    hand::AngleSample value;
    value.raw.assign(6, 5);
    const auto result = driver::angles(value, hand::declared_capabilities(hand::Model::kL6, hand::HandSide::kLeft),
                                       rclcpp::Time(1000000000LL));
    ASSERT_EQ(result.percent_valid.size(), 6u);
    EXPECT_FALSE(result.percent_valid[0]);
    EXPECT_FALSE(result.physical_rad_valid[0]);
    EXPECT_TRUE(std::isnan(result.physical_rad[0]));
    EXPECT_FALSE(result.meta.valid);
}
TEST(Conversions, ConfirmedSaveCapabilityIsPreserved) {
    const auto caps = hand::declared_capabilities(hand::Model::kL6, hand::HandSide::kLeft);
    const auto message = driver::capabilities(caps, {});
    const auto &operations = message.operations.acknowledged_operations;
    EXPECT_NE(std::find(operations.begin(), operations.end(), "save_parameters_confirmed"), operations.end());
}
TEST(Conversions, CommandStagesAreMappedExplicitly) {
    EXPECT_EQ(driver::family_name(hand::CommandFamily::kMotionTime), "motion_time_setting");
    EXPECT_EQ(driver::command_stage(hand::CommandStage::kAccepted), msg::HandCommandStatus::STAGE_ACCEPTED);
    EXPECT_EQ(driver::command_stage(hand::CommandStage::kDispatched), msg::HandCommandStatus::STAGE_DISPATCHED);
    EXPECT_EQ(driver::command_stage(hand::CommandStage::kCancelled), msg::HandCommandStatus::STAGE_FAILED);
}
TEST(Conversions, MetadataUsesDeliveryAndDoesNotTreatCacheAsMeasured) {
    hand::SampleMeta meta;
    meta.valid = true;
    meta.stale = false;
    meta.origin = hand::TelemetryValueOrigin::kCommandCache;
    meta.delivery = hand::TelemetryDelivery::kCommandCache;
    meta.age = 1s;
    meta.collected_at = std::chrono::steady_clock::now() - 1s;
    const auto result = driver::metadata(meta, rclcpp::Time(2000000000LL));
    EXPECT_EQ(result.origin, msg::SampleMetadata::ORIGIN_COMMAND_CACHE);
    EXPECT_DOUBLE_EQ(result.age_s, 1.0);
    EXPECT_GE(result.collected_at.sec, 0);
    EXPECT_LT(result.collected_at.sec, 2);
    meta.delivery = hand::TelemetryDelivery::kCommandResponse;
    EXPECT_EQ(driver::metadata(meta, rclcpp::Time(2000000000LL)).origin,
              msg::SampleMetadata::ORIGIN_DEVICE_CONFIG_RESPONSE);
    meta.origin = hand::TelemetryValueOrigin::kDeviceMeasured;
    EXPECT_EQ(driver::metadata(meta, rclcpp::Time(2000000000LL)).origin,
              msg::SampleMetadata::ORIGIN_DEVICE_COMMAND_FEEDBACK);
    meta.delivery = hand::TelemetryDelivery::kQueryOrReport;
    EXPECT_EQ(driver::metadata(meta, rclcpp::Time(2000000000LL)).origin, msg::SampleMetadata::ORIGIN_NONE);
}
TEST(Conversions, AbsentForceSensorIsNotSyntheticZero) {
    hand::ForceFieldSample input;
    input.name = "fn";
    input.sensor_names = {"thumb"};
    input.raw = {std::nullopt};
    const auto result = driver::force_field(input, rclcpp::Time(0LL));
    ASSERT_EQ(result.sensors.size(), 1u);
    EXPECT_FALSE(result.sensors[0].present);
    EXPECT_TRUE(result.sensors[0].raw.empty());
}

TEST_F(DriverTest, UnsupportedServicesRemainDiscoverableAndNeverTouchBus) {
    make({rclcpp::Parameter("angles_hz", 0.0)});
    node->configure();
    node->activate();
    const auto frames = requests.load();
    auto service = client->create_client<linkerbot_hand_msgs::srv::GetRawValues>(topic("get_torques_limits"));
    ASSERT_TRUE(until([&] { return service->service_is_ready(); }));
    auto request = std::make_shared<linkerbot_hand_msgs::srv::GetRawValues::Request>();
    request->timeout_s = 0.2;
    auto future = service->async_send_request(request);
    ASSERT_TRUE(until([&] { return future.wait_for(0ms) == std::future_status::ready; }));
    const auto response = future.get();
    EXPECT_FALSE(response->success);
    EXPECT_EQ(response->error_code, "unsupported_operation");
    EXPECT_FALSE(response->state.meta.valid);
    EXPECT_EQ(requests.load(), frames);
}
TEST_F(DriverTest, UnsupportedTopicCommandReturnsFailedStatusWithoutSdkId) {
    make({rclcpp::Parameter("angles_hz", 0.0)});
    node->configure();
    node->activate();
    const auto frames = requests.load();
    bool failed = false;
    auto subscription = client->create_subscription<msg::HandCommandStatusBatch>(
        topic("command/status"), driver::status_qos(), [&](const msg::HandCommandStatusBatch::ConstSharedPtr &batch) {
            for (const auto &status : batch->statuses) {
                if (status.client_id == 4321) {
                    failed = status.stage == msg::HandCommandStatus::STAGE_FAILED && status.command_id == 0 &&
                             status.error_code == "unsupported_operation";
                }
            }
        });
    auto pub = client->create_publisher<msg::HandRawCommand>(topic("command/motion_times"), driver::command_qos());
    ASSERT_TRUE(until([&] { return pub->get_subscription_count() == 1; }));
    msg::HandRawCommand command;
    command.client_id = 4321;
    command.user_sequence = 1;
    command.raw.assign(6, 1);
    pub->publish(command);
    ASSERT_TRUE(until([&] { return failed; }));
    EXPECT_EQ(requests.load(), frames);
}
TEST_F(DriverTest, DynamicRatesAndPauseAreAppliedWithoutChangingGeneration) {
    make();
    node->configure();
    node->activate();
    const auto generation = sdk->get_health().connection_generation;
    const auto accepted = node->set_parameters_atomically(
        {rclcpp::Parameter("angles_hz", 50.0), rclcpp::Parameter("publish_rate_hz", 0.0)});
    ASSERT_TRUE(accepted.successful) << accepted.reason;
    EXPECT_EQ(sdk->get_health().connection_generation, generation);
    EXPECT_TRUE(sdk->get_health().polling_active);
    EXPECT_EQ(node->get_parameter("publish_rate_hz").as_double(), 0.0);
    const auto rejected = node->set_parameters_atomically(
        {rclcpp::Parameter("speeds_hz", 100.0), rclcpp::Parameter("publish_rate_hz", 60.0)});
    EXPECT_FALSE(rejected.successful);
    EXPECT_EQ(node->get_parameter("publish_rate_hz").as_double(), 0.0);
    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("publish_rate_hz", 100.0)).successful);
}
TEST_F(DriverTest, ReservedParametersReturnExplicitUnsupportedError) {
    make();
    node->configure();
    for (const auto &parameter :
         {rclcpp::Parameter("angle_feedback", "adaptive"), rclcpp::Parameter("mailbox_capacity", int64_t(256)),
          rclcpp::Parameter("route_queue_capacity", int64_t(256)), rclcpp::Parameter("recovery_timeout_s", 2.0)}) {
        const auto result = node->set_parameter(parameter);
        EXPECT_FALSE(result.successful);
        EXPECT_NE(result.reason.find("unsupported_operation"), std::string::npos) << result.reason;
    }
    EXPECT_EQ(transports, 0);
}
TEST_F(DriverTest, UnknownModelBackendIsMockedBeforeIo) {
    make({rclcpp::Parameter("model", "o20")});
    EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    EXPECT_EQ(transports, 0);
}
TEST_F(DriverTest, StaticParametersCanChangeAfterCleanup) {
    make();
    node->configure();
    node->cleanup();
    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("channel", "can2")).successful);
    EXPECT_EQ(node->get_parameter("channel").as_string(), "can2");
    EXPECT_EQ(transports, 0);
}
TEST(DiscoveryConfiguration, AdapterFieldsMapToRealSdkAndUnknownOptionsReject) {
    hand::DiscoveryConfig config;
    driver::apply_discovery_configuration(config,
                                          "backends: [pcan]\nl30_node_ids: [13]\npcan:\n  channels: [PCAN_USBBUS1]\n  "
                                          "probe_protocols: [can_fd]\n  library_path: libpcan.so\n",
                                          "/tmp");
    EXPECT_EQ(config.l30_node_ids.at(0), 13);
    EXPECT_EQ(config.pcan.channels.at(0), "PCAN_USBBUS1");
    EXPECT_EQ(config.pcan.library_path.value_or(std::filesystem::path{}).string(), "/tmp/libpcan.so");
    EXPECT_THROW(driver::apply_discovery_configuration(config, "pcan: {bus_options: {unmapped: 1}}"),
                 hand::UnsupportedOperationError);
    EXPECT_THROW(driver::apply_discovery_configuration(config, "l30_node_ids: [256]"), hand::ValidationError);
}

TEST(Conversions, FaultBitsUseEachSdkModelDefinitionAndPreserveReservedBits) {
    for (auto model : hand::supported_models()) {
        const auto caps = hand::declared_capabilities(model, hand::HandSide::kLeft);
        const auto *channel = caps.telemetry_channel(hand::TelemetrySemantic::kFaults);
        ASSERT_NE(channel, nullptr);
        hand::DeviceIdentity identity;
        identity.model = model;
        identity.protocol_version = "0.0.4";
        uint8_t mask = 0x2f;
        if (model == hand::Model::kL6) {
            mask = 0x7f;
        } else if (model == hand::Model::kL20) {
            mask = 0xff;
        } else if (model == hand::Model::kO30) {
            mask = 0xcf;
        }
        for (unsigned raw = 0; raw <= 255; ++raw) {
            hand::FaultSample sample;
            sample.raw.assign(channel->channels.size(), raw);
            sample.meta.valid = true;
            sample.meta.stale = false;
            sample.meta.connection_generation = 8;
            sample.meta.sequence = raw + 1;
            const auto result = driver::fault_statuses(sample, caps, identity, rclcpp::Time(0LL));
            ASSERT_EQ(result.statuses.size(), channel->channels.size());
            EXPECT_TRUE(result.meta.valid);
            EXPECT_EQ(result.meta.sequence, raw + 1);
            EXPECT_EQ(result.meta.connection_generation, 8u);
            const auto &status = result.statuses[0];
            EXPECT_EQ(status.raw, raw);
            EXPECT_EQ(status.reserved_bits, (raw & ~mask));
            if (model == hand::Model::kL6) {
                EXPECT_EQ(status.phase_b_over_current, bool(raw & 1));
                EXPECT_EQ(status.phase_a_over_current, bool(raw & 4));
                EXPECT_EQ(status.mcu_over_temperature, bool(raw & 0x40));
                EXPECT_EQ(status.supported_flags.size(), 7u);
            } else if (model == hand::Model::kL20) {
                EXPECT_EQ(status.motor_rotor_lock, bool(raw & 1));
                EXPECT_EQ(status.motor_stall, bool(raw & 4));
                EXPECT_EQ(status.motor_communication_abnormal, bool(raw & 0x80));
            } else if (model == hand::Model::kO30) {
                EXPECT_EQ(status.hand_layer_stall, bool(raw & 1));
                EXPECT_EQ(status.actuator_offline, bool(raw & 0x80));
            } else {
                EXPECT_EQ(status.voltage_abnormal, bool(raw & 1));
                EXPECT_EQ(status.magnetic_encoder_abnormal, bool(raw & 2));
                EXPECT_EQ(status.load_overload, bool(raw & 0x20));
            }
        }
        const auto wire = driver::capabilities(caps, identity);
        for (const auto &telemetry : wire.telemetry) {
            if (telemetry.semantic == "faults") {
                EXPECT_TRUE(telemetry.decoded_statuses);
            }
        }
    }
}

TEST(Conversions, LegacyAndUnknownO30FaultsAreUnavailableWithoutLosingRaw) {
    const auto caps = hand::declared_capabilities(hand::Model::kO30, hand::HandSide::kLeft);
    hand::FaultSample sample;
    sample.raw.assign(caps.telemetry_channel(hand::TelemetrySemantic::kFaults)->channels.size(), 0xff);
    sample.meta.valid = true;
    for (const auto &version : {"0.0.3", "0.0.5", "", "unknown"}) {
        hand::DeviceIdentity identity;
        identity.protocol_version = version;
        const auto result = driver::fault_statuses(sample, caps, identity, rclcpp::Time(0LL));
        EXPECT_FALSE(result.meta.valid);
        ASSERT_EQ(result.statuses.size(), sample.raw.size());
        EXPECT_EQ(result.statuses[0].raw, 0xff);
        EXPECT_TRUE(result.statuses[0].supported_flags.empty());
        EXPECT_EQ(result.statuses[0].reserved_bits, 0xff);
        EXPECT_FALSE(driver::state_supported("fault_statuses", caps, identity));
    }
}

TEST_F(DriverTest, UnsupportedStateProjectionsPublishInvalidSamplesWithoutIo) {
    make({rclcpp::Parameter("angles_hz", 0.0), rclcpp::Parameter("diagnostic_rate_hz", 10.0)});
    node->configure();
    node->activate();
    const auto frames = requests.load();
    bool force = false;
    bool sensors = false;
    bool normal = false;
    bool field = false;
    uint64_t force_generation = 0;
    auto f = client->create_subscription<msg::HandTelemetry>(
        topic("state/force"), driver::state_qos(), [&](const msg::HandTelemetry::ConstSharedPtr &value) {
            force = !value->meta.valid && value->meta.stale && value->raw.empty();
            force_generation = value->meta.connection_generation;
        });
    auto s = client->create_subscription<msg::HandTactileSensorState>(
        topic("state/tactile_sensors"), driver::state_qos(),
        [&](const msg::HandTactileSensorState::ConstSharedPtr &value) {
            sensors = !value->meta.valid && value->sensors.empty();
        });
    auto n = client->create_subscription<msg::HandNormalForceResultant>(
        topic("state/normal_force_resultant"), driver::state_qos(),
        [&](const msg::HandNormalForceResultant::ConstSharedPtr &value) {
            normal = !value->meta.valid && value->normal_force_resultant.empty();
        });
    auto p = client->create_subscription<msg::HandForceField>(
        topic("state/force_px"), driver::state_qos(),
        [&](const msg::HandForceField::ConstSharedPtr &value) { field = !value->meta.valid && value->name == "px"; });
    ASSERT_TRUE(until([&] { return force && sensors && normal && field; }));
    // A later subscriber must also see the unavailable volatile state.
    bool late = false;
    auto l = client->create_subscription<msg::HandTelemetry>(
        topic("state/force"), driver::state_qos(),
        [&](const msg::HandTelemetry::ConstSharedPtr &value) { late = !value->meta.valid; });
    ASSERT_TRUE(until([&] { return late; }));
    EXPECT_EQ(requests.load(), frames);
    EXPECT_EQ(force_generation, sdk->get_health().connection_generation);
    const auto previous_generation = force_generation;
    auto recover = client->create_client<linkerbot_hand_msgs::srv::Recover>(topic("recover"));
    ASSERT_TRUE(until([&] { return recover->service_is_ready(); }));
    auto request = std::make_shared<linkerbot_hand_msgs::srv::Recover::Request>();
    request->timeout_s = 2.0;
    auto result = recover->async_send_request(request);
    ASSERT_TRUE(until([&] { return result.wait_for(0ms) == std::future_status::ready; }));
    ASSERT_TRUE(result.get()->success);
    const auto recovered_frames = requests.load();
    ASSERT_TRUE(until([&] { return force_generation > previous_generation; }));
    EXPECT_EQ(force_generation, sdk->get_health().connection_generation);
    EXPECT_EQ(requests.load(), recovered_frames);
}

TEST_F(DriverTest, FaultPublicationUsesRealSdkSampleAndDiagnosticsExposeUnknownMetrics) {
    make({rclcpp::Parameter("faults_hz", 20.0), rclcpp::Parameter("diagnostic_rate_hz", 10.0)});
    node->configure();
    node->activate();
    bool faults = false;
    std::map<std::string, std::string> values;
    auto f = client->create_subscription<msg::HandFaultStatusState>(
        topic("state/fault_statuses"), driver::state_qos(),
        [&](const msg::HandFaultStatusState::ConstSharedPtr &value) {
            faults = value->meta.valid && value->statuses.size() == 6 && value->statuses[0].mcu_over_temperature;
        });
    auto d = client->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", rclcpp::QoS(10), [&](const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr &array) {
            for (const auto &status : array->status) {
                if (status.name.find("cpp_driver_test_") != std::string::npos) {
                    for (const auto &value : status.values) {
                        values[value.key] = value.value;
                    }
                }
            }
        });
    ASSERT_TRUE(until([&] { return faults && values.count("force_fn_achieved_hz"); }));
    EXPECT_TRUE(std::isnan(std::stod(values.at("transport_bus_errors"))));
    EXPECT_TRUE(std::isnan(std::stod(values.at("channel_angles_command_feedback_samples"))));
    EXPECT_EQ(values.at("channel_angles_feedback_policy"), "query_only");
    EXPECT_GT(std::stod(values.at("last_successful_receive_at")), 0);
    EXPECT_EQ(values.at("unsupported_state_error_code"), "unsupported_operation");
}

TEST(ContainerConfiguration, NamespacesAndPhysicalAliasesCannotHideDuplicates) {
    EXPECT_EQ(driver::normalize_namespace("//left///hand/"), "/left/hand");
    EXPECT_EQ(driver::normalize_namespace("hand", "/robot"), "/robot/hand");
    EXPECT_THROW(driver::normalize_namespace("/"), hand::ValidationError);
    EXPECT_THROW(driver::normalize_namespace("/bad-name"), hand::ValidationError);
    driver::DriverConfig config;
    config.device.interface_name = "can3";
    const auto key = driver::physical_channel(config);
    config.transport_json = R"({"backend":"python_can","interface_type":"socketcan","interface_name":"can3"})";
    EXPECT_EQ(driver::physical_channel(config), key);
}

TEST(DiscoveryConfiguration, GeneratedSettingsRoundTripAdaptersBudgetsAndZeroDelays) {
    hand::DiscoveryConfig input;
    driver::apply_discovery_configuration(
        input,
        "backends: [pcan, slcan, minicanfd]\nchannels: [pcan:PCAN_USBBUS2]\ninter_request_delay_s: 0\n"
        "pcan: {channels: [PCAN_USBBUS2], probe_protocols: [can_fd], library_path: pcan.so}\n"
        "slcan: {channels: [/dev/ttyUSB0], protocol: canable_fd, startup_delay_s: 0}\n"
        "minicanfd: {linux_channels: {2: [1]}, library_path: mini.so, config: {nominal_bitrate: 1000000}}",
        "/tmp");
    const auto serialized = driver::discovery_configuration(input);
    hand::DiscoveryConfig output;
    driver::apply_discovery_configuration(output, serialized, "/different-directory");
    EXPECT_EQ(output.backends, input.backends);
    EXPECT_EQ(output.channels, input.channels);
    EXPECT_EQ(output.pcan.library_path, input.pcan.library_path);
    EXPECT_EQ(output.pcan.probe_protocols, input.pcan.probe_protocols);
    EXPECT_EQ(output.pcan.fd_timing.clock_hz, input.pcan.fd_timing.clock_hz);
    EXPECT_EQ(output.slcan.channels, input.slcan.channels);
    EXPECT_EQ(output.slcan.startup_delay, 0ms);
    EXPECT_EQ(output.inter_request_delay, 0ms);
    EXPECT_EQ(output.minicanfd.library_path, input.minicanfd.library_path);
    ASSERT_EQ(output.minicanfd.channels.size(), 1u);
    EXPECT_EQ(output.minicanfd.channels[0].device_index, 2u);
    EXPECT_EQ(output.minicanfd.channels[0].channel_index, 1u);
    EXPECT_EQ(driver::discovery_configuration(output), serialized);
}
