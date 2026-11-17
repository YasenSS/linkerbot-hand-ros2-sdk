#include "linkerbot_hand_ros_cpp/commands.hpp"
#include "linkerbot/hand/api/commands.hpp"
#include "linkerbot/hand/api/device.hpp"
#include "linkerbot/hand/api/readings.hpp"
#include "linkerbot/hand/capabilities.hpp"
#include "linkerbot/hand/errors.hpp"
#include "linkerbot_hand_msgs/msg/hand_command_status.hpp"
#include "linkerbot_hand_msgs/msg/hand_command_transition.hpp"
#include "linkerbot_hand_msgs/msg/hand_joint_command.hpp"
#include "linkerbot_hand_ros_cpp/conversions.hpp"
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <rclcpp/time.hpp>
#include <stdexcept>
#include <vector>

namespace linkerbot_hand_ros_cpp {
    hand::CommandHandle submit_angles(hand::HandDevice &device, const msg::HandJointCommand &command) {
        if (command.client_id == 0u) {
            throw hand::ValidationError("client_id must be nonzero");
        }
        if (command.representation == msg::HandJointCommand::REPRESENTATION_PERCENT) {
            std::vector<float> values;
            for (double value : command.values) {
                if (!std::isfinite(value) || value < 0 || value > 100) {
                    throw hand::ValidationError("percent must be finite and in 0..100");
                }
                values.push_back(static_cast<float>(value));
            }
            return device.set_percent_angles(values);
        }
        if (command.representation != msg::HandJointCommand::REPRESENTATION_RAW) {
            throw hand::ValidationError("unsupported angle representation");
        }
        std::vector<int32_t> values;
        for (double value : command.values) {
            if (!std::isfinite(value) || value != std::trunc(value) || value < std::numeric_limits<int32_t>::min() ||
                value > std::numeric_limits<int32_t>::max()) {
                throw hand::ValidationError("raw angle must be an exact int32 value");
            }
            values.push_back(static_cast<int32_t>(value));
        }
        return device.set_angles(values);
    }
    hand::CommandHandle submit_raw(hand::HandDevice &device, hand::CommandSemantic semantic,
                                   const std::vector<double> &values) {
        switch (semantic) {
        case hand::CommandSemantic::kSpeeds:
            return device.set_speeds(values);
        case hand::CommandSemantic::kAccelerations:
            return device.set_accelerations(values);
        case hand::CommandSemantic::kCurrentLimits:
            return device.set_current_limits(values);
        case hand::CommandSemantic::kPositionKp:
            return device.set_position_kp(values);
        case hand::CommandSemantic::kPositionKd:
            return device.set_position_kd(values);
        case hand::CommandSemantic::kTorqueKd:
            return device.set_torque_kd(values);
        case hand::CommandSemantic::kTorqueLimits:
            return device.set_torques_limits(values);
        case hand::CommandSemantic::kTorques:
            return device.set_torques(values);
        case hand::CommandSemantic::kMotionTimes:
            return device.set_motion_times(values);
        case hand::CommandSemantic::kStallDetectionTimes:
            return device.set_stall_detection_times(values);
        case hand::CommandSemantic::kStallThresholds:
            return device.set_stall_thresholds(values);
        case hand::CommandSemantic::kStallHoldCurrents:
            return device.set_stall_hold_currents(values);
        case hand::CommandSemantic::kLimitCompensations:
            return device.set_limit_compensations(values);
        default:
            throw hand::UnsupportedOperationError("use the typed angle command endpoint");
        }
    }
    hand::CommandHandle submit_operation(hand::HandDevice &device, hand::DeviceOperation operation) {
        switch (operation) {
        case hand::DeviceOperation::kClearFaults:
            return device.clear_faults();
        case hand::DeviceOperation::kEnable:
            return device.enable();
        case hand::DeviceOperation::kDisable:
            return device.disable();
        case hand::DeviceOperation::kCalibrate:
            return device.calibrate();
        case hand::DeviceOperation::kSaveParameters:
            return device.save_parameters();
        default:
            throw hand::UnsupportedOperationError("operation is not a discrete command");
        }
    }
    hand::RawVectorSample read_raw(hand::HandDevice &device, hand::TelemetrySemantic semantic,
                                   std::chrono::milliseconds timeout) {
        switch (semantic) {
        case hand::TelemetrySemantic::kConfiguredSpeeds:
            return device.get_configured_speeds(timeout);
        case hand::TelemetrySemantic::kConfiguredAccelerations:
            return device.get_configured_accelerations(timeout);
        case hand::TelemetrySemantic::kCurrentLimits:
            return device.get_current_limits(timeout);
        case hand::TelemetrySemantic::kPositionKp:
            return device.get_position_kp(timeout);
        case hand::TelemetrySemantic::kPositionKd:
            return device.get_position_kd(timeout);
        case hand::TelemetrySemantic::kTorqueKd:
            return device.get_torque_kd(timeout);
        case hand::TelemetrySemantic::kTorqueLimits:
            return device.get_torques_limits(timeout);
        case hand::TelemetrySemantic::kMotionTimes:
            return device.get_motion_times(timeout);
        case hand::TelemetrySemantic::kStallDetectionTimes:
            return device.get_stall_detection_times(timeout);
        case hand::TelemetrySemantic::kStallThresholds:
            return device.get_stall_thresholds(timeout);
        case hand::TelemetrySemantic::kStallHoldCurrents:
            return device.get_stall_hold_currents(timeout);
        case hand::TelemetrySemantic::kLimitCompensations:
            return device.get_limit_compensations(timeout);
        default:
            throw hand::UnsupportedOperationError("channel has no raw configuration getter");
        }
    }
    std::string family_name(hand::CommandFamily value) {
        switch (value) {
        case hand::CommandFamily::kAngleTarget:
            return "angle_target";
        case hand::CommandFamily::kSpeedSetting:
            return "speed_setting";
        case hand::CommandFamily::kAccelerationSetting:
            return "acceleration_setting";
        case hand::CommandFamily::kCurrentLimit:
            return "current_limit";
        case hand::CommandFamily::kControlGain:
            return "control_gain";
        case hand::CommandFamily::kTorqueLimit:
            return "torque_limit";
        case hand::CommandFamily::kTorqueTarget:
            return "torque_target";
        case hand::CommandFamily::kMotionTime:
            return "motion_time_setting";
        case hand::CommandFamily::kStallConfiguration:
            return "stall_configuration";
        case hand::CommandFamily::kLimitCompensation:
            return "limit_compensation";
        case hand::CommandFamily::kDiscreteOperation:
            return "discrete_operation";
        }
        throw std::logic_error("unknown command family");
    }
    msg::HandCommandStatus command_status(const hand::CommandHandle &handle, const CommandContext &context,
                                          const rclcpp::Time &now) {
        msg::HandCommandStatus result;
        result.client_id = context.client_id;
        result.user_sequence = context.sequence;
        result.publisher_stamp = context.published;
        result.driver_received_stamp = context.received;
        result.command_id = handle.command_id();
        result.family = family_name(handle.family());
        const auto history = handle.history();
        const auto stage = history.back().stage;
        result.stage = command_stage(stage);
        result.sent_frames = history.back().sent_frames;
        result.ack_support = handle.ack_support() == hand::AckSupport::kUnsupported
                                 ? msg::HandCommandStatus::ACK_UNSUPPORTED
                                 : msg::HandCommandStatus::ACK_SUPPORTED;
        for (const auto &value : history) {
            msg::HandCommandTransition transition;
            transition.stage = command_stage(value.stage);
            transition.stamp = project_time(value.timestamp, now);
            transition.sent_frames = value.sent_frames;
            result.transitions.push_back(transition);
        }
        if (stage == hand::CommandStage::kFailed || stage == hand::CommandStage::kCancelled) {
            result.error_code = stage == hand::CommandStage::kCancelled ? "closed" : "internal";
            if (stage == hand::CommandStage::kCancelled) {
                result.error_message = "command cancelled by SDK lifecycle";
            }
            try {
                if (auto error = handle.error()) {
                    std::rethrow_exception(error);
                }
            } catch (const std::exception &error) {
                result.error_code = error_code(error);
                result.error_message = error.what();
            } catch (...) {
                result.error_message = "non-standard SDK exception";
            }
        }
        return result;
    }
} // namespace linkerbot_hand_ros_cpp
