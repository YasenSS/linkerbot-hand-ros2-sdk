#include "linkerbot/hand/api/readings.hpp"
#include "linkerbot/hand/capabilities.hpp"
#include "linkerbot/hand/errors.hpp"
#include "linkerbot/hand/fault_status.hpp"
#include "linkerbot/hand/identity.hpp"
#include "linkerbot_hand_ros_cpp/conversions.hpp"
#include "linkerbot_hand_ros_cpp/public_contract.hpp"
#include <algorithm>
#include <cstdint>
#include <linkerbot/hand/l20/fault.hpp>
#include <linkerbot/hand/l20lite/fault.hpp>
#include <linkerbot/hand/l30/fault.hpp>
#include <linkerbot/hand/l6/fault.hpp>
#include <linkerbot/hand/o30/fault.hpp>
#include <linkerbot/hand/o6/fault.hpp>
#include <linkerbot_hand_msgs/msg/hand_fault_status_state.hpp>
#include <linkerbot_hand_msgs/msg/hand_joint_fault_status.hpp>
#include <optional>
#include <rclcpp/time.hpp>
#include <string>
#include <utility>

namespace linkerbot_hand_ros_cpp {
    namespace {
        bool fault_state_supported(const hand::DeviceCapabilities &caps,
                                   const std::optional<hand::DeviceIdentity> &identity) {
            if (!caps.supports(hand::TelemetrySemantic::kFaults)) {
                return false;
            }
            if (caps.model == hand::Model::kO30) {
                // Legacy HOP exposes bytes without defined bit meanings. Never decode
                // those bytes using the modern protocol's flags.
                return identity && identity->protocol_version &&
                       (*identity->protocol_version == "0.0.4" || *identity->protocol_version == "V0.0.4");
            }
            return caps.model == hand::Model::kL6 || caps.model == hand::Model::kO6 ||
                   caps.model == hand::Model::kL20Lite || caps.model == hand::Model::kL20 ||
                   caps.model == hand::Model::kL30;
        }
    }

    bool state_supported(const std::string &name, const hand::DeviceCapabilities &caps,
                         const std::optional<hand::DeviceIdentity> &identity) {
        if (name == "fault_statuses") {
            return fault_state_supported(caps, identity);
        }
        const auto *force = caps.telemetry_channel(hand::TelemetrySemantic::kForce);
        const bool descriptor = (force != nullptr) && force->raw_encoding == "o30_tactile_descriptor";
        const bool matrix = (force != nullptr) && force->shape.size() == 3;
        if (name == "tactile") {
            return matrix;
        }
        if (name == "tactile_sensors" || name == "normal_force_resultant") {
            return descriptor;
        }
        if (name == "force") {
            return (force != nullptr) && !descriptor && !matrix;
        }
        if (name.rfind("force_", 0) == 0) {
            const auto fields = caps.force_fields();
            const auto found = std::find_if(fields.begin(), fields.end(), [&](const auto &field) {
                return name == "force_" + std::string(field.name);
            });
            return found != fields.end() && found->supported.value_or(false);
        }
        if (name == "stall_protection") {
            return caps.supports(hand::TelemetrySemantic::kStallDetectionTimes) ||
                   caps.supports(hand::TelemetrySemantic::kStallThresholds) ||
                   caps.supports(hand::TelemetrySemantic::kStallHoldCurrents);
        }
        if (name == "angles") {
            return caps.supports(hand::TelemetrySemantic::kAngles);
        }
        const auto found = std::find_if(kStateInterfaces.begin(), kStateInterfaces.end(),
                                        [&](auto semantic) { return name == semantic_name(semantic); });
        return found != kStateInterfaces.end() && caps.supports(*found);
    }

    namespace {
        msg::HandJointFaultStatus decode_joint_fault(uint8_t raw, hand::Model model) {
            msg::HandJointFaultStatus status;
            status.raw = raw;
            uint8_t mask = 0;
            auto flag = [&](const char *name, auto bit, bool &target) {
                const auto bits = static_cast<uint8_t>(bit);
                mask |= bits;
                target = (raw & bits) != 0;
                status.supported_flags.emplace_back(name);
                if (target) {
                    status.descriptions.emplace_back(name);
                }
            };
            if (model == hand::Model::kL6) {
                using Code = hand::l6::FaultCode;
                flag("mcu_over_temperature", Code::kMcuOvertemp, status.mcu_over_temperature);
                flag("motor_over_temperature", Code::kMotorOvertemp, status.motor_over_temperature);
                flag("overload_2", Code::kOverload2, status.overload_2);
                flag("overload_1", Code::kOverload1, status.overload_1);
                flag("phase_a_over_current", Code::kPhaseAOvercurrent, status.phase_a_over_current);
                flag("phase_c_over_current", Code::kPhaseCOvercurrent, status.phase_c_over_current);
                flag("phase_b_over_current", Code::kPhaseBOvercurrent, status.phase_b_over_current);
            } else if (model == hand::Model::kL20) {
                using Code = hand::l20::L20FaultCode;
                flag("motor_communication_abnormal", Code::kMotorCommAbnormal, status.motor_communication_abnormal);
                flag("soft_rotor_lock", Code::kSoftRotorLock, status.soft_rotor_lock);
                flag("over_temperature", Code::kOverTemperature, status.over_temperature);
                flag("motor_self_test_abnormal", Code::kSelfCheckAbnormal, status.motor_self_test_abnormal);
                flag("voltage_abnormal", Code::kVoltageAbnormal, status.voltage_abnormal);
                flag("motor_stall", Code::kMotorStallFault, status.motor_stall);
                flag("over_current", Code::kMotorOverCurrent, status.over_current);
                flag("motor_rotor_lock", Code::kMotorRotorLock, status.motor_rotor_lock);
            } else if (model == hand::Model::kO30) {
                using Code = hand::o30::FaultCode;
                flag("hand_layer_stall", Code::kHandLayerStall, status.hand_layer_stall);
                flag("actuator_layer_stall", Code::kActuatorLayerStall, status.actuator_layer_stall);
                flag("over_current", Code::kOverCurrent, status.over_current);
                flag("over_temperature", Code::kOverTemperature, status.over_temperature);
                flag("actuator_abnormal", Code::kActuatorAbnormal, status.actuator_abnormal);
                flag("actuator_offline", Code::kActuatorOffline, status.actuator_offline);
            } else {
                // O6 and L20Lite publish identical SDK-defined flags. L30 exposes
                // equivalent named bit positions rather than an enum.
                auto common = [&](auto voltage, auto encoder, auto temperature, auto current, auto load) {
                    flag("voltage_abnormal", voltage, status.voltage_abnormal);
                    flag("magnetic_encoder_abnormal", encoder, status.magnetic_encoder_abnormal);
                    flag("over_temperature", temperature, status.over_temperature);
                    flag("over_current", current, status.over_current);
                    flag("load_overload", load, status.load_overload);
                };
                if (model == hand::Model::kO6) {
                    using Code = hand::o6::FaultCode;
                    common(Code::kVoltage, Code::kEncoder, Code::kOvertemp, Code::kOvercurrent, Code::kOverload);
                } else if (model == hand::Model::kL20Lite) {
                    using Code = hand::l20lite::FaultCode;
                    common(Code::kVoltage, Code::kEncoder, Code::kOvertemp, Code::kOvercurrent, Code::kOverload);
                } else {
                    common(1u << hand::l30::kFaultVoltageBit, 1u << hand::l30::kFaultMagneticEncoderBit,
                           1u << hand::l30::kFaultTemperatureBit, 1u << hand::l30::kFaultCurrentBit,
                           1u << hand::l30::kFaultLoadBit);
                }
            }
            status.reserved_bits = hand::decode_fault_status(raw, mask).reserved_bits;
            return status;
        }
    }
    msg::HandFaultStatusState fault_statuses(const hand::FaultSample &sample, const hand::DeviceCapabilities &caps,
                                             const std::optional<hand::DeviceIdentity> &identity,
                                             const rclcpp::Time &now) {
        msg::HandFaultStatusState result;
        result.meta = metadata(sample.meta, now);
        const auto *channel = caps.telemetry_channel(hand::TelemetrySemantic::kFaults);
        if ((channel == nullptr) || !state_supported("fault_statuses", caps, identity)) {
            result.meta.valid = false;
            result.meta.stale = true;
            if ((channel != nullptr) && sample.raw.size() == channel->channels.size()) {
                result.joint_names.assign(channel->channels.begin(), channel->channels.end());
                for (auto raw : sample.raw) {
                    msg::HandJointFaultStatus status;
                    status.raw = raw;
                    status.reserved_bits = raw;
                    result.statuses.push_back(std::move(status));
                }
            }
            return result;
        }
        if (sample.raw.size() != channel->channels.size()) {
            throw hand::ValidationError("fault byte count does not match capability channels");
        }
        result.joint_names.assign(channel->channels.begin(), channel->channels.end());
        for (const auto raw : sample.raw) {
            result.statuses.push_back(decode_joint_fault(raw, caps.model));
        }
        return result;
    }
} // namespace linkerbot_hand_ros_cpp
