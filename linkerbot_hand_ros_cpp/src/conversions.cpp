#include "linkerbot_hand_ros_cpp/conversions.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace linkerbot_hand_ros_cpp {
    namespace {
        template <typename T> std::vector<std::string> strings(const T &values) {
            std::vector<std::string> result;
            for (const auto &value : values) {
                result.emplace_back(value);
            }
            return result;
        }
        std::string confidence(hand::Confidence value) {
            switch (value) {
            case hand::Confidence::kMeasured:
                return "measured";
            case hand::Confidence::kDocumented:
                return "documented";
            case hand::Confidence::kApproximate:
                return "approximate";
            case hand::Confidence::kConflicting:
                return "conflicting";
            default:
                return "unknown";
            }
        }
        msg::NumericRange range(const hand::NumericRange &value, std::string_view unit = "") {
            msg::NumericRange result;
            result.minimum = value.minimum;
            result.maximum = value.maximum;
            result.integer_only = value.integer_only;
            result.has_step = value.step.has_value();
            result.step = value.step.value_or(0.0);
            result.unit = unit;
            return result;
        }
        msg::PercentMapping percent(const std::optional<hand::PercentMapping> &value) {
            msg::PercentMapping result;
            result.confidence = "unknown";
            if (!value) {
                return result;
            }
            result.supported = true;
            result.raw_at_zero = value->raw_at_zero;
            result.raw_at_hundred = value->raw_at_hundred;
            result.zero_label = value->zero_label;
            result.hundred_label = value->hundred_label;
            result.confidence = confidence(value->confidence);
            switch (value->quantization) {
            case hand::Quantization::kFloor:
                result.quantization = "floor";
                break;
            case hand::Quantization::kCeil:
                result.quantization = "ceil";
                break;
            default:
                result.quantization = "nearest_ties_away_from_zero";
                break;
            }
            return result;
        }
        std::string read_mode(hand::ReadMode value) {
            switch (value) {
            case hand::ReadMode::kQuery:
                return "query";
            case hand::ReadMode::kReport:
                return "report";
            case hand::ReadMode::kQueryAndReport:
                return "query_and_report";
            case hand::ReadMode::kCommandCache:
                return "command_cache";
            case hand::ReadMode::kCommandResponse:
                return "command_response";
            }
            throw std::logic_error("unknown read mode");
        }
        std::string operation_name(hand::DeviceOperation value) {
            switch (value) {
            case hand::DeviceOperation::kReadDeviceInfo:
                return "read_device_info";
            case hand::DeviceOperation::kClearFaults:
                return "clear_faults";
            case hand::DeviceOperation::kEnable:
                return "enable";
            case hand::DeviceOperation::kDisable:
                return "disable";
            case hand::DeviceOperation::kCalibrate:
                return "calibrate";
            case hand::DeviceOperation::kSaveParameters:
                return "save_parameters";
            case hand::DeviceOperation::kSaveParametersConfirmed:
                return "save_parameters_confirmed";
            }
            throw std::logic_error("unknown operation");
        }
        std::string projection(const hand::TelemetryCapability &value) {
            // The C++ SDK has no projection enum yet. These are adapter mappings for
            // its typed ForceSample payloads, not invented device capability values.
            if (value.semantic != hand::TelemetrySemantic::kForce) {
                return "raw_vector";
            }
            if (value.raw_encoding == "o30_tactile_descriptor") {
                return "tactile_descriptor";
            }
            if (value.shape.size() == 3) {
                return "tactile_matrix";
            }
            return "raw_vector";
        }
        msg::IdentityCapability identity_message(const hand::DeviceCapabilities &caps,
                                                 const std::optional<hand::DeviceIdentity> &identity) {
            msg::IdentityCapability result;
            result.model = model_name(caps.model);
            result.side = side_name(caps.side);
            result.verified = identity.has_value();
            if (!identity) {
                return result;
            }
            const auto &value = *identity;
            auto field = [&](const char *name, const auto &source, bool &present, std::string &target) {
                present = source.has_value();
                if (source) {
                    target = *source;
                    result.supported_fields.emplace_back(name);
                }
            };
            field("firmware", value.firmware_version, result.has_firmware, result.firmware);
            field("hardware_revision", value.hardware_version, result.has_hardware_revision, result.hardware_revision);
            field("mechanical_revision", value.mechanical_version, result.has_mechanical_revision,
                  result.mechanical_revision);
            field("serial_number", value.serial_number, result.has_serial_number, result.serial_number);
            field("product_code", value.product_code, result.has_product_code, result.product_code);
            field("protocol_name", value.protocol_name, result.has_protocol_name, result.protocol_name);
            field("protocol_version", value.protocol_version, result.has_protocol_version, result.protocol_version);
            field("microcontroller_uid", value.device_uid, result.has_microcontroller_uid, result.microcontroller_uid);
            // C++ identity.side is resolved, not independent evidence of reported side.
            return result;
        }
    } // namespace

    std::string semantic_name(hand::CommandSemantic value) {
        switch (value) {
        case hand::CommandSemantic::kAngles:
            return "angles";
        case hand::CommandSemantic::kSpeeds:
            return "speeds";
        case hand::CommandSemantic::kAccelerations:
            return "accelerations";
        case hand::CommandSemantic::kCurrentLimits:
            return "current_limits";
        case hand::CommandSemantic::kPositionKp:
            return "position_kp";
        case hand::CommandSemantic::kPositionKd:
            return "position_kd";
        case hand::CommandSemantic::kTorqueKd:
            return "torque_kd";
        case hand::CommandSemantic::kTorqueLimits:
            return "torques_limits";
        case hand::CommandSemantic::kTorques:
            return "torques";
        case hand::CommandSemantic::kMotionTimes:
            return "motion_times";
        case hand::CommandSemantic::kStallDetectionTimes:
            return "stall_detection_times";
        case hand::CommandSemantic::kStallThresholds:
            return "stall_thresholds";
        case hand::CommandSemantic::kStallHoldCurrents:
            return "stall_hold_currents";
        case hand::CommandSemantic::kLimitCompensations:
            return "limit_compensations";
        }
        throw std::logic_error("unknown command semantic");
    }
    std::string semantic_name(hand::TelemetrySemantic value) {
        switch (value) {
        case hand::TelemetrySemantic::kAngles:
            return "angles";
        case hand::TelemetrySemantic::kMeasuredSpeeds:
            return "measured_speeds";
        case hand::TelemetrySemantic::kConfiguredSpeeds:
            return "configured_speeds";
        case hand::TelemetrySemantic::kConfiguredAccelerations:
            return "configured_accelerations";
        case hand::TelemetrySemantic::kCurrents:
            return "currents";
        case hand::TelemetrySemantic::kPositionKp:
            return "position_kp";
        case hand::TelemetrySemantic::kPositionKd:
            return "position_kd";
        case hand::TelemetrySemantic::kTorqueKd:
            return "torque_kd";
        case hand::TelemetrySemantic::kCurrentLimits:
            return "current_limits";
        case hand::TelemetrySemantic::kTorqueLimits:
            return "torques_limits";
        case hand::TelemetrySemantic::kTorques:
            return "torques";
        case hand::TelemetrySemantic::kTemperatures:
            return "temperatures";
        case hand::TelemetrySemantic::kForce:
            return "force";
        case hand::TelemetrySemantic::kForceFns:
            return "force_fns";
        case hand::TelemetrySemantic::kForceFt:
            return "force_ft";
        case hand::TelemetrySemantic::kForceFts:
            return "force_fts";
        case hand::TelemetrySemantic::kForceFd:
            return "force_fd";
        case hand::TelemetrySemantic::kForceFds:
            return "force_fds";
        case hand::TelemetrySemantic::kForcePx:
            return "force_px";
        case hand::TelemetrySemantic::kFaults:
            return "faults";
        case hand::TelemetrySemantic::kVoltages:
            return "voltages";
        case hand::TelemetrySemantic::kMotionTimes:
            return "motion_times";
        case hand::TelemetrySemantic::kStallDetectionTimes:
            return "stall_detection_times";
        case hand::TelemetrySemantic::kStallThresholds:
            return "stall_thresholds";
        case hand::TelemetrySemantic::kStallHoldCurrents:
            return "stall_hold_currents";
        case hand::TelemetrySemantic::kLimitCompensations:
            return "limit_compensations";
        }
        throw std::logic_error("unknown telemetry semantic");
    }
    builtin_interfaces::msg::Time project_time(std::chrono::steady_clock::time_point value, const rclcpp::Time &now) {
        const auto age =
            std::max(std::chrono::steady_clock::duration::zero(), std::chrono::steady_clock::now() - value);
        const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(age).count();
        return rclcpp::Time(std::max<int64_t>(0, now.nanoseconds() - ns), now.get_clock_type());
    }
    msg::SampleMetadata metadata(const hand::SampleMeta &value, const rclcpp::Time &now) {
        msg::SampleMetadata result;
        result.sequence = value.sequence;
        result.valid = value.valid;
        result.stale = value.stale;
        result.connection_generation = value.connection_generation;
        result.within_declared_range = value.within_declared_range;
        if (value.collected_at && value.valid) {
            result.collected_at = project_time(*value.collected_at, now);
        }
        if (value.age) {
            result.age_s = std::max(0.0, std::chrono::duration<double>(*value.age).count());
        } else if (value.collected_at) {
            result.age_s = std::max(
                0.0, std::chrono::duration<double>(std::chrono::steady_clock::now() - *value.collected_at).count());
        }
        if (value.origin == hand::TelemetryValueOrigin::kNone) {
            return result;
        }
        // Delivery distinguishes a real write response from a locally cached
        // target: the SDK's value-origin enum calls both configuration caches.
        if (value.delivery == hand::TelemetryDelivery::kCommandResponse) {
            result.origin = value.origin == hand::TelemetryValueOrigin::kDeviceMeasured
                                ? msg::SampleMetadata::ORIGIN_DEVICE_COMMAND_FEEDBACK
                                : msg::SampleMetadata::ORIGIN_DEVICE_CONFIG_RESPONSE;
        } else if (value.delivery == hand::TelemetryDelivery::kCommandCache ||
                   value.origin == hand::TelemetryValueOrigin::kCommandCache) {
            result.origin = msg::SampleMetadata::ORIGIN_COMMAND_CACHE;
        } else if (value.delivery == hand::TelemetryDelivery::kPeriodicReport) {
            result.origin = msg::SampleMetadata::ORIGIN_DEVICE_REPORT;
        } else if (value.delivery == hand::TelemetryDelivery::kQueryResponse) {
            result.origin = value.origin == hand::TelemetryValueOrigin::kDeviceConfigured
                                ? msg::SampleMetadata::ORIGIN_DEVICE_CONFIG_RESPONSE
                                : msg::SampleMetadata::ORIGIN_DEVICE_RESPONSE;
        }
        // QueryOrReport cannot be distinguished by this SDK facade. Preserve the
        // sample's validity/freshness, but leave wire origin NONE rather than guess.
        return result;
    }
    msg::HandCapabilities capabilities(const hand::DeviceCapabilities &caps,
                                       const std::optional<hand::DeviceIdentity> &identity) {
        msg::HandCapabilities result;
        result.revision = "1.5"; // ROS wire schema; independent of SDK capability schema.
        result.identity = identity_message(caps, identity);
        const auto *angle = caps.command(hand::CommandSemantic::kAngles);
        const auto angle_unit = angle ? angle->raw_unit.value_or("") : std::string_view("");
        for (const auto &value : caps.joints) {
            msg::JointCapability joint;
            joint.index = value.index;
            joint.name = value.name;
            switch (value.motion_type) {
            case hand::JointMotionType::kFlexion:
                joint.motion_type = "flexion";
                break;
            case hand::JointMotionType::kAbduction:
                joint.motion_type = "abduction";
                break;
            case hand::JointMotionType::kRoll:
                joint.motion_type = "roll";
                break;
            case hand::JointMotionType::kRotation:
                joint.motion_type = "rotation";
                break;
            case hand::JointMotionType::kWrist:
                joint.motion_type = "wrist";
                break;
            default:
                joint.motion_type = "unknown";
            }
            joint.raw_angle_range = range(value.raw_angle_range, angle_unit);
            joint.percent_mapping = percent(value.percent_mapping);
            joint.physical_angle_mapping.confidence = "unknown";
            if (value.physical_angle_mapping) {
                const auto &mapping = *value.physical_angle_mapping;
                auto &out = joint.physical_angle_mapping;
                out.supported = true;
                out.raw_at_first = mapping.raw_at_first;
                out.raw_at_second = mapping.raw_at_second;
                out.rad_at_first = mapping.radians_at_first;
                out.rad_at_second = mapping.radians_at_second;
                out.confidence = confidence(mapping.confidence);
            }
            joint.coupled_with = strings(value.coupled_with);
            result.joints.push_back(std::move(joint));
        }
        for (const auto &value : caps.commands) {
            msg::CommandCapability command;
            command.semantic = semantic_name(value.semantic);
            command.channel_names = strings(value.channels);
            command.raw_encoding = value.raw_encoding;
            command.has_raw_unit = value.raw_unit.has_value();
            command.raw_unit = value.raw_unit.value_or("");
            command.has_raw_scale_note = value.raw_scale_note.has_value();
            command.raw_scale_note = value.raw_scale_note.value_or("");
            command.unit_confidence = confidence(value.unit_confidence);
            command.ack_support = value.ack_support == hand::AckSupport::kUnsupported ? "unsupported" : "supported";
            command.returns_current_state = value.returns_current_state;
            for (size_t i = 0; i < value.channels.size(); ++i) {
                if (const auto *raw_range = value.range_for(i)) {
                    command.raw_ranges.push_back(range(*raw_range, command.raw_unit));
                }
                command.percent_mappings.push_back(
                    percent(i < value.percent_mappings.size() ? value.percent_mappings[i] : std::nullopt));
            }
            result.commands.push_back(std::move(command));
        }
        for (const auto &value : caps.telemetry) {
            msg::TelemetryCapability telemetry;
            telemetry.semantic = semantic_name(value.semantic);
            telemetry.channel_names = strings(value.channels);
            for (auto dim : value.shape) {
                telemetry.shape.push_back(dim);
            }
            if (value.raw_range) {
                for (size_t i = 0; i < value.channels.size(); ++i) {
                    telemetry.raw_ranges.push_back(range(*value.raw_range, value.raw_unit.value_or("")));
                }
            }
            telemetry.raw_encoding = value.raw_encoding;
            telemetry.payload_projection = projection(value);
            telemetry.decoded_statuses =
                value.semantic == hand::TelemetrySemantic::kFaults && state_supported("fault_statuses", caps, identity);
            telemetry.has_raw_unit = value.raw_unit.has_value();
            telemetry.raw_unit = value.raw_unit.value_or("");
            telemetry.has_raw_scale_note = value.raw_scale_note.has_value();
            telemetry.raw_scale_note = value.raw_scale_note.value_or("");
            telemetry.unit_confidence = confidence(value.unit_confidence);
            telemetry.read_mode = read_mode(value.read_mode);
            if (value.recommended_period_ms && *value.recommended_period_ms > 0) {
                telemetry.recommended_hz = 1000.0 / *value.recommended_period_ms;
            }
            // minimum_period_ms is scheduling advice, not a measured maximum rate.
            telemetry.stale_after_s = value.freshness_ms.value_or(0.0) / 1000.0;
            telemetry.device_measured = value.device_measured;
            telemetry.device_queried = value.device_queried;
            result.telemetry.push_back(std::move(telemetry));
        }
        for (const auto &value : caps.force_fields()) {
            msg::ForceFieldCapability field;
            field.name = value.name;
            field.support_known = value.supported.has_value();
            field.supported = value.supported.value_or(false);
            if (value.query_channel) {
                field.query_channel = semantic_name(*value.query_channel);
            }
            field.shared_fields = strings(value.shared_fields);
            field.sensor_names = strings(value.sensor_names);
            field.raw_unit = value.raw_unit.value_or("");
            field.raw_encoding = value.raw_encoding.value_or("");
            field.raw_scale_note = value.raw_scale_note.value_or("");
            result.force_fields.push_back(std::move(field));
        }
        auto &transport = result.transport;
        transport.kind = caps.transport.kind == hand::TransportKind::kCan ? "can" : "can_fd";
        transport.has_nominal_bitrate = caps.transport.nominal_bitrate.has_value();
        transport.nominal_bitrate = caps.transport.nominal_bitrate.value_or(0);
        transport.has_data_bitrate = caps.transport.data_bitrate.has_value();
        transport.data_bitrate = caps.transport.data_bitrate.value_or(0);
        transport.addressing_note = caps.transport.addressing_note;
        auto &operations = result.operations;
        operations.read_device_info = caps.operations.read_device_info;
        operations.clear_faults = caps.operations.clear_faults;
        operations.enable = caps.operations.enable;
        operations.disable = caps.operations.disable;
        operations.calibrate = caps.operations.calibrate;
        operations.save_parameters = caps.operations.save_parameters;
        for (auto value : caps.operations.acknowledged_operations) {
            operations.acknowledged_operations.push_back(operation_name(value));
        }
        const hand::TelemetrySemantic stall_semantics[] = {hand::TelemetrySemantic::kStallDetectionTimes,
                                                           hand::TelemetrySemantic::kStallThresholds,
                                                           hand::TelemetrySemantic::kStallHoldCurrents};
        auto &stall = result.stall_protection;
        msg::StallFieldCapability *fields[] = {&stall.detection_time, &stall.threshold, &stall.hold_current};
        bool readable = true;
        bool writable = true;
        for (size_t i = 0; i < 3; ++i) {
            const auto *input = caps.telemetry_channel(stall_semantics[i]);
            const auto command = i == 0   ? hand::CommandSemantic::kStallDetectionTimes
                                 : i == 1 ? hand::CommandSemantic::kStallThresholds
                                          : hand::CommandSemantic::kStallHoldCurrents;
            auto &out = *fields[i];
            out.semantic = semantic_name(stall_semantics[i]);
            out.read_supported = input != nullptr;
            out.write_supported = caps.supports(command);
            readable = readable && out.read_supported;
            writable = writable && out.write_supported;
            if (!input) {
                continue;
            }
            out.channel_names = strings(input->channels);
            out.raw_encoding = input->raw_encoding;
            out.has_raw_unit = input->raw_unit.has_value();
            out.raw_unit = input->raw_unit.value_or("");
            out.has_raw_scale_note = input->raw_scale_note.has_value();
            out.raw_scale_note = input->raw_scale_note.value_or("");
            out.unit_confidence = confidence(input->unit_confidence);
            if (input->raw_range) {
                for (size_t j = 0; j < input->channels.size(); ++j) {
                    out.raw_ranges.push_back(range(*input->raw_range, out.raw_unit));
                }
            }
        }
        stall.supported = readable || writable;
        stall.read_supported = operations.read_stall_protection = readable;
        stall.write_supported = operations.write_stall_protection = writable;
        stall.has_hold_current = fields[2]->read_supported || fields[2]->write_supported;
        stall.channel_names = fields[0]->channel_names;
        stall.physical_semantics_confidence = "unknown";
        return result;
    }
    msg::HandDeviceInfo device_info(const hand::DeviceIdentity &value, const rclcpp::Time &now, uint64_t generation) {
        msg::HandDeviceInfo result;
        result.model = model_name(value.model);
        result.side = side_name(value.side);
        result.has_serial_number = value.serial_number.has_value();
        result.serial_number = value.serial_number.value_or("");
        result.has_product_code = value.product_code.has_value();
        result.product_code = value.product_code.value_or("");
        auto version = [](const std::optional<std::string> &source, msg::VersionField &target) {
            target.available = source.has_value();
            target.raw_text = target.text = source.value_or("");
        };
        version(value.hardware_version, result.pcb_version);
        version(value.firmware_version, result.firmware_version);
        version(value.mechanical_version, result.mechanical_version);
        result.has_protocol_name = value.protocol_name.has_value();
        result.protocol_name = value.protocol_name.value_or("");
        result.has_protocol_version = value.protocol_version.has_value();
        result.protocol_version = value.protocol_version.value_or("");
        result.has_microcontroller_uid = value.device_uid.has_value();
        result.microcontroller_uid = value.device_uid.value_or("");
        result.capability_revision = "1.5";
        hand::SampleMeta meta;
        meta.valid = true;
        meta.stale = false;
        meta.collected_at = value.timestamp;
        meta.connection_generation = generation;
        meta.origin = hand::TelemetryValueOrigin::kDeviceConfigured;
        meta.delivery = hand::TelemetryDelivery::kQueryResponse;
        result.meta = metadata(meta, now);
        return result;
    }
    msg::HandJointState angles(const hand::AngleSample &value, const hand::DeviceCapabilities &caps,
                               const rclcpp::Time &now) {
        msg::HandJointState result;
        for (const auto &joint : caps.joints) {
            result.joint_names.emplace_back(joint.name);
        }
        result.raw = value.raw;
        const auto nan = std::numeric_limits<double>::quiet_NaN();
        for (size_t i = 0; i < caps.joints.size(); ++i) {
            const bool percent_valid = i < value.percent.size() && value.percent[i].has_value();
            const bool physical_valid = i < value.physical_rad.size() && value.physical_rad[i].has_value();
            result.percent.push_back(i < value.percent.size() ? value.percent[i].value_or(nan) : nan);
            result.percent_valid.push_back(percent_valid);
            result.physical_rad.push_back(i < value.physical_rad.size() ? value.physical_rad[i].value_or(nan) : nan);
            result.physical_rad_valid.push_back(physical_valid);
        }
        result.meta = metadata(value.meta, now);
        return result;
    }
    msg::HandTelemetry telemetry(const hand::RawVectorSample &value, const hand::TelemetryCapability &caps,
                                 const rclcpp::Time &now) {
        msg::HandTelemetry result;
        result.semantic = semantic_name(caps.semantic);
        result.channel_names = strings(caps.channels);
        result.raw = value.raw;
        result.raw_unit = caps.raw_unit.value_or("");
        result.raw_scale_note = caps.raw_scale_note.value_or("");
        result.unit_confidence = confidence(caps.unit_confidence);
        result.device_measured = caps.device_measured;
        result.device_queried = caps.device_queried;
        result.meta = metadata(value.meta, now);
        return result;
    }
    msg::HandForceField force_field(const hand::ForceFieldSample &value, const rclcpp::Time &now) {
        msg::HandForceField result;
        result.name = value.name;
        result.query_channel = semantic_name(value.query_channel);
        result.scale_note = value.scale_note.value_or("");
        result.meta = metadata(value.meta, now);
        for (size_t i = 0; i < value.sensor_names.size(); ++i) {
            msg::ForceSensorValue sensor;
            sensor.sensor_name = value.sensor_names[i];
            sensor.present = i < value.raw.size() && value.raw[i].has_value();
            if (i < value.raw.size()) {
                sensor.raw = value.raw[i].value_or(std::vector<double>{});
            }
            if (i < value.units.size()) {
                sensor.unit = value.units[i].value_or("");
            }
            if (i < value.encodings.size()) {
                sensor.encoding = value.encodings[i].value_or("");
            }
            if (i < value.valid_values.size()) {
                for (bool valid : value.valid_values[i].value_or(std::vector<bool>{})) {
                    sensor.valid_values.push_back(valid);
                }
            }
            result.sensors.push_back(std::move(sensor));
        }
        return result;
    }
    msg::HandTactileState tactile(const hand::ForceSample &value, const hand::TelemetryCapability &caps,
                                  const rclcpp::Time &now) {
        msg::HandTactileState result;
        result.meta = metadata(value.meta, now);
        result.raw_unit = caps.raw_unit.value_or("");
        result.raw_scale_note = caps.raw_scale_note.value_or("");
        result.unit_confidence = confidence(caps.unit_confidence);
        for (const auto &input : value.tactile) {
            msg::TactileMatrix matrix;
            matrix.finger_name = input.name;
            matrix.rows = input.rows;
            matrix.columns = input.columns;
            for (const auto &row : input.raw) {
                matrix.raw.insert(matrix.raw.end(), row.begin(), row.end());
            }
            matrix.has_online = input.online.has_value();
            matrix.online = input.online.value_or(false);
            result.fingers.push_back(std::move(matrix));
        }
        return result;
    }
    msg::HandTactileSensorState sensors(const hand::ForceSample &value, const rclcpp::Time &now) {
        msg::HandTactileSensorState result;
        result.meta = metadata(value.meta, now);
        for (const auto &input : value.sensors) {
            msg::HandTactileSensor sensor;
            sensor.finger_name = input.sensor_name;
            sensor.selected_sensor = input.selected_sensor;
            sensor.available = input.available;
            sensor.sensor_type = input.sensor_type;
            sensor.data_unit = input.data_unit;
            sensor.descriptor = input.descriptor;
            sensor.requested_rows = input.rows;
            sensor.requested_columns = input.columns;
            sensor.raw_bytes = input.raw_data;
            sensor.total_data_length = input.raw_data.size();
            for (size_t i = 0; i < input.fields.size(); ++i) {
                const auto &source = input.fields[i];
                msg::HandTactileField field;
                field.semantic = source.semantic;
                field.data_type = source.encoding;
                field.element_count = source.count;
                field.byte_offset = source.offset;
                const auto end = i + 1 < input.fields.size() ? input.fields[i + 1].offset : input.raw_data.size();
                field.byte_length = end >= source.offset ? end - source.offset : 0;
                field.values = source.values;
                // Component count/window flags are unavailable in this SDK facade.
                sensor.fields.push_back(std::move(field));
            }
            result.sensors.push_back(std::move(sensor));
        }
        return result;
    }
    msg::HandNormalForceResultant normal_force(const hand::NormalForceResultantSample &value, const rclcpp::Time &now) {
        msg::HandNormalForceResultant result;
        result.finger_names = value.sensor_names;
        result.normal_force_resultant = value.raw;
        for (const auto &unit : value.units) {
            result.units.push_back(unit.value_or(""));
        }
        result.meta = metadata(value.meta, now);
        return result;
    }
    uint8_t command_stage(hand::CommandStage value) {
        switch (value) {
        case hand::CommandStage::kAccepted:
            return msg::HandCommandStatus::STAGE_ACCEPTED;
        case hand::CommandStage::kDispatched:
            return msg::HandCommandStatus::STAGE_DISPATCHED;
        case hand::CommandStage::kAcknowledged:
            return msg::HandCommandStatus::STAGE_ACKNOWLEDGED;
        case hand::CommandStage::kSuperseded:
            return msg::HandCommandStatus::STAGE_SUPERSEDED;
        case hand::CommandStage::kFailed:
        case hand::CommandStage::kCancelled:
            return msg::HandCommandStatus::STAGE_FAILED;
        }
        throw std::logic_error("unknown command stage");
    }
    std::string error_code(const std::exception &error) {
        if (const auto *typed = dynamic_cast<const hand::HandError *>(&error)) {
            return std::string(hand::error_code_name(typed->code()));
        }
        if (dynamic_cast<const std::invalid_argument *>(&error)) {
            return "validation";
        }
        return "internal";
    }
} // namespace linkerbot_hand_ros_cpp
