#include "linkerbot_hand_ros_cpp/driver_config.hpp"
#include <jsoncpp/json/json.h>
#include <regex>
#include <sstream>

namespace linkerbot_hand_ros_cpp {
    std::string normalize_namespace(const std::string &value, const std::string &parent) {
        if (value.empty()) {
            throw hand::ValidationError("namespace must not be empty/root");
        }
        const auto full = value.front() == '/' ? value : parent + "/" + value;
        std::istringstream stream(full);
        std::string part, result;
        while (std::getline(stream, part, '/')) {
            if (part.empty()) {
                continue;
            }
            if (!std::regex_match(part, std::regex("[A-Za-z_][A-Za-z_0-9]*"))) {
                throw hand::ValidationError("invalid namespace: " + value);
            }
            result += "/" + part;
        }
        if (result.empty()) {
            throw hand::ValidationError("namespace must not be empty/root");
        }
        return result;
    }

    std::string physical_channel(const DriverConfig &config) {
        if (config.transport_json.empty()) {
            return "socketcan:" + config.device.interface_name;
        }
        Json::CharReaderBuilder builder;
        builder["rejectDupKeys"] = true;
        builder["failIfExtra"] = true;
        Json::Value value;
        std::string errors;
        std::istringstream stream(config.transport_json);
        if (!Json::parseFromStream(builder, stream, &value, &errors) || !value.isObject()) {
            throw hand::ValidationError("invalid transport_json: " + errors);
        }
        const auto backend = value["backend"].asString();
        if (backend == "socketcan") {
            return "socketcan:" + value["channel"].asString();
        }
        if (backend == "python_can") {
            return value["interface_type"].asString() + ":" + value["interface_name"].asString();
        }
        if (backend == "vendor_canfd") {
            return "minicanfd:" + std::to_string(value.get("device_index", 0).asUInt()) + ":" +
                   std::to_string(value.get("channel_index", 0).asUInt());
        }
        throw hand::UnsupportedOperationError("no physical channel mapping for " + backend);
    }

    std::string discovery_configuration(const hand::DiscoveryConfig &config) {
        // Persist resolved settings, including absolute library paths. A generated
        // deployment must scan identically even from a different working directory.
        Json::Value result(Json::objectValue);
        auto strings = [](const auto &values) {
            Json::Value array(Json::arrayValue);
            for (const auto &value : values) {
                array.append(value);
            }
            return array;
        };
        auto protocols = [](const auto &values) {
            Json::Value array(Json::arrayValue);
            for (auto value : values) {
                array.append(value == hand::TransportKind::kCan ? "can" : "can_fd");
            }
            return array;
        };
        auto seconds = [](auto value) { return std::chrono::duration<double>(value).count(); };
        result["backends"] = strings(config.backends);
        result["channels"] = strings(config.channels);
        result["total_timeout_s"] = seconds(config.total_timeout);
        result["response_timeout_s"] = seconds(config.response_timeout);
        result["send_timeout_s"] = seconds(config.send_timeout);
        result["inter_request_delay_s"] = seconds(config.inter_request_delay);
        result["route_queue_capacity"] = Json::UInt64(config.route_queue_capacity);
        result["l30_node_ids"] = Json::Value(Json::arrayValue);
        for (auto value : config.l30_node_ids) {
            result["l30_node_ids"].append(unsigned(value));
        }
        result["o30_request_ids"] = strings(config.o30_request_ids);
        auto &pcan = result["pcan"];
        pcan["channels"] = strings(config.pcan.channels);
        pcan["probe_protocols"] = protocols(config.pcan.probe_protocols);
        pcan["nominal_bitrate"] = config.pcan.nominal_bitrate;
        pcan["data_bitrate"] = config.pcan.data_bitrate;
        if (config.pcan.library_path) {
            pcan["library_path"] = std::filesystem::absolute(*config.pcan.library_path).string();
        }
        auto &timing = pcan["fd_timing"];
        const auto &t = config.pcan.fd_timing;
        timing["f_clock"] = t.clock_hz;
        timing["nom_brp"] = t.nominal_prescaler;
        timing["nom_tseg1"] = t.nominal_tseg1;
        timing["nom_tseg2"] = t.nominal_tseg2;
        timing["nom_sjw"] = t.nominal_sjw;
        timing["data_brp"] = t.data_prescaler;
        timing["data_tseg1"] = t.data_tseg1;
        timing["data_tseg2"] = t.data_tseg2;
        timing["data_sjw"] = t.data_sjw;
        auto &slcan = result["slcan"];
        slcan["channels"] = strings(config.slcan.channels);
        slcan["probe_protocols"] = protocols(config.slcan.probe_protocols);
        slcan["nominal_bitrate"] = config.slcan.nominal_bitrate;
        slcan["data_bitrate"] = config.slcan.data_bitrate;
        slcan["serial_baud"] = config.slcan.serial_baud;
        slcan["command_timeout_s"] = seconds(config.slcan.command_timeout);
        slcan["startup_delay_s"] = seconds(config.slcan.startup_delay);
        slcan["protocol"] = config.slcan.protocol == hand::SLCANProtocol::kCanableFd ? "canable_fd" : "lawicel_classic";
        auto &mini = result["minicanfd"];
        if (config.minicanfd.library_path) {
            mini["library_path"] = std::filesystem::absolute(*config.minicanfd.library_path).string();
        }
        mini["config"]["nominal_bitrate"] = config.minicanfd.nominal_bitrate;
        mini["config"]["data_bitrate"] = config.minicanfd.data_bitrate;
        mini["linux_channels"] = Json::Value(Json::objectValue);
        for (const auto &channel : config.minicanfd.channels) {
            mini["linux_channels"][std::to_string(channel.device_index)].append(channel.channel_index);
        }
        return Json::writeString(Json::StreamWriterBuilder(), result);
    }
} // namespace linkerbot_hand_ros_cpp
