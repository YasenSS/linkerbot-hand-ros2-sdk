#pragma once
#include "linkerbot_hand_ros_cpp/driver_config.hpp"
#include <array>
namespace linkerbot_hand_ros_cpp {
    inline constexpr std::array<hand::CommandSemantic, 14> kCommandInterfaces{
        hand::CommandSemantic::kAngles,
        hand::CommandSemantic::kSpeeds,
        hand::CommandSemantic::kAccelerations,
        hand::CommandSemantic::kCurrentLimits,
        hand::CommandSemantic::kTorqueLimits,
        hand::CommandSemantic::kTorques,
        hand::CommandSemantic::kMotionTimes,
        hand::CommandSemantic::kStallDetectionTimes,
        hand::CommandSemantic::kStallThresholds,
        hand::CommandSemantic::kStallHoldCurrents,
        hand::CommandSemantic::kLimitCompensations,
        hand::CommandSemantic::kPositionKp,
        hand::CommandSemantic::kPositionKd,
        hand::CommandSemantic::kTorqueKd};
    inline constexpr std::array<hand::TelemetrySemantic, 9> kGetterInterfaces{
        hand::TelemetrySemantic::kConfiguredSpeeds, hand::TelemetrySemantic::kConfiguredAccelerations,
        hand::TelemetrySemantic::kCurrentLimits,    hand::TelemetrySemantic::kTorqueLimits,
        hand::TelemetrySemantic::kMotionTimes,      hand::TelemetrySemantic::kLimitCompensations,
        hand::TelemetrySemantic::kPositionKp,       hand::TelemetrySemantic::kPositionKd,
        hand::TelemetrySemantic::kTorqueKd};
    inline constexpr std::array<hand::TelemetrySemantic, 19> kStateInterfaces{
        hand::TelemetrySemantic::kMeasuredSpeeds,
        hand::TelemetrySemantic::kConfiguredSpeeds,
        hand::TelemetrySemantic::kConfiguredAccelerations,
        hand::TelemetrySemantic::kCurrents,
        hand::TelemetrySemantic::kCurrentLimits,
        hand::TelemetrySemantic::kTorqueLimits,
        hand::TelemetrySemantic::kTorques,
        hand::TelemetrySemantic::kTemperatures,
        hand::TelemetrySemantic::kFaults,
        hand::TelemetrySemantic::kVoltages,
        hand::TelemetrySemantic::kMotionTimes,
        hand::TelemetrySemantic::kLimitCompensations,
        hand::TelemetrySemantic::kPositionKp,
        hand::TelemetrySemantic::kPositionKd,
        hand::TelemetrySemantic::kTorqueKd,
        hand::TelemetrySemantic::kStallDetectionTimes,
        hand::TelemetrySemantic::kStallThresholds,
        hand::TelemetrySemantic::kStallHoldCurrents,
        hand::TelemetrySemantic::kForce};
} // namespace linkerbot_hand_ros_cpp
