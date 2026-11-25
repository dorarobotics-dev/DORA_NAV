#pragma once

#include "karto_dora/types.hpp"

#include <algorithm>
#include <cmath>

namespace karto_dora {

inline double NormalizeYaw(double yaw) noexcept {
  return std::atan2(std::sin(yaw), std::cos(yaw));
}

inline double SignedYawDifference(double target, double source) noexcept {
  return NormalizeYaw(target - source);
}

inline bool IsRotationDominant(const Pose2d &previous_odometry,
                               const Pose2d &current_odometry,
                               double maximum_translation = 0.02,
                               double minimum_rotation = 0.01) noexcept {
  const double translation = std::hypot(
      current_odometry.x - previous_odometry.x,
      current_odometry.y - previous_odometry.y);
  const double rotation = std::abs(SignedYawDifference(
      current_odometry.yaw, previous_odometry.yaw));
  return translation <= maximum_translation && rotation >= minimum_rotation;
}

inline Pose2d FuseLocalizationPose(const Pose2d &odometry_prediction,
                                   const Pose2d &laser_match,
                                   bool rotation_dominant,
                                   double rotation_yaw_gain,
                                   double translation_yaw_gain,
                                   double maximum_yaw_correction) noexcept {
  Pose2d fused = laser_match;
  const double gain = rotation_dominant
      ? rotation_yaw_gain : translation_yaw_gain;
  const double innovation = std::clamp(
      SignedYawDifference(laser_match.yaw, odometry_prediction.yaw),
      -maximum_yaw_correction, maximum_yaw_correction);
  fused.yaw = NormalizeYaw(odometry_prediction.yaw + gain * innovation);
  return fused;
}

}  // namespace karto_dora
