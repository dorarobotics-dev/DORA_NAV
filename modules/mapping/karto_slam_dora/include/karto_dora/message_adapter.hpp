#pragma once

#include "karto_dora/types.hpp"

#include <stdexcept>
#include <string>
#include <string_view>

namespace karto_dora {

class MessageError : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

LaserScan ParseLaserScan(std::string_view payload);
TimedPose2d ParseOdometry(std::string_view payload);
Pose2d ParseInitialPose(std::string_view payload);
// Convert the internal planar CorrectedPose into DORA_NAV's pose JSON.
// Optional top-level health and prediction metadata is preserved unchanged.
std::string ToDoraPoseMessage(std::string_view corrected_pose_payload);

}  // namespace karto_dora
