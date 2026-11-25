#include "karto_dora/scan_preparer.hpp"

#include <cmath>
#include <limits>

namespace karto_dora {

bool IsFullCircleScan(const LaserScan &scan)
{
  if (scan.ranges.empty() || !std::isfinite(scan.angle_increment)) return false;
  const double span = std::abs(scan.angle_increment) * static_cast<double>(scan.ranges.size());
  return std::abs(span - 2.0 * M_PI) <= std::abs(scan.angle_increment) * 1.5;
}

double KartoMaximumAngle(const LaserScan &scan)
{
  const double intervals = IsFullCircleScan(scan)
      ? static_cast<double>(scan.ranges.size())
      : static_cast<double>(scan.ranges.size() - 1);
  return scan.angle_min + intervals * scan.angle_increment;
}

PreparedScan PrepareFixedBeamScan(const LaserScan &scan,
                                  const OdometryBuffer &odometry,
                                  const Pose2d &lidar_extrinsic,
                                  std::size_t expected_beams)
{
  PreparedScan result;
  result.scan = scan;
  result.stamp = scan.start_stamp;

  if (scan.ranges.size() != expected_beams) {
    result.drop_reason = "unexpected_beam_count";
    return result;
  }
  if (!std::isfinite(scan.angle_increment) || scan.angle_increment <= 0.0) {
    result.drop_reason = "invalid_angle_increment";
    return result;
  }
  if (!std::isfinite(scan.range_min) || !std::isfinite(scan.range_max) ||
      scan.range_min < 0.0 || scan.range_max <= scan.range_min) {
    result.drop_reason = "invalid_range_limits";
    return result;
  }

  const auto base_pose = odometry.Interpolate(scan.start_stamp);
  if (!base_pose) {
    result.drop_reason = "missing_scan_start_odom";
    return result;
  }
  result.base_pose = *base_pose;

  // Remove only an unsupported single-beam discontinuity. Real geometric
  // edges retain support from at least one adjacent beam and are preserved.
  constexpr double kIsolatedReturnJump = 0.5;
  const auto valid_range = [&scan](double range) {
    return std::isfinite(range) && range >= scan.range_min &&
           range <= scan.range_max;
  };
  for (double &range : result.scan.ranges) {
    if (!valid_range(range)) {
      range = std::numeric_limits<double>::quiet_NaN();
    }
  }
  for (std::size_t index = 1; index + 1 < scan.ranges.size(); ++index) {
    const double left = result.scan.ranges[index - 1];
    const double center = result.scan.ranges[index];
    const double right = result.scan.ranges[index + 1];
    if (valid_range(left) && valid_range(center) && valid_range(right) &&
        std::abs(center - left) > kIsolatedReturnJump &&
        std::abs(center - right) > kIsolatedReturnJump) {
      result.scan.ranges[index] = std::numeric_limits<double>::quiet_NaN();
    }
  }

  const double cos_offset = std::cos(lidar_extrinsic.yaw);
  const double sin_offset = std::sin(lidar_extrinsic.yaw);
  result.base_points.reserve(scan.ranges.size());
  for (std::size_t index = 0; index < result.scan.ranges.size(); ++index) {
    const double range = result.scan.ranges[index];
    if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
      continue;
    }
    const double angle = scan.angle_min + static_cast<double>(index) * scan.angle_increment;
    const double laser_x = range * std::cos(angle);
    const double laser_y = range * std::sin(angle);
    result.base_points.push_back({
        lidar_extrinsic.x + cos_offset * laser_x - sin_offset * laser_y,
        lidar_extrinsic.y + sin_offset * laser_x + cos_offset * laser_y});
  }
  return result;
}

}  // namespace karto_dora
