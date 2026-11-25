#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace ydlidar_tmini_plus
{

constexpr double kPi = 3.14159265358979323846;

struct ScanPoint
{
  double angle_rad{0.0};
  double range_m{0.0};
  double intensity{0.0};
};

struct AngleWindow
{
  double min_deg{0.0};
  double max_deg{0.0};
};

struct ScanOptions
{
  std::string frame_id{"lidar"};
  int seq{0};
  double angle_min_deg{0.0};
  double angle_max_deg{360.0};
  double range_min{0.05};
  double range_max{12.0};
  bool clockwise{false};
  std::vector<AngleWindow> mask_windows;
  double scan_time_s{0.0};
  int target_count{450};
  double rotation_speed_hz{0.0};
};

inline double NormalizeDegrees(double angle_deg)
{
  angle_deg = std::fmod(angle_deg, 360.0);
  if (angle_deg < 0.0)
  {
    angle_deg += 360.0;
  }
  return angle_deg;
}

inline double DegToRad(double degrees)
{
  return degrees * kPi / 180.0;
}

inline std::string Trim(std::string value)
{
  value.erase(value.begin(), std::find_if(value.begin(), value.end(),
    [](unsigned char ch) { return !std::isspace(ch); }));
  value.erase(std::find_if(value.rbegin(), value.rend(),
    [](unsigned char ch) { return !std::isspace(ch); }).base(), value.end());
  return value;
}

inline std::vector<AngleWindow> ParseAngleWindows(const std::string &configuration)
{
  std::vector<AngleWindow> windows;
  size_t begin = 0;
  while (begin <= configuration.size())
  {
    const size_t end = configuration.find(',', begin);
    const std::string token = Trim(configuration.substr(
      begin, end == std::string::npos ? std::string::npos : end - begin));
    if (!token.empty())
    {
      const size_t separator = token.find('-');
      if (separator == std::string::npos || token.find('-', separator + 1) != std::string::npos)
      {
        throw std::invalid_argument("invalid lidar mask angle window: " + token);
      }
      const std::string minimum = Trim(token.substr(0, separator));
      const std::string maximum = Trim(token.substr(separator + 1));
      size_t minimum_end = 0, maximum_end = 0;
      const double min_deg = std::stod(minimum, &minimum_end);
      const double max_deg = std::stod(maximum, &maximum_end);
      if (!std::isfinite(min_deg) || !std::isfinite(max_deg) ||
          minimum_end != minimum.size() || maximum_end != maximum.size() ||
          min_deg < 0.0 || min_deg > 360.0 || max_deg < 0.0 || max_deg > 360.0)
      {
        throw std::invalid_argument("lidar mask angles must be within [0, 360]: " + token);
      }
      // Preserve 0/360 so a full-circle mask remains distinguishable from 0-0.
      windows.push_back({min_deg, max_deg});
    }
    if (end == std::string::npos)
    {
      break;
    }
    begin = end + 1;
  }
  return windows;
}

inline bool InAngleWindow(double angle_deg, double min_deg, double max_deg)
{
  if (min_deg <= max_deg)
  {
    return angle_deg >= min_deg && angle_deg <= max_deg;
  }
  return angle_deg >= min_deg || angle_deg <= max_deg;
}

inline bool IsAngleMasked(double angle_deg, const std::vector<AngleWindow> &windows)
{
  const double normalized = NormalizeDegrees(angle_deg);
  for (const auto &window : windows)
  {
    if (std::abs(window.max_deg - window.min_deg) == 360.0 ||
        InAngleWindow(normalized, NormalizeDegrees(window.min_deg),
                      NormalizeDegrees(window.max_deg)))
    {
      return true;
    }
  }
  return false;
}

inline double PublishedStartAngleDegrees(double angle_min_deg, double angle_max_deg)
{
  if (angle_max_deg < angle_min_deg && angle_min_deg > 180.0)
  {
    return angle_min_deg - 360.0;
  }
  return angle_min_deg;
}

template <typename Duration>
inline nlohmann::json HeaderJson(
  const std::string &frame_id,
  int seq,
  const std::chrono::time_point<std::chrono::system_clock, Duration> &stamp)
{
  const auto secs = std::chrono::duration_cast<std::chrono::seconds>(stamp.time_since_epoch());
  const auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(stamp.time_since_epoch() - secs);
  return {
    {"frame_id", frame_id},
    {"seq", seq},
    {"stamp", {{"sec", secs.count()}, {"nanosec", nanos.count()}}}
  };
}

template <typename Duration>
inline nlohmann::json BuildLaserScanJson(
  const std::vector<ScanPoint> &points,
  const ScanOptions &options,
  const std::chrono::time_point<std::chrono::system_clock, Duration> &stamp)
{
  const auto angle_valid = [](double angle) {
    return std::isfinite(angle) && angle >= 0.0 && angle <= 360.0;
  };
  if (!angle_valid(options.angle_min_deg) || !angle_valid(options.angle_max_deg) ||
      options.target_count < 2 || !std::isfinite(options.range_min) ||
      !std::isfinite(options.range_max) || options.range_min < 0.0 ||
      options.range_max <= options.range_min || !std::isfinite(options.scan_time_s) ||
      options.scan_time_s < 0.0 || !std::isfinite(options.rotation_speed_hz) ||
      options.rotation_speed_hz < 0.0)
  {
    throw std::invalid_argument("invalid LaserScan options");
  }
  for (const auto &window : options.mask_windows)
  {
    if (!angle_valid(window.min_deg) || !angle_valid(window.max_deg))
      throw std::invalid_argument("mask angles must be within [0, 360]");
  }
  const int target_count = options.target_count;
  std::vector<double> ranges(static_cast<size_t>(target_count), 0.0);
  std::vector<double> intensities(static_cast<size_t>(target_count), 0.0);
  std::vector<double> best_error(static_cast<size_t>(target_count),
                                 std::numeric_limits<double>::infinity());
  const double span_deg = options.angle_max_deg >= options.angle_min_deg
    ? options.angle_max_deg - options.angle_min_deg
    : (360.0 - options.angle_min_deg) + options.angle_max_deg;
  if (!(span_deg > 0.0 && span_deg <= 360.0))
  {
    throw std::invalid_argument("configured scan angle span must be within (0, 360]");
  }
  const bool full_circle = span_deg == 360.0;
  const double increment_deg = span_deg /
    static_cast<double>(full_circle ? target_count : target_count - 1);

  for (const auto &point : points)
  {
    // Invalid SDK returns must not reserve a bin ahead of a valid neighbour.
    if (!std::isfinite(point.angle_rad) || !std::isfinite(point.range_m) ||
        point.range_m < options.range_min || point.range_m > options.range_max)
      continue;
    double angle_deg = NormalizeDegrees(std::fmod(point.angle_rad, 2.0 * kPi) * 180.0 / kPi);
    // Preserve the configured CLOCKWISE environment-variable semantics.
    if (!options.clockwise)
    {
      angle_deg = NormalizeDegrees(360.0 - angle_deg);
    }
    double relative_deg = NormalizeDegrees(angle_deg - options.angle_min_deg);
    if (!full_circle && relative_deg > span_deg)
    {
      // SDK angles are float radians. Snap only rounding-sized excursions
      // outside the FOV; genuinely outside returns must remain excluded.
      constexpr double angular_precision_deg = 360.0 * std::numeric_limits<float>::epsilon();
      if (360.0 - relative_deg <= angular_precision_deg) relative_deg = 0.0;
      else if (relative_deg - span_deg <= angular_precision_deg) relative_deg = span_deg;
      else continue;
    }
    if (IsAngleMasked(angle_deg, options.mask_windows)) continue;
    int index = static_cast<int>(std::llround(relative_deg / increment_deg));
    if (full_circle) index %= target_count;
    else index = std::clamp(index, 0, target_count - 1);
    const double slot_angle = options.angle_min_deg + static_cast<double>(index) * increment_deg;
    if (IsAngleMasked(slot_angle, options.mask_windows)) continue;
    double error = std::abs(relative_deg - static_cast<double>(index) * increment_deg);
    if (full_circle) error = std::min(error, 360.0 - error);
    if (error >= best_error[static_cast<size_t>(index)])
    {
      continue;
    }
    best_error[static_cast<size_t>(index)] = error;
    ranges[static_cast<size_t>(index)] = point.range_m;
    intensities[static_cast<size_t>(index)] = std::isfinite(point.intensity) ? point.intensity : 0.0;
  }

  const double angle_increment = DegToRad(increment_deg);
  const double published_start = PublishedStartAngleDegrees(
    options.angle_min_deg, options.angle_max_deg);
  nlohmann::json msg;
  msg["header"] = HeaderJson(options.frame_id, options.seq, stamp);
  msg["angle_min"] = DegToRad(published_start);
  msg["angle_max"] = DegToRad(published_start) +
                     angle_increment * static_cast<double>(target_count - 1);
  msg["angle_increment"] = angle_increment;
  msg["scan_time"] = options.scan_time_s;
  // Rebinning and direction reversal lose individual beam acquisition times.
  msg["time_increment"] = 0.0;
  msg["range_min"] = options.range_min;
  msg["range_max"] = options.range_max;
  msg["ranges"] = ranges;
  msg["intensities"] = intensities;
  msg["point_count"] = target_count;
  msg["raw_point_count"] = points.size();
  msg["rotation_speed_hz"] = options.rotation_speed_hz;
  return msg;
}

} // namespace ydlidar_tmini_plus
