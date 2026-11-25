#pragma once

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <string>

namespace ydlidar_tmini_plus
{

struct Config
{
  std::string serial_port{"/dev/ttyUSB0"};
  int serial_baud{230400};
  std::string frame_id{"lidar"};
  std::string output_id{"LaserScan"};
  double angle_min_deg{0.0};
  double angle_max_deg{360.0};
  double range_min{0.05};
  double range_max{12.0};
  bool clockwise{false};
  std::string mask_angle_ranges_deg;
  int target_point_count{450};
  double scan_frequency_hz{10.0};
};

inline std::string EnvString(const char *name, const std::string &fallback)
{
  const char *raw = std::getenv(name);
  return raw == nullptr || *raw == '\0' ? fallback : std::string(raw);
}

inline int ParseInt(const char *name, int fallback)
{
  const char *raw = std::getenv(name);
  if (raw == nullptr || *raw == '\0')
  {
    return fallback;
  }
  errno = 0;
  char *end = nullptr;
  const long value = std::strtol(raw, &end, 10);
  if (errno != 0 || end == raw || *end != '\0' ||
      value < std::numeric_limits<int>::min() || value > std::numeric_limits<int>::max())
  {
    throw std::invalid_argument(std::string("invalid integer for ") + name + ": " + raw);
  }
  return static_cast<int>(value);
}

inline double ParseDouble(const char *name, double fallback)
{
  const char *raw = std::getenv(name);
  if (raw == nullptr || *raw == '\0')
  {
    return fallback;
  }
  errno = 0;
  char *end = nullptr;
  const double value = std::strtod(raw, &end);
  if (errno != 0 || end == raw || *end != '\0' || !std::isfinite(value))
  {
    throw std::invalid_argument(std::string("invalid number for ") + name + ": " + raw);
  }
  return value;
}

inline bool ParseBool(const char *name, bool fallback)
{
  const char *raw = std::getenv(name);
  if (raw == nullptr || *raw == '\0')
  {
    return fallback;
  }
  std::string value(raw);
  std::transform(value.begin(), value.end(), value.begin(),
    [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  if (value == "1" || value == "true" || value == "yes" || value == "on")
  {
    return true;
  }
  if (value == "0" || value == "false" || value == "no" || value == "off")
  {
    return false;
  }
  throw std::invalid_argument(std::string("invalid boolean for ") + name + ": " + raw);
}

inline void ValidateConfig(const Config &config)
{
  const double angle_span = config.angle_max_deg >= config.angle_min_deg
    ? config.angle_max_deg - config.angle_min_deg
    : 360.0 - config.angle_min_deg + config.angle_max_deg;
  if (config.serial_baud <= 0 || !std::isfinite(config.range_min) ||
      !std::isfinite(config.range_max) || config.range_min < 0.0 ||
      config.range_max <= config.range_min || config.target_point_count < 2 ||
      !std::isfinite(config.angle_min_deg) || !std::isfinite(config.angle_max_deg) ||
      config.angle_min_deg < 0.0 || config.angle_min_deg > 360.0 ||
      config.angle_max_deg < 0.0 || config.angle_max_deg > 360.0 ||
      angle_span <= 0.0 || angle_span > 360.0 ||
      !std::isfinite(config.scan_frequency_hz) || config.scan_frequency_hz < 5.0 ||
      config.scan_frequency_hz > 12.0)
  {
    throw std::invalid_argument("YDLIDAR T-mini Plus configuration values are outside valid bounds");
  }
}

inline Config LoadConfigFromEnvironment(Config config = {})
{
  config.serial_port = EnvString("SERIAL_PORT", config.serial_port);
  config.serial_baud = ParseInt("SERIAL_BAUD", config.serial_baud);
  config.frame_id = EnvString("FRAME_ID", config.frame_id);
  config.output_id = EnvString("OUTPUT_ID", config.output_id);
  config.angle_min_deg = ParseDouble("ANGLE_MIN_DEG", config.angle_min_deg);
  config.angle_max_deg = ParseDouble("ANGLE_MAX_DEG", config.angle_max_deg);
  config.range_min = ParseDouble("RANGE_MIN", config.range_min);
  config.range_max = ParseDouble("RANGE_MAX", config.range_max);
  config.clockwise = ParseBool("CLOCKWISE", config.clockwise);
  if (const char *mask = std::getenv("MASK_ANGLE_RANGES_DEG"))
    config.mask_angle_ranges_deg = mask;
  config.target_point_count = ParseInt("TARGET_POINT_COUNT", config.target_point_count);
  config.scan_frequency_hz = ParseDouble("SCAN_FREQUENCY_HZ", config.scan_frequency_hz);

  ValidateConfig(config);
  return config;
}

} // namespace ydlidar_tmini_plus
