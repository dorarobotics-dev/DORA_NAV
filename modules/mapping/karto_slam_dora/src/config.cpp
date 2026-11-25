#include "karto_dora/config.hpp"

#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <string>

namespace karto_dora {
namespace {

double EnvDouble(const char *name, double fallback)
{
  const char *raw = std::getenv(name);
  return raw && *raw ? std::stod(raw) : fallback;
}

int EnvInt(const char *name, int fallback)
{
  const char *raw = std::getenv(name);
  return raw && *raw ? std::stoi(raw) : fallback;
}

std::string EnvString(const char *name, const std::string &fallback)
{
  const char *raw = std::getenv(name);
  return raw && *raw ? raw : fallback;
}

}  // namespace

SlamConfig SlamConfig::ReferenceDefaults()
{
  return SlamConfig{};
}

SlamConfig SlamConfig::FromEnvironment()
{
  return FromEnvironment(ReferenceDefaults());
}

SlamConfig SlamConfig::FromEnvironment(SlamConfig c)
{
  c.mode = EnvString("SLAM_MODE", c.mode);
  c.minimum_travel_distance = EnvDouble("MINIMUM_TRAVEL_DISTANCE", c.minimum_travel_distance);
  c.minimum_travel_heading = EnvDouble("MINIMUM_TRAVEL_HEADING", c.minimum_travel_heading);
  c.scan_buffer_size = EnvInt("SCAN_BUFFER_SIZE", c.scan_buffer_size);
  c.scan_buffer_maximum_scan_distance = EnvDouble(
      "SCAN_BUFFER_MAXIMUM_SCAN_DISTANCE", c.scan_buffer_maximum_scan_distance);
  c.correlation_search_space_dimension = EnvDouble("CORRELATION_SEARCH_SPACE_DIMENSION", c.correlation_search_space_dimension);
  c.correlation_search_space_resolution = EnvDouble("CORRELATION_SEARCH_SPACE_RESOLUTION", c.correlation_search_space_resolution);
  c.correlation_search_space_smear_deviation = EnvDouble("CORRELATION_SEARCH_SPACE_SMEAR_DEVIATION", c.correlation_search_space_smear_deviation);
  c.loop_search_maximum_distance = EnvDouble("LOOP_SEARCH_MAXIMUM_DISTANCE", c.loop_search_maximum_distance);
  c.loop_search_space_dimension = EnvDouble("LOOP_SEARCH_SPACE_DIMENSION", c.loop_search_space_dimension);
  c.loop_search_space_resolution = EnvDouble("LOOP_SEARCH_SPACE_RESOLUTION", c.loop_search_space_resolution);
  c.loop_search_space_smear_deviation = EnvDouble("LOOP_SEARCH_SPACE_SMEAR_DEVIATION", c.loop_search_space_smear_deviation);
  c.loop_match_minimum_response_coarse = EnvDouble("LOOP_MATCH_MINIMUM_RESPONSE_COARSE", c.loop_match_minimum_response_coarse);
  c.loop_match_minimum_response_fine = EnvDouble("LOOP_MATCH_MINIMUM_RESPONSE_FINE", c.loop_match_minimum_response_fine);
  c.loop_match_minimum_chain_size = EnvInt("LOOP_MATCH_MINIMUM_CHAIN_SIZE", c.loop_match_minimum_chain_size);
  c.distance_variance_penalty = EnvDouble("DISTANCE_VARIANCE_PENALTY", c.distance_variance_penalty);
  c.angle_variance_penalty = EnvDouble("ANGLE_VARIANCE_PENALTY", c.angle_variance_penalty);
  c.minimum_angle_penalty = EnvDouble("MINIMUM_ANGLE_PENALTY", c.minimum_angle_penalty);
  c.minimum_distance_penalty = EnvDouble("MINIMUM_DISTANCE_PENALTY", c.minimum_distance_penalty);
  c.max_laser_range = EnvDouble("MAX_LASER_RANGE", c.max_laser_range);
  c.min_laser_range = EnvDouble("MIN_LASER_RANGE", c.min_laser_range);
  c.max_sync_wait = EnvDouble("MAX_SYNC_WAIT", c.max_sync_wait);
  c.odom_buffer_duration = EnvDouble("ODOM_BUFFER_DURATION", c.odom_buffer_duration);
  c.map_resolution = EnvDouble("MAP_RESOLUTION", c.map_resolution);
  c.map_crop_margin = EnvDouble("MAP_CROP_MARGIN", c.map_crop_margin);
  c.localization_maximum_translation_correction = EnvDouble(
      "LOCALIZATION_MAXIMUM_TRANSLATION_CORRECTION",
      c.localization_maximum_translation_correction);
  c.localization_maximum_rotation_correction = EnvDouble(
      "LOCALIZATION_MAXIMUM_ROTATION_CORRECTION",
      c.localization_maximum_rotation_correction);
  c.localization_rotation_yaw_gain = EnvDouble(
      "LOCALIZATION_ROTATION_YAW_GAIN", c.localization_rotation_yaw_gain);
  c.localization_translation_yaw_gain = EnvDouble(
      "LOCALIZATION_TRANSLATION_YAW_GAIN", c.localization_translation_yaw_gain);
  c.localization_maximum_yaw_fusion_correction = EnvDouble(
      "LOCALIZATION_MAXIMUM_YAW_FUSION_CORRECTION",
      c.localization_maximum_yaw_fusion_correction);
  c.localization_minimum_response = EnvDouble(
      "LOCALIZATION_MINIMUM_RESPONSE", c.localization_minimum_response);
  const int expected_beams = EnvInt("EXPECTED_LASER_BEAMS", static_cast<int>(c.expected_laser_beams));
  if (expected_beams < 2) throw std::invalid_argument("expected laser beam count must be at least two");
  c.expected_laser_beams = static_cast<std::size_t>(expected_beams);
  c.lidar_extrinsic.x = EnvDouble("LIDAR_X", c.lidar_extrinsic.x);
  c.lidar_extrinsic.y = EnvDouble("LIDAR_Y", c.lidar_extrinsic.y);
  c.lidar_extrinsic.yaw = EnvDouble("LIDAR_YAW", c.lidar_extrinsic.yaw);
  c.Validate();
  return c;
}

void SlamConfig::Validate() const
{
  if (mode != "new_mapping" && mode != "continue_mapping" && mode != "localization") {
    throw std::invalid_argument("SLAM_MODE must be new_mapping, continue_mapping, or localization");
  }
  const double positive[] = {minimum_travel_distance, minimum_travel_heading,
    scan_buffer_maximum_scan_distance,
    correlation_search_space_dimension, correlation_search_space_resolution,
    loop_search_maximum_distance, loop_search_space_dimension,
    loop_search_space_resolution, max_laser_range, max_sync_wait,
    odom_buffer_duration, map_resolution,
    localization_maximum_translation_correction,
    localization_maximum_rotation_correction,
    localization_maximum_yaw_fusion_correction};
  for (double value : positive) {
    if (!std::isfinite(value) || value <= 0.0) throw std::invalid_argument("SLAM parameter must be positive and finite");
  }
  const double finite[] = {min_laser_range, map_crop_margin, lidar_extrinsic.x,
    lidar_extrinsic.y, lidar_extrinsic.yaw, correlation_search_space_smear_deviation,
    loop_search_space_smear_deviation, loop_match_minimum_response_coarse,
    loop_match_minimum_response_fine, distance_variance_penalty, angle_variance_penalty,
    minimum_angle_penalty, minimum_distance_penalty};
  for (double value : finite) {
    if (!std::isfinite(value)) throw std::invalid_argument("SLAM parameter must be finite");
  }
  if (map_crop_margin < 0.0) throw std::invalid_argument("map crop margin must be nonnegative");
  if (min_laser_range < 0.0 || min_laser_range >= max_laser_range) {
    throw std::invalid_argument("invalid laser range limits");
  }
  if (loop_match_minimum_chain_size < 1) throw std::invalid_argument("loop chain size must be positive");
  if (scan_buffer_size < 1) throw std::invalid_argument("scan buffer size must be positive");
  if (expected_laser_beams < 2) throw std::invalid_argument("expected laser beam count must be at least two");
  if (!std::isfinite(localization_rotation_yaw_gain) ||
      localization_rotation_yaw_gain < 0.0 ||
      localization_rotation_yaw_gain > 1.0 ||
      !std::isfinite(localization_translation_yaw_gain) ||
      localization_translation_yaw_gain < 0.0 ||
      localization_translation_yaw_gain > 1.0) {
    throw std::invalid_argument("localization yaw gains must be within [0, 1]");
  }
  if (!std::isfinite(localization_minimum_response) ||
      localization_minimum_response < 0.0 ||
      localization_minimum_response > 1.0) {
    throw std::invalid_argument(
        "localization minimum response must be within [0, 1]");
  }
}

}  // namespace karto_dora
