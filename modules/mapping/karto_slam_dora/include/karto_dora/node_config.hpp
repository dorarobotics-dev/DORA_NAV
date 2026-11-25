#pragma once

#include "karto_dora/config.hpp"

#include <cstdlib>
#include <filesystem>
#include <set>
#include <yaml-cpp/yaml.h>

namespace karto_dora {

struct NodeSettings {
  SlamConfig slam;
  std::string map_prefix{"map"};
  std::string pose_log_path;

  NodeSettings() { slam.lidar_extrinsic = {}; }
};

inline NodeSettings LoadNodeSettings(const std::string &default_path)
{
  const char *configured_path = std::getenv("KARTO_CONFIG_FILE");
  if (!configured_path || !*configured_path) configured_path = std::getenv("CONFIG_FILE");
  const std::filesystem::path file = std::filesystem::absolute(
      configured_path && *configured_path ? configured_path : default_path).lexically_normal();
  const YAML::Node yaml = YAML::LoadFile(file.string());
  if (!yaml.IsMap()) throw std::invalid_argument("Karto configuration must be a YAML mapping");
  const std::set<std::string> keys = {"mode", "map_prefix", "pose_log_path",
    "minimum_travel_distance", "minimum_travel_heading", "scan_buffer_size",
    "scan_buffer_maximum_scan_distance", "max_laser_range", "min_laser_range",
    "map_resolution", "map_crop_margin", "max_sync_wait", "odom_buffer_duration",
    "expected_laser_beams", "lidar_x", "lidar_y", "lidar_yaw"};
  for (const auto &entry : yaml) {
    if (!entry.first.IsScalar() || !keys.count(entry.first.as<std::string>()))
      throw std::invalid_argument("unknown Karto YAML parameter: " + YAML::Dump(entry.first));
  }
  NodeSettings settings;
  auto &slam = settings.slam;
  if (yaml["mode"]) slam.mode = yaml["mode"].as<std::string>();
  if (yaml["map_prefix"]) settings.map_prefix = yaml["map_prefix"].as<std::string>();
  if (yaml["pose_log_path"]) settings.pose_log_path = yaml["pose_log_path"].as<std::string>();
  if (yaml["minimum_travel_distance"]) slam.minimum_travel_distance = yaml["minimum_travel_distance"].as<double>();
  if (yaml["minimum_travel_heading"]) slam.minimum_travel_heading = yaml["minimum_travel_heading"].as<double>();
  if (yaml["scan_buffer_size"]) slam.scan_buffer_size = yaml["scan_buffer_size"].as<int>();
  if (yaml["scan_buffer_maximum_scan_distance"]) slam.scan_buffer_maximum_scan_distance = yaml["scan_buffer_maximum_scan_distance"].as<double>();
  if (yaml["max_laser_range"]) slam.max_laser_range = yaml["max_laser_range"].as<double>();
  if (yaml["min_laser_range"]) slam.min_laser_range = yaml["min_laser_range"].as<double>();
  if (yaml["map_resolution"]) slam.map_resolution = yaml["map_resolution"].as<double>();
  if (yaml["map_crop_margin"]) slam.map_crop_margin = yaml["map_crop_margin"].as<double>();
  if (yaml["max_sync_wait"]) slam.max_sync_wait = yaml["max_sync_wait"].as<double>();
  if (yaml["odom_buffer_duration"]) slam.odom_buffer_duration = yaml["odom_buffer_duration"].as<double>();
  if (yaml["expected_laser_beams"]) {
    const int beams = yaml["expected_laser_beams"].as<int>();
    if (beams < 2) throw std::invalid_argument("expected laser beam count must be at least two");
    slam.expected_laser_beams = static_cast<std::size_t>(beams);
  }
  if (yaml["lidar_x"]) slam.lidar_extrinsic.x = yaml["lidar_x"].as<double>();
  if (yaml["lidar_y"]) slam.lidar_extrinsic.y = yaml["lidar_y"].as<double>();
  if (yaml["lidar_yaw"]) slam.lidar_extrinsic.yaw = yaml["lidar_yaw"].as<double>();
  slam.Validate();
  settings.slam = SlamConfig::FromEnvironment(slam);
  if (const char *value = std::getenv("MAP_PREFIX")) settings.map_prefix = value;
  if (const char *value = std::getenv("POSE_LOG_PATH")) settings.pose_log_path = value;
  if (settings.map_prefix.empty()) throw std::invalid_argument("map prefix must not be empty");
  const auto resolve = [&](const std::string &path) {
    if (path.empty()) return std::string{};
    const std::filesystem::path p(path);
    return (p.is_absolute() ? p : file.parent_path() / p).lexically_normal().string();
  };
  settings.map_prefix = resolve(settings.map_prefix);
  settings.pose_log_path = resolve(settings.pose_log_path);
  return settings;
}

}  // namespace karto_dora
