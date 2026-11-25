#pragma once

#include "ydlidar_tmini_plus_config.hpp"

#include <set>
#include <yaml-cpp/yaml.h>

namespace ydlidar_tmini_plus
{

inline Config LoadConfigFile(const std::string &path)
{
  const YAML::Node yaml = YAML::LoadFile(path);
  if (!yaml.IsMap())
    throw std::invalid_argument("YDLIDAR configuration must be a YAML mapping: " + path);

  const std::set<std::string> keys = {
    "serial_port", "serial_baud", "frame_id", "output_id", "scan_frequency_hz",
    "target_point_count", "angle_min_deg", "angle_max_deg", "range_min", "range_max",
    "clockwise", "mask_angle_ranges_deg"};
  for (const auto &entry : yaml)
  {
    if (!entry.first.IsScalar() || !keys.count(entry.first.as<std::string>()))
      throw std::invalid_argument("unknown YDLIDAR YAML parameter in " + path + ": " +
                                  YAML::Dump(entry.first));
  }

  Config config;
  if (yaml["serial_port"]) config.serial_port = yaml["serial_port"].as<std::string>();
  if (yaml["serial_baud"]) config.serial_baud = yaml["serial_baud"].as<int>();
  if (yaml["frame_id"]) config.frame_id = yaml["frame_id"].as<std::string>();
  if (yaml["output_id"]) config.output_id = yaml["output_id"].as<std::string>();
  if (yaml["scan_frequency_hz"]) config.scan_frequency_hz = yaml["scan_frequency_hz"].as<double>();
  if (yaml["target_point_count"]) config.target_point_count = yaml["target_point_count"].as<int>();
  if (yaml["angle_min_deg"]) config.angle_min_deg = yaml["angle_min_deg"].as<double>();
  if (yaml["angle_max_deg"]) config.angle_max_deg = yaml["angle_max_deg"].as<double>();
  if (yaml["range_min"]) config.range_min = yaml["range_min"].as<double>();
  if (yaml["range_max"]) config.range_max = yaml["range_max"].as<double>();
  if (yaml["clockwise"]) config.clockwise = yaml["clockwise"].as<bool>();
  if (yaml["mask_angle_ranges_deg"])
    config.mask_angle_ranges_deg = yaml["mask_angle_ranges_deg"].as<std::string>();
  ValidateConfig(config);
  return config;
}

inline Config LoadDriverConfig(const std::string &default_path)
{
  const auto path = EnvString("YDLIDAR_TMINI_PLUS_CONFIG_FILE",
                              EnvString("CONFIG_FILE", default_path));
  return LoadConfigFromEnvironment(LoadConfigFile(path));
}

} // namespace ydlidar_tmini_plus
