#pragma once

#include "karto_dora/types.hpp"

#include <cstddef>
#include <string>

namespace karto_dora {

struct SlamConfig {
  std::string mode{"new_mapping"};
  double minimum_travel_distance{0.05};
  double minimum_travel_heading{0.05};
  int scan_buffer_size{10};
  double scan_buffer_maximum_scan_distance{10.0};
  double correlation_search_space_dimension{0.5};
  double correlation_search_space_resolution{0.01};
  double correlation_search_space_smear_deviation{0.1};
  double loop_search_maximum_distance{1.2};
  double loop_search_space_dimension{2.4};
  double loop_search_space_resolution{0.05};
  double loop_search_space_smear_deviation{0.03};
  double loop_match_minimum_response_coarse{0.35};
  double loop_match_minimum_response_fine{0.45};
  int loop_match_minimum_chain_size{10};
  double distance_variance_penalty{0.5};
  double angle_variance_penalty{1.0};
  double minimum_angle_penalty{0.9};
  double minimum_distance_penalty{0.5};
  double max_laser_range{8.0};
  double min_laser_range{0.05};
  double max_sync_wait{0.2};
  double odom_buffer_duration{3.0};
  double map_resolution{0.05};
  double map_crop_margin{0.5};
  double localization_maximum_translation_correction{0.05};
  double localization_maximum_rotation_correction{0.13962634015954636};
  double localization_rotation_yaw_gain{1.0};
  double localization_translation_yaw_gain{0.70};
  double localization_maximum_yaw_fusion_correction{0.08726646259971647};
  double localization_minimum_response{0.02};
  std::size_t expected_laser_beams{450};
  Pose2d lidar_extrinsic{0.09, 0.06, 0.0};

  static SlamConfig ReferenceDefaults();
  static SlamConfig FromEnvironment();
  static SlamConfig FromEnvironment(SlamConfig seed);
  void Validate() const;
};

}  // namespace karto_dora
