#pragma once

#include "karto_dora/types.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>

namespace karto_dora {

struct LocalizationInnovation {
  double translation{0.0};
  double rotation{0.0};
  bool accepted{false};
};

struct LocalizationInnovationLimits {
  double maximum_translation{0.0};
  double maximum_rotation{0.0};
};

inline LocalizationInnovationLimits SelectInnovationLimits(
    bool explicit_relocalization,
    double continuous_maximum_translation,
    double continuous_maximum_rotation) noexcept {
  if (!explicit_relocalization) {
    return {continuous_maximum_translation, continuous_maximum_rotation};
  }
  constexpr double kRelocalizationMaximumTranslation = 0.5;
  constexpr double kRelocalizationMaximumRotation = 30.0 * M_PI / 180.0;
  return {
      std::max(continuous_maximum_translation,
               kRelocalizationMaximumTranslation),
      std::max(continuous_maximum_rotation,
               kRelocalizationMaximumRotation),
  };
}

inline std::optional<Pose2d> SelectLocalizationReference(
    const std::optional<Pose2d> &predicted,
    const std::optional<Pose2d> &initial_seed) noexcept {
  return predicted ? predicted : initial_seed;
}

class LocalizationInnovationGate {
 public:
  LocalizationInnovationGate(double maximum_translation,
                             double maximum_rotation)
      : maximum_translation_(maximum_translation),
        maximum_rotation_(maximum_rotation) {
    if (!std::isfinite(maximum_translation_) || maximum_translation_ <= 0.0 ||
        !std::isfinite(maximum_rotation_) || maximum_rotation_ <= 0.0) {
      throw std::invalid_argument(
          "localization innovation limits must be positive and finite");
    }
  }

  LocalizationInnovation Evaluate(const Pose2d &predicted,
                                  const Pose2d &matched) const noexcept {
    LocalizationInnovation innovation;
    innovation.translation =
        std::hypot(matched.x - predicted.x, matched.y - predicted.y);
    innovation.rotation = std::abs(std::atan2(
        std::sin(matched.yaw - predicted.yaw),
        std::cos(matched.yaw - predicted.yaw)));
    innovation.accepted = innovation.translation <= maximum_translation_ &&
                          innovation.rotation <= maximum_rotation_;
    return innovation;
  }

 private:
  double maximum_translation_;
  double maximum_rotation_;
};

}  // namespace karto_dora
