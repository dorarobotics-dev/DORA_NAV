#pragma once

#include "karto_dora/types.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <stdexcept>

namespace karto_dora {

enum class LocalizationRecoveryAction {
  Accept,
  AcceptBounded,
  AcceptConfirmed,
  Pending,
  Reject,
};

struct LocalizationRecoveryDecision {
  LocalizationRecoveryAction action{LocalizationRecoveryAction::Reject};
  Pose2d pose;
  double translation{0.0};
  double rotation{0.0};
};

inline Pose2d SelectInnovationCandidate(const Pose2d &raw_laser_match,
                                        const Pose2d &fused_match,
                                        bool rotation_dominant) noexcept {
  if (!rotation_dominant) return raw_laser_match;
  Pose2d candidate = raw_laser_match;
  candidate.yaw = fused_match.yaw;
  return candidate;
}

class LocalizationInnovationRecovery {
 public:
  LocalizationInnovationRecovery(double nominal_translation,
                                 double nominal_rotation,
                                 double minimum_response)
      : nominal_translation_(nominal_translation),
        nominal_rotation_(nominal_rotation),
        minimum_response_(minimum_response) {
    if (!std::isfinite(nominal_translation_) || nominal_translation_ <= 0.0 ||
        !std::isfinite(nominal_rotation_) || nominal_rotation_ <= 0.0 ||
        !std::isfinite(minimum_response_) || minimum_response_ < 0.0 ||
        minimum_response_ > 1.0) {
      throw std::invalid_argument("localization recovery limits must be positive and finite");
    }
  }

  LocalizationRecoveryDecision Evaluate(const Pose2d &reference,
                                        const Pose2d &candidate,
                                        double match_response) noexcept {
    const double translation = Distance(reference, candidate);
    const double rotation = AngularDistance(reference.yaw, candidate.yaw);
    if (match_response < minimum_response_) {
      Reset();
      return {LocalizationRecoveryAction::Reject, reference, translation, rotation};
    }
    if (translation <= nominal_translation_ && rotation <= nominal_rotation_) {
      Reset();
      return {LocalizationRecoveryAction::Accept, candidate, translation, rotation};
    }

    constexpr double kCandidateConsistencyTranslation = 0.08;
    constexpr double kCandidateConsistencyRotation = 5.0 * M_PI / 180.0;
    if (!pending_candidate_ ||
        Distance(*pending_candidate_, candidate) > kCandidateConsistencyTranslation ||
        AngularDistance(pending_candidate_->yaw, candidate.yaw) >
            kCandidateConsistencyRotation) {
      pending_candidate_ = candidate;
      consistent_candidates_ = 1;
    } else {
      pending_candidate_ = AveragePose(*pending_candidate_, consistent_candidates_, candidate);
      ++consistent_candidates_;
    }

    constexpr std::size_t kRequiredConsistentCandidates = 3;
    if (consistent_candidates_ < kRequiredConsistentCandidates) {
      return {LocalizationRecoveryAction::Pending, reference, translation, rotation};
    }

    constexpr double kConfirmedMaximumTranslationStep = 0.15;
    const Pose2d confirmed = *pending_candidate_;
    Reset();
    return {LocalizationRecoveryAction::AcceptConfirmed,
            LimitCorrection(reference, confirmed,
                            kConfirmedMaximumTranslationStep,
                            nominal_rotation_),
            translation, rotation};
  }

  void Reset() noexcept {
    pending_candidate_.reset();
    consistent_candidates_ = 0;
  }

 private:
  static double NormalizeAngle(double angle) noexcept {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  static double AngularDistance(double lhs, double rhs) noexcept {
    return std::abs(NormalizeAngle(rhs - lhs));
  }

  static double Distance(const Pose2d &lhs, const Pose2d &rhs) noexcept {
    return std::hypot(rhs.x - lhs.x, rhs.y - lhs.y);
  }

  static Pose2d LimitCorrection(const Pose2d &reference,
                                const Pose2d &candidate,
                                double maximum_translation,
                                double maximum_rotation) noexcept {
    const double dx = candidate.x - reference.x;
    const double dy = candidate.y - reference.y;
    const double distance = std::hypot(dx, dy);
    const double scale = distance > maximum_translation
        ? maximum_translation / distance
        : 1.0;
    const double yaw_delta = NormalizeAngle(candidate.yaw - reference.yaw);
    const double bounded_yaw_delta =
        std::clamp(yaw_delta, -maximum_rotation, maximum_rotation);
    return {reference.x + dx * scale,
            reference.y + dy * scale,
            NormalizeAngle(reference.yaw + bounded_yaw_delta)};
  }

  static Pose2d AveragePose(const Pose2d &average,
                            std::size_t sample_count,
                            const Pose2d &sample) noexcept {
    const double count = static_cast<double>(sample_count);
    const double yaw_delta = NormalizeAngle(sample.yaw - average.yaw);
    return {(average.x * count + sample.x) / (count + 1.0),
            (average.y * count + sample.y) / (count + 1.0),
            NormalizeAngle(average.yaw + yaw_delta / (count + 1.0))};
  }

  double nominal_translation_;
  double nominal_rotation_;
  double minimum_response_;
  std::optional<Pose2d> pending_candidate_;
  std::size_t consistent_candidates_{0};
};

}  // namespace karto_dora
