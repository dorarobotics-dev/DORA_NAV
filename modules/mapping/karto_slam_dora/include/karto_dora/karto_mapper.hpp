#pragma once

#include "karto_dora/config.hpp"
#include "karto_dora/archive.hpp"
#include "karto_dora/ceres_pose_graph_solver.hpp"
#include "karto_dora/scan_preparer.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace karto_dora {

struct ProcessResult {
  bool accepted{false};
  bool skipped{false};
  bool keyframe{false};
  bool loop_closed{false};
  Pose2d corrected_pose;
  Pose2d raw_corrected_pose;
  double match_response{0.0};
  double covariance_xx{0.0};
  double covariance_yy{0.0};
  double covariance_yaw_yaw{0.0};
  std::size_t constraint_count{0};
};

struct LocalizationFusionRequest {
  Pose2d reference_pose;
  bool rotation_dominant{false};
  double rotation_yaw_gain{0.0};
  double translation_yaw_gain{0.0};
  double maximum_yaw_correction{0.0};
};

enum class GraphConstraintKind { Sequential, NearChain, LoopClosure };

struct GraphNodeSnapshot {
  std::int64_t id{0};
  double stamp{0.0};
  Pose2d odometric_pose;
  Pose2d optimized_pose;
  std::vector<Point2d> points;
};

struct GraphConstraintSnapshot {
  std::int64_t source_id{0};
  std::int64_t target_id{0};
  Pose2d relative_pose;
  std::array<double, 6> information{};
  GraphConstraintKind kind{GraphConstraintKind::Sequential};
};

struct GraphSnapshot {
  std::vector<GraphNodeSnapshot> nodes;
  std::vector<GraphConstraintSnapshot> constraints;
  std::size_t sequential_constraints{0};
  std::size_t near_chain_constraints{0};
  std::size_t loop_closures{0};
  std::size_t loop_candidate_checks{0};
  std::size_t loop_fine_checks{0};
  std::size_t loop_rejections{0};
  OptimizationReport solver;
};

class KartoMapper {
 public:
  explicit KartoMapper(const SlamConfig &config);
  ~KartoMapper();

  KartoMapper(const KartoMapper &) = delete;
  KartoMapper &operator=(const KartoMapper &) = delete;

  ProcessResult Process(const PreparedScan &scan);
  ProcessResult Localize(const PreparedScan &scan,
                         std::optional<Pose2d> predicted_map_pose = std::nullopt,
                         std::optional<LocalizationFusionRequest> fusion = std::nullopt);
  void SetInitialPose(const Pose2d &pose);
  std::optional<Pose2d> last_localization_pose() const;
  void Restore(const PoseGraphArchive &archive);
  std::size_t scan_count() const;
  std::size_t graph_node_count() const;
  GraphSnapshot Snapshot() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace karto_dora
