#include "karto_dora/karto_mapper.hpp"

#include "karto_dora/ceres_pose_graph_solver.hpp"
#include "karto_dora/localization_pose_fusion.hpp"

#include <karto_sdk/Karto.h>
#include <karto_sdk/Mapper.h>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/LU>

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace karto_dora {
namespace {

constexpr const char *kSensorName = "dora_lidar";

class LoopDiagnosticsListener : public karto::MapperLoopClosureListener {
 public:
  void LoopClosureCheck(const std::string &info) override
  {
    if (info.find("COARSE RESPONSE") != std::string::npos) ++candidate_checks;
    if (info.find("FINE RESPONSE") != std::string::npos) ++fine_checks;
    if (info.find("REJECTED") != std::string::npos) ++rejections;
  }

  std::size_t candidate_checks{0};
  std::size_t fine_checks{0};
  std::size_t rejections{0};
};

karto::Pose2 ToKartoPose(const Pose2d &pose)
{
  return {pose.x, pose.y, pose.yaw};
}

Pose2d FromKartoPose(const karto::Pose2 &pose)
{
  return {pose.GetX(), pose.GetY(), pose.GetHeading()};
}

bool RemoveLoopEdge(karto::Graph<karto::LocalizedRangeScan> *graph,
                    int source_id, int target_id)
{
  if (!graph) return false;
  const auto edges = graph->GetEdges();
  for (auto *edge : edges) {
    if (!edge || !edge->GetSource() || !edge->GetTarget()) continue;
    auto *link = dynamic_cast<karto::LinkInfo *>(edge->GetLabel());
    if (!link || link->GetConstraintSource() !=
                   karto::ConstraintSource::LoopClosure) continue;
    const int source = edge->GetSource()->GetObject()->GetUniqueId();
    const int target = edge->GetTarget()->GetObject()->GetUniqueId();
    if (source != source_id || target != target_id) continue;

    for (auto *vertex : {edge->GetSource(), edge->GetTarget()}) {
      const auto adjacent = vertex->GetEdges();
      const auto found = std::find(adjacent.begin(), adjacent.end(), edge);
      if (found != adjacent.end()) {
        vertex->RemoveEdge(static_cast<int>(found - adjacent.begin()));
      }
    }
    const auto graph_edges = graph->GetEdges();
    const auto found = std::find(graph_edges.begin(), graph_edges.end(), edge);
    if (found == graph_edges.end()) return false;
    graph->RemoveEdge(static_cast<int>(found - graph_edges.begin()));
    delete edge;
    return true;
  }
  return false;
}

}  // namespace

struct KartoMapper::Impl {
  explicit Impl(const SlamConfig &input_config) : config(input_config)
  {
    config.Validate();
    sensor.reset(karto::LaserRangeFinder::CreateLaserRangeFinder(
        karto::LaserRangeFinder_Custom, karto::Name(kSensorName)));
    sensor->SetMinimumRange(config.min_laser_range);
    sensor->SetMaximumRange(config.max_laser_range);
    sensor->SetRangeThreshold(config.max_laser_range);
    sensor->SetMinimumAngle(0.0);
    sensor->SetMaximumAngle(2.0 * karto::KT_PI);
    sensor->SetAngularResolution(karto::math::DegreesToRadians(1.0));
    sensor->SetOffsetPose(ToKartoPose(config.lidar_extrinsic));
    sensor->SetIs360Laser(true);
    karto::SensorManager::GetInstance()->RegisterSensor(sensor.get(), true);

    mapper.setParamUseScanMatching(true);
    mapper.setParamUseScanBarycenter(true);
    mapper.setParamMinimumTravelDistance(config.minimum_travel_distance);
    mapper.setParamMinimumTravelHeading(config.minimum_travel_heading);
    mapper.setParamScanBufferSize(config.scan_buffer_size);
    mapper.setParamScanBufferMaximumScanDistance(
        config.scan_buffer_maximum_scan_distance);
    mapper.setParamLinkMatchMinimumResponseFine(
        config.loop_match_minimum_response_fine);
    mapper.setParamLinkScanMaximumDistance(1.5);
    mapper.setParamLoopSearchMaximumDistance(config.loop_search_maximum_distance);
    mapper.setParamDoLoopClosing(config.mode != "localization");
    mapper.setParamLoopMatchMinimumChainSize(config.loop_match_minimum_chain_size);
    mapper.setParamLoopMatchMaximumVarianceCoarse(3.0);
    mapper.setParamLoopMatchMinimumResponseCoarse(config.loop_match_minimum_response_coarse);
    mapper.setParamLoopMatchMinimumResponseFine(config.loop_match_minimum_response_fine);
    mapper.setParamCorrelationSearchSpaceDimension(config.correlation_search_space_dimension);
    mapper.setParamCorrelationSearchSpaceResolution(config.correlation_search_space_resolution);
    mapper.setParamCorrelationSearchSpaceSmearDeviation(config.correlation_search_space_smear_deviation);
    mapper.setParamLoopSearchSpaceDimension(config.loop_search_space_dimension);
    mapper.setParamLoopSearchSpaceResolution(config.loop_search_space_resolution);
    mapper.setParamLoopSearchSpaceSmearDeviation(config.loop_search_space_smear_deviation);
    mapper.setParamDistanceVariancePenalty(config.distance_variance_penalty);
    mapper.setParamAngleVariancePenalty(config.angle_variance_penalty);
    mapper.setParamFineSearchAngleOffset(karto::math::DegreesToRadians(0.2));
    mapper.setParamCoarseSearchAngleOffset(karto::math::DegreesToRadians(20.0));
    mapper.setParamCoarseAngleResolution(karto::math::DegreesToRadians(2.0));
    mapper.setParamMinimumAnglePenalty(config.minimum_angle_penalty);
    mapper.setParamMinimumDistancePenalty(config.minimum_distance_penalty);
    mapper.setParamUseResponseExpansion(true);
    mapper.SetScanSolver(&solver);
    mapper.AddListener(&loop_diagnostics);
  }

  std::optional<Pose2d> last_localization_input;
  std::optional<Pose2d> last_localization_pose;
  std::optional<Pose2d> pending_initial_pose;

  ~Impl()
  {
    mapper.RemoveListener(&loop_diagnostics);
    if (!scans.empty()) {
      mapper.ClearLocalizationBuffer();
      mapper.Reset();
    }
    karto::SensorManager::GetInstance()->UnregisterSensor(sensor.get());
  }

  SlamConfig config;
  std::unique_ptr<karto::LaserRangeFinder> sensor;
  std::vector<std::unique_ptr<karto::LocalizedRangeScan>> scans;
  std::unordered_map<std::int64_t, std::vector<Point2d>> archive_points;
  CeresPoseGraphSolver solver;
  LoopDiagnosticsListener loop_diagnostics;
  std::size_t post_optimization_rejections{0};
  karto::Mapper mapper;
  std::size_t beam_count{0};
};

KartoMapper::KartoMapper(const SlamConfig &config) : impl_(std::make_unique<Impl>(config)) {}
KartoMapper::~KartoMapper() = default;

ProcessResult KartoMapper::Process(const PreparedScan &input)
{
  ProcessResult result;
  if (!input.ok() || input.base_points.size() < 3 || input.scan.ranges.empty()) {
    return result;
  }

  if (impl_->beam_count == 0) {
    impl_->beam_count = input.scan.ranges.size();
    impl_->sensor->SetMinimumRange(input.scan.range_min);
    impl_->sensor->SetMaximumRange(input.scan.range_max);
    impl_->sensor->SetRangeThreshold(std::min(input.scan.range_max,
                                              impl_->config.max_laser_range));
    impl_->sensor->SetMinimumAngle(input.scan.angle_min);
    // OpenKarto's 360-degree convention omits the usual inclusive-endpoint
    // residual, so its model span must cover N increments for N readings.
    impl_->sensor->SetMaximumAngle(KartoMaximumAngle(input.scan));
    impl_->sensor->SetAngularResolution(input.scan.angle_increment);
    impl_->sensor->SetIs360Laser(IsFullCircleScan(input.scan));
  } else if (input.scan.ranges.size() != impl_->beam_count) {
    return result;
  }

  karto::RangeReadingsVector ranges;
  ranges.reserve(input.scan.ranges.size());
  for (const double range : input.scan.ranges) ranges.push_back(range);

  auto owned_scan = std::make_unique<karto::LocalizedRangeScan>(
      karto::Name(kSensorName), ranges);
  owned_scan->SetOdometricPose(ToKartoPose(input.base_pose));
  owned_scan->SetCorrectedPose(ToKartoPose(input.base_pose));
  owned_scan->SetTime(input.stamp);

  const std::size_t edges_before = impl_->mapper.GetGraph()
      ? impl_->mapper.GetGraph()->GetEdges().size() : 0;
  karto::Matrix3 covariance;
  covariance.SetToIdentity();
  result.accepted = impl_->mapper.Process(owned_scan.get(), &covariance);
  if (!result.accepted) {
    return result;
  }

  for (const auto &rejected : impl_->solver.TakeRejectedLoopConstraints()) {
    if (!RemoveLoopEdge(impl_->mapper.GetGraph(), rejected.source,
                        rejected.target)) {
      throw std::runtime_error("optimizer rejected a loop missing from Karto graph " +
                               std::to_string(rejected.source) + "->" +
                               std::to_string(rejected.target));
    }
    ++impl_->post_optimization_rejections;
  }

  result.keyframe = true;
  result.corrected_pose = FromKartoPose(owned_scan->GetCorrectedPose());
  result.raw_corrected_pose = result.corrected_pose;
  result.covariance_xx = covariance(0, 0);
  result.covariance_yy = covariance(1, 1);
  result.covariance_yaw_yaw = covariance(2, 2);
  const auto *graph = impl_->mapper.GetGraph();
  result.constraint_count = graph ? graph->GetEdges().size() : 0;
  result.loop_closed =
      result.constraint_count > edges_before + (impl_->scans.empty() ? 0 : 1);
  const double uncertainty = std::max(0.0, result.covariance_xx) +
                             std::max(0.0, result.covariance_yy) +
                             std::max(0.0, result.covariance_yaw_yaw);
  result.match_response = impl_->scans.empty() ? 1.0 : std::exp(-uncertainty);
  impl_->archive_points.emplace(owned_scan->GetUniqueId(), input.base_points);
  impl_->scans.push_back(std::move(owned_scan));
  return result;
}

std::size_t KartoMapper::scan_count() const { return impl_->scans.size(); }

std::size_t KartoMapper::graph_node_count() const
{
  const auto *graph = impl_->mapper.GetGraph();
  if (!graph) return 0;
  std::size_t count = 0;
  for (const auto &sensor_vertices : graph->GetVertices()) {
    count += sensor_vertices.second.size();
  }
  return count;
}

GraphSnapshot KartoMapper::Snapshot() const
{
  GraphSnapshot snapshot;
  snapshot.loop_candidate_checks = impl_->loop_diagnostics.candidate_checks;
  snapshot.loop_fine_checks = impl_->loop_diagnostics.fine_checks;
  snapshot.loop_rejections = impl_->loop_diagnostics.rejections +
                             impl_->post_optimization_rejections;
  const auto *graph = impl_->mapper.GetGraph();
  if (!graph) {
    snapshot.solver = impl_->solver.LatestReport();
    return snapshot;
  }

  std::unordered_set<std::int64_t> node_ids;
  for (const auto &sensor_vertices : graph->GetVertices()) {
    for (const auto &state_vertex : sensor_vertices.second) {
      const auto *scan = state_vertex.second ? state_vertex.second->GetObject() : nullptr;
      if (!scan) continue;
      GraphNodeSnapshot node;
      node.id = scan->GetUniqueId();
      node.stamp = scan->GetTime();
      node.odometric_pose = FromKartoPose(scan->GetOdometricPose());
      node.optimized_pose = FromKartoPose(scan->GetCorrectedPose());
      const auto points = impl_->archive_points.find(node.id);
      if (points == impl_->archive_points.end()) {
        throw std::runtime_error("missing input points for Karto graph node " +
                                 std::to_string(node.id));
      }
      node.points = points->second;
      if (!node_ids.insert(node.id).second) {
        throw std::runtime_error("duplicate Karto graph node id " + std::to_string(node.id));
      }
      snapshot.nodes.push_back(std::move(node));
    }
  }
  std::sort(snapshot.nodes.begin(), snapshot.nodes.end(),
            [](const auto &a, const auto &b) { return a.id < b.id; });

  for (auto *edge : graph->GetEdges()) {
    if (!edge || !edge->GetSource() || !edge->GetTarget() || !edge->GetLabel()) {
      throw std::runtime_error("Karto graph contains incomplete edge");
    }
    const auto *source = edge->GetSource()->GetObject();
    const auto *target = edge->GetTarget()->GetObject();
    auto *link = dynamic_cast<karto::LinkInfo *>(edge->GetLabel());
    if (!source || !target || !link) throw std::runtime_error("Karto edge has invalid payload");

    GraphConstraintSnapshot constraint;
    constraint.source_id = source->GetUniqueId();
    constraint.target_id = target->GetUniqueId();
    if (!node_ids.count(constraint.source_id) || !node_ids.count(constraint.target_id)) {
      throw std::runtime_error("Karto edge references a missing node");
    }
    constraint.relative_pose = FromKartoPose(link->GetPoseDifference());
    Eigen::Matrix3d covariance;
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) covariance(row, col) = link->GetCovariance()(row, col);
    }
    if (!covariance.allFinite() || Eigen::LLT<Eigen::Matrix3d>(covariance).info() != Eigen::Success) {
      throw std::runtime_error("invalid covariance on Karto edge " +
                               std::to_string(constraint.source_id) + "->" +
                               std::to_string(constraint.target_id));
    }
    const Eigen::Matrix3d information = covariance.inverse();
    if (!information.allFinite()) throw std::runtime_error("non-finite Karto information matrix");
    constraint.information = {information(0, 0), information(0, 1), information(0, 2),
                              information(1, 1), information(1, 2), information(2, 2)};
    switch (link->GetConstraintSource()) {
      case karto::ConstraintSource::Sequential:
        constraint.kind = GraphConstraintKind::Sequential;
        ++snapshot.sequential_constraints;
        break;
      case karto::ConstraintSource::NearChain:
        constraint.kind = GraphConstraintKind::NearChain;
        ++snapshot.near_chain_constraints;
        break;
      case karto::ConstraintSource::LoopClosure:
        constraint.kind = GraphConstraintKind::LoopClosure;
        ++snapshot.loop_closures;
        break;
    }
    snapshot.constraints.push_back(std::move(constraint));
  }
  snapshot.solver = impl_->solver.LatestReport();
  return snapshot;
}

void KartoMapper::Restore(const PoseGraphArchive &archive)
{
  if (!impl_->scans.empty()) throw std::logic_error("mapper restore requires an empty mapper");
  const double minimum_travel_distance = impl_->mapper.getParamMinimumTravelDistance();
  const double minimum_travel_heading = impl_->mapper.getParamMinimumTravelHeading();
  const bool do_loop_closing = impl_->mapper.getParamDoLoopClosing();
  const auto restore_parameters = [&]() {
    impl_->mapper.setParamMinimumTravelDistance(minimum_travel_distance);
    impl_->mapper.setParamMinimumTravelHeading(minimum_travel_heading);
    impl_->mapper.setParamDoLoopClosing(do_loop_closing);
  };
  impl_->mapper.setParamMinimumTravelDistance(0.0);
  impl_->mapper.setParamMinimumTravelHeading(0.0);
  impl_->mapper.setParamDoLoopClosing(false);
  try {
  if (!std::isfinite(archive.laser.angle_increment) || archive.laser.angle_increment <= 0.0 ||
      archive.laser.angle_max <= archive.laser.angle_min) {
    throw std::runtime_error("archived laser geometry is invalid");
  }
  const auto archived_beam_count = static_cast<std::size_t>(std::llround(
      (archive.laser.angle_max - archive.laser.angle_min) /
      archive.laser.angle_increment)) + 1;
  if (archived_beam_count < 2) {
    throw std::runtime_error("archived laser beam count is invalid");
  }
  const bool fast_localization_restore = impl_->config.mode == "localization";
  if (fast_localization_restore) {
    impl_->beam_count = archived_beam_count;
    impl_->sensor->SetMinimumRange(archive.laser.range_min);
    impl_->sensor->SetMaximumRange(archive.laser.range_max);
    impl_->sensor->SetRangeThreshold(std::min(archive.laser.range_max,
                                              impl_->config.max_laser_range));
    impl_->sensor->SetMinimumAngle(archive.laser.angle_min);
    // The archive stores the inclusive last-beam angle. OpenKarto expects the
    // full N-increment span for a 360-degree scanner.
    const double model_angle_max = archive.laser.angle_min +
        static_cast<double>(archived_beam_count) * archive.laser.angle_increment;
    impl_->sensor->SetMaximumAngle(model_angle_max);
    impl_->sensor->SetAngularResolution(archive.laser.angle_increment);
    const double angular_span = static_cast<double>(archived_beam_count) *
        archive.laser.angle_increment;
    impl_->sensor->SetIs360Laser(
        std::abs(angular_span - 2.0 * karto::KT_PI) <=
        1.5 * archive.laser.angle_increment);
    impl_->mapper.Initialize(impl_->sensor->GetRangeThreshold());
  }
  for (std::size_t archive_index = 0; archive_index < archive.scans.size();
       ++archive_index) {
    const auto &stored = archive.scans[archive_index];
    PreparedScan scan;
    scan.stamp = stored.stamp;
    scan.base_pose = stored.odometric_pose;
    scan.base_points = stored.points;
    scan.scan.start_stamp = stored.stamp;
    scan.scan.angle_min = archive.laser.angle_min;
    scan.scan.angle_increment = archive.laser.angle_increment;
    scan.scan.angle_max = scan.scan.angle_min +
        static_cast<double>(archived_beam_count - 1) * scan.scan.angle_increment;
    scan.scan.range_min = archive.laser.range_min;
    scan.scan.range_max = archive.laser.range_max;
    scan.scan.ranges.assign(archived_beam_count, 0.0);
    const double cosine = std::cos(impl_->config.lidar_extrinsic.yaw);
    const double sine = std::sin(impl_->config.lidar_extrinsic.yaw);
    for (const auto &point : stored.points) {
      const double base_x = point.x - impl_->config.lidar_extrinsic.x;
      const double base_y = point.y - impl_->config.lidar_extrinsic.y;
      const double laser_x = cosine * base_x + sine * base_y;
      const double laser_y = -sine * base_x + cosine * base_y;
      const double range = std::hypot(laser_x, laser_y);
      double relative_angle = std::atan2(laser_y, laser_x) - scan.scan.angle_min;
      while (relative_angle < 0.0) relative_angle += 2.0 * karto::KT_PI;
      while (relative_angle >= 2.0 * karto::KT_PI) relative_angle -= 2.0 * karto::KT_PI;
      const auto index = static_cast<std::size_t>(
          std::llround(relative_angle / scan.scan.angle_increment));
      if (index < archived_beam_count) scan.scan.ranges[index] = range;
    }
    if (fast_localization_restore) {
      karto::RangeReadingsVector ranges;
      ranges.reserve(scan.scan.ranges.size());
      for (const double range : scan.scan.ranges) ranges.push_back(range);
      auto owned_scan = std::make_unique<karto::LocalizedRangeScan>(
          karto::Name(kSensorName), ranges);
      owned_scan->SetOdometricPose(ToKartoPose(stored.odometric_pose));
      owned_scan->SetCorrectedPose(ToKartoPose(stored.optimized_pose));
      owned_scan->SetTime(stored.stamp);

      // Localization needs archived scans in the sensor manager and spatial
      // graph, but it does not need scan matching, constraints, or loop
      // closure to be recomputed from an already optimized pose graph.
      auto *sensor_manager = impl_->mapper.GetMapperSensorManager();
      // GetLastScan also creates the per-sensor scan manager on first use,
      // matching the initialization performed by Mapper::Process().
      sensor_manager->GetLastScan(owned_scan->GetSensorName());
      sensor_manager->AddScan(owned_scan.get());
      impl_->mapper.GetGraph()->AddVertex(owned_scan.get());
      sensor_manager->AddRunningScan(owned_scan.get());
      sensor_manager->SetLastScan(owned_scan.get());
      impl_->archive_points.emplace(owned_scan->GetUniqueId(), stored.points);
      impl_->scans.push_back(std::move(owned_scan));
      continue;
    }

    const auto result = Process(scan);
    if (!result.accepted) {
      throw std::runtime_error(
          "failed to restore archived Karto scan at index " +
          std::to_string(archive_index) + " id " + std::to_string(stored.id));
    }
    impl_->scans.back()->SetOdometricPose(ToKartoPose(stored.odometric_pose));
    impl_->scans.back()->SetCorrectedPose(ToKartoPose(stored.optimized_pose));
    if (impl_->config.mode == "continue_mapping" &&
        static_cast<std::int64_t>(impl_->scans.back()->GetUniqueId()) != stored.id) {
      throw std::runtime_error("continue mapping requires archived scan ids in their original contiguous order");
    }
  }
  if (impl_->config.mode == "continue_mapping" && !archive.scans.empty()) {
    auto *graph = impl_->mapper.GetGraph();
    std::unordered_map<std::int64_t, karto::Vertex<karto::LocalizedRangeScan> *> vertices;
    for (const auto &sensor_vertices : graph->GetVertices()) {
      for (const auto &entry : sensor_vertices.second) {
        auto *vertex = entry.second;
        vertices.emplace(vertex->GetObject()->GetUniqueId(), vertex);
      }
    }
    struct RestoredConstraint {
      karto::Vertex<karto::LocalizedRangeScan> *source;
      karto::Vertex<karto::LocalizedRangeScan> *target;
      karto::Pose2 measured_target;
      karto::Matrix3 world_covariance;
      karto::ConstraintSource category;
    };
    std::vector<RestoredConstraint> restored_constraints;
    restored_constraints.reserve(archive.constraints.size());
    for (const auto &constraint : archive.constraints) {
      const auto source = vertices.find(constraint.source_id);
      const auto target = vertices.find(constraint.target_id);
      if (source == vertices.end() || target == vertices.end()) {
        throw std::runtime_error("archived constraint references a missing scan");
      }
      if (!std::isfinite(constraint.relative_pose.x) ||
          !std::isfinite(constraint.relative_pose.y) ||
          !std::isfinite(constraint.relative_pose.yaw)) {
        throw std::runtime_error("archived constraint measurement is not finite");
      }
      const auto &i = constraint.information;
      Eigen::Matrix3d information;
      information << i[0], i[1], i[2], i[1], i[3], i[4], i[2], i[4], i[5];
      const Eigen::LLT<Eigen::Matrix3d> llt(information);
      if (!information.allFinite() || llt.info() != Eigen::Success) {
        throw std::runtime_error("archived constraint information is not positive definite");
      }
      const Eigen::Matrix3d covariance = llt.solve(Eigen::Matrix3d::Identity());
      if (!covariance.allFinite()) throw std::runtime_error("invalid archived constraint covariance");
      karto::Matrix3 local_covariance;
      for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) local_covariance(row, col) = covariance(row, col);
      }
      const auto &source_pose = source->second->GetObject()->GetCorrectedPose();
      karto::Transform local_to_world(karto::Pose2(), source_pose);
      const karto::Pose2 measured_target = local_to_world.TransformPose(ToKartoPose(constraint.relative_pose));
      karto::Matrix3 rotation;
      rotation.FromAxisAngle(0, 0, 1, source_pose.GetHeading());
      karto::ConstraintSource category;
      switch (constraint.category) {
        case ConstraintCategory::Sequential: category = karto::ConstraintSource::Sequential; break;
        case ConstraintCategory::NearChain: category = karto::ConstraintSource::NearChain; break;
        case ConstraintCategory::LoopClosure: category = karto::ConstraintSource::LoopClosure; break;
        default: throw std::runtime_error("invalid archived constraint category");
      }
      restored_constraints.push_back({source->second, target->second, measured_target,
          rotation * local_covariance * rotation.Transpose(), category});
    }

    // Replay rebuilt scan-manager indices and running scans, but its newly
    // matched edges are not the measurements stored in the historical graph.
    for (auto *edge : graph->GetEdges()) {
      impl_->solver.RemoveConstraint(edge->GetSource()->GetObject()->GetUniqueId(),
                                     edge->GetTarget()->GetObject()->GetUniqueId());
    }
    for (const auto &entry : vertices) {
      auto *vertex = entry.second;
      while (!vertex->GetEdges().empty()) vertex->RemoveEdge(0);
      impl_->solver.AddNode(vertex); // Restore the archived optimized pose too.
    }
    while (!graph->GetEdges().empty()) {
      auto *edge = graph->GetEdges().front();
      graph->RemoveEdge(0);
      delete edge;
    }
    impl_->solver.Clear(); // Only clears pending correction output, not nodes.
    for (const auto &constraint : restored_constraints) {
      auto edge = std::make_unique<karto::Edge<karto::LocalizedRangeScan>>(
          constraint.source, constraint.target);
      edge->SetLabel(new karto::LinkInfo(
          constraint.source->GetObject()->GetCorrectedPose(), constraint.measured_target,
          constraint.world_covariance, constraint.category));
      static_cast<karto::Graph<karto::LocalizedRangeScan> *>(graph)->AddEdge(edge.get());
      impl_->solver.AddConstraint(edge.release());
    }
  }
  } catch (...) {
    restore_parameters();
    throw;
  }
  restore_parameters();
}

ProcessResult KartoMapper::Localize(
    const PreparedScan &input, std::optional<Pose2d> predicted_map_pose,
    std::optional<LocalizationFusionRequest> fusion)
{
  ProcessResult result;
  if (!input.ok() || input.base_points.size()<3 || impl_->scans.empty()) return result;
  if (input.scan.ranges.size()!=impl_->beam_count) return result;
  if (impl_->last_localization_input) {
    const auto &last = *impl_->last_localization_input;
    const double distance = std::hypot(input.base_pose.x - last.x,
                                       input.base_pose.y - last.y);
    const double heading = std::abs(std::atan2(
        std::sin(input.base_pose.yaw - last.yaw),
        std::cos(input.base_pose.yaw - last.yaw)));
    if (distance < impl_->config.minimum_travel_distance &&
        heading < impl_->config.minimum_travel_heading) {
      result.skipped = true;
      return result;
    }
  }
  karto::RangeReadingsVector ranges;
  ranges.reserve(input.scan.ranges.size());
  for (const double range : input.scan.ranges) ranges.push_back(range);
  auto scan=std::make_unique<karto::LocalizedRangeScan>(karto::Name(kSensorName),ranges);
  const Pose2d seed_pose = impl_->pending_initial_pose.value_or(
      predicted_map_pose.value_or(input.base_pose));
  scan->SetOdometricPose(ToKartoPose(seed_pose)); scan->SetCorrectedPose(ToKartoPose(seed_pose));
  scan->SetTime(input.stamp);
  karto::Matrix3 covariance; covariance.SetToIdentity();
  kt_double native_response = 0.0;
  if (impl_->pending_initial_pose) {
    result.accepted = impl_->mapper.ProcessAgainstNodesNearBy(
        scan.get(), true, &covariance, &native_response);
    if (!result.accepted) return result;
    impl_->pending_initial_pose.reset();
    impl_->last_localization_input = input.base_pose;
    result.raw_corrected_pose=FromKartoPose(scan->GetCorrectedPose());
    result.corrected_pose = fusion
        ? FuseLocalizationPose(
              fusion->reference_pose, result.raw_corrected_pose,
              fusion->rotation_dominant, fusion->rotation_yaw_gain,
              fusion->translation_yaw_gain,
              fusion->maximum_yaw_correction)
        : result.raw_corrected_pose;
    scan->SetCorrectedPose(ToKartoPose(result.corrected_pose));
    impl_->last_localization_pose = FromKartoPose(scan->GetCorrectedPose());
    result.covariance_xx=covariance(0,0); result.covariance_yy=covariance(1,1);
    result.covariance_yaw_yaw=covariance(2,2);
    result.match_response=std::clamp(static_cast<double>(native_response), 0.0, 1.0);
    result.keyframe=false;
    scan.release();
    return result;
  }
  const bool first_localization_scan = !impl_->last_localization_input.has_value();
  const double mapping_distance=impl_->mapper.getParamMinimumTravelDistance();
  const double mapping_heading=impl_->mapper.getParamMinimumTravelHeading();
  if (first_localization_scan) {
    impl_->mapper.setParamMinimumTravelDistance(0.0);
    impl_->mapper.setParamMinimumTravelHeading(0.0);
  }
  result.accepted=impl_->mapper.ProcessLocalization(
      scan.get(), &covariance, &native_response);
  if (first_localization_scan) {
    impl_->mapper.setParamMinimumTravelDistance(mapping_distance);
    impl_->mapper.setParamMinimumTravelHeading(mapping_heading);
  }
  if (!result.accepted) return result;
  impl_->last_localization_input = input.base_pose;
  result.raw_corrected_pose=FromKartoPose(scan->GetCorrectedPose());
  result.corrected_pose = fusion
      ? FuseLocalizationPose(
            fusion->reference_pose, result.raw_corrected_pose,
            fusion->rotation_dominant, fusion->rotation_yaw_gain,
            fusion->translation_yaw_gain,
            fusion->maximum_yaw_correction)
      : result.raw_corrected_pose;
  scan->SetCorrectedPose(ToKartoPose(result.corrected_pose));
  impl_->last_localization_pose = FromKartoPose(scan->GetCorrectedPose());
  result.covariance_xx=covariance(0,0); result.covariance_yy=covariance(1,1); result.covariance_yaw_yaw=covariance(2,2);
  result.match_response=std::clamp(static_cast<double>(native_response), 0.0, 1.0);
  result.keyframe=false;
  scan.release();  // Mapper localization buffer owns accepted localization scans.
  return result;
}

void KartoMapper::SetInitialPose(const Pose2d &pose)
{
  if (!std::isfinite(pose.x) || !std::isfinite(pose.y) ||
      !std::isfinite(pose.yaw)) {
    throw std::invalid_argument("initial pose must be finite");
  }
  impl_->mapper.ClearLocalizationBuffer();
  impl_->last_localization_input.reset();
  impl_->last_localization_pose.reset();
  impl_->pending_initial_pose = pose;
}

std::optional<Pose2d> KartoMapper::last_localization_pose() const
{
  return impl_->last_localization_pose;
}

}  // namespace karto_dora
