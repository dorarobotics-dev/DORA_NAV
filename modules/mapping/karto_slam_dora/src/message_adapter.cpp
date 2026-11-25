#include "karto_dora/message_adapter.hpp"

#include <cmath>
#include <nlohmann/json.hpp>
#include <string>

namespace karto_dora {
namespace {

using json = nlohmann::json;

double Finite(const json &value, const char *name)
{
  if (!value.is_number()) throw MessageError(std::string("missing numeric ") + name);
  const double result = value.get<double>();
  if (!std::isfinite(result)) throw MessageError(std::string("non-finite ") + name);
  return result;
}

double Stamp(const json &message)
{
  if (!message.contains("header") || !message.at("header").contains("stamp")) {
    throw MessageError("missing header.stamp");
  }
  const auto &stamp = message.at("header").at("stamp");
  const double sec = Finite(stamp.at("sec"), "header.stamp.sec");
  const double nanosec = Finite(stamp.at("nanosec"), "header.stamp.nanosec");
  if (nanosec < 0.0 || nanosec >= 1.0e9) throw MessageError("invalid nanosec");
  return sec + nanosec * 1.0e-9;
}

json Parse(std::string_view payload)
{
  try {
    return json::parse(payload.begin(), payload.end());
  } catch (const json::exception &error) {
    throw MessageError(std::string("invalid JSON: ") + error.what());
  }
}

}  // namespace

LaserScan ParseLaserScan(std::string_view payload)
{
  try {
    const json message = Parse(payload);
    LaserScan scan;
    scan.start_stamp = Stamp(message);
    scan.frame_id = message.at("header").value("frame_id", "lidar");
    scan.angle_min = Finite(message.at("angle_min"), "angle_min");
    scan.angle_max = Finite(message.at("angle_max"), "angle_max");
    scan.angle_increment = Finite(message.at("angle_increment"), "angle_increment");
    scan.scan_time = Finite(message.at("scan_time"), "scan_time");
    scan.time_increment = Finite(message.at("time_increment"), "time_increment");
    scan.range_min = Finite(message.at("range_min"), "range_min");
    scan.range_max = Finite(message.at("range_max"), "range_max");
    if (scan.scan_time < 0.0 || scan.time_increment < 0.0 ||
        scan.range_min < 0.0 || scan.range_max <= scan.range_min ||
        scan.angle_increment < 0.0) {
      throw MessageError("invalid LaserScan limits");
    }
    const auto &ranges = message.at("ranges");
    if (!ranges.is_array() || ranges.size() < 2) throw MessageError("ranges must contain at least two beams");
    scan.ranges.reserve(ranges.size());
    for (const auto &range : ranges) scan.ranges.push_back(Finite(range, "ranges[]"));
    if (message.contains("intensities")) {
      const auto &intensities = message.at("intensities");
      if (!intensities.is_array() || (!intensities.empty() && intensities.size() != ranges.size())) {
        throw MessageError("intensity/range count mismatch");
      }
      for (const auto &intensity : intensities) scan.intensities.push_back(Finite(intensity, "intensities[]"));
    }
    const double beam_span = scan.time_increment * static_cast<double>(scan.ranges.size());
    scan.end_stamp = scan.start_stamp + std::max(scan.scan_time, beam_span);
    if (!(scan.end_stamp >= scan.start_stamp)) throw MessageError("scan timestamp reversal");
    return scan;
  } catch (const MessageError &) {
    throw;
  } catch (const std::exception &error) {
    throw MessageError(std::string("invalid LaserScan: ") + error.what());
  }
}

TimedPose2d ParseOdometry(std::string_view payload)
{
  try {
    const json message = Parse(payload);
    TimedPose2d odom;
    const auto &header = message.at("header");
    odom.stamp = header.contains("timestamp")
      ? Finite(header.at("timestamp"), "header.timestamp") : Stamp(message);
    const auto &pose_container = message.at("pose");
    const auto &pose = pose_container.contains("pose") ? pose_container.at("pose") : pose_container;
    odom.pose.x = Finite(pose.at("position").at("x"), "pose.x");
    odom.pose.y = Finite(pose.at("position").at("y"), "pose.y");
    const auto &orientation = pose.at("orientation");
    const double x = orientation.contains("x") ? Finite(orientation.at("x"), "orientation.x") : 0.0;
    const double y = orientation.contains("y") ? Finite(orientation.at("y"), "orientation.y") : 0.0;
    const double z = Finite(orientation.at("z"), "orientation.z");
    const double w = Finite(orientation.at("w"), "orientation.w");
    const double norm = std::hypot(std::hypot(x, y), std::hypot(z, w));
    if (!std::isfinite(norm) || norm < 1.0e-9) throw MessageError("invalid quaternion");
    const double qx = x / norm, qy = y / norm, qz = z / norm, qw = w / norm;
    odom.pose.yaw = std::atan2(2.0 * (qw * qz + qx * qy),
                               1.0 - 2.0 * (qy * qy + qz * qz));
    const auto &twist_container = message.at("twist");
    const auto &twist = twist_container.contains("twist") ? twist_container.at("twist") : twist_container;
    odom.vx = Finite(twist.at("linear").at("x"), "twist.linear.x");
    odom.vy = Finite(twist.at("linear").at("y"), "twist.linear.y");
    odom.wz = Finite(twist.at("angular").at("z"), "twist.angular.z");
    return odom;
  } catch (const MessageError &) {
    throw;
  } catch (const std::exception &error) {
    throw MessageError(std::string("invalid Odometry: ") + error.what());
  }
}

Pose2d ParseInitialPose(std::string_view payload)
{
  try {
    const json message = Parse(payload);
    Pose2d pose;
    pose.x = Finite(message.at("x"), "x");
    pose.y = Finite(message.at("y"), "y");
    pose.yaw = Finite(message.at("yaw"), "yaw");
    pose.yaw = std::atan2(std::sin(pose.yaw), std::cos(pose.yaw));
    return pose;
  } catch (const MessageError &) {
    throw;
  } catch (const std::exception &error) {
    throw MessageError(std::string("invalid InitialPose: ") + error.what());
  }
}

std::string ToDoraPoseMessage(std::string_view corrected_pose_payload)
{
  try {
    json message = Parse(corrected_pose_payload);
    auto &header = message.at("header");
    double timestamp;
    if (header.contains("stamp")) {
      timestamp = header.at("stamp").is_object()
        ? Stamp(message) : Finite(header.at("stamp"), "header.stamp");
    } else if (header.contains("timestamp")) {
      timestamp = Finite(header.at("timestamp"), "header.timestamp");
    } else {
      throw MessageError("missing corrected pose timestamp");
    }
    const auto &pose = message.at("pose");
    const double x = Finite(pose.at("x"), "pose.x");
    const double y = Finite(pose.at("y"), "pose.y");
    const double raw_yaw = Finite(pose.at("yaw"), "pose.yaw");
    const double yaw = std::atan2(std::sin(raw_yaw), std::cos(raw_yaw));
    header.erase("stamp");
    header["timestamp"] = timestamp;
    message["pose"] = {
      {"position", {{"x", x}, {"y", y}, {"z", 0.0}}},
      {"orientation", {{"x", 0.0}, {"y", 0.0},
                       {"z", std::sin(yaw / 2.0)}, {"w", std::cos(yaw / 2.0)}}}};
    return message.dump();
  } catch (const MessageError &) {
    throw;
  } catch (const std::exception &error) {
    throw MessageError(std::string("invalid CorrectedPose: ") + error.what());
  }
}

}  // namespace karto_dora
