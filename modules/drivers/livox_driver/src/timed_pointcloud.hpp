// SPDX-License-Identifier: MIT
#ifndef LIVOX_DORA_TIMED_POINTCLOUD_HPP_
#define LIVOX_DORA_TIMED_POINTCLOUD_HPP_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>

namespace livox_ros {

inline bool ParseTimedPointCloudEnabled(const char* value) {
  if (!value || std::strcmp(value, "false") == 0 || std::strcmp(value, "0") == 0) {
    return false;
  }
  if (std::strcmp(value, "true") == 0 || std::strcmp(value, "1") == 0) {
    return true;
  }
  throw std::invalid_argument(
      "LIVOX_PUBLISH_TIMED_POINTCLOUD must be true, false, 1 or 0");
}

namespace timed_pointcloud_detail {
inline void AppendLittleEndian(std::vector<char>& out, uint64_t value, unsigned bytes) {
  for (unsigned i = 0; i < bytes; ++i) {
    out.push_back(static_cast<char>((value >> (8 * i)) & 0xff));
  }
}

inline void AppendFloat(std::vector<char>& out, float value) {
  static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
                "timed point clouds require IEEE-754 float32");
  uint32_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  AppendLittleEndian(out, bits, 4);
}
}  // namespace timed_pointcloud_detail

// DLIOPC01 (all numeric fields little-endian, no native struct padding):
// header: magic[8], frame_start_ns:u64, count:u32, flags:u32=0 (24 bytes)
// point: x/y/z/intensity:f32, relative_time_ns:u32, line:u16, tag:u8,
//        reserved:u8=0 (24 bytes).
// StoragePacket::points[*].offset_time is an absolute hardware timestamp in ns.
// Subtract the frame base before narrowing, retaining the real firing times.
template <typename Packet>
std::vector<char> SerializeTimedPointCloud(const Packet& packet) {
  using timed_pointcloud_detail::AppendFloat;
  using timed_pointcloud_detail::AppendLittleEndian;
  constexpr std::size_t kHeaderSize = 24;
  constexpr std::size_t kPointSize = 24;
  if (packet.points.empty() || packet.points.size() != packet.points_num ||
      packet.points.size() > std::numeric_limits<uint32_t>::max() ||
      packet.points.size() > (std::numeric_limits<std::size_t>::max() - kHeaderSize) / kPointSize) {
    throw std::invalid_argument("timed point cloud has an invalid point count");
  }
  std::vector<char> result;
  result.reserve(kHeaderSize + packet.points.size() * kPointSize);
  constexpr char kMagic[] = "DLIOPC01";
  for (unsigned i = 0; i < 8; ++i) result.push_back(kMagic[i]);
  AppendLittleEndian(result, packet.base_time, 8);
  AppendLittleEndian(result, packet.points_num, 4);
  AppendLittleEndian(result, 0, 4);
  for (const auto& point : packet.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z) || !std::isfinite(point.intensity) ||
        point.intensity < 0.0f || point.intensity > 255.0f) {
      throw std::invalid_argument("timed point cloud has invalid coordinates or reflectivity");
    }
    if (point.offset_time < packet.base_time ||
        point.offset_time - packet.base_time > std::numeric_limits<uint32_t>::max()) {
      throw std::invalid_argument("timed point cloud offset is outside uint32 nanoseconds");
    }
    AppendFloat(result, point.x);
    AppendFloat(result, point.y);
    AppendFloat(result, point.z);
    AppendFloat(result, point.intensity);
    AppendLittleEndian(result, point.offset_time - packet.base_time, 4);
    AppendLittleEndian(result, point.line, 2);
    AppendLittleEndian(result, point.tag, 1);
    AppendLittleEndian(result, 0, 1);
  }
  return result;
}

}  // namespace livox_ros

#endif  // LIVOX_DORA_TIMED_POINTCLOUD_HPP_
