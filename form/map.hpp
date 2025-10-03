#pragma once

#include <gtsam/geometry/Pose3.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <limits>

#include <tsl/robin_map.h>

#include "form/separate/feature.hpp"

using gtsam::symbol_shorthand::X;

template <> struct std::hash<Eigen::Matrix<int, 3, 1>> {
  std::size_t operator()(const Eigen::Matrix<int, 3, 1> &voxel) const {
    const uint32_t *vec = reinterpret_cast<const uint32_t *>(voxel.data());
    return (vec[0] * 73856093 ^ vec[1] * 19349669 ^ vec[2] * 83492791);
  }
};

namespace form {

// Result after searching for a point in a map
template <typename Point> struct SearchResult {
  Point point;
  // By default, nothing has been found
  typename Point::type_t distanceSquared =
      std::numeric_limits<typename Point::type_t>::max();

  [[nodiscard]] constexpr bool found() const noexcept {
    return distanceSquared != std::numeric_limits<typename Point::type_t>::max();
  }
};

// XYZ - Scan idx - Line idx
using Keypoint_t = separate::PointXYZNTS<double>;

// forward declaration
class KeypointMap;

class VoxelMap {
private:
  typename Keypoint_t::type_t m_voxel_width = 0.5;
  tsl::robin_map<Eigen::Matrix<int, 3, 1>, std::vector<Keypoint_t>> m_data;

  [[nodiscard]] Eigen::Matrix<int, 3, 1>
  computeCoords(const Keypoint_t &point) const noexcept;

public:
  VoxelMap(double voxel_width) noexcept;

  [[nodiscard]] SearchResult<Keypoint_t>
  find_closest(const Keypoint_t &queryPoint) const noexcept;

  void push_back(const Keypoint_t &point) noexcept;

  static VoxelMap from_keypoint_map(const KeypointMap &keypoint_map,
                                    const gtsam::Values &values) noexcept;

  [[nodiscard]] auto cbegin() const noexcept { return m_data.cbegin(); }
  [[nodiscard]] auto cend() const noexcept { return m_data.cend(); }

  [[nodiscard]] auto begin() noexcept { return m_data.begin(); }
  [[nodiscard]] auto end() noexcept { return m_data.end(); }
  [[nodiscard]] auto begin() const noexcept { return m_data.begin(); }
  [[nodiscard]] auto end() const noexcept { return m_data.end(); }
  [[nodiscard]] auto size() const noexcept { return m_data.size(); }
};

class VoxelMapDouble {
private:
  typename Keypoint_t::type_t m_voxel_width = 0.5;

public:
  VoxelMap m_data_1;
  VoxelMap m_data_2;
  VoxelMapDouble(double voxel_width) noexcept;

  [[nodiscard]] SearchResult<Keypoint_t>
  find_closest(const Keypoint_t &queryPoint) const;

  void push_back(const Keypoint_t &point);

  static VoxelMapDouble from_keypoint_map(const KeypointMap &keypoint_map,
                                          const gtsam::Values &values);

  [[nodiscard]] auto size() const noexcept {
    return m_data_1.size() + m_data_2.size();
  }
};

class KeypointMap {
  using FrameIndex = size_t;

public:
  struct Params {
    typename Keypoint_t::type_t voxelWidth = 0.5;
  };

public:
  Params m_params;
  tsl::robin_map<FrameIndex, std::vector<Keypoint_t>> m_frame_keypoints;

public:
  KeypointMap(const Params &params) noexcept;

  std::vector<Keypoint_t> &get(const FrameIndex &frame_j) noexcept;

  void remove(const FrameIndex &frame_j) noexcept;

  template <typename Iter> void remove(const Iter &iter) noexcept {
    for (const auto &frame : iter) {
      remove(frame);
    }
  }

  const size_t get_size() const noexcept {
    size_t size = 0;
    for (const auto &[_, keypoints] : m_frame_keypoints) {
      size += keypoints.size();
    }
    return size;
  }
};

} // namespace form