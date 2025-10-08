#pragma once

#include <gtsam/geometry/Pose3.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <limits>

#include <tbb/concurrent_vector.h>
#include <tsl/robin_map.h>

using gtsam::symbol_shorthand::X;

template <> struct std::hash<Eigen::Matrix<int, 3, 1>> {
  std::size_t operator()(const Eigen::Matrix<int, 3, 1> &voxel) const {
    const uint32_t *vec = reinterpret_cast<const uint32_t *>(voxel.data());
    return (vec[0] * 73856093 ^ vec[1] * 19349669 ^ vec[2] * 83492791);
  }
};

namespace form {

// Result after searching for a point in a map
template <typename Point> struct Match {
  Point query;
  Point point;

  // By default, nothing has been found
  typename Point::type_t dist_sqrd =
      std::numeric_limits<typename Point::type_t>::max();

  [[nodiscard]] constexpr bool found() const noexcept {
    return dist_sqrd != std::numeric_limits<typename Point::type_t>::max();
  }
};

template <typename Point> class VoxelMap {
private:
  typename Point::type_t m_voxel_width = 0.5;
  tsl::robin_map<Eigen::Matrix<int, 3, 1>, std::vector<Point>> m_data;

  [[nodiscard]] Eigen::Matrix<int, 3, 1>
  computeCoords(const Point &point) const noexcept;

public:
  VoxelMap(double voxel_width) noexcept;

  [[nodiscard]] Match<Point> find_closest(const Point &queryPoint) const noexcept;

  void push_back(const Point &point) noexcept;

  [[nodiscard]] auto cbegin() const noexcept { return m_data.cbegin(); }
  [[nodiscard]] auto cend() const noexcept { return m_data.cend(); }

  [[nodiscard]] auto begin() noexcept { return m_data.begin(); }
  [[nodiscard]] auto end() noexcept { return m_data.end(); }
  [[nodiscard]] auto begin() const noexcept { return m_data.begin(); }
  [[nodiscard]] auto end() const noexcept { return m_data.end(); }
  [[nodiscard]] auto size() const noexcept { return m_data.size(); }
};

struct KeypointMapParams {
  double max_dist_map = 0.1;
};

template <typename Point> class KeypointMap {
  using FrameIndex = size_t;

private:
  KeypointMapParams m_params;
  tsl::robin_map<FrameIndex, std::vector<Point>> m_frame_keypoints;

public:
  KeypointMap() noexcept : m_params() {}

  KeypointMap(const KeypointMapParams &params) noexcept;

  std::vector<Point> &get(const FrameIndex &frame_j) noexcept;

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

  VoxelMap<Point> to_voxel_map(const gtsam::Values &values,
                               double voxel_width) const noexcept;

  void insert_matches(const tbb::concurrent_vector<Match<Point>> &matches);
};

} // namespace form

#include "form/mapping/map.tpp"