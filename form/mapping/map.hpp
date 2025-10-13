// MIT License

// Copyright (c) 2025 Easton Potokar, Taylor Pool, and Michael Kaess

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <limits>

#include <tbb/concurrent_vector.h>
#include <tsl/robin_map.h>

using gtsam::symbol_shorthand::X;

/// @brief Hash function for Eigen::Matrix<int, 3, 1>
/// From kiss-icp:
/// https://github.com/PRBonn/kiss-icp/blob/main/cpp/kiss_icp/core/VoxelUtils.hpp#L45-L51
template <> struct std::hash<Eigen::Matrix<int, 3, 1>> {
  std::size_t operator()(const Eigen::Matrix<int, 3, 1> &voxel) const {
    const uint32_t *vec = reinterpret_cast<const uint32_t *>(voxel.data());
    return (vec[0] * 73856093 ^ vec[1] * 19349669 ^ vec[2] * 83492791);
  }
};

namespace form {

/// @brief A match between a query point and a point in the map
/// If no match is found, dist_sqrd is set to max double
template <typename Point> struct Match {
  /// @brief The query point
  Point query;
  /// @brief The matched point in the map
  Point point;

  /// @brief Squared distance between query and point
  double dist_sqrd = std::numeric_limits<double>::max();

  /// @brief Whether a match was found
  [[nodiscard]] constexpr bool found() const noexcept {
    return dist_sqrd != std::numeric_limits<double>::max();
  }
};

/// @brief A voxel grid map for fast nearest neighbor search
/// Points are stored in a hash map, where the key is the voxel coordinates
/// and the value is a vector of points in that voxel
template <typename Point> class VoxelMap {
private:
  /// @brief The width of each voxel
  double m_voxel_width = 0.5;

  /// @brief The data structure storing the points
  tsl::robin_map<Eigen::Matrix<int, 3, 1>, std::vector<Point>> m_data;

  /// @brief Compute the voxel coordinates for a given point
  [[nodiscard]] Eigen::Matrix<int, 3, 1>
  computeCoords(const Point &point) const noexcept;

public:
  /// @brief Construct a voxel map with the given voxel width
  VoxelMap(double voxel_width) noexcept;

  /// @brief Find the closest point in the map to the given query point
  [[nodiscard]] Match<Point> find_closest(const Point &queryPoint) const noexcept;

  /// @brief Insert a point into the map
  void push_back(const Point &point) noexcept;

  /// Iterators over the voxels
  [[nodiscard]] auto begin() const noexcept { return m_data.begin(); }
  [[nodiscard]] auto end() const noexcept { return m_data.end(); }

  /// @brief Get the number of voxels in the map
  [[nodiscard]] auto size() const noexcept { return m_data.size(); }
};

/// @brief Parameters for the KeypointMap
struct KeypointMapParams {
  /// @brief Minimum distance for a match to be added to the map
  double min_dist_map = 0.1;
};

/// @brief A map of keypoints for each scan
///
/// Stores a mapping from scan index to a vector of keypoints
/// All keypoints are stored in their local frames
/// When matching is needed, the keypoints can be transformed to the global
/// scan into a voxel map using the current pose estimates.
template <typename Point> class KeypointMap {
  using ScanIndex = size_t;

private:
  /// @brief Parameters
  KeypointMapParams m_params;

  /// @brief The data structure storing the keypoints for each scan
  tsl::robin_map<ScanIndex, std::vector<Point>> m_scan_keypoints;

public:
  /// @brief Default constructor
  KeypointMap() noexcept : m_params() {}

  /// @brief Constructor with parameters
  KeypointMap(const KeypointMapParams &params) noexcept;

  /// @brief Get a reference keypoints for a given scan, adding it in if it doesn't
  /// exist yet
  std::vector<Point> &get(const ScanIndex &scan_j) noexcept;

  /// @brief Remove the keypoints for a given scan, if they exist
  void remove(const ScanIndex &scan_j) noexcept;

  /// @brief Remove the keypoints for a list of scans, if they exist
  template <typename Iter> void remove(const Iter &iter) noexcept;

  /// @brief Transform all keypoints to the global scan and insert them into a
  /// voxel map
  VoxelMap<Point> to_voxel_map(const gtsam::Values &values,
                               double voxel_width) const noexcept;

  /// @brief Insert matches into the map, adding only those that are sufficiently
  /// far from existing points based on min_dist_map
  void insert_matches(const tbb::concurrent_vector<Match<Point>> &matches);
};

} // namespace form

#include "form/mapping/map.tpp"