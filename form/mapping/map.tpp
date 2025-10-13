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

#include "form/mapping/map.hpp"
#include <tbb/concurrent_vector.h>

namespace form {

// ------------------------- Voxel Map ------------------------- //
template <typename Point>
VoxelMap<Point>::VoxelMap(double voxel_width) noexcept
    : m_voxel_width((voxel_width)) {}

template <typename Point>
[[nodiscard]] Eigen::Matrix<int, 3, 1>
VoxelMap<Point>::computeCoords(const Point &point) const noexcept {
  return (point.vec3().array() / m_voxel_width).floor().template cast<int>();
}

template <typename Point>
void VoxelMap<Point>::push_back(const Point &point) noexcept {
  const auto coords = computeCoords(point);
  auto search = m_data.find(coords);
  if (search != m_data.end()) {
    search.value().push_back(point);
  } else {
    std::vector<Point> vec;
    vec.reserve(8);
    vec.push_back(point);
    m_data.insert({coords, std::move(vec)});
  }
}

static const std::array<Eigen::Matrix<int, 3, 1>, 27> voxel_shifts{
    {Eigen::Matrix<int, 3, 1>{0, 0, 0},   Eigen::Matrix<int, 3, 1>{1, 0, 0},
     Eigen::Matrix<int, 3, 1>{-1, 0, 0},  Eigen::Matrix<int, 3, 1>{0, 1, 0},
     Eigen::Matrix<int, 3, 1>{0, -1, 0},  Eigen::Matrix<int, 3, 1>{0, 0, 1},
     Eigen::Matrix<int, 3, 1>{0, 0, -1},  Eigen::Matrix<int, 3, 1>{1, 1, 0},
     Eigen::Matrix<int, 3, 1>{1, -1, 0},  Eigen::Matrix<int, 3, 1>{-1, 1, 0},
     Eigen::Matrix<int, 3, 1>{-1, -1, 0}, Eigen::Matrix<int, 3, 1>{1, 0, 1},
     Eigen::Matrix<int, 3, 1>{1, 0, -1},  Eigen::Matrix<int, 3, 1>{-1, 0, 1},
     Eigen::Matrix<int, 3, 1>{-1, 0, -1}, Eigen::Matrix<int, 3, 1>{0, 1, 1},
     Eigen::Matrix<int, 3, 1>{0, 1, -1},  Eigen::Matrix<int, 3, 1>{0, -1, 1},
     Eigen::Matrix<int, 3, 1>{0, -1, -1}, Eigen::Matrix<int, 3, 1>{1, 1, 1},
     Eigen::Matrix<int, 3, 1>{1, 1, -1},  Eigen::Matrix<int, 3, 1>{1, -1, 1},
     Eigen::Matrix<int, 3, 1>{1, -1, -1}, Eigen::Matrix<int, 3, 1>{-1, 1, 1},
     Eigen::Matrix<int, 3, 1>{-1, 1, -1}, Eigen::Matrix<int, 3, 1>{-1, -1, 1},
     Eigen::Matrix<int, 3, 1>{-1, -1, -1}}};

template <typename Point>
[[nodiscard]] Match<Point>
VoxelMap<Point>::find_closest(const Point &queryPoint) const noexcept {
  const Eigen::Matrix<int, 3, 1> coords = computeCoords(queryPoint);
  Match<Point> result;
  result.query = queryPoint;

  for (const auto &shift : voxel_shifts) {
    const Eigen::Matrix<int, 3, 1> shifted_coords = coords + shift;
    auto voxel = m_data.find(shifted_coords);
    if (voxel != m_data.end()) {
      for (const auto &point : voxel->second) {
        const auto dist_sqrd = (point.vec4() - queryPoint.vec4()).squaredNorm();
        if (dist_sqrd < result.dist_sqrd) {
          result.dist_sqrd = dist_sqrd;
          result.point = point;
        }
      }
    }
  }
  return result;
}

// ------------------------- Keypoint Map ------------------------- //
template <typename Point>
KeypointMap<Point>::KeypointMap(const KeypointMapParams &params) noexcept
    : m_params(params) {}

template <typename Point>
std::vector<Point> &KeypointMap<Point>::get(const ScanIndex &scan_j) noexcept {
  // Check if it already exists
  auto search = m_scan_keypoints.find(scan_j);
  if (search != m_scan_keypoints.end()) {
    return search.value();
  }
  // If not, make it and return it
  else {
    m_scan_keypoints.insert(std::make_pair(scan_j, std::vector<Point>()));
    return m_scan_keypoints.at(scan_j);
  }
}

template <typename Point>
void KeypointMap<Point>::remove(const ScanIndex &scan_j) noexcept {
  auto search = m_scan_keypoints.find(scan_j);
  if (search != m_scan_keypoints.end()) {
    m_scan_keypoints.erase(search);
  }
}

template <typename Point>
template <typename Iter>
void KeypointMap<Point>::remove(const Iter &iter) noexcept {
  for (const auto &scan : iter) {
    remove(scan);
  }
}

template <typename Point>
VoxelMap<Point> KeypointMap<Point>::to_voxel_map(const gtsam::Values &values,
                                                 double voxel_width) const noexcept {
  // Create a new map
  VoxelMap<Point> world_map(voxel_width);

  // Iterate over all the keypoints in the map
  for (const auto &[scan_index, keypoints] : m_scan_keypoints) {
    // Get the pose of the scan
    const auto &world_T_scan = values.at<gtsam::Pose3>(X(scan_index));

    // Transform each keypoint into the world scan and add it to the map
    for (const auto &keypoint : keypoints) {
      world_map.push_back(keypoint.transform(world_T_scan));
    }
  }

  return world_map;
}

template <typename Point>
void KeypointMap<Point>::insert_matches(
    const tbb::concurrent_vector<Match<Point>> &matches) {
  // Infer the scan
  if (matches.empty()) {
    return;
  }
  const ScanIndex scan_j = matches.front().query.scan;
  auto &keypoints = get(scan_j);

  double max_dist_map_sqrd = m_params.min_dist_map * m_params.min_dist_map;

  for (const auto &match : matches) {
    if (match.dist_sqrd > max_dist_map_sqrd) {
      keypoints.push_back(match.query);
    }
  }
}

} // namespace form