#pragma once
#include "form/mapping/map.hpp"

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
[[nodiscard]] SearchResult<Point>
VoxelMap<Point>::find_closest(const Point &queryPoint) const noexcept {
  const Eigen::Matrix<int, 3, 1> coords = computeCoords(queryPoint);
  SearchResult<Point> result;

  for (const auto &shift : voxel_shifts) {
    const Eigen::Matrix<int, 3, 1> shifted_coords = coords + shift;
    auto voxel = m_data.find(shifted_coords);
    if (voxel != m_data.end()) {
      for (const auto &point : voxel->second) {
        const auto distance = (point.vec4() - queryPoint.vec4()).squaredNorm();
        if (distance < result.distanceSquared) {
          result.distanceSquared = distance;
          result.point = point;
        }
      }
    }
  }
  return result;
}

template <typename Point>
VoxelMap<Point>
VoxelMap<Point>::from_keypoint_map(const KeypointMap<Point> &keypoint_map,
                                   const gtsam::Values &values) noexcept {
  // Create a new map
  VoxelMap world_map(keypoint_map.m_params.voxelWidth);

  // Iterate over all the keypoints in the map
  for (const auto &[frame_index, keypoints] : keypoint_map.m_frame_keypoints) {
    // Get the pose of the frame
    const auto &world_T_frame = values.at<gtsam::Pose3>(X(frame_index));

    // Transform each keypoint into the world frame and add it to the map
    for (const auto &keypoint : keypoints) {
      world_map.push_back(keypoint.transform(world_T_frame));
    }
  }

  return world_map;
}

// ------------------------- Keypoint Map ------------------------- //
template <typename Point>
KeypointMap<Point>::KeypointMap(const Params &params) noexcept : m_params(params) {}

template <typename Point>
std::vector<Point> &KeypointMap<Point>::get(const FrameIndex &frame_j) noexcept {
  // Check if it already exists
  auto search = m_frame_keypoints.find(frame_j);
  if (search != m_frame_keypoints.end()) {
    return search.value();
  }
  // If not, make it and return it
  else {
    m_frame_keypoints.insert(std::make_pair(frame_j, std::vector<Point>()));
    return m_frame_keypoints.at(frame_j);
  }
}

template <typename Point>
void KeypointMap<Point>::remove(const FrameIndex &frame_j) noexcept {
  auto search = m_frame_keypoints.find(frame_j);
  if (search != m_frame_keypoints.end()) {
    m_frame_keypoints.erase(search);
  }
}

template <typename Point>
VoxelMap<Point>
KeypointMap<Point>::to_voxel_map(const gtsam::Values &values) const noexcept {
  return VoxelMap<Point>::from_keypoint_map(*this, values);
}

} // namespace form