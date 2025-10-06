#include "form/mapping/map.hpp"

namespace form {

// ------------------------- Voxel Map ------------------------- //
VoxelMap::VoxelMap(double voxel_width) noexcept : m_voxel_width((voxel_width)) {}

[[nodiscard]] Eigen::Matrix<int, 3, 1>
VoxelMap::computeCoords(const Keypoint_t &point) const noexcept {
  return (point.array() / m_voxel_width).floor().template cast<int>();
}

void VoxelMap::push_back(const Keypoint_t &point) noexcept {
  const auto coords = computeCoords(point);
  auto search = m_data.find(coords);
  if (search != m_data.end()) {
    search.value().push_back(point);
  } else {
    std::vector<Keypoint_t> vec;
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

[[nodiscard]] SearchResult<Keypoint_t>
VoxelMap::find_closest(const Keypoint_t &queryPoint) const noexcept {
  const Eigen::Matrix<int, 3, 1> coords = computeCoords(queryPoint);
  SearchResult<Keypoint_t> result;

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

VoxelMap VoxelMap::from_keypoint_map(const KeypointMap &keypoint_map,
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

// ------------------------- Voxel Map Double ------------------------- //
VoxelMapDouble::VoxelMapDouble(double voxel_width) noexcept
    : m_data_1(voxel_width), m_data_2(voxel_width), m_voxel_width(voxel_width) {}

void VoxelMapDouble::push_back(const Keypoint_t &point) {
  // Requires the point to have a method to return an integer to distinguish types
  if (point.type() == 0) {
    m_data_1.push_back(point);
  } else if (point.type() == 1) {
    m_data_2.push_back(point);
  } else {
    throw std::runtime_error("Unknown point type in VoxelMapDouble");
  }
}

[[nodiscard]] SearchResult<Keypoint_t>
VoxelMapDouble::find_closest(const Keypoint_t &queryPoint) const {
  if (queryPoint.type() == 0) {
    return m_data_1.find_closest(queryPoint);
  } else if (queryPoint.type() == 1) {
    return m_data_2.find_closest(queryPoint);
  } else {
    throw std::runtime_error("Unknown point type in VoxelMapDouble");
  }
}

VoxelMapDouble VoxelMapDouble::from_keypoint_map(const KeypointMap &keypoint_map,
                                                 const gtsam::Values &values) {
  // Create a new map
  VoxelMapDouble world_map(keypoint_map.m_params.voxelWidth);

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
KeypointMap::KeypointMap(const Params &params) noexcept : m_params(params) {}

std::vector<Keypoint_t> &KeypointMap::get(const FrameIndex &frame_j) noexcept {
  // Check if it already exists
  auto search = m_frame_keypoints.find(frame_j);
  if (search != m_frame_keypoints.end()) {
    return search.value();
  }
  // If not, make it and return it
  else {
    m_frame_keypoints.insert(std::make_pair(frame_j, std::vector<Keypoint_t>()));
    return m_frame_keypoints.at(frame_j);
  }
}

void KeypointMap::remove(const FrameIndex &frame_j) noexcept {
  auto search = m_frame_keypoints.find(frame_j);
  if (search != m_frame_keypoints.end()) {
    m_frame_keypoints.erase(search);
  }
}

} // namespace form