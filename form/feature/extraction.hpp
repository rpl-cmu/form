#pragma once

#include "form/feature/type.hpp"
#include "form/utils.hpp"
#include <Eigen/Dense>

#include <Eigen/src/Core/Matrix.h>
#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/parallel_for.h>

#include <algorithm>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <optional>

namespace form {

struct KeypointExtractionParams {
  // Parameters for keypoint extraction
  size_t neighbor_points = 5;
  size_t num_sectors = 6;
  double planar_threshold = 1.0;
  size_t planar_feats_per_sector = 50;
  size_t point_feats_per_sector = 3;

  // Parameters for normal estimation
  double radius = 1.0;
  size_t min_points = 5;

  // Based on LiDAR info
  double min_norm_squared = 1.0;
  double max_norm_squared = 100.0 * 100.0;
  int num_columns = 1024;
  int num_rows = 64;
};

template <typename Point>
std::vector<bool> compute_valid_points(const std::vector<Point> &scan,
                                       const KeypointExtractionParams &params) {
  using T = typename Point::Scalar;

  if (scan.size() != params.num_columns * params.num_rows) {
    throw std::runtime_error("Provided scan does not match the expected size " +
                             std::to_string(params.num_columns * params.num_rows) +
                             " != " + std::to_string(scan.size()));
  }

  std::vector<bool> mask(scan.size(), true);
  size_t num_points = scan.size();

  // Compute the valid points based on the parameters
  // Structured search (search over each scan line individually over all points
  // [except points on scan line ends]
  for (size_t scan_line_idx = 0; scan_line_idx < params.num_rows; scan_line_idx++) {
    for (size_t line_pt_idx = 0; line_pt_idx < params.num_columns; line_pt_idx++) {
      const size_t idx = (scan_line_idx * params.num_columns) + line_pt_idx;

      // CHECK 1: Due to edge effects, the first and last neighbor_points points
      // of each scan line are invalid
      if (line_pt_idx < params.neighbor_points ||
          line_pt_idx >= params.num_columns - params.neighbor_points) {
        mask[idx] = false;
        continue;
      }

      // CHECK 2: Is the point in the valid range of the LiDAR
      const Point &point = scan[idx];
      const double range2 = point.squaredNorm();
      if (range2 < params.min_norm_squared || range2 > params.max_norm_squared) {
        mask[idx] = false;
        for (size_t i = 1; i <= params.neighbor_points; i++) {
          mask[idx - i] = false;
          mask[idx + i] = false;
        }
        continue;
      }
    } // end line point search
  } // end scan line search

  return mask;
}

/// @brief Structure for storing curvature information for points
template <typename T> struct Curvature {
  /// @brief The index of the point
  size_t index;
  /// @brief The curvature of the point
  T curvature;
  /// @brief Explicit parameterized constructor
  Curvature(size_t index, double curvature) : index(index), curvature(curvature) {}
  /// @brief Default constructor
  Curvature() = default;

  /// @brief Comparison operator for sorting
  bool operator<(const Curvature &other) const {
    return curvature < other.curvature;
  }
};

template <typename Point>
std::vector<Curvature<typename Point::Scalar>>
extract_curvature(const std::vector<Point> &scan, const std::vector<bool> &mask,
                  const KeypointExtractionParams &params, size_t scan_idx) noexcept {

  using T = typename Point::Scalar;
  std::vector<Curvature<T>> curvature;

  // Structured search (search over each scan line individually over all points
  // [except points on scan line ends]
  for (size_t scan_line_idx = 0; scan_line_idx < params.num_rows; scan_line_idx++) {
    for (size_t line_pt_idx = 0; line_pt_idx < params.num_columns; line_pt_idx++) {
      const size_t idx = (scan_line_idx * params.num_columns) + line_pt_idx;
      // If not valid, input max curvature
      if (!mask[idx]) {
        curvature.emplace_back(idx, std::numeric_limits<T>::max());
      }
      // If valid compute the curvature
      else {
        // Initialize with the difference term
        double dx = -(2.0 * params.neighbor_points) * scan[idx].x();
        double dy = -(2.0 * params.neighbor_points) * scan[idx].y();
        double dz = -(2.0 * params.neighbor_points) * scan[idx].z();
        // Iterate over neighbors and accumulate
        for (size_t n = 1; n <= params.neighbor_points; n++) {
          dx = dx + scan[idx - n].x() + scan[idx + n].x();
          dy = dy + scan[idx - n].y() + scan[idx + n].y();
          dz = dz + scan[idx - n].z() + scan[idx + n].z();
        }
        curvature.emplace_back(idx, dx * dx + dy * dy + dz * dz);
      }
    }
  }
  return curvature;
}

template <typename T>
void extract_planar(const size_t &sector_start_point, const size_t &sector_end_point,
                    const std::vector<Curvature<T>> &curvature,
                    const KeypointExtractionParams &params,
                    std::vector<size_t> &out_features,
                    std::vector<bool> &valid_mask) {

  size_t num_sector_planar_features = 0;
  // Iterate through all points in the sector
  for (size_t sorted_curv_idx = sector_start_point;
       sorted_curv_idx < sector_end_point; sorted_curv_idx++) {
    const Curvature curv = curvature[sorted_curv_idx];
    if (valid_mask[curv.index] && curv.curvature < params.planar_threshold) {
      out_features.push_back(curv.index);
      // mark the neighbors as used so they aren't also added in
      for (size_t n = 0; n < params.neighbor_points; n++) {
        valid_mask[curv.index + n] = false;
        valid_mask[curv.index - n] = false;
      }
      num_sector_planar_features++;
    }
    // Early exit if we have found enough features
    if (num_sector_planar_features > params.planar_feats_per_sector)
      break;

  } // end feature search in sector
}

inline void extract_point(const size_t &sector_start_point,
                          const size_t &sector_end_point,
                          const KeypointExtractionParams &params,
                          std::vector<size_t> &out_features,
                          std::vector<bool> &valid_mask) {
  size_t num_sector_point_features = 0;

  if (params.point_feats_per_sector == 0) {
    return; // No point features to extract
  }

  // Figure out how many we may have
  std::vector<size_t> unused_points;
  for (size_t idx = sector_start_point; idx < sector_end_point; idx++) {
    if (valid_mask[idx]) {
      unused_points.push_back(idx);
    }
  }

  // By what factor do we have too many?
  size_t factor = 1 + unused_points.size() / params.point_feats_per_sector;
  // Do "factor" number of passes over the points until we get enough
  // This should help spread them out evenly
  for (size_t offset = 0; offset < factor; offset++) {
    for (size_t unused_idx = offset; unused_idx < unused_points.size();
         unused_idx += factor) {
      const size_t idx = unused_points[unused_idx];
      if (valid_mask[idx]) {
        out_features.push_back(idx);                          // Add to points
        for (size_t n = 0; n < params.neighbor_points; n++) { // update mask
          valid_mask[idx + n] = false;
          valid_mask[idx - n] = false;
        }
        num_sector_point_features++;
      }
      // Early exit if we have found enough features
      if (num_sector_point_features > params.point_feats_per_sector)
        break;
    }
  }
}

template <typename Point>
std::optional<size_t> find_closest(const Point &point, const size_t &start,
                                   const size_t &end, const std::vector<Point> &scan,
                                   const std::vector<bool> &valid_mask) {
  std::optional<size_t> closest_point = std::nullopt;
  double min_dist2 = std::numeric_limits<double>::max();
  for (size_t idx = start; idx < end; idx++) {
    if (!valid_mask[idx]) {
      continue;
    }
    const double dist2 = (scan[idx] - point).squaredNorm();
    if (dist2 < min_dist2) {
      min_dist2 = dist2;
      closest_point = idx;
    }
  }
  return closest_point;
}

template <typename Point>
void find_neighbors(const size_t &idx, const std::vector<Point> &scan,
                    const KeypointExtractionParams &params,
                    std::vector<Point> &out) {
  // search in the positive direction
  const auto &point = scan[idx];
  for (size_t i = 1; i <= params.neighbor_points; i++) {
    const auto &neighbor = scan[idx + i];
    const double range2 = (neighbor - point).squaredNorm();
    if (range2 < params.radius * params.radius) {
      out.push_back(neighbor);
    } else {
      break;
    }
  }

  // search in the negative direction
  for (size_t i = 1; i <= params.neighbor_points; i++) {
    const auto &neighbor = scan[idx - i];
    const double range2 = (neighbor - point).squaredNorm();
    if (range2 < params.radius * params.radius) {
      out.push_back(neighbor);
    } else {
      break;
    }
  }
}

template <typename Point>
std::optional<Eigen::Matrix<typename Point::Scalar, 3, 1>>
compute_normal(const size_t &idx, const std::vector<Point> &scan,
               const KeypointExtractionParams &params,
               const std::vector<bool> &valid_mask) noexcept {
  using T = typename Point::Scalar;
  const size_t scan_line_idx = idx / params.num_columns;
  const auto start = scan.cbegin();
  const auto end = scan.cend();
  const auto &point = scan[idx];

  // First find neighbors on own scan line
  std::vector<Point> neighbors;
  find_neighbors(idx, scan, params, neighbors);

  bool found_other_scanline = false;

  // Get the neighbors of the point on the previous scan line
  if (scan_line_idx > 0) {
    const size_t prev_scan_line_idx = scan_line_idx - 1;
    const auto closest_idx = find_closest(
        point, params.num_columns * prev_scan_line_idx,
        params.num_columns * (prev_scan_line_idx + 1), scan, valid_mask);
    if (closest_idx.has_value()) {
      found_other_scanline = true;
      neighbors.push_back(scan[*closest_idx]);
      find_neighbors(*closest_idx, scan, params, neighbors);
    }
  }

  // Get the neighbors of the point on the next scan line
  if (scan_line_idx < params.num_rows - 1) {
    // std::printf("---- Searching next scan line %zu\n", scan_line_idx + 1);
    const size_t next_scan_line_idx = scan_line_idx + 1;
    const auto closest_idx = find_closest(
        point, params.num_columns * next_scan_line_idx,
        params.num_columns * (next_scan_line_idx + 1), scan, valid_mask);
    if (closest_idx.has_value()) {
      found_other_scanline = true;
      neighbors.push_back(scan[*closest_idx]);
      find_neighbors(closest_idx.value(), scan, params, neighbors);
    }
  }

  // If there's not enough neighbors, return failed
  if (!found_other_scanline || neighbors.size() < params.min_points) {
    return std::nullopt;
  }

  // Compute the covariance matrix
  // std::printf("---- Found %zu neighbors\n", neighbors.size());
  Eigen::Matrix<T, Eigen::Dynamic, 3> A(neighbors.size(), 3);
  for (size_t j = 0; j < neighbors.size(); ++j) {
    A.row(j) = neighbors[j] - point;
  }
  A /= neighbors.size();
  Eigen::Matrix<T, 3, 3> Cov = A.transpose() * A;

  // Eigenvalues + normals
  // std::printf("---- computing eigenvalues\n");
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<T, 3, 3>> b(
      Cov, Eigen::ComputeEigenvectors);
  Eigen::Matrix<T, 3, 1> normal = b.eigenvectors().col(0);
  normal.normalize();

  return normal;
}

// Assume the points are in row-major order
template <typename Point>
[[nodiscard]] std::tuple<tbb::concurrent_vector<PlanarFeat>, std::vector<PointFeat>>
extract_keypoints(const std::vector<Point> &scan,
                  const KeypointExtractionParams &params, size_t scan_idx) noexcept {
  using T = typename Point::Scalar;
  const size_t points_per_sector = params.num_columns / params.num_sectors;

  // First we validate that all the points are good
  std::vector<bool> valid_mask = compute_valid_points(scan, params);

  // Next we compute the curvature of each point
  std::vector<Curvature<T>> curvature =
      extract_curvature(scan, valid_mask, params, scan_idx);

  // Next get the planar features
  std::vector<size_t> planar_indices;
  std::vector<bool> used_points = valid_mask;
  for (size_t scan_line_idx = 0; scan_line_idx < params.num_rows; scan_line_idx++) {
    // Independently detect features in each sector of this scan_line
    for (size_t sector_idx = 0; sector_idx < params.num_sectors; sector_idx++) {
      // Get the point index of the sector start and sector end
      const size_t sector_start_pt =
          (scan_line_idx * params.num_columns) + (sector_idx * points_per_sector);
      // Special case for end point as we add any reminder points to the last
      // sector
      const size_t sector_end_pt = (sector_idx == params.num_sectors - 1)
                                       ? ((scan_line_idx + 1) * params.num_columns)
                                       : sector_start_pt + points_per_sector;

      // Sort the points within the sector based on curvature
      std::sort(curvature.begin() + sector_start_pt,
                curvature.begin() + sector_end_pt);

      // Search smallest to largest [i.e. planar features]
      // WARN: Mutates planar_indices + used_points
      extract_planar(sector_start_pt, sector_end_pt, curvature, params,
                     planar_indices, used_points);

      // Make any left over "holes" into point features

    } // end sector search
  } // end scan line search

  // Next get the point features
  std::vector<size_t> point_indices;
  // Get a mask of points and their neighborhood that were selected as planar
  // features
  // TODO: This may miss some points on the edges of neighborhoods
  std::vector<bool> used_points_copy(scan.size());
  for (size_t idx = 0; idx < used_points.size(); idx++) {
    used_points_copy[idx] = used_points[idx] == valid_mask[idx];
  }

  // Do a basic filter for them as well
  for (size_t scan_line_idx = 0; scan_line_idx < params.num_rows; scan_line_idx++) {
    for (size_t line_pt_idx = 0; line_pt_idx < params.num_columns; line_pt_idx++) {
      const size_t idx = (scan_line_idx * params.num_columns) + line_pt_idx;

      // CHECK 1: Due to edge effects, the first and last neighbor_points points
      // of each scan line are invalid
      if (line_pt_idx < params.neighbor_points ||
          line_pt_idx >= params.num_columns - params.neighbor_points) {
        used_points_copy[idx] = false;
        continue;
      }

      // CHECK 2: Is the point in the valid range of the LiDAR
      const Point &point = scan[idx];
      const double range2 = point.squaredNorm();
      if (range2 < params.min_norm_squared || range2 > params.max_norm_squared) {
        used_points_copy[idx] = false;
        continue;
      }
    }
  }

  for (size_t scan_line_idx = 0; scan_line_idx < params.num_rows; scan_line_idx++) {
    // Independently detect features in each sector of this scan_line
    for (size_t sector_idx = 0; sector_idx < params.num_sectors; sector_idx++) {
      const size_t sector_start_pt =
          (scan_line_idx * params.num_columns) + (sector_idx * points_per_sector);
      // Special case for end point as we add any reminder points to the last
      // sector
      const size_t sector_end_pt = (sector_idx == params.num_sectors - 1)
                                       ? ((scan_line_idx + 1) * params.num_columns)
                                       : sector_start_pt + points_per_sector;

      extract_point(sector_start_pt, sector_end_pt, params, point_indices,
                    used_points_copy);
    }
  }

  // Finally extract all normals
  using size_t_iterator = typename std::vector<size_t>::const_iterator;
  tbb::concurrent_vector<PlanarFeat> result_planar;
  result_planar.reserve(planar_indices.size() + point_indices.size());
  tbb::parallel_for(tbb::blocked_range<size_t_iterator>{planar_indices.cbegin(),
                                                        planar_indices.cend()},
                    [&](const tbb::blocked_range<size_t_iterator> &range) {
                      for (auto it = range.begin(); it != range.end(); ++it) {
                        const size_t idx = *it;
                        const Point &point = scan[idx];
                        std::optional<Eigen::Matrix<T, 3, 1>> normal =
                            compute_normal(idx, scan, params, valid_mask);
                        if (normal.has_value()) {
                          result_planar.emplace_back(
                              static_cast<double>(point.x()),
                              static_cast<double>(point.y()),
                              static_cast<double>(point.z()),
                              static_cast<double>(normal.value().x()),
                              static_cast<double>(normal.value().y()),
                              static_cast<double>(normal.value().z()),
                              static_cast<size_t>(scan_idx));
                        }
                      }
                    });

  // Add the point features
  std::vector<PointFeat> result_point;
  for (const size_t &idx : point_indices) {
    const Point &point = scan[idx];
    result_point.emplace_back(
        static_cast<double>(point.x()), static_cast<double>(point.y()),
        static_cast<double>(point.z()), static_cast<size_t>(scan_idx));
  }

  return std::make_tuple(result_planar, result_point);
}

struct FeatureExtractor {
  using Params = KeypointExtractionParams;
  Params params;

  FeatureExtractor(const Params &params, size_t num_threads) : params(params) {
    static const auto tbb_control_settings = set_num_threads(num_threads);
  }

  template <typename Point>
  std::tuple<std::vector<PlanarFeat>, std::vector<PointFeat>>
  operator()(const std::vector<Point> &scan, size_t scan_idx) const {
    auto [planar, point] = extract_keypoints(scan, params, scan_idx);
    // copy to a std::vector for return
    return std::make_tuple(std::vector<PlanarFeat>(planar.begin(), planar.end()),
                           point);
  }
};

} // namespace form