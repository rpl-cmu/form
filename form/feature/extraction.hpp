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
//
/// This extraction code is largely based on Dan McGann's LOAM implementation
/// (https://github.com/DanMcGann/loam/), with some modifications to remove edge
/// features extraction and add in normal estimation.
#pragma once

#include "form/feature/features.hpp"
#include "form/utils.hpp"
#include <Eigen/Dense>

#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/parallel_for.h>

#include <optional>

namespace form {

/// @brief Structure for storing curvature information for points
template <typename T> struct Curvature {
  /// @brief The index of the point
  size_t index;
  /// @brief The curvature of the point
  T curvature;

  /// @brief Explicit parameterized constructor
  Curvature(size_t index, double curvature) : index(index), curvature(curvature) {}

  /// @brief Comparison operator for sorting
  bool operator<(const Curvature &other) const {
    return curvature < other.curvature;
  }
};

/// @brief Class for extracting planar and point features from a LiDAR scan
class FeatureExtractor {
public:
  struct Params {
    // KEYPOINT EXTRACTION PARAMETERS
    /// @brief Number of neighboring points to use for curvature computation and for
    /// normal estimation
    size_t neighbor_points = 5;
    /// @brief Number of sectors to divide the scan into for feature extraction
    size_t num_sectors = 6;
    /// @brief Threshold for planar feature extraction
    double planar_threshold = 1.0;
    /// @brief Number of planar features to extract per sector
    size_t planar_feats_per_sector = 50;
    /// @brief Number of point features to extract per sector
    size_t point_feats_per_sector = 3;

    // NORMAL ESTIMATION PARAMETERS
    /// @brief Radius for neighborhood search
    double radius = 1.0;
    /// @brief Minimum number of points required for normal estimation
    size_t min_points = 5;

    // MISC LiDAR PARAMETERS
    /// @brief Minimum range for a point to be considered valid
    double min_norm_squared = 1.0;
    /// @brief Maximum range for a point to be considered valid
    double max_norm_squared = 100.0 * 100.0;
    /// @brief Number of columns in the LiDAR scan
    int num_columns = 1024;
    /// @brief Number of rows aka scanlines in the LiDAR scan
    int num_rows = 64;
  };

  Params params;

  /// @brief Explicit parameterized constructor
  FeatureExtractor(const Params &params, size_t num_threads = 0) : params(params) {
    static const auto tbb_control_settings = set_num_threads(num_threads);
  }

  // ------------------------- Entrypoint ------------------------- //
  /// @brief Extract planar and point features from a LiDAR scan
  template <typename Point>
  [[nodiscard]] std::tuple<std::vector<PlanarFeat>, std::vector<PointFeat>>
  extract(const std::vector<Point> &scan, size_t scan_idx) const;

private:
  // ------------------------- Validators ------------------------- //
  /// @brief Compute a mask of valid points based on range and NaN checks, with
  /// neighbors also marked invalid for planar feature extraction
  template <typename Point>
  std::vector<bool> compute_valid_points(const std::vector<Point> &scan) const;

  /// @brief Compute a mask of valid points based on range and NaN checks, without
  /// neighbors marked invalid for point feature extraction
  template <typename Point>
  std::vector<bool> compute_point_valid_points(const std::vector<Point> &scan) const;

  // ------------------------- Computers ------------------------- //
  /// @brief Compute the curvature for each point in the scan
  template <typename Point>
  std::vector<Curvature<typename Point::Scalar>>
  compute_curvature(const std::vector<Point> &scan, const std::vector<bool> &mask,
                    size_t scan_idx) const noexcept;

  /// @brief Compute the normal for a point given its index, the scan, and a validity
  /// mask
  template <typename Point>
  std::optional<Eigen::Matrix<typename Point::Scalar, 3, 1>>
  compute_normal(const size_t &idx, const std::vector<Point> &scan,
                 const std::vector<bool> &valid_mask) const noexcept;

  // ------------------------- Extractors ------------------------- //
  /// @brief Extract planar features from a sector of the scan based on curvature
  template <typename T>
  void extract_planar(const size_t &sector_start_point,
                      const size_t &sector_end_point,
                      const std::vector<Curvature<T>> &curvature,
                      std::vector<size_t> &out_features,
                      std::vector<bool> &valid_mask) const noexcept;

  /// @brief Extract point features from a sector of the scan
  inline void extract_point(const size_t &sector_start_point,
                            const size_t &sector_end_point,
                            std::vector<size_t> &out_features,
                            std::vector<bool> &valid_mask) const noexcept;

  // ------------------------- Helpers ------------------------- //
  /// @brief Find the index of the closest valid point to a given point within a
  /// range (range is usually a scanline)
  template <typename Point>
  std::optional<size_t>
  find_closest(const Point &point, const size_t &start, const size_t &end,
               const std::vector<Point> &scan,
               const std::vector<bool> &valid_mask) const noexcept;

  /// @brief Find the neighboring points of a given point index in the scan, only
  /// including points within neighbor_points and radius
  template <typename Point>
  void find_neighbors(const size_t &idx, const std::vector<Point> &scan,
                      std::vector<Point> &out) const noexcept;
};
} // namespace form

#include "form/feature/extraction.tpp"