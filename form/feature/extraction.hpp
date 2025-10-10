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

class FeatureExtractor {
public:
  struct Params {
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

  Params params;

  FeatureExtractor(const Params &params, size_t num_threads) : params(params) {
    static const auto tbb_control_settings = set_num_threads(num_threads);
  }

  // ------------------------- Entrypoint ------------------------- //
  template <typename Point>
  [[nodiscard]] std::tuple<std::vector<PlanarFeat>, std::vector<PointFeat>>
  extract(const std::vector<Point> &scan, size_t scan_idx) const;

private:
  // ------------------------- Validators ------------------------- //
  template <typename Point>
  std::vector<bool> compute_valid_points(const std::vector<Point> &scan) const;

  template <typename Point>
  std::vector<bool> compute_point_valid_points(const std::vector<Point> &scan) const;

  // ------------------------- Computers ------------------------- //
  template <typename Point>
  std::vector<Curvature<typename Point::Scalar>>
  compute_curvature(const std::vector<Point> &scan, const std::vector<bool> &mask,
                    size_t scan_idx) const noexcept;

  template <typename Point>
  std::optional<Eigen::Matrix<typename Point::Scalar, 3, 1>>
  compute_normal(const size_t &idx, const std::vector<Point> &scan,
                 const std::vector<bool> &valid_mask) const noexcept;

  // ------------------------- Extractors ------------------------- //

  template <typename T>
  void extract_planar(const size_t &sector_start_point,
                      const size_t &sector_end_point,
                      const std::vector<Curvature<T>> &curvature,
                      std::vector<size_t> &out_features,
                      std::vector<bool> &valid_mask) const noexcept;

  inline void extract_point(const size_t &sector_start_point,
                            const size_t &sector_end_point,
                            std::vector<size_t> &out_features,
                            std::vector<bool> &valid_mask) const noexcept;

  // ------------------------- Helpers ------------------------- //
  template <typename Point>
  std::optional<size_t>
  find_closest(const Point &point, const size_t &start, const size_t &end,
               const std::vector<Point> &scan,
               const std::vector<bool> &valid_mask) const noexcept;

  template <typename Point>
  void find_neighbors(const size_t &idx, const std::vector<Point> &scan,
                      std::vector<Point> &out) const noexcept;
};
} // namespace form

#include "form/feature/extraction.tpp"