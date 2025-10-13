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

#include "form/feature/extraction.hpp"
#include "form/feature/features.hpp"
#include "form/mapping/keyscanner.hpp"
#include "form/mapping/map.hpp"
#include "form/optimization/constraints.hpp"
#include "form/optimization/matcher.hpp"

#include <gtsam/geometry/Pose3.h>
#include <tbb/global_control.h>

#include <Eigen/Core>
#include <vector>

namespace form {

/// @brief Main class for the FORM LiDAR odometry system
struct Estimator {
  /// @brief Parameters for the Estimator
  struct Params {
    /// @brief Extraction params
    FeatureExtractor::Params extraction;

    /// @brief Optimization params
    MatcherParams matcher;
    ConstraintManager::Params constraints;

    /// @brief Mapping params
    KeyScanner::Params scans;
    KeypointMapParams map;

    /// @brief number of threads to use (0 = all available)
    size_t num_threads = 0;
  };

  /// @brief Parameters
  Params m_params;

  /// @brief Features extraction
  FeatureExtractor m_extractor;

  /// @brief Optimization
  ConstraintManager m_constraints;
  std::tuple<Matcher<PlanarFeat>, Matcher<PointFeat>> m_matcher;

  /// @brief Mapping
  KeyScanner m_keyscanner;
  std::tuple<KeypointMap<PlanarFeat>, KeypointMap<PointFeat>> m_keypoint_map;

  /// @brief Default constructor with default parameters
  Estimator() : Estimator(Params()) {}

  /// @brief Constructor with custom parameters
  Estimator(const Params &params) noexcept;

  /// @brief Get the current lidar estimate
  gtsam::Pose3 current_lidar_estimate() { return m_constraints.get_current_pose(); }

  /// @brief Register a new scan and return the extracted features
  std::tuple<std::vector<PlanarFeat>, std::vector<PointFeat>>
  register_scan(const std::vector<Eigen::Vector3f> &scan) noexcept;
};

} // namespace form
