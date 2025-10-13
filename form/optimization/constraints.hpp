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

#include "form/feature/factor.hpp"

#include <tsl/robin_map.h>

#include <cstdio>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <optional>

namespace form {

using ScanIndex = size_t;

/// @brief Manages all the constraints between poses
///
/// Internally holds all the factors, values, and optimization routines for
/// estimation
class ConstraintManager {
public:
  /// @brief Parameters for the constraint manager
  /// Rarely changed, defaults should be fine
  struct Params {
    // Used for ablations, optimize a single pose at a time
    bool disable_smoothing = false;

    // These are all hardcoded as they shouldn't need to ever be changed
    // Just scaling to make sure priors & feature factors are balanced
    double planar_constraint_sigma = 0.1;
    gtsam::noiseModel::Base::shared_ptr pose_noise;
    gtsam::LevenbergMarquardtParams opt_params;

    Params() {
      pose_noise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);
      opt_params = gtsam::LevenbergMarquardtParams();
      // We're super dense anyways, don't worry about the ordering
      opt_params.orderingType = gtsam::Ordering::NATURAL;
    }
  };

private:
  /// @brief Parameters
  Params m_params;

  /// @brief Current estimates
  gtsam::Values m_values;

  /// @brief Current scan index
  ScanIndex m_scan = 0;

  /// @brief All that factors that aren't planar factors
  gtsam::NonlinearFactorGraph m_other_factors;

  /// @brief Empty slots in m_other_factors
  std::vector<size_t> m_empty_slots;

  /// @brief Linearization of previous matches & m_other_factors
  std::optional<gtsam::LinearContainerFactor> m_fast_linear = std::nullopt;

  using ConstraintMap =
      tsl::robin_map<ScanIndex, std::tuple<PlanePoint::Ptr, PointPoint::Ptr>>;
  using ConstraintMapMap = tsl::robin_map<ScanIndex, ConstraintMap>;

  /// @brief All the planar and point constraints between scans
  ///
  /// m_constraints[j][i] = (plane_point_factor, point_point_factor)
  /// where j > i
  ConstraintMapMap m_constraints;

public:
  /// @brief Default constructor
  ConstraintManager() : m_params() {}

  /// @brief Constructor with custom parameters
  ConstraintManager(const Params &params) : m_params(params) {}

  // ------------------------- Doers ------------------------- //

  /// @brief Get all the constraints for a given scan going backward
  /// If they don't exist, a new one will be created
  ConstraintMap &get_constraints(const ScanIndex &scan_j) noexcept;

  /// @brief Get all the constraints for the current scan going backward
  ConstraintMap &get_current_constraints() noexcept;

  /// @brief Predict the next pose based on constant velocity assumption
  gtsam::Pose3 predict_next() const noexcept;

  /// @brief Optimize over the existing constraints, but don't save results
  /// fast => linearize previous matches
  gtsam::Values optimize(bool fast = false) noexcept;

  /// @brief Marginalize out the given scans
  void marginalize(const std::vector<ScanIndex> &scans) noexcept;

  /// @brief Add in the next pose and increment everything internally
  ///
  /// Returns the current scan index and an reference to it's empty constraint map
  std::tuple<size_t, ConstraintMap &> step(const gtsam::Pose3 &pose) noexcept;

  // ------------------------- Setters ------------------------- //
  /// @brief Update the internal values with new estimates
  void update_values(const gtsam::Values &values) noexcept;

  /// @brief Update the pose for a given scan
  void update_pose(const ScanIndex &scan, const gtsam::Pose3 &pose) noexcept;

  /// @brief Update the pose for the current scan
  void update_current_pose(const gtsam::Pose3 &pose) noexcept;

  // ------------------------- Getters ------------------------- //
  /// @brief If the constraint manager has been initialized with at least one pose
  /// aka if step() has been called at least once
  bool initialized() const noexcept { return m_values.size() > 0; }

  /// @brief Get the full factor graph
  /// fast => linearize previous matches
  gtsam::NonlinearFactorGraph get_graph(bool fast) noexcept;

  /// @brief Get a factor graph with only current scan as a variable.
  /// Used for ablations
  gtsam::NonlinearFactorGraph get_single_graph() noexcept;

  /// @brief Get a pose for a given scan
  const gtsam::Pose3 get_pose(const ScanIndex &scan) const noexcept;

  /// @brief Get the pose for the current scan
  const gtsam::Pose3 get_current_pose() const noexcept;

  /// @brief Get all the current state estimates
  const gtsam::Values &get_values() const noexcept { return m_values; }

  /// @brief Get the number of connections of scan to all scans newer than or equal
  /// to oldest
  const size_t num_recent_connections(const ScanIndex &scan,
                                      const ScanIndex &oldest) const noexcept;
};

} // namespace form