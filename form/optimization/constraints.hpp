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

using FrameIndex = size_t;

class ConstraintManager {

public:
  using Factor = feature::FeatureFactor;

  struct Params {
    int64_t max_num_keyframes = 50;
    int64_t max_steps_unused_keyframe = 10;
    size_t max_num_recent_frames = 10;
    double keyscan_match_ratio = 0.1;

    bool disable_smoothing = false;

    // These are all hardcoded as they shouldn't need to ever be changed
    // Just scaling to make sure priors & feature factors are balanced
    double planar_constraint_sigma = 0.1;
    gtsam::noiseModel::Base::shared_ptr pose_noise;
    gtsam::noiseModel::Base::shared_ptr vel_noise;
    gtsam::noiseModel::Base::shared_ptr bias_noise;

    gtsam::LevenbergMarquardtParams opt_params;

    Params() {
      pose_noise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);
      vel_noise = gtsam::noiseModel::Isotropic::Sigma(3, 1e-3);
      bias_noise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-2);
      opt_params = gtsam::LevenbergMarquardtParams();
      // We're super dense anyways, don't worry about the ordering
      opt_params.orderingType = gtsam::Ordering::NATURAL;
    }
  };

private:
  Params m_params;

  // ------------------------- Data ------------------------- //
  // graph
  gtsam::Values m_values;
  // all that factors that aren't planar factors
  gtsam::NonlinearFactorGraph m_other_factors;
  FrameIndex m_frame = 0;
  std::optional<gtsam::LinearContainerFactor> m_fast_linear = std::nullopt;
  std::vector<size_t> m_empty_slots;

  // Scan j -- scan i -- Constraints
  // Note that j > i
  // Note that these have to be modified when we rematch.
  using PairwiseConstraintsIndex = tsl::robin_map<
      FrameIndex, tsl::robin_map<FrameIndex, std::tuple<feature::PlanePoint::Ptr,
                                                        feature::PointPoint::Ptr>>>;
  PairwiseConstraintsIndex m_constraints;

  // keyframe management
  std::deque<FrameIndex> m_recent_frame_indices;
  // Could combine these into a single dataset structure
  std::deque<FrameIndex> m_keyframe_indices;
  tsl::robin_map<FrameIndex, size_t> m_keyframe_unused_count;
  tsl::robin_map<FrameIndex, size_t> m_frame_size;

public:
  ConstraintManager() : m_params() {};
  ConstraintManager(const Params &params) : m_params(params) {}

  // ------------------------- Doers ------------------------- //
  // Add all the necessary priors and initial estimates
  void
  initialize(const gtsam::Pose3 &initial_pose,
             std::optional<gtsam::imuBias::ConstantBias> initial_bias = std::nullopt,
             size_t initial_idx = 0) noexcept;

  // Get all the constraints for a given frame going backward
  // If they don't exist, a new one will be created
  tsl::robin_map<FrameIndex,
                 std::tuple<feature::PlanePoint::Ptr, feature::PointPoint::Ptr>> &
  get_constraints(const FrameIndex &frame_j) noexcept;

  // Optimize over the existing constraints, but don't save results
  gtsam::Values optimize(bool fast = false) noexcept;

  // Check if there's a keyframe that needs to be marginalized
  std::vector<FrameIndex> marginalize(bool marginalize_planar = true) noexcept;

  // Marginalize out a specific frame
  void marginalize_frame(const FrameIndex &frame,
                         bool marginalize_planar = true) noexcept;

  // ------------------------- Setters ------------------------- //
  // Add a new pose to the graph
  void add_pose(FrameIndex frame, const gtsam::Pose3 &pose,
                std::optional<gtsam::Velocity3> = std::nullopt) noexcept;
  // Add a new factor to the graph
  void add_factor(const gtsam::NonlinearFactor::shared_ptr &factor) noexcept;
  // Add a keyframe ratio in
  void add_frame_size(FrameIndex frame, size_t size) noexcept {
    m_frame_size.insert_or_assign(frame, size);
  }
  // Update an existing pose
  void update_pose(const FrameIndex &frame, const gtsam::Pose3 &pose) noexcept;
  void update_values(const gtsam::Values &values) noexcept;

  // ------------------------- Getters ------------------------- //
  gtsam::NonlinearFactorGraph get_graph(bool fast) noexcept;
  gtsam::NonlinearFactorGraph get_single_graph() noexcept;
  const gtsam::Pose3 get_pose(const FrameIndex &frame) const noexcept;
  const gtsam::Velocity3 get_velocity(const FrameIndex &frame) const noexcept;
  const gtsam::imuBias::ConstantBias
  get_bias(const FrameIndex &frame) const noexcept;
  const gtsam::Values &get_values() const noexcept { return m_values; }
  const size_t get_num_keyframes() const noexcept {
    return m_keyframe_indices.size();
  }
  const size_t get_num_recent_frames() const noexcept {
    return m_recent_frame_indices.size();
  }
  const size_t get_num_constraints() const noexcept {
    size_t count = 0;
    for (const auto &[_, constraints] : m_constraints) {
      for (const auto &[_, c] : constraints) {
        count += std::get<0>(c)->num_constraints();
        count += std::get<1>(c)->num_constraints();
      }
    }
    return count;
  }

  const bool initialized() const noexcept { return !m_values.empty(); }
  const size_t num_connections(const FrameIndex &frame) const noexcept;
  const size_t num_recent_connections(const FrameIndex &frame) const noexcept;
};

} // namespace form