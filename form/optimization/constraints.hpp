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

struct Frame {
  size_t idx;
  size_t unused_count = 0;
  size_t size = 0;

  Frame(size_t idx_, size_t size_ = 0) : idx(idx_), size(size_) {}
};

class ConstraintManager {

public:
  using Factor = feature::FeatureFactor;

  struct Params {
    // Maximum number of keyframes to keep
    int64_t max_num_keyframes = 50;
    // Maximum number of steps a keyframe can go unused before being removed
    int64_t max_steps_unused_keyframe = 10;
    // Maximum number of recent frames to keep
    size_t max_num_recent_frames = 10;
    // Keyscan matching ratio
    double keyscan_match_ratio = 0.1;
    // Used for ablations, optimize a single pose at a time
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
  // Recent frames
  std::deque<Frame> m_recent_frames;

  // Key frames
  std::deque<Frame> m_keyframes;

  // Current estimates
  gtsam::Values m_values;

  // Current frame index
  FrameIndex m_frame = 0;

  // all that factors that aren't planar factors & empty slots in that graph
  gtsam::NonlinearFactorGraph m_other_factors;
  std::vector<size_t> m_empty_slots;

  // Linearization of previous matches & m_other_factors for fast optimization
  std::optional<gtsam::LinearContainerFactor> m_fast_linear = std::nullopt;

  // Scan j -- scan i -- Constraints
  // Note that j > i
  // These have to be modified when we rematch.
  using PairwiseConstraintsIndex = tsl::robin_map<
      FrameIndex, tsl::robin_map<FrameIndex, std::tuple<feature::PlanePoint::Ptr,
                                                        feature::PointPoint::Ptr>>>;
  PairwiseConstraintsIndex m_constraints;

public:
  ConstraintManager() : m_params() {};
  ConstraintManager(const Params &params) : m_params(params) {}

  // ------------------------- Doers ------------------------- //
  // Get all the constraints for a given frame going backward
  // If they don't exist, a new one will be created
  tsl::robin_map<FrameIndex,
                 std::tuple<feature::PlanePoint::Ptr, feature::PointPoint::Ptr>> &
  get_constraints(const FrameIndex &frame_j) noexcept;

  // Optimize over the existing constraints, but don't save results
  // fast => linearize previous matches
  gtsam::Values optimize(bool fast = false) noexcept;

  // Check what frames need to be marginalized, and do so
  // Returns the list of frames that were marginalized
  std::vector<FrameIndex> marginalize() noexcept;

  // Marginalize out a specific frame
  void marginalize_frame(const FrameIndex &frame) noexcept;

  // ------------------------- Setters ------------------------- //
  // Add a new pose to the graph
  void add_pose(FrameIndex idx, const gtsam::Pose3 &pose, size_t size) noexcept;

  // Update an existing pose or values
  void update_pose(const FrameIndex &frame, const gtsam::Pose3 &pose) noexcept;
  void update_values(const gtsam::Values &values) noexcept;

  // ------------------------- Getters ------------------------- //
  gtsam::NonlinearFactorGraph get_graph(bool fast) noexcept;
  gtsam::NonlinearFactorGraph get_single_graph() noexcept;
  const gtsam::Pose3 get_pose(const FrameIndex &frame) const noexcept;
  const gtsam::Values &get_values() const noexcept { return m_values; }
  const size_t num_recent_connections(const FrameIndex &frame) const noexcept;
};

} // namespace form