#pragma once

#include "form/feature/extraction.hpp"
#include "form/mapping/map.hpp"
#include "form/optimization/constraints.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <tbb/global_control.h>

#include <Eigen/Core>
#include <vector>

namespace form {

struct Match {
  Keypoint_t query;
  Keypoint_t point;
  double dist_sqrd;

  Match(const Keypoint_t &q, const Keypoint_t &p, double d_sqrd)
      : query(q), point(p), dist_sqrd(d_sqrd) {}
};

class Estimator {
public:
  struct Params {
    ConstraintManager::Params constraints;

    // kp extraction params
    feature::FeatureExtractor::Params keypointExtraction;

    // points must be within this percent of range to be matched
    double max_dist_min = 0.1;
    double max_dist_max = 1.0;
    double max_dist_map = 0.1;
    double new_pose_threshold = 1e-4;
    size_t max_num_rematches = 10;

    bool linearize_when_matching = true;
    bool linearize_for_final = false;

    size_t num_threads = 0;
  };

  Params m_params;

  // constraint & graph manager
  ConstraintManager m_constraints;

  // Extractor
  feature::FeatureExtractor m_extractor;

  // keypoint map
  KeypointMap m_keypoint_map;

  // current frame
  using FrameIndex = size_t;
  FrameIndex m_frame;

public:
  Estimator(const Params &params) noexcept;

  void reset(const Params &params) noexcept;

  gtsam::Pose3 current_lidar_estimate() {
    return m_constraints.get_pose(m_frame - 1);
  }

  std::vector<Keypoint_t>
  registerScan(const std::vector<Eigen::Vector3f> &scan) noexcept;
};

} // namespace form