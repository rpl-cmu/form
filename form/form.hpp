#pragma once

#include "form/feature/extraction.hpp"
#include "form/feature/type.hpp"
#include "form/mapping/map.hpp"
#include "form/mapping/scan_handler.hpp"
#include "form/optimization/constraints.hpp"
#include "form/optimization/matcher.hpp"

#include <gtsam/geometry/Pose3.h>
#include <tbb/global_control.h>

#include <Eigen/Core>
#include <vector>

namespace form {

class Estimator {
public:
  struct Params {

    // Extraction params
    FeatureExtractor::Params extraction;

    // Optimization params
    MatcherParams matcher;
    ConstraintManager::Params constraints;

    // Mapping params
    ScanHandler::Params scans;
    KeypointMapParams map;

    // points must be within this percent of range to be matched
    size_t num_threads = 0;
  };

  Params m_params;

  // features
  FeatureExtractor m_extractor;

  // optimization
  ConstraintManager m_constraints;
  std::tuple<Matcher<PlanarFeat>, Matcher<PointFeat>> m_matcher;

  // mapping
  ScanHandler m_scan_handler;
  std::tuple<KeypointMap<PlanarFeat>, KeypointMap<PointFeat>> m_keypoint_map;

public:
  Estimator() : Estimator(Params()) {}

  Estimator(const Params &params) noexcept;

  gtsam::Pose3 current_lidar_estimate() { return m_constraints.get_current_pose(); }

  std::tuple<std::vector<PlanarFeat>, std::vector<PointFeat>>
  register_scan(const std::vector<Eigen::Vector3f> &scan) noexcept;
};

} // namespace form
