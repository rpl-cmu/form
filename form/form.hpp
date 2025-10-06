#pragma once

#include "form/feature/extraction.hpp"
#include "form/feature/type.hpp"
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

template <typename Point> struct Match {
  Point query;
  Point point;
  double dist_sqrd;

  Match(const Point &q, const Point &p, double d_sqrd)
      : query(q), point(p), dist_sqrd(d_sqrd) {}
};

class Estimator {
public:
  struct Params {
    // Keypoint extraction params
    feature::FeatureExtractor::Params keypointExtraction;

    // Optimization params

    // Mapping params
    ConstraintManager::Params constraints;

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
  std::tuple<KeypointMap<PlanarFeat>, KeypointMap<PointFeat>> m_keypoint_map;

  // current frame
  using FrameIndex = size_t;
  FrameIndex m_frame;

public:
  Estimator(const Params &params) noexcept;

  void reset(const Params &params) noexcept;

  gtsam::Pose3 current_lidar_estimate() {
    return m_constraints.get_pose(m_frame - 1);
  }

  std::tuple<std::vector<PlanarFeat>, std::vector<PointFeat>>
  registerScan(const std::vector<Eigen::Vector3f> &scan) noexcept;
};

namespace tuple {
// Some helpers ot make iterating over tuples easier
// https://www.cppstories.com/2022/tuple-iteration-apply/
template <typename TupleT, typename Fn>
[[nodiscard]] auto transform(TupleT &&tp, Fn &&fn) {
  return std::apply(
      [&fn](auto &&...args) {
        return std::make_tuple(fn(std::forward<decltype(args)>(args))...);
      },
      std::forward<TupleT>(tp));
}

template <typename TupleT, typename Fn> void for_each(TupleT &&tp, Fn &&fn) {
  std::apply(
      [&fn](auto &&...args) { (fn(std::forward<decltype(args)>(args)), ...); },
      std::forward<TupleT>(tp));
}

template <typename T, T... S, typename F>
constexpr void for_seq(std::integer_sequence<T, S...>, F &&f) {
  (void(f(std::integral_constant<T, S>{})), ...);
}

} // namespace tuple

} // namespace form
