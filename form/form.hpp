#pragma once

#include "form/feature/extraction.hpp"
#include "form/feature/type.hpp"
#include "form/mapping/map.hpp"
#include "form/mapping/scan_handler.hpp"
#include "form/optimization/constraints.hpp"
#include "form/optimization/matcher.hpp"

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
