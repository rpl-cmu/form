#include "form/form.hpp"
#include <tbb/global_control.h>

namespace form {

using gtsam::Pose3;
using gtsam::Velocity3;
using gtsam::symbol_shorthand::X;

tbb::global_control set_num_threads(size_t num_threads) {
  size_t max_num_threads =
      num_threads > 0 ? num_threads : tbb::this_task_arena::max_concurrency();
  return tbb::global_control(tbb::global_control::max_allowed_parallelism,
                             max_num_threads);
}

Estimator::Estimator(const Estimator::Params &params) noexcept
    : m_params(params), m_extractor(params.extraction),
      m_constraints(params.constraints),
      m_matcher{Matcher<PlanarFeat>(params.matcher),
                Matcher<PointFeat>(params.matcher)},
      m_scan_handler(params.scans),
      m_keypoint_map{KeypointMap<PlanarFeat>(m_params.map),
                     KeypointMap<PointFeat>(m_params.map)} {
  // This global variable requires static duration storage to be able to manipulate
  // the max concurrency from TBB across the entire class
  // TODO: Move this to subclasses as well
  static const auto tbb_control_settings = set_num_threads(params.num_threads);
}

std::tuple<std::vector<PlanarFeat>, std::vector<PointFeat>>
Estimator::register_scan(const std::vector<Eigen::Vector3f> &scan) noexcept {
  constexpr auto SEQ = std::make_index_sequence<2>{};

  //
  // ############################ Feature Extraction ############################ //
  //
  // ------------------------------ Initialization ------------------------------ //
  Pose3 prediction = m_constraints.predict_next();
  size_t frame_idx = m_constraints.add_pose(prediction);

  // ----------------------------- Extract Features ----------------------------- //
  const auto keypoints = m_extractor(scan, frame_idx);
  const auto num_keypoints =
      std::apply([](auto &...kps) { return (kps.size() + ...); }, keypoints);

  //
  // ############################### Optimization ############################### //
  //

  // Create the world map
  const auto world_map = tuple::transform(m_keypoint_map, [&](auto &map) {
    return map.to_voxel_map(m_constraints.get_values(),
                            m_params.matcher.max_dist_matching);
  });

  // Prep storages for matches & constraints
  gtsam::Values new_values;
  auto &scan_constraints = m_constraints.get_current_constraints();
  m_scan_handler.fill_constraints(scan_constraints);

  // ICP loop
  for (size_t idx = 0; idx < m_params.matcher.max_num_rematches; ++idx) {
    auto before = m_constraints.get_current_pose();

    // -------------------------------- Matching -------------------------------- //
    // Match each type of feature
    tuple::for_seq(SEQ, [&](auto I) {
      std::get<I>(m_matcher).template match<I>(std::get<I>(world_map),
                                               std::get<I>(keypoints), before,
                                               m_constraints, scan_constraints);
    });

    // ---------------------- Semi-Linearized Optimization ---------------------- //
    new_values = m_constraints.optimize(true);
    const auto after = new_values.at<Pose3>(X(frame_idx));
    const auto diff = before.localCoordinates(after).norm();
    if (diff < m_params.matcher.new_pose_threshold) {
      break;
    }
    m_constraints.update_current_pose(after);
  }

  // ------------------------ Full Nonlinear Optimization ------------------------ //
  new_values = m_constraints.optimize(false);
  m_constraints.update_values(new_values);

  //
  // ################################## Mapping ################################## //
  //
  // ------------------------------ Map insertions ------------------------------ //
  tuple::for_seq(SEQ, [&](auto I) {
    std::get<I>(m_keypoint_map).insert_matches(std::get<I>(m_matcher).get_matches());
  });

  // ---------------------------- Keyscan Selection ---------------------------- //
  const auto connections = [&](FrameIndex frame) {
    return m_constraints.num_recent_connections(frame, m_scan_handler.oldest_rf());
  };
  auto marg_frames = m_scan_handler.update(frame_idx, num_keypoints, connections);

  // ----------------------------- Marginalization ----------------------------- //
  for (const auto &frame : marg_frames) {
    m_constraints.marginalize_frame(frame);
  }
  tuple::for_each(m_keypoint_map, [&](auto &map) { map.remove(marg_frames); });

  return keypoints;
}

} // namespace form
