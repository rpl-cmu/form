#include "form/form.hpp"

namespace form {

using gtsam::Pose3;
using gtsam::Velocity3;
using gtsam::symbol_shorthand::X;

// ------------------------- Main class ------------------------- //
Estimator::Estimator(const Estimator::Params &params) noexcept
    : m_params(params), m_frame(0), m_constraints(params.constraints),
      m_matcher{Matcher<PlanarFeat>(params.matcher),
                Matcher<PointFeat>(params.matcher)},
      m_keypoint_map{KeypointMap<PlanarFeat>(m_params.map),
                     KeypointMap<PointFeat>(m_params.map)},
      m_scan_handler(params.scans), m_extractor(params.extraction) {
  // This global variable requires static duration storage to be able to manipulate
  // the max concurrency from TBB across the entire class
  // TODO: Move this to subclasses as well
  size_t max_num_threads = params.num_threads > 0
                               ? params.num_threads
                               : tbb::this_task_arena::max_concurrency();
  static const auto tbb_control_settings = tbb::global_control(
      tbb::global_control::max_allowed_parallelism, max_num_threads);
}

std::tuple<std::vector<PlanarFeat>, std::vector<PointFeat>>
Estimator::registerScan(const std::vector<Eigen::Vector3f> &scan) noexcept {
  constexpr auto SEQ = std::make_index_sequence<2>{};

  //
  // ############################ Feature Extraction ############################ //
  //
  const auto keypoints = m_extractor(scan, m_frame);
  const auto num_keypoints =
      std::apply([](auto &...kps) { return (kps.size() + ...); }, keypoints);

  //
  // ############################### Optimization ############################### //
  //
  // ------------------------------ Initialization ------------------------------ //
  Pose3 prediction;
  if (m_frame == 0) {
    prediction = gtsam::Pose3::Identity();
  } else if (m_frame == 1) {
    prediction = m_constraints.get_pose(m_frame - 1);
  } else {
    const auto prev_pose = m_constraints.get_pose(m_frame - 1);
    const auto prev_prev_pose = m_constraints.get_pose(m_frame - 2);
    prediction = prev_pose * (prev_prev_pose.inverse() * prev_pose);
    // normalize rotation to avoid rounding errors
    prediction = Pose3(prediction.rotation().normalized(), prediction.translation());
  }
  m_constraints.add_pose(m_frame, prediction);

  // Create the world map
  const auto world_map = tuple::transform(m_keypoint_map, [&](auto &map) {
    return map.to_voxel_map(m_constraints.get_values(),
                            m_params.matcher.max_dist_matching);
  });

  // Prep storages for matches & constraints
  gtsam::Values new_values;
  auto &scan_constraints = m_constraints.get_constraints(m_frame);
  m_scan_handler.fill_constraints(scan_constraints);

  for (size_t idx = 0; idx < m_params.matcher.max_num_rematches; ++idx) {
    auto before = m_constraints.get_pose(m_frame);

    // -------------------------------- Matching -------------------------------- //
    tuple::for_seq(SEQ, [&](auto I) {
      std::get<I>(m_matcher).template match<I>(std::get<I>(world_map),
                                               std::get<I>(keypoints), before,
                                               m_constraints, scan_constraints);
    });

    // ---------------------- Semi-Linearized Optimization ---------------------- //
    new_values = m_constraints.optimize(true);

    const auto after = new_values.at<Pose3>(X(m_frame));
    const auto diff = before.localCoordinates(after).norm();
    if (diff < m_params.matcher.new_pose_threshold) {
      break;
    }
    m_constraints.update_pose(m_frame, after);
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

  // ------------------------- Keyscan Selection ------------------------- //
  const auto connections = [&](FrameIndex frame) {
    return m_constraints.num_recent_connections(frame,
                                                m_scan_handler.oldest_rf().idx);
  };
  auto marg_frames = m_scan_handler.update(m_frame, num_keypoints, connections);

  // -------------------- Marginalization -------------------- //
  for (const auto &frame : marg_frames) {
    m_constraints.marginalize_frame(frame);
  }
  tuple::for_each(m_keypoint_map, [&](auto &map) { map.remove(marg_frames); });

  ++m_frame;

  return keypoints;
}

} // namespace form
