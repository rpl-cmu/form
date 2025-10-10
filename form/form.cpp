#include "form/form.hpp"
#include "form/utils.hpp"

namespace form {

using gtsam::Pose3;
using gtsam::Velocity3;
using gtsam::symbol_shorthand::X;

Estimator::Estimator(const Estimator::Params &params) noexcept
    : m_params(params), m_extractor(params.extraction, params.num_threads),
      m_constraints(params.constraints),
      m_matcher{Matcher<PlanarFeat>(params.matcher, params.num_threads),
                Matcher<PointFeat>(params.matcher, params.num_threads)},
      m_scan_handler(params.scans),
      m_keypoint_map{KeypointMap<PlanarFeat>(m_params.map),
                     KeypointMap<PointFeat>(m_params.map)} {}

std::tuple<std::vector<PlanarFeat>, std::vector<PointFeat>>
Estimator::register_scan(const std::vector<Eigen::Vector3f> &scan) noexcept {
  constexpr auto SEQ = std::make_index_sequence<2>{};

  //
  // ############################ Feature Extraction ############################ //
  //
  // ------------------------------ Initialization ------------------------------ //
  // This needs to go first to get the frame index
  Pose3 prediction = m_constraints.predict_next();
  auto [frame_idx, scan_constraints] = m_constraints.add_next_pose(prediction);

  // ----------------------------- Extract Features ----------------------------- //
  const auto keypoints = m_extractor(scan, frame_idx);
  const auto num_keypoints =
      std::apply([](auto &...kps) { return (kps.size() + ...); }, keypoints);

  //
  // ############################### Optimization ############################### //
  //
  // ---------------------------- Generate World Map ---------------------------- //
  const auto world_map = tuple::transform(m_keypoint_map, [&](auto &map) {
    return map.to_voxel_map(m_constraints.get_values(),
                            // make voxel size match the max matching distance
                            m_params.matcher.max_dist_matching);
  });

  // ICP loop
  gtsam::Values new_values;
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
  m_constraints.marginalize(marg_frames);
  tuple::for_each(m_keypoint_map, [&](auto &map) { map.remove(marg_frames); });

  return keypoints;
}

} // namespace form
