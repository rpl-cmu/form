#include "form/form.hpp"

namespace form {

using gtsam::Pose3;
using gtsam::Velocity3;
using gtsam::symbol_shorthand::X;

// ------------------------- Main class ------------------------- //
Estimator::Estimator(const Estimator::Params &params) noexcept
    : m_params(params), m_frame(0), m_constraints(params.constraints),
      m_keypoint_map{KeypointMap<PlanarFeat>(m_params.max_dist_max),
                     KeypointMap<PointFeat>(m_params.max_dist_max)},
      m_extractor(params.keypointExtraction) {
  // This global variable requires static duration storage to be able to manipulate
  // the max concurrency from TBB across the entire class
  size_t max_num_threads = params.num_threads > 0
                               ? params.num_threads
                               : tbb::this_task_arena::max_concurrency();
  static const auto tbb_control_settings = tbb::global_control(
      tbb::global_control::max_allowed_parallelism, max_num_threads);
}

void Estimator::reset(const Estimator::Params &params) noexcept {
  m_params = params;
  m_keypoint_map = {KeypointMap<PlanarFeat>(m_params.max_dist_max),
                    KeypointMap<PointFeat>(m_params.max_dist_max)};

  m_constraints = ConstraintManager(params.constraints);
  m_extractor = feature::FeatureExtractor(params.keypointExtraction);

  m_frame = 0;
}

std::tuple<std::vector<PlanarFeat>, std::vector<PointFeat>>
Estimator::registerScan(const std::vector<Eigen::Vector3f> &scan) noexcept {
  constexpr auto seq = std::make_index_sequence<2>{};

  // ------------------------- Initial estimates -------------------------
  // With constant velocity
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

  // ------------------------- Matching ------------------------- //
  // Create world map
  const auto world_map = tuple_transform(m_keypoint_map, [&](auto &map) {
    return map.to_voxel_map(m_constraints.get_values());
  });

  // Extract keypoints
  const auto keypoints = m_extractor(scan, m_frame);

  // Make a spot in constraints & keypoint map
  // This might be too expensive, essentially putting too many on at a time.
  auto &scan_constraints = m_constraints.get_constraints(m_frame);

  auto max_dist_sqrd = m_params.max_dist_max * m_params.max_dist_max;
  auto max_dist_map_sqrd = m_params.max_dist_map * m_params.max_dist_map;

  gtsam::Values new_values;
  auto matches = std::make_tuple(tbb::concurrent_vector<Match<PlanarFeat>>(),
                                 tbb::concurrent_vector<Match<PointFeat>>());
  for_sequence(seq, [&](auto I) {
    std::get<I>(matches).reserve(std::get<I>(keypoints).size());
  });

  // ----------------------- The actual ICP step ----------------------- //
  for (size_t idx_rematch = 0; idx_rematch < m_params.max_num_rematches;
       ++idx_rematch) {

    // ------------------------- Matching part ------------------------- //
    auto init = m_constraints.get_pose(m_frame);

    for_sequence(seq, [&](auto I) {
      // Loop through all of the new keypoints and perform matching
      for (auto it = scan_constraints.begin(); it != scan_constraints.end(); ++it) {
        std::get<I>(it.value())->clear();
      }
      std::get<I>(matches).clear();

      // Find all matches
      const auto range = tbb::blocked_range{std::get<I>(keypoints).cbegin(),
                                            std::get<I>(keypoints).cend()};
      tbb::parallel_for(range, [&](auto &range) {
        for (auto kp = range.begin(); kp != range.end(); ++kp) {
          auto match = std::get<I>(world_map).find_closest(kp->transform(init));

          if (match.found()) {
            auto &scan_pose = m_constraints.get_pose(match.point.scan);
            match.point.transform_in_place(scan_pose.inverse());
          }

          std::get<I>(matches).emplace_back(*kp, match.point, match.distanceSquared);
        }
      });

      // Use matches as needed
      for (const auto &match : std::get<I>(matches)) {
        // if a match is found, save the constraint
        if (match.dist_sqrd < max_dist_sqrd) {
          const auto &keypoint = match.query;
          const auto &point = match.point;
          // Add constraint
          auto &constraints = scan_constraints.at(point.scan);
          std::get<I>(constraints)->push_back(point, keypoint);
        }
      }
    });

    // ------------------------- Optimization -------------------------
    new_values = m_constraints.optimize(m_params.linearize_when_matching);

    // ------------------------- Criteria Check ------------------------- //
    const auto next_frame_pose_before = m_constraints.get_pose(m_frame);
    const auto next_frame_pose_after = new_values.at<Pose3>(X(m_frame));
    const auto diff =
        next_frame_pose_before.localCoordinates(next_frame_pose_after).norm();
    if (diff < m_params.new_pose_threshold) {
      break;
    }
    m_constraints.update_pose(m_frame, next_frame_pose_after);
  }

  // ------------------------- Post ICP ------------------------- //
  // Move some keypoints into map
  for_sequence(seq, [&](auto I) {
    auto &new_kp_map = std::get<I>(m_keypoint_map).get(m_frame);
    for (const auto &match : std::get<I>(matches)) {
      if (match.dist_sqrd > max_dist_map_sqrd) {
        new_kp_map.push_back(match.query);
      }
    }
  });

  // Optimize once more if requested (if they're the same opt, don't run again)
  if (m_params.linearize_for_final != m_params.linearize_when_matching) {
    new_values = m_constraints.optimize(m_params.linearize_for_final);
  }

  m_constraints.update_values(new_values);
  m_constraints.add_frame_size(m_frame, std::get<0>(keypoints).size() +
                                            std::get<1>(keypoints).size());

  // ------------------------- Marginalization ------------------------- //
  auto marg_frame = m_constraints.marginalize(true);
  for_each(m_keypoint_map, [&](auto &map) { map.remove(marg_frame); });

  ++m_frame;

  return keypoints;
}

} // namespace form
