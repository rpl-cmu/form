#include "form/form.hpp"
#include "form/feature/extraction.hpp"
#include "form/map.hpp"

namespace form {

using gtsam::Pose3;
using gtsam::Velocity3;
using gtsam::symbol_shorthand::X;

// ------------------------- Main class ------------------------- //
Estimator::Estimator(const Estimator::Params &params) noexcept
    : m_params(params), m_frame(0), m_constraints(params.constraints),
      m_keypoint_map(KeypointMap::Params{.voxelWidth = m_params.max_dist_max}),
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
  m_keypoint_map =
      KeypointMap(KeypointMap::Params{.voxelWidth = m_params.max_dist_max});

  m_constraints = ConstraintManager(params.constraints);
  m_extractor = feature::FeatureExtractor(params.keypointExtraction);

  m_frame = 0;
}

std::vector<Keypoint_t>
Estimator::registerScan(PointCloud<PointXYZICD<float>> scan) noexcept {

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
  const auto world_map =
      Keypoint_t::Map::from_keypoint_map(m_keypoint_map, m_constraints.get_values());

  // Extract keypoints
  const auto keypoints = m_extractor(scan, m_frame);

  // Make a spot in constraints & keypoint map
  // This might be too expensive, essentially putting too many on at a time.
  auto &scan_constraints = m_constraints.get_constraints(m_frame);
  auto &new_keypoints = m_keypoint_map.get(m_frame);

  auto max_dist_sqrd = m_params.max_dist_max * m_params.max_dist_max;
  auto max_dist_map_sqrd = m_params.max_dist_map * m_params.max_dist_map;

  gtsam::Values new_values;
  tbb::concurrent_vector<Match> matches;
  matches.reserve(keypoints.size());

  // ----------------------- The actual ICP step ----------------------- //
  for (size_t idx_rematch = 0; idx_rematch < m_params.max_num_rematches;
       ++idx_rematch) {
    // ------------------------- Matching part ------------------------- //
    // Loop through all of the new keypoints and perform matching
    for (auto it = scan_constraints.begin(); it != scan_constraints.end(); ++it) {
      std::get<0>(it.value())->clear();
      std::get<1>(it.value())->clear();
    }
    new_keypoints.clear();
    matches.clear();
    auto init = m_constraints.get_pose(m_frame);

    // Find all matches
    using kp_iter = typename std::vector<Keypoint_t>::const_iterator;
    const auto range =
        tbb::blocked_range<kp_iter>{keypoints.cbegin(), keypoints.cend()};
    tbb::parallel_for(
        tbb::blocked_range<kp_iter>{keypoints.cbegin(), keypoints.cend()},
        [&](const tbb::blocked_range<kp_iter> &range) {
          for (auto kp = range.begin(); kp != range.end(); ++kp) {
            auto match = world_map.find_closest(kp->transform(init));

            if (match.found()) {
              auto &scan_pose = m_constraints.get_pose(match.point.scan);
              match.point.transform_in_place(scan_pose.inverse());
            }

            matches.emplace_back(*kp, match.point, match.distanceSquared);
          }
        });

    // Use matches as needed
    for (const auto &match : matches) {
      // if a match is found, save the constraint
      if (match.dist_sqrd < max_dist_sqrd) {
        const auto &keypoint = match.query;
        const auto &point = match.point;
        // Add constraint
        auto &constraints = scan_constraints.at(point.scan);
        if (point.type() == 0) {
          std::get<0>(constraints)->push_back(point, keypoint);
        } else if (point.type() == 1) {
          std::get<1>(constraints)->push_back(point, keypoint);
        }
      }
    }

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
  for (const auto &match : matches) {
    if (match.dist_sqrd > max_dist_map_sqrd) {
      new_keypoints.push_back(match.query);
    }
  }

  // Optimize once more if requested (if they're the same opt, don't run again)
  if (m_params.linearize_for_final != m_params.linearize_when_matching) {
    new_values = m_constraints.optimize(m_params.linearize_for_final);
  }

  m_constraints.update_values(new_values);
  m_constraints.add_frame_size(m_frame, keypoints.size());

  // Handle callbacks
  // for (const auto &callback : callbacks) {
  //   callback(scan, m_frame, keypoints, m_constraints.get_values(),
  //            m_constraints.get_graph(), world_map);
  // }

  // ------------------------- Marginalization ------------------------- //
  auto marg_frame = m_constraints.marginalize(true);
  m_keypoint_map.remove(marg_frame);

  ++m_frame;

  return keypoints;
}

} // namespace form
