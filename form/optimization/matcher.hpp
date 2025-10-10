#include "form/mapping/map.hpp"
#include "form/optimization/constraints.hpp"
#include "form/utils.hpp"
#include <tbb/concurrent_vector.h>
#include <tbb/parallel_for.h>

namespace form {

struct MatcherParams {
  double max_dist_matching = 0.8;
  double new_pose_threshold = 1e-4;
  size_t max_num_rematches = 10;
};

template <typename Point> class Matcher {
public:
  MatcherParams m_params;
  tbb::concurrent_vector<Match<Point>> matches;

public:
  Matcher(const MatcherParams &params, size_t num_threads) : m_params(params) {
    static const auto tbb_control_settings = set_num_threads(num_threads);
  }

  template <int I>
  void match(const VoxelMap<Point> &map, const std::vector<Point> &keypoints,
             const gtsam::Pose3 &init, const ConstraintManager &manager,
             tsl::robin_map<FrameIndex, std::tuple<PlanePoint::Ptr, PointPoint::Ptr>>
                 &scan_constraints) {
    // Set everything up
    matches.clear();
    for (auto it = scan_constraints.begin(); it != scan_constraints.end(); ++it) {
      std::get<I>(it.value())->clear();
    }
    matches.reserve(keypoints.size());
    auto max_dist_sqrd = m_params.max_dist_matching * m_params.max_dist_matching;

    // Do the matching
    const auto range = tbb::blocked_range{keypoints.cbegin(), keypoints.cend()};
    tbb::parallel_for(range, [&](auto &range) {
      for (auto kp = range.begin(); kp != range.end(); ++kp) {
        auto match = map.find_closest(kp->transform(init));

        // move back into local frames
        match.query = *kp;
        if (match.found()) {
          auto &scan_pose = manager.get_pose(match.point.scan);
          match.point.transform_in_place(scan_pose.inverse());
        }

        matches.push_back(match);
      }
    });

    // Save the matches as needed
    for (const auto &match : matches) {
      // if a match is found, save the constraint
      if (match.dist_sqrd < max_dist_sqrd) {
        const auto &keypoint = match.query;
        const auto &point = match.point;
        // Add constraint
        auto &constraints = scan_constraints.at(point.scan);
        std::get<I>(constraints)->push_back(point, keypoint);
      }
    }
  };

  const tbb::concurrent_vector<Match<Point>> &get_matches() const { return matches; }
};
} // namespace form