#include "form/mapping/map.hpp"
#include "form/optimization/constraints.hpp"
#include <tbb/concurrent_vector.h>
#include <tbb/parallel_for.h>

namespace form {

template <typename Point> struct Match {
  Point query;
  Point point;
  double dist_sqrd;

  Match(const Point &q, const Point &p, double d_sqrd)
      : query(q), point(p), dist_sqrd(d_sqrd) {}
};

struct MatcherParams {
  double max_dist_matching = 0.8;
  double max_dist_map = 0.1;
};

template <typename Point> class Matcher {
public:
  MatcherParams m_params;
  tbb::concurrent_vector<Match<Point>> matches;

public:
  Matcher(const MatcherParams &params) : m_params(params) {}

  template <int I>
  void match(
      const VoxelMap<Point> &map, const std::vector<Point> &keypoints,
      const gtsam::Pose3 &init, const ConstraintManager &manager,
      tsl::robin_map<FrameIndex,
                     std::tuple<feature::PlanePoint::Ptr, feature::PointPoint::Ptr>>
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

        if (match.found()) {
          auto &scan_pose = manager.get_pose(match.point.scan);
          match.point.transform_in_place(scan_pose.inverse());
        }

        matches.emplace_back(*kp, match.point, match.distanceSquared);
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

  void insert_map(std::vector<Point> &map) {
    double max_dist_map_sqrd = m_params.max_dist_map * m_params.max_dist_map;
    for (const auto &match : matches) {
      if (match.dist_sqrd > max_dist_map_sqrd) {
        map.push_back(match.query);
      }
    }
  }
};
} // namespace form