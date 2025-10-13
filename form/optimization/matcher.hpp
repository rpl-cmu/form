// MIT License

// Copyright (c) 2025 Easton Potokar, Taylor Pool, and Michael Kaess

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include "form/feature/factor.hpp"
#include "form/mapping/map.hpp"
#include "form/utils.hpp"
#include <gtsam/geometry/Pose3.h>
#include <tbb/concurrent_vector.h>
#include <tbb/parallel_for.h>

namespace form {

/// @brief Parameters for the Matcher
struct MatcherParams {
  /// @brief Maximum distance for a match to be considered valid
  double max_dist_matching = 0.8;

  /// @brief Stopping criteria for ICP rematching
  double new_pose_threshold = 1e-4;

  /// @brief Maximum number of ICP rematches to perform
  size_t max_num_rematches = 10;
};

/// @brief Class for matching keypoints to a map
template <typename Point> class Matcher {
public:
  /// @brief Parameters for the Matcher
  MatcherParams m_params;

  /// @brief Stores the matches found during the last call to match()
  /// Note the point and query stored in each match are in the local frame of their
  /// respective scans
  tbb::concurrent_vector<Match<Point>> matches;

public:
  /// @brief Constructor with parameters
  Matcher(const MatcherParams &params, size_t num_threads = 0) : m_params(params) {
    static const auto tbb_control_settings = set_num_threads(num_threads);
  }

  /// @brief Match keypoints to the map and store them internally
  /// @tparam I Index of the constraint type (0 for PlanePoint, 1 for PointPoint)
  /// @param map The voxel map to match against
  /// @param keypoints The keypoints to match
  /// @param estimates Function that returns the pose estimate for a given FrameIndex
  /// @param scan_constraints Map from FrameIndex to a tuple of constraint vectors
  /// where the matched constraints will be added
  template <int I>
  void match(const VoxelMap<Point> &map, const std::vector<Point> &keypoints,
             const std::function<gtsam::Pose3(size_t)> &estimates,
             tsl::robin_map<size_t, std::tuple<PlanePoint::Ptr, PointPoint::Ptr>>
                 &scan_constraints) {
    // Set everything up
    matches.clear();
    for (auto it = scan_constraints.begin(); it != scan_constraints.end(); ++it) {
      std::get<I>(it.value())->clear();
    }
    matches.reserve(keypoints.size());
    auto max_dist_sqrd = m_params.max_dist_matching * m_params.max_dist_matching;
    auto init = estimates(keypoints.front().scan);

    // Do the matching
    const auto range = tbb::blocked_range{keypoints.cbegin(), keypoints.cend()};
    tbb::parallel_for(range, [&](auto &range) {
      for (auto kp = range.begin(); kp != range.end(); ++kp) {
        auto match = map.find_closest(kp->transform(init));

        // move back into local frames
        match.query = *kp;
        if (match.found()) {
          auto scan_pose = estimates(match.point.scan);
          match.point.transform_in_place(scan_pose.inverse());
        }

        matches.push_back(match);
      }
    });

    // If a match is close enough, add it as a constraint
    for (const auto &match : matches) {
      if (match.dist_sqrd < max_dist_sqrd) {
        const auto &keypoint = match.query;
        const auto &point = match.point;
        // Add constraint
        auto &constraints = scan_constraints.at(point.scan);
        std::get<I>(constraints)->push_back(point, keypoint);
      }
    }
  };

  /// @brief Get the matches found during the last call to match()
  const tbb::concurrent_vector<Match<Point>> &get_matches() const { return matches; }
};
} // namespace form