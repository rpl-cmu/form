#include "form/optimization/constraints.hpp"
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizerParams.h>
#include <gtsam/nonlinear/Values.h>

using gtsam::Pose3;
using gtsam::symbol_shorthand::X;

namespace form {
bool is_empty(const std::tuple<feature::PlanePoint::Ptr, feature::PointPoint::Ptr>
                  &constraints) {
  return std::get<0>(constraints)->num_constraints() == 0 &&
         std::get<1>(constraints)->num_constraints() == 0;
}

tsl::robin_map<FrameIndex,
               std::tuple<feature::PlanePoint::Ptr, feature::PointPoint::Ptr>> &
ConstraintManager::get_constraints(const FrameIndex &frame_j) noexcept {
  // Check if it already exists
  auto search = m_constraints.find(frame_j);
  if (search != m_constraints.end()) {
    return search.value();
  }
  // If not, make it and return it
  else {
    tsl::robin_map<FrameIndex,
                   std::tuple<feature::PlanePoint::Ptr, feature::PointPoint::Ptr>>
        new_constraints;
    // Add empty vectors for all frames
    for (const auto &frame_i : m_keyframes) {
      new_constraints.insert(std::make_pair(
          frame_i.idx, std::make_tuple(std::make_shared<feature::PlanePoint>(),
                                       std::make_shared<feature::PointPoint>())));
    }
    for (const auto &frame_i : m_recent_frames) {
      new_constraints.insert(std::make_pair(
          frame_i.idx, std::make_tuple(std::make_shared<feature::PlanePoint>(),
                                       std::make_shared<feature::PointPoint>())));
    }
    m_constraints.insert(std::make_pair(frame_j, new_constraints));
    return m_constraints.at(frame_j);
  }
}

gtsam::Values ConstraintManager::optimize(bool fast) noexcept {
  if (m_params.disable_smoothing) {
    auto graph = get_single_graph();
    gtsam::Values values;
    values.insert(X(m_frame), get_pose(m_frame));

    DenseLMOptimizer optimizer(graph, values, m_params.opt_params);
    return optimizer.optimize();
  } else {
    auto graph = get_graph(fast);
    DenseLMOptimizer optimizer(graph, m_values, m_params.opt_params);
    return optimizer.optimize();
  }

  // Solve!
}

std::vector<FrameIndex> ConstraintManager::marginalize() noexcept {
  std::vector<FrameIndex> marg_results;

  // First we handle recent frames
  if (m_recent_frames.size() > m_params.max_num_recent_frames) {
    const auto rf = m_recent_frames.front();
    m_recent_frames.pop_front();

    // If it's a keyframe, add it to the keyframe indices
    auto ratio =
        static_cast<double>(num_recent_connections(rf.idx)) /
        (static_cast<double>(rf.size * static_cast<double>(m_recent_frames.size())));

    if (ratio > m_params.keyscan_match_ratio) {
      m_keyframes.push_back(rf);
    }
    // If not, we marginalize it out
    else {
      marginalize_frame(rf.idx);
      marg_results.push_back(rf.idx);
    }
  }

  std::set<FrameIndex> finished_keyframes;
  for (auto &kf : m_keyframes) {
    // If this keyframe is connected to a recent frame, we don't marginalize it
    if (num_recent_connections(kf.idx) > 0) {
      kf.unused_count = 0;
    } else {
      ++kf.unused_count;
    }

    // If this keyframe has been unused for too long, we marginalize it out
    if (kf.unused_count > m_params.max_steps_unused_keyframe) {
      marginalize_frame(kf.idx);
      marg_results.push_back(kf.idx);
      finished_keyframes.insert(kf.idx);
    }
  }

  m_keyframes.erase(std::remove_if(m_keyframes.begin(), m_keyframes.end(),
                                   [&](const Frame &f) {
                                     return finished_keyframes.find(f.idx) !=
                                            finished_keyframes.end();
                                   }),
                    m_keyframes.end());

  // Marginalize keyframe we have too many keyframes
  // This should ideally never be reached in actuality
  if ((m_params.max_num_keyframes > 0 &&
       m_keyframes.size() > m_params.max_num_keyframes)) {
    const auto kf = m_keyframes.front();
    m_keyframes.pop_front();

    marginalize_frame(kf.idx);
    marg_results.push_back(kf.idx);
  }

  return marg_results;
}

void ConstraintManager::marginalize_frame(const FrameIndex &frame) noexcept {

  const auto pose_key = X(frame);
  gtsam::KeyVector keys{pose_key};

  gtsam::NonlinearFactorGraph dropped_factors;

  // Find all other factors that involve the keys to marginalize
  size_t index = 0;
  for (auto iter = m_other_factors.begin(); iter != m_other_factors.end(); ++iter) {
    if (*iter) {
      for (auto key : (*iter)->keys()) {
        if (pose_key == key) {
          dropped_factors.push_back(*iter);
          m_empty_slots.push_back(index);
          *iter = gtsam::NonlinearFactor::shared_ptr();
          break;
        }
      }
    }
    ++index;
  }

  // Planar constraint factors
  for (const auto &[j, scan_constraints] : m_constraints) {
    const auto pose_j_key = X(j);
    for (const auto &[i, planar_constraints] : scan_constraints) {
      if (is_empty(planar_constraints))
        continue;

      if (i == frame || j == frame) {
        dropped_factors.push_back(Factor(X(i), pose_j_key, planar_constraints,
                                         m_params.planar_constraint_sigma));
      }
    }
  }

  // Marginalize out the keys
  const auto linear_graph = dropped_factors.linearize(m_values);
  const auto [eliminated, remaining] =
      linear_graph->eliminatePartialMultifrontal(keys);
  auto marginal_factors =
      gtsam::LinearContainerFactor::ConvertLinearGraph(*remaining, m_values);

  // Add the marginal factor back to the graph
  for (const auto &marginal_factor : marginal_factors) {
    if (m_empty_slots.empty()) {
      m_other_factors.push_back(marginal_factor);
    } else {
      m_other_factors.replace(m_empty_slots.back(), marginal_factor);
      m_empty_slots.pop_back();
    }
  }

  // Erase the keys from the values
  m_values.erase(pose_key);

  // Erase from constraints
  m_constraints.erase(frame);
  for (auto it = m_constraints.begin(); it != m_constraints.end(); ++it) {
    auto search = it->second.find(frame);
    if (search != it->second.end()) {
      it.value().erase(search);
    }
  }
}

// ------------------------- Setters ------------------------- //
void ConstraintManager::update_values(const gtsam::Values &values) noexcept {
  if (m_values.size() == values.size()) {
    m_values = values;
  } else {
    m_values.update(values);
  }
}

void ConstraintManager::add_pose(FrameIndex idx, const gtsam::Pose3 &pose,
                                 size_t size) noexcept {
  Frame new_frame(idx, size);
  m_values.insert(X(idx), pose);
  m_frame = idx;

  // If this is the first frame, add a prior and make it a keyframe
  if (idx == 0) {
    m_other_factors.addPrior(X(0), pose, m_params.pose_noise);
    m_keyframes.push_back(new_frame);
  }
  // Otherwise add it to recent frames
  else {
    m_recent_frames.push_back(new_frame);
    m_fast_linear = std::nullopt;
  }
}

void ConstraintManager::update_pose(const FrameIndex &frame,
                                    const gtsam::Pose3 &pose) noexcept {
  m_values.update(X(frame), pose);
}

// ------------------------- Getters ------------------------- //
gtsam::NonlinearFactorGraph ConstraintManager::get_single_graph() noexcept {
  // Create a new graph
  gtsam::NonlinearFactorGraph graph;

  for (const auto &[i, planar_constraints] : m_constraints.at(m_frame)) {
    if (is_empty(planar_constraints))
      continue;

    auto factor = Factor(X(i), X(m_frame), planar_constraints,
                         m_params.planar_constraint_sigma);
    auto binary_factor =
        BinaryFactorWrapper::Create(get_pose(i), X(m_frame), factor);
    graph.push_back(binary_factor);
  }

  return graph;
}

gtsam::NonlinearFactorGraph ConstraintManager::get_graph(bool fast) noexcept {
  // Add other factors
  auto graph = gtsam::NonlinearFactorGraph(m_other_factors);

  // If we should use linearized factors for previous constraints
  if (fast) {
    // Gather all constraints for the current frame
    for (const auto &[i, planar_constraints] : m_constraints.at(m_frame)) {
      if (is_empty(planar_constraints))
        continue;

      graph.push_back(Factor(X(i), X(m_frame), planar_constraints,
                             m_params.planar_constraint_sigma));
    }

    // If we don't have the linear factor, compute it
    if (!m_fast_linear.has_value()) {
      gtsam::NonlinearFactorGraph previous_matches;
      for (const auto &[j, scan_constraints] : m_constraints) {
        if (j == m_frame) {
          continue; // Skip the current frame, already added in
        }
        const auto pose_j_key = X(j);
        for (const auto &[i, planar_constraints] : scan_constraints) {
          if (is_empty(planar_constraints))
            continue;
          previous_matches.push_back(Factor(X(i), pose_j_key, planar_constraints,
                                            m_params.planar_constraint_sigma));
        }
      }
      // Don't use linearizeToHessianFactor, as it linearizes sequentially.
      // More allocations this way, but it's quicker
      const auto linear_graph = previous_matches.linearize(m_values);
      gtsam::HessianFactor hessian(*linear_graph);
      m_fast_linear = gtsam::LinearContainerFactor(hessian, m_values);
    }

    // Add the fast linear factor
    graph.push_back(*m_fast_linear);
  }

  // Do a full optimization with nothing linearized
  else {
    for (const auto &[j, scan_constraints] : m_constraints) {
      const auto pose_j_key = X(j);
      for (const auto &[i, planar_constraints] : scan_constraints) {
        if (is_empty(planar_constraints))
          continue;
        graph.push_back(Factor(X(i), pose_j_key, planar_constraints,
                               m_params.planar_constraint_sigma));
      }
    }
  }

  return graph;
}

const gtsam::Pose3
ConstraintManager::get_pose(const FrameIndex &frame) const noexcept {
  return m_values.at<gtsam::Pose3>(X(frame));
}

const size_t
ConstraintManager::num_recent_connections(const FrameIndex &frame) const noexcept {
  // oldest recent connection
  auto oldest_recent = m_recent_frames.front();

  // Count the number of recent connections to this frame
  size_t count = 0;
  for (const auto &[idx_j, constraints] : m_constraints) {
    if (idx_j < oldest_recent.idx) {
      continue;
    }

    const auto &frame_constraints = constraints.find(frame);
    if (frame_constraints != constraints.end()) {
      count += std::get<0>(frame_constraints.value())->num_constraints();
      count += std::get<1>(frame_constraints.value())->num_constraints();
    }
  }
  return count;
}

} // namespace form