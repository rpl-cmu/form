#include "form/optimization/constraints.hpp"
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizerParams.h>
#include <gtsam/nonlinear/Values.h>

using gtsam::Pose3;
using gtsam::Velocity3;
using gtsam::imuBias::ConstantBias;
using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

namespace form {
bool is_empty(const std::tuple<feature::PlanePoint::Ptr, feature::PointPoint::Ptr>
                  &constraints) {
  return std::get<0>(constraints)->num_constraints() == 0 &&
         std::get<1>(constraints)->num_constraints() == 0;
}

void ConstraintManager::initialize(const Pose3 &initial_pose,
                                   std::optional<ConstantBias> initial_bias,
                                   size_t initial_idx) noexcept {
  // Initialize the pose
  m_values.insert(X(initial_idx), initial_pose);
  m_other_factors.addPrior(X(0), initial_pose, m_params.pose_noise);

  // If it's also doing imu fusion
  if (initial_bias) {
    gtsam::Velocity3 initial_vel = gtsam::Velocity3::Zero();
    m_values.insert(V(initial_idx), initial_vel);
    m_values.insert(B(initial_idx), *initial_bias);
    m_other_factors.addPrior(V(0), initial_vel, m_params.vel_noise);
    m_other_factors.addPrior(B(0), *initial_bias, m_params.bias_noise);
  }

  m_keyframe_indices.push_back(initial_idx);
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
    for (const auto &frame_i : m_keyframe_indices) {
      new_constraints.insert(std::make_pair(
          frame_i, std::make_tuple(std::make_shared<feature::PlanePoint>(),
                                   std::make_shared<feature::PointPoint>())));
    }
    for (const auto &frame_i : m_recent_frame_indices) {
      new_constraints.insert(std::make_pair(
          frame_i, std::make_tuple(std::make_shared<feature::PlanePoint>(),
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

std::vector<FrameIndex>
ConstraintManager::marginalize(bool marginalize_planar) noexcept {
  std::vector<FrameIndex> marg_results;

  // First we handle recent frames
  if (m_recent_frame_indices.size() > m_params.max_num_recent_frames) {
    const auto frame_index = m_recent_frame_indices.front();
    m_recent_frame_indices.pop_front();

    // If it's a keyframe, add it to the keyframe indices
    auto ratio =
        static_cast<double>(num_recent_connections(frame_index)) /
        (static_cast<double>(m_frame_size[frame_index] *
                             static_cast<double>(m_recent_frame_indices.size())));

    if (ratio > m_params.keyscan_match_ratio) {
      m_keyframe_indices.push_back(frame_index);
      m_keyframe_unused_count.insert(std::make_pair(frame_index, size_t(0)));
      // std::printf("scan %lu to keyframe, %f ratio, keyframes %lu\n", frame_index,
      //             ratio, m_keyframe_indices.size());
    }
    // If not, we marginalize it out
    else {
      marginalize_frame(frame_index, marginalize_planar);
      marg_results.push_back(frame_index);
    }
  }

  std::set<FrameIndex> finished_keyframes;
  for (auto keyframe_idx : m_keyframe_indices) {
    // If this keyframe is connected to a recent frame, we don't marginalize it
    if (num_recent_connections(keyframe_idx) > 0) {
      m_keyframe_unused_count[keyframe_idx] = 0;
    } else {
      ++m_keyframe_unused_count[keyframe_idx];
    }

    // If this keyframe has been unused for too long, we marginalize it out
    if (m_keyframe_unused_count[keyframe_idx] > m_params.max_steps_unused_keyframe) {
      // std::printf("#: %lu, keyframe %lu unused for %lu steps, marginalizing out "
      //             "(oldest: %lu)\n",
      //             m_keyframe_indices.size(), keyframe_idx,
      //             m_keyframe_unused_count[keyframe_idx],
      //             m_keyframe_indices.front());

      marginalize_frame(keyframe_idx, true);
      marg_results.push_back(keyframe_idx);
      m_keyframe_unused_count.erase(keyframe_idx);
      m_frame_size.erase(keyframe_idx);
      finished_keyframes.insert(keyframe_idx);
    }
  }

  m_keyframe_indices.erase(std::remove_if(m_keyframe_indices.begin(),
                                          m_keyframe_indices.end(),
                                          [&](const FrameIndex &idx) {
                                            return finished_keyframes.find(idx) !=
                                                   finished_keyframes.end();
                                          }),
                           m_keyframe_indices.end());

  // Marginalize keyframe we have too many keyframes
  // This should ideally never be reached in actuality
  if ((m_params.max_num_keyframes > 0 &&
       m_keyframe_indices.size() > m_params.max_num_keyframes)) {
    const auto frame_index = m_keyframe_indices.front();
    marginalize_frame(frame_index, true);
    marg_results.push_back(frame_index);
    m_keyframe_unused_count.erase(frame_index);
    m_frame_size.erase(frame_index);
    m_keyframe_indices.pop_front();
  }

  return marg_results;
}

void ConstraintManager::marginalize_frame(const FrameIndex &frame,
                                          bool marginalize_planar) noexcept {

  const auto pose_key = X(frame);
  const auto vel_key = V(frame);
  const auto bias_key = B(frame);

  bool fuse_imu = m_values.exists(vel_key);
  gtsam::KeyVector keys{pose_key};
  if (fuse_imu) {
    keys.push_back(vel_key);
    keys.push_back(bias_key);
  }

  gtsam::NonlinearFactorGraph dropped_factors;

  // Find all other factors that involve the keys to marginalize
  size_t index = 0;
  for (auto iter = m_other_factors.begin(); iter != m_other_factors.end(); ++iter) {
    if (*iter) {
      for (auto key : (*iter)->keys()) {
        if (pose_key == key || vel_key == key || bias_key == key) {
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
  if (marginalize_planar) {
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
  if (fuse_imu) {
    m_values.erase(vel_key);
    m_values.erase(bias_key);
  }

  // Erase from constraints
  m_constraints.erase(frame);
  for (auto it = m_constraints.begin(); it != m_constraints.end(); ++it) {
    auto search = it->second.find(frame);
    if (search != it->second.end()) {
      it.value().erase(search);
    }
  }

  // Erase from keyframe ratio
  // This should always exist, don't bother checking
  auto search = m_frame_size.find(frame);
  if (search != m_frame_size.end()) {
    m_frame_size.erase(search);
  }
}

// ------------------------- Setters ------------------------- //
void ConstraintManager::add_factor(
    const gtsam::NonlinearFactor::shared_ptr &factor) noexcept {
  if (m_empty_slots.empty()) {
    m_other_factors.push_back(factor);
  } else {
    m_other_factors.replace(m_empty_slots.back(), factor);
    m_empty_slots.pop_back();
  }
}

void ConstraintManager::update_values(const gtsam::Values &values) noexcept {
  if (m_values.size() == values.size()) {
    m_values = values;
  } else {
    m_values.update(values);
  }
}

void ConstraintManager::add_pose(FrameIndex frame, const gtsam::Pose3 &pose,
                                 std::optional<gtsam::Velocity3> vel) noexcept {
  if (frame == 0) {
    // If this is the first frame, initialize the values
    initialize(pose, std::nullopt, frame);
    return;
  }

  m_recent_frame_indices.push_back(frame);
  m_values.insert(X(frame), pose);
  m_fast_linear = std::nullopt;
  m_frame = frame;

  if (vel) {
    m_values.insert(V(frame), *vel);
    m_values.insert(B(frame), m_values.at<ConstantBias>(B(frame - 1)));
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

const gtsam::Velocity3
ConstraintManager::get_velocity(const FrameIndex &frame) const noexcept {
  return m_values.at<gtsam::Velocity3>(V(frame));
}

const gtsam::imuBias::ConstantBias
ConstraintManager::get_bias(const FrameIndex &frame) const noexcept {
  return m_values.at<gtsam::imuBias::ConstantBias>(B(frame));
}

const size_t
ConstraintManager::num_connections(const FrameIndex &frame) const noexcept {
  // Count the number of connections to this frame
  size_t count = 0;
  for (const auto &[idx_j, constraints] : m_constraints) {
    if (idx_j == frame) {
      for (const auto &[_, c] : constraints) {
        count += std::get<0>(c)->num_constraints();
        count += std::get<1>(c)->num_constraints();
      }
    } else {
      for (const auto &[idx_i, c] : constraints) {
        if (idx_i == frame) {
          count += std::get<1>(c)->num_constraints();
          count += std::get<0>(c)->num_constraints();
        }
      }
    }
  }
  return count;
}

const size_t
ConstraintManager::num_recent_connections(const FrameIndex &frame) const noexcept {
  // oldest recent connection
  auto oldest_recent = m_recent_frame_indices.front();

  // Count the number of recent connections to this frame
  size_t count = 0;
  for (const auto &[idx_j, constraints] : m_constraints) {
    if (idx_j < oldest_recent) {
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