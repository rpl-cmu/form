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
#include "form/optimization/constraints.hpp"
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizerParams.h>
#include <gtsam/nonlinear/Values.h>
#include <tuple>

using gtsam::Pose3;
using gtsam::symbol_shorthand::X;

namespace form {
bool is_empty(const std::tuple<PlanePoint::Ptr, PointPoint::Ptr> &constraints) {
  return std::get<0>(constraints)->num_constraints() == 0 &&
         std::get<1>(constraints)->num_constraints() == 0;
}

ConstraintManager::ConstraintMap &
ConstraintManager::get_constraints(const ScanIndex &scan_j) noexcept {
  // Check if it already exists
  auto search = m_constraints.find(scan_j);
  if (search != m_constraints.end()) {
    return search.value();
  }
  // If not, make it and return it
  else {
    tsl::robin_map<ScanIndex, std::tuple<PlanePoint::Ptr, PointPoint::Ptr>>
        new_constraints;

    // Make the new entry for each previous scan
    for (const auto key : m_values.keys()) {
      auto i = gtsam::Symbol(key).index();
      if (i == scan_j)
        continue;
      new_constraints.insert(
          std::make_pair(i, std::make_tuple(std::make_shared<PlanePoint>(),
                                            std::make_shared<PointPoint>())));
    }

    m_constraints.insert(std::make_pair(scan_j, new_constraints));
    return m_constraints.at(scan_j);
  }
}

ConstraintManager::ConstraintMap &
ConstraintManager::get_current_constraints() noexcept {
  return get_constraints(m_scan);
}

gtsam::Pose3 ConstraintManager::predict_next() const noexcept {
  // Handle the case where scan = 0 and we haven't started yet
  if (!initialized()) {
    return gtsam::Pose3::Identity();
  }

  size_t scan = m_scan + 1;

  bool prev_exists = scan > 0 && m_values.exists(X(scan - 1));
  bool prev_prev_exists = scan > 1 && m_values.exists(X(scan - 2));

  // If we have the previous 2 poses, do a constant velocity model
  if (prev_exists && prev_prev_exists) {
    const auto prev_pose = get_pose(scan - 1);
    const auto prev_prev_pose = get_pose(scan - 2);
    auto prediction = prev_pose * (prev_prev_pose.inverse() * prev_pose);
    // normalize rotation to avoid rounding errors
    prediction = Pose3(prediction.rotation().normalized(), prediction.translation());
    return prediction;
  }

  // If we just have the previous pose, return that
  else if (prev_exists) {
    return get_pose(scan - 1);
  }

  // If we don't have any previous poses, return identity
  else {
    return gtsam::Pose3::Identity();
  }
}

gtsam::Values ConstraintManager::optimize(bool fast) noexcept {
  if (m_params.disable_smoothing) {
    auto graph = get_single_graph();
    gtsam::Values values;
    values.insert(X(m_scan), get_pose(m_scan));

    DenseLMOptimizer optimizer(graph, values, m_params.opt_params);
    return optimizer.optimize();
  } else {
    auto graph = get_graph(fast);
    DenseLMOptimizer optimizer(graph, m_values, m_params.opt_params);
    return optimizer.optimize();
  }

  // Solve!
}

void ConstraintManager::marginalize(const std::vector<ScanIndex> &scans) noexcept {

  gtsam::KeyVector keys;
  for (const auto &f : scans) {
    keys.push_back(X(f));
  }

  gtsam::NonlinearFactorGraph dropped_factors;

  // Find all other factors that involve the keys to marginalize
  size_t index = 0;
  for (auto iter = m_other_factors.begin(); iter != m_other_factors.end(); ++iter) {
    if (*iter) {
      for (auto key : (*iter)->keys()) {
        if (std::find(keys.begin(), keys.end(), key) != keys.end()) {
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
  auto is_marg_scan = [&scans](const ScanIndex &index) {
    return std::find(scans.begin(), scans.end(), index) != scans.end();
  };

  for (const auto &[j, scan_constraints] : m_constraints) {
    const auto pose_j_key = X(j);
    for (const auto &[i, planar_constraints] : scan_constraints) {
      if (is_empty(planar_constraints))
        continue;

      if (is_marg_scan(i) || is_marg_scan(j)) {
        dropped_factors.push_back(FeatureFactor(X(i), pose_j_key, planar_constraints,
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
  for (const auto &k : keys) {
    m_values.erase(k);
  }

  // Erase from constraints
  for (const auto &f : scans) {
    m_constraints.erase(f);
    for (auto it = m_constraints.begin(); it != m_constraints.end(); ++it) {
      auto search = it->second.find(f);
      if (search != it->second.end()) {
        it.value().erase(search);
      }
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

std::tuple<size_t, ConstraintManager::ConstraintMap &>
ConstraintManager::step(const gtsam::Pose3 &pose) noexcept {
  // Only increment the scan if we've already initialized
  // Skips incrementing for the first scan
  if (initialized()) {
    ++m_scan;
  }

  m_values.insert(X(m_scan), pose);
  m_fast_linear = std::nullopt;

  // If this is the first scan, add a prior
  if (m_scan == 0) {
    m_other_factors.addPrior(X(0), pose, m_params.pose_noise);
  }

  return std::tie(m_scan, get_current_constraints());
}

void ConstraintManager::update_pose(const ScanIndex &scan,
                                    const gtsam::Pose3 &pose) noexcept {
  m_values.update(X(scan), pose);
}

void ConstraintManager::update_current_pose(const gtsam::Pose3 &pose) noexcept {
  update_pose(m_scan, pose);
}

// ------------------------- Getters ------------------------- //
gtsam::NonlinearFactorGraph ConstraintManager::get_single_graph() noexcept {
  // Create a new graph
  gtsam::NonlinearFactorGraph graph;

  for (const auto &[i, planar_constraints] : m_constraints.at(m_scan)) {
    if (is_empty(planar_constraints))
      continue;

    auto factor = FeatureFactor(X(i), X(m_scan), planar_constraints,
                                m_params.planar_constraint_sigma);
    auto binary_factor = BinaryFactorWrapper::Create(get_pose(i), X(m_scan), factor);
    graph.push_back(binary_factor);
  }

  return graph;
}

gtsam::NonlinearFactorGraph ConstraintManager::get_graph(bool fast) noexcept {
  // Add other factors
  auto graph = gtsam::NonlinearFactorGraph(m_other_factors);

  // If we should use linearized factors for previous constraints
  if (fast) {
    // Gather all constraints for the current scan
    for (const auto &[i, planar_constraints] : m_constraints.at(m_scan)) {
      if (is_empty(planar_constraints))
        continue;

      graph.push_back(FeatureFactor(X(i), X(m_scan), planar_constraints,
                                    m_params.planar_constraint_sigma));
    }

    // If we don't have the linear factor, compute it
    if (!m_fast_linear.has_value()) {
      gtsam::NonlinearFactorGraph previous_matches;
      for (const auto &[j, scan_constraints] : m_constraints) {
        if (j == m_scan) {
          continue; // Skip the current scan, already added in
        }
        const auto pose_j_key = X(j);
        for (const auto &[i, planar_constraints] : scan_constraints) {
          if (is_empty(planar_constraints))
            continue;
          previous_matches.push_back(
              FeatureFactor(X(i), pose_j_key, planar_constraints,
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
        graph.push_back(FeatureFactor(X(i), pose_j_key, planar_constraints,
                                      m_params.planar_constraint_sigma));
      }
    }
  }

  return graph;
}

const gtsam::Pose3
ConstraintManager::get_pose(const ScanIndex &scan) const noexcept {
  return m_values.at<gtsam::Pose3>(X(scan));
}

const gtsam::Pose3 ConstraintManager::get_current_pose() const noexcept {
  return get_pose(m_scan);
}

const size_t
ConstraintManager::num_recent_connections(const ScanIndex &scan,
                                          const ScanIndex &oldest) const noexcept {
  // Count the number of recent connections to this scan
  size_t count = 0;
  for (const auto &[idx_j, constraints] : m_constraints) {
    if (idx_j < oldest) {
      continue;
    }

    const auto &scan_constraints = constraints.find(scan);
    if (scan_constraints != constraints.end()) {
      count += std::get<0>(scan_constraints.value())->num_constraints();
      count += std::get<1>(scan_constraints.value())->num_constraints();
    }
  }
  return count;
}

} // namespace form