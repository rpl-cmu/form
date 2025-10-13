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
#pragma once

#include <gtsam/geometry/Pose3.h>
#include <vector>

#include "form/feature/features.hpp"
#include "form/optimization/gtsam.hpp"

namespace form {

static void check(const gtsam::SharedNoiseModel &noiseModel, size_t m) {
  if (noiseModel && m != noiseModel->dim())
    throw std::invalid_argument("bad noise model number");
}

using OptionalJacobian = gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic>;

/// @brief Structure that holds plane-point correspondences and computes their error
///
/// Points and normals are stored in a std::vector that is then mapped to Eigen
/// matrices for efficient computation.
struct PlanePoint {
  /// @brief Shared pointer type
  typedef std::shared_ptr<PlanePoint> Ptr;

  /// @brief Vectors to hold point and normal data
  std::vector<double> p_i;
  std::vector<double> n_i;
  std::vector<double> p_j;

  PlanePoint() = default;

  /// @brief Add a new plane-point correspondence
  void push_back(const PlanarFeat &p_i_, const PlanarFeat &p_j_) {
    p_i.insert(p_i.end(), {p_i_.x, p_i_.y, p_i_.z});
    n_i.insert(n_i.end(), {p_i_.nx, p_i_.ny, p_i_.nz});
    p_j.insert(p_j.end(), {p_j_.x, p_j_.y, p_j_.z});
  }

  /// @brief Clear all stored correspondences
  void clear() noexcept {
    p_i.clear();
    n_i.clear();
    p_j.clear();
  }

  /// @brief Get the number of residuals (one per correspondence)
  size_t num_residuals() const noexcept { return p_i.size() / 3; }

  /// @brief Get the number of constraints / correspondences
  size_t num_constraints() const noexcept { return p_i.size() / 3; }

  /// @brief Evaluate the residual given two poses
  ///
  /// @param Ti The pose of the first frame
  /// @param Tj The pose of the second frame
  /// @param residual_D_Ti Optional Jacobian of the residual w.r.t. Ti
  /// @param residual_D_Tj Optional Jacobian of the residual w.r.t. Tj
  /// @return The residual vector
  [[nodiscard]] gtsam::Vector
  evaluateError(const gtsam::Pose3 &Ti, const gtsam::Pose3 &Tj,
                OptionalJacobian residual_D_Ti = boost::none,
                OptionalJacobian residual_D_Tj = boost::none) const noexcept;
};

/// @brief Structure that holds point-point correspondences and computes their error
///
/// Points are stored in a std::vector that is then mapped to Eigen matrices for
/// efficient computation.
struct PointPoint {
  /// @brief Shared pointer type
  typedef std::shared_ptr<PointPoint> Ptr;

  /// @brief Vectors to hold point data
  std::vector<double> p_i;
  std::vector<double> p_j;

  PointPoint() = default;

  /// @brief Add a new point-point correspondence
  void push_back(const PointFeat &p_i_, const PointFeat &p_j_) {
    p_i.insert(p_i.end(), {p_i_.x, p_i_.y, p_i_.z});
    p_j.insert(p_j.end(), {p_j_.x, p_j_.y, p_j_.z});
  }

  /// @brief Clear all stored correspondences
  void clear() noexcept {
    p_i.clear();
    p_j.clear();
  }

  /// @brief Get the number of residuals (three per correspondence)
  size_t num_residuals() const noexcept { return p_i.size(); }

  /// @brief Get the number of constraints / correspondences
  size_t num_constraints() const noexcept { return p_i.size() / 3; }

  /// @brief Evaluate the residual given two poses
  ///
  /// @param Ti The pose of the first frame
  /// @param Tj The pose of the second frame
  /// @param residual_D_Ti Optional Jacobian of the residual w.r.t. Ti
  /// @param residual_D_Tj Optional Jacobian of the residual w.r.t. Tj
  /// @return The residual vector
  [[nodiscard]] gtsam::Vector
  evaluateError(const gtsam::Pose3 &Ti, const gtsam::Pose3 &Tj,
                OptionalJacobian residual_D_Ti = boost::none,
                OptionalJacobian residual_D_Tj = boost::none) const noexcept;
};

/// @brief A wrapper factor that holds both plane-point and point-point constraints
class FeatureFactor : public DenseFactor {
public:
  /// @brief Plane-point constraints
  PlanePoint::Ptr plane_point;

  /// @brief Point-point constraints
  PointPoint::Ptr point_point;

public:
  /// @brief Constructor
  ///
  /// @param i Key for the first pose
  /// @param j Key for the second pose
  /// @param constraint A tuple containing plane-point and point-point constraints
  /// @param sigma The isotropic noise standard deviation
  FeatureFactor(const gtsam::Key i, const gtsam::Key j,
                const std::tuple<PlanePoint::Ptr, PointPoint::Ptr> &constraint,
                double sigma) noexcept;

  /// @brief Evaluate the residual given two poses
  ///
  /// @param Ti The pose of the first frame
  /// @param Tj The pose of the second frame
  /// @param residual_D_Ti Optional Jacobian of the residual w.r.t. Ti
  /// @param residual_D_Tj Optional Jacobian of the residual w.r.t. Tj
  /// @return The residual vector
  [[nodiscard]] gtsam::Vector
  evaluateError(const gtsam::Pose3 &Ti, const gtsam::Pose3 &Tj,
                boost::optional<gtsam::Matrix &> residual_D_Ti = boost::none,
                boost::optional<gtsam::Matrix &> residual_D_Tj =
                    boost::none) const noexcept override;
};

} // namespace form