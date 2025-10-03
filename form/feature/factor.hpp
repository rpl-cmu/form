#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <vector>

#include "form/feature/type.hpp"
#include "form/gtsam.h"
#include "form/map.hpp"

namespace form::feature {

static void check(const gtsam::SharedNoiseModel &noiseModel, size_t m) {
  if (noiseModel && m != noiseModel->dim())
    throw std::invalid_argument("bad noise model number");
}

using OptionalJacobian = gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic>;

struct PlanePoint {
  typedef std::shared_ptr<PlanePoint> Ptr;

  std::vector<double> p_i;
  std::vector<double> n_i;
  std::vector<double> p_j;

  PlanePoint() = default;

  void push_back(const Keypoint_t &p_i_, const Keypoint_t &p_j_) {
    p_i.insert(p_i.end(), {p_i_.x, p_i_.y, p_i_.z});
    n_i.insert(n_i.end(), {p_i_.nx, p_i_.ny, p_i_.nz});
    p_j.insert(p_j.end(), {p_j_.x, p_j_.y, p_j_.z});
  }

  void clear() noexcept {
    p_i.clear();
    n_i.clear();
    p_j.clear();
  }

  size_t num_residuals() const noexcept { return p_i.size() / 3; }
  size_t num_constraints() const noexcept { return p_i.size() / 3; }

  [[nodiscard]] gtsam::Vector
  evaluateError(const gtsam::Pose3 &Ti, const gtsam::Pose3 &Tj,
                OptionalJacobian residual_D_Ti = boost::none,
                OptionalJacobian residual_D_Tj = boost::none) const noexcept;
};

struct PointPoint {
  typedef std::shared_ptr<PointPoint> Ptr;

  std::vector<double> p_i;
  std::vector<double> p_j;

  PointPoint() = default;

  size_t num_residuals() const noexcept { return p_i.size(); }
  size_t num_constraints() const noexcept { return p_i.size() / 3; }

  void push_back(const Keypoint_t &p_i_, const Keypoint_t &p_j_) {
    p_i.insert(p_i.end(), {p_i_.x, p_i_.y, p_i_.z});
    p_j.insert(p_j.end(), {p_j_.x, p_j_.y, p_j_.z});
  }

  void clear() noexcept {
    p_i.clear();
    p_j.clear();
  }

  [[nodiscard]] gtsam::Vector
  evaluateError(const gtsam::Pose3 &Ti, const gtsam::Pose3 &Tj,
                OptionalJacobian residual_D_Ti = boost::none,
                OptionalJacobian residual_D_Tj = boost::none) const noexcept;
};

class FeatureFactor : public DenseFactor {
public:
  PlanePoint::Ptr plane_plane;
  PointPoint::Ptr point_point;

public:
  FeatureFactor(const gtsam::Key i, const gtsam::Key j,
                const std::tuple<PlanePoint::Ptr, PointPoint::Ptr> &constraint,
                double sigma) noexcept;

  [[nodiscard]] gtsam::Vector
  evaluateError(const gtsam::Pose3 &Ti, const gtsam::Pose3 &Tj,
                boost::optional<gtsam::Matrix &> residual_D_Ti = boost::none,
                boost::optional<gtsam::Matrix &> residual_D_Tj =
                    boost::none) const noexcept override;

  [[nodiscard]] gtsam::Key getKey_i() const noexcept { return keys_[0]; }

  [[nodiscard]] gtsam::Key getKey_j() const noexcept { return keys_[1]; }
};

} // namespace form::feature