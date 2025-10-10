#include "form/feature/factor.hpp"
#include "form/optimization/gtsam.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/Constants.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/NoiseModel.h>

namespace form {
// ------------------------- Separate Computation ------------------------- //
[[nodiscard]] gtsam::Vector
PlanePoint::evaluateError(const gtsam::Pose3 &Ti, const gtsam::Pose3 &Tj,
                          OptionalJacobian H1, OptionalJacobian H2) const noexcept {
  // Only use the added constraints
  Eigen::Map<const Eigen::Matrix3Xd> p_i(this->p_i.data(), 3, num_constraints());
  Eigen::Map<const Eigen::Matrix3Xd> n_i(this->n_i.data(), 3, num_constraints());
  Eigen::Map<const Eigen::Matrix3Xd> p_j(this->p_j.data(), 3, num_constraints());

  // Use broadcasted operations to compute everything at once
  Eigen::Matrix3Xd w_ni = Ti.rotation().matrix() * n_i;
  Eigen::Matrix3Xd w_pi =
      (Ti.rotation().matrix() * p_i).colwise() + Ti.translation();
  Eigen::Matrix3Xd w_pj =
      (Tj.rotation().matrix() * p_j).colwise() + Tj.translation();
  Eigen::Matrix3Xd v = w_pj - w_pi;
  gtsam::Vector residual = (w_ni.array() * v.array()).colwise().sum();

  // H1_r = w_ni^T R_i (p_i)_x + v^T R_i (n_i)_x
  // H1_t = - w_ni^T R_i
  if (H1) {
    H1->resize(num_residuals(), 6);
    Eigen::Matrix3Xd RT_n = Ti.rotation().transpose() * w_ni; // 3xn
    Eigen::Matrix3Xd RT_v = Ti.rotation().transpose() * v;    // 3xn
    H1->col(0) =
        RT_n.row(1).cwiseProduct(p_i.row(2)) - RT_n.row(2).cwiseProduct(p_i.row(1)) -
        RT_v.row(1).cwiseProduct(n_i.row(2)) + RT_v.row(2).cwiseProduct(n_i.row(1));
    H1->col(1) =
        RT_n.row(2).cwiseProduct(p_i.row(0)) - RT_n.row(0).cwiseProduct(p_i.row(2)) -
        RT_v.row(2).cwiseProduct(n_i.row(0)) + RT_v.row(0).cwiseProduct(n_i.row(2));
    H1->col(2) =
        RT_n.row(0).cwiseProduct(p_i.row(1)) - RT_n.row(1).cwiseProduct(p_i.row(0)) -
        RT_v.row(0).cwiseProduct(n_i.row(1)) + RT_v.row(1).cwiseProduct(n_i.row(0));
    H1->rightCols(3) = -RT_n.transpose();
  }

  // H2_r = - w_ni^T R_j (p_j)_x
  // H2_t = w_ni^T R_j
  if (H2) {
    H2->resize(num_residuals(), 6);
    Eigen::Matrix3Xd RT_n = Tj.rotation().transpose() * w_ni; // 3xn
    H2->col(0) =
        -RT_n.row(1).cwiseProduct(p_j.row(2)) + RT_n.row(2).cwiseProduct(p_j.row(1));
    H2->col(1) =
        -RT_n.row(2).cwiseProduct(p_j.row(0)) + RT_n.row(0).cwiseProduct(p_j.row(2));
    H2->col(2) =
        -RT_n.row(0).cwiseProduct(p_j.row(1)) + RT_n.row(1).cwiseProduct(p_j.row(0));
    H2->rightCols(3) = RT_n.transpose();
  }

  return residual;
}

[[nodiscard]] gtsam::Vector
PointPoint::evaluateError(const gtsam::Pose3 &Ti, const gtsam::Pose3 &Tj,
                          OptionalJacobian H1, OptionalJacobian H2) const noexcept {
  // Only use the added constraints
  Eigen::Map<const Eigen::Matrix3Xd> p_i(this->p_i.data(), 3, num_constraints());
  Eigen::Map<const Eigen::Matrix3Xd> p_j(this->p_j.data(), 3, num_constraints());

  // Use broadcasted operations to compute everything at once
  Eigen::Matrix3Xd w_pi =
      (Ti.rotation().matrix() * p_i).colwise() + Ti.translation();
  Eigen::Matrix3Xd w_pj =
      (Tj.rotation().matrix() * p_j).colwise() + Tj.translation();
  Eigen::MatrixXd residual = w_pj - w_pi;
  residual.resize(num_residuals(), 1);

  // H1_r = R_i (p_i)_x
  // H1_t = - R_i
  if (H1) {
    H1->resize(num_residuals(), 6);
    Eigen::Matrix3d Ri = Ti.rotation().matrix() * -1.0f;

    Eigen::Matrix3Xd temp = Ri.col(2) * p_i.row(1) - Ri.col(1) * p_i.row(2);
    H1->col(0) = Eigen::Map<Eigen::VectorXd>(temp.data(), temp.size());
    temp = Ri.col(0) * p_i.row(2) - Ri.col(2) * p_i.row(0);
    H1->col(1) = Eigen::Map<Eigen::VectorXd>(temp.data(), temp.size());
    temp = Ri.col(1) * p_i.row(0) - Ri.col(0) * p_i.row(1);
    H1->col(2) = Eigen::Map<Eigen::VectorXd>(temp.data(), temp.size());
    H1->rightCols(3) = Ri.replicate(num_constraints(), 1);
  }

  // H2_r = - R_j (p_j)_x
  // H2_t = R_j
  if (H2) {
    Eigen::Matrix3d Rj = Tj.rotation().matrix();
    H2->resize(num_residuals(), 6);

    Eigen::Matrix3Xd temp = Rj.col(2) * p_j.row(1) - Rj.col(1) * p_j.row(2);
    H2->col(0) = Eigen::Map<Eigen::VectorXd>(temp.data(), temp.size());
    temp = Rj.col(0) * p_j.row(2) - Rj.col(2) * p_j.row(0);
    H2->col(1) = Eigen::Map<Eigen::VectorXd>(temp.data(), temp.size());
    temp = Rj.col(1) * p_j.row(0) - Rj.col(0) * p_j.row(1);
    H2->col(2) = Eigen::Map<Eigen::VectorXd>(temp.data(), temp.size());
    H2->rightCols(3) = Rj.replicate(num_constraints(), 1);
  }

  return residual;
}

// ------------------------- Separate Combined Factor ------------------------- //
FeatureFactor::FeatureFactor(
    const gtsam::Key i, const gtsam::Key j,
    const std::tuple<PlanePoint::Ptr, PointPoint::Ptr> &constraints,
    double sigma) noexcept
    : DenseFactor(
          FastIsotropic::Sigma(sigma, std::get<0>(constraints)->num_residuals() +
                                          std::get<1>(constraints)->num_residuals()),
          i, j),
      plane_plane(std::get<0>(constraints)), point_point(std::get<1>(constraints)) {}

[[nodiscard]] gtsam::Vector
FeatureFactor::evaluateError(const gtsam::Pose3 &Ti, const gtsam::Pose3 &Tj,
                             boost::optional<gtsam::Matrix &> H1,
                             boost::optional<gtsam::Matrix &> H2) const noexcept {

  size_t size = plane_plane->num_residuals() + point_point->num_residuals();
  gtsam::Vector residual(size);

  if (H1) {
    H1->resize(size, 6);
  }
  if (H2) {
    H2->resize(size, 6);
  }

  // Plane-Plane constraints
  size_t start = 0, end = plane_plane->num_residuals();
  if (plane_plane->num_residuals() > 0) {
    Eigen::MatrixXd H1_temp, H2_temp;
    residual.segment(start, end - start) =
        plane_plane->evaluateError(Ti, Tj, H1 ? &H1_temp : 0, H2 ? &H2_temp : 0);
    if (H1) {
      H1->block(start, 0, end - start, 6) = H1_temp;
    }
    if (H2) {
      H2->block(start, 0, end - start, 6) = H2_temp;
    }
  }
  start = end;

  // Point-Point constraints
  end += point_point->num_residuals();
  if (point_point->num_residuals() > 0) {
    Eigen::MatrixXd H1_temp, H2_temp;
    residual.segment(start, end - start) =
        point_point->evaluateError(Ti, Tj, H1 ? &H1_temp : 0, H2 ? &H2_temp : 0);
    if (H1) {
      H1->block(start, 0, end - start, 6) = H1_temp;
    }
    if (H2) {
      H2->block(start, 0, end - start, 6) = H2_temp;
    }
  }

  return residual;
}

} // namespace form
