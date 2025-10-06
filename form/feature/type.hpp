#pragma once

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <gtsam/geometry/Pose3.h>

namespace form {

struct PointFeat {
  using type_t = double;
  double x;
  double y;
  double z;
  double _ = static_cast<double>(0);
  size_t scan;

  // ------------------------- Constructor ------------------------- //
  PointFeat() = default;

  PointFeat(double x, double y, double z, size_t scan)
      : x(x), y(y), z(z), _(0), scan(scan) {}

  // ------------------------- Position getters ------------------------- //
  [[nodiscard]] inline Eigen::Map<const Eigen::Vector3d> vec3() const noexcept {
    return Eigen::Map<const Eigen::Vector3d>(&x);
  }

  [[nodiscard]] inline Eigen::Map<Eigen::Vector3d> vec3() noexcept {
    return Eigen::Map<Eigen::Vector3d>(&x);
  }

  [[nodiscard]] inline Eigen::Map<const Eigen::Vector4d> vec4() const noexcept {
    return Eigen::Map<const Eigen::Vector4d>(&x);
  }

  // ------------------------- Misc ------------------------- //
  inline void transform_in_place(const gtsam::Pose3 &pose) noexcept {
    vec3() = pose * vec3();
  }

  [[nodiscard]] inline PointFeat transform(const gtsam::Pose3 &pose) const noexcept {
    auto point = *this;
    point.transform_in_place(pose);
    return point;
  }

  [[nodiscard]] inline double squaredNorm() const noexcept {
    return vec4().squaredNorm();
  }

  [[nodiscard]] inline double norm() const noexcept { return vec4().norm(); }

  [[nodiscard]] constexpr bool operator==(const PointFeat &other) const noexcept {
    return x == other.x && y == other.y && z == other.z && scan == other.scan;
  }
};

// For storing in local map
struct PlanarFeat {
  using type_t = double;
  double x;
  double y;
  double z;
  double _ = static_cast<double>(0);
  double nx;
  double ny;
  double nz;
  double _n = static_cast<double>(0);
  size_t scan;

  // ------------------------- Constructor ------------------------- //
  PlanarFeat() = default;

  PlanarFeat(double x, double y, double z, double nx, double ny, double nz,
             size_t scan)
      : x(x), y(y), z(z), _(0), nx(nx), ny(ny), nz(nz), _n(0), scan(scan) {}

  // ------------------------- Position getters ------------------------- //
  [[nodiscard]] inline Eigen::Map<const Eigen::Vector3d> vec3() const noexcept {
    return Eigen::Map<const Eigen::Vector3d>(&x);
  }

  [[nodiscard]] inline Eigen::Map<Eigen::Vector3d> vec3() noexcept {
    return Eigen::Map<Eigen::Vector3d>(&x);
  }

  [[nodiscard]] inline Eigen::Map<const Eigen::Vector4d> vec4() const noexcept {
    return Eigen::Map<const Eigen::Vector4d>(&x);
  }

  // ------------------------- Normal getters ------------------------- //
  [[nodiscard]] inline Eigen::Map<const Eigen::Vector3d> n_vec3() const noexcept {
    return Eigen::Map<const Eigen::Vector3d>(&nx);
  }

  [[nodiscard]] inline Eigen::Map<Eigen::Vector3d> n_vec3() noexcept {
    return Eigen::Map<Eigen::Vector3d>(&nx);
  }

  // ------------------------- Misc ------------------------- //
  inline void transform_in_place(const gtsam::Pose3 &pose) noexcept {
    vec3() = pose * vec3();
    n_vec3() = pose.rotation() * n_vec3();
  }

  [[nodiscard]] inline PlanarFeat
  transform(const gtsam::Pose3 &pose) const noexcept {
    auto point = *this;
    point.transform_in_place(pose);
    return point;
  }

  [[nodiscard]] inline double squaredNorm() const noexcept {
    return vec4().squaredNorm();
  }

  [[nodiscard]] inline double norm() const noexcept { return vec4().norm(); }

  [[nodiscard]] constexpr bool operator==(const PlanarFeat &other) const noexcept {
    return x == other.x && y == other.y && z == other.z && nx == other.nx &&
           ny == other.ny && nz == other.nz && scan == other.scan;
  }
};

} // namespace form