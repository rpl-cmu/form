#pragma once

#include <Eigen/Dense>
#include <gtsam/geometry/Pose3.h>

namespace form {
struct VoxelMapDouble;
}

namespace form::feature {

enum FeatureType {
  Planar = 0, // Planar feature
  Point = 1,  // Point feature
};

// For storing in local map
template <typename T, typename S = size_t> struct PointXYZNTS {
  using type_t = T;

  typedef PointXYZNTS<T, S> Self;
  typedef VoxelMapDouble Map;

  T x;
  T y;
  T z;
  T _ = static_cast<T>(0);
  T nx;
  T ny;
  T nz;
  T _n = static_cast<T>(0);
  S scan;
  FeatureType kind = FeatureType::Point;

  // ------------------------- Constructor ------------------------- //
  PointXYZNTS() = default;

  PointXYZNTS(T x, T y, T z, T nx, T ny, T nz, S scan, FeatureType kind)
      : x(x), y(y), z(z), _(0), nx(nx), ny(ny), nz(nz), _n(0), scan(scan),
        kind(kind) {}

  // ------------------------- Position getters ------------------------- //
  [[nodiscard]] inline Eigen::Map<const Eigen::Matrix<T, 3, 1>>
  vec3() const noexcept {
    return Eigen::Map<const Eigen::Matrix<T, 3, 1>>(&x);
  }

  [[nodiscard]] inline Eigen::Map<Eigen::Matrix<T, 3, 1>> vec3() noexcept {
    return Eigen::Map<Eigen::Matrix<T, 3, 1>>(&x);
  }

  [[nodiscard]] inline Eigen::Map<const Eigen::Matrix<T, 4, 1>>
  vec4() const noexcept {
    return Eigen::Map<const Eigen::Matrix<T, 4, 1>>(&x);
  }

  [[nodiscard]] inline Eigen::Map<const Eigen::Array<T, 3, 1>>
  array() const noexcept {
    return Eigen::Map<const Eigen::Array<T, 3, 1>>(&x);
  }

  [[nodiscard]] inline Eigen::Map<Eigen::Array<T, 3, 1>> array() noexcept {
    return Eigen::Map<Eigen::Array<T, 3, 1>>(&x);
  }

  // ------------------------- Normal getters ------------------------- //
  [[nodiscard]] inline Eigen::Map<const Eigen::Matrix<T, 3, 1>>
  n_vec3() const noexcept {
    return Eigen::Map<const Eigen::Matrix<T, 3, 1>>(&nx);
  }

  [[nodiscard]] inline Eigen::Map<Eigen::Matrix<T, 3, 1>> n_vec3() noexcept {
    return Eigen::Map<Eigen::Matrix<T, 3, 1>>(&nx);
  }

  [[nodiscard]] inline Eigen::Map<const Eigen::Array<T, 3, 1>>
  n_array() const noexcept {
    return Eigen::Map<const Eigen::Array<T, 3, 1>>(&nx);
  }

  [[nodiscard]] inline Eigen::Map<Eigen::Array<T, 3, 1>> n_array() noexcept {
    return Eigen::Map<Eigen::Array<T, 3, 1>>(&nx);
  }

  // ------------------------- Misc ------------------------- //
  inline void transform_in_place(const gtsam::Pose3 &pose) noexcept {
    vec3() = (pose * vec3().template cast<double>()).template cast<T>();
    n_vec3() =
        (pose.rotation() * n_vec3().template cast<double>()).template cast<T>();
  }

  [[nodiscard]] inline PointXYZNTS<T, S>
  transform(const gtsam::Pose3 &pose) const noexcept {
    auto point = *this;
    point.transform_in_place(pose);
    return point;
  }

  [[nodiscard]] inline T squaredNorm() const noexcept {
    return vec4().squaredNorm();
  }

  [[nodiscard]] inline T norm() const noexcept { return vec4().norm(); }

  [[nodiscard]] constexpr bool operator==(const PointXYZNTS &other) const noexcept {
    return x == other.x && y == other.y && z == other.z && nx == other.nx &&
           ny == other.ny && nz == other.nz && scan == other.scan;
  }

  [[nodiscard]] constexpr size_t type() const noexcept {
    return static_cast<size_t>(kind);
  }
};

} // namespace form::feature