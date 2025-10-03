#pragma once

#include <Eigen/Core>
#include <gtsam/geometry/Pose3.h>

namespace form {

// Directly from lidar
template <typename T> struct PointXYZICD {
  using type_t = T;
  T x;
  T y;
  T z;
  T _ = static_cast<T>(0);
  short unsigned int intensity;
  short unsigned int channel;

  // ------------------------- Getters ------------------------- //
  [[nodiscard]] inline Eigen::Map<Eigen::Matrix<T, 3, 1>> vec3() noexcept {
    return Eigen::Map<Eigen::Matrix<T, 3, 1>>(&x);
  }

  [[nodiscard]] inline Eigen::Map<const Eigen::Matrix<T, 3, 1>>
  vec3() const noexcept {
    return Eigen::Map<const Eigen::Matrix<T, 3, 1>>(&x);
  }

  [[nodiscard]] inline Eigen::Map<const Eigen::Matrix<T, 4, 1>>
  vec4() const noexcept {
    return Eigen::Map<const Eigen::Matrix<T, 4, 1>>(&x);
  }

  [[nodiscard]] inline Eigen::Map<Eigen::Array<T, 3, 1>> array() noexcept {
    return Eigen::Map<Eigen::Array<T, 3, 1>>(&x);
  }

  [[nodiscard]] inline Eigen::Map<const Eigen::Array<T, 3, 1>>
  array() const noexcept {
    return Eigen::Map<const Eigen::Array<T, 3, 1>>(&x);
  }

  // ------------------------- Misc ------------------------- //
  inline void transform_in_place(const gtsam::Pose3 &pose) noexcept {
    vec3() = (pose * vec3().template cast<double>()).template cast<T>();
  }

  [[nodiscard]] inline PointXYZICD<T>
  transform(const gtsam::Pose3 &pose) const noexcept {
    auto point = *this;
    point.transform_in_place(pose);
    return point;
  }

  [[nodiscard]] inline T squaredNorm() const noexcept {
    return vec4().squaredNorm();
  }

  [[nodiscard]] inline T norm() const noexcept { return vec4().norm(); }

  [[nodiscard]] constexpr bool operator==(const PointXYZICD &other) const noexcept {
    return x == other.x && y == other.y && z == other.z &&
           intensity == other.intensity && channel == other.channel;
  }

  template <typename S>
  [[nodiscard]] constexpr PointXYZICD<S> cast() const noexcept {
    return {.x = static_cast<S>(x),
            .y = static_cast<S>(y),
            .z = static_cast<S>(z),
            ._ = 0,
            .intensity = intensity,
            .channel = channel};
  }
};

template <typename T> struct PointCloud {
  std::vector<T> points;
  size_t num_columns = 0;
  short unsigned int num_rows = 0;

  inline void transform_in_place(const gtsam::Pose3 &pose) noexcept {
    std::for_each(points.begin(), points.end(),
                  [&](auto &point) { point.transform_in_place(pose); });
  }
};

} // namespace form
