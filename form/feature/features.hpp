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

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <gtsam/geometry/Pose3.h>

namespace form {

/// @brief Point feature structure for storing in scans
struct PointFeat {
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
  /// @brief Returns a const Eigen::Map to the 3D position of the point
  [[nodiscard]] inline Eigen::Map<const Eigen::Vector3d> vec3() const noexcept {
    return Eigen::Map<const Eigen::Vector3d>(&x);
  }

  /// @brief Returns a mutable Eigen::Map to the 3D position of the point
  [[nodiscard]] inline Eigen::Map<Eigen::Vector3d> vec3() noexcept {
    return Eigen::Map<Eigen::Vector3d>(&x);
  }

  /// @brief Returns a const Eigen::Map to the 3D position of the point with SIMD
  /// padding
  [[nodiscard]] inline Eigen::Map<const Eigen::Vector4d> vec4() const noexcept {
    return Eigen::Map<const Eigen::Vector4d>(&x);
  }

  // ------------------------- Misc ------------------------- //
  /// @brief Transforms the point in place using the given pose
  inline void transform_in_place(const gtsam::Pose3 &pose) noexcept {
    vec3() = pose * vec3();
  }

  /// @brief Transforms the point using the given pose, return a new object
  [[nodiscard]] inline PointFeat transform(const gtsam::Pose3 &pose) const noexcept {
    auto point = *this;
    point.transform_in_place(pose);
    return point;
  }

  /// @brief Returns the squared norm of the point
  [[nodiscard]] inline double squaredNorm() const noexcept {
    return vec4().squaredNorm();
  }

  /// @brief Returns the norm of the point
  [[nodiscard]] inline double norm() const noexcept { return vec4().norm(); }

  /// @brief Equality operator
  [[nodiscard]] constexpr bool operator==(const PointFeat &other) const noexcept {
    return x == other.x && y == other.y && z == other.z && scan == other.scan;
  }
};

/// @brief Planar feature structure for storing in scans
struct PlanarFeat {
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
  /// @brief Returns a const Eigen::Map to the 3D position of the point
  [[nodiscard]] inline Eigen::Map<const Eigen::Vector3d> vec3() const noexcept {
    return Eigen::Map<const Eigen::Vector3d>(&x);
  }

  /// @brief Returns a mutable Eigen::Map to the 3D position of the point
  [[nodiscard]] inline Eigen::Map<Eigen::Vector3d> vec3() noexcept {
    return Eigen::Map<Eigen::Vector3d>(&x);
  }

  /// @brief Returns a const Eigen::Map to the 3D position of the point with SIMD
  /// padding
  [[nodiscard]] inline Eigen::Map<const Eigen::Vector4d> vec4() const noexcept {
    return Eigen::Map<const Eigen::Vector4d>(&x);
  }

  // ------------------------- Normal getters ------------------------- //
  /// @brief Returns a const Eigen::Map to the 3D normal of the point
  [[nodiscard]] inline Eigen::Map<const Eigen::Vector3d> n_vec3() const noexcept {
    return Eigen::Map<const Eigen::Vector3d>(&nx);
  }

  /// @brief Returns a mutable Eigen::Map to the 3D normal of the point
  [[nodiscard]] inline Eigen::Map<Eigen::Vector3d> n_vec3() noexcept {
    return Eigen::Map<Eigen::Vector3d>(&nx);
  }

  // ------------------------- Misc ------------------------- //
  /// @brief Transforms the point in place using the given pose
  inline void transform_in_place(const gtsam::Pose3 &pose) noexcept {
    vec3() = pose * vec3();
    n_vec3() = pose.rotation() * n_vec3();
  }

  /// @brief Transforms the point using the given pose, return a new object
  [[nodiscard]] inline PlanarFeat
  transform(const gtsam::Pose3 &pose) const noexcept {
    auto point = *this;
    point.transform_in_place(pose);
    return point;
  }

  /// @brief Returns the squared norm of the point
  [[nodiscard]] inline double squaredNorm() const noexcept {
    return vec4().squaredNorm();
  }

  /// @brief Returns the norm of the point
  [[nodiscard]] inline double norm() const noexcept { return vec4().norm(); }

  /// @brief Equality operator
  [[nodiscard]] constexpr bool operator==(const PlanarFeat &other) const noexcept {
    return x == other.x && y == other.y && z == other.z && nx == other.nx &&
           ny == other.ny && nz == other.nz && scan == other.scan;
  }
};

} // namespace form