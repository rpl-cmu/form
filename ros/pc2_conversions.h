// Adapted from evalio's pc2_conversions.h
// Converts ROS PointCloud2 messages into organized form::PointXYZf vectors
// (row-major, num_rows * num_columns) suitable for FORM's feature extraction.
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "form/utils.hpp"

namespace form_ros {

// ----------------------------- Data getters ----------------------------- //
// Build a lambda that reads a field of type T from raw PointCloud2 byte data
// at the given offset, handling the various PointField datatypes.
template <typename T>
std::function<void(T &, const uint8_t *)> data_getter(uint8_t datatype,
                                                      uint32_t offset) {
  using sensor_msgs::msg::PointField;
  switch (datatype) {
  case PointField::UINT8:
    return [offset](T &value, const uint8_t *data) noexcept {
      value = static_cast<T>(data[offset]);
    };
  case PointField::INT8:
    return [offset](T &value, const uint8_t *data) noexcept {
      value = static_cast<T>(*reinterpret_cast<const int8_t *>(data + offset));
    };
  case PointField::UINT16:
    return [offset](T &value, const uint8_t *data) noexcept {
      uint16_t v;
      std::memcpy(&v, data + offset, sizeof(v));
      value = static_cast<T>(v);
    };
  case PointField::INT16:
    return [offset](T &value, const uint8_t *data) noexcept {
      int16_t v;
      std::memcpy(&v, data + offset, sizeof(v));
      value = static_cast<T>(v);
    };
  case PointField::UINT32:
    return [offset](T &value, const uint8_t *data) noexcept {
      uint32_t v;
      std::memcpy(&v, data + offset, sizeof(v));
      value = static_cast<T>(v);
    };
  case PointField::INT32:
    return [offset](T &value, const uint8_t *data) noexcept {
      int32_t v;
      std::memcpy(&v, data + offset, sizeof(v));
      value = static_cast<T>(v);
    };
  case PointField::FLOAT32:
    return [offset](T &value, const uint8_t *data) noexcept {
      float v;
      std::memcpy(&v, data + offset, sizeof(v));
      value = static_cast<T>(v);
    };
  case PointField::FLOAT64:
    return [offset](T &value, const uint8_t *data) noexcept {
      double v;
      std::memcpy(&v, data + offset, sizeof(v));
      value = static_cast<T>(v);
    };
  default:
    throw std::runtime_error("Unsupported PointField datatype: " +
                             std::to_string(datatype));
  }
}

template <typename T> std::function<void(T &, const uint8_t *)> blank() {
  return [](T &, const uint8_t *) noexcept {};
}

// ----------------------------- Intermediate point ----------------------------- //
// Holds parsed per-point data before reordering into the organized grid.
struct RawPoint {
  float x = 0.0f, y = 0.0f, z = 0.0f;
  uint8_t row = 0;
  uint16_t col = 0;
};

// ----------------------------- Column filling ----------------------------- //
// Fill the column index for points that arrive in row-major order
// (each row's points are contiguous; column resets when row changes).
inline void fill_col_row_major(std::vector<RawPoint> &pts) {
  if (pts.empty())
    return;
  pts[0].col = 0;
  for (size_t i = 1; i < pts.size(); ++i) {
    if (pts[i].row != pts[i - 1].row)
      pts[i].col = 0;
    else
      pts[i].col = pts[i - 1].col + 1;
  }
}

// Fill the column index for points that arrive in column-major order
// (column increments when the row wraps around to a smaller value).
inline void fill_col_col_major(std::vector<RawPoint> &pts) {
  if (pts.empty())
    return;
  pts[0].col = 0;
  for (size_t i = 1; i < pts.size(); ++i) {
    if (pts[i].row < pts[i - 1].row)
      pts[i].col = pts[i - 1].col + 1;
    else
      pts[i].col = pts[i - 1].col;
  }
}

// ----------------------------- Reorder into organized grid
// ----------------------------- // Place parsed points into a row-major grid of size
// (num_rows x num_columns). Points that fall outside the grid are silently dropped;
// gaps are filled with (0,0,0).
inline std::vector<form::PointXYZf> reorder_points(const std::vector<RawPoint> &pts,
                                                   int num_rows, int num_columns) {
  const size_t total =
      static_cast<size_t>(num_rows) * static_cast<size_t>(num_columns);
  std::vector<form::PointXYZf> organized(total, form::PointXYZf(0.0f, 0.0f, 0.0f));

  for (const auto &p : pts) {
    if (p.row < num_rows && p.col < num_columns) {
      const size_t idx =
          static_cast<size_t>(p.row) * static_cast<size_t>(num_columns) +
          static_cast<size_t>(p.col);
      organized[idx] = form::PointXYZf(p.x, p.y, p.z);
    }
  }
  return organized;
}

// ----------------------------- Main conversion ----------------------------- //
/// Convert a ROS PointCloud2 message into an organized vector of form::PointXYZf
/// in row-major order (size = num_rows * num_columns).
///
/// The function inspects the PointCloud2 fields to find:
///   - x, y, z       (required)
///   - ring/row/channel  (required for reordering)
///
/// Points are then reordered into a (num_rows x num_columns) grid using column
/// indices inferred from point ordering (row-major by default).
///
/// @param msg         The incoming PointCloud2 message
/// @param num_rows    Number of LiDAR scan lines
/// @param num_columns Number of points per scan line
/// @return Organized point cloud as vector<PointXYZf> of size num_rows * num_columns
inline std::vector<form::PointXYZf>
PointCloud2ToForm(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg,
                  int num_rows, int num_columns) {
  if (msg->is_bigendian) {
    throw std::runtime_error("Big-endian PointCloud2 is not supported");
  }

  const size_t n_points =
      static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);

  // Build field accessors
  std::function func_x = blank<float>();
  std::function func_y = blank<float>();
  std::function func_z = blank<float>();
  std::function func_row = blank<uint8_t>();

  for (const auto &field : msg->fields) {
    if (field.name == "x")
      func_x = data_getter<float>(field.datatype, field.offset);
    else if (field.name == "y")
      func_y = data_getter<float>(field.datatype, field.offset);
    else if (field.name == "z")
      func_z = data_getter<float>(field.datatype, field.offset);
    else if (field.name == "ring" || field.name == "row" || field.name == "channel")
      func_row = data_getter<uint8_t>(field.datatype, field.offset);
  }

  // Parse all points
  std::vector<RawPoint> raw(n_points);
  const uint8_t *data = msg->data.data();
  for (size_t i = 0; i < n_points; ++i) {
    const uint8_t *pt = data + i * msg->point_step;
    func_x(raw[i].x, pt);
    func_y(raw[i].y, pt);
    func_z(raw[i].z, pt);
    func_row(raw[i].row, pt);
  }

  // Infer column indices (row-major ordering)
  fill_col_row_major(raw);

  // Reorder into organized grid
  return reorder_points(raw, num_rows, num_columns);
}

} // namespace form_ros
